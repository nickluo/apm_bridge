"""Bounded, testable rate-PID search for a captive three-axis rig (not flight tuning)."""

from dataclasses import dataclass, asdict, replace
import math
import re

from sitl_support import conjugate, multiply


AXES = ("roll", "pitch", "yaw")


@dataclass(frozen=True)
class Gains:
    p: int
    i: int
    d: int


@dataclass(frozen=True)
class Limits:
    throttle: float = .12
    rate_dps: float = 5.0
    pulse_s: float = .3
    tail_s: float = .8
    max_tilt_deg: float = 15.0
    max_rate_dps: float = 40.0
    max_gain_change: int = 3
    improvement: float = .10
    timeout_s: float = 180.0

    def validate(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError("bench limits must be finite")
        bounds = ((self.throttle, 0, .2), (self.rate_dps, 1, 10),
                  (self.pulse_s, .15, .4), (self.tail_s, .5, 2),
                  (self.max_tilt_deg, 2, 20), (self.max_rate_dps, 15, 60),
                  (self.improvement, .05, .5), (self.timeout_s, 30, 600))
        if any(not low <= v <= high for v, low, high in bounds) or self.throttle == 0:
            raise ValueError("bench limits outside conservative supported bounds")
        if type(self.max_gain_change) is not int or not 1 <= self.max_gain_change <= 5:
            raise ValueError("max_gain_change must be an integer in [1,5]")


@dataclass(frozen=True)
class HoldLimits:
    hold_gain: float = 2.0
    hold_rate_dps: float = 10.0
    settle_angle_deg: float = 2.0
    settle_rate_dps: float = 2.0
    settle_time_s: float = .2
    settle_timeout_s: float = 4.0
    apply_window_s: float = .2

    def validate(self):
        bounds = ((self.hold_gain, .5, 5), (self.hold_rate_dps, 2, 15),
                  (self.settle_angle_deg, .5, 3), (self.settle_rate_dps, .5, 3),
                  (self.settle_time_s, .1, .5), (self.settle_timeout_s, 1, 10),
                  (self.apply_window_s, .08, .3))
        if any(not math.isfinite(v) or not low <= v <= high for v, low, high in bounds):
            raise ValueError("attitude-hold limits outside supported bounds")


@dataclass(frozen=True)
class AngleLimits:
    angle_step_deg: float = 2.0
    angle_step_s: float = 1.5
    angle_tail_s: float = 1.5

    def validate(self):
        bounds = ((self.angle_step_deg, 1, 5), (self.angle_step_s, .5, 3), (self.angle_tail_s, .5, 3))
        if any(not math.isfinite(v) or not low <= v <= high for v, low, high in bounds):
            raise ValueError("ANGLE step limits outside supported bounds")


def measure_angle(samples, amplitude):
    """Samples: time, target angle, measured angle, cross-axis angle, angular-rate magnitude."""
    if len(samples) < 40 or any(len(s) != 5 or not all(math.isfinite(v) for v in s) for s in samples):
        raise RuntimeError("too few or non-finite ANGLE samples")
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    if min(gaps) <= 0 or max(gaps) > .1:
        raise RuntimeError("stale, reordered or sparse ANGLE samples")
    for direction in (-1, 1):
        pulse = [s for s in samples if direction * s[1] > 0]
        if len(pulse) < 8:
            raise RuntimeError("both ANGLE step directions required")
        settled = pulse[len(pulse) * 3 // 4:]
        gain = sum(direction * s[2] for s in settled) / (len(settled) * amplitude)
        if not .6 <= gain <= 1.4:
            raise RuntimeError("unusable ANGLE response; inspect outer loop/rig/throttle")
    tail = samples[-max(10, len(samples) // 8):]
    tail_rms = math.sqrt(sum(s[2] ** 2 for s in tail) / len(tail))
    cross = max(abs(s[3]) for s in samples)
    if max(abs(s[2]) for s in samples) > 1.5 * amplitude or tail_rms > .3 * amplitude \
            or cross > .6 * amplitude or max(s[4] for s in tail) > 3:
        raise RuntimeError("excessive ANGLE overshoot, ringdown or cross-axis motion")
    duration = samples[-1][0] - samples[0][0]
    error = sum(.5 * ((a[2]-a[1])**2 + (b[2]-b[1])**2) * dt
                for a, b, dt in zip(samples, samples[1:], gaps)) / duration
    return (math.sqrt(error) + tail_rms + cross) / amplitude


def attitude_error(reference, current):
    """Shortest rotation from current to captured orientation, in current body coordinates."""
    normalized = []
    for q in (reference, current):
        if len(q) != 4 or not all(math.isfinite(v) for v in q):
            raise ValueError("invalid hold quaternion")
        norm = math.sqrt(sum(v * v for v in q))
        if not .9 <= norm * norm <= 1.1:
            raise ValueError("invalid hold quaternion norm")
        normalized.append(tuple(v / norm for v in q))
    error = multiply(conjugate(normalized[1]), normalized[0])
    if error[3] < 0:
        error = tuple(-v for v in error)
    length = math.sqrt(sum(v * v for v in error[:3]))
    if length < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = 2 * math.atan2(length, max(0.0, error[3]))
    return tuple(v * angle / length for v in error[:3])


def holding_rates(reference, current, limits, excluded_axis=None):
    error = attitude_error(reference, current)
    rates = [max(-limits.hold_rate_dps, min(limits.hold_rate_dps, math.degrees(v) * limits.hold_gain))
             for v in error]
    if excluded_axis is not None:
        rates[AXES.index(excluded_axis)] = 0.0
    return tuple(rates)


def setting_value(name, text):
    match = re.fullmatch(rf"{re.escape(name)} = ([^\r\n]+)\s*", text)
    if match is None:
        raise RuntimeError(f"invalid CLI readback for {name}: {text!r}")
    return match[1].strip()


def render_cli(profile, selected):
    """Export the complete PID snapshot, without saving/rebooting or changing other settings."""
    if not isinstance(profile, str) or re.fullmatch(r"[0-9]", profile) is None:
        raise ValueError("invalid PID profile index")
    if set(selected) != set(AXES):
        raise ValueError("CLI export requires all three axes")
    lines = [
        "# Captive three-axis rig PID result; not flight validated.",
        "# Disarm using the independent ARM switch before importing.",
        "# Review the matching JSON report and original full FC backup first.",
        "# Requires simplified_pids_mode = OFF and unchanged F/filters/D Max/TPA.",
        "# CLI profile indices are zero-based; untuned axes retain their original PID.",
        f"profile {profile}",
    ]
    for axis in AXES:
        gains = selected[axis]
        if not isinstance(gains, Gains) or any(type(v) is not int for v in asdict(gains).values()) \
                or not 1 <= gains.p <= 200 or any(not 0 <= v <= 200 for v in (gains.i, gains.d)):
            raise ValueError(f"invalid gains for {axis}")
        lines.extend(f"set {term}_{axis} = {getattr(gains, term)}" for term in "pid")
    lines.extend(["# Verify readback, then manually run save to persist and reboot.", "# save"])
    return "\n".join(lines) + "\n"


def candidates(gains, field, change):
    value = getattr(gains, field)
    if value == 0:  # Do not invent a new I/D term on an uncharacterized rig.
        return []
    delta = min(change, max(1, math.ceil(value * .1)))
    return [replace(gains, **{field: v}) for v in (value - delta, value + delta)
            if 1 <= v <= 200]


def measure(samples, amplitude):
    """Samples are (fresh IMU timestamp, reference dps, measured dps, cross-axis dps)."""
    if len(samples) < 40 or any(not all(math.isfinite(v) for v in s) for s in samples):
        raise RuntimeError("too few or non-finite bench samples")
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    if min(gaps) <= 0 or max(gaps) > .1:
        raise RuntimeError("stale, reordered or sparse IMU samples")
    for direction in (-1, 1):
        pulse = [s for s in samples if direction * s[1] > 0]
        if len(pulse) < 8:
            raise RuntimeError("both pulse directions must have enough samples")
        steady = pulse[len(pulse) // 2:]
        gain = sum(direction * s[2] for s in steady) / (len(steady) * amplitude)
        if not .6 <= gain <= 1.4:
            raise RuntimeError(f"unusable rate response gain {gain:.3f}; inspect rig/throttle")
    duration = samples[-1][0] - samples[0][0]
    error = sum(.5 * ((a[2] - a[1]) ** 2 + (b[2] - b[1]) ** 2) * dt
                for a, b, dt in zip(samples, samples[1:], gaps)) / duration
    cross = max(s[3] for s in samples)
    overshoot = max(abs(s[2]) for s in samples) / amplitude
    tail = samples[-max(10, len(samples) // 8):]
    tail_rms = math.sqrt(sum(s[2] ** 2 for s in tail) / len(tail))
    if overshoot > 1.5 or tail_rms > amplitude * .3 or cross > amplitude * .6:
        raise RuntimeError("excessive overshoot, ringdown or cross-axis motion")
    return math.sqrt(error) / amplitude + tail_rms / amplitude + cross / amplitude


class TuningSession:
    """I/O is injected; every accepted result must beat an independent baseline repeat."""

    def __init__(self, cli, trial, idle, check, limits, report):
        self.cli, self.trial, self.idle, self.check = cli, trial, idle, check
        self.limits, self.report = limits, report
        self.profile = None
        self.original = {}
        self.dirty = False

    def profile_check(self):
        text = self.cli(5, "check")
        match = re.fullmatch(r"PID PROFILE ([0-9])", text)
        if not match:
            raise RuntimeError("firmware lacks acknowledged APPLY_PID support")
        profile = match[1]
        if self.profile is not None and profile != self.profile:
            raise RuntimeError("PID profile changed; restore the original profile before recovery")
        return profile

    def read(self, name):
        return setting_value(name, self.cli(0, name))

    def snapshot(self):
        self.profile = self.profile_check()
        if self.read("simplified_pids_mode") != "OFF":
            raise RuntimeError("simplified_pids_mode must already be OFF; no automatic retuning")
        for axis in AXES:
            values = [int(self.read(f"{term}_{axis}")) for term in "pid"]
            if not 1 <= values[0] <= 200 or any(not 0 <= v <= 200 for v in values[1:]):
                raise RuntimeError("baseline gains outside supported bench bounds")
            self.original[axis] = Gains(*values)

    def write(self, axis, gains, recovery=False):
        if not recovery:
            self.check()
        self.profile_check()
        self.dirty = True  # A timed-out write may still have reached the FC.
        for field, value in asdict(gains).items():
            if not recovery:
                self.check()
            name = f"{field}_{axis}"
            if setting_value(name, self.cli(1, f"{name} = {value}")) != str(value):
                raise RuntimeError(f"SET readback mismatch: {name}")
            if self.read(name) != str(value):
                raise RuntimeError(f"GET verification mismatch: {name}")
        if not recovery:
            self.check()
        if self.cli(5, self.profile) != f"PID PROFILE {self.profile}":
            raise RuntimeError("PID apply was not acknowledged")
        self.report["active_gains"] = {"axis": axis, **asdict(gains)}

    def restore(self):
        self.profile_check()
        for axis, gains in self.original.items():
            self.write(axis, gains, recovery=True)
        self.dirty = False

    def run(self, axes):
        selected = dict(self.original)
        for axis in axes:
            self.check()
            self.idle()
            baseline = self.trial(axis)
            best, score = self.original[axis], baseline
            trials = []
            for field in "pid":
                # Candidates stay within the original term's bounded window.
                for candidate in candidates(self.original[axis], field, self.limits.max_gain_change):
                    candidate = replace(best, **{field: getattr(candidate, field)})
                    self.idle()
                    self.write(axis, candidate)
                    candidate_score = self.trial(axis)
                    trials.append({"gains": asdict(candidate), "score": candidate_score})
                    if candidate_score < score:
                        best, score = candidate, candidate_score
            self.idle()
            self.write(axis, self.original[axis])
            repeated_baseline = self.trial(axis)
            if best != self.original[axis]:
                self.idle()
                self.write(axis, best)
                repeated_candidate = self.trial(axis)
                if repeated_candidate <= (1 - self.limits.improvement) * min(baseline, repeated_baseline) \
                        and repeated_candidate <= score * 1.1:
                    selected[axis] = best
            self.idle()
            self.write(axis, selected[axis])
            self.report["axes"][axis] = {"baseline": baseline, "repeat": repeated_baseline,
                                         "trials": trials, "selected": asdict(selected[axis])}
        return selected
