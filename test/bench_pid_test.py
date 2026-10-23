"""Offline tests: no FC, motors, serial connection or AirSim process is opened."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch
from concurrent.futures import Future
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from bench_pid_core import (AXES, Gains, HoldLimits, Limits, TuningSession, attitude_error,
                            holding_rates, candidates, measure, setting_value)
from sitl_support import multiply
from bench_pid_tuner import BenchPidTuner
import bench_pid_core


CLI_GAINS = {"roll": Gains(43, 60, 20), "pitch": Gains(40, 58, 19), "yaw": Gains(30, 0, 0)}


@pytest.mark.parametrize("overrides,expected", [
    ({}, (False, False)),
    ({"enabled": "true"}, (True, False)),
    ({"rig_confirmed": "true"}, (False, True)),
    ({"enabled": "true", "rig_confirmed": "true"}, (True, True)),
    ({"enabled": "false", "rig_confirmed": "false"}, (False, False)),
])
def test_launch_interlocks_override_yaml_as_booleans(tmp_path, overrides, expected):
    import runpy
    import yaml
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    config = tmp_path / "bench.yaml"
    config.write_text(yaml.safe_dump({"bench_pid_tuner": {"ros__parameters": {
        "enabled": not expected[0], "rig_confirmed": not expected[1], "tuning_channel": 11}}}))
    context = LaunchContext()
    context.launch_configurations.update(parameters=str(config), **overrides)
    with patch("launch_ros.actions.Node") as node_factory:
        module = runpy.run_path(str(Path(__file__).resolve().parents[1] / "launch/bench_pid.launch.py"))
        description = module["generate_launch_description"]()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    evaluated = evaluate_parameters(context, normalize_parameters(node_factory.call_args.kwargs["parameters"]))
    assert evaluated[0] == config
    assert evaluated[1] == {"enabled": expected[0], "rig_confirmed": expected[1]}
    assert all(type(value) is bool for value in evaluated[1].values())


def rotation(vector):
    length = math.sqrt(sum(v * v for v in vector))
    if length < 1e-12:
        return (0., 0., 0., 1.)
    scale = math.sin(length / 2) / length
    return tuple(v * scale for v in vector) + (math.cos(length / 2),)


@pytest.mark.parametrize("reference", [(0., 0., 0., 1.), rotation((1.4, -.9, 2.)),
                                        rotation((0., math.pi / 2, 0.)), rotation((math.pi, 0., 0.))])
def test_quaternion_hold_uses_local_shortest_error_at_any_initial_attitude(reference):
    current = multiply(reference, rotation((math.radians(5), 0., 0.)))
    assert attitude_error(reference, current) == pytest.approx((-math.radians(5), 0, 0), abs=1e-8)
    assert holding_rates(reference, current, HoldLimits()) == pytest.approx((-10, 0, 0), abs=1e-7)
    assert attitude_error(reference, tuple(-v for v in current)) == pytest.approx(attitude_error(reference, current))
    assert holding_rates(reference, current, HoldLimits(), "roll") == pytest.approx((0, 0, 0), abs=1e-7)


@pytest.mark.parametrize("field,value", [("hold_gain", 0), ("hold_rate_dps", 100),
                                         ("apply_window_s", 4), ("settle_rate_dps", 5),
                                         ("settle_timeout_s", math.nan)])
def test_unsafe_hold_configuration_rejected(field, value):
    with pytest.raises(ValueError):
        replace(HoldLimits(), **{field: value}).validate()


def free_rig(node, monkeypatch, bias=(2., -1., .5)):
    reference = rotation((.8, 1.4, -.6))
    node.reference = reference
    node.imu.orientation = SimpleNamespace(x=reference[0], y=reference[1], z=reference[2], w=reference[3])
    clock = {"time": 0., "rates": [0., 0., 0.], "q": reference, "powered": [], "zero": []}
    node.imu_stamp = 0.
    node.check = Mock()
    def step(dt):
        node.refresh_command()
        throttle, commanded = node.command
        clock["powered" if throttle else "zero"].append(dt)
        if throttle:
            clock["rates"] = [r + (1 - math.exp(-dt / .015)) * (u - r)
                               for r, u in zip(clock["rates"], commanded)]
        else:
            clock["rates"] = [r + math.radians(a) * dt for r, a in zip(clock["rates"], bias)]
        clock["q"] = multiply(clock["q"], rotation(tuple(v * dt for v in clock["rates"])))
        q = clock["q"]
        node.imu.orientation = SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])
        node.imu.angular_velocity = SimpleNamespace(**dict(zip(("x", "y", "z"), clock["rates"])))
        clock["time"] += dt
        node.imu_stamp = clock["time"]
    monkeypatch.setattr("bench_pid_tuner.time.monotonic", lambda: clock["time"])
    monkeypatch.setattr("bench_pid_tuner.time.sleep", step)
    return clock, step


def test_drifting_free_rig_recovers_under_power_and_excludes_excited_axis_from_hold(monkeypatch):
    node = minimal_node()
    clock, step = free_rig(node, monkeypatch)
    node.command_mode = "zero"
    for _ in range(200):
        step(.01)
    drift = math.degrees(math.sqrt(sum(v * v for v in attitude_error(node.reference, clock["q"]))))
    assert drift > 4  # The previous unpowered idle cannot hold this rig.
    node.idle()
    assert node.command[0] == node.limits.throttle and node.command_mode == "hold"
    assert math.degrees(math.sqrt(sum(v * v for v in attitude_error(node.reference, clock["q"])))) < 2
    samples = node.drive(.3, node.limits.throttle, "roll", 5.)
    assert samples and node.command[1][0] == pytest.approx(math.radians(5))
    assert node.command[0] == node.limits.throttle  # No zero dip between drive stages.
    node.drive(.8, node.limits.throttle, "roll", 0.)
    assert node.command[1][0] == 0.
    node.idle()
    assert node.command_mode == "hold"


def test_angle_step_metric_accepts_tracking_but_rejects_no_response_and_oscillation():
    trace = []
    actual = 0.
    for index in range(600):
        target = 2. if index < 150 else -2. if 300 <= index < 450 else 0.
        rate = 5 * (target - actual)
        actual += rate * .01
        trace.append((index*.01, target, actual, 0., abs(rate)))
    assert bench_pid_core.measure_angle(trace, 2.) < .4
    for bad in ([tuple([s[0], s[1], 0., 0., 0.]) for s in trace],
                [tuple([s[0], s[1], s[2]*2., 0., s[4]]) for s in trace],
                [tuple([s[0], s[1], s[2], 2., s[4]]) for s in trace]):
        with pytest.raises(RuntimeError):
            bench_pid_core.measure_angle(bad, 2.)


@pytest.mark.parametrize("axis", ["roll", "pitch"])
def test_angle_trial_scores_attitude_not_rate_and_keeps_other_axis_level(axis):
    node = minimal_node()
    node.start_mode = "ANGLE"
    node.idle = Mock()
    node.write_report = Mock()
    node.report = {"measurements": []}
    def drive(duration, throttle, selected, reference):
        assert selected == axis
        n = round(duration / .01)
        start = drive.time
        drive.time += duration
        return [(start + i*.01, reference, reference, 0., 0.) for i in range(n)]
    drive.time = 0.
    node.drive = drive
    assert node.trial(axis) == 0.
    assert len(node.report["measurements"][0]["samples"][0]) == 5


def test_angle_yaw_is_rate_test_while_roll_pitch_stay_at_level_targets(monkeypatch):
    node = minimal_node()
    node.start_mode = "ANGLE"
    node.angle_reference = (0., 0., 0., 1.)
    node.reference = node.angle_reference
    node.imu.orientation = SimpleNamespace(x=0., y=0., z=0., w=1.)
    node.command_mode = "excite"
    node.excitation_axis, node.excitation_rate = "yaw", 5.
    node.refresh_command()
    assert node.command == (.12, (0., 0., math.radians(5)))
    node.check = Mock()
    node.idle = Mock()
    node.write_report = Mock()
    node.report = {"measurements": []}
    traces = samples()
    node.drive = Mock(side_effect=[traces[:30], traces[30:110], traces[110:140], traces[140:]])
    assert node.trial("yaw") == 0.
    assert node.drive.call_args_list[0].args == (.3, .12, "yaw", 5.)


def test_preflight_cannot_silently_change_selected_mode():
    node = minimal_node()
    node.start_mode = "ANGLE"
    node.state.mode = "ACRO"
    with patch("bench_pid_tuner.time.monotonic", return_value=100), pytest.raises(RuntimeError, match="startup mode changed"):
        node.health(False)


def test_selected_angle_mode_loss_cannot_fall_back_to_acro_offboard():
    node = minimal_node()
    node.start_mode = "ANGLE"
    node.state.mode = "OFFBOARD"
    with patch("bench_pid_tuner.time.monotonic", return_value=100), pytest.raises(RuntimeError, match="mode lost"):
        node.health(True)
    node.state.mode = "OFFBOARD|ANGLE"
    with patch("bench_pid_tuner.time.monotonic", return_value=100):
        node.health(True)


def test_rate_trial_retains_valid_score_without_measured_axis_recentering(monkeypatch):
    node = minimal_node()
    clock, step = free_rig(node, monkeypatch)
    node.report = {"measurements": []}
    node.write_report = Mock()
    score = node.trial("roll")
    assert score < .3 and len(node.report["measurements"]) == 1
    assert not clock["zero"]
    assert node.command[0] == .12
    assert node.command[1][0] == 0


@pytest.mark.parametrize("bad", [(0., 0., 0., 0.), (math.nan, 0., 0., 1.), (1., 0., 0., 1.)])
def test_invalid_quaternion_cannot_generate_hold_commands(bad):
    with pytest.raises(ValueError):
        holding_rates((0., 0., 0., 1.), bad, HoldLimits())


def test_uncontrollable_hold_times_out_instead_of_raising_throttle(monkeypatch):
    node = minimal_node()
    clock, step = free_rig(node, monkeypatch)
    q = multiply(node.reference, rotation((math.radians(5), 0., 0.)))
    node.imu.orientation = SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])
    monkeypatch.setattr("bench_pid_tuner.time.sleep", lambda dt: clock.update(time=clock["time"] + dt))
    with pytest.raises(RuntimeError, match="did not settle"):
        node.idle()
    assert node.command[0] == .12


def test_live_apply_uses_short_zero_window_then_returns_to_powered_hold(monkeypatch):
    node = minimal_node()
    node.recovering = False
    clock, step = free_rig(node, monkeypatch)
    node.command_mode = "hold"
    observed = []
    def request(cmd, args):
        observed.append((cmd, args, node.command, node.apply_deadline))
        step(.02)
        return "PID PROFILE 0"
    node.cli_request = request
    assert node.cli(5, "0") == "PID PROFILE 0"
    assert observed[0][2] == (0., (0., 0., 0.))
    assert observed[0][3] is not None and clock["time"] < node.hold.apply_window_s
    assert node.command[0] == .12 and node.command_mode == "hold" and node.apply_deadline is None


def test_apply_window_expiry_cancels_output_without_retaking_authority(monkeypatch):
    node = minimal_node()
    node.recovering = False
    clock, step = free_rig(node, monkeypatch)
    node.command_mode = "hold"
    def check():
        if node.apply_deadline is not None and clock["time"] >= node.apply_deadline:
            raise RuntimeError("application window expired")
    node.check.side_effect = check
    def late(cmd, args):
        step(.3)
        return "PID PROFILE 0"
    node.cli_request = late
    with pytest.raises(RuntimeError, match="window expired"):
        node.cli(5, "0")
    assert node.command == (0., (0., 0., 0.)) and not node.active and node.cancel.is_set()
    assert node.mode_pub.publish.call_args.args[0].data is False


def test_lost_apply_ack_expires_short_window_without_retry_or_power_resume(monkeypatch):
    node = minimal_node()
    node.recovering = False
    node.abort_reason = None
    clock, step = free_rig(node, monkeypatch)
    pending = Future()
    node.cli_client = Mock()
    node.cli_client.call_async.return_value = pending
    def check():
        if node.apply_deadline is not None and clock["time"] >= node.apply_deadline:
            raise RuntimeError("zero-output PID application window expired")
    node.check.side_effect = check
    with pytest.raises(RuntimeError, match="window expired"):
        node.cli(5, "0")
    assert node.cli_client.call_async.call_count == 1
    node.cli_client.remove_pending_request.assert_called_once_with(pending)
    assert clock["time"] < node.hold.apply_window_s + .02
    assert node.command == (0., (0., 0., 0.)) and node.cancel.is_set() and not node.active


def test_cancel_during_apply_never_restores_powered_hold(monkeypatch):
    node = minimal_node()
    node.recovering = False
    clock, step = free_rig(node, monkeypatch)
    def check():
        if node.cancel.is_set():
            raise RuntimeError("CH9 cancelled")
    node.check.side_effect = check
    def cancelled(cmd, args):
        node.stop_output("CH9 cancelled")
        return "PID PROFILE 0"
    node.cli_request = cancelled
    with pytest.raises(RuntimeError, match="cancelled"):
        node.cli(5, "0")
    assert node.command_mode == "zero" and not node.active
    assert node.command == (0., (0., 0., 0.))


def test_geodesic_excursion_guard_rejects_combined_rotation_at_tilted_reference():
    node = minimal_node()
    node.reference = rotation((0., math.pi / 2, 0.))
    q = multiply(node.reference, rotation((math.radians(12), math.radians(12), 0.)))
    node.imu.orientation = SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])
    with patch("bench_pid_tuner.time.monotonic", return_value=100), pytest.raises(RuntimeError, match="excursion"):
        node.health(True)


def test_live_apply_rejects_drifting_gyro_before_firmware_call(monkeypatch):
    node = minimal_node()
    node.recovering = False
    clock, step = free_rig(node, monkeypatch, bias=(120., 0., 0.))
    node.cli_request = Mock()
    with pytest.raises(RuntimeError, match="rig moving"):
        node.cli(5, "0")
    node.cli_request.assert_not_called()
    assert not node.active and node.cancel.is_set()


def test_cli_export_contains_exact_profile_and_all_selected_gains_without_save():
    text = bench_pid_core.render_cli("2", CLI_GAINS)
    commands = [line for line in text.splitlines() if line and not line.startswith("#")]
    assert commands == ["profile 2"] + [
        f"set {term}_{axis} = {getattr(CLI_GAINS[axis], term)}" for axis in AXES for term in "pid"]
    assert "# save" in text.splitlines()
    assert "not flight validated" in text
    assert text.endswith("\n")


@pytest.mark.parametrize("profile,gains", [
    ("0\nsave", CLI_GAINS), ("-1", CLI_GAINS), (True, CLI_GAINS),
    ("0", {"roll": Gains(40, 60, 20)}),
    ("0", {**CLI_GAINS, "roll": Gains(0, 60, 20)}),
    ("0", {**CLI_GAINS, "roll": Gains(40, 201, 20)}),
    ("0", {**CLI_GAINS, "yaw": Gains(30, 0, 1.5)}),
    ("0", {**CLI_GAINS, "yaw": Gains(True, 0, 0)})])
def test_cli_export_rejects_invalid_or_incomplete_results(profile, gains):
    with pytest.raises(ValueError):
        bench_pid_core.render_cli(profile, gains)


class FakeCli:
    def __init__(self):
        self.values = {f"{term}_{axis}": str(value) for axis in AXES
                       for term, value in zip("pid", (40, 60, 20))}
        self.values["simplified_pids_mode"] = "OFF"
        self.values.update(acc_trim_roll="0", acc_trim_pitch="0", angle_p_gain="50",
                           angle_feedforward="50", angle_feedforward_smoothing_ms="80",
                           angle_limit="60", angle_earth_ref="100", level_race_mode="OFF")
        self.calls = []
        self.profile = "0"

    def __call__(self, cmd, args):
        self.calls.append((cmd, args))
        if cmd == 5:
            if args == "angle":
                return "ANGLE OFFBOARD 1"
            if args not in ("check", self.profile):
                raise RuntimeError("PID PROFILE MISMATCH")
            return f"PID PROFILE {self.profile}"
        if cmd == 1:
            name, value = args.split(" = ")
            self.values[name] = value
        else:
            name = args
        return f"{name} = {self.values[name]}"


def session(cli=None, trial=None):
    io = cli or FakeCli()
    report = {"axes": {}}
    value = TuningSession(io, trial or (lambda axis: 1.0), Mock(), Mock(), Limits(), report)
    value.snapshot()
    return value, io, report


def samples(scale=1.0, cross=0.0):
    result = []
    for i in range(220):
        target = 5 if 0 <= i < 30 else -5 if 110 <= i < 140 else 0
        result.append((i * .01, target, target * scale, cross))
    return result


def test_metrics_require_both_directions_and_accept_clean_response():
    assert measure(samples(), 5) == 0
    with pytest.raises(RuntimeError, match="both pulse"):
        measure(samples()[30:], 5)


@pytest.mark.parametrize("case", ["no-response", "overshoot", "cross", "stale", "nan"])
def test_invalid_or_unsafe_trials_abort(case):
    trace = samples()
    if case == "no-response":
        trace = samples(0)
    elif case == "overshoot":
        trace = samples(2)
    elif case == "cross":
        trace = samples(cross=4)
    elif case == "stale":
        trace[20] = trace[19]
    else:
        trace[20] = (math.nan, 0, 0, 0)
    with pytest.raises(RuntimeError):
        measure(trace, 5)


@pytest.mark.parametrize("field,value", [
    ("throttle", .21), ("throttle", 0), ("rate_dps", 11), ("pulse_s", 1),
    ("max_gain_change", 6), ("max_tilt_deg", 30), ("timeout_s", math.inf)])
def test_unsafe_configuration_is_rejected(field, value):
    with pytest.raises(ValueError):
        replace(Limits(), **{field: value}).validate()


def test_candidates_are_local_and_do_not_enable_disabled_terms():
    original = Gains(40, 60, 0)
    assert candidates(original, "d", 3) == []
    assert candidates(original, "p", 3) == [Gains(37, 60, 0), Gains(43, 60, 0)]
    assert candidates(Gains(1, 1, 1), "p", 3) == [Gains(2, 1, 1)]


def test_protocol_readback_is_exact_not_substring_or_success_shaped():
    assert setting_value("p_roll", "p_roll = 45") == "45"
    for text in ("p_pitch = 45", "SET FAILED", "prefix p_roll = 45", "p_roll = 45\nextra"):
        with pytest.raises(RuntimeError):
            setting_value("p_roll", text)


def test_writes_apply_runtime_pid_and_restore_without_saving():
    tuner, io, _ = session()
    tuner.write("roll", Gains(43, 60, 20))
    assert tuner.dirty
    assert io.values["p_roll"] == "43"
    assert io.calls[-1] == (5, "0")
    tuner.restore()
    assert io.values["p_roll"] == "40"
    assert not tuner.dirty
    assert all(cmd not in (3, 4) for cmd, _ in io.calls)


def test_failed_write_is_marked_dirty_for_uncertain_outcomes():
    tuner, io, _ = session()
    def fail(cmd, args):
        if cmd == 1:
            io(cmd, args)  # FC applied, but the transport lost the reply.
            raise RuntimeError("timeout")
        return io(cmd, args)
    tuner.cli = fail
    with pytest.raises(RuntimeError, match="timeout"):
        tuner.write("roll", Gains(43, 60, 20))
    assert tuner.dirty
    tuner.cli = io
    tuner.restore()
    assert io.values["p_roll"] == "40"


def test_changed_profile_blocks_write_and_restore_to_wrong_profile():
    tuner, io, _ = session()
    io.profile = "1"
    with pytest.raises(RuntimeError, match="profile changed"):
        tuner.write("roll", Gains(43, 60, 20))
    assert not any(cmd == 1 for cmd, _ in io.calls)


def test_simplified_gains_and_old_firmware_are_not_silently_accepted():
    io = FakeCli()
    io.values["simplified_pids_mode"] = "ON"
    with pytest.raises(RuntimeError, match="simplified"):
        session(io)
    with pytest.raises(RuntimeError, match="APPLY_PID"):
        session(lambda cmd, args: "NOT FOUND")


def test_search_requires_independent_improvement_and_keeps_other_axes():
    io = FakeCli()
    trial = lambda axis: 1.0 if io.values[f"p_{axis}"] != "43" else .7
    tuner, _, report = session(io, trial)
    selected = tuner.run(["roll"])
    assert selected["roll"] == Gains(43, 60, 20)
    assert selected["pitch"] == Gains(40, 60, 20)
    assert io.values["p_roll"] == "43"
    assert report["axes"]["roll"]["repeat"] == 1.0


def test_no_repeatable_improvement_restores_original():
    tuner, io, _ = session()
    selected = tuner.run(["roll"])
    assert selected == tuner.original
    assert io.values["p_roll"] == "40"


def minimal_node():
    node = BenchPidTuner.__new__(BenchPidTuner)
    node.lock = threading.RLock()
    node.cancel = threading.Event()
    node.command = (.12, (.1, 0, 0))
    node.active = node.owned = True
    node.low_seen = False
    node.requested = False
    node.worker = None
    node.rc = [1500, 1500, 1500, 1000, 1000, 1000, 2000, 1000, 2000]
    node.tuning_channel = 8
    node.rc_at = node.state_at = node.imu_at = node.failsafe_at = 100
    node.state = SimpleNamespace(connected=True, armed=True, mode="OFFBOARD")
    node.imu = SimpleNamespace(angular_velocity=SimpleNamespace(x=0., y=0., z=0.))
    node.reference = None
    node.command_mode = "zero"
    node.excitation_axis = None
    node.excitation_rate = 0.0
    node.apply_deadline = None
    node.hold = HoldLimits()
    node.angle = bench_pid_core.AngleLimits()
    node.start_mode = "ACRO"
    node.angle_reference = None
    node.angle_center = (0., 0.)
    node.angle_trim = (0., 0.)
    node.mission = node.failsafe = False
    node.limits = Limits()
    node.publish_command = Mock()
    node.mode_pub = Mock()
    node.status = Mock()
    return node


def cli_node(responses):
    node = minimal_node()
    node.recovering = node.active = node.owned = False
    node.cli_client = Mock()
    node.get_logger = Mock(return_value=Mock())
    futures = []
    for response in responses:
        future = Future()
        future.set_result(response)
        futures.append(future)
    node.cli_client.call_async.side_effect = futures
    return node


def cli_response(ok=True, text="p_roll = 34", error="", status=0):
    return SimpleNamespace(ok=ok, status=status, text=text, error=error)


@pytest.mark.parametrize("cmd,args", [(0, "p_roll"), (0, "simplified_pids_mode"),
                                      (2, "p_roll"), (5, "check")])
def test_readonly_cli_timeout_is_retried_at_most_twice(cmd, args):
    node = cli_node([cli_response(ok=False, error="timeout"), cli_response(ok=False, error="timeout"),
                     cli_response(text="verified")])
    assert node.cli(cmd, args) == "verified"
    assert node.cli_client.call_async.call_count == 3


@pytest.mark.parametrize("cmd,args", [(1, "p_roll = 34"), (5, "0"), (3, ""), (4, "")])
def test_mutating_cli_timeout_is_never_retried(cmd, args):
    node = cli_node([cli_response(ok=False, error="timeout")])
    with pytest.raises(RuntimeError, match="timeout"):
        node.cli(cmd, args)
    assert node.cli_client.call_async.call_count == 1


def test_readonly_retry_is_bounded_and_firmware_errors_fail_immediately():
    node = cli_node([cli_response(ok=False, error="timeout")] * 3)
    with pytest.raises(RuntimeError, match="timeout"):
        node.cli(0, "p_roll")
    assert node.cli_client.call_async.call_count == 3
    node = cli_node([cli_response(status=1, text="NOT FOUND")])
    with pytest.raises(RuntimeError):
        node.cli(0, "p_roll")
    assert node.cli_client.call_async.call_count == 1


def test_read_retry_does_not_continue_after_cancel_or_armed_recovery():
    node = cli_node([cli_response(ok=False, error="timeout")])
    node.active = True
    node.check = Mock(side_effect=RuntimeError("CH9 cancelled"))
    with pytest.raises(RuntimeError, match="cancelled"):
        node.cli(0, "p_roll")
    assert node.cli_client.call_async.call_count == 0
    node = cli_node([cli_response(ok=False, error="timeout")])
    node.recovering = True
    with pytest.raises(RuntimeError, match="disarmed"):
        node.cli(0, "p_roll")
    assert node.cli_client.call_async.call_count == 0


def export_node(tmp_path):
    node = minimal_node()
    node.check = Mock()
    node.session = SimpleNamespace(profile="2", dirty=True)
    node.report_path = tmp_path / "bench-123.json"
    node.report = {"status": "starting", "persisted": False}
    node.get_logger = Mock(return_value=Mock())
    return node


@pytest.mark.parametrize("failure", ["write", "rename"])
def test_report_disk_failure_preserves_previous_report_and_cleans_temporary_file(tmp_path, monkeypatch, failure):
    node = export_node(tmp_path)
    previous = '{"status": "starting"}'
    node.report_path.write_text(previous)
    if failure == "write":
        write_text = Path.write_text
        def fail_write(path, *args, **kwargs):
            write_text(path, "partial")
            raise OSError("disk full")
        monkeypatch.setattr(Path, "write_text", fail_write)
    else:
        monkeypatch.setattr(Path, "replace", Mock(side_effect=OSError("rename failed")))
    with pytest.raises(OSError):
        node.write_report()
    assert node.report_path.read_text() == previous
    assert not list(tmp_path.glob("*.tmp"))
    assert node.session.dirty and node.report["rollback_required"]


def test_success_exports_cli_and_json_then_releases_without_disarming(tmp_path):
    import json
    node = export_node(tmp_path)
    node.complete_session(CLI_GAINS)
    cli_path = tmp_path / "bench-123.cli"
    assert cli_path.read_text() == "# Tuning start mode: ACRO; body-rate response.\n" + bench_pid_core.render_cli("2", CLI_GAINS)
    report = json.loads(node.report_path.read_text())
    assert report["cli_file"] == str(cli_path)
    assert report["selected"]["yaw"] == {"p": 30, "i": 0, "d": 0}
    assert report["status"] == "completed-ram-only"
    assert report["persisted"] is False and report["rollback_required"] is False
    assert not node.session.dirty and not node.active and not node.owned
    assert node.state.armed  # Zero throttle/releasing OFFBOARD is not a disarm.
    node.publish_command.assert_called_once_with(0.0, (0., 0., 0.))
    assert node.mode_pub.publish.call_args.args[0].data is False
    assert str(cli_path) in node.status.call_args.args[0]
    assert not list(tmp_path.glob("*.tmp"))


@pytest.mark.parametrize("failure", ["cli-write", "cli-rename", "json-write", "cancel"])
def test_export_failure_keeps_rollback_required_and_removes_success_cli(tmp_path, monkeypatch, failure):
    node = export_node(tmp_path)
    if failure == "cli-write":
        write_text = Path.write_text
        def fail_write(path, *args, **kwargs):
            if path.suffixes[-2:] == [".cli", ".tmp"]:
                write_text(path, "partial")
                raise OSError("disk full")
            return write_text(path, *args, **kwargs)
        monkeypatch.setattr(Path, "write_text", fail_write)
    elif failure == "cli-rename":
        monkeypatch.setattr(Path, "replace", Mock(side_effect=OSError("rename failed")))
    elif failure == "json-write":
        node.write_report = Mock(side_effect=OSError("report failed"))
    else:
        def cancel_after_report(*args, **kwargs):
            node.stop_output("CH9 cancelled")
        node.write_report = cancel_after_report
        node.check.side_effect = lambda: (_ for _ in ()).throw(RuntimeError("cancelled")) \
            if node.cancel.is_set() else None
    with pytest.raises((OSError, RuntimeError)):
        node.complete_session(CLI_GAINS)
    assert node.session.dirty
    assert "cli_file" not in node.report
    assert not list(tmp_path.glob("*.cli")) and not list(tmp_path.glob("*.tmp"))


@pytest.mark.parametrize("channel", [0, 8, 11, 15])
def test_only_configured_zero_based_tuning_channel_controls_start_cancel_and_health(channel):
    node = minimal_node()
    node.tuning_channel = channel
    node.active = False
    channels = [2000] * 16
    channels[channel] = 1000
    node.on_rc(SimpleNamespace(channels=channels))
    channels[channel] = 2000
    node.on_rc(SimpleNamespace(channels=channels))
    assert node.requested
    with patch("bench_pid_tuner.time.monotonic", return_value=100):
        node.rc_at = 100
        node.health(True)  # All other RC values are irrelevant; state/authority are authoritative.
    node.active = True
    channels[channel] = 1000
    node.on_rc(SimpleNamespace(channels=channels))
    assert node.cancel.is_set() and not node.active
    assert f"CH{channel + 1}" in node.abort_reason


@pytest.mark.parametrize("channel", [0, 8, 15])
def test_missing_tuning_channel_stops_and_requires_new_low_edge(channel):
    node = minimal_node()
    node.tuning_channel = channel
    node.low_seen = node.requested = True
    node.on_rc(SimpleNamespace(channels=[1000] * channel))
    assert not node.active and node.cancel.is_set()
    assert not node.low_seen and not node.requested
    node.on_rc(SimpleNamespace(channels=[2000] * (channel + 1)))
    assert not node.requested


@pytest.mark.parametrize("channel", [-1, 16])
def test_out_of_range_tuning_channel_rejected_at_startup(channel, monkeypatch):
    import os
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    monkeypatch.setenv("ROS_DOMAIN_ID", str(150 + os.getpid() % 60))
    context = Context()
    rclpy.init(context=context)
    node = BenchPidTuner.__new__(BenchPidTuner)
    try:
        with pytest.raises(ValueError, match="zero-based integer"):
            node.__init__(context=context, parameter_overrides=[Parameter("tuning_channel", value=channel)])
    finally:
        Node.destroy_node(node)
        rclpy.shutdown(context=context)


def test_tuning_yaml_contains_only_its_zero_based_switch_channel():
    import yaml
    params = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             "parameters/bench_pid.yaml").read_text())["bench_pid_tuner"]["ros__parameters"]
    assert params["tuning_channel"] == 8
    assert not any(name in params for name in
                   ("arm_channel", "mode_channel", "offboard_channel", "mission_channel"))


def test_ch9_low_cancels_output_and_releases_authority_immediately():
    node = minimal_node()
    node.on_rc(SimpleNamespace(channels=[*node.rc[:8], 1000]))
    assert node.cancel.is_set() and not node.active and not node.owned
    node.publish_command.assert_called_once_with(0.0, (0., 0., 0.))
    assert node.mode_pub.publish.call_args.args[0].data is False


def test_start_requires_low_before_high_and_does_not_repeat_while_held():
    node = minimal_node()
    node.active = False
    node.on_rc(SimpleNamespace(channels=node.rc))
    assert not node.requested
    node.on_rc(SimpleNamespace(channels=[*node.rc[:8], 1000]))
    node.on_rc(SimpleNamespace(channels=[*node.rc[:8], 2000]))
    assert node.requested
    node.requested = False
    node.on_rc(SimpleNamespace(channels=node.rc))
    assert not node.requested


@pytest.mark.parametrize("fault", ["rc-stale", "imu-stale", "state-stale", "failsafe",
                                   "disarm", "offboard-lost", "switch-low", "channel-missing", "mission", "rate"])
def test_health_faults_prevent_any_experiment(fault):
    node = minimal_node()
    if fault == "rc-stale":
        node.rc_at = 99
    elif fault == "imu-stale":
        node.imu_at = 99
    elif fault == "state-stale":
        node.state_at = 99
    elif fault == "failsafe":
        node.failsafe = True
    elif fault == "disarm":
        node.state.armed = False
    elif fault == "offboard-lost":
        node.state.mode = "ANGLE"
    elif fault == "switch-low":
        node.rc[node.tuning_channel] = 1000
    elif fault == "channel-missing":
        node.rc = node.rc[:node.tuning_channel]
    elif fault == "mission":
        node.mission = True
    else:
        node.imu.angular_velocity.x = 2
    with patch("bench_pid_tuner.time.monotonic", return_value=100), pytest.raises(RuntimeError):
        node.health(True)


@pytest.mark.parametrize("scenario", ["complete", "export-failure", "cancel", "custom-channel",
                                      "custom-cancel", "disabled", "angle", "angle-cancel", "angle-unsupported", "angle-too-tilted"])
def test_ros_node_session_and_cancel_recovery_without_hardware(tmp_path, monkeypatch, scenario):
    import os
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import RCIn, State
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool
    from quadrotor_msgs.msg import ControlCommand
    from quadrotor_msgs.srv import CliCommand

    monkeypatch.setenv("ROS_DOMAIN_ID", str(150 + os.getpid() % 60))
    context = Context()
    rclpy.init(context=context)
    fc = Node("fake_bench_fc", context=context)
    custom_channel = scenario in ("custom-channel", "custom-cancel")
    tuning_channel = 15 if custom_channel else 8
    node = BenchPidTuner(context=context, parameter_overrides=[
        Parameter("enabled", value=scenario != "disabled"), Parameter("rig_confirmed", value=True),
        Parameter("axes", value=["roll"]), Parameter("pulse_s", value=.15),
        Parameter("tail_s", value=.5), Parameter("max_gain_change", value=1),
        Parameter("output_dir", value=str(tmp_path)),
        Parameter("tuning_channel", value=tuning_channel)])
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(fc)
    io = FakeCli()
    read_timeout_injected = False
    if scenario == "export-failure":
        monkeypatch.setattr("bench_pid_tuner.render_cli", Mock(side_effect=OSError("export disk full")))
    angle_mode = scenario.startswith("angle")
    reference = rotation((0., 0., 1.8)) if angle_mode else rotation((1.2, -.7, 1.8))
    if scenario == "angle-too-tilted":
        reference = rotation((math.radians(20), 0., 0.))
    current = {"armed": True, "mode": "ANGLE" if angle_mode else "ACRO", "rates": [0., 0., 0.], "q": reference,
               "command": None, "switch": 1000, "apply_checks": 0, "powered_sets": 0}
    outputs = []
    rc_pub = fc.create_publisher(RCIn, "/mavros/rc/in", qos_profile_sensor_data)
    imu_pub = fc.create_publisher(Imu, "/mavros/imu/data", qos_profile_sensor_data)
    state_pub = fc.create_publisher(State, "/mavros/state", 10)
    failsafe_pub = fc.create_publisher(Bool, "/fpv/failsafe", 10)

    def command(msg):
        current["command"] = msg
        outputs.append((time.monotonic(), msg.collective_thrust))

    def mode(msg):
        current["mode"] = ("OFFBOARD|ANGLE" if angle_mode else "OFFBOARD") if msg.data else ("ANGLE" if angle_mode else "ACRO")

    fc.create_subscription(ControlCommand, "/fpv/control_command_raw", command, qos_profile_sensor_data)
    fc.create_subscription(Bool, "/fpv/offboard", mode, 10)

    def cli(request, response):
        nonlocal read_timeout_injected
        command = current["command"]
        if scenario == "angle-unsupported" and request.cmd == 5 and request.args == "angle":
            response.ok, response.status, response.text = True, 1, "PID PROFILE MISMATCH"
            return response
        if current["armed"] and request.cmd == 5 and request.args not in ("check", "angle"):
            assert command is not None and command.collective_thrust == 0
            assert max(abs(v) for v in (command.bodyrates.x, command.bodyrates.y, command.bodyrates.z)) < 1e-8
            if angle_mode:
                assert (command.orientation.x, command.orientation.y, command.orientation.z,
                        command.orientation.w) == (0., 0., 0., 1.)
            assert max(abs(math.degrees(v)) for v in current["rates"]) < 5
            current["apply_checks"] += 1
        if current["armed"] and request.cmd == 1:
            assert command is not None and command.collective_thrust == node.limits.throttle
            current["powered_sets"] += 1
        if scenario == "custom-channel" and request.cmd == 0 and not read_timeout_injected:
            read_timeout_injected = True
            response.ok, response.error = False, "timeout"
            return response
        response.ok, response.status, response.text = True, 0, io(request.cmd, request.args)
        return response

    fc.create_service(CliCommand, "/fpv/cli", cli)
    last = time.monotonic()

    def telemetry():
        nonlocal last
        now = time.monotonic()
        dt, last = now - last, now
        cmd = current["command"]
        powered = cmd is not None and cmd.collective_thrust > 0 and current["mode"].startswith("OFFBOARD")
        if powered:
            goals = (cmd.bodyrates.x, cmd.bodyrates.y, cmd.bodyrates.z)
            if angle_mode:
                assert cmd.control_mode == ControlCommand.ATTITUDE
                target = (cmd.orientation.x, cmd.orientation.y, cmd.orientation.z, cmd.orientation.w)
                from sitl_support import euler
                roll, pitch, _ = euler(target)
                measured_roll, measured_pitch, _ = euler(current["q"])
                goals = ((roll-measured_roll)*5, (pitch-measured_pitch)*5, cmd.bodyrates.z)
            else:
                assert cmd.control_mode == ControlCommand.BODY_RATES
            current["rates"] = [v + (1 - math.exp(-dt / .015)) * (u - v)
                                for v, u in zip(current["rates"], goals)]
        else:
            # A free unpowered rig accelerates under an off-centre load.
            current["rates"] = [v + math.radians(a) * dt for v, a in
                                zip(current["rates"], (.5, -.3, .2))]
        current["q"] = multiply(current["q"], rotation(tuple(v * dt for v in current["rates"])))
        channels = [2000] * 16
        channels[tuning_channel] = current["switch"]
        rc_pub.publish(RCIn(channels=channels))
        state_pub.publish(State(connected=True, armed=current["armed"], mode=current["mode"]))
        failsafe_pub.publish(Bool(data=False))
        imu = Imu()
        imu.header.stamp = fc.get_clock().now().to_msg()
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = current["q"]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = current["rates"]
        imu_pub.publish(imu)

    fc.create_timer(.005, telemetry)

    def pump(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, (node.report, node.abort_reason)
            executor.spin_once(timeout_sec=.005)

    try:
        pump(lambda: node.rc is not None and node.cli_client.service_is_ready(), 5)
        assert not hasattr(node, "bridge_parameters")
        assert all(not node.has_parameter(name) for name in
                   ("arm_channel", "mode_channel", "offboard_channel", "mission_channel"))
        current["switch"] = 2000
        if scenario == "disabled":
            pump(lambda: not node.low_seen and not node.requested, 2)
            assert node.worker is None and not outputs and not io.calls
            assert not list(tmp_path.glob("*.cli"))
            return
        if scenario in ("angle-unsupported", "angle-too-tilted"):
            pump(lambda: node.worker is not None and not node.worker.is_alive(), 5)
            assert not outputs and not any(cmd == 1 for cmd, _ in io.calls)
            if scenario == "angle-too-tilted":
                assert "5 degrees" in node.abort_reason
            else:
                assert node.report["status"] == "aborted"
            return
        if scenario in ("cancel", "custom-cancel", "angle-cancel"):
            pump(lambda: any(cmd == 1 for cmd, _ in io.calls), 10)
            current["switch"] = 1000
            pump(lambda: node.cancel.is_set() and not node.active, 1)
            pump(lambda: current["mode"] == ("ANGLE" if angle_mode else "ACRO"), 1)
            assert node.session.dirty
            current["armed"] = False
            pump(lambda: not node.session.dirty, 10)
            assert io.values["p_roll"] == "40"
            assert node.report["status"] == "aborted-restored"
        else:
            pump(lambda: node.worker is not None and not node.worker.is_alive(), 100)
            if scenario == "export-failure":
                assert node.report["status"] == "aborted", node.report
                assert node.report["rollback_required"] and node.session.dirty
                assert "export disk full" in node.report["error"]
                current["switch"], current["armed"] = 1000, False
                pump(lambda: node.report.get("status") == "aborted-restored", 10)
                assert io.values["p_roll"] == "40"
            else:
                assert node.report["status"] == "completed-ram-only", node.report
                cli_path = node.report_path.with_suffix(".cli")
                assert node.report["cli_file"] == str(cli_path)
                selected = {a: Gains(**g) for a, g in node.report["selected"].items()}
                assert cli_path.read_text().endswith(bench_pid_core.render_cli(io.profile, selected))
                assert f"mode: {'ANGLE' if angle_mode else 'ACRO'}" in cli_path.read_text()
                assert node.report["start_mode"] == ("ANGLE" if angle_mode else "ACRO")
                assert current["armed"]
                assert current["apply_checks"] > 0 and current["powered_sets"] > 0
                assert node.report["reference_quaternion"] != [0, 0, 0, 1]
                if custom_channel:
                    assert read_timeout_injected
        if scenario not in ("complete", "custom-channel", "angle"):
            assert not list(tmp_path.glob("*.cli")) and "cli_file" not in node.report
        assert outputs and max(thrust for _, thrust in outputs) <= .12
        assert all(cmd not in (3, 4) for cmd, _ in io.calls)
        assert fc.count_publishers("/fpv/arm") == 0
        assert list(tmp_path.glob("bench-*.json"))
        assert node.report["tuning_channel"] == tuning_channel
    finally:
        node.stop_output("test ending")
        if node.worker is not None:
            pump(lambda: not node.worker.is_alive(), 5)
        executor.shutdown()
        node.destroy_node()
        fc.destroy_node()
        rclpy.shutdown(context=context)
