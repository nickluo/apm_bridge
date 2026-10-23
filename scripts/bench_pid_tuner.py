#!/usr/bin/env python3
"""Opt-in, RC-switch-controlled captive-rig rate-PID tuning; never arms or saves the FC."""

from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from mavros_msgs.msg import RCIn, State
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from quadrotor_msgs.msg import ControlCommand
from quadrotor_msgs.srv import CliCommand

from bench_pid_core import (AXES, AngleLimits, HoldLimits, Limits, TuningSession, attitude_error,
                            holding_rates, measure, measure_angle, render_cli, setting_value)
from sitl_support import euler


class BenchPidTuner(Node):
    def __init__(self, **kwargs):
        super().__init__("bench_pid_tuner", **kwargs)
        self.enabled = self.declare_parameter("enabled", False).value
        self.confirmed = self.declare_parameter("rig_confirmed", False).value
        self.axes = list(self.declare_parameter("axes", ["roll", "pitch", "yaw"]).value)
        self.limits = Limits(**{
            name: self.declare_parameter(name, value).value
            for name, value in asdict(Limits()).items()})
        self.limits.validate()
        self.hold = HoldLimits(**{
            name: self.declare_parameter(name, value).value
            for name, value in asdict(HoldLimits()).items()})
        self.hold.validate()
        self.angle = AngleLimits(**{
            name: self.declare_parameter(name, value).value
            for name, value in asdict(AngleLimits()).items()})
        self.angle.validate()
        self.start_mode = "ACRO"
        self.angle_reference = None
        self.angle_center = (0., 0.)
        self.angle_trim = (0., 0.)
        if self.hold.hold_rate_dps + self.limits.rate_dps >= self.limits.max_rate_dps:
            raise ValueError("hold plus excitation rate must stay below max_rate_dps")
        if self.hold.settle_angle_deg >= self.limits.max_tilt_deg:
            raise ValueError("settle_angle_deg must stay below max_tilt_deg")
        self.tuning_channel = self.declare_parameter("tuning_channel", 8).value
        if type(self.tuning_channel) is not int or not 0 <= self.tuning_channel < 16:
            raise ValueError("tuning_channel must be a zero-based integer in [0,15]")
        if not self.axes or len(set(self.axes)) != len(self.axes) or any(a not in AXES for a in self.axes):
            raise ValueError("axes must be a nonempty, unique subset of roll/pitch/yaw")
        self.output_dir = Path(self.declare_parameter(
            "output_dir", "~/.ros/bench_pid").value).expanduser().resolve()
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.worker = None
        self.active = self.owned = self.low_seen = self.requested = False
        self.state = self.rc = self.imu = None
        self.state_at = self.rc_at = self.imu_at = self.failsafe_at = 0.0
        self.imu_stamp = None
        self.failsafe = True
        self.mission = False
        self.command = (0.0, (0.0, 0.0, 0.0))
        self.reference = None
        self.command_mode = "zero"
        self.excitation_axis = None
        self.excitation_rate = 0.0
        self.apply_deadline = None
        self.session = None
        self.report_path = None
        self.report = {}
        self.abort_reason = None
        self.recovering = self.recovery_failed = False
        self.next_graph_check = 0.0
        self.command_pub = self.create_publisher(
            ControlCommand, "/fpv/control_command_raw", qos_profile_sensor_data)
        self.mode_pub = self.create_publisher(Bool, "/fpv/offboard", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.cli_client = self.create_client(CliCommand, "/fpv/cli")
        self.create_subscription(RCIn, "/mavros/rc/in", self.on_rc, qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state", self.on_state, 10)
        self.create_subscription(Imu, "/mavros/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, "/fpv/failsafe", self.on_failsafe, 10)
        self.create_subscription(Bool, "/fpv/mission_state", self.on_mission, 10)
        self.create_timer(.01, self.tick)
        self.status("DISABLED" if not self.enabled or not self.confirmed
                    else f"IDLE: CH{self.tuning_channel + 1} low then high")

    def status(self, message):
        self.status_pub.publish(String(data=message))
        self.get_logger().info(message)

    def on_state(self, msg):
        with self.lock:
            self.state, self.state_at = msg, time.monotonic()

    def on_failsafe(self, msg):
        with self.lock:
            self.failsafe, self.failsafe_at = msg.data, time.monotonic()

    def on_mission(self, msg):
        with self.lock:
            self.mission = msg.data
            if msg.data and self.active:
                self.stop_output("competing MISSION")

    def on_rc(self, msg):
        with self.lock:
            previous = self.rc
            self.rc, self.rc_at = list(msg.channels), time.monotonic()
            channel = self.tuning_channel
            if len(self.rc) <= channel:
                self.low_seen = self.requested = False
                if self.active or (self.worker is not None and self.worker.is_alive()):
                    self.stop_output(f"RC stream is missing tuning channel CH{channel + 1}")
                return
            if self.rc[channel] < 1100:
                self.low_seen = True
                self.requested = False
                if self.active or (self.worker is not None and self.worker.is_alive()):
                    self.stop_output(f"CH{channel + 1} cancelled")
            elif self.rc[channel] > 1900 and self.low_seen \
                    and (previous is None or len(previous) <= channel or previous[channel] <= 1900):
                self.low_seen = False
                self.requested = True

    def on_imu(self, msg):
        with self.lock:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q = msg.orientation
            values = (q.x, q.y, q.z, q.w, msg.angular_velocity.x,
                      msg.angular_velocity.y, msg.angular_velocity.z)
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not all(math.isfinite(v) for v in values) or not -.05 <= age <= .2 \
                    or not .9 <= sum(v * v for v in values[:4]) <= 1.1 \
                    or (self.imu_stamp is not None and stamp <= self.imu_stamp):
                if self.active:
                    self.stop_output("invalid/stale/reordered FC IMU")
                return
            self.imu, self.imu_stamp, self.imu_at = msg, stamp, time.monotonic()

    def health(self, require_offboard=False):
        now = time.monotonic()
        if self.state is None or self.rc is None or self.imu is None \
                or now - self.state_at > .3 or now - self.rc_at > .15 \
                or now - self.imu_at > .1 or now - self.failsafe_at > .3:
            raise RuntimeError("fresh RC/state/IMU/failsafe telemetry required")
        if not self.state.connected or not self.state.armed or self.failsafe or self.mission:
            raise RuntimeError("armed, connected, non-failsafe and mission-idle FC required")
        if len(self.rc) <= self.tuning_channel or self.rc[self.tuning_channel] <= 1900:
            raise RuntimeError("tuning switch not high or channel unavailable")
        expected = "OFFBOARD|ANGLE" if self.start_mode == "ANGLE" else "OFFBOARD"
        if require_offboard and self.state.mode != expected:
            raise RuntimeError("FC OFFBOARD authority or selected tuning mode lost")
        if not require_offboard and self.state.mode not in (self.start_mode, expected):
            raise RuntimeError("selected ACRO/ANGLE startup mode changed")
        rates = [math.degrees(v) for v in (self.imu.angular_velocity.x,
                                         self.imu.angular_velocity.y, self.imu.angular_velocity.z)]
        if max(abs(v) for v in rates) > self.limits.max_rate_dps:
            raise RuntimeError("rig angular-rate limit exceeded")
        if self.reference is not None:
            error = attitude_error(self.reference, self.orientation())
            if math.degrees(math.sqrt(sum(v * v for v in error))) > self.limits.max_tilt_deg:
                raise RuntimeError("rig excursion limit exceeded")
        if self.apply_deadline is not None and now >= self.apply_deadline:
            raise RuntimeError("zero-output PID application window expired")

    def orientation(self):
        q = self.imu.orientation
        return (q.x, q.y, q.z, q.w)

    def refresh_command(self):
        if self.command_mode == "zero":
            self.command = (0.0, (0.0, 0.0, 0.0))
            return
        axis = self.excitation_axis if self.command_mode == "excite" else None
        if self.start_mode == "ANGLE":
            yaw_rate = holding_rates(self.angle_reference, self.orientation(), self.hold)[2]
            roll, pitch = self.angle_center
            if axis == "roll":
                roll += self.excitation_rate
            elif axis == "pitch":
                pitch += self.excitation_rate
            elif axis == "yaw":
                yaw_rate = self.excitation_rate
            self.command = (self.limits.throttle, tuple(math.radians(v) for v in (roll, pitch, yaw_rate)))
            return
        rates = list(holding_rates(self.reference, self.orientation(), self.hold, axis))
        if axis is not None:
            rates[AXES.index(axis)] = self.excitation_rate
        self.command = (self.limits.throttle, tuple(math.radians(v) for v in rates))

    def publish_command(self, throttle, rates):
        angle = self.start_mode == "ANGLE"
        msg = ControlCommand(armed=True, control_mode=ControlCommand.ATTITUDE if angle else ControlCommand.BODY_RATES,
                             collective_thrust=float(throttle))
        msg.header.stamp = self.get_clock().now().to_msg()
        if angle:
            roll, pitch, yaw_rate = rates
            cr, sr = math.cos(roll/2), math.sin(roll/2)
            cp, sp = math.cos(pitch/2), math.sin(pitch/2)
            msg.orientation.x, msg.orientation.y = sr*cp, cr*sp
            msg.orientation.z, msg.orientation.w = -sr*sp, cr*cp
            msg.bodyrates.z = yaw_rate
        else:
            msg.bodyrates.x, msg.bodyrates.y, msg.bodyrates.z = rates
        self.command_pub.publish(msg)

    def stop_output(self, reason):
        self.cancel.set()
        self.abort_reason = reason
        self.command_mode = "zero"
        self.apply_deadline = None
        self.command = (0.0, (0.0, 0.0, 0.0))
        if self.active:
            self.publish_command(*self.command)
            self.mode_pub.publish(Bool(data=False))
            self.status(f"STOP: {reason}; disarm using the independent ARM switch")
        self.active = self.owned = False

    def tick(self):
        with self.lock:
            busy = self.worker is not None and self.worker.is_alive()
            if self.active:
                try:
                    self.health(self.owned)
                    if time.monotonic() >= self.next_graph_check:
                        self.next_graph_check = time.monotonic() + .1
                        if self.count_publishers("/fpv/control_command") \
                                or self.count_publishers("/fpv/control_command_raw") != 1:
                            raise RuntimeError("competing control publisher appeared")
                    self.refresh_command()
                    self.publish_command(*self.command)
                except (RuntimeError, ValueError) as exc:
                    self.stop_output(str(exc))
            if self.requested:
                self.requested = False
                if not self.enabled or not self.confirmed:
                    self.status("REFUSED: enabled and rig_confirmed must both be true")
                elif busy or (self.session is not None and self.session.dirty):
                    self.status("REFUSED: session busy or rollback required; disarm first")
                else:
                    self.worker = threading.Thread(target=self.run_session, daemon=True)
                    self.worker.start()
            if not busy and not self.recovery_failed and self.session is not None and self.session.dirty \
                    and self.state is not None and not self.state.armed \
                    and time.monotonic() - self.state_at < .3 and self.rc is not None \
                    and len(self.rc) > self.tuning_channel and self.rc[self.tuning_channel] < 1100:
                self.worker = threading.Thread(target=self.recover, daemon=True)
                self.worker.start()

    def check(self):
        with self.lock:
            if self.cancel.is_set():
                raise RuntimeError(self.abort_reason or "cancelled")
            self.health(self.owned)
            if time.monotonic() - self.started > self.limits.timeout_s:
                raise RuntimeError("bench session time limit exceeded")

    def cli(self, cmd, args):
        live_apply = cmd == 5 and args not in ("check", "angle") and not self.recovering and self.active
        if not live_apply:
            return self.cli_request(cmd, args)
        try:
            with self.lock:
                self.check()
                self.apply_deadline = time.monotonic() + self.hold.apply_window_s
                self.command_mode = "zero"
                self.refresh_command()
                self.publish_command(*self.command)
            # Allow the bridge's 100 Hz sender and FC task to consume the zero frame.
            self.drive(.03, 0.0)
            with self.lock:
                if max(abs(v) for v in self.rates_dps()) >= 3:
                    raise RuntimeError("rig moving during zero-output PID application")
            result = self.cli_request(cmd, args)
            with self.lock:
                self.check()
                self.apply_deadline = None
                self.command_mode = "hold"
                self.refresh_command()
            return result
        except Exception:
            with self.lock:
                self.stop_output("PID application failed; disarm for recovery")
            raise

    def cli_request(self, cmd, args):
        readonly = cmd in (0, 2) or (cmd == 5 and args in ("check", "angle"))
        for attempt in range(3 if readonly else 1):
            if self.recovering:
                with self.lock:
                    if self.state is None or self.state.armed or time.monotonic() - self.state_at > .3:
                        raise RuntimeError("rollback requires fresh disarmed state throughout")
            elif self.cancel.is_set():
                raise RuntimeError(self.abort_reason or "cancelled")
            elif self.active or self.owned:
                self.check()
            if not self.cli_client.service_is_ready():
                raise RuntimeError("/fpv/cli unavailable")
            future = self.cli_client.call_async(CliCommand.Request(cmd=cmd, args=args))
            deadline = time.monotonic() + 4
            timed_out = False
            while not future.done():
                try:
                    if not self.recovering:
                        if self.active or self.owned:
                            self.check()
                        elif self.cancel.is_set():
                            raise RuntimeError(self.abort_reason or "cancelled")
                except Exception:
                    self.cli_client.remove_pending_request(future)
                    raise
                if time.monotonic() >= deadline:
                    self.cli_client.remove_pending_request(future)
                    timed_out = True
                    break
                time.sleep(.01)
            result = None if timed_out else future.result()
            if self.apply_deadline is not None:
                self.check()
            if result is not None and result.ok and result.status == 0:
                return result.text
            retryable = timed_out or (result is not None and not result.ok and result.error == "timeout")
            if readonly and retryable and attempt < 2:
                self.get_logger().warning(f"Read-only CLI timeout for {args}; retry {attempt + 1}/2")
                time.sleep(.02)
                continue
            if timed_out:
                raise RuntimeError(f"CLI timeout; FC write outcome unknown: {cmd} {args}")
            raise RuntimeError(f"CLI error for {args}: {result}")

    def drive(self, duration, throttle, axis=None, rate=0.0):
        samples = []
        deadline = time.monotonic() + duration
        last = None
        while time.monotonic() < deadline:
            with self.lock:
                self.check()
                self.command_mode = "zero" if throttle == 0 else "excite" if axis is not None else "hold"
                self.excitation_axis, self.excitation_rate = axis, rate
                self.refresh_command()
                if self.imu_stamp != last:
                    last = self.imu_stamp
                    measured = list(self.rates_dps())
                    index = AXES.index(axis) if axis else 0
                    if self.start_mode == "ANGLE" and axis in ("roll", "pitch"):
                        angles = [math.degrees(v) for v in euler(self.orientation())]
                        delta = [angles[i] - self.angle_trim[i] for i in range(2)]
                        samples.append((last, rate, delta[index], abs(delta[1-index]), max(abs(v) for v in measured)))
                    else:
                        samples.append((last, rate, measured[index],
                                        math.hypot(*(v for i, v in enumerate(measured) if i != index))))
            time.sleep(.01)
        return samples

    def rates_dps(self):
        return tuple(math.degrees(v) for v in (self.imu.angular_velocity.x,
                     self.imu.angular_velocity.y, self.imu.angular_velocity.z))

    def idle(self):
        deadline = time.monotonic() + self.hold.settle_timeout_s
        settled_since = None
        while time.monotonic() < deadline:
            with self.lock:
                self.check()
                self.command_mode = "hold"
                self.refresh_command()
                target = self.angle_reference if self.start_mode == "ANGLE" else self.reference
                error = attitude_error(target, self.orientation())
                settled = math.degrees(math.sqrt(sum(v * v for v in error))) <= self.hold.settle_angle_deg \
                    and max(abs(v) for v in self.rates_dps()) <= self.hold.settle_rate_dps
                now = time.monotonic()
                settled_since = (settled_since if settled_since is not None else now) if settled else None
                if settled_since is not None and now - settled_since >= self.hold.settle_time_s:
                    return
            time.sleep(.01)
        raise RuntimeError("powered attitude hold did not settle; inspect rig balance/throttle")

    def trial(self, axis):
        self.status(f"TRIAL {axis}: {self.report.get('active_gains', 'original PID')}")
        self.idle()
        samples = []
        angle_trial = self.start_mode == "ANGLE" and axis in ("roll", "pitch")
        amplitude = self.angle.angle_step_deg if angle_trial else self.limits.rate_dps
        pulse = self.angle.angle_step_s if angle_trial else self.limits.pulse_s
        tail = self.angle.angle_tail_s if angle_trial else self.limits.tail_s
        for sign in (1, -1):
            samples += self.drive(pulse, self.limits.throttle, axis, sign * amplitude)
            samples += self.drive(tail, self.limits.throttle, axis, 0.0)
        score = (measure_angle if angle_trial else measure)(samples, amplitude)
        self.report["measurements"].append({"axis": axis, "score": score, "samples": samples})
        self.write_report()
        return score

    def write_report(self, accepted=False):
        self.report["rollback_required"] = self.session is not None and self.session.dirty and not accepted
        temporary = self.report_path.with_suffix(".tmp")
        try:
            temporary.write_text(json.dumps(self.report, indent=2), encoding="utf-8")
            temporary.replace(self.report_path)
        finally:
            temporary.unlink(missing_ok=True)

    def complete_session(self, selected):
        cli_path = self.report_path.with_suffix(".cli")
        temporary = cli_path.with_suffix(".cli.tmp")
        try:
            header = f"# Tuning start mode: {self.start_mode}; " \
                + ("closed-loop ANGLE response, fixed outer-loop settings.\n" if self.start_mode == "ANGLE"
                   else "body-rate response.\n")
            temporary.write_text(header + render_cli(self.session.profile, selected), encoding="utf-8")
            self.check()
            temporary.replace(cli_path)
            self.report.update(selected={a: asdict(g) for a, g in selected.items()},
                               status="completed-ram-only", cli_file=str(cli_path))
            self.write_report(accepted=True)
            with self.lock:
                self.check()
                self.stop_output(f"completed; PID retained in RAM only; CLI: {cli_path}")
                self.session.dirty = False
        except Exception:
            self.report.pop("cli_file", None)
            cli_path.unlink(missing_ok=True)
            raise
        finally:
            temporary.unlink(missing_ok=True)

    def run_session(self):
        try:
            with self.lock:
                self.report_path, self.report = None, {}
                self.reference = None
                if self.state is None or self.state.mode not in ("ACRO", "ANGLE"):
                    raise RuntimeError("start in ACRO or ANGLE, not an existing OFFBOARD session")
                self.start_mode = self.state.mode
                self.health()
                if self.count_publishers("/fpv/control_command") \
                        or self.count_publishers("/fpv/control_command_raw") != 1:
                    raise RuntimeError("stop all other control publishers before bench tuning")
                self.cancel.clear()
                self.abort_reason = None
                self.started = time.monotonic()
                q = self.imu.orientation
                self.reference = (q.x, q.y, q.z, q.w)
                self.angle_reference = None
                if self.start_mode == "ANGLE":
                    if self.angle.angle_step_deg + 5 >= self.limits.max_tilt_deg:
                        raise RuntimeError("ANGLE step plus entry tilt must stay below max_tilt_deg")
                    roll, pitch, yaw = euler(self.reference)
                    if max(abs(math.degrees(v)) for v in (roll, pitch)) > 5:
                        raise RuntimeError("ANGLE tuning starts within 5 degrees of level; reposition rig safely")
                    self.angle_reference = (0., 0., math.sin(yaw/2), math.cos(yaw/2))
                    self.angle_center = (0., 0.)
                self.command_mode = "zero"
                self.apply_deadline = None
                self.report = {"status": "starting", "axes": {}, "measurements": [],
                               "limits": asdict(self.limits), "hold": asdict(self.hold),
                               "start_mode": self.start_mode, "angle": asdict(self.angle),
                               "measurement": "closed-loop-angle-step" if self.start_mode == "ANGLE" else "rate-step",
                               "reference_quaternion": self.reference, "tuning_channel": self.tuning_channel,
                               "persisted": False}
                self.output_dir.mkdir(parents=True, exist_ok=True)
                self.report_path = self.output_dir / f"bench-{time.time_ns()}.json"
                self.session = TuningSession(self.cli, self.trial, self.idle, self.check,
                                             self.limits, self.report)
            self.write_report()
            if self.start_mode == "ANGLE":
                if self.cli(5, "angle") != "ANGLE OFFBOARD 1":
                    raise RuntimeError("firmware lacks native ANGLE offboard support")
                names = ("acc_trim_roll", "acc_trim_pitch", "angle_p_gain", "angle_feedforward",
                         "angle_feedforward_smoothing_ms", "angle_limit", "angle_earth_ref", "level_race_mode")
                outer = {name: setting_value(name, self.cli(0, name)) for name in names}
                if not 1 <= int(outer["angle_p_gain"]) <= 200:
                    raise RuntimeError("ANGLE outer-loop P must already be nonzero")
                self.report["angle_outer_settings"] = outer
                self.angle_trim = tuple(int(outer[f"acc_trim_{axis}"]) / 10 for axis in ("roll", "pitch"))
                if max(abs(v) for v in self.angle_trim) > 3:
                    raise RuntimeError("ANGLE tuning requires small calibrated accelerometer trims")
                roll, pitch = (math.radians(v) for v in self.angle_trim)
                yaw = euler(self.reference)[2]
                cr, sr = math.cos(roll/2), math.sin(roll/2)
                cp, sp = math.cos(pitch/2), math.sin(pitch/2)
                cy, sy = math.cos(yaw/2), math.sin(yaw/2)
                self.angle_reference = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                                        cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
                self.angle_center = (0., 0.)
                self.report["angle_center_quaternion"] = self.angle_reference
                self.report["angle_trim_deg"] = self.angle_trim
                self.write_report()
            with self.lock:
                self.check()
                self.active = True
                self.command_mode = "hold"
                self.refresh_command()
                self.publish_command(*self.command)
                self.mode_pub.publish(Bool(data=True))
            deadline = time.monotonic() + 3
            expected_mode = "OFFBOARD|ANGLE" if self.start_mode == "ANGLE" else "OFFBOARD"
            while self.state.mode != expected_mode:
                self.check()
                if time.monotonic() >= deadline:
                    raise RuntimeError("FC refused OFFBOARD")
                time.sleep(.01)
            with self.lock:
                self.owned = True
                self.command_mode = "hold"
                self.refresh_command()
            self.idle()
            self.session.snapshot()
            self.report["original"] = {a: asdict(g) for a, g in self.session.original.items()}
            self.report["profile"] = self.session.profile
            self.write_report()  # No PID writes until the original gains are on disk.
            selected = self.session.run(self.axes)
            self.idle()
            if self.start_mode == "ANGLE":
                for name, original in self.report["angle_outer_settings"].items():
                    if setting_value(name, self.cli(0, name)) != original:
                        raise RuntimeError("ANGLE outer-loop settings changed during tuning")
            self.complete_session(selected)
        except Exception as exc:
            with self.lock:
                self.stop_output(str(exc))
            self.get_logger().error(f"Bench tuning failed: {exc}")
            if self.report_path is not None:
                self.report["status"], self.report["error"] = "aborted", str(exc)
                self.write_report()

    def recover(self):
        try:
            self.recovering = True
            self.session.restore()
            self.report["status"] = "aborted-restored"
            self.write_report()
            self.status(f"Original PID restored and applied; CH{self.tuning_channel + 1} low then high for a new run")
        except Exception as exc:
            self.get_logger().error(f"ROLLBACK REQUIRED: {exc}; original gains in {self.report_path}")
            # Do not retry unacknowledged writes indefinitely.
            self.enabled, self.recovery_failed = False, True
        finally:
            self.recovering = False


def main():
    # Keep ROS alive long enough to publish zero/release on SIGINT/SIGTERM.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    node = None
    try:
        node = BenchPidTuner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            with node.lock:
                node.stop_output("node shutting down")
                if node.session is not None and node.session.dirty:
                    node.get_logger().error(f"Rollback may be required; see {node.report_path}")
            if node.worker is not None and node.worker.is_alive():
                node.worker.join(timeout=4.5)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
