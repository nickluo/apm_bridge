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

from bench_pid_core import AXES, Limits, TuningSession, measure, render_cli
from sitl_support import conjugate, euler, multiply


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
        if require_offboard and self.state.mode != "OFFBOARD":
            raise RuntimeError("FC OFFBOARD authority lost")
        if not require_offboard and self.state.mode not in ("ACRO", "OFFBOARD"):
            raise RuntimeError("ACRO required; do not tune in stabilized/autopilot modes")
        rates = [math.degrees(v) for v in (self.imu.angular_velocity.x,
                                         self.imu.angular_velocity.y, self.imu.angular_velocity.z)]
        if max(abs(v) for v in rates) > self.limits.max_rate_dps:
            raise RuntimeError("rig angular-rate limit exceeded")
        if self.reference is not None:
            q = self.imu.orientation
            angles = euler(multiply(conjugate(self.reference), (q.x, q.y, q.z, q.w)))
            if max(abs(math.degrees(v)) for v in angles) > self.limits.max_tilt_deg:
                raise RuntimeError("rig excursion limit exceeded")

    def publish_command(self, throttle, rates):
        msg = ControlCommand(armed=True, control_mode=ControlCommand.BODY_RATES,
                             collective_thrust=float(throttle))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.bodyrates.x, msg.bodyrates.y, msg.bodyrates.z = rates
        self.command_pub.publish(msg)

    def stop_output(self, reason):
        self.cancel.set()
        self.abort_reason = reason
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
                    self.publish_command(*self.command)
                except RuntimeError as exc:
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
        readonly = cmd in (0, 2) or (cmd == 5 and args == "check")
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
                if time.monotonic() >= deadline:
                    self.cli_client.remove_pending_request(future)
                    timed_out = True
                    break
                time.sleep(.01)
            result = None if timed_out else future.result()
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
            self.check()
            with self.lock:
                rates = [0.0, 0.0, 0.0]
                if axis is not None:
                    rates[AXES.index(axis)] = math.radians(rate)
                self.command = (throttle, tuple(rates))
                if self.imu_stamp != last:
                    last = self.imu_stamp
                    measured = [math.degrees(v) for v in (
                        self.imu.angular_velocity.x, self.imu.angular_velocity.y, self.imu.angular_velocity.z)]
                    index = AXES.index(axis) if axis else 0
                    samples.append((last, rate, measured[index],
                                    math.hypot(*(v for i, v in enumerate(measured) if i != index))))
            time.sleep(.01)
        with self.lock:
            self.command = (0.0, (0.0, 0.0, 0.0))
        return samples

    def idle(self):
        self.drive(.7, 0.0)
        with self.lock:
            if max(abs(v) for v in (self.imu.angular_velocity.x,
                                   self.imu.angular_velocity.y, self.imu.angular_velocity.z)) > math.radians(3):
                raise RuntimeError("rig did not stop before PID application")

    def trial(self, axis):
        self.status(f"TRIAL {axis}: {self.report.get('active_gains', 'original PID')}")
        self.drive(.5, self.limits.throttle)
        samples = []
        for sign in (1, -1):
            samples += self.drive(self.limits.pulse_s, self.limits.throttle,
                                  axis, sign * self.limits.rate_dps)
            samples += self.drive(self.limits.tail_s, self.limits.throttle, axis, 0.0)
        score = measure(samples, self.limits.rate_dps)
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
            temporary.write_text(render_cli(self.session.profile, selected), encoding="utf-8")
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
                self.health()
                if self.state.mode != "ACRO":
                    raise RuntimeError("start in ACRO, not an existing OFFBOARD session")
                if self.count_publishers("/fpv/control_command") \
                        or self.count_publishers("/fpv/control_command_raw") != 1:
                    raise RuntimeError("stop all other control publishers before bench tuning")
                self.cancel.clear()
                self.abort_reason = None
                self.started = time.monotonic()
                q = self.imu.orientation
                self.reference = (q.x, q.y, q.z, q.w)
                self.report = {"status": "starting", "axes": {}, "measurements": [],
                               "limits": asdict(self.limits), "tuning_channel": self.tuning_channel,
                               "persisted": False}
                self.output_dir.mkdir(parents=True, exist_ok=True)
                self.report_path = self.output_dir / f"bench-{time.time_ns()}.json"
                self.session = TuningSession(self.cli, self.trial, self.idle, self.check,
                                             self.limits, self.report)
            self.session.snapshot()
            self.report["original"] = {a: asdict(g) for a, g in self.session.original.items()}
            self.report["profile"] = self.session.profile
            self.write_report()  # No PID writes until the original gains are on disk.
            with self.lock:
                self.check()
                self.active = True
                self.command = (0.0, (0.0, 0.0, 0.0))
                self.publish_command(*self.command)
                self.mode_pub.publish(Bool(data=True))
            deadline = time.monotonic() + 3
            while self.state.mode != "OFFBOARD":
                self.check()
                if time.monotonic() >= deadline:
                    raise RuntimeError("FC refused OFFBOARD")
                time.sleep(.01)
            self.owned = True
            self.idle()
            selected = self.session.run(self.axes)
            self.idle()
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
