#!/usr/bin/env python3
"""SITL end-to-end test for the custom-link bridge node.

Brings up the full chain: Betaflight SITL firmware (with USE_CUSTOM_LINK on
UART3/tcp:5763), a static level FDM feed + RC feed (OFFBOARD switch on, ARM
switch off - arming must come from the host), then the ROS
custom_link_bridge_node over TCP, and finally exercises:

  1. telemetry rates    : IMU ~200 Hz, RC ~100 Hz, state ~10 Hz
  2. time sync          : FC-stamped message headers lag wall clock < 50 ms
  3. host arm           : ~/arm + control stream arms via 0x20 (BOXOFFBOARD)
  4. offboard takeover  : state.mode == OFFBOARD, motors spin at commanded thrust
  5. watchdog release   : stopping the control stream drops OFFBOARD within ~1 s
  6. host disarm        : ~/arm false disarms the FC

Usage (workspace with apm_bridge built and sourced):
  python3 test/sitl_custom_link_test.py --binary <betaflight repo>/obj/betaflight_2026.6.1_SITL
"""

import argparse
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import RCIn, State
from quadrotor_msgs.msg import ControlCommand
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

RC_PORT = 9004
FDM_PORT = 9003
PWM_PORT = 9002
LINK_TCP_PORT = 5763
GRAVITY = 9.80665

BOX_ARM = 0        # permanent id
BOX_OFFBOARD = 58  # permanent id


class Feed:
    """RC (OFFBOARD switch on) + static level FDM + motor listener."""

    def __init__(self, rc=True, fdm=True, motors=True):
        self.threads = []
        self.running = True
        self.rc_channels = [1500, 1500, 1000, 1500] + [1000] * 12  # AERT + AUX
        self.motors = [0.0] * 4
        self.motor_stamp = 0.0
        if rc:
            t = threading.Thread(target=self._rc_loop, daemon=True)
            self.threads.append(t)
        if fdm:
            t = threading.Thread(target=self._fdm_loop, daemon=True)
            self.threads.append(t)
        if motors:
            t = threading.Thread(target=self._motor_loop, daemon=True)
            self.threads.append(t)
        for t in self.threads:
            t.start()

    def set_rc(self, index, value):
        self.rc_channels[index] = value

    def _rc_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t0 = time.monotonic()
        while self.running:
            pkt = struct.pack("<d16H", time.monotonic() - t0, *self.rc_channels)
            sock.sendto(pkt, ("127.0.0.1", RC_PORT))
            time.sleep(0.02)

    def _fdm_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t0 = time.monotonic()
        while self.running:
            now = time.monotonic() - t0
            pkt = struct.pack(
                "<18d",
                now,
                0.0, 0.0, 0.0,              # gyro
                0.0, 0.0, -GRAVITY,         # negated body specific force (level)
                1.0, 0.0, 0.0, 0.0,         # quaternion (level)
                0.0, 0.0, 0.0,              # velocity ENU
                153.0, -27.5, 30.0,         # lon, lat, alt (mirrored by SITL)
                101325.0,                   # pressure
            )
            sock.sendto(pkt, ("127.0.0.1", FDM_PORT))
            time.sleep(0.02)

    def _motor_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", PWM_PORT))
        sock.settimeout(0.2)
        while self.running:
            try:
                data, _ = sock.recvfrom(64)
                if len(data) >= 16:
                    self.motors = list(struct.unpack("<4f", data[:16]))
                    self.motor_stamp = time.time()
            except socket.timeout:
                pass
            except OSError:
                break

    def stop(self):
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)


class BridgeObserver(Node):
    def __init__(self):
        super().__init__("custom_link_test_observer")
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_count = 0
        self.rc_count = 0
        self.state_count = 0
        self.imu_stamp_lag_max = 0.0
        self.imu_acc_mag = None
        self.state = State()
        self.create_subscription(Imu, "/mavros/imu/data", self._imu, qos_be)
        self.create_subscription(RCIn, "/mavros/rc/in", self._rc, qos_be)
        self.create_subscription(State, "/mavros/state", self._state, 10)
        self.arm_pub = self.create_publisher(Bool, "/fpv/arm", 10)
        self.cmd_pub = self.create_publisher(
            ControlCommand, "/fpv/control_command_raw", qos_be)

    def _imu(self, msg):
        self.imu_count += 1
        lag = (self.get_clock().now().nanoseconds / 1e9) - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9
        self.imu_stamp_lag_max = max(self.imu_stamp_lag_max, abs(lag)) if self.imu_count > 50 else 0.0
        a = msg.linear_acceleration
        self.imu_acc_mag = (a.x**2 + a.y**2 + a.z**2) ** 0.5

    def _rc(self, msg):
        self.rc_count += 1

    def _state(self, msg):
        self.state = msg
        self.state_count += 1

    def publish_command(self, thrust, rate=(0.0, 0.0, 0.0), armed=True):
        c = ControlCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.control_mode = ControlCommand.BODY_RATES
        c.armed = armed
        c.collective_thrust = thrust
        c.bodyrates.x, c.bodyrates.y, c.bodyrates.z = rate
        self.cmd_pub.publish(c)


def wait_for(desc, predicate, timeout=15.0, interval=0.1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            print(f"  [ok] {desc}")
            return True
        time.sleep(interval)
    print(f"  [FAIL] {desc} (timeout {timeout}s)")
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True, help="path to betaflight_SITL executable")
    ap.add_argument("--workdir", default="/tmp/custom_link_sitl")
    args = ap.parse_args()
    binary = os.path.abspath(args.binary)

    results = {}

    # 清理可能残留的旧进程：残留的 FDM/RC feeder 会撞上下一次 SITL 的早期
    # 初始化并触发上游竞态（固件冻结在 t≈10ms）。SITL 匹配用行首锚定，
    # 避免匹配到本脚本的 --binary 参数而自杀。
    subprocess.run(["pkill", "-f", f"^{binary}"], check=False)
    subprocess.run(["pkill", "-f", "sitl_feed.py"], check=False)
    subprocess.run(["pkill", "-f", "custom_link_bridge_node"], check=False)
    time.sleep(1.0)

    os.makedirs(args.workdir, exist_ok=True)
    eeprom = os.path.join(args.workdir, "eeprom.bin")
    if os.path.exists(eeprom):
        os.remove(eeprom)

    # ---- provision: ARM on AUX1, OFFBOARD on AUX2 ----
    cfg = os.path.join(args.workdir, "config.txt")
    with open(cfg, "w") as f:
        f.write("\n".join([
            "set small_angle = 180",
            f"aux 0 {BOX_ARM} 0 1700 2100 0 0",
            f"aux 1 {BOX_OFFBOARD} 1 1700 2100 0 0",
        ]) + "\n")
    res = subprocess.run([binary, "--config", cfg], cwd=args.workdir,
                         capture_output=True, text=True, timeout=60)
    if res.returncode != 0 or not os.path.exists(eeprom):
        print("provisioning failed:", res.stdout[-500:], res.stderr[-500:])
        return 1

    # ---- start SITL ----
    sitl_log = open(os.path.join(args.workdir, "sitl.log"), "w")
    sitl = subprocess.Popen(["stdbuf", "-oL", "-eL", binary], cwd=args.workdir,
                            stdout=sitl_log, stderr=sitl_log,
                            preexec_fn=os.setsid)

    # SITL 的 FDM/RC 接收线程在固件 init 完成前就到包会触发上游初始化竞态
    # （固件冻结在 t≈10ms）。TCP 端口可连只说明 init 走到一半，必须等到
    # 遥测字节真正流出（调度器已在跑）才能开始喂 FDM/RC。
    def wait_link_ready(timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                probe = socket.create_connection(("127.0.0.1", LINK_TCP_PORT), timeout=0.3)
                probe.settimeout(2.0)
                try:
                    data = probe.recv(64)
                except socket.timeout:
                    data = b""
                probe.close()
                if data[:1] == b"\xeb" or b"\xeb\x90" in data:
                    return True
            except OSError:
                pass
            time.sleep(0.3)
        return False

    if not wait_link_ready():
        print("SITL did not stream custom link telemetry")
        return 1
    time.sleep(0.5)  # 让探测连接的 close 事件在 dyad 线程里处理完

    feed = Feed()
    feed.set_rc(5, 2000)  # AUX2 high: BOXOFFBOARD switch on (pilot intent)
    # AUX1 (arm switch) stays low - arming must come from the host

    rclpy.init()
    observer = BridgeObserver()

    # ---- start the bridge node ----
    ros_env = dict(os.environ)
    node = subprocess.Popen(
        ["ros2", "run", "apm_bridge", "custom_link_bridge_node", "--ros-args",
         "-p", "link.transport:=tcp",
         "-p", f"link.tcp_port:={LINK_TCP_PORT}",
         "-p", "mass:=2.0",
         "-r", "__node:=custom_link_bridge_under_test",
         "-r", "~/control_command_raw:=/fpv/control_command_raw",
         "-r", "~/arm:=/fpv/arm"],
        env=ros_env, preexec_fn=os.setsid,
        stdout=open(os.path.join(args.workdir, "node.log"), "w"),
        stderr=subprocess.STDOUT)

    spin = threading.Thread(target=lambda: rclpy.spin(observer), daemon=True)
    spin.start()

    try:
        print("== 1. link bring-up ==")
        ok = wait_for("bridge connected + armed state received",
                      lambda: observer.state_count > 0, timeout=30)

        print("== 2. telemetry rates (3 s window) ==")
        i0, r0, s0 = observer.imu_count, observer.rc_count, observer.state_count
        time.sleep(3.0)
        imu_hz = (observer.imu_count - i0) / 3.0
        rc_hz = (observer.rc_count - r0) / 3.0
        state_hz = (observer.state_count - s0) / 3.0
        print(f"  imu {imu_hz:.1f} Hz, rc {rc_hz:.1f} Hz, state {state_hz:.1f} Hz")
        results["imu_rate"] = 160 <= imu_hz <= 240
        results["rc_rate"] = 80 <= rc_hz <= 120
        results["state_rate"] = 8 <= state_hz <= 12

        print("== 3. time sync quality ==")
        results["stamp_lag"] = observer.imu_stamp_lag_max < 0.05
        print(f"  max |now - imu stamp| = {observer.imu_stamp_lag_max*1000:.1f} ms")
        if observer.imu_acc_mag is not None:
            print(f"  |acc| = {observer.imu_acc_mag:.2f} m/s^2")
            results["acc_sane"] = 7.0 < observer.imu_acc_mag < 12.0
        ok &= results.get("imu_rate", False)

        print("== 4. host arm via 0x20 (BOXOFFBOARD switch held by RC feed) ==")
        observer.arm_pub.publish(Bool(data=True))
        # 控制流以 50 Hz 持续发送，arm 位随帧携带
        def command_stream(duration, thrust):
            t_end = time.time() + duration
            while time.time() < t_end:
                observer.publish_command(thrust)
                time.sleep(0.02)
        stream = threading.Thread(target=command_stream, args=(12.0, 0.5), daemon=True)
        stream.start()
        results["host_arm"] = wait_for("armed", lambda: observer.state.armed, timeout=12)

        print("== 5. OFFBOARD takeover + motor response ==")
        results["offboard_mode"] = wait_for("state.mode == OFFBOARD",
                                            lambda: observer.state.mode == "OFFBOARD")

        # 诊断：从 MSP 读 FC 内部电机值，区分 mixer 无输出 vs PWM 发包断
        def msp_motor():
            try:
                s = socket.create_connection(("127.0.0.1", 5761), timeout=1.0)
                s.settimeout(1.0)
                s.sendall(b"$M<" + bytes([0, 104, 104 ^ 0]))
                buf = b""
                for _ in range(10):
                    d = s.recv(128)
                    if not d:
                        break
                    buf += d
                    i = buf.find(b"$M>")
                    if i >= 0 and len(buf) >= i + 5 + 16:
                        s.close()
                        return list(struct.unpack("<4H", buf[i+5:i+5+8]))
                s.close()
            except OSError:
                pass
            return None

        time.sleep(2.0)
        print(f"  fc internal motors (MSP_MOTOR): {msp_motor()}")

        results["motors_spin"] = wait_for(
            "motors above 0.2", lambda: min(feed.motors) > 0.2, timeout=5)
        print(f"  motors: {[f'{m:.2f}' for m in feed.motors]}")
        stream.join(timeout=13)

        print("== 6. watchdog release (revoke arm -> stream stops) ==")
        # arm-only 流（无新鲜命令）会持续携带 arm=1 + 中性油门：OFFBOARD
        # 保持、电机回落 idle——这是设计语义。验证看门狗须先撤销解锁请求，
        # 节点随即停发 0x20，FC 侧超时退回 RC。
        observer.arm_pub.publish(Bool(data=False))
        results["watchdog_release"] = wait_for(
            "OFFBOARD dropped", lambda: observer.state.mode != "OFFBOARD", timeout=3)
        wait_for("motors back to idle", lambda: max(feed.motors) < 0.15, timeout=3)

        print("== 7. host disarm ==")
        # 再开一小段控制流以携带 arm=0 边沿
        stream2 = threading.Thread(target=command_stream, args=(2.0, 0.0), daemon=True)
        stream2.start()
        observer.arm_pub.publish(Bool(data=False))
        results["host_disarm"] = wait_for(
            "disarmed", lambda: not observer.state.armed, timeout=5)
        stream2.join(timeout=3)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
        feed.stop()
        for p in (node, sitl):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except OSError:
                pass
        for p in (node, sitl):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        sitl_log.close()

    print("\n===== summary =====")
    all_ok = True
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
        all_ok &= bool(v)
    print("RESULT:", "PASS" if all_ok else "FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
