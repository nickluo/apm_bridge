#!/usr/bin/env python3
"""Custom-protocol-only SITL/ROS odometry regression; uses synthetic sensors, not UE.

Run in a sourced ROS workspace with ports 5761/5763 free:
  python3 test/sitl_odometry_test.py --binary /path/to/betaflight_SITL.elf
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import GPSRAW, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from quadrotor_msgs.msg import ControlCommand
from std_msgs.msg import Bool


ODOMETRY = struct.Struct("<IHH3i3h3h3h6fh")
assert ODOMETRY.size == 64


def encode(message, payload):
    body = bytes((message, len(payload), 0)) + payload
    crc = 0
    for value in body:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 65535
    return b"\xeb\x90" + body + struct.pack("<H", crc)


class Parser:
    def __init__(self):
        self.buffer = b""

    def feed(self, data):
        self.buffer += data
        frames = []
        while len(self.buffer) >= 7:
            start = self.buffer.find(b"\xeb\x90")
            if start < 0:
                self.buffer = self.buffer[-1:]
                break
            self.buffer = self.buffer[start:]
            length = self.buffer[3]
            if not 0 < length <= 64:
                raise RuntimeError("invalid custom-link payload length")
            size = length + 7
            if len(self.buffer) < size:
                break
            frame, self.buffer = self.buffer[:size], self.buffer[size:]
            message, payload = frame[2], frame[5:-2]
            if encode(message, payload)[-2:] != frame[-2:]:
                # Seq contributes to CRC; tests generate seq=0, FC has a shared counter.
                body = frame[2:-2]
                crc = 0
                for value in body:
                    crc ^= value << 8
                    for _ in range(8):
                        crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 65535
                if struct.pack("<H", crc) != frame[-2:]:
                    raise RuntimeError("custom-link CRC mismatch")
            frames.append((message, payload))
        return frames


def shutdown(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=3)


class Observer(Node):
    def __init__(self):
        super().__init__("sitl_odometry_test")
        self.state, self.gps = None, None
        self.imu = None
        self.gimbal = None
        self.state_transitions = []
        self.odometry = []
        self.create_subscription(State, "/mavros/state", self.on_state, 10)
        self.create_subscription(GPSRAW, "/mavros/gpsstatus/gpsraw", self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/fpv/odom", self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(Imu, "/mavros/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Imu, "/fpv/gimbal", self.on_gimbal, 10)
        self.arm = self.create_publisher(Bool, "/fpv/arm", 10)
        self.command = self.create_publisher(ControlCommand, "/fpv/control_command_raw",
                                             qos_profile_sensor_data)

    def on_state(self, message):
        if self.state is None or self.state.armed != message.armed:
            self.state_transitions.append((time.monotonic(), message.armed, message.mode))
        self.state = message

    def on_gps(self, message):
        self.gps = message

    def on_odometry(self, message):
        self.odometry.append((time.monotonic(), message))

    def on_imu(self, message):
        self.imu = message

    def on_gimbal(self, message):
        self.gimbal = message


class Simulation:
    def __init__(self, sock, node, gps_sock=None):
        self.sock, self.node = sock, node
        self.gps_sock = gps_sock
        self.nmea_fix = 3
        self.parser = Parser()
        self.frames = []
        self.fast_attitude = None
        self.lat, self.lon, self.alt = 470000000, 80000000, 12300
        self.pressure = 101325
        self.gps_enabled = True
        self.control_enabled = False
        self.heading_rad = 0.0
        self.next_imu = self.next_env = self.next_rc = self.next_control = 0.0
        self.start = time.monotonic()

    def tick(self):
        now = time.monotonic()
        ts = round((now - self.start) * 1e6) & 0xffffffff
        if now >= self.next_imu:
            self.sock.sendall(encode(0x21, struct.pack("<I3h3h4h", ts, 0, 0, 0,
                                                     0, 0, 1000,
                                                     round(16384 * math.cos(self.heading_rad / 2)),
                                                     0, 0, round(16384 * math.sin(self.heading_rad / 2)))))
            self.next_imu = now + .004
        if now >= self.next_env and self.gps_enabled:
            self.sock.sendall(encode(0x22, struct.pack(
                "<IIhiii3hHHBB", ts, self.pressure, 2000, self.lat, self.lon, self.alt,
                0, 0, 0, 0, 0, 1, 12)))
            self.next_env = now + .05
            if self.gps_sock is not None:
                def sentence(fields):
                    body = ",".join(fields)
                    checksum = 0
                    for value in body.encode():
                        checksum ^= value
                    return f"${body}*{checksum:02X}\r\n".encode()
                def coordinate(value, width):
                    degrees = abs(value) / 1e7
                    whole = int(degrees)
                    return f"{whole:0{width}d}{(degrees - whole) * 60:09.6f}"
                self.gps_sock.sendall(sentence(
                    ["GPGGA", time.strftime("%H%M%S") + ".00", coordinate(self.lat, 2),
                     "N" if self.lat >= 0 else "S", coordinate(self.lon, 3),
                     "E" if self.lon >= 0 else "W", "1", "12", "1.00",
                     str(self.alt / 100), "M", "0", "M", "", ""]) +
                    sentence(["GPGSA", "A", str(self.nmea_fix)] + [""] * 12 +
                             ["1.00", "1.00", "1.00"]))
        if now >= self.next_rc:
            channels = [1500, 1500, 1000, 1500] + [1000] * 12
            self.sock.sendall(encode(0x23, struct.pack("<I16H", ts, *channels)))
            self.next_rc = now + .02
        if now >= self.next_control and self.control_enabled:
            self.node.command.publish(ControlCommand(
                armed=True, control_mode=ControlCommand.BODY_RATES, collective_thrust=0.0))
            self.next_control = now + .02
        try:
            data = self.sock.recv(16384)
        except socket.timeout:
            data = None
        if data == b"":
            raise RuntimeError("custom-link relay disconnected")
        if data:
            for message, payload in self.parser.feed(data):
                if message == 0x14:
                    self.frames.append((now, ODOMETRY.unpack(payload)))
                elif message == 0x10:
                    self.fast_attitude = struct.unpack("<I9h", payload)[7:10]
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0)
        time.sleep(.001)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.tick()

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError("SITL/ROS condition timed out")
            self.tick()

    def arm(self):
        self.control_enabled = True
        self.node.arm.publish(Bool(data=True))
        self.wait(lambda: self.node.state is not None and self.node.state.armed)

    def disarm(self):
        self.node.arm.publish(Bool(data=False))
        self.wait(lambda: self.node.state is not None and not self.node.state.armed)
        self.control_enabled = False
        self.pump(.7)

def decoder_test(output):
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    server.settimeout(10)
    port = server.getsockname()[1]
    process = connection = None
    node = Observer()
    try:
        process = subprocess.Popen([
            str(Path(get_package_prefix("apm_bridge")) / "lib/apm_bridge/custom_link_bridge_node"), "--ros-args",
            "-p", "link.transport:=tcp", "-p", f"link.tcp_port:={port}", "-p", "gimbal_port:=''",
            "-p", "gimbal.pitch_hold:=-20.0",
        ], stdout=(output / "decoder.log").open("w"), stderr=subprocess.STDOUT)
        connection = server.accept()[0]
        deadline = time.monotonic() + 5
        while node.count_publishers("/fpv/odom") == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("odometry publisher not discovered")
            rclpy.spin_once(node, timeout_sec=.01)
        start = time.monotonic()

        def send_payload(payload, message_id=0x14):
            end = time.monotonic() + .3
            while time.monotonic() < end:
                connection.sendall(encode(message_id, payload))
                until = time.monotonic() + .01
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.002)

        def payload(yaw=0, flags=15, variances=(.25, .5, .75, 1., 4., 9.), arm_yaw=-9000,
                    position=(100, -200, 300)):
            return ODOMETRY.pack(
                round((time.monotonic() - start) * 1e6), flags, 7,
                *position, 100, 200, 300,
                0, 0, yaw, 10, -20, 30, *variances, arm_yaw)

        deadline = time.monotonic() + 5
        while not node.odometry:
            send_payload(payload(yaw=-6000, arm_yaw=0))
            if time.monotonic() >= deadline:
                raise RuntimeError("valid odometry did not reach the ROS subscriber")
        for compass_deg, enu_deg in ((0, 90), (90, 0), (180, -90), (270, 180)):
            wire_yaw = round(((-compass_deg + 180) % 360 - 180) * 100)
            fast = struct.pack("<I9h", 123456, 0, 0, 100, 0, 0, 1000, 0, 0, wire_yaw)
            send_payload(fast, message_id=0x10)
            assert node.imu is not None
            orientation = node.imu.orientation
            actual = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
            expected = math.radians(enu_deg)
            assert abs(math.atan2(math.sin(actual - expected), math.cos(actual - expected))) < 1e-6
            assert math.isclose(node.imu.angular_velocity.z, math.radians(10), abs_tol=1e-6)
            assert node.gimbal is not None
            camera = node.gimbal
            cq = camera.orientation
            forward = (1 - 2 * (cq.y ** 2 + cq.z ** 2),
                       2 * (cq.x * cq.y + cq.w * cq.z),
                       2 * (cq.x * cq.z - cq.w * cq.y))
            for value, wanted in zip(forward, (math.cos(expected) * math.cos(math.pi / 9),
                                               math.sin(expected) * math.cos(math.pi / 9), math.sin(math.pi / 9))):
                assert math.isclose(value, wanted, abs_tol=1e-6)
            assert camera.header.stamp == node.imu.header.stamp
            assert math.isclose(camera.angular_velocity.x, math.sin(math.pi / 9) * math.radians(10), abs_tol=1e-6)
            assert math.isclose(camera.angular_velocity.z, math.cos(math.pi / 9) * math.radians(10), abs_tol=1e-6)
            assert camera.linear_acceleration_covariance[0] == -1
        print("PASS: actual node selects strapdown for empty hardware port and publishes fixed-pitch camera FLU pose/rates", flush=True)
        for direction in (1, -1):
            previous = None
            for degrees in (0, 10, 20, 30):
                wire_yaw = direction * degrees * 100
                fast = struct.pack("<I9h", 123456, 0, 0, direction * 100,
                                   0, 0, 1000, 0, 0, wire_yaw)
                send_payload(fast, message_id=0x10)
                odom = ODOMETRY.pack(123456, 15, 7, 0, 0, 0, 0, 0, 0,
                                     0, 0, wire_yaw, 0, 0, direction * 100,
                                     .25, .5, .75, 1., 4., 9., 0)
                send_payload(odom)
                q = node.odometry[-1][1].pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
                assert math.isclose(yaw, math.radians(direction * degrees), abs_tol=1e-6)
                assert node.odometry[-1][1].twist.twist.angular.z * direction > 0
                if previous is not None:
                    assert (yaw - previous) * direction > 0
                previous = yaw
        print("PASS: ENU cardinal headings and CCW/CW yaw with matching angular_velocity.z sign", flush=True)
        for yaw, expected, covariance, off_diagonal in (
                (-9000, (1., 2., 3.), (1., 4., 9.), 0.),
                (0, (2., -1., 3.), (4., 1., 9.), 0.),
                (-4500, (3 / math.sqrt(2), 1 / math.sqrt(2), 3.), (2.5, 2.5, 9.), 1.5)):
            count = len(node.odometry)
            send_payload(payload(yaw=yaw))
            assert len(node.odometry) > count
            message = node.odometry[-1][1]
            assert message.header.frame_id == "odom" and message.child_frame_id == "base_link"
            pose, linear, angular = message.pose.pose.position, message.twist.twist.linear, message.twist.twist.angular
            assert (pose.x, pose.y, pose.z) == (1., -2., 3.)
            for actual, wanted in zip((linear.x, linear.y, linear.z), expected):
                assert math.isclose(actual, wanted, abs_tol=1e-6), (actual, wanted)
            for actual, wanted in zip((angular.x, angular.y, angular.z), (1., -2., 3.)):
                assert math.isclose(actual, math.radians(wanted), abs_tol=1e-6)
            for axis, wanted in enumerate(covariance):
                assert math.isclose(message.twist.covariance[axis * 7], wanted, abs_tol=1e-6)
            assert math.isclose(message.twist.covariance[1], off_diagonal, abs_tol=1e-6)
            assert message.pose.covariance[0] == .25
        # Every packet contains the real ARM heading: do not recenter on first reception/current yaw.
        for arm_yaw in (0, -9000, 9000, 18000, -4500, 13500):
            send_payload(payload(yaw=-6000, arm_yaw=arm_yaw))
            message = node.odometry[-1][1]
            theta = math.radians(arm_yaw * .01 + 90)
            c, s = math.cos(theta), math.sin(theta)
            position = message.pose.pose.position
            assert math.isclose(position.x, c - 2 * s, abs_tol=1e-6)
            assert math.isclose(position.y, -s - 2 * c, abs_tol=1e-6)
            assert position.z == 3
            orientation = message.pose.pose.orientation
            local_yaw = math.radians(30) - theta
            dot = abs(orientation.z * math.sin(local_yaw / 2) +
                      orientation.w * math.cos(local_yaw / 2))
            assert dot > 1 - 1e-9, (arm_yaw, orientation)
            assert math.isclose(message.pose.covariance[0], .25 * c * c + .5 * s * s, abs_tol=1e-6)
            assert math.isclose(message.pose.covariance[1], .25 * c * s, abs_tol=1e-6)
            assert math.isclose(message.pose.covariance[7], .25 * s * s + .5 * c * c, abs_tol=1e-6)
            linear = message.twist.twist.linear
            assert math.isclose(linear.x, math.cos(math.pi / 6) + 1, abs_tol=1e-6)
            assert math.isclose(linear.y, -0.5 + 2 * math.cos(math.pi / 6), abs_tol=1e-6)
            assert linear.z == 3
        for missing_flag in (1, 2, 4, 8):
            count = len(node.odometry)
            send_payload(payload(flags=15 ^ missing_flag, position=(123456, -234567, 300)))
            # Identify these packets by position: older valid DDS samples may still arrive.
            assert all(math.hypot(m.pose.pose.position.x, m.pose.pose.position.y) < 10
                       for _, m in node.odometry[count:]), "invalid GPS/ARM/origin/estimate was published"
        for invalid_variance in (float("nan"), -1.):
            count = len(node.odometry)
            send_payload(payload(variances=(invalid_variance, .5, .75, 1., 4., 9.),
                                 position=(123456, -234567, 300)))
            assert all(math.hypot(m.pose.pose.position.x, m.pose.pose.position.y) < 10
                       for _, m in node.odometry[count:]), "invalid covariance was published"
        count = len(node.odometry)
        send_payload(payload(position=(123456, -234567, 300))[:-2])
        assert all(math.hypot(m.pose.pose.position.x, m.pose.pose.position.y) < 10
                   for _, m in node.odometry[count:]), "legacy payload without ARM heading was accepted"
        count = len(node.odometry)
        send_payload(payload(arm_yaw=32767, position=(123456, -234567, 300)))
        assert all(math.hypot(m.pose.pose.position.x, m.pose.pose.position.y) < 10
                   for _, m in node.odometry[count:]), "invalid ARM heading was accepted"
        send_payload(b"\0" * 4)
        print("PASS: arbitrary ARM headings, late reception, pose/covariance rotation, invariant FLU twist and guards", flush=True)
    finally:
        shutdown(process)
        if connection is not None:
            connection.close()
        server.close()
        node.destroy_node()


def firmware_test(binary, output, nmea=False):
    for port in ((5761, 5762, 5763) if nmea else (5761, 5763)):
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", port))
    config = output / "config.txt"
    config.write_text(
        f"feature GPS\nset gps_provider = {'NMEA' if nmea else 'VIRTUAL'}\nset trust_mag = ON\n"
        "set mag_declination = 20\n"
        "set small_angle = 180\nset runaway_takeoff_prevention = OFF\n"
        "set auto_disarm_delay = 0\n"
        "set custom_link_motors_stream = OFF\nmap AETR1234\n"
        "aux 0 0 2 1700 2100 0 0\naux 1 58 1 1700 2100 0 0\n" +
        ("serial 1 2 115200 115200 0 115200\nset gps_auto_config = OFF\n" if nmea else ""))
    subprocess.run([str(binary), "--config", str(config)], cwd=output,
                   stdout=(output / "provision.log").open("w"), stderr=subprocess.STDOUT,
                   check=True, timeout=30)
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        relay_port = reserve.getsockname()[1]
    firmware = bridge = None
    sock = gps_sock = None
    node = Observer()
    simulation = None
    try:
        firmware = subprocess.Popen([str(binary)], cwd=output,
                                    stdout=(output / "firmware.log").open("w"), stderr=subprocess.STDOUT)
        bridge = subprocess.Popen([
            "ros2", "launch", "apm_bridge", "sitl.launch.py",
            f"airsim_link_port:={relay_port}", "gimbal_enabled:=false",
        ], stdout=(output / "bridge.log").open("w"), stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 15
        while sock is None:
            try:
                sock = socket.create_connection(("127.0.0.1", relay_port), .1)
            except ConnectionRefusedError:
                if time.monotonic() >= deadline:
                    raise RuntimeError("SITL relay did not start")
                time.sleep(.1)
        sock.settimeout(.001)
        if nmea:
            gps_sock = socket.create_connection(("127.0.0.1", 5762), 1)
        simulation = Simulation(sock, node, gps_sock)
        simulation.wait(lambda: node.state is not None and node.state.connected and
                        node.gps is not None and node.gps.fix_type == GPSRAW.GPS_FIX_TYPE_3D_FIX)
        simulation.pump(3)
        assert node.state is not None and not node.state.armed and not simulation.frames and not node.odometry
        if nmea:
            simulation.nmea_fix = 2
            simulation.wait(lambda: node.gps is not None and node.gps.fix_type == GPSRAW.GPS_FIX_TYPE_NO_FIX)
            simulation.arm()
            simulation.pump(1)
            assert not simulation.frames and not node.odometry, "NMEA 2D fix was accepted as 3D"
            simulation.nmea_fix = 3
            simulation.wait(lambda: node.gps is not None and node.gps.fix_type == GPSRAW.GPS_FIX_TYPE_3D_FIX)
            simulation.pump(1)
            assert not simulation.frames, "late NMEA 3D fix fabricated an ARM origin"
            simulation.disarm()
            simulation.arm()
            simulation.wait(lambda: len(simulation.frames) > 20 and len(node.odometry) > 20)
            simulation.disarm()
            print("PASS: real NMEA GGA/GSA parser rejects 2D and permits odometry after re-ARM with 3D", flush=True)
            return {"nmea_3d_gate": True}
        simulation.arm()
        simulation.wait(lambda: len(simulation.frames) > 20 and len(node.odometry) > 20)
        initial = simulation.frames[-1][1]
        assert initial[1] == 15 and all(abs(v) < 30 for v in initial[3:6]), initial
        start = time.monotonic()
        simulation.pump(3)
        frames = [f for t, f in simulation.frames if t >= start]
        messages = [m for t, m in node.odometry if t >= start]
        rate = len(frames) / 3
        ros_rate = len(messages) / 3
        assert 95 <= rate <= 105 and 95 <= ros_rate <= 105, (rate, ros_rate)
        intervals = [(frames[i][0] - frames[i - 1][0]) & 0xffffffff for i in range(1, len(frames))]
        assert abs(sum(intervals) / len(intervals) - 10000) < 1000, intervals[:10]
        print(f"PASS: custom 0x14={rate:.2f} Hz, /fpv/odom={ros_rate:.2f} Hz", flush=True)
        simulation.lon += 1800
        simulation.lat += 900
        simulation.alt += 400
        simulation.pressure = round(101325 * (1 - 2.25577e-5 * 4) ** 5.25588)
        simulation.pump(10)
        displaced = simulation.frames[-1][1]
        print("origin IDs:", sorted({f[2] for _, f in simulation.frames}),
              "arm transitions:", node.state_transitions, flush=True)
        assert displaced[2] == initial[2], "unexpected disarm/re-arm during the test"
        assert displaced[-1] == initial[-1], "ARM heading changed during the flight"
        assert displaced[3] > 500 and displaced[4] > 400 and displaced[5] > 100, displaced
        pose = node.odometry[-1][1]
        assert pose.header.frame_id == "odom" and pose.child_frame_id == "base_link"
        theta = math.radians(displaced[-1] * .01 + 90)
        c, s = math.cos(theta), math.sin(theta)
        assert abs(pose.pose.pose.position.x - (c * displaced[3] + s * displaced[4]) * .01) < .1
        assert abs(pose.pose.pose.position.y - (-s * displaced[3] + c * displaced[4]) * .01) < .1
        assert abs(pose.pose.pose.position.z - displaced[5] * .01) < .1
        print("PASS: positive ENU displacement (m):",
              tuple(round(v * .01, 3) for v in displaced[3:6]), flush=True)
        simulation.gps_enabled = False
        simulation.wait(lambda: node.gps is not None and node.gps.fix_type == GPSRAW.GPS_FIX_TYPE_NO_FIX, timeout=12)
        simulation.pump(3)
        assert node.gps is not None and node.gps.fix_type == GPSRAW.GPS_FIX_TYPE_NO_FIX
        paused, paused_ros = len(simulation.frames), len(node.odometry)
        simulation.pump(.7)
        assert len(simulation.frames) == paused and len(node.odometry) == paused_ros
        simulation.gps_enabled = True
        simulation.wait(lambda: len(simulation.frames) > paused + 20)
        recovered = simulation.frames[-1][1]
        assert recovered[2] == initial[2] and recovered[3] > 500 and recovered[4] > 400, recovered
        assert recovered[-1] == initial[-1]
        print("PASS: GPS loss suppresses odometry; recovery retains ARM position and heading", flush=True)
        simulation.disarm()
        paused = len(simulation.frames)
        simulation.pump(.5)
        assert len(simulation.frames) == paused
        simulation.heading_rad = math.radians(60)
        simulation.pump(2)
        simulation.arm()
        simulation.wait(lambda: len(simulation.frames) > paused + 20)
        rearmed = simulation.frames[-1][1]
        assert rearmed[2] == initial[2] + 1 and all(abs(v) < 30 for v in rearmed[3:6]), rearmed
        heading_change = (rearmed[-1] - initial[-1] + 18000) % 36000 - 18000
        assert abs(heading_change) > 3000, (initial[-1], rearmed[-1])
        assert simulation.fast_attitude is not None
        yaw_difference = (rearmed[11] - simulation.fast_attitude[2] + 18000) % 36000 - 18000
        assert abs(yaw_difference) < 100, "0x14 yaw disagrees with 0x10 mathematical yaw"
        assert abs((rearmed[-1] - rearmed[11] + 18000) % 36000 - 18000) < 200, "ARM yaw has the wrong sign"
        print("PASS: re-arm resets position and heading:", rearmed[2], rearmed[-1] * .01, flush=True)
        simulation.disarm()
        simulation.gps_enabled = False
        simulation.wait(lambda: node.gps is not None and node.gps.fix_type == GPSRAW.GPS_FIX_TYPE_NO_FIX, timeout=12)
        simulation.pump(3)
        simulation.arm()
        paused = len(simulation.frames)
        simulation.gps_enabled = True
        simulation.pump(3)
        assert node.gps is not None and node.gps.fix_type == GPSRAW.GPS_FIX_TYPE_3D_FIX and len(simulation.frames) == paused
        print("PASS: GPS acquired after ARM does not fabricate an ARM origin", flush=True)
        simulation.disarm()
        return {"wire_rate_hz": rate, "ros_rate_hz": ros_rate,
                "displacement_cm": displaced[3:6], "passed": True}
    finally:
        if simulation is not None and node.state is not None and node.state.armed:
            try:
                simulation.disarm()
            finally:
                shutdown(bridge)
                shutdown(firmware)
        else:
            shutdown(bridge)
            shutdown(firmware)
        if sock is not None:
            sock.close()
        if gps_sock is not None:
            gps_sock.close()
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="new artifact directory")
    args = parser.parse_args()
    output = args.output or Path(tempfile.mkdtemp(prefix="bf_odom_"))
    if args.output:
        output.mkdir(parents=True, exist_ok=False)
    print(f"artifacts: {output}", flush=True)
    os.environ["ROS_DOMAIN_ID"] = "198"
    rclpy.init()
    try:
        decoder_test(output)
        report = firmware_test(args.binary.resolve(), output)
        nmea_output = output / "nmea"
        nmea_output.mkdir()
        report.update(firmware_test(args.binary.resolve(), nmea_output, nmea=True))
        (output / "result.json").write_text(json.dumps(report, indent=2))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
