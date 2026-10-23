"""Exercise the actual bridge with isolated ROS, fake FC TCP and a gimbal PTY."""

import os
from pathlib import Path
import pty
import socket
import struct
import subprocess
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import RCIn
from std_msgs.msg import Bool, Header, Int8, String

from gimbal_pysim_test import build_reply, parse_ctrl
from sitl_odometry_test import encode, shutdown


@pytest.mark.parametrize("airsim_preview", [False, True])
def test_rc_pulse_starts_and_stops_mission_only(tmp_path, monkeypatch, airsim_preview):
    """AUX4 pulse semantics: a >=0.5 s hold publishes MISSION when vision is
    locked and no mission runs, DISMISS while a mission runs (interception is
    cancelled, the tracker keeps running), and nothing without a lock - the
    tracker start/stop stays with the arm edges. MISSION/DISMISS also drive
    the OFFBOARD/MANUAL custom-link request (log-verified linkage). AirSim adds
    disarmed preview: ON only, never MISSION, even with a lock."""
    monkeypatch.setenv("ROS_DOMAIN_ID", str(160 + os.getpid() % 60))
    binary = Path(os.environ.get("APM_BRIDGE_TEST_BINARY",
                  Path(__file__).resolve().parents[3] / "build/apm_bridge/custom_link_bridge_node"))
    assert binary.is_file(), f"Build the bridge first: {binary}"
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    server.settimeout(5)
    context = Context()
    rclpy.init(context=context)
    node = Node("rc_pulse_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    triggers = []
    node.create_subscription(Header, "/fpv/tracker_trigger",
                             lambda m: triggers.append(m.frame_id), 10)
    rc_in = []
    node.create_subscription(RCIn, "/mavros/rc/in", lambda m: rc_in.append(m.rssi),
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    tracker_pub = node.create_publisher(Int8, "/fpv/tracker_state", 1)
    mission_pub = node.create_publisher(Bool, "/fpv/mission_state", 1)
    process = connection = physics = None
    locked = mission = False
    rc_trigger = 1000  # AUX4 idle level (the latch resets only below 1100)
    armed = not airsim_preview
    log_path = tmp_path / "bridge.log"
    try:
        with log_path.open("w") as log:
            extra = []
            if airsim_preview:
                with socket.socket() as port:
                    port.bind(("127.0.0.1", 0))
                    relay_port = port.getsockname()[1]
                extra = ["-p", "airsim.enabled:=true", "-p", "airsim.gimbal_enabled:=false",
                         "-p", "baro_compensation:=false",
                         "-p", f"airsim.link_port:={relay_port}"]
            process = subprocess.Popen([
                str(binary), "--ros-args", "-p", "link.transport:=tcp",
                "-p", f"link.tcp_port:={server.getsockname()[1]}",
                "-p", "channel_trigger:=7",
                *extra,
            ], stdout=log, stderr=subprocess.STDOUT)
            connection = server.accept()[0]
            connection.setblocking(False)
            if airsim_preview:
                physics = socket.create_connection(("127.0.0.1", relay_port), timeout=5)
                physics.setblocking(False)

            def pump(duration):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    timestamp = round(time.monotonic() * 1e6) & 0xffffffff
                    channels = [1000] * 16
                    channels[7] = rc_trigger
                    connection.sendall(encode(0x11, struct.pack(
                        "<IIih16HB", timestamp, 101325, 200, 2000, *channels, 87)))
                    connection.sendall(encode(0x12, struct.pack(
                        "<IHihBBiiiHHIH", timestamp, 24000, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, int(armed))))
                    tracker_pub.publish(Int8(data=2 if locked else 0))
                    mission_pub.publish(Bool(data=mission))
                    try:
                        connection.recv(4096)
                    except BlockingIOError:
                        pass
                    if physics is not None:
                        try:
                            physics.recv(4096)
                        except BlockingIOError:
                            pass
                    executor.spin_once(timeout_sec=.005)
                    time.sleep(.005)

            def wait_for(predicate):
                deadline = time.monotonic() + 5
                while not predicate():
                    assert time.monotonic() < deadline, \
                        f"triggers={triggers}\n{log_path.read_text()}"
                    pump(.03)

            wait_for(lambda: tracker_pub.get_subscription_count() > 0
                     and mission_pub.get_subscription_count() > 0)
            pump(.2)
            triggers.clear()  # Ignore the armed-edge ON in the non-preview case.
            if airsim_preview:
                # A disarmed RC pulse starts vision only, even with a lock.
                rc_trigger = 2000
                wait_for(lambda: triggers == ["ON"])
                locked = True
                rc_trigger = 1000
                pump(.2)
                rc_trigger = 2000
                wait_for(lambda: triggers == ["ON", "ON"])
                assert "requesting custom-link OFFBOARD" not in log_path.read_text()
                armed = True
                rc_trigger = 1000
                wait_for(lambda: triggers == ["ON", "ON", "ON"])  # Real arm edge.
                locked = False
                pump(.2)
                triggers.clear()
            # No lock: the pulse is ignored (tracker on/off is the arm edge's job).
            rc_trigger = 2000
            pump(.9)
            assert triggers == []
            assert "Trigger pulse ignored" in log_path.read_text()
            # Locked and idle: the pulse starts a MISSION and requests OFFBOARD.
            locked = True
            rc_trigger = 1000
            pump(.3)  # release below 1100 us resets the one-shot latch
            rc_trigger = 2000
            wait_for(lambda: triggers == ["MISSION"])
            assert "requesting custom-link OFFBOARD" in log_path.read_text()
            # The medium frames carry rssi 87; it must flow through to RCIn.
            assert rc_in and rc_in[-1] == 87, f"rc_in rssi={rc_in[-3:]}"

            # Mission running: the pulse dismisses it and requests MANUAL
            # (OFF and DISMISS share the bridge's release handling).
            mission = True
            rc_trigger = 1000
            pump(.3)
            rc_trigger = 2000
            wait_for(lambda: triggers == ["MISSION", "DISMISS"])
            assert "Tracker OFF: requesting custom-link MANUAL" in log_path.read_text()
    finally:
        shutdown(process)
        if connection is not None:
            connection.close()
        if physics is not None:
            physics.close()
        server.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_armed_edge_publishes_tracker_trigger(tmp_path, monkeypatch):
    """0x12 STATUS_ARMED rising edge -> /fpv/tracker_trigger ON (tracker start),
    falling edge -> OFF (tracker + interceptor stop); exactly one frame per
    edge, none while the state is held."""
    monkeypatch.setenv("ROS_DOMAIN_ID", str(160 + os.getpid() % 60))
    binary = Path(os.environ.get("APM_BRIDGE_TEST_BINARY",
                  Path(__file__).resolve().parents[3] / "build/apm_bridge/custom_link_bridge_node"))
    assert binary.is_file(), f"Build the bridge first: {binary}"
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    server.settimeout(5)
    context = Context()
    rclpy.init(context=context)
    node = Node("armed_edge_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    triggers = []
    node.create_subscription(Header, "/fpv/tracker_trigger",
                             lambda m: triggers.append(m.frame_id), 10)
    process = connection = None
    armed = False
    log_path = tmp_path / "bridge.log"
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen([
                str(binary), "--ros-args", "-p", "link.transport:=tcp",
                "-p", f"link.tcp_port:={server.getsockname()[1]}",
            ], stdout=log, stderr=subprocess.STDOUT)
            connection = server.accept()[0]
            connection.setblocking(False)

            def pump(duration):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    timestamp = round(time.monotonic() * 1e6) & 0xffffffff
                    connection.sendall(encode(0x12, struct.pack(
                        "<IHihBBiiiHHIH", timestamp, 24000, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, int(armed))))
                    try:
                        connection.recv(4096)
                    except BlockingIOError:
                        pass
                    executor.spin_once(timeout_sec=.005)
                    time.sleep(.005)

            def wait_for(predicate):
                deadline = time.monotonic() + 5
                while not predicate():
                    assert time.monotonic() < deadline, \
                        f"triggers={triggers}\n{log_path.read_text()}"
                    pump(.03)

            pump(.2)  # disarmed 0x12 stream must not trigger anything
            assert triggers == []
            armed = True
            wait_for(lambda: triggers == ["ON"])
            pump(.2)  # armed held: edge-triggered, still exactly one ON
            assert triggers == ["ON"]
            armed = False
            wait_for(lambda: triggers == ["ON", "OFF"])
            pump(.2)  # disarmed held: no further frames
            assert triggers == ["ON", "OFF"]
    finally:
        shutdown(process)
        if connection is not None:
            connection.close()
        server.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_actual_node_modes_and_los_subscription(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(160 + os.getpid() % 60))
    binary = Path(os.environ.get("APM_BRIDGE_TEST_BINARY",
                  Path(__file__).resolve().parents[3] / "build/apm_bridge/custom_link_bridge_node"))
    assert binary.is_file(), f"Build the bridge first: {binary}"
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    server.settimeout(5)
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    context = Context()
    rclpy.init(context=context)
    node = Node("gimbal_node_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    modes = []
    node.create_subscription(String, "/fpv/gimbal_mode", lambda msg: modes.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    mission_pub = node.create_publisher(Bool, "/fpv/mission_state", 1)
    trigger_pub = node.create_publisher(Header, "/fpv/tracker_trigger", 10)
    los_pub = node.create_publisher(TwistStamped, "/fpv/target_los", 10)
    process = connection = None
    armed = offboard = mission = False
    buffer = bytearray()
    controls = []
    start = time.monotonic()
    last_sent_los = 0
    log_path = tmp_path / "bridge.log"
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen([
                str(binary), "--ros-args", "-p", "link.transport:=tcp",
                "-p", f"link.tcp_port:={server.getsockname()[1]}",
                "-p", f"gimbal_port:={os.ttyname(slave)}",
                "-p", "gimbal.model:=C20S", "-p", "gimbal.pitch_hold:=12.0",
                "-p", "gimbal.pitch_aim:=-55.0",
                "-p", "baro_compensation:=false",
            ], stdout=log, stderr=subprocess.STDOUT)
            connection = server.accept()[0]
            connection.setblocking(False)

            def pump(duration, los=None, frame="camera_flu", age=0.0, fixed_stamp=None):
                nonlocal last_sent_los
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    timestamp = round((time.monotonic() - start) * 1e6) & 0xffffffff
                    connection.sendall(encode(0x10, struct.pack("<I9h", timestamp, 0, 0, 0, 0, 0, 1000, 0, 0, 0)))
                    connection.sendall(encode(0x11, struct.pack("<IIih16HB", timestamp, 101325, 200, 2000, *([1500] * 16), 87)))
                    connection.sendall(encode(0x12, struct.pack("<IHihBBiiiHHIH", timestamp, 24000, 0, 0, 0, 0,
                                                              0, 0, 0, 0, 0, (1 << 13) if offboard else 0, int(armed))))
                    mission_pub.publish(Bool(data=mission))
                    if los is not None:
                        message = TwistStamped()
                        last_sent_los = (node.get_clock().now().nanoseconds - round(age * 1e9)
                                         if fixed_stamp is None else fixed_stamp)
                        message.header.stamp = Time(nanoseconds=last_sent_los).to_msg()
                        message.header.frame_id = frame
                        message.twist.linear.y, message.twist.linear.z = los
                        los_pub.publish(message)
                    try:
                        connection.recv(4096)
                    except BlockingIOError:
                        pass
                    try:
                        buffer.extend(os.read(master, 4096))
                    except BlockingIOError:
                        pass
                    while len(buffer) >= 40:
                        if buffer[:2] != b"\xa9\x5b":
                            del buffer[0]
                            continue
                        packet = parse_ctrl(bytes(buffer[:40]), time.monotonic() - start)
                        del buffer[:40]
                        controls.append(packet)
                        os.write(master, build_reply(cam0=0))
                    executor.spin_once(timeout_sec=.005)
                    time.sleep(.005)

            def wait_for(predicate):
                deadline = time.monotonic() + 3
                while not predicate():
                    assert time.monotonic() < deadline, f"Timed out; modes={modes}\n{log_path.read_text()}"
                    pump(.03)

            wait_for(lambda: modes and modes[-1] == "AIM" and mission_pub.get_subscription_count() > 0)
            armed = True
            pump(2.4)
            assert modes[-1] == "AIM"
            assert controls[-1].wk_mode == [2, 2, 2]
            assert controls[-1].op_value[0] == controls[-1].op_value[2] == 0
            assert abs(controls[-1].op_value[1] - 3500) <= 1
            assert "AIM requested" not in log_path.read_text()
            mission = offboard = True
            wait_for(lambda: modes[-1] == "MISSION")
            pump(.2, (.4, .2))
            assert controls[-1].wk_mode == [1, 1, 1]
            assert controls[-1].op_type == [2, 2, 2]
            assert -300 <= controls[-1].op_value[1] < 0
            assert controls[-1].op_value[0] == controls[-1].op_value[2] == 0
            pump(1.2, (0.0, 0.0))
            assert abs(controls[-1].op_value[1]) <= 1, "HORIZON pitch_hold leaked into centered MISSION"
            pump(1.2, (.4, .2), age=2)
            assert abs(controls[-1].op_value[1]) <= 1
            assert "Rejecting target LOS" in log_path.read_text()
            pump(.2, (.4, .2))
            assert controls[-1].op_value[1] < 0
            repeated_stamp = last_sent_los
            pump(1.2, (.4, .2), fixed_stamp=repeated_stamp)
            assert abs(controls[-1].op_value[1]) <= 1, "repeated timestamps kept motion alive"
            pump(.2, (.4, .2))
            assert controls[-1].op_value[1] < 0
            pump(1.2, (.4, .2), frame="wrong_frame")
            assert abs(controls[-1].op_value[1]) <= 1
            offboard = False
            wait_for(lambda: modes[-1] == "HORIZON")
            pump(2.0)
            assert controls[-1].op_type[1] == 0 and abs(controls[-1].op_value[1] - 1200) <= 1
            offboard = True
            pump(.15)
            assert modes[-1] == "HORIZON", "stale mission resumed after manual takeover"
            mission = False
            pump(.15)
            mission = True
            wait_for(lambda: modes[-1] == "MISSION")
            trigger_pub.publish(Header(frame_id="OFF"))
            wait_for(lambda: modes[-1] == "HORIZON")
            mission = offboard = False
            pump(.1)
            trigger_pub.publish(Header(frame_id="ON"))
            wait_for(lambda: modes[-1] == "AIM")
            assert modes.count("MISSION") == 2
    finally:
        shutdown(process)
        if connection is not None:
            connection.close()
        server.close()
        os.close(master)
        os.close(slave)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
