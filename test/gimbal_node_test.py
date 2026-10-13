"""Exercise the actual bridge with isolated ROS, fake FC TCP and a gimbal PTY."""

import os
from pathlib import Path
import pty
import socket
import struct
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Header, String

from gimbal_pysim_test import build_reply, parse_ctrl
from sitl_odometry_test import encode, shutdown


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
                    connection.sendall(encode(0x11, struct.pack("<IIih16H", timestamp, 101325, 200, 2000, *([1500] * 16))))
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
            pump(.15)
            assert modes[-1] == "AIM"
            assert controls[-1].wk_mode == [2, 2, 2] and controls[-1].op_value == [0, 3500, 0]
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
            pump(.1)
            assert controls[-1].op_type[1] == 0 and controls[-1].op_value[1] == 1200
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
