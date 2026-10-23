"""Native ANGLE raw command transport through the real bridge, with a TCP fake FC only."""
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from quadrotor_msgs.msg import ControlCommand
from std_msgs.msg import Bool

from cli_service_test import FakeFc
from sitl_odometry_test import encode, shutdown


def test_native_angle_transport_and_rate_compatibility(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 40))
    binary = Path(__file__).resolve().parents[3] / "build/apm_bridge/custom_link_bridge_node"
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    server.settimeout(5)
    context = Context()
    rclpy.init(context=context)
    node = Node("bench_angle_transport_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    states = []
    node.create_subscription(State, "/mavros/state", states.append, 10)
    raw = node.create_publisher(ControlCommand, "/custom_link_bridge/control_command_raw", qos_profile_sensor_data)
    offboard = node.create_publisher(Bool, "/custom_link_bridge/offboard", 10)
    process = connection = None
    try:
        with (tmp_path / "bridge.log").open("w") as log:
            process = subprocess.Popen([str(binary), "--ros-args", "-p", "link.transport:=tcp",
                "-p", f"link.tcp_port:={server.getsockname()[1]}", "-p", "gimbal_port:=''"],
                stdout=log, stderr=subprocess.STDOUT)
            connection = server.accept()[0]
            connection.setblocking(False)
            fc = FakeFc(connection)
            def pump(duration):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    assert process.poll() is None
                    slow = bytearray(36)
                    struct.pack_into("<I", slow, 0, round(time.monotonic()*1e6) & 0xffffffff)
                    struct.pack_into("<IH", slow, 30, (1 << 13) | 1, 1)
                    connection.sendall(encode(0x12, slow))
                    fc.pump()
                    executor.spin_once(timeout_sec=.005)
            deadline = time.monotonic() + 5
            while raw.get_subscription_count() == 0 or not states:
                assert time.monotonic() < deadline
                pump(.05)
            assert states[-1].mode == "OFFBOARD|ANGLE"
            command = ControlCommand(armed=True, control_mode=ControlCommand.ATTITUDE, collective_thrust=.12)
            roll, pitch = math.radians(2), math.radians(-3)
            cr, sr, cp, sp = math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2), math.sin(pitch/2)
            command.orientation.x, command.orientation.y = sr*cp, cr*sp
            command.orientation.z, command.orientation.w = -sr*sp, cr*cp
            command.bodyrates.z = math.radians(4)
            for _ in range(5):
                raw.publish(command)
                pump(.05)
            angle = [f for f in fc.controls if f[3] == 3]
            assert angle and all(f[2] == 0 and f[4] == 1120 for f in angle)
            assert all(abs(f[5]-20) <= 1 and abs(f[6]+30) <= 1 and abs(f[7]-40) <= 1 for f in angle)
            raw.publish(ControlCommand(armed=True, control_mode=ControlCommand.BODY_RATES,
                                       collective_thrust=.12))
            pump(.1)
            assert fc.controls[-1][3] == 2
            offboard.publish(Bool(data=False))
            pump(.1)
            assert fc.controls[-1][3] == 0 and all(f[2] == 0 for f in fc.controls)
    finally:
        if connection is not None:
            connection.close()
        server.close()
        if process is not None:
            shutdown(process)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
