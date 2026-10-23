"""Drive the real bridge binary against a fake FC over TCP and exercise the
/fpv/cli service end-to-end (0x24 HOST_CLI up, 0x15 FC_CLI chunked down).

Checks:
  * GET / SET map onto the request text and echo the readback
  * FC-side errors arrive as ok=true + status=ERR (NOT FOUND / SET FAILED)
  * INFO responses reassemble across multiple 56-byte chunks
  * a chunk gap and a length mismatch are detected locally
  * an unresponsive FC yields ok=false error=timeout within the configured
    window, and while that call is in flight the executor keeps serving other
    callbacks (probed by a bad-request call answered on the executor thread)
    and telemetry keeps flowing (reader thread)
  * malformed requests (cmd out of range) are rejected without touching the
    link
"""

import os
from pathlib import Path
import socket
import struct
import subprocess
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import RCIn
from std_msgs.msg import Bool
from quadrotor_msgs.srv import CliCommand

from sitl_odometry_test import encode, shutdown

SYNC = b"\xeb\x90"
CL_CLI_CMD_GET, CL_CLI_CMD_SET, CL_CLI_CMD_INFO, CL_CLI_CMD_BIND_RX, CL_CLI_CMD_SAVE = 0, 1, 2, 3, 4
CL_CLI_CMD_APPLY_PID = 5
CL_CLI_STATUS_OK, CL_CLI_STATUS_ERR = 0, 1
CL_CLI_FLAG_MORE = 0x01


class FakeFc:
    """Minimal FC speaking the custom link: parses uplink, answers 0x24."""

    def __init__(self, connection):
        self.connection = connection
        self.buffer = b""
        self.rx_cli = []          # (req_id, cmd, args) as received
        self.bind_requests = []   # (req_id, args) for cmd 3
        self.save_requests = []   # (req_id, args) for cmd 4
        self.mode_reqs = []       # mode_req of every 0x20 seen
        self.controls = []
        self.seq = 0

    def pump(self):
        try:
            data = self.connection.recv(4096)
        except BlockingIOError:
            data = b""
        if data:
            self.feed(data)

    def feed(self, data):
        self.buffer += data
        while True:
            i = self.buffer.find(SYNC)
            if i < 0:
                self.buffer = self.buffer[-1:]
                return
            if i > 0:
                self.buffer = self.buffer[i:]
            if len(self.buffer) < 5:
                return
            msg_id, length, _seq = self.buffer[2], self.buffer[3], self.buffer[4]
            if length == 0 or length > 64:
                self.buffer = self.buffer[2:]
                continue
            if len(self.buffer) < 7 + length:
                return
            payload = self.buffer[5:5 + length]
            self.buffer = self.buffer[7 + length:]
            if msg_id == 0x24:
                self.handle_cli(payload)
            elif msg_id == 0x20 and length == 20:
                self.mode_reqs.append(payload[11])  # arm u8@10, mode_req u8@11
                self.controls.append(struct.unpack("<QHBBH3h", payload))

    def handle_cli(self, payload):
        req_id = struct.unpack_from("<H", payload, 0)[0]
        cmd = payload[2]
        args = payload[4:].split(b"\x00", 1)[0].decode(errors="replace")
        self.rx_cli.append((req_id, cmd, args))

        if "timeout" in args:
            return                       # black hole: client must time out
        if "gap" in args:
            # first chunk fine, second jumps ahead -> client detects the gap
            self.send_chunk(req_id, CL_CLI_STATUS_OK, 0, b"x" * 56, True, 200)
            self.send_chunk(req_id, CL_CLI_STATUS_OK, 112, b"x" * 56, False, 200)
            return
        if "truncated" in args:
            text = b"y" * 130
            for off in (0, 56):
                self.send_chunk(req_id, CL_CLI_STATUS_OK, off, text[off:off + 56], True, 200)
            # final chunk clears MORE while total_len claims more bytes
            self.send_chunk(req_id, CL_CLI_STATUS_OK, 112, text[112:], False, 200)
            return

        if cmd == CL_CLI_CMD_GET:
            if args.startswith("zzz"):
                self.respond(req_id, CL_CLI_STATUS_ERR, "NOT FOUND")
            else:
                self.respond(req_id, CL_CLI_STATUS_OK, f"{args} = 15")
        elif cmd == CL_CLI_CMD_SET:
            if "=" not in args:
                self.respond(req_id, CL_CLI_STATUS_ERR, "SET FAILED")
            else:
                name, _, value = args.partition("=")
                self.respond(req_id, CL_CLI_STATUS_OK,
                             f"{name.strip()} = {value.strip()}")
        elif cmd == CL_CLI_CMD_INFO:
            if args.startswith("zzz"):
                self.respond(req_id, CL_CLI_STATUS_ERR, "NOT FOUND")
            else:
                # long enough to span several 56-byte chunks
                values = ",".join(f"VALUE{i:02d}" for i in range(18))
                self.respond(req_id, CL_CLI_STATUS_OK,
                             f"pgn=41\ntype=lookup\nvalues={values}\ndefault=VALUE00\n")
        elif cmd == CL_CLI_CMD_BIND_RX:
            self.bind_requests.append((req_id, args))
            self.respond(req_id, CL_CLI_STATUS_OK, "binding...")
        elif cmd == CL_CLI_CMD_SAVE:
            self.save_requests.append((req_id, args))
            self.respond(req_id, CL_CLI_STATUS_OK, "saving...")
        elif cmd == CL_CLI_CMD_APPLY_PID:
            self.respond(req_id, CL_CLI_STATUS_OK, "PID PROFILE 0")

    def respond(self, req_id, status, text):
        data = text.encode()
        for off in range(0, len(data), 56):
            self.send_chunk(req_id, status, off, data[off:off + 56],
                            off + 56 < len(data), len(data))

    def send_chunk(self, req_id, status, offset, chunk, more, total):
        flags = CL_CLI_FLAG_MORE if more else 0
        payload = struct.pack("<HHHBB", req_id, total, offset, status, flags) + chunk
        self.seq = (self.seq + 1) & 0xFF
        self.connection.sendall(encode(0x15, payload))


def test_cli_service_end_to_end(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(120 + os.getpid() % 60))
    binary = Path(os.environ.get("APM_BRIDGE_TEST_BINARY",
                  Path(__file__).resolve().parents[3] / "build/apm_bridge/custom_link_bridge_node"))
    assert binary.is_file(), f"Build the bridge first: {binary}"

    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    server.settimeout(5)

    context = Context()
    rclpy.init(context=context)
    node = Node("cli_service_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    rc_count = [0]
    node.create_subscription(RCIn, "/mavros/rc/in",
                             lambda m: rc_count.__setitem__(0, rc_count[0] + 1),
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    cli = node.create_client(CliCommand, "/fpv/cli")

    process = connection = fc = None
    log_path = tmp_path / "bridge.log"

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            fc.pump()
            timestamp = round(time.monotonic() * 1e6) & 0xFFFFFFFF
            channels = [1500] * 16
            connection.sendall(encode(0x11, struct.pack(
                "<IIih16HB", timestamp, 101325, 200, 2000, *channels, 87)))
            executor.spin_once(timeout_sec=0.005)
            time.sleep(0.005)

    def call(cmd, args, wall_timeout=12.0):
        request = CliCommand.Request()
        request.cmd = cmd
        request.args = args
        future = cli.call_async(request)
        t0 = time.monotonic()
        while not future.done():
            pump(0.02)
            assert time.monotonic() - t0 < wall_timeout, \
                f"service call did not return\n{log_path.read_text()}"
        return future.result(), time.monotonic() - t0

    try:
        with log_path.open("w") as log:
            process = subprocess.Popen([
                str(binary), "--ros-args", "-p", "link.transport:=tcp",
                "-p", f"link.tcp_port:={server.getsockname()[1]}",
                "-p", "cli.timeout_s:=1.0",
            ], stdout=log, stderr=subprocess.STDOUT)
            connection = server.accept()[0]
            connection.setblocking(False)
            fc = FakeFc(connection)
            deadline = time.monotonic() + 15
            while not cli.service_is_ready():
                assert time.monotonic() < deadline, \
                    f"bridge did not offer /fpv/cli\n{log_path.read_text()}"
                pump(0.05)
            pump(0.3)

            # GET: single-chunk success
            response, _ = call(CL_CLI_CMD_GET, "failsafe_delay")
            assert response.ok and response.status == CL_CLI_STATUS_OK
            assert response.text == "failsafe_delay = 15", response.text
            assert response.error == ""

            # SET: readback contract
            response, _ = call(CL_CLI_CMD_SET, "failsafe_delay = 20")
            assert response.ok and response.status == CL_CLI_STATUS_OK
            assert response.text == "failsafe_delay = 20"

            # SET without '=' rejected by the FC-side semantics
            response, _ = call(CL_CLI_CMD_SET, "failsafe_delay")
            assert response.ok and response.status == CL_CLI_STATUS_ERR
            assert response.text == "SET FAILED"

            # GET unknown setting
            response, _ = call(CL_CLI_CMD_GET, "zzz_no_such")
            assert response.ok and response.status == CL_CLI_STATUS_ERR
            assert response.text == "NOT FOUND"

            # INFO: multi-chunk reassembly
            response, _ = call(CL_CLI_CMD_INFO, "motor_pwm_protocol")
            assert response.ok and response.status == CL_CLI_STATUS_OK
            assert response.text.startswith("pgn=41\n")
            assert "type=lookup" in response.text
            assert response.text.count("VALUE") == 19  # 18 values + default
            assert len(response.text) > 2 * 56          # genuinely chunked

            # chunk gap detected locally
            response, _ = call(CL_CLI_CMD_GET, "gap_case")
            assert not response.ok and response.error == "chunk gap"

            # length mismatch (MORE clear before total_len) detected locally
            response, _ = call(CL_CLI_CMD_GET, "truncated_case")
            assert not response.ok and response.error == "length mismatch"

            # timeout: link up but FC stays silent. The deferred-response design
            # must keep the EXECUTOR serving while the worker blocks: probe
            # with a bad-request call whose reply is sent synchronously from
            # the executor thread (cliServiceCallback) - it must complete
            # promptly WHILE the timeout call is still pending. (Telemetry
            # mirroring alone would prove nothing: /mavros/rc/in is published
            # from the client's reader thread, not the executor.)
            rc_before = rc_count[0]
            timeout_req = CliCommand.Request()
            timeout_req.cmd = CL_CLI_CMD_GET
            timeout_req.args = "timeout_case"
            timeout_future = cli.call_async(timeout_req)
            probe_req = CliCommand.Request()
            probe_req.cmd = 9
            probe_req.args = "probe"
            probe_future = cli.call_async(probe_req)
            t0 = time.monotonic()
            while not probe_future.done():
                pump(0.02)
                assert time.monotonic() - t0 < 2.0, \
                    f"executor stalled during a blocked CLI call\n{log_path.read_text()}"
            probe_at = time.monotonic() - t0
            while not timeout_future.done():
                pump(0.02)
                assert time.monotonic() - t0 < 12.0, log_path.read_text()
            response = timeout_future.result()
            elapsed = time.monotonic() - t0
            assert probe_at < 0.8, f"probe answered late: {probe_at:.2f}s"
            assert probe_future.result().error.startswith("bad request")
            assert not response.ok and response.error == "timeout"
            assert 0.9 <= elapsed <= 4.0, elapsed
            # telemetry keeps flowing as well (reader thread -> publishers)
            assert rc_count[0] - rc_before > 10, \
                f"telemetry stalled during CLI call: {rc_count[0] - rc_before} rc frames"

            # bind_rx through the service: action command, no settings engine.
            # Default-off check first: nothing auto-sent since connect.
            assert not fc.bind_requests, "auto bind fired with cli.bind_rx_on_connect unset"
            response, _ = call(CL_CLI_CMD_BIND_RX, "")
            assert response.ok and response.status == CL_CLI_STATUS_OK
            assert response.text == "binding...", response.text
            assert len(fc.bind_requests) == 1
            assert fc.bind_requests[0][1] == ""

            # save through the service (fake FC does not actually persist)
            response, _ = call(CL_CLI_CMD_SAVE, "")
            assert response.ok and response.status == CL_CLI_STATUS_OK
            assert response.text == "saving...", response.text
            assert fc.save_requests and fc.save_requests[-1][1] == ""
            response, _ = call(CL_CLI_CMD_SAVE, "noreboot")
            assert response.ok and fc.save_requests[-1][1] == "noreboot"
            response, _ = call(CL_CLI_CMD_APPLY_PID, "check")
            assert response.ok and response.text == "PID PROFILE 0"
            response, _ = call(CL_CLI_CMD_APPLY_PID, "0")
            assert response.ok and fc.rx_cli[-1][1:] == (CL_CLI_CMD_APPLY_PID, "0")

            # repeated offboard true: the bridge must emit a fresh 0->2 edge
            # (one mode_req=0 frame followed by 2s) so the FC-side apiBlocked
            # unlocks on a re-request
            offb_pub = node.create_publisher(Bool, "/custom_link_bridge/offboard", 10)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and offb_pub.get_subscription_count() < 1:
                pump(0.05)
            fc.mode_reqs.clear()
            offb_pub.publish(Bool(data=True))
            pump(0.5)
            assert fc.mode_reqs and all(m == 2 for m in fc.mode_reqs), fc.mode_reqs
            fc.mode_reqs.clear()
            offb_pub.publish(Bool(data=True))    # repeat true == re-request
            pump(1.0)
            assert 0 in fc.mode_reqs, f"no 0->2 edge: {fc.mode_reqs}"
            z = fc.mode_reqs.index(0)
            assert fc.mode_reqs[z + 1:] and all(m == 2 for m in fc.mode_reqs[z + 1:]), \
                fc.mode_reqs
            assert fc.mode_reqs[-1] == 2

            # malformed request rejected before the link
            n_cli = len(fc.rx_cli)
            response, _ = call(9, "anything")
            assert not response.ok and response.error.startswith("bad request")
            assert len(fc.rx_cli) == n_cli               # never hit the wire

            # the link is still healthy afterwards
            response, _ = call(CL_CLI_CMD_GET, "failsafe_delay")
            assert response.ok and response.text == "failsafe_delay = 15"

            assert not any(level in log_path.read_text() for level in
                           ("FATAL", "process has died"))
    finally:
        shutdown(process)
        if connection is not None:
            connection.close()
        server.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_bind_rx_on_connect(tmp_path, monkeypatch):
    """cli.bind_rx_on_connect:=true: once the link comes up, the node sends
    bind_rx (0x24 cmd 3, empty args) exactly once per connection and logs
    the FC's "binding..." confirmation; a reconnect triggers it again."""
    monkeypatch.setenv("ROS_DOMAIN_ID", str(150 + os.getpid() % 60))
    binary = Path(os.environ.get("APM_BRIDGE_TEST_BINARY",
                  Path(__file__).resolve().parents[3] / "build/apm_bridge/custom_link_bridge_node"))
    assert binary.is_file(), f"Build the bridge first: {binary}"

    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    server.settimeout(5)

    process = connection = fc = None
    log_path = tmp_path / "bridge.log"
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen([
                str(binary), "--ros-args", "-p", "link.transport:=tcp",
                "-p", f"link.tcp_port:={server.getsockname()[1]}",
                "-p", "cli.bind_rx_on_connect:=true",
            ], stdout=log, stderr=subprocess.STDOUT)
            connection = server.accept()[0]
            connection.setblocking(False)
            fc = FakeFc(connection)

            def pump(duration):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    fc.pump()
                    time.sleep(0.005)

            # link up -> exactly one bind request with empty args + INFO log
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and len(fc.bind_requests) < 1:
                pump(0.05)
            pump(1.0)  # allow late duplicates to surface
            assert len(fc.bind_requests) == 1, fc.bind_requests
            assert fc.bind_requests[0][1] == ""
            log_text = log_path.read_text()
            assert "RX bind started on connect (bind_rx): binding..." in log_text

            # a reconnect fires it again (per-connection semantics)
            connection.close()
            fc.connection = connection = server.accept()[0]
            connection.setblocking(False)
            fc.buffer = b""
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and len(fc.bind_requests) < 2:
                pump(0.05)
            assert len(fc.bind_requests) == 2, fc.bind_requests
    finally:
        shutdown(process)
        if connection is not None:
            connection.close()
        server.close()
