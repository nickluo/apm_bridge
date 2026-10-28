#!/usr/bin/env python3
"""MISSION 模式真机云台验证: 真云台(/dev/ttyUSB1) + 真节点 + 假飞控(TCP) + 闭环 LOS 仿真。

场景: P0 AIM上视 → P1 进MISSION → P2 目标-10°(下)收敛 → P3 目标+20°(上)收敛
      → P4 LOS停发(应减速保持) → P5 退MISSION(应回AIM上视)
闭环 LOS: 订阅 /fpv/gimbal 相机俯仰, LOS_pitch(down+) = cam_up - target_up, 20Hz 新鲜发布。
"""
import math, os, socket, struct, subprocess, sys, time
sys.path.insert(0, '/home/nick/workspace/fpv_ws/src/apm_bridge/test')
from sitl_odometry_test import encode

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

BINARY = "/home/nick/workspace/fpv_ws/build/apm_bridge/custom_link_bridge_node"
GIMBAL_PORT = "/dev/ttyUSB1"
LOG = "/tmp/mission_live.log"
DEG = math.pi / 180

class Harness(Node):
    def __init__(self):
        super().__init__('mission_live_harness')
        self.mode = None
        self.cam_pitch = None   # FLU pitch (低头+), 上视为负
        self.modes_hist = []
        self.create_subscription(String, "/fpv/gimbal_mode", self.cb_mode,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Imu, "/fpv/gimbal", self.cb_cam, qos_profile_sensor_data)
        self.mission_pub = self.create_publisher(Bool, "/fpv/mission_state", 1)
        self.los_pub = self.create_publisher(TwistStamped, "/fpv/target_los", 10)
        self.gimbal_sub_count = 0

    def cb_mode(self, m):
        self.mode = m.data
        if not self.modes_hist or self.modes_hist[-1] != m.data:
            self.modes_hist.append(m.data)

    def cb_cam(self, m):
        q = m.orientation
        self.cam_pitch = math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y - q.z*q.x)))))
        self.gimbal_sub_count += 1

def main():
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(); server.settimeout(10)
    port = server.getsockname()[1]
    rclpy.init()
    h = Harness()
    proc = subprocess.Popen([BINARY, "--ros-args",
        "--params-file", "/home/nick/workspace/fpv_ws/src/apm_bridge/parameters/custom_link_parameters.yaml",
        "-p", "link.transport:=tcp", "-p", f"link.tcp_port:={port}",
        "-p", f"gimbal_port:={GIMBAL_PORT}",
        "-p", "baro_compensation:=false"],
        stdout=open(LOG, "w"), stderr=subprocess.STDOUT)
    conn = server.accept()[0]
    conn.setblocking(False)
    print(f"node pid {proc.pid}, tcp {port}", flush=True)

    state = dict(armed=False, mission=False, target=None, stop_los=False, samples=[])

    def pump(duration):
        deadline = time.monotonic() + duration
        los_seq = 0
        while time.monotonic() < deadline:
            ts = round(time.monotonic() * 1e6) & 0xffffffff
            try:
                conn.sendall(encode(0x10, struct.pack("<I9h", ts, 0, 0, 0, 0, 0, 0, 0, 0, 1000)))
                conn.sendall(encode(0x11, struct.pack("<IIih16HB", ts, 101325, 200, 2000, *([1500]*16), 87)))
                mode_flags = (1 << 13) if (state['armed'] and state['mission']) else 0
                conn.sendall(encode(0x12, struct.pack("<IHihBBiiiHHIH", ts, 24000, 0, 0, 0, 0, 0, 0, 0, 0, 0, mode_flags, int(state['armed']))))
                conn.recv(4096)
            except (BlockingIOError, socket.error):
                pass
            h.mission_pub.publish(Bool(data=state['mission']))
            if state['target'] is not None and not state['stop_los'] and h.cam_pitch is not None:
                cam_up = -h.cam_pitch
                los = TwistStamped()
                los.header.stamp = h.get_clock().now().to_msg()
                los.header.frame_id = "camera_flu"
                los.twist.linear.y = math.radians(cam_up - state['target'])  # pitch 下正
                los.twist.linear.z = 0.0
                los_seq += 1
                h.los_pub.publish(los)
            rclpy.spin_once(h, timeout_sec=0.002)
            t = time.monotonic()
            if h.cam_pitch is not None and ((not state['samples']) or (t - state['samples'][-1][0] > 0.05)):
                state['samples'].append((t, h.mode, -h.cam_pitch))  # cam_up
            time.sleep(0.01)

    def phase(name, dur, **kw):
        state.update(kw)
        t0 = time.monotonic()
        print(f"--- {name} ({dur}s) ---", flush=True)
        pump(dur)
        seg = [(m, u) for t, m, u in state['samples'] if t >= t0 + dur - 2.0]
        if seg:
            ups = [u for _, u in seg]
            mean = sum(ups)/len(ups)
            print(f"    末2s: mode={seg[-1][0]} 相机上视角均值 {mean:+.1f}° (范围 {min(ups):+.1f}~{max(ups):+.1f})", flush=True)
            return mean
        return None

    failures = []
    def check(cond, msg):
        print(f"  {'PASS' if cond else 'FAIL'}: {msg}", flush=True)
        if not cond: failures.append(msg)

    try:
        pump(2.0)  # 发现/稳定
        a = phase("P0 AIM 基线", 5, armed=False, mission=False, target=None)
        check(h.mode == "AIM", f"模式=AIM (实际 {h.mode})")
        check(a is not None and a > 70, f"AIM 上视 (均值 {a})")

        phase("P1a 仅武装 (armed, mission=false)", 2, armed=True, mission=False, target=None)
        check(h.mode == "AIM", f"武装后仍 AIM (实际 {h.mode})")
        phase("P1b mission=true", 2, armed=True, mission=True, target=None)
        check(h.mode == "MISSION", f"进入 MISSION (实际 {h.mode}, 历史 {h.modes_hist})")

        m2 = phase("P2 目标 -10° (水平下10°)", 7, armed=True, mission=True, target=-10.0)
        check(m2 is not None and -18 < m2 < -3, f"收敛到目标-10°附近 (末段 {m2})")

        m3 = phase("P3 目标 +20° (上视20°)", 7, armed=True, mission=True, target=20.0)
        check(m3 is not None and 12 < m3 < 28, f"收敛到目标+20°附近 (末段 {m3})")

        m4 = phase("P4 LOS 停发 (应保持)", 4, armed=True, mission=True, target=20.0, stop_los=True)
        check(m4 is not None and 10 < m4 < 30, f"LOS 超时后保持 ~20° (末段 {m4})")

        m5 = phase("P5 退 MISSION -> HORIZON", 6, armed=True, mission=False, target=None, stop_los=True)
        check(h.mode == "HORIZON", f"退回 HORIZON (实际 {h.mode})")
        check(m5 is not None and -10 < m5 < 10, f"HORIZON 回平 ~0° (末段 {m5})")
        m6 = phase("P6 解除武装 -> AIM", 5, armed=False, mission=False, target=None, stop_los=True)
        check(h.mode == "AIM", f"解除武装回 AIM (实际 {h.mode})")
        check(m6 is not None and m6 > 70, f"回 AIM 上视 (末段 {m6})")
    finally:
        proc.terminate()
        try: proc.wait(5)
        except Exception: proc.kill()
        conn.close(); server.close()
        rclpy.shutdown()
    print(f"\n{'ALL PASS' if not failures else 'FAILED: ' + '; '.join(failures)}", flush=True)
    print(f"模式历史: {h.modes_hist}")

if __name__ == "__main__":
    main()
