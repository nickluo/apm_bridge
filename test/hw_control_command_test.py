#!/usr/bin/env python3
"""真机 ROS 链路端到端验证：custom_link.launch.py 启动桥节点 + /fpv/control_command
控制 + /fpv/motor_rpm 回读闭环。

与 hw_custom_link_rpm_test.py（直连串口、绕过 ROS、raw 油门）互补：本脚本走
完整 ROS 数据面——launch 启动与话题重映射、/fpv/arm 与 /fpv/offboard 电平
语义、ControlCommand 集合推力换算（collective_thrust m/s² → 油门，镜像节点
vbat/baro 补偿）、节点控制量新鲜度超时（0.5 s 无指令 → 油门归零）。

QuadX 混控符号约定（同 hw_custom_link_rpm_test.py，台架闭环实证）：
  M0 右后 / M1 右前 / M2 左后 / M3 左前（俯视 M0/M3 桨顺时针）
  roll  右滚指令 + : 左对 (M2,M3) 加速，右对 (M0,M1) 减速
  pitch 低头指令 + : 后对 (M0,M2) 加速，前对 (M1,M3) 减速
  yaw   逆时针指令 +: CW 桨对 (M0,M3) 加速，CCW 桨对 (M1,M2) 减速

安全设计：
  * launch 固定带 bind_rx:=false（bind_rx 会把接收机锁进配对模式直到断电）
  * 必须显式 --confirm-props-off 才进入电机阶段（拆桨确认）
  * 电机时间线：解锁成功瞬间起四电机即以 motor_idle% 慢转（0x20 流默认携带
    mode_req=2，解锁即 OFFBOARD，且 OFFBOARD 下固件旁路 motor_stop），持续
    到阶段 9 上锁才真正停转——"spool-up" 阶段只是在此基础上提高油门
  * 电机阶段前校验：FC 未解锁 + /fpv/motor_rpm 四电机全有效（NaN=无效位）
  * 全程油门上限 --thrust-max（默认 14%）；failsafe/电流超限/转速超限立即
    arm=false + offboard=false 停机
  * 结束后断言回到未解锁且四电机转速归零

用法（先 source 工作区；用 /usr/bin/python3，勿用 miniconda python）：
  source ~/workspace/fpv_ws/install/setup.bash
  /usr/bin/python3 hw_control_command_test.py --port /dev/ttyUSB0  # 链路+遥测
  /usr/bin/python3 hw_control_command_test.py --confirm-props-off  # 全量电机验证
"""

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("PyYAML is required (comes with any sourced ROS 2 environment)")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Vector3
    from mavros_msgs.msg import RCIn, State
    from quadrotor_msgs.msg import ControlCommand
    from sensor_msgs.msg import BatteryState, FluidPressure, Temperature
    from std_msgs.msg import Bool, Float32MultiArray
except ImportError:  # pragma: no cover
    sys.exit("ROS 2 (rclpy/mavros_msgs/quadrotor_msgs) not importable — "
             "source the workspace setup.bash first and run with /usr/bin/python3")

# QuadX 电机分组（下标 = /fpv/motor_rpm.data 下标 = 固件 motor[i]）
RIGHT, LEFT, FRONT, REAR, CW, CCW = (0, 1), (2, 3), (1, 3), (0, 2), (0, 3), (1, 2)
SEA_LEVEL_PA = 101325.0
T0_KELVIN = 273.15


class ThrustModel:
    """quadratic_thrust_model.h forceToThrust 的 Python 镜像（含 vbat/baro 补偿），
    用于把目标油门换算成 ControlCommand.collective_thrust (m/s²)。补偿开关必须
    与节点 YAML 一致（load_model_params 读出；电机阶段前 fail-fast 校验）。"""

    def __init__(self, mp, lipo_cells, voltage_comp=False, baro_comp=True):
        self.A, self.B, self.C, self.D = (float(mp[k]) for k in "ABCD")
        self.n = int(mp.get("n", 1))
        self.thrust_limit = float(mp.get("thrust_limit", 1.0))
        self.spin_k = float(mp.get("spin_k", 0.0))
        self.vbat_a = float(mp.get("vbat_a", 0.0))
        self.vbat_b = float(mp.get("vbat_b", 1.0))
        self.volt_max = lipo_cells * 4.2
        self.volt_ref = lipo_cells * 3.7
        self.voltage_comp = voltage_comp
        self.baro_comp = baro_comp

    def _vbat_scale(self, vbat):
        return (self.vbat_a * vbat + self.vbat_b) / (self.vbat_a * self.volt_ref + self.vbat_b)

    def force_to_throttle(self, force_n, kp, vbat):
        x = force_n / self.n
        # 节点 ctrlCommandCallback 分支语义：voltage_compensation=false 时
        # 走 forceToThrust(无 vbat)（即 legacy 线性油门补偿），true 时走 vbat 模型
        vbat_on = self.voltage_comp and self.vbat_a != 0.0 and vbat > 0.0
        if vbat_on:
            x *= self._vbat_scale(vbat)
        t = ((self.A * x + self.B) * x + self.C) * x + self.D
        if vbat > 0.0 and not vbat_on:
            t *= self.volt_max / vbat
        t = (1.0 - self.spin_k) * t + self.spin_k * t * t
        if self.baro_comp:
            t *= kp
        return min(t, self.thrust_limit)

    def force_for_throttle(self, target, kp, vbat, hi_n=200.0):
        """force_to_throttle 单调升，二分反解目标油门对应的总推力 (N)。"""
        if self.force_to_throttle(hi_n, kp, vbat) < target:
            return None
        lo = 0.0
        for _ in range(80):
            mid = 0.5 * (lo + hi_n)
            if self.force_to_throttle(mid, kp, vbat) < target:
                lo = mid
            else:
                hi_n = mid
        return 0.5 * (lo + hi_n)


class Monitor(Node):
    """订阅桥节点全部对外话题；最新状态 + /fpv/motor_rpm 时间序列。"""

    def __init__(self):
        super().__init__("hw_control_command_test")
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state = None            # mavros_msgs/State 最新值
        self.battery = None          # mavros 电池（voltage V / current A，放电为负）
        self.pressure_pa = None      # 0x11 baro
        self.temp_c = None
        self.rssi = None             # 0x11 RC 链路质量 0..100
        self.failsafe = False
        self.rpm_count = 0           # 0x16 电机数（NaN = 对应 valid 位缺失）
        self.rpm = (float("nan"),) * 8
        self.rpm_log = deque(maxlen=4000)   # (monotonic, rpm 元组)
        self.rpm_ticks = 0
        self.create_subscription(State, "/mavros/state", self._state, 10)
        self.create_subscription(BatteryState, "/mavros/battery", self._battery, 10)
        self.create_subscription(Float32MultiArray, "/fpv/motor_rpm", self._rpm, 10)
        self.create_subscription(FluidPressure, "/fpv/static_pressure", self._pressure, qos_be)
        self.create_subscription(Temperature, "/fpv/temperature_baro", self._temp, qos_be)
        self.create_subscription(RCIn, "/mavros/rc/in", self._rc, qos_be)
        self.create_subscription(Bool, "/fpv/failsafe", self._failsafe, 10)
        self.arm_pub = self.create_publisher(Bool, "/fpv/arm", 10)
        self.offboard_pub = self.create_publisher(Bool, "/fpv/offboard", 10)
        self.cmd_pub = self.create_publisher(ControlCommand, "/fpv/control_command",
                                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _state(self, msg):
        self.state = msg

    def _battery(self, msg):
        self.battery = msg

    def _pressure(self, msg):
        self.pressure_pa = msg.fluid_pressure

    def _temp(self, msg):
        self.temp_c = msg.temperature

    def _rc(self, msg):
        self.rssi = msg.rssi

    def _failsafe(self, msg):
        self.failsafe = bool(msg.data)

    def _rpm(self, msg):
        self.rpm_count = len(msg.data)
        self.rpm = tuple(msg.data) + (float("nan"),) * max(0, 8 - len(msg.data))
        self.rpm_ticks += 1
        self.rpm_log.append((time.monotonic(), tuple(msg.data)))

    # ---- 便捷读取 ----
    def connected(self):
        return self.state is not None and self.state.connected

    def armed(self):
        return self.state is not None and self.state.armed

    def offboard(self):
        return self.state is not None and bool(self.state.guided)

    def voltage(self):
        return self.battery.voltage if self.battery is not None else None

    def current_a(self):
        return abs(self.battery.current) if self.battery is not None else None

    def kp(self):
        if self.pressure_pa is None or self.temp_c is None:
            return None
        return (self.pressure_pa / SEA_LEVEL_PA) * (T0_KELVIN / (T0_KELVIN + self.temp_c))

    def rpm_window(self, t0, t1):
        """[t0,t1] 窗内每电机平均转速（跳过 NaN 样本）；无样本返回 None 元组。"""
        sums, n = [0.0] * 4, [0] * 4
        for t, rpm in self.rpm_log:
            if t0 <= t <= t1:
                for i in range(4):
                    if i < len(rpm) and not math.isnan(rpm[i]):
                        sums[i] += rpm[i]
                        n[i] += 1
        return tuple(sums[i] / n[i] if n[i] else None for i in range(4))

    def publish_level(self, pub, value, hold_s=0.6, rate_hz=20.0):
        """电平语义话题在 best_effort 链路上多発几帧确保送达。"""
        msg = Bool()
        msg.data = bool(value)
        deadline = time.monotonic() + hold_s
        while time.monotonic() < deadline:
            pub.publish(msg)
            time.sleep(1.0 / rate_hz)


class ControlStream:
    """以固定频率发布 /fpv/control_command（BODY_RATES + 集合推力）。"""

    def __init__(self, monitor, rate_hz):
        self.monitor = monitor
        self.period = 1.0 / rate_hz
        self.enabled = threading.Event()
        self.collective = 0.0        # m/s²（线程间 float 赋值原子）
        self.rates = (0.0, 0.0, 0.0)  # FLU rad/s
        self._thread = None

    def start(self, collective, rates=(0.0, 0.0, 0.0)):
        self.collective = collective
        self.rates = rates
        self.enabled.set()
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def set(self, collective=None, rates=None):
        if collective is not None:
            self.collective = collective
        if rates is not None:
            self.rates = rates

    def stop(self):
        self.enabled.clear()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)

    def _run(self):
        while self.enabled.is_set():
            msg = ControlCommand()
            msg.header.stamp = self.monitor.get_clock().now().to_msg()
            msg.control_mode = ControlCommand.BODY_RATES
            msg.armed = True
            msg.is_hover_state = False
            msg.bodyrates = Vector3(x=self.rates[0], y=self.rates[1], z=self.rates[2])
            msg.collective_thrust = self.collective
            self.monitor.cmd_pub.publish(msg)
            time.sleep(self.period)


def pair_delta(rpm, a, b):
    """sum(a) - sum(b)：正 = a 组转速更高。"""
    if any(rpm[i] is None for i in a + b):
        return None
    return sum(rpm[i] for i in a) - sum(rpm[i] for i in b)


def fmt_rpm(rpm):
    return "(" + ", ".join(f"{r:.0f}" if r is not None else "None" for r in rpm[:4]) + ")"


def mean4(rpm):
    vs = [r for r in rpm[:4] if r is not None]
    return sum(vs) / len(vs) if len(vs) == 4 else None


def load_model_params(yaml_path):
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    p = cfg["custom_link_bridge"]["ros__parameters"]
    return (ThrustModel(p["motor_parameters"], int(p["lipo_cells"]),
                        voltage_comp=bool(p.get("voltage_compensation", False)),
                        baro_comp=bool(p.get("baro_compensation", True))),
            float(p["mass"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    default_dir = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--port", default="/dev/ttyUSB0", help="FC 定制链路串口")
    ap.add_argument("--parameters", default=os.path.join(default_dir, "..", "parameters",
                                                         "custom_link_parameters.yaml"),
                    help="桥节点参数 YAML（读取 mass/motor_parameters/lipo_cells）")
    ap.add_argument("--confirm-props-off", action="store_true",
                    help="确认已拆桨，允许进入电机验证阶段")
    ap.add_argument("--cmd-rate", type=float, default=50.0, help="控制指令发布频率 Hz")
    ap.add_argument("--thrust-low", type=float, default=0.06, help="基础油门 (0..1)")
    ap.add_argument("--thrust-high", type=float, default=0.12, help="阶跃油门 (0..1)")
    ap.add_argument("--thrust-max", type=float, default=0.14, help="油门硬上限")
    ap.add_argument("--rate-cmd", type=float, default=120.0, help="差动测试角速度指令 deg/s")
    ap.add_argument("--diff-min", type=float, default=200.0, help="差动断言最小转速差 rpm")
    ap.add_argument("--spin-min", type=float, default=300.0, help="起转断言最小转速 rpm")
    ap.add_argument("--current-max", type=float, default=8000.0, help="安全电流上限 mA")
    ap.add_argument("--rpm-max", type=float, default=40000.0, help="安全转速上限 rpm")
    ap.add_argument("--log", default="hw_control_command_bridge.log",
                    help="ros2 launch 输出日志文件")
    args = ap.parse_args()
    assert 0 < args.thrust_low < args.thrust_high <= args.thrust_max <= 1.0

    # 域隔离（可用 ROS_DOMAIN_ID 覆盖）：测试节点与被测桥同域，避免域 0 幽灵
    # 参与者污染（见 sitl_custom_link_test.py 同款守卫）
    os.environ.setdefault("ROS_DOMAIN_ID", "42")

    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {name}  {detail}")
        ok = ok and bool(cond)

    def wait_for(desc, predicate, timeout=8.0, ignore_abort=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            if abort_flag.get("reason") and not ignore_abort:
                return False
            time.sleep(0.05)
        print(f"  [FAIL] {desc} (timeout {timeout}s)")
        return False

    abort_flag = {}

    # ---- 0. 预检：串口、残留进程、ros2 CLI ----
    print("== 0. preflight ==")
    check("port_exists", os.path.exists(args.port), args.port)
    if shutil.which("ros2") is None:
        sys.exit("ros2 CLI 不在 PATH — 先 source ROS 环境与工作区")
    residue = subprocess.run(["pgrep", "-af", "custom_link_bridge_node"],
                             capture_output=True, text=True)
    check("no_existing_bridge", residue.returncode != 0, residue.stdout.strip())
    if not ok:
        print("已有 custom_link_bridge_node 在跑（会抢占串口）——先停掉再测")
        return 1

    model, mass = load_model_params(args.parameters)
    print(f"  model: n={model.n} limit={model.thrust_limit} mass={mass} kg")

    # ---- 启动节点（custom_link.launch.py，bind_rx:=false 防 RX 配对锁死）----
    launch_cmd = ["ros2", "launch", "apm_bridge", "custom_link.launch.py",
                  f"serial_port:={args.port}", "bind_rx:=false"]
    print("launch:", " ".join(launch_cmd), f"(ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']})")
    launch_log = open(args.log, "w")
    launch_proc = subprocess.Popen(launch_cmd, stdout=launch_log,
                                   stderr=subprocess.STDOUT,
                                   preexec_fn=os.setsid)

    try:
        rclpy.init()
        monitor = Monitor()
    except Exception as exc:
        os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
        raise SystemExit(f"rclpy 初始化失败: {exc}")
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(monitor), daemon=True)
    spin_thread.start()
    stream = ControlStream(monitor, args.cmd_rate)

    def kill_switch():
        """异常即停：断流 + 退 OFFBOARD + 上锁。"""
        stream.stop()
        monitor.publish_level(monitor.offboard_pub, False, hold_s=0.3)
        monitor.publish_level(monitor.arm_pub, False, hold_s=0.3)

    guard_run = threading.Event()       # 阶段 9 主动停用守护（正常收尾上锁不算异常）
    armed_seen = threading.Event()      # 首次观察到解锁后，意外掉 armed 即中止

    def safety_guard():
        while not abort_flag.get("reason") and guard_run.is_set():
            if monitor.armed():
                armed_seen.set()
            rpm4 = monitor.rpm[:4]
            bad = (monitor.failsafe or
                   (monitor.current_a() is not None and monitor.current_a() * 1000 > args.current_max) or
                   any((not math.isnan(r)) and r > args.rpm_max for r in rpm4) or
                   (armed_seen.is_set() and not monitor.armed()))
            if bad:
                abort_flag["reason"] = (f"failsafe={monitor.failsafe} "
                                        f"armed={monitor.armed()} "
                                        f"current={monitor.current_a() and round(monitor.current_a() * 1000)}mA "
                                        f"rpm={tuple(round(r) for r in rpm4 if not math.isnan(r))}")
                print(f"  ABORT: {abort_flag['reason']}")
                kill_switch()
            time.sleep(0.05)

    exit_code = 1
    try:
        # ---- 1. 链路建立 + 遥测活性 ----
        print("== 1. link bring-up (custom_link.launch.py) ==")
        if not wait_for("launch_alive", lambda: launch_proc.poll() is None, timeout=1.0):
            launch_log.flush()
            subprocess.run(["tail", "-30", args.log])
            print("RESULT: FAIL (ros2 launch 已退出)")
            return 1
        if not wait_for("state_topic", lambda: monitor.state is not None, 20.0):
            print("RESULT: FAIL (20s 内未见 /mavros/state)")
            return 1
        if not wait_for("fc_connected", monitor.connected, 15.0):
            launch_log.flush()
            subprocess.run(["tail", "-30", args.log])
            print("RESULT: FAIL (链路未建立 — 检查接线/波特率，须等遥测字节流出)")
            return 1
        check("link_up", True, f"mode={monitor.state.mode}")

        if not wait_for("battery", lambda: monitor.battery is not None, 10.0):
            print("RESULT: FAIL (未见 /mavros/battery)")
            return 1
        if not wait_for("baro", lambda: monitor.kp() is not None, 10.0):
            print("RESULT: FAIL (未见 /fpv/static_pressure 或 temperature_baro)")
            return 1
        print(f"    vbat={monitor.voltage():.2f} V, kp={monitor.kp():.4f}, "
              f"rssi={monitor.rssi if monitor.rssi is not None else 'n/a'}%")

        # ---- 2. /fpv/motor_rpm 基线（未解锁）----
        print("== 2. /fpv/motor_rpm baseline ==")
        t0, c0 = time.monotonic(), monitor.rpm_ticks
        time.sleep(2.0)
        rate = (monitor.rpm_ticks - c0) / (time.monotonic() - t0)
        check("motor_rpm_rate", 50 <= rate <= 160, f"{rate:.1f} Hz")
        check("motor_rpm_count", monitor.rpm_count == 4, f"count={monitor.rpm_count}")
        valid = all(not math.isnan(r) for r in monitor.rpm[:4])
        check("esc_telemetry_valid", valid,
              "" if valid else "存在 NaN（双向 DShot 遥测无效）— 检查 dshot_bidir=ON")
        if not wait_for("disarmed_baseline", lambda: not monitor.armed(), 6.0):
            print("  FC 处于解锁状态 — 拒绝电机测试")
            kill_switch()
            return 1
        rpm0 = monitor.rpm_window(time.monotonic() - 1.5, time.monotonic() - 0.2)
        check("motors_idle_when_disarmed",
              all(r is not None and r < args.spin_min for r in rpm0),
              f"rpm={fmt_rpm(rpm0)}")
        check("no_failsafe", not monitor.failsafe)
        base_current = monitor.current_a()
        if not (valid and ok):
            print(f"\nRESULT: FAIL (遥测基线不满足电机测试前提)")
            return 1

        # ---- 推力换算预检（armed 前验证模型镜像）----
        # 油门硬上限依赖镜像与节点补偿分支一致；baro_compensation=false 时节点
        # kp 初值不可镜像，voltage_compensation=false 时节点实际油门可比目标高
        # ~30%（满电 6S），两者任一关闭都拒绝电机阶段（fail-closed）。
        if not model.voltage_comp or not model.baro_comp:
            print(f"RESULT: FAIL (voltage_compensation={model.voltage_comp}, "
                  f"baro_compensation={model.baro_comp} — --thrust-max 硬上限依赖补偿模型)")
            return 1
        kp, vbat = monitor.kp(), monitor.voltage()
        collective = {}
        for name, throttle in (("low", args.thrust_low), ("high", args.thrust_high)):
            force = model.force_for_throttle(throttle, kp, vbat)
            if force is None:
                print(f"RESULT: FAIL (目标油门 {throttle} 超出模型域)")
                return 1
            collective[name] = force / mass
            rt = model.force_to_throttle(force, kp, vbat)
            check(f"thrust_map_{name}", abs(rt - throttle) < 0.002,
                  f"throttle {throttle:.3f} -> {collective[name]:.3f} m/s² (round-trip {rt:.4f})")
        if not ok:
            return 1

        if not args.confirm_props_off:
            print("\n电机阶段未启用（加 --confirm-props-off 确认拆桨后运行全量验证）")
            print(f"\nRESULT: PASS (link + telemetry, {rate:.0f} Hz motor_rpm)")
            return 0

        # ---- 3. 主机解锁（/fpv/arm 电平，FC 侧全量 tryArm 检查）----
        # 注意：0x20 流默认携带 mode_req=2（client ControlInput 默认），解锁成功
        # 瞬间 OFFBOARD 即接管，且 OFFBOARD 下固件旁路 motor_stop —— 四电机从
        # 解锁起就以 motor_idle% 慢转，直到阶段 9 上锁才真正停转。
        print("== 3. host arm via /fpv/arm (motors idle-spin from here) ==")
        guard = threading.Thread(target=safety_guard, daemon=True)
        guard.start()
        monitor.publish_level(monitor.arm_pub, True)
        if not wait_for("armed", monitor.armed, 12.0):
            print("  解锁失败 — 检查：油门杆低位/RX_FAILSAFE(rssi)/配置器 MSP 连接")
            kill_switch()
            return 1
        check("host_arms", True, f"rssi={monitor.rssi}%")
        time.sleep(0.6)
        rpm_arm = monitor.rpm_window(time.monotonic() - 0.5, time.monotonic() - 0.1)
        print(f"    motors idle-spinning since arming: rpm={fmt_rpm(rpm_arm)}")

        # ---- 4. OFFBOARD 接管（/fpv/offboard；对已接管的模式作显式确认）----
        print("== 4. OFFBOARD via /fpv/offboard ==")
        monitor.publish_level(monitor.offboard_pub, True)
        if not wait_for("offboard_mode", monitor.offboard, 5.0):
            print("  OFFBOARD 未接管 — 重发一次 offboard true（apiBlocked 解锁语义）")
            monitor.publish_level(monitor.offboard_pub, True)
            if not wait_for("offboard_mode_retry", monitor.offboard, 5.0):
                kill_switch()
                print("\nRESULT: FAIL")
                return 1
        check("offboard_engages", True,
              f"mode={monitor.state.mode} current={round((monitor.current_a() or 0) * 1000)}mA")

        # ---- 5. 起转：基础油门 ----
        print(f"== 5. spool-up (throttle {args.thrust_low:.0%}) ==")
        stream.start(collective["low"])
        time.sleep(2.0)
        if abort_flag.get("reason"):
            return 1
        rpm_lo = monitor.rpm_window(time.monotonic() - 1.2, time.monotonic() - 0.1)
        check("all_motors_spin", all(r is not None and r > args.spin_min for r in rpm_lo),
              f"rpm={fmt_rpm(rpm_lo)}")
        mean_lo = mean4(rpm_lo)
        if mean_lo is None:
            stream.stop()
            kill_switch()
            print("\nRESULT: FAIL (rpm 数据缺失)")
            return 1
        spread = max(rpm_lo) - min(rpm_lo)
        check("motors_balanced", spread < max(2000.0, 0.5 * mean_lo), f"spread={spread:.0f} rpm")
        check("rpm_in_expected_band", 2500 <= mean_lo <= 9999,
              f"mean={mean_lo:.0f} rpm @ {args.thrust_low:.0%} (上轮台架实测 ~4550)")
        check("current_rises", monitor.current_a() > base_current + 0.2,
              f"{round(base_current * 1000)} -> {round(monitor.current_a() * 1000)} mA")
        if any(r is None for r in rpm_lo):
            stream.stop()
            kill_switch()
            print("\nRESULT: FAIL")
            return 1

        # ---- 6. 推力阶跃 ----
        print(f"== 6. thrust step {args.thrust_low:.0%} -> {args.thrust_high:.0%} -> {args.thrust_low:.0%} ==")
        stream.set(collective=collective["high"])
        time.sleep(2.0)
        if abort_flag.get("reason"):
            return 1
        rpm_hi = monitor.rpm_window(time.monotonic() - 1.2, time.monotonic() - 0.1)
        mean_hi = mean4(rpm_hi)
        stream.set(collective=collective["low"])
        time.sleep(2.0)
        rpm_lo2 = monitor.rpm_window(time.monotonic() - 1.2, time.monotonic() - 0.1)
        mean_lo2 = mean4(rpm_lo2)
        if mean_hi is None or mean_lo2 is None:
            stream.stop()
            kill_switch()
            print(f"\nRESULT: FAIL (rpm 数据缺失: hi={fmt_rpm(rpm_hi)} lo2={fmt_rpm(rpm_lo2)})")
            return 1
        check("rpm_tracks_thrust", mean_hi > mean_lo * 1.15,
              f"mean {mean_lo:.0f} -> {mean_hi:.0f} rpm")
        check("rpm_tracks_thrust_down", mean_lo2 < mean_hi * 0.9,
              f"mean {mean_hi:.0f} -> {mean_lo2:.0f} rpm")

        # ---- 7. 三轴差动符号（/fpv/control_command bodyrates rad/s）----
        def axis_test(name, axis, up_group, down_group):
            # axis: 0=roll 1=pitch 2=yaw；正指令升速组/降速组按 QuadX 符号表
            print(f"== 7.{axis + 1} {name} differential ==")
            axis_ok = True
            for sign in (+1, -1):
                rates = [0.0, 0.0, 0.0]
                rates[axis] = sign * math.radians(args.rate_cmd)
                stream.set(rates=tuple(rates))
                time.sleep(1.6)
                if abort_flag.get("reason"):
                    axis_ok = False
                    break
                rpm = monitor.rpm_window(time.monotonic() - 1.2, time.monotonic() - 0.1)
                d = pair_delta(rpm, up_group, down_group)
                if d is None:
                    # ESC 遥测丢失（NaN=valid 位缺失）：继续盲发差动指令不安全，
                    # 与阶段 5/6 的数据缺失同处置——立即停机中止
                    stream.stop()
                    kill_switch()
                    abort_flag["reason"] = "esc_telemetry_lost"
                    print(f"\nRESULT: FAIL ({name} 差动窗口 rpm 数据缺失 — ESC 遥测丢失)")
                    return False
                good = abs(d) > args.diff_min and (d > 0) == (sign > 0)
                check(f"{name}_{'+' if sign > 0 else '-'}", good,
                      f"Δ(升组-降组)={d:.0f} rpm")
                axis_ok &= good
            stream.set(rates=(0.0, 0.0, 0.0))
            time.sleep(0.8)
            return axis_ok

        # 正指令升组：roll+ 右滚 → LEFT；pitch+ 低头 → REAR；yaw+ 逆时针 → CW(M0,M3)
        axis_ok = axis_test("roll", 0, LEFT, RIGHT)
        if not abort_flag.get("reason"):
            axis_ok &= axis_test("pitch", 1, REAR, FRONT)
        if not abort_flag.get("reason"):
            axis_ok &= axis_test("yaw", 2, CW, CCW)
        if abort_flag.get("reason"):
            return 1

        # ---- 8. 控制量新鲜度：停发指令 → 节点 0.5s 超时 → 油门归零 ----
        # 注意断言语义：arm 电平维持 0x20 流，OFFBOARD 不掉；OFFBOARD 下固件旁路
        # motor_stop，油门归零 = 四电机回到 motor_idle% 慢转（转速明显高于低于
        # 指令油门水平），并非停转。真正的停转断言在阶段 9 上锁之后。
        print("== 8. command freshness timeout (stop /fpv/control_command at high) ==")
        stream.set(collective=collective["high"], rates=(0.0, 0.0, 0.0))
        time.sleep(1.5)
        stream.stop()
        time.sleep(1.2)
        rpm_stale = monitor.rpm_window(time.monotonic() - 1.0, time.monotonic() - 0.1)
        mean_stale = mean4(rpm_stale)
        check("offboard_held_while_stale", monitor.offboard(),
              "arm 电平维持 0x20 流，OFFBOARD 按设计保持")
        check("motors_drop_to_idle_when_stale",
              mean_stale is not None and mean_stale < mean_hi * 0.85
              and all(r is not None and r > args.spin_min for r in rpm_stale),
              f"rpm={fmt_rpm(rpm_stale)}, mean {mean_hi:.0f} -> "
              f"{mean_stale if mean_stale else float('nan'):.0f} (motor_idle 慢转)")

        # ---- 9. 收尾：退 OFFBOARD + 上锁 ----
        print("== 9. disarm ==")
        guard_run.clear()  # 正常收尾上锁不是异常，先停用守护
        kill_switch()  # 已含 offboard=false + arm=false
        if not wait_for("offboard_drops", lambda: not monitor.offboard(), 5.0,
                        ignore_abort=True):
            pass  # 非致命：上锁后 OFFBOARD 自然退出
        if not wait_for("disarmed", lambda: not monitor.armed(), 8.0, ignore_abort=True):
            monitor.publish_level(monitor.arm_pub, False, hold_s=1.0)
            if not wait_for("disarmed_retry", lambda: not monitor.armed(), 5.0,
                            ignore_abort=True):
                print("  [FAIL] 未能上锁 — 手动断电！")
        check("final_disarmed", not monitor.armed())
        time.sleep(1.5)
        rpm_end = monitor.rpm_window(time.monotonic() - 1.2, time.monotonic() - 0.1)
        check("motors_stop_after_disarm",
              all(r is not None and r < args.spin_min for r in rpm_end),
              f"rpm={fmt_rpm(rpm_end)}")

        print("\n===== summary =====")
        print(f"  axis differential signs: {'PASS' if axis_ok else 'FAIL'}")
        exit_code = 0 if (ok and axis_ok) else 1
        print("\nRESULT:", "PASS" if exit_code == 0 else "FAIL")
        return exit_code
    finally:
        try:
            kill_switch()
        except Exception:
            pass
        try:
            launch_log.flush()
        except Exception:
            pass
        if launch_proc.poll() is None:
            try:
                os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
                launch_proc.wait(timeout=10)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        # ros2 launch 包装器被杀后真节点可能残留（既有坑），兜底清理
        subprocess.run(["pkill", "-f", "custom_link_bridge_node"], check=False)
        monitor.destroy_node()
        rclpy.shutdown()
        print(f"(bridge log: {args.log})")


if __name__ == "__main__":
    sys.exit(main())
