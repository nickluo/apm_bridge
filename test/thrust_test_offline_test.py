#!/usr/bin/python3
"""Offline (no FC, no hardware) test for thrust_test_node.

1. numeric self-test : inv_spin_map round trip + cubic fit round trip on a
   synthetic motor model
2. fake-graph test    : a FakeBridge node emulates the bridge/FC/scale side
   (state/battery/imu topics, arming/set_mode/param services, synthetic
   thrust->weight physics) while the real tool runs against it as a
   subprocess, for both backends (apm / custom). Asserts the command stream,
   MOT_THST_HOVER save/restore, CSV output and fitted coefficients.

Usage (workspace with apm_bridge built and sourced, system python):
  python3 test/thrust_test_offline_test.py
"""

import importlib.util
import math
import os
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
from quadrotor_msgs.msg import ControlCommand
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, Temperature
from std_msgs.msg import Bool, Float32MultiArray

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', 'scripts', 'thrust_test_node.py')

# ---- synthetic motor model: t = A f^3 + B f^2 + C f + D, F_total = n * f ----
SYN = {'A': 5.0e-5, 'B': -1.9e-3, 'C': 5.8e-2, 'D': 5.6e-3}
N_MOTORS = 4
LIPO_CELLS = 12
PACK_V = LIPO_CELLS * 4.2          # volt_max: x_poly == commanded throttle
VOLT_REF = LIPO_CELLS * 3.7         # 标准电压 (标称), vbat 曲线锚点
# 12S 量程自洽的 vbat 直线 y=a*V+b: y(V_ref)=1, 局部等效指数 ~2
VBAT_A = -2.0 / (VOLT_REF + 2.6)
VBAT_B = 1.0 - VBAT_A * VOLT_REF
SAG_PER_THRUST = 2.5                # 电池"IR 压降": 每单位油门掉的电压 [V]
TARE = 8.0                          # kg on the scale at zero thrust
GRAVITY = 9.801
HOVER_ORIGINAL = 0.35


def syn_force_total(t):
    """Invert the synthetic cubic (Newton) -> total force [N] at throttle t."""
    f = 0.5
    for _ in range(60):
        p = SYN['A'] * f ** 3 + SYN['B'] * f ** 2 + SYN['C'] * f + SYN['D']
        dp = 3.0 * SYN['A'] * f * f + 2.0 * SYN['B'] * f + SYN['C']
        f = max(f - (p - t) / dp, 0.0)
    return N_MOTORS * f


def pack_voltage(throttle):
    """电池电压: 标称电压附近随油门线性压降 (确定性 sag, 覆盖 V_ref 两侧)."""
    return VOLT_REF + 0.8 - SAG_PER_THRUST * min(max(throttle, 0.0), 1.0)


def vbat_scale(voltage):
    """phi(V) = (a*V+b)/(a*V_ref+b), 与 quadratic_thrust_model.h 同口径."""
    return (VBAT_A * voltage + VBAT_B) / (VBAT_A * VOLT_REF + VBAT_B)


def force_at(throttle, voltage):
    """vbat 电压模型下的总推力: F(t,V) = F0(t) / phi(V)."""
    return syn_force_total(min(max(throttle, 0.0), 1.0)) / vbat_scale(voltage)


class FakeBridge(Node):
    """Emulates either bridge + FC + bench scale."""

    def __init__(self, backend):
        super().__init__('fake_bridge')
        self.backend = backend
        self.lock = threading.Lock()
        self.armed = False
        self.mode = 'ACRO'
        self.last_cmd_time = 0.0
        self.last_cmd = None
        self.commands = []          # (time, thrust, armed_field, msg)
        self.arm_calls = []         # service (apm) / topic (custom) values
        self.param_sets = []        # (param_id, value)
        self.mode_sets = []         # custom_mode strings

        qos_be10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/mavros/battery', 10)
        self.imu_pub = self.create_publisher(Imu, '/mavros/imu/data', qos_be10)
        self.weight_pub = self.create_publisher(Float32MultiArray, '/current_weight', 10)
        # 两个桥接节点统一提供的环境话题
        self.pressure_pub = self.create_publisher(
            FluidPressure, '/fpv/static_pressure', qos_be10)
        self.temp_pub = self.create_publisher(
            Temperature, '/fpv/temperature_baro', qos_be10)

        self.create_subscription(ControlCommand, '/fpv/control_command_raw',
                                 self.on_command, qos_be10)
        if backend == 'custom':
            self.create_subscription(Bool, '/fpv/arm', self.on_arm_topic, qos_be10)

        if backend == 'apm':
            self.create_service(CommandBool, '/mavros/cmd/arming', self.on_arm_srv)
            self.create_service(SetMode, '/mavros/set_mode', self.on_set_mode)
            self.create_service(ParamGet, '/mavros/param/get', self.on_param_get)
            self.create_service(ParamSet, '/mavros/param/set', self.on_param_set)

        self.create_timer(0.2, self.publish_state)
        self.create_timer(0.1, self.publish_battery)
        self.create_timer(0.02, self.publish_imu)
        self.create_timer(0.1, self.publish_weight)
        self.create_timer(0.5, self.publish_ambient)

    # ---- FC 侧 ----

    def on_command(self, msg):
        with self.lock:
            self.last_cmd_time = time.monotonic()
            self.last_cmd = msg
            self.commands.append((time.monotonic(), msg.collective_thrust,
                                  bool(msg.armed), msg))

    def on_arm_srv(self, request, response):
        with self.lock:
            self.armed = request.value
            self.arm_calls.append(request.value)
        response.success = True
        return response

    def on_arm_topic(self, msg):
        with self.lock:
            self.armed = msg.data
            self.arm_calls.append(msg.data)

    def on_set_mode(self, request, response):
        with self.lock:
            self.mode_sets.append(request.custom_mode)
            self.mode = request.custom_mode
        response.mode_sent = True
        return response

    def on_param_get(self, request, response):
        response.success = (request.param_id == 'MOT_THST_HOVER')
        response.value = ParamValue(real=HOVER_ORIGINAL, integer=0)
        return response

    def on_param_set(self, request, response):
        with self.lock:
            self.param_sets.append((request.param_id, request.value.real))
        response.success = True
        response.value = request.value
        return response

    # ---- 遥测 ----

    def publish_state(self):
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = True
        with self.lock:
            msg.armed = self.armed
            if self.backend == 'custom':
                # OFFBOARD 生效条件: 已解锁 + 控制流新鲜
                msg.mode = ('OFFBOARD' if self.armed and
                            time.monotonic() - self.last_cmd_time < 0.5 else 'ACRO')
            else:
                msg.mode = self.mode
        self.state_pub.publish(msg)

    def publish_battery(self):
        with self.lock:
            throttle = self.last_cmd.collective_thrust if self.last_cmd else 0.0
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = pack_voltage(throttle)
        msg.cell_voltage = [msg.voltage]  # 本项目约定: 整包电压
        self.battery_pub.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation.w = 1.0
        self.imu_pub.publish(msg)

    def publish_ambient(self):
        p = FluidPressure()
        p.fluid_pressure = 101325.0
        self.pressure_pub.publish(p)
        t = Temperature()
        t.temperature = 25.0
        self.temp_pub.publish(t)

    def publish_weight(self):
        with self.lock:
            armed = self.armed
            thrust = self.last_cmd.collective_thrust if self.last_cmd else 0.0
        # 真值物理: vbat 电压模型, 推力随 sag 电压按 1/phi(V) 变化
        force = force_at(thrust, pack_voltage(thrust)) if armed else 0.0
        weight = TARE - force / GRAVITY
        self.weight_pub.publish(Float32MultiArray(data=[weight, weight]))


# ======================================================================
# 1. numeric self-test
# ======================================================================

def load_tool_module():
    spec = importlib.util.spec_from_file_location('thrust_test_node', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def numeric_self_test(results):
    tool = load_tool_module()

    ok = True
    for k in (0.0, 0.1, 0.3):
        for s in (0.1, 0.5, 0.9):
            t = (1.0 - k) * s + k * s * s
            if abs(tool.inv_spin_map(t, k) - s) > 1e-9:
                ok = False
    results['inv_spin_roundtrip'] = ok
    print(f"  [{'PASS' if ok else 'FAIL'}] inv_spin_map round trip")

    points = []
    for t in (0.10, 0.20, 0.30, 0.40):
        points.append({'x_poly': t, 'force_n': syn_force_total(t)})
    fit, err = tool.fit_motor_cubic(points, N_MOTORS)
    tol = 1e-6
    ok = fit is not None and all(abs(fit[k] - SYN[k]) < tol for k in SYN)
    results['fit_roundtrip'] = ok
    if fit:
        print(f"  [{'PASS' if ok else 'FAIL'}] cubic fit round trip: "
              f"A={fit['A']:.3g} B={fit['B']:.3g} C={fit['C']:.3g} D={fit['D']:.3g}")
    else:
        print(f'  [FAIL] cubic fit round trip: {err}')

    # vbat 补偿往返 (口径须与 quadratic_thrust_model.h 一致):
    # 运行时 f_arg = f*phi(V), t = P(f_arg); 反向 f = P^-1(t)/phi(V)
    ok = True
    for V in (PACK_V, VOLT_REF, VOLT_REF - 1.5):
        phi = tool.vbat_scale(VBAT_A, VBAT_B, VOLT_REF, V)
        for f in (3.0, 8.0):
            x = f * phi
            t = SYN['A'] * x ** 3 + SYN['B'] * x ** 2 + SYN['C'] * x + SYN['D']
            f_rt = syn_force_total(t) / N_MOTORS / phi
            if abs(f_rt - f) > 1e-9:
                ok = False
    results['vbat_roundtrip'] = ok
    print(f"  [{'PASS' if ok else 'FAIL'}] vbat scale round trip (force<->phi)")


# ======================================================================
# 2. fake-graph test
# ======================================================================

# 工具参数: 3 档扫油门, 快速流程 (不传 --kp, 走 /fpv/static_pressure 自动采集)
TOOL_ARGS = ['--yes', '--no-plot',
             '--min-thrust', '10', '--max-thrust', '40', '--step', '10',
             '--settle-time', '0.3', '--samples-per-point', '10',
             '--control-rate', '20', '--num-motors', str(N_MOTORS),
             '--lipo-cells', str(LIPO_CELLS),
             '--map-a', str(VBAT_A), '--map-b', str(VBAT_B)]


def run_tool_against_fake(backend, out_dir, results):
    print(f'== fake-graph test: --backend {backend} ==')
    rclpy.init()
    fake = FakeBridge(backend)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(fake)

    def spin_guarded():
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass

    spin = threading.Thread(target=spin_guarded, daemon=True)
    spin.start()
    time.sleep(0.5)  # 让发布/服务就绪

    try:
        check_tool_run(fake, backend, out_dir, results)
    finally:
        executor.remove_node(fake)
        rclpy.shutdown()
        spin.join(timeout=2.0)
        fake.destroy_node()


def check_tool_run(fake, backend, out_dir, results):
    cmd = [sys.executable, SCRIPT, '--backend', backend,
           '--output-dir', out_dir] + TOOL_ARGS
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True)
    try:
        stdout, _ = proc.communicate(timeout=90.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        stdout, _ = proc.communicate()
        print(stdout[-2000:])
        results[f'{backend}_exit'] = False
        return
    if proc.returncode != 0:
        print(stdout[-2000:])

    with fake.lock:
        commands = list(fake.commands)
        param_sets = list(fake.param_sets)
        mode_sets = list(fake.mode_sets)
        arm_calls = list(fake.arm_calls)

    results[f'{backend}_exit'] = proc.returncode == 0

    # ---- kp 自动采集走 /fpv/static_pressure (打印 'KP = x' 而非告警/用户指定) ----
    ok = 'KP = ' in stdout and 'no /fpv/static_pressure' not in stdout
    results[f'{backend}_kp_topic'] = ok
    print(f"  [{'PASS' if ok else 'FAIL'}] kp collected from /fpv/static_pressure")

    # ---- 命令流 ----
    test_cmds = [c for c in commands if c[2]]  # armed 字段为真的命令
    ok = len(test_cmds) > 0
    for _, thrust, _, msg in test_cmds:
        if msg.control_mode != ControlCommand.ATTITUDE:
            ok = False
        if abs(msg.bodyrates.x) > 1e-9 or abs(msg.bodyrates.y) > 1e-9 or \
                abs(msg.bodyrates.z) > 1e-9:
            ok = False
        if abs(msg.orientation.w - 1.0) > 1e-9 or abs(msg.orientation.x) > 1e-9:
            ok = False
    thrusts = sorted({round(c[1], 6) for c in test_cmds})
    results[f'{backend}_command_stream'] = ok and thrusts == [0.1, 0.2, 0.3, 0.4]
    print(f"  [{'PASS' if results[f'{backend}_command_stream'] else 'FAIL'}] "
          f'command stream (ATTITUDE, zero rates, identity quat), '
          f'throttle steps: {thrusts}')

    # ---- 阶梯顺序: 每档先稳定再采集, 0.1 -> 0.2 -> 0.3 -> 0.4 依次首次出现 ----
    order = []
    for _, thrust, armed_field, _ in commands:
        if armed_field:
            r = round(thrust, 6)
            if r not in order:
                order.append(r)
    results[f'{backend}_sweep_order'] = order == [0.1, 0.2, 0.3, 0.4]
    print(f"  [{'PASS' if results[f'{backend}_sweep_order'] else 'FAIL'}] "
          f'sweep order: {order}')

    if backend == 'apm':
        # ---- MOT_THST_HOVER 覆盖与恢复 ----
        hover_sets = [v for pid, v in param_sets if pid == 'MOT_THST_HOVER']
        results[f'{backend}_hover_param'] = (
            any(abs(v - 0.4) < 1e-9 for v in hover_sets) and
            any(abs(v - HOVER_ORIGINAL) < 1e-9 for v in hover_sets))
        print(f"  [{'PASS' if results[f'{backend}_hover_param'] else 'FAIL'}] "
              f'MOT_THST_HOVER override+restore: {hover_sets}')
        # ---- set_mode ----
        results[f'{backend}_set_mode'] = 'GUIDED_NOGPS' in mode_sets
        print(f"  [{'PASS' if results[f'{backend}_set_mode'] else 'FAIL'}] "
              f'set_mode: {mode_sets}')
    else:
        # ---- /fpv/arm keepalive + 退出上锁 ----
        ok = True in arm_calls and False in arm_calls
        results[f'{backend}_arm_topic'] = ok
        print(f"  [{'PASS' if ok else 'FAIL'}] /fpv/arm True->False: "
              f"{len([a for a in arm_calls if a])}x True, "
              f"{len([a for a in arm_calls if not a])}x False")

    # ---- CSV 与拟合 ----
    csv_files = sorted(f for f in os.listdir(out_dir) if f.endswith('.csv'))
    ok = bool(csv_files)
    fitted = None
    if ok:
        import csv as csv_mod
        with open(os.path.join(out_dir, csv_files[-1])) as f:
            rows = list(csv_mod.DictReader(f))
        ok = len(rows) == 4
        for row, t in zip(rows, (0.1, 0.2, 0.3, 0.4)):
            if abs(float(row['thrust_norm']) - t) > 1e-6:
                ok = False
            # 工具记录的是 vbat 归一后的力: 恰好等于标准电压口径 F0(t)
            if abs(float(row['force_N']) - syn_force_total(t)) > 1e-3:
                ok = False
        # 各点电压确实不同 (归一化路径被真实 exercised)
        results[f'{backend}_voltage_varied'] = \
            len({round(float(r['voltage']), 2) for r in rows}) == len(rows)
        print(f"  [{'PASS' if results[f'{backend}_voltage_varied'] else 'FAIL'}] "
              f"voltage sag varied across sweep: "
              f"{[round(float(r['voltage']), 2) for r in rows]}")
        # 结果 sidecar
        yaml_files = sorted(f for f in os.listdir(out_dir)
                            if f.endswith('_result.yaml'))
        if yaml_files:
            with open(os.path.join(out_dir, yaml_files[-1])) as f:
                text = f.read()
            vals = {}
            for key in SYN:
                for line in text.splitlines():
                    if line.strip().startswith(key + ':'):
                        vals[key] = float(line.split(':')[1])
            fitted = vals
            ok = ok and all(abs(vals.get(k, 1e9) - SYN[k]) < 1e-4 for k in SYN)
    results[f'{backend}_csv_fit'] = ok
    print(f"  [{'PASS' if ok else 'FAIL'}] CSV + fitted coefficients "
          f"{fitted if fitted else ''}")


def main():
    results = {}
    print('== numeric self-test ==')
    numeric_self_test(results)

    with tempfile.TemporaryDirectory(prefix='thrust_test_offline_') as out_dir:
        for backend in ('apm', 'custom'):
            run_tool_against_fake(backend, os.path.join(out_dir, backend), results)

    print('\n===== summary =====')
    all_ok = True
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
        all_ok &= bool(v)
    print('RESULT:', 'PASS' if all_ok else 'FAIL')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
