#!/usr/bin/python3
"""油门-拉力曲线自动标定工具 (kestrel_utils/thrust_test_dt30x.py 的 ROS2 移植)。

台架标定: 无人机固定在台秤上 (DT30X 经 scale_reader_node 发布 /current_weight，
或任何遵循 [weight_kg, stable_weight_kg(NaN=未稳定)] 约定的发布器)，通过本包的
桥接节点以 raw 通道直通油门，自动扫油门采集 (油门, 拉力) 数据点并拟合
quadratic_thrust_model 所需的 motor_parameters 系数:

    thrust = kp * ((1-k)*thr + k*thr^2),  thr = A*f^3 + B*f^2 + C*f + D
    f = F_total / n  (单电机受力)
    系数为标准电压 V_ref = lipo_cells*3.7 (标称) 口径: 标定时实测力经
    f_ref = f * (a*V+b)/(a*V_ref+b) 归一 (--map-a/--map-b 来自 --vbat 拟合),
    运行时桥接在多项式入口施加同一因子 (motor_parameters.vbat_a/vbat_b)。

一个工具支持两种后端 (桥接节点需已运行)，两种后端均在 /fpv/static_pressure 与
/fpv/temperature_baro 上提供环境气压/温度 (apm 桥转发 mavros，custom 桥来自 0x11
遥测)，kp 采集两后端口径一致:
  --backend apm     : apm_bridge_node + mavros。工具自设 GUIDED_NOGPS 模式、
                      保存并临时覆盖 MOT_THST_HOVER (退出恢复)、直接调 mavros
                      服务解锁 (不走 /fpv/arm，规避 VehicleNode::armCallback 的
                      嵌套 spin 问题)。
  --backend custom  : custom_link_bridge_node (定制链路 Betaflight)。解锁走
                      /fpv/arm，需飞手保持 BOXOFFBOARD 开关。

采集流程 (自动，无键盘交互):
  从 --min-thrust 起，每档 +--step 直到 --max-thrust；每档等 --settle-time
  稳定后以 10 Hz 连采 --samples-per-point 个稳定重量取平均记为一个数据点。
  Ctrl+C 中止: 走安全关停 (油门回最低、上锁、恢复参数)，已采数据照常输出。

安全: 采集中重量低于 --critical-weight 自动降油门 (防抬离秤)；扫描中意外
掉锁则立即中止。

用法 (需系统 python，conda 环境请先 deactivate):
  ros2 run apm_bridge thrust_test_node --backend custom --lipo-cells 12 --num-motors 4
"""

import argparse
import bisect
import csv
import datetime
import math
import os
import select
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

import numpy as np
from quadrotor_msgs.msg import ControlCommand
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, CommandLong, ParamGet, ParamSet, SetMode
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, Temperature
from std_msgs.msg import Bool, Float32MultiArray

# MAV_CMD_SET_MESSAGE_INTERVAL (与 VehicleNode::setupMavlink 同一命令)
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MSG_ID_ATTITUDE_QUATERNION = 31
MSG_ID_BATTERY_STATUS = 147
MSG_ID_SCALED_PRESSURE = 29

# custom 后端上锁后控制流需继续保持，arm=0 边沿随 0x20 帧送达 FC (SITL 验证语义)
DISARM_STREAM_TIME = 2.0
# 采集时重量样本最长等待 (无秤/秤一直不稳定则放弃该点)
COLLECT_TIMEOUT_MARGIN = 5.0


class bcolors:
    ERROR = '\033[91m'
    WARNING = '\033[93m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    ENDC = '\033[0m'


class FatalError(Exception):
    """启动阶段不可恢复的错误 (安全关停后退出)。"""


# ======================================================================
# 纯函数: 模型反解与拟合 (离线可测)
# ======================================================================

def inv_spin_map(s, k):
    """(1-k)t + k t^2 的逆映射: 已知 s 求 t。k 近 0 时直接返回 s。"""
    if abs(k) <= 1e-9:
        return s
    k_1 = k - 1.0
    inner = k_1 * k_1 + 4.0 * k * s
    if inner < 0.0:
        inner = 0.0
    return (k_1 + math.sqrt(inner)) / (2.0 * k)


def normalize_throttle(thrust_norm, kp, spin_k, voltage, volt_max, voltage_comp):
    """把命令油门 t 反解为多项式输入 thr: t = kp*spin(thr*[volt_max/V])。

    volt_max 归一化的系数在电压补偿开启时才是运行时正确的口径；
    --map-a/--map-b (vbat) 模式下不走本函数的电压项，电压归一在力侧完成。
    """
    x = inv_spin_map(thrust_norm / kp, spin_k)
    if voltage_comp and volt_max > 0.0 and voltage > 0.0:
        x *= voltage / volt_max
    return x


def vbat_scale(a, b, vref, voltage):
    """phi(V) = (a*V+b)/(a*V_ref+b): 当前电压力 <-> 标准电压口径力的换算因子。

    与 quadratic_thrust_model.h 的 vbatScale 同口径，标定与运行时对称使用。
    """
    return (a * voltage + b) / (a * vref + b)


def fit_motor_cubic(points, n_motors):
    """拟合 x = A*y^3 + B*y^2 + C*y + D, y = F_total/n, x = 多项式输入油门。

    points: Recorder 的数据点列表。返回 (fit_dict, err_reason)。
    """
    if len(points) < 4:
        return None, 'samples less than 4, cubic fit needs at least 4 points'
    y = np.array([p['force_n'] / n_motors for p in points])
    x = np.array([p['x_poly'] for p in points])
    # Polynomial.fit 对自变量做窗口缩放，避免 y^3 量级 (1e5) 下的病态 Vandermonde
    poly = np.polynomial.Polynomial.fit(y, x, 3)
    coef = poly.convert().coef  # 升幂: D, C, B, A
    D, C, B, A = (float(v) for v in coef)
    rms = float(np.sqrt(np.mean((poly(y) - x) ** 2)))
    ys = np.linspace(float(np.min(y)), float(np.max(y)), 200)
    deriv_min = float(np.min(3.0 * A * ys * ys + 2.0 * B * ys + C))
    return {'A': A, 'B': B, 'C': C, 'D': D, 'rms': rms,
            'deriv_min': deriv_min, 'n_points': len(points)}, None


def fit_voltage_linear(points):
    """vbat 模式: 相对拉力 y = mean(F)/F 对整包电压 V 的一次拟合，返回 (a, b)。"""
    if len(points) < 3:
        return None, 'samples less than 3'
    forces = np.array([p['force_n'] for p in points])
    voltages = np.array([p['voltage'] for p in points])
    y = float(np.mean(forces)) / forces
    poly = np.polynomial.Polynomial.fit(voltages, y, 1).convert()
    return (float(poly.coef[1]), float(poly.coef[0])), None


def render_param_yaml(node_key, fit, n_motors, spin_k, vbat, notes):
    """生成可直接并入 parameters/*.yaml 的 motor_parameters 片段。

    vbat: (--map-a, --map-b) 元组，非 None 时一并写出 vbat_a/vbat_b。
    """
    lines = [f'{node_key}:',
             '  ros__parameters:',
             '    motor_parameters:']
    for key in ('A', 'B', 'C', 'D'):
        lines.append(f'      {key}: {fit[key]:.9g}')
    lines.append(f'      n: {n_motors}')
    lines.append(f'      spin_k: {spin_k:.9g}')
    if vbat is not None:
        lines.append(f'      vbat_a: {vbat[0]:.9g}')
        lines.append(f'      vbat_b: {vbat[1]:.9g}')
    for note in notes:
        lines.append(f'    # {note}')
    return '\n'.join(lines)


# ======================================================================
# 数据记录
# ======================================================================

class Recorder:
    """数据点按 x 升序插入、按 key 去重 (对齐旧工具 insert_data/insert_vbat)。"""

    def __init__(self):
        self.points = []
        self.keys = set()

    def clear(self):
        self.points = []
        self.keys = set()

    def insert_normal(self, *, throttle_pct, thrust_norm, voltage, weight_kg,
                      force_n, kp, x_poly):
        key = round(throttle_pct, 3)
        if key in self.keys:
            return False
        self.keys.add(key)
        point = {'time': time.time(), 'throttle_pct': throttle_pct,
                 'thrust_norm': thrust_norm, 'voltage': voltage,
                 'weight_kg': weight_kg, 'force_n': force_n,
                 'kp': kp, 'x_poly': x_poly}
        self._insert_sorted(point)
        return True

    def insert_vbat(self, *, voltage, force_n, weight_kg, vmin, vmax):
        if voltage < vmin or voltage > vmax:
            return False
        key = int(voltage * 100)
        if key in self.keys:
            return False
        self.keys.add(key)
        point = {'time': time.time(), 'throttle_pct': float('nan'),
                 'thrust_norm': float('nan'), 'voltage': voltage,
                 'weight_kg': weight_kg, 'force_n': force_n,
                 'kp': float('nan'), 'x_poly': float('nan')}
        self._insert_sorted(point)
        return True

    def _insert_sorted(self, point):
        pos = bisect.bisect([p['x_poly'] if not math.isnan(p['x_poly']) else math.inf
                             for p in self.points], point['x_poly'])
        self.points.insert(pos, point)


# ======================================================================
# 节点
# ======================================================================

class ThrustTestNode(Node):

    def __init__(self, args):
        super().__init__('thrust_test_node')
        self.args = args
        self.lock = threading.RLock()
        self.abort_event = threading.Event()
        self.state_event = threading.Event()
        self.baro_event = threading.Event()
        self.weight_event = threading.Event()

        # ---- 共享状态 (lock 保护) ----
        self.thrust_pct = args.min_thrust
        self.testing = False
        self.disarming = False
        self.armed = False
        self.mode = ''
        self.whole_weight = 0.0            # 去皮基准 (kg)
        self.stable_weight = float('nan')  # 合成后的稳定重量 (kg)
        self.battery_voltage = 0.0         # 整包电压
        self.orientation = None            # 最新 IMU 四元数 (姿态回显)
        self.baro = 101325.0
        self.temp = 25.0
        self.kp_avg = 1.0
        self.quiet = False                 # 采集进度期间抑制 1 Hz 状态行
        self._arm_intent = False           # custom 后端 /fpv/arm 期望电平
        self._voltage_warned = False

        self.recorder = Recorder()
        self.hover_saved = None            # (ParamValue) 原始 MOT_THST_HOVER

        # ---- 发布 ----
        qos_be5 = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_be10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(ControlCommand,
                                             '/fpv/control_command_raw', qos_be5)
        if args.backend == 'custom':
            self.arm_pub = self.create_publisher(Bool, '/fpv/arm', qos_be10)

        # ---- 订阅 ----
        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(BatteryState, '/mavros/battery',
                                 self.on_battery, qos_be10)
        self.create_subscription(Float32MultiArray, '/current_weight',
                                 self.on_weight, qos_be10)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_be10)
        # 环境气压/温度: 两个桥接节点均发布统一的 /fpv 话题 (custom 链路 0x11 遥测，
        # apm 侧由桥转发 mavros)，供 kp 采集
        self.create_subscription(FluidPressure, '/fpv/static_pressure',
                                 self.on_pressure, qos_be10)
        self.create_subscription(Temperature, '/fpv/temperature_baro',
                                 self.on_temperature, qos_be10)

        # ---- 服务 (仅 apm) ----
        if args.backend == 'apm':
            self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
            self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
            self.param_get_client = self.create_client(ParamGet, '/mavros/param/get')
            self.param_set_client = self.create_client(ParamSet, '/mavros/param/set')
            self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # ---- 定时器 ----
        self.control_timer = self.create_timer(1.0 / args.control_rate,
                                               self.control_timer_cb)
        if args.backend == 'custom':
            self.arm_keepalive_timer = self.create_timer(0.5, self.arm_keepalive_cb)
        self.status_timer = self.create_timer(1.0, self.status_timer_cb)

    # ---------------------------------------------------------- 订阅回调

    def on_state(self, msg: State):
        with self.lock:
            was_testing = self.testing
            was_armed = self.armed
            disarming = self.disarming
            self.armed = msg.armed
            self.mode = msg.mode
        self.state_event.set()
        if was_testing and was_armed and not msg.armed and not disarming:
            print(f'\n{bcolors.ERROR}*** FC DISARMED during test! Aborting. ***{bcolors.ENDC}')
            self.abort_event.set()

    def on_battery(self, msg: BatteryState):
        voltage = 0.0
        if len(msg.cell_voltage) > 0:
            voltage = float(msg.cell_voltage[0])  # 本项目约定: cell_voltage[0] = 整包
        elif not math.isnan(msg.voltage):
            voltage = float(msg.voltage)
        with self.lock:
            self.battery_voltage = voltage
            warned = self._voltage_warned
        if (not warned and self.args.lipo_cells >= 2 and 0.0 < voltage < 2.0 * 3.0):
            self._voltage_warned = True
            print(f'{bcolors.WARNING}battery voltage {voltage:.2f} V looks like a single '
                  f'cell -- is /mavros/battery really pack voltage?{bcolors.ENDC}')

    def on_weight(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        weight = float(msg.data[0])
        stable = float(msg.data[1])
        with self.lock:
            if math.isnan(stable):
                self.stable_weight = weight
            else:
                self.stable_weight = stable
                if not self.testing:
                    self.whole_weight = stable
            testing = self.testing
        self.weight_event.set()
        if testing and weight < self.args.critical_weight:
            # 防抬离秤: 每次回调降 0.5% (同旧工具)
            print(f'{bcolors.ERROR}Current weight below CRITICAL_WEIGHT! '
                  f'Forced to decrease throttle.{bcolors.ENDC}')
            self.adjust_thrust(-0.5)

    def on_imu(self, msg: Imu):
        with self.lock:
            self.orientation = msg.orientation

    def on_pressure(self, msg: FluidPressure):
        with self.lock:
            self.baro = msg.fluid_pressure
        self.baro_event.set()

    def on_temperature(self, msg: Temperature):
        with self.lock:
            self.temp = msg.temperature

    # ---------------------------------------------------------- 定时器回调

    def control_timer_cb(self):
        """50 Hz 控制流: ATTITUDE 模式 + 姿态回显 + 零角速度 + 直通油门。

        APM 侧桥接生成 AttitudeTarget{orientation=回显, IGNORE_*_RATE, thrust}，
        与旧工具线上格式一致；custom 侧忽略姿态、透传零角速度 + 油门。
        """
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.control_mode = ControlCommand.ATTITUDE
        with self.lock:
            cmd.armed = self.testing
            cmd.collective_thrust = min(max(self.thrust_pct / 100.0, 0.0), 1.0)
            orientation = self.orientation
        if orientation is not None:
            cmd.orientation = orientation
        else:
            # geometry_msgs/Quaternion 默认全零，非单位四元数，需显式置 w=1
            cmd.orientation.x = 0.0
            cmd.orientation.y = 0.0
            cmd.orientation.z = 0.0
            cmd.orientation.w = 1.0
        # bodyrates 保持 0
        self.cmd_pub.publish(cmd)

    def arm_keepalive_cb(self):
        """custom 后端 arm 请求 2 Hz 重发 (level 语义)。"""
        with self.lock:
            armed_intent = self.testing or self._arm_intent
        self.arm_pub.publish(Bool(data=armed_intent))

    def status_timer_cb(self):
        if self.quiet:
            return
        with self.lock:
            thrust = self.thrust_pct
            weight = self.stable_weight
            voltage = self.battery_voltage
            mode = self.mode
            armed = self.armed
        weight_s = f'{weight:.3f}' if not math.isnan(weight) else '--'
        voltage_s = f'{voltage:.2f}' if voltage > 0.0 else '--'
        print(f'\r[t={thrust:5.1f}%] [w={weight_s} kg] [V={voltage_s}] '
              f'[mode={mode or "--"}{" armed" if armed else ""}] '
              f'[pts={len(self.recorder.points)}]   ', end='', flush=True)

    # ---------------------------------------------------------- 油门控制

    def set_thrust(self, value_pct, force=False):
        """设定油门 (%，钳位 [min,100])。force=True 时直接压到 min (安全动作)。"""
        with self.lock:
            if not force:
                value_pct = min(max(value_pct, self.args.min_thrust), 100.0)
            changed = self.thrust_pct != value_pct
            self.thrust_pct = value_pct
        if changed and not self.quiet:
            print(f'thrust_req = {value_pct:.2f}%')

    def adjust_thrust(self, delta_pct):
        with self.lock:
            self.thrust_pct = min(max(self.thrust_pct + delta_pct,
                                      self.args.min_thrust), 100.0)

    def get_thrust(self):
        with self.lock:
            return self.thrust_pct

    def sleep_abortable(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                return False
            time.sleep(0.05)
        return not self.abort_event.is_set()

    # ---------------------------------------------------------- 服务 (apm)

    def call_service(self, client, request, timeout_sec=15.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(request)
        # rclpy 的 Future.result() 不带超时参数，executor 在后台线程驱动
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            self.get_logger().error(f'service call timed out after {timeout_sec:.0f}s')
            return None
        return future.result()

    def set_message_intervals(self):
        """请求遥测流速率 (尽力而为)。"""
        for msg_id, rate in ((MSG_ID_ATTITUDE_QUATERNION, 20.0),
                             (MSG_ID_BATTERY_STATUS, 5.0),
                             (MSG_ID_SCALED_PRESSURE, 5.0)):
            req = CommandLong.Request()
            req.broadcast = False
            req.command = MAV_CMD_SET_MESSAGE_INTERVAL
            req.param1 = float(msg_id)
            req.param2 = 1.0e6 / rate
            resp = self.call_service(self.command_client, req, timeout_sec=5.0)
            if resp is None or not resp.success:
                print(f'{bcolors.WARNING}set_message_interval msg {msg_id} failed '
                      f'(continuing){bcolors.ENDC}')

    def set_guided_mode(self):
        req = SetMode.Request()
        req.custom_mode = self.args.guided_mode
        resp = self.call_service(self.set_mode_client, req)
        if resp is None or not resp.mode_sent:
            raise FatalError(f'failed to set mode {self.args.guided_mode}')
        # 验证 state.mode (5 s，告警不致命)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self.lock:
                if self.mode == self.args.guided_mode:
                    break
            if not self.sleep_abortable(0.2):
                return
        with self.lock:
            ok = self.mode == self.args.guided_mode
        if not ok:
            print(f'{bcolors.WARNING}mode did not confirm to {self.args.guided_mode} '
                  f'(current: {self.mode}){bcolors.ENDC}')

    def save_and_override_hover_param(self):
        """保存 MOT_THST_HOVER 并临时覆盖为 max_thrust/100，中和悬停推力补偿。

        ArduPilot 的推力线性化会使命令推力与电机输出不成线性，标定前必须搬开。
        """
        for _ in range(3):
            resp = self.call_service(self.param_get_client,
                                     ParamGet.Request(param_id='MOT_THST_HOVER'))
            if resp is not None and resp.success:
                break
            self.sleep_abortable(1.0)
        else:
            print(f'{bcolors.WARNING}cannot read MOT_THST_HOVER, skipping override '
                  f'(curve may be distorted){bcolors.ENDC}')
            return
        self.hover_saved = resp.value
        print(f'{bcolors.OKBLUE}Keep original MOT_THST_HOVER: '
              f'{self._param_value(resp.value)}{bcolors.ENDC}')
        self.set_hover_param(ParamValue(real=self.args.max_thrust / 100.0, integer=0))

    def restore_hover_param(self):
        if self.hover_saved is None:
            return
        if self.set_hover_param(self.hover_saved):
            print('Original MOT_THST_HOVER recovered!')
        else:
            val = self._param_value(self.hover_saved)
            print(f'{bcolors.ERROR}*** FAILED to restore MOT_THST_HOVER! '
                  f'Manually set it back to {val} ***{bcolors.ENDC}')
            self.restore_failed_value = val

    def set_hover_param(self, value: ParamValue):
        for _ in range(3):
            resp = self.call_service(self.param_set_client,
                                     ParamSet.Request(param_id='MOT_THST_HOVER',
                                                      value=value))
            if resp is not None and resp.success:
                return True
            self.sleep_abortable(1.0)
        return False

    @staticmethod
    def _param_value(v: ParamValue):
        return v.integer if v.integer != 0 else v.real

    # ---------------------------------------------------------- 解锁/上锁

    def start_testing(self):
        """解锁 + 油门至 MIN，进入采集状态。失败抛 FatalError。"""
        self.recorder.clear()
        # 先冻结去皮基准: 解锁等待期间电机已在 MIN 油门转动，重量不再是空载值
        with self.lock:
            self.testing = True
        try:
            if self.args.backend == 'apm':
                req = CommandBool.Request()
                req.value = True
                resp = self.call_service(self.arming_client, req)
                if resp is None or not resp.success:
                    raise FatalError('arming service rejected (check pre-arm checks)')
            else:
                self._arm_intent = True
                self.arm_pub.publish(Bool(data=True))
            if not self._wait_armed(True, timeout=15.0):
                raise FatalError('FC did not ARM '
                                 '(custom: is the BOXOFFBOARD switch held?)')
        except Exception:
            with self.lock:
                self.testing = False
            raise
        self.set_thrust(self.args.min_thrust, force=True)
        print(f'{bcolors.OKGREEN}START TESTING...{bcolors.ENDC}')

    def stop_testing(self):
        """油门至 MIN + 上锁。"""
        self.set_thrust(self.args.min_thrust, force=True)
        self._disarm_backend()
        print('STOP TESTING...')

    def _disarm_backend(self):
        """上锁并等待确认。期间控制流保持 (armed 字段)，让 arm=0 边沿随帧送达。"""
        with self.lock:
            self.disarming = True
        try:
            if self.args.backend == 'apm':
                req = CommandBool.Request()
                req.value = False
                self.call_service(self.arming_client, req, timeout_sec=5.0)
            else:
                self._arm_intent = False
                self.arm_pub.publish(Bool(data=False))
            self._wait_armed(False, timeout=max(DISARM_STREAM_TIME, 5.0))
        finally:
            with self.lock:
                self.testing = False
                self.disarming = False

    def _wait_armed(self, target: bool, timeout: float):
        reminded = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                return False
            with self.lock:
                if self.armed == target:
                    return True
            if (self.args.backend == 'custom' and target and not reminded
                    and time.monotonic() > deadline - timeout + 3.0):
                print(f'{bcolors.WARNING}waiting for FC to ARM -- '
                      f'pilot must hold BOXOFFBOARD{bcolors.ENDC}')
                reminded = True
            time.sleep(0.1)
        with self.lock:
            state_now = self.armed
        print(f'{bcolors.ERROR}timeout waiting for armed={target} '
              f'(current: {state_now}){bcolors.ENDC}')
        return False

    # ---------------------------------------------------------- kp

    def live_kp(self):
        with self.lock:
            kp = self.baro / 101325.0
            temp = self.temp - self.args.temp_offset
            kp *= 273.15 / (273.15 + temp)
        return kp

    def collect_kp(self):
        if self.args.kp is not None:
            self.kp_avg = self.args.kp
            print(f'{bcolors.OKBLUE}KP (user) = {self.kp_avg}{bcolors.ENDC}')
            return
        if not self.baro_event.wait(timeout=5.0):
            print(f'{bcolors.WARNING}no /fpv/static_pressure data, KP defaults to 1.0 '
                  f'(use --kp to override){bcolors.ENDC}')
            self.kp_avg = 1.0
            return
        print('Collecting KP...')
        val = 0.0
        for _ in range(10):
            if self.abort_event.is_set():
                break
            val += self.live_kp()
            time.sleep(0.5)
        self.kp_avg = val / 10.0
        print(f'{bcolors.OKBLUE}KP = {self.kp_avg}{bcolors.ENDC}')

    # ---------------------------------------------------------- 采集

    def collect_average(self, samples, period=0.1):
        """采 samples 个稳定重量/电压均值。返回 (weight, voltage, n) 或 None。"""
        self.quiet = True
        w_sum = v_sum = 0.0
        n = 0
        deadline = time.monotonic() + samples * period * 3.0 + COLLECT_TIMEOUT_MARGIN
        try:
            while n < samples and not self.abort_event.is_set():
                with self.lock:
                    sw = self.stable_weight
                    bv = self.battery_voltage
                if not math.isnan(sw):
                    w_sum += sw
                    v_sum += bv
                    n += 1
                    sys.stdout.write(f'\r  collecting {n}/{samples}...')
                    sys.stdout.flush()
                if time.monotonic() > deadline:
                    break
                time.sleep(period)
        finally:
            self.quiet = False
        print()
        if n == 0:
            return None
        return w_sum / n, v_sum / n, n

    def record_point(self):
        """采集并记录当前油门下的一个数据点。"""
        result = self.collect_average(self.args.samples_per_point)
        if result is None:
            print(f'{bcolors.WARNING}no stable scale data at this step, '
                  f'point skipped{bcolors.ENDC}')
            return
        weight, voltage, _ = result
        # 安全降油门后油门可能低于名义档位，按实际值记录
        throttle_pct = self.get_thrust()
        thrust_norm = throttle_pct / 100.0
        volt_max = self.args.lipo_cells * 4.2
        use_map = self.args.map_a != 0.0 or self.args.map_b != 0.0
        x_poly = normalize_throttle(thrust_norm, self.kp_avg, self.args.spin_k,
                                    voltage, volt_max,
                                    not self.args.no_voltage_comp and not use_map)
        with self.lock:
            tare = self.whole_weight
        force_n = (tare - weight) * self.args.gravity
        if use_map:
            # vbat 归一: 实测力换算到标准电压 V_ref = cells*3.7 口径
            force_n *= vbat_scale(self.args.map_a, self.args.map_b,
                                  self.args.lipo_cells * 3.7, voltage)
        ok = self.recorder.insert_normal(throttle_pct=throttle_pct,
                                         thrust_norm=thrust_norm,
                                         voltage=voltage, weight_kg=weight,
                                         force_n=force_n, kp=self.kp_avg,
                                         x_poly=x_poly)
        if ok:
            print(f'Collected Force Data: {tare - weight:.2f} kgF '
                  f'at {throttle_pct:.2f}% throttle, V={voltage:.2f}')
        else:
            print(f'{bcolors.WARNING}Data {throttle_pct:.2f}% Repeated!{bcolors.ENDC}')

    # ---------------------------------------------------------- 关停

    def shutdown_sequence(self):
        """安全关停 (executor 仍在 spin，顺序关键)。"""
        print('\nSTOP TESTING...')
        self.quiet = True
        try:
            self.set_thrust(self.args.min_thrust, force=True)
            self._disarm_backend()
            if self.args.backend == 'apm':
                self.restore_hover_param()
        finally:
            self.quiet = False


# ======================================================================
# 扫描流程
# ======================================================================

def thrust_grid(args):
    grid = []
    t = args.min_thrust
    while t < args.max_thrust - 1e-6:
        grid.append(round(t, 3))
        t += args.step
    grid.append(args.max_thrust)
    return grid


def run_sweep(node: ThrustTestNode, args):
    grid = thrust_grid(args)
    print(f'{bcolors.OKBLUE}Sweeping throttle: {grid} %{bcolors.ENDC}')
    for target in grid:
        if node.abort_event.is_set():
            break
        node.set_thrust(target)
        if not node.sleep_abortable(args.settle_time):
            break
        node.record_point()
        if node.abort_event.is_set():
            break


def run_vbat_drain(node: ThrustTestNode, args):
    """vbat 模式: 固定油门放电，每 10 s 记一组 (V, F)，直到电压低于下限。"""
    vmin = args.lipo_cells * 3.6333
    vmax = args.lipo_cells * 4.2
    print(f'{bcolors.WARNING}START TESTING AT {args.vbat_thrust}% IN 3 SECONDS...{bcolors.ENDC}')
    node.sleep_abortable(3.0)
    node.set_thrust(args.vbat_thrust)
    node.sleep_abortable(2.0)
    while not node.abort_event.is_set():
        with node.lock:
            voltage_now = node.battery_voltage
        if 0.0 < voltage_now <= vmin:
            print('battery reached minimum voltage, drain collection done')
            break
        result = node.collect_average(10, period=1.0)
        if result is None:
            print(f'{bcolors.WARNING}no stable scale data, sample skipped{bcolors.ENDC}')
            continue
        weight, voltage, _ = result
        with node.lock:
            tare = node.whole_weight
        force_n = (tare - weight) * args.gravity
        if node.recorder.insert_vbat(voltage=voltage, force_n=force_n,
                                     weight_kg=weight, vmin=vmin, vmax=vmax):
            print(f'Collected {voltage:.2f} V, {tare - weight:.2f} kgF')
        node.sleep_abortable(1.0)


def confirm_start(args, abort_event):
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print(f'{bcolors.WARNING}stdin is not a tty and --yes not given; '
              f'proceeding anyway{bcolors.ENDC}')
        return True
    print(f'{bcolors.OKGREEN}Press Enter to START the sweep (Ctrl+C to abort)...{bcolors.ENDC}',
          flush=True)
    while not abort_event.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            sys.stdin.readline()
            return True
    return False


# ======================================================================
# 结果输出
# ======================================================================

def write_outputs(node: ThrustTestNode, args, aborted):
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(args.output_dir, exist_ok=True)
    base = os.path.join(args.output_dir, f'thrust_curve_{stamp}')
    points = node.recorder.points

    # ---- CSV ----
    csv_path = base + '.csv'
    try:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'throttle_pct', 'thrust_norm', 'voltage',
                             'weight_kg', 'force_N', 'kp', 'x_poly'])
            for p in points:
                writer.writerow([f"{p['time']:.3f}", f"{p['throttle_pct']:.3f}",
                                 f"{p['thrust_norm']:.6f}", f"{p['voltage']:.3f}",
                                 f"{p['weight_kg']:.4f}", f"{p['force_n']:.3f}",
                                 f"{p['kp']:.5f}", f"{p['x_poly']:.6f}"])
        print(f'CSV written to {csv_path}')
    except OSError as e:
        print(f'{bcolors.ERROR}cannot write CSV: {e}{bcolors.ENDC}')
        csv_path = None

    # ---- 拟合 ----
    node_key = 'apm_bridge' if args.backend == 'apm' else 'custom_link_bridge'
    volt_max = args.lipo_cells * 4.2
    vref = args.lipo_cells * 3.7
    use_map = args.map_a != 0.0 or args.map_b != 0.0
    vbat = (args.map_a, args.map_b) if use_map else None
    notes = [f'fitted {datetime.datetime.now().isoformat(timespec="seconds")}, '
             f'backend={args.backend}, {len(points)} points, kp={node.kp_avg:.5f}, '
             f'gravity={args.gravity}',
             f'compensations divided out: kp, spin_k={args.spin_k}'
             + (f', voltage via vbat map to V_ref={vref:.1f} '
                f'(a={args.map_a:.6g}, b={args.map_b:.6g})' if use_map else
                (', voltage normalized to volt_max=' + format(volt_max, '.1f')
                 if not args.no_voltage_comp else ''))]
    fit = None
    vbat_fit = None
    if args.vbat:
        vbat_fit, err = fit_voltage_linear(points)
        if vbat_fit is None:
            print(f'{bcolors.ERROR}{err}{bcolors.ENDC}')
        else:
            print(f'{bcolors.OKGREEN}VBAT fitted curve coefficients (a, b): '
                  f'{vbat_fit}{bcolors.ENDC}')
            print(f'{bcolors.OKBLUE}sweep with: --map-a {vbat_fit[0]:.10g} '
                  f'--map-b {vbat_fit[1]:.10g}  (force normalized to '
                  f'V_ref={args.lipo_cells * 3.7:.1f}; paste the same values as '
                  f'motor_parameters.vbat_a/vbat_b){bcolors.ENDC}')
    else:
        fit, err = fit_motor_cubic(points, args.num_motors)
        if fit is None:
            print(f'{bcolors.ERROR}{err}{bcolors.ENDC}')
        else:
            print(f'{bcolors.OKGREEN}Fitted motor parameters '
                  f'(A, B, C, D): ({fit["A"]:.9g}, {fit["B"]:.9g}, '
                  f'{fit["C"]:.9g}, {fit["D"]:.9g}){bcolors.ENDC}')
            print(f'residual RMS = {fit["rms"]:.6f} throttle')
            if fit['deriv_min'] <= 0.0:
                print(f'{bcolors.WARNING}WARNING: fitted curve is not monotonic '
                      f'over the sampled range (min dP/df = {fit["deriv_min"]:.4g}){bcolors.ENDC}')
            if not -0.2 <= fit['D'] <= 0.3:
                print(f'{bcolors.WARNING}WARNING: D = {fit["D"]:.4g} outside the '
                      f'usual [-0.2, 0.3] band{bcolors.ENDC}')

    # ---- 结果 sidecar YAML ----
    yaml_path = base + '_result.yaml'
    try:
        with open(yaml_path, 'w') as f:
            f.write('# thrust test result\n')
            f.write(f'date: {datetime.datetime.now().isoformat(timespec="seconds")}\n')
            f.write(f'backend: {args.backend}\n')
            f.write(f'aborted: {aborted}\n')
            f.write(f'points: {len(points)}\n')
            f.write(f'kp: {node.kp_avg:.6f}\n')
            f.write(f'gravity: {args.gravity}\n')
            f.write(f'num_motors: {args.num_motors}\n')
            f.write(f'spin_k: {args.spin_k}\n')
            f.write(f'voltage_compensation: {not args.no_voltage_comp}\n')
            f.write(f'volt_max: {volt_max:.1f}\n')
            f.write(f'volt_ref: {vref:.1f}\n')
            if vbat:
                f.write(f'vbat_a: {vbat[0]!r}\n')
                f.write(f'vbat_b: {vbat[1]!r}\n')
            if getattr(node, 'hover_saved', None) is not None:
                f.write(f'mot_thst_hover_original: {ThrustTestNode._param_value(node.hover_saved)}\n')
                f.write(f'mot_thst_hover_override: {args.max_thrust / 100.0}\n')
                f.write(f'mot_thst_hover_restored: '
                        f'{not getattr(node, "restore_failed_value", None)}\n')
            if getattr(node, 'restore_failed_value', None):
                f.write(f'mot_thst_hover_restore_failed_value: '
                        f'{node.restore_failed_value}\n')
            if fit:
                f.write(f'fit_rms: {fit["rms"]:.6f}\n')
            if vbat_fit:
                f.write(f'vbat_a: {vbat_fit[0]!r}\n')
                f.write(f'vbat_b: {vbat_fit[1]!r}\n')
            f.write('\n# ---- paste into parameters/*.yaml ----\n')
            if fit:
                f.write(render_param_yaml(node_key, fit, args.num_motors,
                                          args.spin_k, vbat, notes) + '\n')
        print(f'result YAML written to {yaml_path}')
    except OSError as e:
        print(f'{bcolors.ERROR}cannot write result YAML: {e}{bcolors.ENDC}')

    # ---- 绘图 (惰性 import，失败不丢数据) ----
    if not args.no_plot:
        plot_results(points, fit, vbat_fit, args, node.kp_avg, base + '.png')
    return csv_path


def plot_results(points, fit, vbat_fit, args, kp, png_path):
    try:
        import matplotlib
        if not os.environ.get('DISPLAY'):
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f'{bcolors.WARNING}matplotlib unavailable ({e}), skipping plot '
              f'(data is in the CSV){bcolors.ENDC}')
        return
    try:
        plt.figure()
        if vbat_fit:
            voltages = [p['voltage'] for p in points]
            forces = [p['force_n'] for p in points]
            mean_f = sum(forces) / len(forces)
            rel = [mean_f / f for f in forces]
            plt.plot(voltages, rel, 'go', label='Samples')
            v_t = [v for v in voltages]
            plt.plot(v_t, [vbat_fit[0] * v + vbat_fit[1] for v in v_t], '-r',
                     label='Fitted a*V+b')
            plt.xlabel('pack voltage [V]')
            plt.ylabel('relative thrust')
        else:
            xs = [p['thrust_norm'] for p in points]
            ys = [p['force_n'] for p in points]
            plt.plot(xs, ys, 'go', label='Samples')
            if fit:
                k = args.spin_k
                f_grid = np.linspace(0.0, max(ys) / args.num_motors * 1.05, 200)
                thr = fit['A'] * f_grid ** 3 + fit['B'] * f_grid ** 2 + \
                    fit['C'] * f_grid + fit['D']
                # 参考电压 (V=volt_max) 下的命令油门: t = kp * spin(thr)
                t_model = ((1.0 - k) * thr + k * thr * thr) * kp
                mask = (t_model >= 0.0) & (t_model <= 1.0)
                plt.plot(t_model[mask], args.num_motors * f_grid[mask], '-r',
                         label='Fitted model')
            plt.xlabel('throttle [0..1]')
            plt.ylabel('thrust [N]')
        plt.legend()
        plt.grid(True)
        plt.savefig(png_path)
        print(f'plot written to {png_path}')
        if os.environ.get('DISPLAY'):
            plt.show()
    except Exception as e:
        print(f'{bcolors.WARNING}plotting failed ({e}), data is in the CSV{bcolors.ENDC}')


# ======================================================================
# main
# ======================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description='Automatic throttle-thrust curve calibration tool '
                    '(ported from kestrel_utils thrust_test_dt30x)')
    parser.add_argument('--backend', choices=['apm', 'custom'], required=True,
                        help='apm: apm_bridge_node + mavros; '
                             'custom: custom_link_bridge_node (Betaflight)')
    parser.add_argument('--vbat', action='store_true',
                        help='battery voltage compensation test '
                             '(fixed throttle drain)')
    parser.add_argument('--map-a', type=float, default=0.0, dest='map_a',
                        help='vbat fit coefficient a (--vbat output); normalizes '
                             'measured force to the V_ref=lipo_cells*3.7 curve; '
                             'paste as motor_parameters.vbat_a')
    parser.add_argument('--map-b', type=float, default=0.0, dest='map_b',
                        help='vbat fit coefficient b, paired with --map-a '
                             '(paste as motor_parameters.vbat_b)')
    parser.add_argument('--min-thrust', type=float, default=5.0, dest='min_thrust',
                        help='sweep start / idle throttle [%%]')
    parser.add_argument('--max-thrust', type=float, default=50.0, dest='max_thrust',
                        help='sweep end throttle [%%]; also the MOT_THST_HOVER '
                             'override (max/100)')
    parser.add_argument('--step', type=float, default=5.0,
                        help='throttle step between sweep points [%%]')
    parser.add_argument('--settle-time', type=float, default=3.0, dest='settle_time',
                        help='stabilization wait at each step [s]')
    parser.add_argument('--samples-per-point', type=int, default=100,
                        dest='samples_per_point',
                        help='scale samples averaged per point (10 Hz)')
    parser.add_argument('--yes', action='store_true',
                        help='skip the Enter confirmation before starting')
    parser.add_argument('--vbat-thrust', type=float, default=25.0, dest='vbat_thrust',
                        help='fixed throttle for --vbat drain test [%%]')
    parser.add_argument('--critical-weight', type=float, default=1.0,
                        dest='critical_weight',
                        help='abort if scale weight drops below this while '
                             'testing [kg]')
    parser.add_argument('--gravity', type=float, default=9.801)
    parser.add_argument('--lipo-cells', type=int, default=6, dest='lipo_cells',
                        help='LiPo cell count (defines volt_max and vbat bounds)')
    parser.add_argument('--num-motors', type=int, default=1, dest='num_motors',
                        help='motor count (per-motor force = total/n)')
    parser.add_argument('--spin-k', type=float, default=0.0, dest='spin_k',
                        help='MOT_THST_EXPO style spin compensation already '
                             'applied by the FC (divided out before fitting)')
    parser.add_argument('--kp', type=float, default=None,
                        help='air-density factor override (default: auto from the '
                             'bridge /fpv/static_pressure topic, 1.0 if absent)')
    parser.add_argument('--temp-offset', type=float, default=0.0, dest='temp_offset',
                        help='baro temperature offset used with --kp-temp [degC]')
    parser.add_argument('--no-voltage-comp', action='store_true', dest='no_voltage_comp',
                        help='do not normalize fitted coefficients to volt_max')
    parser.add_argument('--control-rate', type=float, default=50.0, dest='control_rate',
                        help='control command stream rate [Hz]')
    parser.add_argument('--guided-mode', type=str, default='GUIDED_NOGPS',
                        dest='guided_mode', help='apm backend flight mode')
    parser.add_argument('--output-dir', type=str, default='.', dest='output_dir')
    parser.add_argument('--no-plot', action='store_true', dest='no_plot',
                        help='never open a matplotlib window (still saves PNG '
                             'when possible)')
    return parser.parse_args()


def install_signal_handlers(abort_event):
    state = {'first': True}

    def handler(signum, frame):
        if state['first']:
            state['first'] = False
            print(f'\n{bcolors.WARNING}abort requested (signal {signum}), '
                  f'shutting down safely... (again = force exit){bcolors.ENDC}')
            abort_event.set()
        else:
            os._exit(1)

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


def main():
    args = parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ThrustTestNode(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_guarded():
        # executor 线程崩溃会让所有回调停摆而主线程无感知，兜底转为 abort
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception:
            import traceback
            traceback.print_exc()
            print(f'{bcolors.ERROR}executor thread died, aborting{bcolors.ENDC}')
            node.abort_event.set()

    spin_thread = threading.Thread(target=spin_guarded, daemon=True)
    spin_thread.start()
    install_signal_handlers(node.abort_event)

    exit_code = 0
    aborted = False
    try:
        if not node.state_event.wait(timeout=15.0):
            raise FatalError('no /mavros/state within 15 s -- '
                             'is a bridge node running?')
        if args.backend == 'apm':
            node.set_message_intervals()
            node.set_guided_mode()
            node.save_and_override_hover_param()
        else:
            print(f'{bcolors.OKBLUE}custom-link backend: the pilot must hold the '
                  f'BOXOFFBOARD switch for host arming{bcolors.ENDC}')
        if args.map_a != 0.0 or args.map_b != 0.0:
            print(f'{bcolors.OKBLUE}vbat map active: measured force normalized to '
                  f'V_ref={args.lipo_cells * 3.7:.1f}V (phi=(a*V+b)/(a*V_ref+b)), '
                  f'volt_max normalization skipped{bcolors.ENDC}')
        node.collect_kp()
        if not node.weight_event.wait(timeout=3.0):
            print(f'{bcolors.WARNING}no /current_weight data -- is '
                  f'scale_reader_node running? Points will be skipped.{bcolors.ENDC}')
        if not confirm_start(args, node.abort_event):
            aborted = True
        else:
            node.start_testing()
            if node.abort_event.is_set():
                aborted = True
            elif args.vbat:
                run_vbat_drain(node, args)
            else:
                run_sweep(node, args)
            node.stop_testing()
    except FatalError as e:
        print(f'{bcolors.ERROR}FATAL: {e}{bcolors.ENDC}')
        exit_code = 1
    finally:
        aborted = aborted or node.abort_event.is_set()
        node.shutdown_sequence()
        executor.remove_node(node)
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    write_outputs(node, args, aborted)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
