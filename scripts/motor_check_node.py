#!/usr/bin/python3
"""custom_link 真机联调检查: 解锁 → 恒定油门保持 N 秒 → 上锁。

thrust_test_node 的轻量版冒烟测试，不需要台秤/气压温度，用于上电后快速
验证 custom_link_bridge ↔ FC 链路、arm/disarm 流程与 raw 油门直通
(协议与 thrust_test_node.py 完全一致):

  油门走 /fpv/control_command_raw，collective_thrust 原样透传 FC，
  不经 motor_parameters 模型换算 (即本工具检查的是链路与电机，
  不是推力模型; 要验模型请跑台架标定);
  解锁走 /fpv/arm (2 Hz level 重发)，最终由飞手侧 BOXOFFBOARD 开关把关;
  上锁请求后控制流继续保持 DISARM_STREAM_TIME 秒，让 arm=0 边沿随
  0x20 帧送达 FC。

安全: 保持期间 FC 意外掉锁立即中止; Ctrl+C 随时安全关停 (油门归零 +
上锁)。真机检查请先卸桨或将机体可靠固定!

用法 (需系统 python，conda 环境请先 deactivate):
  ros2 run apm_bridge motor_check_node --thrust 10 --duration 3
"""

import argparse
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

from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from quadrotor_msgs.msg import ControlCommand
from std_msgs.msg import Bool

# 上锁后控制流需继续保持，arm=0 边沿随 0x20 帧送达 FC (同 thrust_test_node)
DISARM_STREAM_TIME = 2.0


class bcolors:
    ERROR = '\033[91m'
    WARNING = '\033[93m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    ENDC = '\033[0m'


class FatalError(Exception):
    """启动阶段不可恢复的错误 (安全关停后退出)。"""


class MotorCheckNode(Node):

    def __init__(self, args):
        super().__init__('motor_check_node')
        self.args = args
        self.lock = threading.RLock()
        self.abort_event = threading.Event()
        self.state_event = threading.Event()

        # ---- 共享状态 (lock 保护) ----
        self.thrust_pct = 0.0
        self.testing = False
        self.disarming = False
        self.armed = False
        self.mode = ''
        self.battery_voltage = 0.0
        self._arm_intent = False   # /fpv/arm 期望电平
        self.quiet = False         # 倒计时期间抑制 1 Hz 状态行

        # ---- 发布 ----
        qos_be5 = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_be10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(ControlCommand,
                                             '/fpv/control_command_raw', qos_be5)
        self.arm_pub = self.create_publisher(Bool, '/fpv/arm', qos_be10)

        # ---- 订阅 ----
        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(BatteryState, '/mavros/battery',
                                 self.on_battery, qos_be10)

        # ---- 定时器 ----
        self.control_timer = self.create_timer(1.0 / args.rate,
                                               self.control_timer_cb)
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
            print(f'\n{bcolors.ERROR}*** FC DISARMED during check! Aborting. ***{bcolors.ENDC}')
            self.abort_event.set()

    def on_battery(self, msg: BatteryState):
        voltage = 0.0
        if len(msg.cell_voltage) > 0:
            voltage = float(msg.cell_voltage[0])  # 本项目约定: cell_voltage[0] = 整包
        elif not math.isnan(msg.voltage):
            voltage = float(msg.voltage)
        with self.lock:
            self.battery_voltage = voltage

    # ---------------------------------------------------------- 定时器回调

    def control_timer_cb(self):
        """控制流: BODY_RATES 模式 + 零角速度 + 直通油门。"""
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.control_mode = ControlCommand.BODY_RATES
        with self.lock:
            cmd.armed = self.testing
            cmd.collective_thrust = min(max(self.thrust_pct / 100.0, 0.0), 1.0)
        # bodyrates 保持 0
        self.cmd_pub.publish(cmd)

    def arm_keepalive_cb(self):
        """arm 请求 2 Hz 重发 (level 语义)。"""
        with self.lock:
            armed_intent = self.testing or self._arm_intent
        self.arm_pub.publish(Bool(data=armed_intent))

    def status_timer_cb(self):
        if self.quiet:
            return
        with self.lock:
            thrust = self.thrust_pct
            weight_v = self.battery_voltage
            mode = self.mode
            armed = self.armed
        voltage_s = f'{weight_v:.2f}' if weight_v > 0.0 else '--'
        print(f'\r[t={thrust:5.1f}%] [V={voltage_s}] '
              f'[mode={mode or "--"}{" armed" if armed else ""}]   ',
              end='', flush=True)

    # ---------------------------------------------------------- 油门控制

    def set_thrust(self, value_pct):
        with self.lock:
            self.thrust_pct = min(max(value_pct, 0.0), 100.0)

    def sleep_abortable(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                return False
            time.sleep(0.05)
        return not self.abort_event.is_set()

    # ---------------------------------------------------------- 解锁/上锁

    def start_testing(self):
        """请求解锁并等待 FC 确认。失败抛 FatalError。"""
        with self.lock:
            self.testing = True
        try:
            self._arm_intent = True
            self.arm_pub.publish(Bool(data=True))
            if not self._wait_armed(True, timeout=15.0):
                raise FatalError('FC did not ARM '
                                 '(custom: is the BOXOFFBOARD switch held?)')
        except Exception:
            with self.lock:
                self.testing = False
            raise
        print(f'{bcolors.OKGREEN}ARMED, spinning up...{bcolors.ENDC}')

    def stop_testing(self):
        """油门归零 + 上锁。"""
        self.set_thrust(0.0)
        self._disarm_backend()
        print('STOP TESTING...')

    def _disarm_backend(self):
        """上锁并等待确认。期间控制流保持 (armed 字段)，让 arm=0 边沿随帧送达。"""
        with self.lock:
            self.disarming = True
        try:
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
            if target and not reminded and time.monotonic() > deadline - timeout + 3.0:
                print(f'{bcolors.WARNING}waiting for FC to ARM -- '
                      f'pilot must hold BOXOFFBOARD{bcolors.ENDC}')
                reminded = True
            time.sleep(0.1)
        with self.lock:
            state_now = self.armed
        print(f'{bcolors.ERROR}timeout waiting for armed={target} '
              f'(current: {state_now}){bcolors.ENDC}')
        return False

    # ---------------------------------------------------------- 保持

    def hold_thrust(self):
        """恒定油门保持 --duration 秒 (0.1 s 粒度倒计时)。"""
        self.quiet = True
        self.set_thrust(self.args.thrust)
        try:
            deadline = time.monotonic() + self.args.duration
            while time.monotonic() < deadline:
                if self.abort_event.is_set():
                    return False
                remain = max(deadline - time.monotonic(), 0.0)
                sys.stdout.write(f'\r  holding {self.args.thrust:.1f}% thrust, '
                                 f'{remain:4.1f}s remaining   ')
                sys.stdout.flush()
                time.sleep(0.1)
            return not self.abort_event.is_set()
        finally:
            self.quiet = False
            print()

    # ---------------------------------------------------------- 关停

    def shutdown_sequence(self):
        """安全关停 (executor 仍在 spin，顺序关键)。"""
        print('\nSTOP TESTING...')
        self.quiet = True
        try:
            self.set_thrust(0.0)
            self._disarm_backend()
        finally:
            self.quiet = False


# ======================================================================
# main
# ======================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description='Custom-link real-drone smoke check: '
                    'arm, hold throttle, disarm')
    parser.add_argument('--thrust', type=float, default=10.0,
                        help='throttle to hold during the check [%%, raw '
                             'passthrough, no motor model]')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='hold duration [s]')
    parser.add_argument('--rate', type=float, default=50.0,
                        help='control command stream rate [Hz]')
    parser.add_argument('--yes', action='store_true',
                        help='skip the Enter confirmation before starting')
    return parser.parse_args()


def confirm_start(args, abort_event):
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print(f'{bcolors.WARNING}stdin is not a tty and --yes not given; '
              f'proceeding anyway{bcolors.ENDC}')
        return True
    print(f'{bcolors.OKGREEN}Press Enter to ARM and hold {args.thrust:.1f}% for '
          f'{args.duration:.1f}s (Ctrl+C to abort)...{bcolors.ENDC}', flush=True)
    while not abort_event.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            sys.stdin.readline()
            return True
    return False


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
    node = MotorCheckNode(args)
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
    try:
        if not node.state_event.wait(timeout=15.0):
            raise FatalError('no /mavros/state within 15 s -- '
                             'is custom_link_bridge running?')
        print(f'{bcolors.OKBLUE}custom-link backend: the pilot must hold the '
              f'BOXOFFBOARD switch for host arming{bcolors.ENDC}')
        if not confirm_start(args, node.abort_event):
            node.abort_event.set()
        else:
            node.start_testing()
            if node.abort_event.is_set():
                pass  # 掉锁中止，走安全关停
            else:
                node.hold_thrust()
            node.stop_testing()
    except FatalError as e:
        print(f'{bcolors.ERROR}FATAL: {e}{bcolors.ENDC}')
        exit_code = 1
    finally:
        node.shutdown_sequence()
        executor.remove_node(node)
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    if exit_code == 0 and node.abort_event.is_set():
        exit_code = 1
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
