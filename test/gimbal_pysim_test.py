#!/usr/bin/env python3
"""XFrobot 云台协议仿真测试 (pty)。

在伪终端上运行 gimbal_selftest harness (链接 xfgimbal 驱动)，本脚本模拟云台固件:
解析 40B 控制包 (A9 5B ...)、立即回 26B 状态包 (B5 9A ...)，并周期注入
"B5 B5 9A" 垃圾字节验证驱动接收状态机的重新同步。

覆盖点:
  1. 发送频率 >= 50Hz (协议要求)
  2. 三轴均为 锁定模式 + 真实角速度控制
  3. 机型门控: 不存在的轴 op_value 恒 0
  4. C200T 默认参数下控制律与旧版实现逐字节一致 (金标回归, 含 float32 精度仿真)
  5. 一次性命令 (陀螺校准/指点平移) 恰好发一包, 命令码与 target_angle 正确
  6. 云台内置目标跟踪 (命令6) 开关与控制量
  7. go_zero 回中位变化触发 (按机型掩掉缺失轴)
  8. 载机惯导数据 uav.valid 门控 (无数据/过期 -> 0)
  9. 接收路径状态回调 (gbc_stat 翻转可见 -> 重同步正常)

用法: /usr/bin/python3 test/gimbal_pysim_test.py [harness_path]
"""

import math
import os
import pty
import re
import select
import struct
import subprocess
import sys
import tempfile
import time

DURATION = 10.8
UAV_ANGLE = (100, -200, 300)  # 0.01deg [R,P,Y]
UAV_ACCEL = (10, -20, 980)    # 0.01m/s2 NEU

# 固定回复内容 (0.01deg): cam_angle=[滚转,俯仰,偏航], mtr_angle=[俯仰,滚转,偏航]
# CAM0_HIGH: [8.9, 9.8) 窗口内 cam_angle[0] (滚转) 从 8deg 变为 80deg,
# 用于触发/退出 C-20S 的大滚转速度保持逻辑
CAM_ANGLE = (800, -1200, 300)
CAM0_HIGH = 8000
MTR_ANGLE = (-2000, 1000, -600)
CAM_RATE = (10, 20, 30)
FW_VER = 36


def cam0_at(t):
    """仿真时刻 t 对应的云台 IMU 滚转角 (0.01deg)。"""
    return CAM0_HIGH if 8.9 <= t < 9.8 else CAM_ANGLE[0]


# ----------------------------------------------------------------------
# 协议工具
# ----------------------------------------------------------------------

CRC_TA = (
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
)


def crc16(data):
    """与驱动 CalculateCrc16 相同的半字节查表 CCITT-16。"""
    crc = 0
    for b in data:
        da = crc >> 12
        crc = ((crc << 4) & 0xFFFF) ^ CRC_TA[da ^ (b >> 4)]
        da = crc >> 12
        crc = ((crc << 4) & 0xFFFF) ^ CRC_TA[da ^ (b & 0x0F)]
    return crc


def f32(x):
    """float32 舍入 (仿真 C++ float 运算精度)。"""
    return struct.unpack('<f', struct.pack('<f', x))[0]


def trunc_i16(x):
    """仿真 C++ static_cast<int16_t>(float) —— 向零截断。"""
    return int(x)  # python int() 对 float 即向零截断


# ----------------------------------------------------------------------
# 40B 控制包解析
# ----------------------------------------------------------------------

class CtrlPacket:
    __slots__ = ('t', 'trig', 'cmd', 'fl_sens', 'wk_mode', 'op_type', 'op_value',
                 'go_zero', 'uav_valid', 'uav_angle', 'uav_accel', 'cam_word', 'target')


def parse_ctrl(raw, t):
    assert crc16(raw[:38]) == ((raw[38] << 8) | raw[39]), 'CRC 校验失败'
    p = CtrlPacket()
    p.t = t
    p.trig = raw[2] & 0x07
    p.cmd = (raw[2] >> 3) & 0x1F
    p.fl_sens = raw[3] >> 3  # 仅记录, 不断言
    p.wk_mode = [0, 0, 0]
    p.op_type = [0, 0, 0]
    p.op_value = [0, 0, 0]
    p.go_zero = 0
    for i in range(3):
        b = raw[4 + 3 * i]
        p.wk_mode[i] = (b >> 4) & 0x03
        p.op_type[i] = (b >> 6) & 0x03
        p.go_zero |= ((b >> 3) & 1) << i
        p.op_value[i] = struct.unpack_from('<h', raw, 5 + 3 * i)[0]
    p.uav_valid = (raw[13] >> 7) & 1
    p.uav_angle = struct.unpack_from('<3h', raw, 14)
    p.uav_accel = struct.unpack_from('<3h', raw, 20)
    p.cam_word = struct.unpack_from('<I', raw, 26)[0]
    p.target = struct.unpack_from('<2f', raw, 30)
    return p


def build_reply(gbc_stat=4, tca=1, inv=0, hw_err=0, cmd_value=4, cmd_stat=1, cam0=None):
    cam_angle = list(CAM_ANGLE)
    if cam0 is not None:
        cam_angle[0] = cam0
    body = struct.pack(
        '<2sBBBB3h3h3h',
        b'\xB5\x9A', FW_VER, hw_err,
        (inv & 1) | ((gbc_stat & 0x07) << 1) | ((tca & 1) << 4),
        (cmd_stat & 0x07) | ((cmd_value & 0x1F) << 3),
        *CAM_RATE, *cam_angle, *MTR_ANGLE)
    crc = crc16(body)
    return body + struct.pack('>H', crc)


# ----------------------------------------------------------------------
# 金标模型: 旧版 (git HEAD) 三轴控制律, float32 精度仿真
# ----------------------------------------------------------------------

def golden_op_values(cam0=CAM_ANGLE[0]):
    cam, mtr = (cam0, CAM_ANGLE[1], CAM_ANGLE[2]), MTR_ANGLE

    def scale12(v):
        return f32(f32(f32(v) * f32(0.01)) * f32(12.0))

    roll = max(-150.0, min(150.0, scale12(-cam[0])))
    pitch = max(-150.0, min(150.0, scale12(-cam[1])))

    mtr_roll = f32(mtr[1] * 0.01)
    mtr_pitch = f32(mtr[0] * 0.01)
    enc = f32(mtr[2] * 0.01)

    maxed = False
    if abs(mtr_roll) >= 50.0:
        roll = 0.0
        maxed = True
    if abs(mtr[0] - cam[1]) < int(40 * 100) and (mtr_pitch <= -135.0 or mtr_pitch >= 40.0):
        pitch = 0.0
        maxed = True

    r_ratio = f32(1.0 - f32(abs(mtr_roll) / f32(50.0)))
    limit = f32(-135.0) if math.copysign(1.0, mtr_pitch) < 0 else f32(40.0)
    p_ratio = f32(1.0 - f32(mtr_pitch / limit))
    scale = min(r_ratio, p_ratio)
    if scale < 0.01:
        scale = 0.01
    elif scale > 1.0:
        scale = 1.0

    if maxed:
        yaw = f32(150.0) * 0.01 if math.copysign(1.0, enc) < 0 else -f32(150.0) * 0.01
    else:
        yaw = f32(f32(f32(-enc) * f32(10.0)) * scale)
        if abs(yaw) < 0.1:
            yaw = 0.0
        else:
            yaw = max(-150.0, min(150.0, yaw))

    # gbc 索引: 0-滚转, 1-俯仰, 2-偏航
    return (trunc_i16(f32(roll * f32(10.0))),
            trunc_i16(f32(pitch * f32(10.0))),
            trunc_i16(f32(yaw * f32(10.0))))


# ----------------------------------------------------------------------
# 仿真主循环
# ----------------------------------------------------------------------

def find_harness(override):
    candidates = [override] if override else []
    if os.environ.get('GIMBAL_SELFTEST'):
        candidates.append(os.environ['GIMBAL_SELFTEST'])
    # 沿符号链接两侧的路径分别向上找工作区 (src/apm_bridge 可能是符号链接)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    real_dir = os.path.dirname(os.path.realpath(__file__))
    for base in (script_dir, real_dir):
        ws = os.path.dirname(os.path.dirname(base))  # <ws>/src/apm_bridge/test -> <ws>
        candidates += [
            os.path.join(ws, 'build', 'apm_bridge', 'gimbal_selftest'),
            os.path.join(ws, 'install', 'apm_bridge', 'lib', 'apm_bridge', 'gimbal_selftest'),
        ]
    candidates.append(os.path.expanduser('~/workspace/fpv_ws/build/apm_bridge/gimbal_selftest'))
    for c in candidates:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    raise SystemExit('找不到 gimbal_selftest, 先 colcon build; 或指定路径/GIMBAL_SELFTEST 环境变量: '
                     + str(candidates))


def run_case(model, harness):
    print(f'===== {model} =====')
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    err_file = tempfile.NamedTemporaryFile(mode='w+', suffix='.log', delete=False)
    proc = subprocess.Popen(
        [harness, slave_name, model, str(DURATION)],
        stdout=subprocess.DEVNULL, stderr=err_file)
    # 注意: 本进程须全程持有 slave fd。若提前关闭, 在 harness 打开 slave 之前
    # master 会立刻 EOF, 仿真循环误判退出并关闭 master, 导致 harness 打开时
    # pts 条目已销毁 (ENOENT)。

    packets = []
    buf = b''
    reply_count = 0
    t0 = time.monotonic()
    try:
        while True:
            r, _, _ = select.select([master], [], [], 0.02)
            if r:
                try:
                    chunk = os.read(master, 4096)
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
            now = time.monotonic() - t0
            # 提取完整包
            while True:
                idx = buf.find(b'\xA9\x5B')
                if idx < 0:
                    buf = buf[-1:] if buf.endswith(b'\xA9') else b''
                    break
                if len(buf) - idx < 40:
                    buf = buf[idx:]
                    break
                raw = buf[idx:idx + 40]
                try:
                    packets.append(parse_ctrl(raw, now))
                except AssertionError:
                    pass  # CRC 坏包 (串扰), 跳过
                buf = buf[idx + 40:]
                # 应答 (应答式通信): 每 10 包注入一次 B5 B5 9A 垃圾测重同步
                reply_count += 1
                junk = b'\xB5\xB5\x9A' + bytes(range(20)) if reply_count % 10 == 0 else b''
                gbc_stat = 1 if 2.3 <= now < 2.8 else 4
                os.write(master, junk + build_reply(gbc_stat=gbc_stat, cam0=cam0_at(now)))
            if now > DURATION + 1.5 or proc.poll() is not None:
                # 等待 dtor 的 STOP 包
                if proc.poll() is not None and now > DURATION + 0.3:
                    break
    finally:
        proc.wait(timeout=5)
        for fd in (master, slave):
            try:
                os.close(fd)
            except OSError:
                pass
        err_file.close()
    with open(err_file.name) as f:
        stderr = f.read()
    os.unlink(err_file.name)

    assert proc.returncode == 0, f'harness 退出码 {proc.returncode}\n{stderr}'
    assert packets, '未收到任何控制包'

    # 机型轴存在性: gbc 索引 0-滚转, 1-俯仰, 2-偏航
    axes = {'C20S': (False, True, False),
            'C40D': (True, True, False),
            'C200T': (True, True, True)}[model]
    span = packets[-1].t - packets[0].t
    rate = len(packets) / span if span > 0 else 0.0

    def in_window(lo, hi, pred=lambda p: True):
        return [p for p in packets if lo <= p.t <= hi and pred(p)]

    # 1. 频率与首包 (START)
    assert rate >= 50.0, f'发送频率不足: {rate:.1f}Hz'
    assert packets[0].cmd == 2, f'首包应为启动命令(2), 实为 {packets[0].cmd}'

    manual = [p for p in packets if p.cmd == 4]

    # 2. 模式: 全部锁定; 手动/跟踪包的相机字段已填充
    #    (start_stop 发出的启动/停止包 gbc/cam 为全零, 不在断言范围)
    #    op_type: C-40D/C-200T 三轴恒真实角速度; C-20S 俯仰轴角度/速度切换 (见 #8)
    for p in packets:
        if p.cmd in (2, 3):
            continue
        assert p.wk_mode == [1, 1, 1], f't={p.t:.2f} wk_mode={p.wk_mode}'
        if model != 'C20S':
            assert p.op_type == [2, 2, 2], f't={p.t:.2f} op_type={p.op_type}'
        else:
            assert p.op_type[0] == 2 and p.op_type[2] == 2, \
                f't={p.t:.2f} C20S 滚转/偏航应为真实角速度: {p.op_type}'
    for p in manual + [q for q in packets if q.cmd in (5, 6)]:
        assert p.cam_word & 0x7F == 30, 'vert_fov1x 应为 30'
        assert (p.cam_word >> 7) & 0xFFFFFF == 1000, 'zoom_value 应为 1000'

    # 3. 机型门控: 缺失轴 op_value 恒 0; 存在轴有非零输出
    assert manual, '无手动控制包'
    for p in manual:
        for i, present in enumerate(axes):
            if not present:
                assert p.op_value[i] == 0, \
                    f't={p.t:.2f} 缺失轴 gbc[{i}] op_value={p.op_value[i]} 应为 0'
    for i, present in enumerate(axes):
        if present:
            assert any(abs(p.op_value[i]) > 50 for p in manual), \
                f'存在轴 gbc[{i}] 始终无输出'

    # 4. 金标回归 (三轴默认参数 == 旧版控制律; 大滚转窗口边界因时钟偏移设保护带)
    if model == 'C200T':
        for p in manual:
            if 8.8 <= p.t <= 10.0:
                continue
            golden = golden_op_values(cam0_at(p.t))
            for i in range(3):
                assert abs(p.op_value[i] - golden[i]) <= 1, \
                    f't={p.t:.2f} gbc[{i}] op_value={p.op_value[i]} 期望 {golden[i]}'

    # 5. 一次性命令
    gyro = in_window(3.9, 4.5, lambda p: p.cmd == 1)
    assert len(gyro) == 1, f'陀螺校准应恰好一包, 实得 {len(gyro)}'
    shift = in_window(4.9, 5.5, lambda p: p.cmd == 5)
    assert len(shift) == 1, f'指点平移应恰好一包, 实得 {len(shift)}'
    assert all(v == 0 for v in shift[0].op_value), '命令5 期间速度应为 0'
    assert abs(shift[0].target[0] - 2.5) < 1e-4 and abs(shift[0].target[1] + 1.5) < 1e-4, \
        f'命令5 target_angle 异常: {shift[0].target}'

    # 6. 目标跟踪开关 (窗口收缩 0.6s 容忍 harness 启动延迟)
    track = in_window(6.6, 7.6)
    assert track, '跟踪窗口无包'
    assert all(p.cmd == 6 for p in track), '跟踪窗口命令码应为 6'
    assert all(abs(p.target[0] - 3.0) < 1e-4 and abs(p.target[1] - 2.0) < 1e-4 for p in track), \
        '命令6 target_angle 异常'
    assert all(all(v == 0 for v in p.op_value) for p in track), '命令6 期间速度应为 0'
    after = in_window(8.7, DURATION)
    assert after and all(p.cmd == 4 for p in after), '跟踪关闭后应回手动控制'
    assert any(abs(p.op_value[i]) > 50 for p in after for i in range(3) if axes[i]), \
        '跟踪关闭后应恢复控制输出'

    # 7. go_zero: 3.0s 翻转后持续为 1 (按机型掩缺失轴)
    gz_mask = sum(1 << i for i, present in enumerate(axes) if present)
    for p in in_window(3.6, DURATION):
        assert p.go_zero == gz_mask, f't={p.t:.2f} go_zero={p.go_zero:03b} 期望 {gz_mask:03b}'
    for p in in_window(1.5, 2.9):
        assert p.go_zero == 0, f't={p.t:.2f} go_zero 提前翻转'

    # 8. C-20S 单轴逻辑: 常态锁定+角度控制(期望角 -5deg → op_value=-500);
    #    [8.9,9.8) 仿真滚转 80deg > 75 → 锁定+真实角速度 op=0 保持;
    #    9.8 后滚转回落 → 恢复角度控制。
    #    C-40D 同时段不受影响 (俯仰仍真实角速度, 大滚转逻辑仅 C-20S)。
    if model == 'C20S':
        for p in in_window(1.0, 2.0) + in_window(10.1, DURATION):
            assert p.op_type[1] == 0, f't={p.t:.2f} 常态俯仰应为角度控制: {p.op_type}'
            assert p.op_value[1] == -500, f't={p.t:.2f} 期望角 -5deg 编码异常: {p.op_value[1]}'
        for p in in_window(9.4, 9.6):
            assert p.op_type[1] == 2, f't={p.t:.2f} 大滚转下俯仰应切速度保持: {p.op_type}'
            assert p.op_value[1] == 0, f't={p.t:.2f} 大滚转下 op_value 应为 0: {p.op_value[1]}'
    if model == 'C40D':
        for p in in_window(9.4, 9.6):
            assert p.op_type[1] == 2, f't={p.t:.2f} C40D 俯仰应保持真实角速度: {p.op_type}'

    # 8. uav 数据门控
    assert all(p.uav_valid == 0 for p in in_window(0.0, 1.9)), '未喂载机数据前 valid 应为 0'
    fed = in_window(2.2, 6.8, lambda p: p.uav_valid == 1)
    assert fed, '喂载机数据后未见 valid=1'
    assert all(p.uav_angle == UAV_ANGLE and p.uav_accel == UAV_ACCEL for p in fed[:5]), \
        '载机姿态角/加速度数值异常'
    assert in_window(7.25, 7.65, lambda p: p.uav_valid == 0), \
        '载机数据停喂后应因过期回 valid=0'

    # 9. 接收路径 (状态回调 => 含 B5 B5 9A 注入下的重同步)
    assert '[status]' in stderr, f'未收到状态回调\n{stderr}'
    assert re.search(r'\bstat=1\b', stderr), f'gbc_stat 翻转未被观测 (重同步失败?)\n{stderr}'
    assert 'selftest done' in stderr

    print(f'  rate={rate:.0f}Hz packets={len(packets)} manual={len(manual)} '
          f'golden={golden_op_values() if model == "C200T" else "n/a"} -> PASS')


def main():
    harness = find_harness(sys.argv[1] if len(sys.argv) > 1 else None)
    print(f'harness: {harness}')
    for model in ('C200T', 'C40D', 'C20S'):
        run_case(model, harness)
    print('ALL PASS')


def test_gimbal_pysim():
    harness = find_harness(None)
    for model in ('C200T', 'C40D', 'C20S'):
        run_case(model, harness)


if __name__ == '__main__':
    main()
