#!/usr/bin/env python3
"""C20S 回中后 AIM 诊断探针: 完整复刻节点 home_on_connect + AIM 速率律,
逐包打印厂商域反馈与驱动计算的控制量, 定位回中后 AIM 不动的原因。

序列:
  0-1s    START, LOCK+零速
  1-3s    FPV+角度0 fl_sens=15   (驱动回中)
  3-13s   LOCK+真实角速度 = clamp(8*(80 - cam_pitch), ±600)°/s  (驱动 AIM 律)

关键观察量: cam_angle[1] (厂商姿态俯仰, 抬头正) 回中后读数; AIM 律算出的 op;
实际运动。若回中后 cam 读数 ≈80° 而 mtr≈0 => 云台姿态参考被 FPV 重置, AIM 误差
恒零 => 冻结根因。
用法: /usr/bin/python3 probe_aim_after_home.py [port]   (默认 /dev/ttyUSB1)
"""
import struct, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"

CRC_TA = [0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
          0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef]

def crc16(data):
    crc = 0
    for b in data:
        da = crc >> 12; crc = ((crc << 4) & 0xFFFF) ^ CRC_TA[da ^ (b >> 4)]
        da = crc >> 12; crc = ((crc << 4) & 0xFFFF) ^ CRC_TA[da ^ (b & 0x0F)]
    return crc

def build_packet(trig, cmd, axes, fl_sens=0, go_mask=0):
    p = bytearray(40)
    p[0], p[1] = 0xA9, 0x5B
    p[2] = (cmd << 3) | (trig & 7)
    p[3] = (fl_sens & 0x1F) << 3
    for i in range(3):
        off = 4 + 3 * i
        wk, op, val = axes[i]
        p[off] = (wk << 4) | (op << 6) | (1 if go_mask & (1 << i) else 0)
        struct.pack_into("<h", p, off + 1, val)
    p[13] = 0
    struct.pack_into("<I", p, 26, (30 & 0x7F) | ((1000 << 7) & 0x7FFFFFFF))
    c = crc16(bytes(p[:38]))
    p[38], p[39] = (c >> 8) & 0xFF, c & 0xFF
    return bytes(p)

def parse(pkt):
    if len(pkt) != 26 or pkt[0] != 0xB5 or pkt[1] != 0x9A:
        return None
    cam_angle = struct.unpack_from("<3h", pkt, 12)
    mtr_angle = struct.unpack_from("<3h", pkt, 18)
    cam_rate = struct.unpack_from("<3h", pkt, 6)
    return dict(cam=cam_angle, mtr=mtr_angle, rate=cam_rate)

LOCK, FPV = 1, 2
ANGLE, RSPEED = 0, 2
IDLE = [(LOCK, RSPEED, 0)] * 3

def main():
    ser = serial.Serial(PORT, 115200, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, timeout=0.004, write_timeout=0.5)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    t0 = time.monotonic()
    trig = 0
    buf = bytearray()
    started = False
    go_mask = 0
    next_print = 0.0
    last_op = 0
    while True:
        t = time.monotonic() - t0
        if t > 13.0:
            ser.write(build_packet(trig, 3, IDLE))
            break
        if not started:
            ser.write(build_packet(trig, 2, IDLE))
            started = True
        pkt = None
        # 先读上一拍反馈再决定本拍控制量 (驱动同款: 用最新反馈算 op)
        # 阶段划分
        if t < 1.0:
            axes, fl = IDLE, 0
        elif t < 3.0:
            axes, fl = [(FPV, ANGLE, 0)] * 3, 15
            if go_mask == 0 and t >= 1.0:
                go_mask = 0x2  # 回中边沿脉冲 (100ms) — 复刻驱动
            elif go_mask and t >= 1.1:
                go_mask = 0
        else:
            axes, fl = None, 0  # 由反馈计算
        pkt_data = None
        while True:
            b = ser.read(1)
            if not b:
                break
            buf.extend(b)
            if len(buf) == 2 and buf[0] != 0xB5:
                buf = bytearray(); continue
            if len(buf) == 2 and buf[1] != 0x9A:
                buf = bytearray([buf[1]]) if buf[1] == 0xB5 else bytearray(); continue
            if len(buf) >= 26:
                if crc16(bytes(buf[:24])) == ((buf[24] << 8) | buf[25]):
                    pkt_data = parse(bytes(buf[:26]))
                buf = bytearray()
                break
        if pkt_data is not None:
            pkt = pkt_data
        if axes is None and pkt is not None:
            cam_pitch = pkt['cam'][1] * 0.01
            joint = -pkt['mtr'][0] * 0.01
            speed = 8.0 * (80.0 - cam_pitch)  # 驱动 AIM 律: 目标 90+pitch_aim(-10)
            if joint <= -40 or joint >= 110:  # 机械限位门 (YAML travel -40/110)
                speed = 0.0
            speed = max(-600.0, min(600.0, speed))
            last_op = int(round(speed * 10.0))
            axes = [(LOCK, RSPEED, 0), (LOCK, RSPEED, last_op), (LOCK, RSPEED, 0)]
        elif axes is None:
            axes = IDLE
        ser.write(build_packet(trig, 4, axes, fl, go_mask))
        trig = (trig + 1) & 7
        if pkt and t >= next_print:
            next_print = t + 0.2
            phase = "BASE" if t < 1 else ("HOME" if t < 3 else "AIM ")
            print(f"t={t:5.2f} {phase} op={last_op:+6d}  cam_pitch={pkt['cam'][1]*0.01:+7.2f} "
                  f"mtr={pkt['mtr'][0]*0.01:+7.2f} rate={pkt['rate'][0]*0.1:+7.1f}dps")
        time.sleep(0.01)

if __name__ == "__main__":
    main()
