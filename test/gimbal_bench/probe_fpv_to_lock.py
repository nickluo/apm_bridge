#!/usr/bin/env python3
"""C20S FPV->LOCK 转换探针: 复刻节点启动回中序列, 判定固件是否在
FPV+角度(fl_sens=15) 之后不再受理 LOCK+真实角速度 命令。

阶段:
  0-1s   START 后 LOCK+零速 (基线)
  1-4s   FPV+角度 0, fl_sens=15 (驱动 home_on_connect 的回中命令)
  4-7s   LOCK+真实角速度 +30°/s (驱动 AIM 速率律等价命令, 抬头正)
  7-8s   LOCK+零速
  8-10s  LOCK+角度 0 (旧探针 E->F 已验证的路径, 兜底对照)

每 0.2s 打印 mtr/cam/rate。若阶段 3 相机不动而阶段 5 动 => 固件在
FPV+fl_sens15 后拒绝 LOCK 速率域; 若两者都动 => 驱动侧问题。
用法: /usr/bin/python3 probe_fpv_to_lock.py [port]   (默认 /dev/ttyUSB1)
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

def build_packet(trig, cmd, axes, fl_sens=0):
    p = bytearray(40)
    p[0], p[1] = 0xA9, 0x5B
    p[2] = (cmd << 3) | (trig & 7)
    p[3] = (fl_sens & 0x1F) << 3
    off = 4
    for wk, op, val in axes:
        p[off] = (wk << 4) | (op << 6)
        struct.pack_into("<h", p, off + 1, val)
        off += 3
    p[13] = 0
    struct.pack_into("<I", p, 26, (30 & 0x7F) | ((1000 << 7) & 0x7FFFFFFF))
    c = crc16(bytes(p[:38]))
    p[38], p[39] = (c >> 8) & 0xFF, c & 0xFF
    return bytes(p)

def parse(pkt):
    if len(pkt) != 26 or pkt[0] != 0xB5 or pkt[1] != 0x9A:
        return None
    stat = (pkt[4] >> 1) & 7
    cam_angle = struct.unpack_from("<3h", pkt, 12)
    mtr_angle = struct.unpack_from("<3h", pkt, 18)
    cam_rate = struct.unpack_from("<3h", pkt, 6)
    return dict(stat=stat, cam=cam_angle, mtr=mtr_angle, rate=cam_rate)

LOCK, FPV = 1, 2
ANGLE, RSPEED = 0, 2
IDLE = [(LOCK, RSPEED, 0)] * 3
PHASES = [
    (0.0,  1.0, "BASE  LOCK+零速",            IDLE, 0),
    (1.0,  4.0, "HOME  FPV+角度0 fl15",       [(FPV, ANGLE, 0)] * 3, 15),
    (4.0,  7.0, "AIM   LOCK+速率 +30°/s",     [(LOCK, RSPEED, 300)] * 3, 0),
    (7.0,  8.0, "STOP  LOCK+零速",            IDLE, 0),
    (8.0, 10.0, "CTRL  LOCK+角度 0",          [(LOCK, ANGLE, 0)] * 3, 0),
]

def phase_at(t):
    for t0, t1, name, axes, fl in PHASES:
        if t0 <= t < t1:
            return name, axes, fl
    return PHASES[-1][2:]

def main():
    ser = serial.Serial(PORT, 115200, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, timeout=0.004, write_timeout=0.5)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    t0 = time.monotonic()
    trig = 0
    buf = bytearray()
    started = False
    cur = None
    next_print = 0.0
    summary = {}
    while True:
        t = time.monotonic() - t0
        if t > 10.0:
            ser.write(build_packet(trig, 3, IDLE))  # STOP
            break
        if not started:
            ser.write(build_packet(trig, 2, IDLE))  # START
            started = True
        name, axes, fl = phase_at(t)
        if name != cur:
            cur = name
            print(f"--- {name}")
        ser.write(build_packet(trig, 4, axes, fl))
        trig = (trig + 1) & 7
        pkt = None
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
                    pkt = parse(bytes(buf[:26]))
                buf = bytearray()
                break
        if pkt:
            summary.setdefault(cur, []).append(pkt)
            if t >= next_print:
                next_print = t + 0.2
                print(f"t={t:5.2f} stat={pkt['stat']} mtr={pkt['mtr'][0]*0.01:+7.2f} "
                      f"cam={pkt['cam'][1]*0.01:+7.2f} rate={pkt['rate'][0]*0.1:+7.1f}dps")
        time.sleep(0.01)
    print()
    for name, rows in summary.items():
        if not rows:
            continue
        m = sum(r['mtr'][0] for r in rows) / len(rows) * 0.01
        c = sum(r['cam'][1] for r in rows) / len(rows) * 0.01
        peak = max(abs(r['rate'][0]) for r in rows) * 0.1
        print(f"{name}: mtr={m:+7.2f} cam={c:+7.2f} peak_rate={peak:.1f}dps n={len(rows)}")

if __name__ == "__main__":
    main()
