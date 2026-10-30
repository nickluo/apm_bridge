#!/usr/bin/env python3
"""C20S go_zero 回中行为探针: 独立于 ROS 驱动, 直接驱动串口。

目的: 回答"真机 go_zero 边沿触发后, 云台做什么、编码器停在哪"——
回中目标是否=编码器零点、运动时长、gbc_stat 是否变化。

阶段 (t 秒):
  0.0  发送 START (cmd=2), 之后 100Hz 手动控制包 (LOCK+真实角速度 0)
  0-2  静置, 记录起始编码器
  2.0  俯仰轴 go_zero 0->1 边沿 (mask=0x2), 保持观察
  12.0 若尚未静止再补一次三轴边沿 (mask=0x7)
  20.0 发送 STOP, 退出

输出: 每 0.2s 一行 [t stat mtr_pitch cam_pitch rate_pitch], 末尾稳态摘要。
用法: /usr/bin/python3 probe_go_zero.py [port]   (默认 /dev/ttyUSB1)
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

def build_packet(trig, cmd, go_zero_mask):
    p = bytearray(40)
    p[0], p[1] = 0xA9, 0x5B
    p[2] = (cmd << 3) | (trig & 7)
    p[3] = 0                                # fl_sens=0
    for i in range(3):                      # 槽序 [0-滚转, 1-俯仰, 2-偏航]
        off = 4 + 3 * i
        wk, op, val = 1, 2, 0               # LOCK + 真实角速度 0
        p[off] = (wk << 4) | (op << 6) | (1 if go_zero_mask & (1 << i) else 0)
        struct.pack_into("<h", p, off + 1, val)
    p[13] = 0                               # uav.valid = 0
    cam32 = (30 & 0x7F) | ((1000 << 7) & 0x7FFFFFFF)
    struct.pack_into("<I", p, 26, cam32)
    c = crc16(bytes(p[:38]))
    p[38], p[39] = (c >> 8) & 0xFF, c & 0xFF
    return bytes(p)

def parse(pkt):
    if len(pkt) != 26 or pkt[0] != 0xB5 or pkt[1] != 0x9A:
        return None
    stat = (pkt[4] >> 1) & 7
    cam_angle = struct.unpack_from("<3h", pkt, 12)   # [滚转, 俯仰, 偏航] 0.01deg
    mtr_angle = struct.unpack_from("<3h", pkt, 18)   # [俯仰, 滚转, 偏航] 0.01deg
    cam_rate = struct.unpack_from("<3h", pkt, 6)     # [俯仰, 滚转, 偏航] 0.1deg/s
    return dict(stat=stat, cam=cam_angle, mtr=mtr_angle, rate=cam_rate)

def main():
    ser = serial.Serial(PORT, 115200, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, timeout=0.004, write_timeout=0.5)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    t0 = time.monotonic()
    trig = 0
    buf = bytearray()
    rows = []
    started = False
    next_print = 0.0
    go_mask = 0
    while True:
        t = time.monotonic() - t0
        if t > 20.0:
            ser.write(build_packet(trig, 3, 0))  # STOP
            break
        if not started and t >= 0.0:
            ser.write(build_packet(trig, 2, 0))  # START
            started = True
        # 边沿时刻: t=2 俯仰轴, t=12 三轴兜底
        mask = 0x2 if t >= 2.0 else 0
        if t >= 12.0:
            mask = 0x7
        if mask != go_mask:
            go_mask = mask
            print(f"--- t={t:5.2f}s: go_zero mask -> 0x{mask:X} (edge)")
        ser.write(build_packet(trig, 4, go_mask))
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
        if pkt and t >= next_print:
            next_print = t + 0.2
            rows.append((t, pkt))
            print(f"t={t:5.2f}  stat={pkt['stat']}  mtr_pitch={pkt['mtr'][0]*0.01:+7.2f}deg  "
                  f"cam_pitch={pkt['cam'][1]*0.01:+7.2f}deg  rate_pitch={pkt['rate'][0]*0.1:+7.1f}dps")
        time.sleep(0.01)
    if rows:
        pre = [r for r in rows if r[0] < 2.0]
        post = [r for r in rows if r[0] > 17.0]
        if pre:
            m0 = sum(r[1]['mtr'][0] for r in pre) / len(pre) * 0.01
            print(f"\n回中前稳态 mtr_pitch = {m0:+.2f} deg ({len(pre)} 采样)")
        if post:
            m1 = sum(r[1]['mtr'][0] for r in post) / len(post) * 0.01
            c1 = sum(r[1]['cam'][1] for r in post) / len(post) * 0.01
            print(f"回中后稳态 mtr_pitch = {m1:+.2f} deg, cam_pitch = {c1:+.2f} deg ({len(post)} 采样)")

if __name__ == "__main__":
    main()
