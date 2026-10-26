#!/usr/bin/env python3
"""C20S 判别第二阶段: 速率指令域 + FPV 相对角域 + 大幅可见动作。
A: 3s 保持 | B: 4s LOCK+ANGLE +20°(肉眼判别) | C: 3s LOCK+REAL_SPEED +30°/s
D: 3s LOCK+REAL_SPEED -30°/s | E: 3s FPV+ANGLE +15° | F: 3s 回 LOCK+ANGLE 0°
"""
import struct, time
import serial

PORT = "/dev/ttyUSB0"
CRC_TA = [0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
          0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef]
def crc16(data):
    crc = 0
    for b in data:
        da = crc >> 12; crc = ((crc << 4) & 0xFFFF) ^ CRC_TA[da ^ (b >> 4)]
        da = crc >> 12; crc = ((crc << 4) & 0xFFFF) ^ CRC_TA[da ^ (b & 0x0F)]
    return crc

def build_packet(trig, axes):
    p = bytearray(40)
    p[0], p[1] = 0xA9, 0x5B
    p[2] = (4 << 3) | (trig & 7)
    p[3] = 0
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
    cam_rate = struct.unpack_from("<3h", pkt, 6)
    cam_angle = struct.unpack_from("<3h", pkt, 12)
    mtr_angle = struct.unpack_from("<3h", pkt, 18)
    return dict(rate=cam_rate, cam=cam_angle, mtr=mtr_angle)

LOCK, FPV = 1, 2
ANGLE, RSPEED = 0, 2
PHASES = [
    ("A 保持",       0.0,  3.0, [(LOCK, RSPEED, 0)] * 3),
    ("B LOCK+角+20°", 3.0,  7.0, [(LOCK, RSPEED, 0), (LOCK, ANGLE, 2000), (LOCK, RSPEED, 0)]),
    ("C 速率+30°/s", 7.0, 10.0, [(LOCK, RSPEED, 0), (LOCK, RSPEED, 300), (LOCK, RSPEED, 0)]),
    ("D 速率-30°/s", 10.0, 13.0, [(LOCK, RSPEED, 0), (LOCK, RSPEED, -300), (LOCK, RSPEED, 0)]),
    ("E FPV+角+15°", 13.0, 16.0, [(FPV, ANGLE, 0), (FPV, ANGLE, 1500), (FPV, ANGLE, 0)]),
    ("F 回0°",       16.0, 19.0, [(LOCK, RSPEED, 0), (LOCK, ANGLE, 0), (LOCK, RSPEED, 0)]),
]

def main():
    ser = serial.Serial(PORT, 115200, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, timeout=0.004, write_timeout=0.5)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    t0 = time.monotonic()
    trig = 0; last = None; buf = bytearray(); samples = []
    cur = None; next_print = 0.0
    try:
        while True:
            now = time.monotonic() - t0
            if now > 19.0: break
            name, p0, p1, axes = next((n, a, b, x) for n, a, b, x in PHASES if a <= now < b) if any(a <= now < b for _, a, b, _ in PHASES) else (None,)*4
            if name and name != cur:
                cur = name
                print(f"\n===== [{now:5.2f}s] {name} =====", flush=True)
            if name:
                ser.write(build_packet(trig, axes)); trig = (trig + 1) & 7
            buf += ser.read(4096)
            while len(buf) >= 26:
                if buf[0] == 0xB5 and buf[1] == 0x9A and crc16(bytes(buf[:24])) == (buf[24] << 8 | buf[25]):
                    pkt = parse(bytes(buf[:26]))
                    if pkt:
                        last = pkt; samples.append((now, pkt))
                    del buf[:26]; continue
                del buf[:1]
            if now >= next_print and last:
                c, m = last["cam"], last["mtr"]
                print(f"t={now:5.2f} cam[1]={c[1]/100:8.2f} mtr[0]={m[0]/100:8.2f} cam[0]={c[0]/100:6.2f} rate[0]={last['rate'][0]/10:7.1f}", flush=True)
                next_print = now + 0.25
            time.sleep(max(0.0, 0.010 - (time.monotonic() - t0 - now)))
    finally:
        for _ in range(200):
            ser.write(build_packet(trig, [(LOCK, RSPEED, 0)] * 3)); trig = (trig + 1) & 7
            time.sleep(0.01)
        ser.close()

    print("\n=== 阶段首/末快照与均值 (deg) ===")
    for name, p0, p1, _ in PHASES:
        seg = [s for t, s in samples if p0 <= t < p1]
        if not seg: continue
        f, l = seg[0], seg[-1]
        mid = seg[len(seg)//2]
        print(f"{name:14s} cam[1]: {f['cam'][1]/100:7.2f} -> {mid['cam'][1]/100:7.2f} -> {l['cam'][1]/100:7.2f}   "
              f"mtr[0]: {f['mtr'][0]/100:7.2f} -> {mid['mtr'][0]/100:7.2f} -> {l['mtr'][0]/100:7.2f}   "
              f"末段率测 rate[0]={l['rate'][0]/10:6.1f}")

if __name__ == "__main__":
    main()
