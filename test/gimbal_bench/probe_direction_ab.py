#!/usr/bin/env python3
"""C20S 方向判别 A/B: 第一段 cam+30°, 回平, 第二段 cam-30°, 回平。观察第一段摆向。"""
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
    return dict(cam=struct.unpack_from("<3h", pkt, 12), mtr=struct.unpack_from("<3h", pkt, 18))

LOCK = 1
ANGLE, RSPEED = 0, 2
PHASES = [
    (0.0,  3.0, [(LOCK, ANGLE, 0)]*3,           "准备: 回平静止 3 秒"),
    (3.0,  9.0, [(LOCK, RSPEED, 0), (LOCK, ANGLE, 3000), (LOCK, RSPEED, 0)],
     "*** 第一段: cam+30 度 (t=3~9s) -- 请观察此刻相机摆向 ***"),
    (9.0, 11.5, [(LOCK, ANGLE, 0)]*3,           "回平"),
    (11.5, 17.5, [(LOCK, RSPEED, 0), (LOCK, ANGLE, -3000), (LOCK, RSPEED, 0)],
     "*** 第二段: cam-30 度 (t=11.5~17.5s) ***"),
    (17.5, 20.0, [(LOCK, ANGLE, 0)]*3,          "回平收尾"),
]

def main():
    ser = serial.Serial(PORT, 115200, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, timeout=0.004, write_timeout=0.5)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    t0 = time.monotonic(); trig = 0; buf = bytearray(); last = None
    cur = None; next_print = 0.0
    try:
        while True:
            now = time.monotonic() - t0
            if now > 20.0: break
            ph = next(x for x in PHASES if x[0] <= now < x[1])
            if ph[3] != cur:
                cur = ph[3]
                print(f"\n[{now:5.2f}s] {cur}", flush=True)
            ser.write(build_packet(trig, ph[2])); trig = (trig + 1) & 7
            buf += ser.read(4096)
            while len(buf) >= 26:
                if buf[0] == 0xB5 and buf[1] == 0x9A and crc16(bytes(buf[:24])) == (buf[24] << 8 | buf[25]):
                    last = parse(bytes(buf[:26])); del buf[:26]; continue
                del buf[:1]
            if now >= next_print and last:
                print(f"  t={now:5.2f} cam[1]={last['cam'][1]/100:7.2f} mtr[0]={last['mtr'][0]/100:7.2f}", flush=True)
                next_print = now + 0.8
            time.sleep(max(0.0, 0.010 - (time.monotonic() - t0 - now)))
    finally:
        for _ in range(150):
            ser.write(build_packet(trig, [(LOCK, RSPEED, 0)] * 3)); trig = (trig + 1) & 7
            time.sleep(0.01)
        ser.close()
    print("\n完成。请回答: 第一段(t=3~9s)相机朝天空还是朝地面摆动?")

if __name__ == "__main__":
    main()
