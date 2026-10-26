#!/usr/bin/env python3
"""C20S 手动转动符号验证记录器: LOCK+零速保持, 50Hz 记录全遥测 60s 到 manual_log.tsv"""
import struct, time
import serial

PORT = "/dev/ttyUSB0"
DURATION = 60.0
import sys
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/c20s_probe/manual_log.tsv"
CRC_TA = [0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
          0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef]
def crc16(data):
    crc = 0
    for b in data:
        da = crc >> 12; crc = ((crc << 4) & 0xFFFF) ^ CRC_TA[da ^ (b >> 4)]
        da = crc >> 12; crc = ((crc << 4) & 0xFFFF) ^ CRC_TA[da ^ (b & 0x0F)]
    return crc
def pkt(trig, cmd, axes):
    p = bytearray(40)
    p[0], p[1] = 0xA9, 0x5B
    p[2] = ((cmd & 0x1F) << 3) | (trig & 7)
    p[3] = 0
    off = 4
    for wk, op, val in axes:
        p[off] = (wk << 4) | (op << 6)
        struct.pack_into("<h", p, off + 1, val)
        off += 3
    p[13] = 0
    struct.pack_into("<I", p, 26, (30 & 0x7F) | ((1000 << 7) & 0x7FFFFFFF))
    c = crc16(bytes(p[:38])); p[38], p[39] = (c >> 8) & 0xFF, c & 0xFF
    return bytes(p)

LOCK = 1
ANGLE, RSPEED = 0, 2
HOLD = [(LOCK, RSPEED, 0)] * 3

ser = serial.Serial(PORT, 115200, timeout=0.004, write_timeout=0.5)
ser.reset_input_buffer()
f = open(OUT, "w")
f.write("t\tcam_roll\tcam_pitch\tcam_yaw\tmtr_pitch\tmtr_roll\tmtr_yaw\trate_p\trate_r\trate_y\n")
t0 = time.monotonic(); trig = 0; buf = bytearray(); n = 0
started = False
try:
    while True:
        now = time.monotonic() - t0
        if now > DURATION: break
        cmd = 2 if (not started and now < 1.0) else 4  # 先 START 确保电机运行
        if cmd == 4: started = True
        ser.write(pkt(trig, cmd, HOLD)); trig = (trig + 1) & 7
        buf += ser.read(4096)
        while len(buf) >= 26:
            if buf[0] == 0xB5 and buf[1] == 0x9A and crc16(bytes(buf[:24])) == (buf[24] << 8 | buf[25]):
                cam = struct.unpack_from("<3h", buf, 12)
                mtr = struct.unpack_from("<3h", buf, 18)
                rate = struct.unpack_from("<3h", buf, 6)
                f.write(f"{now:.3f}\t{cam[0]}\t{cam[1]}\t{cam[2]}\t{mtr[0]}\t{mtr[1]}\t{mtr[2]}\t{rate[0]}\t{rate[1]}\t{rate[2]}\n")
                n += 1
                del buf[:26]; continue
            del buf[:1]
        if int(now) != int(now - 0.02):
            f.flush()
        time.sleep(max(0.0, 0.010 - (time.monotonic() - t0 - now)))
finally:
    f.close(); ser.close()
print(f"logged {n} samples -> {OUT}")
