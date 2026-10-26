#!/usr/bin/env python3
"""C20S 俯仰符号判别探针: 独立于 ROS 驱动, 直接驱动串口。
阶段: 3s 保持(零速) -> 4s LOCK+ANGLE +10° -> 4s LOCK+ANGLE -10° -> 3.5s ANGLE 0° -> 1.5s 零速保持
输出: 每 0.5s 遥测行 + 各阶段末 1s 稳态均值表。安全: 仅 ±10° 小角度, uav 置无效, 无校准/启停命令。
"""
import struct, time, sys
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
    """axes: [(wk_mode, op_type, op_value), ...] 槽序 [0-滚转, 1-俯仰, 2-偏航]"""
    p = bytearray(40)
    p[0], p[1] = 0xA9, 0x5B
    p[2] = (4 << 3) | (trig & 7)          # cmd=4 手动控制
    p[3] = 0                               # fl_sens = 0
    off = 4
    for wk, op, val in axes:
        p[off] = (wk << 4) | (op << 6)     # go_zero=0
        struct.pack_into("<h", p, off + 1, val)
        off += 3
    p[13] = 0                              # uav.valid = 0
    # uav angle/accel 全 0
    cam32 = (30 & 0x7F) | ((1000 << 7) & 0x7FFFFFFF)  # fov 30deg, zoom 1.000x
    struct.pack_into("<I", p, 26, cam32)
    # target_angle 全 0
    c = crc16(bytes(p[:38]))
    p[38], p[39] = (c >> 8) & 0xFF, c & 0xFF
    return bytes(p)

def parse(pkt):
    if len(pkt) != 26 or pkt[0] != 0xB5 or pkt[1] != 0x9A:
        return None
    fw, hw = pkt[2], pkt[3]
    flags = pkt[4]
    inv = flags & 1; stat = (flags >> 1) & 7; tca = (flags >> 4) & 1
    cmd_stat = pkt[5] & 7; cmd_val = (pkt[5] >> 3) & 0x1F
    cam_rate = struct.unpack_from("<3h", pkt, 6)      # [俯仰, 滚转, 偏航] 0.1deg/s
    cam_angle = struct.unpack_from("<3h", pkt, 12)    # [滚转, 俯仰, 偏航] 0.01deg
    mtr_angle = struct.unpack_from("<3h", pkt, 18)    # [俯仰, 滚转, 偏航] 0.01deg
    return dict(fw=fw, hw=hw, inv=inv, stat=stat, tca=tca, cmd=(cmd_stat, cmd_val),
                rate=cam_rate, cam=cam_angle, mtr=mtr_angle)

LOCK, FPV = 1, 2
ANGLE, RSPEED = 0, 2

class Phases:
    def __init__(self):
        self.list = [
            ("HOLD  (零速保持)",   0.0,  3.0, [(LOCK, RSPEED, 0)] * 3),
            ("P+10  (LOCK+角度+10°)", 3.0, 7.0, [(LOCK, RSPEED, 0), (LOCK, ANGLE, 1000), (LOCK, RSPEED, 0)]),
            ("P-10  (LOCK+角度-10°)", 7.0, 11.0, [(LOCK, RSPEED, 0), (LOCK, ANGLE, -1000), (LOCK, RSPEED, 0)]),
            ("BACK0 (LOCK+角度 0°)", 11.0, 14.5, [(LOCK, RSPEED, 0), (LOCK, ANGLE, 0), (LOCK, RSPEED, 0)]),
            ("HOLD2 (零速保持)",   14.5, 16.0, [(LOCK, RSPEED, 0)] * 3),
        ]
    def at(self, t):
        for name, t0, t1, axes in self.list:
            if t0 <= t < t1:
                return name, t0, t1, axes
        return self.list[-1][0], *self.list[-1][1:]

def main():
    ser = serial.Serial(PORT, 115200, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, timeout=0.004, write_timeout=0.5)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ph = Phases()
    t0 = time.monotonic()
    trig = 0; last = None; first = None
    rx = 0; buf = bytearray()
    samples = []           # (t, parsed)
    next_print = 0.0; cur_name = None
    try:
        while True:
            now = time.monotonic() - t0
            if now > 16.0: break
            name, p0, p1, axes = ph.at(now)
            if name != cur_name:
                cur_name = name
                print(f"\n===== [{now:5.2f}s] {name} =====", flush=True)
            ser.write(build_packet(trig, axes)); trig = (trig + 1) & 7
            buf += ser.read(4096)
            while len(buf) >= 26:
                if buf[0] == 0xB5 and buf[1] == 0x9A:
                    if crc16(bytes(buf[:24])) == (buf[24] << 8 | buf[25]):
                        pkt = parse(bytes(buf[:26]))
                        if pkt:
                            rx += 1; last = pkt; first = first or pkt
                            samples.append((now, pkt))
                        del buf[:26]; continue
                del buf[:1]
            if now >= next_print and last:
                c, m = last["cam"], last["mtr"]
                print(f"t={now:5.2f} cam[滚/俯/偏]={c[0]/100:7.2f},{c[1]/100:7.2f},{c[2]/100:7.2f}"
                      f" mtr[俯/滚/偏]={m[0]/100:7.2f},{m[1]/100:7.2f},{m[2]/100:7.2f}"
                      f" rate[俯/滚/偏]={last['rate'][0]/10:6.1f},{last['rate'][1]/10:6.1f},{last['rate'][2]/10:6.1f}"
                      f" stat={last['stat']} tca={last['tca']} inv={last['inv']} hw=0x{last['hw']:02X} rx={rx}", flush=True)
                next_print = now + 0.5
            time.sleep(max(0.0, 0.010 - (time.monotonic() - t0 - now)))
    finally:
        # 收尾: 回水平保持
        for _ in range(150):
            ser.write(build_packet(trig, [(LOCK, RSPEED, 0)] * 3)); trig = (trig + 1) & 7
            time.sleep(0.01)
        ser.close()

    print(f"\n总收包: {rx}")
    if first:
        print(f"首包: fw={first['fw']} hw_err=0x{first['hw']:02X} inv_flag={first['inv']}"
              f" (0吊装/1立装) gbc_stat={first['stat']} tca={first['tca']}")
    print("\n=== 各阶段末 1s 稳态均值 (deg) ===")
    for name, p0, p1, _ in ph.list:
        seg = [s for t, s in samples if p1 - 1.0 <= t < p1]
        if not seg:
            print(f"{name:24s} (无数据)"); continue
        n = len(seg)
        cam1 = sum(s["cam"][1] for s in seg) / n / 100
        mtr0 = sum(s["mtr"][0] for s in seg) / n / 100
        cam0 = sum(s["cam"][0] for s in seg) / n / 100
        print(f"{name:24s} cam_angle[1](俯仰姿态)={cam1:8.2f}   mtr_angle[0](俯仰编码)={mtr0:8.2f}   cam_angle[0](滚转姿态)={cam0:6.2f}   n={n}")

if __name__ == "__main__":
    main()
