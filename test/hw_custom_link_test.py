#!/usr/bin/env python3
"""真机定制协议验证（被动遥测 + 时间同步，不发送任何解锁/控制帧）。

验证项：
  1. 下行三流频率    : 0x10 ~200Hz / 0x11 ~100Hz / 0x12 ~10Hz
  2. 丢帧统计        : 下行全局 Seq 连续性
  3. 数据合理性      : 电压/气压/温度/RC/IMU(静止 |acc|~9.8, gyro~0)/姿态/armed 位
  4. 时间同步        : 0x30->0x31 往返 RTT 统计
  5. FC 时间戳换算   : 遥测 ts 经偏置模型换算后的滞后

用法: python3 hw_custom_link_test.py [--port /dev/ttyUSB0] [--baud 921600] [--duration 15]
"""

import argparse
import collections
import math
import struct
import sys
import time

import serial

SYNC1, SYNC2 = 0xEB, 0x90
MSG_FAST, MSG_MED, MSG_SLOW = 0x10, 0x11, 0x12
MSG_HOST_SYNC, MSG_FC_SYNC = 0x30, 0x31


def crc16(data, seed=0):
    crc = seed
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def signed32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v >= (1 << 31) else v


class Parser:
    """镜像固件逐字节状态机（帧头重扫描/长度校验/CRC）。"""

    def __init__(self):
        self.state = 0
        self.hdr = bytearray(3)
        self.payload = None
        self.crc = 0

    def feed(self, buf, on_frame):
        for c in buf:
            s = self.state
            if s == 0:
                if c == SYNC1:
                    self.state = 1
            elif s == 1:
                self.state = 2 if c == SYNC2 else (1 if c == SYNC1 else 0)
            elif s == 2:
                self.hdr[0] = c
                self.state = 3
            elif s == 3:
                self.hdr[1] = c
                self.state = 4
            elif s == 4:
                self.hdr[2] = c
                if self.hdr[1] == 0 or self.hdr[1] > 64:
                    self.state = 0
                else:
                    self.payload = bytearray()
                    self.state = 5
            elif s == 5:
                self.payload.append(c)
                if len(self.payload) >= self.hdr[1]:
                    self.crc = crc16(self.payload, crc16(bytes(self.hdr)))
                    self.state = 6
            elif s == 6:
                self.state = 7 if c == (self.crc & 0xFF) else 0
            elif s == 7:
                ok = c == (self.crc >> 8)
                msg = (self.hdr[0], self.hdr[2], bytes(self.payload))
                self.state = 0
                if ok:
                    on_frame(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--duration", type=float, default=15.0)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    print(f"opened {args.port} @ {args.baud}")

    counts = collections.Counter()
    seq_gaps = 0
    last_seq = None
    latest = {MSG_FAST: None, MSG_MED: None, MSG_SLOW: None}
    rtts_ms = []
    # 时统模型：B = host_monotonic_us - fc_uptime_ext_us
    bias_us = None
    fc_ref_us = None
    ts_lag_ms = []

    parser = Parser()

    def on_frame(msg):
        nonlocal seq_gaps, last_seq, bias_us, fc_ref_us
        mid, seq, payload = msg
        counts[mid] += 1
        if last_seq is not None and seq != ((last_seq + 1) & 0xFF):
            seq_gaps += 1
        last_seq = seq

        if mid == MSG_FC_SYNC and len(payload) == 16:
            t1, t2, t3 = struct.unpack("<QII", payload)
            t4 = time.monotonic_ns() // 1000
            t2e = t1 + signed32(t2 - t1)
            t3e = t2e + signed32(t3 - t2)
            rtt = (t4 - t1) - (t3e - t2e)
            rtts_ms.append(rtt / 1000.0)
            bias_us = ((t1 - t2e) + (t4 - t3e)) // 2
            fc_ref_us = t3e
            return

        if mid in latest and len(payload) >= 4:
            latest[mid] = payload
            if bias_us is not None and fc_ref_us is not None:
                ts = struct.unpack_from("<I", payload)[0]
                ts_e = fc_ref_us + signed32(ts - fc_ref_us)
                lag = (time.monotonic_ns() // 1000) - (ts_e + bias_us)
                ts_lag_ms.append(lag / 1000.0)

    t0 = time.monotonic()
    next_sync = t0
    while time.monotonic() - t0 < args.duration:
        if time.monotonic() >= next_sync:
            next_sync += 1.0
            t1 = time.monotonic_ns() // 1000
            p = struct.pack("<Q", t1)
            f = bytes([SYNC1, SYNC2, MSG_HOST_SYNC, len(p), 0]) + p
            ser.write(f + struct.pack("<H", crc16(f[2:])))
        data = ser.read(4096)
        if data:
            parser.feed(data, on_frame)
    dt = time.monotonic() - t0
    ser.close()

    print(f"\n===== {args.port} @ {args.baud}, {dt:.1f}s =====")
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {name}  {detail}")
        ok = ok and bool(cond)

    f10 = counts[MSG_FAST] / dt
    f11 = counts[MSG_MED] / dt
    f12 = counts[MSG_SLOW] / dt
    check("0x10 rate ~200Hz", 160 <= f10 <= 240, f"{f10:.1f} Hz")
    check("0x11 rate ~100Hz", 80 <= f11 <= 120, f"{f11:.1f} Hz")
    check("0x12 rate ~10Hz", 8 <= f12 <= 12, f"{f12:.1f} Hz")
    check("no seq gaps", seq_gaps == 0, f"{seq_gaps} gaps")

    if rtts_ms:
        rs = sorted(rtts_ms)
        med = rs[len(rs) // 2]
        check("timesync RTT", med < 20.0,
              f"median {med:.2f} ms, min {rs[0]:.2f}, max {rs[-1]:.2f}, n={len(rs)}")
    else:
        check("timesync RTT", False, "no 0x31 responses")

    if ts_lag_ms:
        m = max(abs(x) for x in ts_lag_ms[50:])
        check("telemetry stamp lag", m < 50.0, f"max |lag| {m:.2f} ms (n={len(ts_lag_ms)})")

    if latest[MSG_FAST]:
        ts = struct.unpack_from("<I", latest[MSG_FAST])[0]
        gyro = struct.unpack_from("<3h", latest[MSG_FAST], 4)
        acc = struct.unpack_from("<3h", latest[MSG_FAST], 10)
        att = struct.unpack_from("<3h", latest[MSG_FAST], 16)
        acc_ms2 = [a * 0.001 * 9.80665 for a in acc]
        acc_mag = math.sqrt(sum(x * x for x in acc_ms2))
        gyro_dps = [g * 0.1 for g in gyro]
        att_deg = [a * 0.01 for a in att]
        check("static |acc| ~9.8", 8.0 < acc_mag < 11.5,
              f"{acc_mag:.2f} m/s^2  [{', '.join(f'{a:+.2f}' for a in acc_ms2)}]")
        check("static gyro ~0", max(abs(g) for g in gyro_dps) < 3.0,
              f"[{', '.join(f'{g:+.1f}' for g in gyro_dps)}] deg/s")
        check("attitude sane", all(abs(a) < 90 for a in att_deg[:2]) and abs(att_deg[2]) <= 180.0,
              f"rpy [{', '.join(f'{a:+.1f}' for a in att_deg)}] deg (yaw wraps at +-180)")

    if latest[MSG_MED]:
        ts, pa, alt_cm, temp = struct.unpack_from("<IIih", latest[MSG_MED])
        rc = struct.unpack_from("<16H", latest[MSG_MED], 14)
        print(f"  info  baro={pa} Pa  alt={alt_cm/100:.2f} m  temp={temp/100:.1f} C")
        print(f"  info  rc[0..7]=[{', '.join(str(int(x)) for x in rc[:8])}]")
        check("baro plausible", 80000 < pa < 110000, f"{pa} Pa")
        check("temp plausible", -20 < temp / 100 < 90, f"{temp/100:.1f} C")

    if latest[MSG_SLOW]:
        (ts, vbat, cur, mah, fix, sats, lat, lon, alt, spd, course,
         mode_flags, status) = struct.unpack_from("<IHihBBiiiHHIH", latest[MSG_SLOW])
        print(f"  info  vbat={vbat/1000:.2f} V  current={cur/1000:.2f} A  mAh={mah}"
              f"  fix={fix} sats={sats} lat={lat/1e7:.6f} lon={lon/1e7:.6f}")
        print(f"  info  mode_flags={mode_flags:#x}  armed={status & 1}  failsafe={(status >> 1) & 1}")
        # USB 供电（未接电池）时 ADC 读数接近 0，属正常
        usb_powered = vbat / 1000 < 0.5
        check("vbat plausible", usb_powered or 6.0 < vbat / 1000 < 85.0,
              f"{vbat/1000:.2f} V" + (" (USB powered, no battery)" if usb_powered else ""))
        check("disarmed", (status & 1) == 0)
        check("no failsafe", (status & 2) == 0)

    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
