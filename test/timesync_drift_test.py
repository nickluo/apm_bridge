#!/usr/bin/env python3
"""时间同步累积误差验证（被动遥测 + 1 Hz 对时，不发任何控制/解锁帧）。

采集原始对时交换 (t1,t2,t3,t4, 双时钟域) 与遥测帧到达时刻，然后离线重放：
  A. 旧版 client.cpp 算法    : NTP 偏置 EWMA(α=0.3/0.1) + 野值门限 4*RTT+2ms，
                               门限外永久拒绝（无重锁定路径）
  B. 改进算法 (=现行实现)    : 质量过滤样本环 + LSQ 偏置/漂移率联合拟合外推 +
                               FC 时钟跳变重置/持续野值重锁定，内部单调钟域
对比指标（遥测换算滞后 lag = 到达时刻 − fcToHost(ts)）：
  - 中位 lag / 稳健 σ (MAD) / 最大偏差        —— 静态精度
  - 2 秒窗内峰峰值 (sawtooth)                 —— 同步间隙内的锯齿
  - lag 随时间斜率                             —— 累积误差（发散）判据
故障注入：
  - +40ms 时钟步进 @60% 处   （模拟 chrony/NTP 对墙钟的阶跃校正）
  - FC 时钟回退 @60% 处      （模拟飞控软重启，uptime 归零、USB 不断）

实机结论 (2026-09): 该 FC 时钟比主机快 ~+6800 ppm，正常漂移(6.8ms/s)即超出
旧算法野值门限 → 偏置冻结 → 误差以 ~400 ms/min 线性累积；改进算法 lag 中位
+0.3ms、斜率≈0。

用法: python3 timesync_drift_test.py [--port /dev/ttyUSB0] [--baud 921600]
                                     [--duration 180] [--csv out.csv]
"""

import argparse
import math
import statistics
import struct
import sys
import time

import serial

SYNC1, SYNC2 = 0xEB, 0x90
MSG_FAST = 0x10
MSG_HOST_SYNC, MSG_FC_SYNC = 0x30, 0x31

RING = 16            # 改进算法样本环长度（个，@1Hz 即 16s）
SKEW_CLAMP = 20000e-6  # ±2% 漂移率钳位（实测 FC 可达 ±7000 ppm）
OUTLIER_US = 20_000  # 相对当前拟合线的野值带
RESET_STREAK = 3     # 连续野值次数 → 整体重置（重锁定）


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
    """镜像固件逐字节状态机。"""

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
                msg = (self.hdr[0], bytes(self.payload))
                self.state = 0
                if ok:
                    on_frame(msg)


# ----------------------------------------------------------------------
# 采集
# ----------------------------------------------------------------------

def collect(ser, duration):
    """返回 (sync_events, fast_arrivals)。

    sync_events : list of dict(t1m, t4m, t1w, t4w, t2, t3)  m=单调钟 us, w=墙钟 us
    fast_arrivals: list of (am, aw, ts32)                   遥测 0x10 到达与 FC 时间戳
    """
    sync_events = []
    fast_arrivals = []
    pending_t1 = None
    parser = Parser()

    def on_frame(msg):
        nonlocal pending_t1
        mid, payload = msg
        if mid == MSG_FC_SYNC and len(payload) == 16:
            t1, t2, t3 = struct.unpack("<QII", payload)
            if t1 == pending_t1:  # 与 client.cpp 相同的过期应答过滤
                m = time.monotonic_ns() // 1000
                w = time.time_ns() // 1000
                sync_events.append(dict(t1m=t1, t4m=m, t1w=cur_wall_send[0],
                                        t4w=w, t2=t2, t3=t3))
                pending_t1 = None
        elif mid == MSG_FAST and len(payload) >= 4:
            # 时间戳用所在 read 块的返回时刻（镜像 C++ 读取线程的行为）
            fast_arrivals.append((cur_mono[0], cur_wall[0], struct.unpack_from("<I", payload)[0]))

    cur_mono = [0]
    cur_wall = [0]
    cur_wall_send = [0]
    t0 = time.monotonic()
    next_sync = t0
    while time.monotonic() - t0 < duration:
        now = time.monotonic()
        if now >= next_sync:
            next_sync += 1.0
            t1 = time.monotonic_ns() // 1000
            cur_wall_send[0] = time.time_ns() // 1000  # 发送时刻的墙钟（域映射用）
            pending_t1 = t1
            p = struct.pack("<Q", t1)
            f = bytes([SYNC1, SYNC2, MSG_HOST_SYNC, len(p), 0]) + p
            ser.write(f + struct.pack("<H", crc16(f[2:])))
        data = ser.read(ser.in_waiting or 1)
        if data:
            cur_mono[0] = time.monotonic_ns() // 1000
            cur_wall[0] = time.time_ns() // 1000
            parser.feed(data, on_frame)
    return sync_events, fast_arrivals


# ----------------------------------------------------------------------
# 故障注入：对原始数据做一次性变换
# ----------------------------------------------------------------------

def apply_wall_step(events, fasts, at_mono, step_us):
    """模拟 chrony/NTP 对墙钟的阶跃校正：at_mono 之后墙钟整体 +step。"""
    ev = [dict(e, t1w=e["t1w"] + step_us, t4w=e["t4w"] + step_us) if e["t4m"] >= at_mono
          else dict(e) for e in events]
    fa = [(am, aw + step_us if am >= at_mono else aw, ts) for (am, aw, ts) in fasts]
    return ev, fa


def apply_fc_reboot(events, at_mono, uptime_reset_us=60_000_000):
    """模拟飞控软重启（USB 不断链）：at_mono 之后 FC uptime 回退 60s。

    直接改写原始 u32 时间戳，extend32 的最近回绕原则会自然把后续样本
    映射到回退后的新时间线，镜像真实重启行为。
    """
    return [dict(e, t2=(e["t2"] - uptime_reset_us) & 0xFFFFFFFF,
                t3=(e["t3"] - uptime_reset_us) & 0xFFFFFFFF) if e["t4m"] >= at_mono
            else dict(e) for e in events]


# ----------------------------------------------------------------------
# 重放：当前 client.cpp 算法（墙钟域，忠实移植）
# ----------------------------------------------------------------------

def replay_current(events, fasts):
    """返回 [(t_mono, lag_us)]，lag 为遥测换算滞后（墙钟域）。"""
    out = []
    valid = False
    off = 0
    ref = 0
    rtt_ms = 0.0
    ev_i = 0
    n = len(events)

    for (am, aw, ts) in fasts:
        while ev_i < n and events[ev_i]["t4m"] <= am:
            e = events[ev_i]
            t1, t4, t2, t3 = e["t1w"], e["t4w"], e["t2"], e["t3"]
            ev_i += 1
            f2 = ref + signed32(t2 - ref)
            f3 = f2 + signed32(t3 - t2)
            rtt = (t4 - t1) - (f3 - f2)
            if rtt < 0 or rtt > 200000:
                continue  # 当前实现：RTT 异常分支不更新 fc_ref
            b = ((t1 - f2) + (t4 - f3)) // 2
            if not valid:
                valid = True
                off = b
            else:
                limit = rtt * 4 + 2000
                if not (-limit <= b - off <= limit):
                    ref = f3
                    continue  # 野值：只更新参考点，偏置保持 → 无重锁定路径
                a = 0.1 if rtt / 1000.0 > 2.0 * rtt_ms + 2.0 else 0.3
                off += int(a * (b - off))
            rtt_ms = rtt / 1000.0 if rtt_ms == 0.0 else rtt_ms + 0.3 * (rtt / 1000.0 - rtt_ms)
            ref = f3
        if valid:
            host = ref + signed32(ts - ref) + off
            lag = aw - host
        else:
            lag = 0.0  # 模型未就绪，退化为到达时刻 → lag=0
        out.append((am, lag))
    return out


# ----------------------------------------------------------------------
# 重放：改进算法（单调钟域 + 质量样本环 + LSQ 偏置/漂移率 + 重锁定）
# ----------------------------------------------------------------------

def replay_improved(events, fasts):
    out = []
    ring = []            # [t4, b] 已过质量门限的样本
    rtt_floor = None
    last_t4 = None
    prev_f3 = None
    ref = 0
    fit = None          # (c0, c1): b(t) = c0 + c1*t（t 为原始 µs，数值中心化后拟合）
    outlier_streak = 0
    ev_i = 0
    n = len(events)

    def refit():
        nonlocal fit
        m = len(ring)
        if m == 0:
            fit = None
            return
        if m < 3:
            fit = None  # 样本不足：退化为取最新样本
            return
        mt = statistics.fmean(s[0] for s in ring)
        mb = statistics.fmean(s[1] for s in ring)
        den = sum((s[0] - mt) ** 2 for s in ring)
        if den <= 0:
            fit = None
            return
        c1 = sum((s[0] - mt) * (s[1] - mb) for s in ring) / den
        c1 = max(-SKEW_CLAMP, min(SKEW_CLAMP, c1))
        fit = (mb - c1 * mt, c1)

    def off_at(t):
        if fit is not None:
            return fit[0] + fit[1] * t
        return ring[-1][1] if ring else None

    for (am, aw, ts) in fasts:
        while ev_i < n and events[ev_i]["t4m"] <= am:
            e = events[ev_i]
            t1, t4, t2, t3 = e["t1m"], e["t4m"], e["t2"], e["t3"]
            ev_i += 1
            f2 = ref + signed32(t2 - ref)
            f3 = f2 + signed32(t3 - t2)
            rtt = (t4 - t1) - (f3 - f2)
            if rtt < 0 or rtt > 200000:
                continue
            # FC 时钟跳变（重启/回退）→ 整体重置
            if prev_f3 is not None and abs(f2 - prev_f3) > 2_000_000:
                ring.clear()
                fit = None
                rtt_floor = None
                outlier_streak = 0
            prev_f3 = f3
            ref = f3
            # RTT 下限老化（允许链路劣化后缓慢抬升，200µs/s）
            if rtt_floor is not None and last_t4 is not None:
                rtt_floor = min(rtt_floor + 0.2 * (t4 - last_t4) / 1000.0, rtt)
            else:
                rtt_floor = rtt
            last_t4 = t4
            if rtt > rtt_floor + max(2000, 2 * rtt_floor):
                continue  # 突发劣化样本不入环
            b = ((t1 - f2) + (t4 - f3)) // 2
            # 相对当前拟合线的野值：单次跳过，持续偏离则重锁定
            if fit is not None and abs(b - off_at(t4)) > OUTLIER_US:
                outlier_streak += 1
                if outlier_streak >= RESET_STREAK:
                    ring.clear()
                    fit = None
                    outlier_streak = 0
                continue
            outlier_streak = 0
            ring.append([t4, b])
            if len(ring) > RING:
                ring.pop(0)
            refit()
        o = off_at(am) if ring else None
        if o is not None:
            host = ref + signed32(ts - ref) + o
            lag = am - host
        else:
            lag = 0.0
        out.append((am, lag))
    return out


# ----------------------------------------------------------------------
# 指标
# ----------------------------------------------------------------------

def metrics(lags):
    ts = [t for t, _ in lags]
    vs = [v for _, v in lags]
    med = statistics.median(vs)
    mad = statistics.median(abs(v - med) for v in vs) * 1.4826
    # 斜率（最小二乘）: µs/s —— >0 表示滞后随时间增长（累积）
    nn = len(ts)
    if nn > 2:
        mt = statistics.fmean(ts)
        mv = statistics.fmean(vs)
        denom = sum((t - mt) ** 2 for t in ts)
        slope = sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / denom if denom else 0.0
    else:
        slope = 0.0
    # 2 秒滑动窗内峰峰值
    saw = 0.0
    j = 0
    for i in range(nn):
        while ts[i] - ts[j] > 2_000_000:
            j += 1
        if i > j:
            window = vs[j:i + 1]
            saw = max(saw, max(window) - min(window))
    return dict(med=med / 1000.0, sigma=mad / 1000.0, slope=slope,
                maxdev=max(abs(v - med) for v in vs) / 1000.0, saw=saw / 1000.0)


def report(name, m):
    print(f"  {name:<34s} med {m['med']:+8.2f} ms  σ {m['sigma']:7.3f}  "
          f"max|dev| {m['maxdev']:8.2f}  2s峰峰 {m['saw']:7.2f}  "
          f"斜率 {m['slope'] * 60000:+8.2f} ms/min")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.001)
    print(f"opened {args.port} @ {args.baud}, collecting {args.duration:.0f}s ...")
    events, fasts = collect(ser, args.duration)
    ser.close()
    print(f"got {len(events)} sync responses, {len(fasts)} FAST frames")
    if len(events) < 10 or not fasts:
        print("FAIL: insufficient data")
        return 1

    if args.csv:
        with open(args.csv, "w") as f:
            f.write("t1m,t4m,t1w,t4w,t2,t3\n")
            for e in events:
                f.write(f"{e['t1m']},{e['t4m']},{e['t1w']},{e['t4w']},{e['t2']},{e['t3']}\n")

    # 原始统计
    raw_b = []
    for e in events:
        t1, t4 = e["t1m"], e["t4m"]
        f2 = e["t2"]
        f3 = f2 + signed32(e["t3"] - e["t2"])
        rtt = (t4 - t1) - (f3 - f2)
        raw_b.append((t4, ((t1 - f2) + (t4 - f3)) // 2, rtt))
    rtts = sorted(r for _, _, r in raw_b)
    print(f"\nRTT: min {rtts[0]/1000:.2f}  p50 {rtts[len(rtts)//2]/1000:.2f}  "
          f"p95 {rtts[int(len(rtts)*0.95)]/1000:.2f}  max {rtts[-1]/1000:.2f} ms")
    # 晶振相对漂移（Theil–Sen 稳健斜率）
    slopes = []
    for i in range(len(raw_b)):
        for j in range(i + 1, len(raw_b)):
            dt = raw_b[j][0] - raw_b[i][0]
            if dt > 30_000_000:
                slopes.append((raw_b[j][1] - raw_b[i][1]) / dt)
                break
    if slopes:
        slopes.sort()
        ppm = slopes[len(slopes) // 2] * 1e6
        print(f"时钟相对漂移 (host vs FC): {ppm:+.1f} ppm "
              f"({ppm*60/1000:+.2f} ms/min)")

    t_mid = events[len(events) // 2]["t4m"]
    step_us = 40_000  # +40ms 墙钟步进

    print("\n===== 重放对比（正常链路） =====")
    report("A 当前算法 (EWMA+门限)", metrics(replay_current(events, fasts)))
    report("B 改进算法 (minRTT+skew)", metrics(replay_improved(events, fasts)))

    print("\n===== 注入: 墙钟 +40ms 步进 @中点 (NTP 校时) =====")
    ev_s, fa_s = apply_wall_step(events, fasts, t_mid, step_us)
    report("A 当前算法", metrics(replay_current(ev_s, fa_s)))
    report("B 改进算法(单调钟域)", metrics(replay_improved(ev_s, fa_s)))

    print("\n===== 注入: FC 软重启 @中点 (uptime 归零) =====")
    ev_r = apply_fc_reboot(events, t_mid)
    report("A 当前算法", metrics(replay_current(ev_r, fasts)))
    report("B 改进算法(跳变重置)", metrics(replay_improved(ev_r, fasts)))

    return 0


if __name__ == "__main__":
    sys.exit(main())
