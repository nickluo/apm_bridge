#!/usr/bin/env python3
"""真机 OFFBOARD 控制正确性验证：0x20 控制指令 + 0x16 电机转速回读闭环。

实验室静态台架（拆桨）专用。机体不动 → 陀螺输出 ≈ 0，电机转速差动直接
反映 PID 输出方向，因此可以用"指令符号 → 转速差符号"验证从串口、协议、
OFFBOARD 仲裁、PID、混控到 DShot/Bluejay ESC 遥测的整条链路。

QuadX 混控符号约定（与固件 mixer 及闭环测试一致）：
  M0 右后 / M1 右前 / M2 左后 / M3 左前（俯视 M0/M3 桨顺时针）
  roll  右滚指令 + : 左对 (M2,M3) 加速，右对 (M0,M1) 减速
  pitch 低头指令 + : 后对 (M0,M2) 加速，前对 (M1,M3) 减速
  yaw   逆时针指令 +: M0,M3（反扭矩顺... 反扭矩 CCW）加速，M1,M2 减速

安全设计：
  * 必须显式 --confirm-props-off 才进入电机阶段（拆桨确认）
  * 电机阶段前校验：FC 未解锁 + 0x16 valid 掩码非 0（ESC 遥测活着）
  * 全程油门上限 --thrust-max（默认 14%），异常（failsafe/电流超限/
    转速超限/进程退出）立即发 arm=0 并停止
  * 结束后断言回到未解锁且四电机转速归零

用法:
  python3 hw_custom_link_rpm_test.py --port /dev/ttyUSB0            # 只测链路+0x16
  python3 hw_custom_link_rpm_test.py --confirm-props-off            # 全量电机验证
  python3 hw_custom_link_rpm_test.py --enable-bidir --confirm-props-off
      # dshot_bidir 未开时：SET dshot_bidir=ON + save(重启) 后自动重试
"""

import argparse
import collections
import struct
import sys
import threading
import time

import serial

SYNC1, SYNC2 = 0xEB, 0x90
MSG_FAST, MSG_MED, MSG_SLOW, MSG_ESC = 0x10, 0x11, 0x12, 0x16
MSG_HOST_CONTROL, MSG_FC_CLI, MSG_HOST_CLI = 0x20, 0x15, 0x24
CMD_GET, CMD_SET = 0, 1
STATUS_OK = 0
FLAG_MORE = 0x01

MODE_OFFBOARD_BIT = 1 << 13
STATUS_ARMED, STATUS_FAILSAFE = 0x0001, 0x0002

# QuadX: (右对, 左对) = (M0,M1),(M2,M3)；(前对, 后对) = (M1,M3),(M0,M2)；
# (CW 桨对 M0,M3, CCW 桨对 M1,M2)
RIGHT, LEFT, FRONT, REAR, CW, CCW = (0, 1), (2, 3), (1, 3), (0, 2), (0, 3), (1, 2)


def crc16(data, seed=0):
    crc = seed
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


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
                    self.crc = crc16(bytes(self.payload), crc16(bytes(self.hdr)))
                    self.state = 6
            elif s == 6:
                self.state = 7 if c == (self.crc & 0xFF) else 0
            elif s == 7:
                ok = c == (self.crc >> 8)
                msg = (self.hdr[0], self.hdr[2], bytes(self.payload))
                self.state = 0
                if ok:
                    on_frame(msg)


class Harness:
    """串口 + 遥测解码 + 100 Hz 控制流发送线程。"""

    def __init__(self, args):
        self.args = args
        self.ser = serial.Serial(args.port, args.baud, timeout=0.01)
        self.parser = Parser()
        self.counts = collections.Counter()
        self.state = {          # 最新遥测快照（读取线程写，主线程读）
            "armed": False, "failsafe": False, "offboard": False,
            "current_ma": 0, "rssi": 0, "gyro": (0, 0, 0),
            "esc_count": 0, "esc_valid": 0, "rpm": (0, 0, 0, 0, 0, 0, 0, 0),
        }
        self.rpm_log = []       # (monotonic, rpm tuple)
        self.cli_frames = []
        self.send_lock = threading.Lock()
        self.ctrl = {"arm": 0, "mode_req": 0, "throttle": 1000, "rate": (0, 0, 0)}
        self.ctrl_on = False
        self.cmd_seq = 0
        self.abort = None       # 置位即停止电机阶段
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()

    # ---- 读取线程 ----
    def _rx_loop(self):
        def on_frame(msg):
            mid, seq, p = msg
            self.counts[mid] += 1
            try:
                if mid == MSG_ESC and len(p) >= 22:
                    count, valid = p[4], p[5]
                    rpm = struct.unpack("<8H", p[6:22])
                    self.state["esc_count"] = count
                    self.state["esc_valid"] = valid
                    self.state["rpm"] = rpm
                    self.rpm_log.append((time.monotonic(), rpm))
                elif mid == MSG_SLOW and len(p) >= 36:
                    mode_flags, status = struct.unpack_from("<IH", p, 30)
                    self.state["armed"] = bool(status & STATUS_ARMED)
                    self.state["failsafe"] = bool(status & STATUS_FAILSAFE)
                    self.state["offboard"] = bool(mode_flags & MODE_OFFBOARD_BIT)
                    self.state["current_ma"] = struct.unpack_from("<i", p, 6)[0]
                elif mid == MSG_FAST and len(p) >= 22:
                    self.state["gyro"] = struct.unpack_from("<3h", p, 4)
                elif mid == MSG_MED and len(p) >= 47:
                    self.state["rssi"] = p[46]
                elif mid == MSG_FC_CLI and len(p) >= 8:
                    self.cli_frames.append(struct.unpack_from("<HHHBB", p) + (p[8:],))
            except struct.error:
                pass

        while self.running:
            data = self.ser.read(4096)
            if data:
                self.parser.feed(data, on_frame)

    # ---- 100 Hz 控制流发送线程 ----
    def _tx_loop(self):
        period = 1.0 / self.args.rate
        next_t = time.monotonic()
        while self.running:
            if self.ctrl_on:
                with self.send_lock:
                    c = dict(self.ctrl)
                self.cmd_seq = (self.cmd_seq + 1) & 0xFFFF
                payload = struct.pack(
                    "<QHBBH3h", int(time.time_ns() // 1000), self.cmd_seq,
                    c["arm"], c["mode_req"], c["throttle"],
                    c["rate"][0], c["rate"][1], c["rate"][2])
                frame = bytes([SYNC1, SYNC2, MSG_HOST_CONTROL, len(payload), 0]) + payload
                try:
                    self.ser.write(frame + struct.pack("<H", crc16(frame[2:])))
                except OSError:
                    pass
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    # ---- CLI 设置读写（复用 hw_custom_link_cli_test 的请求/重组）----
    def cli(self, req_id, cmd, args_str, timeout=2.0):
        self.cli_frames.clear()
        payload = struct.pack("<HBB", req_id, cmd, 0) + args_str.encode() + b"\x00"
        frame = bytes([SYNC1, SYNC2, MSG_HOST_CLI, len(payload), 0]) + payload
        with self.send_lock:
            self.ser.write(frame + struct.pack("<H", crc16(frame[2:])))
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            frames = [f for f in self.cli_frames if f[0] == req_id]
            if frames and not (frames[-1][4] & FLAG_MORE):
                text, expect = b"", 0
                for _, total, offset, _, _, data in frames:
                    if offset != expect:
                        return None, "chunk gap"
                    text += data
                    expect += len(data)
                if len(text) != frames[-1][1]:
                    return None, "length mismatch"
                return (frames[-1][3], text), None
            time.sleep(0.02)
        return None, "timeout"

    def set_control(self, arm=None, mode_req=None, thrust=None, rate=None):
        with self.send_lock:
            if arm is not None:
                self.ctrl["arm"] = arm
            if mode_req is not None:
                self.ctrl["mode_req"] = mode_req
            if thrust is not None:
                self.ctrl["throttle"] = 1000 + int(round(max(0.0, min(1.0, thrust)) * 1000))
            if rate is not None:
                self.ctrl["rate"] = tuple(int(r * 10) for r in rate)

    def stop_all(self):
        self.ctrl_on = False
        self.set_control(arm=0, mode_req=0, thrust=0.0, rate=(0, 0, 0))
        self.ctrl_on = True
        time.sleep(0.3)       # 让 arm=0 帧落地（FC 立即上锁）
        self.ctrl_on = False

    def rpm_window(self, t0, t1):
        """[t0,t1] 时间窗内每电机平均转速（跳过无效位样本）。"""
        sums, n = [0] * 8, [0] * 8
        for t, rpm in self.rpm_log:
            if t0 <= t <= t1:
                for i in range(self.state["esc_count"]):
                    if self.state["esc_valid"] & (1 << i):
                        sums[i] += rpm[i]
                        n[i] += 1
        return tuple(s / n[i] if n[i] else None for i in range(8)), n

    def close(self):
        self.running = False
        time.sleep(0.1)
        self.ser.close()


def mean(values):
    vs = [v for v in values if v is not None]
    return sum(vs) / len(vs) if vs else None


def pair_delta(rpm, a, b):
    """sum(a) - sum(b)：正 = a 组转速更高。"""
    if any(rpm[i] is None for i in a + b):
        return None
    return sum(rpm[i] for i in a) - sum(rpm[i] for i in b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--rate", type=float, default=100.0, help="0x20 控制流频率 Hz")
    ap.add_argument("--thrust-low", type=float, default=0.06, help="基础油门 (0..1)")
    ap.add_argument("--thrust-high", type=float, default=0.12, help="阶跃油门 (0..1)")
    ap.add_argument("--thrust-max", type=float, default=0.14, help="油门硬上限")
    ap.add_argument("--rate-cmd", type=float, default=120.0, help="差动测试角速度指令 deg/s")
    ap.add_argument("--diff-min", type=float, default=200.0, help="差动断言最小转速差 rpm")
    ap.add_argument("--spin-min", type=float, default=300.0, help="起转断言最小转速 rpm")
    ap.add_argument("--current-max", type=float, default=8000.0, help="安全电流上限 mA")
    ap.add_argument("--rpm-max", type=float, default=40000.0, help="安全转速上限 rpm")
    ap.add_argument("--confirm-props-off", action="store_true",
                    help="确认已拆桨，允许进入电机验证阶段")
    ap.add_argument("--enable-bidir", action="store_true",
                    help="dshot_bidir=OFF 时自动 set ON + save(FC 重启)")
    args = ap.parse_args()
    assert 0 < args.thrust_low < args.thrust_high <= args.thrust_max <= 1.0

    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {name}  {detail}")
        ok = ok and bool(cond)

    hw = Harness(args)
    print(f"opened {args.port} @ {args.baud}")

    def wait_for(desc, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            if hw.abort:
                return False
            time.sleep(0.05)
        print(f"  [FAIL] {desc} (timeout {timeout}s)")
        return False

    try:
        # ---- 1. 链路活性（init 竞态守则：等遥测字节流出）----
        print("== 1. link bring-up ==")
        time.sleep(2.0)
        t0, c0 = time.monotonic(), hw.counts[MSG_FAST]
        time.sleep(1.0)
        f10 = (hw.counts[MSG_FAST] - c0) / (time.monotonic() - t0)
        check("telemetry flowing", hw.counts[MSG_FAST] > 100, f"0x10 {f10:.1f} Hz")
        if not ok:
            print("RESULT: FAIL (no telemetry — check wiring/baud)")
            return 1

        # ---- 2. 0x16 ESC 遥测帧 ----
        print("== 2. 0x16 ESC telemetry ==")
        t0, c0 = time.monotonic(), hw.counts[MSG_ESC]
        time.sleep(1.5)
        f16 = (hw.counts[MSG_ESC] - c0) / (time.monotonic() - t0)
        check("esc_telemetry_rate", 50 <= f16 <= 150, f"0x16 {f16:.1f} Hz")
        check("esc_telemetry_count", hw.state["esc_count"] == 4,
              f"count={hw.state['esc_count']}")
        for i, setting in enumerate(("motor_pwm_protocol", "dshot_bidir", "dshot_bitbang", "motor_poles")):
            r, err = hw.cli(0x2001 + i, CMD_GET, setting)
            print(f"    {setting}: {r[1].decode(errors='replace') if r and not err else err}")
        if hw.state["esc_valid"] == 0:
            if args.enable_bidir and not hw.state["armed"]:
                print("  dshot_bidir is OFF: set ON + save (FC will reboot)")
                r, err = hw.cli(0x2F00, CMD_SET, "dshot_bidir = ON")
                check("set_dshot_bidir", r and r[0] == STATUS_OK and not err, str(err))
                r, err = hw.cli(0x2F01, CMD_SET, "save")   # 重启落盘
                print(f"    save: {r[1].decode(errors='replace') if r and not err else err}")
                print("  waiting for FC reboot (telemetry resync)...")
                time.sleep(4.0)
                deadline = time.monotonic() + 15.0
                while time.monotonic() < deadline and hw.counts[MSG_ESC] == 0:
                    time.sleep(0.5)
                time.sleep(2.0)
                check("bidir_enabled_after_save", hw.state["esc_valid"] != 0,
                      f"valid mask=0x{hw.state['esc_valid']:02x}")
            else:
                check("esc_telemetry_valid", False,
                      "valid mask=0x00 — run with --enable-bidir (disarmed) to enable, "
                      "or set dshot_bidir=ON via CLI and save")
        else:
            check("esc_telemetry_valid", True, f"valid mask=0x{hw.state['esc_valid']:02x}")

        if hw.state["esc_valid"] == 0 or not args.confirm_props_off:
            mode = "motor phases SKIPPED" if hw.state["esc_valid"] else "no ESC telemetry"
            print(f"\nRESULT: {'PASS' if ok else 'FAIL'} ({mode})")
            return 0 if ok else 1

        # ---- 以下为电机阶段：拆桨 + ESC 遥测有效才继续 ----
        print("== 3. disarmed baseline ==")
        if not wait_for("disarmed", lambda: not hw.state["armed"], 6.0):
            print("  FC is ARMED — refusing motor test")
            hw.stop_all()
            return 1
        rpm0, _ = hw.rpm_window(time.monotonic() - 1.5, time.monotonic() - 0.2)
        check("motors_idle_when_disarmed",
              all(r is not None and r < args.spin_min for r in rpm0[:4]),
              f"rpm={tuple(round(r or -1) for r in rpm0[:4])}")
        base_current = hw.state["current_ma"]
        gyro = hw.state["gyro"]
        check("gyro_near_zero_static", max(abs(g) for g in gyro) < 200,
              f"gyro 0.1dps={gyro} (静态台架前提)")

        # ---- 4. 主机解锁 + OFFBOARD ----
        print("== 4. host arm + OFFBOARD takeover ==")
        hw.ctrl_on = True
        hw.set_control(arm=1, mode_req=2, thrust=0.0, rate=(0, 0, 0))
        if not wait_for("armed", lambda: hw.state["armed"], 8.0):
            print("  arm failed — check RX (throttle stick low), failsafe, MSP connection")
            hw.stop_all()
            print(f"\nRESULT: FAIL")
            return 1
        check("host_arms", True)
        if not wait_for("offboard_mode", lambda: hw.state["offboard"], 5.0):
            print("  OFFBOARD did not engage")
            hw.stop_all()
            print("\nRESULT: FAIL")
            return 1
        check("offboard_engages", True,
              f"rssi={hw.state['rssi']}% current={hw.state['current_ma']}mA")

        # 安全监视：任何异常 → 立即停
        def safety_guard():
            while hw.ctrl_on:
                if (hw.state["failsafe"] or hw.state["current_ma"] > args.current_max or
                        any(r > args.rpm_max for r in hw.state["rpm"][:4])):
                    hw.abort = (f"failsafe={hw.state['failsafe']} "
                                f"current={hw.state['current_ma']}mA "
                                f"rpm={hw.state['rpm'][:4]}")
                    hw.set_control(arm=0)
                time.sleep(0.05)
        guard = threading.Thread(target=safety_guard, daemon=True)
        guard.start()

        # ---- 5. 起转：基础油门，四电机同步起转 ----
        print(f"== 5. spool-up (thrust {args.thrust_low:.0%}) ==")
        hw.set_control(thrust=args.thrust_low)
        time.sleep(2.0)
        if hw.abort:
            print(f"  ABORT: {hw.abort}")
            hw.stop_all()
            return 1
        t1 = time.monotonic()
        rpm_lo, _ = hw.rpm_window(t1 - 1.2, t1 - 0.1)
        m = mean(rpm_lo[:4])
        check("all_motors_spin", all(r is not None and r > args.spin_min for r in rpm_lo[:4]),
              f"rpm={tuple(round(r or -1) for r in rpm_lo[:4])}")
        if m is None or not all(r is not None for r in rpm_lo[:4]):
            hw.stop_all()
            print("\nRESULT: FAIL")
            return 1
        spread = max(rpm_lo[:4]) - min(rpm_lo[:4])
        check("motors_balanced", spread < max(2000.0, 0.5 * m), f"spread={spread:.0f} rpm")
        check("current_rises", hw.state["current_ma"] > base_current + 200,
              f"{base_current} -> {hw.state['current_ma']} mA")

        # ---- 6. 推力阶跃：转速单调跟随 ----
        print(f"== 6. thrust step {args.thrust_low:.0%} -> {args.thrust_high:.0%} -> {args.thrust_low:.0%} ==")
        hw.set_control(thrust=args.thrust_high)
        time.sleep(2.0)
        if hw.abort:
            print(f"  ABORT: {hw.abort}")
            hw.stop_all()
            return 1
        t2 = time.monotonic()
        rpm_hi, _ = hw.rpm_window(t2 - 1.2, t2 - 0.1)
        m_hi = mean(rpm_hi[:4])
        check("rpm_tracks_thrust", m_hi is not None and m_hi > m * 1.15,
              f"mean {m:.0f} -> {m_hi:.0f} rpm")
        hw.set_control(thrust=args.thrust_low)
        time.sleep(2.0)
        t3 = time.monotonic()
        rpm_lo2, _ = hw.rpm_window(t3 - 1.2, t3 - 0.1)
        m2 = mean(rpm_lo2[:4])
        check("rpm_tracks_thrust_down", m2 is not None and m2 < m_hi * 0.9,
              f"mean {m_hi:.0f} -> {m2:.0f} rpm")

        # ---- 7. 三轴差动符号验证（静态台架核心断言）----
        def axis_test(name, axis):
            # axis: 0=roll 1=pitch 2=yaw; groups = (正指令升速组, 正指令降速组)
            groups = {0: (LEFT, RIGHT), 1: (REAR, FRONT), 2: (CW, CCW)}[axis]
            print(f"== 7.{axis + 1} {name} differential ==")
            axis_ok = True
            for sign in (+1, -1):
                rate = [0, 0, 0]
                rate[axis] = sign * args.rate_cmd
                hw.set_control(rate=rate)
                time.sleep(1.6)
                if hw.abort:
                    axis_ok = False
                    break
                t = time.monotonic()
                rpm, _ = hw.rpm_window(t - 1.2, t - 0.1)
                d = pair_delta(rpm, *groups)
                good = d is not None and abs(d) > args.diff_min and (d > 0) == (sign > 0)
                check(f"{name}_{'+' if sign > 0 else '-'}", good,
                      f"Δ(升组-降组)={d:.0f} rpm" if d is not None else "no data")
                axis_ok &= good
            hw.set_control(rate=(0, 0, 0))
            time.sleep(0.8)
            return axis_ok

        # 正指令的升组：roll+ 右滚 → LEFT；pitch+ 低头 → REAR；yaw+ 逆时针 → CW 桨对(M0,M3)
        axis_ok = axis_test("roll", 0)
        axis_ok &= axis_test("pitch", 1)
        axis_ok &= axis_test("yaw", 2)

        # ---- 8. 看门狗：停流 → OFFBOARD 退出 → 电机停转 ----
        print("== 8. watchdog (stop control stream) ==")
        hw.ctrl_on = False
        if not wait_for("offboard_drops", lambda: not hw.state["offboard"], 5.0):
            hw.stop_all()
        check("watchdog_drops_offboard", not hw.state["offboard"])
        time.sleep(1.5)
        t4 = time.monotonic()
        rpm_wd, _ = hw.rpm_window(t4 - 1.2, t4 - 0.1)
        check("motors_stop_after_watchdog",
              all(r is not None and r < args.spin_min for r in rpm_wd[:4]),
              f"rpm={tuple(round(r or -1) for r in rpm_wd[:4])}")
        if not wait_for("disarmed_after_watchdog", lambda: not hw.state["armed"], 8.0):
            # 主机 arm 是电平语义，停流后 FC 按看门狗上锁；若仍解锁则显式 disarm
            hw.ctrl_on = True
            hw.set_control(arm=0)
            time.sleep(0.5)
            hw.ctrl_on = False
        check("final_disarmed", not hw.state["armed"])

        print("\n===== summary =====")
        print(f"  axis differential signs: {'PASS' if axis_ok else 'FAIL'}")
        print("\nRESULT:", "PASS" if ok and axis_ok else "FAIL")
        return 0 if ok and axis_ok else 1
    finally:
        try:
            hw.stop_all()
        except Exception:
            pass
        hw.close()


if __name__ == "__main__":
    sys.exit(main())
