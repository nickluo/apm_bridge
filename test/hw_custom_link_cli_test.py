#!/usr/bin/env python3
"""真机定制协议 CLI 读写通道验证（0x24 请求 / 0x15 分块响应）。

与 hw_custom_link_test.py 同一安全基线：不发送任何解锁/控制帧（0x20）。
唯一写入是 failsafe_delay 改为原值+1 再立即还原（RAM-only，同 CLI set 不
save，断电/重启即恢复；脚本开始时校验 FC 处于未解锁状态，否则拒绝写入）。

验证项：
  1. 链路活性      : 等待 0x10 遥测流出后才发首条请求（上游 init 竞态同款守则）
  2. GET 直改设置  : "failsafe_delay = N" 单块响应
  3. SET + 回读    : 改为 N+1，响应带回新值；再 GET 确认
  4. 还原          : 写回原值
  5. 错误路径      : 越界 SET / 未知设置 / 缺 '=' 语法错 / 畸形帧(reserved!=0)
  6. INFO 分块重组  : motor_pwm_protocol 元数据多块 offset/total 连续重组
  7. 字符串设置    : craft_name 只读 GET
  8. CLI 期间遥测不断流（0x10 计数持续增长）
  9. 往返延迟统计  : python 侧每请求耗时（读周期 20ms 量化，仅参考）

用法: python3 hw_custom_link_cli_test.py [--port /dev/ttyUSB0] [--baud 921600]
"""

import argparse
import collections
import struct
import sys
import time

import serial

SYNC1, SYNC2 = 0xEB, 0x90
MSG_FAST, MSG_MED, MSG_SLOW = 0x10, 0x11, 0x12
MSG_FC_CLI, MSG_HOST_CLI = 0x15, 0x24

CMD_GET, CMD_SET, CMD_INFO = 0, 1, 2
STATUS_OK, STATUS_ERR = 0, 1
FLAG_MORE = 0x01


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
    ap.add_argument("--timeout", type=float, default=2.0, help="单请求响应超时 (s)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    print(f"opened {args.port} @ {args.baud}")
    parser = Parser()
    counts = collections.Counter()
    cli_frames = []          # (req_id, total, offset, status, flags, data)
    armed = [False]

    def on_frame(msg):
        mid, seq, payload = msg
        counts[mid] += 1
        if mid == MSG_FC_CLI and len(payload) >= 8:
            req_id, total, offset, status, flags = struct.unpack_from("<HHHBB", payload)
            cli_frames.append((req_id, total, offset, status, flags, payload[8:]))
        elif mid == MSG_SLOW and len(payload) >= 36:
            armed[0] = bool(struct.unpack_from("<H", payload, 34)[0] & 1)

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            data = ser.read(4096)
            if data:
                parser.feed(data, on_frame)

    def request(req_id, cmd, args_str, reserved=0, timeout=None):
        """发送一条 0x24 并收集重组后的 0x15 响应。"""
        payload = struct.pack("<HBB", req_id, cmd, reserved) + args_str.encode() + b"\x00"
        frame = bytes([SYNC1, SYNC2, MSG_HOST_CLI, len(payload), 0]) + payload
        ser.write(frame + struct.pack("<H", crc16(frame[2:])))
        t0 = time.monotonic()
        while time.monotonic() - t0 < (timeout or args.timeout):
            pump(0.02)
            frames = [f for f in cli_frames if f[0] == req_id]
            if frames and not (frames[-1][4] & FLAG_MORE):
                break
        else:
            return None, 0, (timeout or args.timeout), "timeout"
        frames = [f for f in cli_frames if f[0] == req_id]
        text, expect = b"", 0
        for _, total, offset, _, _, data in frames:
            if offset != expect:
                return None, total, time.monotonic() - t0, f"chunk gap at {offset}"
            text += data
            expect += len(data)
        total = frames[-1][1]
        if len(text) != total:
            return None, total, time.monotonic() - t0, "length mismatch"
        return (frames[-1][3], text), total, time.monotonic() - t0, None

    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {name}  {detail}")
        ok = ok and bool(cond)

    latencies_ms = []

    def call(name, req_id, cmd, args_str, reserved=0):
        result, _total, elapsed, err = request(req_id, cmd, args_str, reserved)
        latencies_ms.append(elapsed * 1000.0)
        if err is not None:
            print(f"  [req {req_id}] {name}: ERROR {err}")
            return None
        status, text = result
        print(f"  [req {req_id}] {name}: status={status} text={text!r} ({elapsed*1000:.0f} ms)")
        return status, text

    try:
        # ---- 1. 链路活性：等遥测流出（init 竞态守则） ----
        print("== 1. link bring-up ==")
        pump(2.0)
        t0, c0 = time.monotonic(), counts[MSG_FAST]
        pump(1.0)
        f10 = (counts[MSG_FAST] - c0) / (time.monotonic() - t0)
        check("telemetry flowing", counts[MSG_FAST] > 100, f"0x10 {f10:.1f} Hz")

        # ---- 安全闸：已解锁则拒绝任何写入 ----
        print("== 2. disarmed safety gate ==")
        pump(1.5)
        check("disarmed", not armed[0],
              "REFUSING SETs" if armed[0] else "armed bit clear, writes allowed")
        allow_set = not armed[0]

        # ---- 3. GET 直改设置 ----
        print("== 3. GET direct setting ==")
        r = call("GET failsafe_delay", 0x1001, CMD_GET, "failsafe_delay")
        get_ok = r is not None and r[0] == STATUS_OK and \
            r[1].startswith(b"failsafe_delay = ")
        check("get_direct", get_ok)
        orig = int(r[1].split(b" = ")[1]) if get_ok else None

        if allow_set and orig is not None:
            # ---- 4. SET + 回读 + 确认 ----
            print(f"== 4. SET + readback (failsafe_delay = {orig + 1}, was {orig}) ==")
            r = call(f"SET {orig + 1}", 0x1002, CMD_SET, f"failsafe_delay = {orig + 1}")
            check("set_readback", r is not None and r[0] == STATUS_OK and
                  r[1] == f"failsafe_delay = {orig + 1}".encode())
            r = call("GET confirm", 0x1003, CMD_GET, "failsafe_delay")
            check("set_persists_in_ram", r is not None and
                  r[1] == f"failsafe_delay = {orig + 1}".encode())

            # ---- 5. 还原 ----
            print(f"== 5. restore (failsafe_delay = {orig}) ==")
            r = call("restore", 0x1004, CMD_SET, f"failsafe_delay = {orig}")
            check("restore", r is not None and r[0] == STATUS_OK and
                  r[1] == f"failsafe_delay = {orig}".encode())
        else:
            print("  [skip] SET phases skipped (armed or GET failed)")

        # ---- 6. 错误路径 ----
        print("== 6. error paths ==")
        r = call("GET unknown", 0x1005, CMD_GET, "zzz_no_such_setting")
        check("get_unknown", r is not None and r[0] == STATUS_ERR and r[1] == b"NOT FOUND")
        r = call("SET out of range", 0x1006, CMD_SET, "failsafe_delay = 999")
        check("set_range_rejected", r is not None and r[0] == STATUS_ERR and
              r[1] == b"SET FAILED")
        if orig is not None:
            r = call("GET unchanged", 0x1007, CMD_GET, "failsafe_delay")
            check("value_unchanged_after_rejects", r is not None and
                  r[1] == f"failsafe_delay = {orig}".encode())
        r = call("SET syntax", 0x1008, CMD_SET, "failsafe_delay")
        check("set_syntax_rejected", r is not None and r[0] == STATUS_ERR and
              r[1] == b"SET FAILED")
        cli_frames.clear()
        r = call("BAD REQUEST (reserved!=0)", 0x1009, CMD_GET, "failsafe_delay", reserved=1)
        check("bad_request", r is not None and r[0] == STATUS_ERR and
              r[1] == b"BAD REQUEST")

        # ---- 7. INFO 分块重组 ----
        print("== 7. INFO multi-chunk ==")
        cli_frames.clear()
        n_before = counts[MSG_FAST]
        result, total, elapsed, err = request(0x100A, CMD_INFO, "motor_pwm_protocol")
        latencies_ms.append(elapsed * 1000.0)
        if err is not None:
            check("info_chunked", False, f"ERROR {err}")
        else:
            status, text = result
            print(f"  [req 0x100a] INFO motor_pwm_protocol: {len(text)} B, "
                  f"total_len={total}, {elapsed*1000:.0f} ms")
            n_chunks = len([f for f in cli_frames if f[0] == 0x100A])
            print(f"  text: {text[:160].decode(errors='replace')}{'...' if len(text) > 160 else ''}")
            check("info_shape", status == STATUS_OK and b"pgn=" in text and
                  b"type=lookup" in text and b"values=" in text and b"default=" in text)
            check("info_multi_chunk", n_chunks >= 2 and len(text) == total,
                  f"{n_chunks} chunks, reassembled {len(text)}/{total}")

        # ---- 8. 字符串设置只读 ----
        print("== 8. string setting (read-only) ==")
        r = call("GET craft_name", 0x100B, CMD_GET, "craft_name")
        check("get_string", r is not None and r[0] == STATUS_OK and
              r[1].startswith(b"craft_name = "))

        # ---- 9. CLI 期间遥测不断流 ----
        print("== 9. telemetry continuity ==")
        t0, c0 = time.monotonic(), counts[MSG_FAST]
        pump(1.0)
        f10 = (counts[MSG_FAST] - c0) / (time.monotonic() - t0)
        check("telemetry_alive", counts[MSG_FAST] - c0 > 150, f"0x10 {f10:.1f} Hz")

        if latencies_ms:
            ls = sorted(latencies_ms)
            print(f"  info  CLI RTT: median {ls[len(ls)//2]:.0f} ms, "
                  f"min {ls[0]:.0f}, max {ls[-1]:.0f} (python 读周期 20ms 量化)")

        print("\nRESULT:", "PASS" if ok else "FAIL")
        return 0 if ok else 1
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
