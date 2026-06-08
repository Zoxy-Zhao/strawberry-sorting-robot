#!/usr/bin/env python3
"""pickup_v2 MCU 协议联调测试 — Z/HOMX/OPEN/CLOSE/HOME/K 全套

用法（Pi 上）:  python3 test_mcu_protocol.py

前提:
  1. MCU 已烧 NO-STRTOK BUILD 2026-05-15 固件（Debug COM 看 boot 指纹确认）
  2. 舵机 12V 电源已开（OPEN/CLOSE/HOME/K 测试需要）
  3. /dev/serial0 物理接到 MCU SCI9（MCU P109/P110 ↔ Pi GPIO14/15）
  4. 急停按钮（机械）放手边
"""

import os
import sys
import time
import serial

PORT = "/dev/serial0"
BAUD = 115200
SHORT_T = 3.0  # NACK / OPEN 等单关节
LONG_T = 15.0  # HOME / K 多关节缓动可能 5+ 秒

# ANSI 颜色
G = "\033[92m"
R = "\033[91m"
Y = "\033[93m"
B = "\033[94m"
N = "\033[0m"

# HOME 角度 — 与 hal_entry_pickup_v2.c::g_pose_home 同步
HOME_ANGLES = [135, 70, 70, 150, 80, 120]


def open_port():
    if not os.path.exists(PORT):
        print(
            f"{R}FATAL{N} {PORT} 不存在；检查 Pi 串口是否启用 (raspi-config → Interface → Serial)"
        )
        sys.exit(1)
    try:
        s = serial.Serial(PORT, BAUD, timeout=SHORT_T)
    except Exception as e:
        print(f"{R}FATAL{N} 打开 {PORT} 失败: {e}")
        sys.exit(1)
    s.reset_input_buffer()
    s.reset_output_buffer()
    return s


def read_until_nl(s, timeout):
    """读到收到 \\n 或超时为止，返回原始 bytes（可能为空）"""
    deadline = time.monotonic() + timeout
    buf = b""
    s.timeout = 0.2
    while time.monotonic() < deadline:
        chunk = s.read(64)
        if chunk:
            buf += chunk
            if b"\n" in buf:
                return buf
    return buf


def run_step(s, name, cmd_bytes, expected, timeout=SHORT_T, danger=False):
    """
    name:     测试名
    cmd_bytes: 要发的原始字节（含 \\n）
    expected: 期望子串（str）或 list[str]，None 表示任意非空都算 PASS
    danger:   True 时先 prompt 用户确认（OPEN/CLOSE/HOME/K）
    return:   (status, text)  status ∈ PASS|FAIL|TIMEOUT|SKIP|ERROR
    """
    print(f"\n{B}=== {name} ==={N}")
    print(f"  发送: {cmd_bytes!r}")
    if danger:
        ans = (
            input(
                f"  {Y}WARN{N} 这条会让舵机动。已通电？回车继续 / 输 n 跳过 / 输 q 退出: "
            )
            .strip()
            .lower()
        )
        if ans == "q":
            print(f"  {Y}USER QUIT{N}")
            sys.exit(0)
        if ans == "n":
            print(f"  {Y}SKIP{N}")
            return ("SKIP", "")

    try:
        s.reset_input_buffer()
        t0 = time.monotonic()
        s.write(cmd_bytes)
        raw = read_until_nl(s, timeout)
        dt = time.monotonic() - t0
    except Exception as e:
        print(f"  {R}ERROR{N} 串口异常: {e}")
        return ("ERROR", str(e))

    print(f"  回复: {raw!r}  ({dt * 1000:.0f} ms)")
    if not raw:
        print(f"  {R}FAIL TIMEOUT{N} — MCU 无响应。可能 MCU 卡死 / 串口断 / 命令格式错")
        print(f"  {Y}诊断{N} 按 RESET 复活 MCU；需深查时插 USB 看 Debug COM [Dx]/[Nx]")
        return ("TIMEOUT", "")

    text = raw.decode("ascii", errors="replace").strip()
    if expected is None:
        print(f"  {G}PASS{N} 收到非空响应")
        return ("PASS", text)

    expected_list = expected if isinstance(expected, list) else [expected]
    for exp in expected_list:
        if exp in text:
            print(f"  {G}PASS{N} 匹配 '{exp}'")
            return ("PASS", text)
    print(f"  {R}FAIL{N} 期望 {expected_list}，实际 '{text}'")
    return ("FAIL", text)


def banner():
    print(f"{B}=== pickup_v2 MCU 协议联调测试 ==={N}")
    print(f"  端口 {PORT} @ {BAUD}")
    print(f"  {Y}前提{N} MCU 已烧 NO-STRTOK BUILD 2026-05-15，舵机电源就绪，急停在手边")
    print(
        f"  {Y}建议{N} 拔 USB（消除 GND loop，PWM 更稳）；只在 TIMEOUT 时插回看 Debug COM"
    )
    print(f"  {Y}规则{N} 任何一步 TIMEOUT 立即停下，按 RESET 复活后再决定下一步")
    print("  按回车开始 / Ctrl+C 退出")
    try:
        input()
    except KeyboardInterrupt:
        sys.exit(0)


def main():
    banner()
    s = open_port()
    results = []

    try:
        # Test 1-2：NACK 路径，不动舵机，不需通电
        results.append(
            ("Z (1B 非法)", run_step(s, "Test 1: Z\\n", b"Z\n", "NACK BADARG"))
        )
        results.append(
            ("HOMX (4B 非法)", run_step(s, "Test 2: HOMX\\n", b"HOMX\n", "NACK BADARG"))
        )

        # 后续命令会动舵机，确认通电
        print(f"\n{Y}=== 后续会让舵机动，请确认 12V 已通电 ==={N}")
        try:
            input("回车继续 / Ctrl+C 退出: ")
        except KeyboardInterrupt:
            print("\n用户中断")
            return

        # Test 3-5：单关节 / FSR / HOME
        results.append(
            (
                "OPEN",
                run_step(
                    s, "Test 3: OPEN\\n", b"OPEN\n", "READY", timeout=5.0, danger=True
                ),
            )
        )
        results.append(
            (
                "CLOSE",
                run_step(
                    s,
                    "Test 4: CLOSE\\n",
                    b"CLOSE\n",
                    ["READY", "NACK SAFETY"],
                    timeout=10.0,
                    danger=True,
                ),
            )
        )
        results.append(
            (
                "HOME",
                run_step(
                    s,
                    "Test 5: HOME\\n",
                    b"HOME\n",
                    "READY",
                    timeout=LONG_T,
                    danger=True,
                ),
            )
        )

        # Test 6: K + HOME 角度（不动，但走完 K handler 的 6 次 pv2_next_token + strtof）
        cmd6 = ("K " + " ".join(str(a) for a in HOME_ANGLES) + "\n").encode()
        results.append(
            (
                "K HOME",
                run_step(
                    s,
                    f"Test 6: K {' '.join(map(str, HOME_ANGLES))}\\n (6参数解析)",
                    cmd6,
                    "READY",
                    timeout=LONG_T,
                    danger=True,
                ),
            )
        )

        # Test 7: K 小幅偏移（夹爪 120→60），验证 K 真能驱动舵机
        offset = HOME_ANGLES.copy()
        offset[5] = 60
        cmd7 = ("K " + " ".join(str(a) for a in offset) + "\n").encode()
        results.append(
            (
                "K +gripper60",
                run_step(
                    s,
                    f"Test 7: K {' '.join(map(str, offset))}\\n (验证舵机响应)",
                    cmd7,
                    "READY",
                    timeout=LONG_T,
                    danger=True,
                ),
            )
        )

        # 收尾：回 HOME
        results.append(
            (
                "HOME 收尾",
                run_step(
                    s, "收尾: HOME\\n", b"HOME\n", "READY", timeout=LONG_T, danger=False
                ),
            )
        )

    except KeyboardInterrupt:
        print(f"\n{Y}用户 Ctrl+C 中断{N}")
    finally:
        s.close()

    # 汇总
    print(f"\n{B}=== 结果汇总 ==={N}")
    for name, (status, text) in results:
        color = {"PASS": G, "SKIP": Y, "TIMEOUT": R, "FAIL": R, "ERROR": R}.get(
            status, R
        )
        text_brief = text[:40].replace("\n", "\\n").replace("\r", "")
        print(f"  {name:18s} {color}{status:8s}{N}  {text_brief}")

    # 退出码
    bad = sum(1 for _, (st, _) in results if st in ("FAIL", "TIMEOUT", "ERROR"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
