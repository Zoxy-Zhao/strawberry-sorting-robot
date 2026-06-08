"""pickup_v2 串口协议封装 — 实现 docs/协议规范.md。

设计要点：
- 严格请求-响应：每条命令必须读到 READY / NACK / BUSY / BELT_ON / BELT_OFF / STOPPED 才返回。
- 行结束符：所有命令带 ``\\n`` 结尾；MCU 端忽略 ``\\r``。
- 超时：从 config_v2.ACK_TIMEOUT_S 取每个命令的超时阈值。超时由调用方决定是否急停。
- 范围预检：M / K / J 在发送前本地检查参数，越界直接抛 ValueError 不发命令。
- 不直接 import vision/pi/serial_comm（隔离），但 pyserial 用法保持一致。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import serial

import config_v2 as cfg

logger = logging.getLogger(__name__)


# ============================================================
# 响应分类
# ============================================================

# 终结响应（一行）— 收到任意一个就视为命令完成
TERMINAL_OK = frozenset({"READY", "BELT_ON", "BELT_OFF", "STOPPED"})
TERMINAL_BUSY = "BUSY"
TERMINAL_NACK_PREFIX = "NACK"


@dataclass(frozen=True)
class CommandResult:
    """单次命令的结果。"""

    ok: bool  # True 表示收到 READY / BELT_ON / BELT_OFF / STOPPED
    response: str  # 原始响应行（去掉 \r\n）
    is_busy: bool = False  # BUSY
    is_nack: bool = False  # NACK ...
    is_timeout: bool = False  # 本地读取超时


# ============================================================
# 参数范围预检（与协议规范.md 第 7 节一致）
# ============================================================

_M_X_RANGE = (0.0, 320.0)
_M_Y_RANGE = (-200.0, 200.0)
_M_Z_RANGE = (-50.0, 200.0)
_J_CHANNEL_RANGE = (0, 5)
_PLACE_LABELS = frozenset({"A", "B", "C"})


def _check_range(name: str, value: float, lo: float, hi: float) -> None:
    if not (lo <= value <= hi):
        raise ValueError(f"{name}={value} 超出范围 [{lo}, {hi}]")


# ============================================================
# 主类
# ============================================================


class ProtocolV2:
    """与 hal_entry_pickup_v2.c 配对的 Pi 端协议封装。"""

    def __init__(
        self,
        port: str | None = None,
        baudrate: int | None = None,
        byte_timeout_s: float = 0.2,
    ) -> None:
        self.port = port or cfg.SERIAL_PORT
        self.baudrate = baudrate or cfg.BAUD_RATE
        # 注意：byte_timeout_s 是 pyserial 的 read() 字节级超时；
        # 行级超时由 wait_response 自己用 monotonic 循环控制。
        self.byte_timeout_s = byte_timeout_s
        self.ser: serial.Serial | None = None

    # ── 连接管理 ────────────────────────────────────────────
    def open(self) -> None:
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.byte_timeout_s,
        )
        logger.info("ProtocolV2 串口已打开: %s @ %d", self.port, self.baudrate)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("ProtocolV2 串口已关闭")

    def __enter__(self) -> "ProtocolV2":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    # ── 底层收发 ────────────────────────────────────────────
    def _ensure_open(self) -> serial.Serial:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口未打开 — 请先 open() 或用 with 语句")
        return self.ser

    def drain(self) -> None:
        """清空输入缓冲（开始前调用以丢弃 MCU 残留输出）。"""
        ser = self._ensure_open()
        if ser.in_waiting > 0:
            discarded = ser.read(ser.in_waiting)
            logger.debug("丢弃残留 %d 字节", len(discarded))

    def _send_line(self, line: str) -> None:
        ser = self._ensure_open()
        if not line.endswith("\n"):
            line = line + "\n"
        ser.write(line.encode("ascii"))
        ser.flush()
        logger.info("TX → MCU: %s", line.rstrip())

    def wait_response(self, timeout_s: float) -> CommandResult:
        """阻塞读一行响应。整体超时由本函数控制，pyserial 内部用 byte_timeout 轮询。"""
        ser = self._ensure_open()
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = ser.read(1)
            if not chunk:
                continue
            if chunk == b"\r":
                continue
            if chunk == b"\n":
                text = buf.decode("utf-8", errors="replace").strip()
                logger.info("RX ← MCU: %s", text)
                return _classify_response(text)
            buf.extend(chunk)

        # 超时
        partial = buf.decode("utf-8", errors="replace").strip()
        logger.warning("响应超时（已读 %d 字节: %r）", len(buf), partial)
        return CommandResult(
            ok=False,
            response=partial,
            is_timeout=True,
        )

    def _send_and_wait(self, line: str, kind: str) -> CommandResult:
        timeout = cfg.ACK_TIMEOUT_S.get(kind, 5.0)
        self._send_line(line)
        return self.wait_response(timeout)

    # ── 命令封装 ────────────────────────────────────────────
    def send_M(self, x: float, y: float, z: float) -> CommandResult:
        """移臂到工作面坐标 (X, Y, Z) mm。Pi 端只发坐标，由 MCU 内部转 IK 或直接拒绝。"""
        _check_range("M.x", x, *_M_X_RANGE)
        _check_range("M.y", y, *_M_Y_RANGE)
        _check_range("M.z", z, *_M_Z_RANGE)
        return self._send_and_wait(f"M {x:.1f} {y:.1f} {z:.1f}", "M")

    def send_K(self, joints_servo_deg: list[float]) -> CommandResult:
        """直接发 6 个舵机角度 (θ5, θ1, θ0, θ2, θ3, θ4)。范围由 MCU 端 clamp。"""
        if len(joints_servo_deg) != 6:
            raise ValueError(f"K 命令需要 6 个关节角，收到 {len(joints_servo_deg)} 个")
        # 协议规范说 K 是 (θ5, θ1, θ0, θ2, θ3, θ4) 顺序 —
        # 调用方负责按这个顺序传入。这里只做格式化。
        line = "K " + " ".join(f"{a:.1f}" for a in joints_servo_deg)
        return self._send_and_wait(line, "K")

    def send_J(self, channel: int, servo_deg: float) -> CommandResult:
        """单关节调试。channel 0~5 = PCA9685 通道号。"""
        _check_range("J.channel", channel, *_J_CHANNEL_RANGE)
        return self._send_and_wait(f"J {int(channel)} {servo_deg:.1f}", "J")

    def send_open(self) -> CommandResult:
        return self._send_and_wait("OPEN", "OPEN")

    def send_close(self) -> CommandResult:
        return self._send_and_wait("CLOSE", "CLOSE")

    def send_home(self) -> CommandResult:
        return self._send_and_wait("HOME", "HOME")

    def send_place(self, label: str) -> CommandResult:
        if label not in _PLACE_LABELS:
            raise ValueError(f"PLACE 只支持 A/B/C，收到 {label!r}")
        return self._send_and_wait(f"PLACE {label}", "PLACE")

    def send_belt_on(self) -> CommandResult:
        """旧协议兼容：启动传送带。期待 BELT_ON。"""
        return self._send_and_wait("G", "G")

    def send_emergency_stop(self) -> CommandResult:
        """急停。期待 STOPPED 或 BELT_OFF。"""
        return self._send_and_wait("X", "X")


# ============================================================
# 内部工具
# ============================================================


def _classify_response(text: str) -> CommandResult:
    if not text:
        return CommandResult(ok=False, response=text, is_timeout=True)
    if text in TERMINAL_OK:
        return CommandResult(ok=True, response=text)
    if text == TERMINAL_BUSY:
        return CommandResult(ok=False, response=text, is_busy=True)
    if text.startswith(TERMINAL_NACK_PREFIX):
        return CommandResult(ok=False, response=text, is_nack=True)
    # 未知响应（例如 MCU 端的调试 log 串到了 SCI9 — 不应该发生）
    logger.warning("未识别响应: %r", text)
    return CommandResult(ok=False, response=text)
