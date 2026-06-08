"""protocol_v2 单元测试 — 用 FakeSerial 替代真实串口。"""

from __future__ import annotations

import pytest

import config_v2 as cfg
from protocol_v2 import ProtocolV2, _classify_response


# ============================================================
# FakeSerial：模拟 pyserial.Serial 的最小接口
# ============================================================


class FakeSerial:
    """实现 ProtocolV2 用到的全部 pyserial 接口。"""

    def __init__(self, scripted_responses: list[bytes] | None = None) -> None:
        self.is_open = True
        self.timeout = 0.2
        self.tx_log: list[bytes] = []  # 所有 write 的内容
        self._rx_queue = bytearray()
        if scripted_responses:
            for r in scripted_responses:
                self._rx_queue.extend(r)

    def write(self, data: bytes) -> int:
        self.tx_log.append(bytes(data))
        return len(data)

    def flush(self) -> None:
        pass

    def read(self, n: int = 1) -> bytes:
        if not self._rx_queue:
            return b""
        out = bytes(self._rx_queue[:n])
        del self._rx_queue[:n]
        return out

    @property
    def in_waiting(self) -> int:
        return len(self._rx_queue)

    def close(self) -> None:
        self.is_open = False

    def queue_response(self, data: bytes) -> None:
        self._rx_queue.extend(data)


def make_proto(scripted: list[bytes] | None = None) -> tuple[ProtocolV2, FakeSerial]:
    proto = ProtocolV2(port="fake", baudrate=115200)
    fake = FakeSerial(scripted)
    proto.ser = fake  # 绕开真正的 open()
    return proto, fake


# ============================================================
# _classify_response 纯函数
# ============================================================


class TestClassifyResponse:
    def test_ready_is_ok(self):
        r = _classify_response("READY")
        assert r.ok and not r.is_nack and not r.is_busy

    def test_belt_on_is_ok(self):
        assert _classify_response("BELT_ON").ok

    def test_belt_off_is_ok(self):
        assert _classify_response("BELT_OFF").ok

    def test_stopped_is_ok(self):
        assert _classify_response("STOPPED").ok

    def test_busy(self):
        r = _classify_response("BUSY")
        assert not r.ok and r.is_busy and not r.is_nack

    def test_nack_unreachable(self):
        r = _classify_response("NACK UNREACHABLE")
        assert not r.ok and r.is_nack

    def test_nack_safety(self):
        assert _classify_response("NACK SAFETY").is_nack

    def test_nack_badarg(self):
        assert _classify_response("NACK BADARG").is_nack

    def test_empty_is_timeout(self):
        r = _classify_response("")
        assert r.is_timeout and not r.ok

    def test_unknown_response(self):
        r = _classify_response("HELLO_WORLD")
        assert not r.ok and not r.is_nack and not r.is_busy


# ============================================================
# send_M
# ============================================================


class TestSendM:
    def test_normal_path(self):
        proto, fake = make_proto([b"READY\n"])
        result = proto.send_M(100.0, 50.0, 20.0)
        assert result.ok
        assert fake.tx_log == [b"M 100.0 50.0 20.0\n"]

    def test_format_one_decimal(self):
        proto, fake = make_proto([b"READY\n"])
        proto.send_M(123.456, -78.91, 0.0)
        assert fake.tx_log == [b"M 123.5 -78.9 0.0\n"]

    def test_x_too_large_raises(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError, match="M.x"):
            proto.send_M(400.0, 0.0, 0.0)

    def test_x_negative_raises(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError, match="M.x"):
            proto.send_M(-1.0, 0.0, 0.0)

    def test_y_too_large_raises(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError, match="M.y"):
            proto.send_M(100.0, 250.0, 0.0)

    def test_z_too_low_raises(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError, match="M.z"):
            proto.send_M(100.0, 0.0, -100.0)

    def test_nack_unreachable_returns_not_ok(self):
        proto, _ = make_proto([b"NACK UNREACHABLE\n"])
        r = proto.send_M(50.0, 50.0, 50.0)
        assert not r.ok and r.is_nack
        assert r.response == "NACK UNREACHABLE"

    def test_busy_returns_busy(self):
        proto, _ = make_proto([b"BUSY\n"])
        r = proto.send_M(50.0, 50.0, 50.0)
        assert not r.ok and r.is_busy


# ============================================================
# send_K
# ============================================================


class TestSendK:
    def test_six_joints_normal(self):
        proto, fake = make_proto([b"READY\n"])
        result = proto.send_K([135.0, 90.0, 70.0, 105.0, 90.0, 120.0])
        assert result.ok
        assert fake.tx_log == [b"K 135.0 90.0 70.0 105.0 90.0 120.0\n"]

    def test_wrong_count_raises(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError, match="6 个关节角"):
            proto.send_K([1.0, 2.0, 3.0])

    def test_seven_joints_raises(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError):
            proto.send_K([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

    def test_nack_safety(self):
        proto, _ = make_proto([b"NACK SAFETY\n"])
        r = proto.send_K([0, 0, 0, 0, 0, 0])
        assert r.is_nack


# ============================================================
# send_J
# ============================================================


class TestSendJ:
    def test_normal(self):
        proto, fake = make_proto([b"READY\n"])
        proto.send_J(5, 90.0)
        assert fake.tx_log == [b"J 5 90.0\n"]

    def test_channel_int_format(self):
        proto, fake = make_proto([b"READY\n"])
        proto.send_J(0, 45.5)
        assert fake.tx_log == [b"J 0 45.5\n"]

    def test_channel_too_large(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError, match="J.channel"):
            proto.send_J(6, 90.0)

    def test_channel_negative(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError):
            proto.send_J(-1, 90.0)


# ============================================================
# 关键字命令
# ============================================================


class TestKeywords:
    def test_open(self):
        proto, fake = make_proto([b"READY\n"])
        assert proto.send_open().ok
        assert fake.tx_log == [b"OPEN\n"]

    def test_close(self):
        proto, fake = make_proto([b"READY\n"])
        assert proto.send_close().ok
        assert fake.tx_log == [b"CLOSE\n"]

    def test_home(self):
        proto, fake = make_proto([b"READY\n"])
        assert proto.send_home().ok
        assert fake.tx_log == [b"HOME\n"]

    def test_place_a(self):
        proto, fake = make_proto([b"READY\n"])
        assert proto.send_place("A").ok
        assert fake.tx_log == [b"PLACE A\n"]

    def test_place_b(self):
        proto, fake = make_proto([b"READY\n"])
        proto.send_place("B")
        assert fake.tx_log == [b"PLACE B\n"]

    def test_place_invalid_raises(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError, match="A/B/C"):
            proto.send_place("D")

    def test_place_lowercase_raises(self):
        proto, _ = make_proto([])
        with pytest.raises(ValueError):
            proto.send_place("a")


# ============================================================
# 旧协议兼容
# ============================================================


class TestLegacy:
    def test_belt_on(self):
        proto, fake = make_proto([b"BELT_ON\n"])
        result = proto.send_belt_on()
        assert result.ok and result.response == "BELT_ON"
        assert fake.tx_log == [b"G\n"]

    def test_emergency_stop_stopped(self):
        proto, fake = make_proto([b"STOPPED\n"])
        result = proto.send_emergency_stop()
        assert result.ok
        assert fake.tx_log == [b"X\n"]

    def test_emergency_stop_belt_off(self):
        proto, _ = make_proto([b"BELT_OFF\n"])
        assert proto.send_emergency_stop().ok


# ============================================================
# 行结束符处理
# ============================================================


class TestLineEnding:
    def test_crlf_stripped(self):
        proto, _ = make_proto([b"READY\r\n"])
        assert proto.send_home().ok

    def test_lf_only(self):
        proto, _ = make_proto([b"READY\n"])
        assert proto.send_home().ok

    def test_multiple_writes_sequential(self):
        proto, fake = make_proto([b"READY\n", b"READY\n"])
        proto.send_home()
        proto.send_open()
        assert fake.tx_log == [b"HOME\n", b"OPEN\n"]


# ============================================================
# 超时
# ============================================================


class TestTimeout:
    def test_no_response_timeout(self, monkeypatch):
        """无任何响应 → is_timeout=True。"""
        proto, _ = make_proto([])  # 空响应队列
        # 把所有 ACK 超时改成 0.05s 以加速测试
        monkeypatch.setattr(cfg, "ACK_TIMEOUT_S", {k: 0.05 for k in cfg.ACK_TIMEOUT_S})
        result = proto.send_home()
        assert not result.ok and result.is_timeout


# ============================================================
# 连接管理
# ============================================================


class TestConnection:
    def test_send_without_open_raises(self):
        proto = ProtocolV2(port="fake")  # 不设 ser
        with pytest.raises(RuntimeError, match="串口未打开"):
            proto.send_home()

    def test_drain(self):
        proto, fake = make_proto([])
        fake.queue_response(b"garbage_data")
        proto.drain()
        assert fake.in_waiting == 0
