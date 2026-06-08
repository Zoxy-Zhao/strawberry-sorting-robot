"""pytest 配置 — 让测试能 import 上一级目录的 config_v2/kinematics/coord_transform/protocol_v2。

protocol_v2 在 Pi 上依赖 pyserial；PC 跑测试时用 stub 顶替（测试自己注入 FakeSerial）。
"""

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# pyserial stub — 仅当真实 pyserial 不可用时启用
try:
    import serial  # noqa: F401
except ImportError:
    stub = types.ModuleType("serial")

    class _StubSerial:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("pyserial 未安装 — 测试应注入 FakeSerial")

    stub.Serial = _StubSerial
    sys.modules["serial"] = stub
