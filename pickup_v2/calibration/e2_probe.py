"""E2 单点验证工具：给定臂基坐标 → φ_fixed IK → 舵机角 → K 命令发臂。

发完后人工量指尖实际落点 (X,Y,Z)，与目标对比，验收误差 < 8mm。
走直接 K，不依赖 cv2/标定相机，是检验 E1(关节标定)+E1.5(IK φ_fixed) 准不准的本体测试。

用法（在 Pi 上）：
    python e2_probe.py 250 0 97        # 打目标点 (250,0,97) mm
    python e2_probe.py 301.5 -5.3 97.4 # 草莓锚点
    python e2_probe.py --home          # 回 HOME

交互：打印目标/几何角/servo → 回车发送（手放急停旁）→ 量落点记差。
单点验证完务必 --home 收臂再下一个，避免大跳。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "pi"))

import config_v2 as cfg  # noqa: E402
from kinematics import (  # noqa: E402
    IKUnreachableError,
    angles_to_servo_degs,
    ik,
)
from protocol_v2 import ProtocolV2  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description="E2 单点实物验证")
    ap.add_argument("coord", nargs="*", type=float, help="X Y Z (mm, 臂基系)")
    ap.add_argument("--home", action="store_true", help="只回 HOME")
    args = ap.parse_args()

    with ProtocolV2() as proto:
        proto.drain()

        if args.home:
            proto.send_home()
            print("→ 已发 HOME")
            return

        if len(args.coord) != 3:
            ap.error("需要 3 个坐标值：X Y Z（或用 --home）")
        x, y, z = args.coord

        # 先 HOME：预热串口 + 让臂从已知姿态出发（防 MCU 卡死时 K 丢进黑洞）
        rh = proto.send_home()
        if not rh.ok:
            print(f"✗ HOME 未 READY（{rh.response!r}）—— 通信可能断了，先别发 K。")
            print("  排查：按 MCU 复位键、查供电/USB Debug/杜邦线，再重试。")
            return
        print("HOME ok，臂已归位 →")

        try:
            angles = ik(
                (x, y, z),
                gripper_deg=cfg.GRIPPER_OPEN_DEG,
                wrist_rotate_deg=cfg.WRIST_ROTATE_GRASP_DEG,
            )
        except IKUnreachableError as exc:
            print(f"✗ 目标不可达：{exc}")
            return

        servo = angles_to_servo_degs(angles)
        print(f"目标 (X,Y,Z) = ({x:.1f}, {y:.1f}, {z:.1f}) mm")
        print(
            f"几何角  base={angles.base:.1f} shoulder={angles.shoulder:.1f} "
            f"elbow={angles.elbow:.1f} wrist_pitch={angles.wrist_pitch:.1f}"
        )
        print(f"K servo (b,s,e,wp,wr,grip) = {[round(s, 1) for s in servo]}")

        try:
            input("回车发送 K（手放急停旁，Ctrl+C 取消）")
        except KeyboardInterrupt:
            print("\n[取消]")
            return

        r = proto.send_K(servo)
        print("响应:", "READY ✓" if r.ok else f"未 READY: {r.response!r}")
        print("→ 用卷尺/卡尺量指尖实际落点 (X,Y,Z)，记下与目标的差(<8mm 合格)")
        print("→ 测完用 `python e2_probe.py --home` 收臂")


if __name__ == "__main__":
    main()
