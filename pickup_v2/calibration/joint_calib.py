"""单关节 OFFSET/SIGN 交互式标定脚本。

详细流程见 docs/关节标定SOP.md。在 Pi 上运行：

    cd ~/strawberry_grasp/pickup_v2/calibration
    python joint_calib.py base           # 标某个关节
    python joint_calib.py all            # 依次标 base→shoulder→elbow→wrist_pitch
    python joint_calib.py merge          # 合并所有 yaml → joint_offsets.yaml
    python joint_calib.py verify         # 用合并结果跑几个测试姿态

每个关节做两点法线性拟合：servo_deg = OFFSET + SIGN * geom_deg。
"""

from __future__ import annotations

import argparse
import datetime
import logging
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "pi"))

import config_v2 as cfg  # noqa: E402


def _import_proto():
    """延迟 import ProtocolV2，避免 PC 端（无 pyserial）跑 merge/--help 时挂。"""
    from protocol_v2 import ProtocolV2  # noqa: E402

    return ProtocolV2


logging.basicConfig(level=logging.WARNING, format="%(levelname)s | %(message)s")

OUTPUT_DIR = _HERE / "outputs"
HOME_SERVO = {
    cfg.JOINT_BASE: 135.0,
    cfg.JOINT_SHOULDER: 70.0,
    cfg.JOINT_ELBOW: 70.0,
    cfg.JOINT_WRIST_PITCH: 150.0,
    cfg.JOINT_WRIST_ROTATE: 80.0,
    cfg.JOINT_GRIPPER: 120.0,
}


@dataclass(frozen=True)
class CalibPlan:
    """一个关节的标定方案。"""

    joint_id: int
    joint_name: str
    start_servo: float  # 微调起点
    target_geoms_deg: list[float]  # 要采的几何角顺序
    # 标定该关节时，前序关节锁到的几何角（None = 直接用 HOME servo）
    prereq_geom: dict[int, float | None]
    hint: str  # 给用户的提示语


PLANS: dict[str, CalibPlan] = {
    "base": CalibPlan(
        joint_id=cfg.JOINT_BASE,
        joint_name="base",
        start_servo=135.0,
        target_geoms_deg=[0.0, 45.0],
        prereq_geom={},
        hint="把臂从俯视方向对齐桌面参考线。θ=0° 正前方，θ=+45° 左前 45°。",
    ),
    "shoulder": CalibPlan(
        joint_id=cfg.JOINT_SHOULDER,
        joint_name="shoulder",
        start_servo=80.0,
        # 多点覆盖操作区间 53~111°（重力下垂非线性，2 点线性不够）
        target_geoms_deg=[50.0, 70.0, 90.0, 110.0],
        prereq_geom={cfg.JOINT_BASE: 0.0},
        hint="水平仪贴大臂，读相对水平面角度。0=水平外伸，90=竖直朝上。",
    ),
    "elbow": CalibPlan(
        joint_id=cfg.JOINT_ELBOW,
        joint_name="elbow",
        start_servo=130.0,
        # 多点覆盖操作区间 37~112°
        target_geoms_deg=[40.0, 65.0, 90.0, 115.0],
        prereq_geom={cfg.JOINT_BASE: 0.0, cfg.JOINT_SHOULDER: 60.0},
        hint="角度尺卡大臂↔小臂相对角。0=共线伸直，90=直角，越大越折。",
    ),
    "wrist_pitch": CalibPlan(
        joint_id=cfg.JOINT_WRIST_PITCH,
        joint_name="wrist_pitch",
        start_servo=90.0,
        target_geoms_deg=[0.0, -45.0],
        prereq_geom={
            cfg.JOINT_BASE: 0.0,
            cfg.JOINT_SHOULDER: 30.0,
            cfg.JOINT_ELBOW: 30.0,
        },
        hint="α_p=0° 末端与小臂共线；α_p=-45° 末端相对小臂下俯 45°。",
    ),
}


def _interp_points(pts: list[tuple[float, float]], geom: float) -> float:
    """分段线性插值（与 config_v2._interp_points 一致），区间外端段斜率外推。"""
    pts = sorted(pts)
    if len(pts) == 1:
        return pts[0][1]
    if geom <= pts[0][0]:
        (g0, s0), (g1, s1) = pts[0], pts[1]
    elif geom >= pts[-1][0]:
        (g0, s0), (g1, s1) = pts[-2], pts[-1]
    else:
        i = 0
        while i < len(pts) - 1 and not (pts[i][0] <= geom <= pts[i + 1][0]):
            i += 1
        (g0, s0), (g1, s1) = pts[i], pts[i + 1]
    return s0 + (s1 - s0) * (geom - g0) / (g1 - g0)


def _load_prereq_servo(joint_id: int, target_geom: float) -> float:
    """读已标定关节 yaml，分段插值算 servo；找不到就用 HOME。"""
    name = {v.joint_id: k for k, v in PLANS.items()}[joint_id]
    p = OUTPUT_DIR / f"joint_calib_{name}.yaml"
    if p.exists():
        data = yaml.safe_load(p.read_text(encoding="utf-8"))
        pts = data.get("calibration_points")
        if pts and len(pts) >= 2:
            return _interp_points(
                [(d["geom_deg"], d["servo_deg"]) for d in pts], target_geom
            )
        return data["offset_deg"] + data["sign"] * target_geom  # 兼容旧单斜率
    fallback = HOME_SERVO[joint_id]
    print(f"  ⚠ {name} 尚未标定，前序关节用 HOME servo={fallback}（精度可能受影响）")
    return fallback


def _build_six_servos(plan: CalibPlan, current_target_servo: float) -> list[float]:
    """根据 plan + 当前关节扫描值，组装 6 个 servo 值（K 命令用）。

    K 命令顺序：(base, shoulder, elbow, wrist_pitch, wrist_rotate, gripper)。
    """
    servos = dict(HOME_SERVO)
    for jid, geom in plan.prereq_geom.items():
        servos[jid] = (
            _load_prereq_servo(jid, geom) if geom is not None else HOME_SERVO[jid]
        )
    servos[plan.joint_id] = current_target_servo
    return [servos[i] for i in range(6)]


def _interactive_tune(
    proto: ProtocolV2, plan: CalibPlan, target_geom: float, start_servo: float
) -> float | None:
    """让用户用 +/- 微调到目标几何角；返回最终 servo（None = skip/abort）。"""
    print(f"\n── 目标：{plan.joint_name} 几何角 = {target_geom:+.1f}°")
    print(f"   {plan.hint}")
    print("   命令：+1 / -1 / +5 / -5 / =<num> / done / skip / abort")

    cur = float(start_servo)
    _send_six(proto, plan, cur)

    while True:
        cmd = input(f"  servo={cur:6.1f} > ").strip().lower()
        if cmd == "done":
            return cur
        if cmd == "skip":
            return None
        if cmd == "abort":
            print("  ⚠ abort：发急停 X")
            proto.send_emergency_stop()
            raise KeyboardInterrupt()
        new = _parse_step(cmd, cur)
        if new is None:
            print("  ?? 不认识，再来")
            continue
        max_a = cfg.SERVO_PARAMS[plan.joint_id]["max_angle_deg"]
        new = max(0.0, min(float(max_a), new))
        if abs(new - cur) > 5.0:
            print(f"  ⚠ 单步 {abs(new - cur):.1f}° > 5°，拆分发送")
        cur = new
        _send_six(proto, plan, cur)


def _parse_step(cmd: str, cur: float) -> float | None:
    if cmd.startswith("="):
        try:
            return float(cmd[1:])
        except ValueError:
            return None
    table = {"+1": 1.0, "-1": -1.0, "+5": 5.0, "-5": -5.0}
    if cmd in table:
        return cur + table[cmd]
    return None


def _send_six(proto: ProtocolV2, plan: CalibPlan, target_servo: float) -> None:
    six = _build_six_servos(plan, target_servo)
    r = proto.send_K(six)
    if not r.ok:
        print(f"  ⚠ K 命令未 READY：{r.response!r}")


def calibrate_joint(proto: ProtocolV2, plan: CalibPlan) -> dict | None:
    """跑完一个关节的所有标定点，存全部 (geom, servo) 点（多点分段插值用）。"""
    print(f"\n══════════ 标定 {plan.joint_name} (joint_id={plan.joint_id}) ══════════")
    points: list[tuple[float, float]] = []
    start = plan.start_servo
    for tg in plan.target_geoms_deg:
        s = _interactive_tune(proto, plan, tg, start)
        if s is None:
            print(f"  ⚠ 跳过 {plan.joint_name}")
            return None
        points.append((tg, s))
        start = s
    pts = sorted(points)
    # 相邻点局部斜率，供观察非线性（理想≈1.0，<1 多为重力下垂）
    seg = [
        f"{g0:.0f}->{g1:.0f}:{(s1 - s0) / (g1 - g0):+.2f}"
        for (g0, s0), (g1, s1) in zip(pts, pts[1:])
    ]
    print(f"\n  标定点 {[(round(g), round(s)) for g, s in pts]}")
    print(f"  分段斜率 {seg}")
    return {
        "joint": plan.joint_name,
        "joint_id": plan.joint_id,
        "n_points": len(pts),
        "calibration_points": [{"geom_deg": g, "servo_deg": s} for g, s in pts],
        "calibrated_at": datetime.datetime.now().isoformat(timespec="seconds"),
    }


def save_joint_yaml(data: dict) -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    p = OUTPUT_DIR / f"joint_calib_{data['joint']}.yaml"
    p.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"  ✓ 已保存 {p}")
    return p


def merge_results(out_path: Path) -> None:
    points_out, offsets, signs = {}, {}, {}
    for name in PLANS:
        p = OUTPUT_DIR / f"joint_calib_{name}.yaml"
        if not p.exists():
            print(f"  ⚠ 缺少 {p.name}，跳过")
            continue
        d = yaml.safe_load(p.read_text(encoding="utf-8"))
        pts = d.get("calibration_points")
        if pts and len(pts) >= 2:
            sp = sorted((pt["geom_deg"], pt["servo_deg"]) for pt in pts)
        else:  # 兼容旧单斜率格式：构造两点
            off, sgn = float(d["offset_deg"]), float(d["sign"])
            sp = [(0.0, off), (90.0, off + 90.0 * sgn)]
        points_out[name] = [[g, s] for g, s in sp]
        # 派生 offset/sign（0° 截距 + 0° 附近局部斜率），供人读 + 旧 config 兼容
        offsets[name] = round(_interp_points(sp, 0.0), 3)
        signs[name] = round(_interp_points(sp, 1.0) - _interp_points(sp, 0.0), 4)
    merged = {
        "points": points_out,  # 主数据：每关节 [[geom,servo],...]，分段线性插值
        "offsets": offsets,  # 派生兼容字段
        "signs": signs,
        "calibrated_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "note": "由 joint_calib.py merge 生成（多点分段线性）；config_v2 优先用 points",
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        yaml.safe_dump(merged, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"  ✓ 合并写入 {out_path}")


def run_calibration(joint_names: list[str]) -> None:
    ProtocolV2 = _import_proto()
    with ProtocolV2() as proto:
        proto.drain()
        proto.send_home()
        for name in joint_names:
            plan = PLANS[name]
            data = calibrate_joint(proto, plan)
            if data is not None:
                save_joint_yaml(data)
        print("\n标定流程结束。运行 `python joint_calib.py merge` 合并结果。")


def verify() -> None:
    """用 merged yaml 算几个测试姿态并发 K 验证（目测对比）。"""
    merged_p = OUTPUT_DIR / "joint_offsets.yaml"
    if not merged_p.exists():
        print(f"  ⚠ 找不到 {merged_p}，先跑 merge")
        return
    m = yaml.safe_load(merged_p.read_text(encoding="utf-8"))
    offsets, signs = m["offsets"], m["signs"]
    ProtocolV2 = _import_proto()

    def to_servo(name: str, geom: float) -> float:
        return offsets[name] + signs[name] * geom

    tests = [
        (
            "大臂水平+小臂共线",
            {"base": 0.0, "shoulder": 0.0, "elbow": 0.0, "wrist_pitch": 0.0},
        ),
        (
            "大臂朝上 45+小臂下折 45",
            {"base": 0.0, "shoulder": 45.0, "elbow": 45.0, "wrist_pitch": 0.0},
        ),
        (
            "末端垂直朝下",
            {"base": 0.0, "shoulder": 60.0, "elbow": 120.0, "wrist_pitch": -30.0},
        ),
    ]
    with ProtocolV2() as proto:
        proto.drain()
        for desc, geoms in tests:
            six = [
                to_servo("base", geoms["base"]),
                to_servo("shoulder", geoms["shoulder"]),
                to_servo("elbow", geoms["elbow"]),
                to_servo("wrist_pitch", geoms["wrist_pitch"]),
                HOME_SERVO[cfg.JOINT_WRIST_ROTATE],
                HOME_SERVO[cfg.JOINT_GRIPPER],
            ]
            print(f"\n>>> {desc}  servo={['%.1f' % s for s in six]}")
            input("    回车发送（Ctrl+C 中止）")
            proto.send_K(six)
            input("    回车继续下一个")
        proto.send_home()


def main() -> None:
    parser = argparse.ArgumentParser(description="pickup_v2 关节 OFFSET/SIGN 标定")
    parser.add_argument(
        "mode",
        choices=list(PLANS.keys()) + ["all", "merge", "verify"],
        help="要执行的标定阶段",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=OUTPUT_DIR / "joint_offsets.yaml",
        help="merge 模式的输出路径",
    )
    args = parser.parse_args()

    try:
        if args.mode == "merge":
            merge_results(args.out)
        elif args.mode == "verify":
            verify()
        elif args.mode == "all":
            run_calibration(list(PLANS.keys()))
        else:
            run_calibration([args.mode])
    except KeyboardInterrupt:
        print("\n[中止]")


if __name__ == "__main__":
    main()
