"""pickup_v2 全局配置 — 与 vision/pi/config.py 完全独立。

- 连杆参数（来自 docs/机械臂布局与参数.md）
- 舵机标定（来自 docs/舵机标定记录.csv 和 hal_entry.c 的 g_servos[]）
- 标定文件路径
- 工作空间与安全阈值

所有 IK / 坐标转换的常量都在这里集中，便于调优。
"""

from __future__ import annotations

from pathlib import Path

# ============================================================
# 1. 关节索引（与 hal_entry.c joint_id_t 对齐）
# ============================================================
JOINT_BASE = 0  # CH5 / 270deg
JOINT_SHOULDER = 1  # CH1 / 180deg
JOINT_ELBOW = 2  # CH0 / 180deg
JOINT_WRIST_PITCH = 3  # CH2 / 180deg
JOINT_WRIST_ROTATE = 4  # CH3 / 180deg
JOINT_GRIPPER = 5  # CH4 / 180deg / reversed

JOINT_NAMES = {
    JOINT_BASE: "base",
    JOINT_SHOULDER: "shoulder",
    JOINT_ELBOW: "elbow",
    JOINT_WRIST_PITCH: "wrist_pitch",
    JOINT_WRIST_ROTATE: "wrist_rotate",
    JOINT_GRIPPER: "gripper",
}

# ============================================================
# 2. 连杆参数（mm，2026-03-20 装臂后实测，覆盖 docs/机械臂布局与参数.md 图纸值）
# ============================================================
H1 = 100.0  # 底座顶面 → 肩轴
L2 = 115.0  # 肩轴 → 肘轴
L3 = 160.0  # 肘轴 → 腕俯仰轴
# L4/L5/LF 拆分是机械图纸估算，装臂后只实测了"腕俯仰 → 指尖 = 180mm"总长。
# IK / FK 实际只用 L_END，拆分按图纸比例缩放仅作参考。
L4 = 55.0  # 腕俯仰轴 → 腕旋转轴
L5 = 67.0  # 腕旋转轴 → 夹爪根部
LF = 58.0  # 夹爪根部 → 指尖中心（取均值）

# 末端等效连杆（夹爪垂直时，腕俯仰轴 → 指尖的总长，装臂实测 180mm）
L_END = L4 + L5 + LF  # = 180.0 mm

# ============================================================
# 3. 舵机标定（与 hal_entry.c g_servos[] 一致）
# 公式：pulse_us = left_safe + span * angle / max_angle  (non-reversed)
#       pulse_us = right_safe - span * angle / max_angle (reversed)
# 所以 angle 0 ↔ left_safe，angle max ↔ right_safe（reversed 反过来）
# ============================================================
SERVO_PARAMS = {
    JOINT_BASE: {
        "channel": 5,
        "max_angle_deg": 270,
        "left_safe_us": 500,
        "right_safe_us": 2750,
        "reversed": False,
    },
    JOINT_SHOULDER: {
        "channel": 1,
        "max_angle_deg": 180,
        "left_safe_us": 500,
        "right_safe_us": 2750,
        "reversed": False,
    },
    JOINT_ELBOW: {
        "channel": 0,
        "max_angle_deg": 180,
        "left_safe_us": 500,
        "right_safe_us": 2750,
        "reversed": False,
    },
    JOINT_WRIST_PITCH: {
        "channel": 2,
        "max_angle_deg": 180,
        "left_safe_us": 500,
        "right_safe_us": 2750,
        "reversed": False,
    },
    JOINT_WRIST_ROTATE: {
        "channel": 3,
        "max_angle_deg": 180,
        "left_safe_us": 500,
        "right_safe_us": 2750,
        "reversed": False,
    },
    JOINT_GRIPPER: {
        "channel": 4,
        "max_angle_deg": 180,
        "left_safe_us": 380,
        "right_safe_us": 2750,
        "reversed": True,
    },
}

# ============================================================
# 4. 几何角 → 舵机角 映射（OFFSET + SIGN）
# servo_deg = OFFSET + SIGN * geom_deg
#
# 首版采用"最简映射"，让几何角在 IK 输出范围内能完整映射到舵机有效区间：
#   - BASE:        几何 -135..+135 → 舵机 0..270   (OFFSET=135, SIGN=+1)
#   - SHOULDER:    几何 0..180     → 舵机 0..180   (OFFSET=0,   SIGN=+1)
#   - ELBOW:       几何 0..180     → 舵机 0..180   (OFFSET=0,   SIGN=+1)
#   - WRIST_PITCH: 几何 -90..+90   → 舵机 0..180   (OFFSET=90,  SIGN=+1)
#   - WRIST_ROTATE/GRIPPER: 不参与 IK，直接传值
#
# ⚠️ 这套初值能让单元测试（FK/IK 数学自洽）通过，但与现有 HOME / PRE_GRASP
#    舵机值并不一一对应。**阶段 D 联调时**用现有姿态做"反向标定"重写：
#       1. 用 K 命令把臂打到 HOME 舵机值 {135,70,70,150,80,120}
#       2. 量末端实际位置 (X, Y, Z)
#       3. 用 IK 算出对应几何角，再反推 OFFSET/SIGN
#    最终目标：让 ik(实测末端) = HOME 舵机值
# ============================================================
# 默认值（fallback）— 阶段 E 标定（joint_calib.py）写入的 yaml 优先生效。
# HOME 反推法不可行（HOME 不满足 IK 垂直约束），改用单关节扫描法，详见
# docs/关节标定SOP.md。
_JOINT_GEOM_OFFSET_DEFAULT = {
    JOINT_BASE: 135.0,
    JOINT_SHOULDER: 25.0,
    JOINT_ELBOW: 0.0,
    JOINT_WRIST_PITCH: 90.0,
    JOINT_WRIST_ROTATE: 0.0,
    JOINT_GRIPPER: 0.0,
}

_JOINT_GEOM_SIGN_DEFAULT = {
    JOINT_BASE: +1,
    JOINT_SHOULDER: +1,
    JOINT_ELBOW: +1,
    JOINT_WRIST_PITCH: -1,
    JOINT_WRIST_ROTATE: +1,
    JOINT_GRIPPER: +1,
}

JOINT_OFFSETS_PATH = (
    Path(__file__).resolve().parent.parent
    / "calibration"
    / "outputs"
    / "joint_offsets.yaml"
)

_NAME_TO_ID = {
    "base": JOINT_BASE,
    "shoulder": JOINT_SHOULDER,
    "elbow": JOINT_ELBOW,
    "wrist_pitch": JOINT_WRIST_PITCH,
    "wrist_rotate": JOINT_WRIST_ROTATE,
    "gripper": JOINT_GRIPPER,
}


def _default_points() -> dict[int, list[tuple[float, float]]]:
    """由默认 offset+sign 构造每关节两点（线性），无 yaml 时回落。"""
    return {
        jid: [
            (0.0, _JOINT_GEOM_OFFSET_DEFAULT[jid]),
            (
                90.0,
                _JOINT_GEOM_OFFSET_DEFAULT[jid] + 90.0 * _JOINT_GEOM_SIGN_DEFAULT[jid],
            ),
        ]
        for jid in _JOINT_GEOM_OFFSET_DEFAULT
    }


def _interp_points(pts: list[tuple[float, float]], geom: float) -> float:
    """对一组 (geom, servo) 标定点做分段线性插值；区间外用端段斜率外推。"""
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


def _load_calib_points() -> dict[int, list[tuple[float, float]]]:
    """加载每关节标定点。优先级：yaml points: > yaml offsets/signs(两点) > 默认两点。"""
    points = _default_points()
    if not JOINT_OFFSETS_PATH.exists():
        return points
    try:
        import yaml  # 延迟 import，避免 pytest 环境强依赖

        data = yaml.safe_load(JOINT_OFFSETS_PATH.read_text(encoding="utf-8")) or {}
    except Exception as e:  # pragma: no cover
        print(f"[config_v2] 加载 {JOINT_OFFSETS_PATH} 失败: {e}；用默认值")
        return points
    pts_yaml = data.get("points", {})
    off_yaml = data.get("offsets", {})
    sgn_yaml = data.get("signs", {})
    for name, jid in _NAME_TO_ID.items():
        if name in pts_yaml and len(pts_yaml[name]) >= 2:
            points[jid] = sorted((float(g), float(s)) for g, s in pts_yaml[name])
        elif name in off_yaml and name in sgn_yaml:  # 兼容旧单斜率格式
            off, sgn = float(off_yaml[name]), float(sgn_yaml[name])
            points[jid] = [(0.0, off), (90.0, off + 90.0 * sgn)]
    return points


JOINT_CALIB_POINTS = _load_calib_points()


def interp_geom_to_servo(jid: int, geom: float) -> float:
    """几何角 → 舵机角：按该关节标定点分段线性插值（含重力下垂非线性）。"""
    return _interp_points(JOINT_CALIB_POINTS[jid], geom)


# 兼容旧读取（如 test_kinematics 的反推打印）：保留 offset/sign 派生值
JOINT_GEOM_OFFSET_DEG = {
    jid: interp_geom_to_servo(jid, 0.0) for jid in JOINT_CALIB_POINTS
}
JOINT_GEOM_SIGN = {
    jid: interp_geom_to_servo(jid, 1.0) - JOINT_GEOM_OFFSET_DEG[jid]
    for jid in JOINT_CALIB_POINTS
}

# ============================================================
# 6. 工作空间与安全阈值
# ============================================================
# IK 工作空间硬限位
WORKSPACE_R_MAX_MM = L2 + L3 - 5.0  # 留 5mm 安全裕度
WORKSPACE_R_MIN_MM = abs(L2 - L3) + 5.0
WORKSPACE_Z_MIN_MM = -H1 + 10.0  # 不低于地面（H1 以下不安全）

# 末端固定俯仰角 φ_fixed（世界系，0=水平朝外，-90=垂直向下）。
# 垂直向下物理够不到传送带，2026-06-03 实测+roundtrip 双确认改用固定斜角 -43°。
# 详见 docs/标定演进复盘_2026-06-03.md；IK 默认用此值。
PHI_FIXED_DEG = -43.0

# 单步关节角变化最大值（防飞舵）
MAX_JOINT_DELTA_DEG = 60.0

# 默认夹爪角度
GRIPPER_OPEN_DEG = 120
GRIPPER_CLOSE_DEG = 65

# 抓取时腕部旋转(roll)的中性几何角。wrist_rotate 未单独标定(恒等映射)，
# HOME 与已验证抓取姿态的 roll 都在 servo80，故抓取统一用此值，避免 IK 默认 0 把夹爪转偏。
WRIST_ROTATE_GRASP_DEG = 80.0

# ============================================================
# 7. 标定文件路径
# ============================================================
CALIB_DIR = Path(__file__).resolve().parent.parent / "calibration" / "outputs"
INTRINSICS_PATH = CALIB_DIR / "intrinsics.yaml"
HOMOGRAPHY_PATH = CALIB_DIR / "homography.yaml"
ARM_OFFSET_PATH = CALIB_DIR / "arm_offset.yaml"

# ============================================================
# 8. 串口（默认与现有保持一致；新主程序可覆盖）
# ============================================================
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 5.0

# ============================================================
# 9. 协议超时
# ============================================================
ACK_TIMEOUT_S = {
    "M": 8.0,  # 移臂
    "K": 8.0,
    "J": 5.0,
    "OPEN": 3.0,
    "CLOSE": 3.0,
    "HOME": 5.0,
    "PLACE": 8.0,
    "G": 2.0,
    "X": 2.0,
}
