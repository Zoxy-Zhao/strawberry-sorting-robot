"""逆运动学 + 正运动学 + 舵机映射。

约定（与 docs/坐标系与IK.md 一致）：
    α_s ∈ [0°, 180°]   肩相对水平面（0=水平朝外, 90=朝上）
    α_e ∈ [0°, 180°]   肘弯折角（0=伸直, 180=折成一点）— 小臂相对大臂顺时针旋转 α_e
    α_p ∈ [-90°, 90°]  腕俯仰相对小臂（0=共线, - = 末端下俯）

末端约束：末端以固定世界俯仰角 φ_fixed 抓取（默认 cfg.PHI_FIXED_DEG=-43°）。
        垂直向下(-90°)物理够不到传送带，改用固定斜角，详见标定演进复盘_2026-06-03.md。
        ⇒ α_s - α_e + α_p = φ  ⇒  α_p = φ - α_s + α_e（φ=-90° 退化为垂直向下原版）

舵机映射：
    servo_deg = 多点分段线性插值(geom_deg)  （含重力下垂补偿，见 config_v2.interp_geom_to_servo）
    pulse_us  = left_safe + span * servo_deg / max_angle  (non-reversed)
    pulse_us  = right_safe - span * servo_deg / max_angle (reversed)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import config_v2 as cfg


class IKUnreachableError(Exception):
    """目标超出工作空间或几何不可达。"""


@dataclass(frozen=True)
class GeomAngles:
    """六个关节的几何角度（度）— 不是舵机角度。"""

    base: float
    shoulder: float
    elbow: float
    wrist_pitch: float
    wrist_rotate: float
    gripper: float

    def to_list(self) -> list[float]:
        return [
            self.base,
            self.shoulder,
            self.elbow,
            self.wrist_pitch,
            self.wrist_rotate,
            self.gripper,
        ]


# ============================================================
# 1. 工作空间检查
# ============================================================


def _check_workspace(
    x: float, y: float, z: float, phi_deg: float = cfg.PHI_FIXED_DEG
) -> None:
    r = math.sqrt(x * x + y * y)
    # 腕俯仰轴位置：末端沿 φ 方向前伸 L_END，故腕轴 = 目标 - L_END·(cosφ, sinφ)
    phi_rad = math.radians(phi_deg)
    r_w = r - cfg.L_END * math.cos(phi_rad)
    z_w = z - cfg.L_END * math.sin(phi_rad)
    z_rel = z_w - cfg.H1
    d = math.sqrt(r_w * r_w + z_rel * z_rel)

    if d > cfg.L2 + cfg.L3:
        raise IKUnreachableError(
            f"d={d:.1f} > L2+L3={cfg.L2 + cfg.L3:.1f} (超出最大伸展)"
        )
    if d < abs(cfg.L2 - cfg.L3):
        raise IKUnreachableError(
            f"d={d:.1f} < |L2-L3|={abs(cfg.L2 - cfg.L3):.1f} (内部死区)"
        )
    if z < cfg.WORKSPACE_Z_MIN_MM:
        raise IKUnreachableError(f"z={z:.1f} < 安全下限 {cfg.WORKSPACE_Z_MIN_MM:.1f}")


def _safe_acos(x: float) -> float:
    """避免 acos 因浮点误差越界。"""
    return math.acos(max(-1.0, min(1.0, x)))


# ============================================================
# 2. 逆运动学（解析解）
# ============================================================


def ik(
    target_xyz: tuple[float, float, float],
    gripper_deg: float = cfg.GRIPPER_OPEN_DEG,
    wrist_rotate_deg: float = 0.0,
    phi_deg: float = cfg.PHI_FIXED_DEG,
) -> GeomAngles:
    """臂基坐标 (X, Y, Z) mm → 6 个几何角度（度）。

    末端约束：末端以固定世界俯仰角 φ_fixed 抓取目标点（默认 cfg.PHI_FIXED_DEG=-43°）。
    φ=-90° 退化为"垂直向下"原版。垂直向下物理够不到传送带，故改固定斜角。
    """
    x, y, z = target_xyz
    _check_workspace(x, y, z, phi_deg)

    # 步骤 1：底座旋转
    theta_base_rad = math.atan2(y, x)
    r = math.sqrt(x * x + y * y)

    # 步骤 2：腕俯仰轴目标位置（末端沿 φ 前伸 L_END → 腕轴 = 目标 - L_END·(cosφ, sinφ)）
    phi_rad = math.radians(phi_deg)
    r_w = r - cfg.L_END * math.cos(phi_rad)
    z_w = z - cfg.L_END * math.sin(phi_rad)

    # 步骤 3：肩-肘 平面 2-link IK（以肩轴为原点）
    r_rel = r_w
    z_rel = z_w - cfg.H1
    d = math.sqrt(r_rel * r_rel + z_rel * z_rel)

    cos_inner = (cfg.L2**2 + cfg.L3**2 - d * d) / (2 * cfg.L2 * cfg.L3)
    inner = _safe_acos(cos_inner)  # 大小臂之间的"内角" γ
    alpha_e_rad = math.pi - inner  # 几何肘弯折角（伸直=0, 折成一点=π）

    phi_dir = math.atan2(z_rel, r_rel)
    cos_beta = (cfg.L2**2 + d * d - cfg.L3**2) / (2 * cfg.L2 * d)
    beta = _safe_acos(cos_beta)
    alpha_s_rad = phi_dir + beta  # elbow-up 解：大臂在目标连线上方

    # 步骤 4：腕俯仰由末端约束补偿：end_world = α_s - α_e + α_p = φ
    alpha_p_rad = phi_rad - alpha_s_rad + alpha_e_rad

    return GeomAngles(
        base=math.degrees(theta_base_rad),
        shoulder=math.degrees(alpha_s_rad),
        elbow=math.degrees(alpha_e_rad),
        wrist_pitch=math.degrees(alpha_p_rad),
        wrist_rotate=wrist_rotate_deg,
        gripper=gripper_deg,
    )


# ============================================================
# 3. 正运动学（用于自验证）
# ============================================================


def fk(angles: GeomAngles) -> tuple[float, float, float]:
    """6 个几何角度（度）→ 末端指尖位置 (X, Y, Z) mm（臂基系）。

    只用 base/shoulder/elbow/wrist_pitch；wrist_rotate 和 gripper 不影响位置。
    """
    theta_base = math.radians(angles.base)
    alpha_s = math.radians(angles.shoulder)
    alpha_e = math.radians(angles.elbow)
    alpha_p = math.radians(angles.wrist_pitch)

    # 在底座 0° 旋转下的 r-z 平面（肩轴为原点）
    # 大臂端（肘轴）：
    r_elbow = cfg.L2 * math.cos(alpha_s)
    z_elbow = cfg.L2 * math.sin(alpha_s)

    # 小臂方向 = 大臂方向 - α_e（顺时针旋转 α_e）
    forearm_world = alpha_s - alpha_e
    r_wrist = r_elbow + cfg.L3 * math.cos(forearm_world)
    z_wrist = z_elbow + cfg.L3 * math.sin(forearm_world)

    # 末端方向 = 小臂方向 + α_p
    end_world = forearm_world + alpha_p
    r_tip = r_wrist + cfg.L_END * math.cos(end_world)
    z_tip = z_wrist + cfg.L_END * math.sin(end_world)

    # 加上 H1 偏移（肩轴高度）
    z_tip += cfg.H1

    # 应用底座旋转
    x = r_tip * math.cos(theta_base)
    y = r_tip * math.sin(theta_base)
    return x, y, z_tip


# ============================================================
# 4. 几何角 → 舵机角 → us
# ============================================================


def geom_to_servo_deg(joint: int, geom_deg: float) -> float:
    """几何角度 → 舵机角度（多点分段线性插值，含重力下垂补偿；不饱和）。"""
    return cfg.interp_geom_to_servo(joint, geom_deg)


def servo_deg_to_us(joint: int, servo_deg: float) -> int:
    """舵机角度 → 脉宽 us。与 hal_entry.c servo_angle_to_us 等价。"""
    p = cfg.SERVO_PARAMS[joint]
    max_a = p["max_angle_deg"]
    span = p["right_safe_us"] - p["left_safe_us"]
    a = max(0.0, min(float(max_a), servo_deg))
    if p["reversed"]:
        return int(round(p["right_safe_us"] - span * a / max_a))
    return int(round(p["left_safe_us"] + span * a / max_a))


def angles_to_servo_degs(angles: GeomAngles) -> list[float]:
    """6 个几何角度 → 6 个舵机角度（用于发送 K 命令）。会做范围饱和。"""
    geom_list = angles.to_list()
    out = []
    for joint, g in enumerate(geom_list):
        s = geom_to_servo_deg(joint, g)
        max_a = cfg.SERVO_PARAMS[joint]["max_angle_deg"]
        s_clamped = max(0.0, min(float(max_a), s))
        out.append(s_clamped)
    return out


# ============================================================
# 5. 安全检查（IK 求解后跑）
# ============================================================


def check_safety(angles: GeomAngles, prev_angles: GeomAngles | None = None) -> None:
    """关节限位 + 单调性检查；越限抛 IKUnreachableError。"""
    servo_degs = angles_to_servo_degs(angles)
    geom_list = angles.to_list()
    for joint, (g, s) in enumerate(zip(geom_list, servo_degs)):
        max_a = cfg.SERVO_PARAMS[joint]["max_angle_deg"]
        # 检查饱和前的 servo_deg 是否越界（即原始 servo_deg 与饱和后是否一致）
        s_raw = geom_to_servo_deg(joint, g)
        if not (-0.5 <= s_raw <= max_a + 0.5):
            raise IKUnreachableError(
                f"{cfg.JOINT_NAMES[joint]} servo_deg={s_raw:.1f} 超出 [0, {max_a}]"
            )

    if prev_angles is not None:
        prev_list = prev_angles.to_list()
        for joint, (cur, prev) in enumerate(zip(geom_list, prev_list)):
            if abs(cur - prev) > cfg.MAX_JOINT_DELTA_DEG:
                raise IKUnreachableError(
                    f"{cfg.JOINT_NAMES[joint]} 关节差 {abs(cur - prev):.1f}° "
                    f"> {cfg.MAX_JOINT_DELTA_DEG}°（防飞舵）"
                )
