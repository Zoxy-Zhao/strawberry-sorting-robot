"""kinematics.py 单元测试 — 阶段 B 验收依据。

核心测试：
    1. FK(IK(target)) ≈ target — 9 网格点 + 已知姿态
    2. IKUnreachableError — 工作空间外 / 内部死区 / 地面下
    3. 数值稳定 — cos 越界、目标在边界
    4. servo_deg_to_us — 关节角 0/max → left_safe/right_safe（reversed 反过来）
    5. 安全检查 — 关节限位、单调性
"""

from __future__ import annotations

import math

import pytest

import config_v2 as cfg
from kinematics import (
    GeomAngles,
    IKUnreachableError,
    check_safety,
    fk,
    ik,
    servo_deg_to_us,
)


# ============================================================
# 1. FK / IK 自洽（关键验收）
# ============================================================


class TestFKIKRoundtrip:
    """fk(ik(target)) 应回到 target，误差 < 0.1 mm。"""

    # 目标取草莓实际操作区（r≈220~340, z≈70~110），φ_fixed=-43° 下均可达
    @pytest.mark.parametrize(
        "target",
        [
            (250, 0, 97),
            (280, 40, 97),
            (280, -40, 97),
            (300, 0, 80),
            (240, 60, 100),
            (240, -60, 100),
            (320, 0, 90),
            (260, 30, 110),
            (270, -30, 70),
        ],
    )
    def test_roundtrip_grid(self, target):
        joints = ik(target)
        recovered = fk(joints)
        assert math.isclose(recovered[0], target[0], abs_tol=0.1), (
            f"X: {recovered[0]:.3f} vs {target[0]}"
        )
        assert math.isclose(recovered[1], target[1], abs_tol=0.1), (
            f"Y: {recovered[1]:.3f} vs {target[1]}"
        )
        assert math.isclose(recovered[2], target[2], abs_tol=0.1), (
            f"Z: {recovered[2]:.3f} vs {target[2]}"
        )


class TestPhiFixed:
    """末端固定俯仰角 φ_fixed：IK 解出的末端世界朝向应恒等于 φ。"""

    @pytest.mark.parametrize(
        "target", [(250, 0, 97), (280, 40, 97), (300, 0, 80), (240, -60, 100)]
    )
    def test_end_orientation_equals_phi(self, target):
        """end_world = α_s - α_e + α_p 应等于 cfg.PHI_FIXED_DEG。"""
        j = ik(target)
        end_world = j.shoulder - j.elbow + j.wrist_pitch
        assert math.isclose(end_world, cfg.PHI_FIXED_DEG, abs_tol=1e-6), (
            f"end_world={end_world:.3f} != φ_fixed={cfg.PHI_FIXED_DEG}"
        )

    def test_custom_phi_overrides_default(self):
        """显式传 phi_deg=-90 退化为垂直向下，末端朝向应为 -90。"""
        j = ik((200, 0, -50), phi_deg=-90.0)
        end_world = j.shoulder - j.elbow + j.wrist_pitch
        assert math.isclose(end_world, -90.0, abs_tol=1e-6)

    def test_real_strawberry_pose(self):
        """已确认能夹取的草莓点 → 还原真实姿态 sh≈85/el≈83/wp≈-45。"""
        j = ik((301.5, -5.3, 97.4))
        assert math.isclose(j.shoulder, 85.0, abs_tol=1.0)
        assert math.isclose(j.elbow, 83.0, abs_tol=1.5)
        assert math.isclose(j.wrist_pitch, -45.0, abs_tol=1.0)


# ============================================================
# 2. 工作空间 / 越界检查
# ============================================================


class TestUnreachable:
    def test_too_far(self):
        """目标超出最大伸展距离。"""
        with pytest.raises(IKUnreachableError, match="超出最大伸展"):
            ik((500, 0, 0))  # 远超 L2+L3+L_END

    def test_too_close_internal_deadzone(self):
        """目标过近落入内部死区 d < |L2-L3|=45mm。"""
        # φ=-43° 下腕轴 = 目标 - L_END·(cosφ,sinφ) ≈ (r-131.6, z+122.8)。
        # 取 r=130,z=-23 → 腕轴≈(-1.6,99.8)，z_rel≈-0.2，d≈1.7 < 45 → 死区
        with pytest.raises(IKUnreachableError, match="内部死区"):
            ik((130, 0, -23))

    def test_below_ground(self):
        """Z 低于安全地面。"""
        with pytest.raises(IKUnreachableError):
            ik((150, 0, cfg.WORKSPACE_Z_MIN_MM - 10))


# ============================================================
# 3. 数值稳定 — 边界
# ============================================================


class TestNumericalStability:
    def test_target_at_max_reach_minus_epsilon(self):
        """目标恰好在最大伸展边界内一点 — 不应抛 NaN。"""
        # 最大水平伸展 ≈ L2+L3 在腕轴层；末端再下偏 L_END
        # 取 r 让 d = L2+L3-1 = 269
        # 当 z = H1, z_rel = 0, d = r → r = 269
        # 实际目标 X = 269, Z = H1 - L_END = 95 - 147 = -52
        joints = ik((265, 0, -52))  # 留点裕度
        assert not math.isnan(joints.shoulder)
        assert not math.isnan(joints.elbow)

    def test_target_directly_above_shoulder(self):
        """目标接近肩轴正上方 — d 较小但合法。"""
        # r=0, z 大，z_rel = z+L_END-H1 ≈ z+52
        # 取 z = 100, z_rel = 152, d = 152, 在 [10, 270] 内
        # 但是 r=0 时 atan2(0,0) 未定义；放宽用 r=1
        joints = ik((1, 0, 100))
        assert not math.isnan(joints.shoulder)

    def test_zero_y(self):
        """Y=0 时 atan2 应返回 0。"""
        joints = ik((200, 0, -50))
        assert math.isclose(joints.base, 0.0, abs_tol=1e-6)


# ============================================================
# 4. 舵机 us 映射（与 hal_entry.c servo_angle_to_us 对齐）
# ============================================================


class TestServoMapping:
    @pytest.mark.parametrize("joint", list(cfg.SERVO_PARAMS.keys()))
    def test_zero_deg_maps_to_left_safe(self, joint):
        """servo_deg=0 → left_safe_us（reversed=True 时 → right_safe_us）。"""
        p = cfg.SERVO_PARAMS[joint]
        us = servo_deg_to_us(joint, 0.0)
        expected = p["right_safe_us"] if p["reversed"] else p["left_safe_us"]
        assert us == expected

    @pytest.mark.parametrize("joint", list(cfg.SERVO_PARAMS.keys()))
    def test_max_deg_maps_to_right_safe(self, joint):
        """servo_deg=max → right_safe_us（reversed=True 时 → left_safe_us）。"""
        p = cfg.SERVO_PARAMS[joint]
        us = servo_deg_to_us(joint, float(p["max_angle_deg"]))
        expected = p["left_safe_us"] if p["reversed"] else p["right_safe_us"]
        assert us == expected

    def test_clamp_above_max(self):
        """超过 max 应被饱和。"""
        p = cfg.SERVO_PARAMS[cfg.JOINT_SHOULDER]
        us = servo_deg_to_us(cfg.JOINT_SHOULDER, 999.0)
        assert us == p["right_safe_us"]

    def test_clamp_below_zero(self):
        us = servo_deg_to_us(cfg.JOINT_SHOULDER, -10.0)
        assert us == cfg.SERVO_PARAMS[cfg.JOINT_SHOULDER]["left_safe_us"]


# ============================================================
# 5. 安全检查
# ============================================================


class TestSafetyCheck:
    def test_normal_target_passes(self):
        joints = ik((250, 50, 97))  # 草莓操作区内的正常点
        check_safety(joints)  # 不应抛

    def test_large_jump_detected(self):
        a = GeomAngles(
            base=0, shoulder=30, elbow=30, wrist_pitch=0, wrist_rotate=0, gripper=120
        )
        b = GeomAngles(
            base=0, shoulder=120, elbow=30, wrist_pitch=0, wrist_rotate=0, gripper=120
        )
        with pytest.raises(IKUnreachableError, match="防飞舵"):
            check_safety(b, prev_angles=a)


# ============================================================
# 6. 已知姿态对照 — 用 FK 看末端位置（联调时校准用）
# ============================================================


class TestKnownPoses:
    """这些测试在 OFFSET/SIGN 校准前可能 fail，但能给出真实位置便于诊断。

    以下用于阶段 D 联调时的"姿态校准"参考点，不强制通过。
    """

    def test_print_pre_grasp_endpoint(self, capsys):
        """打印 PRE_GRASP 姿态在当前几何映射下的末端位置。"""
        # PRE_GRASP 舵机角 = {126, 117, 86, 135, 80, 120}
        # 反推几何角：geom = (servo - OFFSET) / SIGN
        # 此处仅打印，便于联调时观察
        servo_pose = [126, 117, 86, 135, 80, 120]
        geom_list = []
        for j, s in enumerate(servo_pose):
            off = cfg.JOINT_GEOM_OFFSET_DEG[j]
            sign = cfg.JOINT_GEOM_SIGN[j]
            g = (s - off) / sign if sign != 0 else 0
            geom_list.append(g)
        angles = GeomAngles(*geom_list)
        try:
            tip = fk(angles)
            print(
                f"\n[PRE_GRASP 反推] 几何角={geom_list}, 末端=({tip[0]:.1f}, {tip[1]:.1f}, {tip[2]:.1f})"
            )
        except Exception as e:
            print(f"\n[PRE_GRASP 反推] 失败: {e}")
        # 不 assert：阶段 D 时人工观察后再调整 OFFSET/SIGN
