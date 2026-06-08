"""用"已确认能夹到草莓"的真实姿态，闭环验证 新IK(φ_fixed=-43°) 是否正确。

逻辑：
  真实姿态几何角 --FK--> 草莓位置(x,y,z) --新IK(φ=-43)--> 解出角度
  若解出的角度 ≈ 真实角度 → IK 数学 + φ_fixed 正确。
再顺带暴露 shoulder 的 几何→舵机 config 依赖缺口（标定config vs 抓取config，~14°）。
并扫描传送带 r 区间，给出 shoulder/elbow 实际操作角度范围（决定重标策略）。

PC 上跑：python verify_grasp_roundtrip.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "pi"))

import kinematics as K  # noqa: E402
from kinematics import GeomAngles  # noqa: E402
from pipeline_sim import solve_ik  # noqa: E402

PHI_FIXED = -43.0

# 实测/可信的真实抓取姿态几何角（base/wrist 实测可信，elbow 来自标定内插）
real = GeomAngles(
    base=-1.0,
    shoulder=85.0,
    elbow=83.0,
    wrist_pitch=-45.0,
    wrist_rotate=0.0,
    gripper=0.0,
)

# ---- 1. FK：真实角度 → 草莓位置 ----
x, y, z = K.fk(real)
r = (x * x + y * y) ** 0.5
print("=" * 56)
print("1) 真实抓取姿态 --FK--> 草莓位置")
print(
    f"   角度 base={real.base} shoulder={real.shoulder} "
    f"elbow={real.elbow} wrist_pitch={real.wrist_pitch}"
)
print(f"   草莓位置 (x,y,z) = ({x:.1f}, {y:.1f}, {z:.1f}) mm   r={r:.1f}")

# ---- 2. 新 IK(φ=-43)：位置 → 角度，看能否还原 ----
res = solve_ik(r, z, PHI_FIXED)
print("\n2) 草莓位置 --新IK(φ=-43°)--> 解出角度（应回到真实角度）")
print(f"   shoulder: 解={res.alpha_s:.1f}°  真实=85  差={abs(res.alpha_s - 85):.2f}")
print(f"   elbow   : 解={res.alpha_e:.1f}°  真实=83  差={abs(res.alpha_e - 83):.2f}")
print(f"   wrist   : 解={res.alpha_p:.1f}°  真实=-45 差={abs(res.alpha_p + 45):.2f}")
print(f"   可达性  : {res.reason}")

# ---- 3. 暴露 shoulder 几何→舵机 的 config 依赖缺口（E1.4 后仍在）----
print("\n3) shoulder 几何→舵机 缺口（IK要85°，标定config vs 抓取config 不一致）")
servo_cal = K.geom_to_servo_deg(1, 85.0)  # 4点多点插值（标 shoulder 时 elbow=HOME）
print(f"   4点标定: 几何85° -> 舵机 {servo_cal:.0f}（elbow 锁 HOME 时标的）")
print("   抓取锚点: 几何85° 对应 舵机 130（真实抓取 config，elbow 折角不同）")
print(
    f"   缺口  : {abs(servo_cal - 130):.0f}° —— 重力下垂看 elbow 负载，留 E2 实测处置"
)

# ---- 4. 扫描传送带 r 区间 → shoulder/elbow 操作范围 ----
print("\n4) 传送带 r 区间扫描（z 固定在草莓高度）→ 实际操作角度范围")
zs = z
ss, es, rok = [], [], []
for rr in range(220, 341, 5):
    rr_res = solve_ik(float(rr), zs, PHI_FIXED)
    if rr_res.reachable:
        ss.append(rr_res.alpha_s)
        es.append(rr_res.alpha_e)
        rok.append(rr)
if rok:
    print(f"   可达 r 区间: {min(rok)}~{max(rok)} mm")
    print(f"   shoulder 操作范围: {min(ss):.0f}° ~ {max(ss):.0f}°")
    print(f"   elbow    操作范围: {min(es):.0f}° ~ {max(es):.0f}°")
else:
    print("   该高度无可达点（需检查 z 或 φ）")
print("=" * 56)
