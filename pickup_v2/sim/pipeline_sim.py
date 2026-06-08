"""草莓抓取机械臂 IK 可达性交互仿真（教学/验证用，PC 上跑）。

目的：直观验证 pickup_v2 的逆运动学算法是否合理，尤其是
  - 肩抬角 α_s = φ + β、肘弯角 α_e = 180° - γ 的几何含义（余弦定理）
  - "夹爪垂直朝下(-90°)" vs "固定斜角" 对**可达区域**的影响（你发现的够不到问题）

运行（PC，需 matplotlib + numpy）：
    cd pickup_v2/sim
    python pipeline_sim.py

操作：拖动 3 个滑块
  - target_r : 目标到臂的水平距离 (mm)
  - target_z : 目标高度 (mm，臂基座原点为 0，负=低于基座)
  - phi      : 末端世界俯仰角 (deg，-90=垂直朝下，往 0 走=越来越斜)

界面会画出机械臂当前姿态、肩肘腕三角形、并把"当前 φ 下整个平面的可达区"染色。
左上角文字给出各关节几何角 / 舵机角 / 是否可达 / 不可达原因。

⚠ 本仿真只做平面几何可达性（连杆够不够、关节角是否在量程内），
  不模拟夹爪与传送带的物理碰撞——但会把 180mm 的末端连杆画出来，你可以肉眼看碰撞。
  连杆参数与 config_v2.py 一致；IK 数学是 kinematics.py 的参数化推广（φ 可调）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# ============================================================
# 连杆参数（与 pickup_v2/pi/config_v2.py 实测值一致）
# ============================================================
H1 = 100.0  # 底座顶面 → 肩轴 高度
L2 = 115.0  # 肩轴 → 肘轴
L3 = 160.0  # 肘轴 → 腕俯仰轴
L_END = 180.0  # 腕俯仰轴 → 指尖（末端等效连杆）

# 关节几何角量程（与 kinematics.py docstring 约定一致）
#   α_s ∈ [0,180]   肩相对水平面
#   α_e ∈ [0,180]   肘弯折角（0=伸直）
#   α_p ∈ [-90,90]  腕俯仰相对小臂（受腕舵机 0..180 限制：servo=90-α_p）
ALPHA_S_RANGE = (0.0, 180.0)
ALPHA_E_RANGE = (0.0, 180.0)
ALPHA_P_RANGE = (-90.0, 90.0)

# 默认 OFFSET/SIGN（config_v2 首版默认值，E1 标定后会被 yaml 覆盖）
# 仅用于把几何角换算成"舵机角"展示，几何可达性不依赖它。
OFFSET = {"shoulder": 25.0, "elbow": 0.0, "wrist_pitch": 90.0}
SIGN = {"shoulder": +1, "elbow": +1, "wrist_pitch": -1}


@dataclass
class IKResult:
    reachable: bool
    reason: str
    alpha_s: float  # 肩抬角 (deg)
    alpha_e: float  # 肘弯角 (deg)
    alpha_p: float  # 腕俯仰 (deg)
    # 平面内各关节坐标 (r, z)，用于画臂
    shoulder: tuple[float, float]
    elbow: tuple[float, float]
    wrist: tuple[float, float]
    tip: tuple[float, float]
    wrist_target: tuple[float, float]  # 腕轴应到达的目标点
    d: float  # 肩→腕轴 直线距离


def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))


def solve_ik(r: float, z: float, phi_deg: float) -> IKResult:
    """参数化 IK：给定平面目标 (r, z) 和末端世界俯仰角 φ，解肩/肘/腕几何角。

    φ = -90° 时退化为 kinematics.py 的"垂直朝下"原版（可自行验证公式一致）。
    """
    phi = math.radians(phi_deg)

    # ── 步骤1：末端约束 → 腕俯仰轴目标位置 ──
    # 末端方向单位向量 (cosφ, sinφ)，指尖在腕轴前方 L_END 处，
    # 所以腕轴 = 目标点 - L_END·(cosφ, sinφ)。
    # φ=-90 时 cos=0,sin=-1 → r_w=r, z_w=z+L_END（与原代码一致）。
    r_w = r - L_END * math.cos(phi)
    z_w = z - L_END * math.sin(phi)

    # ── 步骤2：肩-肘 平面两连杆，以肩轴(0,H1)为原点 ──
    r_rel = r_w - 0.0
    z_rel = z_w - H1
    d = math.hypot(r_rel, z_rel)

    s_pos = (0.0, H1)

    # 几何可达性：连杆够不够
    if d > L2 + L3:
        return IKResult(
            False,
            f"太远：d={d:.0f} > L2+L3={L2 + L3:.0f}",
            0,
            0,
            0,
            s_pos,
            s_pos,
            (r_w, z_w),
            (r, z),
            (r_w, z_w),
            d,
        )
    if d < abs(L2 - L3):
        return IKResult(
            False,
            f"太近死区：d={d:.0f} < |L2-L3|={abs(L2 - L3):.0f}",
            0,
            0,
            0,
            s_pos,
            s_pos,
            (r_w, z_w),
            (r, z),
            (r_w, z_w),
            d,
        )

    # ── 步骤3：余弦定理解肘弯角 α_e ──
    cos_inner = (L2**2 + L3**2 - d * d) / (2 * L2 * L3)
    inner = math.acos(_clamp(cos_inner))  # 大小臂内夹角 γ
    alpha_e = math.degrees(math.pi - inner)  # 弯折量：伸直=0

    # ── 步骤4：肩抬角 α_s = φ_dir + β（elbow-up 解）──
    phi_dir = math.atan2(z_rel, r_rel)  # 肩→腕轴 连线仰角
    cos_beta = (L2**2 + d * d - L3**2) / (2 * L2 * d)
    beta = math.acos(_clamp(cos_beta))
    alpha_s = math.degrees(phi_dir + beta)

    # ── 步骤5：腕俯仰由末端约束补偿 ──
    # end_world = α_s - α_e + α_p = φ  ⟹  α_p = φ - α_s + α_e
    alpha_p = phi_deg - alpha_s + alpha_e

    # 各关节平面坐标（用于画臂 / 验证 FK 回到目标）
    a_s = math.radians(alpha_s)
    elbow = (L2 * math.cos(a_s), H1 + L2 * math.sin(a_s))
    forearm_world = a_s - math.radians(alpha_e)
    wrist = (
        elbow[0] + L3 * math.cos(forearm_world),
        elbow[1] + L3 * math.sin(forearm_world),
    )
    end_world = forearm_world + math.radians(alpha_p)
    tip = (
        wrist[0] + L_END * math.cos(end_world),
        wrist[1] + L_END * math.sin(end_world),
    )

    # 关节量程检查
    bad = []
    if not (ALPHA_S_RANGE[0] <= alpha_s <= ALPHA_S_RANGE[1]):
        bad.append(f"肩 α_s={alpha_s:.0f}° 超{ALPHA_S_RANGE}")
    if not (ALPHA_E_RANGE[0] <= alpha_e <= ALPHA_E_RANGE[1]):
        bad.append(f"肘 α_e={alpha_e:.0f}° 超{ALPHA_E_RANGE}")
    if not (ALPHA_P_RANGE[0] <= alpha_p <= ALPHA_P_RANGE[1]):
        bad.append(f"腕 α_p={alpha_p:.0f}° 超{ALPHA_P_RANGE}")

    reachable = len(bad) == 0
    reason = "可达 ✓" if reachable else "关节超量程: " + "; ".join(bad)
    return IKResult(
        reachable,
        reason,
        alpha_s,
        alpha_e,
        alpha_p,
        s_pos,
        elbow,
        wrist,
        tip,
        (r_w, z_w),
        d,
    )


def geom_to_servo(joint: str, geom: float) -> float:
    return OFFSET[joint] + SIGN[joint] * geom


def reachable_mask(
    phi_deg: float, r_grid: np.ndarray, z_grid: np.ndarray
) -> np.ndarray:
    """对 r-z 网格逐点判可达，返回布尔矩阵（用于染色当前 φ 的可达区）。"""
    mask = np.zeros((len(z_grid), len(r_grid)), dtype=bool)
    for i, z in enumerate(z_grid):
        for j, r in enumerate(r_grid):
            mask[i, j] = solve_ik(float(r), float(z), phi_deg).reachable
    return mask


# ============================================================
# 交互式可视化（matplotlib）
# ============================================================
def run_app() -> None:
    import matplotlib

    # 中文字体（Windows 用黑体；缺字体不致命，仅中文变方块）
    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False

    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    R_MIN, R_MAX = 0.0, 320.0
    Z_MIN, Z_MAX = -130.0, 280.0
    r_grid = np.linspace(R_MIN, R_MAX, 70)
    z_grid = np.linspace(Z_MIN, Z_MAX, 70)

    fig, ax = plt.subplots(figsize=(9, 8))
    plt.subplots_adjust(left=0.10, right=0.97, top=0.93, bottom=0.26)

    # 初值
    state = {"r": 200.0, "z": -40.0, "phi": -90.0}

    # 可达区底图（随 φ 重算）
    mask = reachable_mask(state["phi"], r_grid, z_grid)
    im = ax.imshow(
        mask,
        origin="lower",
        extent=[R_MIN, R_MAX, Z_MIN, Z_MAX],
        aspect="equal",
        cmap="Greens",
        alpha=0.35,
        vmin=0,
        vmax=1,
        zorder=0,
    )

    # 静态参考：基座、工作面 z=0、最大伸展圆
    ax.plot([0], [0], "ks", ms=10, zorder=5)
    ax.annotate("臂基座(0,0)", (0, 0), textcoords="offset points", xytext=(6, -14))
    ax.axhline(0, color="gray", ls=":", lw=1, zorder=1)
    th = np.linspace(-0.3, 1.6, 50)
    ax.plot(
        (L2 + L3) * np.cos(th),
        H1 + (L2 + L3) * np.sin(th),
        "b:",
        lw=1,
        alpha=0.5,
        zorder=1,
        label="肩轴最大伸展",
    )

    # 动态对象
    (arm_line,) = ax.plot(
        [], [], "-o", color="#d9534f", lw=4, ms=7, zorder=6, label="机械臂"
    )
    (gripper_line,) = ax.plot(
        [], [], "-", color="#f0ad4e", lw=6, zorder=6, label="末端(夹爪180mm)"
    )
    (tri_line,) = ax.plot(
        [], [], "--", color="#5bc0de", lw=1.2, zorder=4, label="肩肘腕三角"
    )
    (target_pt,) = ax.plot(
        [], [], "*", color="purple", ms=18, zorder=7, label="目标草莓"
    )
    txt = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        family="monospace",
        bbox=dict(boxstyle="round", fc="white", alpha=0.85),
    )

    ax.set_xlim(R_MIN, R_MAX)
    ax.set_ylim(Z_MIN, Z_MAX)
    ax.set_xlabel("r  水平距离 (mm)")
    ax.set_ylabel("z  高度 (mm)")
    ax.set_title("机械臂 IK 可达性仿真（侧视 r-z 平面）")
    ax.legend(loc="lower right", fontsize=8)
    ax.grid(alpha=0.2)

    # 滑块
    ax_r = plt.axes([0.12, 0.15, 0.70, 0.03])
    ax_z = plt.axes([0.12, 0.10, 0.70, 0.03])
    ax_phi = plt.axes([0.12, 0.05, 0.70, 0.03])
    s_r = Slider(ax_r, "target_r", R_MIN, R_MAX, valinit=state["r"], valstep=2)
    s_z = Slider(ax_z, "target_z", Z_MIN, Z_MAX, valinit=state["z"], valstep=2)
    s_phi = Slider(
        ax_phi, "phi(末端俯仰)", -90.0, -10.0, valinit=state["phi"], valstep=1
    )

    def redraw_mask() -> None:
        im.set_data(reachable_mask(state["phi"], r_grid, z_grid))

    def update(_=None) -> None:
        state["r"], state["z"], state["phi"] = s_r.val, s_z.val, s_phi.val
        res = solve_ik(state["r"], state["z"], state["phi"])

        target_pt.set_data([state["r"]], [state["z"]])
        if res.reachable or res.d <= L2 + L3:
            s, e, w, t = res.shoulder, res.elbow, res.wrist, res.tip
            arm_line.set_data([0, s[0], e[0], w[0]], [0, s[1], e[1], w[1]])
            gripper_line.set_data([w[0], t[0]], [w[1], t[1]])
            tri_line.set_data([s[0], e[0], w[0]], [s[1], e[1], w[1]])
        else:
            arm_line.set_data([], [])
            gripper_line.set_data([], [])
            tri_line.set_data([], [])

        color = "green" if res.reachable else "red"
        arm_line.set_color("#5cb85c" if res.reachable else "#d9534f")
        sv_s = geom_to_servo("shoulder", res.alpha_s)
        sv_e = geom_to_servo("elbow", res.alpha_e)
        sv_p = geom_to_servo("wrist_pitch", res.alpha_p)
        txt.set_text(
            f"目标 (r={state['r']:.0f}, z={state['z']:.0f})  φ={state['phi']:.0f}°\n"
            f"肩→腕直线 d = {res.d:.0f} mm  (L2+L3={L2 + L3:.0f})\n"
            f"─ 几何角 ─\n"
            f"  肩 α_s = {res.alpha_s:6.1f}°   → 舵机 {sv_s:6.1f}\n"
            f"  肘 α_e = {res.alpha_e:6.1f}°   → 舵机 {sv_e:6.1f}\n"
            f"  腕 α_p = {res.alpha_p:6.1f}°   → 舵机 {sv_p:6.1f}\n"
            f"{res.reason}"
        )
        txt.set_color(color)
        fig.canvas.draw_idle()

    def on_phi(_=None) -> None:
        state["phi"] = s_phi.val
        redraw_mask()
        update()

    s_r.on_changed(update)
    s_z.on_changed(update)
    s_phi.on_changed(on_phi)

    update()
    print(
        "提示：拖 target_r/target_z 移动草莓；拖 phi 看垂直(-90)→斜角 可达绿区怎么扩大。"
    )
    plt.show()


if __name__ == "__main__":
    run_app()
