"""相机 → 单应 → 臂坐标 链路可视化（教学/验证 v2，PC 上跑）。

把 pickup_v2 视觉定位链路 E3/E4/E5 做成可视化：
  像素(u,v) ─E3去畸变→ 干净像素 ─E4单应→ 工作面(Xw,Yw) ─E5偏移→ 臂坐标(X,Y) → 可达性

做法：用一个合成相机（已知内参 K + 位姿）斜看传送带平面，
  - 正向：把传送带网格 + 草莓投影成"相机画面"（带透视 + 可选镜头畸变）
  - 反向：对草莓像素跑完整恢复链路，验证能否还原真实世界坐标
这样你能直观看到：相机斜看为什么需要单应、畸变是什么、偏移把坐标钉到臂上。

运行（PC，需 matplotlib + numpy）：
    cd pickup_v2/sim
    python camera_sim.py

数学与 pi/coord_transform.py 等价（那边用 cv2.undistortPoints + perspectiveTransform）。
平面假设：草莓都在传送带平面 Zw=0 上（与冻结决策一致）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# 复用 v1 的 IK 可达性判定
try:
    from pipeline_sim import solve_ik
except Exception:  # pragma: no cover
    solve_ik = None  # 单独跑相机面板时也能用


# ============================================================
# 合成相机模型（针孔 + 径向畸变）
# ============================================================
@dataclass
class Camera:
    # 内参（OV5647 @ 640x480 量级的示意值）
    fx: float = 550.0
    fy: float = 550.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    # 径向畸变系数（k1<0 = 桶形畸变，广角常见）
    k1: float = -0.28
    k2: float = 0.10
    # 外参：相机在世界中的位置 + 注视点（mm），世界 Zw 朝上
    eye: np.ndarray = field(default_factory=lambda: np.array([150.0, -120.0, 380.0]))
    look_at: np.ndarray = field(default_factory=lambda: np.array([150.0, 150.0, 0.0]))

    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def Rt(self) -> tuple[np.ndarray, np.ndarray]:
        """世界→相机 的旋转 R 和平移 t（look-at，OpenCV 风格 +Z 朝前 +Y 朝下）。"""
        world_up = np.array([0.0, 0.0, 1.0])
        z = self.look_at - self.eye
        z = z / np.linalg.norm(z)  # 相机 +Z：注视方向
        x = np.cross(z, world_up)
        x = x / np.linalg.norm(x)  # 相机 +X：水平向右
        y = np.cross(z, x)  # 相机 +Y：图像向下
        R = np.stack([x, y, z], axis=0)  # 行向量即相机轴
        t = -R @ self.eye
        return R, t


def _distort(
    xn: np.ndarray, yn: np.ndarray, cam: Camera
) -> tuple[np.ndarray, np.ndarray]:
    """归一化坐标上施加径向畸变（正向，渲染相机画面用）。"""
    r2 = xn * xn + yn * yn
    f = 1 + cam.k1 * r2 + cam.k2 * r2 * r2
    return xn * f, yn * f


def world_to_pixel(
    pts_world_xy: np.ndarray, cam: Camera, distort: bool = True
) -> np.ndarray:
    """传送带平面点 (Xw,Yw,Zw=0) → 像素 (u,v)。正向投影，用于渲染相机画面。"""
    pts = np.atleast_2d(pts_world_xy).astype(float)
    Pw = np.column_stack([pts[:, 0], pts[:, 1], np.zeros(len(pts))])
    R, t = cam.Rt()
    Pc = (R @ Pw.T).T + t  # 世界 → 相机坐标
    xn = Pc[:, 0] / Pc[:, 2]
    yn = Pc[:, 1] / Pc[:, 2]
    if distort:
        xn, yn = _distort(xn, yn, cam)
    u = cam.fx * xn + cam.cx
    v = cam.fy * yn + cam.cy
    return np.column_stack([u, v])


def homography_pixel_to_world(cam: Camera) -> np.ndarray:
    """E4：像素 → 工作面 mm 的单应矩阵（理想无畸变情形）。

    平面 Zw=0 时投影退化为单应：pixel ~ K·[r1 | r2 | t]·[Xw,Yw,1]ᵀ。
    取逆即得 像素→世界。真实标定中这个 H 是用点对 cv2.findHomography 估出来的，
    这里因为是合成相机，可直接解析算出（充当"标定完美"的基准）。
    """
    R, t = cam.Rt()
    H_w2p = cam.K() @ np.column_stack([R[:, 0], R[:, 1], t])
    return np.linalg.inv(H_w2p)


def undistort_pixel(uv: np.ndarray, cam: Camera, iters: int = 8) -> np.ndarray:
    """E3：去畸变（迭代反解径向畸变），等价 cv2.undistortPoints。"""
    uv = np.atleast_2d(uv).astype(float)
    xd = (uv[:, 0] - cam.cx) / cam.fx
    yd = (uv[:, 1] - cam.cy) / cam.fy
    xn, yn = xd.copy(), yd.copy()
    for _ in range(iters):  # 不动点迭代求未畸变归一化坐标
        r2 = xn * xn + yn * yn
        f = 1 + cam.k1 * r2 + cam.k2 * r2 * r2
        xn = xd / f
        yn = yd / f
    u = cam.fx * xn + cam.cx
    v = cam.fy * yn + cam.cy
    return np.column_stack([u, v])


def apply_homography(uv: np.ndarray, H: np.ndarray) -> np.ndarray:
    """对像素施加单应 → 工作面坐标。等价 cv2.perspectiveTransform。"""
    uv = np.atleast_2d(uv).astype(float)
    pts = np.column_stack([uv[:, 0], uv[:, 1], np.ones(len(uv))])
    w = (H @ pts.T).T
    return w[:, :2] / w[:, 2:3]


def pixel_to_arm(uv, cam: Camera, offset_xy, use_undistort: bool = True):
    """完整恢复链路：像素 →(E3)→ 去畸变 →(E4)→ 工作面 →(E5)→ 臂坐标。

    返回 (Xw, Yw, Xarm, Yarm, r_arm)。
    """
    uv = np.atleast_2d(uv).astype(float)
    clean = undistort_pixel(uv, cam) if use_undistort else uv
    world = apply_homography(clean, homography_pixel_to_world(cam))
    Xw, Yw = float(world[0, 0]), float(world[0, 1])
    Xa = Xw - offset_xy[0]
    Ya = Yw - offset_xy[1]
    r_arm = float(np.hypot(Xa, Ya))
    return Xw, Yw, Xa, Ya, r_arm


# ============================================================
# 交互式可视化
# ============================================================
# 传送带平面范围（mm）与抓取假设
BELT_X = (0.0, 300.0)
BELT_Y = (0.0, 300.0)
GRASP_Z_ARM = -60.0  # 草莓在臂坐标系的高度（传送带低于基座）
GRASP_PHI = -55.0  # 末端固定斜角（可达性检查用，可在 v1 里细调）


def _grid_polyline_pixels(cam: Camera, distort: bool):
    """把传送带网格线密采样后投到像素，返回若干折线（展示透视+畸变弯曲）。"""
    lines = []
    xs = np.linspace(*BELT_X, 7)
    ys = np.linspace(*BELT_Y, 7)
    for x in xs:  # 竖线（Yw 变化）
        seg = np.column_stack([np.full(40, x), np.linspace(*BELT_Y, 40)])
        lines.append(world_to_pixel(seg, cam, distort))
    for y in ys:  # 横线（Xw 变化）
        seg = np.column_stack([np.linspace(*BELT_X, 40), np.full(40, y)])
        lines.append(world_to_pixel(seg, cam, distort))
    return lines


def run_app() -> None:
    import matplotlib

    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt
    from matplotlib.widgets import CheckButtons, Slider

    cam = Camera()
    state = {
        "X": 150.0,
        "Y": 150.0,
        "ox": 150.0,
        "oy": -60.0,
        "render_dist": True,
        "use_e3": True,
    }

    fig, (ax_img, ax_world) = plt.subplots(1, 2, figsize=(15, 7.5))
    plt.subplots_adjust(left=0.06, right=0.98, top=0.92, bottom=0.30, wspace=0.22)

    # ---- 左：相机画面（像素空间，v 朝下）----
    ax_img.set_title("相机画面（像素 u-v）")
    ax_img.set_xlim(0, cam.width)
    ax_img.set_ylim(cam.height, 0)  # 翻转 y，像真实图像
    ax_img.set_xlabel("u (px)")
    ax_img.set_ylabel("v (px)")
    ax_img.set_aspect("equal")
    grid_artists = []
    (straw_px,) = ax_img.plot(
        [], [], "*", color="purple", ms=20, zorder=6, label="草莓像素(u,v)"
    )
    ax_img.plot([cam.cx], [cam.cy], "+", color="red", ms=12, label="光心(cx,cy)")
    ax_img.legend(loc="upper right", fontsize=8)

    # ---- 右：俯视世界 + 臂坐标 ----
    ax_world.set_title("俯视：工作面世界坐标 + 机械臂")
    ax_world.set_xlabel("Xw (mm)")
    ax_world.set_ylabel("Yw (mm)")
    ax_world.set_aspect("equal")
    ax_world.add_patch(
        plt.Rectangle(
            (BELT_X[0], BELT_Y[0]),
            BELT_X[1] - BELT_X[0],
            BELT_Y[1] - BELT_Y[0],
            fc="#eef7ee",
            ec="green",
            lw=1.2,
            label="传送带平面",
        )
    )
    for gx in np.linspace(*BELT_X, 7):
        ax_world.axvline(gx, color="green", alpha=0.12)
    for gy in np.linspace(*BELT_Y, 7):
        ax_world.axhline(gy, color="green", alpha=0.12)
    (straw_true,) = ax_world.plot(
        [], [], "*", color="purple", ms=18, zorder=6, label="草莓真实"
    )
    (straw_rec,) = ax_world.plot(
        [], [], "o", color="orange", ms=9, mfc="none", mew=2, zorder=7, label="链路恢复"
    )
    (arm_base,) = ax_world.plot([], [], "ks", ms=12, zorder=6, label="臂基座(E5原点)")
    (reach_line,) = ax_world.plot([], [], "-", color="gray", lw=1.2, zorder=4)
    ax_world.set_xlim(BELT_X[0] - 80, BELT_X[1] + 40)
    ax_world.set_ylim(cam.eye[1] - 20, BELT_Y[1] + 40)
    info = ax_world.text(
        0.02,
        0.98,
        "",
        transform=ax_world.transAxes,
        va="top",
        fontsize=9,
        family="monospace",
        bbox=dict(boxstyle="round", fc="white", alpha=0.85),
    )
    ax_world.legend(loc="lower right", fontsize=8)

    # ---- 滑块 + 勾选框 ----
    s_X = Slider(
        plt.axes([0.08, 0.20, 0.34, 0.03]),
        "草莓 Xw",
        *BELT_X,
        valinit=state["X"],
        valstep=2,
    )
    s_Y = Slider(
        plt.axes([0.08, 0.15, 0.34, 0.03]),
        "草莓 Yw",
        *BELT_Y,
        valinit=state["Y"],
        valstep=2,
    )
    s_ox = Slider(
        plt.axes([0.08, 0.10, 0.34, 0.03]),
        "E5偏移 ox",
        -100,
        250,
        valinit=state["ox"],
        valstep=2,
    )
    s_oy = Slider(
        plt.axes([0.08, 0.05, 0.34, 0.03]),
        "E5偏移 oy",
        -150,
        200,
        valinit=state["oy"],
        valstep=2,
    )
    checks = CheckButtons(
        plt.axes([0.55, 0.05, 0.22, 0.16]),
        ["渲染镜头畸变", "启用E3去畸变恢复"],
        [True, True],
    )

    def redraw_grid():
        for a in grid_artists:
            a.remove()
        grid_artists.clear()
        for ln in _grid_polyline_pixels(cam, state["render_dist"]):
            (a,) = ax_img.plot(ln[:, 0], ln[:, 1], color="green", alpha=0.4, lw=1)
            grid_artists.append(a)

    def update(_=None):
        state.update(X=s_X.val, Y=s_Y.val, ox=s_ox.val, oy=s_oy.val)
        uv = world_to_pixel((state["X"], state["Y"]), cam, state["render_dist"])[0]
        straw_px.set_data([uv[0]], [uv[1]])

        Xw, Yw, Xa, Ya, r = pixel_to_arm(
            uv, cam, (state["ox"], state["oy"]), state["use_e3"]
        )
        straw_true.set_data([state["X"]], [state["Y"]])
        straw_rec.set_data([Xw], [Yw])
        arm_base.set_data([state["ox"]], [state["oy"]])
        reach_line.set_data([state["ox"], Xw], [state["oy"], Yw])

        err = float(np.hypot(Xw - state["X"], Yw - state["Y"]))
        verdict = "（v1未导入）"
        if solve_ik is not None:
            res = solve_ik(r, GRASP_Z_ARM, GRASP_PHI)
            verdict = res.reason
        info.set_text(
            f"像素 (u,v)=({uv[0]:.0f},{uv[1]:.0f})\n"
            f"─E3+E4→ 世界 ({Xw:.1f},{Yw:.1f}) mm\n"
            f"   还原误差 = {err:.2f} mm "
            f"{'(E3关:边缘会偏)' if not state['use_e3'] else ''}\n"
            f"─E5偏移→ 臂坐标 ({Xa:.1f},{Ya:.1f})  r={r:.0f}\n"
            f"抓取 z={GRASP_Z_ARM:.0f} φ={GRASP_PHI:.0f}° → {verdict}"
        )
        fig.canvas.draw_idle()

    def on_check(label):
        if label == "渲染镜头畸变":
            state["render_dist"] = not state["render_dist"]
            redraw_grid()
        elif label == "启用E3去畸变恢复":
            state["use_e3"] = not state["use_e3"]
        update()

    for s in (s_X, s_Y, s_ox, s_oy):
        s.on_changed(update)
    checks.on_clicked(on_check)

    redraw_grid()
    update()
    print(
        "提示：拖草莓滑块看像素↔世界对应；关掉'E3去畸变'看边缘误差；拖 E5偏移 看臂坐标平移。"
    )
    plt.show()


if __name__ == "__main__":
    run_app()
