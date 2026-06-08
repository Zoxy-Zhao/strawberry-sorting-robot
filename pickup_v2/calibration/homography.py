"""单应矩阵标定（像素坐标 → 工作面世界坐标）。

工作面假定为机械臂下方一块**水平平面**（传送带或工位面）。
在该平面上选 4-9 个已知世界坐标的标定点（如棋盘格交点、贴在工位上的 Aruco/打印靶），
在图像里点出对应像素位置，cv2.findHomography 求出 3x3 单应矩阵 H。

子命令：
    pick    交互式打点（鼠标左键点像素，命令行输入对应世界坐标 mm）
    fit     从 CSV 读点对 → cv2.findHomography → 输出 yaml
    apply   对单点像素 (u, v) 应用 H，输出工作面坐标 (X, Y) mm
    verify  把所有点对回投，看残差

CSV 格式（用于 fit）：
    u_px,v_px,X_mm,Y_mm
    320,240,0,0
    420,240,50,0
    ...

用法示例：
    python homography.py pick --image work_plane.png --out points.csv
    python homography.py fit --src points.csv --out homography.yaml
    python homography.py apply --params homography.yaml --uv 320 240
    python homography.py verify --params homography.yaml --src points.csv

约定：
    - 工作面坐标系原点和方向由你定（建议机械臂基座中心为原点，x 沿传送带前进方向，单位 mm）
    - 输出 yaml 与 intrinsic_calib.py 兼容，可一起加载
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


def cmd_pick(args: argparse.Namespace) -> int:
    img = cv2.imread(args.image)
    if img is None:
        print(f"[ERR] 读不到图 {args.image}", file=sys.stderr)
        return 2

    points: list[tuple[int, int, float, float]] = []
    pending_uv: list[tuple[int, int]] = []

    def on_mouse(event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pending_uv.append((x, y))
            print(
                f"[PICK] 像素 ({x}, {y})  请在终端输入对应世界坐标 'X Y'（mm），或 'skip' 放弃"
            )

    cv2.namedWindow("homography pick (left=add, q=quit)")
    cv2.setMouseCallback("homography pick (left=add, q=quit)", on_mouse)

    print(
        "[PICK] 鼠标左键依次点已知世界坐标的标定点；至少 4 个（建议 6-9 个），按 q 完成"
    )

    while True:
        vis = img.copy()
        for i, (u, v, X, Y) in enumerate(points):
            cv2.circle(vis, (u, v), 6, (0, 255, 0), -1)
            cv2.putText(
                vis,
                f"{i}:({X:.0f},{Y:.0f})",
                (u + 8, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
        for u, v in pending_uv:
            cv2.circle(vis, (u, v), 6, (0, 165, 255), 2)
        cv2.imshow("homography pick (left=add, q=quit)", vis)

        if pending_uv:
            (u, v) = pending_uv[-1]
            cv2.waitKey(50)
            try:
                line = input(f"  ({u},{v}) 对应世界坐标 X Y mm: ").strip()
            except EOFError:
                break
            pending_uv.pop()
            if line.lower() in ("skip", "s", ""):
                print("  跳过")
                continue
            try:
                X, Y = (float(t) for t in line.split())
            except ValueError:
                print("  格式错，跳过")
                continue
            points.append((u, v, X, Y))
            print(f"  已记录 #{len(points)}: ({u},{v}) -> ({X},{Y}) mm")
            continue

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()

    if len(points) < 4:
        print(f"[ERR] 只点了 {len(points)} 个点，至少要 4 个", file=sys.stderr)
        return 2

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["u_px", "v_px", "X_mm", "Y_mm"])
        for row in points:
            w.writerow(row)
    print(f"[DONE] 写入 {len(points)} 个点对到 {out}")
    return 0


def _read_points(csv_path: Path) -> tuple[np.ndarray, np.ndarray]:
    pts_img: list[list[float]] = []
    pts_world: list[list[float]] = []
    with open(csv_path, "r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            pts_img.append([float(row["u_px"]), float(row["v_px"])])
            pts_world.append([float(row["X_mm"]), float(row["Y_mm"])])
    return np.array(pts_img, dtype=np.float64), np.array(pts_world, dtype=np.float64)


def cmd_fit(args: argparse.Namespace) -> int:
    src = Path(args.src)
    pts_img, pts_world = _read_points(src)
    if len(pts_img) < 4:
        print(f"[ERR] {src} 只有 {len(pts_img)} 个点对，至少 4 个", file=sys.stderr)
        return 2

    H, mask = cv2.findHomography(
        pts_img, pts_world, method=cv2.RANSAC, ransacReprojThreshold=2.0
    )
    if H is None:
        print("[ERR] findHomography 失败（点共线？分布不够发散？）", file=sys.stderr)
        return 2

    inliers = int(mask.sum()) if mask is not None else len(pts_img)

    proj = cv2.perspectiveTransform(pts_img.reshape(-1, 1, 2), H).reshape(-1, 2)
    err_per_point = np.linalg.norm(proj - pts_world, axis=1)
    mean_err = float(np.mean(err_per_point))
    max_err = float(np.max(err_per_point))

    out = {
        "n_points": len(pts_img),
        "n_inliers": inliers,
        "mean_reproj_error_mm": mean_err,
        "max_reproj_error_mm": max_err,
        "homography_pixel_to_world_mm": H.tolist(),
        "source_csv": str(src),
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)

    print(f"[DONE] 写入 {out_path}")
    print(f"  inliers: {inliers}/{len(pts_img)}")
    print(f"  mean / max reprojection error: {mean_err:.3f} / {max_err:.3f} mm")
    if mean_err > 5.0:
        print("  [WARN] 平均误差 > 5mm，建议重新标定（点位分布要覆盖整个工作面）")
    return 0


def cmd_apply(args: argparse.Namespace) -> int:
    with open(args.params, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)
    H = np.array(params["homography_pixel_to_world_mm"])
    u, v = args.uv
    pt = np.array([[[u, v]]], dtype=np.float64)
    world = cv2.perspectiveTransform(pt, H).flatten()
    print(f"({u}, {v}) px -> ({world[0]:.2f}, {world[1]:.2f}) mm")
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    with open(args.params, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)
    H = np.array(params["homography_pixel_to_world_mm"])
    pts_img, pts_world = _read_points(Path(args.src))
    proj = cv2.perspectiveTransform(pts_img.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - pts_world, axis=1)
    for i, (gt, pr, e) in enumerate(zip(pts_world, proj, err)):
        flag = " *" if e > 3.0 else ""
        print(
            f"  #{i}: gt=({gt[0]:7.2f},{gt[1]:7.2f}) pred=({pr[0]:7.2f},{pr[1]:7.2f}) err={e:.3f} mm{flag}"
        )
    print(
        f"\n  mean = {float(np.mean(err)):.3f} mm,  max = {float(np.max(err)):.3f} mm  (* 表示 > 3mm)"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("pick", help="交互式打点")
    a.add_argument("--image", required=True, help="工作面静态图")
    a.add_argument("--out", default="points.csv")

    b = sub.add_parser("fit", help="拟合单应矩阵")
    b.add_argument("--src", required=True, help="点对 CSV")
    b.add_argument("--out", default="homography.yaml")

    c = sub.add_parser("apply", help="对单点应用 H")
    c.add_argument("--params", required=True)
    c.add_argument("--uv", type=float, nargs=2, required=True, metavar=("U", "V"))

    d = sub.add_parser("verify", help="所有点回投残差")
    d.add_argument("--params", required=True)
    d.add_argument("--src", required=True)

    return ap


def main() -> int:
    args = build_parser().parse_args()
    return {"pick": cmd_pick, "fit": cmd_fit, "apply": cmd_apply, "verify": cmd_verify}[
        args.cmd
    ](args)


if __name__ == "__main__":
    raise SystemExit(main())
