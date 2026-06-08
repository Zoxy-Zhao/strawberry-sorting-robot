"""相机内参标定（OpenCV 棋盘格法）。

子命令：
    capture   实时采图（按 c 保存当前帧，q 退出）
    compute   读图片目录，提取角点 → cv2.calibrateCamera → 保存 yaml
    undistort 用标定结果对图片去畸变，肉眼验证

用法示例：
    python intrinsic_calib.py capture --camera 0 --out images/
    python intrinsic_calib.py compute --src images/ --pattern 9x6 --square 25 --out intrinsics.yaml
    python intrinsic_calib.py undistort --params intrinsics.yaml --src images/ --out undistorted/

约定：
    - 棋盘格规格用 "WxH" 表示内角点数（不是方格数），默认 9x6
    - square 单位 mm；输出的 fx/fy/cx/cy 单位为像素，畸变系数 5 元组 (k1, k2, p1, p2, k3)
    - yaml 输出格式与 vision/calibration/homography.py 兼容
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


def parse_pattern(s: str) -> tuple[int, int]:
    w, h = s.lower().split("x")
    return int(w), int(h)


def cmd_capture(args: argparse.Namespace) -> int:
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[ERR] 打不开相机 {args.camera}", file=sys.stderr)
        return 2

    pattern = parse_pattern(args.pattern)
    saved = 0
    print(f"[CAP] 棋盘格 {pattern[0]}x{pattern[1]} 内角点；按 c 保存，q 退出")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] 读帧失败，重试...", file=sys.stderr)
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern, None)

        vis = frame.copy()
        if found:
            cv2.drawChessboardCorners(vis, pattern, corners, found)
        status = f"saved={saved}  found={'Y' if found else 'N'}"
        cv2.putText(
            vis,
            status,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if found else (0, 0, 255),
            2,
        )
        cv2.imshow("intrinsic_calib (c=save, q=quit)", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("c"):
            if not found:
                print("[SKIP] 未检测到完整棋盘，放弃保存")
                continue
            saved += 1
            fname = out_dir / f"calib_{saved:03d}.png"
            cv2.imwrite(str(fname), frame)
            print(f"[SAVE] {fname}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"[DONE] 共保存 {saved} 张到 {out_dir}")
    return 0


def cmd_compute(args: argparse.Namespace) -> int:
    src = Path(args.src)
    images = sorted([*src.glob("*.png"), *src.glob("*.jpg"), *src.glob("*.jpeg")])
    if len(images) < 10:
        print(f"[ERR] {src} 只找到 {len(images)} 张，建议 ≥ 20 张", file=sys.stderr)
        return 2

    pattern = parse_pattern(args.pattern)
    square = float(args.square)

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : pattern[0], 0 : pattern[1]].T.reshape(-1, 2) * square

    obj_points: list[np.ndarray] = []
    img_points: list[np.ndarray] = []
    img_size: tuple[int, int] | None = None
    used: list[str] = []
    skipped: list[str] = []

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    for p in images:
        img = cv2.imread(str(p))
        if img is None:
            skipped.append(f"{p.name} (read fail)")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not found:
            skipped.append(f"{p.name} (no corners)")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners)
        used.append(p.name)

    if len(used) < 10:
        print(f"[ERR] 有效图 {len(used)} 张，<10 张，无法标定", file=sys.stderr)
        return 2

    print(f"[CALIB] 用 {len(used)} 张，跳过 {len(skipped)} 张，开始标定...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )

    per_view_err = []
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
        per_view_err.append(err)
    mean_err = float(np.mean(per_view_err))
    max_err = float(np.max(per_view_err))

    out = {
        "image_size": [int(img_size[0]), int(img_size[1])],
        "pattern_inner_corners": [pattern[0], pattern[1]],
        "square_mm": square,
        "n_used": len(used),
        "n_skipped": len(skipped),
        "rms_reproj_error_px": float(rms),
        "mean_per_view_error_px": mean_err,
        "max_per_view_error_px": max_err,
        "camera_matrix": K.tolist(),
        "dist_coeffs": dist.flatten().tolist(),
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)

    print(f"[DONE] 写入 {out_path}")
    print(
        f"  RMS reprojection error: {rms:.4f} px (越小越好；< 0.5 px 算优秀，< 1.0 px 可用)"
    )
    print(f"  mean / max per-view error: {mean_err:.4f} / {max_err:.4f} px")
    if skipped:
        print(
            f"  跳过 {len(skipped)} 张：{skipped[:3]}{'...' if len(skipped) > 3 else ''}"
        )
    return 0


def cmd_undistort(args: argparse.Namespace) -> int:
    with open(args.params, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)
    K = np.array(params["camera_matrix"])
    dist = np.array(params["dist_coeffs"])

    src = Path(args.src)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    images = sorted([*src.glob("*.png"), *src.glob("*.jpg"), *src.glob("*.jpeg")])
    for p in images:
        img = cv2.imread(str(p))
        if img is None:
            continue
        h, w = img.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0)
        und = cv2.undistort(img, K, dist, None, new_K)
        cv2.imwrite(str(out / p.name), und)
    print(f"[DONE] 去畸变 {len(images)} 张 → {out}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("capture", help="实时采图")
    a.add_argument("--camera", type=int, default=0)
    a.add_argument("--pattern", default="9x6", help="内角点数 WxH，默认 9x6")
    a.add_argument("--out", required=True)

    b = sub.add_parser("compute", help="批量标定")
    b.add_argument("--src", required=True)
    b.add_argument("--pattern", default="9x6")
    b.add_argument("--square", type=float, default=25.0, help="方格边长 mm")
    b.add_argument("--out", default="intrinsics.yaml")

    c = sub.add_parser("undistort", help="去畸变验证")
    c.add_argument("--params", required=True)
    c.add_argument("--src", required=True)
    c.add_argument("--out", required=True)

    return ap


def main() -> int:
    args = build_parser().parse_args()
    return {"capture": cmd_capture, "compute": cmd_compute, "undistort": cmd_undistort}[
        args.cmd
    ](args)


if __name__ == "__main__":
    raise SystemExit(main())
