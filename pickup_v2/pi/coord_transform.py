"""坐标变换：图像像素 → 工作面 mm → 臂基 mm。

依赖标定文件：
    intrinsics.yaml   — camera_matrix + dist_coeffs
    homography.yaml   — 像素 → 工作面 mm 的 3x3 单应矩阵
    arm_offset.yaml   — 工作面原点在臂基系下的偏移 + 草莓抓取点 Z

文件未生成时（阶段 A 未跑完）→ load_calibration 抛 FileNotFoundError。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

import config_v2 as cfg


@dataclass(frozen=True)
class Calibration:
    K: np.ndarray  # 3x3 相机内参
    dist: np.ndarray  # (5,) 畸变系数
    H: np.ndarray  # 3x3 像素→工作面 mm 单应矩阵
    offset_x_mm: float  # 工作面原点 X（臂基系）
    offset_y_mm: float
    offset_z_mm: float  # 工作面所在 Z（臂基系，通常负）
    grasp_z_mm: float  # 草莓抓取点 Z 高度（工作面表面之上）


def load_calibration(
    intrinsics_path: Path | None = None,
    homography_path: Path | None = None,
    arm_offset_path: Path | None = None,
) -> Calibration:
    intrinsics_path = intrinsics_path or cfg.INTRINSICS_PATH
    homography_path = homography_path or cfg.HOMOGRAPHY_PATH
    arm_offset_path = arm_offset_path or cfg.ARM_OFFSET_PATH

    for p in (intrinsics_path, homography_path, arm_offset_path):
        if not p.exists():
            raise FileNotFoundError(f"标定文件未生成: {p}（请先跑阶段 A）")

    intr = yaml.safe_load(intrinsics_path.read_text(encoding="utf-8"))
    homo = yaml.safe_load(homography_path.read_text(encoding="utf-8"))
    off = yaml.safe_load(arm_offset_path.read_text(encoding="utf-8"))

    return Calibration(
        K=np.array(intr["camera_matrix"], dtype=np.float64),
        dist=np.array(intr["dist_coeffs"], dtype=np.float64),
        H=np.array(homo["homography_pixel_to_world_mm"], dtype=np.float64),
        offset_x_mm=float(off["offset_x_mm"]),
        offset_y_mm=float(off["offset_y_mm"]),
        offset_z_mm=float(off["offset_z_mm"]),
        grasp_z_mm=float(off["strawberry_grasp_z_mm"]),
    )


def pixel_to_workspace(
    uv: tuple[float, float], calib: Calibration
) -> tuple[float, float, float]:
    """图像像素 (u, v) → 工作面坐标 (Xw, Yw, 0) mm。"""
    pt = np.array([[[float(uv[0]), float(uv[1])]]], dtype=np.float64)
    pt_undist = cv2.undistortPoints(pt, calib.K, calib.dist, P=calib.K)
    xy = cv2.perspectiveTransform(pt_undist, calib.H).flatten()
    return float(xy[0]), float(xy[1]), 0.0


def workspace_to_armbase(
    xyz_w: tuple[float, float, float], calib: Calibration
) -> tuple[float, float, float]:
    """工作面坐标 → 臂基坐标。Z 用工作面 Z + 草莓抓取点高度。"""
    xa = xyz_w[0] - calib.offset_x_mm
    ya = xyz_w[1] - calib.offset_y_mm
    za = calib.offset_z_mm + calib.grasp_z_mm
    return xa, ya, za


def pixel_to_armbase(
    uv: tuple[float, float], calib: Calibration
) -> tuple[float, float, float]:
    """一步直达：像素 → 臂基坐标（包含抓取点 Z 偏移）。"""
    xyz_w = pixel_to_workspace(uv, calib)
    return workspace_to_armbase(xyz_w, calib)


def bbox_to_armbase(
    bbox_xyxy: tuple[float, float, float, float],
    calib: Calibration,
    use_bottom_center: bool = True,
) -> tuple[float, float, float]:
    """YOLO bbox → 臂基坐标。

    use_bottom_center=True：取 bbox 底边中心（草莓接触平面的点）
    use_bottom_center=False：取 bbox 中心
    """
    x1, y1, x2, y2 = bbox_xyxy
    u = (x1 + x2) / 2
    v = y2 if use_bottom_center else (y1 + y2) / 2
    return pixel_to_armbase((u, v), calib)
