"""coord_transform.py 单元测试。

用临时 yaml 文件 mock 标定数据，不依赖阶段 A 的真实标定输出。
"""

from __future__ import annotations

import math

import pytest
import yaml

from coord_transform import (
    bbox_to_armbase,
    load_calibration,
    pixel_to_armbase,
    pixel_to_workspace,
    workspace_to_armbase,
)


# ============================================================
# Fixtures — 制造 mock 标定数据
# ============================================================


@pytest.fixture
def mock_calib_files(tmp_path):
    """生成 3 份 mock yaml：
    - 内参：identity K, 零畸变
    - 单应矩阵：identity（像素值 = 工作面 mm 值，便于推算）
    - 偏移：(100, 0, -80) + grasp_z=18
    """
    intr_path = tmp_path / "intrinsics.yaml"
    homo_path = tmp_path / "homography.yaml"
    off_path = tmp_path / "arm_offset.yaml"

    intr_path.write_text(
        yaml.safe_dump(
            {
                "camera_matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
            }
        )
    )
    homo_path.write_text(
        yaml.safe_dump(
            {
                "homography_pixel_to_world_mm": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
            }
        )
    )
    off_path.write_text(
        yaml.safe_dump(
            {
                "offset_x_mm": 100.0,
                "offset_y_mm": 0.0,
                "offset_z_mm": -80.0,
                "strawberry_grasp_z_mm": 18.0,
            }
        )
    )
    return intr_path, homo_path, off_path


@pytest.fixture
def mock_calib(mock_calib_files):
    return load_calibration(*mock_calib_files)


# ============================================================
# 1. load_calibration
# ============================================================


class TestLoadCalibration:
    def test_load_ok(self, mock_calib):
        assert mock_calib.K.shape == (3, 3)
        assert mock_calib.dist.shape == (5,)
        assert mock_calib.H.shape == (3, 3)
        assert mock_calib.offset_x_mm == 100.0
        assert mock_calib.grasp_z_mm == 18.0

    def test_missing_file_raises(self, tmp_path):
        intr = tmp_path / "no_such.yaml"
        homo = tmp_path / "homo.yaml"
        off = tmp_path / "off.yaml"
        with pytest.raises(FileNotFoundError):
            load_calibration(intr, homo, off)


# ============================================================
# 2. pixel_to_workspace（identity H + identity K → 像素值 = mm 值）
# ============================================================


class TestPixelToWorkspace:
    def test_identity_mapping(self, mock_calib):
        x, y, z = pixel_to_workspace((50, 30), mock_calib)
        assert math.isclose(x, 50.0, abs_tol=0.01)
        assert math.isclose(y, 30.0, abs_tol=0.01)
        assert z == 0.0

    def test_zero_pixel(self, mock_calib):
        x, y, _ = pixel_to_workspace((0, 0), mock_calib)
        assert math.isclose(x, 0.0, abs_tol=0.01)
        assert math.isclose(y, 0.0, abs_tol=0.01)


# ============================================================
# 3. workspace_to_armbase
# ============================================================


class TestWorkspaceToArmbase:
    def test_offset_applied(self, mock_calib):
        # 工作面 (200, 50, 0) → 臂基 X = 200-100 = 100, Y = 50-0 = 50
        # Z = offset_z + grasp_z = -80 + 18 = -62
        x, y, z = workspace_to_armbase((200, 50, 0), mock_calib)
        assert math.isclose(x, 100.0, abs_tol=0.01)
        assert math.isclose(y, 50.0, abs_tol=0.01)
        assert math.isclose(z, -62.0, abs_tol=0.01)

    def test_origin_workspace(self, mock_calib):
        x, y, z = workspace_to_armbase((0, 0, 0), mock_calib)
        assert math.isclose(x, -100.0, abs_tol=0.01)
        assert math.isclose(y, 0.0, abs_tol=0.01)
        assert math.isclose(z, -62.0, abs_tol=0.01)


# ============================================================
# 4. pixel_to_armbase 一站式
# ============================================================


class TestPixelToArmbase:
    def test_combined(self, mock_calib):
        # 像素 (200, 50) → 工作面 (200, 50, 0) → 臂基 (100, 50, -62)
        x, y, z = pixel_to_armbase((200, 50), mock_calib)
        assert math.isclose(x, 100.0, abs_tol=0.01)
        assert math.isclose(y, 50.0, abs_tol=0.01)
        assert math.isclose(z, -62.0, abs_tol=0.01)


# ============================================================
# 5. bbox_to_armbase
# ============================================================


class TestBboxToArmbase:
    def test_bottom_center(self, mock_calib):
        # bbox = (100, 100, 200, 200) → 底边中心 (150, 200)
        # → 工作面 (150, 200, 0) → 臂基 (50, 200, -62)
        x, y, z = bbox_to_armbase(
            (100, 100, 200, 200), mock_calib, use_bottom_center=True
        )
        assert math.isclose(x, 50.0, abs_tol=0.01)
        assert math.isclose(y, 200.0, abs_tol=0.01)
        assert math.isclose(z, -62.0, abs_tol=0.01)

    def test_geometric_center(self, mock_calib):
        # bbox = (100, 100, 200, 200) → 中心 (150, 150)
        x, y, z = bbox_to_armbase(
            (100, 100, 200, 200), mock_calib, use_bottom_center=False
        )
        assert math.isclose(x, 50.0, abs_tol=0.01)
        assert math.isclose(y, 150.0, abs_tol=0.01)


# ============================================================
# 6. 真实 H 矩阵（非 identity）— 模拟实际单应矩阵
# ============================================================


class TestRealisticHomography:
    @pytest.fixture
    def realistic_calib(self, tmp_path):
        """构造一个 H：像素 (cx, cy)=(320, 240) 映射到工作面 (0, 0)，
        像素每 1px 对应工作面 0.5mm（相机俯拍工作面比例约 1:0.5）。
        """
        intr = tmp_path / "intr.yaml"
        homo = tmp_path / "homo.yaml"
        off = tmp_path / "off.yaml"

        intr.write_text(
            yaml.safe_dump(
                {
                    "camera_matrix": [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
                }
            )
        )
        # H 把像素 (320, 240) 映射到 (0, 0)，像素 (321, 240) 映射到 (0.5, 0)
        # H = [[0.5, 0, -160], [0, 0.5, -120], [0, 0, 1]]
        homo.write_text(
            yaml.safe_dump(
                {
                    "homography_pixel_to_world_mm": [
                        [0.5, 0.0, -160.0],
                        [0.0, 0.5, -120.0],
                        [0.0, 0.0, 1.0],
                    ],
                }
            )
        )
        off.write_text(
            yaml.safe_dump(
                {
                    "offset_x_mm": 0.0,
                    "offset_y_mm": 0.0,
                    "offset_z_mm": 0.0,
                    "strawberry_grasp_z_mm": 0.0,
                }
            )
        )
        return load_calibration(intr, homo, off)

    def test_image_center_maps_to_origin(self, realistic_calib):
        x, y, _ = pixel_to_workspace((320, 240), realistic_calib)
        assert math.isclose(x, 0.0, abs_tol=0.01)
        assert math.isclose(y, 0.0, abs_tol=0.01)

    def test_pixel_offset(self, realistic_calib):
        x, y, _ = pixel_to_workspace((420, 240), realistic_calib)
        # 100 px right → 50 mm right
        assert math.isclose(x, 50.0, abs_tol=0.01)
        assert math.isclose(y, 0.0, abs_tol=0.01)
