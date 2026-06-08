# calibration/ — 阶段 A 标定流程

> 详细的 SOP 见 `../docs/标定流程SOP.md`，本文是**操作快速参考**。

## 文件清单

| 文件 | 来源 | 运行环境 |
|---|---|---|
| `intrinsic_calib.py` | 拷自 sort_pro，未改动 | PC（计算 / 验证去畸变） |
| `homography.py` | 拷自 sort_pro，未改动 | PC（交互打点 / 拟合 / 验证） |
| `capture_pi.py` | 本项目新增（picamera2 适配） | **Pi**（CSI 摄像头采图） |
| `outputs/` | 标定结果 | — |

## 工作流（三步走）

### 步骤 1 — 内参标定

**Pi 端（采图）：**
```bash
cd ~/vs_code/strawberry_grasp/pickup_v2/calibration
python capture_pi.py chessboard --pattern 9x6 --out outputs/intrinsic_images
# 浏览器开 http://<Pi_IP>:8080 看预览
# 终端输入 c 保存，q 退出，至少采 20 张不同角度
```

**把图传回 PC：**
```bash
# 在 PC PowerShell（你的 Windows）
scp -r pi@<Pi_IP>:~/vs_code/strawberry_grasp/pickup_v2/calibration/outputs/intrinsic_images `
    D:\VS_code\projects\strawberry_grasp\pickup_v2\calibration\outputs\
```

**PC 端（计算）：**
```bash
cd D:\VS_code\projects\strawberry_grasp\pickup_v2\calibration
python intrinsic_calib.py compute `
    --src outputs/intrinsic_images `
    --pattern 9x6 --square 25 `
    --out outputs/intrinsics.yaml
# 验收：rms_reproj_error_px < 1.0
```

**可选验证：**
```bash
python intrinsic_calib.py undistort `
    --params outputs/intrinsics.yaml `
    --src outputs/intrinsic_images `
    --out outputs/undistorted
# 肉眼看直线变直
```

### 步骤 2 — 工作面单应矩阵

**Pi 端（拍工作面）：**
```bash
# 在传送带上贴 4-9 个标记点（建议直接贴一张棋盘格做交点）
python capture_pi.py workplane --out outputs/workplane.png
# 终端输入 c 拍一张
```

**传回 PC：**
```bash
scp pi@<Pi_IP>:~/vs_code/strawberry_grasp/pickup_v2/calibration/outputs/workplane.png `
    D:\VS_code\projects\strawberry_grasp\pickup_v2\calibration\outputs\
```

**PC 端（打点 + 拟合）：**
```bash
# 鼠标左键点像素，终端输入对应世界坐标 X Y mm
python homography.py pick --image outputs/workplane.png --out outputs/points.csv
# 至少 4 个点（建议 8-9 个，分布发散）

# 拟合
python homography.py fit --src outputs/points.csv --out outputs/homography.yaml
# 验收：mean_reproj_error_mm < 5

# 验证残差
python homography.py verify --params outputs/homography.yaml --src outputs/points.csv
```

### 步骤 3 — 工作面 → 臂基偏移

**手动量直尺，写文件：**

```yaml
# outputs/arm_offset.yaml
offset_x_mm: 180        # 臂基→工作面原点 X，按实测填
offset_y_mm: 0
offset_z_mm: -85        # 工作面所在 Z（通常负值）
strawberry_grasp_z_mm: 18   # 草莓抓取点 Z，从工作面表面起
note: "测量于 YYYY-MM-DD"
```

详细测量步骤见 `../docs/标定流程SOP.md` 第 3 节。

## 验收检查（全做完才算阶段 A 通过）

- [ ] `outputs/intrinsics.yaml` 存在，RMS < 1.0 px
- [ ] `outputs/homography.yaml` 存在，mean_err < 5 mm
- [ ] `outputs/arm_offset.yaml` 三个数已实测
- [ ] 端到端验证：已知物体位置 → 像素 → 工作面 → 臂基坐标，误差 < 8 mm

通过后进阶段 B（IK 算法）。

## 现场重标定

设备搬运后，**只重做步骤 2**（工作面单应矩阵），5 分钟即可。内参和偏移一般不变（除非镜头被拆 / 机械重装）。
