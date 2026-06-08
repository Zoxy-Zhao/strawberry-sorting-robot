# 标定流程 SOP

> 一次性标定（首次部署）和现场重标定（搬运后）的完整操作手册。每一步都有验收标准，标定不过关不进下一步。

## 0. 标定前置条件

- [ ] 摄像头已固定在最终位置（俯拍传送带），位姿不会再变
- [ ] 机械臂底座已固定在最终位置
- [ ] 传送带已就位
- [ ] PC 上 Python 环境装好 `opencv-python`、`numpy`、`pyyaml`
- [ ] 准备 A4 打印的棋盘格（9×6 内角点，方格 25mm，刚性背板贴牢）
- [ ] 准备 4-9 张圆点贴纸或打印靶（用于工作面标定）
- [ ] 直尺或卡尺（量工作面到臂基的偏移）

## 1. 阶段 A1：相机内参标定

### 1.1 采集棋盘格图像

```bash
cd D:\VS_code\projects\strawberry_grasp\pickup_v2\calibration
python intrinsic_calib.py capture --camera 0 --pattern 9x6 \
       --out outputs/intrinsic_images
```

**操作：**
- 把棋盘格放在摄像头视野内不同位置和角度
- 每个角度等画面里的棋盘角点都能被画绿框（说明检测到了），按 `c` 保存
- **采集 ≥ 20 张**，覆盖：
  - 左上 / 右上 / 左下 / 右下 / 中心
  - 倾斜（上下倾、左右倾、对角倾各 2-3 张）
  - 远近（近景占满画面、远景占 1/3 画面）
- 完成后按 `q` 退出

**验收：**
- [ ] 至少 20 张图保存到 `outputs/intrinsic_images/`
- [ ] 每张图人眼看都能找出完整棋盘

### 1.2 计算内参

```bash
python intrinsic_calib.py compute --src outputs/intrinsic_images \
       --pattern 9x6 --square 25 --out outputs/intrinsics.yaml
```

**输出关键字段：**
```yaml
rms_reproj_error_px: 0.42       # 越小越好
camera_matrix: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
dist_coeffs: [k1, k2, p1, p2, k3]
```

**验收：**
- [ ] `rms_reproj_error_px < 1.0`（< 0.5 优秀）
- [ ] 输出文件 `outputs/intrinsics.yaml` 存在

**不过关怎么办：** 重采集，重点补倾斜角度和边缘位置的图。

### 1.3 去畸变验证

```bash
python intrinsic_calib.py undistort --params outputs/intrinsics.yaml \
       --src outputs/intrinsic_images --out outputs/undistorted
```

肉眼对比 `intrinsic_images/` 和 `undistorted/` 中同一张图：直线变直了说明去畸变有效。

## 2. 阶段 A2：工作面单应矩阵标定

### 2.1 在传送带平面布点

**布点要求：**
- 4 个角 + 4 个中点，共 8-9 个点（覆盖整个抓取工作区）
- 每个点是一张直径 ~20mm 的圆点贴纸或打印靶
- **建议用棋盘格直接贴在传送带上**（已有交点，直接拿坐标）

**坐标系约定（建议）：**
- 原点：传送带最靠近机械臂的一侧的中心点
- X+ 方向：沿传送带前进方向
- Y+ 方向：从机械臂看过去的左侧
- 单位：mm

**例如棋盘格 9×6, 25mm：**
- 取角点 (0,0), (200,0), (0,125), (200,125), (100,62.5)... 等（自己挑分布发散的）

### 2.2 拍一张工作面照片

```bash
# 把摄像头摆好，工作面整个铺平，不要遮挡
python intrinsic_calib.py capture --camera 0 --out outputs/workplane_image
# 按 c 保存一张就够（按 q 退出）
mv outputs/workplane_image/calib_001.png outputs/workplane.png
```

### 2.3 交互式打点

```bash
python homography.py pick --image outputs/workplane.png --out outputs/points.csv
```

**操作：**
- 鼠标左键依次点击工作面上每个标记点
- 终端会问 "对应世界坐标 X Y mm"，输入实测的 mm 坐标，例如 `100 50`
- 全部点完按 `q` 完成（至少 4 个，建议 8-9 个）

### 2.4 拟合单应矩阵

```bash
python homography.py fit --src outputs/points.csv --out outputs/homography.yaml
```

**输出关键字段：**
```yaml
n_inliers: 8/9
mean_reproj_error_mm: 2.3       # 越小越好
max_reproj_error_mm: 4.1
homography_pixel_to_world_mm: [[...3x3 matrix...]]
```

**验收：**
- [ ] `mean_reproj_error_mm < 5`（< 8 可接受）
- [ ] `max_reproj_error_mm < 8`
- [ ] inliers ≥ 总点数 80%

### 2.5 验证

```bash
python homography.py verify --params outputs/homography.yaml \
       --src outputs/points.csv
```

每个点的 gt vs pred 对比表 + 总残差。`*` 标记的点（误差 > 3mm）需要重新检查贴纸位置或像素点击精度。

```bash
# 单点测试：随便点一个像素，看输出 mm 是否合理
python homography.py apply --params outputs/homography.yaml --uv 320 240
```

## 3. 阶段 A3：工作面 → 臂基偏移

### 3.1 量出三个数

用直尺/卡尺量：

| 量项 | 含义 | 单位 | 典型值 |
|---|---|---|---|
| `offset_x_mm` | 工作面原点的 X 坐标在臂基坐标系下 | mm | 取决于布局 |
| `offset_y_mm` | Y 坐标 | mm | 取决于布局 |
| `offset_z_mm` | 工作面所在平面的 Z 坐标（臂基系；通常负值，因为传送带低于底座顶面） | mm | -50 ~ -100 |

**操作：**
1. 把机械臂打到 HOME 姿态
2. 用直尺量出底座旋转轴中心到工作面原点（你贴的那个 0,0 标记点）的水平距离 → `offset_x_mm`
3. 量左右偏移 → `offset_y_mm`
4. 量底座顶面到传送带表面的高度差 → `-offset_z_mm`（注意符号）

### 3.2 写入文件

```yaml
# pickup_v2/calibration/outputs/arm_offset.yaml
offset_x_mm: 180      # 示例值，按实测填
offset_y_mm: 0
offset_z_mm: -85
strawberry_grasp_z_mm: 18    # 草莓抓取点高度（草莓表面到夹爪指尖）
note: "测量于 2026-XX-XX，三角板量"
```

### 3.3 端到端验证

```bash
# 把一颗已知位置的物体（例如棋盘格 (100, 50)）放在工作面上
# 拍照 → 检测到像素位置 → 应用 H → 减偏移 → 应该接近实际位置
python -c "
import yaml, numpy as np, cv2
H = np.array(yaml.safe_load(open('outputs/homography.yaml'))['homography_pixel_to_world_mm'])
off = yaml.safe_load(open('outputs/arm_offset.yaml'))
uv = (320, 240)  # 替换为实测像素
pt = np.array([[[uv[0], uv[1]]]], dtype=np.float64)
xy = cv2.perspectiveTransform(pt, H).flatten()
print(f'像素 {uv} → 工作面 ({xy[0]:.1f}, {xy[1]:.1f}) mm → 臂基 ({xy[0]-off[\"offset_x_mm\"]:.1f}, {xy[1]-off[\"offset_y_mm\"]:.1f})')
"
```

**验收：** 输出值与实测位置误差 < 8mm

## 4. 阶段 A4：抓取点 Z 高度标定

### 4.1 手动找抓取点

1. 把一颗草莓放在工作面上
2. 用示教模式（按 `T` 键进入）把机械臂手动开到能稳稳夹住草莓的位置
3. 量此时夹爪指尖到工作面表面的高度差（即夹爪指尖的 Z = 草莓抓取点 Z）
4. 这就是 `strawberry_grasp_z_mm`，写到 `arm_offset.yaml`

⚠️ 这个值会随着草莓大小变化，取一个**平均值**（15-20mm 通常够用）即可，V 型夹爪 + 软指会吃掉差异。

## 5. 现场 5 分钟重标定 SOP（运输后）

如果设备搬运过，**只需重做单应矩阵（步骤 2）**：

1. 摄像头位姿可能漂了，但内参没变（镜头没拆）
2. 工作面位姿可能漂了，但偏移变化在 ±5mm 内（大致可接受）
3. **重做步骤 2.2 → 2.5**（拍照 → 打点 → 拟合 → 验证）
4. 时间：5 分钟

## 6. 标定结果文件总览

完成后 `pickup_v2/calibration/outputs/` 应包含：

```
outputs/
├── intrinsic_images/             # 棋盘格采集图（≥20）
├── undistorted/                   # 去畸变验证图
├── workplane.png                  # 工作面照片（用于打点）
├── points.csv                     # 工作面点对（像素 ↔ mm）
├── intrinsics.yaml                # 内参 + 畸变系数
├── homography.yaml                # 单应矩阵
└── arm_offset.yaml                # 工作面 → 臂基偏移
```

## 7. 常见问题

| 问题 | 排查方向 |
|---|---|
| RMS > 1.0 | 棋盘格不平整 / 采图过少 / 没覆盖边角 |
| 单应误差 > 5mm | 标记点位置不准 / 像素点击不准 / 摄像头有运动模糊 |
| 端到端误差 > 10mm | 优先怀疑 offset 测量 → 重新量直尺 |
| 整体偏一个方向 | 工作面坐标系定义反了（X/Y 方向） |
| 光照变化导致检测漂 | 标定时和实际工作时光照尽量一致 |

---
**文档版本：** v1.0（2026-05-09）
