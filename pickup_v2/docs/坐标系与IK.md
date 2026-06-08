# 坐标系与逆运动学

> 本文档是 `pi/coord_transform.py` 和 `pi/kinematics.py` 的实现依据。所有数学符号、单位、约定都在这里定死。

## 1. 4 层坐标系

| 层 | 名称 | 单位 | 原点 | X / Y / Z 方向 |
|---|---|---|---|---|
| ① | 图像坐标 | px | 图像左上角 | X→右、Y→下 |
| ② | 工作面坐标 | mm | 标定时人为指定（建议传送带某固定标记点） | X 沿传送带前进方向；Y 与 X 垂直、向相机左侧；Z 向上（恒为 0） |
| ③ | 臂基坐标 | mm | 机械臂底座旋转轴中心 | HOME 朝前为 X+；右手系定 Y+ 朝臂左侧；Z+ 向上 |
| ④ | 关节角 | deg | — | 见下表 |

## 2. 关节角约定

| 关节 | 通道 | 符号 | 0° 位置 | + 方向 | 物理范围 |
|---|---|---|---|---|---|
| 底座旋转 | CH5 | θ_base | 臂指 X+（朝前） | 俯视逆时针 | 0-270° |
| 肩 | CH1 | θ_shoulder | 大臂朝上（Z+） | 大臂前倾 | 0-180° |
| 肘 | CH0 | θ_elbow | 小臂与大臂共线（伸直） | 小臂回折 | 0-180° |
| 腕俯仰 | CH2 | θ_pitch | 末端与小臂共线 | 末端下俯 | 0-180° |
| 腕旋转 | CH3 | θ_roll | 夹爪开合面朝 X+ | 俯视逆时针 | 0-180° |
| 夹爪 | CH4 | θ_grip | 全开 | 闭合 | 0-180° |

⚠️ 上表是**几何关节角约定**，与舵机 us 值的映射由 `舵机标定记录.csv` 表述（每个关节 reversed/safe_us 不同）。这一映射放在 `kinematics.py` 末尾的 `joint_angle_to_servo_us()`。

## 3. 连杆参数（已知，来自 `docs/机械臂布局与参数.md`）

| 符号 | 值 (mm) | 含义 |
|---|---:|---|
| H1 | 95 | 底座顶面到肩轴 |
| L2 | 130 | 肩轴到肘轴 |
| L3 | 140 | 肘轴到腕俯仰轴 |
| L4 | 45 | 腕俯仰轴到腕旋转轴 |
| L5 | 55 | 腕旋转轴到夹爪根部 |
| Lf | 47 | 夹爪根部到指尖中心（取均值） |

**末端等效连杆：** `L_end = L4 + L5 + Lf = 147 mm`（夹爪垂直向下时，腕俯仰轴到指尖的总长度）

## 4. 像素 → 工作面（一步：单应矩阵）

```python
# coord_transform.py 关键片段（伪代码）
import cv2
import numpy as np
import yaml

def load_calibration(intrinsics_path, homography_path):
    K = np.array(yaml.safe_load(open(intrinsics_path))["camera_matrix"])
    dist = np.array(yaml.safe_load(open(intrinsics_path))["dist_coeffs"])
    H = np.array(yaml.safe_load(open(homography_path))["homography_pixel_to_world_mm"])
    return K, dist, H

def pixel_to_workspace(uv, K, dist, H):
    """图像坐标 (u, v) → 工作面坐标 (Xw, Yw, 0) mm"""
    pt = np.array([[[uv[0], uv[1]]]], dtype=np.float64)
    pt_undist = cv2.undistortPoints(pt, K, dist, P=K)        # 去畸变
    Xw_Yw = cv2.perspectiveTransform(pt_undist, H).flatten()  # 应用 H
    return float(Xw_Yw[0]), float(Xw_Yw[1]), 0.0
```

## 5. 工作面 → 臂基（一步：平移）

```python
def workspace_to_armbase(xyz_w, offset_x_mm, offset_y_mm, offset_z_mm):
    """工作面坐标 → 臂基坐标。offset 由 arm_offset.yaml 指定。"""
    Xa = xyz_w[0] - offset_x_mm
    Ya = xyz_w[1] - offset_y_mm
    Za = xyz_w[2] - offset_z_mm + STRAWBERRY_GRASP_Z_MM   # 抓取点高度 ≈ 草莓半径
    return Xa, Ya, Za
```

⚠️ `offset_*` 三个参数 = 工作面原点在臂基坐标系下的坐标。**用直尺量一次即可**，标定流程见 `标定流程SOP.md`。

## 6. 解析逆运动学（核心算法）

**已知：** 目标 (X, Y, Z) 臂基坐标 mm
**求：** (θ_base, θ_shoulder, θ_elbow, θ_pitch, θ_roll, θ_grip) deg
**约束：** 末端（夹爪指尖）从正上方垂直向下指向目标

### 6.1 步骤 1：底座旋转

```
θ_base = atan2(Y, X)         # 让臂指向目标方位
r = sqrt(X² + Y²)            # 目标在底座旋转面内的水平距离
```

### 6.2 步骤 2：折算腕俯仰轴位置

夹爪垂直向下 → 腕俯仰轴正好在目标点正上方 `L_end` 处：

```
r_wrist = r                   # 腕俯仰轴 r 坐标 = 目标 r（夹爪正下）
z_wrist = Z + L_end           # 腕俯仰轴 z 坐标（臂基系）
```

### 6.3 步骤 3：肩 + 肘 平面 2-link IK

把肩轴当作平面坐标系原点（高度差 H1 已经在 z_wrist 里减掉，所以这里 z_wrist 应当是相对肩轴的高度）：

```
z_rel = z_wrist - H1          # 相对肩轴高度
d = sqrt(r_wrist² + z_rel²)   # 肩轴到腕俯仰轴的直线距离

# 工作空间检查
if d > L2 + L3:               return UNREACHABLE  # 超出最大伸展
if d < abs(L2 - L3):          return UNREACHABLE  # 内部死区
if z_rel < -H1:               return UNREACHABLE  # 末端低于地面（H1 以下不安全）

# 余弦定理求肘内角
cos_elbow = (L2² + L3² - d²) / (2 * L2 * L3)
cos_elbow = clip(cos_elbow, -1.0, 1.0)
θ_elbow_inner = acos(cos_elbow)              # 肘的内夹角（0=伸直, π=折回）

# 选 elbow-up 解（小臂朝上方折）
α = atan2(z_rel, r_wrist)
cos_β = (L2² + d² - L3²) / (2 * L2 * d)
cos_β = clip(cos_β, -1.0, 1.0)
β = acos(cos_β)
θ_shoulder_internal = α + β                  # 肩相对水平面的角度
θ_elbow_internal = π - θ_elbow_inner          # 肘的外角（0=折回, π=伸直）
```

⚠️ `θ_shoulder_internal` 和 `θ_elbow_internal` 是**几何角度**，需要再映射到表 2 的"0° 位置 / + 方向"约定。映射关系在 `kinematics.py` 中实现，单元测试覆盖。

### 6.4 步骤 4：腕俯仰

末端方向角约束（夹爪垂直向下，世界 Z- 方向 = -π/2 from horizontal）：

```
# 在臂的垂直平面内，θ_shoulder + θ_elbow + θ_pitch_geom = -π/2
θ_pitch_geom = -π/2 - θ_shoulder_internal + (π - θ_elbow_internal)
# 化简：
θ_pitch_geom = -π/2 - θ_shoulder_internal - θ_elbow_internal + π
```

⚠️ 上式假设三角度方向同一约定，实现时务必结合表 2 的 + 方向重新对齐。**单元测试用 FK 反算检查**。

### 6.5 步骤 5：腕旋转 + 夹爪

```
θ_roll = 0                    # 默认不旋转，未来扩展用 bbox 长轴定向
θ_grip = GRIPPER_OPEN_ANGLE   # 先开爪，到位后单独发 CLOSE 命令
```

## 7. 正运动学（用于自验证 FK）

输入关节角，求末端位置：

```python
def fk(theta_base, theta_shoulder, theta_elbow, theta_pitch):
    # 在垂直面内（先忽略 base 旋转）
    x_shoulder, z_shoulder = 0, H1
    x_elbow = x_shoulder + L2 * cos(theta_shoulder)
    z_elbow = z_shoulder + L2 * sin(theta_shoulder)
    x_wrist = x_elbow + L3 * cos(theta_shoulder + theta_elbow_signed)
    z_wrist = z_elbow + L3 * sin(theta_shoulder + theta_elbow_signed)
    # 末端在腕俯仰轴正下方 L_end 处（垂直约束）
    x_tip = x_wrist
    z_tip = z_wrist - L_end
    # 应用底座旋转
    X = x_tip * cos(theta_base)
    Y = x_tip * sin(theta_base)
    Z = z_tip
    return X, Y, Z
```

**单元测试核心：**
```python
target = (150, 80, 20)
joints = ik(target)
recovered = fk(joints)
assert np.allclose(target, recovered, atol=0.1)   # 0.1mm 容差
```

## 8. 关节角 → 舵机 us 映射

每个关节的标定数据来自 `docs/舵机标定记录.csv`，结构为：

```python
SERVO_CALIB = {
    JOINT_BASE:     {"reversed": False, "min_us": 500, "max_us": 2500, "min_deg": 0,   "max_deg": 270},
    JOINT_SHOULDER: {"reversed": True,  "min_us": 600, "max_us": 2400, "min_deg": 0,   "max_deg": 180},
    # ...
}

def joint_angle_to_servo_us(joint, angle_deg):
    p = SERVO_CALIB[joint]
    if p["reversed"]:
        angle_deg = (p["max_deg"] + p["min_deg"]) - angle_deg
    angle_clamped = clip(angle_deg, p["min_deg"], p["max_deg"])
    return lerp(angle_clamped, p["min_deg"], p["max_deg"], p["min_us"], p["max_us"])
```

⚠️ **本文档定义算法形态，最终值在阶段 B 测试时用真实舵机 us 数据填表。**

## 9. 安全检查清单（IK 求解前后必跑）

- [ ] 工作空间：`d ∈ [|L2-L3|, L2+L3]`
- [ ] 高度安全：`Z ≥ -H1 + 10mm`（不撞地面）
- [ ] 关节限位：每个 θ ∈ 标定范围
- [ ] 单调性：相邻两次 IK 解任一关节差 < 60°（防飞舵）
- [ ] 数值稳定：所有 sqrt / acos 输入先 clip 到 [-1, 1]
- [ ] NaN / inf 拦截

任一检查失败 → 抛 `IKUnreachableError`，主程序应捕获并发 `NACK UNREACHABLE` 给 MCU（或 Pi 端直接拒绝）。

## 10. 实现文件对照

| 函数 | 文件 | 测试 |
|---|---|---|
| `pixel_to_workspace()` | `pi/coord_transform.py` | `tests/test_coord.py` |
| `workspace_to_armbase()` | `pi/coord_transform.py` | `tests/test_coord.py` |
| `ik(target_xyz)` | `pi/kinematics.py` | `tests/test_kinematics.py` |
| `fk(joints)` | `pi/kinematics.py` | 同上（用于交叉验证） |
| `joint_angle_to_servo_us()` | `pi/kinematics.py` | 标定数据回归测试 |

---
**文档版本：** v1.0（2026-05-09）
