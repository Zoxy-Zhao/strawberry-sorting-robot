# 关节 OFFSET/SIGN 标定 SOP

> 单关节扫描标定法。用 `joint_calib.py` 交互式跑，每个关节 2~3 个目标几何角，拟合 `servo_deg = OFFSET + SIGN * geom_deg`。

## 0. 为什么需要这个标定

`config_v2.py` 里的 `JOINT_GEOM_OFFSET_DEG` / `JOINT_GEOM_SIGN` 决定**几何角度（IK 输出）↔ 舵机角度（PCA9685 输入）** 的换算关系。装臂时机械零位 ≠ 舵机零位、舵机正方向 ≠ 几何正方向都很常见，这两组常量必须用真实姿态反推。

老版本（cfbd63c 之前）用 HOME 姿态反推不可行——HOME 末端不满足"垂直朝下"约束，几何上 IK 不可达。所以改用**单关节独立标定**：一次只动一个关节，用量角器/水平仪直接读几何角。

## 1. 前置条件

- [ ] MCU 烧录的是 `hal_entry_pickup_v2.c`（cfbd63c 或更新版）
- [ ] Pi ↔ MCU 串口连通（SCI9，115200）
- [ ] 桌面/工作台留出至少 60 × 60 cm 让臂自由摆动
- [ ] 工具：
  - 手机数字水平仪 app（推荐"水平仪"或"Bubble Level"，量角度精度 ±0.5°）
  - 直角尺一把（量 90° 弯折角）
  - 直尺（量 base 朝向用）
  - 记号笔 + 胶带（在桌面标"正前方"参考线）

## 2. 标定顺序（依赖链）

```
base (独立)
 ↓
shoulder (需要 base 朝向)
 ↓
elbow (需要 shoulder 几何角)
 ↓
wrist_pitch (需要 shoulder + elbow 几何角)
```

**强烈建议一次跑完**——中途断开，重新跑要确认前序关节没被碰过。

## 3. 安全规则

- 每次微调 servo **最大步长 5°**，防止飞舵
- 标定中如果听到舵机异响或看到机械干涉，立刻 Ctrl+C，发 `X\n` 急停
- 标 shoulder/elbow 时小臂可能扫到桌面，**留出至少 20cm 余量**
- 整套标定结束前不要切电源，否则要重测

## 4. 操作流程

### 4.1 启动脚本

```bash
# 在 Pi 上
cd ~/strawberry_grasp/pickup_v2/calibration
python joint_calib.py all
# 或单独标某个关节：
python joint_calib.py base
python joint_calib.py shoulder
python joint_calib.py elbow
python joint_calib.py wrist_pitch
```

### 4.2 交互命令

每个标定点会进入微调循环，可用命令：

| 输入 | 动作 |
|---|---|
| `+1` / `-1` | servo 微调 ±1° |
| `+5` / `-5` | servo 微调 ±5° |
| `=70` | 直接设 servo 为 70°（绝对值）|
| `done` | 当前 servo 值满足目标几何角，记录并进下一点 |
| `skip` | 放弃当前关节，保留旧值 |
| `abort` | 急停 + 退出整个标定 |

### 4.3 每个关节的标定点

| 关节 | 目标几何角 | 怎么测 | 起始 servo |
|---|---|---|---|
| **base** | θ=0° | 臂指向桌面"正前方"参考线 | 135 |
| | θ=+45° | 用直尺量臂相对正前方左转 45° | 拟合点 |
| **shoulder** | α_s=0° | 水平仪贴大臂，读 0°（水平）| 70 |
| | α_s=+45° | 水平仪读 +45° | 拟合点 |
| **elbow** | α_e=0° | 大臂小臂共线（看一条直线）| 0 |
| | α_e=+90° | 直角尺卡大小臂内角=90° | 拟合点 |
| **wrist_pitch** | α_p=0° | 水平仪贴末端，与小臂方向读数差=0° | 90 |
| | α_p=-45° | 末端相对小臂下俯 45° | 拟合点 |

**关键约定**（与 `kinematics.py` 一致）：
- α_s：大臂相对**水平面**的角度，0=水平朝外，+90=竖直朝上
- α_e：大臂和小臂的**弯折角**，0=完全伸直（共线），180=折成一点
- α_p：末端相对**小臂方向**的俯仰，0=共线，负=末端下俯

## 5. 输出

每个关节标定完成后写一个 yaml：

```yaml
# outputs/joint_calib_shoulder.yaml
joint: shoulder
joint_id: 1
offset_deg: 65.2
sign: 1
calibration_points:
  - geom_deg: 0.0
    servo_deg: 65.2
  - geom_deg: 45.0
    servo_deg: 110.5
fit_residual_deg: 0.3
calibrated_at: "2026-05-16T10:30:00"
```

四个关节都标定后，运行：

```bash
python joint_calib.py merge
```

生成合并文件，`config_v2.py` 启动时会自动加载（找不到则用默认值）。

## 6. 验收

合并完后，跑自验证：

```bash
python joint_calib.py verify
```

会用刚标定的 OFFSET/SIGN 算几个测试姿态的舵机值，让你**目测对比**机械臂位置。验收标准：
- 大臂水平时舵机算出的值，把臂打过去后水平仪读数误差 < 3°
- 末端垂直朝下时，末端方向偏离铅垂线 < 5°
- 若超差，重做对应关节

## 7. 完成后

1. 把 `outputs/joint_offsets.yaml` commit 到仓库
2. 更新 memory `project_pickup_v2.md`，标记阶段 E 的 OFFSET/SIGN 标定完成
3. 进入阶段 E 下一步：相机标定（已有 SOP）+ 端到端验证
