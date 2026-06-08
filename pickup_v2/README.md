# pickup_v2 — 任意位置草莓抓取系统

将草莓抓取从「固定姿态序列」升级为「视觉定位 + 逆运动学」的完全隔离实现。

## 隔离原则

本目录下的所有内容**完全独立于现有的 `vision/pi/` 和 `D:\e2studio_test\Robotic_arm\src\hal_entry.c`**：

- 不修改任何现有文件
- 新主程序 `pi/main_pickup.py` 独立入口，与 `vision/pi/main.py` 互不影响
- 新协议命令字 `M / K / J / OPEN / CLOSE / HOME` 与旧 `A / B / C / G / X` 平行存在
- MCU 端生成完整副本 `mcu/hal_entry_pickup_v2.c`，由你手动决定是否替换
- 想放弃整个功能，删 `pickup_v2/` 即可，原项目零痕迹

## 目录结构

```
pickup_v2/
├── README.md                    # 本文件
├── docs/
│   ├── 设计方案.md               # 总体设计（架构、阶段划分）
│   ├── 坐标系与IK.md             # 4 层坐标系定义 + 解析 IK 推导
│   ├── 标定流程SOP.md            # 现场标定操作手册
│   ├── 协议规范.md               # 扩展串口协议（命令字、时序、错误码）
│   ├── MCU实现说明.md            # hal_entry_pickup_v2.c 的设计要点
│   └── 验收标准.md
├── calibration/                 # 标定代码（拷自 strawberry_sort_pro）
│   ├── intrinsic_calib.py       # 相机内参（棋盘格法）
│   ├── homography.py            # 单应矩阵（像素↔工作面 mm）
│   └── outputs/                 # 标定结果存这里
│       ├── intrinsics.yaml
│       ├── homography.yaml
│       └── arm_offset.yaml      # 工作面原点 → 臂基偏移
├── pi/                          # Pi 端新代码
│   ├── config_v2.py             # 新配置（不 import 旧 config.py）
│   ├── coord_transform.py       # 像素 → 工作面 → 臂基坐标
│   ├── kinematics.py            # 解析逆运动学
│   ├── protocol_v2.py           # 新串口协议封装
│   ├── main_pickup.py           # 新主程序入口
│   └── tests/
│       ├── test_kinematics.py   # FK/IK 交叉验证
│       └── test_coord.py        # 坐标转换测试
└── mcu/
    └── hal_entry_pickup_v2.c    # MCU 端完整副本（不替换原文件）
```

## 阶段执行清单（更新于 2026-05-11）

| 阶段 | 内容 | 验收 | 状态 |
|---|---|---|---|
| A | 标定代码 + 内参/单应矩阵 | RMS<1px, 单应误差<5mm | ✅ 工具就绪 / ⏳ 待 Pi 上跑出 yaml |
| B | Python 端 IK + 单元测试 | FK(IK(p))≈p 误差<0.1mm | ✅ 完成（43/43 全过） |
| C | hal_entry_pickup_v2.c | `K` 命令走指定关节角 | ✅ 完成 + 交叉审查修复 |
| D | Pi-MCU 联调 | 像素→工作面→臂尖目测对齐 | ✅ Pi 代码就绪（85/85 测试） / ⏳ 待硬件联调 |
| E | 实物抓取测试 | 9 网格点抓取成功率 | ⏳ 未开始 |

**进度与技术复盘：** [`NEXT_SESSION.md`](NEXT_SESSION.md)

## 入口文档

先读 [`docs/设计方案.md`](docs/设计方案.md)。

## 与现有项目的关系

- 视觉模型：复用 `vision/pi/detector.py` 的 YOLO 推理（import，不改）
- 摄像头：复用 `vision/pi/camera.py`（import，不改）
- 舵机标定：参考 `docs/舵机标定记录.csv`（不改）
- 连杆尺寸：参考 `docs/机械臂布局与参数.md`（不改）
- MCU 现有代码：完全冻结，新版本作为完整副本独立存在
