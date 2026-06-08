# 树莓派目录整理记录 (MANIFEST)

> 整理日期：2026-06-02
> 方案：保守（只移动/归档，不删除）
> 整理对象：`<user>@<PI_IP>:~/vs_code/strawberry_grasp/`
> 原则：**只移动/归档，不删除任何文件**；vision 主线代码一律不动。

## 1. 整理背景

Pi 上 `~/vs_code/strawberry_grasp/` 长期是**平铺目录**，三类代码混在一起：
- vision 视觉主线（国赛答辩版，互相 import）
- pickup_v2 旧平铺副本（已被新建的 `pickup_v2/` 正式版取代，重复）
- 独立调试脚本

平铺导致难以分辨哪些在用、哪些是废弃副本。本次按用途归类。

## 2. 移动清单（原路径 → 新路径）

| 原文件 | 新位置 | 类别 | 作用 |
|---|---|---|---|
| `adc_test.py` | `tools/adc_test.py` | 调试脚本 | ADC 读数测试 |
| `pi_uart_test.py` | `tools/pi_uart_test.py` | 调试脚本 | 串口收发测试 |
| `diag_serial.py` | `tools/diag_serial.py` | 调试脚本 | 串口诊断 |
| `config_v2.py`(5/11旧) | `archive_flat/config_v2.py` | 废弃副本 | 被 `pickup_v2/pi/config_v2.py`(新版) 取代 |
| `protocol_v2.py` | `archive_flat/protocol_v2.py` | 废弃副本 | 被 `pickup_v2/pi/protocol_v2.py` 取代 |
| `smoke_test_mcu.py` | `archive_flat/smoke_test_mcu.py` | 废弃副本 | 旧冒烟测试 |
| `test_mcu_protocol.py` | `archive_flat/test_mcu_protocol.py` | 废弃副本 | 被 `pickup_v2/pi/test_mcu_protocol.py` 取代 |
| `__pycache__/` | `archive_flat/__pycache___root/` | 缓存 | 根目录旧字节码缓存（归档不删） |

## 3. 整理后的目录结构

```
~/vs_code/strawberry_grasp/
├── (vision 主线 10 个 .py 留原位)
│   ├── config.py          # 视觉基础配置（被下列 import）
│   ├── camera.py          # → config
│   ├── detector.py        # → config, ultralytics
│   ├── serial_comm.py     # → config
│   ├── main.py            # → config, camera, detector, serial_comm（视觉主程序）
│   ├── capture_dataset.py # → camera, config
│   ├── calibrate.py       # → config, camera, detector（初赛 ROI 校准工具）
│   └── test_camera/color/detector.py
├── models/strawberry_yolov8n.pt   # YOLO 权重 6M（不动）
├── logs/                          # 运行日志 153 个（不动）
├── pickup_v2/                     # 任意位置抓取，正式版
│   ├── pi/          (config_v2 新版, protocol_v2, kinematics, coord_transform, main_pickup ...)
│   └── calibration/ (joint_calib.py, intrinsic_calib.py, homography.py ...)
├── tools/                         # 独立调试脚本（无 import 依赖，可直接跑）
├── archive_flat/                  # 废弃平铺副本 + 旧缓存（保留备查，勿用）
└── MANIFEST.md                    # 本记录
```

## 4. 重要说明

- **vision 主线零改动**：`config/camera/detector/main/serial_comm/calibrate/capture_dataset` 及测试文件全部留在根目录原位，import 关系不变，运行命令不变（`cd ~/vs_code/strawberry_grasp && python main.py`）。
- **pickup_v2 用新路径**：以后跑抓取/标定一律用 `~/vs_code/strawberry_grasp/pickup_v2/`。根目录已无 `config_v2.py`/`protocol_v2.py`——旧命令 `python test_mcu_protocol.py` 失效，改用 `cd pickup_v2/pi && python test_mcu_protocol.py`。
- **archive_flat/ 是死代码**：仅留作历史备查，不要再 import 或运行其中文件。
- **tools/ 脚本独立**：无本地 import 依赖，`cd tools && python adc_test.py` 直接可跑。

## 5. 回滚（如需恢复平铺）

```bash
cd ~/vs_code/strawberry_grasp
mv tools/adc_test.py tools/pi_uart_test.py tools/diag_serial.py .
mv archive_flat/config_v2.py archive_flat/protocol_v2.py \
   archive_flat/smoke_test_mcu.py archive_flat/test_mcu_protocol.py .
mv archive_flat/__pycache___root __pycache__
rmdir tools archive_flat   # 仅当已空
```

## 6. home 根目录整理（2026-06-02 追加）

`/home/<user>` 根目录原有 2 个散落文件，移到新建的 `~/misc/`（只移不删）：

| 原文件 | 新位置 | 说明 |
|---|---|---|
| `minicom.log`(180B) | `~/misc/minicom.log` | 串口日志 |
| `test.wav`(862K) | `~/misc/test.wav` | 测试音频 |

home 根其余条目（`Desktop/Documents/Downloads/...` 系统目录、`robot_env` 虚拟环境、`vs_code` 仓库、`.bashrc/.ssh/.config` 等隐藏配置）**一律未动**。

> 注：`vs_code/` 下还有其它与本项目无关的目录，本次**不处理**。

回滚：`mv ~/misc/minicom.log ~/misc/test.wav ~/ && rmdir ~/misc`
