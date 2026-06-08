# mcu/ — MCU 端代码副本

本目录存放新版 MCU 代码，**不会自动覆盖**原 `D:\e2studio_test\Robotic_arm\src\hal_entry.c`。

## 文件清单

| 文件 | 说明 |
|---|---|
| `hal_entry_pickup_v2.c` | 完整副本 + pickup_v2 改动（1942 行）|

## 与原文件的差异

基于 `D:\e2studio_test\Robotic_arm\src\hal_entry.c`（1771 行）派生，新增：

- 文件头注释（pickup_v2 标识）
- `#include <stdlib.h>` (strtof / atoi)
- `STATE_PICKUP_IDLE` / `STATE_PICKUP_MOVE` 加入 state_t enum
- `PI_CMD_BUF_SIZE = 64` define
- `g_pi_cmd_buf` / `g_pi_cmd_len` 行缓冲全局变量
- `process_pi_char_legacy()` — 抽出旧单字符协议处理
- 7 个新命令处理器：
  - `cmd_K_handler()` / `cmd_M_handler()` / `cmd_J_handler()`
  - `cmd_OPEN_handler()` / `cmd_CLOSE_handler()`
  - `cmd_HOME_handler()` / `cmd_PLACE_handler()`
- `parse_extended_cmd()` — 命令分发器（strtok 解析）
- `process_pi_char()` 改写为行缓冲 + 单字符兼容
- 主循环 switch 加 `STATE_PICKUP_IDLE` / `STATE_PICKUP_MOVE` 的空 case

**未改动：** g_servos[] 标定、所有 pose 常量、I2C/PCA9685 函数、舵机控制、FSR 反馈、TinyML、示教模式、急停。

## 烧录与验证

详见 `../docs/MCU实现说明.md` 第 5 节。快速版：

1. **导入**：在 e2studio 把这个文件覆盖到 `D:\e2studio_test\Robotic_arm\src\hal_entry.c`（建议先备份原文件）
2. **编译**：Project → Build Project，期望 0 error
3. **烧录**：
   ```bash
   pyocd flash -t r7fa6m5bf2 build/Debug/Robotic_arm.elf
   ```
4. **冒烟测试**（PuTTY 连 SCI9 / 115200）：
   - `HOME\n` → `READY\n`
   - `J 5 90\n` → 底座转到 90°
   - `K 135 70 70 150 80 120\n` → 等同 PRE_GRASP 姿态
   - 旧协议：`G`（不带 \n）→ `BELT_ON\n`（兼容性确认）
   - 错误：`K 9999 0 0 0 0 0\n` → `NACK SAFETY\n`

## 回滚

如果新版有问题，**烧录原 hal_entry.c 即可**回到旧版本。硬件无任何不可逆改动。

## 推荐：编译前先做代码审查

C 代码改动较大（多个新函数 + 行缓冲逻辑），建议在 e2studio 编译前完整审查一遍。重点检查：

- strtok 的 NULL pointer 处理
- 浮点解析失败的 strtof 行为（返回 0 vs 异常）
- 缓冲溢出（PI_CMD_BUF_SIZE = 64 是否够用）
- 状态机切换的中断安全性
- ISR 上下文里访问 g_pi_cmd_buf 的并发问题
