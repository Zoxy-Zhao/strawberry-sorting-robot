# MCU 实现说明 — hal_entry_pickup_v2.c

> 本文档说明新版 `mcu/hal_entry_pickup_v2.c` 与原 `mcu/hal_entry.c` 的差异和设计要点。**原文件不动**，新版作为完整副本独立存在，由你手动决定是否替换。

## 1. 总体改动概览

新版**保留原文件 100% 功能**（HOME/分类/FSR/TinyML/示教/急停），在以下位置新增能力：

| 改动点 | 类型 | 位置（原文件行号参考） |
|---|---|---|
| 新增行缓冲全局变量 | 新增 | 全局变量区（~241 行附近） |
| 新增 STATE_PICKUP_IDLE / STATE_PICKUP_MOVE 枚举值 | 新增 | state_t 定义（~88 行） |
| 新增 `parse_float_token()` `parse_int_token()` 工具 | 新增 | 工具函数区（~530 行附近） |
| 新增 `cmd_M_handler()` `cmd_K_handler()` 等命令处理函数 | 新增 | process_pi_char 之前 |
| 改造 `process_pi_char()` 为行缓冲 + 单字符兼容 | 修改 | ~1043 行 |
| 主循环新增 STATE_PICKUP_MOVE 分支 | 新增 | ~1542 行 switch |
| 新增 `apply_joint_angles()`（直接走 K 命令的关节角） | 新增 | servo 函数区 |

**复用不改：** `g_servos[]`、所有现有 pose 常量、`servo_set_angle()`、`robot_apply_pose_ex()`、FSR 函数族、TinyML 函数族、示教模式函数族。

## 2. 行缓冲解析逻辑

### 2.1 数据结构

```c
#define PI_CMD_BUF_SIZE 64
static char    g_pi_cmd_buf[PI_CMD_BUF_SIZE];
static uint8_t g_pi_cmd_len = 0;
```

### 2.2 改造后的 process_pi_char

```c
static void process_pi_char(uint8_t rx)
{
    /* 旧协议兼容：单字符命令 + 行缓冲为空 → 立即处理 */
    if (g_pi_cmd_len == 0)
    {
        char c = normalize_upper((char) rx);
        if (c == 'A' || c == 'B' || c == 'C' || c == 'G' || c == 'X')
        {
            process_pi_char_legacy(rx);   /* 抽出原 switch 逻辑成独立函数 */
            return;
        }
    }

    /* 新协议路径：累积到行缓冲，遇 \n 解析 */
    if (rx == '\r')
    {
        return;   /* 忽略 */
    }

    if (rx == '\n')
    {
        if (g_pi_cmd_len > 0)
        {
            g_pi_cmd_buf[g_pi_cmd_len] = '\0';
            parse_extended_cmd(g_pi_cmd_buf);
            g_pi_cmd_len = 0;
        }
        return;
    }

    if (g_pi_cmd_len < PI_CMD_BUF_SIZE - 1)
    {
        g_pi_cmd_buf[g_pi_cmd_len++] = (char) rx;
    }
    else
    {
        /* 行过长，清空并报错 */
        g_pi_cmd_len = 0;
        uart9_send_blocking("NACK BADARG\n");
    }
}
```

### 2.3 命令分发

```c
static void parse_extended_cmd(char * line)
{
    /* 取第一个 token（命令字） */
    char * tok = strtok(line, " \t");
    if (!tok) { uart9_send_blocking("NACK BADARG\n"); return; }

    if      (strcmp(tok, "M") == 0)     cmd_M_handler();
    else if (strcmp(tok, "K") == 0)     cmd_K_handler();
    else if (strcmp(tok, "J") == 0)     cmd_J_handler();
    else if (strcmp(tok, "OPEN") == 0)  cmd_OPEN_handler();
    else if (strcmp(tok, "CLOSE") == 0) cmd_CLOSE_handler();
    else if (strcmp(tok, "HOME") == 0)  cmd_HOME_handler();
    else if (strcmp(tok, "PLACE") == 0) cmd_PLACE_handler();
    else                                 uart9_send_blocking("NACK BADARG\n");
}
```

⚠️ `strtok()` 会修改 `line`，使用后续的 `strtok(NULL, ...)` 取后续 token。

## 3. 命令处理函数（关键三个）

### 3.1 cmd_K_handler — 直接发关节角

```c
static void cmd_K_handler(void)
{
    /* 解析 6 个 float */
    uint16_t angles[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        char * t = strtok(NULL, " \t");
        if (!t) { uart9_send_blocking("NACK BADARG\n"); return; }
        float a = strtof(t, NULL);
        if (a < 0.0f || a > 270.0f) { uart9_send_blocking("NACK SAFETY\n"); return; }
        angles[i] = (uint16_t) a;
    }

    /* 状态机检查 */
    if (g_state != STATE_IDLE && g_state != STATE_PICKUP_IDLE)
    {
        uart9_send_blocking("BUSY\n"); return;
    }

    /* 走 robot_apply_pose_ex 插值 */
    robot_pose_t target;
    for (int i = 0; i < SERVO_COUNT; i++) target.angle_deg[i] = angles[i];
    fsp_err_t err = robot_apply_pose_slow(&target);
    if (err != FSP_SUCCESS) { uart9_send_blocking("NACK SAFETY\n"); return; }

    g_current_pose = target;
    uart9_send_blocking("READY\n");
}
```

### 3.2 cmd_M_handler — 工作面坐标（MCU 不算 IK，仅作语法糖）

**首版决定：** Pi 端已经算好 IK，所以 MCU 端 `M X Y Z` 等价于报错 — Pi 端实际上发的是 K 命令。

```c
static void cmd_M_handler(void)
{
    /* 首版不实现 MCU 端 IK，直接拒绝 */
    uart9_send_blocking("NACK BADARG\n");
    /* 备注：Pi 端在 protocol_v2.py 里转换 M → K 后再发 */
}
```

⚠️ 如果未来想把 IK 搬到 MCU，这里替换实现即可，协议不变。

### 3.3 cmd_J_handler — 单关节调试

```c
static void cmd_J_handler(void)
{
    char * t1 = strtok(NULL, " \t");
    char * t2 = strtok(NULL, " \t");
    if (!t1 || !t2) { uart9_send_blocking("NACK BADARG\n"); return; }
    int ch = atoi(t1);
    float angle = strtof(t2, NULL);
    if (ch < 0 || ch >= SERVO_COUNT) { uart9_send_blocking("NACK BADARG\n"); return; }
    if (angle < 0 || angle > 270)    { uart9_send_blocking("NACK SAFETY\n"); return; }

    fsp_err_t err = servo_set_angle((joint_id_t) ch, (uint16_t) angle);
    if (err != FSP_SUCCESS) { uart9_send_blocking("NACK SAFETY\n"); return; }
    g_current_pose.angle_deg[ch] = (uint16_t) angle;
    uart9_send_blocking("READY\n");
}
```

### 3.4 cmd_OPEN/CLOSE/HOME/PLACE_handler

简短直接：
- OPEN：`servo_set_angle(JOINT_GRIPPER, GRIPPER_OPEN_ANGLE)`
- CLOSE：调用现有 `fsr_grasp_with_feedback()`
- HOME：`robot_apply_pose_slow(&g_pose_home)`
- PLACE A → `robot_apply_pose_slow(&g_pose_place_a)`，B/C 同理

## 4. 状态机扩展

新增两个状态：

```c
typedef enum e_state
{
    STATE_IDLE,
    STATE_BELT_RUN,
    STATE_BELT_STOP,
    STATE_PRE_GRASP,
    STATE_GRASP,
    STATE_LIFT,
    STATE_PLACE,
    STATE_RETURN,
    STATE_PICKUP_IDLE,    /* 新增：Pi 主导抓取流程的待机 */
    STATE_PICKUP_MOVE     /* 新增：正在执行 M/K 命令的移动 */
} state_t;
```

**新版抓取流程（K 命令路径）的状态转换：**
```
STATE_IDLE
  ├ 收到 G\n → STATE_BELT_RUN（旧路径）
  └ 收到 K/M/HOME/PLACE → STATE_PICKUP_MOVE → 执行完毕 → STATE_PICKUP_IDLE
```

进入 STATE_PICKUP_IDLE 后 MCU 等待 Pi 下一条命令；收到 `G\n` 可回到旧路径。

## 5. 烧录与测试

### 5.1 烧录方式（不变）

```bash
# 用 pyocd（来自 project_uart_config.md 记忆）
pyocd flash -t r7fa6m5bf2 build/Debug/Robotic_arm.elf
```

### 5.2 增量验证步骤

1. **冒烟：** 烧录后用 PuTTY 连 SCI9（115200），手动发 `HOME\n` → 应回 `READY\n`
2. **单关节：** 发 `J 5 135\n` → 底座转到 135°，目测正确
3. **完整 K：** 发 `K 135 70 70 150 80 120\n` → 应等同 PRE_GRASP 姿态
4. **旧协议兼容：** 发 `G`（不带 \n） → 应回 `BELT_ON\n`（确认旧路径未被破坏）
5. **错误处理：** 发 `K 9999 0 0 0 0 0\n` → 应回 `NACK SAFETY\n`

### 5.3 回滚方案

如果新版有问题，**重新烧录原 hal_entry.c 即可回到旧版本**，硬件没有任何不可逆改动。

## 6. 编译注意事项

- 新版多用 `<string.h>` (strtok, strcmp) 和 `<stdlib.h>` (strtof, atoi)
- e2studio FSP 项目这两个头文件默认可用，无需额外配置
- 全局静态缓冲（`g_pi_cmd_buf[64]`）增量内存 64 字节，可忽略
- 编译 warning 重点关注：未使用变量、隐式转换

---
**文档版本：** v1.0（2026-05-09）
