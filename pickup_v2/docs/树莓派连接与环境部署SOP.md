# 树莓派连接与环境部署 SOP

> 从零到能跑 pickup_v2 标定的**可复现**操作手册。所有命令均于 2026-06-02 实测通过。
> 适用：换电脑、Pi 重装、IP 变动后快速恢复连接与开发环境。

## 0. 设备信息

| 项 | 值 |
|---|---|
| Pi 用户@IP | `<user>@<PI_IP>`（**手机热点网段，IP 会变**，见 §1） |
| Pi 仓库路径 | `~/vs_code/strawberry_grasp` |
| Pi Python 虚拟环境 | `~/robot_env`（`--system-site-packages`） |
| PC SSH 别名 | `pi`（配在 `~/.ssh/config`） |
| PC SSH key | `~/.ssh/id_ed25519`（ed25519） |

---

## 1. 网络连接

PC 和 Pi 必须在**同一局域网**（同一手机热点或路由器）。

```bash
# Pi 上查自己 IP（接显示器或已能连时）：
hostname -I
# PC 上确认能 ping 通：
ping <PI_IP>
```

**IP 变了怎么办**（换热点/重启后常见）：在 Pi 上 `hostname -I` 拿到新 IP，改 PC 的 `~/.ssh/config` 里 `HostName` 即可，别名 `pi` 不变。

---

## 2. SSH 配置（一次性，换电脑才需重做）

### 2.1 配置别名

编辑 `C:\Users\<user>\.ssh\config`（没有就新建），加：

```
Host pi
    HostName <PI_IP>
    User <user>
```

之后所有命令用 `ssh pi` 代替 `ssh <user>@<PI_IP>`。

### 2.2 配置免密登录（key 认证）

**在 Git Bash 里执行**（这步有 Unix 管道，PowerShell 不适用）：

```bash
# ① 检查是否已有 key，没有才生成
ls ~/.ssh/id_ed25519.pub 2>/dev/null || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519

# ② 把公钥装到 Pi（会让你输一次 Pi 密码）
#    ⚠ 必须单行，不要在 chmod 600 后断行（断行会被传到远程拆成两条命令报错）
cat ~/.ssh/id_ed25519.pub | ssh pi "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && echo KEY_INSTALLED"

# ③ 验证免密（不再要密码就成功）
ssh -o BatchMode=yes pi "echo CONNECT_OK"
```

看到 `KEY_INSTALLED` 和 `CONNECT_OK` 即配好。

---

## 3. 开终端：PowerShell vs Git Bash

| 场景 | 用哪个 | 怎么开 |
|---|---|---|
| `ssh pi` 登录、跑远程命令、日常操作 | **PowerShell 即可** | `Win+X` → 终端 |
| 带 Unix 管道/工具的命令（`cat\|ssh`、`scp -r`、`rsync`、`grep`） | **Git Bash** | 开始菜单搜 "Git Bash" |

**关键**：纯 `ssh`/`scp` 在 PowerShell 完全可用，Windows 自带 OpenSSH。只有 Unix 管道语法才必须 Git Bash。

---

## 4. 连接 Pi

```bash
ssh pi                      # 交互登录
ssh pi "命令"                # 跑一条远程命令就返回
ssh -t pi "命令"             # 跑交互式程序（如标定脚本，-t 分配伪终端）
```

---

## 5. Pi 上的 Python 环境

标定/抓取统一用虚拟环境 `~/robot_env`（`--system-site-packages`，能看到系统包）。

```bash
# 直接用绝对路径调用，无需 activate：
~/robot_env/bin/python <脚本>
~/robot_env/bin/pip install <包>
```

**已装依赖**（pickup_v2 标定所需）：

| 包 | 版本 | 用途 |
|---|---|---|
| pyserial | 3.5 | 串口（protocol_v2） |
| PyYAML | 6.0.3 | 标定文件读写（joint_calib / config_v2） |
| ultralytics / opencv / picamera2 / numpy(<2) | — | 视觉（vision 主线，标定不需要） |

**注意（来自踩坑记录）**：Pi 上 piwheels 源曾导致坏 wheel，pip 装包若异常优先指定官方 PyPI；numpy 须 `<2`。

```bash
# 缺包时补装示例：
~/robot_env/bin/pip install pyyaml
```

---

## 6. 传代码 PC → Pi

本机 Git Bash **没有 rsync**，用 `scp -r`。**务必保持 pickup_v2 目录结构**（pi/ 和 calibration/ 平级，否则跨目录相对路径会断）。

```bash
# Git Bash 里执行（路径用 /d/ 风格）：
ssh pi "mkdir -p ~/vs_code/strawberry_grasp/pickup_v2"
scp -r <本地仓库路径>/pickup_v2/pi \
       <本地仓库路径>/pickup_v2/calibration \
       pi:vs_code/strawberry_grasp/pickup_v2/
```

> PowerShell 里 scp 路径写 Windows 风格（如 `<本地仓库路径>/pickup_v2/pi`）也可。

---

## 7. 验证标定环境就绪

```bash
ssh pi "cd ~/vs_code/strawberry_grasp/pickup_v2/calibration && \
        ~/robot_env/bin/python joint_calib.py --help"
```

能打印 usage（不报 ModuleNotFoundError）即环境就绪。

---

## 8. 常用命令速查

```bash
# 连接
ssh pi
ssh -t pi "cd ~/vs_code/strawberry_grasp/pickup_v2/calibration && ~/robot_env/bin/python joint_calib.py base"

# 看 Pi 目录结构
ssh pi "ls ~/vs_code/strawberry_grasp"

# 传单个文件
scp /d/VS_code/projects/strawberry_grasp/pickup_v2/pi/config_v2.py pi:vs_code/strawberry_grasp/pickup_v2/pi/

# 从 Pi 拉文件回 PC（如标定结果）
scp pi:vs_code/strawberry_grasp/pickup_v2/calibration/outputs/joint_offsets.yaml /d/VS_code/projects/strawberry_grasp/pickup_v2/calibration/outputs/
```

---

## 9. 故障排查

| 现象 | 原因 | 解决 |
|---|---|---|
| `ping` 不通 | IP 变了 / 不同网 | Pi `hostname -I` 拿新 IP，改 config |
| `Connection closed by ... port 22` | 免密 key 没装/失效 | 重做 §2.2 |
| 免密仍要密码 | `authorized_keys` 权限错 | `ssh pi "chmod 600 ~/.ssh/authorized_keys"` |
| `ModuleNotFoundError: yaml/serial` | venv 缺包 | `~/robot_env/bin/pip install <包>` |
| `joint_offsets.yaml` 读不到 | 目录结构没保持 | 确认 pi/ 与 calibration/ 平级（§6） |

---

**文档版本** v1.0（2026-06-02）｜配套 `Pi环境整理_MANIFEST.md`（目录整理记录）
