"""
全局配置 — 所有可调参数集中管理
部署到 Pi: ~/vs_code/strawberry_grasp/config.py
"""

# ── 串口 ──
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 5  # 秒

# ── 摄像头（CSI via libcamera） ──
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# ── YOLO 模型 ──
MODEL_PATH = "models/strawberry_yolov8n.pt"
CONFIDENCE_THRESHOLD = 0.5
IOU_THRESHOLD = 0.45

# ── 类别映射 → MCU 指令 ──
# class_id: (英文名, 中文名, MCU指令)
CLASS_MAP = {
    0: ("ripe",      "成熟",   "A"),
    1: ("semi_ripe", "半成熟", "B"),
    2: ("unripe",    "未熟",   "C"),
}

# ── 检测控制 ──
DETECT_COOLDOWN = 12.0    # 发送指令后的冷却时间（秒），等 MCU 完成抓取
MIN_BBOX_AREA = 0.005     # 最小 bbox 面积占比（过滤远处小目标）
DETECT_REGION = (0.28, 0.60, 0.53, 0.95)  # 画面底部中偏左，草莓到夹爪前方触发
DETECT_CONFIRM_FRAMES = 4 # 连续 N 帧检测到才触发，防误触

# ── 日志 ──
LOG_DIR = "logs"
LOG_LEVEL = "INFO"
