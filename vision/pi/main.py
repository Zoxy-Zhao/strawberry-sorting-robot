"""
草莓分拣主流水线 — 采图 → YOLO 推理 → 串口发送分类指令
部署到 Pi: ~/vs_code/strawberry_grasp/main.py

用法:
  python main.py              # 正常运行（连接 MCU）
  python main.py --dry-run    # 干跑模式（不发串口，只看检测结果）
  python main.py --preview    # 开启网页实时预览（浏览器打开 http://<Pi_IP>:8080）
  python main.py --dry-run --preview  # 干跑 + 预览
"""

import os
import sys
import time
import logging
import argparse
import threading
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import config
from camera import create_camera
from detector import StrawberryDetector
from serial_comm import SerialComm

# ── 网页预览（MJPEG 推流） ──

PREVIEW_PORT = 8080
_latest_frame = None
_frame_lock = threading.Lock()

# 类别对应的框颜色 (BGR)
_CLASS_COLORS = {
    0: (0, 0, 255),    # ripe → 红色
    1: (0, 165, 255),  # semi_ripe → 橙色
    2: (0, 255, 0),    # unripe → 绿色
}


class MJPEGHandler(BaseHTTPRequestHandler):
    """MJPEG 流式推送"""

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = """<!DOCTYPE html>
<html><head><title>草莓分拣 - 实时预览</title></head>
<body style="margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh">
<img src="/stream" style="max-width:100%;max-height:100vh">
</body></html>"""
            self.wfile.write(html.encode())
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                try:
                    with _frame_lock:
                        frame = _latest_frame
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    _, jpg = cv2.imencode(".jpg", frame,
                                         [cv2.IMWRITE_JPEG_QUALITY, 70])
                    data = jpg.tobytes()
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n".encode())
                    self.wfile.write(b"\r\n")
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    break
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


def start_preview_server():
    server = HTTPServer(("0.0.0.0", PREVIEW_PORT), MJPEGHandler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def update_preview(frame):
    global _latest_frame
    with _frame_lock:
        _latest_frame = frame


def draw_detections(frame, detections):
    """在帧上绘制所有检测框和标签，返回新帧（不修改原帧）
    detections: 单个 Detection 或 Detection 列表
    """
    if not isinstance(detections, list):
        detections = [detections]
    vis = frame.copy()
    for det in detections:
        color = _CLASS_COLORS.get(det.class_id, (255, 255, 255))
        x1, y1, x2, y2 = int(det.x1), int(det.y1), int(det.x2), int(det.y2)
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        label = f"{det.class_name} {det.confidence:.2f}"
        font_scale, thickness = 0.6, 2
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                      font_scale, thickness)
        cv2.rectangle(vis, (x1, y1 - th - 8), (x1 + tw, y1), color, -1)
        cv2.putText(vis, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (255, 255, 255), thickness)
    return vis


def setup_logging():
    """配置日志：终端 + 文件"""
    os.makedirs(config.LOG_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(config.LOG_DIR, f"run_{timestamp}.log")

    logging.basicConfig(
        level=getattr(logging, config.LOG_LEVEL),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(log_file, encoding="utf-8"),
        ],
    )
    return logging.getLogger("main")


def parse_args():
    parser = argparse.ArgumentParser(description="草莓分拣视觉系统")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="干跑模式：只做视觉检测，不发送串口指令",
    )
    parser.add_argument(
        "--preview", action="store_true",
        help="开启网页实时预览（浏览器打开 http://<Pi_IP>:8080）",
    )
    parser.add_argument(
        "--camera", default="csi", choices=["csi", "usb"],
        help="摄像头类型 (默认: csi)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    logger = setup_logging()

    logger.info("=" * 50)
    logger.info("草莓分拣视觉系统启动")
    logger.info("模式: %s", "干跑(不发串口)" if args.dry_run else "正常(连接MCU)")
    logger.info("预览: %s", "开启" if args.preview else "关闭")
    logger.info("=" * 50)

    # ── 启动网页预览 ──
    if args.preview:
        start_preview_server()
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            ip = "Pi的IP"
        logger.info("网页预览已启动: http://%s:%d", ip, PREVIEW_PORT)

    # ── 初始化摄像头 ──
    logger.info("正在初始化摄像头 (%s)...", args.camera)
    cam = create_camera(args.camera)

    # ── 初始化 YOLO 检测器 ──
    logger.info("正在加载 YOLO 模型...")
    detector = StrawberryDetector()

    # ── 初始化串口 ──
    comm = None
    if not args.dry_run:
        logger.info("正在连接 MCU 串口...")
        comm = SerialComm()
        comm.open()
        # 清空缓冲区，不做 Ping（MCU 不支持 P 命令）
        comm.drain_buffer()
        logger.info("MCU 串口已打开")

    # ── 状态机 ──
    # Pi 侧状态：与 MCU 协议配合
    #   SEND_G     → 发 G 启动传送带，等 MCU 回 BELT_ON
    #   DETECTING  → 传送带运行中，持续检测草莓
    #   WAIT_READY → 已发分类指令，等 MCU 完成动作回 READY
    PI_STATE_SEND_G = "SEND_G"
    PI_STATE_DETECTING = "DETECTING"
    PI_STATE_WAIT_READY = "WAIT_READY"

    pi_state = PI_STATE_SEND_G
    frame_count = 0
    detect_count = 0
    fps_start = time.monotonic()
    fps_frames = 0
    confirm_count = 0         # 连续检测到的帧数
    confirm_cmd = None        # 连续检测到的分类指令

    logger.info("主循环开始，Ctrl+C 退出")
    logger.info("-" * 50)

    try:
        while True:
            # 采图
            frame = cam.read_frame()
            frame_count += 1
            fps_frames += 1

            # 计算 FPS（每 30 帧更新一次）
            if fps_frames >= 30:
                elapsed = time.monotonic() - fps_start
                fps = fps_frames / elapsed if elapsed > 0 else 0
                logger.info("FPS: %.1f | 总帧数: %d | 检测次数: %d | 状态: %s",
                            fps, frame_count, detect_count, pi_state)
                fps_start = time.monotonic()
                fps_frames = 0

            # 每帧都做 YOLO 推理（保持预览实时）
            all_dets = detector.detect(frame)
            best_det = all_dets[0] if all_dets else None

            # 更新预览（显示所有检测框 + 触发区域 + 状态）
            if args.preview:
                if all_dets:
                    vis = draw_detections(frame, all_dets)
                else:
                    vis = frame.copy()
                # 画触发区域（绿色矩形）
                if config.DETECT_REGION is not None:
                    rx1, ry1, rx2, ry2 = config.DETECT_REGION
                    h, w = vis.shape[:2]
                    pt1 = (int(rx1 * w), int(ry1 * h))
                    pt2 = (int(rx2 * w), int(ry2 * h))
                    cv2.rectangle(vis, pt1, pt2, (0, 255, 0), 2)
                    cv2.putText(vis, "TRIGGER ZONE", (pt1[0], pt1[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                # 显示状态和确认计数
                status = f"State: {pi_state}"
                if pi_state == PI_STATE_DETECTING and confirm_count > 0:
                    status += f" | Confirm: {confirm_count}/{config.DETECT_CONFIRM_FRAMES}"
                cv2.putText(vis, status, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                update_preview(vis)

            # ── 干跑模式：只检测不发指令 ──
            if args.dry_run:
                if best_det is not None:
                    detect_count += 1
                    logger.info(
                        "检测到: %s (%s) 置信度=%.2f cmd=%s bbox=(%.0f,%.0f,%.0f,%.0f)",
                        best_det.class_cn, best_det.class_name,
                        best_det.confidence, best_det.mcu_cmd,
                        best_det.x1, best_det.y1, best_det.x2, best_det.y2,
                    )
                continue

            # ── 正常模式：Pi 状态机 ──
            if pi_state == PI_STATE_SEND_G:
                # 发 G 启动传送带
                comm.ser.write(b"G")
                comm.ser.flush()
                logger.info("TX → MCU: G (启动传送带)")
                # 等 MCU 回复 BELT_ON
                response = comm.ser.readline()
                if response:
                    text = response.decode("utf-8", errors="replace").strip()
                    logger.info("RX ← MCU: %s", text)
                    if "BELT_ON" in text:
                        pi_state = PI_STATE_DETECTING
                        confirm_count = 0
                        confirm_cmd = None
                        logger.info("传送带已启动，开始检测...")
                    elif "BUSY" in text:
                        logger.warning("MCU 忙，1s 后重试")
                        time.sleep(1.0)
                    else:
                        logger.warning("MCU 回复未预期: %s，1s 后重试", text)
                        time.sleep(1.0)
                else:
                    logger.warning("MCU 无回复，1s 后重试")
                    time.sleep(1.0)

            elif pi_state == PI_STATE_DETECTING:
                # 传送带运行中，等待检测到草莓
                if best_det is None:
                    # 没检测到，重置连续计数
                    confirm_count = 0
                    confirm_cmd = None
                    continue

                detect_count += 1
                logger.info(
                    "检测到: %s (%s) 置信度=%.2f cmd=%s bbox=(%.0f,%.0f,%.0f,%.0f)",
                    best_det.class_cn, best_det.class_name,
                    best_det.confidence, best_det.mcu_cmd,
                    best_det.x1, best_det.y1, best_det.x2, best_det.y2,
                )

                # 连续帧确认：同一类别连续 N 帧才触发
                if best_det.mcu_cmd == confirm_cmd:
                    confirm_count += 1
                else:
                    confirm_count = 1
                    confirm_cmd = best_det.mcu_cmd

                if confirm_count < config.DETECT_CONFIRM_FRAMES:
                    logger.info("确认中: %d/%d", confirm_count, config.DETECT_CONFIRM_FRAMES)
                    continue

                # 确认通过，发送分类指令
                confirm_count = 0
                confirm_cmd = None
                comm.ser.write(best_det.mcu_cmd.encode("ascii"))
                comm.ser.flush()
                logger.info("TX → MCU: %s (已确认，发送分类指令)", best_det.mcu_cmd)
                pi_state = PI_STATE_WAIT_READY
                wait_ready_start = time.monotonic()
                logger.info("等待 MCU 完成抓取动作...")

            elif pi_state == PI_STATE_WAIT_READY:
                # 超时保护：MCU 无响应时自动恢复
                if time.monotonic() - wait_ready_start > config.DETECT_COOLDOWN:
                    logger.error("等待 MCU READY 超时 (%.0fs)，强制回到 SEND_G",
                                 config.DETECT_COOLDOWN)
                    pi_state = PI_STATE_SEND_G
                    continue
                # 非阻塞检查 MCU 是否发回 READY
                if comm.ser.in_waiting > 0:
                    response = comm.ser.readline()
                    if response:
                        text = response.decode("utf-8", errors="replace").strip()
                        logger.info("RX ← MCU: %s", text)
                        if "READY" in text:
                            logger.info("MCU 动作完成，开始下一轮")
                            pi_state = PI_STATE_SEND_G

    except KeyboardInterrupt:
        logger.info("\n收到 Ctrl+C，正在退出...")

    finally:
        cam.close()
        if comm:
            comm.close()
        logger.info("系统已关闭。总帧数=%d，检测次数=%d", frame_count, detect_count)


if __name__ == "__main__":
    main()
