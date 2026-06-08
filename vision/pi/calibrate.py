"""
触发区域快速校准工具
在 Pi 上运行，浏览器打开预览，终端按键实时调节绿色触发框位置。
调好后按 s 保存到 config.py。

用法:
  python calibrate.py              # 默认步进 0.02
  python calibrate.py --step 0.05  # 大步进，快速粗调
"""

import sys
import time
import threading
import argparse
import re
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2

import config
from camera import create_camera
from detector import StrawberryDetector

# ── 全局状态 ──
_latest_frame = None
_frame_lock = threading.Lock()

# 当前触发区域（可变）
region = list(config.DETECT_REGION) if config.DETECT_REGION else [0.25, 0.60, 0.55, 0.95]

# 类别颜色
_CLASS_COLORS = {
    0: (0, 0, 255),
    1: (0, 165, 255),
    2: (0, 255, 0),
}


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = """<!DOCTYPE html>
<html><head><title>校准工具</title></head>
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

    def log_message(self, fmt, *args):
        pass


def update_frame(frame):
    global _latest_frame
    with _frame_lock:
        _latest_frame = frame


def draw_overlay(frame, detections, step):
    vis = frame.copy()
    h, w = vis.shape[:2]

    # 画所有检测框
    for det in detections:
        color = _CLASS_COLORS.get(det.class_id, (255, 255, 255))
        x1, y1, x2, y2 = int(det.x1), int(det.y1), int(det.x2), int(det.y2)
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        label = f"{det.class_name} {det.confidence:.2f}"
        cv2.putText(vis, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)

        # 画检测框中心点
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        cv2.circle(vis, (cx, cy), 5, color, -1)

        # 判断中心是否在触发区域内
        nx, ny = cx / w, cy / h
        in_zone = region[0] <= nx <= region[2] and region[1] <= ny <= region[3]
        status = "IN ZONE" if in_zone else "OUT"
        cv2.putText(vis, status, (x1, y2 + 16), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0) if in_zone else (0, 0, 255), 2)

    # 画触发区域
    pt1 = (int(region[0] * w), int(region[1] * h))
    pt2 = (int(region[2] * w), int(region[3] * h))
    cv2.rectangle(vis, pt1, pt2, (0, 255, 0), 2)
    cv2.putText(vis, "TRIGGER ZONE", (pt1[0], pt1[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # 左上角显示当前参数
    info = f"Region: ({region[0]:.2f}, {region[1]:.2f}, {region[2]:.2f}, {region[3]:.2f})  Step: {step:.2f}"
    cv2.putText(vis, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    return vis


def save_to_config():
    """把当前 region 写回 config.py"""
    config_path = "config.py"
    with open(config_path, "r", encoding="utf-8") as f:
        content = f.read()

    new_value = f"({region[0]:.2f}, {region[1]:.2f}, {region[2]:.2f}, {region[3]:.2f})"
    new_content = re.sub(
        r"DETECT_REGION\s*=\s*\([\d., ]+\)",
        f"DETECT_REGION = {new_value}",
        content,
    )

    with open(config_path, "w", encoding="utf-8") as f:
        f.write(new_content)

    print(f"\n[SAVED] DETECT_REGION = {new_value} → config.py")


def print_help(step):
    print("\n" + "=" * 55)
    print("  触发区域校准工具")
    print("=" * 55)
    print(f"  当前: ({region[0]:.2f}, {region[1]:.2f}, {region[2]:.2f}, {region[3]:.2f})")
    print(f"  步进: {step:.2f}")
    print("-" * 55)
    print("  整体移动:")
    print("    w/s = 上/下移    a/d = 左/右移")
    print("  调节大小:")
    print("    i/k = 上边 上/下    o/l = 下边 上/下")
    print("    j/; = 左边 左/右    u/p = 右边 左/右")
    print("  其他:")
    print("    +/- = 增大/减小步进")
    print("    r   = 重置为默认区域")
    print("    v   = 保存并退出")
    print("    q   = 不保存退出")
    print("    h   = 显示帮助")
    print("=" * 55)


def main():
    parser = argparse.ArgumentParser(description="触发区域校准工具")
    parser.add_argument("--step", type=float, default=0.02, help="调节步进 (默认 0.02)")
    args = parser.parse_args()
    step = args.step

    # 启动摄像头
    print("[CAL] 正在初始化摄像头...")
    cam = create_camera("csi")

    # 启动检测器
    print("[CAL] 正在加载 YOLO 模型...")
    detector = StrawberryDetector()

    # 启动预览服务器
    server = HTTPServer(("0.0.0.0", 8080), MJPEGHandler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()

    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        ip = "Pi的IP"
    print(f"[CAL] 预览: http://{ip}:8080")

    # 启动采图+推理线程
    running = True

    def capture_loop():
        while running:
            frame = cam.read_frame()
            dets = detector.detect(frame)
            vis = draw_overlay(frame, dets, step)
            update_frame(vis)

    threading.Thread(target=capture_loop, daemon=True).start()

    print_help(step)

    # 终端输入循环
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while True:
            ch = sys.stdin.read(1)

            if ch == 'w':  # 整体上移
                region[1] = max(0.0, region[1] - step)
                region[3] = max(region[1] + 0.05, region[3] - step)
            elif ch == 's':  # 整体下移
                region[1] = min(region[3] - 0.05, region[1] + step)
                region[3] = min(1.0, region[3] + step)
            elif ch == 'a':  # 整体左移
                region[0] = max(0.0, region[0] - step)
                region[2] = max(region[0] + 0.05, region[2] - step)
            elif ch == 'd':  # 整体右移
                region[0] = min(region[2] - 0.05, region[0] + step)
                region[2] = min(1.0, region[2] + step)
            elif ch == 'i':  # 上边上移（框变高）
                region[1] = max(0.0, region[1] - step)
            elif ch == 'k':  # 上边下移（框变矮）
                region[1] = min(region[3] - 0.05, region[1] + step)
            elif ch == 'o':  # 下边下移（框变高）
                region[3] = min(1.0, region[3] + step)
            elif ch == 'l':  # 下边上移（框变矮）
                region[3] = max(region[1] + 0.05, region[3] - step)
            elif ch == 'j':  # 左边左移（框变宽）
                region[0] = max(0.0, region[0] - step)
            elif ch == ';':  # 左边右移（框变窄）
                region[0] = min(region[2] - 0.05, region[0] + step)
            elif ch == 'u':  # 右边右移（框变宽）
                region[2] = min(1.0, region[2] + step)
            elif ch == 'p':  # 右边左移（框变窄）
                region[2] = max(region[0] + 0.05, region[2] - step)
            elif ch == '+' or ch == '=':
                step = min(0.20, step + 0.01)
            elif ch == '-':
                step = max(0.01, step - 0.01)
            elif ch == 'r':  # 重置
                region[:] = [0.25, 0.60, 0.55, 0.95]
                print("[CAL] 已重置为默认区域")
            elif ch == 'v':  # 保存并退出
                save_to_config()
                break
            elif ch == 'q':  # 不保存退出
                print("\n[CAL] 未保存，退出")
                break
            elif ch == 'h':
                print_help(step)
                continue
            else:
                continue

            # 打印当前值
            sys.stdout.write(f"\r  Region: ({region[0]:.2f}, {region[1]:.2f}, {region[2]:.2f}, {region[3]:.2f})  Step: {step:.2f}  ")
            sys.stdout.flush()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        running = False
        cam.close()
        print("\n[CAL] 已关闭")


if __name__ == "__main__":
    main()
