"""
数据采集脚本 — 在传送带场景下用 CSI 摄像头拍摄草莓图片
带网页实时预览，在 Windows 浏览器打开 http://<Pi_IP>:8080 即可看到画面

部署到 Pi: ~/vs_code/strawberry_grasp/capture_dataset.py

用法:
  python capture_dataset.py

操作（在 SSH 终端里输入）:
  r = 拍一张并标记为 ripe（成熟）
  s = 拍一张并标记为 semi_ripe（半成熟）
  u = 拍一张并标记为 unripe（未熟）
  c = 连拍模式（每 0.5s 自动拍一张，按 Enter 停止）
  n = 查看统计
  q = 退出
"""

import os
import sys
import time
import threading
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import serial

from camera import create_camera
import config


# ── 配置 ──
SAVE_DIR = "dataset_captured"
PREVIEW_PORT = 8080


# ── 串口控制（传送带） ──

class BeltController:
    """通过串口控制 MCU 传送带"""

    def __init__(self):
        self.ser = None
        self.running = False

    def connect(self):
        try:
            self.ser = serial.Serial(config.SERIAL_PORT, config.BAUD_RATE, timeout=2)
            print(f"[传送带] 串口已连接: {config.SERIAL_PORT}")
            return True
        except serial.SerialException as e:
            print(f"[传送带] 串口连接失败: {e}")
            print("[传送带] 传送带控制不可用，但拍照功能正常")
            return False

    def start(self):
        if not self.ser or not self.ser.is_open:
            print("[传送带] 串口未连接")
            return
        self.ser.write(b"G")
        self.ser.flush()
        resp = self.ser.readline().decode("utf-8", errors="replace").strip()
        if resp:
            print(f"[传送带] 已启动 (MCU: {resp})")
        else:
            print("[传送带] 已发送启动命令")
        self.running = True

    def stop(self):
        if not self.ser or not self.ser.is_open:
            print("[传送带] 串口未连接")
            return
        self.ser.write(b"X")
        self.ser.flush()
        resp = self.ser.readline().decode("utf-8", errors="replace").strip()
        if resp:
            print(f"[传送带] 已停止 (MCU: {resp})")
        else:
            print("[传送带] 已发送停止命令")
        self.running = False

    def close(self):
        if self.ser and self.ser.is_open:
            if self.running:
                self.stop()
            self.ser.close()

# 类别快捷键映射
KEY_MAP = {
    "r": ("ripe", "成熟"),
    "s": ("semi_ripe", "半成熟"),
    "u": ("unripe", "未熟"),
}

# 全局变量：最新帧（供预览线程读取）
_latest_frame = None
_frame_lock = threading.Lock()


# ── 网页预览服务 ──

class MJPEGHandler(BaseHTTPRequestHandler):
    """MJPEG 流式推送，浏览器打开即可看到实时画面"""

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = """<!DOCTYPE html>
<html><head><title>摄像头预览</title></head>
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
                    time.sleep(0.1)  # ~10fps 预览
                except (BrokenPipeError, ConnectionResetError):
                    break
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass  # 不打印 HTTP 日志，避免刷屏


def start_preview_server():
    """在后台线程启动 MJPEG 预览服务"""
    server = HTTPServer(("0.0.0.0", PREVIEW_PORT), MJPEGHandler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def update_preview(frame):
    """更新预览画面"""
    global _latest_frame
    with _frame_lock:
        _latest_frame = frame


# ── 数据采集逻辑 ──

def ensure_dirs():
    for cls_name, _ in KEY_MAP.values():
        path = os.path.join(SAVE_DIR, cls_name)
        os.makedirs(path, exist_ok=True)
    print(f"[Dataset] 保存目录: {os.path.abspath(SAVE_DIR)}")


def get_next_index(cls_dir: str) -> int:
    existing = [
        f for f in os.listdir(cls_dir)
        if f.endswith((".jpg", ".png"))
    ]
    if not existing:
        return 1
    nums = []
    for f in existing:
        name = os.path.splitext(f)[0]
        parts = name.split("_")
        if parts[-1].isdigit():
            nums.append(int(parts[-1]))
    return max(nums, default=0) + 1


def save_frame(frame, cls_name: str, cls_dir: str) -> str:
    idx = get_next_index(cls_dir)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{cls_name}_{timestamp}_{idx:04d}.jpg"
    filepath = os.path.join(cls_dir, filename)
    cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return filepath


def count_all():
    counts = {}
    for cls_name, cls_cn in KEY_MAP.values():
        cls_dir = os.path.join(SAVE_DIR, cls_name)
        if os.path.exists(cls_dir):
            n = len([f for f in os.listdir(cls_dir) if f.endswith((".jpg", ".png"))])
        else:
            n = 0
        counts[cls_cn] = n
    return counts


def burst_capture(cam, cls_name: str, cls_cn: str, interval: float = 0.5):
    cls_dir = os.path.join(SAVE_DIR, cls_name)
    print(f"\n[连拍] 类别={cls_cn}，间隔={interval}s，按 Enter 停止...")

    import select
    count = 0
    try:
        while True:
            frame = cam.read_frame()
            update_preview(frame)
            filepath = save_frame(frame, cls_name, cls_dir)
            count += 1
            print(f"  [{count}] 已保存: {filepath}")
            time.sleep(interval)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break
    except KeyboardInterrupt:
        pass

    print(f"[连拍] 结束，共拍 {count} 张")


def preview_loop(cam):
    """后台线程持续更新预览画面"""
    while True:
        try:
            frame = cam.read_frame()
            update_preview(frame)
            time.sleep(0.1)
        except Exception:
            break


def main():
    ensure_dirs()

    print("\n正在启动 CSI 摄像头...")
    cam = create_camera("csi")

    # 启动网页预览
    start_preview_server()

    # 启动后台预览刷新线程
    preview_thread = threading.Thread(target=preview_loop, args=(cam,), daemon=True)
    preview_thread.start()

    # 连接 MCU 串口（传送带控制）
    belt = BeltController()
    belt.connect()

    # 获取 Pi 的 IP 地址
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        ip = "Pi的IP"

    print("\n" + "=" * 50)
    print("草莓数据采集工具")
    print("=" * 50)
    print(f"  实时预览: http://{ip}:{PREVIEW_PORT}")
    print("  在 Windows 浏览器里打开上面的地址看画面")
    print("=" * 50)
    print("快捷键:")
    print("  r = 拍 ripe（成熟，红色）")
    print("  s = 拍 semi_ripe（半成熟，转色）")
    print("  u = 拍 unripe（未熟，绿色）")
    print("  c = 进入连拍模式（需先选类别）")
    print("  g = 启动传送带")
    print("  x = 停止传送带")
    print("  n = 查看统计")
    print("  q = 退出")
    print("=" * 50)

    try:
        while True:
            choice = input("\n按键: ").strip().lower()

            if choice in KEY_MAP:
                cls_name, cls_cn = KEY_MAP[choice]
                cls_dir = os.path.join(SAVE_DIR, cls_name)
                with _frame_lock:
                    frame = _latest_frame
                if frame is not None:
                    filepath = save_frame(frame, cls_name, cls_dir)
                    print(f"  已保存 [{cls_cn}]: {filepath}")
                else:
                    print("  错误：摄像头未就绪")

            elif choice == "c":
                print("  选择连拍类别: r=成熟 / s=半成熟 / u=未熟")
                cls_key = input("  类别: ").strip().lower()
                if cls_key in KEY_MAP:
                    cls_name, cls_cn = KEY_MAP[cls_key]
                    burst_capture(cam, cls_name, cls_cn)
                else:
                    print("  无效类别")

            elif choice == "g":
                belt.start()

            elif choice == "x":
                belt.stop()

            elif choice == "n":
                counts = count_all()
                print("  当前采集统计:")
                for cls_cn, n in counts.items():
                    print(f"    {cls_cn}: {n} 张")

            elif choice == "q":
                break

            else:
                print("  无效按键，请输入 r/s/u/c/g/x/n/q")

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        belt.close()
        cam.close()

    counts = count_all()
    print("\n最终统计:")
    for cls_cn, n in counts.items():
        print(f"  {cls_cn}: {n} 张")
    print("完成！")


if __name__ == "__main__":
    main()
