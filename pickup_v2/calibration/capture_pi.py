"""Pi 端采图脚本（CSI 摄像头 / picamera2）。

用于阶段 A 标定的两类采图：
    1. 棋盘格采图（相机内参标定，≥ 20 张）
    2. 工作面单张照片（用于 homography.py 打点）

部署位置：拷到 Pi 上 ~/vs_code/strawberry_grasp/pickup_v2/calibration/
运行：在 Pi 端 SSH 终端
    # 棋盘格连拍模式（按 c 保存当前帧，q 退出）
    python capture_pi.py chessboard --pattern 9x6 --out outputs/intrinsic_images

    # 工作面单张
    python capture_pi.py workplane --out outputs/workplane.png

预览：浏览器打开 http://<Pi_IP>:8080 看实时画面（带棋盘格检测框）。

依赖：picamera2（已安装在 Pi 虚拟环境里）+ opencv-python + numpy
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import cv2
import numpy as np


# ── 默认参数 ──
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
PREVIEW_PORT = 8080


# ── 全局共享帧（预览用） ──
_latest_frame: np.ndarray | None = None
_frame_lock = threading.Lock()


def _update_preview(frame: np.ndarray) -> None:
    global _latest_frame
    with _frame_lock:
        _latest_frame = frame


# ── 摄像头封装（独立于 vision/pi/camera.py，不引入旧 config.py） ──


class CameraCSI:
    def __init__(self, width: int = DEFAULT_WIDTH, height: int = DEFAULT_HEIGHT):
        from picamera2 import Picamera2

        self.cam = Picamera2()
        cfg = self.cam.create_preview_configuration(
            main={"size": (width, height), "format": "BGR888"}
        )
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(1.0)
        print(f"[Camera] CSI 启动 {width}x{height}")

    def read_frame(self) -> np.ndarray:
        frame = self.cam.capture_array()
        # picamera2 BGR888 输出实际通道顺序需反转（与现有 camera.py 一致）
        return frame[:, :, ::-1].copy()

    def close(self) -> None:
        self.cam.stop()
        print("[Camera] CSI 已关闭")


# ── MJPEG 预览（与 main.py 同款，简化版） ──


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(
                b"<!DOCTYPE html><html><body style='margin:0;background:#111;'>"
                b"<img src='/stream' style='width:100%;'></body></html>"
            )
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            while True:
                try:
                    with _frame_lock:
                        frame = _latest_frame
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    data = jpg.tobytes()
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data + b"\r\n")
                    time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    break
        else:
            self.send_error(404)

    def log_message(self, format: str, *args) -> None:
        pass


def _start_preview() -> None:
    server = HTTPServer(("0.0.0.0", PREVIEW_PORT), MJPEGHandler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()


def _get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "Pi 的 IP"


def _parse_pattern(s: str) -> tuple[int, int]:
    w, h = s.lower().split("x")
    return int(w), int(h)


# ── 子命令：棋盘格采图 ──


def cmd_chessboard(args: argparse.Namespace) -> int:
    pattern = _parse_pattern(args.pattern)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    cam = CameraCSI(args.width, args.height)
    _start_preview()

    print(
        f"[CHESSBOARD] 模式 {pattern[0]}x{pattern[1]}，预览: http://{_get_local_ip()}:{PREVIEW_PORT}"
    )
    print("  在终端输入 c 保存，q 退出")

    saved = 0
    try:
        while True:
            frame = cam.read_frame()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern, None)

            vis = frame.copy()
            if found:
                cv2.drawChessboardCorners(vis, pattern, corners, found)
            status = f"saved={saved} found={'Y' if found else 'N'}"
            cv2.putText(
                vis,
                status,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0) if found else (0, 0, 255),
                2,
            )
            _update_preview(vis)

            # 非阻塞读 stdin
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip().lower()
                if line == "q":
                    break
                if line == "c":
                    if not found:
                        print("  [SKIP] 未检测到完整棋盘，放弃")
                        continue
                    saved += 1
                    fname = out_dir / f"calib_{saved:03d}.png"
                    cv2.imwrite(str(fname), frame)
                    print(f"  [SAVE] {fname}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()

    print(f"[DONE] 共保存 {saved} 张到 {out_dir}")
    return 0


# ── 子命令：工作面单张照片 ──


def cmd_workplane(args: argparse.Namespace) -> int:
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    cam = CameraCSI(args.width, args.height)
    _start_preview()

    print(f"[WORKPLANE] 预览: http://{_get_local_ip()}:{PREVIEW_PORT}")
    print("  调整工作面 → 终端输入 c 拍一张，q 退出")

    try:
        while True:
            frame = cam.read_frame()
            _update_preview(frame)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip().lower()
                if line == "q":
                    break
                if line == "c":
                    cv2.imwrite(str(out_path), frame)
                    print(f"  [SAVE] {out_path}")
                    break
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()

    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Pi 端 CSI 摄像头采图（棋盘格 / 工作面）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("chessboard", help="棋盘格连拍（用于内参标定）")
    a.add_argument("--pattern", default="9x6")
    a.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    a.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    a.add_argument("--out", default="outputs/intrinsic_images")

    b = sub.add_parser("workplane", help="工作面单张照片（用于单应矩阵打点）")
    b.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    b.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    b.add_argument("--out", default="outputs/workplane.png")

    return ap


def main() -> int:
    args = build_parser().parse_args()
    return {"chessboard": cmd_chessboard, "workplane": cmd_workplane}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
