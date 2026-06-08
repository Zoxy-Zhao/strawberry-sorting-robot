"""
摄像头测试脚本 — 验证 CSI 摄像头能否正常采图
部署到 Pi: ~/vs_code/strawberry_grasp/test_camera.py

用法:
  python test_camera.py           # 测试 CSI 摄像头
  python test_camera.py --usb     # 测试 USB 摄像头
"""

import os
import sys
import time
import argparse

import cv2

from camera import create_camera


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--usb", action="store_true", help="使用 USB 摄像头")
    parser.add_argument("--frames", type=int, default=100, help="采集帧数")
    parser.add_argument("--save", type=int, default=5, help="保存前 N 帧到文件")
    args = parser.parse_args()

    backend = "usb" if args.usb else "csi"
    print(f"正在启动 {backend.upper()} 摄像头...")

    cam = create_camera(backend)

    save_dir = "test_frames"
    os.makedirs(save_dir, exist_ok=True)

    print(f"开始采集 {args.frames} 帧...")
    start = time.monotonic()
    saved = 0

    try:
        for i in range(args.frames):
            frame = cam.read_frame()

            if saved < args.save:
                path = os.path.join(save_dir, f"frame_{i:04d}.jpg")
                cv2.imwrite(path, frame)
                print(f"  帧 {i}: shape={frame.shape}, 已保存 → {path}")
                saved += 1

    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)
    finally:
        cam.close()

    elapsed = time.monotonic() - start
    fps = args.frames / elapsed if elapsed > 0 else 0
    print(f"\n结果:")
    print(f"  采集帧数: {args.frames}")
    print(f"  总耗时: {elapsed:.2f}s")
    print(f"  平均 FPS: {fps:.1f}")
    print(f"  保存到: {save_dir}/ ({saved} 帧)")

    if fps >= 5:
        print("  状态: 通过 (FPS >= 5)")
    else:
        print("  状态: 警告 (FPS < 5，可能影响实时检测)")


if __name__ == "__main__":
    main()
