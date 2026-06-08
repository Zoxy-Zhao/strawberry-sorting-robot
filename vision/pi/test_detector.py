"""
离线推理测试 — 用图片文件测试 YOLO 模型（不需要摄像头）
部署到 Pi: ~/vs_code/strawberry_grasp/test_detector.py

用法:
  python test_detector.py                          # 测试 test_frames/ 目录
  python test_detector.py --dir dataset_captured/  # 测试自采数据
  python test_detector.py --image some_photo.jpg   # 测试单张图片
"""

import os
import sys
import time
import argparse
import glob

import cv2

from detector import StrawberryDetector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="test_frames", help="图片目录")
    parser.add_argument("--image", default=None, help="单张图片路径")
    args = parser.parse_args()

    detector = StrawberryDetector()

    # 收集图片路径
    if args.image:
        paths = [args.image]
    else:
        patterns = ["*.jpg", "*.png", "*.jpeg"]
        paths = []
        for pat in patterns:
            paths.extend(glob.glob(os.path.join(args.dir, "**", pat), recursive=True))
        paths.sort()

    if not paths:
        print(f"未找到图片。目录: {args.dir}")
        sys.exit(1)

    print(f"找到 {len(paths)} 张图片，开始推理...\n")

    total_time = 0.0
    results_summary = {"ripe": 0, "semi_ripe": 0, "unripe": 0, "none": 0}

    for path in paths:
        frame = cv2.imread(path)
        if frame is None:
            print(f"  跳过（无法读取）: {path}")
            continue

        t0 = time.monotonic()
        det = detector.detect_best(frame)
        t1 = time.monotonic()
        infer_ms = (t1 - t0) * 1000
        total_time += infer_ms

        if det:
            print(f"  {os.path.basename(path):30s} → "
                  f"{det.class_cn}({det.class_name}) "
                  f"conf={det.confidence:.2f} "
                  f"cmd={det.mcu_cmd} "
                  f"time={infer_ms:.0f}ms")
            results_summary[det.class_name] += 1
        else:
            print(f"  {os.path.basename(path):30s} → 未检测到目标 time={infer_ms:.0f}ms")
            results_summary["none"] += 1

    n = len(paths)
    avg_ms = total_time / n if n > 0 else 0
    avg_fps = 1000 / avg_ms if avg_ms > 0 else 0

    print(f"\n{'=' * 50}")
    print(f"测试结果:")
    print(f"  图片总数: {n}")
    print(f"  平均推理时间: {avg_ms:.0f}ms")
    print(f"  等效 FPS: {avg_fps:.1f}")
    print(f"  检测统计:")
    for cls, count in results_summary.items():
        print(f"    {cls:12s}: {count}")

    if avg_fps >= 5:
        print(f"\n  状态: 通过 (FPS={avg_fps:.1f} >= 5)")
    else:
        print(f"\n  状态: 警告 (FPS={avg_fps:.1f} < 5)")


if __name__ == "__main__":
    main()
