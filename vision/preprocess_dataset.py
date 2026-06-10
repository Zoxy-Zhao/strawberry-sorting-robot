"""
草莓数据集预处理：模糊过滤 + 统一重命名
在 Windows PC 上运行

用法:
  python preprocess_dataset.py --input <原始图片目录> --output <输出目录> --preview
  python preprocess_dataset.py --input <原始图片目录> --output <输出目录>
"""

import argparse
import shutil
from pathlib import Path

import cv2

CLASSES = ("ripe", "semi_ripe", "unripe")
# 额外支持的非标准文件夹（如"多个"），会被重命名为 mixed
EXTRA_DIRS = {"多个": "mixed"}


def parse_args():
    parser = argparse.ArgumentParser(
        description="草莓数据集预处理：模糊过滤 + 统一重命名"
    )
    parser.add_argument("--input", required=True, type=Path, help="输入根目录")
    parser.add_argument("--output", required=True, type=Path, help="输出根目录")
    parser.add_argument(
        "--blur-threshold", type=float, default=50.0, help="模糊阈值，默认 50"
    )
    parser.add_argument("--preview", action="store_true", help="仅预览统计，不复制文件")
    return parser.parse_args()


def blur_score(image_path: Path) -> float:
    gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise ValueError("无法读取图像")
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def hist_lines(scores, bins=8, width=24):
    if not scores:
        return ["  无数据"]
    low, high = min(scores), max(scores)
    if low == high:
        return [
            f"  {low:7.2f}-{high:7.2f} | {'#' * min(len(scores), width)} ({len(scores)})"
        ]
    step = (high - low) / bins
    counts = [0] * bins
    for score in scores:
        idx = min(int((score - low) / step), bins - 1)
        counts[idx] += 1
    scale = max(counts) or 1
    lines = []
    for i, count in enumerate(counts):
        start = low + i * step
        end = high if i == bins - 1 else low + (i + 1) * step
        bar = "#" * int(count / scale * width) if count else ""
        lines.append(f"  {start:7.2f}-{end:7.2f} | {bar} ({count})")
    return lines


def collect_images(class_dir: Path):
    items = []
    images = sorted(
        p for p in class_dir.iterdir() if p.is_file() and p.suffix.lower() == ".jpg"
    )
    for image_path in images:
        try:
            items.append((image_path, blur_score(image_path)))
        except ValueError:
            print(f"[警告] 跳过无法读取的文件: {image_path}")
    return items


def print_class_report(cls, items, threshold, preview):
    scores = [score for _, score in items]
    kept = [(path, score) for path, score in items if score >= threshold]
    removed = len(items) - len(kept)
    print(f"\n[{cls}] 原始数量: {len(items)}")
    if scores:
        mean_score = sum(scores) / len(scores)
        print(
            f"  模糊分数统计: 最小={min(scores):.2f}  最大={max(scores):.2f}  均值={mean_score:.2f}"
        )
        print("  分数直方图:")
        for line in hist_lines(scores):
            print(line)
    else:
        print("  未找到 .jpg 图片")
    action = "预计剔除" if preview else "剔除"
    print(f"  {action}数量(< {threshold:.2f}): {removed}")
    if preview:
        print("  逐图分数预览:")
        for path, score in items:
            status = "保留" if score >= threshold else "剔除"
            print(f"    {path.name}: {score:.2f} -> {status}")
    return len(items), removed, len(kept), kept


def print_summary(summary):
    print("\n处理汇总:")
    print(f"{'类别':<12}{'处理前':>8}{'剔除':>8}{'处理后':>8}")
    print("-" * 36)
    for cls in summary:
        before, removed, after = summary[cls]
        print(f"{cls:<12}{before:>8}{removed:>8}{after:>8}")


def main():
    args = parse_args()
    if args.input.resolve() == args.output.resolve() and not args.preview:
        raise SystemExit("[错误] 输入目录和输出目录不能相同")
    print("开始扫描数据集...")
    if args.preview:
        print("当前为预览模式：仅输出统计信息，不会复制或删除任何文件。")

    # 收集所有要处理的文件夹：标准类别 + 额外文件夹
    all_dirs = [(cls, cls) for cls in CLASSES]
    for src_name, dst_name in EXTRA_DIRS.items():
        extra_dir = args.input / src_name
        if extra_dir.is_dir():
            all_dirs.append((src_name, dst_name))

    summary = {}
    kept_map = {}
    for src_name, dst_name in all_dirs:
        class_dir = args.input / src_name
        if not class_dir.is_dir():
            print(f"\n[警告] 缺少类别目录: {class_dir}")
            items = []
        else:
            items = collect_images(class_dir)
        before, removed, after, kept = print_class_report(
            dst_name, items, args.blur_threshold, args.preview
        )
        summary[dst_name] = (before, removed, after)
        kept_map[dst_name] = kept

    if not args.preview:
        print("\n开始复制并统一重命名...")
        for dst_name, kept in kept_map.items():
            out_dir = args.output / dst_name
            out_dir.mkdir(parents=True, exist_ok=True)
            for idx, (src, _) in enumerate(kept, start=1):
                dst = out_dir / f"{dst_name}_{idx:04d}.jpg"
                shutil.copy2(src, dst)
            print(f"[{dst_name}] 已保留并复制 {len(kept)} 张到 {out_dir}")
        print("\n最终各类别保留数量:")
        for dst_name in summary:
            print(f"  {dst_name}: {summary[dst_name][2]} 张")

    print_summary(summary)


if __name__ == "__main__":
    main()
