"""
田间数据集按类别抽样脚本
从已标注的 YOLO 数据集中按主类别随机抽样指定数量的图片+标签

用法:
  python sample_field_data.py --src <已标注数据集目录> --output <输出目录> --preview
  python sample_field_data.py --src <已标注数据集目录> --output <输出目录> --ripe-count 70 --semi-count 50 --unripe-count 0
"""

import argparse
import random
import shutil
from collections import Counter, defaultdict
from pathlib import Path

CLASS_NAMES = {0: "熟果", 1: "半熟", 2: "青果"}


def parse_args():
    parser = argparse.ArgumentParser(description="按主类别抽样 YOLO 田间数据")
    parser.add_argument("--src", required=True, type=Path, help="源数据集根目录")
    parser.add_argument("--output", required=True, type=Path, help="输出目录")
    parser.add_argument("--ripe-count", type=int, default=70, help="类别 0 抽样数量")
    parser.add_argument("--semi-count", type=int, default=50, help="类别 1 抽样数量")
    parser.add_argument("--unripe-count", type=int, default=0, help="类别 2 抽样数量")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    parser.add_argument("--preview", action="store_true", help="仅预览，不复制文件")
    return parser.parse_args()


def parse_label(label_path: Path):
    present, counts = set(), Counter()
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        try:
            cls_id = int(parts[0])
        except ValueError:
            continue
        if cls_id in CLASS_NAMES:
            present.add(cls_id)
            counts[cls_id] += 1
    return present, counts


def primary_class(counts: Counter):
    return min(counts, key=lambda cls_id: (-counts[cls_id], cls_id))


def collect_groups(src: Path):
    image_dir = src / "images" / "train"
    label_dir = src / "labels" / "train"
    groups = defaultdict(list)
    for label_path in sorted(label_dir.glob("*.txt")):
        if label_path.name == "classes.txt":
            continue
        present, counts = parse_label(label_path)
        if not present:
            print(f"[警告] 跳过空标注或无效标注: {label_path.name}")
            continue
        image_path = image_dir / f"{label_path.stem}.jpg"
        if not image_path.exists():
            print(f"[警告] 找不到对应图片: {image_path.name}")
            continue
        groups[primary_class(counts)].append((image_path, label_path, present, counts))
    return groups


def sample_group(items, count):
    if count >= len(items):
        return list(items)
    return random.sample(items, count)


def main():
    args = parse_args()
    random.seed(args.seed)
    groups = collect_groups(args.src)
    requested = {0: args.ripe_count, 1: args.semi_count, 2: args.unripe_count}

    print("各主类别图片数量：")
    for cls_id in (0, 1, 2):
        print(f"  类别 {cls_id}（{CLASS_NAMES[cls_id]}）: {len(groups[cls_id])} 张")

    selected = {}
    for cls_id in (0, 1, 2):
        chosen = sample_group(groups[cls_id], requested[cls_id])
        selected[cls_id] = chosen
        print(
            f"\n类别 {cls_id}（{CLASS_NAMES[cls_id]}）抽样: "
            f"请求 {requested[cls_id]}，实际 {len(chosen)}"
        )
        if not chosen:
            print("  未选中任何图片")
            continue
        for image_path, _, present, counts in chosen:
            details = ", ".join(f"{k}:{counts[k]}" for k in sorted(present))
            print(f"  - {image_path.name}  类别统计[{details}]")

    if args.preview:
        print("\n预览模式结束：未复制任何文件。")
        return

    out_img = args.output / "images"
    out_lbl = args.output / "labels"
    out_img.mkdir(parents=True, exist_ok=True)
    out_lbl.mkdir(parents=True, exist_ok=True)
    for cls_id in (0, 1, 2):
        for image_path, label_path, _, _ in selected[cls_id]:
            shutil.copy2(image_path, out_img / image_path.name)
            shutil.copy2(label_path, out_lbl / label_path.name)
    total = sum(len(items) for items in selected.values())
    print(f"\n复制完成，共输出 {total} 组图片和标注。")


if __name__ == "__main__":
    main()
