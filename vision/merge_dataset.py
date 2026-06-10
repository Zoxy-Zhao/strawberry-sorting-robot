"""
合并多来源草莓数据集为单个 YOLO 数据集（train/val 划分）

用法（路径按自己的数据位置填写）:
  python merge_dataset.py --field-src <田间精选目录> --bright-img <强光图片目录> \
      --bright-lbl <强光标签目录> --dim-img <弱光图片目录> --dim-lbl <弱光标签目录> \
      --output <输出目录> --preview
"""

import argparse
import random
import shutil
from collections import Counter
from pathlib import Path

BRIGHT_MAP = {
    "ripe": "ripe",
    "semi_ripe": "semi_ripe",
    "unripe": "unripe",
    "mixed": "mixed",
}

DIM_MAP = {
    "ripe": "ripe",
    "semi_ripe": "semi_sipe",
    "unripe": "unripe",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="合并多个数据源为 YOLO train/val 数据集"
    )
    parser.add_argument("--field-src", type=Path, required=True, help="田间精选根目录")
    parser.add_argument("--bright-img", type=Path, required=True, help="强光图片根目录")
    parser.add_argument("--bright-lbl", type=Path, required=True, help="强光标签根目录")
    parser.add_argument("--dim-img", type=Path, required=True, help="弱光图片根目录")
    parser.add_argument("--dim-lbl", type=Path, required=True, help="弱光标签根目录")
    parser.add_argument("--output", type=Path, required=True, help="输出目录")
    parser.add_argument(
        "--val-ratio", type=float, default=0.15, help="验证集比例，默认 0.15"
    )
    parser.add_argument("--seed", type=int, default=42, help="随机种子，默认 42")
    parser.add_argument("--preview", action="store_true", help="仅预览，不复制文件")
    return parser.parse_args()


def warn(msg):
    print(f"[警告] {msg}")


def add_pair(pairs, image_path: Path, label_path: Path, new_stem: str):
    if not label_path.exists():
        warn(f"缺少标签，已跳过: {image_path}")
        return
    pairs.append(
        {
            "img": image_path,
            "lbl": label_path,
            "img_name": f"{new_stem}{image_path.suffix.lower()}",
            "lbl_name": f"{new_stem}.txt",
        }
    )


def collect_field(src: Path):
    img_dir = src / "images"
    lbl_dir = src / "labels"
    if not img_dir.is_dir():
        warn(f"缺少图片目录: {img_dir}")
        return []
    if not lbl_dir.is_dir():
        warn(f"缺少标签目录: {lbl_dir}")
        return []
    pairs = []
    for image_path in sorted(img_dir.glob("*.jpg")):
        add_pair(
            pairs,
            image_path,
            lbl_dir / f"{image_path.stem}.txt",
            f"field_{image_path.stem}",
        )
    return pairs


def collect_grouped(img_root: Path, lbl_root: Path, mapping, prefix: str):
    pairs = []
    for img_sub, lbl_sub in mapping.items():
        img_dir = img_root / img_sub
        lbl_dir = lbl_root / lbl_sub
        if not img_dir.is_dir():
            warn(f"缺少图片目录: {img_dir}")
            continue
        if not lbl_dir.is_dir():
            warn(f"缺少标签目录: {lbl_dir}")
            continue
        for image_path in sorted(img_dir.glob("*.jpg")):
            add_pair(
                pairs,
                image_path,
                lbl_dir / f"{image_path.stem}.txt",
                f"{prefix}{img_sub}_{image_path.stem}",
            )
    return pairs


def count_classes(items):
    counter = Counter()
    for item in items:
        text = item["lbl"].read_text(encoding="utf-8", errors="ignore")
        for line in text.splitlines():
            parts = line.strip().split()
            if parts:
                counter[parts[0]] += 1
    return counter


def print_dist(title, items):
    dist = count_classes(items)
    print(f"\n{title}: {len(items)} 张")
    if not dist:
        print("  类别分布: 无标注框")
        return
    names = {0: "ripe", 1: "semi_ripe", 2: "unripe"}
    print("  类别分布:")
    for cls in sorted(dist, key=lambda x: int(x) if x.isdigit() else x):
        name = names.get(int(cls), cls) if cls.isdigit() else cls
        print(f"    类别 {cls} ({name}): {dist[cls]} 框")


def copy_split(items, out_root: Path, split: str):
    img_dir = out_root / "images" / split
    lbl_dir = out_root / "labels" / split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)
    for item in items:
        shutil.copy2(item["img"], img_dir / item["img_name"])
        shutil.copy2(item["lbl"], lbl_dir / item["lbl_name"])


def main():
    args = parse_args()
    if not 0 < args.val_ratio < 1:
        raise SystemExit("[错误] --val-ratio 必须在 0 和 1 之间")

    print("开始收集数据集...")
    if args.preview:
        print("当前为预览模式：只统计，不复制文件。\n")

    field_pairs = collect_field(args.field_src)
    bright_pairs = collect_grouped(
        args.bright_img, args.bright_lbl, BRIGHT_MAP, "bright_"
    )
    dim_pairs = collect_grouped(args.dim_img, args.dim_lbl, DIM_MAP, "dim_")
    pairs = field_pairs + bright_pairs + dim_pairs

    print(f"田间精选: {len(field_pairs)} 对")
    print(f"强光自采: {len(bright_pairs)} 对")
    print(f"弱光自采: {len(dim_pairs)} 对")
    print(f"合计: {len(pairs)} 对")

    rng = random.Random(args.seed)
    rng.shuffle(pairs)

    total = len(pairs)
    val_count = int(total * args.val_ratio)
    train_items = pairs[val_count:]
    val_items = pairs[:val_count]

    print_dist("训练集", train_items)
    print_dist("验证集", val_items)

    if args.preview:
        print("\n预览结束，未复制任何文件。")
        return

    print(f"\n开始复制到: {args.output}")
    copy_split(train_items, args.output, "train")
    copy_split(val_items, args.output, "val")
    print("合并完成！")


if __name__ == "__main__":
    main()
