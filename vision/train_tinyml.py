"""
TinyML 夹取状态分类器 — 训练脚本
=================================
用合成 FSR 力曲线数据训练 tiny MLP，导出 C 头文件供 RA6M5 端侧推理。

模型架构: 16 → 8 → 4 → 3  (总参数量 187)
输入:     最近 16 步的 FSR delta 值（归一化 /1000）
输出:     3 类 — STABLE_GRASP / SLIP_RISK / OVERFORCE

用法:
    python train_tinyml.py                          # 合成数据训练
    python train_tinyml.py --data fsr_log.csv       # 真实数据训练
    python train_tinyml.py --out path/to/weights.h  # 指定输出路径
"""

import argparse
import os
import numpy as np

# ── 超参 ──
WINDOW_SIZE = 16
H1_SIZE = 8
H2_SIZE = 4
NUM_CLASSES = 3
LEARNING_RATE = 0.05
EPOCHS = 800
SAMPLES_PER_CLASS = 300

CLASS_NAMES = ["STABLE_GRASP", "SLIP_RISK", "OVERFORCE"]

# 默认输出到仓库 mcu/ 目录（烧录时随固件一起复制到 e2studio 工程 src/）
DEFAULT_OUTPUT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "mcu", "tinyml_weights.h"
)


# ================================================================
#  合成数据生成
# ================================================================
def generate_synthetic_data(n_per_class=SAMPLES_PER_CLASS, seed=42):
    """基于 FSR 物理特性生成三类合成力曲线。

    Class 0 - STABLE_GRASP:
        接近阶段低噪声 → 接触后 delta 平稳上升到 400-700 区间并稳定。
    Class 1 - SLIP_RISK:
        始终低 delta（< 250），无明显接触或接触后快速下降。
    Class 2 - OVERFORCE:
        接触后 delta 快速攀升到 800+ 以上。
    """
    rng = np.random.RandomState(seed)
    X, y = [], []

    for _ in range(n_per_class):
        # ── Class 0: STABLE ──
        seq = np.zeros(WINDOW_SIZE, dtype=np.float32)
        contact = rng.randint(5, 11)
        for i in range(contact):
            seq[i] = rng.uniform(0, 0.08)
        for i in range(contact, WINDOW_SIZE):
            progress = (i - contact + 1) / (WINDOW_SIZE - contact)
            seq[i] = 0.35 + progress * rng.uniform(0.1, 0.35)
            seq[i] += rng.normal(0, 0.025)
        seq = np.clip(seq, 0, 2.0)
        X.append(seq)
        y.append(0)

        # ── Class 1: SLIP_RISK ──
        seq = np.zeros(WINDOW_SIZE, dtype=np.float32)
        pattern = rng.choice(["no_contact", "weak_contact"])
        if pattern == "no_contact":
            for i in range(WINDOW_SIZE):
                seq[i] = rng.uniform(0, 0.15)
        else:
            contact = rng.randint(8, 14)
            for i in range(contact):
                seq[i] = rng.uniform(0, 0.1)
            for i in range(contact, WINDOW_SIZE):
                seq[i] = rng.uniform(0.15, 0.3)
                seq[i] += rng.normal(0, 0.05)
        seq = np.clip(seq, 0, 2.0)
        X.append(seq)
        y.append(1)

        # ── Class 2: OVERFORCE ──
        seq = np.zeros(WINDOW_SIZE, dtype=np.float32)
        contact = rng.randint(3, 8)
        for i in range(contact):
            seq[i] = rng.uniform(0, 0.08)
        for i in range(contact, WINDOW_SIZE):
            progress = (i - contact + 1) / (WINDOW_SIZE - contact)
            seq[i] = 0.4 + progress * rng.uniform(0.5, 1.2)
            seq[i] += rng.normal(0, 0.04)
        seq = np.clip(seq, 0, 2.5)
        X.append(seq)
        y.append(2)

    return np.array(X, dtype=np.float32), np.array(y, dtype=np.int64)


def load_csv_data(filepath):
    """加载真实 FSR 数据 CSV（由 MCU 串口采集）。

    CSV 格式: 每行 17 列，前 16 列是 delta 值，最后 1 列是标签 (0/1/2)
    """
    data = np.loadtxt(filepath, delimiter=",", dtype=np.float32)
    X = data[:, :WINDOW_SIZE] / 1000.0
    y = data[:, -1].astype(np.int64)
    return X, y


# ================================================================
#  Numpy MLP 训练
# ================================================================
def relu(x):
    return np.maximum(0, x)


def softmax(x):
    e = np.exp(x - x.max(axis=1, keepdims=True))
    return e / e.sum(axis=1, keepdims=True)


def train(X, y, epochs=EPOCHS, lr=LEARNING_RATE):
    rng = np.random.RandomState(0)
    n_samples = len(y)

    # Xavier 初始化
    w1 = rng.randn(WINDOW_SIZE, H1_SIZE).astype(np.float32) * np.sqrt(2.0 / WINDOW_SIZE)
    b1 = np.zeros(H1_SIZE, dtype=np.float32)
    w2 = rng.randn(H1_SIZE, H2_SIZE).astype(np.float32) * np.sqrt(2.0 / H1_SIZE)
    b2 = np.zeros(H2_SIZE, dtype=np.float32)
    w3 = rng.randn(H2_SIZE, NUM_CLASSES).astype(np.float32) * np.sqrt(2.0 / H2_SIZE)
    b3 = np.zeros(NUM_CLASSES, dtype=np.float32)

    for epoch in range(epochs):
        # ── Forward ──
        z1 = X @ w1 + b1
        a1 = relu(z1)
        z2 = a1 @ w2 + b2
        a2 = relu(z2)
        z3 = a2 @ w3 + b3
        probs = softmax(z3)

        # ── Loss ──
        loss = -np.log(probs[np.arange(n_samples), y] + 1e-8).mean()

        # ── Backward ──
        dz3 = probs.copy()
        dz3[np.arange(n_samples), y] -= 1.0
        dz3 /= n_samples

        dw3 = a2.T @ dz3
        db3 = dz3.sum(axis=0)
        da2 = dz3 @ w3.T
        dz2 = da2 * (z2 > 0).astype(np.float32)

        dw2 = a1.T @ dz2
        db2 = dz2.sum(axis=0)
        da1 = dz2 @ w2.T
        dz1 = da1 * (z1 > 0).astype(np.float32)

        dw1 = X.T @ dz1
        db1 = dz1.sum(axis=0)

        # ── SGD Update ──
        w1 -= lr * dw1
        b1 -= lr * db1
        w2 -= lr * dw2
        b2 -= lr * db2
        w3 -= lr * dw3
        b3 -= lr * db3

        if epoch % 100 == 0 or epoch == epochs - 1:
            preds = np.argmax(probs, axis=1)
            acc = (preds == y).mean() * 100
            print(f"  Epoch {epoch:4d}/{epochs}  loss={loss:.4f}  acc={acc:.1f}%")

    return w1, b1, w2, b2, w3, b3


# ================================================================
#  导出 C 头文件
# ================================================================
def fmt_array_1d(name, arr):
    vals = ", ".join(f"{v:.6f}f" for v in arr.flatten())
    return f"static const float {name}[{len(arr)}] = {{{vals}}};\n"


def fmt_array_2d(name, arr):
    rows, cols = arr.shape
    lines = [f"static const float {name}[{rows}][{cols}] = {{"]
    for r in range(rows):
        vals = ", ".join(f"{v:.6f}f" for v in arr[r])
        comma = "," if r < rows - 1 else ""
        lines.append(f"    {{{vals}}}{comma}")
    lines.append("};\n")
    return "\n".join(lines)


def export_c_header(w1, b1, w2, b2, w3, b3, filepath):
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("/* ================================================================\n")
        f.write(" *  tinyml_weights.h — Auto-generated by train_tinyml.py\n")
        f.write(" *  DO NOT EDIT — re-run train_tinyml.py to regenerate\n")
        f.write(" *\n")
        f.write(
            f" *  Architecture: {WINDOW_SIZE} -> {H1_SIZE} -> {H2_SIZE} -> {NUM_CLASSES}\n"
        )
        total = (
            WINDOW_SIZE * H1_SIZE
            + H1_SIZE
            + H1_SIZE * H2_SIZE
            + H2_SIZE
            + H2_SIZE * NUM_CLASSES
            + NUM_CLASSES
        )
        f.write(f" *  Total params: {total}\n")
        f.write(
            " * ================================================================ */\n\n"
        )
        f.write("#ifndef TINYML_WEIGHTS_H\n#define TINYML_WEIGHTS_H\n\n")

        f.write(f"#define TINYML_INPUT_LEN    {WINDOW_SIZE}U\n")
        f.write(f"#define TINYML_H1_SIZE      {H1_SIZE}U\n")
        f.write(f"#define TINYML_H2_SIZE      {H2_SIZE}U\n")
        f.write(f"#define TINYML_NUM_CLASSES   {NUM_CLASSES}U\n\n")

        f.write("/* Layer 1: input -> hidden1 */\n")
        f.write(fmt_array_2d("tinyml_w1", w1))
        f.write(fmt_array_1d("tinyml_b1", b1))
        f.write("\n/* Layer 2: hidden1 -> hidden2 */\n")
        f.write(fmt_array_2d("tinyml_w2", w2))
        f.write(fmt_array_1d("tinyml_b2", b2))
        f.write("\n/* Output: hidden2 -> classes */\n")
        f.write(fmt_array_2d("tinyml_w3", w3))
        f.write(fmt_array_1d("tinyml_b3", b3))

        f.write("\n#endif /* TINYML_WEIGHTS_H */\n")

    print(f"\n  Weights exported to: {filepath}")
    print(f"  Total parameters: {total}")
    size_bytes = total * 4
    print(f"  Flash usage: {size_bytes} bytes ({size_bytes / 1024:.1f} KB)")


# ================================================================
#  验证
# ================================================================
def evaluate(X, y, w1, b1, w2, b2, w3, b3):
    a1 = relu(X @ w1 + b1)
    a2 = relu(a1 @ w2 + b2)
    z3 = a2 @ w3 + b3
    preds = np.argmax(z3, axis=1)
    acc = (preds == y).mean() * 100

    print(f"\n  Overall accuracy: {acc:.1f}%")
    for c in range(NUM_CLASSES):
        mask = y == c
        c_acc = (preds[mask] == c).mean() * 100 if mask.sum() > 0 else 0
        print(f"    {CLASS_NAMES[c]:15s}: {c_acc:.1f}% ({mask.sum()} samples)")


# ================================================================
#  Main
# ================================================================
def main():
    parser = argparse.ArgumentParser(description="Train TinyML grasp classifier")
    parser.add_argument(
        "--data", type=str, default=None, help="Path to real FSR CSV data (optional)"
    )
    parser.add_argument(
        "--out", type=str, default=DEFAULT_OUTPUT, help="Output C header path"
    )
    parser.add_argument("--epochs", type=int, default=EPOCHS)
    parser.add_argument("--lr", type=float, default=LEARNING_RATE)
    args = parser.parse_args()

    print("=" * 50)
    print("  TinyML Grasp Classifier Training")
    print("=" * 50)

    # 数据
    if args.data and os.path.exists(args.data):
        print(f"\n  Loading real data from: {args.data}")
        X, y = load_csv_data(args.data)
    else:
        print("\n  Generating synthetic FSR data...")
        X, y = generate_synthetic_data()

    print(f"  Dataset: {len(y)} samples, {NUM_CLASSES} classes")
    for c in range(NUM_CLASSES):
        print(f"    {CLASS_NAMES[c]:15s}: {(y == c).sum()} samples")

    # 训练
    print(f"\n  Training MLP ({WINDOW_SIZE}->{H1_SIZE}->{H2_SIZE}->{NUM_CLASSES})...")
    w1, b1, w2, b2, w3, b3 = train(X, y, epochs=args.epochs, lr=args.lr)

    # 验证
    evaluate(X, y, w1, b1, w2, b2, w3, b3)

    # 导出
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    export_c_header(w1, b1, w2, b2, w3, b3, args.out)

    print("\n  Done!")


if __name__ == "__main__":
    main()
