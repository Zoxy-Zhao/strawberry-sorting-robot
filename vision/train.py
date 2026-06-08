"""
草莓成熟度检测 — YOLOv8n 训练脚本
在 Windows PC (有 GPU) 上运行
"""

from ultralytics import YOLO


def main():
    # 加载 YOLOv8n 预训练权重（COCO）
    model = YOLO("yolov8n.pt")

    # 训练配置
    results = model.train(
        data="D:/VS_code/projects/strawberry_grasp/vision/data.yaml",
        epochs=100,
        imgsz=640,
        batch=16,
        device=0,  # GPU

        # 项目输出
        project="runs",
        name="strawberry_v1",

        # 数据增强（针对草莓场景优化）
        hsv_h=0.02,       # 色相扰动（草莓颜色敏感，不要太大）
        hsv_s=0.7,        # 饱和度扰动
        hsv_v=0.4,        # 亮度扰动
        degrees=15.0,     # 旋转
        translate=0.1,    # 平移
        scale=0.5,        # 缩放
        fliplr=0.5,       # 水平翻转
        flipud=0.0,       # 不做垂直翻转（草莓有方向性）
        mosaic=1.0,       # Mosaic 增强
        mixup=0.1,        # MixUp 增强

        # 优化器
        optimizer="AdamW",
        lr0=0.001,
        lrf=0.01,
        weight_decay=0.0005,

        # 早停
        patience=20,

        # 工作线程
        workers=4,

        # 保存
        save=True,
        save_period=10,   # 每 10 epoch 保存一次检查点
        plots=True,       # 生成训练曲线图
    )

    print("\n训练完成！")
    print(f"最佳权重: runs/strawberry_v1/weights/best.pt")
    print(f"训练曲线: runs/strawberry_v1/results.png")

    # 在测试集上评估
    print("\n正在测试集上评估...")
    model = YOLO("runs/strawberry_v1/weights/best.pt")
    metrics = model.val(data="data.yaml", split="test")
    print(f"\n测试集结果:")
    print(f"  mAP@0.5:     {metrics.box.map50:.4f}")
    print(f"  mAP@0.5:0.95: {metrics.box.map:.4f}")
    for i, name in enumerate(["ripe", "semi_ripe", "unripe"]):
        print(f"  {name:12s}  AP@0.5={metrics.box.ap50[i]:.4f}")


if __name__ == "__main__":
    main()
