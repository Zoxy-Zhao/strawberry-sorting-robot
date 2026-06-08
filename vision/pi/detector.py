"""
YOLO 推理模块 — 草莓成熟度检测
部署到 Pi: ~/vs_code/strawberry_grasp/detector.py
"""

from dataclasses import dataclass

import numpy as np
from ultralytics import YOLO

import config


@dataclass
class Detection:
    """单个检测结果"""
    class_id: int
    class_name: str       # ripe / semi_ripe / unripe
    class_cn: str         # 成熟 / 半成熟 / 未熟
    mcu_cmd: str          # A / B / C
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

    @property
    def area(self) -> float:
        return (self.x2 - self.x1) * (self.y2 - self.y1)


class StrawberryDetector:
    """YOLOv8n 草莓检测器"""

    def __init__(self, model_path=None, conf=None, iou=None):
        model_path = model_path or config.MODEL_PATH
        self.conf = conf or config.CONFIDENCE_THRESHOLD
        self.iou = iou or config.IOU_THRESHOLD

        print(f"[Detector] 正在加载模型: {model_path}")
        self.model = YOLO(model_path)
        print(f"[Detector] 模型加载完成，置信度阈值={self.conf}")

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """
        输入 BGR 帧，返回检测结果列表（按置信度降序排列）
        """
        results = self.model.predict(
            frame,
            conf=self.conf,
            iou=self.iou,
            verbose=False,
        )

        detections = []
        if len(results) == 0 or results[0].boxes is None:
            return detections

        boxes = results[0].boxes
        img_h, img_w = frame.shape[:2]

        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i].item())
            conf = float(boxes.conf[i].item())
            x1, y1, x2, y2 = boxes.xyxy[i].tolist()

            # 查找类别映射
            if cls_id not in config.CLASS_MAP:
                continue
            name, cn, cmd = config.CLASS_MAP[cls_id]

            # 过滤太小的目标
            bbox_area = ((x2 - x1) * (y2 - y1)) / (img_w * img_h)
            if bbox_area < config.MIN_BBOX_AREA:
                continue

            # 检测区域过滤
            if config.DETECT_REGION is not None:
                rx1, ry1, rx2, ry2 = config.DETECT_REGION
                cx = (x1 + x2) / 2 / img_w
                cy = (y1 + y2) / 2 / img_h
                if not (rx1 <= cx <= rx2 and ry1 <= cy <= ry2):
                    continue

            detections.append(Detection(
                class_id=cls_id,
                class_name=name,
                class_cn=cn,
                mcu_cmd=cmd,
                confidence=conf,
                x1=x1, y1=y1, x2=x2, y2=y2,
            ))

        # 按置信度降序排列
        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections

    def detect_best(self, frame: np.ndarray) -> Detection | None:
        """返回置信度最高的检测结果，无目标返回 None"""
        dets = self.detect(frame)
        return dets[0] if dets else None
