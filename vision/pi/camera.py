"""
摄像头采图模块 — 支持 CSI (Picamera2) 和 USB (OpenCV) 两种后端
部署到 Pi: ~/vs_code/strawberry_grasp/camera.py
"""

import time
import numpy as np

import config


class CameraCSI:
    """树莓派 CSI 摄像头，通过 Picamera2 + libcamera 驱动"""

    def __init__(self, width=None, height=None):
        from picamera2 import Picamera2

        self.width = width or config.CAMERA_WIDTH
        self.height = height or config.CAMERA_HEIGHT

        self.cam = Picamera2()
        cam_config = self.cam.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "BGR888"}
        )
        self.cam.configure(cam_config)
        self.cam.start()

        # 等待自动曝光稳定
        time.sleep(1.0)
        print(f"[Camera] CSI 摄像头已启动: {self.width}x{self.height}")

    def read_frame(self) -> np.ndarray:
        """返回 BGR 格式的 numpy 数组（与 OpenCV 一致）"""
        frame = self.cam.capture_array()
        return frame[:, :, ::-1].copy()

    def close(self):
        self.cam.stop()
        print("[Camera] CSI 摄像头已关闭")


class CameraUSB:
    """USB 摄像头，通过 OpenCV VideoCapture 驱动"""

    def __init__(self, device_id=0, width=None, height=None):
        import cv2

        self.width = width or config.CAMERA_WIDTH
        self.height = height or config.CAMERA_HEIGHT

        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开 USB 摄像头 (device={device_id})")

        time.sleep(0.5)
        print(f"[Camera] USB 摄像头已启动: {self.width}x{self.height}")

    def read_frame(self) -> np.ndarray:
        """返回 BGR 格式的 numpy 数组"""
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("USB 摄像头读取帧失败")
        return frame

    def close(self):
        self.cap.release()
        print("[Camera] USB 摄像头已关闭")


def create_camera(backend="csi", **kwargs):
    """工厂函数，根据 backend 创建对应的摄像头实例"""
    if backend == "csi":
        return CameraCSI(**kwargs)
    elif backend == "usb":
        return CameraUSB(**kwargs)
    else:
        raise ValueError(f"未知的摄像头后端: {backend}，支持 'csi' 或 'usb'")
