"""
摄像头颜色测试 — 测试不同转换方式
"""
from picamera2 import Picamera2
import cv2
import time

cam = Picamera2()
cfg = cam.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
cam.configure(cfg)
cam.start()
time.sleep(1)

frame = cam.capture_array()

# 原始保存
cv2.imwrite("test_1_original.jpg", frame)
print("test_1_original.jpg — 不做任何转换")

# 手动交换 R 和 B 通道
swapped = frame[:, :, ::-1].copy()
cv2.imwrite("test_2_swapped.jpg", swapped)
print("test_2_swapped.jpg — 交换R和B通道")

cam.stop()
cam.close()

print("看哪张颜色正确，告诉我")
