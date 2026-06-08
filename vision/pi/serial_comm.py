"""
串口通信模块 — Pi ↔ MCU (RA6M5)
部署到 Pi: ~/vs_code/strawberry_grasp/serial_comm.py
"""

import time
import logging

import serial

import config

logger = logging.getLogger(__name__)


class SerialComm:
    """与 MCU 的串口通信"""

    def __init__(self, port=None, baudrate=None, timeout=None):
        self.port = port or config.SERIAL_PORT
        self.baudrate = baudrate or config.BAUD_RATE
        self.timeout = timeout or config.SERIAL_TIMEOUT
        self.ser = None

    def open(self):
        """打开串口连接"""
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
        )
        logger.info("串口已打开: %s @ %d", self.port, self.baudrate)

    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")

    def ping(self) -> bool:
        """发送 P 测试连通性，返回是否收到回复"""
        if not self.ser or not self.ser.is_open:
            return False

        old_timeout = self.ser.timeout
        self.ser.timeout = 0.1

        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"P")
            self.ser.flush()

            end_time = time.monotonic() + 2.0
            response = b""
            while time.monotonic() < end_time:
                chunk = self.ser.read(1)
                if chunk:
                    response += chunk
                    time.sleep(0.05)
                    if self.ser.in_waiting:
                        response += self.ser.read(self.ser.in_waiting)
                    break

            if response:
                text = response.decode("utf-8", errors="replace").strip()
                logger.info("Ping 成功，MCU 回复: %s", text)
                return True
            else:
                logger.warning("Ping 超时，MCU 无回复")
                return False
        finally:
            self.ser.timeout = old_timeout

    def send_classification(self, cmd: str) -> str | None:
        """
        发送分类指令 (A/B/C)，返回 MCU 回复文本
        """
        if not self.ser or not self.ser.is_open:
            logger.error("串口未打开")
            return None

        self.ser.write(cmd.encode("ascii"))
        self.ser.flush()
        logger.info("TX → MCU: %s", cmd)

        response = self.ser.readline()
        if response:
            text = response.decode("utf-8", errors="replace").strip()
            logger.info("RX ← MCU: %s", text)
            return text
        else:
            logger.warning("MCU 无回复 (超时)")
            return None

    def drain_buffer(self):
        """清空接收缓冲区中的残留数据"""
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            discarded = self.ser.read(self.ser.in_waiting)
            logger.debug("丢弃缓冲区数据: %d 字节", len(discarded))
