"""
ADC 压力传感器测试工具 — 在树莓派上运行，通过串口与 MCU test0 工程交互

用法:
  python adc_test.py

命令:
  r = 读一次 ADC
  c = 开启连续模式 (500ms)
  s = 停止连续模式
  t = 开关 P415 继电器
  p = Ping 测试
  q = 退出
"""

import serial
import threading
import sys

SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200


def reader_thread(ser):
    """后台线程：持续读取并打印 MCU 返回的数据"""
    while True:
        try:
            data = ser.readline()
            if data:
                print(data.decode("utf-8", errors="replace").rstrip())
        except serial.SerialException:
            break
        except Exception:
            break


def main():
    print(f"连接 MCU: {SERIAL_PORT} @ {BAUD_RATE}")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("已连接。输入命令 (r/c/s/t/p)，q 退出\n")

    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    try:
        while True:
            cmd = input().strip()
            if not cmd:
                continue
            if cmd.lower() == "q":
                print("退出")
                break
            ser.write(cmd[0].encode("utf-8"))
    except KeyboardInterrupt:
        print("\nCtrl+C, 退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
