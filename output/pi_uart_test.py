import time
import serial

SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200


def ping_test(ser):
    print("INFO Running startup ping test: sending P")
    old_timeout = ser.timeout
    ser.timeout = 0.1
    response = b""

    try:
        ser.reset_input_buffer()
        ser.write(b"P")
        ser.flush()

        end_time = time.monotonic() + 2.0
        while time.monotonic() < end_time:
            chunk = ser.read(1)
            if chunk:
                response += chunk
                time.sleep(0.05)
                if ser.in_waiting:
                    response += ser.read(ser.in_waiting)
                break
    finally:
        ser.timeout = old_timeout

    if response:
        text = response.decode("utf-8", errors="replace").strip()
        print(f"RX response: {text if text else response!r}")
    else:
        print("RX No response (timeout)")


def read_buffer(ser):
    waiting = ser.in_waiting
    if waiting > 0:
        data = ser.read(waiting)
        text = data.decode("utf-8", errors="replace").strip()
        print(f"RX buffered: {text if text else data!r}")
    else:
        print("RX buffered: <empty>")


def send_command(ser, cmd):
    ser.write(cmd.encode("ascii"))
    ser.flush()
    print(f"TX Sent: {cmd}")

    response = ser.readline()
    if response:
        text = response.decode("utf-8", errors="replace").strip()
        print(f"RX response: {text if text else response!r}")
    else:
        print("RX No response (timeout)")


def main():
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
        print(f"INFO Serial port opened: {SERIAL_PORT} @ {BAUD_RATE}")

        ping_test(ser)

        while True:
            print("\nMenu:")
            print("1 = Send A")
            print("2 = Send B")
            print("3 = Send C")
            print("r = Read buffer")
            print("q = Quit")

            choice = input("Select: ").strip().lower()

            if choice == "1":
                send_command(ser, "A")
            elif choice == "2":
                send_command(ser, "B")
            elif choice == "3":
                send_command(ser, "C")
            elif choice == "r":
                read_buffer(ser)
            elif choice == "q":
                break
            else:
                print("Invalid selection.")
    except KeyboardInterrupt:
        print("\nINFO KeyboardInterrupt received, exiting.")
    except serial.SerialException as e:
        print(f"ERROR Serial exception: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("INFO Serial port closed.")


if __name__ == "__main__":
    main()
