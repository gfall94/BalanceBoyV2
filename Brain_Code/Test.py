import serial
import threading
import time

# Globale Variable für die letzten empfangenen Werte
latest_values = (0.0, 0.0)
last = time.time()
lock = threading.Lock()


def serial_reader():
    global latest_values
    ser = serial.Serial("/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0668FF485671664867185737-if02", baudrate=115200, timeout=0.1)
    global last

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    with lock:
                        latest_values = (x, y)
                        print(f"x = {x}, y = {y}, dt = {time.time() - last}")
                        last = time.time()
        except (ValueError, UnicodeDecodeError):
            continue  # Fehlerhafte Zeile ignorieren


# Starte den Serial-Thread
thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

# Hauptschleife mit 50 Hz
while True:
    with lock:
        x, y = latest_values
    # print(f"x = {x}, y = {y}, dt = {time.time()-last}")
    time.sleep(1 / 50)
