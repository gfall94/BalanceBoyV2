import socket
import time
from imu import IMU

UDP_IP = "192.168.1.55"  # IP deines PCs (ändern!)
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
imu = IMU(debug=False)
imu.start()

try:
    while True:
        quat_i, quat_j, quat_k, quat_real = imu.bno.geomagnetic_quaternion
        message = f"{quat_i},{quat_j},{quat_k},{quat_real}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.05)  # 20 Hz Sendeintervall
except KeyboardInterrupt:
    print("Beenden...")
    imu.stop()
    sock.close()
