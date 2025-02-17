import time
import board
import threading
import busio
import numpy as np
from adafruit_bno08x import (
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
    BNO_REPORT_ROTATION_VECTOR,
    )
from adafruit_bno08x.i2c import BNO08X_I2C
import requests
import json
from scipy.spatial.transform import Rotation as R
import math

def quaternion_to_euler(i, j, k, r):
    """
    Convert a quaternion into euler angles [roll, pitch, yaw]
    - roll is rotation around x in radians (CCW)
    - pitch is rotation around y in radians (CCW)
    - yaw is rotation around z in radians (CCW)
    """
    quat = np.array([r, i, j, k])  # scipy erwartet die Reihenfolge [w, x, y, z]
    euler = R.from_quat(quat, scalar_first=True).as_euler('xyz', degrees=False)  # Konvertierung nach Roll, Pitch, Yaw
    return euler[0], euler[1], euler[2]


class IMU:
    def __init__(self, debug=False, freq=50):
        self.debug = debug
        self.freq = freq
        self.running = False
        self.thread = None
        self.webhook_url = "http://127.0.0.1:5000/imu/pv"

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            self.bno = BNO08X_I2C(self.i2c)
            #self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            #self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            #self.bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            print("IMU erfolgreich initialisiert.")
        except Exception as e:
            print("IMU-Initialisierung Fehler:", e)
            self.bno = None

    def _imu_loop(self):
        last_time = 0.0
        while self.running:
            now = time.perf_counter()
            if (now - last_time) < (1 / self.freq):
                continue
            frequency = (1 / (now - last_time))
            last_time = now

            try:
                gyro_x, gyro_y, gyro_z = self.bno.gyro
                #quat_i, quat_j, quat_k, quat_real = self.bno.geomagnetic_quaternion
                quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
                roll, pitch, yaw = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)

                data = {
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                    'gyro_x': gyro_x,
                    'gyro_y': gyro_y,
                    'gyro_z': gyro_z,
                    'time': now,
                    'frequency': frequency
                }

                r = requests.post(self.webhook_url, data=json.dumps(data), headers={'Content-Type': 'application/json'})

                if self.debug:
                    print("Webhook Response: ", r)
                    print("Frequency:", frequency)
                    print("Calibration Status:", self.bno.calibration_status)

                    print("Gyro:")
                    print(f"X: {gyro_x:.6f}  Y: {gyro_y:.6f} Z: {gyro_z:.6f} rads/s")

                    print("Geomagnetic Quaternion:")
                    print(f"I: {quat_i:.6f}  J: {quat_j:.6f} K: {quat_k:.6f}  Real: {quat_real:.6f}")

                    print("Euler Angles:")
                    print(f"Roll: {roll:.6f}  Pitch: {pitch:.6f} Yaw: {yaw:.6f}")

                    print("")
            except Exception as e:
                print("IMU-Fehler:", e)

    def start(self):
        """Startet das IMU-Tracking in einem separaten Thread."""
        if self.bno and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._imu_loop, daemon=True)
            self.thread.start()
            print("IMU-Thread gestartet.")

    def stop(self):
        """Stoppt das IMU-Tracking."""
        self.running = False
        if self.thread:
            self.thread.join()
            print("IMU-Thread gestoppt.")
