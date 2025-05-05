import logging
import time
import board
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
from scipy.spatial.transform import Rotation as R

class IMU:
    def __init__(self, name = "IMU", logging_level = logging.INFO):
        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.data = {
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "gyro_x": self.gyro_x,
            "gyro_y": self.gyro_y,
            "gyro_z": self.gyro_z,
            "time": self.now,
            "frequency": self.frequency
        }

        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging_level)

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
            self.bno = BNO08X_I2C(self.i2c)
            #self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            #self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            #self.bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.logger.info("Erfolgreich initialisiert.")
        except Exception as e:
            self.logger.critical(str(e))
            self.bno = None

    def quaternion_to_euler(self, i, j, k, r):
        """
        Convert a quaternion into euler angles [roll, pitch, yaw]
        - roll is rotation around x in radians (CCW)
        - pitch is rotation around y in radians (CCW)
        - yaw is rotation around z in radians (CCW)
        """
        quat = np.array([r, i, j, k])  # scipy erwartet die Reihenfolge [w, x, y, z]
        euler = R.from_quat(quat, scalar_first=True).as_euler("xyz", degrees=False)  # Konvertierung nach Roll, Pitch, Yaw
        return euler[0], euler[1], euler[2]

    def loop(self):
            self.now = time.perf_counter()
            self.frequency = (1 / (self.now - self.last_time))
            self.last_time = self.now

            try:
                # tmp = time.perf_counter()
                self.gyro_x, self.gyro_y, self.gyro_z = self.bno.gyro
                # print(f"\n\nGyro: {time.perf_counter()-tmp}")

                # quat_i, quat_j, quat_k, quat_real = self.bno.geomagnetic_quaternion

                # tmp = time.perf_counter()
                quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
                # print(f"\n\nQuat: {time.perf_counter()-tmp}")

                # tmp = time.perf_counter()
                pitch, self.roll, self.yaw = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
                # print(f"Euler: {time.perf_counter()-tmp}")

                self.pitch = pitch

                self.data = {
                    "roll": self.roll,
                    "pitch": -self.pitch+0.022725,
                    "yaw": self.yaw,
                    "gyro_x": -self.gyro_x,
                    "gyro_y": self.gyro_y,
                    "gyro_z": self.gyro_z,
                    "time": self.now,
                    "frequency": self.frequency
                }

                self.logger.debug(self.data)
            except Exception as e:
                self.logger.error(str(e))

            return self.data

