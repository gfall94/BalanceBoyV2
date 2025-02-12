import time
import board
import threading
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


class IMU:
    def __init__(self, debug=False):
        self.debug = debug
        self.running = False
        self.thread = None

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            self.bno = BNO08X_I2C(self.i2c)

            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
            print("IMU erfolgreich initialisiert.")
        except Exception as e:
            print("IMU-Initialisierung Fehler:", e)
            self.bno = None

    def _imu_loop(self):
        last_time = time.perf_counter()
        while self.running:
            now = time.perf_counter()
            if now - last_time < 1 / 200:
                continue
            last_time = now

            try:
                if self.debug:
                    print("Frequency:", 1 / (now - last_time))

                print("Acceleration:")
                accel_x, accel_y, accel_z = self.bno.acceleration
                print(f"X: {accel_x:.6f}  Y: {accel_y:.6f} Z: {accel_z:.6f}  m/s²")

                print("Gyro:")
                gyro_x, gyro_y, gyro_z = self.bno.gyro
                print(f"X: {gyro_x:.6f}  Y: {gyro_y:.6f} Z: {gyro_z:.6f} rads/s")

                print("Magnetometer:")
                mag_x, mag_y, mag_z = self.bno.magnetic
                print(f"X: {mag_x:.6f}  Y: {mag_y:.6f} Z: {mag_z:.6f} µT")

                print("Rotation Vector Quaternion:")
                quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
                print(f"I: {quat_i:.6f}  J: {quat_j:.6f} K: {quat_k:.6f}  Real: {quat_real:.6f}")

                print("Geomagnetic Quaternion:")
                quat_i, quat_j, quat_k, quat_real = self.bno.geomagnetic_quaternion
                print(f"I: {quat_i:.6f}  J: {quat_j:.6f} K: {quat_k:.6f}  Real: {quat_real:.6f}")

                print("Calibration Status:", self.bno.calibration_status)
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
