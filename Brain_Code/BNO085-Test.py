import time
import board
import threading
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

def imu_thread():
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        bno = BNO08X_I2C(i2c)

        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    except Exception as e:
        print("IMU-Initialisierung Fehler: ", e)

    last_time = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now - last_time < 1 / 200:
            continue
        last_time = now
        try:
            time.sleep(0.5)
            print("Acceleration:")
            # noinspection PyUnboundLocalVariable
            accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
            print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
            print("")

            print("Gyro:")
            gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
            print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
            print("")

            print("Magnetometer:")
            mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
            print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
            print("")

            print("Rotation Vector Quaternion:")
            quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
            print(
                "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
            )

            print("Geomagnetic Quaternion:")
            quat_i, quat_j, quat_k, quat_real = bno.geomagnetic_quaternion
            print(
                "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
            )

            print("Calibration Status:")
            calibration_status = bno.calibration_status
            print(
                "Status: %d" % calibration_status
            )
            print("")
        except Exception as e:
            print("IMU-Fehler: ",e)

# Threads starten
for func in [imu_thread]:
    threading.Thread(target=func, daemon=True).start()

while True:
    time.sleep(1)