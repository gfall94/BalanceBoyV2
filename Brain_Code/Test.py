import traceback
import copy
import json
import math
import time
from LowPass import LowPassFilter
from pid import PID
from lqr import LQR
import zmq

# Globals
freq_sp = 50.0
data = {
    "main": {
        "upright": False,
        "upright_tol": 3.0,
        "activation_delay": 5.0,
        "en": False,
        "en_tol": 30.0,
        "time": 0.0,
        "frequency": 0.0
    },
    "physics": {
        "r": 57.75 / 1000,  # wheel radius in m
        "R": 138.441 / 1000,  # wheel to cog in m
        "g": 9.81,  # gravitation im m/s^2
        "m": 885.54481 / 1000,  # mass of robot in kg
        "J": 0.00545106520548,  # Inertia of chassis kg*m^2
        "tau_m": 1,  # time constant of motors
        "K_m": 1  # motor Gain
    },
    "motor_left": {
        "sp": 0.0,
        "en": False,
        "position": 0.0,
        "velocity": 0.0,
        "velocity_LP": 0.0,
        "time": 0.0,
        "frequency": 0.0
    },
    "motor_right": {
        "sp": 0.0,
        "en": False,
        "position": 0.0,
        "velocity": 0.0,
        "velocity_LP": 0.0,
        "time": 0.0,
        "frequency": 0.0
    },
    "imu": {
        'roll': 0.0,
        'pitch': 0.0,
        'pitch_LP': 0.0,
        'yaw': 0.0,
        'gyro_x': 0.0,
        'gyro_x_LP': 0.0,
        'gyro_y': 0.0,
        'gyro_z': 0.0,
        'time': 0.0,
        'frequency': 0.0
    },
    "sp": {
        "p": 0.0,
        "x": 0.0,
        "pv": 0.0,
        "v": 0.0,
        "yaw": 0.0
    },
    "lqr": {
        "config": {
            "Q": [100, 5, 50, 25],
            "R": 1.0
        },
        "out": 0.0,
        "en": False,
        "time": 0.0,
        "frequency": 0.0
    },
    "pitch_pid": {
        "config": {
            "Kp": 80.0,
            "Ki": 5.0,
            "Kd": 2.0
        },
        "out": 0.0,
        "en": False,
        "time": 0.0,
        "frequency": 0.0
    },
    "velocity_pid": {
        "config": {
            "Kp": 0.01,
            "Ki": 0.03,
            "Kd": 0.0
        },
        "out": 0.0,
        "en": False,
        "time": 0.0,
        "frequency": 0.0
    }
}
data_last = data
# Main
now = time.perf_counter()
last_time = now
dt = 0.0
frequency = 50.0
upright_time = 0.0
# IMU
# Motor
# Filter
pitch_LP = LowPassFilter(25.0)
gyro_x_LP = LowPassFilter(25.0)
motor_left_vel_LP = LowPassFilter(5.0)
motor_right_vel_LP = LowPassFilter(5.0)
# Controller
pitch_controller = PID(config=data["pitch_pid"]["config"], mini=-100, maxi=100)
velocity_controller = PID(config=data["velocity_pid"]["config"], mini=-10.0, maxi=10.0)

lqr_controller = LQR(config=data["lqr"]["config"], physics=data["physics"], freq=frequency)
# Communication

while True:
    try:
        now = time.perf_counter()
        if (now - last_time) > (1 / freq_sp):
            dt = now - last_time
            frequency = (1 / dt)
            last_time = now

            # Program Control
            # check pitch
            if (data_last["main"]["upright_tol"] > math.degrees(data["imu"]["pitch"]) > -
            data_last["main"]["upright_tol"]) and not data_last["main"]["upright"]:
                data["main"]["upright"] = True
                upright_time = time.perf_counter()

            if data_last["main"]["en_tol"] > math.degrees(data["imu"]["pitch"]) > -data_last["main"][
                "en_tol"]:
                if (upright_time + data["main"]["activation_delay"]) < time.perf_counter():
                    data["main"]["en"] = True
            else:
                data["main"]["upright"] = False
                upright_time = time.perf_counter()
                data["main"]["en"] = False

            # enabled?
            if not data["main"]["en"]:
                data["motor_left"]["en"] = False
                data["motor_right"]["en"] = False
                data["pitch_pid"]["en"] = False
                data["velocity_pid"]["en"] = False
                data["lqr"]["en"] = False
            else:
                data["motor_left"]["en"] = True
                data["motor_right"]["en"] = True
                data["pitch_pid"]["en"] = True
                data["velocity_pid"]["en"] = False
                data["lqr"]["en"] = True

            # reset
            if not (data["main"]["en"] == data_last["main"]["en"]):
                pitch_controller.reset()
                velocity_controller.reset()

            # Read IMU
            # Read Motors

            # Filtering
            data["imu"]["gyro_x"] = (data["imu"]["pitch"] - data_last["imu"]["pitch"]) * dt
            data["imu"]["gyro_x_LP"] = gyro_x_LP.filter(data["imu"]["gyro_x"])
            data["imu"]["pitch_LP"] = pitch_LP.filter(data["imu"]["pitch"])
            data["motor_left"]["velocity_LP"] = motor_left_vel_LP.filter(data["motor_left"]["velocity"])
            data["motor_right"]["velocity_LP"] = motor_right_vel_LP.filter(data["motor_right"]["velocity"])

            # Controller
            # data["velocity_pid"] = velocity_controller.loop(sp=0.0,x=(data["motor_left"]["velocity_LP"]+data["motor_right"]["velocity_LP"])/2,data=data["velocity_pid"])
            # data["pitch_pid"] = pitch_controller.loop(sp=data["velocity_pid"]["out"],x=data["imu"]["pitch_LP"],data=data["pitch_pid"])
            data["lqr"] = lqr_controller.loop(sp=data["sp"],
                                              x1=data["imu"]["pitch_LP"],
                                              x2=(data["motor_left"]["position"] + data["motor_right"]["position"]) / 2,
                                              x3=data["imu"]["gyro_x_LP"],
                                              x4=(data["motor_left"]["velocity"] + data["motor_right"]["velocity"]) / 2,
                                              data=data["lqr"])

            # Set Motors
            data["motor_left"]["sp"] = data["lqr"]["out"]
            data["motor_right"]["sp"] = data["lqr"]["out"]
            print(data["lqr"]["out"])


            # Misc
            data["main"]["frequency"] = frequency
            data["main"]["time"] = now
            data_last = copy.deepcopy(data)


            # Debugging
            # print( f"{frequency:.3f}\t")
            # print(json.dumps(data, indent=4))
            # print(str(controller.integral))
        else:
            time.sleep(1 / 1000)
    except Exception as e:
        print(str(e))
        traceback.format_exc()