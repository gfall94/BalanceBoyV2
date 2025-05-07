import traceback
import copy
import json
import math
import time
import threading

import numpy as np

from imu import IMU
from motor import MOTOR
from LowPass import LowPassFilter
from pid import PID
from lqr import LQR
from kalman import KalmanFilter
from ps4_controller import PS4Controller
from eyes import EYES
import zmq


current_values = [0.0] * 8

def main_loop():
    # Globals
    global current_values
    freq_sp = 50.0
    data = {
        "main": {
            "mode": 0,  # 0=off, 1=armed, 2=tolerance, 3=activation_delay 4=on
            "upright": False,
            "upright_tol": math.radians(3.0),
            "activation_delay": 5.0,
            "en": False,
            "tol": math.radians(30.0),
            "in_tol": False,
            "yaw": 0.0,
            "time": 0.0,
            "frequency": 0.0
        },
        "physics": {
            "r": 57.75 / 1000,  # wheel radius in m
            "r_y": 272.59000 / 1000,  # wheel distance (center to center)
            "R": 138.441 / 1000,  # wheel to cog in m
            "g": 9.81,  # gravitation im m/s^2
            "m": 885.54481 / 1000,  # mass of robot in kg
            "J": 25612452.77099 / 1000000000,  # 0.00545106520548, # Inertia of chassis kg*m^2
            "tau_m": (0.103 + 0.108) / 2,  # time constant of motors
            "K_m": (0.231 + 0.235) / 2,  # motor Gain
            "v_max": 1.0,  # m/s
            "y_max": math.pi / 4  # rad/s
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
        "sp_LP": {
            "p": 0.0,
            "x": 0.0,
            "pv": 0.0,
            "v": 0.0,
            "yaw": 0.0
        },
        "lqr": {
            "config": {
                "Q": [100, 15, 50, 25],
                "R": 1.0
            },
            "out": 0.0,
            "en": False,
            "time": 0.0,
            "frequency": 0.0
        },
        "kalman": {
            "config": {
                "Q": [50, 50, 25, 25],
                "R": [0.05, 0.05, 0.5, 0.5]
            },
            "out": {
                "p": 0.0,
                "x": 0.0,
                "pv": 0.0,
                "v": 0.0
            },
            "time": 0.0,
            "frequency": 0.0
        },
        "yaw_pid": {
            "config": {
                "Kp": 15,
                "Ki": 1,
                "Kd": 0.0
            },
            "out": 0.0,
            "en": False,
            "time": 0.0,
            "frequency": 0.0
        },
        "ps4": {
            "time": 0,
            "frequency": 0,
            "connected": False,
            "x": False,
            "v": False,
            "d": False,
            "o": False,
            "l1": False,
            "r1": False,
            "share": False,
            "option": False,
            "l3": False,
            "r3": False,
            "ps": False,
            "dpad_x": 0,
            "dpad_y": 0,
            "left_x": 0,
            "left_y": 0,
            "right_x": 0,
            "right_y": 0,
            "l2": 0,
            "r2": 0
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
    imu = IMU()
    # Motor
    motor_left = MOTOR(name="Motor_Left",
                       port="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0668FF485671664867185737-if02",
                       invert=True, min_max=7.5)
    motor_right = MOTOR(name="Motor_Right",
                        port="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF485671664867185641-if02",
                        invert=True, min_max=7.5)
    # Filter
    pitch_LP = LowPassFilter(25.0)
    gyro_x_LP = LowPassFilter(25.0)
    motor_left_vel_LP = LowPassFilter(5.0)
    motor_right_vel_LP = LowPassFilter(5.0)
    p_sp_LP = LowPassFilter(0.5)
    x_sp_LP = LowPassFilter(0.5)
    pv_sp_LP = LowPassFilter(0.5)
    v_sp_LP = LowPassFilter(0.5)
    yaw_sp_LP = LowPassFilter(0.5)

    kalman = KalmanFilter(config=data["kalman"]["config"], physics=data["physics"], freq=freq_sp)
    # Controller
    lqr_controller = LQR(config=data["lqr"]["config"], physics=data["physics"], freq=freq_sp)
    yaw_controller = PID(config=data["yaw_pid"]["config"], mini=-50, maxi=50)

    # Communication
    # context = zmq.Context()
    # socket = context.socket(zmq.REP)
    # port = "5556"
    # socket.bind("tcp://*:%s" % port)

    # PS4Controller
    ps4 = PS4Controller()
    ps4.start()
    ps4_connection_last = now
    ps4_connection_timeout = 5

    # Eyes
    eyes = EYES()
    try:
        while True:
            try:
                now = time.perf_counter()
                if (now - last_time) > (1 / freq_sp):
                    dt = now - last_time
                    frequency = (1 / dt)
                    last_time = now

                    # check pitch tolerance
                    data["main"]["in_tol"] = (data["main"]["tol"] > data["imu"]["pitch"] > -data["main"]["tol"])
                    # check upright
                    data["main"]["upright"] = (data["main"]["upright_tol"] > data["imu"]["pitch"] > -data["main"]["upright_tol"])

                    # Read PS4-Input
                    data["ps4"] = ps4.get()
                    if data["ps4"]["connected"]:
                        ps4_connection_last = now

                    # Program Control
                    match data["main"]["mode"]:
                        case 0: # off
                            if data["ps4"]["x"] and not data_last["ps4"]["x"]:
                                data["main"]["mode"] = 1
                            if ps4_connection_last + ps4_connection_timeout < now:
                                data["main"]["mode"] = 1

                            eyes.fill_range(0,eyes.num_leds_total,"red")
                        case 1:  # armed
                            if data["ps4"]["x"] and not data_last["ps4"]["x"]:
                                data["main"]["mode"] = 0
                            if data["main"]["in_tol"]:
                                data["main"]["mode"] = 2

                            eyes.fill_range(0, eyes.num_leds_total, "red")
                            eyes.line(orientation="horizontal", module_name="left",
                                      color1_name="yellow", value1=-data["kalman"]["out"]["p"],
                                      min_value1=-data["main"]["tol"], max_value1=data["main"]["tol"])
                            eyes.line(orientation="horizontal", module_name="right",
                                      color1_name="yellow", value1=-data["kalman"]["out"]["p"],
                                      min_value1=-data["main"]["tol"], max_value1=data["main"]["tol"])
                        case 2:  # tolerance
                            if data["ps4"]["x"] and not data_last["ps4"]["x"]:
                                data["main"]["mode"] = 0
                            if not data["main"]["in_tol"]:
                                data["main"]["mode"] = 1
                            if data["main"]["upright"]:
                                data["main"]["mode"] = 3
                                upright_time = now

                            eyes.fill_range(0, eyes.num_leds_total, "yellow")
                            eyes.line(orientation="horizontal", module_name="left",
                                      color1_name="red", value1=-data["kalman"]["out"]["p"],
                                      min_value1=-data["main"]["tol"], max_value1=data["main"]["tol"])
                            eyes.line(orientation="horizontal", module_name="right",
                                      color1_name="red", value1=-data["kalman"]["out"]["p"],
                                      min_value1=-data["main"]["tol"], max_value1=data["main"]["tol"])
                        case 3:  # activation_delay
                            if data["ps4"]["x"] and not data_last["ps4"]["x"]:
                                data["main"]["mode"] = 0
                            if not data["main"]["upright"]:
                                data["main"]["mode"] = 2
                            if (upright_time + data["main"]["activation_delay"]) < now:
                                data["main"]["mode"] = 4

                            eyes.fill_range(0, eyes.num_leds_total, "green")
                            eyes.line(orientation="horizontal", module_name="left",
                                      color1_name="white", value1=-data["kalman"]["out"]["p"],
                                      min_value1=-data["main"]["tol"], max_value1=data["main"]["tol"])
                            eyes.line(orientation="horizontal", module_name="right",
                                      color1_name="white", value1=-data["kalman"]["out"]["p"],
                                      min_value1=-data["main"]["tol"], max_value1=data["main"]["tol"])
                        case 4: # on
                            if data["ps4"]["x"] and not data_last["ps4"]["x"]:
                                data["main"]["mode"] = 0
                            if not data["main"]["in_tol"]:
                                data["main"]["mode"] = 0

                            eyes.clear_range(0, eyes.num_leds_total)
                            eyes.line(orientation="cross", module_name="left",
                                      color1_name="blue", value1=data["ps4"]["left_x"],
                                      min_value1=0, max_value1=255,
                                      color2_name="red", value2=-(data["ps4"]["r2"]-data["ps4"]["l2"]),
                                      min_value2=-255,
                                      max_value2=255,
                                      comb_color_name="green")
                            eyes.line(orientation="cross", module_name="right",
                                      color1_name="blue", value1=data["yaw_pid"]["out"],
                                      min_value1=-50, max_value1=+50,
                                      color2_name="red", value2=-data["lqr"]["out"],
                                      min_value2=-100/2, max_value2=100/2,
                                      comb_color_name="green")

                    # enabled?
                    data["main"]["en"] = data["main"]["mode"]==4
                    if not data["main"]["en"]:
                        data["motor_left"]["en"] = False
                        data["motor_right"]["en"] = False
                        data["yaw_pid"]["en"] = False
                        data["lqr"]["en"] = False
                    else:
                        data["motor_left"]["en"] = True
                        data["motor_right"]["en"] = True
                        data["yaw_pid"]["en"] = True
                        data["lqr"]["en"] = True

                    # reset
                    if not (data["main"]["mode"] == data_last["main"]["mode"]):
                        imu.reset()
                        motor_left.reset()
                        motor_right.reset()
                        yaw_controller.reset()
                        data["sp"] = {
                                "p":0.0,
                                "x":0.0,
                                "pv": 0.0,
                                "v": 0.0,
                                "yaw": 0.0
                                }
                        data["sp_LP"] = {
                                "p":0.0,
                                "x":0.0,
                                "pv": 0.0,
                                "v": 0.0,
                                "yaw": 0.0
                                }

                    # Read IMU
                    data["imu"] = imu.loop()
                    # Read Motors
                    data["motor_left"] = motor_left.get()
                    data["motor_right"] = motor_right.get()
                    data["main"]["yaw"] = data["physics"]["r"] * (data["motor_right"]["position"] - data["motor_left"]["position"]) / data["physics"]["r_y"]

                    # Process inputs
                    if data["main"]["mode"]==4:
                        d = 0
                        r = 0
                        if not data["ps4"]["dpad_y"] == data_last["ps4"]["dpad_y"]:
                            d = d + 0.5 * data["ps4"]["dpad_y"]# distance in meters per click
                        if not data["ps4"]["dpad_x"] == data_last["ps4"]["dpad_x"]:
                            r = r + 1/8 * data["ps4"]["dpad_x"] # angle per click

                        v_sp = np.interp((data["ps4"]["r2"]-data["ps4"]["l2"]),[-255, 255],[-data["physics"]["v_max"], data["physics"]["v_max"]] )
                        y_sp = np.interp(data["ps4"]["left_x"],[0, 255],[-data["physics"]["y_max"], data["physics"]["y_max"]] )
                        d = d + v_sp*dt  # distance in meters per click
                        r = r + y_sp*dt
                        data["sp"]["x"] = data["sp"]["x"] + d / data["physics"]["r"]
                        data["sp"]["yaw"] = data["sp"]["yaw"] + r * 2 * math.pi




                    # Filtering
                    data["imu"]["gyro_x_LP"] = gyro_x_LP.filter(data["imu"]["gyro_x"])
                    data["imu"]["pitch_LP"] = pitch_LP.filter(data["imu"]["pitch"])
                    data["motor_left"]["velocity_LP"] = motor_left_vel_LP.filter(data["motor_left"]["velocity"])
                    data["motor_right"]["velocity_LP"] = motor_right_vel_LP.filter(data["motor_right"]["velocity"])

                    data["sp_LP"]["p"] = p_sp_LP.filter(data["sp"]["p"])
                    data["sp_LP"]["x"] = x_sp_LP.filter(data["sp"]["x"])
                    data["sp_LP"]["pv"] = pv_sp_LP.filter(data["sp"]["pv"])
                    data["sp_LP"]["v"] = v_sp_LP.filter(data["sp"]["v"])
                    data["sp_LP"]["yaw"] = yaw_sp_LP.filter(data["sp"]["yaw"])

                    data["kalman"] = kalman.loop(u = data["lqr"]["out"],
                                                x1 = data["imu"]["pitch_LP"],
                                                x2 = (data["motor_left"]["position"]+data["motor_right"]["position"])/2,
                                                x3 = data["imu"]["gyro_x_LP"],
                                                x4 = (data["motor_left"]["velocity_LP"]+data["motor_right"]["velocity_LP"])/2,
                                                data = data["kalman"])

                    # Controller
                    data["lqr"] = lqr_controller.loop(sp = data["sp_LP"],
                                                      x1 = data["kalman"]["out"]["p"],
                                                      x2 = data["kalman"]["out"]["x"],
                                                      x3 = data["kalman"]["out"]["pv"],
                                                      x4 = data["kalman"]["out"]["v"],
                                                      data=data["lqr"])
                    data["yaw_pid"] = yaw_controller.loop(sp=data["sp_LP"]["yaw"],x=data["main"]["yaw"],data=data["yaw_pid"])

                    # Set Motors
                    data["motor_left"]["sp"] = data["lqr"]["out"] - data["yaw_pid"]["out"]
                    data["motor_right"]["sp"] = data["lqr"]["out"] + data["yaw_pid"]["out"]
                    motor_left.set(data["motor_left"])
                    motor_right.set(data["motor_right"])

                    # Show Eyes
                    eyes.show()

                    # Misc
                    data["main"]["frequency"] = frequency
                    data["main"]["time"] = now
                    data_last = copy.deepcopy(data)

                    # Communication
                    # msg = socket.recv_string()
                    # if not "blabla" in msg:
                    #     if "lqr" in msg:
                    #         rcv = json.loads(msg)
                    #         print(f"got data: {rcv}")
                    #         data["yaw_pid"]["config"] = rcv["yaw"]
                    #         data["lqr"]["config"] = rcv["lqr"]
                    #         lqr_controller.calc_gains(data["lqr"]["config"])
                    #         data["kalman"]["config"] = rcv["kalman"]
                    #         kalman.calc_gains(data["kalman"]["config"])
                    #     if "sp" in msg:
                    #         rcv = json.loads(msg)
                    #         print(f"got data: {rcv}")
                    #         data["sp"] = rcv["sp"]
                    #
                    # message = json.dumps(data)
                    # socket.send_string(message)
                    current_values = [data["kalman"]["out"]["p"],
                                      data["sp_LP"]["p"],
                                      data["kalman"]["out"]["x"],
                                      data["sp_LP"]["x"],
                                      data["main"]["yaw"],
                                      data["sp_LP"]["yaw"],
                                      data["kalman"]["out"]["p"],
                                      data["kalman"]["out"]["p"]
                                        ]

                    # Debugging
                    mode = data["main"]["mode"]
                    print(f"{frequency:.3f}\t{mode}")
                    # value = data["imu"]["yaw"]
                    # value2 = data["main"]["yaw"]
                    # print( f"imu:{value:.3f}\tmot:{value2:.3f}")
                    # value = data["sp"]["yaw"]
                    # value3 = data["main"]["yaw"]
                    # value2 = data["sp_LP"]["yaw"]
                    # print( f"y_sp:{value:.3f}, y_sp_LP:{value2:.3f}, out:{value3:.3f}")
                    # value = data["sp"]["x"]
                    # value3 = data["kalman"]["out"]["x"]
                    # value2 = data["sp_LP"]["x"]
                    # print( f"x_sp:{value:.3f}, x_sp_LP:{value2:.3f}, out:{value3:.3f}")
                    # print(json.dumps(data, indent=4))
                    # print(str(controller.integral))
                else:
                    time.sleep(1/1000)
            except Exception as e:
                print(str(e))
                traceback.format_exc()
    except Exception as e:
        print(str(e))
    finally:
        data["motor_left"]["en"] = False
        data["motor_right"]["en"] = False
        motor_left.set(data["motor_left"])
        motor_right.set(data["motor_right"])
        eyes.clear()

from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from starlette.responses import RedirectResponse
import asyncio

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/")
async def read_index():
    return RedirectResponse(url="/static/client.html")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global current_values
    await websocket.accept()
    try:
        while True:
            await websocket.send_text(json.dumps(current_values))
            await asyncio.sleep(0.02)  # 50 Hz
    except Exception as e:
        print(f"Verbindung geschlossen: {e}")

def start_system():
    # Main-Thread starten
    main_thread = threading.Thread(target=main_loop, daemon=True)
    main_thread.start()

    # FastAPI Server starten
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

# Starten
if __name__ == "__main__":
    start_system()