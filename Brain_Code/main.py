import math
import time
import threading
from imu import IMU
from motor import MOTOR
from balance_PID import BalancePID
from LowPass import LowPassFilter
# from flask import Flask, request, jsonify
import json
import copy
import logging
import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
port = "5556"
socket.bind("tcp://*:%s" % port)

logging.basicConfig(format='%(name)s - %(levelname)s - %(message)s', level=logging.DEBUG)

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
    "motor_left": {
        "sp": 0.0,
        "en": False,
        "position": 0.0,
        "velocity": 0.0,
        "time": 0.0,
        "frequency": 0.0
    },
    "motor_right": {
        "sp": 0.0,
        "en": False,
        "position": 0.0,
        "velocity": 0.0,
        "time": 0.0,
        "frequency": 0.0
    },
    "imu": {
        'roll': 0.0,
        'pitch': 0.0,
        'pitch_LP': 0.0,
        'yaw': 0.0,
        'gyro_x': 0.0,
        'gyro_y': 0.0,
        'gyro_z': 0.0,
        'time': 0.0,
        'frequency': 0.0
    },
    "sp": {
        "p":0.0,
        "x":0.0,
        "pv": 0.0,
        "v": 0.0,
        "yaw": 0.0
    },
    "lqr": {
        "config": {
            "Q": [[100, 0, 0, 0],
                  [0, 5, 0, 0],
                  [0, 0, 50, 0],
                  [0, 0, 0, 25]],
            "R": 1.0,
            "K": [0.0, 0.0, 0.0, 0.0]
        },
        "out": 0.0,
        "en": False,
        "time": 0.0,
        "frequency": 0.0
    },
    "balance_pid": {
            "config": {
                # "Kp": [80.0, 10.0, 4.0, 2.0],
                # "Ki": [5.0, 0.0, 0.5, 0.1],
                # "Kd": [2.0, 0.1, 0.3, 0.2]
                "Kp": [80.0, 0.0, 0.0, 0.0],
                "Ki": [5.0, 0.0, 0.0, 0.0],
                "Kd": [2.0, 0.0, 0.0, 0.0]
            },
            "out": 0.0,
            "en": False,
            "time": 0.0,
            "frequency": 0.0
        }
}

class MAIN:
    def __init__(self, name="Main", logging_level=logging.DEBUG, freq=50, data_struct=None, imu=None, motor_left=None, motor_right=None, controller=None):
        self.freq = freq
        self.running = False
        self.thread = None
        self.frequency = 0.0
        self.data = data_struct
        self.data_last = self.data
        self.imu = imu
        self.imu_LP = LowPassFilter(cutoff_hz=5.0)
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.controller = controller
        self.upright_time = 0.0

        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging_level)
        logging.getLogger("IMU").setLevel(logging.ERROR)
        logging.getLogger("Motor_Left").setLevel(logging.ERROR)
        logging.getLogger("Motor_Right").setLevel(logging.ERROR)
        logging.getLogger("BalancePID").setLevel(logging.ERROR)


    def _main_loop(self):
        last_time = 0.0
        while self.running:
            self.now = time.perf_counter()
            if (self.now - last_time) < (1 / self.freq):
                continue
            self.frequency = (1 / (self.now - last_time))
            last_time = self.now

            try:
                # check pitch
                if (self.data_last["main"]["upright_tol"] > math.degrees(self.data["imu"]["pitch"]) > -self.data_last["main"]["upright_tol"]) and not self.data_last["main"]["upright"]:
                    self.data["main"]["upright"] = True
                    self.upright_time = time.perf_counter()

                if self.data_last["main"]["en_tol"] > math.degrees(self.data["imu"]["pitch"]) > -self.data_last["main"]["en_tol"]:
                    if (self.upright_time + self.data["main"]["activation_delay"]) < time.perf_counter():
                        self.data["main"]["en"] = True
                else:
                    self.data["main"]["upright"] = False
                    self.upright_time = time.perf_counter()
                    self.data["main"]["en"] = False

                # enabled?
                if not self.data["main"]["en"]:
                    self.data["motor_left"]["en"] = False
                    self.data["motor_right"]["en"] = False
                    self.data["balance_pid"]["en"] = False
                else:
                    self.data["motor_left"]["en"] = True
                    self.data["motor_right"]["en"] = True
                    self.data["balance_pid"]["en"] = True

                # reset
                # self.logger.error(self.data["main"]["en"] == self.data_last["main"]["en"])
                if  not (self.data["main"]["en"] == self.data_last["main"]["en"]):
                    self.motor_left.reset()
                    self.motor_right.reset()
                    self.controller.reset()

                # get measurements
                self.data["imu"] = self.imu.get()
                self.data["imu"]["pitch_LP"] = self.imu_LP.filter(self.data["imu"]["pitch"])
                self.data["motor_left"] = self.motor_left.get()
                self.data["motor_right"] = self.motor_right.get()

                # controller
                self.controller.set(self.data)
                self.data["balance_pid"] = self.controller.get()

                # set commands
                self.data["motor_left"]["sp"] = self.data["balance_pid"]["out"]
                self.data["motor_right"]["sp"] = self.data["balance_pid"]["out"]
                self.motor_left.set(self.data["motor_left"])
                self.motor_right.set(self.data["motor_right"])

                self.data["main"]["frequency"] = self.frequency
                self.data["main"]["time"] = self.now

                # self.logger.debug(json.dumps(self.data, indent=4))
                # self.logger.debug(json.dumps(self.data["motor_left"], indent=4))
                # self.logger.debug("Right: " + json.dumps(self.data["motor_right"], indent=4))
                # self.logger.debug("IMU: " + json.dumps(self.data["imu"], indent=4))
                # self.logger.debug("Balance_PID: " + json.dumps(self.data["balance_pid"], indent=4))
                # self.logger.debug(
                #     "Pitch: " + str(math.degrees(self.data["imu"]["pitch"]))
                #     + " PID SP: "+ str(self.data["balance_pid"]["out"])
                #     + " Mot SP: "+ str(self.data["motor_left"]["sp"])
                #     )

                self.data_last = copy.deepcopy(self.data)

            except Exception as e:
                self.logger.critical(str(e))

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._main_loop, daemon=True)
        self.thread.start()
        self.logger.info("Thread gestartet.")

    def stop(self):
        if self.thread:
            self.thread.join()
            self.logger.info("Thread gestoppt.")
        self.running = False

    def get(self):
        return self.data

    def set_pid(self,data):
        self.data["balance_pid"]["config"] = data


imu = IMU(logging_level=logging.ERROR, freq=100)
imu.start()

motor_left = MOTOR(name = "Motor_Left", logging_level=logging.ERROR, freq=50, port="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0668FF485671664867185737-if02", invert=True)
motor_left.start()
motor_right = MOTOR(name = "Motor_Right", logging_level=logging.ERROR, freq=50, port="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF485671664867185641-if02", invert=True)
motor_right.start()

balancePID = BalancePID(logging_level=logging.ERROR, freq=50, config=data["balance_pid"]["config"])
balancePID.start()

main_loop = MAIN(logging_level=logging.DEBUG, freq=50, data_struct=data, imu=imu, motor_left=motor_left, motor_right=motor_right, controller=balancePID)
main_loop.start()

if __name__ == '__main__':
    while True:
        msg = socket.recv_string()
        if not "blabla" in msg:
            rcv = json.loads(msg)
            print(f"got data: {rcv}")
            main_loop.set_pid(rcv)

        message = json.dumps(data)
        socket.send_string(message)