import time
import threading
import requests
import json

class LQR:
    def __init__(self, debug=False, freq=50):
        self.data = {
            "imu": {
                "pitch": 0.0,
                "pitch_d": 0.0,
                "yaw": 0.0
            },
            "motor_left": {
                "position": 0.0,
                "velocity": 0.0,
                "sp": 0.0,
                "en": False
            },
            "motor_right": {
                "position": 0.0,
                "velocity": 0.0,
                "sp": 0.0,
                "en": False
            },
            "sp": {
                "lqr": [0.0, 0.0, 0.0, 0.0],
                "yaw": [0.0, 0.0]
            },
            "gains": {
                "lqr": [0.0, 0.0, 0.0, 0.0],
                "yaw": [0.0, 0.0, 0.0]
            },
            "frequency": 0.0,
            "running": False,
            "config": {
                "debug": debug,
                "frequency": freq,
                "max_sp_mot": 1.0,
                "TODO": True,
                "mode": "OFF",
                "inertia": "TODO",
                "q": [0.0, 0.0, 0.0, 0.0],
                "r": [0.0]
            },
            "webhook": {
                "imu": "http://127.0.0.1:5000/imu/pv",
                "mot_left_pv": "http://127.0.0.1:5000/mot_left/pv",
                "mot_left_sp": "http://127.0.0.1:5000/mot_left/sp",
                "mot_right_pv": "http://127.0.0.1:5000/mot_right/pv",
                "mot_right_sp": "http://127.0.0.1:5000/mot_right/sp",
                "sp": "http://127.0.0.1:5000/lqr/sp",
                "config": "http://127.0.0.1:5000/lqr/config"
            }
        }
        self.now = 0.0
        self.thread = None


    def _set_data(self, webhook, data):
        try:
            r = requests.post(webhook, data=json.dumps(data), headers={'Content-Type': 'application/json'})
            if self.data["debug"]:
                print("Webhook Response: ", r)
        except Exception as e:
            print("LQR set data failed for "+webhook+":", e)

    def _get_data(self, webhook):
        try:
            response = requests.get(webhook)
            if response.status_code == 200:
                data = response.json()
                return data
        except Exception as e:
            print("LQR get data failed for "+webhook+":", e)

    def _get_pv(self):
        data = self._get_data(self.data["webhook"]["imu"])
        self.data["imu"]["pitch"] = data["pitch"]
        self.data["imu"]["pitch_d"] = data["gyro_x"]
        self.data["imu"]["yaw"] = data["yaw"]

        data = self._get_data(self.data["webhook"]["mot_left_pv"])
        self.data["motor_left"]["position"] = data["position"]
        self.data["motor_left"]["velocity"] = data["velocity"]

        data = self._get_data(self.data["webhook"]["mot_right_pv"])
        self.data["motor_right"]["position"] = data["position"]
        self.data["motor_right"]["velocity"] = data["velocity"]

    def _get_sp(self):
        data = self._get_data(self.data["webhook"]["sp"])
        # TODO

    def _get_config(self):
        data = self._get_data(self.data["webhook"]["config"])
        # TODO

    def _set_motors(self):
        data = {
            "value": self.data["motor_left"]["sp"],
            "en": self.data["motor_left"]["en"],
            'time': self.now
        }
        self._set_data(self.data["webhook"]["mot_left"], data)
        data = {
            "value": self.data["motor_right"]["sp"],
            "en": self.data["motor_right"]["en"],
            'time': self.now
        }
        self._set_data(self.data["webhook"]["mot_right"], data)

    def _calc_gains(self):
        data = "TODO"
        data = [0.0,0.0,0.0,0.0]
        self.data["gains"]["lqr"] = data
        data = [0.0,0.0,0.0]
        self.data["gains"]["yaw"] = data

    def _controller(self):
        data="TODO"
        data = 0.0
        self.data["motor_left"]["sp"] = data
        self.data["motor_right"]["sp"] = data

    def _lqr_loop(self):
        last_time = 0.0
        while self.data["running"]:
            now = time.perf_counter()
            if (now - last_time) < (1 / self.data["config"]["freq"]):
                continue
            self.data["frequency"] = (1 / (now - last_time))
            last_time = now

            try:
                self._get_pv()
                self._get_sp()
                self._get_config()
                self._controller()
                self._set_motors()

                if self.data["config"]["debug"]:
                    print("Frequency:", self.data["frequency"])

                    print("Gyro:")
                    #print(f"X: {gyro_x:.6f}  Y: {gyro_y:.6f} Z: {gyro_z:.6f} rads/s")

                    print("")
            except Exception as e:
                print("LQR-Fehler:", e)

    def start(self):
        """Startet das Motor-Tracking in einem separaten Thread."""
        if not self.data["running"]:
            self.data["running"] = True
            self.thread = threading.Thread(target=self._lqr_loop, daemon=True)
            self.thread.start()
            print("LQR-Thread gestartet.")

    def stop(self):
        """Stoppt das Motor-Tracking."""
        self.data["running"] = False
        if self.thread:
            self.thread.join()
            print("LQR-Thread gestoppt.")
