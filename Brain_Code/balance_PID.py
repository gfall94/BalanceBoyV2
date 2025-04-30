import logging
import time
import threading
import numpy as np

class BalancePID:
    def __init__(self, name = "BalancePID", logging_level = logging.INFO, freq=50, config={}):
        self.freq = freq
        self.running = False
        self.thread = None
        self.frequency = 0.0
        self.now = 0.0
        self.dt = 0.0
        self.en = False
        self.config = config

        self.Kp = np.array(self.config["Kp"])
        self.Ki = np.array(self.config["Ki"])
        self.Kd = np.array(self.config["Kd"])

        self.sp = np.array([0.0, 0.0, 0.0, 0.0])

        self.integral = np.zeros(4)
        self.derivative = np.zeros(4)
        self.error = np.zeros(4)
        self.prev_error = np.zeros(4)

        self.pitch = 0.0
        self.pos = 0.0
        self.pitch_vel = 0.0
        self.vel = 0.0
        self.x = np.array([self.pitch, self.pos, self.pitch_vel, self.vel])

        self.out = 0.0
        self.data = {
            "config": self.config,
            "out": self.out,
            "en": self.en,
            "time": self.now,
            "frequency": self.frequency
        }

        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging_level)

        try:
            # hier gains berechnen
            self.logger.info("Erfolgreich initialisiert.")
        except Exception as e:
            self.logger.critical(str(e))

    def _balance_pid_loop(self):
        last_time = 0.0
        while self.running:
            self.now = time.perf_counter()
            self.dt = self.now - last_time
            if self.dt < (1 / self.freq):
                continue
            self.frequency = (1 / self.dt)
            last_time = self.now

            try:
                if self.en:
                    self.error = self.sp-self.x

                    self.integral += self.error * self.dt
                    self.derivative = (self.error - self.prev_error) / self.dt
                    self.prev_error = self.error

                    u_components = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
                    out = np.sum(u_components)

                    self.out = max(min(out, 1.0), -1.0)
                else:
                    self.out = 0.0

                self.data = {
                    "config": self.config,
                    "out": self.out,
                    "en": self.en,
                    "time": self.now,
                    "frequency": self.frequency
                }

                self.logger.debug(self.data)
            except Exception as e:
                self.logger.error(str(e))

    def start(self):
        """Startet das IMU-Tracking in einem separaten Thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._balance_pid_loop, daemon=True)
            self.thread.start()
            self.logger.info("Thread gestartet.")


    def stop(self):
        """Stoppt das IMU-Tracking."""
        if self.thread:
            self.thread.join()
            self.logger.info("Thread gestoppt.")
        self.running = False

    def get(self):
        return self.data

    def set(self, data):
        self.sp = np.array([data["sp"]["p"], data["sp"]["x"], data["sp"]["pv"], data["sp"]["v"]])
        self.config = data["balance_pid"]["config"]
        self.Kp = np.array(self.config["Kp"])
        self.Ki = np.array(self.config["Ki"])
        self.Kd = np.array(self.config["Kd"])

        self.en = data["balance_pid"]["en"]

        self.pitch = data["imu"]["pitch"]
        self.pitch_vel = data["imu"]["gyro_y"]

        self.pos = (data["motor_left"]["position"]+data["motor_right"]["position"])/2
        self.vel = (data["motor_left"]["velocity"]+data["motor_right"]["velocity"])/2

        self.x = np.array([self.pitch, self.pos, self.pitch_vel, self.vel])

    def reset(self):
        self.integral = np.zeros(4)
        self.derivative = np.zeros(4)
        self.error = np.zeros(4)
        self.prev_error = np.zeros(4)



