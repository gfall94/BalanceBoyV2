import logging
import time
import numpy as np

class PID:
    def __init__(self, name = "PID", logging_level = logging.INFO, config={}, mini=0.0, maxi=0.0):
        self.running = False
        self.thread = None
        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0
        self.dt = 0.0
        self.en = False
        self.config = config

        self.Kp = self.config["Kp"]
        self.Ki = self.config["Ki"]
        self.Kd = self.config["Kd"]
        self.min = mini
        self.max = maxi

        self.sp = 0.0

        self.integral = 0.0
        self.derivative = 0.0
        self.error = 0.0
        self.prev_error = 0.0

        self.x = 0.0

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

    def _loop(self):
        self.now = time.perf_counter()
        self.dt = self.now - self.last_time
        self.frequency = (1 / self.dt)
        self.last_time = self.now

        try:
            if self.en:
                self.error = self.sp-self.x

                integral = self.integral + self.error * self.dt
                self.integral = max(min(integral, self.max), self.min)
                self.derivative = (self.error - self.prev_error) / self.dt
                self.prev_error = self.error

                out = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative


                self.out = max(min(out, self.max), self.min)
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

    def loop(self, sp, x, data):
        self.sp = sp
        self.config = data["config"]
        self.Kp = np.array(self.config["Kp"])
        self.Ki = np.array(self.config["Ki"])
        self.Kd = np.array(self.config["Kd"])

        self.en = data["en"]

        self.x = x

        self._loop()

        return self.data


    def reset(self):
        self.integral = 0.0
        self.derivative = 0.0
        self.error = 0.0
        self.prev_error = 0.0



