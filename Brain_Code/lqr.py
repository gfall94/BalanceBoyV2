import traceback
import logging
import time
import numpy as np
import control

class LQR:
    def __init__(self, name = "LQR", logging_level = logging.INFO, freq=50.0, config={}, physics={}, min_max=100.0):
        self.freq = freq
        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0
        self.dt = 0.0
        self.en = False
        self.config = config
        self.physics = physics

        self.Q = np.diag(self.config["Q"])
        self.R_c = np.array([[self.config["R"]]])

        self.r = self.physics["r"]
        self.R = self.physics["R"]
        self.g = self.physics["g"]
        self.m = self.physics["m"]
        self.J = self.physics["J"]
        self.tau_m = self.physics["tau_m"]
        self.K_m = self.physics["K_m"]

        self.A = np.array([[0, 0, 1, 0],
                            [0, 0, 0, 1],
                            [(self.m * self.g * self.R) / self.J, 0, 0, (self.m * self.r * self.R) / (self.tau_m * self.J)],
                            [0, 0, 0, -1 / self.tau_m]])

        self.B = np.array([[0],
                            [0],
                            [(-self.K_m * self.m * self.r * self.R) / (self.tau_m * self.J)],
                            [self.K_m / self.tau_m]])

        self.C = np.array([[1, 1, 1, 1]])
        self.D = np.array([[0]])

        self.Gains = None

        self.min = -min_max
        self.max = min_max

        self.x = np.array([0.0, 0.0, 0.0, 0.0])
        self.sp = np.array([0.0, 0.0, 0.0, 0.0])

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
            self.calc_gains(self.config)
            self.logger.info("Erfolgreich initialisiert.")
        except Exception as e:
            self.logger.critical(str(e))
            traceback.format_exc()

    def _loop(self):
        self.now = time.perf_counter()
        self.dt = self.now - self.last_time
        self.frequency = (1 / self.dt)
        self.last_time = self.now

        try:
            if self.en:

                u = -self.Gains @ (self.x - self.sp)
                self.out = float(max(min(u, self.max), self.min))
                # self.out = 100.0
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
            traceback.format_exc()

    def loop(self, sp, x1, x2, x3, x4, data):
        self.sp = np.array([sp["p"], sp["x"], sp["pv"], sp["v"]])
        # self.sp = np.array([0,0,0,0])

        self.en = data["en"]

        self.x = np.array([x1, x2, x3, x4])

        self._loop()

        return self.data

    def calc_gains(self, config):
        self.config = config
        Q = np.diag(self.config["Q"])
        R_c = np.array([[self.config["R"]]])
        self.Q = Q
        self.R_c = R_c

        # 1. create system
        sys_cont = control.StateSpace(self.A, self.B, self.C, self.D)

        # 2. discretize
        Ts = 1/self.freq
        sys_disc = control.c2d(sys_cont, Ts, method='zoh')

        Ad = sys_disc.A
        Bd = sys_disc.B

        # 3. calc gains
        self.Gains, S, E = control.dlqr(Ad, Bd, self.Q, self.R_c)
        print("Diskreter LQR Gain K:", self.Gains)
    # def reset(self):
    #     self.integral = 0.0
    #     self.derivative = 0.0
    #     self.error = 0.0
    #     self.prev_error = 0.0



