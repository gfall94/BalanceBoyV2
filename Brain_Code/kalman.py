import traceback
import logging
import time
import numpy as np
import control

class KalmanFilter:
    def __init__(self, name="KalmanFilter", logging_level=logging.INFO, freq=50.0, config={}, physics={}):
        self.freq = freq
        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0
        self.dt = 0.0
        self.config = config
        self.physics = physics

        # 🟢 Empfohlene Startwerte für Balancer-Roboter
        # self.Q = np.diag(self.config["Q"])  # Prozessrauschen (Modell)
        # self.R = np.diag(self.config["R"])  # Messrauschen (Sensoren)
        self.Q = np.diag([50, 50, 25, 25])  # Prozessrauschen
        self.R = np.diag([0.05, 0.05, 0.5, 0.5])  # Messrauschen

        # Physikalische Parameter
        self.r = self.physics["r"]
        self.R_phys = self.physics["R"]
        self.g = self.physics["g"]
        self.m = self.physics["m"]
        self.J = self.physics["J"]
        self.tau_m = self.physics["tau_m"]
        self.K_m = self.physics["K_m"]

        # Systemmatrizen
        self.A = np.array([[0, 0, 1, 0],
                           [0, 0, 0, 1],
                           [(self.m * self.g * self.R_phys) / self.J, 0, 0, (self.m * self.r * self.R_phys) / (self.tau_m * self.J)],
                           [0, 0, 0, -1 / self.tau_m]])

        self.B = np.array([[0],
                           [0],
                           [(-self.K_m * self.m * self.r * self.R_phys) / (self.tau_m * self.J)],
                           [self.K_m / self.tau_m]])

        self.C = np.eye(4)  # Volle Zustandsmessung
        self.D = np.zeros((4,1))

        self.Ad = None
        self.Bd = None
        self.Cd = None
        self.Dd = None

        self.L = None  # Kalman-Gain

        self.x_hat = np.zeros((4, 1))  # Zustands-Schätzung
        self.u = 0.0  # Steuerungseingang

        self.out = [0, 0, 0, 0]

        self.data = {
            "x_hat": self.x_hat.flatten().tolist(),
            "time": self.now,
            "frequency": self.frequency
        }

        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging_level)

        try:
            self.calc_gains(self.config)
            self.logger.info("Kalman-Filter erfolgreich initialisiert.")
        except Exception as e:
            self.logger.critical(str(e))
            traceback.format_exc()

    def _loop(self, y_meas):
        self.now = time.perf_counter()
        self.dt = self.now - self.last_time
        self.frequency = (1 / self.dt)
        self.last_time = self.now

        try:
            # 1. Prediction
            self.x_hat = self.Ad @ self.x_hat + self.Bd * self.u

            # 2. Correction
            y_hat = self.C @ self.x_hat
            innovation = y_meas.reshape(-1, 1) - y_hat
            self.x_hat = self.x_hat + self.L @ innovation

            self.out = self.x_hat.flatten().tolist()
            # print(self.out)

            self.data = {
                "out": {
                "p": self.out[0],
                "x": self.out[1],
                "pv": self.out[2],
                "v": self.out[3]
            },
                "time": self.now,
                "frequency": self.frequency
            }

            self.logger.debug(self.data)
        except Exception as e:
            self.logger.error(str(e))
            traceback.format_exc()

    def loop(self, x1, x2, x3, x4, u, data):
        y_meas = np.array([x1, x2, x3, x4])
        self.u = u

        self._loop(y_meas)

        return self.data

    def calc_gains(self, config):
        self.config = config
        self.Q = np.diag(self.config["Q"])  # Prozessrauschen (Modell)
        self.R = np.diag(self.config["R"])  # Messrauschen (Sensoren)

        # 1. System erstellen
        sys_cont = control.StateSpace(self.A, self.B, self.C, self.D)

        # 2. Diskretisieren
        Ts = 1 / self.freq
        sys_disc = control.c2d(sys_cont, Ts, method='zoh')

        self.Ad = sys_disc.A
        self.Bd = sys_disc.B

        # 3. Kalman-Gain berechnen
        self.L, P, E = control.dlqe(self.Ad, np.eye(self.Ad.shape[0]), self.C, self.Q, self.R)
        print("Kalman Gain L:", self.L)
