import numpy as np
import control
import logging
import traceback

class KalmanObserver:
    def __init__(self, name = "Kalman", logging_level = logging.INFO, freq=50.0, config={}, physics={}):
        self.freq = freq
        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0
        self.dt = 0.0
        self.en = False
        self.config = config
        self.physics = physics

        self.r = self.physics["r"]
        self.R = self.physics["R"]
        self.g = self.physics["g"]
        self.m = self.physics["m"]
        self.J = self.physics["J"]
        self.tau_m = self.physics["tau_m"]
        self.K_m = self.physics["K_m"]

        self.A = np.array([[0, 0, 1, 0],
                           [0, 0, 0, 1],
                           [(self.m * self.g * self.R) / self.J, 0, 0,
                            (self.m * self.r * self.R) / (self.tau_m * self.J)],
                           [0, 0, 0, -1 / self.tau_m]])

        self.B = np.array([[0],
                           [0],
                           [(-self.K_m * self.m * self.r * self.R) / (self.tau_m * self.J)],
                           [self.K_m / self.tau_m]])

        self.C = np.array([[1, 1, 1, 1]])
        self.D = np.array([[0]])

        self.Ad = None
        self.Bd = None
        self.Cd = None
        self.Dd = None

        self.Q = np.eye(self.A.shape[0])
        self.R_k = np.eye(self.C.shape[0])

        self.P = np.eye(self.A.shape[0])

        self.x_hat = np.zeros((self.A.shape[0], 1))
        self.x = np.array([0.0, 0.0, 0.0, 0.0])

        self.out = {
                "p":0.0,
                "x":0.0,
                "pv": 0.0,
                "v": 0.0
                }
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
            self.calc_system()
            self.logger.info("Erfolgreich initialisiert.")
        except Exception as e:
            self.logger.critical(str(e))
            traceback.format_exc()

    def calc_system(self):
        # 1. create system
        sys_cont = control.StateSpace(self.A, self.B, self.C, self.D)

        # 2. discretize
        Ts = 1/self.freq
        sys_disc = control.c2d(sys_cont, Ts, method='zoh')

        self.Ad = sys_disc.A
        self.Bd = sys_disc.B
        self.Cd = sys_disc.C
        self.Dd = sys_disc.D

    def predict(self, u):
        """
        Vorhersage-Schritt
        u: Eingangssignal (Steuersignal), muss 2D np.array sein z.B. np.array([[u]])
        """
        try:
            # x(k|k-1) = A * x(k-1|k-1) + B * u
            self.x_hat = self.Ad @ self.x_hat + self.Bd @ u

            # P(k|k-1) = A * P(k-1|k-1) * A.T + Q
            self.P = self.Ad @ self.P @ self.Ad.T + self.Q

        except Exception as e:
            self.logger.error("Fehler im Predict-Schritt: " + str(e))
            traceback.format_exc()

    def update(self, y):
        """
        Update-Schritt mit Messung
        y: Messvektor, muss 2D np.array sein z.B. np.array([[y1], [y2], [y3], [y4]])
        """
        try:
            # Innovation: y - C * x
            y_tilde = y - self.Cd @ self.x_hat

            # Innovationskovarianz: S = C * P * C.T + R
            S = self.Cd @ self.P @ self.Cd.T + self.R_k

            # Kalman-Gain
            K = self.P @ self.Cd.T @ np.linalg.inv(S)

            # Zustandsschätzung aktualisieren: x(k|k) = x(k|k-1) + K * (y - C * x)
            self.x_hat = self.x_hat + K @ y_tilde

            # Fehlerkovarianz aktualisieren: P = (I - K * C) * P
            I = np.eye(self.P.shape[0])
            self.P = (I - K @ self.Cd) @ self.P

        except Exception as e:
            self.logger.error("Fehler im Update-Schritt: " + str(e))
            traceback.format_exc()

    def loop(self, u, y1, y2, y3, y4, data):
        """
        Kompletter Kalman-Filter-Schritt (Prediction + Update)
        u: Eingangssignal (np.array([[u]]))
        y: Messvektor (np.array([[y1], [y2], ...]))
        Gibt den geschätzten Zustand x_hat zurück
        """
        self.config = data["config"]
        Q = np.diag(self.config["Q"])
        R = np.array([[self.config["R"]]])
        if (not np.array_equal(self.Q, Q)) or (not np.array_equal(self.R_k, R)):
            self.Q = Q
            self.R_k = R

        self.en = data["en"]

        y = np.array([[y1], [y2], [y3], [y4]])
        u_ = np.array([[u]])
        self.predict(u_)
        self.update(y)

        self.out = {
            "p": self.x_hat[0],
            "x": self.x_hat[1],
            "pv": self.x_hat[2],
            "v": self.x_hat[3]
        }

        self.data["out"] = self.out

        return self.data
