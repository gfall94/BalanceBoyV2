import logging
import time
import threading

class LQR:
    def __init__(self, name = "LQR", logging_level = logging.INFO, freq=50, config={}):
        self.freq = freq
        self.running = False
        self.thread = None
        self.frequency = 0.0
        self.now = 0.0
        self.en = False
        self.config = config
        self.sp = {
            "p":0.0,
            "x":0.0,
            "pv": 0.0,
            "v": 0.0,
            "yaw": 0.0
            }
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

    def _lqr_loop(self):
        last_time = 0.0
        while self.running:
            self.now = time.perf_counter()
            if (self.now - last_time) < (1 / self.freq):
                continue
            self.frequency = (1 / (self.now - last_time))
            last_time = self.now

            try:
                if self.en:
                    bla=1
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
            self.thread = threading.Thread(target=self._lqr_loop, daemon=True)
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

    def set(self, sp, data):
        self.sp = sp
        self.config = data["config"]
        self.en = data["en"]

