import evdev
from select import select
import traceback
import logging
import threading
import time

class PS4Controller():
    def __init__(self, name="PS4", logging_level=logging.INFO):
        self.dev = evdev.InputDevice('/dev/input/event2')

        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0
        self.dt = 0.0

        self.thread = None
        self.running = False
        self.lock = threading.Lock()

        self.v_max = 5.0
        self.y_max = 5.0

        self.data = {
            "p": 0.0,
            "x": 0.0,
            "pv": 0.0,
            "v": 0.0,
            "yaw": 0.0
        }

        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging_level)

        try:
            self.logger.info("PS4-Controller erfolgreich initialisiert.")
        except Exception as e:
            self.logger.critical(str(e))
            traceback.print_exc()

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._thread_loop, daemon=True)
            self.thread.start()
            self.logger.info("PS4-Controller-Thread gestartet.")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            self.logger.info("PS4-Controller-Thread gestoppt.")

    def _thread_loop(self):
        while self.running:
            try:
                self.loop()
            except Exception as e:
                self.logger.error(f"Fehler im PS4-Thread: {e}")
                traceback.print_exc()
            time.sleep(0.001)  # 1ms Pause, um CPU-Last zu senken

    def loop(self):
        r, w, x = select([self.dev], [], [], 0.001)  # 1ms Timeout
        if self.dev in r:
            for event in self.dev.read():
                if event.type == evdev.ecodes.EV_ABS:
                    # print(evdev.categorize(event))  # Hier kannst du weitermachen
                    bla=1