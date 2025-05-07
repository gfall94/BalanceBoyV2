import evdev
from evdev import ecodes
from select import select
import traceback
import logging
import threading
import time
import numpy as np

class PS4Controller():
    def __init__(self, name="PS4", logging_level=logging.INFO):

        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0
        self.dt = 0.0

        self.thread = None
        self.running = False
        self.lock = threading.Lock()

        self.dev = None
        self.connected = False

        # Buttons
        self.x = False
        self.v = False
        self.d = False
        self.o = False
        self.l1 = False
        self.r1 = False
        self.share = False
        self.option = False
        self.l3 = False
        self.r3 = False
        self.ps = False

        # D-Pad
        self.dpad_x = 0
        self.dpad_y = 0

        # Joystick-Positionen
        self.left_x = 0
        self.left_y = 0
        self.right_x = 0
        self.right_y = 0
        self.l2 = 0
        self.r2 = 0
        self.left_x_offset = -3
        self.left_y_offset = 0
        self.right_x_offset = 5
        self.right_y_offset = -3
        self.l2_offset = 0
        self.r2_offset = 0
        self.min = 0
        self.max = 255

        # Mapping der PS4-Tasten
        self.button_map = {
            ecodes.BTN_SOUTH: "X",
            ecodes.BTN_EAST: "Kreis",
            ecodes.BTN_NORTH: "Dreieck",
            ecodes.BTN_WEST: "Viereck",
            ecodes.BTN_TL: "L1",
            ecodes.BTN_TR: "R1",
            ecodes.BTN_SELECT: "Share",
            ecodes.BTN_START: "Options",
            ecodes.BTN_THUMBL: "L3",
            ecodes.BTN_THUMBR: "R3",
            ecodes.BTN_MODE: "PS-Button",
        }

        self.data = {
                "time": 0,
                "frequency": 0,
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

        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging_level)

        try:
            self.try_reconnect()
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
                self.now = time.perf_counter()
                self.dt = self.now - self.last_time
                self.frequency = (1 / self.dt)
                self.last_time = self.now

                self.loop()
            except Exception as e:
                self.logger.error(f"Fehler im PS4-Thread: {e}")
                traceback.print_exc()
            time.sleep(0.001)  # 1ms Pause, um CPU-Last zu senken

    def _process_inputs(self):
        r, w, x = select([self.dev], [], [], 0.001)  # 1ms Timeout
        if self.dev in r:
            for event in self.dev.read():

                # Buttons
                if event.type == ecodes.EV_KEY:
                    # if event.value == 1:  # Taste gedrückt
                    if event.code in self.button_map:
                        match event.code:
                            case ecodes.BTN_SOUTH:
                                self.x = bool(event.value)
                            case ecodes.BTN_EAST:
                                self.o = bool(event.value)
                            case ecodes.BTN_NORTH:
                                self.d = bool(event.value)
                            case ecodes.BTN_WEST:
                                self.v = bool(event.value)
                            case ecodes.BTN_TL:
                                self.l1 = bool(event.value)
                            case ecodes.BTN_TR:
                                self.r1 = bool(event.value)
                            case ecodes.BTN_SELECT:
                                self.share = bool(event.value)
                            case ecodes.BTN_START:
                                self.option = bool(event.value)
                            case ecodes.BTN_THUMBL:
                                self.l3 = bool(event.value)
                            case ecodes.BTN_THUMBR:
                                self.r3 = bool(event.value)
                            case ecodes.BTN_MODE:
                                self.ps = bool(event.value)
                # Joysticks
                elif event.type == ecodes.EV_ABS:
                    match event.code:
                        case ecodes.ABS_X:
                            self.left_x = np.clip(255-event.value-self.left_x_offset, self.min, self.max)
                            if ((self.max-self.min)-(self.max-self.min)/2)+5 > self.left_x > ((self.max-self.min)-(self.max-self.min)/2)-5:
                                self.left_x = (self.max-self.min)/2
                        case ecodes.ABS_Y:
                            self.left_y = np.clip(255-event.value-self.left_y_offset, self.min, self.max)
                        case ecodes.ABS_RX:
                            self.right_x = np.clip(255-event.value-self.right_x_offset, self.min, self.max)
                        case ecodes.ABS_RY:
                            self.right_y = np.clip(255-event.value-self.right_y_offset, self.min, self.max)
                        case ecodes.ABS_Z:
                            self.l2 = np.clip(event.value-self.l2_offset, self.min, self.max)
                        case ecodes.ABS_RZ:
                            self.r2 = np.clip(event.value-self.r2_offset, self.min, self.max)

                        # D-Pad
                        case ecodes.ABS_HAT0X:
                            self.dpad_x = -event.value
                        case ecodes.ABS_HAT0Y:
                            self.dpad_y = -event.value
                    # print(f"L: x={self.left_x}, y={self.left_y}\tR: x={self.right_x}, y={self.right_y}\tH: l={self.l2}, r={self.r2}")
                    # print(f"D-Pad x: {self.dpad_x}, y: {self.dpad_y}")

    def loop(self):
            if self.is_connected():
                with self.lock:
                    self._process_inputs()
            else:
                print("Controller getrennt. Versuche Reconnect...")
                success = self.try_reconnect()
                if success:
                    print("Reconnect erfolgreich.")
                else:
                    print("Reconnect fehlgeschlagen. Neuer Versuch in 2 Sekunden.")
                    time.sleep(2)  # Wartezeit zwischen Reconnect-Versuchen
                return


    def get(self):
        with self.lock:
            self.data = {
                "time": self.now,
                "frequency": self.frequency,
                "connected": self.connected,
                "x": self.x,
                "v": self.v,
                "d": self.d,
                "o": self.o,
                "l1": self.l1,
                "r1": self.r1,
                "share": self.share,
                "option": self.option,
                "l3": self.l3,
                "r3": self.r3,
                "ps": self.ps,
                "dpad_x": self.dpad_x,
                "dpad_y": self.dpad_y,
                "left_x": self.left_x,
                "left_y": self.left_y,
                "right_x": self.right_x,
                "right_y": self.right_y,
                "l2": self.l2,
                "r2": self.r2
            }
        return self.data

    def restart(self):
        """Stoppt und startet die Klasse neu."""
        self.logger.info("Starte PS4-Controller neu...")
        self.stop()
        try:
            self.dev.close()
        except Exception:
            pass
        self.dev = None
        try:
            self.dev = evdev.InputDevice('/dev/input/event2')
            self.logger.info("PS4-Controller neu verbunden.")
        except Exception as e:
            self.logger.critical(f"Neustart fehlgeschlagen: {e}")
            traceback.print_exc()
        self.start()

    def try_reconnect(self):
        """Versucht, den Controller wieder zu verbinden, wenn er getrennt wurde."""
        try:
            # Suche nach PS4-Controller-Device
            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
            for device in devices:
                if "Sony" in device.name or "Wireless Controller" in device.name:
                    self.dev = evdev.InputDevice(device.path)
                    self.logger.info(f"Controller neu verbunden: {device.path}")
                    return True
            return False
        except Exception as e:
            self.logger.error(f"Fehler beim Reconnect: {e}")
            return False

    def is_connected(self):
        """Prüft, ob der Controller noch verbunden ist."""
        with self.lock:
            if self.dev is None:
                self.connected = False
                return False
            try:
                self.dev.read_one()  # Dummy-Leseversuch
                self.connected = True
                return True
            except (OSError, IOError):
                self.connected = False
                return False