import time
import serial
import logging
import struct
import json

class MOTOR:
    def __init__(self, name="Motor", logging_level=logging.INFO, port="/dev/ttyACM0", invert=False, min_max=7.5):
        self.invert = invert
        self.factor = (min_max/100.0)
        self.pos = 0.0
        self.offset = 0.0
        self.vel = 0.0
        self.sp = 0.0
        self.en = False
        self.en_last = False
        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0
        self.data = {
            "sp": self.sp,
            "en": self.en,
            "position": self.pos,
            "velocity": self.vel,
            "velocity_LP": self.vel,
            "time": self.now,
            "frequency": self.frequency}

        self.name = name
        self.logger = logging.getLogger(self.name)
        self.logger.setLevel(logging_level)

        try:
            self.mot = serial.Serial(port, 921600, timeout=0.1)

            # # foc_current
            # config_string = "TT2\n"
            # voltage
            config_string = "TT0\n"
            # # dc_current
            # config_string = "TT1\n"
            self.mot.write(config_string.encode('utf-8'))
            # torque control
            config_string = "TC0\n"
            self.mot.write(config_string.encode('utf-8'))

            self.logger.info("Erfolgreich initialisiert.")
        except Exception as e:
            self.logger.error("Initialisierung fehlgeschlagen: " + str(e))
            self.mot = None

    def _read_floats_with_markers(self):
        while True:
            byte = self.mot.read(size=1)
            if not byte:
                return None
            if byte[0] == 0x02:  # Startmarker gefunden
                data = self.mot.read(size=8)  # Zwei Float-Werte (2 x 4 Bytes)
                if len(data) != 8:
                    continue  # Unvollständige Daten, erneut versuchen
                end = self.mot.read(size=1)
                if not end or end[0] != 0x03:
                    continue  # Endmarker nicht gefunden, erneut versuchen
                self.mot.reset_input_buffer()
                return struct.unpack('<ff', data)

    def _rcv_data(self):
        try:
            floats = self._read_floats_with_markers()
            if floats:
                pos, vel = floats
                if self.invert:
                    self.pos = -pos - self.offset
                    self.vel = -vel
                else:
                    self.pos = pos - self.offset
                    self.vel = vel
        except (ValueError, UnicodeDecodeError):
            self.logger.info("Falsches Datenformat")

    def _send_data(self):
        try:
            # Motor ein-/ausschalten
            if not self.en:
                en_string = "TE0\n"
                self.mot.write(en_string.encode('utf-8'))
            elif self.en and not self.en_last:
                en_string = "TE1\n"
                self.mot.write(en_string.encode('utf-8'))
            self.en_last = self.en

            # Sollwert senden (mit 3 Nachkommastellen)
            if self.en:
                if self.invert:
                    sp_string = f"T{(-self.sp*self.factor):.3f}\n"
                else:
                    sp_string = f"T{(self.sp*self.factor):.3f}\n"
                self.mot.write(sp_string.encode('utf-8'))
            # self.logger.error(self.data)
            # print(self.name+json.dumps(self.data, indent=4))
        except Exception as e:
            self.logger.error("Senden fehlgeschlagen: " + str(e))

    def set(self, data):
        self.en = data["en"]
        self.data["en"] = self.en
        self.sp = data["sp"]
        self.data["sp"] = self.sp
        self._send_data()
        self.logger.debug("set")

    def get(self):
        self.now = time.perf_counter()
        self.frequency = (1 / (self.now - self.last_time))
        self.last_time = self.now
        self._rcv_data()
        self.data["position"] = self.pos
        self.data["velocity"] = self.vel
        self.data["time"] = self.now
        self.data["frequency"] = self.frequency
        self.logger.debug("get")
        return self.data

    def reset(self):
        self.logger.debug("reset")
        self.offset = self.offset + self.pos
        self.pos = 0.0