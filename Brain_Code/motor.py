import time
import threading
import json
import serial
import logging
import struct

class MOTOR:
    def __init__(self, name="Motor", logging_level=logging.INFO, freq=50, port="/dev/ttyACM0", invert=False):
        self.freq = freq  # Sende-Frequenz in Hz
        self.invert = invert
        self.pos = 0.0
        self.offset = 0.0
        self.vel = 0.0
        self.sp = 0.0
        self.en = False
        self.en_last = False
        self.frequency = 0.0
        self.now = 0.0
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.data = {
            "sp": self.sp,
            "en": self.en,
            "position": self.pos,
            "velocity": self.vel,
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
            # config_string = "TT0\n"
            # dc_current
            config_string = "TT1\n"
            self.mot.write(config_string.encode('utf-8'))
            # torque control
            config_string = "TC0\n"
            self.mot.write(config_string.encode('utf-8'))

            self.logger.info("Erfolgreich initialisiert.")
        except Exception as e:
            self.logger.error("Initialisierung fehlgeschlagen: " + str(e))
            self.mot = None

    def read_floats_with_markers(self):
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
            floats = self.read_floats_with_markers()
            if floats:
                pos, vel = floats
                with self.lock:
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
            with self.lock:
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
                        sp_string = f"T{(-self.sp):.3f}\n"
                    else:
                        sp_string = f"T{self.sp:.3f}\n"
                    self.mot.write(sp_string.encode('utf-8'))

                # flushOutput() entfernt (nicht nötig und langsam)

        except Exception as e:
            self.logger.error("Senden fehlgeschlagen: " + str(e))

    def _motor_loop(self):
        last_time = 0.0
        send_interval = 1 / self.freq  # Intervall in Sekunden
        last_send_time = time.perf_counter()
        self.mot.flushInput()
        while self.running:
            try:
                self._rcv_data()

                now = time.perf_counter()
                with self.lock:
                    self.now = now
                    self.frequency = (1 / (self.now - last_time)) if last_time != 0 else 0.0
                    last_time = self.now

                # Sollwert regelmäßig senden
                if (now - last_send_time) >= send_interval:
                    self._send_data()
                    last_send_time = now

            except Exception as e:
                self.logger.error(str(e))

            time.sleep(0.001)  # 1ms sleep, damit CPU nicht 100% läuft

    def start(self):
        """Startet das Motor-Tracking in einem separaten Thread."""
        if self.mot and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._motor_loop, daemon=True)
            self.thread.start()
            self.logger.info("Thread gestartet.")

    def stop(self):
        """Stoppt das Motor-Tracking."""
        if self.thread:
            self.thread.join()
            self.logger.info("Thread gestoppt.")
        self.running = False

    def set(self, data):
        with self.lock:
            self.en = data["en"]
            self.data["en"] = self.en
            self.sp = data["sp"]
            self.data["sp"] = self.sp
        # Kein direktes _send_data() mehr hier nötig → passiert jetzt regelmäßig in Loop
        self.logger.debug("set")

    def get(self):
        with self.lock:
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
