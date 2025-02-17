import time
import threading
import requests
import json
import serial

class MOTOR:
    def __init__(self, debug=False, freq=50, port="/dev/ttyACM0", side="left", invert=False):
        self.debug = debug
        self.freq = freq
        self.side = side
        self.invert = invert
        self.pos = 0.0
        self.vel = 0.0
        self.sp = 0.0
        self.en = None
        self.running = False
        self.thread = None
        self.webhook_url_pv = "http://127.0.0.1:5000/motor_"+side+"/pv"
        self.webhook_url_sp = "http://127.0.0.1:5000/motor_"+side+"/sp"

        try:
            self.mot = serial.Serial(port, 115200, timeout=1.0)
            print("Motor ("+side+") erfolgreich initialisiert.")
        except Exception as e:
            print("Motor-Initialisierung ("+side+") Fehler:", e)
            self.mot = None

    def _rcv_data(self,now,frequency):
        line = self.mot.readline().decode('utf-8').strip()
        if line:  # Prüfen, ob Zeile nicht leer ist
            values = line.split(",")  # Daten aufteilen
            if len(values) == 2:  # Prüfen, ob genau zwei Werte vorhanden sind
                try:
                    self.pos = float(values[0])
                    self.vel = float(values[1])


                except ValueError:
                    print("Motor-PV (" + self.side + ") Fehler: Ungültige Zahlenwerte erhalten")
            else:
                print("Motor-PV (" + self.side + ") Fehler: Falsches Datenformat")

        data = {
            'position': self.pos,
            'velocity': self.vel,
            'time': now,
            'frequency': frequency
        }

        r = requests.post(self.webhook_url_pv, data=json.dumps(data), headers={'Content-Type': 'application/json'})
        if self.debug:
            print("Webhook Response: ", r)

    def _send_data(self):
        try:
            response = requests.get(self.webhook_url_sp)
            if response.status_code == 200:
                data = response.json()
                if self.en != data["en"]:
                    self.en = data["en"]
                    if not self.en:
                        en_string = "TE0"
                    else:
                        en_string = "TE1"
                    self.mot.write(en_string.encode('utf-8'))
                    # print(en_string)

                if self.en:
                    self.sp = data["value"]
                    sp_string = "T"+str(self.sp)
                    self.mot.write(sp_string.encode('utf-8'))

        except Exception as e:
            print("Motor-SP (" + self.side + ") Fehler:", e)

    def _motor_loop(self):
        last_time = 0.0
        while self.running:
            now = time.perf_counter()
            if (now - last_time) < (1 / self.freq):
                continue
            frequency = (1 / (now - last_time))
            last_time = now

            try:
                self._rcv_data(now,frequency)
                self._send_data()

                if self.debug:
                    print("Frequency:", frequency)

                    print("Gyro:")
                    #print(f"X: {gyro_x:.6f}  Y: {gyro_y:.6f} Z: {gyro_z:.6f} rads/s")

                    print("")
            except Exception as e:
                print("Motor ("+self.side+")-Fehler:", e)

    def start(self):
        """Startet das Motor-Tracking in einem separaten Thread."""
        if self.mot and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._motor_loop, daemon=True)
            self.thread.start()
            print("Motor("+self.side+")-Thread gestartet.")

    def stop(self):
        """Stoppt das Motor-Tracking."""
        self.running = False
        if self.thread:
            self.thread.join()
            print("Motor("+self.side+")-Thread gestoppt.")
