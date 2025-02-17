import requests
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import threading
from collections import deque
from matplotlib.widgets import Button


class LivePlotter(threading.Thread):
    def __init__(self, url, keys_to_plot, title = "Echtzeit-Plot", time_window=60, update_interval=10):
        """
        Initialisiert den Live-Plotter.

        :param url: Die URL, von der die JSON-Daten abgerufen werden.
        :param keys_to_plot: Liste der Datenkeys, die geplottet werden sollen.
        :param time_window: Zeitfenster für die X-Achse in Sekunden (Standard: 60).
        :param update_interval: Aktualisierungsintervall in Millisekunden (Standard: 10ms).
        """
        super().__init__()
        self.url = url
        self.keys_to_plot = set(keys_to_plot)
        self.title = title
        self.time_window = time_window
        self.update_interval = update_interval
        self.running = True
        self.start_time = time.time()

        # Datenpuffer für Zeitstempel & Werte
        self.timestamps = deque(maxlen=1000)
        self.data_buffers = {key: deque(maxlen=1000) for key in keys_to_plot}

        # Plot initialisieren
        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(bottom=0.2)
        self.ax.set_xlabel("Zeit (s)")
        self.ax.set_title(self.title)
        self.ax.grid(True)

        # Linien für geplottete Werte
        self.lines = {key: self.ax.plot([], [], label=key)[0] for key in keys_to_plot}
        self.ax.legend()

        # Play/Pause-Knopf
        ax_button = plt.axes([0.8, 0.05, 0.1, 0.075])
        self.button = Button(ax_button, "Play/Pause")
        self.button.on_clicked(self.toggle_animation)

        # Matplotlib-Animation
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=self.update_interval)

    def toggle_animation(self, event):
        """Startet oder stoppt die Animation."""
        if self.running:
            self.ani.event_source.stop()
        else:
            self.ani.event_source.start()
        self.running = not self.running

    def fetch_data(self):
        """Holt die Daten von der API und speichert sie."""
        try:
            response = requests.get(self.url, timeout=1)
            if response.status_code == 200:
                data = response.json()
                current_time = time.time() - self.start_time
                self.timestamps.append(current_time)

                # Füge nur die ausgewählten Werte hinzu
                for key in self.keys_to_plot:
                    if key in data:
                        self.data_buffers[key].append(data[key])

        except requests.RequestException as e:
            print(f"Fehler beim Abrufen der Daten von {self.url}: {e}")

    def update_plot(self, frame):
        """Aktualisiert den Plot mit neuen Daten."""
        if self.running:
            self.fetch_data()

            if self.timestamps:
                min_time = self.timestamps[-1] - self.time_window
                self.ax.set_xlim(min_time, self.timestamps[-1])

                # Dynamische Y-Skalierung für alle Werte
                all_values = [val for key in self.keys_to_plot for val in self.data_buffers[key]]
                if all_values:
                    self.ax.set_ylim(min(all_values) - 5, max(all_values) + 5)

                # Y-Achsen-Beschriftung dynamisch setzen
                self.ax.set_ylabel("Messwerte")

                # Linien aktualisieren
                for key in self.keys_to_plot:
                    self.lines[key].set_data(self.timestamps, self.data_buffers[key])

        return self.lines.values()

    def start(self):
        """Startet den Matplotlib-Plot im Thread."""
        plt.show()


# Beispielaufruf:
if __name__ == "__main__":
    url = "http://192.168.1.101:5000/imu/pv"
    keys = ["pitch", "yaw", "frequency"]  # Welche Werte geplottet werden sollen
    title = "IMU PV"
    imu_pv = LivePlotter(url, keys, title)

    url = "http://192.168.1.101:5000/motor_left/pv"
    keys = ["position", "velocity", "frequency"]  # Welche Werte geplottet werden sollen
    title = "Motor left PV"
    motor_left_pv = LivePlotter(url, keys, title)

    url = "http://192.168.1.101:5000/motor_left/sp"
    keys = ["sp", "en", "frequency"]  # Welche Werte geplottet werden sollen
    title = "Motor left SP"
    # motor_left_sp = LivePlotter(url, keys, title)

    url = "http://192.168.1.101:5000/motor_right/pv"
    keys = ["position", "velocity"]  # Welche Werte geplottet werden sollen
    title = "Motor right PV"
    #motor_right_pv = LivePlotter(url, keys, title)

    url = "http://192.168.1.101:5000/motor_right/sp"
    keys = ["position", "velocity"]  # Welche Werte geplottet werden sollen
    title = "Motor right SP"
    # motor_right_sp = LivePlotter(url, keys, title)

    imu_pv.start()  # Startet den Plot in einem eigenen Thread
    motor_left_pv.start()  # Startet den Plot in einem eigenen Thread
    # -motor_left_sp.start()  # Startet den Plot in einem eigenen Thread
    # motor_right_pv.start()  # Startet den Plot in einem eigenen Thread
    #motor_right_sp.start()  # Startet den Plot in einem eigenen Thread
