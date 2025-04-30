import time
import zmq
import json

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

from collections import deque

port = "5556"
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.101:%s" % port)


time_window = 10
update_interval = 75
running = True
start_time = time.time()

# Datenpuffer für Zeitstempel & Werte
timestamps = deque(maxlen=time_window*update_interval)
keys_to_plot = ["pitch", "yaw"]
data_buffers = {key: deque(maxlen=time_window*update_interval) for key in keys_to_plot}

# Plot initialisieren
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
ax.set_xlabel("Zeit (s)")
ax.grid(True)

# Linien für geplottete Werte
lines = {key: ax.plot([], [], label=key)[0] for key in keys_to_plot}
ax.legend()

# Play/Pause-Knopf
def toggle_animation(event):
    """Startet oder stoppt die Animation."""
    global running
    if running:
        ani.event_source.stop()
    else:
        ani.event_source.start()
    running = not running
ax_button = plt.axes([0.8, 0.05, 0.1, 0.075])
button = Button(ax_button, "Play/Pause")
button.on_clicked(toggle_animation)


def fetch_data():
    """Holt die Daten von der API und speichert sie."""
    try:
        socket.send_string("hier daten senden")
        data = json.loads(socket.recv_string())
        current_time = time.perf_counter() - start_time
        timestamps.append(current_time)

        # Füge nur die ausgewählten Werte hinzu
        data_buffers["pitch"].append(data["imu"]["pitch"])
        data_buffers["yaw"].append(data["imu"]["yaw"])


    except Exception as e:
        print("Fehler beim Empfangen:", e)

def update_plot(frame):
    """Aktualisiert den Plot mit neuen Daten."""
    if running:
        fetch_data()

        if timestamps:
            min_time = timestamps[-1] - time_window
            ax.set_xlim(min_time, timestamps[-1])

            # Dynamische Y-Skalierung für alle Werte
            all_values = [val for key in keys_to_plot for val in data_buffers[key]]
            if all_values:
                ax.set_ylim(min(all_values) - 5, max(all_values) + 5)

            # Y-Achsen-Beschriftung dynamisch setzen
            ax.set_ylabel("Messwerte")

            # Linien aktualisieren
            for key in keys_to_plot:
                lines[key].set_data(timestamps, data_buffers[key])

    return lines.values()

# Matplotlib-Animation
ani = animation.FuncAnimation(fig, update_plot, interval=update_interval)
plt.show()

