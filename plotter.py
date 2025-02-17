import requests
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from collections import deque
from matplotlib.widgets import Button

# Server-URL (hier anpassen)
URL = "http://192.168.1.101:5000/imu/pv"  # Ersetze dies mit der echten URL

# Zeitfenster für den Plot in Sekunden
TIME_WINDOW = 60

# Datenpuffer
timestamps = deque(maxlen=1000)
pitch_values = deque(maxlen=1000)
roll_values = deque(maxlen=1000)
yaw_values = deque(maxlen=1000)
gyro_x_values = deque(maxlen=1000)
gyro_y_values = deque(maxlen=1000)
gyro_z_values = deque(maxlen=1000)

# Initialisiere den Plot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("Winkel (Grad)")
ax.set_title("IMU-Daten")
ax.grid(True)

pitch_line, = ax.plot([], [], label="Pitch")
roll_line, = ax.plot([], [], label="Roll")
yaw_line, = ax.plot([], [], label="Yaw")
gyro_x_line, = ax.plot([], [], label="Gyro X")
gyro_y_line, = ax.plot([], [], label="Gyro Y")
gyro_z_line, = ax.plot([], [], label="Gyro Z")
ax.legend()

# Startzeit für relative Zeitachse
start_time = time.time()

# Steuerung für Play/Pause
running = True


def toggle_animation(event):
    global running, ani
    if running:
        ani.event_source.stop()
    else:
        ani.event_source.start()
    running = not running


# Play/Pause-Knopf
ax_button = plt.axes([0.8, 0.05, 0.1, 0.075])
button = Button(ax_button, "Play/Pause")
button.on_clicked(toggle_animation)


def fetch_data():
    """Holt die Daten vom Server."""
    try:
        response = requests.get(URL, timeout=1)
        if response.status_code == 200:
            data = response.json()
            current_time = time.time() - start_time
            timestamps.append(current_time)
            pitch_values.append(data["pitch"])
            roll_values.append(data["roll"])
            yaw_values.append(data["yaw"])
            gyro_x_values.append(data["gyro_x"])
            gyro_y_values.append(data["gyro_y"])
            gyro_z_values.append(data["gyro_z"])
    except requests.RequestException as e:
        print("Fehler beim Abrufen der Daten:", e)


def update_plot(frame):
    """Aktualisiert den Plot mit neuen Daten."""
    if running:
        fetch_data()
        if timestamps:
            min_time = timestamps[-1] - TIME_WINDOW
            ax.set_xlim(min_time, timestamps[-1])

            # Dynamische Skalierung der Y-Achse
            all_values = list(pitch_values) + list(roll_values) + list(yaw_values)
            if all_values:
                ax.set_ylim(min(all_values) - 5, max(all_values) + 5)

            pitch_line.set_data(timestamps, pitch_values)
            roll_line.set_data(timestamps, roll_values)
            yaw_line.set_data(timestamps, yaw_values)
            gyro_x_line.set_data(timestamps, gyro_x_values)
            gyro_y_line.set_data(timestamps, gyro_y_values)
            gyro_z_line.set_data(timestamps, gyro_z_values)
    return pitch_line, roll_line, yaw_line, gyro_x_line, gyro_y_line, gyro_z_line


# Animation starten
ani = animation.FuncAnimation(fig, update_plot, interval=10)
plt.show()
