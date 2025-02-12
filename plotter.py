import asyncio
import websockets
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import queue
import threading

WS_SERVER = "ws://192.168.1.101:5005"  # Raspberry Pi IP
BUFFER_SIZE = 200  # Anzahl der Samples

# Zeitachse
time_data = np.linspace(-BUFFER_SIZE, 0, BUFFER_SIZE)
quat_i_data = np.ones(BUFFER_SIZE)
quat_j_data = np.zeros(BUFFER_SIZE)
quat_k_data = np.zeros(BUFFER_SIZE)
quat_real_data = np.zeros(BUFFER_SIZE)

data_queue = queue.Queue()

fig, ax = plt.subplots()
ax.set_ylim(-10, 10)
ax.set_xlim(-BUFFER_SIZE, 0)
ax.set_xlabel("Zeit (Samples)")
ax.set_ylabel("Geomagnetic Quaternion")
ax.set_title("Live IMU-Daten")

line_i, = ax.plot(time_data, quat_i_data, label="i", color="red")
line_j, = ax.plot(time_data, quat_j_data, label="j", color="green")
line_k, = ax.plot(time_data, quat_k_data, label="k", color="blue")
line_r, = ax.plot(time_data, quat_real_data, label="r", color="black")
ax.legend()

async def receive_data():
    """ Empfängt Daten über WebSockets """
    async with websockets.connect(WS_SERVER) as websocket:
        while True:
            try:
                data = await websocket.recv()
                quat_i, quat_j, quat_k, quat_real = map(float, data.split(","))
                data_queue.put((quat_i, quat_j, quat_k, quat_real))
            except Exception as e:
                print("Fehler beim Empfang:", e)
                break

# Starte den Empfangs-Thread
threading.Thread(target=lambda: asyncio.run(receive_data()), daemon=True).start()

def update(frame):
    """ Holt die neuesten Werte aus der Queue und aktualisiert den Plot """
    global quat_i_data, quat_j_data, quat_k_data, quat_real_data

    while not data_queue.empty():
        quat_i, quat_j, quat_k, quat_real = data_queue.get()

        quat_i_data = np.roll(quat_i_data, -1)
        quat_i_data[-1] = quat_i
        quat_j_data = np.roll(quat_j_data, -1)
        quat_j_data[-1] = quat_j
        quat_k_data = np.roll(quat_k_data, -1)
        quat_k_data[-1] = quat_k
        quat_real_data = np.roll(quat_real_data, -1)
        quat_real_data[-1] = quat_real

        line_i.set_ydata(quat_i_data)
        line_j.set_ydata(quat_j_data)
        line_k.set_ydata(quat_k_data)
        line_r.set_ydata(quat_real_data)

    return line_i, line_j, line_k, line_r

ani = animation.FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
plt.show()
