import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

BUFFER_SIZE = 200
time_data = np.linspace(-BUFFER_SIZE, 0, BUFFER_SIZE)
quat_i_data = np.zeros(BUFFER_SIZE)
quat_j_data = np.zeros(BUFFER_SIZE)
quat_k_data = np.zeros(BUFFER_SIZE)
quat_real_data = np.zeros(BUFFER_SIZE)

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


def update(frame):
    global quat_i_data, quat_j_data, quat_k_data, quat_real_data
    try:
        data, _ = sock.recvfrom(1024)
        quat_i, quat_j, quat_k, quat_real = map(float, data.decode().split(","))

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
    except Exception as e:
        print("Fehler:", e)

    return line_i, line_j, line_k


ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.show()
