import numpy as np
import control
import matplotlib.pyplot as plt
import time
t1 = time.perf_counter()
# physics
r = 57.75/1000 # wheel radius in m
R = 138.441/1000 # wheel to cog in m
g = 9.81 # gravitation im m/s^2
m = 885.54481/1000 # mass of robot in kg
J = 0.00545106520548 # Inertia of chassis kg*m^2
tau_m = 1 # time constant of motors
K_m = 1 # motor Gain

# 1. Kontinuierliches System definieren
A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [(m*g*R)/J, 0, 0, (m*r*R)/(tau_m*J)],
              [0, 0, 0, -1/tau_m]])

B = np.array([[0],
              [0],
              [(-K_m*m*r*R)/(tau_m*J)],
              [K_m/tau_m]])

C = np.array([[1, 1, 1, 1]])  # nur erster Zustand als Ausgang
D = np.array([[0]])

sys_cont = control.StateSpace(A, B, C, D)

# 2. Diskretisieren (mit Abtastzeit Ts)
Ts = 0.02  # 20 ms Abtastzeit (also 50 Hz)
sys_disc = control.c2d(sys_cont, Ts, method='zoh')

Ad = sys_disc.A
Bd = sys_disc.B

# 3. Diskreten LQR berechnen
Q = np.diag([100, 5, 50, 25])  # Gewichtung der 4 Zustände
R = np.array([[1]])       # Gewichtung des Eingangs

K, S, E = control.dlqr(Ad, Bd, Q, R)
print("Diskreter LQR Gain K:", K)
print(time.perf_counter()-t1)

# 4. Simulation (mit Setpoint)
steps = 2500
T = np.arange(steps) * Ts

xout = []
uout = []
x = np.array([0.2, 0, 0, 0])[:, None]

# Setpoint für alle 4 States
x_set = np.array([0, 0, 0, 0])[:, None]  # z.B. nur x1 auf 1 regeln

for _ in T:
    # u = -K * (x - x_set)
    u = -K @ (x - x_set)
    # x_next = Ad * x + Bd * u
    x = Ad @ x + Bd @ u
    xout.append(x.flatten())
    uout.append(u.flatten())

xout = np.array(xout).T
uout = np.array(uout).flatten()

# 5. Plotten
plt.figure(figsize=(10, 6))

# Zustände
plt.subplot(2, 1, 1)
for i in range(4):
    plt.plot(T, xout[i], label=f'x{i+1}')
plt.title('Zustände x[k]')
plt.xlabel('Zeit [s]')
plt.ylabel('Zustände')
plt.grid()
plt.legend()

# Eingangssignal
plt.subplot(2, 1, 2)
plt.plot(T, uout, label='u (Regelgröße)', color='orange')
plt.title('Regelgröße u[k]')
plt.xlabel('Zeit [s]')
plt.ylabel('u')
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
