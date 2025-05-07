import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import control as ctrl

# CSV-Daten laden
df = pd.read_csv("motor_measurement.csv")

# Extrahiere die Spalten
t = df["time"].values
t = t-min(t)
u = df["u"].values  # Eingang
y = df["y1"].values  # Ausgang

# PT1-Modell: berücksichtigt u(t)
def pt1_response(t, K, tau):
    return K * u * (1 - np.exp(-t / tau))

# Curve Fit
popt, pcov = curve_fit(pt1_response, t, y, p0=[1, 1])
K_fit, tau_fit = popt

print(f"Identifiziertes K: {K_fit:.3f}")
print(f"Identifiziertes tau: {tau_fit:.3f}")

# Gefittetes Modell berechnen
y_fit = pt1_response(t, K_fit, tau_fit)

# Plot
plt.figure(figsize=(10, 6))
plt.plot(t, u, label="Input u(t)", color='gray', linestyle=':')
plt.plot(t, y, label="Gemessen y(t)", color='blue')
plt.plot(t, y_fit, '--', label="Gefittetes Modell y_fit(t)", color='red')
plt.xlabel("Zeit [s]")
plt.ylabel("Signal")
plt.legend()
plt.grid(True)
plt.show()
