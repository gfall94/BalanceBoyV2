import serial
import time
import matplotlib.pyplot as plt

# Öffne die serielle Verbindung
ser = serial.Serial('COM4', 921600, timeout=0.1)
time.sleep(2)  # Warten bis Arduino bereit ist

# Ergebnisse speichern
latenzen = []

print("Starte Test...")

for i in range(100):
    # Sende T-Befehl (z.B. mit Ziel 0.0)
    string = "TE1\n"
    ser.write(string.encode('utf-8'))

    start_time = time.perf_counter()

    # Warten auf Antwort
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith("Latenz(us):"):
            latency_us = int(line.split(':')[1])
            latenzen.append(latency_us)
            print(f"{i + 1}/100: {latency_us} us")
            break

    # Minimal warten, bevor nächster Befehl kommt
    time.sleep(0.005)  # 5ms / 200Hz

ser.close()

# Plot anzeigen
plt.plot(latenzen, marker='o')
plt.xlabel('Befehl #')
plt.ylabel('Latenz (us)')
plt.title('Arduino Antwort-Latenz')
plt.grid(True)
plt.show()
