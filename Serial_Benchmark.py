import serial
import time
import matplotlib.pyplot as plt

# Öffne die serielle Verbindung
ser = serial.Serial('COM5', 921600, timeout=0.1)
time.sleep(2)  # Warten bis Arduino bereit ist

# Ergebnisse speichern
latenzen = []

print("Starte Test...")

for i in range(1000):
    # Sende T-Befehl (z.B. mit Ziel 0.0)
    string = "T0.000\n"
    ser.write(string.encode('utf-8'))

    start_time = time.perf_counter()

    # Warten auf Antwort
    while True:
        line = ser.readline().decode('utf-8').strip()
        print(line)
        if line.startswith("Latenz(us):"):
            latency_us = int(line.split(':')[1])
            latenzen.append(1/(latency_us/1000000.0))
            print(f"{i + 1}/1000: {latency_us/1000.0} ms")
            break

    # Minimal warten, bevor nächster Befehl kommt
    dl = 1/100.0
    time.sleep(dl)

ser.close()

# Plot anzeigen
plt.plot(latenzen, marker='o')
plt.xlabel('Befehl #')
plt.ylabel('Frequenz')
plt.title('Arduino Antwort-Latenz')
plt.grid(True)
plt.show()
