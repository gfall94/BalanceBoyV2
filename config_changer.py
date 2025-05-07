import time
import zmq
import json

port = "5556"
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.101:%s" % port)

# TODO
# motoren von 0-7,5 auf 0-100 geändert
pid = {
    "yaw": {
        "Kp": 25.0,
        "Ki": 100.0,
        "Kd": 2.0
    },
    "lqr": {
            "Q": [100, 15, 50, 25],
            "R": 1.0
            # "Q": [75, 15, 25, 5],
            # "R": 1.0
        },
    "kalman": {
            "Q": [50, 50, 25, 25], # Prozessrauschen (Modell)
            "R": [5e-2, 5e-2, 0.5, 0.5] # Messrauschen (Sensoren)
        }
}

def fetch_data():
    """Holt die Daten von der API und speichert sie."""
    try:
        for i in range(5):
            socket.send_string(json.dumps(pid))
            rcv = json.loads(socket.recv_string())
            time.sleep(0.2)

    except Exception as e:
        print("Fehler bei der Kommunikation:", e)


fetch_data()