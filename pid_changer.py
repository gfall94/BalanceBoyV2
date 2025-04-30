import time
import zmq
import json

port = "5556"
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.101:%s" % port)

pid = {
    "Kp": [-2.0, 0.0, 0.0, 0.0],
    "Ki": [-0.0, 0.0, 0.0, 0.0],
    "Kd": [-0.0, 0.0, 0.0, 0.0]
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