import time
import zmq
import json

port = "5556"
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.101:%s" % port)

sp = {"sp": {
        "p":0.0,
        "x":0.0,
        "pv": 0.0,
        "v": -3.0,
        "yaw": 0.0}}

def fetch_data():
    """Holt die Daten von der API und speichert sie."""
    try:
        for i in range(5):
            socket.send_string(json.dumps(sp))
            rcv = json.loads(socket.recv_string())
            time.sleep(0.2)

    except Exception as e:
        print("Fehler bei der Kommunikation:", e)


fetch_data()