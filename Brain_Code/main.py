import time

from click import pause

from imu import IMU
from motor import MOTOR
from flask import Flask, request, jsonify
import json
import logging

IMU_TASK = IMU(debug=False, freq=50)
IMU_TASK.start()

MOTOR_LEFT_TASK = MOTOR(debug=False, freq=50, side="left", port="/dev/ttyACM0")
MOTOR_LEFT_TASK.start()
# MOTOR_RIGHT_TASK = MOTOR(debug=False, freq=50, side="right")
# MOTOR_RIGHT_TASK.start()

app = Flask(__name__)
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

data_json = {
    "motor_left": {
        "pv": None,
        "sp": {
            "value": 0.0,
            "en": False
        }
    },
    "motor_right": {
        "pv": None,
        "sp": {
            "value": 0.0,
            "en": False
        }
    },
    "imu": {
        "pv": None
    },
    "lqr": {
        "sp": None,
        "config": None,
    }
}

def data_handler(name,valuename):
    """Generiert eine Flask-Route."""
    global data_json
    if request.method == 'POST':
        data = request.get_json()
        if data:
            data_json[name][valuename] = data
            #print(json.dumps(data_json, indent=4))
            return jsonify({"message": f"Daten für {name}/{valuename} erfolgreich gespeichert"}), 201
        else:
            return jsonify({"error": "Keine gültigen JSON-Daten erhalten"}), 400
    elif request.method == 'GET':
        data = data_json[name][valuename]
        if data:
            return jsonify(data), 200
        else:
            return jsonify({"message": f"Keine Daten für {name}/{valuename} vorhanden"}), 404

@app.route('/imu/pv', methods=['GET', 'POST'])
def imu():
    return data_handler("imu","pv")

@app.route('/motor_left/pv', methods=['GET', 'POST'])
def motor_left_pv():
    return data_handler("motor_left","pv")

@app.route('/motor_left/sp', methods=['GET', 'POST'])
def motor_left_sp():
    return data_handler("motor_left","sp")

@app.route('/motor_right/pv', methods=['GET', 'POST'])
def motor_right_pv():
    return data_handler("motor_right","pv")

@app.route('/motor_right/sp', methods=['GET', 'POST'])
def motor_right_sp():
    return data_handler("motor_right","sp")

@app.route('/lqr/sp', methods=['GET', 'POST'])
def lqr_sp():
    return data_handler("lqr","sp")

@app.route('/lqr/config', methods=['GET', 'POST'])
def lqr_sp():
    return data_handler("lqr","config")


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)