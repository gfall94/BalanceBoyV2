import time
from imu import IMU
from flask import Flask, request, jsonify
import json

IMU_SENSOR = IMU(debug=False, freq=50)
IMU_SENSOR.start()

app = Flask(__name__)

imu_json = {}

@app.route('/imu', methods=['GET', 'POST'])
def imu():
    global imu_json
    if request.method == 'POST':
        # Daten aus der POST-Anfrage extrahieren
        data = request.get_json()
        if data:
            # Speichern der Daten in der globalen Variable
            imu_json = data
            return jsonify({"message": "Daten erfolgreich gespeichert"}), 201
        else:
            return jsonify({"error": "Keine gültigen JSON-Daten erhalten"}), 400
    elif request.method == 'GET':
        # Rückgabe der gespeicherten Daten
        data = imu_json
        if data:
            return imu_json, 200
        else:
            return jsonify({"message": "Keine Daten vorhanden"}), 404


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)