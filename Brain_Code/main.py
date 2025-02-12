import asyncio
import websockets
import time
from imu import IMU

IMU_SENSOR = IMU(debug=False)
IMU_SENSOR.start()

async def send_data(websocket, path):
    """ Sendet IMU-Daten an den Client """
    try:
        while True:
            quat_i, quat_j, quat_k, quat_real = IMU_SENSOR.bno.geomagnetic_quaternion
            message = f"{quat_i},{quat_j},{quat_k},{quat_real}"
            await websocket.send(message)
            await asyncio.sleep(0.05)  # 20 Hz
    except websockets.exceptions.ConnectionClosed:
        print("Verbindung geschlossen")

async def main():
    server = await websockets.serve(send_data, "0.0.0.0", 5005)
    print("WebSocket-Server läuft auf Port 5005")
    await server.wait_closed()

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Beenden...")
    IMU_SENSOR.stop()
