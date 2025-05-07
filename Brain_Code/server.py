import asyncio
import random
import json
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from starlette.responses import RedirectResponse

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/")
async def read_index():
    return RedirectResponse(url="/static/client.html")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # 8 simulierte Sensorwerte
            sensor_values = [round(random.uniform(0, 50), 2) for _ in range(8)]
            await websocket.send_text(json.dumps(sensor_values))
            await asyncio.sleep(0.02)  # 50 Hz
    except Exception as e:
        print(f"Verbindung geschlossen: {e}")