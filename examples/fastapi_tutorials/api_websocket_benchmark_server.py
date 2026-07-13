"""FastAPI server exposing both a REST endpoint and a WebSocket endpoint for joint position updates.

Run with:
    python api_websocket_benchmark_server.py
"""

import time

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

app = FastAPI()


@app.post("/joint_positions")
async def set_joint_positions(request: dict) -> dict:
    server_receive_time = time.time()
    server_send_time = time.time()
    return {
        "positions": request["positions"],
        "client_send_time": request["client_send_time"],
        "server_receive_time": server_receive_time,
        "server_send_time": server_send_time,
    }


@app.websocket("/ws/joint_positions")
async def websocket_joint_positions(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            request = await websocket.receive_json()
            server_receive_time = time.time()
            server_send_time = time.time()
            await websocket.send_json(
                {
                    "positions": request["positions"],
                    "client_send_time": request["client_send_time"],
                    "server_receive_time": server_receive_time,
                    "server_send_time": server_send_time,
                }
            )
    except WebSocketDisconnect:
        pass


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
