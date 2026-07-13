"""
Bridge example: stream a joint trajectory over the WebSocket for fast updates.

The HTTP routes (`move_j` / `set_j`) cost a full request/response per command.
When you want to servo an articulation at a high rate, open the WebSocket instead
and push one small frame per update:

  1. PUT /articulations {prim_path, urdf_path?} (HTTP) -> {articulation_id, ...}
  2. connect ws://host:port/articulations/{id}/stream_joint_positions
  3. send {"positions": [rad, ...]} frames at a fixed rate

The stream is fire-and-forget: the bridge writes the joint state directly on each
frame and sends nothing back, so the client is never blocked waiting on a reply.
Because each frame teleports the DOF state, a dense stream produces continuous
motion; if the stream pauses, the position drive settles toward its last target,
so keep streaming to hold a pose.

Run:  python robot_stream_joint_positions.py
      python robot_stream_joint_positions.py --prim /World/ur10e

Requires the ``requests`` and ``websockets`` packages
(``pip install requests websockets``).
"""

import argparse
import asyncio
import json

import numpy as np
import requests
import websockets

HOST = "127.0.0.1"
PORT = 8766
ROBOT_PRIM_PATH = "/World/ur10e"

# Trajectory endpoints (degrees for readability; converted to radians on the wire).
START_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
END_DEG = [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0]

# Stream a densely-sampled path so the teleported frames read as continuous motion.
NUM_POINTS = 200
RATE_HZ = 60.0

DEFAULT_TIMEOUT = 30.0


def create_articulation(base, prim_path):
    """PUT /articulations -> register/bind the robot and return its info."""
    response = requests.put(
        base.rstrip("/") + "/articulations",
        json={"prim_path": prim_path, "urdf_path": None},
        timeout=DEFAULT_TIMEOUT,
    )
    response.raise_for_status()
    return response.json()


async def stream_trajectory(ws_url, positions_rad):
    """Open the stream and push one joint-position frame per trajectory point."""
    period = 1.0 / RATE_HZ
    async with websockets.connect(ws_url) as websocket:
        for q in positions_rad:
            await websocket.send(json.dumps({"positions": q.tolist()}))
            await asyncio.sleep(period)


def main():
    parser = argparse.ArgumentParser(
        description="Stream a joint trajectory over the bridge WebSocket.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", default=ROBOT_PRIM_PATH,
                        help="prim path of the robot in the stage")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    info = create_articulation(base, args.prim)
    articulation_id = info["articulation_id"]
    print(f"created robot: articulation_id={articulation_id} prim_path={info['prim_path']}")
    print(f"  num_dof={info['num_dof']} dof_names={info['dof_names']}")

    positions_rad = np.deg2rad(np.linspace(START_DEG, END_DEG, num=NUM_POINTS))
    ws_url = f"ws://{args.host}:{args.port}/articulations/{articulation_id}/stream_joint_positions"
    print(f"streaming {NUM_POINTS} points at {RATE_HZ:.0f} Hz -> {ws_url}")
    asyncio.run(stream_trajectory(ws_url, positions_rad))
    print("stream complete")


if __name__ == "__main__":
    main()
