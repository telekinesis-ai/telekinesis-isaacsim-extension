"""
Bridge example: receive the articulation's state over the WebSocket as it changes.

The HTTP getters (`joints_state` / `articulation_state`) cost a full
request/response per sample, and each one is served between simulator frames. To
follow an articulation continuously, open the state stream instead and read the
frames the bridge pushes:

  1. PUT /articulations {prim_path, urdf_path?} (HTTP) -> {articulation_id, ...}
  2. connect ws://host:port/articulations/{id}/stream_articulation_state
  3. read one frame per simulator update

Each frame is the same JSON object the `articulation_state` getter returns: joint
positions, velocities and efforts (radians), applied efforts, measured joint
forces, the last applied action, and the root link's world pose and velocity.
Nothing is sent while the timeline is stopped, so press Play before running this.

Run:  python robot_stream_articulation_state.py
      python robot_stream_articulation_state.py --prim /World/ur10e

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

# How many frames to print before disconnecting.
NUM_FRAMES = 60

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


async def print_state_frames(ws_url):
    """Open the state stream and print the first NUM_FRAMES frames."""
    async with websockets.connect(ws_url) as websocket:
        for frame_index in range(NUM_FRAMES):
            state = json.loads(await websocket.recv())
            positions_deg = np.rad2deg(state["joint_positions"])
            print(
                f"frame {frame_index:3d}  t={state['timestamp']:8.3f}s  "
                f"q(deg)={np.round(positions_deg, 2).tolist()}"
            )


def main():
    """Parse CLI args and run the example."""
    parser = argparse.ArgumentParser(
        description="Read an articulation's state from the bridge WebSocket."
    )
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--prim", default=ROBOT_PRIM_PATH, help="prim path of the robot in the stage"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    info = create_articulation(base, args.prim)
    articulation_id = info["articulation_id"]
    print(f"created robot: articulation_id={articulation_id} prim_path={info['prim_path']}")
    print(f"  num_dof={info['num_dof']} dof_names={info['dof_names']}")

    ws_url = (
        f"ws://{args.host}:{args.port}"
        f"/articulations/{articulation_id}/stream_articulation_state"
    )
    print(f"reading {NUM_FRAMES} frames <- {ws_url}")
    asyncio.run(print_state_frames(ws_url))
    print("stream complete")


if __name__ == "__main__":
    main()
