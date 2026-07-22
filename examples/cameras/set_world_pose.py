"""
PUT /cameras/{id}/world_pose {position?, orientation?, camera_axes?} ->
{position, orientation}

Moves the camera in the world frame. ``orientation`` is a scalar-first
(w, x, y, z) quaternion. ``camera_axes`` is world (default), ros or usd.

Run:  python set_world_pose.py --id camera1 --pos 2 2 2
      python set_world_pose.py --id camera1 --pos 2 2 2 --quat 0.707 0 0.707 0 --axes ros
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Set one camera's world pose and print the resulting pose."""
    parser = argparse.ArgumentParser(description="Set a camera's world pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    parser.add_argument("--pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"))
    parser.add_argument("--quat", type=float, nargs=4, default=None, metavar=("W", "X", "Y", "Z"))
    parser.add_argument("--axes", default="world", choices=["world", "ros", "usd"])
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        f"/cameras/{args.camera_id}/world_pose",
        {"position": args.pos, "orientation": args.quat, "camera_axes": args.axes},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
