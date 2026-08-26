"""
PUT /lidars/{id}/local_pose {translation?, orientation?} -> {translation, orientation}

Moves the lidar relative to its parent prim. ``orientation`` is a scalar-first
(w, x, y, z) quaternion.

Run:  python set_local_pose.py --id lidar1 --pos 0.1 0 0.2
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
    """Set one lidar's local pose and print the resulting pose."""
    parser = argparse.ArgumentParser(description="Set a lidar's local pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lidar_id", help="lidar_id from put_lidar.py"
    )
    parser.add_argument("--pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"))
    parser.add_argument("--quat", type=float, nargs=4, default=None, metavar=("W", "X", "Y", "Z"))
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        f"/lidars/{args.lidar_id}/local_pose",
        {"translation": args.pos, "orientation": args.quat},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
