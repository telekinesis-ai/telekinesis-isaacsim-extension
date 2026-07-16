"""
Standalone bridge example: get a prim's world pose.

Flow (all over HTTP):
  1. GET /prims/poses?prim_path=...&coordinate_system=world -> {"pose": [x,y,z,rx,ry,rz]}

Run:  python get_prim_pose.py --prim /World/Cube

Requires the ``requests`` package (``pip install requests``).
"""
import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, params=None, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(
        method, base.rstrip("/") + path, params=params, json=body, timeout=DEFAULT_TIMEOUT,
    )
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Print a prim's current world-space pose."""
    parser = argparse.ArgumentParser(description="Get a prim's world pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    pose = _request(base, "GET", "/prims/poses", params={
        "prim_path": args.prim, "coordinate_system": "world", "rotation_type": "cartesian",
    })
    print(f"pose: {pose}")


if __name__ == "__main__":
    main()
