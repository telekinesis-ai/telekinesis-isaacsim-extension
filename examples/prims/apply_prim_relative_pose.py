"""
Standalone bridge example: nudge a prim by a relative pose.

Flow (all over HTTP):
  1. GET /prims/poses -> read the current world pose
  2. POST /prims/poses/relative {prim_path, relative_pose} -> pre-multiply by a small offset
  3. GET /prims/poses -> confirm the nudge

Run:  python apply_prim_relative_pose.py --prim /World/Cube

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
    """Pre-multiply a prim's world pose by a small relative offset."""
    parser = argparse.ArgumentParser(description="Nudge a prim by a relative pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    params = {"prim_path": args.prim, "coordinate_system": "world", "rotation_type": "cartesian"}
    print(f"pose (before): {_request(base, 'GET', '/prims/poses', params=params)}")

    # Apply a +1cm X offset relative to the prim's current pose.
    _request(base, "POST", "/prims/poses/relative", body={
        "prim_path": args.prim,
        "relative_pose": {"pose": [0.01, 0, 0, 0, 0, 0]},
        "object_first": False,
    })
    print(f"pose (after +1cm X nudge): {_request(base, 'GET', '/prims/poses', params=params)}")


if __name__ == "__main__":
    main()
