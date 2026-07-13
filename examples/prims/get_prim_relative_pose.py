"""
Standalone bridge example: get the pose of one prim expressed in another's frame.

Flow (all over HTTP):
  1. GET /prims/poses/relative?prim_path_1=...&prim_path_2=... -> pose of prim 2 in prim 1's frame

Run:  python get_prim_relative_pose.py --prim1 /World/Cube --prim2 /World/Sphere

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
    """Print the pose of ``--prim2`` expressed in ``--prim1``'s frame."""
    parser = argparse.ArgumentParser(
        description="Get the pose of prim 2 expressed in prim 1's frame.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim1", required=True, help="reference prim path")
    parser.add_argument("--prim2", required=True, help="target prim path")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    pose = _request(base, "GET", "/prims/poses/relative", params={
        "prim_path_1": args.prim1, "prim_path_2": args.prim2,
        "mode": "normal", "rotation_type": "cartesian",
    })
    print(f"relative pose: {pose}")


if __name__ == "__main__":
    main()
