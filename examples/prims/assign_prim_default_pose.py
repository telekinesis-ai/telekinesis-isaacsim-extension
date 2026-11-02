"""
Standalone bridge example: record a prim's current pose as its default.

Flow (all over HTTP):
  1. PUT /prims/poses/default {prim_path} -> records the prim's current local pose
  2. GET /prims/poses/default -> confirms it was stored

Run:  python assign_prim_default_pose.py --prim /World/Cube

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
        method,
        base.rstrip("/") + path,
        params=params,
        json=body,
        timeout=DEFAULT_TIMEOUT,
    )
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Record a prim's current pose as its default, then confirm it was stored."""
    parser = argparse.ArgumentParser(description="Record a prim's current pose as its default.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    _request(base, "PUT", "/prims/poses/default", body={"prim_path": args.prim})
    print(f"default poses (after assign): {_request(base, 'GET', '/prims/poses/default')}")


if __name__ == "__main__":
    main()
