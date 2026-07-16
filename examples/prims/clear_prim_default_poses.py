"""
Standalone bridge example: forget every stored default pose.

Flow (all over HTTP):
  1. PUT /prims/poses/default {prim_path} -> record one, so there's something to clear
  2. DELETE /prims/poses/default -> forgets every stored default pose
  3. GET /prims/poses/default -> confirms it's empty

Run:  python clear_prim_default_poses.py --prim /World/Cube

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
    """Record one default pose, then forget every stored default pose."""
    parser = argparse.ArgumentParser(description="Forget every stored default pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"

    # Record one, so there's something to clear.
    _request(base, "PUT", "/prims/poses/default", body={"prim_path": args.prim})
    print(f"default poses (before clear): {_request(base, 'GET', '/prims/poses/default')}")

    _request(base, "DELETE", "/prims/poses/default")
    print(f"default poses (after clear):  {_request(base, 'GET', '/prims/poses/default')}")


if __name__ == "__main__":
    main()
