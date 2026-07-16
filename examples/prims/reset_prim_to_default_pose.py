"""
Standalone bridge example: restore a prim to its stored default pose.

Flow (all over HTTP):
  1. PUT /prims/poses/default {prim_path} -> record the current pose as default
  2. POST /prims/poses/relative -> nudge the prim away from it
  3. POST /prims/poses/default/reset {prim_path} -> snap back to the default
  4. GET /prims/poses -> confirm it matches the original

Run:  python reset_prim_to_default_pose.py --prim /World/Cube

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
    """Record a default pose, nudge away from it, then reset back to it."""
    parser = argparse.ArgumentParser(description="Restore a prim to its stored default pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    params = {"prim_path": args.prim, "coordinate_system": "world", "rotation_type": "cartesian"}
    original_pose = _request(base, "GET", "/prims/poses", params=params)["pose"]
    print(f"pose (original): {original_pose}")

    # Record the current pose as the default, then nudge away from it.
    _request(base, "PUT", "/prims/poses/default", body={"prim_path": args.prim})
    _request(base, "POST", "/prims/poses/relative", body={
        "prim_path": args.prim,
        "relative_pose": {"pose": [0.05, 0, 0, 0, 0, 0]},
        "object_first": False,
    })
    print(f"pose (after nudge): {_request(base, 'GET', '/prims/poses', params=params)}")

    # Snap back to the stored default and confirm it matches the original.
    _request(base, "POST", "/prims/poses/default/reset", body={"prim_path": args.prim})
    restored_pose = _request(base, "GET", "/prims/poses", params=params)["pose"]
    print(f"pose (restored): {restored_pose}")
    print(f"matches original: {restored_pose == original_pose}")


if __name__ == "__main__":
    main()
