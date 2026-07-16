"""
Standalone bridge example: set a prim's world pose.

Flow (all over HTTP):
  1. GET /prims/poses?prim_path=...&coordinate_system=world -> read the current pose
  2. PUT /prims/poses {prim_path, input_pose} -> nudge it +1cm on X
  3. GET /prims/poses -> confirm

Run:  python update_prim_pose.py --prim /World/Cube

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
    """Nudge a prim's world pose +1cm on X, then confirm the new pose."""
    parser = argparse.ArgumentParser(description="Set a prim's world pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    params = {"prim_path": args.prim, "coordinate_system": "world", "rotation_type": "cartesian"}

    # Read the current pose first so we know what "+1cm" is relative to.
    original_pose = _request(base, "GET", "/prims/poses", params=params)["pose"]
    print(f"pose (original): {original_pose}")

    nudged_pose = list(original_pose)
    nudged_pose[0] += 0.01
    _request(
        base,
        "PUT",
        "/prims/poses",
        body={"prim_path": args.prim, "input_pose": {"pose": nudged_pose}},
    )
    print(f"pose (after +1cm X): {_request(base, 'GET', '/prims/poses', params=params)}")


if __name__ == "__main__":
    main()
