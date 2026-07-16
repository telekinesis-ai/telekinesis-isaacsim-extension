"""
POST /articulations/{id}/joint_velocities {joint_velocities}  (rad/s)
-> fire-and-forget; the joints hold that velocity until the next call.

Requires the id to already be registered -- run put_articulation.py first to
register a prim and get its articulation_id.

Run:  python set_joint_velocities.py --id articulation1

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import time

import requests

HOST = "127.0.0.1"
PORT = 8766
TARGET_VELOCITY_RAD_S = 0.2
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Drive one articulation's joints at a fixed velocity, then stop, by id."""
    parser = argparse.ArgumentParser(
        description="Drive an articulation's joints at a target velocity."
    )
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id",
        required=True,
        dest="articulation_id",
        help="articulation_id from a prior PUT /articulations (see put_articulation.py)",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"

    # Look up how many joints this articulation drives, to size the velocity array.
    info = _request(base, "GET", f"/articulations/{args.articulation_id}")
    num_dof = info["num_dof"]

    # Drive every joint at the same target velocity (fire-and-forget).
    velocities = [TARGET_VELOCITY_RAD_S] * num_dof
    print(f"joint velocities (rad/s): {velocities}")
    response = _request(
        base,
        "POST",
        f"/articulations/{args.articulation_id}/joint_velocities",
        {"joint_velocities": velocities},
    )
    print(f"response: {response}")

    print("holding velocity for 2s...")
    time.sleep(2)

    # Zero velocity stops the joints (they hold the last command otherwise).
    print("stopping (velocities=0)")
    response = _request(
        base,
        "POST",
        f"/articulations/{args.articulation_id}/joint_velocities",
        {"joint_velocities": [0.0] * num_dof},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
