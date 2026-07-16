"""
POST /articulations/{id}/joint_efforts {joint_efforts, indices?}
-> {applied, joint_efforts, indices}

Direct torque/force drive that bypasses the position/velocity drive entirely
-- only takes effect if the target joint's drive stiffness and damping are
zero (or it has no drive at all). Requires the id to already be registered --
run put_articulation.py first to register a prim and get its articulation_id.

Run:  python set_joint_efforts.py --id articulation1

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import time

import requests

HOST = "127.0.0.1"
PORT = 8766
TARGET_EFFORT = 1.0
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Command a small effort on every driven joint, then zero it, by id."""
    parser = argparse.ArgumentParser(
        description="Drive an articulation's joints with a target effort."
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

    info = _request(base, "GET", f"/articulations/{args.articulation_id}")
    num_dof = info["num_dof"]

    efforts = [TARGET_EFFORT] * num_dof
    print(f"joint efforts: {efforts}")
    response = _request(
        base,
        "POST",
        f"/articulations/{args.articulation_id}/joint_efforts",
        {"joint_efforts": efforts},
    )
    print(f"response: {response}")

    print("holding for 2s...")
    time.sleep(2)

    print("zeroing effort")
    response = _request(
        base,
        "POST",
        f"/articulations/{args.articulation_id}/joint_efforts",
        {"joint_efforts": [0.0] * num_dof},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
