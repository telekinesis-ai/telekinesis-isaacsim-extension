"""
POST /articulations/{id}/set_j {joint_positions}  (radians)
-> teleports the joints directly to the target; returns immediately, no blocking.

This differs from move_j (see robot_set_joint_position.py): set_j snaps the arm
to the pose in a single step instead of driving it there over time.

Requires the id to already be registered -- run put_articulation.py first to
register a prim and get its articulation_id.

Run:  python teleport_joint_positions.py --id articulation1

Requires the ``requests`` and ``numpy`` packages.
"""

import argparse

import numpy as np
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
    """Teleport one articulation's joints straight to a target, by id."""
    parser = argparse.ArgumentParser(description="Teleport an articulation's joints (set_j).")
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

    # Teleport: snap the joints to the target in one step (no driving/settling).
    target_joint_positions = [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0]
    print(f"Target joint positions (deg): {target_joint_positions}")
    response = _request(
        base,
        "POST",
        f"/articulations/{args.articulation_id}/set_j",
        {"joint_positions": np.deg2rad(target_joint_positions).tolist()},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
