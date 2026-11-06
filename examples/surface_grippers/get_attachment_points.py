"""
GET /surface_grippers/{id}/attachment_points -> {attachment_points: [...]}

One entry per suction cup: the D6 joint it grips with, the two bodies that joint
connects and its local frame on each, the Z axis translation drive that pulls a
gripped object in, the rotation (degrees) and translation (meters) limits that let
it swivel and slide, and the clearance offset and forward axis that aim the grip.

``body_1`` is what the joint is parked against while the gripper is open; during a
grip it reads back as the object being held.

Run:  python get_attachment_points.py --id surface_gripper1
"""

import argparse

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
    """Print one surface gripper's attachment points."""
    parser = argparse.ArgumentParser(description="Get a surface gripper's attachment points.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id",
        required=True,
        dest="surface_gripper_id",
        help="surface_gripper_id from put_surface_gripper.py",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "GET", f"/surface_grippers/{args.surface_gripper_id}/attachment_points"
    )
    for point in response["attachment_points"]:
        print(f"\n{point['prim_path']}")
        for key, value in point.items():
            if key != "prim_path":
                print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
