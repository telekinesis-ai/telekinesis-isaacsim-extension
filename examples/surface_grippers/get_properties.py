"""
GET /surface_grippers/{id}/properties ->
{coaxial_force_limit, shear_force_limit, max_grip_distance, retry_interval,
 forward_axis}

The gripper's grip behaviour: the loads (newtons) that break a grip along and
across the forward axis, how far (meters) it reaches for a surface, how long
(seconds) a close keeps retrying, and which axis it grips along. A property the
asset leaves unauthored reads back as null.

The per-cup rotation and translation limits live on the attachment points; see
get_attachment_points.py.

Run:  python get_properties.py --id surface_gripper1
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
    """Print one surface gripper's grip-behaviour properties."""
    parser = argparse.ArgumentParser(description="Get a surface gripper's properties.")
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
    response = _request(base, "GET", f"/surface_grippers/{args.surface_gripper_id}/properties")
    for key, value in response.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
