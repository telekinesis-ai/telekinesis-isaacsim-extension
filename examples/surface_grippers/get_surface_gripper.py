"""
GET /surface_grippers/{id} ->
{surface_gripper_id, prim_path, gripper_prim_path, attachment_point_paths,
 properties, status, gripped_objects, grip_distance, simulated}

Everything the bridge knows about one registered suction gripper. 404 if the id
was never registered (or was dropped when the stage changed).

Run:  python get_surface_gripper.py --id surface_gripper1
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
    """Print one registered surface gripper's description."""
    parser = argparse.ArgumentParser(description="Get one surface gripper.")
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
    response = _request(base, "GET", f"/surface_grippers/{args.surface_gripper_id}")
    for key, value in response.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
