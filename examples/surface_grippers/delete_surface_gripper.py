"""
DELETE /surface_grippers/{id} -> {deleted: id}

Unregisters the gripper. The USD prim stays in the stage -- only the bridge's
binding to it goes away, so the id stops working and PUTting the same prim path
again hands back a fresh one. Any record that this gripper was assembled onto an
arm is dropped too, so the pair can be assembled again after re-creating it.

Run:  python delete_surface_gripper.py --id surface_gripper1
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
    """Unregister one surface gripper."""
    parser = argparse.ArgumentParser(description="Delete (unregister) a surface gripper.")
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
    response = _request(base, "DELETE", f"/surface_grippers/{args.surface_gripper_id}")
    print(f"response: {response}")


if __name__ == "__main__":
    main()
