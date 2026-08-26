"""
PUT /lidars/{id}/vertical_fov {value} -> {vertical_fov}

Sets the vertical field of view (degrees).

Run:  python set_vertical_fov.py --id lidar1 --value 30
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
    """Set one lidar's vertical field of view."""
    parser = argparse.ArgumentParser(description="Set a lidar's vertical fov.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lidar_id", help="lidar_id from put_lidar.py"
    )
    parser.add_argument("--value", type=float, required=True, help="vertical fov in degrees")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "PUT", f"/lidars/{args.lidar_id}/vertical_fov", {"value": args.value}
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
