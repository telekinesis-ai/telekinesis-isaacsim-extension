"""
GET /lidars/{id}/is_lidar_sensor -> {is_lidar_sensor}

Whether the registered prim currently resolves to a live PhysX lidar sensor.

Run:  python is_lidar_sensor.py --id lidar1
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
    """Check whether one lidar's prim is a live PhysX lidar sensor."""
    parser = argparse.ArgumentParser(description="Check a lidar's is_lidar_sensor flag.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lidar_id", help="lidar_id from put_lidar.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'GET', f'/lidars/{args.lidar_id}/is_lidar_sensor')}")


if __name__ == "__main__":
    main()
