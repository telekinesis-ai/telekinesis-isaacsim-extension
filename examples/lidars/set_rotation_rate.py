"""
PUT /lidars/{id}/rotation_rate {value} -> {rotation_rate}

Sets the rotation rate (Hz); 0 means an instantaneous full-sweep lidar.

Run:  python set_rotation_rate.py --id lidar1 --value 20
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
    """Set one lidar's rotation rate."""
    parser = argparse.ArgumentParser(description="Set a lidar's rotation rate.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lidar_id", help="lidar_id from put_lidar.py"
    )
    parser.add_argument("--value", type=float, required=True, help="rotation rate in Hz")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "PUT", f"/lidars/{args.lidar_id}/rotation_rate", {"value": args.value}
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
