"""
PUT /lidars/{id}/high_lod {value} -> {high_lod}

Sets whether the sensor renders at high level-of-detail.

Run:  python set_high_lod.py --id lidar1 --value true
      python set_high_lod.py --id lidar1 --value false
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


def _bool(text):
    """Parse an argparse bool flag from true/false/1/0 (argparse has no bool type)."""
    if text.lower() in ("1", "true", "yes"):
        return True
    if text.lower() in ("0", "false", "no"):
        return False
    raise argparse.ArgumentTypeError(f"expected true/false, got {text!r}")


def main():
    """Set whether one lidar renders at high level-of-detail."""
    parser = argparse.ArgumentParser(description="Set a lidar's high_lod flag.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lidar_id", help="lidar_id from put_lidar.py"
    )
    parser.add_argument("--value", type=_bool, required=True, help="true/false")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "PUT", f"/lidars/{args.lidar_id}/high_lod", {"value": args.value}
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
