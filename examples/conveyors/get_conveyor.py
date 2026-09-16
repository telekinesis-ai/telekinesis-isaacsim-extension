"""
GET /conveyors/{id} -> {conveyor_id, prim_path, belt_prim_path, drive,
direction, nominal_speed, velocity, running}

``velocity`` is the signed speed currently set on the belt, along the
travel direction its scene authored; ``running`` is whether that speed is
non-zero with the belt's drive switched on.

Requires the id to already be registered -- run put_conveyor.py first.

Run:  python get_conveyor.py --id conveyor1
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
    """Fetch one conveyor's description and current state by id."""
    parser = argparse.ArgumentParser(description="Read a conveyor's state.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="conveyor_id", help="conveyor_id from put_conveyor.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'GET', f'/conveyors/{args.conveyor_id}')}")


if __name__ == "__main__":
    main()
