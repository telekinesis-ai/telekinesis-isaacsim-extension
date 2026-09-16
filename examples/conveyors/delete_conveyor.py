"""
DELETE /conveyors/{id} -> {deleted: conveyor_id}

Unregisters the conveyor. The USD prim stays in the stage, and a belt
that is running keeps running -- run stop_conveyor.py first if it
should not.

Run:  python delete_conveyor.py --id conveyor1
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
    """Unregister one conveyor by id."""
    parser = argparse.ArgumentParser(description="Unregister a conveyor.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="conveyor_id", help="conveyor_id from put_conveyor.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'DELETE', f'/conveyors/{args.conveyor_id}')}")


if __name__ == "__main__":
    main()
