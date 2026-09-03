"""
POST /conveyors/{id}/stop -> {conveyor_id, velocity, running, ...}

Stops the belt. The running velocity the scene authored stays on the
stage, so start_conveyor.py without --velocity runs it at that speed
again.

Requires the id to already be registered -- run put_conveyor.py first.

Run:  python stop_conveyor.py --id conveyor1
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
    """Stop one conveyor belt."""
    parser = argparse.ArgumentParser(description="Stop a conveyor belt.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="conveyor_id", help="conveyor_id from put_conveyor.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'POST', f'/conveyors/{args.conveyor_id}/stop')}")


if __name__ == "__main__":
    main()
