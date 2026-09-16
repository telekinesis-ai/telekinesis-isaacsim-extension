"""
DELETE /lightbeams/{id} -> {deleted: lightbeam_id}

Unregisters the sensor. The USD prim stays in the stage, so it can be
registered again with put_lightbeam.py.

Run:  python delete_lightbeam.py --id lightbeam1
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
    """Unregister one lightbeam sensor by id."""
    parser = argparse.ArgumentParser(description="Unregister a lightbeam sensor.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lightbeam_id", help="lightbeam_id from put_lightbeam.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'DELETE', f'/lightbeams/{args.lightbeam_id}')}")


if __name__ == "__main__":
    main()
