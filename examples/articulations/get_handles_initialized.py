"""
GET /articulations/{id}/handles_initialized -> {handles_initialized}

Whether the device's handle is currently valid, without paying for a full
re-bind. Requires the id to already be registered -- run put_articulation.py
first to register a prim and get its articulation_id.

Run:  python get_handles_initialized.py --id articulation1
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
    """Check whether one articulation's handle is currently valid."""
    parser = argparse.ArgumentParser(description="Check whether an articulation's handle is valid.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="articulation_id",
        help="articulation_id from a prior PUT /articulations (see put_articulation.py)")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(base, "GET", f"/articulations/{args.articulation_id}/handles_initialized")
    print(f"response: {response}")


if __name__ == "__main__":
    main()
