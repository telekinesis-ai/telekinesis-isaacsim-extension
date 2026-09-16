"""
GET /lightbeams -> {lightbeam_id: prim_path, ...}

Every lightbeam sensor registered with the bridge, mapped to the prim it
was registered against. Empty until put_lightbeam.py has run.

Run:  python get_lightbeams_list.py
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
    """List every registered lightbeam sensor."""
    parser = argparse.ArgumentParser(description="List the registered lightbeam sensors.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'GET', '/lightbeams')}")


if __name__ == "__main__":
    main()
