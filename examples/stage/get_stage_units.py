"""
Standalone bridge example: get the stage's meters-per-unit scale.

Flow (all over HTTP):
  1. GET /stage/units -> {"meters_per_unit": float}

Run:  python get_stage_units.py

Requires the ``requests`` package (``pip install requests``).
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
    """Print the stage's meters-per-unit scale."""
    parser = argparse.ArgumentParser(description="Get the stage's meters-per-unit scale.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    units = _request(base, "GET", "/stage/units")
    print(f"stage units: {units}")


if __name__ == "__main__":
    main()
