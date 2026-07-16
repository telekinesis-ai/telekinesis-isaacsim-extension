"""
Standalone bridge example: set the stage's meters-per-unit scale.

Flow (all over HTTP):
  1. GET /stage/units -> read the current scale (so we can restore it)
  2. PUT /stage/units {meters_per_unit} -> set a new scale
  3. GET /stage/units -> confirm, then restore the original

Run:  python update_stage_units.py --meters_per_unit 1.0

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
    """Set the stage's meters-per-unit scale, then restore the original value."""
    parser = argparse.ArgumentParser(description="Set the stage's meters-per-unit scale.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--meters_per_unit", type=float, default=1.0)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"

    # Read the current scale first so we can restore it afterwards.
    original = _request(base, "GET", "/stage/units")["meters_per_unit"]
    print(f"units (original): {original}")

    _request(base, "PUT", "/stage/units", {"meters_per_unit": args.meters_per_unit})
    print(f"units (set):      {_request(base, 'GET', '/stage/units')}")

    _request(base, "PUT", "/stage/units", {"meters_per_unit": original})
    print(f"units (restored): {_request(base, 'GET', '/stage/units')}")


if __name__ == "__main__":
    main()
