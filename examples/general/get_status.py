"""
Standalone bridge example: liveness check.

Flow (all over HTTP):
  1. GET /status -> {"status": "OK"} while the bridge is running

When to use: before anything else, to isolate "the bridge never started"
(extension not enabled, Isaac Sim not running) from "the bridge is up but
this specific call failed" (bad prim path, stage not ready, etc). It takes
no arguments and works regardless of whether a stage is open or an
articulation is registered, so it's the cheapest possible first check when
another script raises a connection error or an unexpected response.

Run:  python get_status.py

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
    """Check whether the bridge is up and responding."""
    parser = argparse.ArgumentParser(description="Check bridge liveness.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"status: {_request(base, 'GET', '/status')}")


if __name__ == "__main__":
    main()
