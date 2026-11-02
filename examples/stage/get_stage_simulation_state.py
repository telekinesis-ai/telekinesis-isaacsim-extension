"""
Standalone bridge example: read the current timeline state.

Flow (all over HTTP):
  1. GET /stage/simulation -> {"timeline": "playing" | "paused" | "stopped"}

Run:  python get_stage_simulation_state.py

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
    """Print the current simulation timeline state (playing/paused/stopped)."""
    parser = argparse.ArgumentParser(description="Read the current simulation timeline state.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    state = _request(base, "GET", "/stage/simulation")
    print(f"simulation state: {state}")


if __name__ == "__main__":
    main()
