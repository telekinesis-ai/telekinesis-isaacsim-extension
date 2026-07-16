"""
Standalone bridge example: drive the simulation timeline (play/pause/stop).

Flow (all over HTTP):
  1. PATCH /stage/simulation/timeline/{action} -> play, pause, or stop
  2. GET /stage/simulation -> current timeline state

Run:  python stage_timeline_control.py

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
    """Drive the simulation timeline through play, pause, and stop."""
    parser = argparse.ArgumentParser(
        description="Drive the simulation timeline: play / pause / stop."
    )
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"sim (initial):      {_request(base, 'GET', '/stage/simulation')}")

    _request(base, "PATCH", "/stage/simulation/timeline/play")
    print(f"sim (after play):   {_request(base, 'GET', '/stage/simulation')}")

    _request(base, "PATCH", "/stage/simulation/timeline/pause")
    print(f"sim (after pause):  {_request(base, 'GET', '/stage/simulation')}")

    _request(base, "PATCH", "/stage/simulation/timeline/stop")
    print(f"sim (after stop):   {_request(base, 'GET', '/stage/simulation')}")


if __name__ == "__main__":
    main()
