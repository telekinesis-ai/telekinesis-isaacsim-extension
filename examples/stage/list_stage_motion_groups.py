"""
Standalone bridge example: list articulation root prims in the stage.

Flow (all over HTTP):
  1. GET /stage/motion-groups -> prim paths of every articulation root (potential robots)

Run:  python list_stage_motion_groups.py

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
    """List every articulation root prim currently in the stage."""
    parser = argparse.ArgumentParser(description="List articulation root prims in the stage.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    motion_groups = _request(base, "GET", "/stage/motion-groups")
    print(f"motion groups: {motion_groups}")


if __name__ == "__main__":
    main()
