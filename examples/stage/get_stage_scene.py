"""
Standalone bridge example: get the URI of the open USD stage.

Flow (all over HTTP):
  1. GET /stage/scene -> URI of the currently open stage (empty string if none)

Run:  python get_stage_scene.py

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
    """Print the URI of the currently open USD stage."""
    parser = argparse.ArgumentParser(description="Get the URI of the open USD stage.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    scene = _request(base, "GET", "/stage/scene")
    print(f"active scene: {scene}")


if __name__ == "__main__":
    main()
