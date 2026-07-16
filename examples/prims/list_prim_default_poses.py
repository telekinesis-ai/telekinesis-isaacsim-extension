"""
Standalone bridge example: list all stored default poses.

Flow (all over HTTP):
  1. GET /prims/poses/default -> {prim_path: {"pose": [...]}, ...}

Run:  python list_prim_default_poses.py

Requires the ``requests`` package (``pip install requests``).
"""
import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, params=None, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(
        method, base.rstrip("/") + path, params=params, json=body, timeout=DEFAULT_TIMEOUT,
    )
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Print every prim that currently has a stored default pose."""
    parser = argparse.ArgumentParser(description="List every prim's stored default pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"default poses: {_request(base, 'GET', '/prims/poses/default')}")


if __name__ == "__main__":
    main()
