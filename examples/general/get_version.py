"""
Standalone bridge example: list installed extension versions.

Flow (all over HTTP):
  1. GET /version -> {extension_name: version, ...} for installed (enabled) extensions

Run:  python get_version.py

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
    """List the installed (enabled) extensions and their versions."""
    parser = argparse.ArgumentParser(description="List installed extension versions.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"versions: {_request(base, 'GET', '/version')}")


if __name__ == "__main__":
    main()
