"""
Standalone bridge example: show or hide a prim.

Flow (all over HTTP):
  1. PATCH /prims/visibility {prim_path, visibility: "hide"} -> hides the prim
  2. PATCH /prims/visibility {prim_path, visibility: "show"} -> shows it again

Run:  python set_prim_visibility.py --prim /World/Cube

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import time

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, params=None, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(
        method,
        base.rstrip("/") + path,
        params=params,
        json=body,
        timeout=DEFAULT_TIMEOUT,
    )
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Hide a prim, wait a beat, then show it again."""
    parser = argparse.ArgumentParser(description="Show or hide a prim.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    _request(
        base, "PATCH", "/prims/visibility", body={"prim_path": args.prim, "visibility": "hide"}
    )
    print(f"{args.prim} hidden")
    time.sleep(1)

    _request(
        base, "PATCH", "/prims/visibility", body={"prim_path": args.prim, "visibility": "show"}
    )
    print(f"{args.prim} shown")


if __name__ == "__main__":
    main()
