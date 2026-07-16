"""
Standalone bridge example: remove user metadata previously stored on a prim.

Flow (all over HTTP):
  1. PUT /prims/metadata {prim_path, metadata} -> store something to remove
  2. DELETE /prims/metadata?prim_path=... -> removes it

Run:  python remove_prim_metadata.py --prim /World/Cube

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
    """Store metadata on a prim, then remove it."""
    parser = argparse.ArgumentParser(description="Remove metadata previously stored on a prim.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"

    # Store something first, so there's metadata to remove.
    _request(base, "PUT", "/prims/metadata", body={
        "prim_path": args.prim, "metadata": {"category": "test_category", "type": "test_type"},
    })
    print(f"metadata set on {args.prim}")

    _request(base, "DELETE", "/prims/metadata", params={"prim_path": args.prim})
    print(f"metadata removed from {args.prim}")


if __name__ == "__main__":
    main()
