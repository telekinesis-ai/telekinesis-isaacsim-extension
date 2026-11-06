"""
Standalone bridge example: store user metadata on a prim.

Flow (all over HTTP):
  1. PUT /prims/metadata {prim_path, metadata: {category, type}} -> stores it under customData

Run:  python set_prim_metadata.py --prim /World/Cube --category fixture --type table

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
        method,
        base.rstrip("/") + path,
        params=params,
        json=body,
        timeout=DEFAULT_TIMEOUT,
    )
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Store {category, type} metadata on a prim under customData."""
    parser = argparse.ArgumentParser(description="Store {category, type} metadata on a prim.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    parser.add_argument("--category", default="test_category")
    parser.add_argument("--type", dest="type_", default="test_type")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    _request(
        base,
        "PUT",
        "/prims/metadata",
        body={
            "prim_path": args.prim,
            "metadata": {"category": args.category, "type": args.type_},
        },
    )
    print(f"metadata set on {args.prim}: category={args.category} type={args.type_}")


if __name__ == "__main__":
    main()
