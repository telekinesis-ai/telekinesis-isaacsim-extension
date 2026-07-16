"""
Standalone bridge example: enable or disable collision on a prim.

Flow (all over HTTP):
  1. PATCH /prims/physics/colliders/ {prim_path, enable: false} -> disables collision
  2. PATCH /prims/physics/colliders/ {prim_path, enable: true}  -> re-enables it

Run:  python update_prim_colliders.py --prim /World/Cube

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
    """Disable collision on a prim, then re-enable it."""
    parser = argparse.ArgumentParser(description="Enable or disable collision on a prim.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path, e.g. /World/Cube")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    _request(
        base, "PATCH", "/prims/physics/colliders/", body={"prim_path": args.prim, "enable": False})
    print(f"collider disabled: {args.prim}")

    _request(
        base, "PATCH", "/prims/physics/colliders/", body={"prim_path": args.prim, "enable": True})
    print(f"collider enabled: {args.prim}")


if __name__ == "__main__":
    main()
