"""
Standalone bridge example: open a USD stage.

Flow (all over HTTP):
  1. PUT /stage/scene {uri} -> opens the stage at ``uri`` (replaces the current stage)
  2. GET /stage/scene -> confirms the new URI

Run:  python open_stage_scene.py --uri omniverse://localhost/Users/test/scene.usd

Requires the ``requests`` package (``pip install requests``).
"""
import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 60.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Open a USD stage at ``--uri`` and confirm it's now active."""
    parser = argparse.ArgumentParser(description="Open a USD stage.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--uri", required=True, help="USD stage URI to open")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"

    # Opening a stage replaces whatever stage is currently loaded.
    _request(base, "PUT", "/stage/scene", {"uri": args.uri})
    print(f"opened: {args.uri}")

    print(f"active scene: {_request(base, 'GET', '/stage/scene')}")


if __name__ == "__main__":
    main()
