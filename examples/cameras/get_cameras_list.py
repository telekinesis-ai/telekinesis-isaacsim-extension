"""
GET /cameras -> {camera_id: prim_path, ...}

Lists every registered camera. Run put_camera.py first to register one.

Run:  python get_cameras_list.py
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
    """List every registered camera id and its prim path."""
    parser = argparse.ArgumentParser(description="List registered cameras.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'GET', '/cameras')}")


if __name__ == "__main__":
    main()
