"""
GET /cameras/{id}/clipping_range -> {clipping_range: [near, far]}

Reads the near/far clipping distances (stage units). Geometry nearer than near or
farther than far is not rendered.

Run:  python get_clipping_range.py --id camera1
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
    """Fetch one camera's clipping range and print it."""
    parser = argparse.ArgumentParser(description="Read a camera's clipping range.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--id", required=True, dest="camera_id", help="camera_id from put_camera.py")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'GET', f'/cameras/{args.camera_id}/clipping_range')}")


if __name__ == "__main__":
    main()
