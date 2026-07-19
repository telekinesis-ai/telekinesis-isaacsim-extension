"""
GET /cameras/{id}/render_product_path -> {render_product_path}

Reads the USD path of the render product backing this camera. Useful for
debugging or wiring the render product into other tooling.

Run:  python get_render_product_path.py --id camera1
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
    """Fetch one camera's render product path and print it."""
    parser = argparse.ArgumentParser(description="Read a camera's render product path.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--id", required=True, dest="camera_id", help="camera_id from put_camera.py")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'GET', f'/cameras/{args.camera_id}/render_product_path')}")


if __name__ == "__main__":
    main()
