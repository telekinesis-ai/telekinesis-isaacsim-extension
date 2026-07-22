"""
GET /cameras/{id}/rgb -> {rgb: [[[r, g, b], ...], ...] or null}

Reads the latest RGB frame without re-pumping (use capture.py to force a fresh
frame). Prints the image shape, not the pixels.

Run:  python get_rgb.py --id camera1
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


def _shape(value):
    """Nested-list dimensions, for a compact printout of a big image array."""
    dims = []
    while isinstance(value, list):
        dims.append(len(value))
        value = value[0] if value else None
    return dims


def main():
    """Fetch one camera's latest RGB frame and print its shape."""
    parser = argparse.ArgumentParser(description="Read a camera's latest RGB image.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    rgb = _request(base, "GET", f"/cameras/{args.camera_id}/rgb")["rgb"]
    print("rgb:", "null (not ready)" if rgb is None else f"shape {_shape(rgb)}")


if __name__ == "__main__":
    main()
