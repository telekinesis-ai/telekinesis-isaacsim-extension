"""
PUT /cameras/{id}/resolution {width, height} -> {resolution: [width, height]}

Sets the camera's pixel resolution; apertures are kept in sync for square pixels.

Run:  python set_resolution.py --id camera1 --width 1920 --height 1080
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
    """Set one camera's resolution and print the resulting value."""
    parser = argparse.ArgumentParser(description="Set a camera's resolution.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--id", required=True, dest="camera_id", help="camera_id from put_camera.py")
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        f"/cameras/{args.camera_id}/resolution",
        {"width": args.width, "height": args.height},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
