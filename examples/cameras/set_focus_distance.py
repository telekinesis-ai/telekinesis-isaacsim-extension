"""
PUT /cameras/{id}/focus_distance {value} -> {focus_distance}

Sets the distance from the camera to the focus plane (stage units). Only visible
with depth-of-field enabled (lens_aperture / fStop > 0).

Run:  python set_focus_distance.py --id camera1 --value 4.0
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
    """Set one camera's focus distance and print the resulting value."""
    parser = argparse.ArgumentParser(description="Set a camera's focus distance.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    parser.add_argument("--value", type=float, required=True, help="focus distance in stage units")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "PUT", f"/cameras/{args.camera_id}/focus_distance", {"value": args.value}
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
