"""
GET /cameras/{id}/local_pose?camera_axes -> {translation, orientation}

Reads the camera's local-frame (parent-relative) pose. ``orientation`` is a
scalar-first (w, x, y, z) quaternion. ``camera_axes`` is world (default), ros or usd.

Run:  python get_local_pose.py --id camera1
      python get_local_pose.py --id camera1 --axes ros
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
    """Fetch one camera's local pose and print it."""
    parser = argparse.ArgumentParser(description="Read a camera's local pose.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    parser.add_argument("--axes", default="world", choices=["world", "ros", "usd"])
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    path = f"/cameras/{args.camera_id}/local_pose?camera_axes={args.axes}"
    print(f"response: {_request(base, 'GET', path)}")


if __name__ == "__main__":
    main()
