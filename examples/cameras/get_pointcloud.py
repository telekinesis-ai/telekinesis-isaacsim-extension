"""
GET /cameras/{id}/pointcloud?world_frame -> {pointcloud: [[x, y, z], ...] or null}

Reads the latest pointcloud without re-pumping (use capture.py to force a fresh
frame). Points are camera-relative by default; pass --world-frame for points in
world coordinates. Prints the point count, not the points.

Requires the camera to have been bound with the "pointcloud" data type.

Run:  python get_pointcloud.py --id camera1
      python get_pointcloud.py --id camera1 --world-frame
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
    """Nested-list dimensions, for a compact printout of a big array."""
    dims = []
    while isinstance(value, list):
        dims.append(len(value))
        value = value[0] if value else None
    return dims


def main():
    """Fetch one camera's latest pointcloud and print its shape."""
    parser = argparse.ArgumentParser(description="Read a camera's latest pointcloud.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    parser.add_argument(
        "--world-frame",
        action="store_true",
        dest="world_frame",
        help="return world-frame points instead of camera-relative",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    path = f"/cameras/{args.camera_id}/pointcloud?world_frame={str(args.world_frame).lower()}"
    pointcloud = _request(base, "GET", path)["pointcloud"]
    print(
        "pointcloud:", "null (not ready)" if pointcloud is None else f"shape {_shape(pointcloud)}"
    )


if __name__ == "__main__":
    main()
