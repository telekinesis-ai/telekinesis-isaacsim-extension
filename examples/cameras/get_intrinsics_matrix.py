"""
GET /cameras/{id}/intrinsics_matrix -> {intrinsics_matrix: [[fx, 0, cx], ...]}

Reads the 3x3 camera intrinsics matrix (pixels). Only valid for pinhole models;
a non-pinhole lens distortion model returns a 400.

Run:  python get_intrinsics_matrix.py --id camera1
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
    """Fetch one camera's intrinsics matrix and print it row by row."""
    parser = argparse.ArgumentParser(description="Read a camera's intrinsics matrix.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--id", required=True, dest="camera_id", help="camera_id from put_camera.py")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    matrix = _request(base, "GET", f"/cameras/{args.camera_id}/intrinsics_matrix")["intrinsics_matrix"]
    print("intrinsics_matrix:")
    for row in matrix:
        print(f"  {row}")


if __name__ == "__main__":
    main()
