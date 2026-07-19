"""
GET /cameras/{id}/lens_distortion_model -> {lens_distortion_model}

Reads the lens distortion model name ("pinhole" if unset).

Run:  python get_lens_distortion_model.py --id camera1
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
    """Fetch one camera's lens distortion model and print it."""
    parser = argparse.ArgumentParser(description="Read a camera's lens distortion model.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--id", required=True, dest="camera_id", help="camera_id from put_camera.py")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'GET', f'/cameras/{args.camera_id}/lens_distortion_model')}")


if __name__ == "__main__":
    main()
