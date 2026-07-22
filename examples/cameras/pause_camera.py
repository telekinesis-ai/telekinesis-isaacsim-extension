"""
POST /cameras/{id}/pause -> {paused: true}

Stops the camera from collecting data / updating its frame. Resume with
resume_camera.py.

Run:  python pause_camera.py --id camera1
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
    """Pause one camera's data collection and print the result."""
    parser = argparse.ArgumentParser(description="Pause a camera's data collection.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'POST', f'/cameras/{args.camera_id}/pause')}")


if __name__ == "__main__":
    main()
