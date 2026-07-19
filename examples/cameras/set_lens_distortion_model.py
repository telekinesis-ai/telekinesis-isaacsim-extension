"""
PUT /cameras/{id}/lens_distortion_model {value} -> {lens_distortion_model}

Sets the lens distortion model and applies its matching schema. Common values:
"pinhole", "opencvPinhole", "opencvFisheye", "ftheta".

Run:  python set_lens_distortion_model.py --id camera1 --value opencvPinhole
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
    """Set one camera's lens distortion model and print the resulting value."""
    parser = argparse.ArgumentParser(description="Set a camera's lens distortion model.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--id", required=True, dest="camera_id", help="camera_id from put_camera.py")
    parser.add_argument("--value", required=True, help="distortion model name, e.g. opencvPinhole")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "PUT", f"/cameras/{args.camera_id}/lens_distortion_model", {"value": args.value}
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
