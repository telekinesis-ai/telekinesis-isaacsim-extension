"""
PUT /cameras/{id}/horizontal_aperture {value, maintain_square_pixels?} ->
{horizontal_aperture}

Sets the horizontal aperture / simulated sensor width (stage units). By default
the vertical aperture is kept in sync so pixels stay square; pass --no-square to
change only this axis.

Run:  python set_horizontal_aperture.py --id camera1 --value 20.955
      python set_horizontal_aperture.py --id camera1 --value 20.955 --no-square
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
    """Set one camera's horizontal aperture and print the resulting value."""
    parser = argparse.ArgumentParser(description="Set a camera's horizontal aperture.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    parser.add_argument("--value", type=float, required=True, help="aperture in stage units")
    parser.add_argument(
        "--no-square",
        action="store_false",
        dest="maintain_square_pixels",
        help="do not adjust the vertical aperture to keep pixels square",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        f"/cameras/{args.camera_id}/horizontal_aperture",
        {"value": args.value, "maintain_square_pixels": args.maintain_square_pixels},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
