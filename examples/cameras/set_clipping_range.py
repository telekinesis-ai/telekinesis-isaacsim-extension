"""
PUT /cameras/{id}/clipping_range {near_distance?, far_distance?} ->
{clipping_range: [near, far]}

Sets the near/far clipping distances (stage units). Omit either flag to leave
that bound unchanged.

Run:  python set_clipping_range.py --id camera1 --near 0.05 --far 1000
      python set_clipping_range.py --id camera1 --far 500
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
    """Set one camera's clipping range and print the resulting value."""
    parser = argparse.ArgumentParser(description="Set a camera's clipping range.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--id", required=True, dest="camera_id", help="camera_id from put_camera.py")
    parser.add_argument("--near", type=float, default=None, help="near distance (stage units)")
    parser.add_argument("--far", type=float, default=None, help="far distance (stage units)")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        f"/cameras/{args.camera_id}/clipping_range",
        {"near_distance": args.near, "far_distance": args.far},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
