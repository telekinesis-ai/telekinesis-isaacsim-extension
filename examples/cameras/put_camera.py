"""
PUT /cameras {prim_path, resolution?, frequency?} ->
{camera_id, prim_path, resolution, active_data_types, supported_data_types,
 focal_length, ...}

Registers (and binds) a camera at ``prim_path``. If no Camera prim exists there,
Isaac Sim creates one. A new camera produces ``rgb``/``rgba``; other render
outputs are activated the first time capture.py asks for them. Registering the
same prim again re-binds it to the same id.

Requires Isaac Sim launched with ``--enable_cameras``.

Run:  python put_camera.py
      python put_camera.py --prim /World/Camera
"""

import argparse
import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0

CAMERA_PRIM_PATH = "/World/Camera"


def _request(base, method, path, body=None):
    """Send one request; exit with a clear message on failure instead of a
    traceback."""
    try:
        response = requests.request(
            method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT
        )
        response.raise_for_status()
    except requests.exceptions.HTTPError as exc:
        raise SystemExit(
            f"{method} {path} failed ({response.status_code}): {response.json()['detail']}"
        ) from exc
    except requests.exceptions.RequestException as exc:
        raise SystemExit(
            f"Could not reach {base} -- "
            f"is Isaac Sim running (with --enable_cameras) and the bridge extension "
            f"loaded? ({exc})"
        ) from exc

    return response.json() if response.content else None


def main():
    """Register a prim as a camera and print what the bridge learned about it."""
    parser = argparse.ArgumentParser(description="Register a prim as a camera.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", default=CAMERA_PRIM_PATH, help="camera prim path to register")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        "/cameras",
        {
            "prim_path": args.prim,
            "resolution": [args.width, args.height],
        },
    )

    for key, value in response.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
