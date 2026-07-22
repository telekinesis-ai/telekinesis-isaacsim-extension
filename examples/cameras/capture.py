"""
POST /cameras/{id}/capture {data_types?} ->
{<data_type>: <array or null>, ..., rendering_frame, timestamp}

Pumps one frame and returns the requested outputs (all bound types if omitted).
Image arrays can be large; this prints only each output's shape, not its pixels.

Run:  python capture.py --id camera1
      python capture.py --id camera1 --data-types rgb depth
"""

import argparse

import cv2
import numpy as np
import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, body=None):
    """Send one request; on failure exit with the server's detail message."""
    try:
        response = requests.request(
            method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT
        )
        response.raise_for_status()
    except requests.exceptions.HTTPError as exc:
        raise SystemExit(
            f"{method} {path} failed ({response.status_code}): {response.json().get('detail')}"
        ) from exc
    return response.json() if response.content else None


def _shape(value):
    """Nested-list dimensions, for a compact printout of a big image array."""
    dims = []
    while isinstance(value, list):
        dims.append(len(value))
        value = value[0] if value else None
    return dims


def main():
    """Capture one frame and print each output's shape (or scalar value)."""
    parser = argparse.ArgumentParser(description="Capture a frame from a camera.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    parser.add_argument("--data-types", nargs="+", default=None, help="subset to capture")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "POST", f"/cameras/{args.camera_id}/capture", {"data_types": args.data_types}
    )

    for key, value in response.items():
        if isinstance(value, list):
            print(f"{key}: shape {_shape(value)}")
        else:
            print(f"{key}: {value}")

    _show(response)


def _show(response):
    """Display any rgb/depth outputs in the response with OpenCV.

    The arrays arrive as JSON nested lists, so rebuild numpy arrays first. Depth
    background pixels came back as null (JSON has no inf) -> NaN here, so only the
    finite range is normalized for display; NaN pixels show as black.
    """
    shown = False
    rgb = response.get("rgb")
    if rgb is not None:
        arr = np.asarray(rgb)
        print(f"rgb dtype={arr.dtype} min={arr.min()} max={arr.max()}")
        # Isaac's rgb (LdrColor) annotator is float32 in [0, 1]; scale to 8-bit.
        # If a build returns uint8 [0, 255] instead, it's already in range.
        if arr.dtype.kind == "f":
            arr = arr * 255.0
        frame = np.clip(arr, 0, 255).astype(np.uint8)
        # pylint: disable=no-member
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # pylint: disable=no-member
        cv2.imshow("rgb", frame)
        shown = True

    depth = response.get("depth")
    if depth is not None:
        depth = np.array(depth, dtype=np.float32)  # null -> NaN
        vis = np.zeros(depth.shape, dtype=np.uint8)
        finite = np.isfinite(depth)
        if finite.any():
            lo, hi = float(depth[finite].min()), float(depth[finite].max())
            scaled = (depth - lo) / (hi - lo + 1e-9)
            vis = np.clip(scaled * 255.0, 0, 255).astype(np.uint8)
            vis[~finite] = 0
        # pylint: disable=no-member
        cv2.imshow("depth", vis)
        shown = True

    if shown:
        print("press any key in the image window to close")
        # pylint: disable=no-member
        cv2.waitKey(0)
        # pylint: disable=no-member
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
