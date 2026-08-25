"""
POST /cameras/{id}/capture {data_types?, world_frame?} -> binary frame carrying
{<data_type>: <array or null>, ..., rendering_frame, timestamp}

Pumps one frame and returns the requested outputs (every output currently active
if omitted, which is rgb/rgba on a new camera). Requesting a supported output
that is not active yet activates it, so that first capture takes a few extra
frames of annotator warmup. GET /cameras/{id} lists both sets.

The response is a binary frame rather than JSON -- see _decode below. Image arrays
can be large; this prints only each output's shape, not its pixels.

Run:  python capture.py --id camera1
      python capture.py --id camera1 --data-types rgb depth
"""

import argparse
import json

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
        # Errors stay JSON even on the routes that answer binary on success.
        raise SystemExit(
            f"{method} {path} failed ({response.status_code}): {response.json().get('detail')}"
        ) from exc
    return response.content


def _decode(payload):
    """Rebuild one binary camera frame into the dict the route describes.

    Frame layout: 4-byte magic b"TKB1", the manifest length as a little-endian
    uint32, that many bytes of UTF-8 JSON manifest, then every array's raw bytes
    concatenated. The manifest's "structure" is the response body with each array
    replaced by {"__ndarray__": <index>}, and its "arrays" list gives each array's
    shape, dtype and slice of the array region. Arrays are C-contiguous in native
    byte order, so np.frombuffer views them without copying -- the arrays this
    returns are read-only.
    """
    if payload[:4] != b"TKB1":
        raise ValueError(f"not a camera binary frame: {payload[:4]!r}")
    manifest_length = int.from_bytes(payload[4:8], "little")
    manifest = json.loads(payload[8 : 8 + manifest_length])
    region = 8 + manifest_length

    arrays = []
    for entry in manifest["arrays"]:
        start = region + entry["offset"]
        dtype = entry["dtype"]
        if isinstance(dtype, list):  # record dtype, sent as numpy's descr
            dtype = np.dtype([tuple(field) for field in dtype])
        arrays.append(
            np.frombuffer(payload[start : start + entry["nbytes"]], dtype=dtype).reshape(
                entry["shape"]
            )
        )
    return _restore(manifest["structure"], arrays)


def _restore(value, arrays):
    """Put the decoded arrays back where their markers are."""
    if isinstance(value, dict):
        if len(value) == 1 and "__ndarray__" in value:
            return arrays[value["__ndarray__"]]
        return {key: _restore(item, arrays) for key, item in value.items()}
    if isinstance(value, list):
        return [_restore(item, arrays) for item in value]
    return value


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
    frame = _decode(
        _request(
            base, "POST", f"/cameras/{args.camera_id}/capture", {"data_types": args.data_types}
        )
    )

    for key, value in frame.items():
        if isinstance(value, np.ndarray):
            print(f"{key}: shape {value.shape} dtype {value.dtype}")
        else:
            print(f"{key}: {value}")

    _show(frame)


def _show(frame):
    """Display any rgb/depth outputs in the frame with OpenCV.

    Depth pixels that hit nothing are inf, so only the finite range is normalized
    for display and those pixels show as black. The decoded arrays are read-only
    views into the response body, which is fine here: every step below allocates a
    new array rather than writing into the decoded one.
    """
    shown = False
    rgb = frame.get("rgb")
    if rgb is not None:
        print(f"rgb dtype={rgb.dtype} min={rgb.min()} max={rgb.max()}")
        # Isaac's rgb (LdrColor) annotator is float32 in [0, 1]; scale to 8-bit.
        # If a build returns uint8 [0, 255] instead, it's already in range.
        if rgb.dtype.kind == "f":
            rgb = rgb * 255.0
        image = np.clip(rgb, 0, 255).astype(np.uint8)
        # pylint: disable=no-member
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # pylint: disable=no-member
        cv2.imshow("rgb", image)
        shown = True

    depth = frame.get("depth")
    if depth is not None:
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
