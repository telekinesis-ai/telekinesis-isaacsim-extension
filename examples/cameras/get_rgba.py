"""
GET /cameras/{id}/rgba -> binary frame carrying {rgba: (H, W, 4) uint8 or null}

Reads the latest RGBA frame without re-pumping (use capture.py to force a fresh
frame). The response is a binary frame, not JSON -- see _decode below. Prints the
image shape, not the pixels.

Run:  python get_rgba.py --id camera1
"""

import argparse
import json

import numpy as np
import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, body=None):
    """Send one request and return the raw response body."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
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
    """Fetch one camera's latest RGBA frame and print its shape."""
    parser = argparse.ArgumentParser(description="Read a camera's latest RGBA image.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="camera_id", help="camera_id from put_camera.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    rgba = _decode(_request(base, "GET", f"/cameras/{args.camera_id}/rgba"))["rgba"]
    print("rgba:", "null (not ready)" if rgba is None else f"shape {rgba.shape} dtype {rgba.dtype}")


if __name__ == "__main__":
    main()
