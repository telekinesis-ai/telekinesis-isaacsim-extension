# SPDX-License-Identifier: Apache-2.0
"""Binary frame format for the camera image routes.

The camera image routes answer ``application/octet-stream`` rather than JSON: a
1280x720 rgb frame is 2.8 MB of raw bytes but 58 MB as JSON nested lists, and
encoding those lists takes seconds on the thread that also renders and steps
physics. One frame carries any number of arrays plus the scalars that go with
them, and has the same layout over an HTTP body and a WebSocket binary message.

Layout::

    offset  size   contents
    0       4      magic b"TKB1", ASCII, identifies format and version
    4       4      manifest length N, uint32 little-endian
    8       N      manifest, UTF-8 JSON
    8+N     ...    every array's raw bytes, concatenated

The manifest describes the rest::

    {
      "structure": {"rgb": {"__ndarray__": 0}, "rendering_frame": 412,
                    "timestamp": 12.3456},
      "arrays": [{"shape": [720, 1280, 3], "dtype": "uint8",
                  "offset": 0, "nbytes": 2764800}]
    }

``structure`` is the response body the routes would otherwise have returned as
JSON, with every array replaced by ``{"__ndarray__": <index into arrays>}``.
Nesting is preserved, so an output that is itself a dict (``semantic_segmentation``
is ``{"data": ..., "info": {...}}``) needs no special handling. An output that is
not ready yet stays ``null``, exactly as it was in the JSON responses.

Each ``arrays`` entry describes one array's slice of the array region: ``offset``
counts from the start of that region (``8+N``), not from the start of the frame.
Arrays are C-contiguous and in the host's native byte order, so a client reads one
back by slicing the region and reinterpreting the bytes with ``shape`` and
``dtype`` -- no copy required. ``dtype`` is a numpy dtype name (``"uint8"``,
``"float32"``); for the record-dtype outputs (``occlusion``, the ``bounding_box_*``
family) it is instead the list of field descriptors numpy calls a dtype ``descr``,
since a name does not describe those.

Unlike the JSON responses, ``inf`` and ``nan`` inside an array survive as
themselves: a depth frame reports background pixels that hit nothing as ``inf``
rather than ``null``. Scalars in the manifest are still mapped to ``null``, because
the manifest is strict JSON with no ``Infinity``/``NaN`` tokens.

Errors are unaffected and still answer JSON ``{"detail": ...}``, so a client
decides how to read a response by its content type, not by its status alone.
"""

import json
import struct

import numpy as np

# Identifies the format and its version. A future incompatible layout gets a new
# tag ("TKB2") so a client can reject it outright instead of misreading it.
MAGIC = b"TKB1"

MEDIA_TYPE = "application/octet-stream"

# Handed to FastAPI's ``responses=`` so the generated OpenAPI document advertises
# the real content type. Without it the spec keeps claiming application/json,
# which nothing in the repo would catch.
OCTET_STREAM_RESPONSES = {200: {"content": {MEDIA_TYPE: {}}}}


def encode(payload):
    """Pack one response body into a binary frame.

    ``payload`` is the dict a camera route produces: numpy arrays for the render
    outputs, plain Python for everything else. Arrays are written out in the order
    they are encountered and replaced in the manifest by their index; the rest of
    the structure is carried through as JSON.

    Every array must be C-contiguous, which is what :mod:`..core.camera` returns.
    Raises ``ValueError`` if a scalar in the structure is not JSON-serializable, or
    is a non-finite float that the strict-JSON manifest cannot represent.
    """
    arrays = []
    structure = _extract_arrays(payload, arrays)

    chunks = []
    descriptors = []
    offset = 0
    for array in arrays:
        raw = array.tobytes()
        chunks.append(raw)
        descriptors.append(
            {
                "shape": list(array.shape),
                "dtype": _dtype_name(array.dtype),
                "offset": offset,
                "nbytes": len(raw),
            }
        )
        offset += len(raw)

    manifest = json.dumps(
        {"structure": structure, "arrays": descriptors}, allow_nan=False
    ).encode("utf-8")
    return b"".join([MAGIC, struct.pack("<I", len(manifest)), manifest, *chunks])


def _extract_arrays(value, arrays):
    """Return a JSON-ready copy of ``value`` with arrays replaced by index markers.

    Appends each array it finds to ``arrays`` in encounter order, so the marker it
    leaves behind is that array's position in the frame.
    """
    if isinstance(value, np.ndarray):
        arrays.append(value)
        return {"__ndarray__": len(arrays) - 1}
    if isinstance(value, dict):
        return {key: _extract_arrays(item, arrays) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_extract_arrays(item, arrays) for item in value]
    return value


def _dtype_name(dtype):
    """Describe ``dtype`` in a form a client can rebuild it from.

    A plain numeric dtype round-trips through its name. A record dtype (the
    bounding-box and occlusion outputs) does not, so its field descriptors are sent
    instead.
    """
    if dtype.fields is None:
        return dtype.name
    return [list(field) for field in dtype.descr]
