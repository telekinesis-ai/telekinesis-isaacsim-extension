"""
WS /lidars/{id}/stream_scan -> one frame per simulator update, shaped like the
POST .../capture response ({<data_type>: <array or null>, ..., num_cols_ticked,
timestamp}).

Push stream: the client sends nothing after connecting. Prints each frame's
shape (not its values), since a stream of point clouds would otherwise flood
the terminal. Stop with Ctrl+C, or pass --count to stop automatically.

Run:  python stream_scan.py --id lidar1
      python stream_scan.py --id lidar1 --count 10
"""

import argparse
import asyncio
import json

import websockets

HOST = "127.0.0.1"
PORT = 8766


def _shape(value):
    """Nested-list dimensions, for a compact printout of a big array."""
    dims = []
    while isinstance(value, list):
        dims.append(len(value))
        value = value[0] if value else None
    return dims


def _brief(frame):
    """One-line summary of a frame (shapes for arrays, values otherwise)."""
    parts = (
        f"{key}: shape {_shape(value)}" if isinstance(value, list) else f"{key}: {value}"
        for key, value in frame.items()
    )
    return "{" + ", ".join(parts) + "}"


async def _stream(uri, count):
    """Connect and print up to ``count`` frames (or forever if ``count`` is None)."""
    async with websockets.connect(uri) as websocket:
        received = 0
        async for message in websocket:
            print(f"frame {received}: {_brief(json.loads(message))}")
            received += 1
            if count is not None and received >= count:
                return


def main():
    """Stream scans from one lidar and print each frame's shape."""
    parser = argparse.ArgumentParser(description="Stream scans from a lidar.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lidar_id", help="lidar_id from put_lidar.py"
    )
    parser.add_argument(
        "--count", type=int, default=None, help="stop after this many frames (default: forever)"
    )
    args = parser.parse_args()

    uri = f"ws://{args.host}:{args.port}/lidars/{args.lidar_id}/stream_scan"
    try:
        asyncio.run(_stream(uri, args.count))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
