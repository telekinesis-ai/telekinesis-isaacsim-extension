"""
POST /lidars/{id}/capture {data_types?} ->
{<data_type>: <array or null>, ..., num_cols_ticked, timestamp}

Pumps one frame and returns the requested outputs (all bound types if
omitted). Point clouds/buffers can be large; this prints only each output's
shape, not its values.

Run:  python capture.py --id lidar1
      python capture.py --id lidar1 --data-types point_cloud depth
"""

import argparse

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
    """Nested-list dimensions, for a compact printout of a big array."""
    dims = []
    while isinstance(value, list):
        dims.append(len(value))
        value = value[0] if value else None
    return dims


def main():
    """Capture one scan from a lidar and print each output's shape (or scalar value)."""
    parser = argparse.ArgumentParser(description="Capture a scan from a lidar.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lidar_id", help="lidar_id from put_lidar.py"
    )
    parser.add_argument("--data-types", nargs="+", default=None, help="subset to capture")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "POST", f"/lidars/{args.lidar_id}/capture", {"data_types": args.data_types}
    )

    for key, value in response.items():
        if isinstance(value, list):
            print(f"{key}: shape {_shape(value)}")
        else:
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
