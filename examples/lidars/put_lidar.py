"""
PUT /lidars {prim_path, min_range?, max_range?, horizontal_fov?, vertical_fov?,
horizontal_resolution?, vertical_resolution?, rotation_rate?, high_lod?,
draw_points?, draw_lines?, yaw_offset?, data_types?} ->
{lidar_id, prim_path, min_range, max_range, ...}

Registers (and binds) a legacy PhysX lidar at ``prim_path``. If no Lidar prim
exists there, Isaac Sim creates one. ``data_types`` picks the outputs to
produce (default ["point_cloud"]). Registering the same prim again re-binds it
to the same id with the new configuration.

Run:  python put_lidar.py
      python put_lidar.py --prim /World/Lidar
      python put_lidar.py --prim /World/Lidar --data-types point_cloud depth
"""

import argparse
import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0

LIDAR_PRIM_PATH = "/World/Lidar"


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
            f"is Isaac Sim running and the bridge extension loaded? ({exc})"
        ) from exc

    return response.json() if response.content else None


def main():
    """Register a prim as a lidar and print what the bridge learned about it."""
    parser = argparse.ArgumentParser(description="Register a prim as a lidar.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", default=LIDAR_PRIM_PATH, help="lidar prim path to register")
    parser.add_argument(
        "--rotation-rate", type=float, default=20.0, help="Hz, 0 = instantaneous full-sweep lidar"
    )
    parser.add_argument(
        "--data-types", nargs="+", default=None, help="scan outputs, e.g. point_cloud depth"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        "/lidars",
        {
            "prim_path": args.prim,
            "rotation_rate": args.rotation_rate,
            "data_types": args.data_types,
        },
    )

    for key, value in response.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
