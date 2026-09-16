"""
PUT /lightbeams {prim_path} ->
{lightbeam_id, prim_path, num_rays, curtain_length, forward_axis,
curtain_axis, min_range, max_range, enabled}

Registers (and binds) one lightbeam sensor. ``prim_path`` has to name an
existing ``IsaacLightBeamSensor`` prim: unlike a lidar, the bridge does not
create one, because a lightbeam's placement and aim are the whole sensor.

Registering it enables the sensor if the scene left it disabled -- a disabled
sensor never reports a hit -- and blocks while its first reading comes up,
which takes a few physics steps and starts the timeline. Registering the same
prim again keeps its id and re-binds it, which is what it needs after the
timeline has been stopped and replayed.

Run:  python put_lightbeam.py
      python put_lightbeam.py --prim /World/LightBeam_Sensor
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
# Binding waits out the physics steps the sensor's first reading needs.
DEFAULT_TIMEOUT = 60.0

LIGHTBEAM_PRIM_PATH = "/World/LightBeam_Sensor"


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
    """Register a prim as a lightbeam sensor and print its beam configuration."""
    parser = argparse.ArgumentParser(description="Register a prim as a lightbeam sensor.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--prim", default=LIGHTBEAM_PRIM_PATH, help="lightbeam sensor prim path to register"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(base, "PUT", "/lightbeams", {"prim_path": args.prim})

    for key, value in response.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
