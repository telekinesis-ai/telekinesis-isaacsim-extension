"""
PATCH /lightbeams/{id}/configuration {num_rays?, curtain_length?,
forward_axis?, curtain_axis?, min_range?, max_range?} ->
{num_rays, curtain_length, forward_axis, curtain_axis, min_range, max_range, ...}

Sets the sensor's beam layout and detection range. Fields left out are
untouched, so one property can be changed without restating the rest.

A single beam is a point detector along the forward axis. More than one beam
spreads the beams evenly over ``curtain_length`` along the curtain axis, which
is what lets the sensor detect an object whose height is not known in advance
-- so several beams need a positive curtain length, or every beam is cast from
the same place.

``min_range`` is a blind zone the beams start beyond, so an object closer than
it is not seen at all. ``max_range`` is both the furthest an object is seen and
the distance an unbroken beam reports.

Takes effect on the next physics step, including while the timeline plays: the
sensor's component re-reads all of it on change. Requires the id to already be
registered -- run put_lightbeam.py first.

Run:  python set_configuration.py --id lightbeam1 --num-rays 3 --curtain-length 0.2
      python set_configuration.py --id lightbeam1 --min-range 0.4 --max-range 1.2
      python set_configuration.py --id lightbeam1 --forward-axis 1 0 0
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
    """Change one lightbeam sensor's beam layout and print the result."""
    parser = argparse.ArgumentParser(description="Set a lightbeam sensor's configuration.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lightbeam_id", help="lightbeam_id from put_lightbeam.py"
    )
    parser.add_argument("--num-rays", type=int, default=None, help="beams in the curtain")
    parser.add_argument(
        "--curtain-length", type=float, default=None, help="meters the beams are spread over"
    )
    parser.add_argument(
        "--forward-axis",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="direction the beams are cast in, in the sensor's frame",
    )
    parser.add_argument(
        "--curtain-axis",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="direction the curtain is spread along, in the sensor's frame",
    )
    parser.add_argument("--min-range", type=float, default=None, help="blind zone in meters")
    parser.add_argument("--max-range", type=float, default=None, help="detection range in meters")
    args = parser.parse_args()

    # Only the fields actually passed are sent: the bridge reads an absent
    # field as "leave this property alone".
    body = {
        "num_rays": args.num_rays,
        "curtain_length": args.curtain_length,
        "forward_axis": args.forward_axis,
        "curtain_axis": args.curtain_axis,
        "min_range": args.min_range,
        "max_range": args.max_range,
    }
    body = {key: value for key, value in body.items() if value is not None}

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base, "PATCH", f"/lightbeams/{args.lightbeam_id}/configuration", body
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
