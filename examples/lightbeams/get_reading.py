"""
GET /lightbeams/{id}/reading -> {num_rays, broken, beam_hit,
linear_depth, hit_pos}

``broken`` is true when any one beam is broken -- the whole output of the
photoelectric switch this sensor stands in for. ``linear_depth`` is per
beam in meters and reads back as the sensor's max_range for a beam that is
not broken; ``hit_pos`` is per beam in the sensor's own frame.

The sensor is sampled rather than queried, so this answers 409 while the
timeline is stopped: the buffers then hold a stale reading that looks like
a clear beam. Requires the id to already be registered -- run
put_lightbeam.py first.

Run:  python get_reading.py --id lightbeam1
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
    """Read one lightbeam sensor's beams."""
    parser = argparse.ArgumentParser(description="Read a lightbeam sensor.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lightbeam_id", help="lightbeam_id from put_lightbeam.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'GET', f'/lightbeams/{args.lightbeam_id}/reading')}")


if __name__ == "__main__":
    main()
