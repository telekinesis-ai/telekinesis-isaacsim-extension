"""
POST /conveyors/{id}/start {velocity?} ->
{conveyor_id, woken_bodies, velocity, running, ...}

Runs the belt at a signed speed along the travel direction its scene authored,
so reversing it means passing a negative ``--velocity`` rather than a
direction. Omitting it runs the belt at the speed the scene authored, which
``nominal_speed`` reports.

``woken_bodies`` is how many sleeping rigid bodies under the conveyor's cargo
root were woken so the starting belt can pick them up -- zero when no cargo
root was given to put_conveyor.py, or while the timeline is stopped.

Needs the timeline to be playing to have any effect: a surface velocity moves
nothing unless physics is stepping. put_conveyor.py starts it, so this only
matters if the timeline was stopped since -- the bridge then logs a warning and
the belt runs as soon as it plays again.

Requires the id to already be registered -- run put_conveyor.py first.

Run:  python start_conveyor.py --id conveyor1
      python start_conveyor.py --id conveyor1 --velocity -0.5
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
    """Start one conveyor belt and print the state it ended up in."""
    parser = argparse.ArgumentParser(description="Start a conveyor belt.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="conveyor_id", help="conveyor_id from put_conveyor.py"
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=None,
        help="signed m/s along the belt's authored direction (default: its own speed)",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "POST",
        f"/conveyors/{args.conveyor_id}/start",
        {"velocity": args.velocity},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
