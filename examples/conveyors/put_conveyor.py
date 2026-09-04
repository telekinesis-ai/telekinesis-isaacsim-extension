"""
PUT /conveyors {prim_path, cargo_root?} ->
{conveyor_id, prim_path, belt_prim_path, drive, direction, nominal_speed,
velocity, running}

Registers (and binds) one conveyor belt. ``prim_path`` may be the conveyor
asset's root, the belt rigid body itself, or any prim in between. The belt has
to be provisioned in the stage already -- carrying a non-zero surface velocity,
or driven by an ``IsaacConveyor`` node -- because its travel direction is read
from the scene rather than sent here.

``cargo_root`` names the prim whose sleeping rigid bodies are woken every time
the belt starts. PhysX leaves sleeping bodies out of the contact solve, so a
belt cannot pick up a box that came to rest while it was stopped. Narrow it to
the prims the belt actually carries; waking a whole warehouse costs a pass over
every prim in it.

Registering the same prim again keeps its id and reuses the travel direction and
authored speed captured the first time, rather than re-reading an attribute
``start`` has since written a command into. So register a belt at rest the first
time; after that a reversed run cannot flip its direction.

Registering plays the timeline: a belt carries nothing while physics is stopped.

Run:  python put_conveyor.py
      python put_conveyor.py --prim /World/ConveyorBelt_A11
      python put_conveyor.py --prim /World/ConveyorBelt_A08 --cargo-root /World
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0

CONVEYOR_PRIM_PATH = "/World/ConveyorBelt_A08"


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
    """Register a prim as a conveyor and print what the bridge learned about it."""
    parser = argparse.ArgumentParser(description="Register a prim as a conveyor.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--prim", default=CONVEYOR_PRIM_PATH, help="conveyor prim path to register"
    )
    parser.add_argument(
        "--cargo-root",
        default=None,
        help="prim whose sleeping rigid bodies are woken when the belt starts",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        "/conveyors",
        {"prim_path": args.prim, "cargo_root": args.cargo_root},
    )

    for key, value in response.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
