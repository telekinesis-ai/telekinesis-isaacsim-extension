"""
POST /surface_grippers/{id}/close {asynchronous?} ->
{done, timed_out, status, gripped_objects, grip_distance, simulated}

Grips whatever the gripper's cups can reach. Blocking by default, so the status in
the response is the settled one rather than the stale value a status read straight
after the command would give.

Three outcomes, told apart by ``status`` and ``gripped_objects``:

* ``Closed`` -- every cup gripped something.
* ``Closing`` with a non-empty ``gripped_objects`` -- a partial grip: some cups
  hold an object and the rest keep retrying. Often perfectly usable.
* ``Open`` with an empty ``gripped_objects`` -- nothing was in reach and the
  gripper gave up (after the ``retry_interval`` property's worth of trying).

``timed_out`` is true if none of the three settled within roughly 30 seconds of
simulation. With ``--asynchronous`` the command is only issued: ``done`` is false
and the status is still the previous one, so poll get_status.py yourself.

Requires the timeline to be playing: with the simulation stopped there is no
gripper to act on the command (409).

Run:  python close_gripper.py --id surface_gripper1
      python close_gripper.py --id surface_gripper1 --asynchronous
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
# A blocking close/open waits on the simulation, whose backstop is ~30 s, so give
# the HTTP client more than that.
DEFAULT_TIMEOUT = 60.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Close one surface gripper and print what it ended up holding."""
    parser = argparse.ArgumentParser(description="Close a surface gripper.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id",
        required=True,
        dest="surface_gripper_id",
        help="surface_gripper_id from put_surface_gripper.py",
    )
    parser.add_argument(
        "--asynchronous",
        action="store_true",
        help="return immediately instead of waiting for the gripper to settle",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "POST",
        f"/surface_grippers/{args.surface_gripper_id}/close",
        {"asynchronous": args.asynchronous},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
