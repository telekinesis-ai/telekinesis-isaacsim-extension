"""
GET /surface_grippers/{id}/status ->
{status, gripped_objects, grip_distance, simulated}

``status`` is ``Open``, ``Closing`` or ``Closed``. It changes on a physics step
rather than when the command is sent, so reading it straight after a close or open
still reports the previous value -- which is why close and open block by default
and return the settled status themselves. Poll this route only after an
asynchronous close/open.

``Closing`` with a non-empty ``gripped_objects`` is a partial grip: some cups hold
something and the rest are still trying. ``simulated`` is false while the timeline
is stopped, in which case the status is the last one the simulation wrote rather
than a live reading.

Run:  python get_status.py --id surface_gripper1
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
    """Print one surface gripper's current status."""
    parser = argparse.ArgumentParser(description="Get a surface gripper's status.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id",
        required=True,
        dest="surface_gripper_id",
        help="surface_gripper_id from put_surface_gripper.py",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(base, "GET", f"/surface_grippers/{args.surface_gripper_id}/status")
    print(f"response: {response}")


if __name__ == "__main__":
    main()
