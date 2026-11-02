"""
GET /articulations/{id}/applied_joint_efforts -> {joint_efforts}

Efforts last commanded via POST .../joint_efforts on the driven joints -- what
was asked for, not what was measured (see get_joints_state.py for
measured effort). Requires the id to already be registered -- run
put_articulation.py first to register a prim and get its articulation_id.

Run:  python get_applied_joint_efforts.py --id articulation1
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
    """Fetch the effort last commanded to an articulation's driven joints."""
    parser = argparse.ArgumentParser(
        description="Read an articulation's last-commanded joint efforts."
    )
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id",
        required=True,
        dest="articulation_id",
        help="articulation_id from a prior PUT /articulations (see put_articulation.py)",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(base, "GET", f"/articulations/{args.articulation_id}/applied_joint_efforts")
    print(f"response: {response}")


if __name__ == "__main__":
    main()
