"""
PUT /articulations/{id}/linear_velocity {velocity}  ([x, y, z]) -> {velocity}

Set the root link's linear (translational) velocity only (leaves angular
untouched). Only meaningful for a floating-base articulation (mobile base,
humanoid). Requires the id to already be registered -- run
put_articulation.py first to register a prim and get its articulation_id.

Run:  python set_linear_velocity.py --id articulation1
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
    """Set an articulation's root-link linear velocity."""
    parser = argparse.ArgumentParser(description="Set an articulation's root-link linear velocity.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id",
        required=True,
        dest="articulation_id",
        help="articulation_id from a prior PUT /articulations (see put_articulation.py); "
        "meaningful only on a floating-base articulation",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PUT",
        f"/articulations/{args.articulation_id}/linear_velocity",
        {"velocity": [0.1, 0.0, 0.0]},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
