"""
GET /articulations/{id}/enabled_self_collisions -> {enabled}

Whether this articulation's own links can collide with each other. Requires
the id to already be registered -- run put_articulation.py first to register
a prim and get its articulation_id.

Run:  python get_enabled_self_collisions.py --id articulation1
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
    """Fetch an articulation's self-collision flag."""
    parser = argparse.ArgumentParser(description="Read an articulation's self-collision flag.")
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
    response = _request(
        base, "GET", f"/articulations/{args.articulation_id}/enabled_self_collisions"
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
