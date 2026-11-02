"""
POST /articulations/{id}/dof_gains {stiffness?, damping?, max_effort?, indices?}
-> {dof_properties}

Retunes the position drive's stiffness / damping / maximum effort. Each may be
a single value applied to every driven joint or omitted to leave it untouched.
Requires the id to already be registered -- run put_articulation.py first to
register a prim and get its articulation_id.

Run:  python set_dof_gains.py --id articulation1

Requires the ``requests`` package (``pip install requests``).
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
TARGET_STIFFNESS = 1000.0
TARGET_DAMPING = 100.0
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Retune every driven joint's position drive stiffness and damping, by id."""
    parser = argparse.ArgumentParser(description="Retune an articulation's position drive gains.")
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

    print(f"stiffness: {TARGET_STIFFNESS}, damping: {TARGET_DAMPING}")
    response = _request(
        base,
        "POST",
        f"/articulations/{args.articulation_id}/dof_gains",
        {"stiffness": TARGET_STIFFNESS, "damping": TARGET_DAMPING},
    )
    print(f"response: {response}")


if __name__ == "__main__":
    main()
