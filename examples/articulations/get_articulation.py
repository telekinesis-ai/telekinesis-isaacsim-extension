"""
GET /articulations/{id} -> {articulation_id, prim_path, num_dof, dof_names, state}

Requires the id to already be registered.

Run `python put_articulations.py` to register a prim path as an articulation.

Run:  python get_articulation.py --id articulation1
"""
import argparse
import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    try:
        response.raise_for_status()
    except requests.exceptions.HTTPError:
        # The useful part is the server's "detail" message, which raise_for_status()
        # never surfaces on its own -- print it before re-raising.
        try:
            print(f"error detail: {response.json().get('detail')}")
        except ValueError:
            pass
        raise
    return response.json() if response.content else None


def main():
    """Fetch one articulation's info by id."""
    parser = argparse.ArgumentParser(description="Get info for one articulation.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="articulation_id",
        help="articulation_id from a prior PUT /articulations (see put_articulation.py)")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(base, "GET", f"/articulations/{args.articulation_id}")
    print(f"response: {response}")


if __name__ == "__main__":
    main()
