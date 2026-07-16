"""
Standalone bridge example: unregister an articulation.

Deletes the articulation from the bridge's registry, but does not delete
the USD prim itself.

Run `python put_articulations.py` to register a prim path as an articulation.

Run:  python delete_articulation.py
      python delete_articulation.py --prim /World/ur10e

Requires the ``requests`` package (``pip install requests``).
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
    """Register an articulation, delete it by id, then confirm it's gone."""
    parser = argparse.ArgumentParser(description="Unregister an articulation.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--id", required=True, dest="articulation_id",
                        help="articulation_id from a prior PUT /articulations")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"

    # Unregister it (the USD prim itself is untouched).
    response = _request(base, "DELETE", f"/articulations/{args.articulation_id}")
    print(f"deleted articulation: {args.articulation_id}")
    print(f"response: {response}")


if __name__ == "__main__":
    main()
