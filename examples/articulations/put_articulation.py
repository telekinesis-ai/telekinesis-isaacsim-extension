"""
PUT /articulations {prim_path, urdf_path?} -> 
{articulation_id, num_dof, dof_names, ...}

Run:  python put_articulation.py
      python put_articulation.py --prim /World/ur10e
"""
import argparse
import requests

HOST = "127.0.0.1"
PORT = 8766
ROBOT_PRIM_PATH = "/World/ur10e"
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
    """Register a prim as an articulation and print what the bridge learned about it."""
    parser = argparse.ArgumentParser(description="Register a prim as an articulation.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", default=ROBOT_PRIM_PATH, help="prim path to register")
    parser.add_argument("--urdf", default=None, help="optional URDF to import at --prim first")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"

    # Registering the same prim_path again just re-binds it to the same id.
    response = _request(
        base, "PUT", "/articulations", {"prim_path": args.prim, "urdf_path": args.urdf})
    print(f"response: {response}")


if __name__ == "__main__":
    main()
