"""
PUT /articulations {prim_path, urdf_path?} ->
{articulation_id, prim_path, prim_source, num_dof, dof_names, ...}

``prim_source`` reports which actually happened: ``"isaac_usd"`` if the prim
was already in the stage (any urdf_path given had no effect), or
``"imported_urdf"`` if it was imported from that URDF file just now.

Two ways to load the articulation
1. Pass the urdf and prim path (Recommended)
2. Import usd and then register using it's prim path

Run:  python put_articulation.py
      python put_articulation.py --prim /World/ur10e
      python put_articulation.py --prim /World/ur10e --urdf ur10e.urdf
"""
import argparse
import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0

# Default prim path if you load a urd10e from isaacsim asset store
ROBOT_PRIM_PATH = "/World/ur10e"

def _request(base, method, path, body=None):
    """Send one request; exit with a clear message on failure instead of a
    traceback."""
    try:
        response = requests.request(method, base.rstrip("/") + path,
                                    json=body,
                                    timeout=DEFAULT_TIMEOUT)
        response.raise_for_status()
    except requests.exceptions.HTTPError as exc:

        # Bridge rejected the request
        # Response JSON body always has a "detail" message.
        raise SystemExit(f"{method} {path} failed ({response.status_code}): "
                         f"{response.json()['detail']}") from exc
    except requests.exceptions.RequestException as exc:
        # No response at all -- most likely the bridge isn't running.
        raise SystemExit(f"Could not reach {base} -- "
                         f"is Isaac Sim running with the bridge extension "
                         f"loaded? ({exc})") from exc

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

    # Print the bridge's response
    print(f"response: {response}")

    # Get detailed print
    keys = response.keys()
    for key in keys:
        print(f"{key}: {response[key]}")


if __name__ == "__main__":
    main()
