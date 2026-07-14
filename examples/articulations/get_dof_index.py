"""
GET /articulations/{id}/dof_index/{joint_name} -> {name, index}

DOF index of joint_name within the device's driven subset (the order
get_joints_state's joint_positions uses). Requires the id to already be
registered -- run put_articulation.py first to register a prim and get its
articulation_id, then pick one of its dof_names (see get_articulation.py).

Run:  python get_dof_index.py --id articulation1 --joint-name shoulder_pan_joint
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
    """Look up a driven joint's DOF index by name."""
    parser = argparse.ArgumentParser(description="Look up a driven joint's DOF index by name.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="articulation_id",
        help="articulation_id from a prior PUT /articulations (see put_articulation.py)")
    parser.add_argument("--joint-name", required=True, help="one of the articulation's dof_names")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(base, "GET", f"/articulations/{args.articulation_id}/dof_index/{args.joint_name}")
    print(f"response: {response}")


if __name__ == "__main__":
    main()
