"""
Standalone bridge example: create articulation -> move through joint target.

Same as robot_set_joint.py but targets a second robot prim (``/World/ur10e_01``)
-- handy for checking that two robots in one stage are driven independently
(run this and robot_set_joint.py side by side; each gets its own articulation_id).

Converted from the old socket protocol. The bridge is now a single HTTP/JSON
server on 127.0.0.1:8766; articulations are addressed by the id from
PUT /articulations (e.g. ``robot1``), not a per-device TCP port.

Run:  python robot_set_joint2.py
      python robot_set_joint2.py --prim /World/ur10e_01

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import math

import requests

HOST = "127.0.0.1"
PORT = 8766
ROBOT_PRIM_PATH = "/World/ur10e_01"

# Joint targets (degrees here for readability; converted to radians on the wire).
TARGET_DEG = [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0]
TARGET_DEG2 = [90.0, -90.0, 0.0, 0.0, 90.0, 0.0]
TARGETS_DEG = [TARGET_DEG, TARGET_DEG2, TARGET_DEG]

# A move blocks server-side until it finishes, so allow well over the bridge's
# own motion cap (~30 s) before the HTTP client gives up.
DEFAULT_TIMEOUT = 120.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def run_robot(base, prim_path):
    info = _request(
        base,
        "PUT",
        "/articulations",
        {"prim_path": prim_path, "device_type": "robot", "urdf_path": None},
    )
    articulation_id = info["articulation_id"]
    print(f"created robot: articulation_id={articulation_id} prim_path={info['prim_path']}")
    print(f"  num_dof={info['num_dof']} dof_names={info['dof_names']}")

    for target_deg in TARGETS_DEG:
        print(f"move_j target (deg): {target_deg}")
        status = _request(
            base,
            "POST",
            f"/articulations/{articulation_id}/robot/move_j",
            {"q": [math.radians(d) for d in target_deg]},
        )
        print(f"  done={status['done']} (max_error={status['max_error']:.2e} rad)")


def main():
    parser = argparse.ArgumentParser(
        description="Second-robot joint-target example for the Isaac Sim bridge.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--prim",
        default=ROBOT_PRIM_PATH,
        help="prim path of the robot in the stage")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"bridge: {base}  articulations: {_request(base, 'GET', '/articulations')}")
    run_robot(base, args.prim)


if __name__ == "__main__":
    main()
