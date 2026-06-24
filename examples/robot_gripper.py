"""
Standalone bridge example: robot AND gripper, over HTTP.

Converted from the old socket protocol. The bridge is now a single HTTP/JSON
server on 127.0.0.1:8765 (see telekinesis_isaacsim_bridge); there is no
per-device TCP port anymore, so this talks plain HTTP like gripper.py and
bridge_smoke_test.py.

Robot:   create -> read joints -> move_j -> read joints.
Gripper: create -> read -> close -> read -> open -> read.

Each articulation is addressed by the id returned from PUT /articulations (e.g.
``robot1`` / ``gripper1``). Moves block server-side: the POST returns only once
the move has completed -- no client-side polling.

Run:  python robot_gripper.py
      python robot_gripper.py --robot-prim /World/ur10e --gripper-prim /World/Robotiq_2F_85_edit

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import math

import requests

HOST = "127.0.0.1"
PORT = 8765
ROBOT_PRIM_PATH = "/World/ur10e"
GRIPPER_PRIM_PATH = "/World/Robotiq_2F_85_edit"

TARGET_DEG = [90.0, -90.0, 0.0, 0.0, 90.0, 0.0]

# A move blocks server-side until it finishes, so allow well over the bridge's
# own motion cap (~30 s) before the HTTP client gives up.
DEFAULT_TIMEOUT = 120.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def create_articulation(base, prim_path, device_type):
    """Register an articulation on the bridge; return its id and create info."""
    info = _request(
        base,
        "PUT",
        "/articulations",
        {"prim_path": prim_path, "device_type": device_type, "urdf_path": None},
    )
    print(f"created {device_type}: articulation_id={info['articulation_id']} prim_path={info['prim_path']}")
    return info["articulation_id"], info


def robot_joints_deg(base, articulation_id):
    """Current robot joint positions in degrees (wire is radians)."""
    return [round(math.degrees(q), 3) for q in _request(base, "GET", f"/articulations/{articulation_id}/robot/state")["q"]]


def run_robot(base, prim_path):
    articulation_id, info = create_articulation(base, prim_path, "robot")
    print(f"  num_dof={info['num_dof']} dof_names={info['dof_names']}")
    print(f"joints before (deg): {robot_joints_deg(base, articulation_id)}")
    print(f"move_j target (deg): {TARGET_DEG}")
    _request(base, "POST", f"/articulations/{articulation_id}/robot/move_j", {"q": [math.radians(d) for d in TARGET_DEG]})
    print(f"joints after  (deg): {robot_joints_deg(base, articulation_id)}")


def gripper_fraction(base, articulation_id):
    """Current gripper closed-ness fraction (0.0 open .. 1.0 closed)."""
    return round(_request(base, "GET", f"/articulations/{articulation_id}/gripper/state")["fraction"], 3)


def run_gripper(base, prim_path):
    articulation_id, _ = create_articulation(base, prim_path, "gripper")
    print(f"gripper fraction (start): {gripper_fraction(base, articulation_id)}")
    _request(base, "POST", f"/articulations/{articulation_id}/gripper/close")
    print(f"gripper fraction (closed): {gripper_fraction(base, articulation_id)}")
    _request(base, "POST", f"/articulations/{articulation_id}/gripper/open")
    print(f"gripper fraction (opened): {gripper_fraction(base, articulation_id)}")


def main():
    parser = argparse.ArgumentParser(description="Robot + gripper example for the Isaac Sim bridge.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--robot-prim", default=ROBOT_PRIM_PATH, help="prim path of the robot")
    parser.add_argument("--gripper-prim", default=GRIPPER_PRIM_PATH, help="prim path of the gripper")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"bridge: {base}  articulations: {_request(base, 'GET', '/articulations')}")

    print("\n--- robot ---")
    run_robot(base, args.robot_prim)
    print("\n--- gripper ---")
    run_gripper(base, args.gripper_prim)


if __name__ == "__main__":
    main()
