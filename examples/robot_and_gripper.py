"""
Standalone bridge example: robot AND gripper, over HTTP.

The bridge is a single HTTP/JSON server on 127.0.0.1:8766
(see telekinesis_isaacsim_bridge), device-agnostic at the wire: both the robot and
the gripper are articulations driven by POST /articulations/{id}/joint_positions.
The only difference is the client: a robot sets all joints in radians; a gripper
narrows itself to its actuated joint and maps a closed-ness fraction to an angle.

Robot:   create -> read joints -> move -> read joints.
Gripper: create -> discover/narrow driver -> read -> close -> read -> open -> read.

Each articulation is addressed by the id returned from PUT /articulations. Moves
block server-side: the call returns only once the move has completed.

Run:  python robot_and_gripper.py
      python robot_and_gripper.py --robot-prim /World/ur10e --gripper-prim /World/Robotiq_2F_85_edit

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import numpy as np

import requests

HOST = "127.0.0.1"
PORT = 8766
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


def robot_joints_deg(base, articulation_id):
    """Current joint positions in degrees (wire is radians)."""
    q = _request(base, "GET", f"/articulations/{articulation_id}/joint_state")["q"]
    return np.rad2deg(q).round(3).tolist()


def run_robot(base, prim_path):
    info = _request(base, "PUT", "/articulations", {"prim_path": prim_path, "urdf_path": None})
    articulation_id = info["articulation_id"]
    print(f"created robot: articulation_id={articulation_id} prim_path={info['prim_path']}")
    print(f"  num_dof={info['num_dof']} dof_names={info['dof_names']}")
    print(f"joints before (deg): {robot_joints_deg(base, articulation_id)}")
    print(f"move target (deg): {TARGET_DEG}")
    _request(
        base,
        "POST",
        f"/articulations/{articulation_id}/joint_positions",
        {"positions": np.deg2rad(TARGET_DEG).tolist()},
    )
    print(f"joints after  (deg): {robot_joints_deg(base, articulation_id)}")


def gripper_fraction(base, articulation_id, opened_rad, closed_rad):
    """Current closed-ness fraction (0.0 open .. 1.0 closed), inverted from the angle."""
    current = _request(base, "GET", f"/articulations/{articulation_id}/joint_state")["q"][0]
    span = closed_rad - opened_rad
    return round(0.0 if span == 0 else (current - opened_rad) / span, 3)


def run_gripper(base, prim_path):
    info = _request(base, "PUT", "/articulations", {"prim_path": prim_path, "urdf_path": None})
    articulation_id = info["articulation_id"]
    print(f"created gripper: articulation_id={articulation_id} prim_path={info['prim_path']}")

    driver = _request(base, "GET", f"/articulations/{articulation_id}/driver_joint")
    _request(base, "PUT", f"/articulations/{articulation_id}/driven_joints", {"joint_names": [driver["name"]]})
    opened_rad, closed_rad = _request(base, "GET", f"/articulations/{articulation_id}/joint_limits")["limits"][0]
    print(f"  driver joint='{driver['name']}' open={opened_rad:.3f} closed={closed_rad:.3f} rad")

    print(f"gripper fraction (start): {gripper_fraction(base, articulation_id, opened_rad, closed_rad)}")
    _request(base, "POST", f"/articulations/{articulation_id}/joint_positions", {"positions": [closed_rad]})
    print(f"gripper fraction (closed): {gripper_fraction(base, articulation_id, opened_rad, closed_rad)}")
    _request(base, "POST", f"/articulations/{articulation_id}/joint_positions", {"positions": [opened_rad]})
    print(f"gripper fraction (opened): {gripper_fraction(base, articulation_id, opened_rad, closed_rad)}")


def main():
    parser = argparse.ArgumentParser(
        description="Robot + gripper example for the Isaac Sim bridge.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--robot-prim", default=ROBOT_PRIM_PATH, help="prim path of the robot")
    parser.add_argument(
        "--gripper-prim",
        default=GRIPPER_PRIM_PATH,
        help="prim path of the gripper")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"bridge: {base}  articulations: {_request(base, 'GET', '/articulations')}")

    print("\n--- robot ---")
    run_robot(base, args.robot_prim)
    print("\n--- gripper ---")
    run_gripper(base, args.gripper_prim)


if __name__ == "__main__":
    main()
