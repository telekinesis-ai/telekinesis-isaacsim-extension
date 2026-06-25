"""
Standalone smoke test for the FastAPI bridge: create -> move.

Pure stdlib (urllib + JSON), no synapse / no Isaac Sim client libs, and -- unlike
the other examples -- not even the ``requests`` package, so it stays a single
self-contained file you can run from any Python. Talks to the
telekinesis_isaacsim_bridge extension's single HTTP server (127.0.0.1:8766).

The bridge is device-agnostic: it drives an articulation's joints (radians) and
reports reached/stalled; there is no robot/gripper notion on the wire. This script
shows the raw calls a client makes:

Robot flow:
  1. PUT /articulations {prim_path, urdf_path?} -> {articulation_id, num_dof, dof_names, state}
  2. POST /articulations/{id}/joint_positions {positions}  (radians)
     -> blocks, returns {done, reached, max_error, q, target}

Gripper flow (the robot/gripper-specific bits done inline here):
  1. PUT /articulations {prim_path, urdf_path?}
  2. GET /articulations/{id}/driver_joint                  -> {name, index}
  3. PUT /articulations/{id}/driven_joints {[driver_name]} -> narrow to one joint
  4. GET /articulations/{id}/joint_limits                  -> {limits: [[open, closed]]}
  5. close (fraction 1.0) -> open (fraction 0.0): map fraction to a joint angle and
     POST /articulations/{id}/joint_positions {positions:[rad]}, each blocking.

Run from any Python:  python bridge_smoke_test.py --prim /World/ur10e2
                      python bridge_smoke_test.py --device gripper --prim /World/robotiq
"""

import argparse
import json
import math
import urllib.request

HOST = "127.0.0.1"
PORT = 8766

# Robot joint targets (degrees for readability; converted to radians on the wire).
ROBOT_TARGETS_DEG = [
    [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0],
    [90.0, -90.0, 0.0, 0.0, 90.0, 0.0],
    [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0],
]

# A move blocks server-side until it finishes, so allow well over the bridge's
# own motion cap (~30 s) before the HTTP client gives up.
REQUEST_TIMEOUT_S = 120.0


def _request(base, method, path, body=None):
    """Send one HTTP request and return the decoded JSON (raise with server detail on error)."""
    data = json.dumps(body).encode() if body is not None else None
    headers = {"Content-Type": "application/json"} if data is not None else {}
    req = urllib.request.Request(base + path, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=REQUEST_TIMEOUT_S) as response:
            return json.load(response)
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode()
        raise RuntimeError(f"{method} {path} -> {exc.code}: {detail}") from None


def run_robot(base, prim_path, urdf_path):
    info = _request(base, "PUT", "/articulations", {"prim_path": prim_path, "urdf_path": urdf_path})
    articulation_id = info["articulation_id"]
    print(f"created robot: articulation_id={articulation_id} prim_path={info['prim_path']}")
    print(f"  num_dof={info['num_dof']} dof_names={info['dof_names']}")

    for target_deg in ROBOT_TARGETS_DEG:
        target_rad = [math.radians(d) for d in target_deg]
        print(f"move_j target (deg): {target_deg}")
        status = _request(
            base, "POST", f"/articulations/{articulation_id}/joint_positions", {"positions": target_rad}
        )
        print(f"  done={status['done']} reached={status['reached']} (max_error={status['max_error']:.2e} rad)")


def run_gripper(base, prim_path, urdf_path):
    info = _request(base, "PUT", "/articulations", {"prim_path": prim_path, "urdf_path": urdf_path})
    articulation_id = info["articulation_id"]
    print(f"created gripper: articulation_id={articulation_id} prim_path={info['prim_path']}")

    # Discover the actuated joint and narrow the device to it (gripper-specific,
    # done client-side -- the bridge only does the USD discovery and the narrowing).
    driver = _request(base, "GET", f"/articulations/{articulation_id}/driver_joint")
    _request(base, "PUT", f"/articulations/{articulation_id}/driven_joints", {"joint_names": [driver["name"]]})
    opened_rad, closed_rad = _request(base, "GET", f"/articulations/{articulation_id}/joint_limits")["limits"][0]
    print(f"  driver joint='{driver['name']}' open={opened_rad:.3f} closed={closed_rad:.3f} rad")

    for label, fraction in (("close", 1.0), ("open", 0.0)):
        target_rad = opened_rad + fraction * (closed_rad - opened_rad)
        print(f"gripper {label}")
        status = _request(
            base, "POST", f"/articulations/{articulation_id}/joint_positions", {"positions": [target_rad]}
        )
        print(f"  done (reached={status['reached']} q={status['q'][0]:.3f} rad)")


def main():
    parser = argparse.ArgumentParser(description="Smoke test the Isaac Sim FastAPI bridge.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--device", choices=("robot", "gripper"), default="robot")
    parser.add_argument(
        "--prim",
        default="/World/ur10e",
        help="prim path of the device in the stage")
    parser.add_argument(
        "--urdf",
        default=None,
        help="optional URDF to import if the prim isn't in the stage")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"bridge: {base}  articulations: {_request(base, 'GET', '/articulations')}")

    if args.device == "robot":
        run_robot(base, args.prim, args.urdf)
    else:
        run_gripper(base, args.prim, args.urdf)


if __name__ == "__main__":
    main()
