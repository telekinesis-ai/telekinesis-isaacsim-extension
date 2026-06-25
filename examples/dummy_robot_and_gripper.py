"""
Bridge example: build thin client doubles, assemble the robot, and drive the rig.

The other examples make raw HTTP calls so the extension API is visible. This one
shows the *other* half: how a real client wraps that generic API into reusable
robot/gripper objects. The bridge stays device-agnostic -- it only drives joint
indices and reports state -- so all the robot/gripper meaning (which joints to
drive, the fraction<->angle math, open/close) lives here, in ``DummyRobot`` and
``DummyGripper``. Both classes are defined inline below so you can read the whole
client side in one file.

It walks through most of the API end to end (everything except URDF loading, which
the *_load_from_urdf examples cover):
  * create two articulations (an arm and a gripper) from prims already in the stage
  * discover + narrow the gripper to its actuated driver joint
  * read joint limits
  * assemble the gripper onto the arm -> one shared articulation
  * move the arm (all joints) and the gripper (fraction -> one joint)
  * read joint state back

Each move comes in two flavours: the original method (``move_j`` / ``move`` /
``open`` / ``close``) lets the bridge block server-side, while a second ``*_and_wait``
variant fires the move asynchronously and runs the same reached-OR-stalled detection
on the client (:func:`_wait_for_motion`) -- so a collision or a grasp ends the wait
instead of hanging, with the wait loop visible here rather than inside the bridge.

Run:  python dummy_robot_and_gripper.py
      python dummy_robot_and_gripper.py --arm-prim /World/ur10e --gripper-prim /World/Robotiq_2F_85_edit

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import math
import time

import requests

HOST = "127.0.0.1"
PORT = 8766
ARM_PRIM_PATH = "/World/ur10e"
GRIPPER_PRIM_PATH = "/World/Robotiq_2F_85_edit"
ARM_MOUNT_LINK = "wrist_3_link"  # the arm's flange (a Link or Site), not an empty frame

# Arm joint target (degrees here for readability; radians on the wire).
TARGET_DEG = [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0]
TARGET_DEG2 = [90.0, -90.0, 0.0, 0.0, 90.0, 0.0]

# A blocking move runs to completion server-side (over the bridge's ~30 s motion
# cap), so give the HTTP client a generous timeout before it gives up.
DEFAULT_TIMEOUT = 120.0


def request(base, method, path, body=None):
    """Send one request to the bridge and return the decoded JSON (None if empty)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


# Client-side motion-finish thresholds, mirroring the bridge's blocking loop
# (core/articulation.py): a move is done when the joints are within REACH_TOL of
# target (reached cleanly), or have stayed below SETTLE_VEL for SETTLE_POLLS
# consecutive polls (stalled -- settled at a steady-state offset, or blocked by a
# grasped object). The consecutive requirement keeps the stall test from firing
# before the drive has accelerated the joints.
WAIT_REACH_TOLERANCE_RAD = 5e-3
WAIT_SETTLE_VELOCITY_RAD_S = 5e-3
WAIT_SETTLE_POLLS = 5


def _wait_for_motion(
    base,
    articulation_id,
    target_rad,
    reach_tolerance_rad=WAIT_REACH_TOLERANCE_RAD,
    settle_velocity_rad_s=WAIT_SETTLE_VELOCITY_RAD_S,
    settle_polls=WAIT_SETTLE_POLLS,
    timeout_s=30.0,
    poll_s=0.02,
):
    """Poll ``joint_state`` until the move reaches ``target_rad`` or stalls.

    This is the same reached-OR-stalled decision the bridge runs server-side when
    ``asynchronous=False``, done here on the client instead: read the driven joints'
    positions and velocities each poll, return ``reached=True`` once every joint is
    within ``reach_tolerance_rad`` of target, or ``reached=False`` once the joints
    have been below ``settle_velocity_rad_s`` for ``settle_polls`` consecutive polls
    (so a grasp or a collision ends the wait instead of hanging). ``done=False`` on
    timeout. Sim keeps stepping (the timeline plays), so polling converges.
    """
    deadline = time.monotonic() + timeout_s
    settled_polls = 0
    max_error = float("inf")
    q = None
    while time.monotonic() < deadline:
        state = request(base, "GET", f"/articulations/{articulation_id}/joint_state")
        q, qd = state["q"], state["qd"]
        max_error = max(abs(a - b) for a, b in zip(q, target_rad))
        if max_error < reach_tolerance_rad:
            return {"done": True, "reached": True, "max_error": max_error, "q": q}
        max_speed = max(abs(v) for v in qd) if qd else 0.0
        settled_polls = settled_polls + 1 if max_speed < settle_velocity_rad_s else 0
        if settled_polls >= settle_polls:
            return {"done": True, "reached": False, "max_error": max_error, "q": q}
        time.sleep(poll_s)
    return {"done": False, "reached": False, "max_error": max_error, "q": q}


class DummyRobot:
    """A robot built on the generic articulation API: drives all joints in radians."""

    def __init__(self, base, prim_path, urdf_path=None):
        self._base = base
        info = request(base, "PUT", "/articulations", {"prim_path": prim_path, "urdf_path": urdf_path})
        self.articulation_id = info["articulation_id"]
        self.prim_path = info["prim_path"]
        self.dof_names = info["dof_names"]
        self.num_dof = info["num_dof"]
        # [lower, upper] radian limits per joint, same order as joint_state's q.
        self.joint_limits = request(base, "GET", f"/articulations/{self.articulation_id}/joint_limits")["limits"]

    def move_j(self, q_rad, asynchronous=False):
        """Drive all joints to ``q_rad``. Blocks until reached/stalled unless async."""
        return request(
            self._base,
            "POST",
            f"/articulations/{self.articulation_id}/joint_positions",
            {"positions": list(q_rad), "asynchronous": asynchronous},
        )

    def move_j_and_wait(self, q_rad, timeout_s=30.0):
        """Second version of ``move_j`` that waits for the motion to finish client-side.

        Fires the move asynchronously, then runs the reached-OR-stalled detection
        here (see :func:`_wait_for_motion`) instead of letting the bridge block. The
        result is the same -- it returns once the arm reaches the target or stalls
        (e.g. it is blocked), so a collision ends the call rather than hanging -- but
        the wait loop is visible on the client side.
        """
        q_rad = list(q_rad)
        self.move_j(q_rad, asynchronous=True)
        return _wait_for_motion(self._base, self.articulation_id, q_rad, timeout_s=timeout_s)

    def get_state(self):
        """Current joint positions / velocities / torques (radians)."""
        return request(self._base, "GET", f"/articulations/{self.articulation_id}/joint_state")


class DummyGripper:
    """A gripper built on the generic articulation API.

    Narrows the device to its actuated driver joint, then maps a closed-ness
    ``fraction`` (0.0 open .. 1.0 closed) to a joint angle using the bridge's
    reported limits. The bridge sees only "drive this one joint to this angle".
    """

    def __init__(self, base, prim_path, urdf_path=None):
        self._base = base
        info = request(base, "PUT", "/articulations", {"prim_path": prim_path, "urdf_path": urdf_path})
        self.articulation_id = info["articulation_id"]
        self.prim_path = info["prim_path"]

        # Discover the actuated joint (USD schema walk on the bridge) and narrow the
        # device to it -- after this the gripper drives exactly one joint.
        driver = request(base, "GET", f"/articulations/{self.articulation_id}/driver_joint")
        self.driver_joint = driver["name"]
        request(
            base, "PUT", f"/articulations/{self.articulation_id}/driven_joints",
            {"joint_names": [self.driver_joint]},
        )

        # With the device narrowed to the driver, joint_limits is a single pair.
        # Convention: lower = open, upper = closed (flip per gripper if reversed).
        limits = request(base, "GET", f"/articulations/{self.articulation_id}/joint_limits")["limits"]
        self.opened_rad, self.closed_rad = limits[0]

    def move(self, fraction, asynchronous=False):
        """Drive the finger to ``fraction`` closed-ness (0 open .. 1 closed)."""
        fraction = min(max(float(fraction), 0.0), 1.0)
        target_rad = self.opened_rad + fraction * (self.closed_rad - self.opened_rad)
        return request(
            self._base,
            "POST",
            f"/articulations/{self.articulation_id}/joint_positions",
            {"positions": [target_rad], "asynchronous": asynchronous},
        )

    def move_and_wait(self, fraction, timeout_s=30.0):
        """Second version of ``move`` that waits for the motion to finish client-side.

        Fires the move asynchronously, then runs the reached-OR-stalled detection
        here (see :func:`_wait_for_motion`). For a gripper the *stall* case is the
        interesting one: when the finger closes onto an object it stops short of the
        commanded angle, so this returns ``reached=False`` -- a successful grasp --
        instead of waiting forever. The resulting closed-ness ``fraction`` is added
        to the returned status.
        """
        fraction = min(max(float(fraction), 0.0), 1.0)
        target_rad = self.opened_rad + fraction * (self.closed_rad - self.opened_rad)
        self.move(fraction, asynchronous=True)
        status = _wait_for_motion(self._base, self.articulation_id, [target_rad], timeout_s=timeout_s)
        status["fraction"] = self.fraction()
        return status

    def open(self, asynchronous=False):
        """Fully open the gripper (fraction 0.0)."""
        return self.move(0.0, asynchronous=asynchronous)

    def close(self, asynchronous=False):
        """Fully close the gripper (fraction 1.0)."""
        return self.move(1.0, asynchronous=asynchronous)

    def open_and_wait(self, timeout_s=30.0):
        """Fully open the gripper (fraction 0.0), waiting for the motion to finish."""
        return self.move_and_wait(0.0, timeout_s=timeout_s)

    def close_and_wait(self, timeout_s=30.0):
        """Fully close the gripper (fraction 1.0), waiting for the motion to finish."""
        return self.move_and_wait(1.0, timeout_s=timeout_s)

    def fraction(self):
        """Current closed-ness fraction, inverted from the driver joint angle."""
        current = request(self._base, "GET", f"/articulations/{self.articulation_id}/joint_state")["q"][0]
        span = self.closed_rad - self.opened_rad
        return 0.0 if span == 0 else (current - self.opened_rad) / span


def run(base, arm_prim, gripper_prim, arm_mount_link):
    # 1) Create the arm and the gripper from prims already in the stage.
    arm = DummyRobot(base, arm_prim)
    print(f"created arm: articulation_id={arm.articulation_id} prim_path={arm.prim_path} dof={arm.num_dof}")
    gripper = DummyGripper(base, gripper_prim)
    print(f"created gripper: articulation_id={gripper.articulation_id} prim_path={gripper.prim_path}")
    print(f"  driver joint='{gripper.driver_joint}' open={gripper.opened_rad:.3f} closed={gripper.closed_rad:.3f} rad")

    # 2) Assemble the gripper onto the arm -> one shared articulation (the gripper is
    #    already narrowed to its driver joint, so the merge folds in just that one).
    #    Re-posting for the same pair is a no-op (merged['already_assembled'] True).
    merged = request(
        base, "POST", f"/articulations/{arm.articulation_id}/assemble_robot",
        {
            "gripper_articulation_id": gripper.articulation_id,
            "arm_mount_link": arm_mount_link,
            "gripper_mount_link": None,  # auto-discover the gripper's base link
            "offset": None,
        },
    )
    print(f"assembled: shared articulation {merged['articulation']} num_dof={merged['num_dof']}")
    print(f"  arm drives {merged['robot']['num_dof']} dof: {merged['robot']['dof_names']}")

    # 3) Move the arm (all joints) on the shared rig. move_j blocks server-side;
    #    move_j_and_wait fires async and runs the same reached-or-stalled wait here.
    print(f"\nmove arm target (deg): {TARGET_DEG}  (server-blocking move_j)")
    status = arm.move_j([math.radians(d) for d in TARGET_DEG])
    print(f"  done={status['done']} reached={status['reached']} (max_error={status['max_error']:.2e} rad)")

    print(f"move arm back to zero  (client-side move_j_and_wait)")
    status = arm.move_j_and_wait(TARGET_DEG2)
    print(f"  done={status['done']} reached={status['reached']} (max_error={status['max_error']:.2e} rad)")
    print(f"  arm joints now (deg): {[round(math.degrees(v), 2) for v in arm.get_state()['q']]}")

    # 4) Move the gripper (fraction -> one joint) on the shared rig. close_and_wait
    #    returns reached=False if the finger stalls on an object (a successful grasp).
    print(f"\ngripper fraction (start): {gripper.fraction():.3f}")
    status = gripper.close_and_wait()
    print(f"gripper closed: reached={status['reached']} fraction={status['fraction']:.3f}")
    status = gripper.open_and_wait()
    print(f"gripper opened: reached={status['reached']} fraction={status['fraction']:.3f}")


def main():
    parser = argparse.ArgumentParser(
        description="DummyRobot + DummyGripper: assemble the robot and drive the rig via the bridge.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--arm-prim", default=ARM_PRIM_PATH, help="prim path of the arm in the stage")
    parser.add_argument("--gripper-prim", default=GRIPPER_PRIM_PATH, help="prim path of the gripper in the stage")
    parser.add_argument("--arm-mount-link", default=ARM_MOUNT_LINK, help="flange link (or Site) on the arm")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"bridge: {base}  articulations: {request(base, 'GET', '/articulations')}")
    run(base, args.arm_prim, args.gripper_prim, args.arm_mount_link)


if __name__ == "__main__":
    main()
