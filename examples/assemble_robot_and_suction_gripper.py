"""
Bridge example: attach a suction gripper to an arm, then move the arm and grip.

The suction counterpart to ``assemble_robot_and_gripper.py``. Creates the arm as an
articulation and the suction gripper as a surface gripper, then POSTs
``/articulations/{arm_id}/assemble_robot`` with the gripper's id.

The same route takes either kind of gripper: the bridge tells them apart by which
registry the ``gripper_id`` is in, and attaches accordingly. For a suction gripper
that means nothing is merged -- there are no joints to fold into the arm's
articulation, so the arm's DOF are unchanged and the gripper keeps its own id and
its own close/open routes. What assembly does is place the gripper at the flange,
join the two with a fixed joint excluded from the articulation, and re-park the
gripper's attachment points onto the arm's mount link so they can grip at all.

Assembly mutates USD and is not idempotent, so the bridge records each completed
assembly: POSTing ``assemble_robot`` again for the same pair is a no-op that just
returns the info with ``already_assembled=True``.

Flow (all over HTTP):
  1. PUT  /articulations {arm, urdf_path}                    -> arm articulation
  2. PUT  /surface_grippers {gripper prim}                    -> surface gripper
  3. POST /articulations/{arm_id}/assemble_robot {gripper_id,
        arm_mount_link, gripper_mount_link?, offset?, mask_collisions?}
  4. POST /articulations/{arm_id}/move_j {joint_positions}   (radians) -> blocks
  5. POST /surface_grippers/{grip_id}/close                  -> blocks, reports the grip
  6. POST /surface_grippers/{grip_id}/open                   -> blocks

``arm_mount_link`` is the arm's flange (a RigidBodyAPI link or a Site, e.g. UR
``wrist_3_link``), NOT an empty frame like ``tool0`` / ``flange``.

``gripper_mount_link`` is left unset here, which means the registered gripper prim
itself -- and that prim has to be the gripper's rigid body, since the fixed joint
needs one. Pass ``--gripper-mount-link`` when the asset's root is a plain Xform and
the rigid body sits inside it.

Run:  python assemble_robot_and_suction_gripper.py
      python assemble_robot_and_suction_gripper.py --arm-mount-link wrist_3_link

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import numpy as np

import requests

try:
    import telekinesis_urdfs
except ImportError as e:
    raise ImportError(
        "telekinesis-urdfs is not installed. "
        "Install it from: https://github.com/telekinesis-ai/telekinesis-urdfs"
    ) from e

HOST = "127.0.0.1"
PORT = 8766

ARM_PRIM_PATH = "/World/ur10e"
GRIPPER_PRIM_PATH = "/World/defitech_modelled_surface_gripper"

# The arm mount is its flange (a Link or Site). The gripper mount is left None so
# the registered gripper prim itself is used -- pass --gripper-mount-link to point
# at the gripper's rigid body when its root is a plain Xform.
ARM_MOUNT_LINK = "wrist_3_link"
GRIPPER_MOUNT_LINK = None

# Optional mount offset baked into the fixed joint (meters / XYZ Euler degrees).
# None => flush attach (no transform).
ATTACH_OFFSET = None

# Load the arm URDF so the bridge can import it if the prim isn't in the stage.
try:
    robot_description = telekinesis_urdfs.load("UniversalRobotsUR10e")
    if not robot_description.urdf_path.is_file():
        raise ValueError(f"No urdf found, i.e. {robot_description.urdf_path} is not a file")
except Exception as e:
    raise RuntimeError(
        "Failed to load robot description 'UniversalRobotsUR10e'. "
        "Ensure telekinesis-urdfs is installed: "
        "https://github.com/telekinesis-ai/telekinesis-urdfs"
    ) from e

ARM_URDF_PATH = str(robot_description.urdf_path)

# Arm joint target once assembled (degrees here for readability; radians on the wire).
TARGET_DEG = [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0]

# Importing/assembling and the blocking moves and grips can each take a while
# server-side, so give the HTTP client a generous timeout (over the bridge's ~30 s
# motion and actuation caps).
DEFAULT_TIMEOUT = 120.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def run(base, arm_prim, arm_urdf, gripper_prim, arm_mount_link, gripper_mount_link, offset):
    """Create the arm and suction gripper, attach them, then move and grip."""
    # 1) Create the arm (urdf_path imports it if arm_prim isn't in the stage).
    arm = _request(base, "PUT", "/articulations", {"prim_path": arm_prim, "urdf_path": arm_urdf})
    arm_id = arm["articulation_id"]
    print(
        f"created arm: articulation_id={arm_id} prim_path={arm['prim_path']} dof={arm['num_dof']}"
    )

    # 2) Create the suction gripper. No urdf_path counterpart: the asset has to be
    #    in the stage already.
    gripper = _request(base, "PUT", "/surface_grippers", {"prim_path": gripper_prim})
    gripper_id = gripper["surface_gripper_id"]
    print(
        f"created gripper: surface_gripper_id={gripper_id} "
        f"gripper_prim_path={gripper['gripper_prim_path']} "
        f"attachment_points={len(gripper['attachment_point_paths'])}"
    )

    # 3) Attach the gripper to the arm. Running this again for the same pair is a
    #    no-op (merged['already_assembled'] is True).
    merged = _request(
        base,
        "POST",
        f"/articulations/{arm_id}/assemble_robot",
        {
            "gripper_id": gripper_id,
            "arm_mount_link": arm_mount_link,
            "gripper_mount_link": gripper_mount_link,
            "offset": offset,
            "mask_collisions": True,
        },
    )
    print(
        f"assembled: {merged['gripper_kind']} gripper on {merged['articulation']} "
        f"num_dof={merged['num_dof']} (already_assembled={merged['already_assembled']})"
    )
    print(
        f"  mounts: arm={merged['arm_mount_link']} "
        f"gripper={merged['gripper_mount_link']} (resolved)"
    )
    print(f"  fixed joint: {merged['fixed_joint']}")

    # 4) Move the arm. Its DOF are unchanged by a suction gripper, so this is the
    #    same 6-value payload as a standalone arm.
    print(f"move arm target (deg): {TARGET_DEG}")
    status = _request(
        base,
        "POST",
        f"/articulations/{arm_id}/move_j",
        {"joint_positions": np.deg2rad(TARGET_DEG).tolist()},
    )
    print(
        f"  done={status['done']} reached={status['reached']} "
        f"(max_error={status['max_error']:.2e} rad)"
    )

    # 5) Close then open the gripper. Both block, so the status they report is the
    #    settled one -- reading /status straight after the command would still give
    #    the previous value, since the gripper acts on the next physics step.
    for action in ("close", "open"):
        print(f"gripper {action}")
        status = _request(
            base, "POST", f"/surface_grippers/{gripper_id}/{action}", {"asynchronous": False}
        )
        print(
            f"  done={status['done']} timed_out={status['timed_out']} "
            f"status={status['status']} gripped_objects={status['gripped_objects']}"
        )


def main():
    """Parse CLI args and run the example."""
    parser = argparse.ArgumentParser(
        description="Attach a suction gripper to an arm via the Isaac Sim bridge."
    )
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--arm-prim", default=ARM_PRIM_PATH, help="prim path to import/bind the arm at"
    )
    parser.add_argument(
        "--gripper-prim",
        default=GRIPPER_PRIM_PATH,
        help="prim path of the suction gripper asset in the stage",
    )
    parser.add_argument("--arm-urdf", default=ARM_URDF_PATH, help="path to the arm URDF to import")
    parser.add_argument(
        "--arm-mount-link", default=ARM_MOUNT_LINK, help="flange link (or Site) on the arm"
    )
    parser.add_argument(
        "--gripper-mount-link",
        default=GRIPPER_MOUNT_LINK,
        help="gripper rigid body to mount at (omit to use the registered gripper prim)",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"bridge: {base}  surface grippers: {_request(base, 'GET', '/surface_grippers')}")
    run(
        base,
        args.arm_prim,
        args.arm_urdf,
        args.gripper_prim,
        args.arm_mount_link,
        args.gripper_mount_link,
        ATTACH_OFFSET,
    )


if __name__ == "__main__":
    main()
