"""
Bridge example: attach a gripper onto an arm, then drive the assembled rig.

Creates an arm and a gripper as two separate articulations, then POSTs
``/articulations/{arm_id}/robot/attach_tool`` to assemble them with
``RobotAssembler``. After that the arm and gripper share ONE articulation, but
each keeps driving only its own joints -- so the *same* ``move_j`` / gripper
calls work unchanged whether or not the tool is attached.

Flow (all over HTTP):
  1. PUT /articulations {arm,  device_type:"robot",   urdf_path} -> arm articulation
  2. PUT /articulations {grip, device_type:"gripper", urdf_path?} -> gripper articulation
  3. POST /articulations/{arm_id}/robot/attach_tool {gripper_articulation_id,
         arm_mount_link, gripper_mount_link?, offset?} -> merged articulation
  4. POST /articulations/{arm_id}/robot/move_j {q}      (radians) -> blocks until done
  5. POST /articulations/{grip_id}/gripper/close / open -> blocks until done

``arm_mount_link`` is the arm's flange (a RigidBodyAPI link or a Site, e.g. UR
``wrist_3_link``), NOT an empty frame like ``tool0`` / ``flange``. Omit
``gripper_mount_link`` (the default here) to let the bridge auto-discover the
gripper's base link -- attaching to the wrong body silently breaks the merge.

Run:  python attach_tool.py
      python attach_tool.py --arm-mount-link wrist_3_link --gripper-mount-link base_link

Requires the ``requests`` package (``pip install requests``).
"""

import argparse
import math

import requests

try:
    import telekinesis_urdfs
except ImportError as e:
    raise ImportError(
        "telekinesis-urdfs is not installed. "
        "Install it from: https://github.com/telekinesis-ai/telekinesis-urdfs"
    ) from e

HOST = "127.0.0.1"
PORT = 8765

ARM_PRIM_PATH = "/World/ur10e"
GRIPPER_PRIM_PATH = "/World/Robotiq_2F_85_edit"

# Links to join the two robots at. The arm mount is its flange (a Link or Site).
# The gripper mount is left None so the bridge auto-discovers the gripper's base
# link (its articulation root) -- pass --gripper-mount-link to override.
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

# Importing/assembling and the blocking moves can each take a while server-side,
# so give the HTTP client a generous timeout (over the bridge's ~30 s motion cap).
DEFAULT_TIMEOUT = 120.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def run(base, arm_prim, arm_urdf, gripper_prim, gripper_urdf, arm_mount_link, gripper_mount_link, offset):
    # 1) Create the arm (urdf_path imports it if arm_prim isn't in the stage).
    arm = _request(
        base, "PUT", "/articulations",
        {"prim_path": arm_prim, "device_type": "robot", "urdf_path": arm_urdf},
    )
    arm_id = arm["articulation_id"]
    print(f"created arm: articulation_id={arm_id} prim_path={arm['prim_path']} dof={arm['num_dof']}")

    # 2) Create the gripper.
    gripper = _request(
        base, "PUT", "/articulations",
        {"prim_path": gripper_prim, "device_type": "gripper", "urdf_path": gripper_urdf},
    )
    gripper_id = gripper["articulation_id"]
    print(f"created gripper: articulation_id={gripper_id} prim_path={gripper['prim_path']}")

    # 3) Attach the gripper onto the arm -> one shared articulation.
    merged = _request(
        base, "POST", f"/articulations/{arm_id}/robot/attach_tool",
        {
            "gripper_articulation_id": gripper_id,
            "arm_mount_link": arm_mount_link,
            "gripper_mount_link": gripper_mount_link,
            "offset": offset,
        },
    )
    print(f"attached: shared articulation {merged['articulation']} num_dof={merged['num_dof']}")
    print(f"  mounts: arm={merged['arm_mount_link']} gripper={merged['gripper_mount_link']} (resolved)")
    print(f"  arm drives {merged['robot']['num_dof']} dof: {merged['robot']['dof_names']}")

    # 4) Move the arm with the same 6-value payload as a standalone arm.
    print(f"move_j target (deg): {TARGET_DEG}")
    status = _request(
        base, "POST", f"/articulations/{arm_id}/robot/move_j",
        {"q": [math.radians(d) for d in TARGET_DEG]},
    )
    print(f"  done={status['done']} (max_error={status['max_error']:.2e} rad)")

    # 5) Close then open the gripper on the shared rig.
    for label in ("close", "open"):
        print(f"gripper {label}")
        status = _request(base, "POST", f"/articulations/{gripper_id}/gripper/{label}")
        print(f"  done (reached={status['reached']} fraction={status['fraction']:.3f})")



def main():
    parser = argparse.ArgumentParser(description="Attach a gripper onto an arm via the Isaac Sim bridge.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--arm-prim", default=ARM_PRIM_PATH, help="prim path to import/bind the arm at")
    parser.add_argument("--gripper-prim", default=GRIPPER_PRIM_PATH, help="prim path of the gripper in the stage")
    parser.add_argument("--arm-urdf", default=ARM_URDF_PATH, help="path to the arm URDF to import")
    parser.add_argument("--gripper-urdf", default=None, help="optional URDF to import the gripper from")
    parser.add_argument("--arm-mount-link", default=ARM_MOUNT_LINK, help="flange link (or Site) on the arm")
    parser.add_argument(
        "--gripper-mount-link",
        default=GRIPPER_MOUNT_LINK,
        help="gripper base link to mount at (omit to auto-discover the gripper's root link)",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"bridge: {base}  articulations: {_request(base, 'GET', '/articulations')}")
    run(
        base,
        args.arm_prim, args.arm_urdf,
        args.gripper_prim, args.gripper_urdf,
        args.arm_mount_link, args.gripper_mount_link,
        ATTACH_OFFSET,
    )


if __name__ == "__main__":
    main()
