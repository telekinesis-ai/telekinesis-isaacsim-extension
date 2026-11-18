"""Pick objects in Isaac Sim at poses taught with ghost objects and place them at others.

Teach the poses in Isaac Sim first, with the UR10e and its Hand-E gripper in the stage:
right-click the robot and choose Connect to Telekinesis, then right-click the gripper and choose
Ghost Object. Name the ghosts PickPose_01, PickPose_02, ... and place each one on an object,
and name others PlacePose_01, PlacePose_02, ... and place each one where an object goes. Then,
with Isaac Sim and the Telekinesis Isaac Sim Server running, run:
    python examples/ur10e_hande_ghost_pick_place.py

The ghosts are paired in name order: the object at PickPose_01 is placed at PlacePose_01, and so
on.
"""

import requests
from telekinesis.synapse.robots.manipulators import universal_robots
from telekinesis.synapse.tools.parallel_grippers import robotiq

BRIDGE_URL = "http://127.0.0.1:8766"
ROBOT_PRIM_PATH = "/World/vention_table/robot_mount/UR10e_assembly/UR10e"
GRIPPER_PRIM_PATH = "/World/vention_table/robot_mount/UR10e_assembly/Hand_E"

PICK_GHOST_PREFIX = "PickPose"
PLACE_GHOST_PREFIX = "PlacePose"
APPROACH_OFFSET_Z = 0.1  # m, pre-pick and pre-place distance above the poses

# Joint angles in degrees.
HOME_JOINTS = [-90, -90, 90, -90, -90, 0]
INTERMEDIATE_JOINTS = [-45, -90, 90, -90, -90, 0]


def ghost_poses(prefix: str) -> list[list[float]]:
    """Return the poses of the robot's ghost objects whose names start with ``prefix``, in name
    order.

    Each pose is ``[x, y, z, rx, ry, rz]`` in the robot's base frame, metres and XYZ Euler
    degrees: where the robot's flange has to go for the gripper to stand where the ghost does.
    """
    response = requests.get(
        f"{BRIDGE_URL}/teaching/ghost-objects",
        params={"robot_prim_path": ROBOT_PRIM_PATH},
        timeout=30,
    )
    if not response.ok:
        raise SystemExit(f"Listing the ghost objects failed: {response.json()['detail']}")
    ghosts = sorted(
        (ghost for ghost in response.json() if ghost["name"].startswith(prefix)),
        key=lambda ghost: ghost["name"],
    )
    unreachable = [ghost["name"] for ghost in ghosts if not ghost["reachable"]]
    if unreachable:
        raise SystemExit(f"The robot cannot reach the ghost objects {unreachable}.")
    return [ghost["pose"] for ghost in ghosts]


def main() -> None:
    """Read the taught ghost poses, then pick and place one object per PickPose/PlacePose pair."""
    pick_poses = ghost_poses(PICK_GHOST_PREFIX)
    place_poses = ghost_poses(PLACE_GHOST_PREFIX)
    if not pick_poses or len(pick_poses) != len(place_poses):
        raise SystemExit(
            f"Found {len(pick_poses)} {PICK_GHOST_PREFIX} and {len(place_poses)} "
            f"{PLACE_GHOST_PREFIX} ghost objects; teach one place pose for every pick pose."
        )

    robot = universal_robots.UniversalRobotsUR10E(name="ur10e")
    gripper = robotiq.RobotiqHandE()

    robot.connect(simulation_prim_path=ROBOT_PRIM_PATH)
    gripper.connect(simulation_prim_path=GRIPPER_PRIM_PATH)
    robot.attach_tool(gripper)

    try:
        for index, (pick_pose, place_pose) in enumerate(zip(pick_poses, place_poses)):
            print(f"Object {index + 1}: picking")
            robot.set_joint_positions(HOME_JOINTS)
            robot.pick(pick_pose, APPROACH_OFFSET_Z)

            print(f"Object {index + 1}: placing")
            robot.set_joint_positions(INTERMEDIATE_JOINTS)
            robot.place(place_pose, APPROACH_OFFSET_Z)
        robot.set_joint_positions(HOME_JOINTS)
    finally:
        gripper.disconnect()
        robot.disconnect()
        robot.shutdown()


if __name__ == "__main__":
    main()
