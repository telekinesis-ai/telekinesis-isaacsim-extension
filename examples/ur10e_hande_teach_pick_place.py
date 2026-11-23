"""Pick objects in Isaac Sim at poses taught with Add Pose and place them in a row.

Teach the poses in Isaac Sim first, with the UR10e and its Hand-E gripper in the stage:
right-click the gripper and choose Telekinesis Isaac Sim Bridge > Add Pose > From Tool. Name the
poses pick_pose, pick_pose_01, pick_pose_02, ... and place each one on an object, and name one
more start_place_pose and place it where the first object goes. Then, with Isaac Sim and the
Telekinesis Isaac Sim Server running, run:
    python examples/ur10e_hande_teach_pick_place.py

The objects are picked in name order of the pick poses, and each is placed at start_place_pose
shifted by PLACE_STEP_Y metres in y for every object already placed.
"""

import requests
from telekinesis.synapse.robots.manipulators import universal_robots
from telekinesis.synapse.tools.parallel_grippers import robotiq

BRIDGE_URL = "http://127.0.0.1:8766"
ROBOT_PRIM_PATH = "/World/vention_table/robot_mount/UR10e_assembly/UR10e"
GRIPPER_PRIM_PATH = "/World/vention_table/robot_mount/UR10e_assembly/Hand_E"

PICK_POSE_PREFIX = "pick_pose"
PLACE_START_POSE_NAME = "start_place_pose"
PLACE_STEP_Y = 0.04  # m, shift in y of each further object's place pose
APPROACH_OFFSET_Z = 0.1  # m, pre-pick and pre-place distance above the poses

# Joint angles in degrees.
HOME_JOINTS = [-90, -90, 90, -90, -90, 0]
INTERMEDIATE_JOINTS = [-45, -90, 90, -90, -90, 0]


def taught_poses(prefix: str) -> list[list[float]]:
    """Return the poses of the robot's taught poses whose names start with ``prefix``, in name
    order.

    Each pose is ``[x, y, z, rx, ry, rz]`` in the robot's base frame, metres and XYZ Euler
    degrees: where the robot's flange has to go for the gripper to stand where the pose does.
    """
    response = requests.get(
        f"{BRIDGE_URL}/teaching/poses",
        params={"robot_prim_path": ROBOT_PRIM_PATH},
        timeout=30,
    )
    if not response.ok:
        raise SystemExit(f"Listing the poses failed: {response.json()['detail']}")
    matching_poses = sorted(
        (pose for pose in response.json() if pose["name"].startswith(prefix)),
        key=lambda pose: pose["name"],
    )
    unreachable = [pose["name"] for pose in matching_poses if not pose["reachable"]]
    if unreachable:
        raise SystemExit(f"The robot cannot reach the poses {unreachable}.")
    return [pose["pose"] for pose in matching_poses]


def main() -> None:
    """Read the taught poses, pick each object and place it in a row from the start pose."""
    pick_poses = taught_poses(PICK_POSE_PREFIX)
    place_start_poses = taught_poses(PLACE_START_POSE_NAME)
    if not pick_poses or len(place_start_poses) != 1:
        raise SystemExit(
            f"Found {len(pick_poses)} {PICK_POSE_PREFIX}* and {len(place_start_poses)} "
            f"{PLACE_START_POSE_NAME} poses; teach at least one pick pose and exactly "
            f"one {PLACE_START_POSE_NAME}."
        )

    robot = universal_robots.UniversalRobotsUR10E(name="ur10e")
    gripper = robotiq.RobotiqHandE()

    robot.connect(simulation_prim_path=ROBOT_PRIM_PATH)
    gripper.connect(simulation_prim_path=GRIPPER_PRIM_PATH)
    robot.attach_tool(gripper)

    try:
        for index, pick_pose in enumerate(pick_poses):
            place_pose = list(place_start_poses[0])
            place_pose[1] += index * PLACE_STEP_Y
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
