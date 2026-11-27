"""Pick objects at poses taught with Add Pose and place them in a row, in Isaac Sim or on the
real robot.

Teach the poses in Isaac Sim first, with the UR10e and its Hand-E gripper in the stage:
right-click the gripper and choose Telekinesis Isaac Sim Bridge > Add Pose > From Tool. Name the
poses pick_pose, pick_pose_01, pick_pose_02, ... and place each one on an object, and name one
more start_place_pose and place it where the first object goes.

In Isaac Sim (RUN_ON_REAL_ROBOT = False), the poses are read from the bridge. With Isaac Sim and
the Telekinesis Isaac Sim Server running, run:
    python examples/ur10e_hande_teach_pick_place.py

On the real robot (RUN_ON_REAL_ROBOT = True), Isaac Sim is not needed: copy each pose's
telekinesis:pose:pose_in_base_frame from Isaac Sim's Property window into PICK_POSES and
PLACE_START_POSE, set ROBOT_IP, and run the same command.

The objects are picked in the order of the pick poses, and each is placed at the place start
pose shifted by PLACE_STEP_Y metres in y for every object already placed. Every pose is
[x, y, z, rx, ry, rz] of the robot's tool_base TCP, the Hand-E's base, in its base frame, metres
and extrinsic XYZ Euler degrees. attach_tool adds that TCP at TOOL_BASE_IN_TOOL0, the Hand-E's
base in tool0 as Add Tool mounted it; if the Hand-E sits differently on the real flange, measure
it and change TOOL_BASE_IN_TOOL0, and the taught poses stay valid.
"""

import requests
from telekinesis.synapse.robots.manipulators import universal_robots
from telekinesis.synapse.tools.parallel_grippers import robotiq

# False: run in Isaac Sim with the poses taught there. True: run on the real robot with the
# poses written below.
RUN_ON_REAL_ROBOT = False

BRIDGE_URL = "http://127.0.0.1:8766"
ROBOT_PRIM_PATH = "/World/UR10e_assembly/UR10e"
GRIPPER_PRIM_PATH = "/World/UR10e_assembly/Hand_E"
PICK_POSE_PREFIX = "pick_pose"
PLACE_START_POSE_NAME = "start_place_pose"

# The real robot's controller; the Hand-E is reached through its URCap on the same controller.
ROBOT_IP = "192.168.1.100"
GRIPPER_IP = "192.168.1.100"
# The taught poses for the real robot, copied from each pose's
# telekinesis:pose:pose_in_base_frame in Isaac Sim, in pick order. Left empty on purpose: the
# script refuses to move the real robot until they are filled in.
PICK_POSES: list[list[float]] = [
    [0.0884, 0.4983, 0.1331, 180, 0, 0],
    [-0.0164, 0.4983, 0.1331, 180, 0, 0],
    [-0.1349, 0.4983, 0.1331, 180, 0, 0],
]
PLACE_START_POSE: list[float] = [-0.6837, -0.0609, 0.1331, 180, 0, 0]
# The Hand-E's base in tool0, [x, y, z, rx, ry, rz] in metres and XYZ Euler degrees: on the
# coupler, as Add Tool mounted it in Isaac Sim.
TOOL_BASE_IN_TOOL0 = [0.0, 0.0, 0.0098, 0.0, 0.0, 0.0]

PLACE_STEP_Y = 0.04  # m, shift in y of each further object's place pose
APPROACH_OFFSET_Z = 0.1  # m, pre-pick and pre-place distance above the poses

# Joint angles in degrees.
HOME_JOINTS = [-90, -90, 90, -90, -90, 0]
INTERMEDIATE_JOINTS = [-45, -90, 90, -90, -90, 0]


def get_real_robot_poses() -> tuple[list[list[float]], list[float]]:
    """Return the pick poses and the place start pose written in this script."""
    if not PICK_POSES or len(PLACE_START_POSE) != 6:
        raise SystemExit(
            "Fill in PICK_POSES and PLACE_START_POSE with the poses' "
            "telekinesis:pose:pose_in_base_frame values from Isaac Sim first."
        )
    return PICK_POSES, PLACE_START_POSE


def get_isaacsim_poses() -> tuple[list[list[float]], list[float]]:
    """Return the pick poses, in name order, and the place start pose taught in Isaac Sim."""
    response = requests.get(
        f"{BRIDGE_URL}/teaching/poses",
        params={"robot_prim_path": ROBOT_PRIM_PATH},
        timeout=30,
    )
    if not response.ok:
        raise SystemExit(f"Listing the poses failed: {response.json()['detail']}")
    poses = sorted(response.json(), key=lambda pose: pose["name"])
    unreachable = [pose["name"] for pose in poses if not pose["reachable"]]
    if unreachable:
        raise SystemExit(f"The robot cannot reach the poses {unreachable}.")
    tool_base_tcp_name = universal_robots.UniversalRobotsUR10E.tool_base_tcp_name
    not_tool_base = [pose["name"] for pose in poses if pose["tcp"] != tool_base_tcp_name]
    if not_tool_base:
        raise SystemExit(
            f"The poses {not_tool_base} place another TCP than {tool_base_tcp_name}; "
            "teach them again with Add Pose > From Tool."
        )
    pick_poses = [pose["pose"] for pose in poses if pose["name"].startswith(PICK_POSE_PREFIX)]
    place_start_poses = [pose["pose"] for pose in poses if pose["name"] == PLACE_START_POSE_NAME]
    if not pick_poses or len(place_start_poses) != 1:
        raise SystemExit(
            f"Found {len(pick_poses)} {PICK_POSE_PREFIX}* and {len(place_start_poses)} "
            f"{PLACE_START_POSE_NAME} poses; teach at least one pick pose and exactly "
            f"one {PLACE_START_POSE_NAME}."
        )
    return pick_poses, place_start_poses[0]


def main() -> None:
    """Get the poses, pick each object and place it in a row from the place start pose."""
    if RUN_ON_REAL_ROBOT:
        pick_poses, place_start_pose = get_real_robot_poses()
    else:
        pick_poses, place_start_pose = get_isaacsim_poses()

    robot = universal_robots.UniversalRobotsUR10E(name="ur10e")
    gripper = robotiq.RobotiqHandE()

    if RUN_ON_REAL_ROBOT:
        robot.connect(ip=ROBOT_IP)
        gripper.connect(ip=GRIPPER_IP)
    else:
        robot.connect(simulation_prim_path=ROBOT_PRIM_PATH)
        gripper.connect(simulation_prim_path=GRIPPER_PRIM_PATH)
    robot.attach_tool(gripper, transform=TOOL_BASE_IN_TOOL0)
    robot.active_tcp = robot.tool_base_tcp_name

    try:
        for index, pick_pose in enumerate(pick_poses):
            place_pose = list(place_start_pose)
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
