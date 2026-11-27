"""Pick objects in Isaac Sim at poses taught with Add Pose, passing through intermediate joint
positions taught with Add Pose > From Joint Positions, and place each at its own taught place
pose.

Build the scene and teach the poses as in the Build the Scene From GUI tutorial, with the UR10e
and its Hand-E gripper in the stage:
- Add Pose > From Tool or From TCP: pick_pose, pick_pose_01, pick_pose_02, ... on the objects,
  in pick order, and place_pose, place_pose_01, place_pose_02, ... where they go, one per pick
  pose. Each pose may place any TCP of the robot; at least one must place tool_base (From Tool,
  or Convert to Tool Pose), see add_taught_tcps.
- Add Pose > From Joint Positions: intermediate_joint_positions, the arm between the table
  and the place area.

With Isaac Sim and the Telekinesis Isaac Sim Server running, run:
    python examples/ur10e_hande_gui_scene_pick_place.py

The poses are read from the bridge: the pick and place poses as [x, y, z, rx, ry, rz] of the
TCP each places, in the robot's base frame (metres and extrinsic XYZ Euler degrees), and
intermediate_joint_positions as the joint positions it stores. The robot is given every TCP
the poses place, and each pick and place moves the TCP of its pose. The object picked at the
n-th pick pose, in name order, is placed at the n-th place pose.
"""

import math

import requests
from telekinesis.synapse.robots.manipulators import universal_robots
from telekinesis.synapse.tools.parallel_grippers import robotiq
from telekinesis.tf import tfutils

BRIDGE_URL = "http://127.0.0.1:8766"
ROBOT_PRIM_PATH = "/World/vention_table/robot_mount/UR10e_assembly/UR10e"
GRIPPER_PRIM_PATH = "/World/vention_table/robot_mount/UR10e_assembly/Hand_E"
PICK_POSE_PREFIX = "pick_pose"
PLACE_POSE_PREFIX = "place_pose"
INTERMEDIATE_POSE_NAME = "intermediate_joint_positions"

APPROACH_OFFSET_Z = 0.1  # m, pre-pick and pre-place distance above the poses

# Joint angles in degrees.
HOME_JOINTS = [-90, -90, 90, -90, -90, 0]


def get_isaacsim_poses() -> tuple[list[dict], list[dict], list[float]]:
    """Return the pick poses and the place poses, each in name order as listed by the bridge's
    GET /teaching/poses, and the intermediate joint positions in degrees, as taught in Isaac
    Sim."""
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

    pick_poses = [pose for pose in poses if pose["name"].startswith(PICK_POSE_PREFIX)]
    place_poses = [pose for pose in poses if pose["name"].startswith(PLACE_POSE_PREFIX)]
    intermediate_poses = [pose for pose in poses if pose["name"] == INTERMEDIATE_POSE_NAME]
    problems = []
    if not pick_poses:
        problems.append(
            f"No pick pose: right-click the robot > Telekinesis Isaac Sim Bridge > Add Pose > "
            f"From Tool, move the pose onto the first object and name it {PICK_POSE_PREFIX}."
        )
    if len(place_poses) != len(pick_poses):
        problems.append(
            f"{len(pick_poses)} pick poses but {len(place_poses)} place poses: every pick pose "
            f"needs one place pose. Teach, rename or delete poses so the counts match; the n-th "
            f"{PICK_POSE_PREFIX}* by name goes to the n-th {PLACE_POSE_PREFIX}*."
        )
    if len(intermediate_poses) != 1:
        problems.append(
            f"{len(intermediate_poses)} poses named {INTERMEDIATE_POSE_NAME}, exactly one is "
            f"needed: right-click the robot > Telekinesis Isaac Sim Bridge > Add Pose > From "
            f"Joint Positions, or set INTERMEDIATE_POSE_NAME to the name you gave it."
        )
    if problems:
        used = pick_poses + place_poses + intermediate_poses
        unused = [pose["name"] for pose in poses if pose not in used]
        if unused:
            problems.append(
                f"These poses match no name the script looks for and are ignored: {unused}. "
                f"Rename one to {PICK_POSE_PREFIX}_{len(pick_poses):02d} or "
                f"{PLACE_POSE_PREFIX}_{len(place_poses):02d} if it is meant as the next pick or "
                f"place pose."
            )
        raise SystemExit(
            f"The poses of {ROBOT_PRIM_PATH} are not enough to run:\n"
            f"  pick poses ({PICK_POSE_PREFIX}*): {[pose['name'] for pose in pick_poses]}\n"
            f"  place poses ({PLACE_POSE_PREFIX}*): {[pose['name'] for pose in place_poses]}\n"
            f"  intermediate joint positions ({INTERMEDIATE_POSE_NAME}): "
            f"{[pose['name'] for pose in intermediate_poses]}\n"
            + "".join(f"- {problem}\n" for problem in problems)
        )

    # The bridge stores radians; set_joint_positions takes degrees.
    return (
        pick_poses,
        place_poses,
        [math.degrees(position) for position in intermediate_poses[0]["joint_positions"]],
    )


def get_tcp_in_tool0(robot: universal_robots.UniversalRobotsUR10E, pose: dict) -> list[float]:
    """Return the TCP a taught pose places in tool0, [x, y, z, rx, ry, rz] in metres and
    extrinsic XYZ Euler degrees, as it stood in Isaac Sim when the pose was solved.

    The pose is the TCP in the robot's base frame, and forward kinematics at the pose's stored
    joints gives tool0 in the same frame, so tool0^-1 * pose is the TCP in tool0.

    Args:
        robot: The robot the pose was taught for.
        pose: A reachable pose from GET /teaching/poses.
    """
    joints = [math.degrees(position) for position in pose["joint_positions"]]
    base_T_tool0 = tfutils.pose_to_transformation_matrix(
        robot.forward_kinematics(joints, robot.default_tcp), "deg"
    )
    base_T_tcp = tfutils.pose_to_transformation_matrix(pose["pose"], "deg")
    tool0_T_tcp = tfutils.invert_transformation_matrix(base_T_tool0) @ base_T_tcp
    return [float(value) for value in tfutils.transformation_matrix_to_pose(tool0_T_tcp, "deg")]


def add_taught_tcps(
    robot: universal_robots.UniversalRobotsUR10E, gripper: robotiq.RobotiqHandE, poses: list[dict]
) -> None:
    """Attach the gripper and add every other TCP the poses place to the robot, each where it
    stood in Isaac Sim, so each pose moves its own TCP.

    tool_base, the gripper's base, is the offset attach_tool needs. A physics joint needs a
    rigid body on both sides, and tool0, like the flange, is an empty frame with none: Isaac Sim
    cannot fix the gripper to it, so Add Tool fixes the gripper to the robot's last link
    (wrist_3_link) wherever it was placed, on the coupler here. Where the gripper sits in tool0
    is therefore stored nowhere the robot can read, and is worked out from a pose of tool_base.
    TCPs added with Add TCP are fixed to tool0 and are added with add_tcp. The default TCP needs
    nothing.

    Raises:
        SystemExit: If no pose places tool_base.
    """
    tcp_offsets = {}
    for pose in poses:
        # A pose taught before poses stored their TCP places the default TCP.
        tcp_name = pose["tcp"] or robot.default_tcp
        if tcp_name != robot.default_tcp and tcp_name not in tcp_offsets:
            tcp_offsets[tcp_name] = get_tcp_in_tool0(robot, pose)
    if robot.tool_base_tcp_name not in tcp_offsets:
        raise SystemExit(
            f"No pose places {robot.tool_base_tcp_name}, so where the gripper sits is unknown; "
            "teach one with Add Pose > From Tool, or right-click one > Convert to Tool Pose."
        )
    robot.attach_tool(gripper, transform=tcp_offsets.pop(robot.tool_base_tcp_name))
    for tcp_name, tcp_in_tool0 in tcp_offsets.items():
        robot.add_tcp(tcp_name, tcp_in_tool0, set_active=False)


def main() -> None:
    """Get the poses, then pick each object and place it at its place pose."""
    pick_poses, place_poses, intermediate_joints = get_isaacsim_poses()

    robot = universal_robots.UniversalRobotsUR10E(name="ur10e")
    gripper = robotiq.RobotiqHandE()
    robot.connect(simulation_prim_path=ROBOT_PRIM_PATH)
    gripper.connect(simulation_prim_path=GRIPPER_PRIM_PATH)
    add_taught_tcps(robot, gripper, pick_poses + place_poses)

    try:
        for index, (pick_pose, place_pose) in enumerate(zip(pick_poses, place_poses)):
            print(f"Object {index + 1}: picking")
            robot.set_joint_positions(HOME_JOINTS)
            robot.active_tcp = pick_pose["tcp"] or robot.default_tcp
            robot.pick(pick_pose["pose"], APPROACH_OFFSET_Z)

            print(f"Object {index + 1}: placing")
            robot.set_joint_positions(intermediate_joints)
            robot.active_tcp = place_pose["tcp"] or robot.default_tcp
            robot.place(place_pose["pose"], APPROACH_OFFSET_Z)
        robot.set_joint_positions(HOME_JOINTS)
    finally:
        gripper.disconnect()
        robot.disconnect()
        robot.shutdown()


if __name__ == "__main__":
    main()
