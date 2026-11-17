"""Pick three objects in Isaac Sim and place them in a row.

Start Isaac Sim with the Synapse extension and the UR10e with its Hand-E gripper
in the stage, then run:
    python examples/robot_gripper_isaacsim_pick_place.py

Each object is picked at one of PICK_POSES and placed at PLACE_START shifted by
PLACE_STEP_Y metres in y for every object already placed.
"""

from telekinesis.synapse.robots.manipulators import universal_robots
from telekinesis.synapse.tools.parallel_grippers import robotiq

ROBOT_PRIM_PATH = "/World/vention_table/robot_mount/UR10e_assembly/UR10e"
GRIPPER_PRIM_PATH = "/World/vention_table/robot_mount/UR10e_assembly/Hand_E"

# Cartesian poses [x, y, z, rx, ry, rz] in metres and Euler XYZ degrees, robot base frame.
PICK_POSES = [
    [0.09, 0.509, 0.15, 180, 0, 0],
    [-0.015, 0.509, 0.15, 180, 0, 0],
    [-0.134, 0.509, 0.15, 180, 0, 0],
]
PLACE_START = [-0.6, -0.1, 0.15, 180, 0, 90]
PLACE_STEP_Y = 0.04
APPROACH_OFFSET_Z = 0.1  # m, pre-pick and pre-place distance above the poses

# Motion limits of every move: joint in deg/s and deg/s^2, Cartesian in m/s and m/s^2.
JOINT_SPEED = 30.0
JOINT_ACCELERATION = 40.0
CARTESIAN_SPEED = 0.5
CARTESIAN_ACCELERATION = 0.7

# Joint angles in degrees.
HOME_JOINTS = [-90, -90, 90, -90, -90, 0]
INTERMEDIATE_JOINTS = [-45, -90, 90, -90, -90, 0]


def place_pose(index: int) -> list[float]:
    """Return the place pose for the object with the given index, shifted in y from PLACE_START."""
    pose = list(PLACE_START)
    pose[1] += index * PLACE_STEP_Y
    return pose


def main() -> None:
    robot = universal_robots.UniversalRobotsUR10E(name="ur10e")
    gripper = robotiq.RobotiqHandE()

    robot.connect(simulation_prim_path=ROBOT_PRIM_PATH)
    gripper.connect(simulation_prim_path=GRIPPER_PRIM_PATH)
    robot.attach_tool(gripper)


    try:
        for index, pick_pose in enumerate(PICK_POSES):
            print(f"Object {index + 1}: picking")
            robot.set_joint_positions(HOME_JOINTS)
            robot.pick(pick_pose, APPROACH_OFFSET_Z)

            robot.set_joint_positions(INTERMEDIATE_JOINTS)
            robot.place(place_pose(index), APPROACH_OFFSET_Z)
        robot.set_joint_positions(HOME_JOINTS)
    finally:
        gripper.disconnect()
        robot.disconnect()
        robot.shutdown()


if __name__ == "__main__":
    main()
