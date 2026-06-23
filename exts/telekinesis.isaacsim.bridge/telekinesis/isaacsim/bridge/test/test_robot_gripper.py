"""
Standalone bridge test: robot AND gripper.

Pure stdlib (socket + JSON), no synapse / no Isaac Sim.

Robot:   read joints -> set joints -> read joints.
Gripper: read -> close -> read -> open -> read.

Run from any Python:  python test_robot_gripper.py
"""

import json
import math
import socket

HOST = "127.0.0.1"
CONNECTION_PORT = 8765
ROBOT_PRIM_PATH = "/World/ur10e"
GRIPPER_PRIM_PATH = "/World/Robotiq_2F_85_edit"

TARGET_DEG = [90.0, -90.0, 0.0, 0.0, 90.0, 0.0]
MOVE_WAIT_FINISHED = 1  # block until the gripper reaches the target


def request(sock, message_type, **params):
    """Send one newline-JSON request and return the result dict (raise on error)."""
    sock.sendall((json.dumps({"type": message_type, **params}) + "\n").encode())
    buffer = b""
    while b"\n" not in buffer:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("bridge closed the connection")
        buffer += chunk
    response = json.loads(buffer.split(b"\n", 1)[0].decode())
    if not response.get("ok"):
        raise RuntimeError(response.get("error", "request failed"))
    return response["result"]


def connect_device(prim_path, device_type):
    """Handshake on the connection server; return a socket to the device port + its info."""
    connection = socket.create_connection((HOST, CONNECTION_PORT))
    info = request(connection, "connect", prim_path=prim_path, device_type=device_type, urdf_path=None)
    connection.close()
    print(f"connected {device_type}: prim_path={info['prim_path']} port={info['port']}")
    return socket.create_connection((HOST, info["port"])), info


def robot_joints_deg(sock):
    """Current robot joint positions in degrees (wire is radians)."""
    return [round(math.degrees(q), 3) for q in request(sock, "get_state")["q"]]


def gripper_close(sock):
    """Close the gripper fully; return the status int."""
    return request(sock, "gripper_close", move_mode=MOVE_WAIT_FINISHED)["status"]


def gripper_open(sock):
    """Open the gripper fully; return the status int."""
    return request(sock, "gripper_open", move_mode=MOVE_WAIT_FINISHED)["status"]


def gripper_fraction(sock):
    """Current gripper closed-ness fraction (0.0 open .. 1.0 closed)."""
    return round(request(sock, "gripper_state")["fraction"], 3)


def main():
    # --- robot: read -> set -> read ---
    robot, info = connect_device(ROBOT_PRIM_PATH, "robot")
    print(f"num_dof={info['num_dof']} dof_names={info['dof_names']}")
    print(f"joints before (deg): {robot_joints_deg(robot)}")
    print(f"set_joint target (deg): {TARGET_DEG}")
    request(robot, "move_j", q=[math.radians(d) for d in TARGET_DEG], asynchronous=False)
    print(f"joints after  (deg): {robot_joints_deg(robot)}")
    robot.close()

    # --- gripper: read -> close -> read -> open -> read ---
    gripper, _ = connect_device(GRIPPER_PRIM_PATH, "gripper")
    print(f"gripper fraction (start): {gripper_fraction(gripper)}")
    gripper_close(gripper)
    print(f"gripper fraction (closed): {gripper_fraction(gripper)}")
    gripper_open(gripper)
    print(f"gripper fraction (opened): {gripper_fraction(gripper)}")
    gripper.close()

    print("disconnected")


if __name__ == "__main__":
    main()
