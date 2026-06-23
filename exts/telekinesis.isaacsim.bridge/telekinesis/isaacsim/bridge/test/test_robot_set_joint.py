"""
Standalone bridge test: connect -> set joint -> disconnect (robot only).

Pure stdlib (socket + JSON), no synapse / no Isaac Sim. Talks directly to the
telekinesis.isaacsim.bridge extension running inside Isaac Sim.

Flow:
  1. Open the connection server (127.0.0.1:8765) and `connect` to a robot prim.
     The reply carries the dedicated per-device port + num_dof/dof_names/state.
  2. Open that per-device port and send a `move_j` (joint targets in RADIANS).
  3. Close the sockets.

Run from any Python:  python test_robot_set_joint.py
"""

import json
import math
import socket

HOST = "127.0.0.1"
CONNECTION_PORT = 8765
ROBOT_PRIM_PATH = "/World/ur10e2"

# Joint target (degrees here for readability; converted to radians on the wire).
TARGET_DEG = [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0]
TARGET_DEG2 = [90.0, -90.0, 0.0, 0.0, 90.0, 0.0]



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


def main():
    robot, info = connect_device(ROBOT_PRIM_PATH, "robot")
    print(f"num_dof={info['num_dof']} dof_names={info['dof_names']}")

    target_rad = [math.radians(d) for d in TARGET_DEG]
    print(f"set_joint target (deg): {TARGET_DEG}")
    request(robot, "move_j", q=target_rad, asynchronous=False)
    print("move_j done (arm reached target)")

    target_rad = [math.radians(d) for d in TARGET_DEG2]
    print(f"set_joint target (deg): {TARGET_DEG2}")
    request(robot, "move_j", q=target_rad, asynchronous=False)
    print("move_j done (arm reached target)")

    target_rad = [math.radians(d) for d in TARGET_DEG]
    print(f"set_joint target (deg): {TARGET_DEG}")
    request(robot, "move_j", q=target_rad, asynchronous=False)
    print("move_j done (arm reached target)")




    robot.close()
    print("disconnected")


if __name__ == "__main__":
    main()
