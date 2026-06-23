# SPDX-License-Identifier: Apache-2.0
"""
The single well-known entry point of the bridge.

A client `connect`s by naming a prim path, a device kind ("robot" / "gripper"),
and optionally a URDF to import if the prim isn't already in the stage. The
connection server resolves the prim, binds the device, spins up a dedicated
ArticulationServer on a fresh (OS-assigned) port, and replies with that port plus
the device's initial info. The client then talks to that port directly.

Devices are cached by resolved prim path, so reconnecting to the same prim
returns the existing server instead of binding twice.
"""

import asyncio

import omni.usd

from ..core.gripper_articulation import GripperArticulation
from ..core.robot_articulation import RobotArticulation
from ..core.urdf_loader import import_urdf_at
from .articulation_server import ArticulationServer
from .json_server import JsonLineServer
from .protocol import CONNECT, DISCONNECT

DEVICE_ROBOT = "robot"
DEVICE_GRIPPER = "gripper"


class ConnectionServer:
    """Listens on the well-known port and spawns one ArticulationServer per device."""

    def __init__(self, host="127.0.0.1", port=8765):
        self._host = host
        self._connection = JsonLineServer(self._handle_connect, host=host, port=port, name="connection")
        self._device_servers = {}  # resolved prim_path -> ArticulationServer
        self._serve_task = None

    def start(self):
        """Schedule the connection listener on Isaac Sim's asyncio loop."""
        self._serve_task = asyncio.ensure_future(self._connection.start())

    async def _handle_connect(self, request):
        message_type = request.get("type")

        if message_type == DISCONNECT:
            device_server = self._device_servers.pop(request["prim_path"], None)
            if device_server is not None:
                device_server.stop()
            return {"disconnected": request["prim_path"]}

        if message_type != CONNECT:
            raise ValueError(f"connection server only handles '{CONNECT}' / '{DISCONNECT}', got '{message_type}'")

        device_type = request["device_type"]
        requested_prim_path = request["prim_path"]
        urdf_path = request.get("urdf_path")

        # Resolve the prim: use it if it's already in the stage, otherwise import
        # the URDF at that path. (Importing requires the timeline stopped, which
        # import_urdf_at handles; the device then plays it again on bind.)
        stage = omni.usd.get_context().get_stage()
        existing = stage.GetPrimAtPath(requested_prim_path)
        if existing and existing.IsValid():
            prim_path = requested_prim_path
        elif urdf_path:
            prim_path = import_urdf_at(stage, urdf_path, requested_prim_path)
        else:
            raise ValueError(f"'{requested_prim_path}' is not in the stage and no urdf_path was given to load it")

        # Create the device + its dedicated server once; reuse on reconnect.
        if prim_path not in self._device_servers:
            if device_type == DEVICE_ROBOT:
                device = RobotArticulation(prim_path)
            elif device_type == DEVICE_GRIPPER:
                device = GripperArticulation(prim_path)
            else:
                raise ValueError(f"unknown device_type '{device_type}' (expected '{DEVICE_ROBOT}' / '{DEVICE_GRIPPER}')")

            device_server = ArticulationServer(device, host=self._host)
            await device_server.start()
            self._device_servers[prim_path] = device_server

        # (Re)bind on every connect: a new client session may follow a timeline
        # stop/replay, which invalidates the cached articulation handle (its
        # state reads return None). bind() reuses the handle and re-initializes.
        device_server = self._device_servers[prim_path]
        await device_server.device.bind()
        return {"prim_path": prim_path, "port": device_server.port, **device_server.device.handshake()}

    def stop(self):
        """Stop every device server and the connection listener."""
        for device_server in self._device_servers.values():
            device_server.stop()
        self._device_servers = {}
        self._connection.stop()
        if self._serve_task is not None:
            self._serve_task.cancel()
            self._serve_task = None
