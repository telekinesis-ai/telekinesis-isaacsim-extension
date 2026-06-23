# SPDX-License-Identifier: Apache-2.0
"""
Per-device server: owns one bound articulation device (a RobotArticulation or a
GripperArticulation) and a JsonLineServer dedicated to it on an OS-assigned port.
The connection server spins one of these up per `connect` and hands its port back
to the client, which then talks to it directly for moves / state.
"""

from .json_server import JsonLineServer


class ArticulationServer:
    """Binds a device's async `handle` dispatcher to its own JsonLineServer."""

    def __init__(self, device, host="127.0.0.1"):
        # `device` exposes `.prim_path` and async `.handle(request) -> result`.
        self.device = device
        self._server = JsonLineServer(device.handle, host=host, port=0, name=device.prim_path)

    async def start(self):
        """Start the dedicated server; returns its OS-assigned port."""
        return await self._server.start()

    @property
    def port(self):
        return self._server.port

    def stop(self):
        self._server.stop()
