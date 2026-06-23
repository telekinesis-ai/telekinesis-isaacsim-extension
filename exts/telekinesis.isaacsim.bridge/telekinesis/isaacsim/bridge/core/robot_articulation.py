# SPDX-License-Identifier: Apache-2.0
"""
Robot arm device: binds one SingleArticulation and drives it in joint space.

Everything here is in native Isaac units (radians). The per-device server's
request handler calls these coroutines directly and awaits completion (the
`async def main()` blocking style) -- no background worker, no command queue.
"""

import asyncio
import time

import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

from ..comm.protocol import GET_STATE, HANDSHAKE, MOVE_J

_REACH_TOLERANCE_RAD = 1e-4
_BIND_RETRIES = 60


class RobotArticulation:
    """Binds an arm articulation at ``prim_path`` and moves its joints (radians)."""

    def __init__(self, prim_path, name="robot"):
        self.prim_path = prim_path
        self._name = name
        self._articulation = None
        self.num_dof = 0
        self.dof_names = []
        self._move_lock = asyncio.Lock()
        self._start_time = time.monotonic()

    async def bind(self):
        """(Re)initialize the articulation against the current physics view.

        Safe to call repeatedly: reuses the handle and just re-initializes, which
        is needed when the timeline was stopped/replayed between client sessions
        (a stale handle returns None from get_joint_positions). Only reports bound
        once joint state actually reads back non-None.
        """
        omni.timeline.get_timeline_interface().play()
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()

        for _ in range(_BIND_RETRIES):
            try:
                if self._articulation is None:
                    self._articulation = SingleArticulation(prim_path=self.prim_path, name=self._name)
                self._articulation.initialize()
                if self._articulation.num_dof and self._articulation.get_joint_positions() is not None:
                    self.num_dof = self._articulation.num_dof
                    self.dof_names = list(self._articulation.dof_names)
                    print(f"[bridge] bound robot {self.prim_path}: {self.num_dof} dof {self.dof_names}")
                    return
            except Exception:
                pass
            await app.next_update_async()

        raise RuntimeError(f"articulation at {self.prim_path} did not become valid")

    async def move_j(self, q_rad, asynchronous=False):
        """Drive all joints to ``q_rad``; block until reached unless ``asynchronous``."""
        target = np.asarray(q_rad, dtype=float)
        if target.shape != (self.num_dof,):
            raise ValueError(f"expected {self.num_dof} joint positions, got {target.shape[0]}")

        async with self._move_lock:
            self._articulation.apply_action(
                ArticulationAction(
                    joint_positions=target,
                    joint_indices=list(range(self.num_dof)),
                )
            )
            if asynchronous:
                return {}

            app = omni.kit.app.get_app()
            while True:
                await app.next_update_async()
                current = self._articulation.get_joint_positions()
                if current is not None and np.max(np.abs(current - target)) < _REACH_TOLERANCE_RAD:
                    break
        return {}

    def get_state(self):
        """Snapshot of joint positions / velocities / torques (rad) + a timestamp."""
        velocities = self._articulation.get_joint_velocities()
        efforts = self._articulation.get_measured_joint_efforts()
        return {
            "q": self._articulation.get_joint_positions().tolist(),
            "qd": velocities.tolist() if velocities is not None else [0.0] * self.num_dof,
            "torque": efforts.tolist() if efforts is not None else [0.0] * self.num_dof,
            "timestamp": time.monotonic() - self._start_time,
        }

    def handshake(self):
        return {"num_dof": self.num_dof, "dof_names": self.dof_names, "state": self.get_state()}

    async def handle(self, request):
        """Route a wire request to the matching method."""
        message_type = request["type"]
        if message_type == MOVE_J:
            return await self.move_j(request["q"], request.get("asynchronous", False))
        if message_type == GET_STATE:
            return self.get_state()
        if message_type == HANDSHAKE:
            return self.handshake()
        raise ValueError(f"robot does not handle '{message_type}'")
