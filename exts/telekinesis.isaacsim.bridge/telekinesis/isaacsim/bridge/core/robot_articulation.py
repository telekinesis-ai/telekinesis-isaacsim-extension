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
from loguru import logger

# A PhysX position drive settles with a small steady-state offset (gravity, drive
# stiffness, the URDF importer's default gains), so an over-tight position
# tolerance is never satisfied and the move looks "never done". We call a move
# done when either the joints are close to target, OR they have stopped moving
# (settled) within a coarser band -- the drive won't close the gap any further.
_REACH_TOLERANCE_RAD = 5e-3       # ~0.3 deg: target reached cleanly
_SETTLED_VELOCITY_RAD_S = 5e-3    # joints no longer moving
_SETTLED_TOLERANCE_RAD = 5e-2     # ~2.9 deg: accept residual offset once settled
_BIND_RETRIES = 60
# Backstop for the blocking move loop: at ~60 fps this is ~30 s. A move that has
# not reached *or* settled by then returns its last status with done=False rather
# than blocking the request forever.
_MOTION_MAX_FRAMES = 1800


class RobotArticulation:
    """Binds an arm articulation at ``prim_path`` and moves its joints (radians)."""

    def __init__(self, prim_path, name="robot", joint_names=None):
        """Store prim path and initialise joint state; call `bind()` before use."""
        self.prim_path = prim_path
        self._name = name
        self._articulation = None
        self.num_dof = 0
        self.dof_names = []
        # The DOFs this device actually drives, as names. None => all of them (a
        # standalone arm owns every joint). When the arm and a gripper are merged
        # into one articulation, the assembly step overwrites ``joint_names`` with
        # just the arm's joints; ``bind`` then resolves them to ``joint_indices``
        # so move_j/motion_status address only those columns of the shared rig.
        self.joint_names = joint_names
        self.joint_indices = []
        self._move_lock = asyncio.Lock()
        self._start_time = time.monotonic()
        self._target = None  # last commanded joint target (rad); drives motion_status()

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
                    self._resolve_driven_joints()
                    logger.info(f"[bridge] bound robot {self.prim_path}: {self.num_dof} dof {self.dof_names}")
                    return
            except Exception:
                pass
            await app.next_update_async()

        raise RuntimeError(f"articulation at {self.prim_path} did not become valid")

    def _resolve_driven_joints(self):
        """Work out which columns of ``self._articulation`` this device drives.

        By joint NAME, so it survives a merged (assembled) rig's DOF ordering.
        ``joint_names is None`` => a standalone arm that owns every DOF. The
        reported ``dof_names``/``num_dof`` are the driven subset, so ``info`` and
        ``get_state`` match ``move_j`` whether or not a gripper is attached.
        """
        all_names = list(self._articulation.dof_names)
        if self.joint_names is None:
            self.joint_indices = list(range(len(all_names)))
        else:
            self.joint_indices = [all_names.index(n) for n in self.joint_names if n in all_names]
        self.dof_names = [all_names[i] for i in self.joint_indices]
        self.num_dof = len(self.joint_indices)

    def adopt_shared_articulation(self, articulation, joint_names):
        """Re-point this arm at a shared (assembled) articulation.

        After ``attach_tool`` the arm and gripper are one articulation; both
        devices hold this same handle. The arm keeps driving only its own joints
        (``joint_names``), resolved into the merged DOF order. No rebind needed --
        the handle is already initialized; a later reconnect re-resolves by name.
        """
        self._articulation = articulation
        self.joint_names = list(joint_names)
        self._resolve_driven_joints()
        self._target = None
        logger.info(f"[bridge] arm {self.prim_path} on shared articulation: drives {self.dof_names} at {self.joint_indices}")

    async def move_j(self, q_rad):
        """Move all joints to ``q_rad`` and return once the move completes.

        Blocking by design (the ``async def main()`` style the prototype uses):
        applies the action, then awaits ``next_update_async`` in a loop until
        ``motion_status`` reports done. Because this handler runs on Isaac Sim's own
        asyncio loop, each ``next_update_async`` steps one physics frame *and* yields
        control, so other requests keep being served while this one waits -- no
        background worker and no client-side ``/motion`` polling. The ``_move_lock``
        serializes repeat commands to this one device for the whole motion.
        """
        target = np.asarray(q_rad, dtype=float)
        if target.shape != (len(self.joint_indices),):
            raise ValueError(f"expected {len(self.joint_indices)} joint positions, got {target.shape[0]}")

        app = omni.kit.app.get_app()
        async with self._move_lock:
            self._target = target
            self._articulation.apply_action(
                ArticulationAction(
                    joint_positions=target,
                    joint_indices=self.joint_indices,
                )
            )
            status = self.motion_status()
            frames = 0
            while not status["done"] and frames < _MOTION_MAX_FRAMES:
                await app.next_update_async()
                status = self.motion_status()
                frames += 1
        return status

    def motion_status(self):
        """Whether the last commanded move has completed (the ``move_j`` loop's exit test).

        Done when the joints are within ``_REACH_TOLERANCE_RAD`` of target, or
        when they have settled (velocities below ``_SETTLED_VELOCITY_RAD_S``)
        within the coarser ``_SETTLED_TOLERANCE_RAD`` band -- the position drive
        won't reduce a steady-state offset further, so waiting longer is futile.
        The settled check also can't fire before the move starts: until the arm
        has travelled close to target, ``max_error`` exceeds the coarse band.
        """
        current = self._articulation.get_joint_positions()
        if self._target is None or current is None:
            return {"done": True, "max_error": 0.0}
        # Only the driven columns: on a merged rig ``current`` spans every DOF.
        q = current[self.joint_indices]
        max_error = float(np.max(np.abs(q - self._target)))
        reached = max_error < _REACH_TOLERANCE_RAD

        velocities = self._articulation.get_joint_velocities()
        max_speed = float(np.max(np.abs(velocities[self.joint_indices]))) if velocities is not None else 0.0
        settled = max_speed < _SETTLED_VELOCITY_RAD_S and max_error < _SETTLED_TOLERANCE_RAD

        return {
            "done": reached or settled,
            "max_error": max_error,
            "max_speed": max_speed,
            "q": q.tolist(),
            "target": self._target.tolist(),
        }

    def get_state(self):
        """Snapshot of this arm's joint positions / velocities / torques (rad) + a
        timestamp. Reports only the driven columns (``joint_indices``), so a merged
        rig still presents just the arm's joints."""
        positions = self._articulation.get_joint_positions()
        velocities = self._articulation.get_joint_velocities()
        efforts = self._articulation.get_measured_joint_efforts()
        return {
            "q": positions[self.joint_indices].tolist(),
            "qd": velocities[self.joint_indices].tolist() if velocities is not None else [0.0] * self.num_dof,
            "torque": efforts[self.joint_indices].tolist() if efforts is not None else [0.0] * self.num_dof,
            "timestamp": time.monotonic() - self._start_time,
        }

    def info(self):
        """Static description of this device (not a handshake; the bind happens
        in the server's PUT /articulations): dof count, joint names, and current state."""
        return {"num_dof": self.num_dof, "dof_names": self.dof_names, "state": self.get_state()}
