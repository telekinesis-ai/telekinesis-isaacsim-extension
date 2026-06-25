# SPDX-License-Identifier: Apache-2.0
"""
Generic articulation device: binds one SingleArticulation and drives a chosen
subset of its joints in joint space.

This is the single device the bridge exposes -- there is no separate robot vs
gripper class. The extension stays generic: it applies an ``ArticulationAction``
to whatever DOF indices are currently the device's "driven subset" and reports
whether the move reached its target or stalled. All robot/gripper *semantics*
live in the client:

* A robot drives every DOF: leave the driven subset at its default (all joints)
  and send raw joint angles (radians) to :meth:`set_joint_positions`.
* A gripper drives one actuated joint: the client first reads the driver joint
  (:func:`find_driver_joint`, exposed as a getter), narrows the driven subset to
  it via :meth:`set_driven_joints`, then sends a single angle. The
  fraction<->radians math and open/close convenience are the client's job; the
  joint limits it needs come from :meth:`get_joint_limits`.

Everything here is native Isaac units (radians). The request handler calls these
coroutines directly and awaits completion (the ``async def main()`` blocking
style) -- no background worker, no command queue.
"""

import asyncio
import time

import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
import carb
from pxr import Usd, UsdPhysics

# A PhysX position drive settles with a small steady-state offset (gravity, drive
# stiffness, the URDF importer's default gains), so an over-tight position
# tolerance is never satisfied and a move looks "never done". A move is done when
# the joints are within _REACH_TOLERANCE_RAD of target (reached cleanly), OR when
# they have stopped moving for _SETTLED_FRAMES consecutive frames (stalled --
# either settled at a steady-state offset, or blocked by a grasped object). The
# consecutive-frame requirement keeps the stall test from firing before the drive
# has accelerated the joints. The frame cap is a ~30 s backstop at 60 fps.
_REACH_TOLERANCE_RAD = 5e-3       # ~0.3 deg: target reached cleanly
_SETTLED_VELOCITY_RAD_S = 5e-3    # joints no longer moving
_SETTLED_FRAMES = 5               # consecutive low-velocity frames => stalled
_BIND_RETRIES = 60
_MOTION_MAX_FRAMES = 1800


def find_driver_joint(stage, prim_path, dof_names):
    """Return the gripper's actuated driver joint name from ``dof_names``.

    A single-input gripper has one actuated joint; the rest are mimic joints that
    follow it. The driver is the non-mimic joint, preferring one with a
    UsdPhysics.DriveAPI. Falls back to the first DOF if detection is inconclusive.
    This is a USD/PhysX schema walk a pure HTTP client cannot do, so it stays in
    the extension (exposed as the ``/driver_joint`` getter).
    """
    root = stage.GetPrimAtPath(prim_path)
    if root.IsA(UsdPhysics.Joint):  # importer may return the root joint, not the container
        root = root.GetParent()
    joints = {p.GetName(): p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)}

    fallback = None
    for name in dof_names:
        prim = joints.get(name)
        if prim is None or any("MimicJoint" in s for s in prim.GetAppliedSchemas()):
            continue
        if prim.HasAPI(UsdPhysics.DriveAPI):
            return name
        fallback = fallback or name
    return fallback or dof_names[0]


class Articulation:
    """Binds an articulation at ``prim_path`` and drives its driven joints (radians)."""

    def __init__(self, prim_path, name="articulation", joint_names=None):
        """Store prim path and initialise joint state; call `bind()` before use."""
        self.prim_path = prim_path
        self._name = name
        self._articulation = None
        self.num_dof = 0
        self.dof_names = []
        # The DOFs this device actually drives, as names. None => all of them (the
        # default; a robot owns every joint). The client narrows this to one joint
        # for a gripper via set_driven_joints; the assembly step overwrites it with
        # just the arm's (or gripper's) joints. bind/_resolve_driven_joints turns
        # the names into joint_indices so moves address only those columns.
        self.joint_names = joint_names
        self.joint_indices = []
        self._move_lock = asyncio.Lock()
        self._start_time = time.monotonic()
        self._target = None  # last commanded joint target (rad)

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
                    carb.log_info(f"[bridge] bound articulation {self.prim_path}: {self.num_dof} dof {self.dof_names}")
                    return
            except Exception:
                pass
            await app.next_update_async()

        raise RuntimeError(f"articulation at {self.prim_path} did not become valid")

    def _resolve_driven_joints(self):
        """Work out which columns of ``self._articulation`` this device drives.

        By joint NAME, so it survives a merged (assembled) rig's DOF ordering.
        ``joint_names is None`` => drive every DOF. The reported
        ``dof_names``/``num_dof`` are the driven subset, so ``info``, ``get_state``,
        and ``get_joint_limits`` all match what ``set_joint_positions`` moves.
        """
        all_names = list(self._articulation.dof_names)
        if self.joint_names is None:
            self.joint_indices = list(range(len(all_names)))
        else:
            self.joint_indices = [all_names.index(n) for n in self.joint_names if n in all_names]
        self.dof_names = [all_names[i] for i in self.joint_indices]
        self.num_dof = len(self.joint_indices)

    def set_driven_joints(self, joint_names):
        """Narrow (or widen) the set of joints this device drives, by name.

        The "SET the driving joints" step: a gripper client first reads the driver
        joint (``/driver_joint``) and passes ``[driver_name]`` here, after which the
        device behaves exactly like a robot with one joint. Re-resolves against the
        current handle; forgets any pending move target.
        """
        self.joint_names = list(joint_names)
        self._resolve_driven_joints()
        self._target = None
        carb.log_info(f"[bridge] articulation {self.prim_path} drives {self.dof_names} at {self.joint_indices}")

    def adopt_shared_articulation(self, articulation, joint_names):
        """Re-point this device at a shared (assembled) articulation.

        After ``assemble_robot`` the arm and gripper are one articulation; both
        devices hold this same handle. Each keeps driving only its own joints
        (``joint_names``), resolved into the merged DOF order. No rebind needed --
        the handle is already initialized; a later reconnect re-resolves by name.
        """
        self._articulation = articulation
        self.joint_names = list(joint_names)
        self._resolve_driven_joints()
        self._target = None
        carb.log_info(
            f"[bridge] articulation {self.prim_path} on shared articulation: "
            f"drives {self.dof_names} at {self.joint_indices}"
        )

    async def set_joint_positions(self, positions, indices=None, asynchronous=False):
        """Drive ``positions`` (radians) onto the chosen joint ``indices``.

        ``indices`` defaults to this device's driven subset (``joint_indices``), so
        a robot client sends all joint angles and a gripper client (narrowed to its
        driver) sends one -- neither needs to know the merged DOF order.

        ``asynchronous=True`` applies the action and returns immediately (the client
        polls ``get_state`` / decides "done" itself). ``asynchronous=False`` blocks,
        awaiting ``next_update_async`` in a loop until the joints reach the target or
        stall, then returns the final status with ``reached`` telling the two apart.
        Each ``next_update_async`` steps one physics frame and yields Isaac Sim's
        loop, so other requests keep being served while a blocking move waits. The
        ``_move_lock`` serializes repeat commands to this one device.
        """
        target = np.asarray(positions, dtype=float)
        idx = list(self.joint_indices) if indices is None else list(indices)
        if target.shape != (len(idx),):
            raise ValueError(f"expected {len(idx)} joint positions, got {target.shape[0]}")

        app = omni.kit.app.get_app()
        async with self._move_lock:
            self._target = target
            self._articulation.apply_action(
                ArticulationAction(joint_positions=target, joint_indices=idx)
            )

            if asynchronous:
                # Fire-and-forget: the action is queued; the client owns "done".
                return {"done": False, "reached": False, "applied": True, "target": target.tolist()}

            reached = False
            stalled_frames = 0
            max_error = float("inf")
            q = self._articulation.get_joint_positions()[idx]
            for _ in range(_MOTION_MAX_FRAMES):
                await app.next_update_async()
                q = self._articulation.get_joint_positions()[idx]
                max_error = float(np.max(np.abs(q - target)))
                if max_error < _REACH_TOLERANCE_RAD:
                    reached = True
                    break
                velocities = self._articulation.get_joint_velocities()
                max_speed = float(np.max(np.abs(velocities[idx]))) if velocities is not None else 0.0
                stalled_frames = stalled_frames + 1 if max_speed < _SETTLED_VELOCITY_RAD_S else 0
                if stalled_frames >= _SETTLED_FRAMES:
                    break

        return {
            "done": True,
            "reached": reached,
            "max_error": max_error,
            "q": q.tolist(),
            "target": target.tolist(),
        }

    def get_state(self):
        """Snapshot of the driven joints' positions / velocities / torques (rad) +
        a timestamp. Reports only the driven columns (``joint_indices``), so a
        merged rig still presents just this device's joints."""
        positions = self._articulation.get_joint_positions()
        velocities = self._articulation.get_joint_velocities()
        efforts = self._articulation.get_measured_joint_efforts()
        return {
            "q": positions[self.joint_indices].tolist(),
            "qd": velocities[self.joint_indices].tolist() if velocities is not None else [0.0] * self.num_dof,
            "torque": efforts[self.joint_indices].tolist() if efforts is not None else [0.0] * self.num_dof,
            "timestamp": time.monotonic() - self._start_time,
        }

    def get_joint_limits(self):
        """``[lower, upper]`` radian limits for each driven joint, in ``get_state``'s
        ``q`` order.

        SingleArticulation exposes no ``get_dof_limits()``; we read it off the
        physics view, whose array carries a leading per-environment batch dimension
        (``(num_envs, num_dof, 2)``), so squeeze it to index by DOF directly. For a
        gripper narrowed to its driver this is one ``(open, closed)`` pair the client
        uses to map a fraction to an angle.
        """
        limits = self._articulation._articulation_view.get_dof_limits()
        if hasattr(limits, "ndim") and limits.ndim == 3:
            limits = limits[0]
        return [[float(limits[i][0]), float(limits[i][1])] for i in self.joint_indices]

    def info(self):
        """Static description of this device (dof count, driven joint names, and
        current state). The bind happens in the service's PUT /articulations."""
        return {"num_dof": self.num_dof, "dof_names": self.dof_names, "state": self.get_state()}
