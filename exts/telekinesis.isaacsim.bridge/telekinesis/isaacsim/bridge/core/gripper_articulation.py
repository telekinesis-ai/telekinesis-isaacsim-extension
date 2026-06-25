# SPDX-License-Identifier: Apache-2.0
"""
Gripper device: binds one SingleArticulation and drives its single actuated joint
between detected open/closed limits.

`fraction` is closed-ness: 0.0 = fully open, 1.0 = fully closed (matching the
synapse gripper interface, which converts mm/device/percent to this fraction).
Same blocking-in-handler pattern as the robot device.
"""

import asyncio

import omni.kit.app
import omni.timeline
import omni.usd
from loguru import logger
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import Usd, UsdPhysics

# ObjectStatus int mirrored by the synapse gripper interface (a completed move
# reports "at destination" whether it reached the target or stalled on an object).
STATUS_AT_DEST = 3

_REACH_TOLERANCE_RAD = 1e-3
_BIND_RETRIES = 60
# Blocking-move loop tuning. A gripper finishes either by reaching its commanded
# target, or by stalling against a grasped object: the driver joint stops moving
# short of the target. We treat a sustained low velocity (several consecutive
# frames, so the check can't fire before the drive accelerates the joint) as a
# stall. The frame cap is a backstop (~30 s at 60 fps).
_SETTLED_VELOCITY_RAD_S = 5e-3
_SETTLED_FRAMES = 5
_MOTION_MAX_FRAMES = 1800


def driven_joint_limits(articulation, driven_index):
    """Return ``(lower, upper)`` radian limits of one DOF on a SingleArticulation.

    SingleArticulation exposes no ``get_dof_limits()``; we read it off the physics
    view. That view's array carries a leading per-environment batch dimension
    (``(num_envs, num_dof, 2)``), so squeeze it to index by DOF directly.
    """
    limits = articulation._articulation_view.get_dof_limits()
    if hasattr(limits, "ndim") and limits.ndim == 3:
        limits = limits[0]
    return float(limits[driven_index][0]), float(limits[driven_index][1])


def find_driver_joint(stage, prim_path, dof_names):
    """Return the gripper's actuated driver joint name from ``dof_names``.

    A single-input gripper has one actuated joint; the rest are mimic joints that
    follow it. The driver is the non-mimic joint, preferring one with a
    UsdPhysics.DriveAPI. Falls back to the first DOF if detection is inconclusive.
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


class GripperArticulation:
    """Binds a gripper articulation and drives its actuated joint by closed-ness fraction."""

    def __init__(self, prim_path, name="gripper"):
        """Store prim path and initialise drive state; call `bind()` before use."""
        self.prim_path = prim_path
        self._name = name
        self._articulation = None
        self.num_dof = 0
        self.dof_names = []
        self._driven_index = 0
        self._opened_target_rad = 0.0
        self._closed_target_rad = 0.0
        self._move_lock = asyncio.Lock()

    async def bind(self):
        """(Re)initialize the gripper, then detect the driven joint and its limits.

        Safe to call repeatedly: reuses the handle and re-initializes against the
        current physics view (needed when the timeline was stopped/replayed
        between client sessions). Only proceeds once joint state reads non-None.
        """
        omni.timeline.get_timeline_interface().play()
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()

        bound = False
        for _ in range(_BIND_RETRIES):
            try:
                if self._articulation is None:
                    self._articulation = SingleArticulation(prim_path=self.prim_path, name=self._name)
                self._articulation.initialize()
                if self._articulation.num_dof and self._articulation.get_joint_positions() is not None:
                    self.num_dof = self._articulation.num_dof
                    self.dof_names = list(self._articulation.dof_names)
                    bound = True
                    break
            except Exception:
                pass
            await app.next_update_async()

        if not bound:
            raise RuntimeError(f"gripper articulation at {self.prim_path} did not become valid")

        stage = omni.usd.get_context().get_stage()
        driver_joint = find_driver_joint(stage, self.prim_path, self.dof_names)
        self._driven_index = self.dof_names.index(driver_joint)

        # Map the driven joint's limits to open/closed (lower = open, upper = closed
        # by convention; override per gripper later if a model is reversed).
        self._opened_target_rad, self._closed_target_rad = driven_joint_limits(
            self._articulation, self._driven_index
        )
        logger.info(
            f"[bridge] bound gripper {self.prim_path}: driver '{driver_joint}' "
            f"open={self._opened_target_rad:.3f} closed={self._closed_target_rad:.3f} rad"
        )

    def adopt_shared_articulation(self, articulation, driver_name):
        """Re-point this gripper at a shared (assembled) articulation.

        After ``attach_tool`` the arm and gripper are one articulation rooted at
        the arm; both devices hold this same handle. The gripper keeps driving its
        own actuated joint (``driver_name``), re-resolved into the merged DOF order
        with its open/closed limits read from the merged articulation. No rebind
        needed -- the handle is already initialized.
        """
        self._articulation = articulation
        self.dof_names = list(articulation.dof_names)
        self.num_dof = articulation.num_dof
        self._driven_index = self.dof_names.index(driver_name)

        self._opened_target_rad, self._closed_target_rad = driven_joint_limits(
            articulation, self._driven_index
        )
        logger.info(
            f"[bridge] gripper {self.prim_path} on shared articulation: driver "
            f"'{driver_name}' at index {self._driven_index} "
            f"open={self._opened_target_rad:.3f} closed={self._closed_target_rad:.3f} rad"
        )

    async def gripper_move(self, fraction):
        """Move the actuated joint to ``fraction`` closed-ness (0 open .. 1 closed) and block until done.

        Blocking by design, like the robot's ``move_j``: applies the action then
        awaits ``next_update_async`` in a loop until the finger reaches the target
        or stalls against a grasped object. Each ``next_update_async`` steps one
        physics frame and yields Isaac Sim's loop, so other requests keep running.
        Returns the final state with ``reached`` telling target-vs-stall apart.
        """
        fraction = min(max(float(fraction), 0.0), 1.0)
        target_rad = self._opened_target_rad + fraction * (self._closed_target_rad - self._opened_target_rad)

        app = omni.kit.app.get_app()
        async with self._move_lock:
            self._articulation.apply_action(
                ArticulationAction(joint_positions=[target_rad], joint_indices=[self._driven_index])
            )

            reached = False
            stalled_frames = 0
            for _ in range(_MOTION_MAX_FRAMES):
                await app.next_update_async()
                position = float(self._articulation.get_joint_positions()[self._driven_index])
                if abs(position - target_rad) < _REACH_TOLERANCE_RAD:
                    reached = True
                    break
                velocities = self._articulation.get_joint_velocities()
                speed = abs(float(velocities[self._driven_index])) if velocities is not None else 0.0
                stalled_frames = stalled_frames + 1 if speed < _SETTLED_VELOCITY_RAD_S else 0
                if stalled_frames >= _SETTLED_FRAMES:
                    break

        return {"done": True, "reached": reached, "status": STATUS_AT_DEST, **self.gripper_state()}

    async def gripper_open(self):
        """Fully open the gripper (fraction=0.0)."""
        return await self.gripper_move(0.0)

    async def gripper_close(self):
        """Fully close the gripper (fraction=1.0)."""
        return await self.gripper_move(1.0)

    def gripper_state(self):
        """Current closed-ness fraction of the actuated joint."""
        current = float(self._articulation.get_joint_positions()[self._driven_index])
        span = self._closed_target_rad - self._opened_target_rad
        fraction = 0.0 if span == 0 else (current - self._opened_target_rad) / span
        return {"fraction": fraction}

    def info(self):
        """Static description of this device (not a handshake; the bind happens
        in the server's PUT /articulations): dof count, joint names, and current state."""
        return {"num_dof": self.num_dof, "dof_names": self.dof_names, "state": self.gripper_state()}
