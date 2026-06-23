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
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import Usd, UsdPhysics

from ..comm.protocol import GRIPPER_CLOSE, GRIPPER_MOVE, GRIPPER_OPEN, GRIPPER_STATE, HANDSHAKE

# ObjectStatus ints mirrored by the synapse gripper interface.
STATUS_MOVING = 0
STATUS_AT_DEST = 3
# MoveMode ints.
MOVE_START = 0  # async: dispatch and return MOVING
MOVE_WAIT = 1   # blocking: step until reached, return AT_DEST

_REACH_TOLERANCE_RAD = 1e-3
_BIND_RETRIES = 60


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
        joint_limits = self._articulation.get_dof_limits()
        self._opened_target_rad = float(joint_limits[self._driven_index][0])
        self._closed_target_rad = float(joint_limits[self._driven_index][1])
        print(
            f"[bridge] bound gripper {self.prim_path}: driver '{driver_joint}' "
            f"open={self._opened_target_rad:.3f} closed={self._closed_target_rad:.3f} rad"
        )

    async def gripper_move(self, fraction, move_mode=MOVE_WAIT):
        """Drive the actuated joint to ``fraction`` closed-ness (0 open .. 1 closed)."""
        fraction = min(max(float(fraction), 0.0), 1.0)
        target_rad = self._opened_target_rad + fraction * (self._closed_target_rad - self._opened_target_rad)

        async with self._move_lock:
            self._articulation.apply_action(
                ArticulationAction(joint_positions=[target_rad], joint_indices=[self._driven_index])
            )
            if move_mode == MOVE_START:
                return {"status": STATUS_MOVING}

            app = omni.kit.app.get_app()
            while True:
                await app.next_update_async()
                current = self._articulation.get_joint_positions()[self._driven_index]
                if abs(current - target_rad) < _REACH_TOLERANCE_RAD:
                    break
        return {"status": STATUS_AT_DEST}

    async def gripper_open(self, move_mode=MOVE_WAIT):
        return await self.gripper_move(0.0, move_mode)

    async def gripper_close(self, move_mode=MOVE_WAIT):
        return await self.gripper_move(1.0, move_mode)

    def gripper_state(self):
        """Current closed-ness fraction of the actuated joint."""
        current = float(self._articulation.get_joint_positions()[self._driven_index])
        span = self._closed_target_rad - self._opened_target_rad
        fraction = 0.0 if span == 0 else (current - self._opened_target_rad) / span
        return {"fraction": fraction}

    def handshake(self):
        return {"num_dof": self.num_dof, "dof_names": self.dof_names, "state": self.gripper_state()}

    async def handle(self, request):
        """Route a wire request to the matching method."""
        message_type = request["type"]
        if message_type == GRIPPER_MOVE:
            return await self.gripper_move(request["fraction"], request.get("move_mode", MOVE_WAIT))
        if message_type == GRIPPER_OPEN:
            return await self.gripper_open(request.get("move_mode", MOVE_WAIT))
        if message_type == GRIPPER_CLOSE:
            return await self.gripper_close(request.get("move_mode", MOVE_WAIT))
        if message_type == GRIPPER_STATE:
            return self.gripper_state()
        if message_type == HANDSHAKE:
            return self.handshake()
        raise ValueError(f"gripper does not handle '{message_type}'")
