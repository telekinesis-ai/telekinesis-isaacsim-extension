# SPDX-License-Identifier: Apache-2.0
"""The surface gripper registry: the orchestration layer behind the surface gripper routes.

``SurfaceGripperService`` is the suction counterpart to
:class:`..services.articulations.ArticulationService`: an in-memory table mapping
``surface_gripper_id`` -> :class:`..core.surface_gripper.SurfaceGripper`, plus the
operations on it -- create (wrap + bind), look up, delete, list, close/open the
gripper, read its status, and get/set its properties and attachment points.

Ids are ``surface_gripper1``, ``surface_gripper2``, ... The prefix is not
cosmetic: ``assemble_robot`` takes one ``gripper_id`` for either kind of gripper
and tells them apart by which registry the id is in, so the two id spaces must not
collide.

It owns mutable state (the device table and the id counter), so exactly one
instance is shared across all requests -- :class:`BridgeServer` builds it once and
stashes it on ``app.state`` for the routers to reach via ``Depends`` (see
:mod:`..comm.dependencies`).

Transport coupling is deliberately minimal: bad input raises
``fastapi.HTTPException`` (400), a failed bind raises it (422) and acting on a
gripper the simulation is not running raises it (409), so the routers stay
one-liners. The ``..core.surface_gripper`` import pulls in isaacsim, so this
module imports only inside Isaac Sim -- same as the articulation service.

Wire units mirror the rest of the bridge: meters for lengths, degrees for angular
limits, newtons for force limits, seconds for the retry interval.
"""

import asyncio

from fastapi import HTTPException

from ...core.asset_loader import reference_usd_at
from ...core.surface_gripper import SurfaceGripper


class SurfaceGripperService:
    """The surface gripper registry shared by every request.

    Holds the ``surface_gripper_id`` -> device table and an id counter, and exposes
    create / get / delete / list, close / open / status, and the property and
    attachment-point getters and setters. One instance per running bridge.
    """

    def __init__(self):
        self._devices = {}  # surface_gripper_id -> SurfaceGripper
        self._id_by_prim = {}  # requested prim_path -> surface_gripper_id (stable on re-create)
        self._count = 0  # for ids like surface_gripper1, surface_gripper2
        # prim_path -> asyncio.Lock, serializes concurrent creates of the same prim
        self._create_locks = {}
        # Called with a surface_gripper_id when one is deleted, so the articulation
        # service can drop any assembly record naming it. Set by BridgeServer when
        # it composes the two services; None until then.
        self.on_deleted = None
        # Awaited after loading a gripper asset stopped the timeline, so the
        # articulation service can repair the device handles that stop killed. Set
        # by BridgeServer alongside on_deleted; None until then.
        self.on_timeline_stopped = None

    def clear(self):
        """Drop every bound gripper (called when the bridge stops or the stage changes)."""
        self._devices = {}
        self._id_by_prim = {}
        self._count = 0
        self._create_locks = {}

    async def create_surface_gripper(self, prim_path, usd_path=None):
        """Register (and bind) the surface gripper at ``prim_path`` and return its info.

        ``prim_path`` is the gripper prim itself or any ancestor of it -- usually
        the gripper asset's root, which is also the prim ``assemble_robot`` attaches
        to the arm. The ``IsaacSurfaceGripper`` prim is found by searching that
        subtree.

        One gripper per *requested* prim; PUTting the same prim again returns the
        same id and rebinds. The bind runs every time: the gripper's component is
        rebuilt whenever the timeline is stopped and replayed, and again when the
        gripper is assembled onto an arm.

        A suction gripper has no URDF representation, so there is no ``urdf_path``
        counterpart to the articulation route. It can still be loaded rather than
        prepared by hand: ``usd_path`` is a prepared USD asset that is referenced
        onto ``prim_path`` when nothing is there yet.
        """
        # Normalize a trailing slash so "/World/gripper" and "/World/gripper/"
        # register as the same gripper (USD paths are case-sensitive, so case is
        # left alone). Mirrors the articulation service.
        prim_path = prim_path.rstrip("/") or "/"

        # Serialize concurrent creates of the SAME prim_path: two clients racing to
        # register the same gripper would otherwise both see "not yet registered"
        # and each allocate an id/device, the second clobbering the first.
        # setdefault is synchronous, so concurrent callers land on one Lock.
        lock = self._create_locks.setdefault(prim_path, asyncio.Lock())
        async with lock:
            await self._load_if_missing(prim_path, usd_path)

            surface_gripper_id = self._id_by_prim.get(prim_path)
            if surface_gripper_id is None:
                self._count += 1
                surface_gripper_id = f"surface_gripper{self._count}"
                self._devices[surface_gripper_id] = SurfaceGripper(prim_path)
                self._id_by_prim[prim_path] = surface_gripper_id

            device = self._devices[surface_gripper_id]
            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: the request was well-formed but the prim couldn't actually be
                # bound as a surface gripper (semantic/runtime failure, not a bad
                # input value).
                raise HTTPException(status_code=422, detail=str(exc)) from exc
            return {
                "surface_gripper_id": surface_gripper_id,
                "prim_path": device.prim_path,
                **device.info(),
            }

    async def _load_if_missing(self, prim_path, usd_path):
        """Reference ``usd_path`` onto ``prim_path`` unless a prim is already there.

        A prim that is already in the stage is used as it is and ``usd_path`` has no
        effect, so a client may pass it defensively on every connect. Absent both,
        there is nothing to bind and the request is rejected.

        Loading stops the timeline, which kills the physics handle of every device
        bound before it, so the devices the articulation service holds are repaired
        in this same request rather than left dead until their next client rebind.
        """
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise HTTPException(status_code=409, detail="no USD stage is open")

        existing = stage.GetPrimAtPath(prim_path)
        if existing and existing.IsValid():
            return
        if not usd_path:
            raise HTTPException(
                status_code=400,
                detail=(
                    f"'{prim_path}' is not in the stage and no usd_path was given to load it"
                ),
            )
        try:
            await reference_usd_at(stage, usd_path, prim_path)
        except RuntimeError as exc:
            # 422: well-formed request, but loading the asset itself failed.
            raise HTTPException(status_code=422, detail=str(exc)) from exc

        if self.on_timeline_stopped is not None:
            await self.on_timeline_stopped()

    def get_surface_gripper(self, surface_gripper_id):
        """Info for one registered gripper (id, prims, properties, state), or 404."""
        device = self.get_device(surface_gripper_id)
        return {
            "surface_gripper_id": surface_gripper_id,
            "prim_path": device.prim_path,
            **device.info(),
        }

    def delete_surface_gripper(self, surface_gripper_id):
        """Unregister the gripper (the USD prim is left in the stage). 404 if unknown."""
        if surface_gripper_id not in self._devices:
            raise HTTPException(
                status_code=404,
                detail=f"no surface gripper registered with id '{surface_gripper_id}'",
            )
        del self._devices[surface_gripper_id]
        for prim, registered_id in list(self._id_by_prim.items()):
            if registered_id == surface_gripper_id:
                del self._id_by_prim[prim]
                # Drop the create-lock too, or it lingers forever under a prim_path
                # that no longer maps to anything.
                self._create_locks.pop(prim, None)
        if self.on_deleted is not None:
            # Forget any assembly this gripper took part in, so a later re-create +
            # assemble of the same pair is not refused as already assembled.
            self.on_deleted(surface_gripper_id)
        return {"deleted": surface_gripper_id}

    def list_surface_grippers(self):
        """Return a ``{surface_gripper_id: prim_path}`` map of every registered gripper."""
        return {
            surface_gripper_id: device.prim_path
            for surface_gripper_id, device in self._devices.items()
        }

    # -- actuation --------------------------------------------------------------

    async def close_gripper(self, surface_gripper_id, asynchronous):
        """Close the gripper. Blocking unless ``asynchronous``. See
        :meth:`..core.surface_gripper.SurfaceGripper.close`.
        """
        try:
            return await self.get_device(surface_gripper_id).close(asynchronous)
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    async def open_gripper(self, surface_gripper_id, asynchronous):
        """Release everything the gripper holds. Blocking unless ``asynchronous``. See
        :meth:`..core.surface_gripper.SurfaceGripper.open`.
        """
        try:
            return await self.get_device(surface_gripper_id).open(asynchronous)
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    def get_status(self, surface_gripper_id):
        """Current status, gripped objects and grip distance of one gripper."""
        return self.get_device(surface_gripper_id).get_status()

    # -- properties + attachment points -----------------------------------------

    def get_properties(self, surface_gripper_id):
        """The gripper's grip-behaviour properties."""
        return self.get_device(surface_gripper_id).get_properties()

    def set_properties(
        self,
        surface_gripper_id,
        coaxial_force_limit,
        shear_force_limit,
        max_grip_distance,
        retry_interval,
        forward_axis,
        rotation_limits,
        translation_limits,
    ):
        """Set the gripper's grip-behaviour properties; returns the resulting values.

        Any argument left ``None`` is not touched. The rotation and translation
        limits are written to every attachment point (USD stores them there, not on
        the gripper). See
        :meth:`..core.surface_gripper.SurfaceGripper.set_properties`.
        """
        device = self.get_device(surface_gripper_id)
        try:
            return device.set_properties(
                coaxial_force_limit=coaxial_force_limit,
                shear_force_limit=shear_force_limit,
                max_grip_distance=max_grip_distance,
                retry_interval=retry_interval,
                forward_axis=forward_axis,
                rotation_limits=rotation_limits,
                translation_limits=translation_limits,
            )
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_attachment_points(self, surface_gripper_id):
        """Per-attachment-point properties of one gripper, in the order it grips with them."""
        return {"attachment_points": self.get_device(surface_gripper_id).get_attachment_points()}

    def set_attachment_point_properties(
        self,
        surface_gripper_id,
        joint_paths,
        local_pose_0,
        local_pose_1,
        z_axis_translation_drive_stiffness,
        z_axis_translation_drive_damping,
        rotation_limits,
        translation_limits,
        clearance_offset,
        forward_axis,
    ):
        """Set properties on the gripper's attachment points; returns their new state.

        ``joint_paths`` defaults to every attachment point; any other argument left
        ``None`` is not touched. See
        :meth:`..core.surface_gripper.SurfaceGripper.set_attachment_point_properties`.
        """
        device = self.get_device(surface_gripper_id)
        try:
            return {
                "attachment_points": device.set_attachment_point_properties(
                    joint_paths=joint_paths,
                    local_pose_0=local_pose_0,
                    local_pose_1=local_pose_1,
                    z_axis_translation_drive_stiffness=z_axis_translation_drive_stiffness,
                    z_axis_translation_drive_damping=z_axis_translation_drive_damping,
                    rotation_limits=rotation_limits,
                    translation_limits=translation_limits,
                    clearance_offset=clearance_offset,
                    forward_axis=forward_axis,
                )
            }
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_device(self, surface_gripper_id):
        """Resolve a ``surface_gripper_id`` to its device object, or 404."""
        device = self._devices.get(surface_gripper_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=(
                    f"no surface gripper registered with id '{surface_gripper_id}', "
                    "call PUT /surface_grippers to create one"
                ),
            )
        return device

    def find_device(self, surface_gripper_id):
        """The device for ``surface_gripper_id``, or ``None`` if this registry has no such id.

        The non-raising counterpart to :meth:`get_device`, for ``assemble_robot``:
        it is handed one ``gripper_id`` and has to work out which registry it
        belongs to before it can report a useful error.
        """
        return self._devices.get(surface_gripper_id)
