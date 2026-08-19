# SPDX-License-Identifier: Apache-2.0
"""The articulation registry: the orchestration layer behind the articulation HTTP routes.

``ArticulationService`` is the in-memory table mapping ``articulation_id`` ->
:class:`..core.articulation.SingleArticulation`, plus the operations on it: create
(import + bind), look up, delete, list, drive its joints, query its state /
limits / driver joint, narrow which joints it drives, and ``assemble_robot``
(assemble a gripper onto an arm so both share one articulation). There is no robot
vs gripper device type -- one generic device drives whatever joint subset it is
pointed at; the robot/gripper semantics live in the client.

It owns mutable state (the device table and the id counter), so exactly one
instance is shared across all requests -- :class:`BridgeServer` builds it once and
stashes it on ``app.state`` for the routers to reach via ``Depends`` (see
:mod:`..comm.dependencies`).

Transport coupling is deliberately minimal: error cases raise
``fastapi.HTTPException`` so the routers stay one-liners. omni/USD imports are
lazy (inside the methods that need them) so this module imports outside Isaac Sim.

Wire units mirror the rest of the bridge: radians for joints, meters for lengths.
"""

import asyncio

from fastapi import HTTPException

from ...core.articulation import SingleArticulation, find_driver_joint
from ...core.robot_assembler import assemble_tool, attach_surface_gripper, bind_shared_articulation
from ...core.urdf_loader import import_urdf_at


class ArticulationService:
    """The articulation registry shared by every request.

    Holds the ``articulation_id`` -> device table and an id counter, and exposes
    create / get / delete / list, the generic joint setter and getters, the
    driven-subset setter, the driver-joint discovery getter, and ``assemble_robot``.
    One instance per running bridge.

    ``assemble_robot`` accepts either kind of gripper, so this service composes the
    surface gripper registry (the way :class:`..services.prims.PrimService` composes
    the stage service) to resolve a ``gripper_id`` that belongs to it.
    """

    def __init__(self, surface_gripper_service):
        self._devices = {}  # articulation_id -> SingleArticulation
        self._id_by_prim = {}  # requested prim_path -> articulation_id (stable on re-create)
        self._count = 0  # for ids like articulation1, articulation2
        # arm_id -> {gripper_id, gripper_kind, arm_mount_link, gripper_mount_link}
        self._assemblies = {}
        # prim_path -> asyncio.Lock, serializes concurrent creates of the same prim
        self._create_locks = {}
        # arm_id -> asyncio.Lock, serializes concurrent assembles of the same arm
        self._assembly_locks = {}
        self._surface_gripper_service = surface_gripper_service

    def clear(self):
        """Drop every bound device (called when the bridge stops or the stage changes)."""
        self._devices = {}
        self._id_by_prim = {}
        self._count = 0
        self._assemblies = {}
        self._create_locks = {}
        self._assembly_locks = {}

    async def create_articulation(self, prim_path, urdf_path):
        """Register (and bind) the articulation at ``prim_path`` and return its info.

        One articulation per *requested* prim; PUTting the same prim again returns
        the same id (and rebinds). We key on the requested path, so a client that
        passes ``urdf_path`` defensively gets the same id whether the robot had to
        be imported or was already in the stage. Ids are 1-based: ``articulation1``,
        ``articulation2``, ... The bind runs every time: a new client session may
        follow a timeline stop/replay, which invalidates the cached articulation
        handle.
        """
        # Normalize a trailing slash so "/World/ur10e" and "/World/ur10e/" register as
        # the same articulation (USD paths are case-sensitive, so case is left alone).
        prim_path = prim_path.rstrip("/") or "/"

        # Serialize concurrent creates of the SAME prim_path (two clients racing to
        # register the same robot could otherwise both see "not yet registered" and
        # each allocate their own id/device, with the second clobbering the first's
        # registry entry). setdefault is synchronous -- no await inside it -- so
        # concurrent callers for the same prim_path always land on the same Lock.
        lock = self._create_locks.setdefault(prim_path, asyncio.Lock())
        async with lock:
            resolved, prim_source = await self._resolve_prim(prim_path, urdf_path)

            articulation_id = self._id_by_prim.get(prim_path)
            if articulation_id is None:
                self._count += 1
                articulation_id = f"articulation{self._count}"
                self._devices[articulation_id] = SingleArticulation(resolved, name=articulation_id)
                self._id_by_prim[prim_path] = articulation_id

            device = self._devices[articulation_id]
            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: the request was well-formed but the prim couldn't actually
                # be bound (semantic/runtime failure, not a bad input value).
                raise HTTPException(status_code=422, detail=str(exc)) from exc
            return {
                "articulation_id": articulation_id,
                "prim_path": resolved,
                "prim_source": prim_source,
                **device.info(),
            }

    def get_articulation(self, articulation_id):
        """Info for one registered articulation (id, prim, dof, state), or 404."""
        device = self.get_device(articulation_id)
        return {"articulation_id": articulation_id, "prim_path": device.prim_path, **device.info()}

    def delete_articulation(self, articulation_id):
        """Unregister the articulation (counterpart to ``create_articulation``).

        Drops the registry binding so the id is no longer known; the USD prim
        itself is left in the stage. 404 if the id was never registered.
        """
        if articulation_id not in self._devices:
            raise HTTPException(
                status_code=404,
                detail=f"no articulation registered with id '{articulation_id}'",
            )
        del self._devices[articulation_id]
        for prim, registered_id in list(self._id_by_prim.items()):
            if registered_id == articulation_id:
                del self._id_by_prim[prim]
                # Drop the create-lock too, or it lingers forever under a prim_path
                # that no longer maps to anything -- a slow leak for any client that
                # registers-and-deletes repeatedly (e.g. re-registering after a
                # stage reload under a fresh id each time).
                self._create_locks.pop(prim, None)
        # Forget any assembly this id took part in (as the arm or the gripper), so a
        # later re-create + assemble of the same pair is not blocked by a stale record.
        for arm_id, record in list(self._assemblies.items()):
            if articulation_id in (arm_id, record["gripper_id"]):
                del self._assemblies[arm_id]
        self._assembly_locks.pop(articulation_id, None)
        return {"deleted": articulation_id}

    def list_articulations(self):
        """Return a ``{articulation_id: prim_path}`` map of every registered articulation."""
        return {
            articulation_id: device.prim_path for articulation_id, device in self._devices.items()
        }

    # -- driving + introspection ------------------------------------------------

    async def move_j(self, articulation_id, positions, indices, asynchronous):
        """Drive joint ``positions`` (radians) onto the device's chosen joints.

        ``indices`` may be None (drive the device's current driven subset).
        Blocking unless ``asynchronous`` is true. See
        :meth:`..core.articulation.SingleArticulation.move_j`.
        """
        try:
            return await self.get_device(articulation_id).move_j(positions, indices, asynchronous)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    async def set_j(self, articulation_id, positions, indices):
        """Teleport the device's chosen joints directly to ``positions`` (radians).

        ``indices`` may be None (teleport the device's current driven subset). The
        move is immediate. See
        :meth:`..core.articulation.SingleArticulation.set_j`.
        """
        try:
            return await self.get_device(articulation_id).set_j(positions, indices)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def stream_joint_positions(self, articulation_id, positions, indices):
        """Retarget the device's chosen joints' drive for a high-rate stream. See
        :meth:`..core.articulation.SingleArticulation.stream_joint_positions`.

        Deliberately does NOT translate ValueError to HTTPException -- the
        WebSocket route catches ValueError itself, per-frame, to skip a bad
        frame and keep the stream open rather than closing the whole connection.
        """
        self.get_device(articulation_id).stream_joint_positions(positions, indices)

    def set_joint_velocities(self, articulation_id, velocities, indices):
        """Drive joint ``velocities`` (rad/s) onto the device's chosen joints.

        Fire-and-forget (velocity drive holds until the next command). ``indices``
        may be None (drive the device's current driven subset). See
        :meth:`..core.articulation.SingleArticulation.set_joint_velocities`.
        """
        try:
            return self.get_device(articulation_id).set_joint_velocities(velocities, indices)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_joints_state(self, articulation_id):
        """Current joint positions / velocities / efforts of the driven subset."""
        return self.get_device(articulation_id).get_joints_state()

    def get_articulation_state(self, articulation_id):
        """Every per-frame quantity of one articulation in a single snapshot, or
        ``None`` when its handle is not currently readable. See
        :meth:`..core.articulation.SingleArticulation.get_articulation_state`.
        """
        return self.get_device(articulation_id).get_articulation_state()

    def get_dof_limits(self, articulation_id):
        """``[lower, upper]`` radian limits per driven joint (``get_joints_state`` order)."""
        return {"limits": self.get_device(articulation_id).get_dof_limits()}

    def set_driven_joints(self, articulation_id, joint_names):
        """Narrow the device's driven joints to ``joint_names``; return its new info."""
        device = self.get_device(articulation_id)
        try:
            device.set_driven_joints(joint_names)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"articulation_id": articulation_id, "prim_path": device.prim_path, **device.info()}

    # -- extended introspection / physics tuning ---------------------------

    def get_handles_initialized(self, articulation_id):
        """Whether the device's handle is currently valid, without a re-bind."""
        return {"handles_initialized": self.get_device(articulation_id).handles_initialized()}

    def get_num_bodies(self, articulation_id):
        """Number of rigid-body links in the underlying articulation."""
        return {"num_bodies": self.get_device(articulation_id).num_bodies()}

    def get_dof_properties(self, articulation_id):
        """Per-driven-joint drive properties (limits, drive mode, gains)."""
        return {"dof_properties": self.get_device(articulation_id).dof_properties()}

    def set_dof_gains(self, articulation_id, stiffness, damping, max_effort, indices):
        """Set the position drive's stiffness / damping / effort ceiling; return the
        driven joints' resulting drive properties."""
        device = self.get_device(articulation_id)
        try:
            return device.set_dof_gains(
                stiffness=stiffness,
                damping=damping,
                max_effort=max_effort,
                indices=indices,
            )
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_dof_index(self, articulation_id, joint_name):
        """DOF index of ``joint_name`` within the device's driven subset."""
        device = self.get_device(articulation_id)
        try:
            index = device.dof_index(joint_name)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"name": joint_name, "index": index}

    def get_applied_joint_efforts(self, articulation_id):
        """Efforts last commanded via ``set_joint_efforts`` on the driven joints."""
        return {"joint_efforts": self.get_device(articulation_id).get_applied_joint_efforts()}

    def get_measured_joint_forces(self, articulation_id):
        """Measured 6-axis joint reaction force/torque per driven joint."""
        return {"joint_forces": self.get_device(articulation_id).get_measured_joint_forces()}

    def get_joints_default_state(self, articulation_id):
        """Stored joint-space home pose for the driven joints."""
        return self.get_device(articulation_id).get_joints_default_state()

    def set_joints_default_state(
        self, articulation_id, joint_positions, joint_velocities, joint_efforts
    ):
        """Set the driven joints' stored home pose (applied on the next reset)."""
        device = self.get_device(articulation_id)
        try:
            device.set_joints_default_state(joint_positions, joint_velocities, joint_efforts)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return device.get_joints_default_state()

    def get_applied_action(self, articulation_id):
        """Last ``ArticulationAction`` PhysX actually received for this articulation."""
        return self.get_device(articulation_id).get_applied_action()

    def set_joint_efforts(self, articulation_id, efforts, indices):
        """Command raw torque/force directly on the chosen joints (bypasses the drive)."""
        device = self.get_device(articulation_id)
        try:
            return device.set_joint_efforts(efforts, indices)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def enable_gravity(self, articulation_id):
        """Enable gravity for the whole articulation (body-level, not per-joint)."""
        self.get_device(articulation_id).enable_gravity()
        return {"gravity_enabled": True}

    def disable_gravity(self, articulation_id):
        """Disable gravity for the whole articulation (body-level, not per-joint)."""
        self.get_device(articulation_id).disable_gravity()
        return {"gravity_enabled": False}

    def get_world_velocity(self, articulation_id):
        """Root link's full 6-DOF world-space velocity."""
        return {"velocity": self.get_device(articulation_id).get_world_velocity()}

    def set_world_velocity(self, articulation_id, velocity):
        """Set the root link's full 6-DOF world-space velocity."""
        device = self.get_device(articulation_id)
        try:
            device.set_world_velocity(velocity)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"velocity": device.get_world_velocity()}

    def get_linear_velocity(self, articulation_id):
        """Root link's linear (translational) velocity."""
        return {"velocity": self.get_device(articulation_id).get_linear_velocity()}

    def set_linear_velocity(self, articulation_id, velocity):
        """Set the root link's linear velocity only."""
        device = self.get_device(articulation_id)
        try:
            device.set_linear_velocity(velocity)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"velocity": device.get_linear_velocity()}

    def get_angular_velocity(self, articulation_id):
        """Root link's angular (rotational) velocity."""
        return {"velocity": self.get_device(articulation_id).get_angular_velocity()}

    def set_angular_velocity(self, articulation_id, velocity):
        """Set the root link's angular velocity only."""
        device = self.get_device(articulation_id)
        try:
            device.set_angular_velocity(velocity)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"velocity": device.get_angular_velocity()}

    def get_solver_position_iteration_count(self, articulation_id):
        """PhysX position-solver iteration count."""
        return {"count": self.get_device(articulation_id).get_solver_position_iteration_count()}

    def set_solver_position_iteration_count(self, articulation_id, count):
        """Set the PhysX position-solver iteration count."""
        device = self.get_device(articulation_id)
        try:
            device.set_solver_position_iteration_count(count)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"count": device.get_solver_position_iteration_count()}

    def get_solver_velocity_iteration_count(self, articulation_id):
        """PhysX velocity-solver iteration count."""
        return {"count": self.get_device(articulation_id).get_solver_velocity_iteration_count()}

    def set_solver_velocity_iteration_count(self, articulation_id, count):
        """Set the PhysX velocity-solver iteration count."""
        device = self.get_device(articulation_id)
        try:
            device.set_solver_velocity_iteration_count(count)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"count": device.get_solver_velocity_iteration_count()}

    def get_stabilization_threshold(self, articulation_id):
        """Mass-normalized kinetic energy below which PhysX may stabilize this articulation."""
        return {"threshold": self.get_device(articulation_id).get_stabilization_threshold()}

    def set_stabilization_threshold(self, articulation_id, threshold):
        """Set the stabilization threshold."""
        device = self.get_device(articulation_id)
        try:
            device.set_stabilization_threshold(threshold)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"threshold": device.get_stabilization_threshold()}

    def get_enabled_self_collisions(self, articulation_id):
        """Whether this articulation's own links can collide with each other."""
        return {"enabled": self.get_device(articulation_id).get_enabled_self_collisions()}

    def set_enabled_self_collisions(self, articulation_id, enabled):
        """Enable/disable self-collision between this articulation's own links."""
        device = self.get_device(articulation_id)
        device.set_enabled_self_collisions(enabled)
        return {"enabled": device.get_enabled_self_collisions()}

    def get_sleep_threshold(self, articulation_id):
        """Velocity threshold below which PhysX lets this articulation sleep."""
        return {"threshold": self.get_device(articulation_id).get_sleep_threshold()}

    def set_sleep_threshold(self, articulation_id, threshold):
        """Set the sleep threshold."""
        device = self.get_device(articulation_id)
        try:
            device.set_sleep_threshold(threshold)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"threshold": device.get_sleep_threshold()}

    def get_driver_joint(self, articulation_id):
        """Discover the gripper's actuated driver joint: its name and DOF index.

        A USD/PhysX schema walk (mimic vs DriveAPI) the client cannot do itself.
        The index is into the device's current ``dof_names`` (so it is valid both
        standalone and after assembly).
        """
        device = self.get_device(articulation_id)
        try:
            name = find_driver_joint(self._stage(), device.prim_path, device.dof_names)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"name": name, "index": device.dof_names.index(name)}

    async def assemble_robot(
        self, arm_id, gripper_id, arm_mount_link, gripper_mount_link, offset, mask_collisions
    ):
        """Assemble a gripper onto the arm, whichever kind of gripper it is.

        The path articulation is the arm; ``gripper_id`` names the gripper and
        decides how it is attached. An id from PUT /articulations is an articulated
        gripper and is *merged* into the arm, so the two share one articulation
        afterwards. An id from PUT /surface_grippers is a suction gripper and is
        *bolted on*, staying a device of its own with its own close/open routes.
        The two id spaces do not overlap, so the caller passes one field either way.

        Assembly mutates USD and is not idempotent: running it twice on the same
        pair would build a second fixed joint (and, for an articulated gripper,
        re-root an already-merged tree). So each completed assembly is recorded
        (``self._assemblies``); a repeat call for the same arm+gripper is a no-op
        that just returns the existing info with ``already_assembled=True``. The
        record is cleared with the registry when the stage changes, so a fresh stage
        assembles again.

        ``mask_collisions`` only applies to a suction gripper (see
        :class:`..comm.models.AssembleRobotRequest`).
        """
        import carb

        # Serialize concurrent assembles of the SAME arm (two requests racing past
        # the already-assembled check below could otherwise both proceed and each
        # build a fixed joint / re-root the tree -- assembly is not idempotent at
        # the USD level, only this guard makes repeat calls safe).
        lock = self._assembly_locks.setdefault(arm_id, asyncio.Lock())
        async with lock:
            self.get_device(arm_id)  # 404 early if the arm itself is unknown
            surface_gripper = self._surface_gripper_service.find_device(gripper_id)
            if surface_gripper is None and gripper_id not in self._devices:
                raise HTTPException(
                    status_code=404,
                    detail=(
                        f"no gripper registered with id '{gripper_id}': it is neither an "
                        "articulation (PUT /articulations) nor a surface gripper "
                        "(PUT /surface_grippers)"
                    ),
                )

            existing = self._assemblies.get(arm_id)
            if existing is not None and existing["gripper_id"] == gripper_id:
                carb.log_info(
                    f"[bridge] {arm_id} + {gripper_id} already assembled; skipping re-assembly."
                )
                return self._assembly_info(arm_id, existing, already_assembled=True)

            if surface_gripper is not None:
                record = await self._attach_surface_gripper(
                    arm_id,
                    gripper_id,
                    surface_gripper,
                    arm_mount_link,
                    gripper_mount_link,
                    offset,
                    mask_collisions,
                )
            else:
                record = await self._merge_articulated_gripper(
                    arm_id, gripper_id, arm_mount_link, gripper_mount_link, offset
                )
            self._assemblies[arm_id] = record
            return self._assembly_info(arm_id, record, already_assembled=False)

    async def _merge_articulated_gripper(
        self, arm_id, gripper_id, arm_mount_link, gripper_mount_link, offset
    ):
        """Merge an articulated gripper into the arm so both share one articulation.

        Each device's joint identities are captured by NAME *before* assembly (paths
        and raw indices may not survive the topology change); the two prims are
        assembled with ``RobotAssembler``, the single merged articulation is bound,
        and that same handle is handed to both devices -- each re-resolving its own
        joints by name. After this the unchanged ``move_j`` route drives the shared
        rig.

        The gripper's driven joints are whatever the client already narrowed them to
        (its driver joint); if the client never narrowed them, the driver is
        discovered here so a single actuated joint is folded in, not every mimic.

        ``gripper_mount_link`` may be None: the gripper must be joined at its *base*
        link (the articulation's root), so it is auto-discovered via
        ``get_articulation_base_link_name`` rather than trusting a guessed name -- a
        non-root mount silently corrupts the merge (gripper joints never fold in).
        """
        arm = self.get_device(arm_id)
        gripper = self.get_device(gripper_id)
        stage = self._stage()

        arm_names = list(arm.dof_names)
        if gripper.joint_names:
            gripper_driven = list(gripper.dof_names)
        else:
            try:
                gripper_driven = [find_driver_joint(stage, gripper.prim_path, gripper.dof_names)]
            except ValueError as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc
        if gripper_mount_link is None:
            gripper_mount_link = gripper.base_link_name()

        arm_leaf = arm.prim_path.rstrip("/").rsplit("/", 1)[-1]
        await assemble_tool(
            stage,
            arm.prim_path,
            arm_mount_link,
            gripper.prim_path,
            gripper_mount_link,
            offset,
            namespace="Tool",
            variant=f"{arm_leaf}_with_tool",
        )

        shared = await bind_shared_articulation(arm.prim_path, name=f"{arm_leaf}_with_tool")
        arm.adopt_shared_articulation(shared, arm_names)
        gripper.adopt_shared_articulation(shared, gripper_driven)

        return {
            "gripper_id": gripper_id,
            "gripper_kind": "articulated",
            "arm_mount_link": arm_mount_link,
            "gripper_mount_link": gripper_mount_link,  # resolved (auto-discovered if omitted)
        }

    async def _attach_surface_gripper(
        self,
        arm_id,
        gripper_id,
        gripper,
        arm_mount_link,
        gripper_mount_link,
        offset,
        mask_collisions,
    ):
        """Bolt a suction gripper onto the arm's mount link with a fixed joint.

        Nothing is merged: a suction gripper has no joints to fold into the arm's
        articulation, so it keeps its own id and its own close/open routes and the
        arm's DOF are unchanged. What assembly does instead is place the gripper at
        the flange, join the two with a fixed joint excluded from the articulation,
        and re-park the gripper's attachment points onto the arm's mount link so
        they can grip (see
        :func:`..core.robot_assembler.attach_surface_gripper`).

        ``gripper_mount_link`` may be None, meaning the registered gripper prim
        itself -- unlike the articulated path, there is no articulation root to
        auto-discover. Whichever prim it resolves to has to be the gripper's rigid
        body: a fixed joint needs one, and mounting the arm to a mesh or a frame
        inside the gripper would join it to a part of the gripper rather than to the
        gripper.

        Stopping the timeline to edit USD invalidates the arm's articulation handle
        and the gripper's component, so both are re-bound afterwards -- without that
        the next ``move_j`` on the arm fails and the gripper cannot be actuated.
        """
        arm = self.get_device(arm_id)
        stage = self._stage()

        try:
            gripper_mount_path = gripper.mount_body_path(gripper_mount_link)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

        arm_names = list(arm.dof_names)
        arm_leaf = arm.prim_path.rstrip("/").rsplit("/", 1)[-1]
        try:
            fixed_joint = await attach_surface_gripper(
                stage,
                arm.prim_path,
                arm_mount_link,
                gripper.prim_path,
                gripper_mount_path,
                gripper.attachment_point_paths,
                offset,
                mask_collisions=mask_collisions,
            )
        except RuntimeError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

        # The arm's own handle, re-bound against the post-assembly stage. Its driven
        # joints are unchanged (the gripper contributes none), but the handle itself
        # did not survive the stop/play, so it is re-adopted rather than re-created.
        shared = await bind_shared_articulation(arm.prim_path, name=f"{arm_leaf}_with_tool")
        arm.adopt_shared_articulation(shared, arm_names)
        try:
            await gripper.bind()
        except RuntimeError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

        return {
            "gripper_id": gripper_id,
            "gripper_kind": "surface",
            "arm_mount_link": arm_mount_link,
            "gripper_mount_link": gripper_mount_path,  # resolved (the gripper prim if omitted)
            "fixed_joint": fixed_joint,
        }

    def _assembly_info(self, arm_id, record, already_assembled):
        """Build the assembled-rig response from an assembly ``record``.

        Used both right after an assembly and for a repeat (no-op) call, so the wire
        shape is identical either way. Reads live ``info()`` so the state is current.
        ``num_dof`` / ``dof_names`` describe the arm's articulation -- which for an
        articulated gripper is the merged rig and for a suction gripper is just the
        arm, since a suction gripper adds no DOF. ``gripper_kind`` says which
        happened, and the ``gripper`` block carries whichever device's info.
        """
        arm = self.get_device(arm_id)
        gripper_id = record["gripper_id"]
        shared = arm.shared_info()
        if record["gripper_kind"] == "surface":
            device = self._surface_gripper_service.get_device(gripper_id)
            gripper_info = {"surface_gripper_id": gripper_id, **device.info()}
        else:
            gripper_info = {"articulation_id": gripper_id, **self.get_device(gripper_id).info()}
        return {
            "articulation": arm.prim_path,
            "num_dof": shared["num_dof"],
            "dof_names": shared["dof_names"],
            "gripper_kind": record["gripper_kind"],
            "arm_mount_link": record["arm_mount_link"],
            "gripper_mount_link": record["gripper_mount_link"],  # resolved
            "fixed_joint": record.get("fixed_joint"),
            "already_assembled": already_assembled,
            "robot": {"articulation_id": arm_id, **arm.info()},
            "gripper": gripper_info,
        }

    def forget_assembly(self, gripper_id):
        """Drop any assembly record naming ``gripper_id``, so the pair can be re-assembled.

        Called by the surface gripper registry when one of its grippers is deleted;
        the articulated case is handled inline by ``delete_articulation``. Without
        it, re-creating and re-assembling the same gripper would be refused as
        already assembled and would silently skip the actual assembly.
        """
        for arm_id, record in list(self._assemblies.items()):
            if record["gripper_id"] == gripper_id:
                del self._assemblies[arm_id]

    def get_device(self, articulation_id):
        """Resolve an ``articulation_id`` to its device object, or 404."""
        device = self._devices.get(articulation_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=(
                    f"no articulation registered with id '{articulation_id}', "
                    "call PUT /articulations to create one"
                ),
            )
        return device

    # -- internals ----------------------------------------------------------

    @staticmethod
    def _stage():
        """The open USD stage, or 409. Mirrors :meth:`..services.stage.StageService.stage`."""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise HTTPException(status_code=409, detail="no USD stage is open")
        return stage

    async def _resolve_prim(self, requested_prim_path, urdf_path):
        """Use the prim if it's already in the stage, else import the URDF at it.

        Returns ``(resolved_path, prim_source)``. ``prim_source`` reports which of
        the two actually happened -- ``"isaac_usd"`` if the prim was already in
        the stage (any ``urdf_path`` given had no effect), or ``"imported_urdf"``
        if it was imported from that URDF file just now. Without this, a client
        that passes ``urdf_path`` defensively (import if missing, reuse if not)
        has no way to tell which one actually happened.
        """
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        existing = stage.GetPrimAtPath(requested_prim_path)
        if existing and existing.IsValid():
            return requested_prim_path, "isaac_usd"
        if urdf_path:
            try:
                resolved = await import_urdf_at(stage, urdf_path, requested_prim_path)
            except RuntimeError as exc:
                # 422: well-formed request, but the URDF import itself failed.
                raise HTTPException(status_code=422, detail=str(exc)) from exc
            return resolved, "imported_urdf"
        raise HTTPException(
            status_code=400,
            detail=(
                f"'{requested_prim_path}' is not in the stage and no urdf_path was given to load it"
            ),
        )
