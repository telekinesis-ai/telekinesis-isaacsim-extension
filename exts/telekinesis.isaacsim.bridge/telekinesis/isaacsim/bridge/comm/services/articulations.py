# SPDX-License-Identifier: Apache-2.0
"""The articulation registry: the orchestration layer behind the articulation HTTP routes.

``ArticulationService`` is the in-memory table mapping ``articulation_id`` ->
:class:`..core.articulation.Articulation`, plus the operations on it: create
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

from fastapi import HTTPException

from ...core.articulation import Articulation, find_driver_joint
from ...core.robot_assembler import assemble_tool, bind_shared_articulation, get_articulation_base_link_name
from ...core.urdf_loader import import_urdf_at


class ArticulationService:
    """The articulation registry shared by every request.

    Holds the ``articulation_id`` -> device table and an id counter, and exposes
    create / get / delete / list, the generic joint setter and getters, the
    driven-subset setter, the driver-joint discovery getter, and ``assemble_robot``.
    One instance per running bridge.
    """

    def __init__(self):
        self._devices = {}      # articulation_id -> Articulation
        self._id_by_prim = {}   # requested prim_path -> articulation_id (stable on re-create)
        self._count = 0         # for ids like articulation1, articulation2
        self._assemblies = {}   # arm_id -> {gripper_id, arm_mount_link, gripper_mount_link}

    def clear(self):
        """Drop every bound device (called when the bridge stops or the stage changes)."""
        self._devices = {}
        self._id_by_prim = {}
        self._count = 0
        self._assemblies = {}

    async def create_articulation(self, prim_path, urdf_path):
        """Register (and bind) the articulation at ``prim_path`` and return its info.

        One articulation per *requested* prim; PUTting the same prim again returns
        the same id (and rebinds). We key on the requested path, not the resolved
        one: importing a URDF resolves to the nested articulation root
        (``/World/ur10e/root_joint``) while an already-present prim resolves to
        itself (``/World/ur10e``), so keying on the resolved path would hand the
        same articulation two ids across the load-vs-already-loaded paths. Ids are
        1-based: ``articulation1``, ``articulation2``, ... The bind runs every time:
        a new client session may follow a timeline stop/replay, which invalidates
        the cached articulation handle.
        """
        resolved = await self._resolve_prim(prim_path, urdf_path)

        articulation_id = self._id_by_prim.get(prim_path)
        if articulation_id is None:
            self._count += 1
            articulation_id = f"articulation{self._count}"
            self._devices[articulation_id] = Articulation(resolved, name=articulation_id)
            self._id_by_prim[prim_path] = articulation_id

        device = self._devices[articulation_id]
        await device.bind()
        return {"articulation_id": articulation_id, "prim_path": resolved, **device.info()}

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
        # Forget any assembly this id took part in (as the arm or the gripper), so a
        # later re-create + assemble of the same pair is not blocked by a stale record.
        for arm_id, record in list(self._assemblies.items()):
            if arm_id == articulation_id or record["gripper_id"] == articulation_id:
                del self._assemblies[arm_id]
        return {"deleted": articulation_id}

    def list_articulations(self):
        """Return a ``{articulation_id: prim_path}`` map of every registered articulation."""
        return {articulation_id: device.prim_path for articulation_id, device in self._devices.items()}

    # -- driving + introspection ------------------------------------------------

    async def move_j(self, articulation_id, positions, indices, asynchronous):
        """Drive joint ``positions`` (radians) onto the device's chosen joints.

        ``indices`` may be None (drive the device's current driven subset).
        Blocking unless ``asynchronous`` is true. See
        :meth:`..core.articulation.Articulation.move_j`.
        """
        return await self.get_device(articulation_id).move_j(positions, indices, asynchronous)

    async def set_j(self, articulation_id, positions, indices):
        """Teleport the device's chosen joints directly to ``positions`` (radians).

        ``indices`` may be None (teleport the device's current driven subset). The
        move is immediate. See
        :meth:`..core.articulation.Articulation.set_j`.
        """
        return await self.get_device(articulation_id).set_j(positions, indices)

    def stream_joint_positions(self, articulation_id, positions, indices):
        """Teleport the device's chosen joints for a high-rate stream. See
        :meth:`..core.articulation.Articulation.stream_joint_positions`."""
        self.get_device(articulation_id).stream_joint_positions(positions, indices)

    def get_joint_state(self, articulation_id):
        """Current joint positions / velocities / torques of the driven subset."""
        return self.get_device(articulation_id).get_state()

    def get_joint_limits(self, articulation_id):
        """``[lower, upper]`` radian limits per driven joint (``get_state`` q order)."""
        return {"limits": self.get_device(articulation_id).get_joint_limits()}

    def set_driven_joints(self, articulation_id, joint_names):
        """Narrow the device's driven joints to ``joint_names``; return its new info."""
        device = self.get_device(articulation_id)
        device.set_driven_joints(joint_names)
        return {"articulation_id": articulation_id, "prim_path": device.prim_path, **device.info()}

    def get_driver_joint(self, articulation_id):
        """Discover the gripper's actuated driver joint: its name and DOF index.

        A USD/PhysX schema walk (mimic vs DriveAPI) the client cannot do itself.
        The index is into the device's current ``dof_names`` (so it is valid both
        standalone and after assembly).
        """
        import omni.usd

        device = self.get_device(articulation_id)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise HTTPException(status_code=409, detail="no USD stage is open")
        name = find_driver_joint(stage, device.prim_path, device.dof_names)
        return {"name": name, "index": device.dof_names.index(name)}

    async def assemble_robot(self, arm_id, gripper_id, arm_mount_link, gripper_mount_link, offset):
        """Assemble the gripper onto the arm; both devices then share one articulation.

        The path articulation is the arm, ``gripper_id`` names the gripper. We
        capture each device's joint identities by NAME *before* assembly (paths and
        raw indices may not survive the topology change), assemble the two prims
        with ``RobotAssembler``, bind the single merged articulation, and hand that
        same handle to both devices -- each re-resolving its own joints by name.
        After this, the unchanged ``move_j`` route drives the shared rig.

        Assembly mutates USD and is not idempotent: running it twice on the same pair
        would build a second fixed joint and re-root an already-merged tree. So we
        record each completed assembly (``self._assemblies``); a repeat call for the
        same arm+gripper is a no-op that just returns the existing merged info with
        ``already_assembled=True``. The record is cleared with the registry when the
        stage changes, so a fresh stage assembles again.

        The gripper's driven joints are whatever the client already narrowed them
        to (its driver joint); if the client never narrowed them, we discover the
        driver here so a single actuated joint is folded in, not every mimic.

        ``gripper_mount_link`` may be None: the gripper must be joined at its *base*
        link (the articulation's root), so we auto-discover it via
        ``get_articulation_base_link_name`` rather than trusting a guessed name -- a
        non-root mount silently corrupts the merge (gripper joints never fold in).
        """
        import carb
        import omni.usd

        arm = self.get_device(arm_id)
        gripper = self.get_device(gripper_id)

        existing = self._assemblies.get(arm_id)
        if existing is not None and existing["gripper_id"] == gripper_id:
            carb.log_info(f"[bridge] {arm_id} + {gripper_id} already assembled; skipping re-assembly.")
            return self._assembly_info(
                arm_id, gripper_id, existing["arm_mount_link"], existing["gripper_mount_link"],
                already_assembled=True,
            )

        arm_names = list(arm.dof_names)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise HTTPException(status_code=409, detail="no USD stage is open")

        if gripper.joint_names:
            gripper_driven = list(gripper.dof_names)
        else:
            gripper_driven = [find_driver_joint(stage, gripper.prim_path, gripper.dof_names)]
        if gripper_mount_link is None:
            gripper_mount_link = get_articulation_base_link_name(gripper._articulation)

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

        self._assemblies[arm_id] = {
            "gripper_id": gripper_id,
            "arm_mount_link": arm_mount_link,
            "gripper_mount_link": gripper_mount_link,  # resolved (auto-discovered if omitted)
        }
        return self._assembly_info(arm_id, gripper_id, arm_mount_link, gripper_mount_link, already_assembled=False)

    def _assembly_info(self, arm_id, gripper_id, arm_mount_link, gripper_mount_link, already_assembled):
        """Build the merged-rig response from the two devices' shared handle.

        Used both right after an assembly and for a repeat (no-op) call, so the wire
        shape is identical either way. Reads live ``info()`` so the joint state is
        current; ``shared`` is the merged handle both devices now hold.
        """
        arm = self.get_device(arm_id)
        gripper = self.get_device(gripper_id)
        shared = arm._articulation
        return {
            "articulation": arm.prim_path,
            "num_dof": shared.num_dof,
            "dof_names": list(shared.dof_names),
            "arm_mount_link": arm_mount_link,
            "gripper_mount_link": gripper_mount_link,  # resolved (auto-discovered if omitted)
            "already_assembled": already_assembled,
            "robot": {"articulation_id": arm_id, **arm.info()},
            "gripper": {"articulation_id": gripper_id, **gripper.info()},
        }

    def get_device(self, articulation_id):
        """Resolve an ``articulation_id`` to its device object, or 404."""
        device = self._devices.get(articulation_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=f"no articulation registered with id '{articulation_id}', call PUT /articulations to create one",
            )
        return device

    # -- internals ----------------------------------------------------------

    async def _resolve_prim(self, requested_prim_path, urdf_path):
        """Use the prim if it's already in the stage, else import the URDF at it."""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        existing = stage.GetPrimAtPath(requested_prim_path)
        if existing and existing.IsValid():
            return requested_prim_path
        if urdf_path:
            return await import_urdf_at(stage, urdf_path, requested_prim_path)
        raise HTTPException(
            status_code=400,
            detail=f"'{requested_prim_path}' is not in the stage and no urdf_path was given to load it",
        )
