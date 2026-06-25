# SPDX-License-Identifier: Apache-2.0
"""The articulation registry: the orchestration layer behind the articulation,
robot, and gripper HTTP routes.

``ArticulationService`` is the in-memory table mapping ``articulation_id`` ->
device (a :class:`RobotArticulation` / :class:`GripperArticulation` from
:mod:`..core`), plus the operations on it: create (import + bind), look up,
delete, list, and ``attach_tool`` (assemble a gripper onto an arm so both share
one articulation). It owns mutable state (the device table and the id counters),
so exactly one instance is shared across all requests -- :class:`BridgeServer`
builds it once and stashes it on ``app.state`` for the routers to reach via
``Depends`` (see :mod:`..comm.dependencies`).

Transport coupling is deliberately minimal: error cases raise
``fastapi.HTTPException`` so the routers stay one-liners. omni/USD imports are
lazy (inside the methods that need them) so this module imports outside Isaac Sim.

Wire units mirror the rest of the bridge: radians for joints, meters for lengths,
gripper ``fraction`` is closed-ness (0.0 open .. 1.0 closed).
"""

from fastapi import HTTPException

from ..core.gripper_articulation import GripperArticulation
from ..core.robot_articulation import RobotArticulation
from ..core.robot_assembler import assemble_tool, bind_shared_articulation, get_articulation_base_link_name
from ..core.urdf_loader import import_urdf_at

DEVICE_ROBOT = "robot"
DEVICE_GRIPPER = "gripper"


class ArticulationService:
    """The articulation registry shared by every request.

    Holds the ``articulation_id`` -> device table and the per-type id counters,
    and exposes ``create_articulation`` / ``get_articulation`` /
    ``delete_articulation`` / ``attach_tool`` / ``get_device`` /
    ``list_articulations``. One instance per running bridge.
    """

    def __init__(self):
        self._devices = {}      # articulation_id -> RobotArticulation | GripperArticulation
        self._id_by_prim = {}   # requested prim_path -> articulation_id (stable on re-create)
        self._counters = {}     # device_type -> count, for ids like robot1, gripper2

    def clear(self):
        """Drop every bound device (called when the bridge stops)."""
        self._devices = {}
        self._id_by_prim = {}
        self._counters = {}

    async def create_articulation(self, prim_path, device_type, urdf_path):
        """Register (and bind) the articulation at ``prim_path`` and return its info.

        One articulation per *requested* prim; PUTting the same prim again returns
        the same id (and rebinds). We key on the requested path, not the resolved
        one: importing a URDF resolves to the nested articulation root
        (``/World/ur10e/root_joint``) while an already-present prim resolves to
        itself (``/World/ur10e``), so keying on the resolved path would hand the
        same robot two ids across the load-vs-already-loaded paths. Ids are
        per-type and 1-based: ``robot1``, ``robot2``, ``gripper1``, ... The bind
        runs every time: a new client session may follow a timeline stop/replay,
        which invalidates the cached articulation handle.
        """
        resolved = await self._resolve_prim(prim_path, urdf_path)

        articulation_id = self._id_by_prim.get(prim_path)
        if articulation_id is None:
            device = self._make_device(device_type, resolved)
            self._counters[device_type] = self._counters.get(device_type, 0) + 1
            articulation_id = f"{device_type}{self._counters[device_type]}"
            self._devices[articulation_id] = device
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
        return {"deleted": articulation_id}

    async def attach_tool(self, arm_id, gripper_id, arm_mount_link, gripper_mount_link, offset):
        """Assemble the gripper onto the arm; both devices then share one articulation.

        The path articulation is the arm, ``gripper_id`` names the gripper. We
        capture each device's joint identities by NAME *before* assembly (paths and
        raw indices may not survive the topology change), assemble the two prims
        with ``RobotAssembler``, bind the single merged articulation, and hand that
        same handle to both devices -- each re-resolving its own joints by name.
        After this, the unchanged ``move_j`` / gripper routes drive the shared rig.

        ``gripper_mount_link`` may be None: the gripper must be joined at its *base*
        link (the articulation's root), so we auto-discover it via
        ``get_articulation_base_link_name`` rather than trusting a guessed name -- a
        non-root mount silently corrupts the merge (gripper joints never fold in).
        """
        import omni.usd

        arm = self.get_device(arm_id)
        gripper = self.get_device(gripper_id)
        if not isinstance(arm, RobotArticulation):
            raise HTTPException(status_code=400, detail=f"articulation '{arm_id}' is not a robot")
        if not isinstance(gripper, GripperArticulation):
            raise HTTPException(status_code=400, detail=f"articulation '{gripper_id}' is not a gripper")

        arm_names = list(arm.dof_names)
        driver_name = gripper.dof_names[gripper._driven_index]
        if gripper_mount_link is None:
            gripper_mount_link = get_articulation_base_link_name(gripper._articulation)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise HTTPException(status_code=409, detail="no USD stage is open")

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
        gripper.adopt_shared_articulation(shared, driver_name)

        return {
            "articulation": arm.prim_path,
            "num_dof": shared.num_dof,
            "dof_names": list(shared.dof_names),
            "arm_mount_link": arm_mount_link,
            "gripper_mount_link": gripper_mount_link,  # resolved (auto-discovered if omitted)
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

    def list_articulations(self):
        """Return a ``{articulation_id: prim_path}`` map of every registered articulation."""
        return {articulation_id: device.prim_path for articulation_id, device in self._devices.items()}

    # -- internals ----------------------------------------------------------

    def _make_device(self, device_type, prim_path):
        if device_type == DEVICE_ROBOT:
            return RobotArticulation(prim_path)
        if device_type == DEVICE_GRIPPER:
            return GripperArticulation(prim_path)
        raise HTTPException(
            status_code=400,
            detail=f"unknown device_type '{device_type}' (expected '{DEVICE_ROBOT}' / '{DEVICE_GRIPPER}')",
        )

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
