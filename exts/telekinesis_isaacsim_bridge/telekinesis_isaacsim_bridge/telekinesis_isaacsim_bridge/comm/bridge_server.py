# SPDX-License-Identifier: Apache-2.0
"""
References(importing routes)
- https://github.com/fastapi/full-stack-fastapi-template/blob/master/backend/app/api/main.py
- https://github.com/fastapi/full-stack-fastapi-template/blob/2a6eeda62976e97d6e7104648e2011b4ab10ecda/backend/app/main.py#L30 

The single bridge server. One FastAPI app on one well-known port serves every
articulation in the scene; each is addressed by an id handed back from
PUT /articulations (e.g. ``robot1`` / ``gripper1``), not by TCP port as the old
per-device-server design did.

Structure
---------
``BridgeServer`` plays two roles:

* the **uvicorn lifecycle** (``start`` / ``stop``), and
* the **articulation registry** -- the in-memory table mapping ``articulation_id``
  -> articulation, plus the ``create_articulation`` / ``get_articulation_info`` /
  ``delete_articulation`` / ``get_device`` / ``list_articulations`` operations on it.

The HTTP handlers live in :mod:`.routers` as plain ``APIRouter``s grouped by
domain (articulations / robot / gripper). ``_build_app`` stashes this instance on
``app.state.registry`` so those routers reach the table via a small ``Depends``
-- no closures over ``self``, no circular import. Request bodies live in
:mod:`.models`.

Lifecycle / threading
----------------------
Isaac Sim owns the process's asyncio loop on the main thread. We do NOT call
``uvicorn.run`` (it would create a second, competing loop). Instead we build a
``uvicorn.Server`` and schedule its ``serve()`` coroutine onto Isaac's existing
loop with ``asyncio.ensure_future`` -- exactly as the old ConnectionServer did.
Because every request handler then runs on the main thread, the device methods
may touch SingleArticulation / omni.timeline APIs directly: no thread marshalling,
no command queue.

Multi-process control
---------------------
HTTP is multi-client. Two scripts in two terminals control two robots by POSTing
to two different ``/articulations/{articulation_id}/...`` paths on the same port;
they run as concurrent coroutines on Isaac's loop and both advance every physics
step. Each device's own ``_move_lock`` serializes commands to that one device.

Wire units (shared mental model with the Synapse client)
-------------------------------------------------------
Native Isaac units: radians for joints, meters for lengths. Gripper ``fraction``
is closed-ness: 0.0 = fully open, 1.0 = fully closed.
"""

import asyncio

import omni.usd
import uvicorn
from fastapi import FastAPI, HTTPException

from ..core.gripper_articulation import GripperArticulation
from ..core.robot_articulation import RobotArticulation
from ..core.robot_assembler import get_articulation_base_link_name, assemble_tool, bind_shared_articulation
from ..core.urdf_loader import import_urdf_at
from .routers import ALL_ROUTERS

DEVICE_ROBOT = "robot"
DEVICE_GRIPPER = "gripper"


class BridgeServer:
    """One FastAPI app serving all articulations; mirrors the old start()/stop() shape.

    Also acts as the articulation registry the routers depend on
    (``create_articulation`` / ``get_articulation_info`` / ``delete_articulation``
    / ``get_device`` / ``list_articulations``).
    """

    def __init__(self, host="127.0.0.1", port=8765):
        self._host = host
        self._port = port
        self._devices = {}      # articulation_id -> RobotArticulation | GripperArticulation
        self._id_by_prim = {}   # requested prim_path -> articulation_id (stable on re-create)
        self._counters = {}     # device_type -> count, for ids like robot1, gripper2
        self._app = self._build_app()
        self._server = None
        self._serve_task = None

    def start(self):
        """Schedule uvicorn's serve loop on Isaac Sim's asyncio loop (main thread)."""
        config = uvicorn.Config(self._app, host=self._host, port=self._port, lifespan="off", log_level="warning")
        self._server = uvicorn.Server(config)
        # We run inside Isaac's loop on the main thread; let Isaac keep its signal
        # handling, and don't let uvicorn hijack Ctrl+C.
        self._server.install_signal_handlers = lambda: None
        self._serve_task = asyncio.ensure_future(self._server.serve())
        print(f"[bridge] server listening on {self._host}:{self._port}")

    def stop(self):
        """Ask uvicorn to exit and drop every bound device."""
        self._devices = {}
        self._id_by_prim = {}
        self._counters = {}
        if self._server is not None:
            self._server.should_exit = True
            self._server = None
        if self._serve_task is not None:
            self._serve_task.cancel()
            self._serve_task = None
        print("[bridge] server stopped.")

    # -- articulation registry (used by the routers via app.state.registry) --

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

    def _build_app(self):
        app = FastAPI(title="telekinesis isaac-sim bridge")
        # The routers reach this device table via Depends(get_registry).
        app.state.registry = self
        for router in ALL_ROUTERS:
            app.include_router(router)
        return app
