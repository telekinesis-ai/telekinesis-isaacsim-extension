# SPDX-License-Identifier: Apache-2.0
# pylint: disable=line-too-long
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
``BridgeServer`` has two narrow jobs:

* the **uvicorn lifecycle** (``start`` / ``stop``), and
* **building the FastAPI app** (``_build_app``): it constructs the services
  (:mod:`..services`) and stashes them on ``app.state`` so the routers reach them
  via ``Depends`` (see :mod:`.dependencies`), then mounts the routers.

The orchestration that used to live here -- the articulation registry, the stage
and prim logic -- now lives in :mod:`..services`. The HTTP handlers live in
:mod:`.routers` as plain ``APIRouter``s; request bodies live in :mod:`.models`.

Lifecycle / threading
----------------------
Isaac Sim owns the process's asyncio loop on the main thread. We do NOT call
``uvicorn.run`` (it would create a second, competing loop). Instead we build a
``uvicorn.Server`` and schedule its ``serve()`` coroutine onto Isaac's existing
loop with ``asyncio.ensure_future`` -- exactly as the old ConnectionServer did.
Because every request handler then runs on the main thread, the service methods
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
# pylint: enable=line-too-long

import asyncio

import carb
import fastapi
import omni.timeline
import omni.usd
import uvicorn
from fastapi import responses
from pxr import Tf, Usd

from .routers import ALL_ROUTERS
from .services.articulations import ArticulationService
from .services.asset_library import AssetLibraryService
from .services.cameras import CameraService
from .services.conveyors import ConveyorService
from .services.general import GeneralService
from .services.lidars import LidarService
from .services.lightbeams import LightBeamService
from .services.prims import PrimService
from .services.stage import StageService
from .services.surface_grippers import SurfaceGripperService


class BridgeServer:
    """One FastAPI app serving all articulations; owns the uvicorn start()/stop() lifecycle.

    The request logic lives in the services (built and shared in ``_build_app``);
    this class runs the server, clears the device registries on a stage change or a
    stop, restores them from a stage that is opened, unregisters the devices whose
    prims are removed from the stage, and rebinds the articulations such a removal
    broke. The services the extension's GUI uses are public, so it drives the same
    instances the routes do.
    """

    articulation_service: ArticulationService
    asset_library_service: AssetLibraryService
    stage_service: StageService
    surface_gripper_service: SurfaceGripperService

    def __init__(self, host="127.0.0.1", port=8766):
        """Build the FastAPI app (and its services); call `start()` to begin serving."""
        self._host = host
        self._port = port
        # Every service holding bound devices, in restore order, so they are cleared
        # together on a stage change or a stop. Set by _build_app.
        self._device_services = ()
        # Set while no restore is running. Each restore replaces it with its own event,
        # so a cancelled restore that finishes late cannot release requests early.
        self._device_restore_finished = asyncio.Event()
        self._device_restore_finished.set()
        self._restore_task = None
        # Isaac Sim's loop, which every service runs on. A stage notice arrives on the
        # thread that made the change, which need not be this one.
        self._event_loop = asyncio.get_event_loop()
        self._app = self._build_app()
        self._server = None
        self._serve_task = None
        self._objects_changed_listener = None

    def start(self):
        """Schedule uvicorn's serve loop on Isaac Sim's asyncio loop (main thread)."""
        config = uvicorn.Config(
            self._app,
            host=self._host,
            port=self._port,
            lifespan="off",
            log_level="warning",
        )
        self._server = uvicorn.Server(config)
        # We run inside Isaac's loop on the main thread; let Isaac keep its signal
        # handling, and don't let uvicorn hijack Ctrl+C.
        self._server.install_signal_handlers = lambda: None
        self._serve_task = asyncio.ensure_future(self._server.serve())
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            self._listen_for_removed_prims(stage)
        carb.log_info(f"[bridge] server listening on {self._host}:{self._port}")

    def reset_devices(self):
        """Drop every bound device while leaving the server running.

        Called when a stage is closed, and before a stage that is opened is restored:
        the device table holds handles into the previous stage, which are stale the
        moment that stage is replaced. Clearing the registry (rather than restarting
        uvicorn) keeps the port bound and the app alive. A restore still running is
        cancelled, so it cannot register devices into the cleared registries, and the
        previous stage is no longer listened to for removed prims.
        """
        if self._objects_changed_listener is not None:
            self._objects_changed_listener.Revoke()
            self._objects_changed_listener = None
        if self._restore_task is not None:
            self._restore_task.cancel()
            self._restore_task = None
            # Released here: a task cancelled before its first step never runs its finally.
            self._device_restore_finished.set()
        for service in self._device_services:
            service.clear()
        if self._device_services:
            carb.log_info("[bridge] cleared device registry.")

    def restore_devices(self):
        """Rebuild every device registry from the bridge state the open stage stores.

        Called when a stage is opened. The registries are cleared, the ids the stage
        stores are reserved at once, and the devices are then restored in the
        background: surface grippers first, then articulations, which record the
        assemblies between them, then cameras, lidars, lightbeams and conveyors.
        Restoring binds devices, which plays the timeline; once every device is
        restored the timeline is stopped again, unless it was already playing.

        Every request waits until the restore finishes, so no client binds a device
        that stopping the timeline would invalidate, or is handed an id the stage
        stores for another prim. A stage that stores no device changes nothing.
        """
        self.reset_devices()
        stage = omni.usd.get_context().get_stage()
        self._listen_for_removed_prims(stage)
        stored_prim_paths = [
            (service, service.reserve_stored_ids(stage)) for service in self._device_services
        ]
        if not any(prim_paths for _, prim_paths in stored_prim_paths):
            return
        restore_finished = asyncio.Event()
        self._device_restore_finished = restore_finished
        self._restore_task = asyncio.ensure_future(
            self._restore_stored_devices(stored_prim_paths, restore_finished)
        )

    async def _restore_stored_devices(self, stored_prim_paths, restore_finished):
        """Restore each service's devices, then stop the timeline and release requests.

        Args:
            stored_prim_paths: ``(service, prim_paths)`` pairs in restore order, as
                :meth:`restore_devices` reserved them.
            restore_finished: The event this restore sets when it ends, however it ends.
        """
        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        try:
            restored = 0
            for service, prim_paths in stored_prim_paths:
                restored += await service.restore_devices(prim_paths)
            stored = sum(len(prim_paths) for _, prim_paths in stored_prim_paths)
            carb.log_info(f"[bridge] restored {restored} of the {stored} devices the stage stores.")
        finally:
            # A cancelled restore leaves the timeline to the stage event that cancelled it.
            if self._restore_task is asyncio.current_task():
                self._restore_task = None
                if not was_playing:
                    timeline.stop()
            restore_finished.set()

    async def _wait_for_device_restore(self):
        """Hold a request until no device restore is running (see :meth:`restore_devices`)."""
        while not self._device_restore_finished.is_set():
            await self._device_restore_finished.wait()

    def _listen_for_removed_prims(self, stage):
        """Unregister the devices whose prims are removed from ``stage``, until the next reset.

        Only the open stage is listened to: Isaac Sim tools such as the Robot Inspector
        build stages of their own on worker threads, and a change to one of those never
        removes a device's prim.
        """
        self._objects_changed_listener = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._on_stage_objects_changed, stage
        )

    def _on_stage_objects_changed(self, notice, sender):
        """Unregister the devices whose prims a change to the open stage removed.

        Only a change that adds, removes or renames prims can remove a device's prim.
        The stage is then checked on Isaac Sim's event loop rather than here: USD calls
        this while it is still processing the change, on whichever thread made it, and
        releasing a device can edit the stage. Once those devices are unregistered, the
        articulations the removal broke are rebound (see
        ``ArticulationService.rebind_devices_after_prims_removed``).
        """
        if notice.GetResyncedPaths():
            for service in self._device_services:
                self._event_loop.call_soon_threadsafe(service.unregister_removed_devices)
            self._event_loop.call_soon_threadsafe(
                asyncio.ensure_future,
                self.articulation_service.rebind_devices_after_prims_removed(),
            )

    def stop(self):
        """Ask uvicorn to exit and drop every bound device."""
        self.reset_devices()
        if self._server is not None:
            self._server.should_exit = True
            self._server = None
        if self._serve_task is not None:
            self._serve_task.cancel()
            self._serve_task = None
        carb.log_info("[bridge] server stopped.")

    def _build_app(self):
        """Construct the FastAPI app: build the shared services, stash them on
        ``app.state`` for the dependency providers, and mount every router."""
        app = fastapi.FastAPI(
            title="telekinesis isaac-sim bridge",
            dependencies=[fastapi.Depends(self._wait_for_device_restore)],
        )

        @app.exception_handler(Exception)
        async def _unhandled_exception_handler(request: fastapi.Request, exc: Exception):
            # Backstop for anything a service didn't translate into an HTTPException:
            # guarantees the client always gets JSON with a "detail", never Starlette's
            # bare-text 500, which the client's own error-detail printing can't parse.
            carb.log_error(
                f"[bridge] unhandled error on {request.method} {request.url.path}: {exc!r}"
            )
            return responses.JSONResponse(status_code=500, content={"detail": str(exc)})

        # One shared instance of each service for the app's lifetime. The routers
        # reach them via Depends (see .dependencies), keyed by these app.state names.
        # The services that load assets download a catalogue asset through this one.
        asset_library_service = AssetLibraryService()
        surface_gripper_service = SurfaceGripperService(asset_library_service)
        articulation_service = ArticulationService(surface_gripper_service, asset_library_service)
        camera_service = CameraService()
        lidar_service = LidarService()
        lightbeam_service = LightBeamService()
        conveyor_service = ConveyorService()
        stage_service = StageService(asset_library_service)
        # assemble_robot accepts either kind of gripper, so the two registries know
        # about each other: the articulation service resolves a surface gripper id,
        # and a deleted surface gripper drops the assembly record naming it. A
        # surface gripper loaded from a USD asset also stops the timeline, so the
        # articulation service repairs the handles that stop invalidated.
        surface_gripper_service.on_deleted = articulation_service.forget_assembly
        surface_gripper_service.on_timeline_stopped = articulation_service.rebind_stale_devices

        app.state.articulation_service = articulation_service
        app.state.surface_gripper_service = surface_gripper_service
        app.state.camera_service = camera_service
        app.state.lidar_service = lidar_service
        app.state.lightbeam_service = lightbeam_service
        app.state.conveyor_service = conveyor_service
        app.state.stage_service = stage_service
        prim_service = PrimService(stage_service)  # composes the stage service
        # Deleting a prim stops and replays the timeline, so the articulation
        # service repairs the handles that stop invalidated.
        prim_service.on_timeline_stopped = articulation_service.rebind_stale_devices
        app.state.prim_service = prim_service
        app.state.general_service = GeneralService()
        app.state.asset_library_service = asset_library_service
        self.articulation_service = articulation_service
        self.asset_library_service = asset_library_service
        self.stage_service = stage_service
        self.surface_gripper_service = surface_gripper_service
        self._device_services = (
            surface_gripper_service,
            articulation_service,
            camera_service,
            lidar_service,
            lightbeam_service,
            conveyor_service,
        )

        for router in ALL_ROUTERS:
            app.include_router(router)
        return app
