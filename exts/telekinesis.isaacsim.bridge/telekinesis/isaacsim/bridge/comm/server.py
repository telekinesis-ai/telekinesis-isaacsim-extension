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

import uvicorn
import fastapi
from fastapi import responses
import carb

from .services.articulations import ArticulationService
from .services.cameras import CameraService
from .services.general import GeneralService
from .services.prims import PrimService
from .services.stage import StageService
from .services.surface_grippers import SurfaceGripperService
from .routers import ALL_ROUTERS


class BridgeServer:
    """One FastAPI app serving all articulations; owns the uvicorn start()/stop() lifecycle.

    The request logic lives in the services (built and shared in ``_build_app``);
    this class only runs the server and clears the articulation registry on stop.
    """

    def __init__(self, host="127.0.0.1", port=8766):
        """Build the FastAPI app (and its services); call `start()` to begin serving."""
        self._host = host
        self._port = port
        # Every service holding bound devices, so they are cleared together on a
        # stage change or a stop. Set by _build_app.
        self._device_services = ()
        self._app = self._build_app()
        self._server = None
        self._serve_task = None

    def start(self):
        """Schedule uvicorn's serve loop on Isaac Sim's asyncio loop (main thread)."""
        config = uvicorn.Config(
            self._app, host=self._host, port=self._port, lifespan="off", log_level="warning"
        )
        self._server = uvicorn.Server(config)
        # We run inside Isaac's loop on the main thread; let Isaac keep its signal
        # handling, and don't let uvicorn hijack Ctrl+C.
        self._server.install_signal_handlers = lambda: None
        self._serve_task = asyncio.ensure_future(self._server.serve())
        carb.log_info(f"[bridge] server listening on {self._host}:{self._port}")

    def reset_devices(self):
        """Drop every bound device while leaving the server running.

        Called when a new stage is opened: the device table holds
        SingleArticulation handles into the previous stage, which are stale the
        moment that stage is replaced. Clearing the registry (rather than
        restarting uvicorn) keeps the port bound and the app alive -- clients
        just re-PUT their articulations against the new stage.
        """
        for service in self._device_services:
            service.clear()
        if self._device_services:
            carb.log_info("[bridge] cleared device registry (stage changed).")

    def stop(self):
        """Ask uvicorn to exit and drop every bound device."""
        for service in self._device_services:
            service.clear()
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
        app = fastapi.FastAPI(title="telekinesis isaac-sim bridge")

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
        surface_gripper_service = SurfaceGripperService()
        articulation_service = ArticulationService(surface_gripper_service)
        camera_service = CameraService()
        stage_service = StageService()
        # assemble_robot accepts either kind of gripper, so the two registries know
        # about each other: the articulation service resolves a surface gripper id,
        # and a deleted surface gripper drops the assembly record naming it.
        surface_gripper_service.on_deleted = articulation_service.forget_assembly

        app.state.articulation_service = articulation_service
        app.state.surface_gripper_service = surface_gripper_service
        app.state.camera_service = camera_service
        app.state.stage_service = stage_service
        app.state.prim_service = PrimService(stage_service)  # composes the stage service
        app.state.general_service = GeneralService()
        self._device_services = (articulation_service, surface_gripper_service, camera_service)

        for router in ALL_ROUTERS:
            app.include_router(router)
        return app
