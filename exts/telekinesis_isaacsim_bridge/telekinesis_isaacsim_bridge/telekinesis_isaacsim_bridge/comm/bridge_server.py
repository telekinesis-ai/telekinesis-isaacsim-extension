# SPDX-License-Identifier: Apache-2.0
"""
References(importing routes)
- https://github.com/fastapi/full-stack-fastapi-template/blob/master/backend/app/api/main.py
- https://github.com/fastapi/full-stack-fastapi-template/blob/2a6eeda62976e97d6e7104648e2011b4ab10ecda/backend/app/main.py#L30 

The single bridge server. One FastAPI app on one well-known port serves every
device in the scene; devices are addressed by an opaque ``device_id`` handed back
from ``/connect`` (not by TCP port, as the old per-device-server design did).

Structure
---------
``BridgeServer`` plays two roles:

* the **uvicorn lifecycle** (``start`` / ``stop``), and
* the **device registry** -- the in-memory table mapping ``device_id`` ->
  articulation, plus the ``connect`` / ``disconnect`` / ``get`` / ``list_devices``
  operations on it.

The HTTP handlers live in :mod:`.routers` as plain ``APIRouter``s grouped by
domain (lifecycle / robot / gripper). ``_build_app`` stashes this instance on
``app.state.registry`` so those routers reach the device table via a small
``Depends`` -- no closures over ``self``, no circular import. Request bodies live
in :mod:`.models`.

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
to two different ``/devices/{device_id}/...`` paths on the same port; they run as
concurrent coroutines on Isaac's loop and both advance every physics step. Each
device's own ``_move_lock`` serializes commands to that one device.

Wire units (shared mental model with the Synapse client)
-------------------------------------------------------
Native Isaac units: radians for joints, meters for lengths. Gripper ``fraction``
is closed-ness: 0.0 = fully open, 1.0 = fully closed.
"""

import asyncio

import omni.usd
import uvicorn
from fastapi import FastAPI, HTTPException

from .routers import ALL_ROUTERS


class BridgeServer:
    """One FastAPI app serving all devices; mirrors the old start()/stop() shape.

    Also acts as the device registry the routers depend on (``connect`` /
    ``disconnect`` / ``get`` / ``list_devices``).
    """

    def __init__(self, host="127.0.0.1", port=8765):
        self._host = host
        self._port = port
        self._devices = {}      # device_id -> RobotArticulation | GripperArticulation
        self._id_by_prim = {}   # resolved prim_path -> device_id (stable on reconnect)
        self._counter = 0
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
        if self._server is not None:
            self._server.should_exit = True
            self._server = None
        if self._serve_task is not None:
            self._serve_task.cancel()
            self._serve_task = None
        print("[bridge] server stopped.")

    # -- device registry (used by the routers via app.state.registry) -------

    async def connect(self, prim_path, device_type, urdf_path):
        """Bind (or rebind) the device at ``prim_path`` and return its info.

        One device per resolved prim; reconnecting to it returns the same id. The
        bind runs on every connect: a new client session may follow a timeline
        stop/replay, which invalidates the cached articulation handle.
        """
        raise NotImplementedError()

    def disconnect(self, device_id):
        """Drop the binding for one device (counterpart to ``connect``)."""
        raise NotImplementedError()

    def get(self, device_id):
        """Resolve a ``device_id`` to its articulation, or 404."""
        device = self._devices.get(device_id)
        if device is None:
            raise HTTPException(status_code=404, detail=f"unknown device_id '{device_id}'")
        return device

    def list_devices(self):
        return {device_id: device.prim_path for device_id, device in self._devices.items()}

    # -- internals ----------------------------------------------------------

    def _make_device(self, device_type, prim_path):
        raise NotImplementedError()

    def _resolve_prim(self, requested_prim_path, urdf_path):
        """Use the prim if it's already in the stage, else import the URDF at it."""
        raise NotImplementedError()

    def _build_app(self):
        app = FastAPI(title="telekinesis isaac-sim bridge")
        # The routers reach this device table via Depends(get_registry).
        app.state.registry = self
        for router in ALL_ROUTERS:
            app.include_router(router)
        return app
