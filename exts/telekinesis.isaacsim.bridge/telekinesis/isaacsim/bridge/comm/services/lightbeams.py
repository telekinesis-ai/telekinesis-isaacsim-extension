# SPDX-License-Identifier: Apache-2.0
"""The lightbeam registry: the orchestration layer behind the lightbeam HTTP routes.

``LightBeamService`` is the sibling of :class:`..services.lidars.LidarService` for the
other legacy PhysX range sensor: an in-memory table mapping ``lightbeam_id`` ->
:class:`..core.lightbeam.LightBeamSensor`, plus the operations on it -- create
(bind), look up, delete, list, read the beams, and get/set the beam layout.

It owns mutable state (the device table and the id counter), so exactly one instance
is shared across all requests -- :class:`BridgeServer` builds it once and stashes it
on ``app.state`` for the routers to reach via ``Depends`` (see
:mod:`..comm.dependencies`).

Transport coupling is deliberately minimal: bad input raises
``fastapi.HTTPException`` (400), a failed bind raises it (422) and reading a stopped
simulation raises it (409), so the routers stay one-liners. The ``..core.lightbeam``
import pulls in isaacsim, so this module imports only inside Isaac Sim -- same as the
lidar service.

Wire units mirror the rest of the bridge: meters for ranges and distances.
"""

import asyncio

from fastapi import HTTPException

from ...core.lightbeam import LightBeamSensor


class LightBeamService:
    """The lightbeam registry shared by every request.

    Holds the ``lightbeam_id`` -> device table and an id counter, and exposes create /
    get / delete / list, the beam reading, and the layout getters and setters. One
    instance per running bridge.
    """

    def __init__(self):
        self._devices = {}  # lightbeam_id -> LightBeamSensor
        self._id_by_prim = {}  # requested prim_path -> lightbeam_id (stable on re-create)
        self._count = 0  # for ids like lightbeam1, lightbeam2
        # prim_path -> asyncio.Lock, serializes concurrent creates of the same prim
        self._create_locks = {}

    def clear(self):
        """Drop every bound sensor (called when the bridge stops or the stage changes)."""
        for device in self._devices.values():
            self._safe_destroy(device)
        self._devices = {}
        self._id_by_prim = {}
        self._count = 0
        self._create_locks = {}

    @staticmethod
    def _safe_destroy(device):
        """Best-effort ``device.destroy()`` -- swallow errors so teardown (which may
        run after the stage is already gone) never raises."""
        try:
            device.destroy()
        except Exception:  # cleanup must not fail the caller (stage may be gone)
            pass

    async def create_lightbeam(self, prim_path):
        """Register (and bind) the lightbeam sensor at ``prim_path`` and return its info.

        One sensor per *requested* prim; PUTting the same prim again keeps its id and
        re-binds it, which is what a sensor needs after the timeline has been stopped
        and replayed. Ids are 1-based: ``lightbeam1``, ``lightbeam2``, ...
        """
        prim_path = prim_path.rstrip("/") or "/"

        # Serialize concurrent creates of the SAME prim_path (mirrors the lidar
        # service): two clients racing to register the same sensor would otherwise both
        # allocate an id/device, the second clobbering the first. setdefault is
        # synchronous, so concurrent callers land on one Lock.
        lock = self._create_locks.setdefault(prim_path, asyncio.Lock())
        async with lock:
            existing_id = self._id_by_prim.get(prim_path)
            if existing_id is None:
                # Reserve a fresh id synchronously -- before the bind() await below,
                # which yields the loop. The per-prim lock does NOT serialize creates
                # of *different* prims, so deferring the increment past the await
                # would let two of them grab the same lightbeamN.
                self._count += 1
                lightbeam_id = f"lightbeam{self._count}"
            else:
                lightbeam_id = existing_id

            # Build + bind the NEW device before touching any existing one, so a bad
            # re-PUT leaves the currently-registered sensor untouched and working.
            try:
                device = LightBeamSensor(prim_path, name=lightbeam_id)
            except ValueError as exc:
                # Bad input value: a prim that is not a lightbeam sensor, raised at
                # construction. 400, not the 500 a bare Exception would otherwise
                # become. Construction has no await, so rolling back a freshly-reserved
                # id here is race-free.
                if existing_id is None:
                    self._count -= 1
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: well-formed request, but the sensor couldn't be brought up. Free
                # its partial resources.
                self._safe_destroy(device)
                raise HTTPException(status_code=422, detail=str(exc)) from exc

            # Success: commit. Free any prior device for this prim so a re-PUT releases
            # the old one instead of leaking it to the GC.
            previous = self._devices.get(lightbeam_id)
            if previous is not None and previous is not device:
                self._safe_destroy(previous)
            self._id_by_prim[prim_path] = lightbeam_id  # no-op on re-PUT; commits a fresh id
            self._devices[lightbeam_id] = device
            return {"lightbeam_id": lightbeam_id, **device.info()}

    def get_lightbeam(self, lightbeam_id):
        """Info for one registered sensor (id, prim, beam layout, range), or 404."""
        device = self.get_device(lightbeam_id)
        return {"lightbeam_id": lightbeam_id, **device.info()}

    def delete_lightbeam(self, lightbeam_id):
        """Unregister the sensor (the USD prim is left in the stage). 404 if unknown."""
        device = self._devices.get(lightbeam_id)
        if device is None:
            raise HTTPException(
                status_code=404, detail=f"no lightbeam registered with id '{lightbeam_id}'"
            )
        self._safe_destroy(device)
        del self._devices[lightbeam_id]
        for prim, registered_id in list(self._id_by_prim.items()):
            if registered_id == lightbeam_id:
                del self._id_by_prim[prim]
                self._create_locks.pop(prim, None)
        return {"deleted": lightbeam_id}

    def list_lightbeams(self):
        """Return a ``{lightbeam_id: prim_path}`` map of every registered sensor."""
        return {
            lightbeam_id: device.prim_path
            for lightbeam_id, device in self._devices.items()
        }

    # -- readings --------------------------------------------------------------

    def read(self, lightbeam_id):
        """The beams as of the last physics step. See
        :meth:`..core.lightbeam.LightBeamSensor.read`."""
        try:
            return self.get_device(lightbeam_id).read()
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    # -- beam layout / range config -------------------------------------------

    def set_configuration(
        self,
        lightbeam_id,
        num_rays,
        curtain_length,
        forward_axis,
        curtain_axis,
        min_range,
        max_range,
    ):
        """Set the beam layout and range (fields left null are untouched); returns them."""
        device = self.get_device(lightbeam_id)
        try:
            device.configure(
                num_rays=num_rays,
                curtain_length=curtain_length,
                forward_axis=forward_axis,
                curtain_axis=curtain_axis,
                min_range=min_range,
                max_range=max_range,
            )
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return device.info()

    # -- collection control ----------------------------------------------------

    def pause(self, lightbeam_id):
        """Stop PhysX computing this sensor."""
        self.get_device(lightbeam_id).pause()
        return {"paused": True}

    def resume(self, lightbeam_id):
        """Resume sensor computation."""
        self.get_device(lightbeam_id).resume()
        return {"paused": False}

    def is_paused(self, lightbeam_id):
        """Whether sensor computation is currently switched off."""
        return {"paused": self.get_device(lightbeam_id).is_paused()}

    # -- internals -------------------------------------------------------------

    def get_device(self, lightbeam_id):
        """Resolve a ``lightbeam_id`` to its device object, or 404."""
        device = self._devices.get(lightbeam_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=(
                    f"no lightbeam registered with id '{lightbeam_id}', call PUT "
                    "/lightbeams to create one"
                ),
            )
        return device
