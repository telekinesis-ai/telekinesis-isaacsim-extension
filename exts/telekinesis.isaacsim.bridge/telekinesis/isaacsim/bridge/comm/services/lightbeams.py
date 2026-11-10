# SPDX-License-Identifier: Apache-2.0
"""The lightbeam registry: the orchestration layer behind the lightbeam HTTP routes.

``LightBeamService`` is the sibling of :class:`..services.lidars.LidarService` for the
other legacy PhysX range sensor: the :class:`.device_registry.DeviceRegistry` mapping
``lightbeam_id`` -> :class:`..core.lightbeam.LightBeamSensor`, plus the operations on
it -- create (bind), look up, delete, list, read the beams, and get/set the beam
layout.

It owns mutable state (the device table), so exactly one instance
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

import omni.usd
from fastapi import HTTPException

from ...core.lightbeam import LightBeamSensor
from .device_registry import DeviceRegistry


class LightBeamService(DeviceRegistry):
    """The lightbeam registry shared by every request.

    Holds the ``lightbeam_id`` -> device table and exposes create / get / delete /
    list, the beam reading, and the layout getters and setters. Each sensor's prim
    stores its id, so reopening a saved stage restores the sensor under the same id,
    paused if it was saved paused. One instance per running bridge.
    """

    device_kind = "lightbeam"

    def release_device(self, device):
        """Best-effort ``device.destroy()`` -- swallow errors so teardown (which may
        run after the stage is already gone) never raises."""
        try:
            device.destroy()
        except Exception:  # cleanup must not fail the caller (stage may be gone)
            pass

    async def restore_device(self, prim_path):
        """Create the sensor at ``prim_path`` and pause it again if it was saved paused."""
        # Creating a sensor enables it, so the prim's own enabled state is read first.
        sensor_prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
        paused = not sensor_prim.GetAttribute("enabled").Get()
        lightbeam = await self.create_lightbeam(prim_path)
        if paused:
            self.pause(lightbeam["lightbeam_id"])

    async def create_lightbeam(self, prim_path):
        """Register (and bind) the lightbeam sensor at ``prim_path`` and return its info.

        One sensor per *requested* prim; PUTting the same prim again keeps its id and
        re-binds it, which is what a sensor needs after the timeline has been stopped
        and replayed. Ids are 1-based: ``lightbeam1``, ``lightbeam2``, ... The id is
        stored on the prim, so it survives saving and reopening the stage.
        """
        prim_path = prim_path.rstrip("/") or "/"

        # Serialize concurrent creates of the SAME prim_path (mirrors the lidar
        # service): two clients racing to register the same sensor would otherwise both
        # allocate an id/device, the second clobbering the first. setdefault is
        # synchronous, so concurrent callers land on one Lock.
        lock = self._create_locks.setdefault(prim_path, asyncio.Lock())
        async with lock:
            lightbeam_id = self.allocate_device_id(prim_path)

            # Build + bind the NEW device before touching any existing one, so a bad
            # re-PUT leaves the currently-registered sensor untouched and working.
            try:
                device = LightBeamSensor(prim_path, name=lightbeam_id)
            except ValueError as exc:
                # Bad input value: a prim that is not a lightbeam sensor, raised at
                # construction. 400, not the 500 a bare Exception would otherwise
                # become.
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: well-formed request, but the sensor couldn't be brought up. Free
                # its partial resources.
                self.release_device(device)
                raise HTTPException(status_code=422, detail=str(exc)) from exc

            # Success: commit. Free any prior device for this prim so a re-PUT releases
            # the old one instead of leaking it to the GC.
            previous = self._devices.get(lightbeam_id)
            if previous is not None and previous is not device:
                self.release_device(previous)
            self.register_device(prim_path, lightbeam_id, device)
            return {"lightbeam_id": lightbeam_id, **device.info()}

    def get_lightbeam(self, lightbeam_id):
        """Info for one registered sensor (id, prim, beam layout, range), or 404."""
        device = self.get_device(lightbeam_id)
        return {"lightbeam_id": lightbeam_id, **device.info()}

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
