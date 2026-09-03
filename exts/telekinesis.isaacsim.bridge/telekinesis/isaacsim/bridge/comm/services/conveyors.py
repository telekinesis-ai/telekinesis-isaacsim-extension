# SPDX-License-Identifier: Apache-2.0
"""The conveyor registry: the orchestration layer behind the conveyor HTTP routes.

``ConveyorService`` is the actuator counterpart to
:class:`..services.lidars.LidarService`: an in-memory table mapping ``conveyor_id``
-> :class:`..core.conveyor.Conveyor`, plus the operations on it -- create (bind),
look up, delete, list, start and stop.

It owns mutable state (the device table and the id counter), so exactly one instance
is shared across all requests -- :class:`BridgeServer` builds it once and stashes it
on ``app.state`` for the routers to reach via ``Depends`` (see
:mod:`..comm.dependencies`).

Transport coupling is deliberately minimal: bad input raises
``fastapi.HTTPException`` (400) and a failed bind raises it (422) so the routers stay
one-liners. The ``..core.conveyor`` import pulls in isaacsim, so this module imports
only inside Isaac Sim -- same as the lidar service.

Wire units mirror the rest of the bridge: meters per second along the belt's authored
travel direction, radians per second for a curved belt.
"""

import asyncio

from fastapi import HTTPException

from ...core.conveyor import Conveyor


class ConveyorService:
    """The conveyor registry shared by every request.

    Holds the ``conveyor_id`` -> device table and an id counter, and exposes create /
    get / delete / list plus start and stop. One instance per running bridge.
    """

    def __init__(self):
        self._devices = {}  # conveyor_id -> Conveyor
        self._id_by_prim = {}  # requested prim_path -> conveyor_id (stable on re-create)
        self._count = 0  # for ids like conveyor1, conveyor2
        # prim_path -> asyncio.Lock, serializes concurrent creates of the same prim
        self._create_locks = {}

    def clear(self):
        """Drop every bound conveyor (called when the bridge stops or the stage changes).

        Belts that were left running keep running: the registry holds no authority over
        the stage, and a stage change has taken the belt with it anyway.
        """
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

    async def create_conveyor(self, prim_path, cargo_root):
        """Register (and bind) the conveyor at ``prim_path`` and return its info.

        One conveyor per *requested* prim; PUTting the same prim again keeps its id but
        **rebuilds the device**, re-reading the belt's authored velocity from the stage,
        so the response always reflects the request. A belt that is currently running
        is not stopped by that, but its authored running speed is then read back from a
        belt somebody has already overwritten -- register belts at rest. Ids are
        1-based: ``conveyor1``, ``conveyor2``, ...

        Binding plays the timeline: a stopped simulation is the one state in which a
        belt silently does nothing (see :meth:`..core.conveyor.Conveyor.bind`).
        """
        prim_path = prim_path.rstrip("/") or "/"

        # Serialize concurrent creates of the SAME prim_path (mirrors the lidar
        # service): two clients racing to register the same conveyor would otherwise
        # both allocate an id/device, the second clobbering the first. setdefault is
        # synchronous, so concurrent callers land on one Lock.
        lock = self._create_locks.setdefault(prim_path, asyncio.Lock())
        async with lock:
            existing_id = self._id_by_prim.get(prim_path)
            if existing_id is None:
                # Reserve a fresh id synchronously -- before the bind() await below,
                # which yields the loop. The per-prim lock does NOT serialize creates
                # of *different* prims, so deferring the increment past the await
                # would let two of them grab the same conveyorN.
                self._count += 1
                conveyor_id = f"conveyor{self._count}"
            else:
                conveyor_id = existing_id

            # Build + bind the NEW device before touching any existing one, so a bad
            # re-PUT leaves the currently-registered conveyor untouched and working.
            try:
                device = Conveyor(prim_path, name=conveyor_id, cargo_root=cargo_root)
            except ValueError as exc:
                # Bad input value: a prim that is not a conveyor, or one whose travel
                # direction cannot be read. 400, not the 500 a bare Exception would
                # otherwise become. Construction has no await, so rolling back a
                # freshly-reserved id here is race-free.
                if existing_id is None:
                    self._count -= 1
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: well-formed request, but the belt couldn't be brought up.
                self._safe_destroy(device)
                raise HTTPException(status_code=422, detail=str(exc)) from exc

            # Success: commit. Free any prior device for this prim so a re-PUT releases
            # the old one instead of leaking it to the GC.
            previous = self._devices.get(conveyor_id)
            if previous is not None and previous is not device:
                self._safe_destroy(previous)
            self._id_by_prim[prim_path] = conveyor_id  # no-op on re-PUT; commits an id
            self._devices[conveyor_id] = device
            return {"conveyor_id": conveyor_id, **device.info()}

    def get_conveyor(self, conveyor_id):
        """Info for one registered conveyor (id, prims, drive, speed, running), or 404."""
        device = self.get_device(conveyor_id)
        return {"conveyor_id": conveyor_id, **device.info()}

    def delete_conveyor(self, conveyor_id):
        """Unregister the conveyor (the USD prim is left in the stage, and a running belt
        keeps running). 404 if unknown."""
        device = self._devices.get(conveyor_id)
        if device is None:
            raise HTTPException(
                status_code=404, detail=f"no conveyor registered with id '{conveyor_id}'"
            )
        self._safe_destroy(device)
        del self._devices[conveyor_id]
        for prim, registered_id in list(self._id_by_prim.items()):
            if registered_id == conveyor_id:
                del self._id_by_prim[prim]
                self._create_locks.pop(prim, None)
        return {"deleted": conveyor_id}

    def list_conveyors(self):
        """Return a ``{conveyor_id: prim_path}`` map of every registered conveyor."""
        return {
            conveyor_id: device.prim_path for conveyor_id, device in self._devices.items()
        }

    # -- control ---------------------------------------------------------------

    def start(self, conveyor_id, velocity):
        """Run the belt (``velocity`` None = its authored speed) and report its state.

        ``woken_bodies`` is how many sleeping rigid bodies under the conveyor's cargo
        root were woken so the starting belt can pick them up -- zero when no cargo
        root was configured, or while the timeline is stopped.
        """
        device = self.get_device(conveyor_id)
        try:
            woken = device.start(velocity)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"conveyor_id": conveyor_id, "woken_bodies": woken, **device.info()}

    def stop(self, conveyor_id):
        """Stop the belt and report its state."""
        device = self.get_device(conveyor_id)
        device.stop()
        return {"conveyor_id": conveyor_id, **device.info()}

    # -- internals -------------------------------------------------------------

    def get_device(self, conveyor_id):
        """Resolve a ``conveyor_id`` to its device object, or 404."""
        device = self._devices.get(conveyor_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=(
                    f"no conveyor registered with id '{conveyor_id}', call PUT /conveyors "
                    "to create one"
                ),
            )
        return device
