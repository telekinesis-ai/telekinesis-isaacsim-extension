# SPDX-License-Identifier: Apache-2.0
"""The conveyor registry: the orchestration layer behind the conveyor HTTP routes.

``ConveyorService`` is the actuator counterpart to
:class:`..services.lidars.LidarService`: the :class:`.device_registry.DeviceRegistry`
mapping ``conveyor_id`` -> :class:`..core.conveyor.Conveyor`, plus the operations on
it -- create (bind), look up, delete, list, start and stop.

It owns mutable state (the device table), so exactly one instance
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

import omni.usd
from fastapi import HTTPException
from pxr import Gf, Sdf

from ...core.conveyor import Conveyor
from .device_registry import DeviceRegistry


class ConveyorService(DeviceRegistry):
    """The conveyor registry shared by every request.

    Holds the ``conveyor_id`` -> device table and exposes create / get / delete / list
    plus start and stop. Each conveyor's prim stores its id, cargo root and the travel
    direction and speed captured from it, so reopening a saved stage restores the
    conveyor under the same id, running in the direction the scene authored even when
    the belt was saved running in reverse. One instance per running bridge.
    """

    device_kind = "conveyor"

    def release_device(self, device):
        """Best-effort ``device.destroy()`` -- swallow errors so teardown (which may
        run after the stage is already gone) never raises.

        Belts that were left running keep running: the registry holds no authority
        over the stage.
        """
        try:
            device.destroy()
        except Exception:  # cleanup must not fail the caller (stage may be gone)
            pass

    async def restore_device(self, prim_path):
        """Create the conveyor at ``prim_path`` with the cargo root its prim stores."""
        relationship = (
            omni.usd.get_context()
            .get_stage()
            .GetPrimAtPath(prim_path)
            .GetRelationship(self.stored_property_name("cargo_root"))
        )
        targets = relationship.GetTargets() if relationship else []
        await self.create_conveyor(prim_path, targets[0].pathString if targets else None)

    async def create_conveyor(self, prim_path, cargo_root):
        """Register (and bind) the conveyor at ``prim_path`` and return its info.

        One conveyor per *requested* prim; PUTting the same prim again keeps its id but
        **rebuilds the device**, so the response always reflects the request. A belt
        that is currently running is not stopped by that, and its travel direction and
        authored speed are reused from its first binding rather than re-read, because
        by then the attribute they were read from holds whatever speed was last
        commanded. Ids are 1-based: ``conveyor1``, ``conveyor2``, ...

        The id, the cargo root and that first capture are stored on the prim, so they
        survive saving and reopening the stage; ``DELETE /conveyors/{id}`` removes them,
        and the next PUT reads the belt again.

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
            conveyor_id = self.allocate_device_id(prim_path)

            # Build + bind the NEW device before touching any existing one, so a bad
            # re-PUT leaves the currently-registered conveyor untouched and working.
            try:
                device = Conveyor(
                    prim_path,
                    name=conveyor_id,
                    cargo_root=cargo_root,
                    authored=self._read_stored_capture(prim_path),
                )
            except ValueError as exc:
                # Bad input value: a prim that is not a conveyor, or one whose travel
                # direction cannot be read. 400, not the 500 a bare Exception would
                # otherwise become.
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: well-formed request, but the belt couldn't be brought up.
                self.release_device(device)
                raise HTTPException(status_code=422, detail=str(exc)) from exc

            # Success: commit. Free any prior device for this prim so a re-PUT releases
            # the old one instead of leaking it to the GC.
            previous = self._devices.get(conveyor_id)
            if previous is not None and previous is not device:
                self.release_device(previous)
            self.register_device(prim_path, conveyor_id, device)
            self._store_capture(prim_path, device.authored)
            if cargo_root is None:
                self.remove_stored_property(prim_path, "cargo_root")
            else:
                omni.usd.get_context().get_stage().GetPrimAtPath(prim_path).CreateRelationship(
                    self.stored_property_name("cargo_root")
                ).SetTargets([Sdf.Path(cargo_root)])
            return {"conveyor_id": conveyor_id, **device.info()}

    def _read_stored_capture(self, prim_path):
        """Return the capture a previous binding of the belt stored on ``prim_path``.

        In the form :attr:`..core.conveyor.Conveyor.authored` gives it, or ``None``
        when the prim stores none, so the belt is read from the stage.
        """
        drive = self.read_stored_value(prim_path, "authored_drive")
        if drive is None:
            return None
        direction = self.read_stored_value(prim_path, "authored_direction")
        return {
            "drive": drive,
            "attribute": self.read_stored_value(prim_path, "authored_attribute"),
            "direction": None if direction is None else tuple(direction),
            "nominal_speed": self.read_stored_value(prim_path, "authored_nominal_speed"),
        }

    def _store_capture(self, prim_path, authored):
        """Store ``authored``, a :attr:`..core.conveyor.Conveyor.authored` capture, on a prim."""
        self.write_stored_value(
            prim_path, "authored_drive", Sdf.ValueTypeNames.Token, authored["drive"]
        )
        self.write_stored_value(
            prim_path, "authored_attribute", Sdf.ValueTypeNames.Token, authored["attribute"]
        )
        self.write_stored_value(
            prim_path,
            "authored_nominal_speed",
            Sdf.ValueTypeNames.Double,
            authored["nominal_speed"],
        )
        if authored["direction"] is None:
            self.remove_stored_property(prim_path, "authored_direction")
        else:
            self.write_stored_value(
                prim_path,
                "authored_direction",
                Sdf.ValueTypeNames.Float3,
                Gf.Vec3f(*authored["direction"]),
            )

    def get_conveyor(self, conveyor_id):
        """Info for one registered conveyor (id, prims, drive, speed, running), or 404."""
        device = self.get_device(conveyor_id)
        return {"conveyor_id": conveyor_id, **device.info()}

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
