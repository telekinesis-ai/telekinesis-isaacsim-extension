# SPDX-License-Identifier: Apache-2.0
"""The device registry every device service is built on.

:class:`DeviceRegistry` owns what the articulation, surface gripper, camera, lidar,
lightbeam and conveyor registries share: the ``device_id`` -> device table, the id each
requested prim is registered under, the per-prim create locks, id allocation, and the
bridge state stored on each device prim so a saved stage brings its devices back when it
is reopened.

Stored state lives in plain attributes (and relationships) named
``telekinesis:<device_kind>:<name>`` on the requested prim, the path a client PUTs. Every
registered device stores its id there as ``telekinesis:<device_kind>:id``; a service adds
the settings only it holds in memory, such as a camera's resolution. These names and the
``<device_kind><number>`` id format are an on-disk format kept in saved stages: changing
either leaves the devices of stages saved before unrestored.

omni/USD imports are lazy (inside the methods that need them) so this module, like the
articulation service built on it, imports outside Isaac Sim.
"""

import abc
import re

from fastapi import HTTPException


class DeviceRegistry(abc.ABC):
    """The id table, id allocation, stored state and restore of one kind of device.

    A service subclasses it, sets :attr:`device_kind`, and implements
    :meth:`release_device` and :meth:`restore_device`. Its create method allocates the
    device's id with :meth:`allocate_device_id` and registers the device with
    :meth:`register_device`, both inside the prim's create lock. Ids are
    ``<device_kind><number>``, 1-based, and stay with their prim: PUTting the same prim
    again, or reopening a saved stage, gives back the same id.

    When a stage is opened, :meth:`reserve_stored_ids` reserves the ids its prims store,
    before any create can allocate one, and :meth:`restore_devices` then registers the
    device of each such prim. A device whose prim is removed from the stage is
    unregistered by :meth:`unregister_removed_devices`.

    Example:
        class LightBeamService(DeviceRegistry):
            device_kind = "lightbeam"

            def release_device(self, device):
                device.destroy()

            async def restore_device(self, prim_path):
                await self.create_lightbeam(prim_path)
    """

    device_kind: str

    def __init__(self):
        self._devices = {}  # device_id -> device
        self._id_by_prim = {}  # requested prim_path -> device_id
        self._count = 0  # highest id number handed out or stored in the stage
        # prim_path -> asyncio.Lock, serializes concurrent creates of the same prim
        self._create_locks = {}
        # device_id -> the prim that took it, for ids read from the stage. A copy of a
        # device prim carries its source's id, which stays with the prim that took it first.
        self._prim_by_stored_id = {}

    @abc.abstractmethod
    def release_device(self, device):
        """Free what ``device`` holds once the registry drops it.

        Called when the device is deleted, replaced by a new device for the same prim,
        or cleared with the registry, possibly after its stage is gone. Must not raise
        and must not write to the stage. Returns nothing.
        """

    @abc.abstractmethod
    async def restore_device(self, prim_path):
        """Register and bind the device of a prim that stores an id of this kind.

        Called once per such prim when a stage is opened, after :meth:`clear` and
        :meth:`reserve_stored_ids`, while client requests wait for the restore. Create
        the device the way the service's create method does, from the settings the prim
        stores (see :meth:`read_stored_value`) rather than a request's, so it comes back
        under its stored id and as it was saved. Raises when the device cannot be
        restored; :meth:`restore_devices` logs the error and moves on. Returns nothing.
        """

    def clear(self):
        """Drop every registered device, keeping the state the stage stores.

        Called when the bridge stops and when a stage is opened or closed.
        """
        for device in self._devices.values():
            self.release_device(device)
        self._devices = {}
        self._id_by_prim = {}
        self._count = 0
        self._create_locks = {}
        self._prim_by_stored_id = {}

    def get_device(self, device_id):
        """Return the device registered as ``device_id``.

        Raises:
            HTTPException: 404 if no device of this kind has that id.
        """
        device = self._devices.get(device_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=(
                    f"no {self.device_kind.replace('_', ' ')} registered with id '{device_id}', "
                    f"call PUT /{self.device_kind}s to create one"
                ),
            )
        return device

    def list_devices(self):
        """Return a ``{device_id: prim_path}`` map of every registered device."""
        return {device_id: device.prim_path for device_id, device in self._devices.items()}

    def delete_device(self, device_id):
        """Unregister ``device_id`` and remove the state its prim stores.

        The prim itself stays in the stage, so a later create of it gets a fresh id
        and starts from no stored settings.

        Returns:
            ``{"deleted": device_id}``.

        Raises:
            HTTPException: 404 if no device of this kind has that id.
        """
        device = self._devices.get(device_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=f"no {self.device_kind.replace('_', ' ')} registered with id '{device_id}'",
            )
        self.release_device(device)
        del self._devices[device_id]
        self._prim_by_stored_id.pop(device_id, None)
        for prim_path, registered_id in list(self._id_by_prim.items()):
            if registered_id == device_id:
                del self._id_by_prim[prim_path]
                self._create_locks.pop(prim_path, None)
                self._remove_stored_state(prim_path)
        return {"deleted": device_id}

    def unregister_removed_devices(self):
        """Unregister every device whose prim is no longer on the open stage.

        Called after prims are removed from the stage by any means -- the Stage window,
        a script, ``DELETE /prims`` -- or renamed or moved, which removes them from their
        old path. Each such device is unregistered as :meth:`delete_device` does, so an
        assembly it took part in is forgotten too. A prim brought back later, say by
        undo, still stores its id, so a create of it gets that id again. A prim that is
        itself deactivated is still on the stage and stays registered.
        """
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        for device_id, device in list(self._devices.items()):
            if not stage.GetPrimAtPath(device.prim_path):
                self.delete_device(device_id)

    def allocate_device_id(self, prim_path):
        """Return the id a create of ``prim_path`` registers its device under.

        That is the prim's registered id; else the id the prim stores, unless another
        prim holds it; else a fresh one. Called inside the prim's create lock. The id is
        reserved on return, so a concurrent create of another prim cannot be handed the
        same one while this create awaits its bind. A fresh id whose create fails is
        not handed out again.
        """
        registered_id = self._id_by_prim.get(prim_path)
        if registered_id is not None:
            return registered_id
        number = self._stored_id_number(self.read_stored_value(prim_path, "id"))
        if number is not None:
            device_id = f"{self.device_kind}{number}"
            if (
                device_id not in self._devices
                and self._prim_by_stored_id.setdefault(device_id, prim_path) == prim_path
            ):
                self._count = max(self._count, number)
                return device_id
        self._count += 1
        return f"{self.device_kind}{self._count}"

    def register_device(self, prim_path, device_id, device):
        """Register ``device`` as ``device_id`` for ``prim_path`` and store the id on the prim.

        ``device_id`` comes from :meth:`allocate_device_id`. Registering a prim again
        replaces its device without releasing the previous one.
        """
        from pxr import Sdf

        self._devices[device_id] = device
        self._id_by_prim[prim_path] = device_id
        self.write_stored_value(prim_path, "id", Sdf.ValueTypeNames.String, device_id)

    def reserve_stored_ids(self, stage):
        """Reserve the ids the prims of a newly opened ``stage`` store for this kind.

        Called right after :meth:`clear`, before anything awaits, so no create can be
        handed an id a prim of the stage stores. A stored id that is malformed or that
        an earlier prim already stores is not reserved; that prim gets a fresh id when
        it is restored.

        Returns:
            The path of every prim storing an id of this kind, in stage order, for
            :meth:`restore_devices`.
        """
        attribute_name = self.stored_property_name("id")
        prim_paths = []
        for prim in stage.Traverse():
            attribute = prim.GetAttribute(attribute_name)
            if not attribute or not attribute.HasAuthoredValue():
                continue
            prim_path = prim.GetPath().pathString
            prim_paths.append(prim_path)
            number = self._stored_id_number(attribute.Get())
            if number is not None:
                self._count = max(self._count, number)
                self._prim_by_stored_id.setdefault(f"{self.device_kind}{number}", prim_path)
        return prim_paths

    async def restore_devices(self, prim_paths):
        """Restore the device of each of ``prim_paths``, as :meth:`reserve_stored_ids` listed them.

        A prim registered meanwhile is left as it is. A device that cannot be restored
        is logged and skipped, keeping its stored state, so a later create of its prim
        still gets its stored id.

        Returns:
            How many devices were restored.
        """
        import carb

        restored = 0
        for prim_path in prim_paths:
            if prim_path in self._id_by_prim:
                continue
            try:
                await self.restore_device(prim_path)
            except Exception as exc:  # one device that fails must not keep the others back
                carb.log_warn(
                    f"[bridge] could not restore the {self.device_kind.replace('_', ' ')} "
                    f"at {prim_path} from the stage: {exc}"
                )
                continue
            restored += 1
        return restored

    def stored_property_name(self, name):
        """Return ``telekinesis:<device_kind>:<name>``, the full name of a stored property."""
        return f"telekinesis:{self.device_kind}:{name}"

    def read_stored_value(self, prim_path, name):
        """Return the value of the stored attribute ``name`` on ``prim_path``.

        ``None`` when no stage is open, the prim is missing, or it stores no such value.
        """
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path) if stage is not None else None
        attribute = prim.GetAttribute(self.stored_property_name(name)) if prim else None
        return attribute.Get() if attribute else None

    def write_stored_value(self, prim_path, name, value_type, value):
        """Author ``value`` as the stored attribute ``name`` on ``prim_path``.

        ``value_type`` is the attribute's ``Sdf.ValueTypeNames`` type. The value is written
        to the stage's edit target, so it is saved with the stage.
        """
        import omni.usd

        prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
        prim.CreateAttribute(self.stored_property_name(name), value_type).Set(value)

    def remove_stored_property(self, prim_path, name):
        """Remove the stored attribute or relationship ``name`` from ``prim_path``, if present."""
        import omni.usd

        prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
        prim.RemoveProperty(self.stored_property_name(name))

    def _remove_stored_state(self, prim_path):
        """Remove every stored property of this kind from ``prim_path``, if the prim exists."""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path) if stage is not None else None
        if not prim:
            return
        for stored_property in prim.GetPropertiesInNamespace(f"telekinesis:{self.device_kind}"):
            prim.RemoveProperty(stored_property.GetName())

    def _stored_id_number(self, stored_id):
        """Return the number of a stored ``<device_kind><number>`` id, or ``None`` if malformed."""
        if not isinstance(stored_id, str):
            return None
        match = re.fullmatch(rf"{re.escape(self.device_kind)}([1-9][0-9]*)", stored_id)
        return int(match.group(1)) if match else None
