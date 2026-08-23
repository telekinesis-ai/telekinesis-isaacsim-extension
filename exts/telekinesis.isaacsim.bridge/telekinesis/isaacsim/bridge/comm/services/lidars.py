# SPDX-License-Identifier: Apache-2.0
"""The lidar registry: the orchestration layer behind the lidar HTTP routes.

``LidarService`` is the sensor counterpart to
:class:`..services.cameras.CameraService`: an in-memory table mapping
``lidar_id`` -> :class:`..core.lidar.Lidar`, plus the operations on it -- create
(wrap/create + bind), look up, delete, list, capture sensor outputs, and
get/set the lidar's pose and scan configuration.

It owns mutable state (the device table and the id counter), so exactly one
instance is shared across all requests -- :class:`BridgeServer` builds it once
and stashes it on ``app.state`` for the routers to reach via ``Depends`` (see
:mod:`..comm.dependencies`).

Transport coupling is deliberately minimal: bad input raises
``fastapi.HTTPException`` (400) and a failed bind raises it (422) so the
routers stay one-liners. The ``..core.lidar`` import pulls in isaacsim, so
this module imports only inside Isaac Sim -- same as the camera service.

Wire units mirror the rest of the bridge: stage units (meters) for
poses/range, degrees for fov/resolution/yaw, radians for the runtime
azimuth/zenith buffers, plain arrays for scan data.
"""

import asyncio

from fastapi import HTTPException

from ...core.lidar import Lidar


class LidarService:
    """The lidar registry shared by every request.

    Holds the ``lidar_id`` -> device table and an id counter, and exposes
    create / get / delete / list, capture, and the pose/config getters and
    setters. One instance per running bridge.
    """

    def __init__(self):
        self._devices = {}  # lidar_id -> Lidar
        self._id_by_prim = {}  # requested prim_path -> lidar_id (stable on re-create)
        self._count = 0  # for ids like lidar1, lidar2
        # prim_path -> asyncio.Lock, serializes concurrent creates of the same prim
        self._create_locks = {}

    def clear(self):
        """Drop every bound lidar (called when the bridge stops or the stage changes)."""
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

    async def create_lidar(
        self,
        prim_path,
        min_range,
        max_range,
        horizontal_fov,
        vertical_fov,
        horizontal_resolution,
        vertical_resolution,
        rotation_rate,
        high_lod,
        draw_points,
        draw_lines,
        yaw_offset,
        data_types,
    ):
        """Register (and bind) the lidar at ``prim_path`` and return its info.

        One lidar per *requested* prim; PUTting the same prim again keeps its
        id but **rebuilds the device with the new config** (range / fov /
        resolution / rotation_rate / data_types / ...), so the response always
        reflects the request. Ids are 1-based: ``lidar1``, ``lidar2``, ...
        """
        prim_path = prim_path.rstrip("/") or "/"

        # Serialize concurrent creates of the SAME prim_path (mirrors the
        # camera service): two clients racing to register the same lidar would
        # otherwise both allocate an id/device, the second clobbering the
        # first. setdefault is synchronous, so concurrent callers land on one Lock.
        lock = self._create_locks.setdefault(prim_path, asyncio.Lock())
        async with lock:
            existing_id = self._id_by_prim.get(prim_path)
            if existing_id is None:
                # Reserve a fresh id synchronously -- before the bind() await below,
                # which yields the loop. The per-prim lock does NOT serialize creates
                # of *different* prims, so deferring the increment past the await
                # would let two of them grab the same lidarN.
                self._count += 1
                lidar_id = f"lidar{self._count}"
            else:
                lidar_id = existing_id

            # Build + bind the NEW device before touching any existing one, so a bad
            # re-PUT (bad config, or a prim that won't bind) leaves the
            # currently-registered lidar untouched and working.
            try:
                device = Lidar(
                    prim_path,
                    name=lidar_id,
                    min_range=min_range,
                    max_range=max_range,
                    horizontal_fov=horizontal_fov,
                    vertical_fov=vertical_fov,
                    horizontal_resolution=horizontal_resolution,
                    vertical_resolution=vertical_resolution,
                    rotation_rate=rotation_rate,
                    high_lod=high_lod,
                    draw_points=draw_points,
                    draw_lines=draw_lines,
                    yaw_offset=yaw_offset,
                    data_types=data_types,
                )
            except Exception as exc:
                # Bad input value: unknown data_type, or a prim that isn't a
                # Lidar -- all raised at construction. 400, not the 500 a bare
                # Exception would otherwise become. Construction has no await,
                # so rolling back a freshly-reserved id here is race-free.
                if existing_id is None:
                    self._count -= 1
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: well-formed request, but the lidar couldn't be brought
                # up. Free its partial resources.
                self._safe_destroy(device)
                raise HTTPException(status_code=422, detail=str(exc)) from exc

            # Success: commit. Free any prior device for this prim so a re-PUT
            # releases the old one instead of leaking it to the GC.
            previous = self._devices.get(lidar_id)
            if previous is not None and previous is not device:
                self._safe_destroy(previous)
            self._id_by_prim[prim_path] = lidar_id  # no-op on re-PUT; commits a fresh id
            self._devices[lidar_id] = device
            return {"lidar_id": lidar_id, "prim_path": device.prim_path, **device.info()}

    def get_lidar(self, lidar_id):
        """Info for one registered lidar (id, prim, config), or 404."""
        device = self.get_device(lidar_id)
        return {"lidar_id": lidar_id, "prim_path": device.prim_path, **device.info()}

    def delete_lidar(self, lidar_id):
        """Unregister the lidar (the USD prim is left in the stage). 404 if unknown."""
        device = self._devices.get(lidar_id)
        if device is None:
            raise HTTPException(status_code=404, detail=f"no lidar registered with id '{lidar_id}'")
        self._safe_destroy(device)
        del self._devices[lidar_id]
        for prim, registered_id in list(self._id_by_prim.items()):
            if registered_id == lidar_id:
                del self._id_by_prim[prim]
                self._create_locks.pop(prim, None)
        return {"deleted": lidar_id}

    def list_lidars(self):
        """Return a ``{lidar_id: prim_path}`` map of every registered lidar."""
        return {lidar_id: device.prim_path for lidar_id, device in self._devices.items()}

    # -- capture ---------------------------------------------------------------

    async def capture(self, lidar_id, data_types):
        """Pump one frame and return the requested outputs. ``data_types`` may be
        None (return every bound output). See :meth:`..core.lidar.Lidar.capture`."""
        try:
            return await self.get_device(lidar_id).capture(data_types)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_depth_data(self, lidar_id):
        """Latest quantized depth buffer (num_rows, num_cols), or null."""
        return {"depth": self.get_device(lidar_id).get_depth_data()}

    def get_linear_depth_data(self, lidar_id):
        """Latest linear depth buffer in meters (num_rows, num_cols), or null."""
        return {"linear_depth": self.get_device(lidar_id).get_linear_depth_data()}

    def get_intensity_data(self, lidar_id):
        """Latest return-intensity buffer (num_rows, num_cols), or null."""
        return {"intensity": self.get_device(lidar_id).get_intensity_data()}

    def get_zenith_data(self, lidar_id):
        """Per-row vertical scan angles (radians), or null."""
        return {"zenith": self.get_device(lidar_id).get_zenith_data()}

    def get_azimuth_data(self, lidar_id):
        """Per-column horizontal scan angles (radians), or null."""
        return {"azimuth": self.get_device(lidar_id).get_azimuth_data()}

    def get_point_cloud_data(self, lidar_id):
        """Latest hit points in world frame (N, 3), or null."""
        return {"point_cloud": self.get_device(lidar_id).get_point_cloud_data()}

    def get_semantic_data(self, lidar_id):
        """Per-hit semantic ids, or null. Requires enable_semantics."""
        return {"semantic": self.get_device(lidar_id).get_semantic_data()}

    # -- pose --------------------------------------------------------------------

    def get_world_pose(self, lidar_id):
        """World-frame pose {position, orientation}."""
        return self.get_device(lidar_id).get_world_pose()

    def set_world_pose(self, lidar_id, position, orientation):
        """Set the world-frame pose; returns the resulting pose."""
        device = self.get_device(lidar_id)
        try:
            device.set_world_pose(position, orientation)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return device.get_world_pose()

    def get_local_pose(self, lidar_id):
        """Local-frame (parent-relative) pose {translation, orientation}."""
        return self.get_device(lidar_id).get_local_pose()

    def set_local_pose(self, lidar_id, translation, orientation):
        """Set the local-frame pose; returns the resulting pose."""
        device = self.get_device(lidar_id)
        try:
            device.set_local_pose(translation, orientation)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return device.get_local_pose()

    # -- rate / geometry config --------------------------------------------------

    def get_min_range(self, lidar_id):
        """Minimum sensing range (stage units)."""
        return {"min_range": self.get_device(lidar_id).get_min_range()}

    def set_min_range(self, lidar_id, value):
        """Set the minimum sensing range (stage units)."""
        device = self.get_device(lidar_id)
        device.set_min_range(value)
        return {"min_range": device.get_min_range()}

    def get_max_range(self, lidar_id):
        """Maximum sensing range (stage units)."""
        return {"max_range": self.get_device(lidar_id).get_max_range()}

    def set_max_range(self, lidar_id, value):
        """Set the maximum sensing range (stage units)."""
        device = self.get_device(lidar_id)
        device.set_max_range(value)
        return {"max_range": device.get_max_range()}

    def get_horizontal_fov(self, lidar_id):
        """Horizontal field of view (degrees)."""
        return {"horizontal_fov": self.get_device(lidar_id).get_horizontal_fov()}

    def set_horizontal_fov(self, lidar_id, value):
        """Set the horizontal field of view (degrees)."""
        device = self.get_device(lidar_id)
        device.set_horizontal_fov(value)
        return {"horizontal_fov": device.get_horizontal_fov()}

    def get_vertical_fov(self, lidar_id):
        """Vertical field of view (degrees)."""
        return {"vertical_fov": self.get_device(lidar_id).get_vertical_fov()}

    def set_vertical_fov(self, lidar_id, value):
        """Set the vertical field of view (degrees)."""
        device = self.get_device(lidar_id)
        device.set_vertical_fov(value)
        return {"vertical_fov": device.get_vertical_fov()}

    def get_horizontal_resolution(self, lidar_id):
        """Horizontal angular resolution (degrees per column)."""
        return {"horizontal_resolution": self.get_device(lidar_id).get_horizontal_resolution()}

    def set_horizontal_resolution(self, lidar_id, value):
        """Set the horizontal angular resolution (degrees per column)."""
        device = self.get_device(lidar_id)
        device.set_horizontal_resolution(value)
        return {"horizontal_resolution": device.get_horizontal_resolution()}

    def get_vertical_resolution(self, lidar_id):
        """Vertical angular resolution (degrees per row)."""
        return {"vertical_resolution": self.get_device(lidar_id).get_vertical_resolution()}

    def set_vertical_resolution(self, lidar_id, value):
        """Set the vertical angular resolution (degrees per row)."""
        device = self.get_device(lidar_id)
        device.set_vertical_resolution(value)
        return {"vertical_resolution": device.get_vertical_resolution()}

    def get_rotation_rate(self, lidar_id):
        """Rotation rate (Hz); 0 means an instantaneous full-sweep lidar."""
        return {"rotation_rate": self.get_device(lidar_id).get_rotation_rate()}

    def set_rotation_rate(self, lidar_id, value):
        """Set the rotation rate (Hz)."""
        device = self.get_device(lidar_id)
        device.set_rotation_rate(value)
        return {"rotation_rate": device.get_rotation_rate()}

    def get_yaw_offset(self, lidar_id):
        """Yaw offset applied to the scan pattern (degrees)."""
        return {"yaw_offset": self.get_device(lidar_id).get_yaw_offset()}

    def set_yaw_offset(self, lidar_id, value):
        """Set the yaw offset (degrees)."""
        device = self.get_device(lidar_id)
        device.set_yaw_offset(value)
        return {"yaw_offset": device.get_yaw_offset()}

    def get_high_lod(self, lidar_id):
        """Whether the sensor renders at high level-of-detail."""
        return {"high_lod": self.get_device(lidar_id).get_high_lod()}

    def set_high_lod(self, lidar_id, value):
        """Set whether the sensor renders at high level-of-detail."""
        device = self.get_device(lidar_id)
        device.set_high_lod(value)
        return {"high_lod": device.get_high_lod()}

    def get_draw_points(self, lidar_id):
        """Whether hit points are drawn in the viewport."""
        return {"draw_points": self.get_device(lidar_id).get_draw_points()}

    def set_draw_points(self, lidar_id, value):
        """Set whether hit points are drawn in the viewport."""
        device = self.get_device(lidar_id)
        device.set_draw_points(value)
        return {"draw_points": device.get_draw_points()}

    def get_draw_lines(self, lidar_id):
        """Whether scan rays are drawn in the viewport."""
        return {"draw_lines": self.get_device(lidar_id).get_draw_lines()}

    def set_draw_lines(self, lidar_id, value):
        """Set whether scan rays are drawn in the viewport."""
        device = self.get_device(lidar_id)
        device.set_draw_lines(value)
        return {"draw_lines": device.get_draw_lines()}

    def get_enable_semantics(self, lidar_id):
        """Whether per-hit semantic labels are captured."""
        return {"enable_semantics": self.get_device(lidar_id).get_enable_semantics()}

    def set_enable_semantics(self, lidar_id, value):
        """Set whether per-hit semantic labels are captured."""
        device = self.get_device(lidar_id)
        device.set_enable_semantics(value)
        return {"enable_semantics": device.get_enable_semantics()}

    # -- introspection -----------------------------------------------------------

    def get_num_rows(self, lidar_id):
        """Number of scan rows (vertical channels) the sensor currently reports."""
        return {"num_rows": self.get_device(lidar_id).get_num_rows()}

    def get_num_cols(self, lidar_id):
        """Number of scan columns (horizontal samples) a full sweep produces."""
        return {"num_cols": self.get_device(lidar_id).get_num_cols()}

    def get_num_cols_ticked(self, lidar_id):
        """Number of scan columns completed so far this physics step."""
        return {"num_cols_ticked": self.get_device(lidar_id).get_num_cols_ticked()}

    def get_azimuth_range(self, lidar_id):
        """[min, max] horizontal scan angles (radians)."""
        return {"azimuth_range": self.get_device(lidar_id).get_azimuth_range()}

    def get_zenith_range(self, lidar_id):
        """[min, max] vertical scan angles (radians)."""
        return {"zenith_range": self.get_device(lidar_id).get_zenith_range()}

    def is_lidar_sensor(self, lidar_id):
        """Whether the registered prim currently resolves to a live PhysX lidar sensor."""
        return {"is_lidar_sensor": self.get_device(lidar_id).is_lidar_sensor()}

    # -- collection control --------------------------------------------------

    def pause(self, lidar_id):
        """Pause sensor computation."""
        self.get_device(lidar_id).pause()
        return {"paused": True}

    def resume(self, lidar_id):
        """Resume sensor computation."""
        self.get_device(lidar_id).resume()
        return {"paused": False}

    def is_paused(self, lidar_id):
        """Whether sensor computation is currently paused."""
        return {"paused": self.get_device(lidar_id).is_paused()}

    # -- internals -------------------------------------------------------------

    def get_device(self, lidar_id):
        """Resolve a ``lidar_id`` to its device object, or 404."""
        device = self._devices.get(lidar_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=(f"no lidar registered with id '{lidar_id}', call PUT /lidars to create one"),
            )
        return device
