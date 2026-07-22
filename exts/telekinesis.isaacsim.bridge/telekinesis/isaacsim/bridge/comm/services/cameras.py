# SPDX-License-Identifier: Apache-2.0
"""The camera registry: the orchestration layer behind the camera HTTP routes.

``CameraService`` is the sensor counterpart to
:class:`..services.articulations.ArticulationService`: an in-memory table mapping
``camera_id`` -> :class:`..core.camera.Camera`, plus the operations on it -- create
(wrap + bind), look up, delete, list, capture render outputs, and get/set the
camera's pose and optical parameters.

It owns mutable state (the device table and the id counter), so exactly one
instance is shared across all requests -- :class:`BridgeServer` builds it once and
stashes it on ``app.state`` for the routers to reach via ``Depends`` (see
:mod:`..comm.dependencies`).

Transport coupling is deliberately minimal: bad input raises
``fastapi.HTTPException`` (400) and a failed bind raises it (422) so the routers
stay one-liners. The ``..core.camera`` import pulls in isaacsim, so this module
imports only inside Isaac Sim -- same as the articulation service.

Wire units mirror the rest of the bridge: stage units (meters) for poses/apertures,
pixels for resolution/intrinsics, plain arrays for images.
"""

import asyncio

from fastapi import HTTPException

from ...core.camera import Camera


class CameraService:
    """The camera registry shared by every request.

    Holds the ``camera_id`` -> device table and an id counter, and exposes
    create / get / delete / list, capture, and the pose/optics getters and setters.
    One instance per running bridge.
    """

    def __init__(self):
        self._devices = {}  # camera_id -> Camera
        self._id_by_prim = {}  # requested prim_path -> camera_id (stable on re-create)
        self._count = 0  # for ids like camera1, camera2
        # prim_path -> asyncio.Lock, serializes concurrent creates of the same prim
        self._create_locks = {}

    def clear(self):
        """Drop every bound camera (called when the bridge stops or the stage changes).

        Unlike an articulation, a camera owns a render product, annotators, and event
        subscriptions, so each device is explicitly destroyed rather than left to the
        GC finalizer.
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

    async def create_camera(self, prim_path, resolution, data_types, frequency):
        """Register (and bind) the camera at ``prim_path`` and return its info.

        One camera per *requested* prim; PUTting the same prim again keeps its id
        but **rebuilds the device with the new config** (resolution / data_types /
        frequency), so the response always reflects the request. Ids are 1-based:
        ``camera1``, ``camera2``, ...
        """
        prim_path = prim_path.rstrip("/") or "/"

        # Serialize concurrent creates of the SAME prim_path (mirrors the
        # articulation service): two clients racing to register the same camera
        # would otherwise both allocate an id/device, the second clobbering the
        # first. setdefault is synchronous, so concurrent callers land on one Lock.
        lock = self._create_locks.setdefault(prim_path, asyncio.Lock())
        async with lock:
            existing_id = self._id_by_prim.get(prim_path)
            if existing_id is None:
                # Reserve a fresh id synchronously -- before the bind() await below,
                # which yields the loop. The per-prim lock does NOT serialize creates
                # of *different* prims, so deferring the increment past the await
                # would let two of them grab the same cameraN.
                self._count += 1
                camera_id = f"camera{self._count}"
            else:
                camera_id = existing_id

            # Build + bind the NEW device before touching any existing one, so a bad
            # re-PUT (wrong resolution/data_types/frequency, or a prim that won't
            # bind) leaves the currently-registered camera untouched and working.
            try:
                device = Camera(
                    prim_path,
                    name=camera_id,
                    resolution=tuple(resolution),
                    data_types=data_types,
                    frequency=frequency,
                )
            except Exception as exc:
                # Bad input value: unknown data_type, non-divisor frequency, or a
                # prim that isn't a Camera -- all raised at construction. 400, not
                # the 500 a bare Exception would otherwise become. Construction has
                # no await, so rolling back a freshly-reserved id here is race-free.
                if existing_id is None:
                    self._count -= 1
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: well-formed request, but the camera couldn't be brought up
                # (needs --enable_cameras / RTX warmup). Free its partial resources.
                self._safe_destroy(device)
                raise HTTPException(status_code=422, detail=str(exc)) from exc

            # Success: commit. Free any prior device for this prim so a re-PUT frees
            # the old render product/annotators instead of leaking them to the GC.
            previous = self._devices.get(camera_id)
            if previous is not None and previous is not device:
                self._safe_destroy(previous)
            self._id_by_prim[prim_path] = camera_id  # no-op on re-PUT; commits a fresh id
            self._devices[camera_id] = device
            return {"camera_id": camera_id, "prim_path": device.prim_path, **device.info()}

    def get_camera(self, camera_id):
        """Info for one registered camera (id, prim, resolution, optics), or 404."""
        device = self.get_device(camera_id)
        return {"camera_id": camera_id, "prim_path": device.prim_path, **device.info()}

    def delete_camera(self, camera_id):
        """Unregister the camera (the USD prim is left in the stage). 404 if unknown."""
        device = self._devices.get(camera_id)
        if device is None:
            raise HTTPException(
                status_code=404, detail=f"no camera registered with id '{camera_id}'"
            )
        # Free the render product / annotators / event subs this device owns.
        self._safe_destroy(device)
        del self._devices[camera_id]
        for prim, registered_id in list(self._id_by_prim.items()):
            if registered_id == camera_id:
                del self._id_by_prim[prim]
                self._create_locks.pop(prim, None)
        return {"deleted": camera_id}

    def list_cameras(self):
        """Return a ``{camera_id: prim_path}`` map of every registered camera."""
        return {camera_id: device.prim_path for camera_id, device in self._devices.items()}

    # -- capture ---------------------------------------------------------------

    async def capture(self, camera_id, data_types):
        """Pump one frame and return the requested outputs. ``data_types`` may be
        None (return every bound output). See :meth:`..core.camera.Camera.capture`."""
        try:
            return await self.get_device(camera_id).capture(data_types)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_rgb(self, camera_id):
        """Latest RGB image (H, W, 3), or null if not ready."""
        return {"rgb": self.get_device(camera_id).get_rgb()}

    def get_rgba(self, camera_id):
        """Latest RGBA image (H, W, 4), or null."""
        return {"rgba": self.get_device(camera_id).get_rgba()}

    def get_depth(self, camera_id):
        """Latest depth image (H, W), or null."""
        return {"depth": self.get_device(camera_id).get_depth()}

    def get_pointcloud(self, camera_id, world_frame=True):
        """Latest pointcloud (N, 3) in world or camera frame."""
        return {"pointcloud": self.get_device(camera_id).get_pointcloud(world_frame=world_frame)}

    # -- pose ------------------------------------------------------------------

    def get_world_pose(self, camera_id, camera_axes):
        """World-frame pose {position, orientation}."""
        try:
            return self.get_device(camera_id).get_world_pose(camera_axes)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def set_world_pose(self, camera_id, position, orientation, camera_axes):
        """Set the world-frame pose; returns the resulting pose."""
        device = self.get_device(camera_id)
        try:
            device.set_world_pose(position, orientation, camera_axes)
            return device.get_world_pose(camera_axes)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_local_pose(self, camera_id, camera_axes):
        """Local-frame (parent-relative) pose {translation, orientation}."""
        try:
            return self.get_device(camera_id).get_local_pose(camera_axes)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def set_local_pose(self, camera_id, translation, orientation, camera_axes):
        """Set the local-frame pose; returns the resulting pose."""
        device = self.get_device(camera_id)
        try:
            device.set_local_pose(translation, orientation, camera_axes)
            return device.get_local_pose(camera_axes)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    # -- optics / configuration ------------------------------------------------

    def get_resolution(self, camera_id):
        """Current [width, height] in pixels."""
        return {"resolution": self.get_device(camera_id).get_resolution()}

    def set_resolution(self, camera_id, width, height):
        """Set [width, height] in pixels; returns the resulting resolution."""
        device = self.get_device(camera_id)
        try:
            device.set_resolution((width, height))
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"resolution": device.get_resolution()}

    def get_focal_length(self, camera_id):
        """Focal length (stage units)."""
        return {"focal_length": self.get_device(camera_id).get_focal_length()}

    def set_focal_length(self, camera_id, value):
        """Set the focal length (stage units)."""
        device = self.get_device(camera_id)
        device.set_focal_length(value)
        return {"focal_length": device.get_focal_length()}

    def get_focus_distance(self, camera_id):
        """Distance from camera to focus plane (stage units)."""
        return {"focus_distance": self.get_device(camera_id).get_focus_distance()}

    def set_focus_distance(self, camera_id, value):
        """Set the focus distance (stage units)."""
        device = self.get_device(camera_id)
        device.set_focus_distance(value)
        return {"focus_distance": device.get_focus_distance()}

    def get_lens_aperture(self, camera_id):
        """fStop value (0 disables depth-of-field)."""
        return {"lens_aperture": self.get_device(camera_id).get_lens_aperture()}

    def set_lens_aperture(self, camera_id, value):
        """Set the fStop value."""
        device = self.get_device(camera_id)
        device.set_lens_aperture(value)
        return {"lens_aperture": device.get_lens_aperture()}

    def get_horizontal_aperture(self, camera_id):
        """Horizontal aperture / sensor width (stage units)."""
        return {"horizontal_aperture": self.get_device(camera_id).get_horizontal_aperture()}

    def set_horizontal_aperture(self, camera_id, value, maintain_square_pixels):
        """Set the horizontal aperture (stage units)."""
        device = self.get_device(camera_id)
        device.set_horizontal_aperture(value, maintain_square_pixels)
        return {"horizontal_aperture": device.get_horizontal_aperture()}

    def get_vertical_aperture(self, camera_id):
        """Vertical aperture / sensor height (stage units)."""
        return {"vertical_aperture": self.get_device(camera_id).get_vertical_aperture()}

    def set_vertical_aperture(self, camera_id, value, maintain_square_pixels):
        """Set the vertical aperture (stage units)."""
        device = self.get_device(camera_id)
        device.set_vertical_aperture(value, maintain_square_pixels)
        return {"vertical_aperture": device.get_vertical_aperture()}

    def get_clipping_range(self, camera_id):
        """[near, far] clipping distances (stage units)."""
        return {"clipping_range": self.get_device(camera_id).get_clipping_range()}

    def set_clipping_range(self, camera_id, near_distance, far_distance):
        """Set near/far clipping distances; either may be None to leave unchanged."""
        device = self.get_device(camera_id)
        device.set_clipping_range(near_distance, far_distance)
        return {"clipping_range": device.get_clipping_range()}

    def get_frequency(self, camera_id):
        """Current acquisition frequency (Hz)."""
        return {"frequency": self.get_device(camera_id).get_frequency()}

    def set_frequency(self, camera_id, value):
        """Set the acquisition frequency (Hz)."""
        device = self.get_device(camera_id)
        try:
            device.set_frequency(value)
        except Exception as exc:  # isaacsim raises a bare Exception for a bad divisor
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"frequency": device.get_frequency()}

    def get_projection_mode(self, camera_id):
        """`perspective` or `orthographic`."""
        return {"projection_mode": self.get_device(camera_id).get_projection_mode()}

    def set_projection_mode(self, camera_id, value):
        """Set the projection mode (`perspective`/`orthographic`)."""
        device = self.get_device(camera_id)
        device.set_projection_mode(value)
        return {"projection_mode": device.get_projection_mode()}

    def get_stereo_role(self, camera_id):
        """`mono`, `left` or `right`."""
        return {"stereo_role": self.get_device(camera_id).get_stereo_role()}

    def set_stereo_role(self, camera_id, value):
        """Set the stereo role (`mono`/`left`/`right`)."""
        device = self.get_device(camera_id)
        device.set_stereo_role(value)
        return {"stereo_role": device.get_stereo_role()}

    def get_lens_distortion_model(self, camera_id):
        """Lens distortion model name (`pinhole` if unset)."""
        return {"lens_distortion_model": self.get_device(camera_id).get_lens_distortion_model()}

    def set_lens_distortion_model(self, camera_id, value):
        """Set the lens distortion model (applies the matching schema)."""
        device = self.get_device(camera_id)
        device.set_lens_distortion_model(value)
        return {"lens_distortion_model": device.get_lens_distortion_model()}

    # -- introspection ---------------------------------------------------------

    def get_intrinsics_matrix(self, camera_id):
        """3x3 intrinsics matrix (pinhole models only)."""
        try:
            return {"intrinsics_matrix": self.get_device(camera_id).get_intrinsics_matrix()}
        except Exception as exc:  # non-pinhole model raises a bare Exception
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_fov(self, camera_id):
        """Horizontal and vertical field of view."""
        device = self.get_device(camera_id)
        return {
            "horizontal_fov": device.get_horizontal_fov(),
            "vertical_fov": device.get_vertical_fov(),
        }

    def get_render_product_path(self, camera_id):
        """Path to the render product attached to this camera."""
        return {"render_product_path": self.get_device(camera_id).get_render_product_path()}

    def get_supported_annotators(self, camera_id):
        """Annotator names that can be attached to this camera."""
        return {"supported_annotators": self.get_device(camera_id).supported_annotators()}

    # -- collection control ----------------------------------------------------

    def pause(self, camera_id):
        """Pause data collection / frame updates."""
        self.get_device(camera_id).pause()
        return {"paused": True}

    def resume(self, camera_id):
        """Resume data collection / frame updates."""
        self.get_device(camera_id).resume()
        return {"paused": False}

    def is_paused(self, camera_id):
        """Whether data collection is currently paused."""
        return {"paused": self.get_device(camera_id).is_paused()}

    # -- internals -------------------------------------------------------------

    def get_device(self, camera_id):
        """Resolve a ``camera_id`` to its device object, or 404."""
        device = self._devices.get(camera_id)
        if device is None:
            raise HTTPException(
                status_code=404,
                detail=(
                    f"no camera registered with id '{camera_id}', call PUT /cameras to create one"
                ),
            )
        return device
