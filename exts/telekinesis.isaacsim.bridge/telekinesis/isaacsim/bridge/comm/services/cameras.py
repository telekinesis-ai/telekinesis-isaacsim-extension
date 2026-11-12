# SPDX-License-Identifier: Apache-2.0
"""The camera registry: the orchestration layer behind the camera HTTP routes.

``CameraService`` is the sensor counterpart to
:class:`..services.articulations.ArticulationService`: the
:class:`.device_registry.DeviceRegistry` mapping ``camera_id`` ->
:class:`..core.camera.Camera`, plus the operations on it -- create (wrap + bind),
look up, delete, list, capture render outputs, and get/set the camera's pose and
optical parameters.

It owns mutable state (the device table), so exactly one
instance is shared across all requests -- :class:`BridgeServer` builds it once and
stashes it on ``app.state`` for the routers to reach via ``Depends`` (see
:mod:`..comm.dependencies`).

Transport coupling is deliberately minimal: bad input raises
``fastapi.HTTPException`` (400) and a failed bind raises it (422) so the routers
stay one-liners. The ``..core.camera`` import pulls in isaacsim, so this module
imports only inside Isaac Sim -- same as the articulation service.

Wire units mirror the rest of the bridge: stage units (meters) for poses/apertures,
pixels for resolution/intrinsics. Render outputs are carried as numpy arrays; the
routers encode them into the binary frame :mod:`..comm.binary` describes, while
every other value here is JSON.
"""

import asyncio

from fastapi import HTTPException
from pxr import Gf, Sdf

from ...core.camera import Camera, capture_cameras, discover_camera_prims
from .device_registry import DeviceRegistry


class CameraService(DeviceRegistry):
    """The camera registry shared by every request.

    Holds the ``camera_id`` -> device table and exposes create / get / delete / list,
    capture, and the pose/optics getters and setters. Each camera's prim stores its id,
    resolution and requested frequency, so reopening a saved stage restores the camera
    under the same id with the same settings; it comes back running, even if it was
    paused. One instance per running bridge.
    """

    device_kind = "camera"

    def release_device(self, device):
        """Best-effort ``device.destroy()``, freeing the camera's render product.

        Unlike an articulation, a camera owns a render product, annotators, and event
        subscriptions, so it is explicitly destroyed rather than left to the GC
        finalizer. Errors are swallowed, since teardown may run after the stage is gone.
        """
        try:
            device.destroy()
        except Exception:  # cleanup must not fail the caller (stage may be gone)
            pass

    async def restore_device(self, prim_path):
        """Create the camera at ``prim_path`` with the resolution and frequency it stores."""
        await self.create_camera(
            prim_path,
            self.read_stored_value(prim_path, "resolution"),
            self.read_stored_value(prim_path, "frequency"),
        )

    async def create_camera(self, prim_path, resolution, frequency):
        """Register (and bind) the camera at ``prim_path`` and return its info.

        One camera per *requested* prim; PUTting the same prim again keeps its id **and
        the camera already bound to that prim**, applying the requested resolution /
        frequency to it, so the response always reflects the request. Ids are 1-based:
        ``camera1``, ``camera2``, ... The id, the resolution and a requested frequency
        are stored on the prim, so they survive saving and reopening the stage.

        The device is reused rather than rebuilt because Isaac Sim hands a second camera
        over the same prim the *same* render product (replicator returns an existing one
        for a matching prim and resolution), so freeing the first camera afterwards would
        take the second one's annotators off the render product with it.

        A newly registered camera produces ``rgb``/``rgba``; the annotators for
        other render outputs are attached the first time :meth:`capture` asks for
        them. The response reports both sets: ``active_data_types`` (produced now)
        and ``supported_data_types`` (everything ``capture`` accepts).
        """
        prim_path = prim_path.rstrip("/") or "/"

        # Serialize concurrent creates of the SAME prim_path (mirrors the
        # articulation service): two clients racing to register the same camera
        # would otherwise both allocate an id/device, the second clobbering the
        # first. setdefault is synchronous, so concurrent callers land on one Lock.
        lock = self._create_locks.setdefault(prim_path, asyncio.Lock())
        async with lock:
            camera_id = self._id_by_prim.get(prim_path)
            device = None if camera_id is None else self._devices.get(camera_id)

            if device is not None and device.is_bound:
                # Reuse: the registered camera still owns its render product, so the
                # requested configuration is applied to it in place.
                self._configure(device, resolution, frequency)
            else:
                camera_id = self.allocate_device_id(prim_path)
                try:
                    replacement = Camera(
                        prim_path,
                        name=camera_id,
                        resolution=tuple(resolution),
                        frequency=frequency,
                    )
                except Exception as exc:
                    # Bad input value: a non-divisor frequency, or a prim that isn't a
                    # Camera -- all raised at construction. 400, not the 500 a bare
                    # Exception would otherwise become.
                    raise HTTPException(status_code=400, detail=str(exc)) from exc
                if device is not None:
                    # An unbound device holds no render product, so freeing it here
                    # cannot strand the replacement's annotators.
                    self.release_device(device)
                device = replacement
                self.register_device(prim_path, camera_id, device)
            self.write_stored_value(
                prim_path, "resolution", Sdf.ValueTypeNames.Int2, Gf.Vec2i(*device.resolution)
            )
            if frequency is not None:
                self.write_stored_value(prim_path, "frequency", Sdf.ValueTypeNames.Float, frequency)

            try:
                await device.bind()
            except RuntimeError as exc:
                # 422: well-formed request, but the camera couldn't be brought up
                # (needs --enable_cameras / RTX warmup). The device stays registered, so
                # a retry rebinds it rather than spending an id per attempt.
                raise HTTPException(status_code=422, detail=str(exc)) from exc
            return {
                "camera_id": camera_id,
                "prim_path": device.prim_path,
                **device.info(),
            }

    @staticmethod
    def _configure(device, resolution, frequency):
        """Apply a create request's resolution / frequency to an already-bound camera.

        Only what the request changes is written, so a re-PUT of the same configuration
        does not touch the render product. A value the camera rejects is a bad input
        value, hence 400 rather than the 500 a bare ValueError would become.
        """
        try:
            if tuple(resolution) != device.resolution:
                device.set_resolution(resolution)
            if frequency is not None and float(frequency) != device.get_frequency():
                device.set_frequency(frequency)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def get_camera(self, camera_id):
        """Info for one registered camera (id, prim, resolution, optics), or 404."""
        device = self.get_device(camera_id)
        return {"camera_id": camera_id, "prim_path": device.prim_path, **device.info()}

    @staticmethod
    def discover_cameras(prim_path):
        """Return ``{"prim_paths": [...]}``, every camera prim at or below ``prim_path``.

        404 if the prim is not in the open stage, 400 if no camera is at or below it.
        See :func:`..core.camera.discover_camera_prims`."""
        try:
            prim_paths = discover_camera_prims(prim_path)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        if not prim_paths:
            raise HTTPException(status_code=400, detail=f"no camera prim at or below {prim_path!r}")
        return {"prim_paths": prim_paths}

    # -- capture ---------------------------------------------------------------

    async def capture(self, camera_id, data_types, world_frame=False):
        """Pump one frame and return the requested outputs. ``data_types`` may be
        None (return every output currently active). An output that is not active yet
        is activated here, which costs a few frames of warmup on that first capture.
        ``world_frame`` applies to the ``pointcloud`` output alone.
        See :meth:`..core.camera.Camera.capture`."""
        try:
            return await self.get_device(camera_id).capture(data_types, world_frame=world_frame)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except RuntimeError as exc:
            # 422, mirroring a failed bind: the request named real data types, but a
            # newly attached annotator never produced data.
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    async def capture_cameras(self, captures, world_frame=False):
        """Pump one frame and return the requested outputs of several cameras, keyed
        by ``camera_id``, all from the same rendering frame. ``captures`` maps each
        ``camera_id`` to the data types to read from it. Errors map as in
        :meth:`capture`. See :func:`..core.camera.capture_cameras`."""
        if not captures:
            raise HTTPException(status_code=400, detail="captures must name at least one camera")
        cameras_with_data_types = {
            camera_id: (self.get_device(camera_id), data_types)
            for camera_id, data_types in captures.items()
        }
        try:
            return await capture_cameras(cameras_with_data_types, world_frame=world_frame)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    def get_rgb(self, camera_id):
        """Latest RGB image as a (H, W, 3) uint8 array, or None if not ready."""
        return {"rgb": self.get_device(camera_id).get_rgb()}

    def get_rgba(self, camera_id):
        """Latest RGBA image as a (H, W, 4) uint8 array, or None."""
        return {"rgba": self.get_device(camera_id).get_rgba()}

    def get_depth(self, camera_id):
        """Latest depth image as a (H, W) float32 array, or None. Pixels that hit
        nothing are inf."""
        return {"depth": self.get_device(camera_id).get_depth()}

    def get_pointcloud(self, camera_id, world_frame=False):
        """Latest pointcloud as an (N, 3) array in world or, by default, camera
        frame."""
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
        """Set [width, height] in pixels, also stored on the prim; returns the resulting
        resolution."""
        device = self.get_device(camera_id)
        try:
            device.set_resolution((width, height))
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        self.write_stored_value(
            device.prim_path, "resolution", Sdf.ValueTypeNames.Int2, Gf.Vec2i(*device.resolution)
        )
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
        """Set the acquisition frequency (Hz), also stored on the prim."""
        device = self.get_device(camera_id)
        try:
            device.set_frequency(value)
        except Exception as exc:  # isaacsim raises a bare Exception for a bad divisor
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        self.write_stored_value(device.prim_path, "frequency", Sdf.ValueTypeNames.Float, value)
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
