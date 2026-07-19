# SPDX-License-Identifier: Apache-2.0
"""
Generic camera sensor device: binds one isaacsim.sensors.camera.Camera handle at
a prim path and exposes its full surface over the bridge.

This is the sensor counterpart to :mod:`..core.articulation`. Where that wraps a
single articulation and drives its joints, this wraps a single Isaac Sim camera
and reads/authors its render outputs and optical parameters. The design mirrors
``SingleArticulation``:

* Named to match the ``isaacsim.sensors.camera.Camera`` handle it wraps (imported
  as the ``camera_sensor`` module, not the bare class, so it doesn't shadow this
  one). This class adds the bridge's bind/retry/warmup and HTTP-facing conversion
  on top of that single Isaac Sim handle.
* Every public method of the underlying ``Camera`` is exposed one-to-one with the
  same name (the project convention: "match isaacsim naming"). The correspondence
  is asserted by ``tests/test_camera_api_mapping.py``. The only additions are the
  bridge-specific :meth:`bind`, :meth:`capture`, and :meth:`info` (see
  ``BRIDGE_ONLY_METHODS`` below).

Native Isaac units throughout: stage units (meters) for poses/apertures, pixels
for resolution/intrinsics, row-major image arrays. Every method returns
JSON-serializable Python (lists/floats/ints/dicts), never raw numpy/warp/torch --
the future service layer maps ``ValueError`` (bad client input) and ``RuntimeError``
(bind failure) to HTTP errors, exactly as the articulation service does.

The capture path follows the same ``async def`` / ``next_update_async`` blocking
style as the articulation (no background worker, no command queue). Cameras
require RTX sensor rendering: Isaac Sim must be launched with ``--enable_cameras``
and the render product needs a few frames of warmup before it yields data, which
:meth:`bind` waits out with the same retry-loop shape as the articulation.
"""

import math
import time

import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.sensors import camera as camera_sensor
import carb

# -- static tables ----------------------------------------------------------
#
# Plain module-level constants (no isaacsim needed to read them). The one-to-one
# mapping test (tests/test_camera_api_mapping.py) ast-parses ISAAC_CAMERA_CLASS
# and BRIDGE_ONLY_METHODS out of this file rather than importing it, so this
# module stays importable only inside the Isaac Sim runtime while the test still
# runs anywhere.

# The underlying handle this wrapper mirrors one-to-one, as (module, qualname).
ISAAC_CAMERA_CLASS = ("isaacsim.sensors.camera", "Camera")

# Non-rgb data types -> the Camera.add_*_to_frame method that attaches the
# annotator serving them. rgb/rgba are served by the rgb annotator that
# Camera.initialize() attaches by default, so they are absent here. "depth" is an
# alias for "distance_to_image_plane".
DATA_TYPE_TO_ADDER = {
    "depth": "add_distance_to_image_plane_to_frame",
    "distance_to_image_plane": "add_distance_to_image_plane_to_frame",
    "distance_to_camera": "add_distance_to_camera_to_frame",
    "normals": "add_normals_to_frame",
    "motion_vectors": "add_motion_vectors_to_frame",
    "occlusion": "add_occlusion_to_frame",
    "semantic_segmentation": "add_semantic_segmentation_to_frame",
    "instance_id_segmentation": "add_instance_id_segmentation_to_frame",
    "instance_segmentation": "add_instance_segmentation_to_frame",
    "bounding_box_2d_tight": "add_bounding_box_2d_tight_to_frame",
    "bounding_box_2d_loose": "add_bounding_box_2d_loose_to_frame",
    "bounding_box_3d": "add_bounding_box_3d_to_frame",
    "pointcloud": "add_pointcloud_to_frame",
}

# Everything the client may request from capture(): rgb/rgba plus every
# annotator-backed type above.
SUPPORTED_DATA_TYPES = frozenset({"rgb", "rgba"}) | frozenset(DATA_TYPE_TO_ADDER)

# Pose conventions Camera.get_world_pose / set_world_pose accept (isaacsim naming).
POSE_AXES = ("world", "ros", "usd")

# Public methods on the wrapper with no one-to-one counterpart on the underlying
# isaacsim Camera -- bridge conveniences. The mapping test allows exactly these
# as extras.
BRIDGE_ONLY_METHODS = frozenset({"bind", "capture", "info"})

# Cameras need several frames of RTX warmup before the render product produces a
# non-empty frame; reuse the articulation's retry budget so a freshly created
# camera has time to come up before bind gives up.
_BIND_RETRIES = 60

# Read image/annotator data onto the host so it converts to plain Python lists
# without a device round-trip in every getter.
_HOST_DEVICE = "cpu"


def _to_json(value):
    """Convert an Isaac return value into JSON-serializable Python.

    Handles the numpy/warp/torch arrays, tuples, and nested dicts the camera API
    returns; passes plain scalars through and normalizes numpy scalar types.
    Returns ``None`` unchanged (several getters return ``None`` before data is
    available).

    Non-finite floats (``inf``/``nan``) are mapped to ``None``: a depth frame has
    ``inf`` for background pixels that hit nothing, and Starlette serializes
    responses with ``allow_nan=False``, so leaving them in would make the whole
    response invalid JSON (a 500). ``None`` is JSON-legal and unambiguous versus a
    real distance.
    """
    if value is None:
        return None
    if hasattr(value, "numpy"):  # warp array / cpu torch tensor -> numpy
        try:
            value = value.numpy()
        except Exception:  # pragma: no cover - defensive; fall through to below
            pass
    if isinstance(value, np.ndarray):
        if value.dtype.kind == "f" and not np.isfinite(value).all():
            obj = value.astype(object)  # boxes each entry as a plain Python float
            obj[~np.isfinite(value)] = None
            return obj.tolist()
        return value.tolist()
    if isinstance(value, dict):
        return {k: _to_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_json(v) for v in value]
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        value = float(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class Camera:
    """Binds a single camera at ``prim_path`` and exposes its full API.

    Named to match the isaacsim.sensors.camera.Camera handle it wraps (imported as
    the ``camera_sensor`` module, not the bare class, so it doesn't shadow this
    one). Construction wraps/creates the USD Camera prim; :meth:`bind` initializes
    the render product, attaches the requested annotators, and waits for warmup.
    """

    def __init__(
        self,
        prim_path,
        name="camera",
        frequency=None,
        dt=None,
        resolution=(1280, 720),
        data_types=None,
        position=None,
        orientation=None,
    ):
        """Wrap/create the camera prim and validate ``data_types``; call :meth:`bind`
        before capturing.

        ``resolution`` is ``(width, height)`` in pixels (isaacsim convention).
        ``data_types`` is the set of outputs to produce (default ``["rgb"]``); each
        must be in :data:`SUPPORTED_DATA_TYPES` or this raises ``ValueError``,
        mirroring the articulation's unknown-joint-name check. ``rgb`` is always
        available (attached by ``initialize``).
        """
        self.prim_path = prim_path
        self._name = name
        self.resolution = tuple(resolution)
        self.data_types = self._validate_data_types(data_types or ["rgb"])
        self._camera = camera_sensor.Camera(
            prim_path=prim_path,
            name=name,
            frequency=frequency,
            dt=dt,
            resolution=self.resolution,
            position=None if position is None else np.asarray(position, dtype=float),
            orientation=None if orientation is None else np.asarray(orientation, dtype=float),
        )
        self._initialized = False
        self._start_time = time.monotonic()

    @staticmethod
    def _validate_data_types(data_types):
        """Return ``data_types`` as a list, raising ``ValueError`` on any unknown one."""
        requested = list(data_types)
        unknown = [d for d in requested if d not in SUPPORTED_DATA_TYPES]
        if unknown:
            raise ValueError(
                f"unknown camera data type(s) {unknown}; supported: {sorted(SUPPORTED_DATA_TYPES)}"
            )
        return requested

    # -- bridge lifecycle / capture (not part of the isaacsim surface) ---------

    async def bind(self):
        """Initialize the camera against the current render view and wait for warmup.

        Mirrors :meth:`..core.articulation.SingleArticulation.bind`: play the
        timeline, pump ``next_update_async`` a couple frames, call ``initialize()``
        (attaches the rgb annotator), register the annotators for the remaining
        ``data_types``, then loop until a frame reads back non-empty (RTX warmup).
        ``carb.log_info`` on success; raises ``RuntimeError`` carrying the last
        error if the camera never yields data.
        """
        omni.timeline.get_timeline_interface().play()
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()

        if not self._initialized:
            self._camera.initialize()
            self._register_annotators()
            self._initialized = True

        probe = self.data_types[0]
        last_exc = None
        for _ in range(_BIND_RETRIES):
            await app.next_update_async()
            try:
                if self._read(probe) is not None:
                    carb.log_info(
                        f"[bridge] bound camera {self.prim_path}: "
                        f"{self.resolution} {self.data_types}"
                    )
                    return
            except Exception as exc:  # data not ready yet -> keep pumping frames
                last_exc = exc

        detail = f"camera at {self.prim_path} did not produce data"
        if last_exc is not None:
            detail += f" (last error: {last_exc!r})"
        raise RuntimeError(detail)

    def _register_annotators(self):
        """Attach the annotator for each requested data type.

        ``rgb``/``rgba`` are served by the rgb annotator that ``initialize`` already
        attached, so they are skipped; every other type maps to an ``add_*_to_frame``
        via :data:`DATA_TYPE_TO_ADDER`.
        """
        for data_type in self.data_types:
            adder = DATA_TYPE_TO_ADDER.get(data_type)
            if adder is not None:
                getattr(self._camera, adder)()

    def _read(self, data_type):
        """Read one annotator's latest value as JSON, dispatching to the dedicated
        getter where the handle has one (rgb/rgba/depth/pointcloud) and falling back
        to the current frame otherwise. Returns ``None`` when data is not ready."""
        if data_type == "rgb":
            return _to_json(self._camera.get_rgb(device=_HOST_DEVICE))
        if data_type == "rgba":
            return _to_json(self._camera.get_rgba(device=_HOST_DEVICE))
        if data_type in ("depth", "distance_to_image_plane"):
            return _to_json(self._camera.get_depth(device=_HOST_DEVICE))
        if data_type == "pointcloud":
            return _to_json(self._camera.get_pointcloud(device=_HOST_DEVICE))
        return _to_json(self._camera.get_current_frame().get(data_type))

    async def capture(self, data_types=None):
        """Pump one frame and return a JSON snapshot of the requested annotators.

        The core read method (analogue of the articulation's ``move_j``): awaits one
        ``next_update_async`` so the render product is current, then returns
        ``{<data_type>: <array/list or None>, ...}`` plus ``rendering_frame`` and a
        monotonic ``timestamp`` -- mirroring ``get_joints_state``'s dict-with-timestamp
        shape. ``data_types`` defaults to every type this camera was bound with;
        requesting a type this camera wasn't bound with (so no annotator is attached
        to produce it) raises ``ValueError`` rather than silently returning ``null``.
        """
        requested = list(data_types) if data_types is not None else list(self.data_types)
        # rgb/rgba are always attached by initialize(); every other type must have
        # been bound at create time (its annotator attached in _register_annotators).
        available = set(self.data_types) | {"rgb", "rgba"}
        unavailable = [d for d in requested if d not in available]
        if unavailable:
            raise ValueError(
                f"camera at {self.prim_path} was not bound with data type(s) {unavailable}; "
                f"available: {sorted(available)}"
            )
        await omni.kit.app.get_app().next_update_async()
        out = {data_type: self._read(data_type) for data_type in requested}
        out["rendering_frame"] = _to_json(self._camera.get_current_frame().get("rendering_frame"))
        out["timestamp"] = time.monotonic() - self._start_time
        return out

    def info(self):
        """Static description of this camera: prim path, bound data types, resolution,
        frequency, focal length, apertures, clipping range, projection, and lens
        distortion model. The analogue of ``SingleArticulation.info()``."""
        return {
            "prim_path": self.prim_path,
            "data_types": list(self.data_types),
            "resolution": list(self.resolution),
            "frequency": self.get_frequency(),
            "focal_length": self.get_focal_length(),
            "horizontal_aperture": self.get_horizontal_aperture(),
            "vertical_aperture": self.get_vertical_aperture(),
            "clipping_range": self.get_clipping_range(),
            "projection_mode": self.get_projection_mode(),
            "lens_distortion_model": self.get_lens_distortion_model(),
        }

    # -- lifecycle passthroughs (isaacsim.sensors.camera.Camera surface) -------

    def initialize(self, physics_sim_view=None, attach_rgb_annotator=True):
        """Initialize the underlying handle after a world reset. See
        ``camera_sensor.Camera.initialize``. Prefer :meth:`bind`, which also attaches
        annotators and waits for warmup."""
        self._camera.initialize(physics_sim_view, attach_rgb_annotator)
        self._initialized = True

    def post_reset(self):
        """Reset the sensor's timing state after a simulation reset."""
        self._camera.post_reset()

    def destroy(self):
        """Detach annotators and destroy the render product for this camera."""
        self._camera.destroy()
        self._initialized = False

    def resume(self):
        """Resume data collection / frame updates."""
        self._camera.resume()

    def pause(self):
        """Pause data collection / frame updates."""
        self._camera.pause()

    def is_paused(self):
        """Whether data collection is currently paused."""
        return bool(self._camera.is_paused())

    def get_render_product_path(self):
        """Path to the render product attached to this camera."""
        return self._camera.get_render_product_path()

    def supported_annotators(self):
        """List of annotator names that can be attached to this camera."""
        return list(self._camera.supported_annotators)

    def get_current_frame(self, clone=False):
        """JSON snapshot of the current frame dict (all attached annotators plus
        rendering time/frame)."""
        return _to_json(self._camera.get_current_frame(clone=clone))

    # -- rate control ----------------------------------------------------------

    def set_frequency(self, value):
        """Set the acquisition frequency (Hz). Must be a whole number that divides the
        rendering frequency.

        Rejects a fractional value rather than silently truncating it (``int(2.9)``
        -> ``2``), which would apply a rate the client never asked for.
        """
        if value != int(value):
            raise ValueError(f"frequency must be a whole number of Hz, got {value}")
        self._camera.set_frequency(int(value))

    def get_frequency(self):
        """Current acquisition frequency (Hz)."""
        return float(self._camera.get_frequency())

    def set_dt(self, value):
        """Set the acquisition period (seconds). Must be a multiple of the render dt."""
        self._camera.set_dt(float(value))

    def get_dt(self):
        """Current acquisition period (seconds)."""
        return float(self._camera.get_dt())

    # -- resolution / geometry -------------------------------------------------

    def set_resolution(self, value, maintain_square_pixels=True):
        """Set ``(width, height)`` in pixels; updates apertures to keep square pixels."""
        if len(value) != 2:
            raise ValueError(f"resolution must be (width, height), got {value}")
        self.resolution = (int(value[0]), int(value[1]))
        self._camera.set_resolution(self.resolution, maintain_square_pixels)

    def get_resolution(self):
        """Current ``[width, height]`` in pixels."""
        return list(self._camera.get_resolution())

    def get_aspect_ratio(self):
        """Width / height."""
        return float(self._camera.get_aspect_ratio())

    def get_horizontal_fov(self):
        """Horizontal field of view."""
        return float(self._camera.get_horizontal_fov())

    def get_vertical_fov(self):
        """Vertical field of view."""
        return float(self._camera.get_vertical_fov())

    # -- pose ------------------------------------------------------------------

    def get_world_pose(self, camera_axes="world"):
        """World-frame pose as ``{"position": [x, y, z], "orientation": [w, x, y, z]}``.
        ``camera_axes`` is one of :data:`POSE_AXES` (``world``/``ros``/``usd``)."""
        self._check_axes(camera_axes)
        position, orientation = self._camera.get_world_pose(camera_axes=camera_axes)
        return {"position": _to_json(position), "orientation": _to_json(orientation)}

    def set_world_pose(self, position=None, orientation=None, camera_axes="world"):
        """Set the world-frame pose. ``position`` is ``[x, y, z]``, ``orientation`` a
        ``[w, x, y, z]`` quaternion; either may be ``None`` to leave it untouched."""
        self._check_axes(camera_axes)
        self._camera.set_world_pose(
            self._vector(position, 3, "position"),
            self._vector(orientation, 4, "orientation"),
            camera_axes=camera_axes,
        )

    def get_local_pose(self, camera_axes="world"):
        """Local-frame (parent-relative) pose, same shape as :meth:`get_world_pose`."""
        self._check_axes(camera_axes)
        translation, orientation = self._camera.get_local_pose(camera_axes=camera_axes)
        return {"translation": _to_json(translation), "orientation": _to_json(orientation)}

    def set_local_pose(self, translation=None, orientation=None, camera_axes="world"):
        """Set the local-frame pose. See :meth:`set_world_pose`."""
        self._check_axes(camera_axes)
        self._camera.set_local_pose(
            self._vector(translation, 3, "translation"),
            self._vector(orientation, 4, "orientation"),
            camera_axes=camera_axes,
        )

    @staticmethod
    def _vector(value, length, name):
        """Return ``value`` as a float array of exactly ``length``, or ``None``.

        Validates the length here so a wrong-sized vector is a clean ``ValueError``
        (mapped to 400) rather than a deep numpy/BaseSensor error surfacing as a 500.
        """
        if value is None:
            return None
        arr = np.asarray(value, dtype=float)
        if arr.shape != (length,):
            raise ValueError(f"{name} must have {length} values, got {list(arr.shape) or 'a scalar'}")
        return arr

    @staticmethod
    def _check_axes(camera_axes):
        if camera_axes not in POSE_AXES:
            raise ValueError(f"camera_axes must be one of {POSE_AXES}, got {camera_axes!r}")

    # -- annotator management --------------------------------------------------

    def attach_annotator(self, annotator_name, **kwargs):
        """Attach an annotator by its replicator name. Requires :meth:`bind`/
        :meth:`initialize` first."""
        self._camera.attach_annotator(annotator_name, **kwargs)

    def detach_annotator(self, annotator_name):
        """Detach a previously attached annotator by name."""
        self._camera.detach_annotator(annotator_name)

    def add_rgb_to_frame(self, init_params=None):
        """Attach the rgb annotator."""
        self._camera.add_rgb_to_frame(init_params)

    def remove_rgb_from_frame(self):
        """Detach the rgb annotator."""
        self._camera.remove_rgb_from_frame()

    def add_normals_to_frame(self, init_params=None):
        """Attach the normals annotator."""
        self._camera.add_normals_to_frame(init_params)

    def remove_normals_from_frame(self):
        """Detach the normals annotator."""
        self._camera.remove_normals_from_frame()

    def add_motion_vectors_to_frame(self, init_params=None):
        """Attach the motion-vectors annotator."""
        self._camera.add_motion_vectors_to_frame(init_params)

    def remove_motion_vectors_from_frame(self):
        """Detach the motion-vectors annotator."""
        self._camera.remove_motion_vectors_from_frame()

    def add_occlusion_to_frame(self, init_params=None):
        """Attach the occlusion annotator."""
        self._camera.add_occlusion_to_frame(init_params)

    def remove_occlusion_from_frame(self):
        """Detach the occlusion annotator."""
        self._camera.remove_occlusion_from_frame()

    def add_distance_to_image_plane_to_frame(self, init_params=None):
        """Attach the distance-to-image-plane (depth) annotator."""
        self._camera.add_distance_to_image_plane_to_frame(init_params)

    def remove_distance_to_image_plane_from_frame(self):
        """Detach the distance-to-image-plane annotator."""
        self._camera.remove_distance_to_image_plane_from_frame()

    def add_distance_to_camera_to_frame(self, init_params=None):
        """Attach the distance-to-camera annotator."""
        self._camera.add_distance_to_camera_to_frame(init_params)

    def remove_distance_to_camera_from_frame(self):
        """Detach the distance-to-camera annotator."""
        self._camera.remove_distance_to_camera_from_frame()

    def add_bounding_box_2d_tight_to_frame(self, init_params=None):
        """Attach the tight 2D bounding-box annotator."""
        self._camera.add_bounding_box_2d_tight_to_frame(init_params)

    def remove_bounding_box_2d_tight_from_frame(self):
        """Detach the tight 2D bounding-box annotator."""
        self._camera.remove_bounding_box_2d_tight_from_frame()

    def add_bounding_box_2d_loose_to_frame(self, init_params=None):
        """Attach the loose 2D bounding-box annotator."""
        self._camera.add_bounding_box_2d_loose_to_frame(init_params)

    def remove_bounding_box_2d_loose_from_frame(self):
        """Detach the loose 2D bounding-box annotator."""
        self._camera.remove_bounding_box_2d_loose_from_frame()

    def add_bounding_box_3d_to_frame(self, init_params=None):
        """Attach the 3D bounding-box annotator."""
        self._camera.add_bounding_box_3d_to_frame(init_params)

    def remove_bounding_box_3d_from_frame(self):
        """Detach the 3D bounding-box annotator."""
        self._camera.remove_bounding_box_3d_from_frame()

    def add_semantic_segmentation_to_frame(self, init_params=None):
        """Attach the semantic-segmentation annotator."""
        self._camera.add_semantic_segmentation_to_frame(init_params)

    def remove_semantic_segmentation_from_frame(self):
        """Detach the semantic-segmentation annotator."""
        self._camera.remove_semantic_segmentation_from_frame()

    def add_instance_id_segmentation_to_frame(self, init_params=None):
        """Attach the instance-id-segmentation annotator."""
        self._camera.add_instance_id_segmentation_to_frame(init_params)

    def remove_instance_id_segmentation_from_frame(self):
        """Detach the instance-id-segmentation annotator."""
        self._camera.remove_instance_id_segmentation_from_frame()

    def add_instance_segmentation_to_frame(self, init_params=None):
        """Attach the instance-segmentation annotator."""
        self._camera.add_instance_segmentation_to_frame(init_params)

    def remove_instance_segmentation_from_frame(self):
        """Detach the instance-segmentation annotator."""
        self._camera.remove_instance_segmentation_from_frame()

    def add_pointcloud_to_frame(self, include_unlabelled=True, init_params=None):
        """Attach the pointcloud annotator."""
        self._camera.add_pointcloud_to_frame(include_unlabelled, init_params)

    def remove_pointcloud_from_frame(self):
        """Detach the pointcloud annotator."""
        self._camera.remove_pointcloud_from_frame()

    # -- data getters ----------------------------------------------------------

    def get_rgb(self, device=_HOST_DEVICE):
        """Latest RGB image as a nested list ``(H, W, 3)``, or ``None`` if not ready."""
        return _to_json(self._camera.get_rgb(device=device))

    def get_rgba(self, device=_HOST_DEVICE):
        """Latest RGBA image as a nested list ``(H, W, 4)``, or ``None``."""
        return _to_json(self._camera.get_rgba(device=device))

    def get_depth(self, device=_HOST_DEVICE):
        """Latest depth (distance to image plane) as a nested list ``(H, W)``, or ``None``."""
        return _to_json(self._camera.get_depth(device=device))

    def get_pointcloud(self, device=_HOST_DEVICE, world_frame=True):
        """Latest pointcloud as a nested list ``(N, 3)`` in world or camera frame."""
        return _to_json(self._camera.get_pointcloud(device=device, world_frame=world_frame))

    # -- optics ----------------------------------------------------------------

    def get_focal_length(self):
        """Focal length (stage units)."""
        return float(self._camera.get_focal_length())

    def set_focal_length(self, value):
        """Set the focal length (stage units)."""
        self._camera.set_focal_length(float(value))

    def get_focus_distance(self):
        """Distance from camera to focus plane (stage units)."""
        return float(self._camera.get_focus_distance())

    def set_focus_distance(self, value):
        """Set the focus distance (stage units)."""
        self._camera.set_focus_distance(float(value))

    def get_lens_aperture(self):
        """fStop value (0 disables depth-of-field)."""
        return float(self._camera.get_lens_aperture())

    def set_lens_aperture(self, value):
        """Set the fStop value."""
        self._camera.set_lens_aperture(float(value))

    def get_horizontal_aperture(self):
        """Horizontal aperture / sensor width (stage units)."""
        return float(self._camera.get_horizontal_aperture())

    def set_horizontal_aperture(self, value, maintain_square_pixels=True):
        """Set the horizontal aperture (stage units)."""
        self._camera.set_horizontal_aperture(float(value), maintain_square_pixels)

    def get_vertical_aperture(self):
        """Vertical aperture / sensor height (stage units)."""
        return float(self._camera.get_vertical_aperture())

    def set_vertical_aperture(self, value, maintain_square_pixels=True):
        """Set the vertical aperture (stage units)."""
        self._camera.set_vertical_aperture(float(value), maintain_square_pixels)

    def get_clipping_range(self):
        """``[near, far]`` clipping distances (stage units)."""
        return list(self._camera.get_clipping_range())

    def set_clipping_range(self, near_distance=None, far_distance=None):
        """Set near/far clipping distances; either may be ``None`` to leave unchanged."""
        self._camera.set_clipping_range(near_distance, far_distance)

    def get_projection_type(self):
        """[Deprecated in isaacsim] Camera projection type; prefer
        :meth:`get_lens_distortion_model`."""
        return self._camera.get_projection_type()

    def set_projection_type(self, value):
        """[Deprecated in isaacsim] Set the projection type; prefer
        :meth:`set_lens_distortion_model`."""
        self._camera.set_projection_type(value)

    def get_lens_distortion_model(self):
        """Lens distortion model name (``pinhole`` if unset)."""
        return self._camera.get_lens_distortion_model()

    def set_lens_distortion_model(self, value):
        """Set the lens distortion model (applies the matching schema)."""
        self._camera.set_lens_distortion_model(value)

    def get_projection_mode(self):
        """``perspective`` or ``orthographic``."""
        return self._camera.get_projection_mode()

    def set_projection_mode(self, value):
        """Set the projection mode (``perspective``/``orthographic``)."""
        self._camera.set_projection_mode(value)

    def get_stereo_role(self):
        """``mono``, ``left`` or ``right``."""
        return self._camera.get_stereo_role()

    def set_stereo_role(self, value):
        """Set the stereo role (``mono``/``left``/``right``)."""
        self._camera.set_stereo_role(value)

    # -- matrices / projection helpers ----------------------------------------

    def get_intrinsics_matrix(self):
        """3x3 intrinsics matrix (pinhole models only) as a nested list."""
        return _to_json(self._camera.get_intrinsics_matrix(device=_HOST_DEVICE))

    def get_view_matrix_ros(self):
        """World -> ROS-camera-frame view matrix as a nested list."""
        return _to_json(self._camera.get_view_matrix_ros(device=_HOST_DEVICE))

    def get_image_coords_from_world_points(self, points_3d):
        """Project world points ``(N, 3)`` to pixel coords ``(N, 2)`` (pinhole)."""
        return _to_json(self._camera.get_image_coords_from_world_points(np.asarray(points_3d, dtype=float)))

    def get_camera_points_from_image_coords(self, points_2d, depth):
        """Back-project pixel coords ``(N, 2)`` + ``depth`` ``(N,)`` to camera-frame
        points ``(N, 3)`` (pinhole)."""
        return _to_json(
            self._camera.get_camera_points_from_image_coords(
                np.asarray(points_2d, dtype=float), np.asarray(depth, dtype=float), device=_HOST_DEVICE
            )
        )

    def get_world_points_from_image_coords(self, points_2d, depth):
        """Back-project pixel coords ``(N, 2)`` + ``depth`` ``(N,)`` to world-frame
        points ``(N, 3)`` (pinhole)."""
        return _to_json(
            self._camera.get_world_points_from_image_coords(
                np.asarray(points_2d, dtype=float), np.asarray(depth, dtype=float), device=_HOST_DEVICE
            )
        )

    # -- shutter ---------------------------------------------------------------

    def set_shutter_properties(self, delay_open=None, delay_close=None):
        """Set motion-blur shutter open/close delays."""
        self._camera.set_shutter_properties(delay_open, delay_close)

    def get_shutter_properties(self):
        """``[delay_open, delay_close]`` shutter delays."""
        return list(self._camera.get_shutter_properties())

    # -- lens distortion models ------------------------------------------------

    def set_fisheye_polynomial_properties(
        self, nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, polynomial
    ):
        """[Deprecated in isaacsim] Set fisheye-polynomial distortion parameters."""
        self._camera.set_fisheye_polynomial_properties(
            nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, polynomial
        )

    def set_matching_fisheye_polynomial_properties(
        self, nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, distortion_model, distortion_fn
    ):
        """[Deprecated in isaacsim] Approximate an OpenCV fisheye model with ftheta
        polynomial coefficients."""
        self._camera.set_matching_fisheye_polynomial_properties(
            nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, distortion_model, distortion_fn
        )

    def get_fisheye_polynomial_properties(self):
        """Fisheye-polynomial parameters as a JSON list."""
        return _to_json(self._camera.get_fisheye_polynomial_properties())

    def set_rational_polynomial_properties(
        self, nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, distortion_model
    ):
        """[Deprecated in isaacsim] Set rational-polynomial distortion (routes to
        OpenCV pinhole)."""
        self._camera.set_rational_polynomial_properties(
            nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, distortion_model
        )

    def set_kannala_brandt_properties(
        self, nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, distortion_model
    ):
        """[Deprecated in isaacsim] Set Kannala-Brandt distortion (routes to OpenCV
        fisheye)."""
        self._camera.set_kannala_brandt_properties(
            nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, distortion_model
        )

    def set_ftheta_properties(
        self, nominal_height=None, nominal_width=None, optical_center=None, max_fov=None, distortion_coefficients=None
    ):
        """Apply the F-theta lens distortion model and set its parameters."""
        self._camera.set_ftheta_properties(
            nominal_height, nominal_width, optical_center, max_fov, distortion_coefficients
        )

    def get_ftheta_properties(self):
        """F-theta distortion parameters as a JSON list."""
        return _to_json(self._camera.get_ftheta_properties())

    def set_kannala_brandt_k3_properties(
        self, nominal_height=None, nominal_width=None, optical_center=None, max_fov=None, distortion_coefficients=None
    ):
        """Apply the Kannala-Brandt K3 lens distortion model and set its parameters."""
        self._camera.set_kannala_brandt_k3_properties(
            nominal_height, nominal_width, optical_center, max_fov, distortion_coefficients
        )

    def get_kannala_brandt_k3_properties(self):
        """Kannala-Brandt K3 distortion parameters as a JSON list."""
        return _to_json(self._camera.get_kannala_brandt_k3_properties())

    def set_rad_tan_thin_prism_properties(
        self, nominal_height=None, nominal_width=None, optical_center=None, max_fov=None, distortion_coefficients=None
    ):
        """Apply the Radial-Tangential Thin-Prism lens distortion model and set its
        parameters."""
        self._camera.set_rad_tan_thin_prism_properties(
            nominal_height, nominal_width, optical_center, max_fov, distortion_coefficients
        )

    def get_rad_tan_thin_prism_properties(self):
        """Radial-Tangential Thin-Prism distortion parameters as a JSON list."""
        return _to_json(self._camera.get_rad_tan_thin_prism_properties())

    def set_lut_properties(
        self,
        nominal_height=None,
        nominal_width=None,
        optical_center=None,
        ray_enter_direction_texture=None,
        ray_exit_position_texture=None,
    ):
        """Apply the LUT lens distortion model and set its texture parameters."""
        self._camera.set_lut_properties(
            nominal_height, nominal_width, optical_center, ray_enter_direction_texture, ray_exit_position_texture
        )

    def get_lut_properties(self):
        """LUT distortion parameters as a JSON list."""
        return _to_json(self._camera.get_lut_properties())

    def set_opencv_pinhole_properties(self, cx=None, cy=None, fx=None, fy=None, pinhole=None):
        """Apply the OpenCV pinhole distortion model and set its parameters."""
        self._camera.set_opencv_pinhole_properties(cx=cx, cy=cy, fx=fx, fy=fy, pinhole=pinhole)

    def get_opencv_pinhole_properties(self):
        """OpenCV pinhole distortion parameters as a JSON list."""
        return _to_json(self._camera.get_opencv_pinhole_properties())

    def set_opencv_fisheye_properties(self, cx=None, cy=None, fx=None, fy=None, fisheye=None):
        """Apply the OpenCV fisheye distortion model and set its parameters."""
        self._camera.set_opencv_fisheye_properties(cx=cx, cy=cy, fx=fx, fy=fy, fisheye=fisheye)

    def get_opencv_fisheye_properties(self):
        """OpenCV fisheye distortion parameters as a JSON list."""
        return _to_json(self._camera.get_opencv_fisheye_properties())
