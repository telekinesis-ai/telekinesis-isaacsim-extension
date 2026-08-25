# SPDX-License-Identifier: Apache-2.0
"""
Generic lidar sensor device: binds one legacy PhysX Lidar prim at a prim path
and exposes its full surface over the bridge.

This is the sensor counterpart to :mod:`..core.camera`, for the legacy PhysX
range-sensor Lidar rather than the RTX camera pipeline. Where ``Camera`` wraps a
single ``isaacsim.sensors.camera.Camera`` handle, this wraps one Lidar prim
through ``isaacsim.sensors.physx._range_sensor`` -- a global, singleton
interface (``acquire_lidar_sensor_interface()``) that reads the *runtime*
per-frame buffers (depth/intensity/point cloud/...) for a sensor, keyed by
prim path. There is no per-instance Python object here, unlike
``camera_sensor.Camera``.

The sensor's *static* configuration (fov, resolution, range, rotation rate,
...) has no getter/setter on that interface -- it lives as plain USD attributes
on the prim (set by the ``RangeSensorCreateLidar`` kit command that creates
it), so this class reads/writes them directly via ``Usd.Prim.GetAttribute``
rather than importing the schema's Python bindings.

So unlike ``Camera``, which has one clean 1:1 method mapping onto a single
isaacsim class, this class composes two references and there is no upstream
class for a one-to-one mapping test to check against. The bridge-only
additions are the same three as ``Camera``'s: :meth:`bind`, :meth:`capture`,
:meth:`info` (see ``BRIDGE_ONLY_METHODS`` below).

The prim itself is created via the ``RangeSensorCreateLidar`` kit command (the
legacy sensor has no isaacsim wrapper class like ``Camera`` that creates/wraps
a prim for you). Re-running :class:`Lidar`'s constructor against the same
`prim_path` re-applies the requested configuration to the existing prim rather
than erroring, matching the camera service's "re-PUT rebuilds the device"
contract.

Native Isaac units throughout: stage units (meters) for range/poses, degrees
for fov/resolution/yaw (matching the ``RangeSensorCreateLidar`` command and USD
attributes), radians for the runtime azimuth/zenith buffers (matching the
``_range_sensor`` interface). Every method returns JSON-serializable Python
(lists/floats/ints/dicts), never raw numpy -- the future service layer maps
``ValueError`` (bad client input) and ``RuntimeError`` (bind failure) to HTTP
errors, exactly as the camera service does.

The capture path follows the same ``async def`` / ``next_update_async``
blocking style as the camera (no background worker, no command queue). A
rotating lidar (``rotation_rate`` > 0) may not complete a full sweep in one
physics step; :meth:`capture` reports ``num_cols_ticked`` alongside the data so
a client can tell a partial sweep from a full one.
"""

import time

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from isaacsim.core import prims
from isaacsim.sensors.physx import _range_sensor
import carb

# -- static tables ----------------------------------------------------------

# The interface method (isaacsim.sensors.physx._range_sensor.LidarSensorInterface)
# that serves each data type this wrapper can capture. The lidar counterpart to
# camera.py's DATA_TYPE_TO_ADDER: instead of attaching an annotator, reading a
# type just means calling the matching interface getter for this prim path.
DATA_TYPE_TO_GETTER = {
    "depth": "get_depth_data",
    "linear_depth": "get_linear_depth_data",
    "intensity": "get_intensity_data",
    "zenith": "get_zenith_data",
    "azimuth": "get_azimuth_data",
    "point_cloud": "get_point_cloud_data",
    "semantic": "get_semantic_data",
}

# Everything the client may request from capture(). "semantic" additionally
# requires enable_semantics -- see Lidar.__init__.
SUPPORTED_DATA_TYPES = frozenset(DATA_TYPE_TO_GETTER)

# Public methods on the wrapper with no one-to-one counterpart on the
# underlying isaacsim surface -- bridge conveniences, matching camera.py's
# constant of the same name (see the module docstring for why there is no
# ast-parsed mapping test for this module).
BRIDGE_ONLY_METHODS = frozenset({"bind", "capture", "info"})

# Legacy PhysX lidars need a few physics steps before the first sweep lands in
# the runtime buffers; reuse the camera's retry budget so a freshly created
# lidar has time to come up before bind gives up.
_BIND_RETRIES = 60

# Raw USD attribute names the RangeSensorCreateLidar command creates on a Lidar
# prim, keyed by the config kwarg this class's constructor takes for each one.
# Read/written directly via Usd.Prim.GetAttribute so this module needs no
# schema-specific pxr import (just plain Usd, via the stage).
_CONFIG_ATTRS = {
    "min_range": "minRange",
    "max_range": "maxRange",
    "draw_points": "drawPoints",
    "draw_lines": "drawLines",
    "horizontal_fov": "horizontalFov",
    "vertical_fov": "verticalFov",
    "horizontal_resolution": "horizontalResolution",
    "vertical_resolution": "verticalResolution",
    "rotation_rate": "rotationRate",
    "high_lod": "highLod",
    "yaw_offset": "yawOffset",
    "enable_semantics": "enableSemantics",
}

# Config fields that are booleans on the USD prim; everything else in
# _CONFIG_ATTRS is a float. Drives the Get*/Set* cast in the config accessors.
_BOOL_CONFIG_FIELDS = frozenset({"draw_points", "draw_lines", "high_lod", "enable_semantics"})

# Base RangeSensor schema attribute toggling whether PhysX computes this
# sensor each step -- the pause/resume/is_paused backing store.
_ENABLED_ATTR = "enabled"


def _to_json(value):  # pylint: disable=too-many-return-statements
    """Convert an Isaac/numpy return value into JSON-serializable Python.

    Same contract as ``..core.camera._to_json`` (duplicated rather than
    imported -- see the module docstring: each core wrapper is self-contained).
    Non-finite floats (``inf``/``nan``) map to ``None``: a lidar ray that hits
    nothing reads back as ``inf`` at ``max_range``, and Starlette serializes
    with ``allow_nan=False``, so leaving them in would make the response
    invalid JSON (a 500).
    """
    if value is None:
        return None
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
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def _finalize(raw):
    """``_to_json(raw)``, but an empty array (buffer not populated yet) reads as
    ``None`` rather than ``[]`` -- the lidar counterpart to a camera getter
    returning ``None`` before RTX warmup completes."""
    if raw is None or (hasattr(raw, "size") and raw.size == 0):
        return None
    return _to_json(raw)


class Lidar:
    """Binds a single legacy PhysX Lidar at ``prim_path`` and exposes its full API.

    Construction creates the Lidar prim (via the ``RangeSensorCreateLidar`` kit
    command) if none exists yet at `prim_path`, then applies the requested
    configuration either way -- so re-constructing against an already-bound
    prim reconfigures it in place. :meth:`bind` then waits for the runtime
    buffers to come up before the device is usable.
    """

    def __init__(
        self,
        prim_path,
        name="lidar",
        min_range=0.4,
        max_range=100.0,
        horizontal_fov=360.0,
        vertical_fov=30.0,
        horizontal_resolution=0.4,
        vertical_resolution=4.0,
        rotation_rate=20.0,
        high_lod=False,
        draw_points=False,
        draw_lines=False,
        yaw_offset=0.0,
        data_types=None,
        position=None,
        orientation=None,
    ):
        """Create/reconfigure the lidar prim and validate ``data_types``; call
        :meth:`bind` before capturing.

        ``rotation_rate`` is in Hz (``0`` means an instantaneous full-sweep
        lidar rather than a physically rotating one). ``data_types`` is the set
        of outputs to produce (default ``["point_cloud"]``); each must be in
        :data:`SUPPORTED_DATA_TYPES` or this raises ``ValueError``, mirroring
        the camera's unknown-data-type check. Requesting ``"semantic"`` sets
        the prim's ``enableSemantics`` attribute; every other type is always
        readable off the interface once the sensor is bound.
        """
        self.prim_path = prim_path
        self._name = name
        self.data_types = self._validate_data_types(data_types or ["point_cloud"])
        self._lidar = _range_sensor.acquire_lidar_sensor_interface()
        self._prim = self._apply_config(
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
            enable_semantics="semantic" in self.data_types,
        )
        self._xform = prims.SingleXFormPrim(prim_path)
        if position is not None or orientation is not None:
            self._xform.set_world_pose(
                None if position is None else np.asarray(position, dtype=float),
                None if orientation is None else np.asarray(orientation, dtype=float),
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
                f"unknown lidar data type(s) {unknown}; supported: {sorted(SUPPORTED_DATA_TYPES)}"
            )
        return requested

    def _apply_config(self, **config):
        """Create the Lidar prim if ``prim_path`` doesn't exist yet, then set every
        config attribute from ``config`` on it either way. Returns the ``Usd.Prim``.

        ``RangeSensorCreateLidar`` takes a parent path plus a relative child
        name rather than one absolute path, so an absolute ``prim_path`` is
        split at its last ``/`` before calling it.
        """
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.prim_path)
        if not prim.IsValid():
            parent_path, _, name = self.prim_path.rpartition("/")
            success, _ = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path="/" + name,
                parent=parent_path or "/",
                min_range=config["min_range"],
                max_range=config["max_range"],
                draw_points=config["draw_points"],
                draw_lines=config["draw_lines"],
                horizontal_fov=config["horizontal_fov"],
                vertical_fov=config["vertical_fov"],
                horizontal_resolution=config["horizontal_resolution"],
                vertical_resolution=config["vertical_resolution"],
                rotation_rate=config["rotation_rate"],
                high_lod=config["high_lod"],
                yaw_offset=config["yaw_offset"],
                enable_semantics=config["enable_semantics"],
            )
            if not success:
                raise RuntimeError(f"failed to create lidar prim at {self.prim_path}")
            prim = stage.GetPrimAtPath(self.prim_path)

        if not prim.HasAttribute(_CONFIG_ATTRS["min_range"]):
            raise ValueError(
                f"prim at {self.prim_path} is not a Lidar (missing range-sensor attributes)"
            )

        # Re-apply every field regardless of create-vs-wrap, so a re-PUT with a
        # new config actually takes effect on an already-bound prim.
        for field, attr_name in _CONFIG_ATTRS.items():
            value = config[field]
            prim.GetAttribute(attr_name).Set(
                bool(value) if field in _BOOL_CONFIG_FIELDS else float(value)
            )
        return prim

    # -- bridge lifecycle / capture (not part of the isaacsim surface) ---------

    async def bind(self):
        """Play the timeline and wait for the runtime buffers to come up.

        Mirrors :meth:`..core.camera.Camera.bind`: play the timeline, pump
        ``next_update_async`` a couple frames, then loop until a frame reads
        back non-empty. ``carb.log_info`` on success; raises ``RuntimeError``
        carrying the last error if the lidar never yields data.
        """
        omni.timeline.get_timeline_interface().play()
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()

        probe = self.data_types[0]
        last_exc = None
        for _ in range(_BIND_RETRIES):
            await app.next_update_async()
            try:
                if self._read(probe) is not None:
                    self._initialized = True
                    carb.log_info(f"[bridge] bound lidar {self.prim_path}: {self.data_types}")
                    return
            except Exception as exc:  # data not ready yet -> keep pumping frames
                last_exc = exc

        detail = f"lidar at {self.prim_path} did not produce data"
        if last_exc is not None:
            detail += f" (last error: {last_exc!r})"
        raise RuntimeError(detail)

    def _read(self, data_type):
        """Read one data type's latest buffer as JSON, or ``None`` if not ready yet."""
        method_name = DATA_TYPE_TO_GETTER.get(data_type)
        if method_name is None:
            raise ValueError(f"unknown lidar data type {data_type!r}")
        return _finalize(getattr(self._lidar, method_name)(self.prim_path))

    async def capture(self, data_types=None):
        """Pump one frame and return a JSON snapshot of the requested outputs.

        The core read method (analogue of the camera's ``capture``): awaits one
        ``next_update_async`` so the sensor is current, then returns
        ``{<data_type>: <array or None>, ...}`` plus ``num_cols_ticked`` and a
        monotonic ``timestamp``. ``data_types`` defaults to every type this
        lidar was bound with; requesting a type this lidar wasn't bound with
        raises ``ValueError`` rather than silently returning ``null``.
        """
        requested = list(data_types) if data_types is not None else list(self.data_types)
        unavailable = [d for d in requested if d not in self.data_types]
        if unavailable:
            raise ValueError(
                f"lidar at {self.prim_path} was not bound with data type(s) {unavailable}; "
                f"available: {sorted(self.data_types)}"
            )
        await omni.kit.app.get_app().next_update_async()
        out = {data_type: self._read(data_type) for data_type in requested}
        out["num_cols_ticked"] = self.get_num_cols_ticked()

        app = omni.kit.app.get_app()
        num_cols = self.get_num_cols()

        collected_cols = 0
        chunks = {data_type: [] for data_type in requested}
        static_data = {}

        while collected_cols < num_cols:
            await app.next_update_async()

            cols_ticked = self.get_num_cols_ticked()
            if cols_ticked <= 0:
                continue

            cols_to_take = min(cols_ticked, num_cols - collected_cols)

            for data_type in requested:
                value = self._read(data_type)

                if value is None:
                    continue

                # Zenith describes the vertical channels and does not change
                # across the horizontal sweep.
                if data_type == "zenith":
                    static_data[data_type] = value
                    continue

                array = np.asarray(value)

                # Find the horizontal-column axis. For the point cloud you are
                # currently seeing (cols, rows, xyz), this will be axis 0.
                matching_axes = [
                    axis
                    for axis, size in enumerate(array.shape)
                    if size == cols_ticked
                ]

                if not matching_axes:
                    # Data that is not indexed by horizontal scan column.
                    static_data[data_type] = value
                    continue

                column_axis = matching_axes[0]

                # If this tick would push us past one full revolution,
                # keep only the remaining required columns.
                if cols_to_take < cols_ticked:
                    slices = [slice(None)] * array.ndim
                    slices[column_axis] = slice(0, cols_to_take)
                    array = array[tuple(slices)]

                chunks[data_type].append((array, column_axis))

            collected_cols += cols_to_take

        out = {}

        for data_type in requested:
            if data_type in static_data:
                out[data_type] = _to_json(static_data[data_type])
                continue

            data_chunks = chunks[data_type]

            if not data_chunks:
                out[data_type] = None
                continue

            column_axis = data_chunks[0][1]
            arrays = [array for array, _ in data_chunks]

            out[data_type] = _to_json(
                np.concatenate(arrays, axis=column_axis)
            )

        out["timestamp"] = time.monotonic() - self._start_time
        return out

    def info(self):
        """Static description of this lidar: prim path, bound data types, range,
        fov, resolution, rotation rate, and current row/column counts. The
        analogue of ``Camera.info()``."""
        return {
            "prim_path": self.prim_path,
            "data_types": list(self.data_types),
            "min_range": self.get_min_range(),
            "max_range": self.get_max_range(),
            "horizontal_fov": self.get_horizontal_fov(),
            "vertical_fov": self.get_vertical_fov(),
            "horizontal_resolution": self.get_horizontal_resolution(),
            "vertical_resolution": self.get_vertical_resolution(),
            "rotation_rate": self.get_rotation_rate(),
            "high_lod": self.get_high_lod(),
            "yaw_offset": self.get_yaw_offset(),
            "enable_semantics": self.get_enable_semantics(),
            "num_rows": self.get_num_rows(),
            "num_cols": self.get_num_cols(),
        }

    # -- lifecycle / collection control -----------------------------------------

    def destroy(self):
        """Mark this wrapper uninitialized.

        Unlike ``Camera`` (which owns a render product and annotators to free),
        the legacy interface holds no per-instance resource -- the PhysX Lidar
        sensor lives and dies with its USD prim, which the bridge leaves in the
        stage on delete, exactly like a camera's prim.
        """
        self._initialized = False

    def pause(self):
        """Pause sensor computation (clears the prim's ``enabled`` attribute)."""
        self._prim.GetAttribute(_ENABLED_ATTR).Set(False)

    def resume(self):
        """Resume sensor computation (sets the prim's ``enabled`` attribute)."""
        self._prim.GetAttribute(_ENABLED_ATTR).Set(True)

    def is_paused(self):
        """Whether sensor computation is currently paused."""
        return not bool(self._prim.GetAttribute(_ENABLED_ATTR).Get())

    def is_lidar_sensor(self):
        """Whether ``prim_path`` currently resolves to a live PhysX lidar sensor."""
        return bool(self._lidar.is_lidar_sensor(self.prim_path))

    # -- pose --------------------------------------------------------------------

    def get_world_pose(self):
        """World-frame pose as ``{"position": [x, y, z], "orientation": [w, x, y, z]}``."""
        position, orientation = self._xform.get_world_pose()
        return {"position": _to_json(position), "orientation": _to_json(orientation)}

    def set_world_pose(self, position=None, orientation=None):
        """Set the world-frame pose. ``position`` is ``[x, y, z]``, ``orientation`` a
        ``[w, x, y, z]`` quaternion; either may be ``None`` to leave it untouched."""
        self._xform.set_world_pose(
            self._vector(position, 3, "position"),
            self._vector(orientation, 4, "orientation"),
        )

    def get_local_pose(self):
        """Local-frame (parent-relative) pose, same shape as :meth:`get_world_pose`
        but keyed ``translation`` instead of ``position``."""
        translation, orientation = self._xform.get_local_pose()
        return {"translation": _to_json(translation), "orientation": _to_json(orientation)}

    def set_local_pose(self, translation=None, orientation=None):
        """Set the local-frame pose. See :meth:`set_world_pose`."""
        self._xform.set_local_pose(
            self._vector(translation, 3, "translation"),
            self._vector(orientation, 4, "orientation"),
        )

    @staticmethod
    def _vector(value, length, name):
        """Return ``value`` as a float array of exactly ``length``, or ``None``.

        Validates the length here so a wrong-sized vector is a clean
        ``ValueError`` (mapped to 400) rather than a deep numpy error surfacing
        as a 500.
        """
        if value is None:
            return None
        arr = np.asarray(value, dtype=float)
        if arr.shape != (length,):
            raise ValueError(
                f"{name} must have {length} values, got {list(arr.shape) or 'a scalar'}"
            )
        return arr

    # -- rate / geometry config (raw USD attributes -- see _CONFIG_ATTRS) -------

    def get_min_range(self):
        """Minimum sensing range (stage units)."""
        return float(self._prim.GetAttribute(_CONFIG_ATTRS["min_range"]).Get())

    def set_min_range(self, value):
        """Set the minimum sensing range (stage units)."""
        self._prim.GetAttribute(_CONFIG_ATTRS["min_range"]).Set(float(value))

    def get_max_range(self):
        """Maximum sensing range (stage units)."""
        return float(self._prim.GetAttribute(_CONFIG_ATTRS["max_range"]).Get())

    def set_max_range(self, value):
        """Set the maximum sensing range (stage units)."""
        self._prim.GetAttribute(_CONFIG_ATTRS["max_range"]).Set(float(value))

    def get_horizontal_fov(self):
        """Horizontal field of view (degrees)."""
        return float(self._prim.GetAttribute(_CONFIG_ATTRS["horizontal_fov"]).Get())

    def set_horizontal_fov(self, value):
        """Set the horizontal field of view (degrees)."""
        self._prim.GetAttribute(_CONFIG_ATTRS["horizontal_fov"]).Set(float(value))

    def get_vertical_fov(self):
        """Vertical field of view (degrees)."""
        return float(self._prim.GetAttribute(_CONFIG_ATTRS["vertical_fov"]).Get())

    def set_vertical_fov(self, value):
        """Set the vertical field of view (degrees)."""
        self._prim.GetAttribute(_CONFIG_ATTRS["vertical_fov"]).Set(float(value))

    def get_horizontal_resolution(self):
        """Horizontal angular resolution (degrees per column)."""
        return float(self._prim.GetAttribute(_CONFIG_ATTRS["horizontal_resolution"]).Get())

    def set_horizontal_resolution(self, value):
        """Set the horizontal angular resolution (degrees per column)."""
        self._prim.GetAttribute(_CONFIG_ATTRS["horizontal_resolution"]).Set(float(value))

    def get_vertical_resolution(self):
        """Vertical angular resolution (degrees per row)."""
        return float(self._prim.GetAttribute(_CONFIG_ATTRS["vertical_resolution"]).Get())

    def set_vertical_resolution(self, value):
        """Set the vertical angular resolution (degrees per row)."""
        self._prim.GetAttribute(_CONFIG_ATTRS["vertical_resolution"]).Set(float(value))

    def get_rotation_rate(self):
        """Rotation rate (Hz); ``0`` means an instantaneous full-sweep lidar."""
        return float(self._prim.GetAttribute(_CONFIG_ATTRS["rotation_rate"]).Get())

    def set_rotation_rate(self, value):
        """Set the rotation rate (Hz)."""
        self._prim.GetAttribute(_CONFIG_ATTRS["rotation_rate"]).Set(float(value))

    def get_yaw_offset(self):
        """Yaw offset applied to the scan pattern (degrees)."""
        return float(self._prim.GetAttribute(_CONFIG_ATTRS["yaw_offset"]).Get())

    def set_yaw_offset(self, value):
        """Set the yaw offset (degrees)."""
        self._prim.GetAttribute(_CONFIG_ATTRS["yaw_offset"]).Set(float(value))

    def get_high_lod(self):
        """Whether the sensor renders at high level-of-detail."""
        return bool(self._prim.GetAttribute(_CONFIG_ATTRS["high_lod"]).Get())

    def set_high_lod(self, value):
        """Set whether the sensor renders at high level-of-detail."""
        self._prim.GetAttribute(_CONFIG_ATTRS["high_lod"]).Set(bool(value))

    def get_draw_points(self):
        """Whether hit points are drawn in the viewport."""
        return bool(self._prim.GetAttribute(_CONFIG_ATTRS["draw_points"]).Get())

    def set_draw_points(self, value):
        """Set whether hit points are drawn in the viewport."""
        self._prim.GetAttribute(_CONFIG_ATTRS["draw_points"]).Set(bool(value))

    def get_draw_lines(self):
        """Whether scan rays are drawn in the viewport."""
        return bool(self._prim.GetAttribute(_CONFIG_ATTRS["draw_lines"]).Get())

    def set_draw_lines(self, value):
        """Set whether scan rays are drawn in the viewport."""
        self._prim.GetAttribute(_CONFIG_ATTRS["draw_lines"]).Set(bool(value))

    def get_enable_semantics(self):
        """Whether per-hit semantic labels are captured (``semantic`` data type)."""
        return bool(self._prim.GetAttribute(_CONFIG_ATTRS["enable_semantics"]).Get())

    def set_enable_semantics(self, value):
        """Set whether per-hit semantic labels are captured."""
        self._prim.GetAttribute(_CONFIG_ATTRS["enable_semantics"]).Set(bool(value))

    # -- data getters (isaacsim.sensors.physx._range_sensor interface) ----------

    def get_depth_data(self):
        """Latest quantized depth buffer ``(num_rows, num_cols)``, or ``None``."""
        return _finalize(self._lidar.get_depth_data(self.prim_path))

    def get_linear_depth_data(self):
        """Latest linear depth buffer in meters ``(num_rows, num_cols)``, or ``None``."""
        return _finalize(self._lidar.get_linear_depth_data(self.prim_path))

    def get_intensity_data(self):
        """Latest return-intensity buffer ``(num_rows, num_cols)``, or ``None``."""
        return _finalize(self._lidar.get_intensity_data(self.prim_path))

    def get_zenith_data(self):
        """Per-row vertical scan angles (radians), or ``None``."""
        return _finalize(self._lidar.get_zenith_data(self.prim_path))

    def get_azimuth_data(self):
        """Per-column horizontal scan angles (radians), or ``None``."""
        return _finalize(self._lidar.get_azimuth_data(self.prim_path))

    def get_point_cloud_data(self):
        """Latest hit points in world frame, ``(num_rows, num_cols, 3)``, or ``None``."""
        return _finalize(self._lidar.get_point_cloud_data(self.prim_path))

    def get_semantic_data(self):
        """Per-hit semantic ids ``(num_rows, num_cols)``, or ``None``. Requires
        ``enable_semantics``."""
        return _finalize(self._lidar.get_semantic_data(self.prim_path))

    # -- introspection (isaacsim.sensors.physx._range_sensor interface) ---------

    def get_num_rows(self):
        """Number of scan rows (vertical channels) the sensor currently reports."""
        return int(self._lidar.get_num_rows(self.prim_path))

    def get_num_cols(self):
        """Number of scan columns (horizontal samples) a full sweep produces."""
        return int(self._lidar.get_num_cols(self.prim_path))

    def get_num_cols_ticked(self):
        """Number of scan columns completed so far this physics step (a rotating
        lidar may not finish a full sweep in one step)."""
        return int(self._lidar.get_num_cols_ticked(self.prim_path))

    def get_azimuth_range(self):
        """``[min, max]`` horizontal scan angles (radians), or ``None`` if the
        azimuth buffer isn't populated yet.

        The ``_range_sensor`` interface has no dedicated range getter, so this
        derives the range from :meth:`get_azimuth_data`, the per-column angles
        it does expose.
        """
        azimuth = self._lidar.get_azimuth_data(self.prim_path)
        return None if azimuth.size == 0 else [float(azimuth.min()), float(azimuth.max())]

    def get_zenith_range(self):
        """``[min, max]`` vertical scan angles (radians), or ``None`` if the
        zenith buffer isn't populated yet.

        Derived from :meth:`get_zenith_data` -- see :meth:`get_azimuth_range`.
        """
        zenith = self._lidar.get_zenith_data(self.prim_path)
        return None if zenith.size == 0 else [float(zenith.min()), float(zenith.max())]
