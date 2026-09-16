# SPDX-License-Identifier: Apache-2.0
"""
Lightbeam sensor device: binds one ``IsaacLightBeamSensor`` prim and reads its beams.

A lightbeam sensor casts one PhysX ray per beam along its forward axis, spread evenly
along its curtain axis when it has more than one, and records per beam whether the
beam was broken and how far away the surface that broke it is. Only prims with a
collider are seen; purely visual geometry is invisible to the beam.

Like the lidar (:mod:`.lidar`) this is a legacy PhysX range sensor: the prim carries
the *static* configuration as plain USD attributes, while the per-step readings come
from the global ``isaacsim.sensors.physx._range_sensor`` interface, keyed by prim
path. There is no per-instance Python object to wrap. Unlike the lidar, the sensor's
component re-reads its attributes on every change, so :meth:`configure` takes effect
on the next physics step even while the timeline plays.

The sensor is sampled, not queried: a reading is whatever the last physics step left
in the buffers, so it only means anything while the timeline plays. Before the first
step every beam reads as clear, which is why reading a stopped simulation is refused
here rather than answered with a beam that looks unbroken.

A beam that is not broken reports its distance as the sensor's maximum range, not as
zero and not as infinity. A broken beam reports the distance from the sensor origin to
the surface, which is never below the sensor's minimum range -- the minimum range is a
blind zone the ray starts beyond, so an object closer than it is not seen at all.

Native Isaac units: meters for ranges and distances. Hit positions are in the sensor's
own frame, not the stage's.

``isaacsim.sensors.physx`` is deprecated as of Isaac Sim 6.0 in favour of the raycast
sensor in ``isaacsim.sensors.experimental.physics``, but it still ships and it is what
drives the ``IsaacLightBeamSensor`` prims existing scenes are authored with.
"""

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.sensors.physx import _range_sensor
import carb
from pxr import Gf

# Type name of the prim this wraps, as created by IsaacSensorCreateLightBeamSensor.
LIGHTBEAM_PRIM_TYPE = "IsaacLightBeamSensor"

# Public methods with no counterpart on an upstream isaacsim class -- the legacy
# sensor has no wrapper class, so the whole surface is the bridge's own (mirrors
# ``..core.lidar.BRIDGE_ONLY_METHODS``).
BRIDGE_ONLY_METHODS = frozenset({"bind", "read", "configure", "info"})

# A freshly played sensor needs a few physics steps before its first reading lands in
# the runtime buffers; same retry budget as the camera and the lidar.
_BIND_RETRIES = 60

# Raw USD attribute names on an IsaacLightBeamSensor prim, keyed by the config field
# this class uses for each one. Read and written directly via Usd.Prim.GetAttribute, so
# this module needs no schema-specific pxr import.
_CONFIG_ATTRS = {
    "num_rays": "numRays",
    "curtain_length": "curtainLength",
    "forward_axis": "forwardAxis",
    "curtain_axis": "curtainAxis",
    "min_range": "minRange",
    "max_range": "maxRange",
}

# Config fields that are XYZ vectors on the prim; the rest are a float, except
# num_rays, which is an int.
_VECTOR_CONFIG_FIELDS = frozenset({"forward_axis", "curtain_axis"})

# IsaacBaseSensor attribute toggling whether PhysX computes this sensor each step --
# the pause/resume/is_paused backing store.
_ENABLED_ATTR = "enabled"


def _values(raw, dtype=float):
    """Return one per-beam buffer as a plain list, or ``None`` if it is not populated.

    An empty buffer means the sensor has not produced a reading yet, which reads back
    as ``null`` rather than as an empty list -- the lightbeam counterpart to a camera
    getter answering ``None`` before its warmup completes.
    """
    if raw is None or len(raw) == 0:
        return None
    return np.asarray(raw, dtype=dtype).tolist()


class LightBeamSensor:
    """Binds a single ``IsaacLightBeamSensor`` at ``prim_path`` and exposes its readings.

    The prim has to exist in the stage: unlike a lidar, a lightbeam's placement and
    aim are the whole sensor, so nothing here creates one. Registering a sensor enables
    it if the scene left it disabled, because a disabled sensor never reports a hit;
    :meth:`pause` switches it off again. :meth:`bind` then waits for the runtime buffers
    to come up before the device is usable.
    """

    def __init__(self, prim_path, name="lightbeam"):
        """Bind the sensor prim and read back its beam configuration.

        Raises ``ValueError`` if no stage is open, or the path does not resolve to an
        ``IsaacLightBeamSensor`` prim.
        """
        self.prim_path = prim_path
        self._name = name

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise ValueError("no USD stage is open")

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"prim {prim_path!r} not found in the open stage")
        if prim.GetTypeName() != LIGHTBEAM_PRIM_TYPE:
            raise ValueError(
                f"prim {prim_path!r} is a {prim.GetTypeName() or 'typeless prim'}, not a "
                f"{LIGHTBEAM_PRIM_TYPE}"
            )

        self._prim = prim
        self._sensor = _range_sensor.acquire_lightbeam_sensor_interface()
        self._initialized = False
        if self.is_paused():
            self.resume()
            carb.log_info(f"[bridge] enabled the disabled lightbeam sensor at {prim_path}")

    # -- bridge lifecycle / readings (not part of the isaacsim surface) ---------

    async def bind(self):
        """Play the timeline and wait for the sensor's runtime buffers to come up.

        Mirrors :meth:`..core.lidar.Lidar.bind`: play the timeline, pump a couple of
        frames, then loop until a reading comes back populated. Raises ``RuntimeError``
        carrying the last error if the sensor never produces one.
        """
        omni.timeline.get_timeline_interface().play()
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()

        last_exc = None
        for _ in range(_BIND_RETRIES):
            await app.next_update_async()
            try:
                if _values(self._sensor.get_beam_hit_data(self.prim_path), bool) is not None:
                    self._initialized = True
                    carb.log_info(
                        f"[bridge] bound lightbeam {self.prim_path}: "
                        f"{self.get_num_rays()} beam(s)"
                    )
                    return
            except Exception as exc:  # buffers not ready yet -> keep pumping frames
                last_exc = exc

        detail = f"lightbeam sensor at {self.prim_path} did not produce a reading"
        if last_exc is not None:
            detail += f" (last error: {last_exc!r})"
        raise RuntimeError(detail)

    def read(self):
        """Return the beams as of the last physics step.

        ``broken`` is true when *any* beam is broken, which is what makes a curtain
        useful for detecting an object whose height is not known in advance, and is the
        whole output of the photoelectric switch this sensor stands in for.
        ``linear_depth`` is a raycast distance rather than something a real light
        barrier measures, so treat it as ground truth for setting a system up, not as a
        reading hardware would also give.

        Every per-beam field is ``null`` until the sensor has produced its first
        reading. Raises ``RuntimeError`` while the timeline is not playing, since the
        buffers then hold a stale reading that looks like a clear beam.
        """
        if not omni.timeline.get_timeline_interface().is_playing():
            raise RuntimeError(
                f"lightbeam sensor {self.prim_path} cannot be read while the simulation is "
                "stopped; start the timeline first"
            )

        beam_hit = _values(self._sensor.get_beam_hit_data(self.prim_path), bool)
        return {
            "num_rays": self.get_num_rays(),
            "broken": None if beam_hit is None else any(beam_hit),
            "beam_hit": beam_hit,
            "linear_depth": _values(self._sensor.get_linear_depth_data(self.prim_path)),
            "hit_pos": _values(self._sensor.get_hit_pos_data(self.prim_path)),
        }

    def configure(
        self,
        num_rays=None,
        curtain_length=None,
        forward_axis=None,
        curtain_axis=None,
        min_range=None,
        max_range=None,
    ):
        """Change the beam layout and detection range, leaving unset fields untouched.

        A single-beam sensor is a point detector along its forward axis. Giving it more
        than one beam spreads the beams evenly over ``curtain_length`` along the curtain
        axis, which is what lets it detect an object of unknown height. The axes are XYZ
        vectors in the sensor's own frame.

        The sensor's component re-reads all of this on change, so a write takes effect
        on the next physics step, including while the timeline plays.

        Raises ``ValueError`` if fewer than one beam is requested, several beams are
        requested over a curtain of zero length, an axis is not three values or has zero
        length, the minimum range is negative, or the maximum range is below the
        minimum range.
        """
        requested = {
            "num_rays": num_rays,
            "curtain_length": curtain_length,
            "forward_axis": forward_axis,
            "curtain_axis": curtain_axis,
            "min_range": min_range,
            "max_range": max_range,
        }
        # Cast and check every value before writing any of them, so a request that is
        # bad in one field does not leave the sensor half reconfigured.
        writes = {
            field: self._config_value(field, value)
            for field, value in requested.items()
            if value is not None
        }
        # Then validate against the configuration the sensor will end up with rather
        # than only against the fields given, so a partial write cannot leave it in a
        # combination it cannot scan with.
        resulting = {
            field: writes.get(field, self._get_config(field)) for field in _CONFIG_ATTRS
        }

        if resulting["num_rays"] < 1:
            raise ValueError("a lightbeam sensor needs at least one beam")
        if resulting["num_rays"] > 1 and resulting["curtain_length"] <= 0.0:
            raise ValueError(
                "a curtain of several beams needs a positive curtain_length, or every beam "
                "is cast from the same place"
            )
        if resulting["min_range"] < 0.0:
            raise ValueError("min_range cannot be negative")
        if resulting["max_range"] < resulting["min_range"]:
            raise ValueError("max_range cannot be below min_range")

        for field, value in writes.items():
            self._prim.GetAttribute(_CONFIG_ATTRS[field]).Set(value)

    def info(self):
        """Static description of this sensor: prim path, beam layout, range, enabled.

        ``num_rays`` is the beam count authored on the prim rather than the one the
        running sensor has, so a description taken straight after :meth:`configure`
        reports what was just set -- the sensor picks that up on the next physics step,
        and :meth:`read` reports the count its own reading was taken with.
        """
        return {
            "prim_path": self.prim_path,
            "num_rays": self._get_config("num_rays"),
            "curtain_length": self._get_config("curtain_length"),
            "forward_axis": self._get_config("forward_axis"),
            "curtain_axis": self._get_config("curtain_axis"),
            "min_range": self._get_config("min_range"),
            "max_range": self._get_config("max_range"),
            "enabled": not self.is_paused(),
        }

    # -- lifecycle / collection control -----------------------------------------

    def destroy(self):
        """Mark this wrapper uninitialized.

        The legacy interface holds no per-instance resource: the PhysX sensor lives and
        dies with its USD prim, which the bridge leaves in the stage on delete, exactly
        like a lidar's.
        """
        self._initialized = False

    def pause(self):
        """Stop PhysX computing this sensor (clears the prim's ``enabled`` attribute)."""
        self._prim.GetAttribute(_ENABLED_ATTR).Set(False)

    def resume(self):
        """Resume sensor computation (sets the prim's ``enabled`` attribute)."""
        self._prim.GetAttribute(_ENABLED_ATTR).Set(True)

    def is_paused(self):
        """Whether sensor computation is currently switched off."""
        return not bool(self._prim.GetAttribute(_ENABLED_ATTR).Get())

    # -- readings ---------------------------------------------------------------

    def get_num_rays(self):
        """Number of beams in the curtain."""
        return int(self._sensor.get_num_rays(self.prim_path))

    # -- beam layout / range config (raw USD attributes -- see _CONFIG_ATTRS) ---

    def _get_config(self, field):
        """Read one config attribute off the prim, in the type its schema declares it."""
        value = self._prim.GetAttribute(_CONFIG_ATTRS[field]).Get()
        if field in _VECTOR_CONFIG_FIELDS:
            return None if value is None else [float(component) for component in value]
        if field == "num_rays":
            return 1 if value is None else int(value)
        return 0.0 if value is None else float(value)

    @staticmethod
    def _config_value(field, value):
        """Return one config value in the type its schema declares, or raise ``ValueError``.

        An axis is checked here rather than left to USD, so a wrong-sized or zero
        vector is a clean 400 instead of a deep pxr error surfacing as a 500.
        """
        if field not in _VECTOR_CONFIG_FIELDS:
            return int(value) if field == "num_rays" else float(value)

        components = [float(component) for component in value]
        if len(components) != 3:
            raise ValueError(f"{field} must have 3 values, got {len(components)}")
        vector = Gf.Vec3f(*components)
        if vector.GetLength() == 0.0:
            raise ValueError(f"{field} cannot have zero length")
        return vector
