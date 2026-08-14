# SPDX-License-Identifier: Apache-2.0
"""
Surface (suction) gripper device: binds one ``IsaacSurfaceGripper`` prim and
actuates it through the ``isaacsim.robot.surface_gripper`` runtime interface.

This is the suction counterpart to :mod:`.articulation`. A suction gripper has no
joints to drive: it is a typed ``IsaacSurfaceGripper`` prim plus a set of D6
"attachment point" joints, and it is actuated by a close/open command that makes
PhysX weld whatever the attachment points can reach. So it is a device of its own
rather than an articulation with a driver joint.

The asset is expected to be a prepared USD gripper -- a suction gripper cannot be
imported from a URDF, since neither the gripper prim nor its attachment points
have a URDF equivalent. Registration therefore takes only a prim path.

What a bound gripper needs in the stage:

* one prim of type ``IsaacSurfaceGripper`` at, or below, the registered prim path;
* at least one D6 joint listed in that prim's ``isaac:attachmentPoints``
  relation, each with a valid ``physics:body0`` and ``physics:body1``.

``physics:body1`` is the *parked* body: the gripper component overwrites it with
the gripped object on close and leaves it in place on release, but PhysX only
creates the joint at all if it is a valid rigid body to begin with. A gripper
whose attachment points are unparked never grips and reports no error beyond a
warning in the log; :func:`..core.robot_assembler.attach_surface_gripper` re-parks
them onto the arm's mount link as part of assembly.

Units on the wire mirror the rest of the bridge: meters for lengths, degrees for
angular limits (the native unit of ``UsdPhysics`` angular limits), newtons for
force limits, seconds for the retry interval.
"""

import asyncio

import omni.kit.app
import omni.timeline
import omni.usd
import carb
from pxr import Gf, Sdf, Usd, UsdPhysics
from scipy.spatial import transform
from usd.schema.isaac import robot_schema

# The gripper component transitions state on a physics step, so a close/open is
# not observable in the frame it was issued: a blocking actuation pumps frames
# until the status settles. Closing -> Closed needs EVERY attachment point to
# grip; a gripper that catches an object with only some of them stays Closing
# forever, which is a real partial grip rather than a failure. So a close also
# settles once the gripped-object set has stopped changing. The frame cap is a
# ~30 s backstop at 60 fps, matching the articulation's motion cap.
_SETTLED_FRAMES = 5  # consecutive frames with an unchanged grip => settled
_ACTUATION_MAX_FRAMES = 1800
_BIND_RETRIES = 60

_SURFACE_GRIPPER_TYPE = robot_schema.Classes.SURFACE_GRIPPER.value
_ATTACHMENT_POINTS = robot_schema.Relations.ATTACHMENT_POINTS.name

# isaac:gripDistance is written by the gripper component rather than authored by
# the schema's CreateSurfaceGripper, and the Python robot schema does not name it,
# so it is named here.
_GRIP_DISTANCE = "isaac:gripDistance"

# Asset-authoring convention: some grippers group their attachment-point joints
# under a prim with this name. The isaac:attachmentPoints relation is what the
# gripper component actually reads, so this is only consulted when that relation
# is empty (see _resolve_attachment_points).
_ATTACHMENT_POINTS_SCOPE = "suction_joints"

# Wire axis name -> the axis token UsdPhysics uses in its limit/drive namespaces
# (``limit:rotX:physics:low``, ``drive:transZ:physics:stiffness``).
_AXES = {"x": "X", "y": "Y", "z": "Z"}

# The one D6 degree of freedom a suction cup is driven along: the cup is pulled
# in against the gripped surface along its forward axis.
_SUCTION_DRIVE = "transZ"

# The bridge's "XYZ Euler degrees" (see models.Transformation) means three
# rotations about the FIXED x, y then z axes -- scipy's lowercase, extrinsic
# "xyz". Isaac's own euler helpers and Gf.Rotation.Decompose use other
# conventions, so a joint's local rotation would silently fail to round-trip
# through either of them.
_EULER_SEQUENCE = "xyz"


def find_surface_gripper_prim(stage, root_path):
    """Path of the ``IsaacSurfaceGripper`` prim at or below ``root_path``.

    Raises ``RuntimeError`` if the subtree holds no surface gripper, or more than
    one -- an asset with several grippers has to be registered one gripper prim at
    a time, since a single id can only actuate one of them.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise RuntimeError(f"prim {root_path!r} not found in the open stage")

    found = [
        prim.GetPath().pathString
        for prim in Usd.PrimRange(root)
        if prim.GetTypeName() == _SURFACE_GRIPPER_TYPE
    ]
    if not found:
        raise RuntimeError(
            f"no prim of type {_SURFACE_GRIPPER_TYPE} at or below {root_path!r}; "
            "a suction gripper asset must carry one"
        )
    if len(found) > 1:
        raise RuntimeError(
            f"{len(found)} {_SURFACE_GRIPPER_TYPE} prims at or below {root_path!r} ({found}); "
            "register the gripper prim itself to pick one"
        )
    return found[0]


def _axis_limits(prim, kind):
    """``{axis: {minimum, maximum}}`` for the ``rot``/``trans`` limits on a D6 joint.

    Only axes the joint actually limits are reported; the D6 axes it leaves free
    carry no limit attributes and are omitted.
    """
    limits = {}
    for wire_axis, axis in _AXES.items():
        low = prim.GetAttribute(f"limit:{kind}{axis}:physics:low")
        high = prim.GetAttribute(f"limit:{kind}{axis}:physics:high")
        if not low and not high:
            continue
        limits[wire_axis] = {
            "minimum": low.Get() if low else None,
            "maximum": high.Get() if high else None,
        }
    return limits


def _apply_axis_limits(prim, kind, limits):
    """Write ``rot``/``trans`` limits onto a D6 joint, one axis at a time.

    ``limits`` is a :class:`..comm.models.AxisLimits`; an axis left ``None`` on it
    is not touched. The matching ``PhysicsLimitAPI`` is applied first, since a limit
    value PhysX has not been told to read has no effect.
    """
    for wire_axis, axis in _AXES.items():
        bounds = getattr(limits, wire_axis)
        if bounds is None:
            continue
        limit = UsdPhysics.LimitAPI.Apply(prim, f"{kind}{axis}")
        limit.CreateLowAttr().Set(float(bounds.minimum))
        limit.CreateHighAttr().Set(float(bounds.maximum))


def _quaternion_to_rotation(quaternion):
    """XYZ Euler degrees for a USD quaternion (``physics:localRot*``)."""
    imaginary = quaternion.GetImaginary()
    rotation = transform.Rotation.from_quat([*imaginary, quaternion.GetReal()])
    return [float(angle) for angle in rotation.as_euler(_EULER_SEQUENCE, degrees=True)]


def _rotation_to_quaternion(rotation_deg):
    """USD quaternion (``Gf.Quatf``) for XYZ Euler degrees."""
    rotation = transform.Rotation.from_euler(_EULER_SEQUENCE, rotation_deg, degrees=True)
    x, y, z, w = rotation.as_quat()
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


def _float_attribute(prim, name):
    """Value of the float attribute ``name`` on ``prim``, or ``None`` if unauthored."""
    attribute = prim.GetAttribute(name)
    return attribute.Get() if attribute else None


def _set_attribute(prim, attribute, value):
    """Set a ``robot_schema.Attributes`` value on ``prim``, creating it if absent.

    ``CreateSurfaceGripper`` authors most of them, but a gripper assembled by hand
    in the UI can be missing one, and the gripper's forward axis and an attachment
    point's clearance offset are optional in the schema.
    """
    existing = prim.GetAttribute(attribute.name)
    if not existing:
        existing = prim.CreateAttribute(attribute.name, attribute.type, False)
    existing.Set(value)


class SurfaceGripper:
    """Binds the ``IsaacSurfaceGripper`` prim at ``prim_path`` and actuates it.

    ``prim_path`` may be the gripper prim itself or any ancestor of it (typically
    the gripper asset's root), which is what makes the same path usable both here
    and as the prim to attach to an arm.
    """

    def __init__(self, prim_path):
        """Store the prim path; call :meth:`bind` before anything else."""
        self.prim_path = prim_path
        self.gripper_prim_path = None
        self.attachment_point_paths = []
        self._interface = None
        # Serializes close/open on this one gripper: a blocking actuation spans
        # many frames, and a second command issued mid-flight would leave the
        # first one's settle loop watching for a status its command no longer aims at.
        self._action_lock = asyncio.Lock()

    async def bind(self):
        """(Re)resolve the gripper prim, its attachment points and the runtime interface.

        Safe to call repeatedly, and required again after the timeline is stopped
        and replayed or after the gripper is assembled onto an arm: both rebuild
        the gripper component from USD.

        Unlike an articulation this does not start the timeline. A surface
        gripper's properties and attachment points are editable while the
        simulation is stopped; only actuation and live status need it playing.
        """
        from isaacsim.robot.surface_gripper import _surface_gripper

        app = omni.kit.app.get_app()
        last_error = None
        for _ in range(_BIND_RETRIES):
            try:
                stage = self._stage()
                self.gripper_prim_path = find_surface_gripper_prim(stage, self.prim_path)
                self.attachment_point_paths = self._resolve_attachment_points(stage)
                if self.attachment_point_paths:
                    self._interface = _surface_gripper.acquire_surface_gripper_interface()
                    # Mirror the runtime status and gripped objects into USD. Worth
                    # doing for its own sake (isaac:status becomes readable while
                    # stopped), but also necessary: the gripper component re-reads
                    # isaac:status from USD whenever any of its properties change,
                    # so a stale USD status would silently release a live grip the
                    # first time a property is written.
                    self._interface.set_write_to_usd(True)
                    carb.log_info(
                        f"[bridge] bound surface gripper {self.gripper_prim_path}: "
                        f"{len(self.attachment_point_paths)} attachment point(s)"
                    )
                    return
            except RuntimeError as exc:
                last_error = exc
            await app.next_update_async()

        detail = f"surface gripper at {self.prim_path} did not become valid"
        if last_error is not None:
            detail += f" (last error: {last_error})"
        else:
            detail += (
                f"; {self.gripper_prim_path or self.prim_path} lists no attachment points in "
                f"its {_ATTACHMENT_POINTS} relation, so it cannot grip anything"
            )
        raise RuntimeError(detail)

    def info(self):
        """Static description of this gripper: its prims, properties and current state."""
        return {
            "gripper_prim_path": self.gripper_prim_path,
            "attachment_point_paths": list(self.attachment_point_paths),
            "properties": self.get_properties(),
            **self.get_status(),
        }

    # -- actuation --------------------------------------------------------------

    async def close(self, asynchronous=False):
        """Close the gripper: grip whatever its attachment points can reach.

        ``asynchronous=True`` issues the command and returns immediately with the
        status as it stands in that frame -- which is still the old one, since the
        gripper component acts on the next physics step. ``asynchronous=False``
        (the default) issues the command and then waits.

        A blocking close is done once the gripper has stopped trying, and reports
        what it ended up with rather than treating anything as an error. Three
        outcomes, told apart by ``status`` and ``gripped_objects``:

        * ``Closed`` -- every attachment point gripped something.
        * ``Closing`` with a non-empty ``gripped_objects`` -- a partial grip: some
          attachment points hold an object and the rest keep retrying. A caller
          may well accept this.
        * ``Open`` with an empty ``gripped_objects`` -- nothing was in reach and
          the gripper gave up. How long it keeps trying first is the
          ``retry_interval`` property.

        ``timed_out`` is true if none of the three settled within roughly 30
        seconds of simulation.

        Requires the timeline to be playing: with the simulation stopped there is
        no gripper component to act on the command.
        """
        return await self._actuate(closing=True, asynchronous=asynchronous)

    async def open(self, asynchronous=False):
        """Release everything the gripper holds.

        The blocking form waits until the gripper reports ``Open`` and holds
        nothing. Otherwise identical to :meth:`close`.
        """
        return await self._actuate(closing=False, asynchronous=asynchronous)

    def get_status(self):
        """Current ``status``, ``gripped_objects`` and ``grip_distance``.

        ``status`` is one of ``Open`` / ``Closing`` / ``Closed``, read from the
        running gripper component while the timeline is playing and from the USD
        ``isaac:status`` attribute otherwise. ``simulated`` reports which of the
        two it came from: a status read while the simulation is stopped is the
        last one the component wrote, not a live reading.

        ``gripped_objects`` are the prim paths of the bodies currently held (empty
        while stopped). ``grip_distance`` is the measured distance to the gripped
        surface in meters, or ``None`` while the gripper holds nothing.
        """
        simulated = omni.timeline.get_timeline_interface().is_playing()
        if simulated:
            status = self._interface.get_gripper_status(self.gripper_prim_path).name
            gripped_objects = list(self._interface.get_gripped_objects(self.gripper_prim_path))
        else:
            authored = self._gripper_prim().GetAttribute(robot_schema.Attributes.STATUS.name)
            status = authored.Get() if authored else None
            gripped_objects = []
        return {
            "status": status,
            "gripped_objects": gripped_objects,
            "grip_distance": _float_attribute(self._gripper_prim(), _GRIP_DISTANCE),
            "simulated": simulated,
        }

    # -- gripper properties -----------------------------------------------------

    def get_properties(self):
        """The gripper's grip-behaviour properties, as ``set_properties`` accepts them.

        ``coaxial_force_limit`` and ``shear_force_limit`` are the newton loads that
        break a grip along and across the forward axis; ``max_grip_distance`` is
        how far (meters) an attachment point reaches for a surface;
        ``retry_interval`` is how long (seconds) a close keeps trying before it
        gives up on the attachment points that found nothing; ``forward_axis`` is
        the axis the gripper grips along. A property the asset leaves unauthored
        reads back as ``None``.

        ``rotation_limits`` and ``translation_limits`` are per-attachment-point
        rather than gripper-wide, so they are reported by
        :meth:`get_attachment_points` and not here.
        """
        prim = self._gripper_prim()
        forward_axis = prim.GetAttribute(robot_schema.Attributes.FORWARD_AXIS.name)
        return {
            "coaxial_force_limit": _float_attribute(
                prim, robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name
            ),
            "shear_force_limit": _float_attribute(
                prim, robot_schema.Attributes.SHEAR_FORCE_LIMIT.name
            ),
            "max_grip_distance": _float_attribute(
                prim, robot_schema.Attributes.MAX_GRIP_DISTANCE.name
            ),
            "retry_interval": _float_attribute(prim, robot_schema.Attributes.RETRY_INTERVAL.name),
            "forward_axis": forward_axis.Get() if forward_axis else None,
        }

    def set_properties(
        self,
        coaxial_force_limit=None,
        shear_force_limit=None,
        max_grip_distance=None,
        retry_interval=None,
        forward_axis=None,
        rotation_limits=None,
        translation_limits=None,
    ):
        """Set the gripper's grip-behaviour properties; returns the resulting values.

        Every argument is optional and one left ``None`` is not touched. Units are
        newtons for the force limits, meters for the grip distance, seconds for the
        retry interval. ``forward_axis`` is ``"X"``, ``"Y"`` or ``"Z"``.

        ``rotation_limits`` (degrees) and ``translation_limits`` (meters) are how
        far an attachment point may swivel and slide once it has gripped. USD
        stores them on each attachment-point joint rather than on the gripper, so
        passing them here writes the same limits to every attachment point; use
        :meth:`set_attachment_point_properties` to give individual points
        different limits.

        The running gripper picks these up on the next step -- no rebind or
        restart -- and they can equally be set while the simulation is stopped.
        """
        prim = self._gripper_prim()
        if coaxial_force_limit is not None:
            _set_attribute(
                prim, robot_schema.Attributes.COAXIAL_FORCE_LIMIT, float(coaxial_force_limit)
            )
        if shear_force_limit is not None:
            _set_attribute(
                prim, robot_schema.Attributes.SHEAR_FORCE_LIMIT, float(shear_force_limit)
            )
        if max_grip_distance is not None:
            _set_attribute(
                prim, robot_schema.Attributes.MAX_GRIP_DISTANCE, float(max_grip_distance)
            )
        if retry_interval is not None:
            _set_attribute(prim, robot_schema.Attributes.RETRY_INTERVAL, float(retry_interval))
        if forward_axis is not None:
            _set_attribute(
                prim, robot_schema.Attributes.FORWARD_AXIS, self._validate_axis(forward_axis)
            )
        if rotation_limits is not None or translation_limits is not None:
            self.set_attachment_point_properties(
                rotation_limits=rotation_limits, translation_limits=translation_limits
            )
        return self.get_properties()

    # -- attachment points (the D6 joints) --------------------------------------

    def get_attachment_points(self):
        """Per-attachment-point properties, one entry per D6 joint, in relation order.

        Each entry carries the joint's ``prim_path``, the two bodies it joins
        (``body_0`` / ``body_1``) and their local frames on it (``local_pose_0`` /
        ``local_pose_1``, translation in meters and XYZ Euler degrees), the suction
        drive's ``z_axis_translation_drive_stiffness`` and
        ``z_axis_translation_drive_damping``, its ``rotation_limits`` (degrees) and
        ``translation_limits`` (meters), and the ``clearance_offset`` (meters) and
        ``forward_axis`` that aim the grip along.

        ``body_1`` is the body the joint is parked against while the gripper is
        open; the gripper replaces it with the gripped object on close, so a
        reading taken during a grip reports that object instead.
        """
        return [self._attachment_point_info(path) for path in self.attachment_point_paths]

    def set_attachment_point_properties(
        self,
        joint_paths=None,
        local_pose_0=None,
        local_pose_1=None,
        z_axis_translation_drive_stiffness=None,
        z_axis_translation_drive_damping=None,
        rotation_limits=None,
        translation_limits=None,
        clearance_offset=None,
        forward_axis=None,
    ):
        """Set properties on the gripper's attachment points; returns their new state.

        ``joint_paths`` selects which attachment points to write, defaulting to all
        of them. Every other argument is optional and one left ``None`` is not
        touched; within a pose, a null translation or orientation is not touched
        either.

        The drive stiffness and damping govern how firmly a gripped object is
        pulled in along the forward axis, the limits how far it may then swivel
        (degrees) and slide (meters), and the clearance offset (meters) how far
        ahead of the cup the search for a surface begins.

        The local poses are the joint's frames on its two bodies. They normally
        come from assembly rather than by hand: attaching a gripper to an arm
        re-parks every attachment point and recomputes ``local_pose_1`` to match.

        ``clearance_offset`` and ``forward_axis`` are read when the gripper
        component starts, so a change to either applies from the next Stop+Play
        rather than immediately.
        """
        for path in self._select_attachment_points(joint_paths):
            prim = self._joint_prim(path)
            if local_pose_0 is not None:
                self._set_local_pose(prim, 0, local_pose_0)
            if local_pose_1 is not None:
                self._set_local_pose(prim, 1, local_pose_1)
            if (
                z_axis_translation_drive_stiffness is not None
                or z_axis_translation_drive_damping is not None
            ):
                drive = UsdPhysics.DriveAPI.Apply(prim, _SUCTION_DRIVE)
                if z_axis_translation_drive_stiffness is not None:
                    drive.CreateStiffnessAttr().Set(float(z_axis_translation_drive_stiffness))
                if z_axis_translation_drive_damping is not None:
                    drive.CreateDampingAttr().Set(float(z_axis_translation_drive_damping))
            if rotation_limits is not None:
                _apply_axis_limits(prim, "rot", rotation_limits)
            if translation_limits is not None:
                _apply_axis_limits(prim, "trans", translation_limits)
            if clearance_offset is not None:
                _set_attribute(
                    prim, robot_schema.Attributes.CLEARANCE_OFFSET, float(clearance_offset)
                )
            if forward_axis is not None:
                _set_attribute(
                    prim, robot_schema.Attributes.FORWARD_AXIS, self._validate_axis(forward_axis)
                )
        return self.get_attachment_points()

    # -- assembly support -------------------------------------------------------

    def mount_body_path(self, mount_prim_path=None):
        """Path of the rigid body a fixed joint can attach this gripper by.

        ``mount_prim_path`` defaults to the registered prim. It must itself carry
        ``UsdPhysics.RigidBodyAPI``: mounting the gripper by a mesh or a frame
        somewhere inside it would join the arm to a part of the gripper rather
        than to the gripper, so a prim without the API is rejected instead of
        searched below.
        """
        path = mount_prim_path or self.prim_path
        prim = self._stage().GetPrimAtPath(path)
        if not prim.IsValid():
            raise ValueError(f"gripper mount prim {path!r} not found in the open stage")
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(
                f"gripper mount prim {path!r} is not a rigid body (no UsdPhysics.RigidBodyAPI); "
                "a fixed joint needs one, so give the gripper's mount body explicitly"
            )
        return path

    # -- internals --------------------------------------------------------------

    async def _actuate(self, closing, asynchronous):
        """Issue a close/open and, unless ``asynchronous``, wait for it to settle."""
        if not omni.timeline.get_timeline_interface().is_playing():
            raise RuntimeError(
                f"surface gripper {self.gripper_prim_path} cannot be actuated while the "
                "simulation is stopped; start the timeline first"
            )

        app = omni.kit.app.get_app()
        async with self._action_lock:
            command = self._interface.close_gripper if closing else self._interface.open_gripper
            if not command(self.gripper_prim_path):
                raise RuntimeError(
                    f"the simulation did not accept the command for surface gripper "
                    f"{self.gripper_prim_path}; it is not registered with the running physics "
                    "scene, so re-create it (PUT) after the last Stop+Play"
                )

            if asynchronous:
                # Fire-and-forget: the command is queued for the next physics step,
                # so the status reported here is still the previous one. The caller
                # owns "done" and polls the status route.
                return {"done": False, "timed_out": False, **self.get_status()}

            settled_frames = 0
            previous_grip = None
            for _ in range(_ACTUATION_MAX_FRAMES):
                await app.next_update_async()
                state = self.get_status()
                if not closing:
                    if state["status"] == "Open" and not state["gripped_objects"]:
                        return {"done": True, "timed_out": False, **state}
                    continue

                # Closed: gripped on every point. Open: found nothing in reach and
                # gave up (the component drops back to Open once the retry
                # interval runs out). Either way the close is over.
                if state["status"] in ("Closed", "Open"):
                    return {"done": True, "timed_out": False, **state}
                if state["gripped_objects"]:
                    settled_frames = (
                        settled_frames + 1 if state["gripped_objects"] == previous_grip else 0
                    )
                    if settled_frames >= _SETTLED_FRAMES:
                        return {"done": True, "timed_out": False, **state}
                else:
                    settled_frames = 0
                previous_grip = state["gripped_objects"]

        return {"done": True, "timed_out": True, **self.get_status()}

    def _resolve_attachment_points(self, stage):
        """Paths of the D6 joints the gripper grips with, in the order it reads them.

        ``isaac:attachmentPoints`` is authoritative -- it is the only list the
        gripper component looks at. A gripper that lists none cannot grip, so
        rather than bind a gripper that will silently do nothing, fall back to the
        joints grouped under a ``suction_joints`` prim (a common authoring layout)
        and write them into the relation, saying so in the log.
        """
        prim = stage.GetPrimAtPath(self.gripper_prim_path)
        relation = prim.GetRelationship(_ATTACHMENT_POINTS)
        targets = list(relation.GetTargets()) if relation else []
        if targets:
            return [path.pathString for path in targets]

        grouped = [
            joint.GetPath()
            for scope in Usd.PrimRange(stage.GetPrimAtPath(self.prim_path))
            if scope.GetName() == _ATTACHMENT_POINTS_SCOPE
            for joint in Usd.PrimRange(scope)
            if joint.IsA(UsdPhysics.Joint)
        ]
        if not grouped:
            return []

        carb.log_warn(
            f"[bridge] surface gripper {self.gripper_prim_path} lists no attachment points in "
            f"{_ATTACHMENT_POINTS}; adopting the {len(grouped)} joint(s) found under "
            f"{_ATTACHMENT_POINTS_SCOPE!r} and writing them into the relation. Author the "
            "relation in the asset to control which joints grip."
        )
        if not relation:
            relation = prim.CreateRelationship(_ATTACHMENT_POINTS, False)
        relation.SetTargets(grouped)
        return [path.pathString for path in grouped]

    def _select_attachment_points(self, joint_paths):
        """Validate ``joint_paths`` against this gripper's attachment points.

        ``None`` selects them all. A path that is not one of them is rejected
        rather than written to: a joint the gripper does not grip with would take
        the properties and have no effect on anything.
        """
        if joint_paths is None:
            return list(self.attachment_point_paths)
        unknown = [path for path in joint_paths if path not in self.attachment_point_paths]
        if unknown:
            raise ValueError(
                f"{unknown} are not attachment points of surface gripper "
                f"{self.gripper_prim_path}; it grips with {self.attachment_point_paths}"
            )
        return list(joint_paths)

    def _attachment_point_info(self, path):
        """Every reported property of one attachment-point joint."""
        prim = self._joint_prim(path)
        joint = UsdPhysics.Joint(prim)
        forward_axis = prim.GetAttribute(robot_schema.Attributes.FORWARD_AXIS.name)
        return {
            "prim_path": path,
            "body_0": [target.pathString for target in joint.GetBody0Rel().GetTargets()],
            "body_1": [target.pathString for target in joint.GetBody1Rel().GetTargets()],
            "local_pose_0": self._local_pose(prim, 0),
            "local_pose_1": self._local_pose(prim, 1),
            "z_axis_translation_drive_stiffness": _float_attribute(
                prim, f"drive:{_SUCTION_DRIVE}:physics:stiffness"
            ),
            "z_axis_translation_drive_damping": _float_attribute(
                prim, f"drive:{_SUCTION_DRIVE}:physics:damping"
            ),
            "rotation_limits": _axis_limits(prim, "rot"),
            "translation_limits": _axis_limits(prim, "trans"),
            "clearance_offset": _float_attribute(
                prim, robot_schema.Attributes.CLEARANCE_OFFSET.name
            ),
            "forward_axis": forward_axis.Get() if forward_axis else None,
        }

    @staticmethod
    def _local_pose(prim, body):
        """One of a joint's two local frames as ``{translation, rotation}``.

        USD stores the frame's rotation as a quaternion; it is reported as XYZ Euler
        degrees, the rotation form the rest of the bridge uses.
        """
        position = prim.GetAttribute(f"physics:localPos{body}")
        rotation = prim.GetAttribute(f"physics:localRot{body}")
        quaternion = rotation.Get() if rotation else None
        return {
            "translation": list(position.Get()) if position and position.Get() else None,
            "rotation": _quaternion_to_rotation(quaternion) if quaternion is not None else None,
        }

    @staticmethod
    def _set_local_pose(prim, body, pose):
        """Write one of a joint's two local frames from a ``models.JointLocalPose``."""
        joint = UsdPhysics.Joint(prim)
        if pose.translation is not None:
            if len(pose.translation) != 3:
                raise ValueError(
                    f"expected an [x, y, z] translation, got {len(pose.translation)} value(s)"
                )
            attribute = joint.CreateLocalPos0Attr() if body == 0 else joint.CreateLocalPos1Attr()
            attribute.Set(Gf.Vec3f(*(float(value) for value in pose.translation)))
        if pose.rotation is not None:
            if len(pose.rotation) != 3:
                raise ValueError(
                    f"expected [rx, ry, rz] Euler degrees, got {len(pose.rotation)} value(s)"
                )
            attribute = joint.CreateLocalRot0Attr() if body == 0 else joint.CreateLocalRot1Attr()
            attribute.Set(_rotation_to_quaternion(pose.rotation))

    @staticmethod
    def _validate_axis(axis):
        """Normalize a forward-axis token, rejecting anything but X / Y / Z."""
        token = str(axis).upper()
        if token not in ("X", "Y", "Z"):
            raise ValueError(f"forward_axis must be one of 'X', 'Y', 'Z', got {axis!r}")
        return token

    def _stage(self):
        """The open USD stage, or ``RuntimeError`` if there is none."""
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("no USD stage is open")
        return stage

    def _gripper_prim(self):
        """The bound ``IsaacSurfaceGripper`` prim."""
        prim = self._stage().GetPrimAtPath(self.gripper_prim_path)
        if not prim.IsValid():
            raise RuntimeError(
                f"surface gripper prim {self.gripper_prim_path} is gone from the stage; "
                "re-create the gripper (PUT) against the current stage"
            )
        return prim

    def _joint_prim(self, path):
        """One attachment-point joint prim, or ``ValueError`` if it left the stage."""
        prim = self._stage().GetPrimAtPath(Sdf.Path(path))
        if not prim.IsValid() or not prim.IsA(UsdPhysics.Joint):
            raise ValueError(f"attachment point {path!r} is not a physics joint in the open stage")
        return prim
