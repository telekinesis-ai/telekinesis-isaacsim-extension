# SPDX-License-Identifier: Apache-2.0
"""Request bodies for the bridge HTTP API.

Reference:
-https://github.com/fastapi/full-stack-fastapi-template/blob/master/backend/app/models.py

Kept separate from the routers so the wire schema is one obvious place to look
as the API grows. Wire units are native Isaac: radians for joints. The bridge is
device-agnostic -- robot/gripper semantics (e.g. a gripper ``fraction``) live in
the client, which maps them to joint angles before calling these routes.
"""

from enum import Enum

from pydantic import BaseModel, Field


class CreateArticulationRequest(BaseModel):
    """Body of PUT /articulations -- register (and bind) one articulation."""

    prim_path: str = Field(min_length=1)
    urdf_path: str | None = None


class JointPositionsRequest(BaseModel):
    """Body of POST /articulations/{id}/move_j -- drive joints (radians).

    ``joint_positions`` are the target angles for ``indices`` (default: the
    device's current driven joints, so a robot sends all of them and a gripper
    narrowed to its driver sends one). ``asynchronous`` true applies the action and
    returns immediately (the client decides when the move is done); false blocks
    until the joints reach the target or stall.
    """

    joint_positions: list[float]
    indices: list[int] | None = None
    asynchronous: bool = False


class SetJointPositionsRequest(BaseModel):
    """Body of POST /articulations/{id}/set_j -- teleport joints (radians).

    ``joint_positions`` are placed directly onto ``indices`` (default: the
    device's current driven joints), so the joints jump to them in a single step
    rather than being driven there over time. There is no ``asynchronous`` flag: a
    teleport is immediate.
    """

    joint_positions: list[float]
    indices: list[int] | None = None


class JointVelocitiesRequest(BaseModel):
    """Body of POST /articulations/{id}/joint_velocities -- drive joints (rad/s).

    The velocity counterpart to :class:`JointPositionsRequest`, for driving joints
    at a target rate rather than to a pose (drive wheels, a spinning tool, a
    conveyor, or slewing any joint at a controlled speed). ``joint_velocities`` are
    the target angular speeds for ``indices`` (default: the device's current
    driven joints). Fire-and-forget: the commanded speeds hold until the next
    call, so ``{0, ...}`` stops the joints (there is no ``asynchronous`` flag --
    a velocity-driven joint never "reaches" a target). Any higher-level kinematics
    (e.g. twist->wheel) live in the client.
    """

    joint_velocities: list[float]
    indices: list[int] | None = None


class SetDrivenJointsRequest(BaseModel):
    """Body of PUT /articulations/{id}/driven_joints -- narrow the driven joints.

    ``joint_names`` are the joints the device should drive from now on (e.g. a
    gripper client passes ``[driver_name]`` discovered via GET /driver_joint).
    """

    joint_names: list[str]


class JointEffortsRequest(BaseModel):
    """Body of POST /articulations/{id}/joint_efforts -- direct torque/force drive.

    Bypasses the position/velocity drive; only takes effect if the target
    joint's drive stiffness and damping are zero. ``indices`` defaults to the
    device's current driven joints, matching :class:`JointVelocitiesRequest`.
    """

    joint_efforts: list[float]
    indices: list[int] | None = None


class DofGainsRequest(BaseModel):
    """Body of POST /articulations/{id}/dof_gains -- retune the position drive.

    ``stiffness`` and ``damping`` are the drive's proportional and derivative
    gains, ``max_effort`` the largest torque/force it may apply. Each may be a
    single value for every addressed joint or one value per joint in ``indices``
    order; any field left ``None`` is not touched. ``indices`` defaults to the
    device's current driven joints, matching :class:`JointEffortsRequest`.

    Used to correct gains a robot was imported with -- a URDF joint declaring a
    zero effort limit yields a drive that cannot move it -- and to retune tracking
    without re-importing. Applies to the running simulation, not the stage.
    """

    stiffness: list[float] | float | None = None
    damping: list[float] | float | None = None
    max_effort: list[float] | float | None = None
    indices: list[int] | None = None


class DefaultJointStateRequest(BaseModel):
    """Body of PUT /articulations/{id}/joints_default_state -- the joint-space
    "home pose" applied on the next reset (Stop+Play). Any field left ``None``
    is not touched.
    """

    joint_positions: list[float] | None = None
    joint_velocities: list[float] | None = None
    joint_efforts: list[float] | None = None


class SetEnabledRequest(BaseModel):
    """Body of a boolean on/off toggle (gravity, self-collisions)."""

    enabled: bool


class SetVelocityRequest(BaseModel):
    """Body of PUT .../linear_velocity or .../angular_velocity -- a 3D vector
    (``[x, y, z]``) for the articulation's root link. Only meaningful for a
    floating-base articulation (mobile base, humanoid)."""

    velocity: list[float]


class SetWorldVelocityRequest(BaseModel):
    """Body of PUT .../world_velocity -- the root link's full 6-DOF velocity
    (``[vx, vy, vz, wx, wy, wz]``, linear then angular)."""

    velocity: list[float]


class SolverIterationCountRequest(BaseModel):
    """Body of PUT .../solver/position_iterations or .../velocity_iterations."""

    count: int


class SolverThresholdRequest(BaseModel):
    """Body of PUT .../solver/stabilization_threshold or .../solver/sleep_threshold."""

    threshold: float


class PrimPathRequest(BaseModel):
    """Body of PUT /prims/poses/default or POST /prims/poses/default/reset.

    A named model for consistency with every other body-carrying route (was
    previously a bare ``Body(..., embed=False)`` string on these two only).
    """

    prim_path: str


class Transformation(BaseModel):
    """A rigid transform: translation (meters) + rotation (XYZ Euler degrees).

    Generic on purpose -- used here as the gripper mount offset for assemble_robot,
    but reusable wherever a request needs a pose delta. An all-zero transform (the
    default) is the identity.
    """

    translation: list[float] = [0.0, 0.0, 0.0]  # meters (x, y, z)
    rotation: list[float] = [0.0, 0.0, 0.0]  # XYZ Euler degrees


class AssembleRobotRequest(BaseModel):
    """Body of POST /articulations/{articulation_id}/assemble_robot.

    The path ``articulation_id`` is the arm. This names the *gripper* articulation
    to assemble onto it and the links to join them at. ``arm_mount_link`` is the
    arm's flange (e.g. UR ``wrist_3_link``) -- a RigidBodyAPI link or a Site, not an
    empty frame like ``tool0``. ``gripper_mount_link`` is the gripper's base link;
    leave it null/omitted to auto-discover the gripper articulation's root link (its
    base), which is what the fixed joint must attach to. After assembling, arm and
    gripper share one articulation; each device keeps driving only its own joints.
    ``offset`` is an optional mount transform baked into the fixed joint
    (null/omitted => flush attach). Assembling the same pair again is a no-op (the
    bridge records that they are already assembled and just returns the merged info).
    """

    gripper_articulation_id: str
    arm_mount_link: str
    gripper_mount_link: str | None = None
    offset: Transformation | None = None


# -- Stage (mirrors the extension's Omniservice schemas) --------------------


class OpenSceneRequest(BaseModel):
    """Body of PUT /stage/scene (spec: ``UsdStageModel``)."""

    uri: str  # USD stage to open


class StageUnits(BaseModel):
    """Body/response of the stage-units routes."""

    meters_per_unit: float = 1.0


class TimelineAction(str, Enum):
    """Allowed values for PATCH /stage/simulation/timeline/{action}."""

    # Lowercase on purpose: these are str Enum members, so the member name IS the
    # wire value -- UPPER_CASE would mean the JSON value on the wire is uppercase too.
    play = "play"  # pylint: disable=invalid-name
    pause = "pause"  # pylint: disable=invalid-name
    stop = "stop"  # pylint: disable=invalid-name


# -- Prims (mirrors the extension's Omniservice schemas) --------------------


class WSPose(BaseModel):
    """A pose in world/local space. ``cartesian`` form is rotation-vector
    (``[x, y, z, rx, ry, rz]``, axis*angle in rad); ``quaternions`` form is
    ``[x, y, z, qw, qx, qy, qz]``. Length is not enforced so both fit one model.
    """

    pose: list[float]


class UpdatePoseRequest(BaseModel):
    """Body of PUT /prims/poses."""

    input_pose: WSPose
    prim_path: str


class ApplyRelativePoseRequest(BaseModel):
    """Body of POST /prims/poses/relative."""

    relative_pose: WSPose
    prim_path: str
    object_first: bool = False


class VisibilityAction(str, Enum):
    """Allowed values for PATCH /prims/visibility."""

    # Lowercase on purpose: these are str Enum members, so the member name IS the
    # wire value -- UPPER_CASE would mean the JSON value on the wire is uppercase too.
    show = "show"  # pylint: disable=invalid-name
    hide = "hide"  # pylint: disable=invalid-name


class SetVisibilityRequest(BaseModel):
    """Body of PATCH /prims/visibility."""

    prim_path: str
    visibility: VisibilityAction = VisibilityAction.show


class PrimMetadata(BaseModel):
    """Spec ``CustomPrimData`` -- user metadata stored on a prim."""

    category: str
    type: str


class SetPrimMetadataRequest(BaseModel):
    """Body of PUT /prims/metadata."""

    metadata: PrimMetadata
    prim_path: str


class SetJointStateRequest(BaseModel):
    """Body of PATCH /prims/physics/joints."""

    enable: bool
    prim_path: str


class UpdateCollidersRequest(BaseModel):
    """Body of PATCH /prims/physics/colliders/."""

    enable: bool
    prim_path: str


# -- Cameras (device registry, analogous to articulations) ------------------


class CreateCameraRequest(BaseModel):
    """Body of PUT /cameras -- register (and bind) one camera.

    ``resolution`` is ``[width, height]`` in pixels. ``data_types`` is the set of
    render outputs to produce (default ``["rgb"]``); each must be a supported
    camera data type. ``frequency`` (Hz) is optional; ``None`` follows the render
    loop.
    """

    prim_path: str = Field(min_length=1)
    resolution: list[int] = [1280, 720]
    data_types: list[str] | None = None
    frequency: float | None = None


class CaptureRequest(BaseModel):
    """Body of POST /cameras/{id}/capture -- pump one frame and read outputs.

    ``data_types`` selects which of the camera's bound outputs to return; ``None``
    returns them all.
    """

    data_types: list[str] | None = None


class CameraWorldPoseRequest(BaseModel):
    """Body of PUT /cameras/{id}/world_pose.

    ``position`` is ``[x, y, z]`` (stage units), ``orientation`` a scalar-first
    ``[w, x, y, z]`` quaternion; either may be null to leave it untouched.
    ``camera_axes`` is one of ``world``/``ros``/``usd``.
    """

    position: list[float] | None = None
    orientation: list[float] | None = None
    camera_axes: str = "world"


class CameraLocalPoseRequest(BaseModel):
    """Body of PUT /cameras/{id}/local_pose -- parent-relative pose.

    Like :class:`CameraWorldPoseRequest` but with ``translation`` in place of
    ``position``.
    """

    translation: list[float] | None = None
    orientation: list[float] | None = None
    camera_axes: str = "world"


class CameraResolutionRequest(BaseModel):
    """Body of PUT /cameras/{id}/resolution -- ``[width, height]`` in pixels."""

    width: int
    height: int


class CameraFloatValueRequest(BaseModel):
    """Body of the single-float camera setters (focal length, focus distance,
    lens aperture/fStop, frequency)."""

    value: float


class CameraApertureRequest(BaseModel):
    """Body of PUT .../horizontal_aperture or .../vertical_aperture (stage units).

    ``maintain_square_pixels`` keeps the paired aperture in sync so pixels stay
    square.
    """

    value: float
    maintain_square_pixels: bool = True


class CameraClippingRangeRequest(BaseModel):
    """Body of PUT /cameras/{id}/clipping_range (stage units). Either field null
    leaves that bound unchanged."""

    near_distance: float | None = None
    far_distance: float | None = None


class CameraStringValueRequest(BaseModel):
    """Body of the string-valued camera setters (projection mode, stereo role,
    lens distortion model)."""

    value: str
