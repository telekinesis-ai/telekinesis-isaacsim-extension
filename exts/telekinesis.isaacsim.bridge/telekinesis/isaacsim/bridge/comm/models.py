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

from pydantic import BaseModel


class CreateArticulationRequest(BaseModel):
    """Body of PUT /articulations -- register (and bind) one articulation."""

    prim_path: str
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


class Transformation(BaseModel):
    """A rigid transform: translation (meters) + rotation (XYZ Euler degrees).

    Generic on purpose -- used here as the gripper mount offset for assemble_robot,
    but reusable wherever a request needs a pose delta. An all-zero transform (the
    default) is the identity.
    """

    translation: list[float] = [0.0, 0.0, 0.0]  # meters (x, y, z)
    rotation: list[float] = [0.0, 0.0, 0.0]      # XYZ Euler degrees


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

    play = "play"
    pause = "pause"
    stop = "stop"


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
    show = "show"
    hide = "hide"


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
