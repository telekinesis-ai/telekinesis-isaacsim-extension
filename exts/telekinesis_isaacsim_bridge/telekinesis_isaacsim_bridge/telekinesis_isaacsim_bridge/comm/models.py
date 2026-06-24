# SPDX-License-Identifier: Apache-2.0
"""Request bodies for the bridge HTTP API.

Reference:
-https://github.com/fastapi/full-stack-fastapi-template/blob/master/backend/app/models.py

Kept separate from the routers so the wire schema is one obvious place to look
as the API grows. Wire units are native Isaac: radians for joints, the gripper
``fraction`` is closed-ness (0.0 open .. 1.0 closed).
"""

from enum import Enum

from pydantic import BaseModel


class CreateArticulationRequest(BaseModel):
    """Body of PUT /articulations -- register (and bind) one articulation."""

    prim_path: str
    device_type: str  # "robot" | "gripper"
    urdf_path: str | None = None


class MoveJRequest(BaseModel):
    q: list[float]


class GripperMoveRequest(BaseModel):
    fraction: float


class Transformation(BaseModel):
    """A rigid transform: translation (meters) + rotation (XYZ Euler degrees).

    Generic on purpose -- used here as the gripper mount offset for attach_tool,
    but reusable wherever a request needs a pose delta. An all-zero transform (the
    default) is the identity.
    """

    translation: list[float] = [0.0, 0.0, 0.0]  # meters (x, y, z)
    rotation: list[float] = [0.0, 0.0, 0.0]      # XYZ Euler degrees


class AttachToolRequest(BaseModel):
    """Body of POST /articulations/{articulation_id}/robot/attach_tool.

    The path ``articulation_id`` is the arm. This names the *gripper* articulation
    to attach and the rigid-body links to join them at (e.g. UR ``wrist_3_link`` and the
    gripper's base link -- must be RigidBodyAPI links, not empty frames like
    ``tool0``). After attaching, arm and gripper share one articulation; each
    device keeps driving only its own joints. ``offset`` is an optional mount
    transform baked into the fixed joint (null/omitted => flush attach).
    """

    gripper_articulation_id: str
    arm_mount_link: str
    gripper_mount_link: str
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
