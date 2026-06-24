# SPDX-License-Identifier: Apache-2.0
"""Request bodies for the bridge HTTP API.

Reference:
-https://github.com/fastapi/full-stack-fastapi-template/blob/master/backend/app/models.py

Kept separate from the routers so the wire schema is one obvious place to look
as the API grows. 
"""

from enum import Enum

from pydantic import BaseModel

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
