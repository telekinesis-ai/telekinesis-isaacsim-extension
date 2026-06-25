# SPDX-License-Identifier: Apache-2.0
"""HTTP routers for the bridge, grouped by domain.

Refereces:
Prim and articulation:
- https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.assets.html#isaaclab.assets.Articulation
- https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sim.utils.html#module-isaaclab.sim.utils.prims

Three ``APIRouter``s -- articulations (create/get/delete/list), robot, and
gripper -- so each device kind owns a parallel namespace under
``/articulations/{articulation_id}/<kind>/...``. They are intentionally thin:
every handler just resolves the shared registry and forwards to it.

The registry is the ``BridgeServer`` instance itself, stashed on ``app.state`` by
``BridgeServer._build_app`` and pulled back here via the ``get_registry``
dependency. That keeps these routers as plain module-level objects (no closure
over ``self``, no circular import) while still sharing one device table.
"""

from fastapi import APIRouter, Body, Depends, HTTPException, Query, Request, status

from .models import (
    ApplyRelativePoseRequest,
    AttachToolRequest,
    CreateArticulationRequest,
    GripperMoveRequest,
    MoveJRequest,
    OpenSceneRequest,
    SetJointStateRequest,
    SetPrimMetadataRequest,
    SetVisibilityRequest,
    StageUnits,
    TimelineAction,
    UpdateCollidersRequest,
    UpdatePoseRequest,
    VisibilityAction,
)


def get_registry(request: Request):
    """Return the shared device registry attached to the app by BridgeServer."""
    return request.app.state.registry


def not_implemented():
    """Uniform 501 for routes we mirror from the spec but haven't wired up yet."""
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="Not implemented",
    )


# -- articulations (device-kind agnostic) -----------------------------------

articulations = APIRouter(tags=["articulations"])


@articulations.put("/articulations", summary="Create Articulation")
async def create_articulation(req: CreateArticulationRequest, reg=Depends(get_registry)):
    # Register (and bind) one articulation; returns its id, prim, dof and state.
    return await reg.create_articulation(req.prim_path, req.device_type, req.urdf_path)


@articulations.get("/articulations", summary="List Articulations")
async def list_articulations(reg=Depends(get_registry)):
    return reg.list_articulations()


@articulations.get("/articulations/{articulation_id}", summary="Get Articulation")
async def get_articulation(articulation_id: str, reg=Depends(get_registry)):
    # Id + static description (prim, dof, names, current state). 404 if unknown.
    return reg.get_articulation_info(articulation_id)


@articulations.delete("/articulations/{articulation_id}", summary="Delete Articulation")
async def delete_articulation(articulation_id: str, reg=Depends(get_registry)):
    # Unregister the articulation (the USD prim is left in the stage).
    return reg.delete_articulation(articulation_id)


# -- robot ------------------------------------------------------------------

robot = APIRouter(prefix="/articulations/{articulation_id}/robot", tags=["robot"])


@robot.post("/move_j")
async def move_j(articulation_id: str, req: MoveJRequest, reg=Depends(get_registry)):
    # Blocks until the move completes, then returns the final motion status.
    return await reg.get_device(articulation_id).move_j(req.q)


@robot.get("/state")
async def get_state(articulation_id: str, reg=Depends(get_registry)):
    return reg.get_device(articulation_id).get_state()


@robot.post("/attach_tool", summary="Attach Tool")
async def attach_tool(articulation_id: str, req: AttachToolRequest, reg=Depends(get_registry)):
    # Assemble a gripper onto this arm; both devices then share one articulation.
    return await reg.attach_tool(
        articulation_id, req.gripper_articulation_id, req.arm_mount_link, req.gripper_mount_link, req.offset
    )


# -- gripper ----------------------------------------------------------------

gripper = APIRouter(prefix="/articulations/{articulation_id}/gripper", tags=["gripper"])


@gripper.post("/move")
async def gripper_move(articulation_id: str, req: GripperMoveRequest, reg=Depends(get_registry)):
    # Blocks until the finger reaches the target or stalls; returns final state.
    return await reg.get_device(articulation_id).gripper_move(req.fraction)


@gripper.post("/open")
async def gripper_open(articulation_id: str, reg=Depends(get_registry)):
    return await reg.get_device(articulation_id).gripper_open()


@gripper.post("/close")
async def gripper_close(articulation_id: str, reg=Depends(get_registry)):
    return await reg.get_device(articulation_id).gripper_close()


@gripper.get("/state")
async def gripper_state(articulation_id: str, reg=Depends(get_registry)):
    return reg.get_device(articulation_id).gripper_state()


# ---------------------------------------------------------------------------
#
# These routers reproduce the spec's General / Stage / Prims surface so the
# bridge can grow into a drop-in replacement. General, Stage (bar configuration
# import/export) and Prims are implemented; remaining stubs answer
# 501 Not Implemented until their logic is wired up.
#
# Omni/pxr imports are done lazily inside the handlers so this module still
# imports outside Isaac Sim (e.g. in tests).
# ---------------------------------------------------------------------------


def _extension_versions():
    """Best-effort map of enabled Kit extensions -> version.

    Runs inside Isaac Sim, so ``omni.kit.app`` is importable (imported lazily so
    this module still loads outside Isaac, e.g. in tests). Defensive: the
    manager's summary dict shape varies across Kit releases, so a missing
    ``version`` is recovered from the id (``"<name>-<version>"``) rather than
    raising.
    """
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    versions = {}
    for ext in manager.get_extensions():
        if not ext.get("enabled"):
            continue
        name = ext.get("name", "")
        version = ext.get("version")
        if version is None:
            ext_id = ext.get("id", "")
            version = ext_id[len(name) + 1:] or None  # strip the "<name>-" prefix
        versions[name] = version
    return versions


general = APIRouter(tags=["general"])


@general.get("/status", summary="Get Status")
async def get_status():
    """Service health. Returns ``OK`` while the bridge is running."""
    return {"status": "OK"}


@general.get("/version", summary="Get Versions")
async def get_versions():
    """List installed (enabled) extensions with their versions."""
    return _extension_versions()


stage = APIRouter(prefix="/stage", tags=["stage"])


def _stage():
    """Current USD stage, or 409 if no stage is open."""
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise HTTPException(status_code=409, detail="no USD stage is open")
    return stage


@stage.get("/scene", summary="Get Active Scene")
async def get_active_scene():
    """URI/identifier of the open USD stage (empty string if none)."""
    import omni.usd

    return omni.usd.get_context().get_stage_url() or ""


@stage.put("/scene", summary="Open Scene")
async def open_scene(req: OpenSceneRequest):
    """Open the USD stage at ``uri`` (replaces the current stage)."""
    import omni.usd

    success, error = await omni.usd.get_context().open_stage_async(req.uri)
    if not success:
        raise HTTPException(status_code=400, detail=f"could not open '{req.uri}': {error}")


@stage.get("/motion-groups", summary="List Stage Motion Groups")
async def list_stage_motion_groups():
    """Prim paths of every articulation root in the stage (potential robots)."""
    from pxr import Usd, UsdPhysics

    stage = _stage()
    return [
        prim.GetPath().pathString
        for prim in Usd.PrimRange(stage.GetPseudoRoot())
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]


@stage.get("/units", summary="Get Stage Units")
async def get_stage_units():
    """Linear scale of the stage in meters per unit."""
    from pxr import UsdGeom

    return StageUnits(meters_per_unit=UsdGeom.GetStageMetersPerUnit(_stage()))


@stage.put("/units", summary="Update Stage Units")
async def update_stage_units(units: StageUnits):
    """Set the stage's meters-per-unit scale."""
    from pxr import UsdGeom

    UsdGeom.SetStageMetersPerUnit(_stage(), units.meters_per_unit)


@stage.patch("/simulation/timeline/{action}", summary="Timeline Action")
async def timeline_action(action: TimelineAction):
    """Drive the simulation timeline: play / pause / stop."""
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    {
        TimelineAction.play: timeline.play,
        TimelineAction.pause: timeline.pause,
        TimelineAction.stop: timeline.stop,
    }[action]()


@stage.get("/simulation", summary="Simulation State")
async def simulation_state():
    """Current timeline state: ``playing`` / ``paused`` / ``stopped``."""
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        state = "playing"
    elif timeline.is_stopped():
        state = "stopped"
    else:
        state = "paused"
    return {"timeline": state}


@stage.get("/configuration", summary="Export Configuration")
async def export_configuration():
    not_implemented()


@stage.post("/configuration", summary="Import Configuration")
async def import_configuration():
    not_implemented()


prims = APIRouter(prefix="/prims", tags=["prims"])

# customData namespace where we persist a prim's "default pose" (local-space,
# 6-float rotation-vector form). Mirrors how the extension remembers a pose to
# reset prims back to.
_DEFAULT_POSE_KEY = "telekinesis:default_pose"


def _prim_or_404(prim_path):
    """Resolve a prim path on the open stage, or raise 404."""
    prim = _stage().GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise HTTPException(status_code=404, detail=f"prim '{prim_path}' not found")
    return prim


def _matrix_to_pose(matrix, rotation_type):
    """Gf.Matrix4d -> {"pose": [...]} in cartesian (rotvec) or quaternion form."""
    import math

    translation = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotation()
    head = [translation[0], translation[1], translation[2]]
    if rotation_type == "quaternions":
        quat = rotation.GetQuat()
        imaginary = quat.GetImaginary()
        return {"pose": head + [quat.GetReal(), imaginary[0], imaginary[1], imaginary[2]]}
    # cartesian: rotation vector = axis * angle (radians)
    axis = rotation.GetAxis()
    angle = math.radians(rotation.GetAngle())
    return {"pose": head + [axis[0] * angle, axis[1] * angle, axis[2] * angle]}


def _wspose_to_matrix(pose):
    """6-float rotation-vector pose -> Gf.Matrix4d."""
    import math

    from pxr import Gf

    x, y, z, rx, ry, rz = pose
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle:
        rotation = Gf.Rotation(Gf.Vec3d(rx / angle, ry / angle, rz / angle), math.degrees(angle))
    else:
        rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotateOnly(rotation)
    matrix.SetTranslateOnly(Gf.Vec3d(x, y, z))
    return matrix


def _world_matrix(prim):
    from pxr import Usd, UsdGeom

    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def _local_matrix(prim):
    from pxr import Usd, UsdGeom

    return UsdGeom.Xformable(prim).GetLocalTransformation(Usd.TimeCode.Default())


def _set_local_matrix(prim, local_matrix):
    from pxr import UsdGeom

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(local_matrix)


def _set_world_matrix(prim, world_matrix):
    """Set a prim's pose given a world-space matrix (converts through the parent)."""
    from pxr import Usd, UsdGeom

    parent = prim.GetParent()
    if parent and parent.IsValid() and parent.IsA(UsdGeom.Xformable):
        parent_world = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local_matrix = world_matrix * parent_world.GetInverse()
    else:
        local_matrix = world_matrix
    _set_local_matrix(prim, local_matrix)


@prims.get("/poses", summary="Get Pose")
async def get_pose(
    prim_path: str,
    coordinate_system: str = Query("local", pattern="^(world|local)$"),
    rotation_type: str = Query("cartesian", pattern="^(cartesian|quaternions)$"),
):
    """Pose of a prim, in world or local space, as rotvec or quaternion."""
    prim = _prim_or_404(prim_path)
    matrix = _world_matrix(prim) if coordinate_system == "world" else _local_matrix(prim)
    return _matrix_to_pose(matrix, rotation_type)


@prims.put("/poses", summary="Update Pose")
async def update_pose(req: UpdatePoseRequest):
    """Set a prim's world-space pose from a rotation-vector ``WSPose``."""
    prim = _prim_or_404(req.prim_path)
    _set_world_matrix(prim, _wspose_to_matrix(req.input_pose.pose))


@prims.get("/poses/relative", summary="Get Relative Pose")
async def get_relative_pose(
    prim_path_1: str,
    prim_path_2: str,
    mode: str = Query("normal", pattern="^(normal|inverse_first|inverse_second|inverse_both)$"),
    rotation_type: str = Query("cartesian", pattern="^(cartesian|quaternions)$"),
):
    """Pose of prim 2 expressed in prim 1's frame.

    ``mode`` optionally inverts either world transform before composing:
    ``relative = world2 * inverse(world1)``.
    """
    a = _world_matrix(_prim_or_404(prim_path_1))
    b = _world_matrix(_prim_or_404(prim_path_2))
    if mode in ("inverse_first", "inverse_both"):
        a = a.GetInverse()
    if mode in ("inverse_second", "inverse_both"):
        b = b.GetInverse()
    return _matrix_to_pose(b * a.GetInverse(), rotation_type)


@prims.post("/poses/relative", summary="Apply Relative Pose")
async def apply_relative_pose(req: ApplyRelativePoseRequest):
    """Pre/post-multiply a prim's world pose by a relative ``WSPose``.

    ``object_first`` chooses the multiplication order: ``world * relative`` when
    true (move in the prim's own frame), ``relative * world`` otherwise (move in
    world frame).
    """
    prim = _prim_or_404(req.prim_path)
    relative = _wspose_to_matrix(req.relative_pose.pose)
    current = _world_matrix(prim)
    new_world = (current * relative) if req.object_first else (relative * current)
    _set_world_matrix(prim, new_world)


@prims.get("/poses/default", summary="List Default Poses")
async def list_default_poses():
    """Map every prim that has a stored default pose to that pose (local rotvec)."""
    from pxr import Usd

    poses = {}
    for prim in Usd.PrimRange(_stage().GetPseudoRoot()):
        stored = prim.GetCustomDataByKey(_DEFAULT_POSE_KEY)
        if stored is not None:
            poses[prim.GetPath().pathString] = {"pose": list(stored)}
    return poses


@prims.put("/poses/default", summary="Assign Default Poses")
async def assign_default_poses(prim_path: str = Body(..., embed=False)):
    """Record the prim's current local pose as its default (for later reset)."""
    prim = _prim_or_404(prim_path)
    prim.SetCustomDataByKey(_DEFAULT_POSE_KEY, _matrix_to_pose(_local_matrix(prim), "cartesian")["pose"])


@prims.delete("/poses/default", summary="Clear Default Poses")
async def clear_default_poses():
    """Forget every stored default pose."""
    from pxr import Usd

    for prim in Usd.PrimRange(_stage().GetPseudoRoot()):
        if prim.GetCustomDataByKey(_DEFAULT_POSE_KEY) is not None:
            prim.ClearCustomDataByKey(_DEFAULT_POSE_KEY)


@prims.post("/poses/default/reset", summary="Reset Prim Poses To Default")
async def reset_to_default_poses(prim_path: str = Body(..., embed=False)):
    """Restore a prim to its stored default pose (404 if none was assigned)."""
    prim = _prim_or_404(prim_path)
    stored = prim.GetCustomDataByKey(_DEFAULT_POSE_KEY)
    if stored is None:
        raise HTTPException(status_code=404, detail=f"prim '{prim_path}' has no default pose")
    _set_local_matrix(prim, _wspose_to_matrix(list(stored)))


@prims.put("/metadata", summary="Set Prim Metadata")
async def set_prim_metadata(req: SetPrimMetadataRequest):
    """Store user metadata (category/type) on a prim under customData."""
    prim = _prim_or_404(req.prim_path)
    prim.SetCustomDataByKey("telekinesis:metadata", req.metadata.model_dump())


@prims.delete("/metadata", summary="Remove Prim Metadata")
async def remove_prim_metadata(prim_path: str):
    """Remove the user metadata previously stored on a prim."""
    prim = _prim_or_404(prim_path)
    prim.ClearCustomDataByKey("telekinesis:metadata")


@prims.patch("/visibility", summary="Set Prim Visibility")
async def set_prim_visibility(req: SetVisibilityRequest):
    """Show or hide a prim (UsdGeom imageable visibility)."""
    from pxr import UsdGeom

    imageable = UsdGeom.Imageable(_prim_or_404(req.prim_path))
    if req.visibility == VisibilityAction.show:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()


@prims.patch("/physics/joints", summary="Set Joints")
async def set_joint_state(req: SetJointStateRequest):
    """Enable or disable a physics joint."""
    from pxr import UsdPhysics

    prim = _prim_or_404(req.prim_path)
    if not prim.IsA(UsdPhysics.Joint):
        raise HTTPException(status_code=400, detail=f"prim '{req.prim_path}' is not a physics joint")
    UsdPhysics.Joint(prim).CreateJointEnabledAttr(req.enable)


@prims.patch("/physics/colliders/", summary="Update Colliders")
async def update_colliders(req: UpdateCollidersRequest):
    """Enable or disable collision on a prim (applies the CollisionAPI as needed)."""
    from pxr import UsdPhysics

    prim = _prim_or_404(req.prim_path)
    collision = UsdPhysics.CollisionAPI(prim)
    if not collision:
        collision = UsdPhysics.CollisionAPI.Apply(prim)
    collision.CreateCollisionEnabledAttr(req.enable)


@prims.put("/labels", summary="Set Semantic Label")
async def set_semantic_label():
    not_implemented()


@prims.get("/labels", summary="List Semantic Labels")
async def list_semantic_labels():
    not_implemented()


@prims.delete("/labels", summary="Clear Semantic Labels")
async def clear_semantic_labels():
    not_implemented()


@prims.put("/selected", summary="Select Prims")
async def select_prims():
    not_implemented()


@prims.get("/selected", summary="List Selected Prims")
async def list_selected_prims():
    not_implemented()


# -- nucleus ----------------------------------------------------------------

nucleus = APIRouter(prefix="/nucleus", tags=["nucleus"])


@nucleus.post("/server", summary="Add Nucleus Server")
async def add_nucleus_server():
    not_implemented()


@nucleus.get("/servers", summary="List Nucleus Servers")
async def list_nucleus_servers():
    not_implemented()


@nucleus.post("/server/token", summary="Add Nucleus API Token")
async def add_nucleus_api_token():
    not_implemented()


@nucleus.delete("/server/token", summary="Remove Nucleus API Token")
async def remove_nucleus_api_token():
    not_implemented()


@nucleus.delete("/server/tokens", summary="Remove All Nucleus API Tokens")
async def remove_all_nucleus_api_tokens():
    not_implemented()


# -- manipulators -----------------------------------------------------------

manipulators = APIRouter(prefix="/manipulators", tags=["manipulators"])


@manipulators.post("/motion-groups", summary="Create Motion Group")
async def create_motion_group():
    not_implemented()


@manipulators.get("/motion-groups", summary="List Motion Groups")
async def list_motion_groups():
    not_implemented()


@manipulators.get("/motion-groups/{prim_path:path}", summary="Get Motion Group")
async def get_motion_group(prim_path: str):
    not_implemented()


@manipulators.put("/motion-groups/{prim_path:path}", summary="Update Motion Group Motion Stream")
async def update_motion_group_motion_stream(prim_path: str):
    not_implemented()


@manipulators.delete("/motion-groups/{prim_path:path}", summary="Remove Motion Group")
async def remove_motion_group(prim_path: str):
    not_implemented()


@manipulators.delete("/motion-groups", summary="Clear Motion Groups")
async def clear_motion_groups():
    not_implemented()


# -- periphery (cameras) ----------------------------------------------------

periphery = APIRouter(prefix="/periphery", tags=["periphery"])


@periphery.get("/cameras/prims", summary="List Camera Prims")
async def list_camera_prims():
    not_implemented()


@periphery.get("/cameras/active", summary="Get Active Camera")
async def get_active_camera():
    not_implemented()


@periphery.put("/cameras/active", summary="Set Active Camera")
async def set_active_camera():
    not_implemented()


@periphery.get("/cameras/capture/color", summary="Capture Color Image")
async def capture_color_image():
    not_implemented()


@periphery.get("/cameras/capture/normals", summary="Capture Normals Image")
async def capture_normals_image():
    not_implemented()


@periphery.get("/cameras/capture/depth", summary="Capture Depth Image")
async def capture_depth_image():
    not_implemented()


@periphery.get("/cameras/capture/pointcloud", summary="Capture Point Cloud")
async def capture_pointcloud():
    not_implemented()


@periphery.get("/cameras/capture/bounding-box-2d", summary="Capture Bounding Box 2D")
async def capture_bounding_box_2d():
    not_implemented()


@periphery.get("/cameras/capture/bounding-box-3d", summary="Capture Bounding Box 3D")
async def capture_bounding_box_3d():
    not_implemented()


@periphery.get("/cameras/capture/instance-segmentation", summary="Capture Instance Segmentation")
async def capture_instance_segmentation():
    not_implemented()


@periphery.get("/cameras/capture/semantic-segmentation", summary="Capture Semantic Segmentation")
async def capture_semantic_segmentation():
    not_implemented()


# -- trajectories -----------------------------------------------------------

trajectories = APIRouter(prefix="/trajectories", tags=["trajectories"])


@trajectories.post("/", summary="Create Trajectory")
async def create_trajectory():
    not_implemented()


@trajectories.get("/", summary="List Trajectories")
async def list_trajectories():
    not_implemented()


@trajectories.patch("/{name}", summary="Update Trajectory")
async def update_trajectory(name: str):
    not_implemented()


@trajectories.delete("/{name}", summary="Remove Trajectory")
async def remove_trajectory(name: str):
    not_implemented()


@trajectories.post("/{name}/markers", summary="Create Trajectory Markers")
async def create_trajectory_markers(name: str):
    not_implemented()


@trajectories.delete("/{name}/markers", summary="Remove Trajectory Markers")
async def remove_trajectory_markers(name: str):
    not_implemented()


# -- teaching ---------------------------------------------------------------

teaching = APIRouter(prefix="/teaching", tags=["teaching"])


@teaching.get("/ghost-objects/sources", summary="List Ghost Object Sources")
async def list_ghost_object_sources():
    not_implemented()


@teaching.get("/tcps/sources", summary="List TCP Sources")
async def list_tcp_sources():
    not_implemented()


@teaching.post("/ghost-objects", summary="Create Ghost Object")
async def create_ghost_object():
    not_implemented()


@teaching.delete("/ghost-objects", summary="Clear Ghost Objects")
async def clear_ghost_objects():
    not_implemented()


@teaching.get("/ghost-objects", summary="List Ghost Objects")
async def list_ghost_objects():
    not_implemented()


@teaching.get("/ghost-objects/export", summary="Export Ghost Objects")
async def export_ghost_objects():
    not_implemented()


# -- trajectory planner -----------------------------------------------------

trajectory_planner = APIRouter(prefix="/trajectory-planner", tags=["trajectory-planner"])


@trajectory_planner.get("/export", summary="Export Trajectory Plans")
async def export_trajectory_plans():
    not_implemented()


@trajectory_planner.get("/{skill_name}/export", summary="Export Trajectory Plan Skill")
async def export_trajectory_plan_skill(skill_name: str):
    not_implemented()


# -- overlays ---------------------------------------------------------------

overlays = APIRouter(prefix="/overlays", tags=["overlays"])


@overlays.put("/robot/visibility", summary="Set Robot Overlay Visibility")
async def set_robot_overlay_visibility():
    not_implemented()


# -- physics / collision world ----------------------------------------------

physics = APIRouter(prefix="/physics", tags=["physics"])


@physics.post("/collision/sweep", summary="Sweep Collisions")
async def sweep_collisions():
    not_implemented()


ALL_ROUTERS = (
    articulations, robot, gripper,
    general, stage, prims,
    nucleus, manipulators, periphery,
    trajectories, teaching, trajectory_planner,
    overlays, physics,
)
