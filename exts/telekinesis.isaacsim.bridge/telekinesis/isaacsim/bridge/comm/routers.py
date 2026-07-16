# SPDX-License-Identifier: Apache-2.0
"""HTTP routers for the bridge, grouped by domain.

Refereces:
Prim and articulation:
- https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.assets.html#isaaclab.assets.Articulation
- https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sim.utils.html#module-isaaclab.sim.utils.prims

The routers are intentionally thin: every handler receives the request, calls one
service method, and returns the result. All orchestration / USD logic lives in
:mod:`..services`; the services are injected via ``Depends`` (see
:mod:`.dependencies`), which pulls each shared service off ``app.state`` where
:class:`..comm.server.BridgeServer` stashed it. Request bodies live in
:mod:`.models`.

Routes mirrored from the spec but not yet wired up answer 501 via
``not_implemented()``.
"""

from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect, status

from .dependencies import (
    get_articulation_service,
    get_general_service,
    get_prim_service,
    get_stage_service,
)
from .models import (
    ApplyRelativePoseRequest,
    AssembleRobotRequest,
    CreateArticulationRequest,
    DefaultJointStateRequest,
    JointEffortsRequest,
    JointPositionsRequest,
    JointVelocitiesRequest,
    OpenSceneRequest,
    PrimPathRequest,
    SetDrivenJointsRequest,
    SetEnabledRequest,
    SetJointPositionsRequest,
    SetJointStateRequest,
    SetPrimMetadataRequest,
    SetVelocityRequest,
    SetVisibilityRequest,
    SetWorldVelocityRequest,
    SolverIterationCountRequest,
    SolverThresholdRequest,
    StageUnits,
    TimelineAction,
    UpdateCollidersRequest,
    UpdatePoseRequest,
    VisibilityAction,
)


def not_implemented():
    """Uniform 501 for routes we mirror from the spec but haven't wired up yet."""
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="Not implemented",
    )


# -- articulations (device-kind agnostic) -----------------------------------

articulations = APIRouter(tags=["articulations"])


@articulations.put("/articulations", summary="Create Articulation")
async def create_articulation(req: CreateArticulationRequest, articulation_service=Depends(get_articulation_service)):
    # Register (and bind) one articulation; returns its id, prim, dof and state.
    return await articulation_service.create_articulation(req.prim_path, req.urdf_path)


@articulations.get("/articulations", summary="List Articulations")
async def list_articulations(articulation_service=Depends(get_articulation_service)):
    return articulation_service.list_articulations()


@articulations.get("/articulations/{articulation_id}", summary="Get Articulation")
async def get_articulation(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Id + static description (prim, dof, names, current state). 404 if unknown.
    return articulation_service.get_articulation(articulation_id)


@articulations.delete("/articulations/{articulation_id}", summary="Delete Articulation")
async def delete_articulation(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Unregister the articulation (the USD prim is left in the stage).
    return articulation_service.delete_articulation(articulation_id)


@articulations.post("/articulations/{articulation_id}/move_j", summary="Move Joint Positions")
async def move_j(
    articulation_id: str, req: JointPositionsRequest, articulation_service=Depends(get_articulation_service)
):
    # Drive the joints (radians). Blocks until reached/stalled unless asynchronous.
    return await articulation_service.move_j(
        articulation_id, req.joint_positions, req.indices, req.asynchronous)


@articulations.post("/articulations/{articulation_id}/set_j", summary="Set Joint Positions")
async def set_j(
    articulation_id: str, req: SetJointPositionsRequest, articulation_service=Depends(get_articulation_service)
):
    # Teleport the joints directly to the targets (radians); immediate, no blocking.
    return await articulation_service.set_j(articulation_id, req.joint_positions, req.indices)


@articulations.websocket("/articulations/{articulation_id}/stream_joint_positions")
async def stream_joint_positions(websocket: WebSocket, articulation_id: str):
    # High-rate teleport stream: the client opens one connection bound to this
    # articulation and pushes {"joint_positions": [...], "indices": [...]?} frames
    # (radians). Fire-and-forget -- the server applies each frame and sends nothing
    # back, so the client streams at full rate.
    service = websocket.app.state.articulation_service
    await websocket.accept()
    try:
        service.get_device(articulation_id)
    except HTTPException as exc:
        await websocket.close(code=1008, reason=exc.detail)
        return
    try:
        while True:
            try:
                # receive_json() itself can raise (e.g. json.JSONDecodeError, a
                # ValueError subclass, for a frame that isn't valid JSON) -- kept
                # inside this same try so a malformed frame is skipped exactly
                # like a well-formed-but-wrong-shape one, not left to crash the
                # connection.
                message = await websocket.receive_json()
                service.stream_joint_positions(
                    articulation_id, message["joint_positions"], message.get("indices")
                )
            except (KeyError, ValueError):
                # Bad frame (not valid JSON, or missing/mismatched joint_positions):
                # skip it, keep the stream open.
                pass
            except HTTPException as exc:
                # The device was deleted mid-stream (get_device now 404s) -- close, don't crash.
                await websocket.close(code=1008, reason=exc.detail)
                return
    except WebSocketDisconnect:
        pass


@articulations.post("/articulations/{articulation_id}/joint_velocities", summary="Set Joint Velocities")
async def set_joint_velocities(
    articulation_id: str, req: JointVelocitiesRequest, articulation_service=Depends(get_articulation_service)
):
    # Drive the joints at a velocity (rad/s). Fire-and-forget; holds until the next call.
    return articulation_service.set_joint_velocities(
        articulation_id, req.joint_velocities, req.indices)


@articulations.get("/articulations/{articulation_id}/joints_state", summary="Get Joints State")
async def get_joints_state(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Current positions / velocities / torques of the driven joints.
    return articulation_service.get_joints_state(articulation_id)


@articulations.get("/articulations/{articulation_id}/dof_limits", summary="Get Dof Limits")
async def get_dof_limits(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # [lower, upper] radian limits per driven joint (in joints_state joint_positions order).
    return articulation_service.get_dof_limits(articulation_id)


@articulations.get("/articulations/{articulation_id}/driver_joint", summary="Get Driver Joint")
async def get_driver_joint(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Discover a gripper's actuated driver joint (name + DOF index) via USD schema.
    return articulation_service.get_driver_joint(articulation_id)


@articulations.put("/articulations/{articulation_id}/driven_joints", summary="Set Driven Joints")
async def set_driven_joints(
    articulation_id: str, req: SetDrivenJointsRequest, articulation_service=Depends(get_articulation_service)
):
    # Narrow which joints this device drives (e.g. a gripper to its driver joint).
    return articulation_service.set_driven_joints(articulation_id, req.joint_names)


@articulations.post("/articulations/{articulation_id}/assemble_robot", summary="Assemble Robot")
async def assemble_robot(articulation_id: str, req: AssembleRobotRequest, articulation_service=Depends(get_articulation_service)):
    # Assemble a gripper onto this arm; both devices then share one articulation.
    # A no-op if this arm and gripper are already assembled.
    return await articulation_service.assemble_robot(
        articulation_id, req.gripper_articulation_id, req.arm_mount_link, req.gripper_mount_link, req.offset
    )


# -- articulations: extended SingleArticulation surface ---------------------
#
# The rest of SingleArticulation's API not covered above: joint-level extras
# (efforts, forces, default state, applied action), floating-base motion
# (gravity, world/linear/angular velocity), and PhysX solver tuning.


@articulations.get("/articulations/{articulation_id}/handles_initialized", summary="Get Handles Initialized")
async def get_handles_initialized(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Whether the device's handle is currently valid, without a re-bind.
    return articulation_service.get_handles_initialized(articulation_id)


@articulations.get("/articulations/{articulation_id}/num_bodies", summary="Get Num Bodies")
async def get_num_bodies(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Number of rigid-body links in the underlying articulation.
    return articulation_service.get_num_bodies(articulation_id)


@articulations.get("/articulations/{articulation_id}/dof_properties", summary="Get Dof Properties")
async def get_dof_properties(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Per-driven-joint drive properties (limits, drive mode, stiffness, damping).
    return articulation_service.get_dof_properties(articulation_id)


@articulations.get("/articulations/{articulation_id}/dof_index/{joint_name}", summary="Get Dof Index")
async def get_dof_index(
    articulation_id: str, joint_name: str, articulation_service=Depends(get_articulation_service)
):
    # DOF index of joint_name within the device's driven subset.
    return articulation_service.get_dof_index(articulation_id, joint_name)


@articulations.get("/articulations/{articulation_id}/applied_joint_efforts", summary="Get Applied Joint Efforts")
async def get_applied_joint_efforts(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Efforts last commanded via set_joint_efforts (what was asked for, not measured).
    return articulation_service.get_applied_joint_efforts(articulation_id)


@articulations.get("/articulations/{articulation_id}/measured_joint_forces", summary="Get Measured Joint Forces")
async def get_measured_joint_forces(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Measured 6-axis joint reaction force/torque per driven joint.
    return articulation_service.get_measured_joint_forces(articulation_id)


@articulations.get("/articulations/{articulation_id}/joints_default_state", summary="Get Joints Default State")
async def get_joints_default_state(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Stored joint-space home pose for the driven joints.
    return articulation_service.get_joints_default_state(articulation_id)


@articulations.put("/articulations/{articulation_id}/joints_default_state", summary="Set Joints Default State")
async def set_joints_default_state(
    articulation_id: str, req: DefaultJointStateRequest, articulation_service=Depends(get_articulation_service)
):
    # Set the driven joints' stored home pose, applied on the next reset.
    return articulation_service.set_joints_default_state(
        articulation_id, req.joint_positions, req.joint_velocities, req.joint_efforts)


@articulations.get("/articulations/{articulation_id}/applied_action", summary="Get Applied Action")
async def get_applied_action(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Last ArticulationAction PhysX actually received for the whole articulation.
    return articulation_service.get_applied_action(articulation_id)


@articulations.post("/articulations/{articulation_id}/joint_efforts", summary="Set Joint Efforts")
async def set_joint_efforts(
    articulation_id: str, req: JointEffortsRequest, articulation_service=Depends(get_articulation_service)
):
    # Command raw torque/force directly on the chosen joints; bypasses the drive.
    return articulation_service.set_joint_efforts(articulation_id, req.joint_efforts, req.indices)


@articulations.post("/articulations/{articulation_id}/enable_gravity", summary="Enable Gravity")
async def enable_gravity(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Gravity affects the whole articulation (body-level, not per-joint).
    return articulation_service.enable_gravity(articulation_id)


@articulations.post("/articulations/{articulation_id}/disable_gravity", summary="Disable Gravity")
async def disable_gravity(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Gravity no longer affects the whole articulation (body-level, not per-joint).
    return articulation_service.disable_gravity(articulation_id)


@articulations.get("/articulations/{articulation_id}/world_velocity", summary="Get World Velocity")
async def get_world_velocity(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Root link's full 6-DOF world-space velocity (meaningful only for a floating base).
    return articulation_service.get_world_velocity(articulation_id)


@articulations.put("/articulations/{articulation_id}/world_velocity", summary="Set World Velocity")
async def set_world_velocity(
    articulation_id: str, req: SetWorldVelocityRequest, articulation_service=Depends(get_articulation_service)
):
    return articulation_service.set_world_velocity(articulation_id, req.velocity)


@articulations.get("/articulations/{articulation_id}/linear_velocity", summary="Get Linear Velocity")
async def get_linear_velocity(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Root link's linear (translational) velocity.
    return articulation_service.get_linear_velocity(articulation_id)


@articulations.put("/articulations/{articulation_id}/linear_velocity", summary="Set Linear Velocity")
async def set_linear_velocity(
    articulation_id: str, req: SetVelocityRequest, articulation_service=Depends(get_articulation_service)
):
    return articulation_service.set_linear_velocity(articulation_id, req.velocity)


@articulations.get("/articulations/{articulation_id}/angular_velocity", summary="Get Angular Velocity")
async def get_angular_velocity(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    # Root link's angular (rotational) velocity.
    return articulation_service.get_angular_velocity(articulation_id)


@articulations.put("/articulations/{articulation_id}/angular_velocity", summary="Set Angular Velocity")
async def set_angular_velocity(
    articulation_id: str, req: SetVelocityRequest, articulation_service=Depends(get_articulation_service)
):
    return articulation_service.set_angular_velocity(articulation_id, req.velocity)


@articulations.get("/articulations/{articulation_id}/solver/position_iteration_count", summary="Get Solver Position Iteration Count")
async def get_solver_position_iteration_count(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    return articulation_service.get_solver_position_iteration_count(articulation_id)


@articulations.put("/articulations/{articulation_id}/solver/position_iteration_count", summary="Set Solver Position Iteration Count")
async def set_solver_position_iteration_count(
    articulation_id: str, req: SolverIterationCountRequest, articulation_service=Depends(get_articulation_service)
):
    return articulation_service.set_solver_position_iteration_count(articulation_id, req.count)


@articulations.get("/articulations/{articulation_id}/solver/velocity_iteration_count", summary="Get Solver Velocity Iteration Count")
async def get_solver_velocity_iteration_count(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    return articulation_service.get_solver_velocity_iteration_count(articulation_id)


@articulations.put("/articulations/{articulation_id}/solver/velocity_iteration_count", summary="Set Solver Velocity Iteration Count")
async def set_solver_velocity_iteration_count(
    articulation_id: str, req: SolverIterationCountRequest, articulation_service=Depends(get_articulation_service)
):
    return articulation_service.set_solver_velocity_iteration_count(articulation_id, req.count)


@articulations.get("/articulations/{articulation_id}/solver/stabilization_threshold", summary="Get Stabilization Threshold")
async def get_stabilization_threshold(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    return articulation_service.get_stabilization_threshold(articulation_id)


@articulations.put("/articulations/{articulation_id}/solver/stabilization_threshold", summary="Set Stabilization Threshold")
async def set_stabilization_threshold(
    articulation_id: str, req: SolverThresholdRequest, articulation_service=Depends(get_articulation_service)
):
    return articulation_service.set_stabilization_threshold(articulation_id, req.threshold)


@articulations.get("/articulations/{articulation_id}/enabled_self_collisions", summary="Get Enabled Self Collisions")
async def get_enabled_self_collisions(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    return articulation_service.get_enabled_self_collisions(articulation_id)


@articulations.put("/articulations/{articulation_id}/enabled_self_collisions", summary="Set Enabled Self Collisions")
async def set_enabled_self_collisions(
    articulation_id: str, req: SetEnabledRequest, articulation_service=Depends(get_articulation_service)
):
    return articulation_service.set_enabled_self_collisions(articulation_id, req.enabled)


@articulations.get("/articulations/{articulation_id}/solver/sleep_threshold", summary="Get Sleep Threshold")
async def get_sleep_threshold(articulation_id: str, articulation_service=Depends(get_articulation_service)):
    return articulation_service.get_sleep_threshold(articulation_id)


@articulations.put("/articulations/{articulation_id}/solver/sleep_threshold", summary="Set Sleep Threshold")
async def set_sleep_threshold(
    articulation_id: str, req: SolverThresholdRequest, articulation_service=Depends(get_articulation_service)
):
    return articulation_service.set_sleep_threshold(articulation_id, req.threshold)


# ---------------------------------------------------------------------------
#
# These routers reproduce the spec's General / Stage / Prims surface so the
# bridge can grow into a drop-in replacement. General, Stage (bar configuration
# import/export) and Prims are implemented; remaining stubs answer
# 501 Not Implemented until their logic is wired up.
# ---------------------------------------------------------------------------


general = APIRouter(tags=["general"])


@general.get("/status", summary="Get Status")
async def get_status(general_service=Depends(get_general_service)):
    """Service health. Returns ``OK`` while the bridge is running."""
    return general_service.status()


@general.get("/version", summary="Get Versions")
async def get_versions(general_service=Depends(get_general_service)):
    """List installed (enabled) extensions with their versions."""
    return general_service.versions()


stage = APIRouter(prefix="/stage", tags=["stage"])


@stage.get("/scene", summary="Get Active Scene")
async def get_active_scene(stage_service=Depends(get_stage_service)):
    """URI/identifier of the open USD stage (empty string if none)."""
    return stage_service.get_active_scene()


@stage.put("/scene", summary="Open Scene")
async def open_scene(req: OpenSceneRequest, stage_service=Depends(get_stage_service)):
    """Open the USD stage at ``uri`` (replaces the current stage)."""
    await stage_service.open_scene(req.uri)


@stage.get("/motion-groups", summary="List Stage Motion Groups")
async def list_stage_motion_groups(stage_service=Depends(get_stage_service)):
    """Prim paths of every articulation root in the stage (potential robots)."""
    return stage_service.list_motion_groups()


@stage.get("/units", summary="Get Stage Units")
async def get_stage_units(stage_service=Depends(get_stage_service)):
    """Linear scale of the stage in meters per unit."""
    return StageUnits(meters_per_unit=stage_service.get_units())


@stage.put("/units", summary="Update Stage Units")
async def update_stage_units(units: StageUnits, stage_service=Depends(get_stage_service)):
    """Set the stage's meters-per-unit scale."""
    stage_service.update_units(units.meters_per_unit)


@stage.patch("/simulation/timeline/{action}", summary="Timeline Action")
async def timeline_action(action: TimelineAction, stage_service=Depends(get_stage_service)):
    """Drive the simulation timeline: play / pause / stop."""
    stage_service.timeline_action(action)


@stage.get("/simulation", summary="Simulation State")
async def simulation_state(stage_service=Depends(get_stage_service)):
    """Current timeline state: ``playing`` / ``paused`` / ``stopped``."""
    return stage_service.simulation_state()


@stage.get("/configuration", summary="Export Configuration")
async def export_configuration():
    not_implemented()


@stage.post("/configuration", summary="Import Configuration")
async def import_configuration():
    not_implemented()


prims = APIRouter(prefix="/prims", tags=["prims"])


@prims.get("/poses", summary="Get Pose")
async def get_pose(
    prim_path: str,
    coordinate_system: str = Query("local", pattern="^(world|local)$"),
    rotation_type: str = Query("cartesian", pattern="^(cartesian|quaternions)$"),
    prim_service=Depends(get_prim_service),
):
    """Pose of a prim, in world or local space, as rotvec or quaternion."""
    return prim_service.get_pose(prim_path, coordinate_system, rotation_type)


@prims.put("/poses", summary="Update Pose")
async def update_pose(req: UpdatePoseRequest, prim_service=Depends(get_prim_service)):
    """Set a prim's world-space pose from a rotation-vector ``WSPose``."""
    prim_service.update_pose(req.prim_path, req.input_pose.pose)


@prims.get("/poses/relative", summary="Get Relative Pose")
async def get_relative_pose(
    prim_path_1: str,
    prim_path_2: str,
    mode: str = Query("normal", pattern="^(normal|inverse_first|inverse_second|inverse_both)$"),
    rotation_type: str = Query("cartesian", pattern="^(cartesian|quaternions)$"),
    prim_service=Depends(get_prim_service),
):
    """Pose of prim 2 expressed in prim 1's frame."""
    return prim_service.get_relative_pose(prim_path_1, prim_path_2, mode, rotation_type)


@prims.post("/poses/relative", summary="Apply Relative Pose")
async def apply_relative_pose(req: ApplyRelativePoseRequest, prim_service=Depends(get_prim_service)):
    """Pre/post-multiply a prim's world pose by a relative ``WSPose``."""
    prim_service.apply_relative_pose(req.prim_path, req.relative_pose.pose, req.object_first)


@prims.get("/poses/default", summary="List Default Poses")
async def list_default_poses(prim_service=Depends(get_prim_service)):
    """Map every prim that has a stored default pose to that pose (local rotvec)."""
    return prim_service.list_default_poses()


@prims.put("/poses/default", summary="Assign Default Poses")
async def assign_default_poses(req: PrimPathRequest, prim_service=Depends(get_prim_service)):
    """Record the prim's current local pose as its default (for later reset)."""
    prim_service.assign_default_pose(req.prim_path)


@prims.delete("/poses/default", summary="Clear Default Poses")
async def clear_default_poses(prim_service=Depends(get_prim_service)):
    """Forget every stored default pose."""
    prim_service.clear_default_poses()


@prims.post("/poses/default/reset", summary="Reset Prim Poses To Default")
async def reset_to_default_poses(req: PrimPathRequest, prim_service=Depends(get_prim_service)):
    """Restore a prim to its stored default pose (404 if none was assigned)."""
    prim_service.reset_to_default_pose(req.prim_path)


@prims.put("/metadata", summary="Set Prim Metadata")
async def set_prim_metadata(req: SetPrimMetadataRequest, prim_service=Depends(get_prim_service)):
    """Store user metadata (category/type) on a prim under customData."""
    prim_service.set_metadata(req.prim_path, req.metadata.model_dump())


@prims.delete("/metadata", summary="Remove Prim Metadata")
async def remove_prim_metadata(prim_path: str, prim_service=Depends(get_prim_service)):
    """Remove the user metadata previously stored on a prim."""
    prim_service.remove_metadata(prim_path)


@prims.patch("/visibility", summary="Set Prim Visibility")
async def set_prim_visibility(req: SetVisibilityRequest, prim_service=Depends(get_prim_service)):
    """Show or hide a prim (UsdGeom imageable visibility)."""
    prim_service.set_visibility(req.prim_path, req.visibility == VisibilityAction.show)


@prims.patch("/physics/joints", summary="Set Joints")
async def set_joint_state(req: SetJointStateRequest, prim_service=Depends(get_prim_service)):
    """Enable or disable a physics joint."""
    prim_service.set_joint_state(req.prim_path, req.enable)


@prims.patch("/physics/colliders/", summary="Update Colliders")
async def update_colliders(req: UpdateCollidersRequest, prim_service=Depends(get_prim_service)):
    """Enable or disable collision on a prim (applies the CollisionAPI as needed)."""
    prim_service.update_colliders(req.prim_path, req.enable)


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
    articulations,
    general, stage, prims,
    nucleus, manipulators, periphery,
    trajectories, teaching, trajectory_planner,
    overlays, physics,
)
