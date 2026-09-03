# SPDX-License-Identifier: Apache-2.0
# pylint: disable=line-too-long
"""HTTP routers for the bridge, grouped by domain.

References:
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
# pylint: enable=line-too-long

import asyncio

from fastapi import (
    APIRouter,
    Depends,
    HTTPException,
    Query,
    Response,
    WebSocket,
    WebSocketDisconnect,
    status,
)

from . import binary
from .dependencies import (
    get_articulation_service,
    get_camera_service,
    get_conveyor_service,
    get_general_service,
    get_lidar_service,
    get_lightbeam_service,
    get_prim_service,
    get_stage_service,
    get_surface_gripper_service,
)
from .models import (
    AddToSceneRequest,
    ApplyRelativePoseRequest,
    AssembleRobotRequest,
    AttachmentPointPropertiesRequest,
    CameraApertureRequest,
    CameraClippingRangeRequest,
    CameraFloatValueRequest,
    CameraLocalPoseRequest,
    CameraResolutionRequest,
    CameraStringValueRequest,
    CameraWorldPoseRequest,
    CaptureRequest,
    ConveyorVelocityRequest,
    CreateArticulationRequest,
    CreateCameraRequest,
    CreateConveyorRequest,
    CreateLidarRequest,
    CreateLightBeamRequest,
    CreateSurfaceGripperRequest,
    DefaultJointStateRequest,
    DofGainsRequest,
    GripperActionRequest,
    JointEffortsRequest,
    JointPositionsRequest,
    JointVelocitiesRequest,
    LidarBoolValueRequest,
    LidarCaptureRequest,
    LidarFloatValueRequest,
    LidarLocalPoseRequest,
    LidarWorldPoseRequest,
    LightBeamConfigurationRequest,
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
    SurfaceGripperPropertiesRequest,
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


async def run_until_first_finished(*coroutines):
    """Run the given coroutines concurrently until one finishes, then stop the rest.

    Used by the WebSocket routes below, which each need one coroutine per direction
    of the connection. Isaac Sim owns the event loop these run on, so a task left
    behind when a connection ends would keep running for the life of the simulator:
    the survivors are always cancelled and awaited before returning.

    Re-raises whatever the finished coroutine raised, except a client disconnect --
    that is how a stream normally ends, not a failure.
    """
    tasks = [asyncio.ensure_future(coroutine) for coroutine in coroutines]
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for task in tasks:
            task.cancel()
        # A cancelled task is not finished until it has been awaited.
        await asyncio.gather(*tasks, return_exceptions=True)

    # asyncio.wait stores an exception on the task instead of raising it, so a
    # genuine failure would otherwise vanish here.
    for task in done:
        error = task.exception()
        if error is not None and not isinstance(error, WebSocketDisconnect):
            raise error


# -- articulations (device-kind agnostic) -----------------------------------

articulations = APIRouter(tags=["articulations"])


@articulations.put("/articulations", summary="Create Articulation")
async def create_articulation(
    req: CreateArticulationRequest, articulation_service=Depends(get_articulation_service)
):
    """Register (and bind) one articulation; returns its id, prim, dof and state."""
    return await articulation_service.create_articulation(
        req.prim_path, req.urdf_path, req.usd_path
    )


@articulations.get("/articulations", summary="List Articulations")
async def list_articulations(articulation_service=Depends(get_articulation_service)):
    """Every registered articulation id mapped to its prim path."""
    return articulation_service.list_articulations()


@articulations.get("/articulations/{articulation_id}", summary="Get Articulation")
async def get_articulation(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Id + static description (prim, dof, names, current state). 404 if unknown."""
    return articulation_service.get_articulation(articulation_id)


@articulations.delete("/articulations/{articulation_id}", summary="Delete Articulation")
async def delete_articulation(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Unregister the articulation (the USD prim is left in the stage)."""
    return articulation_service.delete_articulation(articulation_id)


@articulations.post("/articulations/{articulation_id}/move_j", summary="Move Joint Positions")
async def move_j(
    articulation_id: str,
    req: JointPositionsRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Drive the joints (radians). Blocks until reached/stalled unless asynchronous."""
    return await articulation_service.move_j(
        articulation_id, req.joint_positions, req.indices, req.asynchronous
    )


@articulations.post("/articulations/{articulation_id}/set_j", summary="Set Joint Positions")
async def set_j(
    articulation_id: str,
    req: SetJointPositionsRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Teleport the joints directly to the targets (radians); immediate, no blocking."""
    return await articulation_service.set_j(articulation_id, req.joint_positions, req.indices)


@articulations.websocket("/articulations/{articulation_id}/stream_joint_positions")
async def stream_joint_positions(websocket: WebSocket, articulation_id: str):
    """High-rate drive-target stream: the client opens one connection bound to this
    articulation and pushes {"joint_positions": [...], "indices": [...]?} frames (radians).
    Fire-and-forget -- the server applies each frame and sends nothing back, so the client
    streams at full rate.

    Each frame retargets the position drive, so the joints are driven toward the
    stream rather than placed on it: the measured pose trails the commanded one by
    the drive's response. Read the result back from
    ``stream_articulation_state`` rather than assuming the commanded pose was reached.

    Only the newest frame is applied, once per simulator update; frames that arrive
    while one is already waiting are discarded. A client streaming faster than the
    simulator updates therefore keeps the robot where it wants it *now* instead of
    working through a backlog of stale positions -- so a drop in frame rate makes the
    motion coarser, never late. Send a dense path if the intermediate positions
    matter, and pace it at the simulator's update rate.
    """
    # Imported here rather than at module scope: scripts/generate_openapi.py imports
    # this module with no Isaac Sim in the interpreter.
    import omni.kit.app  # pylint: disable=import-outside-toplevel

    service = websocket.app.state.articulation_service
    await websocket.accept()
    try:
        service.get_device(articulation_id)
    except HTTPException as exc:
        await websocket.close(code=1008, reason=exc.detail)
        return

    # Newest frame received but not yet applied. Receiving only stores it, which is
    # cheap enough that the receive loop always keeps pace with the client and no
    # backlog can build.
    latest_frame = None

    async def receive_frames():
        nonlocal latest_frame
        while True:
            try:
                # receive_json() itself can raise (e.g. json.JSONDecodeError, a
                # ValueError subclass, for a frame that isn't valid JSON).
                latest_frame = await websocket.receive_json()
            except ValueError:
                # Not valid JSON: skip it, keep the stream open.
                pass

    async def apply_frames():
        nonlocal latest_frame
        app = omni.kit.app.get_app()
        while True:
            await app.next_update_async()
            if latest_frame is None:
                continue
            frame, latest_frame = latest_frame, None
            try:
                # pylint can't tell the `continue` above rules out None here.
                service.stream_joint_positions(
                    articulation_id,
                    frame["joint_positions"],  # pylint: disable=unsubscriptable-object
                    frame.get("indices"),
                )
            except (AttributeError, KeyError, TypeError, ValueError):
                # Well-formed JSON of the wrong shape (not an object, or missing /
                # mismatched joint_positions): skip it, keep the stream open.
                pass
            except HTTPException as exc:
                # The device was deleted mid-stream (get_device now 404s) -- close, don't crash.
                await websocket.close(code=1008, reason=exc.detail)
                return

    await run_until_first_finished(receive_frames(), apply_frames())


@articulations.post(
    "/articulations/{articulation_id}/joint_velocities", summary="Set Joint Velocities"
)
async def set_joint_velocities(
    articulation_id: str,
    req: JointVelocitiesRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Drive the joints at a velocity (rad/s). Fire-and-forget; holds until the next call."""
    return articulation_service.set_joint_velocities(
        articulation_id, req.joint_velocities, req.indices
    )


@articulations.get("/articulations/{articulation_id}/joints_state", summary="Get Joints State")
async def get_joints_state(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Current positions / velocities / torques of the driven joints."""
    return articulation_service.get_joints_state(articulation_id)


@articulations.get(
    "/articulations/{articulation_id}/articulation_state", summary="Get Articulation State"
)
async def get_articulation_state(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Every per-frame quantity in one snapshot, or null if the handle is unreadable."""
    return articulation_service.get_articulation_state(articulation_id)


@articulations.websocket("/articulations/{articulation_id}/stream_articulation_state")
async def stream_articulation_state(websocket: WebSocket, articulation_id: str):
    """State push stream: the client opens one connection bound to this articulation and
    the server sends one frame per simulator update, shaped exactly like the
    ``articulation_state`` getter's response (radians). The client sends nothing.

    Nothing is sent while the timeline is stopped or the articulation's handle is
    otherwise unreadable; the connection stays open and frames resume on play. A
    client that reads more slowly than the server sends misses frames rather than
    falling steadily further behind.
    """
    # Imported here rather than at module scope: scripts/generate_openapi.py imports
    # this module with no Isaac Sim in the interpreter.
    import omni.kit.app  # pylint: disable=import-outside-toplevel

    service = websocket.app.state.articulation_service
    await websocket.accept()
    try:
        service.get_device(articulation_id)
    except HTTPException as exc:
        await websocket.close(code=1008, reason=exc.detail)
        return

    async def send_state_frames():
        app = omni.kit.app.get_app()
        while True:
            await app.next_update_async()
            try:
                state = service.get_articulation_state(articulation_id)
            except HTTPException as exc:
                # The device was deleted mid-stream (get_device now 404s) -- close, don't crash.
                await websocket.close(code=1008, reason=exc.detail)
                return
            if state is None:
                continue
            try:
                await websocket.send_json(state)
            except (TypeError, ValueError):
                # The state itself could not be serialized -- a real fault worth
                # surfacing, not a connection problem.
                raise
            except Exception:  # pylint: disable=broad-except
                # The client disconnected between reading the state and sending
                # it. Pushing to that socket is all this does, so stop.
                return

    async def watch_for_disconnect():
        # Nothing is expected on this connection, but something has to read from it:
        # a disconnect is delivered as a received message, so a send-only handler
        # would never notice the client had gone and would push frames forever.
        while True:
            message = await websocket.receive()
            if message["type"] == "websocket.disconnect":
                return

    await run_until_first_finished(send_state_frames(), watch_for_disconnect())


@articulations.get("/articulations/{articulation_id}/dof_limits", summary="Get Dof Limits")
async def get_dof_limits(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """[lower, upper] radian limits per driven joint (in joints_state joint_positions order)."""
    return articulation_service.get_dof_limits(articulation_id)


@articulations.get("/articulations/{articulation_id}/driver_joint", summary="Get Driver Joint")
async def get_driver_joint(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Discover a gripper's actuated driver joint (name + DOF index) via USD schema."""
    return articulation_service.get_driver_joint(articulation_id)


@articulations.put("/articulations/{articulation_id}/driven_joints", summary="Set Driven Joints")
async def set_driven_joints(
    articulation_id: str,
    req: SetDrivenJointsRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Narrow which joints this device drives (e.g. a gripper to its driver joint)."""
    return articulation_service.set_driven_joints(articulation_id, req.joint_names)


@articulations.post("/articulations/{articulation_id}/assemble_robot", summary="Assemble Robot")
async def assemble_robot(
    articulation_id: str,
    req: AssembleRobotRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Assemble a gripper onto this arm, either articulated (merged into the arm's
    articulation) or suction (bolted on, staying its own device). A no-op if this arm
    and gripper are already assembled.
    """
    return await articulation_service.assemble_robot(
        articulation_id,
        req.gripper_id,
        req.arm_mount_link,
        req.gripper_mount_link,
        req.offset,
        req.mask_collisions,
    )


# -- articulations: extended SingleArticulation surface ---------------------
#
# The rest of SingleArticulation's API not covered above: joint-level extras
# (efforts, forces, default state, applied action), floating-base motion
# (gravity, world/linear/angular velocity), and PhysX solver tuning.


@articulations.get(
    "/articulations/{articulation_id}/handles_initialized", summary="Get Handles Initialized"
)
async def get_handles_initialized(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Whether the device's handle is currently valid, without a re-bind."""
    return articulation_service.get_handles_initialized(articulation_id)


@articulations.get("/articulations/{articulation_id}/num_bodies", summary="Get Num Bodies")
async def get_num_bodies(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Number of rigid-body links in the underlying articulation."""
    return articulation_service.get_num_bodies(articulation_id)


@articulations.get("/articulations/{articulation_id}/dof_properties", summary="Get Dof Properties")
async def get_dof_properties(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Per-driven-joint drive properties (limits, drive mode, stiffness, damping)."""
    return articulation_service.get_dof_properties(articulation_id)


@articulations.post("/articulations/{articulation_id}/dof_gains", summary="Set Dof Gains")
async def set_dof_gains(
    articulation_id: str,
    req: DofGainsRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Retune the position drive's stiffness / damping / maximum effort. Returns the
    driven joints' resulting drive properties."""
    return articulation_service.set_dof_gains(
        articulation_id, req.stiffness, req.damping, req.max_effort, req.indices
    )


@articulations.get(
    "/articulations/{articulation_id}/dof_index/{joint_name}", summary="Get Dof Index"
)
async def get_dof_index(
    articulation_id: str, joint_name: str, articulation_service=Depends(get_articulation_service)
):
    """DOF index of joint_name within the device's driven subset."""
    return articulation_service.get_dof_index(articulation_id, joint_name)


@articulations.get(
    "/articulations/{articulation_id}/applied_joint_efforts", summary="Get Applied Joint Efforts"
)
async def get_applied_joint_efforts(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Efforts last commanded via set_joint_efforts (what was asked for, not measured)."""
    return articulation_service.get_applied_joint_efforts(articulation_id)


@articulations.get(
    "/articulations/{articulation_id}/measured_joint_forces", summary="Get Measured Joint Forces"
)
async def get_measured_joint_forces(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Measured 6-axis joint reaction force/torque per driven joint."""
    return articulation_service.get_measured_joint_forces(articulation_id)


@articulations.get(
    "/articulations/{articulation_id}/joints_default_state", summary="Get Joints Default State"
)
async def get_joints_default_state(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Stored joint-space home pose for the driven joints."""
    return articulation_service.get_joints_default_state(articulation_id)


@articulations.put(
    "/articulations/{articulation_id}/joints_default_state", summary="Set Joints Default State"
)
async def set_joints_default_state(
    articulation_id: str,
    req: DefaultJointStateRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Set the driven joints' stored home pose, applied on the next reset."""
    return articulation_service.set_joints_default_state(
        articulation_id, req.joint_positions, req.joint_velocities, req.joint_efforts
    )


@articulations.get("/articulations/{articulation_id}/applied_action", summary="Get Applied Action")
async def get_applied_action(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Last ArticulationAction PhysX actually received for the whole articulation."""
    return articulation_service.get_applied_action(articulation_id)


@articulations.post("/articulations/{articulation_id}/joint_efforts", summary="Set Joint Efforts")
async def set_joint_efforts(
    articulation_id: str,
    req: JointEffortsRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Command raw torque/force directly on the chosen joints; bypasses the drive."""
    return articulation_service.set_joint_efforts(articulation_id, req.joint_efforts, req.indices)


@articulations.post("/articulations/{articulation_id}/enable_gravity", summary="Enable Gravity")
async def enable_gravity(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Gravity affects the whole articulation (body-level, not per-joint)."""
    return articulation_service.enable_gravity(articulation_id)


@articulations.post("/articulations/{articulation_id}/disable_gravity", summary="Disable Gravity")
async def disable_gravity(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Gravity no longer affects the whole articulation (body-level, not per-joint)."""
    return articulation_service.disable_gravity(articulation_id)


@articulations.get("/articulations/{articulation_id}/world_velocity", summary="Get World Velocity")
async def get_world_velocity(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Root link's full 6-DOF world-space velocity (meaningful only for a floating base)."""
    return articulation_service.get_world_velocity(articulation_id)


@articulations.put("/articulations/{articulation_id}/world_velocity", summary="Set World Velocity")
async def set_world_velocity(
    articulation_id: str,
    req: SetWorldVelocityRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Set the root link's full 6-DOF world-space velocity."""
    return articulation_service.set_world_velocity(articulation_id, req.velocity)


@articulations.get(
    "/articulations/{articulation_id}/linear_velocity", summary="Get Linear Velocity"
)
async def get_linear_velocity(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Root link's linear (translational) velocity."""
    return articulation_service.get_linear_velocity(articulation_id)


@articulations.put(
    "/articulations/{articulation_id}/linear_velocity", summary="Set Linear Velocity"
)
async def set_linear_velocity(
    articulation_id: str,
    req: SetVelocityRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Set the root link's linear velocity only (leaves angular untouched)."""
    return articulation_service.set_linear_velocity(articulation_id, req.velocity)


@articulations.get(
    "/articulations/{articulation_id}/angular_velocity", summary="Get Angular Velocity"
)
async def get_angular_velocity(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Root link's angular (rotational) velocity."""
    return articulation_service.get_angular_velocity(articulation_id)


@articulations.put(
    "/articulations/{articulation_id}/angular_velocity", summary="Set Angular Velocity"
)
async def set_angular_velocity(
    articulation_id: str,
    req: SetVelocityRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Set the root link's angular velocity only (leaves linear untouched)."""
    return articulation_service.set_angular_velocity(articulation_id, req.velocity)


@articulations.get(
    "/articulations/{articulation_id}/solver/position_iteration_count",
    summary="Get Solver Position Iteration Count",
)
async def get_solver_position_iteration_count(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """PhysX position-solver iteration count for this articulation."""
    return articulation_service.get_solver_position_iteration_count(articulation_id)


@articulations.put(
    "/articulations/{articulation_id}/solver/position_iteration_count",
    summary="Set Solver Position Iteration Count",
)
async def set_solver_position_iteration_count(
    articulation_id: str,
    req: SolverIterationCountRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Set the PhysX position-solver iteration count (accuracy vs. perf)."""
    return articulation_service.set_solver_position_iteration_count(articulation_id, req.count)


@articulations.get(
    "/articulations/{articulation_id}/solver/velocity_iteration_count",
    summary="Get Solver Velocity Iteration Count",
)
async def get_solver_velocity_iteration_count(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """PhysX velocity-solver iteration count for this articulation."""
    return articulation_service.get_solver_velocity_iteration_count(articulation_id)


@articulations.put(
    "/articulations/{articulation_id}/solver/velocity_iteration_count",
    summary="Set Solver Velocity Iteration Count",
)
async def set_solver_velocity_iteration_count(
    articulation_id: str,
    req: SolverIterationCountRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Set the PhysX velocity-solver iteration count (accuracy vs. perf)."""
    return articulation_service.set_solver_velocity_iteration_count(articulation_id, req.count)


@articulations.get(
    "/articulations/{articulation_id}/solver/stabilization_threshold",
    summary="Get Stabilization Threshold",
)
async def get_stabilization_threshold(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Mass-normalized kinetic energy below which PhysX may stabilize this articulation."""
    return articulation_service.get_stabilization_threshold(articulation_id)


@articulations.put(
    "/articulations/{articulation_id}/solver/stabilization_threshold",
    summary="Set Stabilization Threshold",
)
async def set_stabilization_threshold(
    articulation_id: str,
    req: SolverThresholdRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Set the stabilization threshold."""
    return articulation_service.set_stabilization_threshold(articulation_id, req.threshold)


@articulations.get(
    "/articulations/{articulation_id}/enabled_self_collisions",
    summary="Get Enabled Self Collisions",
)
async def get_enabled_self_collisions(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Whether this articulation's own links can collide with each other."""
    return articulation_service.get_enabled_self_collisions(articulation_id)


@articulations.put(
    "/articulations/{articulation_id}/enabled_self_collisions",
    summary="Set Enabled Self Collisions",
)
async def set_enabled_self_collisions(
    articulation_id: str,
    req: SetEnabledRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Enable/disable self-collision between this articulation's own links."""
    return articulation_service.set_enabled_self_collisions(articulation_id, req.enabled)


@articulations.get(
    "/articulations/{articulation_id}/solver/sleep_threshold", summary="Get Sleep Threshold"
)
async def get_sleep_threshold(
    articulation_id: str, articulation_service=Depends(get_articulation_service)
):
    """Velocity threshold below which PhysX lets this articulation sleep."""
    return articulation_service.get_sleep_threshold(articulation_id)


@articulations.put(
    "/articulations/{articulation_id}/solver/sleep_threshold", summary="Set Sleep Threshold"
)
async def set_sleep_threshold(
    articulation_id: str,
    req: SolverThresholdRequest,
    articulation_service=Depends(get_articulation_service),
):
    """Set the sleep threshold."""
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


@stage.post("/scene/reference", summary="Add To Scene")
async def add_to_scene(req: AddToSceneRequest, stage_service=Depends(get_stage_service)):
    """Reference a USD asset into the open stage at ``prim_path`` (keeps the stage)."""
    return await stage_service.add_to_scene(req.uri, req.prim_path)


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
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@stage.post("/configuration", summary="Import Configuration")
async def import_configuration():
    """Mirrored from the spec; not yet implemented."""
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
async def apply_relative_pose(
    req: ApplyRelativePoseRequest, prim_service=Depends(get_prim_service)
):
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
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@prims.get("/labels", summary="List Semantic Labels")
async def list_semantic_labels():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@prims.delete("/labels", summary="Clear Semantic Labels")
async def clear_semantic_labels():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@prims.put("/selected", summary="Select Prims")
async def select_prims():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@prims.get("/selected", summary="List Selected Prims")
async def list_selected_prims():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


# -- nucleus ----------------------------------------------------------------

nucleus = APIRouter(prefix="/nucleus", tags=["nucleus"])


@nucleus.post("/server", summary="Add Nucleus Server")
async def add_nucleus_server():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@nucleus.get("/servers", summary="List Nucleus Servers")
async def list_nucleus_servers():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@nucleus.post("/server/token", summary="Add Nucleus API Token")
async def add_nucleus_api_token():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@nucleus.delete("/server/token", summary="Remove Nucleus API Token")
async def remove_nucleus_api_token():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@nucleus.delete("/server/tokens", summary="Remove All Nucleus API Tokens")
async def remove_all_nucleus_api_tokens():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


# -- manipulators -----------------------------------------------------------

manipulators = APIRouter(prefix="/manipulators", tags=["manipulators"])


@manipulators.post("/motion-groups", summary="Create Motion Group")
async def create_motion_group():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@manipulators.get("/motion-groups", summary="List Motion Groups")
async def list_motion_groups():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@manipulators.get("/motion-groups/{prim_path:path}", summary="Get Motion Group")
async def get_motion_group(prim_path: str):
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@manipulators.put("/motion-groups/{prim_path:path}", summary="Update Motion Group Motion Stream")
async def update_motion_group_motion_stream(prim_path: str):
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@manipulators.delete("/motion-groups/{prim_path:path}", summary="Remove Motion Group")
async def remove_motion_group(prim_path: str):
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@manipulators.delete("/motion-groups", summary="Clear Motion Groups")
async def clear_motion_groups():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


# -- cameras (device registry, analogous to articulations) ------------------
#
# A camera is a registered device with an id, exactly like an articulation:
# PUT /cameras hands back a camera_id, then every other route addresses that id.
# (This replaces the old Wandelbots-mirrored /periphery stubs, which modelled a
# single "active camera" with no ids and did not fit the registry design.)

cameras = APIRouter(tags=["cameras"])


@cameras.put("/cameras", summary="Create Camera")
async def create_camera(req: CreateCameraRequest, camera_service=Depends(get_camera_service)):
    """Register (and bind) one camera; returns its id, prim, resolution, optics, and
    its active and supported data types."""
    return await camera_service.create_camera(req.prim_path, req.resolution, req.frequency)


@cameras.get("/cameras", summary="List Cameras")
async def list_cameras(camera_service=Depends(get_camera_service)):
    """Every registered camera id mapped to its prim path."""
    return camera_service.list_cameras()


@cameras.get("/cameras/{camera_id}", summary="Get Camera")
async def get_camera(camera_id: str, camera_service=Depends(get_camera_service)):
    """Id + description (prim, resolution, optics, active/supported data types). 404 if
    unknown."""
    return camera_service.get_camera(camera_id)


@cameras.delete("/cameras/{camera_id}", summary="Delete Camera")
async def delete_camera(camera_id: str, camera_service=Depends(get_camera_service)):
    """Unregister the camera (the USD prim is left in the stage)."""
    return camera_service.delete_camera(camera_id)


@cameras.post(
    "/cameras/{camera_id}/capture",
    summary="Capture Frame",
    response_class=Response,
    responses=binary.OCTET_STREAM_RESPONSES,
)
async def capture(camera_id: str, req: CaptureRequest, camera_service=Depends(get_camera_service)):
    """Pump one frame and return the requested outputs (all active ones if omitted)
    as one binary frame (see :mod:`.binary`).

    A captured pointcloud is in the camera frame unless ``world_frame`` is set."""
    frame = await camera_service.capture(camera_id, req.data_types, req.world_frame)
    return Response(binary.encode(frame), media_type=binary.MEDIA_TYPE)


@cameras.get(
    "/cameras/{camera_id}/rgb",
    summary="Get Rgb",
    response_class=Response,
    responses=binary.OCTET_STREAM_RESPONSES,
)
async def get_rgb(camera_id: str, camera_service=Depends(get_camera_service)):
    """Latest RGB image (H, W, 3) uint8, as a binary frame (see :mod:`.binary`); the
    output is null in the frame if it is not ready."""
    return Response(
        binary.encode(camera_service.get_rgb(camera_id)), media_type=binary.MEDIA_TYPE
    )


@cameras.get(
    "/cameras/{camera_id}/rgba",
    summary="Get Rgba",
    response_class=Response,
    responses=binary.OCTET_STREAM_RESPONSES,
)
async def get_rgba(camera_id: str, camera_service=Depends(get_camera_service)):
    """Latest RGBA image (H, W, 4) uint8, as a binary frame (see :mod:`.binary`)."""
    return Response(
        binary.encode(camera_service.get_rgba(camera_id)), media_type=binary.MEDIA_TYPE
    )


@cameras.get(
    "/cameras/{camera_id}/depth",
    summary="Get Depth",
    response_class=Response,
    responses=binary.OCTET_STREAM_RESPONSES,
)
async def get_depth(camera_id: str, camera_service=Depends(get_camera_service)):
    """Latest depth image (H, W) float32, as a binary frame (see :mod:`.binary`).
    Pixels that hit nothing are inf."""
    return Response(
        binary.encode(camera_service.get_depth(camera_id)), media_type=binary.MEDIA_TYPE
    )


@cameras.get(
    "/cameras/{camera_id}/pointcloud",
    summary="Get Pointcloud",
    response_class=Response,
    responses=binary.OCTET_STREAM_RESPONSES,
)
async def get_pointcloud(
    camera_id: str, world_frame: bool = Query(False), camera_service=Depends(get_camera_service)
):
    """Latest pointcloud (N, 3) in camera (default) or world frame, as a binary frame
    (see :mod:`.binary`)."""
    return Response(
        binary.encode(camera_service.get_pointcloud(camera_id, world_frame)),
        media_type=binary.MEDIA_TYPE,
    )


@cameras.get("/cameras/{camera_id}/world_pose", summary="Get World Pose")
async def get_camera_world_pose(
    camera_id: str, camera_axes: str = Query("world"), camera_service=Depends(get_camera_service)
):
    """World-frame pose {position, orientation}. camera_axes: world/ros/usd."""
    return camera_service.get_world_pose(camera_id, camera_axes)


@cameras.put("/cameras/{camera_id}/world_pose", summary="Set World Pose")
async def set_camera_world_pose(
    camera_id: str, req: CameraWorldPoseRequest, camera_service=Depends(get_camera_service)
):
    """Set the world-frame pose; returns the resulting pose."""
    return camera_service.set_world_pose(camera_id, req.position, req.orientation, req.camera_axes)


@cameras.get("/cameras/{camera_id}/local_pose", summary="Get Local Pose")
async def get_camera_local_pose(
    camera_id: str, camera_axes: str = Query("world"), camera_service=Depends(get_camera_service)
):
    """Local-frame (parent-relative) pose {translation, orientation}."""
    return camera_service.get_local_pose(camera_id, camera_axes)


@cameras.put("/cameras/{camera_id}/local_pose", summary="Set Local Pose")
async def set_camera_local_pose(
    camera_id: str, req: CameraLocalPoseRequest, camera_service=Depends(get_camera_service)
):
    """Set the local-frame pose; returns the resulting pose."""
    return camera_service.set_local_pose(
        camera_id, req.translation, req.orientation, req.camera_axes
    )


@cameras.get("/cameras/{camera_id}/resolution", summary="Get Resolution")
async def get_camera_resolution(camera_id: str, camera_service=Depends(get_camera_service)):
    """Current [width, height] in pixels."""
    return camera_service.get_resolution(camera_id)


@cameras.put("/cameras/{camera_id}/resolution", summary="Set Resolution")
async def set_camera_resolution(
    camera_id: str, req: CameraResolutionRequest, camera_service=Depends(get_camera_service)
):
    """Set [width, height] in pixels; returns the resulting resolution."""
    return camera_service.set_resolution(camera_id, req.width, req.height)


@cameras.get("/cameras/{camera_id}/focal_length", summary="Get Focal Length")
async def get_focal_length(camera_id: str, camera_service=Depends(get_camera_service)):
    """Focal length (stage units)."""
    return camera_service.get_focal_length(camera_id)


@cameras.put("/cameras/{camera_id}/focal_length", summary="Set Focal Length")
async def set_focal_length(
    camera_id: str, req: CameraFloatValueRequest, camera_service=Depends(get_camera_service)
):
    """Set the focal length (stage units)."""
    return camera_service.set_focal_length(camera_id, req.value)


@cameras.get("/cameras/{camera_id}/focus_distance", summary="Get Focus Distance")
async def get_focus_distance(camera_id: str, camera_service=Depends(get_camera_service)):
    """Distance from camera to focus plane (stage units)."""
    return camera_service.get_focus_distance(camera_id)


@cameras.put("/cameras/{camera_id}/focus_distance", summary="Set Focus Distance")
async def set_focus_distance(
    camera_id: str, req: CameraFloatValueRequest, camera_service=Depends(get_camera_service)
):
    """Set the focus distance (stage units)."""
    return camera_service.set_focus_distance(camera_id, req.value)


@cameras.get("/cameras/{camera_id}/lens_aperture", summary="Get Lens Aperture")
async def get_lens_aperture(camera_id: str, camera_service=Depends(get_camera_service)):
    """fStop value (0 disables depth-of-field)."""
    return camera_service.get_lens_aperture(camera_id)


@cameras.put("/cameras/{camera_id}/lens_aperture", summary="Set Lens Aperture")
async def set_lens_aperture(
    camera_id: str, req: CameraFloatValueRequest, camera_service=Depends(get_camera_service)
):
    """Set the fStop value."""
    return camera_service.set_lens_aperture(camera_id, req.value)


@cameras.get("/cameras/{camera_id}/horizontal_aperture", summary="Get Horizontal Aperture")
async def get_horizontal_aperture(camera_id: str, camera_service=Depends(get_camera_service)):
    """Horizontal aperture / sensor width (stage units)."""
    return camera_service.get_horizontal_aperture(camera_id)


@cameras.put("/cameras/{camera_id}/horizontal_aperture", summary="Set Horizontal Aperture")
async def set_horizontal_aperture(
    camera_id: str, req: CameraApertureRequest, camera_service=Depends(get_camera_service)
):
    """Set the horizontal aperture (stage units)."""
    return camera_service.set_horizontal_aperture(camera_id, req.value, req.maintain_square_pixels)


@cameras.get("/cameras/{camera_id}/vertical_aperture", summary="Get Vertical Aperture")
async def get_vertical_aperture(camera_id: str, camera_service=Depends(get_camera_service)):
    """Vertical aperture / sensor height (stage units)."""
    return camera_service.get_vertical_aperture(camera_id)


@cameras.put("/cameras/{camera_id}/vertical_aperture", summary="Set Vertical Aperture")
async def set_vertical_aperture(
    camera_id: str, req: CameraApertureRequest, camera_service=Depends(get_camera_service)
):
    """Set the vertical aperture (stage units)."""
    return camera_service.set_vertical_aperture(camera_id, req.value, req.maintain_square_pixels)


@cameras.get("/cameras/{camera_id}/clipping_range", summary="Get Clipping Range")
async def get_clipping_range(camera_id: str, camera_service=Depends(get_camera_service)):
    """[near, far] clipping distances (stage units)."""
    return camera_service.get_clipping_range(camera_id)


@cameras.put("/cameras/{camera_id}/clipping_range", summary="Set Clipping Range")
async def set_clipping_range(
    camera_id: str, req: CameraClippingRangeRequest, camera_service=Depends(get_camera_service)
):
    """Set near/far clipping distances; either may be null to leave unchanged."""
    return camera_service.set_clipping_range(camera_id, req.near_distance, req.far_distance)


@cameras.get("/cameras/{camera_id}/frequency", summary="Get Frequency")
async def get_camera_frequency(camera_id: str, camera_service=Depends(get_camera_service)):
    """Current acquisition frequency (Hz)."""
    return camera_service.get_frequency(camera_id)


@cameras.put("/cameras/{camera_id}/frequency", summary="Set Frequency")
async def set_camera_frequency(
    camera_id: str, req: CameraFloatValueRequest, camera_service=Depends(get_camera_service)
):
    """Set the acquisition frequency (Hz). Must divide the rendering frequency."""
    return camera_service.set_frequency(camera_id, req.value)


@cameras.get("/cameras/{camera_id}/projection_mode", summary="Get Projection Mode")
async def get_projection_mode(camera_id: str, camera_service=Depends(get_camera_service)):
    """perspective or orthographic."""
    return camera_service.get_projection_mode(camera_id)


@cameras.put("/cameras/{camera_id}/projection_mode", summary="Set Projection Mode")
async def set_projection_mode(
    camera_id: str, req: CameraStringValueRequest, camera_service=Depends(get_camera_service)
):
    """Set the projection mode (perspective/orthographic)."""
    return camera_service.set_projection_mode(camera_id, req.value)


@cameras.get("/cameras/{camera_id}/stereo_role", summary="Get Stereo Role")
async def get_stereo_role(camera_id: str, camera_service=Depends(get_camera_service)):
    """mono, left or right."""
    return camera_service.get_stereo_role(camera_id)


@cameras.put("/cameras/{camera_id}/stereo_role", summary="Set Stereo Role")
async def set_stereo_role(
    camera_id: str, req: CameraStringValueRequest, camera_service=Depends(get_camera_service)
):
    """Set the stereo role (mono/left/right)."""
    return camera_service.set_stereo_role(camera_id, req.value)


@cameras.get("/cameras/{camera_id}/lens_distortion_model", summary="Get Lens Distortion Model")
async def get_lens_distortion_model(camera_id: str, camera_service=Depends(get_camera_service)):
    """Lens distortion model name (pinhole if unset)."""
    return camera_service.get_lens_distortion_model(camera_id)


@cameras.put("/cameras/{camera_id}/lens_distortion_model", summary="Set Lens Distortion Model")
async def set_lens_distortion_model(
    camera_id: str, req: CameraStringValueRequest, camera_service=Depends(get_camera_service)
):
    """Set the lens distortion model (applies the matching schema)."""
    return camera_service.set_lens_distortion_model(camera_id, req.value)


@cameras.get("/cameras/{camera_id}/intrinsics_matrix", summary="Get Intrinsics Matrix")
async def get_intrinsics_matrix(camera_id: str, camera_service=Depends(get_camera_service)):
    """3x3 intrinsics matrix (pinhole models only)."""
    return camera_service.get_intrinsics_matrix(camera_id)


@cameras.get("/cameras/{camera_id}/fov", summary="Get Fov")
async def get_fov(camera_id: str, camera_service=Depends(get_camera_service)):
    """Horizontal and vertical field of view."""
    return camera_service.get_fov(camera_id)


@cameras.get("/cameras/{camera_id}/render_product_path", summary="Get Render Product Path")
async def get_render_product_path(camera_id: str, camera_service=Depends(get_camera_service)):
    """Path to the render product attached to this camera."""
    return camera_service.get_render_product_path(camera_id)


@cameras.get("/cameras/{camera_id}/supported_annotators", summary="Get Supported Annotators")
async def get_supported_annotators(camera_id: str, camera_service=Depends(get_camera_service)):
    """Annotator names that can be attached to this camera."""
    return camera_service.get_supported_annotators(camera_id)


@cameras.post("/cameras/{camera_id}/pause", summary="Pause Camera")
async def pause_camera(camera_id: str, camera_service=Depends(get_camera_service)):
    """Pause data collection / frame updates."""
    return camera_service.pause(camera_id)


@cameras.post("/cameras/{camera_id}/resume", summary="Resume Camera")
async def resume_camera(camera_id: str, camera_service=Depends(get_camera_service)):
    """Resume data collection / frame updates."""
    return camera_service.resume(camera_id)


@cameras.get("/cameras/{camera_id}/is_paused", summary="Get Is Paused")
async def is_paused(camera_id: str, camera_service=Depends(get_camera_service)):
    """Whether data collection is currently paused."""
    return camera_service.is_paused(camera_id)


# -- lidars (device registry, analogous to cameras) --------------------------
#
# A lidar is a registered device with an id, exactly like a camera: PUT /lidars
# hands back a lidar_id, then every other route addresses that id. Wraps the
# legacy PhysX Lidar sensor (isaacsim.sensors.physx._range_sensor).

lidars = APIRouter(tags=["lidars"])


@lidars.put("/lidars", summary="Create Lidar")
async def create_lidar(req: CreateLidarRequest, lidar_service=Depends(get_lidar_service)):
    """Register (and bind) one lidar; returns its id, prim, and scan configuration."""
    return await lidar_service.create_lidar(
        req.prim_path,
        req.min_range,
        req.max_range,
        req.horizontal_fov,
        req.vertical_fov,
        req.horizontal_resolution,
        req.vertical_resolution,
        req.rotation_rate,
        req.high_lod,
        req.draw_points,
        req.draw_lines,
        req.yaw_offset,
        req.data_types,
    )


@lidars.get("/lidars", summary="List Lidars")
async def list_lidars(lidar_service=Depends(get_lidar_service)):
    """Every registered lidar id mapped to its prim path."""
    return lidar_service.list_lidars()


@lidars.get("/lidars/{lidar_id}", summary="Get Lidar")
async def get_lidar(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Id + static description (prim, scan configuration). 404 if unknown."""
    return lidar_service.get_lidar(lidar_id)


@lidars.delete("/lidars/{lidar_id}", summary="Delete Lidar")
async def delete_lidar(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Unregister the lidar (the USD prim is left in the stage)."""
    return lidar_service.delete_lidar(lidar_id)


@lidars.post("/lidars/{lidar_id}/capture", summary="Capture Scan")
async def capture_lidar(
    lidar_id: str, req: LidarCaptureRequest, lidar_service=Depends(get_lidar_service)
):
    """Pump one frame and return the requested outputs (all bound types if omitted)."""
    return await lidar_service.capture(lidar_id, req.data_types)


@lidars.websocket("/lidars/{lidar_id}/stream_scan")
async def stream_scan(websocket: WebSocket, lidar_id: str):
    """Scan push stream: the client opens one connection bound to this lidar and
    the server sends one frame per simulator update, shaped exactly like the
    ``capture`` response (every bound data type plus ``num_cols_ticked`` and
    ``timestamp``). The client sends nothing.

    Unlike the articulation state stream, capturing a lidar frame already
    blocks on one ``next_update_async`` (see
    :meth:`..core.lidar.Lidar.capture`), so this handler paces itself off that
    call instead of pumping the app loop itself. A client that reads more
    slowly than the server sends misses frames rather than falling steadily
    further behind.
    """
    service = websocket.app.state.lidar_service
    await websocket.accept()
    try:
        service.get_device(lidar_id)
    except HTTPException as exc:
        await websocket.close(code=1008, reason=exc.detail)
        return

    async def send_scan_frames():
        while True:
            try:
                frame = await service.capture(lidar_id, None)
            except HTTPException as exc:
                # The device was deleted mid-stream (get_device now 404s) -- close, don't crash.
                await websocket.close(code=1008, reason=exc.detail)
                return
            try:
                await websocket.send_json(frame)
            except (TypeError, ValueError):
                # The frame itself could not be serialized -- a real fault worth
                # surfacing, not a connection problem.
                raise
            except Exception:  # pylint: disable=broad-except
                # The client disconnected between capturing and sending the
                # frame. Pushing to that socket is all this does, so stop.
                return

    async def watch_for_disconnect():
        # Nothing is expected on this connection, but something has to read from it:
        # a disconnect is delivered as a received message, so a send-only handler
        # would never notice the client had gone and would push frames forever.
        while True:
            message = await websocket.receive()
            if message["type"] == "websocket.disconnect":
                return

    await run_until_first_finished(send_scan_frames(), watch_for_disconnect())


@lidars.get("/lidars/{lidar_id}/depth", summary="Get Depth")
async def get_lidar_depth(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Latest quantized depth buffer (num_rows, num_cols), or null."""
    return lidar_service.get_depth_data(lidar_id)


@lidars.get("/lidars/{lidar_id}/linear_depth", summary="Get Linear Depth")
async def get_lidar_linear_depth(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Latest linear depth buffer in meters (num_rows, num_cols), or null."""
    return lidar_service.get_linear_depth_data(lidar_id)


@lidars.get("/lidars/{lidar_id}/intensity", summary="Get Intensity")
async def get_lidar_intensity(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Latest return-intensity buffer (num_rows, num_cols), or null."""
    return lidar_service.get_intensity_data(lidar_id)


@lidars.get("/lidars/{lidar_id}/zenith", summary="Get Zenith")
async def get_lidar_zenith(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Per-row vertical scan angles (radians), or null."""
    return lidar_service.get_zenith_data(lidar_id)


@lidars.get("/lidars/{lidar_id}/azimuth", summary="Get Azimuth")
async def get_lidar_azimuth(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Per-column horizontal scan angles (radians), or null."""
    return lidar_service.get_azimuth_data(lidar_id)


@lidars.get("/lidars/{lidar_id}/point_cloud", summary="Get Point Cloud")
async def get_lidar_point_cloud(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Latest hit points in world frame (N, 3), or null."""
    return lidar_service.get_point_cloud_data(lidar_id)


@lidars.get("/lidars/{lidar_id}/semantic", summary="Get Semantic")
async def get_lidar_semantic(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Per-hit semantic ids, or null. Requires enable_semantics."""
    return lidar_service.get_semantic_data(lidar_id)


@lidars.get("/lidars/{lidar_id}/world_pose", summary="Get World Pose")
async def get_lidar_world_pose(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """World-frame pose {position, orientation}."""
    return lidar_service.get_world_pose(lidar_id)


@lidars.put("/lidars/{lidar_id}/world_pose", summary="Set World Pose")
async def set_lidar_world_pose(
    lidar_id: str, req: LidarWorldPoseRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the world-frame pose; returns the resulting pose."""
    return lidar_service.set_world_pose(lidar_id, req.position, req.orientation)


@lidars.get("/lidars/{lidar_id}/local_pose", summary="Get Local Pose")
async def get_lidar_local_pose(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Local-frame (parent-relative) pose {translation, orientation}."""
    return lidar_service.get_local_pose(lidar_id)


@lidars.put("/lidars/{lidar_id}/local_pose", summary="Set Local Pose")
async def set_lidar_local_pose(
    lidar_id: str, req: LidarLocalPoseRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the local-frame pose; returns the resulting pose."""
    return lidar_service.set_local_pose(lidar_id, req.translation, req.orientation)


@lidars.get("/lidars/{lidar_id}/min_range", summary="Get Min Range")
async def get_lidar_min_range(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Minimum sensing range (stage units)."""
    return lidar_service.get_min_range(lidar_id)


@lidars.put("/lidars/{lidar_id}/min_range", summary="Set Min Range")
async def set_lidar_min_range(
    lidar_id: str, req: LidarFloatValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the minimum sensing range (stage units)."""
    return lidar_service.set_min_range(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/max_range", summary="Get Max Range")
async def get_lidar_max_range(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Maximum sensing range (stage units)."""
    return lidar_service.get_max_range(lidar_id)


@lidars.put("/lidars/{lidar_id}/max_range", summary="Set Max Range")
async def set_lidar_max_range(
    lidar_id: str, req: LidarFloatValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the maximum sensing range (stage units)."""
    return lidar_service.set_max_range(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/horizontal_fov", summary="Get Horizontal Fov")
async def get_lidar_horizontal_fov(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Horizontal field of view (degrees)."""
    return lidar_service.get_horizontal_fov(lidar_id)


@lidars.put("/lidars/{lidar_id}/horizontal_fov", summary="Set Horizontal Fov")
async def set_lidar_horizontal_fov(
    lidar_id: str, req: LidarFloatValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the horizontal field of view (degrees)."""
    return lidar_service.set_horizontal_fov(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/vertical_fov", summary="Get Vertical Fov")
async def get_lidar_vertical_fov(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Vertical field of view (degrees)."""
    return lidar_service.get_vertical_fov(lidar_id)


@lidars.put("/lidars/{lidar_id}/vertical_fov", summary="Set Vertical Fov")
async def set_lidar_vertical_fov(
    lidar_id: str, req: LidarFloatValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the vertical field of view (degrees)."""
    return lidar_service.set_vertical_fov(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/horizontal_resolution", summary="Get Horizontal Resolution")
async def get_lidar_horizontal_resolution(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Horizontal angular resolution (degrees per column)."""
    return lidar_service.get_horizontal_resolution(lidar_id)


@lidars.put("/lidars/{lidar_id}/horizontal_resolution", summary="Set Horizontal Resolution")
async def set_lidar_horizontal_resolution(
    lidar_id: str, req: LidarFloatValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the horizontal angular resolution (degrees per column)."""
    return lidar_service.set_horizontal_resolution(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/vertical_resolution", summary="Get Vertical Resolution")
async def get_lidar_vertical_resolution(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Vertical angular resolution (degrees per row)."""
    return lidar_service.get_vertical_resolution(lidar_id)


@lidars.put("/lidars/{lidar_id}/vertical_resolution", summary="Set Vertical Resolution")
async def set_lidar_vertical_resolution(
    lidar_id: str, req: LidarFloatValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the vertical angular resolution (degrees per row)."""
    return lidar_service.set_vertical_resolution(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/rotation_rate", summary="Get Rotation Rate")
async def get_lidar_rotation_rate(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Rotation rate (Hz); 0 means an instantaneous full-sweep lidar."""
    return lidar_service.get_rotation_rate(lidar_id)


@lidars.put("/lidars/{lidar_id}/rotation_rate", summary="Set Rotation Rate")
async def set_lidar_rotation_rate(
    lidar_id: str, req: LidarFloatValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the rotation rate (Hz)."""
    return lidar_service.set_rotation_rate(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/yaw_offset", summary="Get Yaw Offset")
async def get_lidar_yaw_offset(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Yaw offset applied to the scan pattern (degrees)."""
    return lidar_service.get_yaw_offset(lidar_id)


@lidars.put("/lidars/{lidar_id}/yaw_offset", summary="Set Yaw Offset")
async def set_lidar_yaw_offset(
    lidar_id: str, req: LidarFloatValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set the yaw offset (degrees)."""
    return lidar_service.set_yaw_offset(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/high_lod", summary="Get High Lod")
async def get_lidar_high_lod(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Whether the sensor renders at high level-of-detail."""
    return lidar_service.get_high_lod(lidar_id)


@lidars.put("/lidars/{lidar_id}/high_lod", summary="Set High Lod")
async def set_lidar_high_lod(
    lidar_id: str, req: LidarBoolValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set whether the sensor renders at high level-of-detail."""
    return lidar_service.set_high_lod(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/draw_points", summary="Get Draw Points")
async def get_lidar_draw_points(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Whether hit points are drawn in the viewport."""
    return lidar_service.get_draw_points(lidar_id)


@lidars.put("/lidars/{lidar_id}/draw_points", summary="Set Draw Points")
async def set_lidar_draw_points(
    lidar_id: str, req: LidarBoolValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set whether hit points are drawn in the viewport."""
    return lidar_service.set_draw_points(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/draw_lines", summary="Get Draw Lines")
async def get_lidar_draw_lines(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Whether scan rays are drawn in the viewport."""
    return lidar_service.get_draw_lines(lidar_id)


@lidars.put("/lidars/{lidar_id}/draw_lines", summary="Set Draw Lines")
async def set_lidar_draw_lines(
    lidar_id: str, req: LidarBoolValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set whether scan rays are drawn in the viewport."""
    return lidar_service.set_draw_lines(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/enable_semantics", summary="Get Enable Semantics")
async def get_lidar_enable_semantics(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Whether per-hit semantic labels are captured."""
    return lidar_service.get_enable_semantics(lidar_id)


@lidars.put("/lidars/{lidar_id}/enable_semantics", summary="Set Enable Semantics")
async def set_lidar_enable_semantics(
    lidar_id: str, req: LidarBoolValueRequest, lidar_service=Depends(get_lidar_service)
):
    """Set whether per-hit semantic labels are captured."""
    return lidar_service.set_enable_semantics(lidar_id, req.value)


@lidars.get("/lidars/{lidar_id}/num_rows", summary="Get Num Rows")
async def get_lidar_num_rows(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Number of scan rows (vertical channels) the sensor currently reports."""
    return lidar_service.get_num_rows(lidar_id)


@lidars.get("/lidars/{lidar_id}/num_cols", summary="Get Num Cols")
async def get_lidar_num_cols(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Number of scan columns (horizontal samples) a full sweep produces."""
    return lidar_service.get_num_cols(lidar_id)


@lidars.get("/lidars/{lidar_id}/num_cols_ticked", summary="Get Num Cols Ticked")
async def get_lidar_num_cols_ticked(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Number of scan columns completed so far this physics step."""
    return lidar_service.get_num_cols_ticked(lidar_id)


@lidars.get("/lidars/{lidar_id}/azimuth_range", summary="Get Azimuth Range")
async def get_lidar_azimuth_range(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """[min, max] horizontal scan angles (radians)."""
    return lidar_service.get_azimuth_range(lidar_id)


@lidars.get("/lidars/{lidar_id}/zenith_range", summary="Get Zenith Range")
async def get_lidar_zenith_range(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """[min, max] vertical scan angles (radians)."""
    return lidar_service.get_zenith_range(lidar_id)


@lidars.get("/lidars/{lidar_id}/is_lidar_sensor", summary="Get Is Lidar Sensor")
async def get_is_lidar_sensor(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Whether the registered prim currently resolves to a live PhysX lidar sensor."""
    return lidar_service.is_lidar_sensor(lidar_id)


@lidars.post("/lidars/{lidar_id}/pause", summary="Pause Lidar")
async def pause_lidar(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Pause sensor computation."""
    return lidar_service.pause(lidar_id)


@lidars.post("/lidars/{lidar_id}/resume", summary="Resume Lidar")
async def resume_lidar(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Resume sensor computation."""
    return lidar_service.resume(lidar_id)


@lidars.get("/lidars/{lidar_id}/is_paused", summary="Get Is Paused")
async def is_lidar_paused(lidar_id: str, lidar_service=Depends(get_lidar_service)):
    """Whether sensor computation is currently paused."""
    return lidar_service.is_paused(lidar_id)


# -- conveyors (device registry, analogous to surface grippers) -------------
#
# A conveyor is a registered device with an id, exactly like a camera or a
# gripper: PUT /conveyors hands back a conveyor_id, then every other route
# addresses that id.
#
# It is neither an articulation nor a sensor: there is nothing to drive and
# nothing to read, so instead of move_j or capture it has start and stop. A belt
# runs at a signed speed along the travel direction its scene authored, so
# reversing one means starting it with a negative velocity.

conveyors = APIRouter(tags=["conveyors"])


@conveyors.put("/conveyors", summary="Create Conveyor")
async def create_conveyor(
    req: CreateConveyorRequest, conveyor_service=Depends(get_conveyor_service)
):
    """Register (and bind) one conveyor belt; returns its id, prims, drive and speed.

    Binding plays the timeline, because a belt only carries cargo while physics steps.
    """
    return await conveyor_service.create_conveyor(req.prim_path, req.cargo_root)


@conveyors.get("/conveyors", summary="List Conveyors")
async def list_conveyors(conveyor_service=Depends(get_conveyor_service)):
    """Every registered conveyor id mapped to its prim path."""
    return conveyor_service.list_conveyors()


@conveyors.get("/conveyors/{conveyor_id}", summary="Get Conveyor")
async def get_conveyor(conveyor_id: str, conveyor_service=Depends(get_conveyor_service)):
    """Id + description (prims, drive, authored direction and speed, current speed,
    whether it is running). 404 if unknown."""
    return conveyor_service.get_conveyor(conveyor_id)


@conveyors.delete("/conveyors/{conveyor_id}", summary="Delete Conveyor")
async def delete_conveyor(conveyor_id: str, conveyor_service=Depends(get_conveyor_service)):
    """Unregister the conveyor (the USD prim is left in the stage, and a running belt
    keeps running -- stop it first)."""
    return conveyor_service.delete_conveyor(conveyor_id)


@conveyors.post("/conveyors/{conveyor_id}/start", summary="Start Conveyor")
async def start_conveyor(
    conveyor_id: str,
    req: ConveyorVelocityRequest,
    conveyor_service=Depends(get_conveyor_service),
):
    """Run the belt at a signed speed (its authored speed if omitted) and wake its cargo.

    Requires the timeline to be playing to have any effect: a surface velocity only
    moves anything while physics steps, and sleeping cargo can only be woken while the
    simulation is running.
    """
    return conveyor_service.start(conveyor_id, req.velocity)


@conveyors.post("/conveyors/{conveyor_id}/stop", summary="Stop Conveyor")
async def stop_conveyor(conveyor_id: str, conveyor_service=Depends(get_conveyor_service)):
    """Stop the belt, leaving the running velocity its scene authored on the stage."""
    return conveyor_service.stop(conveyor_id)


# -- lightbeams (device registry, analogous to lidars) ----------------------
#
# A lightbeam sensor is a registered device with an id, exactly like a lidar:
# PUT /lightbeams hands back a lightbeam_id, then every other route addresses
# that id. Wraps the other legacy PhysX range sensor
# (isaacsim.sensors.physx._range_sensor), so it reads like a lidar with a handful
# of rays instead of a sweep.

lightbeams = APIRouter(tags=["lightbeams"])


@lightbeams.put("/lightbeams", summary="Create Lightbeam")
async def create_lightbeam(
    req: CreateLightBeamRequest, lightbeam_service=Depends(get_lightbeam_service)
):
    """Register (and bind) one lightbeam sensor; returns its id, prim, layout and range."""
    return await lightbeam_service.create_lightbeam(req.prim_path)


@lightbeams.get("/lightbeams", summary="List Lightbeams")
async def list_lightbeams(lightbeam_service=Depends(get_lightbeam_service)):
    """Every registered lightbeam id mapped to its prim path."""
    return lightbeam_service.list_lightbeams()


@lightbeams.get("/lightbeams/{lightbeam_id}", summary="Get Lightbeam")
async def get_lightbeam(lightbeam_id: str, lightbeam_service=Depends(get_lightbeam_service)):
    """Id + static description (prim, beam layout, range). 404 if unknown."""
    return lightbeam_service.get_lightbeam(lightbeam_id)


@lightbeams.delete("/lightbeams/{lightbeam_id}", summary="Delete Lightbeam")
async def delete_lightbeam(
    lightbeam_id: str, lightbeam_service=Depends(get_lightbeam_service)
):
    """Unregister the sensor (the USD prim is left in the stage)."""
    return lightbeam_service.delete_lightbeam(lightbeam_id)


@lightbeams.get("/lightbeams/{lightbeam_id}/reading", summary="Read Lightbeam")
async def read_lightbeam(lightbeam_id: str, lightbeam_service=Depends(get_lightbeam_service)):
    """The beams as of the last physics step: num_rays, broken, beam_hit, linear_depth
    and hit_pos.

    ``broken`` is true when any beam is broken -- the whole output of the photoelectric
    switch this sensor stands in for. ``linear_depth`` is in meters and reads back as
    the sensor's max_range for a beam that is not broken; ``hit_pos`` is in the sensor's
    own frame. Every per-beam field is null until the sensor has produced its first
    reading. 409 while the timeline is stopped, since the buffers then hold a stale
    reading that looks like a clear beam.
    """
    return lightbeam_service.read(lightbeam_id)


@lightbeams.patch("/lightbeams/{lightbeam_id}/configuration", summary="Set Configuration")
async def set_lightbeam_configuration(
    lightbeam_id: str,
    req: LightBeamConfigurationRequest,
    lightbeam_service=Depends(get_lightbeam_service),
):
    """Set the beam layout and range (fields left null are untouched); returns them.

    Takes effect on the next physics step, including while the timeline plays: the
    sensor's component re-reads all of it on change.
    """
    return lightbeam_service.set_configuration(
        lightbeam_id,
        req.num_rays,
        req.curtain_length,
        req.forward_axis,
        req.curtain_axis,
        req.min_range,
        req.max_range,
    )


@lightbeams.post("/lightbeams/{lightbeam_id}/pause", summary="Pause Lightbeam")
async def pause_lightbeam(lightbeam_id: str, lightbeam_service=Depends(get_lightbeam_service)):
    """Pause sensor computation."""
    return lightbeam_service.pause(lightbeam_id)


@lightbeams.post("/lightbeams/{lightbeam_id}/resume", summary="Resume Lightbeam")
async def resume_lightbeam(
    lightbeam_id: str, lightbeam_service=Depends(get_lightbeam_service)
):
    """Resume sensor computation."""
    return lightbeam_service.resume(lightbeam_id)


@lightbeams.get("/lightbeams/{lightbeam_id}/is_paused", summary="Get Is Paused")
async def is_lightbeam_paused(
    lightbeam_id: str, lightbeam_service=Depends(get_lightbeam_service)
):
    """Whether sensor computation is currently paused."""
    return lightbeam_service.is_paused(lightbeam_id)


# -- surface (suction) grippers ---------------------------------------------
#
# A suction gripper is a registered device with an id, exactly like an articulation
# or a camera: PUT /surface_grippers hands back a surface_gripper_id, then every
# other route addresses that id -- including
# /articulations/{arm}/assemble_robot, which takes it as its gripper_id.
#
# It is not an articulation: there are no joints to drive, so instead of move_j it
# has close and open, and instead of joint limits and gains it has grip properties
# and attachment points.

surface_grippers = APIRouter(tags=["surface_grippers"])


@surface_grippers.put("/surface_grippers", summary="Create Surface Gripper")
async def create_surface_gripper(
    req: CreateSurfaceGripperRequest,
    surface_gripper_service=Depends(get_surface_gripper_service),
):
    """Register (and bind) one suction gripper; returns its id, prims and state."""
    return await surface_gripper_service.create_surface_gripper(req.prim_path, req.usd_path)


@surface_grippers.get("/surface_grippers", summary="List Surface Grippers")
async def list_surface_grippers(surface_gripper_service=Depends(get_surface_gripper_service)):
    """Every registered surface gripper id mapped to its prim path."""
    return surface_gripper_service.list_surface_grippers()


@surface_grippers.get("/surface_grippers/{surface_gripper_id}", summary="Get Surface Gripper")
async def get_surface_gripper(
    surface_gripper_id: str, surface_gripper_service=Depends(get_surface_gripper_service)
):
    """Id + description (prims, attachment points, properties, state). 404 if unknown."""
    return surface_gripper_service.get_surface_gripper(surface_gripper_id)


@surface_grippers.delete("/surface_grippers/{surface_gripper_id}", summary="Delete Surface Gripper")
async def delete_surface_gripper(
    surface_gripper_id: str, surface_gripper_service=Depends(get_surface_gripper_service)
):
    """Unregister the surface gripper (the USD prim is left in the stage)."""
    return surface_gripper_service.delete_surface_gripper(surface_gripper_id)


@surface_grippers.post("/surface_grippers/{surface_gripper_id}/close", summary="Close Gripper")
async def close_gripper(
    surface_gripper_id: str,
    req: GripperActionRequest,
    surface_gripper_service=Depends(get_surface_gripper_service),
):
    """Grip whatever the gripper's attachment points can reach; blocks unless asynchronous."""
    return await surface_gripper_service.close_gripper(surface_gripper_id, req.asynchronous)


@surface_grippers.post("/surface_grippers/{surface_gripper_id}/open", summary="Open Gripper")
async def open_gripper(
    surface_gripper_id: str,
    req: GripperActionRequest,
    surface_gripper_service=Depends(get_surface_gripper_service),
):
    """Release everything the gripper holds; blocks unless asynchronous."""
    return await surface_gripper_service.open_gripper(surface_gripper_id, req.asynchronous)


@surface_grippers.get("/surface_grippers/{surface_gripper_id}/status", summary="Get Gripper Status")
async def get_gripper_status(
    surface_gripper_id: str, surface_gripper_service=Depends(get_surface_gripper_service)
):
    """Current status (Open/Closing/Closed), gripped objects and grip distance."""
    return surface_gripper_service.get_status(surface_gripper_id)


@surface_grippers.get("/surface_grippers/{surface_gripper_id}/properties", summary="Get Properties")
async def get_surface_gripper_properties(
    surface_gripper_id: str, surface_gripper_service=Depends(get_surface_gripper_service)
):
    """The gripper's grip-behaviour properties (force limits, reach, retry, forward axis)."""
    return surface_gripper_service.get_properties(surface_gripper_id)


@surface_grippers.patch(
    "/surface_grippers/{surface_gripper_id}/properties", summary="Set Properties"
)
async def set_surface_gripper_properties(
    surface_gripper_id: str,
    req: SurfaceGripperPropertiesRequest,
    surface_gripper_service=Depends(get_surface_gripper_service),
):
    """Set the grip-behaviour properties (fields left null are untouched); returns them."""
    return surface_gripper_service.set_properties(
        surface_gripper_id,
        req.coaxial_force_limit,
        req.shear_force_limit,
        req.max_grip_distance,
        req.retry_interval,
        req.forward_axis,
        req.rotation_limits,
        req.translation_limits,
    )


@surface_grippers.get(
    "/surface_grippers/{surface_gripper_id}/attachment_points", summary="Get Attachment Points"
)
async def get_attachment_points(
    surface_gripper_id: str, surface_gripper_service=Depends(get_surface_gripper_service)
):
    """Per-attachment-point properties (bodies, local poses, drive, limits, clearance)."""
    return surface_gripper_service.get_attachment_points(surface_gripper_id)


@surface_grippers.patch(
    "/surface_grippers/{surface_gripper_id}/attachment_points",
    summary="Set Attachment Point Properties",
)
async def set_attachment_point_properties(
    surface_gripper_id: str,
    req: AttachmentPointPropertiesRequest,
    surface_gripper_service=Depends(get_surface_gripper_service),
):
    """Set properties on the selected attachment points (all of them by default)."""
    return surface_gripper_service.set_attachment_point_properties(
        surface_gripper_id,
        req.joint_paths,
        req.local_pose_0,
        req.local_pose_1,
        req.z_axis_translation_drive_stiffness,
        req.z_axis_translation_drive_damping,
        req.rotation_limits,
        req.translation_limits,
        req.clearance_offset,
        req.forward_axis,
    )


# -- trajectories -----------------------------------------------------------

trajectories = APIRouter(prefix="/trajectories", tags=["trajectories"])


@trajectories.post("/", summary="Create Trajectory")
async def create_trajectory():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@trajectories.get("/", summary="List Trajectories")
async def list_trajectories():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@trajectories.patch("/{name}", summary="Update Trajectory")
async def update_trajectory(name: str):
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@trajectories.delete("/{name}", summary="Remove Trajectory")
async def remove_trajectory(name: str):
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@trajectories.post("/{name}/markers", summary="Create Trajectory Markers")
async def create_trajectory_markers(name: str):
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@trajectories.delete("/{name}/markers", summary="Remove Trajectory Markers")
async def remove_trajectory_markers(name: str):
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


# -- teaching ---------------------------------------------------------------

teaching = APIRouter(prefix="/teaching", tags=["teaching"])


@teaching.get("/ghost-objects/sources", summary="List Ghost Object Sources")
async def list_ghost_object_sources():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@teaching.get("/tcps/sources", summary="List TCP Sources")
async def list_tcp_sources():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@teaching.post("/ghost-objects", summary="Create Ghost Object")
async def create_ghost_object():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@teaching.delete("/ghost-objects", summary="Clear Ghost Objects")
async def clear_ghost_objects():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@teaching.get("/ghost-objects", summary="List Ghost Objects")
async def list_ghost_objects():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@teaching.get("/ghost-objects/export", summary="Export Ghost Objects")
async def export_ghost_objects():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


# -- trajectory planner -----------------------------------------------------

trajectory_planner = APIRouter(prefix="/trajectory-planner", tags=["trajectory-planner"])


@trajectory_planner.get("/export", summary="Export Trajectory Plans")
async def export_trajectory_plans():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


@trajectory_planner.get("/{skill_name}/export", summary="Export Trajectory Plan Skill")
async def export_trajectory_plan_skill(skill_name: str):
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


# -- overlays ---------------------------------------------------------------

overlays = APIRouter(prefix="/overlays", tags=["overlays"])


@overlays.put("/robot/visibility", summary="Set Robot Overlay Visibility")
async def set_robot_overlay_visibility():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


# -- physics / collision world ----------------------------------------------

physics = APIRouter(prefix="/physics", tags=["physics"])


@physics.post("/collision/sweep", summary="Sweep Collisions")
async def sweep_collisions():
    """Mirrored from the spec; not yet implemented."""
    not_implemented()


ALL_ROUTERS = (
    articulations,
    surface_grippers,
    cameras,
    lidars,
    conveyors,
    lightbeams,
    general,
    stage,
    prims,
    nucleus,
    manipulators,
    trajectories,
    teaching,
    trajectory_planner,
    overlays,
    physics,
)
