"""Wandelbots NOVA - Isaac Sim Extension API.

Stub implementation of the Omniservice REST API.

Routes mirror the official spec documented at:
https://wandelbotsgmbh.github.io/wandelbots-isaacsim-extension/

Every endpoint is registered with the correct method, path and path
parameters, but the body is not implemented yet. Each handler raises a
``501 Not Implemented`` so the route surfaces in the OpenAPI/Swagger UI
(``/docs``) while making it explicit that no logic is wired up.
"""

from fastapi import APIRouter, FastAPI, HTTPException, status

app = FastAPI(
    title="Wandelbots NOVA - Isaac Sim Extension API",
    description=(
        "This extension enables a seamless connection between NVIDIA Isaac "
        "Sim and Wandelbots NOVA. These are stub routes."
    ),
)


def not_implemented() -> None:
    """Raise a uniform 501 for every stubbed endpoint."""
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="Not implemented",
    )


# ---------------------------------------------------------------------------
# Stage
# ---------------------------------------------------------------------------
stage = APIRouter(prefix="/stage", tags=["Stage"])


@stage.get("/scene", summary="Get Active Scene")
async def get_active_scene():
    not_implemented()


@stage.put("/scene", summary="Open Scene")
async def open_scene():
    not_implemented()


@stage.get("/motion-groups", summary="List Stage Motion Groups")
async def list_stage_motion_groups():
    not_implemented()


@stage.get("/units", summary="Get Stage Units")
async def get_stage_units():
    not_implemented()


@stage.put("/units", summary="Update Stage Units")
async def update_stage_units():
    not_implemented()


@stage.patch("/simulation/timeline/{action}", summary="Timeline Action")
async def timeline_action(action: str):
    not_implemented()


@stage.get("/simulation", summary="Simulation State")
async def simulation_state():
    not_implemented()


@stage.get("/configuration", summary="Export Configuration")
async def export_configuration():
    not_implemented()


@stage.post("/configuration", summary="Import Configuration")
async def import_configuration():
    not_implemented()


# ---------------------------------------------------------------------------
# Prims
# ---------------------------------------------------------------------------
prims = APIRouter(prefix="/prims", tags=["Prims"])


@prims.get("/poses", summary="Get Pose")
async def get_pose():
    not_implemented()


@prims.put("/poses", summary="Update Pose")
async def update_pose():
    not_implemented()


@prims.get("/poses/relative", summary="Get Relative Pose")
async def get_relative_pose():
    not_implemented()


@prims.post("/poses/relative", summary="Apply Relative Pose")
async def apply_relative_pose():
    not_implemented()


@prims.put("/labels", summary="Set Semantic Label")
async def set_semantic_label():
    not_implemented()


@prims.get("/labels", summary="List Semantic Labels")
async def list_semantic_labels():
    not_implemented()


@prims.delete("/labels", summary="Clear Semantic Labels")
async def clear_semantic_labels():
    not_implemented()


@prims.get("/poses/default", summary="List Default Poses")
async def list_default_poses():
    not_implemented()


@prims.put("/poses/default", summary="Assign Default Poses")
async def assign_default_poses():
    not_implemented()


@prims.delete("/poses/default", summary="Clear Default Poses")
async def clear_default_poses():
    not_implemented()


@prims.post("/poses/default/reset", summary="Reset Prim Poses To Default")
async def reset_to_default_poses():
    not_implemented()


@prims.put("/metadata", summary="Set Prim Metadata")
async def set_prim_metadata():
    not_implemented()


@prims.delete("/metadata", summary="Remove Prim Metadata")
async def remove_prim_metadata():
    not_implemented()


@prims.patch("/visibility", summary="Set Prim Visibility")
async def set_prim_visibility():
    not_implemented()


@prims.get("/selected", summary="List Selected Prims")
async def list_selected_prims():
    not_implemented()


@prims.put("/selected", summary="Select Prims")
async def select_prims():
    not_implemented()


@prims.patch("/physics/joints", summary="Set Joints")
async def set_joint_state():
    not_implemented()


@prims.patch("/physics/colliders/", summary="Update Colliders")
async def update_colliders():
    not_implemented()


# ---------------------------------------------------------------------------
# Periphery (Camera)
# ---------------------------------------------------------------------------
periphery = APIRouter(prefix="/periphery/cameras", tags=["Periphery (Camera)"])


@periphery.get("/prims", summary="List Camera Prims")
async def list_camera_prims():
    not_implemented()


@periphery.get("/active", summary="Get Active Camera")
async def get_active_camera():
    not_implemented()


@periphery.put("/active", summary="Set Active Camera")
async def set_active_camera():
    not_implemented()


@periphery.get("/capture/color", summary="Capture Color Image")
async def capture_color_image():
    not_implemented()


@periphery.get("/capture/normals", summary="Capture Normals Image")
async def capture_normals_image():
    not_implemented()


@periphery.get("/capture/depth", summary="Capture Depth Image")
async def capture_depth_image():
    not_implemented()


@periphery.get("/capture/pointcloud", summary="Capture Pointcloud")
async def capture_pointcloud():
    not_implemented()


@periphery.get("/capture/bounding-box-2d", summary="Capture Boundingbox 2D")
async def capture_boundingbox_2d():
    not_implemented()


@periphery.get("/capture/bounding-box-3d", summary="Capture Boundingbox 3D")
async def capture_boundingbox_3d():
    not_implemented()


@periphery.get("/capture/instance-segmentation", summary="Capture Instance Segmentation")
async def capture_instance_segmentation():
    not_implemented()


@periphery.get("/capture/semantic-segmentation", summary="Capture Semantic Segmentation")
async def capture_semantic_segmentation():
    not_implemented()


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------
ui = APIRouter(prefix="/ui", tags=["UI"])


@ui.get("/visibility", summary="Get Visibility")
async def get_visibility():
    not_implemented()


@ui.patch("/visibility", summary="Set Visibility")
async def set_visibility():
    not_implemented()


# ---------------------------------------------------------------------------
# Teaching
# ---------------------------------------------------------------------------
teaching = APIRouter(prefix="/teaching", tags=["Teaching"])


@teaching.get("/ghost-objects/sources", summary="List Ghost Object Sources")
async def list_ghost_object_sources():
    not_implemented()


@teaching.get("/tcps/sources", summary="List Tcp Sources")
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


# ---------------------------------------------------------------------------
# Trajectory Planner
# ---------------------------------------------------------------------------
trajectory_planner = APIRouter(prefix="/trajectory-planner", tags=["Trajectory Planner"])


@trajectory_planner.get("/export", summary="Export Trajectory Plans")
async def export_trajectory_plans():
    not_implemented()


@trajectory_planner.get("/{skill_name}/export", summary="Export Trajectory Plan Skill")
async def export_trajectory_plan_skill(skill_name: str):
    not_implemented()


# ---------------------------------------------------------------------------
# Manipulators (Motion-Group)
# ---------------------------------------------------------------------------
manipulators = APIRouter(prefix="/manipulators", tags=["Manipulators (Motion-Group)"])


@manipulators.get("/motion-groups", summary="List Motion Groups")
async def list_motion_groups():
    not_implemented()


@manipulators.post("/motion-groups", summary="Create Motion Group")
async def create_motion_group():
    not_implemented()


@manipulators.delete("/motion-groups", summary="Clear Motion Groups")
async def clear_motion_groups():
    not_implemented()


@manipulators.put("/motion-groups/{prim_path}", summary="Update Motion Group Motion Stream")
async def update_motion_group_stream(prim_path: str):
    not_implemented()


@manipulators.get("/motion-groups/{prim_path}", summary="Get Motion Group")
async def get_motion_group(prim_path: str):
    not_implemented()


@manipulators.delete("/motion-groups/{prim_path}", summary="Remove Motion Group")
async def remove_motion_group(prim_path: str):
    not_implemented()


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------
trajectories = APIRouter(prefix="/trajectories", tags=["Trajectory"])


@trajectories.get("/", summary="List Trajectories")
async def list_trajectories():
    not_implemented()


@trajectories.post("/", summary="Create Trajectory")
async def create_trajectory():
    not_implemented()


@trajectories.patch("/{name}", summary="Update Trajectory")
async def update_trajectory(name: str):
    not_implemented()


@trajectories.delete("/{name}", summary="Remove Trajectory")
async def remove_trajectory(name: str):
    not_implemented()


@trajectories.post("/{name}/markers", summary="Create Markers")
async def create_trajectory_markers(name: str):
    not_implemented()


@trajectories.delete("/{name}/markers", summary="Remove Markers")
async def remove_trajectory_markers(name: str):
    not_implemented()


# ---------------------------------------------------------------------------
# Collision World
# ---------------------------------------------------------------------------
physics = APIRouter(prefix="/physics", tags=["Collision World"])


@physics.post("/collision/sweep", summary="Sweep Collisions")
async def sweep_collisions():
    not_implemented()


# ---------------------------------------------------------------------------
# Robot Overlay (Experimental)
# ---------------------------------------------------------------------------
overlays = APIRouter(prefix="/overlays", tags=["Robot Overlay (Experimental)"])


@overlays.put("/robot/visibility", summary="Set Robot Overlay Visibility")
async def set_robot_overlay_visibility():
    not_implemented()


# ---------------------------------------------------------------------------
# Nucleus
# ---------------------------------------------------------------------------
nucleus = APIRouter(prefix="/nucleus", tags=["Nucleus"])


@nucleus.post("/server", summary="Add Nucleus Server")
async def add_nucleus_server():
    not_implemented()


@nucleus.get("/servers", summary="List Nucleus Servers")
async def list_nucleus_servers():
    not_implemented()


@nucleus.post("/server/token", summary="Add Nucleus Api Token")
async def add_nucleus_api_token():
    not_implemented()


@nucleus.delete("/server/token", summary="Remove Nucleus Api Token")
async def remove_nucleus_api_token():
    not_implemented()


@nucleus.delete("/server/tokens", summary="Remove All Nucleus Api Tokens")
async def remove_all_nucleus_api_tokens():
    not_implemented()


# ---------------------------------------------------------------------------
# Misc / top-level
# ---------------------------------------------------------------------------
misc = APIRouter(tags=["default"])


@misc.get("/status", summary="Get Status")
async def get_status():
    not_implemented()


@misc.get("/version", summary="Get Versions")
async def get_versions():
    not_implemented()


@misc.post("/auth/token", summary="Authenticate")
async def authenticate():
    not_implemented()


# ---------------------------------------------------------------------------
# Register routers
# ---------------------------------------------------------------------------
app.include_router(stage)
app.include_router(prims)
app.include_router(periphery)
app.include_router(ui)
app.include_router(teaching)
app.include_router(trajectory_planner)
app.include_router(manipulators)
app.include_router(trajectories)
app.include_router(physics)
app.include_router(overlays)
app.include_router(nucleus)
app.include_router(misc)
