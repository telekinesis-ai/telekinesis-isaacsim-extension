"""Example client for the Wandelbots NOVA - Isaac Sim Extension API.

One function per route, matching the endpoints in ``main.py`` and the spec at
https://wandelbotsgmbh.github.io/wandelbots-isaacsim-extension/

These are examples of *how to reach a given route* - they only build the
request (method, path, path params, query params, body) and return the decoded
response. No business logic.

Every function takes ``base`` as its first argument, e.g.::

    import client

    base = "http://localhost:8080"
    print(client.get_status(base))
    print(client.list_motion_groups(base))
    client.open_scene(base, {"path": "/World/my_scene.usd"})

Requires the ``requests`` package (``pip install requests``).
"""

import requests

DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, params=None, body=None):
    """Send one HTTP request and return the decoded response.

    Returns parsed JSON when the response is JSON, otherwise the raw bytes
    (e.g. captured images). Raises ``requests.HTTPError`` on a 4xx/5xx.
    """
    response = requests.request(
        method,
        base.rstrip("/") + path,
        params=params,
        json=body,
        timeout=DEFAULT_TIMEOUT,
    )
    response.raise_for_status()
    if response.headers.get("content-type", "").startswith("application/json"):
        return response.json()
    return response.content


# ======================================================================
# Stage
# ======================================================================
def get_active_scene(base):
    return _request(base, "GET", "/stage/scene")


def open_scene(base, body):
    return _request(base, "PUT", "/stage/scene", body=body)


def list_stage_motion_groups(base):
    return _request(base, "GET", "/stage/motion-groups")


def get_stage_units(base):
    return _request(base, "GET", "/stage/units")


def update_stage_units(base, body):
    return _request(base, "PUT", "/stage/units", body=body)


def timeline_action(base, action):
    # action is typically one of: play, pause, stop
    return _request(base, "PATCH", f"/stage/simulation/timeline/{action}")


def simulation_state(base):
    return _request(base, "GET", "/stage/simulation")


def export_configuration(base):
    return _request(base, "GET", "/stage/configuration")


def import_configuration(base, body):
    return _request(base, "POST", "/stage/configuration", body=body)


# ======================================================================
# Prims
# ======================================================================
def get_pose(base, prim_path):
    return _request(base, "GET", "/prims/poses", params={"prim_path": prim_path})


def update_pose(base, body):
    return _request(base, "PUT", "/prims/poses", body=body)


def get_relative_pose(base, params):
    return _request(base, "GET", "/prims/poses/relative", params=params)


def apply_relative_pose(base, body):
    return _request(base, "POST", "/prims/poses/relative", body=body)


def set_semantic_label(base, body):
    return _request(base, "PUT", "/prims/labels", body=body)


def list_semantic_labels(base):
    return _request(base, "GET", "/prims/labels")


def clear_semantic_labels(base):
    return _request(base, "DELETE", "/prims/labels")


def list_default_poses(base):
    return _request(base, "GET", "/prims/poses/default")


def assign_default_poses(base, body):
    return _request(base, "PUT", "/prims/poses/default", body=body)


def clear_default_poses(base):
    return _request(base, "DELETE", "/prims/poses/default")


def reset_to_default_poses(base, body=None):
    return _request(base, "POST", "/prims/poses/default/reset", body=body)


def set_prim_metadata(base, body):
    return _request(base, "PUT", "/prims/metadata", body=body)


def remove_prim_metadata(base, body):
    return _request(base, "DELETE", "/prims/metadata", body=body)


def set_prim_visibility(base, body):
    return _request(base, "PATCH", "/prims/visibility", body=body)


def list_selected_prims(base):
    return _request(base, "GET", "/prims/selected")


def select_prims(base, body):
    return _request(base, "PUT", "/prims/selected", body=body)


def set_joint_state(base, body):
    return _request(base, "PATCH", "/prims/physics/joints", body=body)


def update_colliders(base, body):
    return _request(base, "PATCH", "/prims/physics/colliders/", body=body)


# ======================================================================
# Periphery (Camera)
# ======================================================================
def list_camera_prims(base):
    return _request(base, "GET", "/periphery/cameras/prims")


def get_active_camera(base):
    return _request(base, "GET", "/periphery/cameras/active")


def set_active_camera(base, body):
    return _request(base, "PUT", "/periphery/cameras/active", body=body)


def capture_color_image(base, params=None):
    return _request(base, "GET", "/periphery/cameras/capture/color", params=params)


def capture_normals_image(base, params=None):
    return _request(base, "GET", "/periphery/cameras/capture/normals", params=params)


def capture_depth_image(base, params=None):
    return _request(base, "GET", "/periphery/cameras/capture/depth", params=params)


def capture_pointcloud(base, params=None):
    return _request(base, "GET", "/periphery/cameras/capture/pointcloud", params=params)


def capture_boundingbox_2d(base, params=None):
    return _request(base, "GET", "/periphery/cameras/capture/bounding-box-2d", params=params)


def capture_boundingbox_3d(base, params=None):
    return _request(base, "GET", "/periphery/cameras/capture/bounding-box-3d", params=params)


def capture_instance_segmentation(base, params=None):
    return _request(base, "GET", "/periphery/cameras/capture/instance-segmentation", params=params)


def capture_semantic_segmentation(base, params=None):
    return _request(base, "GET", "/periphery/cameras/capture/semantic-segmentation", params=params)


# ======================================================================
# UI
# ======================================================================
def get_visibility(base):
    return _request(base, "GET", "/ui/visibility")


def set_visibility(base, body):
    return _request(base, "PATCH", "/ui/visibility", body=body)


# ======================================================================
# Teaching
# ======================================================================
def list_ghost_object_sources(base):
    return _request(base, "GET", "/teaching/ghost-objects/sources")


def list_tcp_sources(base):
    return _request(base, "GET", "/teaching/tcps/sources")


def create_ghost_object(base, body):
    return _request(base, "POST", "/teaching/ghost-objects", body=body)


def clear_ghost_objects(base):
    return _request(base, "DELETE", "/teaching/ghost-objects")


def list_ghost_objects(base):
    return _request(base, "GET", "/teaching/ghost-objects")


# ======================================================================
# Trajectory Planner
# ======================================================================
def export_trajectory_plans(base):
    return _request(base, "GET", "/trajectory-planner/export")


def export_trajectory_plan_skill(base, skill_name):
    return _request(base, "GET", f"/trajectory-planner/{skill_name}/export")


# ======================================================================
# Manipulators (Motion-Group)
# ======================================================================
def list_motion_groups(base):
    return _request(base, "GET", "/manipulators/motion-groups")


def create_motion_group(base, body):
    return _request(base, "POST", "/manipulators/motion-groups", body=body)


def clear_motion_groups(base):
    return _request(base, "DELETE", "/manipulators/motion-groups")


def update_motion_group_stream(base, prim_path, body):
    return _request(base, "PUT", f"/manipulators/motion-groups/{prim_path}", body=body)


def get_motion_group(base, prim_path):
    return _request(base, "GET", f"/manipulators/motion-groups/{prim_path}")


def remove_motion_group(base, prim_path):
    return _request(base, "DELETE", f"/manipulators/motion-groups/{prim_path}")


# ======================================================================
# Trajectory
# ======================================================================
def list_trajectories(base):
    return _request(base, "GET", "/trajectories/")


def create_trajectory(base, body):
    return _request(base, "POST", "/trajectories/", body=body)


def update_trajectory(base, name, body):
    return _request(base, "PATCH", f"/trajectories/{name}", body=body)


def remove_trajectory(base, name):
    return _request(base, "DELETE", f"/trajectories/{name}")


def create_trajectory_markers(base, name, body):
    return _request(base, "POST", f"/trajectories/{name}/markers", body=body)


def remove_trajectory_markers(base, name):
    return _request(base, "DELETE", f"/trajectories/{name}/markers")


# ======================================================================
# Collision World
# ======================================================================
def sweep_collisions(base, body):
    return _request(base, "POST", "/physics/collision/sweep", body=body)


# ======================================================================
# Robot Overlay (Experimental)
# ======================================================================
def set_robot_overlay_visibility(base, body):
    return _request(base, "PUT", "/overlays/robot/visibility", body=body)


# ======================================================================
# Nucleus
# ======================================================================
def add_nucleus_server(base, body):
    return _request(base, "POST", "/nucleus/server", body=body)


def list_nucleus_servers(base):
    return _request(base, "GET", "/nucleus/servers")


def add_nucleus_api_token(base, body):
    return _request(base, "POST", "/nucleus/server/token", body=body)


def remove_nucleus_api_token(base, body):
    return _request(base, "DELETE", "/nucleus/server/token", body=body)


def remove_all_nucleus_api_tokens(base):
    return _request(base, "DELETE", "/nucleus/server/tokens")


# ======================================================================
# Misc / top-level
# ======================================================================
def get_status(base):
    return _request(base, "GET", "/status")


def get_versions(base):
    return _request(base, "GET", "/version")


def authenticate(base, body):
    return _request(base, "POST", "/auth/token", body=body)


if __name__ == "__main__":
    # Minimal example run against a locally running server (`uvicorn main:app`).
    # The stub server returns 501 for every route, so these calls raise
    # requests.HTTPError until the routes are implemented - expected for now.
    base = "http://localhost:8080"
    try:
        print("status:", get_status(base))
        print("version:", get_versions(base))
        print("motion groups:", list_motion_groups(base))
    except requests.HTTPError as exc:
        print(f"request failed: {exc.response.status_code} {exc.request.url}")
