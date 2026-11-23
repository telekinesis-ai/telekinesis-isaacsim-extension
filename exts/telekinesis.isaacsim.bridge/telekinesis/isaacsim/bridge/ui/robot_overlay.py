# SPDX-License-Identifier: Apache-2.0
"""The robot overlay: the robot drawn see-through in the solution of the selected pose."""

import asyncio

import carb
import omni.usd
from fastapi import HTTPException
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from ..comm.services.poses import (
    POSE_REACHABLE_ATTRIBUTE,
    PoseService,
    define_ghost_material,
)
from ..core import meshes

OVERLAY_ROOT_PATH = "/TelekinesisRobotOverlay"
OVERLAY_MATERIAL_PATH = f"{OVERLAY_ROOT_PATH}/Material"
OVERLAY_LINKS_PATH = f"{OVERLAY_ROOT_PATH}/Links"
# robot overlay colour (#A936DA).
OVERLAY_COLOR = Gf.Vec3f(0xA9, 0x36, 0xDA) / 255.0


# It reacts to selection and solve events on its own, so destroy() is all a caller needs.
class RobotOverlay:  # pylint: disable=too-few-public-methods
    """A see-through copy of a robot, drawn in the solution of the selected pose.

    While exactly one pose is selected and the robot can reach it, the pose's robot is
    drawn with every link where the pose's stored solution puts it, so the arm visibly reaches
    the pose; it follows the pose when the pose is solved again, e.g. while it is dragged. It
    is hidden while the pose is unreachable and whenever no single pose is selected. Each link
    is the link's own visible meshes merged into one, in overlay colour.

    The overlay is authored only in the stage's session layer, under :data:`OVERLAY_ROOT_PATH`,
    so it is never saved and never marks the stage modified; the Stage window does not list it,
    the viewport cannot select it, and it has no physics.

    Example:
        overlay = RobotOverlay(bridge_server.pose_service)
        overlay.destroy()
    """

    def __init__(self, pose_service: PoseService):
        self._pose_service = pose_service
        # The selected pose, or None while not exactly one pose is selected.
        self._pose_path: Sdf.Path | None = None
        # The robot whose links are built in the overlay, None while none are.
        self._overlay_robot_path: str | None = None
        # The refresh running or last run, and whether a change arrived since it last read the
        # selection, so the overlay always ends at the latest solution.
        self._refresh: asyncio.Future | None = None
        self._refresh_outdated = False
        self._stage_event_subscription = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop(
                self._on_stage_event, name="telekinesis robot overlay selection"
            )
        )
        pose_service.on_pose_solved = self._on_pose_solved

    def destroy(self):
        """Remove the overlay and stop following the selection."""
        self._stage_event_subscription = None
        self._pose_service.on_pose_solved = None
        self._remove()

    def _on_stage_event(self, event):
        """Follow the selection; forget the overlay when another stage is opened or closed."""
        if event.type in (int(omni.usd.StageEventType.OPENED), int(omni.usd.StageEventType.CLOSED)):
            # The session layer and everything in it belong to the stage that was replaced.
            self._pose_path = None
            self._overlay_robot_path = None
        elif event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            stage = omni.usd.get_context().get_stage()
            selected = omni.usd.get_context().get_selection().get_selected_prim_paths()
            pose_prim = stage.GetPrimAtPath(selected[0]) if len(selected) == 1 else None
            self._pose_path = (
                pose_prim.GetPath()
                if pose_prim and pose_prim.HasAttribute(POSE_REACHABLE_ATTRIBUTE)
                else None
            )
            self._schedule_refresh()

    def _on_pose_solved(self, pose_path: Sdf.Path):
        """Redraw the overlay when the selected pose's solution was written."""
        if pose_path == self._pose_path:
            self._schedule_refresh()

    def _schedule_refresh(self):
        """Bring the overlay in line with the selection; returns at once."""
        self._refresh_outdated = True
        if self._refresh is None or self._refresh.done():
            self._refresh = asyncio.ensure_future(self._refresh_until_current())

    async def _refresh_until_current(self):
        """Redraw the overlay until no change is left unapplied. A pose whose links cannot be
        computed, e.g. because the server is not running, hides the overlay and logs why."""
        while self._refresh_outdated:
            self._refresh_outdated = False
            pose_path = self._pose_path
            if pose_path is None:
                self._hide()
                continue
            try:
                result = await self._pose_service.link_world_transforms(pose_path)
            except HTTPException as error:
                carb.log_warn(f"[bridge] no robot overlay for {pose_path}: {error.detail}")
                self._hide()
                continue
            if self._refresh_outdated:
                # The selection or solution changed while the links were computed.
                continue
            if result is None:
                self._hide()
                continue
            robot_prim_path, link_world_transforms = result
            self._show(robot_prim_path, link_world_transforms)

    def _show(self, robot_prim_path: str, link_world_transforms: dict[str, Gf.Matrix4d]):
        """Place the overlay's links at ``link_world_transforms`` and show it, building the
        links of ``robot_prim_path`` first when the overlay holds another robot or none."""
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if robot_prim_path != self._overlay_robot_path:
                self._build_links(stage, robot_prim_path, set(link_world_transforms))
            for link_name, link_world in link_world_transforms.items():
                link_mesh = UsdGeom.Mesh.Get(stage, f"{OVERLAY_LINKS_PATH}/{link_name}")
                if link_mesh:
                    link_mesh.MakeMatrixXform().Set(link_world)
            UsdGeom.Imageable(stage.GetPrimAtPath(OVERLAY_ROOT_PATH)).MakeVisible()

    def _build_links(self, stage: Usd.Stage, robot_prim_path: str, link_names: set[str]):
        """Replace the overlay's links with those of the robot at ``robot_prim_path``.

        Each link named in ``link_names`` becomes one mesh, its visible meshes merged in its own
        frame without the meshes of the links nested under it. Links without meshes, such as
        ``tool0``, get none. Must run in the session layer's edit context.
        """
        stage.RemovePrim(OVERLAY_ROOT_PATH)
        root = UsdGeom.Xform.Define(stage, OVERLAY_ROOT_PATH)
        omni.usd.editor.set_hide_in_stage_window(root.GetPrim(), True)
        material = define_ghost_material(
            stage, Sdf.Path(OVERLAY_MATERIAL_PATH), OVERLAY_COLOR
        )
        UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(
            material, UsdShade.Tokens.strongerThanDescendants
        )
        UsdGeom.Xform.Define(stage, OVERLAY_LINKS_PATH)
        # A link may hold a child of its own name, such as an ABB link_1's visual "link_1"; the
        # traversal reaches the link before its children, so the first prim of a name is the link.
        link_paths = {}
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
            if prim.GetName() in link_names and prim.GetName() not in link_paths:
                link_paths[prim.GetName()] = prim.GetPath()
        for link_name, link_path in link_paths.items():
            points, face_vertex_counts, face_vertex_indices = meshes.merge_meshes_in_frame(
                stage,
                link_path.pathString,
                omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(link_path)),
                excluded_paths={path for path in link_paths.values() if path != link_path},
            )
            if not face_vertex_counts:
                continue
            link_mesh = UsdGeom.Mesh.Define(stage, f"{OVERLAY_LINKS_PATH}/{link_name}")
            link_mesh.CreatePointsAttr(points)
            link_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
            link_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
        omni.usd.get_context().set_pickable(OVERLAY_ROOT_PATH, False)
        self._overlay_robot_path = robot_prim_path

    def _hide(self):
        """Hide the overlay, keeping its links for the next pose of the same robot."""
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetSessionLayer().GetPrimAtPath(OVERLAY_ROOT_PATH):
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdGeom.Imageable(stage.GetPrimAtPath(OVERLAY_ROOT_PATH)).MakeInvisible()

    def _remove(self):
        """Remove the overlay from the stage. Does nothing when there is none."""
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetSessionLayer().GetPrimAtPath(OVERLAY_ROOT_PATH):
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.RemovePrim(OVERLAY_ROOT_PATH)
        self._overlay_robot_path = None
