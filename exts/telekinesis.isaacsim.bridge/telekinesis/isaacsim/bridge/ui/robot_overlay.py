# SPDX-License-Identifier: Apache-2.0
"""The robot overlay: the robot drawn see-through in the solution of the selected pose, or at
joint positions being taught."""

import asyncio

import carb
import omni.usd
from fastapi import HTTPException
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from ..comm.services.articulations import MOUNT_LINK_ATTRIBUTE
from ..comm.services.poses import POSE_REACHABLE_ATTRIBUTE, PoseService
from ..core import meshes
from . import asset_ghost

OVERLAY_ROOT_PATH = "/TelekinesisRobotOverlay"
OVERLAY_MATERIAL_PATH = f"{OVERLAY_ROOT_PATH}/Material"
OVERLAY_LINKS_PATH = f"{OVERLAY_ROOT_PATH}/Links"
OVERLAY_TOOL_PATH = f"{OVERLAY_ROOT_PATH}/Tool"


class RobotOverlay:
    """A see-through copy of a robot, drawn in the solution of the selected pose or at joint
    positions being taught.

    While exactly one pose is selected and the robot can reach it, the pose's robot is
    drawn with every link where the pose's stored solution puts it, so the arm visibly reaches
    the pose; it follows the pose when the pose is solved again, e.g. while it is dragged. For a
    frame pose, which shows only arrows, the robot's tool is drawn with it; a ghost pose shows
    the tool itself. It is hidden while the pose is unreachable and whenever no single pose is selected. Each link
    is the link's own visible meshes merged into one, in the see-through blue of the Add window
    previews.

    While joint positions are shown with :meth:`show_joint_positions`, the robot and its tool
    are drawn at them instead, whatever is selected, until :meth:`stop_showing_joint_positions`.

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
        # The joint positions shown, as (robot prim path, tool prim path, joints), or None while
        # the overlay follows the selected pose.
        self._shown_joint_positions: tuple[str, str | None, dict] | None = None
        # The robot and tool whose meshes are built in the overlay, as (robot prim path, tool
        # prim path or None), None while none are.
        self._overlay_parts: tuple[str, str | None] | None = None
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

    def show_joint_positions(self, robot_prim_path: str, tool_prim_path: str | None, joints: dict):
        """Draw the robot at ``robot_prim_path`` and its tool at ``joints`` instead of the
        selected pose's solution; returns at once.

        Args:
            robot_prim_path: The robot prim, which stores its Synapse class name.
            tool_prim_path: The tool mounted on the robot's mount link, drawn with it, or
                ``None`` for a robot without a tool.
            joints: ``{"joint_names": [...], "joint_positions": [...]}`` in radians, naming
                every joint of the robot. Positions the server rejects hide the overlay and log
                why.
        """
        self._shown_joint_positions = (robot_prim_path, tool_prim_path, joints)
        self._schedule_refresh()

    def stop_showing_joint_positions(self):
        """Follow the selected pose again; returns at once."""
        self._shown_joint_positions = None
        self._schedule_refresh()

    def _on_stage_event(self, event):
        """Follow the selection; forget the overlay when another stage is opened or closed."""
        if event.type in (int(omni.usd.StageEventType.OPENED), int(omni.usd.StageEventType.CLOSED)):
            # The session layer and everything in it belong to the stage that was replaced.
            self._pose_path = None
            self._shown_joint_positions = None
            self._overlay_parts = None
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
        """Redraw the overlay until no change is left unapplied. Links that cannot be computed,
        e.g. because the server is not running, hide the overlay and log why."""
        while self._refresh_outdated:
            self._refresh_outdated = False
            shown_joint_positions = self._shown_joint_positions
            pose_path = self._pose_path
            tool_prim_path = None
            try:
                if shown_joint_positions is not None:
                    robot_prim_path, tool_prim_path, joints = shown_joint_positions
                    result = (
                        robot_prim_path,
                        await self._pose_service.robot_link_world_transforms(
                            robot_prim_path, joints
                        ),
                    )
                elif pose_path is not None:
                    result = await self._pose_service.link_world_transforms(pose_path)
                    # A ghost pose is the tool already; a frame pose is only arrows.
                    pose_prim = omni.usd.get_context().get_stage().GetPrimAtPath(pose_path)
                    if result is not None and pose_prim and not pose_prim.IsA(UsdGeom.Mesh):
                        _, tool_prim_path = await self._pose_service.find_robot_and_tool(
                            result[0]
                        )
                else:
                    result = None
            except HTTPException as error:
                subject = pose_path if shown_joint_positions is None else shown_joint_positions[0]
                carb.log_warn(f"[bridge] no robot overlay for {subject}: {error.detail}")
                self._hide()
                continue
            if self._refresh_outdated:
                # The selection, solution or joint positions changed while the links were
                # computed.
                continue
            if result is None:
                self._hide()
                continue
            robot_prim_path, link_world_transforms = result
            self._show(robot_prim_path, tool_prim_path, link_world_transforms)

    def _show(
        self,
        robot_prim_path: str,
        tool_prim_path: str | None,
        link_world_transforms: dict[str, Gf.Matrix4d],
    ):
        """Place the overlay's links at ``link_world_transforms`` and show it, building the
        meshes of ``robot_prim_path`` and ``tool_prim_path`` first when the overlay holds
        others. The tool follows the robot's mount link."""
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if (robot_prim_path, tool_prim_path) != self._overlay_parts:
                self._build_links(
                    stage, robot_prim_path, tool_prim_path, set(link_world_transforms)
                )
            for link_name, link_world in link_world_transforms.items():
                link_mesh = UsdGeom.Mesh.Get(stage, f"{OVERLAY_LINKS_PATH}/{link_name}")
                if link_mesh:
                    link_mesh.MakeMatrixXform().Set(link_world)
            tool_mesh = UsdGeom.Mesh.Get(stage, OVERLAY_TOOL_PATH)
            if tool_mesh:
                mount_link = (
                    stage.GetPrimAtPath(robot_prim_path).GetAttribute(MOUNT_LINK_ATTRIBUTE).Get()
                )
                tool_mesh.MakeMatrixXform().Set(link_world_transforms[mount_link])
            UsdGeom.Imageable(stage.GetPrimAtPath(OVERLAY_ROOT_PATH)).MakeVisible()

    def _build_links(
        self,
        stage: Usd.Stage,
        robot_prim_path: str,
        tool_prim_path: str | None,
        link_names: set[str],
    ):
        """Replace the overlay's meshes with those of the robot at ``robot_prim_path`` and of
        the tool at ``tool_prim_path``, when one is given.

        Each link named in ``link_names`` becomes one mesh, its visible meshes merged in its own
        frame without the meshes of the links nested under it. Links without meshes, such as
        ``tool0``, get none. The tool becomes one mesh at :data:`OVERLAY_TOOL_PATH`, merged in
        the frame of the robot's mount link, the link the tool is fixed to; none is built when
        the robot stores no mount link among ``link_names``. Must run in the session layer's
        edit context.
        """
        stage.RemovePrim(OVERLAY_ROOT_PATH)
        root = UsdGeom.Xform.Define(stage, OVERLAY_ROOT_PATH)
        omni.usd.editor.set_hide_in_stage_window(root.GetPrim(), True)
        material = asset_ghost.define_preview_material(stage, OVERLAY_MATERIAL_PATH)
        UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(
            material, UsdShade.Tokens.strongerThanDescendants
        )
        UsdGeom.Xform.Define(stage, OVERLAY_LINKS_PATH)
        link_paths = meshes.find_link_paths(stage, robot_prim_path, link_names)
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
        mount_link = stage.GetPrimAtPath(robot_prim_path).GetAttribute(MOUNT_LINK_ATTRIBUTE).Get()
        if tool_prim_path is not None and mount_link in link_paths:
            points, face_vertex_counts, face_vertex_indices = meshes.merge_meshes_in_frame(
                stage,
                tool_prim_path,
                omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(link_paths[mount_link])),
            )
            tool_mesh = UsdGeom.Mesh.Define(stage, OVERLAY_TOOL_PATH)
            tool_mesh.CreatePointsAttr(points)
            tool_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
            tool_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
        omni.usd.get_context().set_pickable(OVERLAY_ROOT_PATH, False)
        self._overlay_parts = (robot_prim_path, tool_prim_path)

    def _hide(self):
        """Hide the overlay, keeping its meshes for the next drawing of the same robot."""
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
        self._overlay_parts = None
