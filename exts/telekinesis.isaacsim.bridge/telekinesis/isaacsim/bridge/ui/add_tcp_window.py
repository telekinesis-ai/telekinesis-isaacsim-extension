# SPDX-License-Identifier: Apache-2.0
"""The Add TCP window, which adds a TCP to a robot."""

import asyncio

import omni.usd
from fastapi import HTTPException
from omni import ui
from omni.kit import notification_manager
from omni.kit.window.property import templates
from pxr import Gf, Sdf, Usd, UsdGeom

from ..comm import models
from ..comm.services.poses import PoseService
from ..core import robot_assembler
from .add_asset_window import (
    BUTTON_ROW_GAP,
    BUTTON_WIDTH,
    WINDOW_WIDTH,
    build_gizmo_hint,
    deselect_all,
)
from .placement_gizmo import PlacementGizmo
from .transformation_fields import TransformationFields

# Name the Name field starts with. Not "tcp", which the Synapse active-TCP marker group under
# the mount link may hold already.
DEFAULT_TCP_NAME = "custom_tcp"
TCP_PREVIEW_PATH = "/TelekinesisTcpPreview"
TCP_PREVIEW_MARKER_PATH = f"{TCP_PREVIEW_PATH}/Tcp"


class AddTcpWindow:
    """A window that adds a TCP to a robot added with Add Robot.

    The TCP is a frame fixed to the robot's default TCP, such as ``tool0`` on a flange, at the
    entered Position (metres) and Rotation (XYZ Euler degrees) in that frame: the offset
    Synapse's ``add_tcp`` takes. While the window is open, a preview of the TCP's red/green/blue
    axis arrows stands where the entered offset puts it, with a move/rotate gizmo on it:
    dragging the gizmo enters where it is dragged to in Position and Rotation. The gizmo moves
    along and turns about the TCP's own axes, offers rotation while the viewport's Rotate tool
    (E) is selected and translation otherwise, and edits nothing in the stage. Nothing in the
    stage can be selected while the window is open, so Kit's own gizmo never shows beside it.
    **Add** creates the TCP as an Xform named after the Name field under the default TCP frame
    and leaves the window open for the next TCP. Once the window is closed, the TCP can be
    selected and moved with Kit's gizmo or the Property window, and right-clicking it offers Add
    Pose for that TCP. The Telekinesis Isaac Sim Server gets the robot's TCPs when the robot is
    loaded there, and again whenever one changes; adding a TCP needs the server only for a robot
    that stores no default TCP yet. Failures are notified as warnings that stay until dismissed.

    The preview is authored only in the stage's session layer, under :data:`TCP_PREVIEW_PATH`,
    so it is never saved and never marks the stage modified; the Stage window does not list it
    and the viewport cannot select it. It and the gizmo are removed when the window closes.

    Example:
        window = AddTcpWindow(bridge_server.pose_service)
        window.show("/World/vention_table/robot_mount/UR10e_assembly/UR10e")
    """

    def __init__(self, pose_service: PoseService):
        self._pose_service = pose_service
        # The prim the window was opened on, which the TCP is added to the robot of.
        self._robot_part_prim_path = ""
        # The robot's default TCP frame, which the preview stands in.
        self._default_tcp_prim_path = ""
        self._gizmo = PlacementGizmo(on_dragged_fn=self._on_gizmo_dragged)
        # Clears every selection while the window is open; None while it is closed.
        self._selection_subscription = None

        self._window = ui.Window("Add TCP", width=WINDOW_WIDTH, height=0, visible=False)
        self._window.set_visibility_changed_fn(self._on_visibility_changed)
        with self._window.frame, ui.VStack(spacing=5, height=0):
            with ui.HStack(height=0):
                ui.Label(
                    "Robot",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="The robot the TCP is added to",
                )
                self._robot_model = ui.StringField(read_only=True).model
            with ui.HStack(height=0):
                ui.Label(
                    "Name",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="The TCP's name, which is also its prim's name",
                )
                self._name_model = ui.StringField().model
            self._frame_label = ui.Label("", word_wrap=True)
            self._transformation_fields = TransformationFields(on_changed_fn=self._place_preview)
            build_gizmo_hint()
            ui.Spacer(height=BUTTON_ROW_GAP)
            with ui.HStack(spacing=5, height=0):
                ui.Spacer()
                self._add_button = ui.Button(
                    "Add", width=BUTTON_WIDTH, clicked_fn=self._on_add_clicked
                )
                ui.Spacer()

    def show(self, robot_part_prim_path: str):
        """Open the window for the robot ``robot_part_prim_path`` is part of; returns at once.

        The window opens once the robot and its default TCP are found, with the Name field at
        ``custom_tcp``, the offset at zero, and the preview at the default TCP. It stays closed
        when the prim is part of no robot added with Add Robot, or the default TCP cannot be
        found, which is notified.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.
        """
        asyncio.ensure_future(self._open(robot_part_prim_path))

    def close(self):
        """Close the window and remove the preview."""
        self._window.visible = False

    def destroy(self):
        """Remove the preview and the gizmo and release the window's widgets."""
        self._remove_preview()
        self._gizmo.destroy()
        self._selection_subscription = None
        self._window.destroy()

    async def _open(self, robot_part_prim_path: str):
        """Find the robot and its default TCP, show the preview, reset the fields and open the
        window."""
        try:
            robot_prim_path, default_tcp_prim_path = await self._pose_service.find_default_tcp(
                robot_part_prim_path
            )
        except HTTPException as error:
            self._post_failure(f"Opening Add TCP failed: {error.detail}")
            return
        self._robot_part_prim_path = robot_part_prim_path
        self._default_tcp_prim_path = default_tcp_prim_path
        self._robot_model.set_value(robot_prim_path)
        self._frame_label.text = (
            f"Offset from the default TCP {Sdf.Path(default_tcp_prim_path).name}:"
        )
        self._name_model.set_value(DEFAULT_TCP_NAME)
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.RemovePrim(TCP_PREVIEW_PATH)
            root = UsdGeom.Xform.Define(stage, TCP_PREVIEW_PATH)
            omni.usd.editor.set_hide_in_stage_window(root.GetPrim(), True)
            robot_assembler.define_tcp_marker(stage, TCP_PREVIEW_MARKER_PATH, None)
        omni.usd.get_context().set_pickable(TCP_PREVIEW_PATH, False)
        # Fields already at zero do not report a change, so the preview is placed explicitly.
        self._transformation_fields.reset()
        self._place_preview()
        self._window.visible = True

    def _place_preview(self):
        """Move the preview and the gizmo to the entered offset from the default TCP where it
        stands now. Does nothing while no preview is shown."""
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetSessionLayer().GetPrimAtPath(TCP_PREVIEW_PATH):
            return
        offset = self._transformation_fields.transformation()
        default_tcp_world = self._default_tcp_world_transform()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdGeom.Xform(stage.GetPrimAtPath(TCP_PREVIEW_PATH)).MakeMatrixXform().Set(
                default_tcp_world
            )
            translate_op, rotate_op = UsdGeom.Xform(
                stage.GetPrimAtPath(TCP_PREVIEW_MARKER_PATH)
            ).GetOrderedXformOps()
            translate_op.Set(Gf.Vec3d(*offset.translation))
            rotate_op.Set(Gf.Vec3f(*offset.rotation))
        # Gf composes row-vector first: the offset in the default TCP frame, then that frame.
        self._gizmo.show(robot_assembler.offset_matrix(offset) * default_tcp_world)

    def _on_gizmo_dragged(self, pose: Gf.Matrix4d):
        """Enter the dragged world ``pose``, read in the default TCP frame, in Position and
        Rotation, which moves the preview after it."""
        translation, rotation = robot_assembler.matrix_to_translation_rotation(
            pose * self._default_tcp_world_transform().GetInverse()
        )
        self._transformation_fields.set_transformation(
            models.Transformation(translation=translation, rotation=rotation)
        )

    def _default_tcp_world_transform(self) -> Gf.Matrix4d:
        """Return the world transform of the robot's default TCP frame where it stands now."""
        stage = omni.usd.get_context().get_stage()
        return omni.usd.get_world_transform_matrix(
            stage.GetPrimAtPath(self._default_tcp_prim_path)
        )

    def _remove_preview(self):
        """Remove the preview and the gizmo. Does nothing to a preview that is not shown."""
        self._gizmo.hide()
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetSessionLayer().GetPrimAtPath(TCP_PREVIEW_PATH):
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.RemovePrim(TCP_PREVIEW_PATH)

    def _on_visibility_changed(self, visible: bool):
        """Keep the stage unselected while the window is open, and remove the preview and the
        gizmo when it closes."""
        if visible:
            # A selected prim gets Kit's own gizmo, which would stand beside the window's.
            deselect_all()
            self._selection_subscription = (
                omni.usd.get_context()
                .get_stage_event_stream()
                .create_subscription_to_pop_by_type(
                    int(omni.usd.StageEventType.SELECTION_CHANGED),
                    lambda _: deselect_all(),
                    name="telekinesis add tcp window selection lock",
                )
            )
        else:
            self._selection_subscription = None
            self._remove_preview()

    def _on_add_clicked(self):
        """Add the TCP in the background."""
        asyncio.ensure_future(self._add_tcp())

    async def _add_tcp(self):
        """Create the TCP and notify the outcome. Add is disabled while it runs."""
        self._add_button.enabled = False
        try:
            tcp_prim_path = await self._pose_service.create_tcp(
                self._robot_part_prim_path,
                self._name_model.get_value_as_string(),
                self._transformation_fields.transformation(),
            )
        except HTTPException as error:
            self._post_failure(f"Adding a TCP failed: {error.detail}")
            return
        finally:
            self._add_button.enabled = True
        notification_manager.post_notification(f"Added TCP {tcp_prim_path}.")

    @staticmethod
    def _post_failure(message: str):
        """Notify ``message`` as a warning that stays until dismissed."""
        notification_manager.post_notification(
            message,
            hide_after_timeout=False,
            status=notification_manager.NotificationStatus.WARNING,
            button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
        )
