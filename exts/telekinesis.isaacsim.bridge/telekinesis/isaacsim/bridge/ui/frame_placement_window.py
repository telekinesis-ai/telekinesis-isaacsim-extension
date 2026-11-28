# SPDX-License-Identifier: Apache-2.0
"""The base of the windows that place a frame on a robot with a preview and a gizmo."""

import abc
import asyncio

import omni.usd
from fastapi import HTTPException
from omni import ui
from omni.kit import notification_manager
from omni.kit.window.property import templates
from pxr import Gf, Sdf, Usd, UsdGeom

from ..comm import models
from ..core import robot_assembler
from .add_asset_window import (
    BUTTON_ROW_GAP,
    BUTTON_WIDTH,
    WINDOW_HEIGHT,
    WINDOW_WIDTH,
    build_gizmo_hint,
    deselect_all,
)
from .placement_gizmo import PlacementGizmo
from .transformation_fields import TransformationFields


class FramePlacementWindow(abc.ABC):
    """A window that places a frame relative to a reference frame of a robot, such as its
    default TCP or base frame, and adds something there.

    The frame is entered as Position (metres) and Rotation (XYZ Euler degrees) in the reference
    frame. While the window is open, a preview of the frame's red/green/blue axis arrows stands
    where the entered values put it, with a move/rotate gizmo on it: dragging the gizmo enters
    where it is dragged to in Position and Rotation. The gizmo moves along and turns about the
    frame's own axes, offers rotation while the viewport's Rotate tool (E) is selected and
    translation otherwise, and edits nothing in the stage. Nothing in the stage can be selected
    while the window is open, so Kit's own gizmo never shows beside it. **Add** adds at the
    entered frame and leaves the window open for the next one. Failures are notified as warnings
    that stay until dismissed.

    The preview is authored only in the stage's session layer, under the window's preview path,
    so it is never saved and never marks the stage modified; the Stage window does not list it
    and the viewport cannot select it. It and the gizmo are removed when the window closes.

    A window fills in its own rows, finds its robot and reference frame, and says what **Add**
    does; this class owns the rest of the flow.

    Example:
        window = AddTcpWindow(bridge_server.pose_service)
        window.show("/World/vention_table/robot_mount/UR10e_assembly/UR10e")
    """

    def __init__(self, title: str, preview_path: str, frame_description: str):
        """Build the window, closed.

        Args:
            title: The window's title.
            preview_path: Session-layer path of the preview, unique to the window, so windows
                open at once keep their own preview.
            frame_description: What Position and Rotation are, shown before the reference
                frame's name, e.g. ``"Offset from the default TCP"``.
        """
        self._preview_path = preview_path
        self._preview_marker_path = f"{preview_path}/Frame"
        self._frame_description = frame_description
        # The robot the window was opened on and the frame Position and Rotation are read in.
        self._robot_prim_path = ""
        self._reference_frame_prim_path = ""
        self._gizmo = PlacementGizmo(on_dragged_fn=self._on_gizmo_dragged)
        # Clears every selection while the window is open; None while it is closed.
        self._selection_subscription = None

        self._window = ui.Window(title, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, visible=False)
        self._window.set_visibility_changed_fn(self._on_visibility_changed)
        with self._window.frame, ui.VStack(spacing=5, height=0):
            with ui.HStack(height=0):
                ui.Label(
                    "Robot",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="The robot the window works on",
                )
                self._robot_model = ui.StringField(read_only=True).model
            self.build_extra_rows()
            self._frame_label = ui.Label("", word_wrap=True)
            self._transformation_fields = TransformationFields(on_changed_fn=self._place_preview)
            build_gizmo_hint()
            self._status_label = ui.Label("", word_wrap=True, visible=False)
            ui.Spacer(height=BUTTON_ROW_GAP)
            with ui.HStack(spacing=5, height=0):
                ui.Spacer()
                self._add_button = ui.Button(
                    "Add", width=BUTTON_WIDTH, clicked_fn=self._on_add_clicked
                )
                ui.Spacer()

    @abc.abstractmethod
    def build_extra_rows(self):
        """Build the window's own rows, placed between the Robot row and Position.

        Called once while the window is built, inside its layout.
        """

    @abc.abstractmethod
    async def load_robot(
        self, robot_part_prim_path: str
    ) -> tuple[str, str, models.Transformation]:
        """Find the robot ``robot_part_prim_path`` is part of and fill the window's own rows.

        Called each time the window is opened, before it is shown.

        Returns:
            ``(robot_prim_path, reference_frame_prim_path, initial_transformation)``: the
            robot, the frame Position and Rotation are read in, and the values they start at.

        Raises:
            HTTPException: When the window cannot work on the prim; the detail is notified and
                the window stays closed.
        """

    @abc.abstractmethod
    async def add_frame(self, transformation: models.Transformation) -> str:
        """Add what the window adds at ``transformation`` in the reference frame.

        Called when Add is clicked; Add is disabled until it returns.

        Returns:
            The message notified on success.

        Raises:
            HTTPException: When adding fails; ``"Adding failed: <detail>"`` is notified.
        """

    def on_frame_moved(self):
        """React to the entered frame changing, e.g. to show the robot reaching it.

        Called after the preview was moved to the entered frame, also once when the window
        opens. Does nothing by default.
        """

    def on_closed(self):
        """Clean up the window's own drawing when the window closes. Does nothing by default."""

    def show(self, robot_part_prim_path: str):
        """Open the window for the robot ``robot_part_prim_path`` is part of; returns at once.

        The window opens once its robot and reference frame are found, with the preview at the
        initial frame. It stays closed when they cannot be found, which is notified.

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

    def set_status(self, text: str):
        """Show ``text`` above Add, or hide the line when ``text`` is empty."""
        self._status_label.text = text
        self._status_label.visible = bool(text)

    def reference_frame_world_transform(self) -> Gf.Matrix4d:
        """Return the world transform of the reference frame where it stands now."""
        stage = omni.usd.get_context().get_stage()
        return omni.usd.get_world_transform_matrix(
            stage.GetPrimAtPath(self._reference_frame_prim_path)
        )

    async def _open(self, robot_part_prim_path: str):
        """Find the robot and reference frame, show the preview at the initial frame and open
        the window."""
        try:
            (
                robot_prim_path,
                reference_frame_prim_path,
                initial_transformation,
            ) = await self.load_robot(robot_part_prim_path)
        except HTTPException as error:
            self._post_failure(f"Opening {self._window.title} failed: {error.detail}")
            return
        self._robot_prim_path = robot_prim_path
        self._reference_frame_prim_path = reference_frame_prim_path
        self._robot_model.set_value(robot_prim_path)
        self._frame_label.text = (
            f"{self._frame_description} {Sdf.Path(reference_frame_prim_path).name}:"
        )
        self.set_status("")
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.RemovePrim(self._preview_path)
            root = UsdGeom.Xform.Define(stage, self._preview_path)
            omni.usd.editor.set_hide_in_stage_window(root.GetPrim(), True)
            robot_assembler.define_tcp_marker(stage, self._preview_marker_path, None)
        omni.usd.get_context().set_pickable(self._preview_path, False)
        self._transformation_fields.set_transformation(initial_transformation)
        # Fields already at those values do not report a change, so the preview is placed
        # explicitly.
        self._place_preview()
        self._window.visible = True

    def _place_preview(self):
        """Move the preview and the gizmo to the entered frame in the reference frame where it
        stands now, then call :meth:`on_frame_moved`. Does nothing while no preview is shown."""
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetSessionLayer().GetPrimAtPath(self._preview_path):
            return
        transformation = self._transformation_fields.transformation()
        reference_frame_world = self.reference_frame_world_transform()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdGeom.Xform(stage.GetPrimAtPath(self._preview_path)).MakeMatrixXform().Set(
                reference_frame_world
            )
            translate_op, rotate_op = UsdGeom.Xform(
                stage.GetPrimAtPath(self._preview_marker_path)
            ).GetOrderedXformOps()
            translate_op.Set(Gf.Vec3d(*transformation.translation))
            rotate_op.Set(Gf.Vec3f(*transformation.rotation))
        # Gf composes row-vector first: the frame in the reference frame, then that frame.
        self._gizmo.show(robot_assembler.offset_matrix(transformation) * reference_frame_world)
        self.on_frame_moved()

    def _on_gizmo_dragged(self, pose: Gf.Matrix4d):
        """Enter the dragged world ``pose``, read in the reference frame, in Position and
        Rotation, which moves the preview after it."""
        translation, rotation = robot_assembler.matrix_to_translation_rotation(
            pose * self.reference_frame_world_transform().GetInverse()
        )
        self._transformation_fields.set_transformation(
            models.Transformation(translation=translation, rotation=rotation)
        )

    def _remove_preview(self):
        """Remove the preview and the gizmo. Does nothing to a preview that is not shown."""
        self._gizmo.hide()
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetSessionLayer().GetPrimAtPath(self._preview_path):
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.RemovePrim(self._preview_path)

    def _on_visibility_changed(self, visible: bool):
        """Keep the stage unselected while the window is open, and remove the preview, the
        gizmo and the window's own drawing when it closes."""
        if visible:
            # A selected prim gets Kit's own gizmo, which would stand beside the window's.
            deselect_all()
            self._selection_subscription = (
                omni.usd.get_context()
                .get_stage_event_stream()
                .create_subscription_to_pop_by_type(
                    int(omni.usd.StageEventType.SELECTION_CHANGED),
                    lambda _: deselect_all(),
                    name=f"telekinesis {self._window.title} selection lock",
                )
            )
        else:
            self._selection_subscription = None
            self._remove_preview()
            self.on_closed()

    def _on_add_clicked(self):
        """Add in the background."""
        asyncio.ensure_future(self._add())

    async def _add(self):
        """Add at the entered frame and notify the outcome. Add is disabled while it runs."""
        self._add_button.enabled = False
        try:
            message = await self.add_frame(self._transformation_fields.transformation())
        except HTTPException as error:
            self._post_failure(f"Adding failed: {error.detail}")
            return
        finally:
            self._add_button.enabled = True
        notification_manager.post_notification(message)

    @staticmethod
    def _post_failure(message: str):
        """Notify ``message`` as a warning that stays until dismissed."""
        notification_manager.post_notification(
            message,
            hide_after_timeout=False,
            status=notification_manager.NotificationStatus.WARNING,
            button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
        )
