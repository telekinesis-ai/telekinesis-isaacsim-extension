# SPDX-License-Identifier: Apache-2.0
"""A move/rotate gizmo in the viewport that reports where it is dragged, without editing the stage."""

from collections.abc import Callable

from omni.kit.manipulator import transform
from omni.kit.viewport import utility as viewport_utility
from omni.ui import scene as sc
from pxr import Gf

# Name of the viewport frame the gizmo's scene view is built in.
GIZMO_FRAME_NAME = "telekinesis.isaacsim.bridge.placement_gizmo"


class PlacementGizmoModel(transform.AbstractTransformManipulatorModel):
    """The pose a :class:`PlacementGizmo` stands at, and the operation it offers.

    The gizmo is drawn world-aligned at the pose's position, so a drag moves along
    and turns about the world axes.
    """

    def __init__(self, on_dragged_fn: Callable[[Gf.Matrix4d], None]):
        """``on_dragged_fn`` receives every world pose the gizmo is dragged to."""
        super().__init__()
        self.pose = Gf.Matrix4d(1.0)
        # Whether a drag is under way; the pose is then driven by the drag alone.
        self.dragging = False
        self._operation = transform.Operation.TRANSLATE
        self._on_dragged_fn = on_dragged_fn

    def set_pose(self, pose: Gf.Matrix4d):
        """Stand the gizmo at the world pose ``pose``; ignored while it is dragged."""
        if self.dragging:
            return
        self.pose = Gf.Matrix4d(pose)
        self._item_changed(self._transform_item)

    def drag_to(self, pose: Gf.Matrix4d):
        """Move the gizmo to the dragged world pose ``pose`` and report it."""
        self.pose = Gf.Matrix4d(pose)
        self._item_changed(self._transform_item)
        self._on_dragged_fn(Gf.Matrix4d(pose))

    def set_operation(self, operation: transform.Operation):
        """Offer ``operation`` (translate or rotate) from now on."""
        if operation != self._operation:
            self._operation = operation
            self._item_changed(self._transform_item)

    def get_operation(self) -> transform.Operation:
        """Return the operation the gizmo offers."""
        return self._operation

    def get_as_floats(self, item):
        """Return the gizmo's transform: the pose's position only, keeping it world-aligned."""
        if item == self._transform_item:
            return sc.Matrix44.get_translation_matrix(*self.pose.ExtractTranslation())
        return None


class PlacementTranslateGesture(transform.TranslateChangedGesture):
    """Moves the model's pose along a dragged world axis or plane."""

    def on_began(self):
        """Remember the pose the drag starts from."""
        self._begin_pose = Gf.Matrix4d(self.sender.model.pose)
        self._moved = Gf.Vec3d(0.0)
        self.sender.model.dragging = True

    def on_changed(self):
        """Move the pose by the distance dragged so far, keeping its orientation."""
        if not isinstance(self.gesture_payload, transform.TranslateDragGesturePayload):
            return
        # Summed from the per-update deltas, as Kit's own gizmo does: the gizmo follows
        # the drag, so the payload's running total is measured from a moving origin.
        self._moved += Gf.Vec3d(*self.gesture_payload.moved_delta)
        pose = Gf.Matrix4d(self._begin_pose)
        pose.SetTranslateOnly(self._begin_pose.ExtractTranslation() + self._moved)
        self.sender.model.drag_to(pose)

    def on_ended(self):
        """Let the window place the gizmo again."""
        self.sender.model.dragging = False

    def on_canceled(self):
        """Let the window place the gizmo again."""
        self.sender.model.dragging = False


class PlacementRotateGesture(transform.RotateChangedGesture):
    """Turns the model's pose about a dragged world axis through its position."""

    def on_began(self):
        """Remember the pose the drag starts from."""
        self._begin_pose = Gf.Matrix4d(self.sender.model.pose)
        self.sender.model.dragging = True

    def on_changed(self):
        """Turn the pose by the angle dragged so far, keeping its position."""
        payload = self.gesture_payload
        if not isinstance(payload, transform.RotateDragGesturePayload):
            return
        # A free rotation turns about an axis that changes during the drag, so it is
        # applied step by step; an axis drag reports the angle since it began.
        if payload.free_rotation:
            start_pose, angle = self.sender.model.pose, payload.angle_delta
        else:
            start_pose, angle = self._begin_pose, payload.angle
        orientation = Gf.Matrix4d(start_pose)
        orientation.SetTranslateOnly(Gf.Vec3d(0.0))
        # Gf composes row-vector first: the start orientation, then the world rotation.
        pose = orientation * Gf.Matrix4d(1.0).SetRotate(
            Gf.Rotation(Gf.Vec3d(*payload.axis[:3]), angle)
        )
        pose.SetTranslateOnly(start_pose.ExtractTranslation())
        self.sender.model.drag_to(pose)

    def on_ended(self):
        """Let the window place the gizmo again."""
        self.sender.model.dragging = False

    def on_canceled(self):
        """Let the window place the gizmo again."""
        self.sender.model.dragging = False


class PlacementGizmo:
    """A move/rotate gizmo drawn over the active viewport, which edits nothing in the stage.

    It stands where :meth:`show` puts it and reports each pose it is dragged to; the
    caller decides what the pose means. Its axes are the world axes. It offers
    rotation while the viewport's Rotate tool is selected (E) and translation
    otherwise (W, or any other tool). Dragging it neither selects prims nor adds to
    undo.

    Example:
        gizmo = PlacementGizmo(on_dragged_fn=lambda pose: print(pose.ExtractTranslation()))
        gizmo.show(Gf.Matrix4d(1.0))
        gizmo.hide()
    """

    def __init__(self, on_dragged_fn: Callable[[Gf.Matrix4d], None]):
        """``on_dragged_fn`` receives every world pose the gizmo is dragged to."""
        self._model = PlacementGizmoModel(on_dragged_fn)
        self._viewport_window = None
        self._scene_view: sc.SceneView | None = None
        self._operation_listener = transform.OpSettingsListener()
        self._operation_subscription = self._operation_listener.subscribe_listener(
            self._on_viewport_tool_changed
        )
        self._follow_viewport_tool(self._operation_listener.selected_op)

    def show(self, pose: Gf.Matrix4d):
        """Show the gizmo at the world pose ``pose``, over the active viewport.

        Does nothing while the gizmo is being dragged, or when there is no viewport.
        """
        if self._scene_view is None:
            self._viewport_window = viewport_utility.get_active_viewport_window()
            if self._viewport_window is None:
                return
            with self._viewport_window.get_frame(GIZMO_FRAME_NAME):
                self._scene_view = sc.SceneView()
                with self._scene_view.scene:
                    transform.TransformManipulator(
                        model=self._model,
                        gestures=[
                            PlacementTranslateGesture(),
                            PlacementRotateGesture(),
                        ],
                    )
            self._viewport_window.viewport_api.add_scene_view(self._scene_view)
        self._model.set_pose(pose)

    def hide(self):
        """Remove the gizmo from the viewport. Does nothing when it is not shown."""
        if self._scene_view is None:
            return
        self._scene_view.scene.clear()
        self._viewport_window.viewport_api.remove_scene_view(self._scene_view)
        self._viewport_window.get_frame(GIZMO_FRAME_NAME).clear()
        self._scene_view = None
        self._viewport_window = None
        self._model.dragging = False

    def destroy(self):
        """Remove the gizmo and stop following the viewport's tool."""
        self.hide()
        self._operation_subscription.unsubscribe()
        self._operation_listener.destroy()

    def _on_viewport_tool_changed(self, callback_type, value):
        """Switch between translation and rotation with the viewport's tool."""
        if callback_type == transform.OpSettingsListener.CallbackType.OP_CHANGED:
            self._follow_viewport_tool(value)

    def _follow_viewport_tool(self, viewport_operation: str):
        """Offer rotation for the Rotate tool and translation for every other tool."""
        self._model.set_operation(
            transform.Operation.ROTATE
            if viewport_operation == transform.Constants.TRANSFORM_OP_ROTATE
            else transform.Operation.TRANSLATE
        )
