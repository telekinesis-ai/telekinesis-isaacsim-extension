# SPDX-License-Identifier: Apache-2.0
"""A move/rotate gizmo in the viewport that reports where it is dragged,
without editing the stage.
"""

import math
from collections.abc import Callable

from omni.kit.manipulator import transform
from omni.kit.manipulator.tool import snap
from omni.kit.viewport import utility as viewport_utility
from omni.ui import scene as sc
from pxr import Gf

# Name of the viewport frame the gizmo's scene view is built in.
GIZMO_FRAME_NAME = "telekinesis.isaacsim.bridge.placement_gizmo"


class PlacementGizmoModel(transform.AbstractTransformManipulatorModel):
    """The pose a :class:`PlacementGizmo` stands at, and the operation it offers.

    The gizmo is drawn in the pose's own frame, so a drag moves along and turns about
    the pose's axes.
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
        """Return the gizmo's transform: the pose without its scale, so the gizmo keeps its size."""
        if item == self._transform_item:
            return [value for row in self.pose.RemoveScaleShear() for value in row]
        return None


class PlacementTranslateGesture(transform.TranslateChangedGesture):
    """Moves the model's pose along a dragged axis or plane of the pose, or onto the
    surface under the cursor when its center is dragged with snapping to a surface on."""

    def __init__(
        self,
        snap_manager: snap.SnapProviderManager,
        snap_settings: transform.SnapSettingsListener,
    ):
        """``snap_manager`` finds the surface under the cursor with the snap providers
        selected in the viewport's snap toolbar; ``snap_settings`` tells whether snapping
        to them is on."""
        super().__init__()
        self._snap_manager = snap_manager
        self._snap_settings = snap_settings
        # Whether the current drag snaps; decided when it begins.
        self._snapping = False

    def on_began(self):
        """Remember the pose the drag starts from, and start snapping when it applies."""
        self._begin_pose = Gf.Matrix4d(self.sender.model.pose)
        self._moved = Gf.Vec3d(0.0)
        self.sender.model.dragging = True
        # Only the center snaps, as in Kit's own gizmo: an axis or plane drag stays on it.
        self._snapping = (
            isinstance(self.gesture_payload, transform.TranslateDragGesturePayload)
            and self.gesture_payload.axis == [1, 1, 1]
            and bool(self._snap_settings.snap_enabled)
            and bool(self._snap_settings.snap_provider)
        )
        if self._snapping:
            # The previews under the gizmo are unpickable already, so nothing is excluded.
            # The gesture is read by the providers only in Viewport 1.
            self._snap_manager.on_began([], gesture=self)

    def on_changed(self):
        """Move the pose by the distance dragged so far, keeping its orientation, or onto
        the surface under the cursor while snapping."""
        if not isinstance(self.gesture_payload, transform.TranslateDragGesturePayload):
            return
        # Summed from the per-update deltas, as Kit's own gizmo does: the gizmo follows
        # the drag, so the payload's running total is measured from a moving origin. The
        # deltas are along the gizmo's axes, which a translation does not turn.
        self._moved += Gf.Vec3d(*self.gesture_payload.moved_delta)
        pose = Gf.Matrix4d(self._begin_pose)
        pose.SetTranslateOnly(
            self._begin_pose.ExtractTranslation()
            + self._begin_pose.RemoveScaleShear().TransformDir(self._moved)
        )
        if self._snapping:
            cursor_ndc = self.sender.transform_space(
                sc.Space.WORLD, sc.Space.NDC, self.gesture_payload.ray_closest_point
            )
            if self._snap_manager.get_snap_pos(
                pose, cursor_ndc, self.sender.scene_view, self._on_snapped
            ):
                return
        self.sender.model.drag_to(pose)

    def _on_snapped(self, position=None, orient: Gf.Rotation | None = None, **_):
        """Move the pose to the snapped world ``position`` and, when the snap toolbar's
        Conform to target is on, turn it to ``orient``.

        Called by the snap providers, possibly a frame after the drag update that asked,
        so a result arriving after the drag has ended is dropped. A result without a
        finite position (nothing under the cursor) leaves the pose where it is.
        """
        if self.state != sc.GestureState.CHANGED:
            return
        if position is None or not all(math.isfinite(value) for value in position):
            return
        pose = Gf.Transform(self.sender.model.pose)
        pose.SetTranslation(Gf.Vec3d(*position))
        if orient is not None:
            pose.SetRotation(orient)
        self.sender.model.drag_to(pose.GetMatrix())

    def on_ended(self):
        """Stop snapping and let the window place the gizmo again."""
        self._stop_snapping()
        self.sender.model.dragging = False

    def on_canceled(self):
        """Stop snapping and let the window place the gizmo again."""
        self._stop_snapping()
        self.sender.model.dragging = False

    def _stop_snapping(self):
        """End the snap providers' drag when the drag snapped."""
        if self._snapping:
            self._snap_manager.on_ended()
            self._snapping = False


class PlacementRotateGesture(transform.RotateChangedGesture):
    """Turns the model's pose about a dragged axis through its position."""

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
        axis = Gf.Vec3d(*payload.axis[:3])
        if not (payload.free_rotation or payload.screen_space):
            # A ring of the gizmo reports its axis in the gizmo's frame; the free and
            # screen-space rotations report theirs in world axes.
            axis = start_pose.RemoveScaleShear().TransformDir(axis)
        orientation = Gf.Matrix4d(start_pose)
        orientation.SetTranslateOnly(Gf.Vec3d(0.0))
        # Gf composes row-vector first: the start orientation, then the world rotation.
        pose = orientation * Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(axis, angle))
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
    caller decides what the pose means. Its axes are the pose's own axes. It offers
    rotation while the viewport's Rotate tool is selected (E) and translation
    otherwise (W, or any other tool). With snapping on and a snap provider (Surface,
    Prim or Grid) selected in the viewport's snap toolbar, dragging its center square
    moves the pose to what the provider finds under the cursor, and turns it to the
    surface when the toolbar's Conform to target is on; dragging an axis or a plane
    does not snap. Dragging it neither selects prims nor adds to undo.

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
        # Built with the scene view: the providers need the viewport it is shown in.
        self._snap_manager: snap.SnapProviderManager | None = None
        # The settings Kit's own gizmo snaps by, so the snap toolbar drives both.
        self._snap_settings = transform.SnapSettingsListener(
            provider_setting_path=snap.settings_constants.SNAP_PROVIDER_NAME_SETTING_PATH
        )
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
            self._snap_manager = snap.SnapProviderManager(
                viewport_api=self._viewport_window.viewport_api
            )
            with self._viewport_window.get_frame(GIZMO_FRAME_NAME):
                self._scene_view = sc.SceneView()
                with self._scene_view.scene:
                    transform.TransformManipulator(
                        model=self._model,
                        gestures=[
                            PlacementTranslateGesture(self._snap_manager, self._snap_settings),
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
        self._snap_manager.destroy()
        self._snap_manager = None
        self._scene_view = None
        self._viewport_window = None
        self._model.dragging = False

    def destroy(self):
        """Remove the gizmo and stop following the viewport's tool and snap settings."""
        self.hide()
        self._snap_settings.destroy()
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
