# SPDX-License-Identifier: Apache-2.0
"""The Add Pose From Joint Positions window, which teaches a pose by jogging a robot's joints."""

import asyncio
import math

import omni.usd
from fastapi import HTTPException
from omni import ui
from omni.kit import notification_manager
from omni.kit.window.property import templates
from pxr import UsdGeom, UsdPhysics

from ..comm.services.poses import PoseService
from ..core import articulation
from .add_asset_window import BUTTON_ROW_GAP, BUTTON_WIDTH, WINDOW_HEIGHT, WINDOW_WIDTH
from .robot_overlay import RobotOverlay

# Range, in degrees, of a revolute joint that stores no limits, such as a continuous joint: one
# turn either way reaches every position.
UNLIMITED_REVOLUTE_RANGE = 360.0


class AddPoseFromJointPositionsWindow:
    """A window that teaches a robot a pose from joint positions.

    It has one slider per joint of the robot, in degrees for a revolute joint and metres for a
    prismatic one, bounded by the joint's limits and starting at the robot's current joint
    positions. While the window is open, the robot and its tool, if it has one, are drawn
    see-through at the sliders' positions instead of the selected pose's solution. **Add** creates
    a pose where the robot's default TCP is at those positions, storing them as the pose's
    solution, and leaves the window open for the next pose. The pose is a ghost of the tool, or a
    bare ``Xform`` for a robot without a tool. The new pose is hidden, as with the Stage window's eye icon,
    unless **Pose Visible After Add** is checked. Failures are notified as warnings that stay
    until dismissed.

    Example:
        window = AddPoseFromJointPositionsWindow(bridge_server.pose_service, robot_overlay)
        window.show("/World/vention_table/robot_mount/UR10e_assembly/UR10e")
    """

    def __init__(self, pose_service: PoseService, robot_overlay: RobotOverlay):
        self._pose_service = pose_service
        self._robot_overlay = robot_overlay
        # The robot the window teaches and its tool, None for a robot without one; set each time
        # the window is opened.
        self._robot_prim_path = ""
        self._tool_prim_path: str | None = None
        # One entry per joint slider: the joint's name, the slider's model, and whether the
        # joint is revolute, so its slider is in degrees.
        self._joint_sliders: list[tuple[str, ui.AbstractValueModel, bool]] = []

        self._window = ui.Window(
            "Add Pose From Joint Positions",
            width=WINDOW_WIDTH,
            height=WINDOW_HEIGHT,
            visible=False,
        )
        self._window.set_visibility_changed_fn(self._on_visibility_changed)
        with self._window.frame, ui.VStack(spacing=5, height=0):
            with ui.HStack(height=0):
                ui.Label(
                    "Robot",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="The robot the pose is taught to",
                )
                self._robot_model = ui.StringField(read_only=True).model
            self._joints_stack = ui.VStack(spacing=5, height=0)
            with ui.HStack(height=0):
                ui.Label(
                    "Pose Visible After Add",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="Leave the new pose visible; otherwise it is hidden as with the "
                    "Stage window's eye icon",
                )
                self._pose_visible_model = ui.CheckBox().model
            ui.Spacer(height=BUTTON_ROW_GAP)
            with ui.HStack(spacing=5, height=0):
                ui.Spacer()
                self._add_button = ui.Button(
                    "Add", width=BUTTON_WIDTH, clicked_fn=self._on_add_clicked
                )
                ui.Spacer()

    def show(self, robot_part_prim_path: str):
        """Open the window for the robot ``robot_part_prim_path`` is part of; returns at once.

        The window opens once the robot, its tool and its current joint positions are found,
        with the sliders at those positions. It stays closed when the prim is part of no robot
        added with Add Robot, which is notified.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.
        """
        asyncio.ensure_future(self._open(robot_part_prim_path))

    def close(self):
        """Close the window, which stops drawing the robot at the sliders' positions."""
        self._window.visible = False

    def destroy(self):
        """Release the window's widgets."""
        self._window.destroy()

    async def _open(self, robot_part_prim_path: str):
        """Find the robot and its joints, build one slider per joint, and open the window."""
        try:
            robot_prim_path, tool_prim_path = await self._pose_service.find_robot_and_tool(
                robot_part_prim_path
            )
            current_joints = await self._pose_service.current_joint_positions(robot_prim_path)
        except HTTPException as error:
            self._post_failure(f"Opening Add Pose From Joint Positions failed: {error.detail}")
            return
        self._robot_prim_path, self._tool_prim_path = robot_prim_path, tool_prim_path
        self._robot_model.set_value(robot_prim_path)
        joint_prims = articulation.joint_prims(omni.usd.get_context().get_stage(), robot_prim_path)
        self._joint_sliders = []
        self._joints_stack.clear()
        with self._joints_stack:
            for joint_name, position in zip(
                current_joints["joint_names"], current_joints["joint_positions"]
            ):
                # Joints of a gripper merged into the robot's articulation are not the robot's.
                joint_prim = joint_prims.get(joint_name)
                if joint_prim is None:
                    continue
                revolute = joint_prim.IsA(UsdPhysics.RevoluteJoint)
                if revolute:
                    joint = UsdPhysics.RevoluteJoint(joint_prim)
                    position = math.degrees(position)
                elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
                    joint = UsdPhysics.PrismaticJoint(joint_prim)
                else:
                    continue
                lower_limit = joint.GetLowerLimitAttr().Get()
                upper_limit = joint.GetUpperLimitAttr().Get()
                if revolute and (lower_limit is None or upper_limit is None):
                    lower_limit, upper_limit = -UNLIMITED_REVOLUTE_RANGE, UNLIMITED_REVOLUTE_RANGE
                with ui.HStack(height=0):
                    ui.Label(
                        joint_name,
                        width=templates.LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Degrees" if revolute else "Metres",
                    )
                    slider = ui.FloatSlider(min=lower_limit, max=upper_limit)
                slider.model.set_value(position)
                slider.model.add_value_changed_fn(lambda _: self._draw_robot_at_sliders())
                self._joint_sliders.append((joint_name, slider.model, revolute))
        self._window.visible = True
        self._draw_robot_at_sliders()

    def _slider_joint_positions(self) -> dict:
        """Return the sliders' joint positions as ``{"joint_names": [...],
        "joint_positions": [...]}``, in radians and metres."""
        return {
            "joint_names": [joint_name for joint_name, _, _ in self._joint_sliders],
            "joint_positions": [
                math.radians(model.get_value_as_float()) if revolute else model.get_value_as_float()
                for _, model, revolute in self._joint_sliders
            ],
        }

    def _draw_robot_at_sliders(self):
        """Draw the robot and its tool at the sliders' joint positions."""
        self._robot_overlay.show_joint_positions(
            self._robot_prim_path, self._tool_prim_path, self._slider_joint_positions()
        )

    def _on_visibility_changed(self, visible: bool):
        """Let the robot overlay follow the selected pose again once the window closes."""
        if not visible:
            self._robot_overlay.stop_showing_joint_positions()

    def _on_add_clicked(self):
        """Add the pose in the background."""
        asyncio.ensure_future(self._add_pose())

    async def _add_pose(self):
        """Create a pose at the sliders' joint positions, hide it unless Pose Visible After
        Add is checked, and notify the outcome. Add is disabled while it runs."""
        self._add_button.enabled = False
        try:
            pose = await self._pose_service.create_pose_from_joint_positions(
                self._robot_prim_path, self._slider_joint_positions()
            )
        except HTTPException as error:
            self._post_failure(f"Adding a pose failed: {error.detail}")
            return
        finally:
            self._add_button.enabled = True
        if self._pose_visible_model.get_value_as_bool():
            notification_manager.post_notification(f"Added pose {pose['prim_path']}.")
            return
        pose_prim = omni.usd.get_context().get_stage().GetPrimAtPath(pose["prim_path"])
        UsdGeom.Imageable(pose_prim).MakeInvisible()
        notification_manager.post_notification(f"Added pose {pose['prim_path']} (hidden).")

    @staticmethod
    def _post_failure(message: str):
        """Notify ``message`` as a warning that stays until dismissed."""
        notification_manager.post_notification(
            message,
            hide_after_timeout=False,
            status=notification_manager.NotificationStatus.WARNING,
            button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
        )
