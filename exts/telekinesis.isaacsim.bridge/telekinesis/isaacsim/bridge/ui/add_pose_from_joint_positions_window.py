# SPDX-License-Identifier: Apache-2.0
"""The Add Pose From Joint Positions window, which teaches a pose by jogging a robot's joints."""

import asyncio
import contextlib
import functools
import math

import omni.usd
from fastapi import HTTPException
from omni import ui
from omni.kit import notification_manager
from omni.kit.window.property import templates
from pxr import UsdGeom, UsdPhysics

from ..comm.isaacsim_server_client import IsaacSimServerClient, IsaacSimServerError
from ..comm.services.poses import PoseService, find_tcp_name
from ..core import articulation
from ..global_variables import EXTENSION_TITLE
from .add_asset_window import BUTTON_ROW_GAP, BUTTON_WIDTH, WINDOW_HEIGHT, WINDOW_WIDTH
from .robot_overlay import RobotOverlay

# Range, in degrees, of a revolute joint that stores no limits, such as a continuous joint: one
# turn either way reaches every position.
UNLIMITED_REVOLUTE_RANGE = 360.0
# Width, in pixels, of the value field right of each joint's slider.
JOINT_VALUE_FIELD_WIDTH = 80
# Shown below Add while the Telekinesis Isaac Sim Server cannot be reached.
SERVER_UNREACHABLE_WARNING = (
    "The Telekinesis Isaac Sim Server is not connected. Start it with the command "
    "telekinesis-isaacsim-server, then select Connect in the menu "
    f"{EXTENSION_TITLE} > Telekinesis Isaac Sim Server."
)
# The warning's box, as omni.ui's 0xAABBGGRR: a yellow #FFD43B border on the same colour,
# mostly transparent, like the Add windows' move/rotate hint.
WARNING_BORDER_COLOR = 0xFF3BD4FF
WARNING_BACKGROUND_COLOR = 0x333BD4FF


class AddPoseFromJointPositionsWindow:
    """A window that teaches a robot a pose from joint positions.

    Each joint of the robot has a slider and a value field beside it, in degrees for a revolute
    joint and metres for a prismatic one, starting at the robot's current joint positions. Both
    stay within the joint's limits: a typed value outside them is clamped to the nearest limit.
    While the window is open, the robot and its tool, if it has one, are drawn see-through at
    those positions instead of the selected pose's solution.

    **Add** creates a pose where the robot's TCP is at those positions, storing them as the pose's
    solution, and leaves the window open for the next pose. The TCP is the one the window was
    opened on, or the robot's default TCP, as the TCP row shows. The pose is a ghost of
    the whole robot at those positions and its tool, if it has one. The new pose is hidden, as with
    the Stage window's eye icon, unless **Pose Visible After Add** is checked.

    The joints and **Add** are locked, with a yellow notice below saying how to connect, while the
    Telekinesis Isaac Sim Server cannot be reached, since drawing the robot and adding the pose
    both need it. The server is checked whenever the window opens, and the lock follows every
    later request to the server, so connecting in the Telekinesis Isaac Sim Server window
    unlocks the window. Failures are notified as warnings that stay until dismissed.

    Example:
        window = AddPoseFromJointPositionsWindow(
            bridge_server.pose_service, robot_overlay, bridge_server.isaacsim_server_client
        )
        window.show("/World/vention_table/robot_mount/UR10e_assembly/UR10e")
    """

    def __init__(
        self,
        pose_service: PoseService,
        robot_overlay: RobotOverlay,
        server_client: IsaacSimServerClient,
    ):
        self._pose_service = pose_service
        self._robot_overlay = robot_overlay
        self._server_client = server_client
        server_client.on_reachability_changed = self._apply_server_reachability
        # The prim the window was opened on, which names the pose's TCP when it is a TCP; the
        # robot the window teaches and its tool, None for a robot without one. Set each time the
        # window is opened.
        self._robot_part_prim_path = ""
        self._robot_prim_path = ""
        self._tool_prim_path: str | None = None
        # One entry per joint: the joint's name, the value model its slider and field share,
        # and whether the joint is revolute, so its value is in degrees.
        self._joint_values: list[tuple[str, ui.AbstractValueModel, bool]] = []
        # The joints' sliders and fields, locked while the server cannot be reached.
        self._joint_widgets: list[ui.Widget] = []

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
            with ui.HStack(height=0):
                ui.Label(
                    "TCP",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="The TCP the pose places. Right-click a TCP to teach a pose for it",
                )
                self._tcp_model = ui.StringField(read_only=True).model
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
            self._server_warning = ui.ZStack(height=0, visible=False)
            with self._server_warning:
                ui.Rectangle(
                    style={
                        "background_color": WARNING_BACKGROUND_COLOR,
                        "border_color": WARNING_BORDER_COLOR,
                        "border_width": 1,
                        "border_radius": 4,
                    }
                )
                with ui.VStack(height=0):
                    ui.Spacer(height=6)
                    with ui.HStack(height=0):
                        ui.Spacer(width=8)
                        ui.Label(SERVER_UNREACHABLE_WARNING, word_wrap=True)
                        ui.Spacer(width=8)
                    ui.Spacer(height=6)

    def show(self, robot_part_prim_path: str):
        """Open the window for the robot ``robot_part_prim_path`` is part of; returns at once.

        The window opens once the robot, its tool and its current joint positions are found,
        with the joints at those positions, and then checks the server. It stays closed when the
        prim is part of no robot added with Add Robot, which is notified.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool; a TCP of the
                robot to teach poses for that TCP.
        """
        asyncio.ensure_future(self._open(robot_part_prim_path))

    def close(self):
        """Close the window, which stops drawing the robot at the joints' positions."""
        self._window.visible = False

    def destroy(self):
        """Stop following the server's reachability and release the window's widgets."""
        self._server_client.on_reachability_changed = None
        self._window.destroy()

    async def _open(self, robot_part_prim_path: str):
        """Find the robot and its joints, build a slider and value field per joint, open the
        window, and lock or unlock it as the server answers."""
        try:
            robot_prim_path, tool_prim_path = await self._pose_service.find_robot_and_tool(
                robot_part_prim_path
            )
            current_joints = await self._pose_service.current_joint_positions(robot_prim_path)
        except HTTPException as error:
            self._post_failure(f"Opening Add Pose From Joint Positions failed: {error.detail}")
            return
        self._robot_part_prim_path = robot_part_prim_path
        self._robot_prim_path, self._tool_prim_path = robot_prim_path, tool_prim_path
        self._robot_model.set_value(robot_prim_path)
        stage = omni.usd.get_context().get_stage()
        tcp_name = find_tcp_name(stage.GetPrimAtPath(robot_part_prim_path), robot_prim_path)
        self._tcp_model.set_value(tcp_name or "Default TCP")
        joint_prims = articulation.joint_prims(stage, robot_prim_path)
        self._joint_values = []
        self._joint_widgets = []
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
                # One model for the slider and the field keeps them in step, as Isaac Sim's
                # combo_floatfield_slider_builder does.
                value_model = ui.SimpleFloatModel(position)
                with ui.HStack(spacing=5, height=0):
                    ui.Label(
                        joint_name,
                        width=templates.LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Degrees" if revolute else "Metres",
                    )
                    # The value is shown in the field beside the slider, so the slider's own
                    # text is transparent.
                    self._joint_widgets.append(
                        ui.FloatSlider(
                            model=value_model,
                            min=lower_limit,
                            max=upper_limit,
                            style={"color": 0x00000000},
                        )
                    )
                    self._joint_widgets.append(
                        ui.FloatField(model=value_model, width=JOINT_VALUE_FIELD_WIDTH)
                    )
                value_model.add_value_changed_fn(
                    functools.partial(
                        self._on_joint_value_changed,
                        lower_limit=lower_limit,
                        upper_limit=upper_limit,
                    )
                )
                self._joint_values.append((joint_name, value_model, revolute))
        self._window.visible = True
        # Only whether the server answers matters here; the client records it in reachable.
        with contextlib.suppress(IsaacSimServerError):
            await self._server_client.list_robots()
        self._apply_server_reachability(self._server_client.reachable)

    def _apply_server_reachability(self, reachable: bool):
        """Lock the joints and Add, and show the notice, while the server is not ``reachable``;
        otherwise unlock them and, while the window is open, draw the robot at the joints.

        Also called by the server client whenever the server's reachability changes, e.g. when
        Connect in the Telekinesis Isaac Sim Server window reaches a server that was down.
        """
        for widget in self._joint_widgets:
            widget.enabled = reachable
        self._add_button.enabled = reachable
        self._server_warning.visible = not reachable
        if reachable and self._window.visible:
            self._draw_robot_at_joints()

    def _on_joint_value_changed(
        self, value_model: ui.AbstractValueModel, lower_limit: float, upper_limit: float
    ):
        """Clamp a joint's value to its limits, then draw the robot at the joints.

        A value typed outside the limits is replaced by the nearest limit, which changes the
        value again and draws from that call.
        """
        value = value_model.get_value_as_float()
        clamped_value = min(max(value, lower_limit), upper_limit)
        if clamped_value != value:
            value_model.set_value(clamped_value)
            return
        self._draw_robot_at_joints()

    def _joint_positions(self) -> dict:
        """Return the joints' positions as ``{"joint_names": [...], "joint_positions": [...]}``,
        in radians and metres."""
        return {
            "joint_names": [joint_name for joint_name, _, _ in self._joint_values],
            "joint_positions": [
                math.radians(value_model.get_value_as_float())
                if revolute
                else value_model.get_value_as_float()
                for _, value_model, revolute in self._joint_values
            ],
        }

    def _draw_robot_at_joints(self):
        """Draw the robot and its tool at the joints' positions."""
        self._robot_overlay.show_joint_positions(
            self._robot_prim_path, self._tool_prim_path, self._joint_positions()
        )

    def _on_visibility_changed(self, visible: bool):
        """Let the robot overlay follow the selected pose again once the window closes."""
        if not visible:
            self._robot_overlay.stop_showing_joint_positions()

    def _on_add_clicked(self):
        """Add the pose in the background."""
        asyncio.ensure_future(self._add_pose())

    async def _add_pose(self):
        """Create a pose at the joints' positions, hide it unless Pose Visible After Add is
        checked, and notify the outcome. Add is disabled while it runs, and stays disabled
        afterwards if the server could not be reached meanwhile."""
        self._add_button.enabled = False
        try:
            pose = await self._pose_service.create_pose_from_joint_positions(
                self._robot_part_prim_path, self._joint_positions()
            )
        except HTTPException as error:
            self._post_failure(f"Adding a pose failed: {error.detail}")
            return
        finally:
            self._add_button.enabled = bool(self._server_client.reachable)
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
