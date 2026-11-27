# SPDX-License-Identifier: Apache-2.0
"""The Convert to Frame Pose window, which shows a pose as the frame of a TCP the user picks."""

import asyncio

import omni.usd
from fastapi import HTTPException
from isaacsim.gui.components import element_wrappers
from omni import ui
from omni.kit import notification_manager
from omni.kit.window.property import templates
from pxr import Sdf

from ..comm.services.poses import POSE_ROBOT_RELATIONSHIP, POSE_TCP_ATTRIBUTE, PoseService
from .add_asset_window import BUTTON_ROW_GAP, BUTTON_WIDTH, WINDOW_WIDTH

# Height, in pixels, the window opens at: it has fewer rows than the Add windows.
WINDOW_HEIGHT = 130


class ConvertToFramePoseWindow:
    """A window that turns a pose into a frame pose of a TCP picked in its **TCP** dropdown.

    The dropdown lists the robot's default TCP and its TCPs added with Add TCP or Add Tool, and
    starts at the TCP the pose places. **Convert** keeps the robot's joints for the pose and
    moves the pose to where the picked TCP is for them, shown as that TCP's red/green/blue
    arrows, then closes the window and selects the pose. Failures are notified as warnings that
    stay until dismissed; a failed conversion leaves the pose and the window as they were.

    Example:
        window = ConvertToFramePoseWindow(bridge_server.pose_service)
        window.show("/World/vention_table/robot_mount/UR10e_assembly/poses/Hand_E_Pose")
    """

    def __init__(self, pose_service: PoseService):
        self._pose_service = pose_service
        # The pose the window was opened on.
        self._pose_prim_path = ""

        self._window = ui.Window(
            "Convert to Frame Pose", width=WINDOW_WIDTH, height=WINDOW_HEIGHT, visible=False
        )
        with self._window.frame, ui.VStack(spacing=5, height=0):
            with ui.HStack(height=0):
                ui.Label(
                    "Pose",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="The pose that is converted",
                )
                self._pose_model = ui.StringField(read_only=True).model
            self._tcp_dropdown = element_wrappers.DropDown(
                "TCP",
                tooltip="The TCP the frame pose places; the robot's joints for the pose stay the "
                "same",
                add_flourish=False,
            )
            ui.Spacer(height=BUTTON_ROW_GAP)
            with ui.HStack(spacing=5, height=0):
                ui.Spacer()
                self._convert_button = ui.Button(
                    "Convert",
                    width=BUTTON_WIDTH,
                    clicked_fn=lambda: asyncio.ensure_future(self._convert()),
                )
                ui.Spacer()

    def show(self, pose_prim_path: str):
        """Open the window for the pose at ``pose_prim_path``; returns at once.

        The window opens once the pose's robot and its TCPs are found. It stays closed when they
        cannot be found, which is notified.
        """
        asyncio.ensure_future(self._open(pose_prim_path))

    def destroy(self):
        """Release the window's widgets."""
        self._window.destroy()

    async def _open(self, pose_prim_path: str):
        """List the robot's TCPs, pick the one the pose places, and open the window."""
        pose_prim = omni.usd.get_context().get_stage().GetPrimAtPath(pose_prim_path)
        robot_targets = pose_prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
        try:
            if not robot_targets:
                raise HTTPException(422, f"{pose_prim_path} names no robot.")
            robot_prim_path, default_tcp_prim_path = await self._pose_service.find_default_tcp(
                robot_targets[0].pathString
            )
        except HTTPException as error:
            _post_failure(f"Opening Convert to Frame Pose failed: {error.detail}")
            return
        default_tcp_name = Sdf.Path(default_tcp_prim_path).name
        labels = [f"Default TCP ({default_tcp_name})"]
        labels += list(self._pose_service.robot_tcp_paths(robot_prim_path))
        pose_tcp_name = pose_prim.GetAttribute(POSE_TCP_ATTRIBUTE).Get()
        self._tcp_dropdown.set_items(
            labels, select_index=labels.index(pose_tcp_name) if pose_tcp_name in labels else 0
        )
        self._pose_prim_path = pose_prim_path
        self._pose_model.set_value(pose_prim_path)
        self._window.visible = True

    async def _convert(self):
        """Convert the pose for the picked TCP, then close the window and select the pose.
        Convert is disabled while it runs."""
        # The default TCP is the first entry; the others are labelled with the TCP's name.
        tcp_name = (
            None
            if self._tcp_dropdown.get_selection_index() == 0
            else self._tcp_dropdown.get_selection()
        )
        self._convert_button.enabled = False
        try:
            await self._pose_service.convert_pose(self._pose_prim_path, tcp_name, ghost=False)
        except HTTPException as error:
            _post_failure(f"Converting the pose failed: {error.detail}")
            return
        finally:
            self._convert_button.enabled = True
        self._window.visible = False
        omni.usd.get_context().get_selection().set_selected_prim_paths(
            [self._pose_prim_path], True
        )


def _post_failure(message: str):
    """Notify ``message`` as a warning that stays until dismissed."""
    notification_manager.post_notification(
        message,
        hide_after_timeout=False,
        status=notification_manager.NotificationStatus.WARNING,
        button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
    )
