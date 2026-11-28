# SPDX-License-Identifier: Apache-2.0
"""The Convert to TCP Pose window, which shows a pose as the frame of a TCP the user picks."""

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
    """A window that turns poses of one robot into frame poses of a TCP picked in its **TCP**
    dropdown.

    The dropdown lists the robot's default TCP and its TCPs added with Add TCP or Add Tool, and
    starts at the TCP the poses place when they all place the same one. **Convert** keeps the
    robot's joints for each pose and moves the pose to where the picked TCP is for them, shown
    as that TCP's red/green/blue arrows, then closes the window and selects the poses. Failures
    are notified as warnings that stay until dismissed; a pose that fails to convert stays as it
    was, the others are still converted, and the window stays open.

    Example:
        window = ConvertToFramePoseWindow(bridge_server.pose_service)
        window.show(["/World/vention_table/robot_mount/UR10e_assembly/poses/Hand_E_Pose"])
    """

    def __init__(self, pose_service: PoseService):
        self._pose_service = pose_service
        # The poses the window was opened on.
        self._pose_prim_paths: list[str] = []

        self._window = ui.Window(
            "Convert to TCP Pose", width=WINDOW_WIDTH, height=WINDOW_HEIGHT, visible=False
        )
        with self._window.frame, ui.VStack(spacing=5, height=0):
            with ui.HStack(height=0):
                ui.Label(
                    "Pose",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="The pose, or the number of poses, that is converted",
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

    def show(self, pose_prim_paths: list[str]):
        """Open the window for the poses at ``pose_prim_paths``; returns at once.

        The window opens once the poses' robot and its TCPs are found. It stays closed when they
        cannot be found or the poses belong to different robots, which is notified.
        """
        asyncio.ensure_future(self._open(pose_prim_paths))

    def destroy(self):
        """Release the window's widgets."""
        self._window.destroy()

    async def _open(self, pose_prim_paths: list[str]):
        """List the poses' robot's TCPs, pick the one the poses place, and open the window."""
        stage = omni.usd.get_context().get_stage()
        pose_prims = [stage.GetPrimAtPath(path) for path in pose_prim_paths]
        robot_paths = []
        try:
            for pose_prim in pose_prims:
                robot_targets = pose_prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
                if not robot_targets:
                    raise HTTPException(422, f"{pose_prim.GetPath()} names no robot.")
                robot_paths.append(robot_targets[0])
            if len(set(robot_paths)) > 1:
                raise HTTPException(
                    422,
                    "The poses belong to different robots. Convert one robot's poses at a time.",
                )
            robot_prim_path, default_tcp_prim_path = await self._pose_service.find_default_tcp(
                robot_paths[0].pathString
            )
        except HTTPException as error:
            _post_failure(f"Opening Convert to TCP Pose failed: {error.detail}")
            return
        default_tcp_name = Sdf.Path(default_tcp_prim_path).name
        labels = [f"Default TCP ({default_tcp_name})"]
        labels += list(self._pose_service.robot_tcp_paths(robot_prim_path))
        pose_tcp_names = {prim.GetAttribute(POSE_TCP_ATTRIBUTE).Get() for prim in pose_prims}
        pose_tcp_name = pose_tcp_names.pop() if len(pose_tcp_names) == 1 else None
        self._tcp_dropdown.set_items(
            labels, select_index=labels.index(pose_tcp_name) if pose_tcp_name in labels else 0
        )
        self._pose_prim_paths = pose_prim_paths
        self._pose_model.set_value(
            pose_prim_paths[0] if len(pose_prim_paths) == 1 else f"{len(pose_prim_paths)} poses"
        )
        self._window.visible = True

    async def _convert(self):
        """Convert the poses for the picked TCP, then close the window and select the poses.
        Convert is disabled while it runs."""
        # The default TCP is the first entry; the others are labelled with the TCP's name.
        tcp_name = (
            None
            if self._tcp_dropdown.get_selection_index() == 0
            else self._tcp_dropdown.get_selection()
        )
        failures = []
        self._convert_button.enabled = False
        try:
            for pose_prim_path in self._pose_prim_paths:
                try:
                    await self._pose_service.convert_pose(pose_prim_path, tcp_name, ghost=False)
                except HTTPException as error:
                    failures.append(f"Converting {pose_prim_path} failed: {error.detail}")
        finally:
            self._convert_button.enabled = True
        if failures:
            _post_failure("\n".join(failures))
            return
        self._window.visible = False
        omni.usd.get_context().get_selection().set_selected_prim_paths(self._pose_prim_paths, True)


def _post_failure(message: str):
    """Notify ``message`` as a warning that stays until dismissed."""
    notification_manager.post_notification(
        message,
        hide_after_timeout=False,
        status=notification_manager.NotificationStatus.WARNING,
        button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
    )
