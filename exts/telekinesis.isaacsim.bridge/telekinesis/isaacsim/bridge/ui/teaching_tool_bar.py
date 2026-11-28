# SPDX-License-Identifier: Apache-2.0
"""The Teaching Tool Bar under the viewport: the selected pose's robot, TCP and pose, a Move to
button, and the motion settings behind a settings button."""

import asyncio
import math
import time

import omni.kit.app
import omni.usd
from fastapi import HTTPException
from omni import ui
from omni.kit import notification_manager
from pxr import Sdf, Usd

from ..comm.services.articulations import ROBOT_DEFAULT_TCP_ATTRIBUTE
from ..comm.services.move_to import MotionCommand, MoveToService
from ..comm.services.poses import (
    POSE_REACHABLE_ATTRIBUTE,
    POSE_ROBOT_RELATIONSHIP,
    POSE_TCP_ATTRIBUTE,
    PoseService,
)

# Layout of the tool bar: the share of the viewport's height it takes when
# docked under it, its row height, and the widths of its fields and icon buttons, in pixels.
DOCKED_HEIGHT_RATIO = 0.1
ROW_HEIGHT = 25
TCP_FIELD_WIDTH = 200
POSE_FIELD_WIDTH = 300
ICON_BUTTON_WIDTH = 30
MOVE_TO_BUTTON_WIDTH = 90
SETTINGS_FIELD_WIDTH = 120
# Grey of secondary text, as omni.ui's 0xAABBGGRR.
SECONDARY_TEXT_COLOR = 0xFF9E9E9E
SETTINGS_ICON = "${glyphs}/cog.svg"
# The Follow button's icon while following and while not.
FOLLOWING_ICON = "${glyphs}/link.svg"
NOT_FOLLOWING_ICON = "${glyphs}/unlink.svg"
# Seconds the followed pose must stay still before the robot moves to it, so a drag is followed
# where it pauses rather than on every frame.
FOLLOW_SETTLE_SECONDS = 0.1
# Joint speed, in degrees per second, the tool bar starts with, and the range it accepts.
DEFAULT_JOINT_SPEED = 40.0
MIN_JOINT_SPEED = 1.0
MAX_JOINT_SPEED = 180.0
# TCP speed along a line, in millimetres per second, the tool bar starts with, and the range it
# accepts; the range is the tool bar's, not the robot's limit.
DEFAULT_LINE_SPEED = 200.0
MIN_LINE_SPEED = 1.0
MAX_LINE_SPEED = 1000.0


class TeachingToolBar:
    """A one-row tool bar docked under the viewport for the selected pose.

    It opens when exactly one pose is selected and hides when anything else is selected, or when
    a stage is opened or closed. On the left it shows the pose's robot; on the right the TCP the
    pose is solved for (the TCP the pose places, or the robot's default TCP for a pose taught
    before poses stored their TCP), greyed out since it cannot be changed here, then a dropdown
    of the robot's poses that selects the picked pose in the stage, a Move to button, a Follow
    button and a settings button. The settings button opens a popup with the motion command,
    Joint P2P to start with or Line, the joint speed Joint P2P moves at in degrees per second,
    40 to start with, accepted from 1 to 180, and the TCP speed Line moves at in millimetres
    per second, 200 to start with, accepted from 1 to 1000. Opened from the menu while no pose
    is selected, it says "Select a pose from the scene".

    Holding Move to moves the pose's robot to the pose with the motion command at its speed
    (see :class:`MoveToService`), and releasing it stops the robot where it is. The button reads
    "Planning..." while the motion is planned and "Executing..." while the robot moves. A move
    that cannot be planned shows a warning. Selecting anything else, or the tool bar hiding,
    stops the move too.

    Clicking Follow makes the robot follow the pose: it moves to the pose at once, and again,
    with the settings of that moment, whenever the pose was moved and has stayed still for
    :data:`FOLLOW_SETTLE_SECONDS`; a move that is running is finished first. While following,
    Move to is disabled and reads "Following", a move that cannot be planned shows its warning
    and the robot keeps following, and selecting another pose follows that pose instead.
    Clicking Follow again stops the robot where it is; selecting anything other than one pose,
    or the tool bar hiding, stops following too.

    Example:
        tool_bar = TeachingToolBar(move_to_service, pose_service)
        tool_bar.show()
    """

    def __init__(self, move_to_service: MoveToService, pose_service: PoseService):
        self._move_to_service = move_to_service
        self._pose_service = pose_service
        # The running move, cancelled to stop the robot, and the button showing its state.
        self._move_to_task: asyncio.Task | None = None
        self._move_to_button: ui.Button | None = None
        # The running follow, cancelled to stop it, the pose it follows, when that pose was last
        # moved without the robot having followed it yet, and the button toggling it.
        self._follow_task: asyncio.Task | None = None
        self._followed_pose_path: str | None = None
        self._followed_pose_moved_at: float | None = None
        self._follow_button: ui.Button | None = None
        self._tool_bar = ui.ToolBar("Teaching Tool Bar", visible=False)
        self._joint_speed_model = ui.SimpleFloatModel(DEFAULT_JOINT_SPEED)
        self._line_speed_model = ui.SimpleFloatModel(DEFAULT_LINE_SPEED)
        self._settings_window = ui.Window(
            "Teaching Settings",
            width=300,
            visible=False,
            auto_resize=True,
            flags=ui.WINDOW_FLAGS_POPUP | ui.WINDOW_FLAGS_NO_COLLAPSE,
        )
        with self._settings_window.frame, ui.VStack(height=0, spacing=8):
            with ui.HStack(spacing=8):
                ui.Label("Motion command", width=ui.Fraction(1))
                self._motion_command_model = ui.ComboBox(
                    0,
                    *[motion_command.value for motion_command in MotionCommand],
                    width=ui.Pixel(SETTINGS_FIELD_WIDTH),
                    tooltip=(
                        "Joint P2P: all joints move together to the pose\n"
                        "Line: the TCP moves along a straight line to the pose"
                    ),
                ).model
            with ui.HStack(spacing=8):
                ui.Label("Joint speed (deg/s)", width=ui.Fraction(1))
                ui.FloatDrag(
                    model=self._joint_speed_model,
                    min=MIN_JOINT_SPEED,
                    max=MAX_JOINT_SPEED,
                    step=1.0,
                    width=ui.Pixel(SETTINGS_FIELD_WIDTH),
                    tooltip="Joint P2P: speed of the fastest-moving joint",
                )
            with ui.HStack(spacing=8):
                ui.Label("Line speed (mm/s)", width=ui.Fraction(1))
                ui.FloatDrag(
                    model=self._line_speed_model,
                    min=MIN_LINE_SPEED,
                    max=MAX_LINE_SPEED,
                    step=1.0,
                    width=ui.Pixel(SETTINGS_FIELD_WIDTH),
                    tooltip="Line: speed of the TCP along the line",
                )
        self._stage_event_subscription = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop(
                self._on_stage_event, name="telekinesis teaching tool bar selection"
            )
        )

    def show(self):
        """Open the tool bar docked under the viewport, for the selected pose or saying "Select a
        pose from the scene"."""
        self._show_pose(_selected_pose())

    def destroy(self):
        """Stop a running move or follow, stop following the selection, and release the tool
        bar's and the popup's widgets."""
        self._stop_follow()
        self._stop_move_to()
        self._stage_event_subscription = None
        self._settings_window.destroy()
        self._tool_bar.destroy()

    def _on_stage_event(self, event):
        """Show the selected pose, or hide when no single pose is selected or the stage is
        replaced."""
        if event.type in (int(omni.usd.StageEventType.OPENED), int(omni.usd.StageEventType.CLOSED)):
            self._hide()
        elif event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            pose_prim = _selected_pose()
            if pose_prim is None:
                self._hide()
            else:
                self._show_pose(pose_prim)

    def _hide(self):
        """Stop a running move or follow and hide the tool bar and its settings popup."""
        self._stop_follow()
        self._stop_move_to()
        self._settings_window.visible = False
        self._tool_bar.visible = False

    def _show_pose(self, pose_prim: Usd.Prim | None):
        """Stop a running move, rebuild the row for ``pose_prim``, or for no pose with ``None``,
        and open the tool bar if it is hidden. A running follow follows ``pose_prim`` instead,
        or stops with ``None``."""
        was_following = self._follow_task is not None
        self._stop_follow()
        # Rebuilding the row drops the held button without its release event.
        self._stop_move_to()
        self._move_to_button = None
        self._follow_button = None
        with (
            self._tool_bar.frame,
            ui.ScrollingFrame(
                height=ui.Pixel(ROW_HEIGHT),
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            ),
            ui.HStack(spacing=4),
        ):
            ui.Spacer(width=ui.Pixel(10))
            if pose_prim is None:
                ui.Label("Select a pose from the scene")
            else:
                self._build_pose_row(pose_prim)
        if was_following and pose_prim is not None:
            self._start_follow(pose_prim.GetPath().pathString)
        if not self._tool_bar.visible:
            self._tool_bar.visible = True
            asyncio.ensure_future(self._dock_under_viewport())

    def _build_pose_row(self, pose_prim: Usd.Prim):
        """Build the robot name, the TCP, the pose dropdown, the Move to button, the Follow
        button and the settings button of ``pose_prim``'s row."""
        robot_targets = pose_prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
        robot_prim = pose_prim.GetStage().GetPrimAtPath(robot_targets[0]) if robot_targets else None
        tcp_name = pose_prim.GetAttribute(POSE_TCP_ATTRIBUTE).Get()
        if not tcp_name:
            default_tcp = (
                robot_prim.GetAttribute(ROBOT_DEFAULT_TCP_ATTRIBUTE).Get() if robot_prim else None
            )
            tcp_name = f"Default TCP ({default_tcp})" if default_tcp else "Default TCP"
        ui.Label(
            robot_prim.GetName() if robot_prim else "No robot",
            style={"color": SECONDARY_TEXT_COLOR},
            tooltip="The robot the pose belongs to",
        )
        ui.Spacer(width=ui.Fraction(1))
        ui.ComboBox(
            0,
            tcp_name,
            enabled=False,
            width=ui.Pixel(TCP_FIELD_WIDTH),
            tooltip="The TCP the robot brings to the pose: the TCP the pose places, or the "
            "robot's default TCP for an older pose",
        )
        pose_paths = (
            _robot_pose_paths(pose_prim.GetStage(), robot_prim.GetPath())
            if robot_prim
            else [pose_prim.GetPath()]
        )
        pose_dropdown = ui.ComboBox(
            pose_paths.index(pose_prim.GetPath()),
            *[path.name for path in pose_paths],
            width=ui.Pixel(POSE_FIELD_WIDTH),
            tooltip="The robot's poses; picking one selects it in the stage",
        )
        pose_dropdown.model.add_item_changed_fn(
            lambda model, _: (
                omni.usd.get_context()
                .get_selection()
                .set_selected_prim_paths(
                    [pose_paths[model.get_item_value_model().as_int].pathString], True
                )
            )
        )
        pose_path = pose_prim.GetPath().pathString
        self._move_to_button = ui.Button(
            "Move to",
            width=ui.Pixel(MOVE_TO_BUTTON_WIDTH),
            height=ui.Fraction(1),
            mouse_pressed_fn=lambda x, y, button, modifier: self._start_move_to(pose_path),
            mouse_released_fn=lambda x, y, button, modifier: self._stop_move_to(),
            tooltip="Hold to move the robot to the pose; release to stop it where it is",
        )
        self._follow_button = ui.Button(
            image_url=NOT_FOLLOWING_ICON,
            width=ui.Pixel(ICON_BUTTON_WIDTH),
            height=ui.Fraction(1),
            clicked_fn=lambda: self._toggle_follow(pose_path),
            style={"margin": 0, "background_color": "transparent"},
            tooltip="Follow: the robot moves to the pose, and again whenever it is moved",
        )
        ui.Button(
            image_url=SETTINGS_ICON,
            width=ui.Pixel(ICON_BUTTON_WIDTH),
            height=ui.Fraction(1),
            mouse_pressed_fn=lambda x, y, button, modifier: self._show_settings(x, y),
            style={"margin": 0, "background_color": "transparent"},
            tooltip="Motion settings",
        )

    def _start_move_to(self, pose_path: str):
        """Start moving the robot of the pose at ``pose_path`` to the pose, stopping a move
        that is still running; nothing happens while following."""
        if self._follow_task is not None:
            return
        self._stop_move_to()
        self._move_to_task = asyncio.ensure_future(self._move_to(pose_path))

    def _stop_move_to(self):
        """Stop the running move, if any, so the robot stops where it is."""
        if self._move_to_task is not None and not self._move_to_task.done():
            self._move_to_task.cancel()
        self._move_to_task = None

    async def _move_to(self, pose_path: str):
        """Move the robot of the pose at ``pose_path`` to the pose with the tool bar's motion
        command at its speed, showing the move's state on the button and a warning if it cannot
        be planned."""
        motion_command = list(MotionCommand)[
            self._motion_command_model.get_item_value_model().as_int
        ]
        if motion_command is MotionCommand.LINE:
            speed = self._line_speed_model.as_float / 1000.0  # mm/s to the server's m/s
        else:
            speed = math.radians(self._joint_speed_model.as_float)
        self._set_move_to_text("Planning...")
        try:
            motion = await self._move_to_service.plan_move_to_pose(pose_path, motion_command, speed)
            self._set_move_to_text("Executing...")
            await self._move_to_service.execute(motion)
        except HTTPException as error:
            notification_manager.post_notification(
                f"Move to failed: {error.detail}",
                status=notification_manager.NotificationStatus.WARNING,
            )
        finally:
            self._set_move_to_text("Move to")

    def _set_move_to_text(self, text: str):
        """Show ``text`` on the Move to button of the current row, or "Following" while
        following."""
        if self._move_to_button is not None:
            self._move_to_button.text = "Following" if self._follow_task is not None else text

    def _toggle_follow(self, pose_path: str):
        """Start following the pose at ``pose_path``, or stop following."""
        if self._follow_task is None:
            self._start_follow(pose_path)
        else:
            self._stop_follow()

    def _start_follow(self, pose_path: str):
        """Follow the pose at ``pose_path``, stopping a running move first, and show it on the
        row's buttons."""
        self._stop_move_to()
        self._followed_pose_path = pose_path
        self._followed_pose_moved_at = None
        self._pose_service.on_pose_moved = self._on_pose_moved
        self._follow_task = asyncio.ensure_future(self._follow())
        if self._follow_button is not None:
            self._follow_button.image_url = FOLLOWING_ICON
        if self._move_to_button is not None:
            self._move_to_button.enabled = False
        self._set_move_to_text("Following")

    def _stop_follow(self):
        """Stop following, if following, so the robot stops where it is, and show it on the
        row's buttons."""
        if self._follow_task is None:
            return
        self._pose_service.on_pose_moved = None
        self._follow_task.cancel()
        self._follow_task = None
        self._followed_pose_path = None
        if self._follow_button is not None:
            self._follow_button.image_url = NOT_FOLLOWING_ICON
        if self._move_to_button is not None:
            self._move_to_button.enabled = True
        self._set_move_to_text("Move to")

    def _on_pose_moved(self, pose_path: Sdf.Path):
        """Note when the followed pose was moved, so :meth:`_follow` moves the robot to it."""
        if pose_path.pathString == self._followed_pose_path:
            self._followed_pose_moved_at = time.monotonic()

    async def _follow(self):
        """Move the robot to the followed pose, then again whenever the pose was moved and has
        stayed still for :data:`FOLLOW_SETTLE_SECONDS`, until cancelled."""
        app = omni.kit.app.get_app()
        await self._move_to(self._followed_pose_path)
        while True:
            await app.next_update_async()
            moved_at = self._followed_pose_moved_at
            if moved_at is not None and time.monotonic() - moved_at >= FOLLOW_SETTLE_SECONDS:
                self._followed_pose_moved_at = None
                await self._move_to(self._followed_pose_path)

    def _show_settings(self, x: float, y: float):
        """Open the settings popup at screen position ``x``, ``y``, where the settings button was
        pressed."""
        self._settings_window.position_x, self._settings_window.position_y = x, y
        self._settings_window.visible = True
        self._settings_window.focus()

    async def _dock_under_viewport(self):
        """Dock the tool bar along the bottom of the viewport."""
        # A window made visible this frame cannot be docked until the next one.
        await omni.kit.app.get_app().next_update_async()
        viewport = ui.Workspace.get_window("Viewport")
        if viewport:
            self._tool_bar.dock_in(viewport, ui.DockPosition.BOTTOM, DOCKED_HEIGHT_RATIO)


def _selected_pose() -> Usd.Prim | None:
    """Return the selected prim if exactly one prim is selected and it is a pose, else ``None``."""
    stage = omni.usd.get_context().get_stage()
    selected = omni.usd.get_context().get_selection().get_selected_prim_paths()
    if stage is None or len(selected) != 1:
        return None
    prim = stage.GetPrimAtPath(selected[0])
    return prim if prim and prim.HasAttribute(POSE_REACHABLE_ATTRIBUTE) else None


def _robot_pose_paths(stage: Usd.Stage, robot_path: Sdf.Path) -> list[Sdf.Path]:
    """Return the paths of the poses of the robot at ``robot_path``, in stage order."""
    return [
        prim.GetPath()
        for prim in stage.Traverse()
        if prim.HasAttribute(POSE_REACHABLE_ATTRIBUTE)
        and robot_path in prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
    ]
