# SPDX-License-Identifier: Apache-2.0
"""The Add Pose From TCP window, which teaches a pose by dragging a TCP's frame."""

import asyncio

import omni.usd
from fastapi import HTTPException
from isaacsim.gui.components import element_wrappers
from pxr import Sdf

from ..comm import models
from ..comm.services.poses import PoseService, find_tcp_name
from ..core import robot_assembler
from .frame_placement_window import FramePlacementWindow
from .robot_overlay import RobotOverlay

POSE_TARGET_PREVIEW_PATH = "/TelekinesisPoseTargetPreview"


class AddPoseFromTcpWindow(FramePlacementWindow):
    """A window that teaches a robot a pose by placing the frame one of its TCPs should go to.

    The **TCP** dropdown picks the robot's default TCP or one of its TCPs added with Add TCP;
    opening the window on a TCP picks that TCP. The frame is entered as Position (metres) and
    Rotation (XYZ Euler degrees) in the robot's base frame, the pose a program moves that TCP
    to, and starts where the picked TCP stands now. While the window is open, a preview of the
    frame's red/green/blue axis arrows stands there, with a move/rotate gizmo on it that enters
    where it is dragged to; the robot and its tool, if it has one, are drawn see-through where
    the robot's inverse kinematics puts them with the picked TCP on the frame, solved by the
    Telekinesis Isaac Sim Server whenever the frame or the TCP changes. A line above Add says
    whether the robot reaches the frame, or why it could not be solved. Picking another TCP
    keeps the frame where it is. Nothing in the stage can be selected while the window is open.

    **Add** creates a pose of the picked TCP at the frame, shown as that frame's red/green/blue
    arrows, and leaves the window open for the next pose. A pose out of reach is created too,
    marked unreachable. Failures are notified as warnings that stay until dismissed.

    The preview is authored only in the stage's session layer, under
    :data:`POSE_TARGET_PREVIEW_PATH`, so it is never saved and never marks the stage modified.
    It, the gizmo and the see-through robot are removed when the window closes.

    Example:
        window = AddPoseFromTcpWindow(bridge_server.pose_service, robot_overlay)
        window.show("/World/vention_table/robot_mount/UR10e_assembly/UR10e")
    """

    def __init__(self, pose_service: PoseService, robot_overlay: RobotOverlay):
        self._pose_service = pose_service
        self._robot_overlay = robot_overlay
        # The prim the window was opened on and the robot's tool, None for a robot without one.
        self._robot_part_prim_path = ""
        self._tool_prim_path: str | None = None
        # Prim path of each entry of the TCP dropdown, by its label, the default TCP first.
        self._tcp_paths_by_label: dict[str, str] = {}
        # The last solution found, which seeds the next solve so the arm keeps its configuration.
        self._last_solution: dict | None = None
        # The solve running or last run, and whether the frame or TCP changed since it read them,
        # so the drawn robot always ends at the latest frame.
        self._solve: asyncio.Future | None = None
        self._solve_outdated = False
        super().__init__("Add Pose From TCP", POSE_TARGET_PREVIEW_PATH, "Pose in the base frame")

    def build_extra_rows(self):
        """Add the TCP dropdown."""
        self._tcp_dropdown = element_wrappers.DropDown(
            "TCP",
            tooltip="The TCP the pose places. Add more TCPs with Add TCP",
            on_selection_fn=lambda _: self.on_frame_moved(),
            add_flourish=False,
        )

    async def load_robot(
        self, robot_part_prim_path: str
    ) -> tuple[str, str, models.Transformation]:
        """Find the robot, its tool, base frame and TCPs, pick the TCP the window was opened on
        or the default TCP, and start the frame where that TCP stands now."""
        robot_prim_path, tool_prim_path = await self._pose_service.find_robot_and_tool(
            robot_part_prim_path
        )
        _, default_tcp_prim_path = await self._pose_service.find_default_tcp(robot_prim_path)
        _, base_frame_prim_path = await self._pose_service.find_base_frame(robot_prim_path)
        self._robot_part_prim_path = robot_part_prim_path
        self._tool_prim_path = tool_prim_path
        self._last_solution = None
        self._tcp_paths_by_label = {
            f"Default TCP ({Sdf.Path(default_tcp_prim_path).name})": default_tcp_prim_path,
            **self._pose_service.robot_tcp_paths(robot_prim_path),
        }
        stage = omni.usd.get_context().get_stage()
        opened_tcp_name = find_tcp_name(stage.GetPrimAtPath(robot_part_prim_path), robot_prim_path)
        labels = list(self._tcp_paths_by_label)
        self._tcp_dropdown.set_items(
            labels, select_index=labels.index(opened_tcp_name) if opened_tcp_name else 0
        )

        tcp_world = omni.usd.get_world_transform_matrix(
            stage.GetPrimAtPath(self._tcp_paths_by_label[self._tcp_dropdown.get_selection()])
        )
        base_world = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(base_frame_prim_path))
        # Gf composes row-vector first: the TCP in world, then world into the base frame.
        translation, rotation = robot_assembler.matrix_to_translation_rotation(
            tcp_world * base_world.GetInverse()
        )
        return (
            robot_prim_path,
            base_frame_prim_path,
            models.Transformation(translation=translation, rotation=rotation),
        )

    async def add_frame(self, transformation: models.Transformation) -> str:
        """Create a pose of the picked TCP at ``transformation`` in the base frame."""
        pose = await self._pose_service.create_pose_from_tcp(
            self._robot_part_prim_path, self._selected_tcp_name(), transformation
        )
        if pose["reachable"]:
            return f"Added pose {pose['prim_path']}."
        return f"Added pose {pose['prim_path']} (out of reach)."

    def on_frame_moved(self):
        """Solve the robot reaching the frame with the picked TCP and draw it; returns at once."""
        self._solve_outdated = True
        if self._solve is None or self._solve.done():
            self._solve = asyncio.ensure_future(self._solve_until_current())

    def on_closed(self):
        """Stop drawing the robot reaching the frame."""
        self._robot_overlay.stop_showing_joint_positions()

    def _selected_tcp_name(self) -> str | None:
        """Return the name of the picked TCP, or ``None`` for the default TCP."""
        selected_label = self._tcp_dropdown.get_selection()
        return None if selected_label == next(iter(self._tcp_paths_by_label)) else selected_label

    async def _solve_until_current(self):
        """Solve and draw the robot reaching the frame until the frame and TCP did not change
        during the last solve. The robot is hidden while the frame is out of reach or cannot be
        solved, and the line above Add says which."""
        while self._solve_outdated:
            self._solve_outdated = False
            try:
                solution = await self._pose_service.solve_tcp_pose(
                    self._robot_prim_path,
                    self._selected_tcp_name(),
                    self._transformation_fields.transformation(),
                    self._last_solution,
                )
            except HTTPException as error:
                solution = None
                status = f"The robot could not be solved for the frame: {error.detail}"
            else:
                status = (
                    "The robot reaches the frame."
                    if solution is not None
                    else "The frame is out of the robot's reach."
                )
            # The window may have closed during the solve, which already hid the robot.
            if not self._window.visible:
                return
            self.set_status(status)
            if solution is None:
                self._robot_overlay.stop_showing_joint_positions()
                continue
            self._last_solution = solution
            self._robot_overlay.show_joint_positions(
                self._robot_prim_path, self._tool_prim_path, solution
            )
