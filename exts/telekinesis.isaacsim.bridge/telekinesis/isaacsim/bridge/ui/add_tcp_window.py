# SPDX-License-Identifier: Apache-2.0
"""The Add TCP window, which adds a TCP to a robot."""

from omni import ui
from omni.kit.window.property import templates

from ..comm import models
from ..comm.services.poses import PoseService
from .frame_placement_window import FramePlacementWindow

# Name the Name field starts with. Not "tcp", which the Synapse active-TCP marker group under
# the mount link may hold already.
DEFAULT_TCP_NAME = "custom_tcp"
TCP_PREVIEW_PATH = "/TelekinesisTcpPreview"


class AddTcpWindow(FramePlacementWindow):
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
        super().__init__("Add TCP", TCP_PREVIEW_PATH, "Offset from the default TCP")

    def build_extra_rows(self):
        """Add the Name row."""
        with ui.HStack(height=0):
            ui.Label(
                "Name",
                width=templates.LABEL_WIDTH,
                alignment=ui.Alignment.LEFT_CENTER,
                tooltip="The TCP's name, which is also its prim's name",
            )
            self._name_model = ui.StringField().model

    async def load_robot(
        self, robot_part_prim_path: str
    ) -> tuple[str, str, models.Transformation]:
        """Find the robot and its default TCP, and start with Name at ``custom_tcp`` and the
        offset at zero."""
        robot_prim_path, default_tcp_prim_path = await self._pose_service.find_default_tcp(
            robot_part_prim_path
        )
        self._robot_part_prim_path = robot_part_prim_path
        self._name_model.set_value(DEFAULT_TCP_NAME)
        return robot_prim_path, default_tcp_prim_path, models.Transformation()

    async def add_frame(self, transformation: models.Transformation) -> str:
        """Create the TCP named after the Name field at ``transformation``, with its arrows
        shown."""
        tcp_prim_path = await self._pose_service.create_tcp(
            self._robot_part_prim_path,
            self._name_model.get_value_as_string(),
            transformation,
            visible=True,
        )
        return f"Added TCP {tcp_prim_path}."
