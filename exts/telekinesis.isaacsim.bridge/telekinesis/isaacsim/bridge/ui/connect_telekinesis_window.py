# SPDX-License-Identifier: Apache-2.0
"""The Connect to Telekinesis window, which connects a robot in the stage to Telekinesis."""

import asyncio

import carb
import omni.usd
from isaacsim.gui.components import element_wrappers
from omni import ui
from omni.kit import notification_manager
from omni.kit.window.property import templates
from pxr import Sdf

from ..comm.isaacsim_server_client import IsaacSimServerClient, IsaacSimServerError
from ..comm.services.articulations import (
    ROBOT_CLASS_NAME_ATTRIBUTE,
    ROBOT_ID_ATTRIBUTE,
    ArticulationService,
)
from .add_asset_window import BUTTON_ROW_GAP, BUTTON_WIDTH, NO_SELECTION_LABEL, WINDOW_WIDTH

# Height, in pixels, the window opens at: it has fewer rows than the Add windows.
WINDOW_HEIGHT = 150


class ConnectTelekinesisWindow:
    """A window that connects a robot in the stage to Telekinesis.

    The window is opened on a right-clicked prim that is a robot or part of one, such as its
    assembly, a link or a mounted tool; the Robot row shows the robot it connects. The user
    picks the robot's model, a Synapse robot class such as ``UniversalRobotsUR10E``, from the
    classes the Telekinesis Isaac Sim Server offers; a robot connected before has its model
    preselected. Connect loads the model on the server under a new id and saves the id and the
    class name on the robot prim as :data:`ROBOT_ID_ATTRIBUTE` and
    :data:`ROBOT_CLASS_NAME_ATTRIBUTE`, so they are kept when the stage is saved. Connecting a
    robot again replaces both, and the server robot of its old id is deleted.

    The server is contacted only when the window opens and when Connect is pressed. When it is
    not reachable, or the right-clicked prim is part of no robot, the status line says so and
    Connect is disabled. The window closes when another stage is opened.

    Example:
        window = ConnectTelekinesisWindow(
            bridge_server.articulation_service, IsaacSimServerClient()
        )
        window.show("/World/UR10e_assembly/UR10e")
    """

    def __init__(
        self, articulation_service: ArticulationService, server_client: IsaacSimServerClient
    ):
        self._articulation_service = articulation_service
        self._server_client = server_client
        # Prim path of the robot Connect connects; None until the window finds one.
        self._robot_prim_path: str | None = None
        # The robot and model-list load of the last show(), cancelled by the next one.
        self._load: asyncio.Future | None = None

        self._window = ui.Window(
            "Connect to Telekinesis", width=WINDOW_WIDTH, height=WINDOW_HEIGHT, visible=False
        )
        with self._window.frame, ui.VStack(spacing=5, height=0):
            with ui.HStack(height=0):
                ui.Label(
                    "Robot",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="Robot that Connect connects to Telekinesis",
                )
                self._robot_path_field = ui.StringField(read_only=True).model
            self._class_name_dropdown = element_wrappers.DropDown(
                "Robot Model",
                tooltip="Synapse robot class the robot is loaded as",
                add_flourish=False,
            )
            ui.Spacer(height=BUTTON_ROW_GAP)
            with ui.HStack(spacing=5, height=0):
                ui.Spacer()
                self._connect_button = ui.Button(
                    "Connect", width=BUTTON_WIDTH, clicked_fn=self._on_connect_clicked
                )
                ui.Spacer()
            self._status_label = ui.Label("", word_wrap=True)

    def show(self, clicked_prim_path: str):
        """Open the window for the robot ``clicked_prim_path`` is part of.

        Returns at once; the robot is found and the model list loaded from the server
        asynchronously, with Connect disabled until both are known.
        """
        self._window.visible = True
        if self._load is not None:
            self._load.cancel()
        self._load = asyncio.ensure_future(self._load_robot_and_class_names(clicked_prim_path))

    def close(self):
        """Close the window. A connect already running carries on."""
        self._window.visible = False

    def destroy(self):
        """Release the window's widgets."""
        self._window.destroy()

    async def _load_robot_and_class_names(self, clicked_prim_path: str):
        """Show the robot ``clicked_prim_path`` is part of and offer the server's robot models,
        with the robot's stored model selected. Enables Connect once both are known."""
        self._connect_button.enabled = False
        self._robot_prim_path = None
        self._robot_path_field.set_value("")
        self._class_name_dropdown.set_items([NO_SELECTION_LABEL], select_index=0)

        robot_id = await self._articulation_service.find_robot_for_prim(clicked_prim_path)
        if robot_id is None:
            self._status_label.text = (
                f"'{clicked_prim_path}' is not part of a robot. Right-click a robot added with "
                "Add Robot."
            )
            return
        robot_prim_path = self._articulation_service.get_device(robot_id).prim_path
        self._robot_path_field.set_value(robot_prim_path)

        self._status_label.text = "Loading robot models..."
        try:
            class_names = await self._server_client.list_robot_class_names()
        except IsaacSimServerError as error:
            self._status_label.text = str(error)
            return
        items = [NO_SELECTION_LABEL, *class_names]
        stored_class_name = (
            omni.usd.get_context()
            .get_stage()
            .GetPrimAtPath(robot_prim_path)
            .GetAttribute(ROBOT_CLASS_NAME_ATTRIBUTE)
            .Get()
        )
        self._class_name_dropdown.set_items(
            items, select_index=items.index(stored_class_name) if stored_class_name in items else 0
        )
        self._robot_prim_path = robot_prim_path
        self._status_label.text = ""
        self._connect_button.enabled = True

    def _on_connect_clicked(self):
        """Check that a model is selected, then connect the robot in the background."""
        class_name = self._class_name_dropdown.get_selection()
        if class_name == NO_SELECTION_LABEL:
            self._status_label.text = "Select a robot model first."
            return
        asyncio.ensure_future(self._connect(self._robot_prim_path, class_name))

    async def _connect(self, robot_prim_path: str, class_name: str):
        """Load ``class_name`` on the server, save the new id and class name on the robot prim,
        and delete the server robot of the id it stored before.

        A failure leaves the window open with the reason in the status line. Failing to delete
        the old server robot only logs a warning: the robot is connected either way, and a server
        that restarted no longer has the old robot (404).
        """
        self._connect_button.enabled = False
        self._status_label.text = f"Loading {class_name} on the Telekinesis Isaac Sim Server..."
        try:
            robot = await self._server_client.create_robot(class_name)
        except IsaacSimServerError as error:
            self._status_label.text = f"Connect failed: {error}"
            self._connect_button.enabled = True
            return

        robot_prim = omni.usd.get_context().get_stage().GetPrimAtPath(robot_prim_path)
        previous_robot_id = robot_prim.GetAttribute(ROBOT_ID_ATTRIBUTE).Get()
        robot_prim.CreateAttribute(ROBOT_ID_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(robot["id"])
        robot_prim.CreateAttribute(ROBOT_CLASS_NAME_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(
            class_name
        )
        if previous_robot_id:
            try:
                await self._server_client.delete_robot(previous_robot_id)
            except IsaacSimServerError as error:
                if error.status_code != 404:
                    carb.log_warn(
                        f"[bridge] could not delete the previous server robot "
                        f"{previous_robot_id} of {robot_prim_path}: {error}"
                    )

        self._window.visible = False
        self._status_label.text = ""
        self._connect_button.enabled = True
        notification_manager.post_notification(
            f"Connected {robot_prim_path} to Telekinesis as {class_name}."
        )
