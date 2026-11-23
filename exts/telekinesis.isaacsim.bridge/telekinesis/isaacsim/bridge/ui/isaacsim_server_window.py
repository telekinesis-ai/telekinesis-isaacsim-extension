# SPDX-License-Identifier: Apache-2.0
"""The Telekinesis Isaac Sim Server window, which connects the bridge to the server."""

import asyncio

from fastapi import HTTPException
from omni import ui
from omni.kit.window.property import templates

from ..comm.isaacsim_server_client import IsaacSimServerClient, IsaacSimServerError
from ..comm.services.poses import PoseService
from ..global_variables import ISAACSIM_SERVER_URL
from .add_asset_window import BUTTON_ROW_GAP, BUTTON_WIDTH, WINDOW_WIDTH

# Height, in pixels, the window opens at: it has fewer rows than the Add windows.
WINDOW_HEIGHT = 170
# Diameter, in pixels, of the status light.
STATUS_LIGHT_SIZE = 12
# Status light colours, as omni.ui's 0xAABBGGRR: green when the server answers, red otherwise.
CONNECTED_COLOR = 0xFF50AF4C
NOT_CONNECTED_COLOR = 0xFF3643F4


class IsaacSimServerWindow:
    """A window that shows whether the Telekinesis Isaac Sim Server is reachable and connects the
    stage's robots to it.

    The server is checked each time the window is opened and on Connect: the status is green
    Connected when it answered and red Disconnected when it did not; nothing polls the server in
    between. Connect also loads every robot of the stage on the server under a new id, from the
    Synapse class name the robot stores, and lists the loaded robots. A robot that is not
    connected is loaded the first time a pose needs it, so Connect is only needed after the
    server restarted.

    Example:
        window = IsaacSimServerWindow(bridge_server.pose_service, IsaacSimServerClient())
        window.show()
    """

    def __init__(self, pose_service: PoseService, server_client: IsaacSimServerClient):
        self._pose_service = pose_service
        self._server_client = server_client

        self._window = ui.Window(
            "Telekinesis Isaac Sim Server", width=WINDOW_WIDTH, height=WINDOW_HEIGHT, visible=False
        )
        with self._window.frame, ui.VStack(spacing=5, height=0):
            with ui.HStack(height=0):
                ui.Label(
                    "Server",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="URL of the Telekinesis Isaac Sim Server's API",
                )
                ui.StringField(read_only=True).model.set_value(ISAACSIM_SERVER_URL)
            with ui.HStack(height=0):
                ui.Label(
                    "Status",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="Whether the server answered when the window was opened or Connect "
                    "was pressed",
                )
                with ui.VStack(width=STATUS_LIGHT_SIZE):
                    ui.Spacer()
                    self._status_light = ui.Circle(
                        width=STATUS_LIGHT_SIZE,
                        height=STATUS_LIGHT_SIZE,
                        style={"background_color": NOT_CONNECTED_COLOR},
                    )
                    ui.Spacer()
                ui.Spacer(width=8)
                self._status_label = ui.Label("Disconnected", word_wrap=True)
            with ui.HStack(height=0):
                ui.Label(
                    "Robots",
                    width=templates.LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="Robots of the stage the last Connect loaded on the server",
                )
                self._robots_label = ui.Label("", word_wrap=True)
            ui.Spacer(height=BUTTON_ROW_GAP)
            with ui.HStack(spacing=5, height=0):
                ui.Spacer()
                self._connect_button = ui.Button(
                    "Connect", width=BUTTON_WIDTH, clicked_fn=self._on_connect_clicked
                )
                ui.Spacer()

    def show(self):
        """Open the window and check the server in the background."""
        self._window.visible = True
        asyncio.ensure_future(self._check_server())

    def destroy(self):
        """Release the window's widgets."""
        self._window.destroy()

    def _on_connect_clicked(self):
        """Connect in the background."""
        asyncio.ensure_future(self._connect())

    async def _connect(self):
        """Check the server, then load the stage's robots on it and list them.

        A failure to load a robot leaves the status Connected, since the server answered, and
        says why in the robots line.
        """
        self._connect_button.enabled = False
        self._robots_label.text = ""
        try:
            if not await self._check_server():
                return
            try:
                class_names = await self._pose_service.connect_stage_robots()
            except HTTPException as error:
                self._robots_label.text = f"Loading a robot failed: {error.detail}"
                return
            self._robots_label.text = (
                "\n".join(f"{path} ({class_name})" for path, class_name in class_names.items())
                or "The stage has no robot added with Add Robot."
            )
        finally:
            self._connect_button.enabled = True

    async def _check_server(self) -> bool:
        """Ask the server for its robots, show Connected or Disconnected, and return whether the
        server answered."""
        try:
            await self._server_client.list_robots()
        except IsaacSimServerError:
            self._status_light.set_style({"background_color": NOT_CONNECTED_COLOR})
            self._status_label.text = "Disconnected"
            return False
        self._status_light.set_style({"background_color": CONNECTED_COLOR})
        self._status_label.text = "Connected"
        return True
