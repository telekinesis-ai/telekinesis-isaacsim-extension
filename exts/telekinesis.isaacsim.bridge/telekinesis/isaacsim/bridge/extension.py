# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.  # pylint: disable=line-too-long
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Kit extension entry point: registers the toolbar UI panel and hosts the bridge server.

This file serves as a basic template for the standard boilerplate operations that make a
UI-based extension appear on the toolbar. Most users will be able to make their desired UI
extension by interacting solely with UIBuilder (see .ui_builder.py).
"""

import asyncio
import gc

import carb
import omni
import omni.kit.commands
import omni.kit.context_menu
import omni.physx as _physx
import omni.timeline
import omni.usd
from fastapi import HTTPException
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni import ui
from omni.kit import notification_manager
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType
from pxr import Sdf, Usd

from .comm.server import BridgeServer
from .comm.services.articulations import ASSEMBLY_ROBOT_RELATIONSHIP, ROBOT_CLASS_NAME_ATTRIBUTE
from .global_variables import BRIDGE_HOST, BRIDGE_PORT, EXTENSION_TITLE
from .ui.add_asset_window import AddAssetWindow
from .ui.add_camera_window import AddCameraWindow
from .ui.add_mesh_window import AddMeshWindow
from .ui.add_pose_from_joint_positions_window import AddPoseFromJointPositionsWindow
from .ui.add_pose_from_tcp_window import AddPoseFromTcpWindow
from .ui.add_robot_window import AddRobotWindow
from .ui.add_tcp_window import AddTcpWindow
from .ui.add_tool_window import AddToolWindow
from .ui.isaacsim_server_window import IsaacSimServerWindow
from .ui.robot_overlay import RobotOverlay
from .ui_builder import UIBuilder


class Extension(omni.ext.IExt):
    """Kit extension lifecycle: builds the toolbar UI, hosts the bridge server, and wires
    UIBuilder's callbacks to Kit's menu, timeline, physics-step, and stage events.

        on_menu_callback: called when the extension is opened
        on_timeline_event: called when the timeline is stopped, paused, or played
        on_physics_step: called on every physics step
        on_stage_event: called when the stage is opened or closed
        cleanup: called when resources such as physics subscriptions should be released
        build_ui: user function that creates the desired UI
    """

    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE,
            width=600,
            height=500,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(
                name=EXTENSION_TITLE,
                onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"),
            ),
            MenuItemDescription(
                name="Telekinesis Isaac Sim Server",
                onclick_action=(ext_id, "ShowIsaacSimServerWindow"),
            ),
        ]

        add_menu_items(self._menu_items, EXTENSION_TITLE)

        # Filled in with User Functions
        self.ui_builder = UIBuilder()

        # Events
        self._usd_context = omni.usd.get_context()
        self._physx_iface = _physx.get_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

        # Host the single bridge server. Clients create articulations here and
        # address them by the articulation_id it returns (see BridgeServer). A bind
        # failure (e.g. port already in use) must never break startup.
        self._bridge_server = BridgeServer(host=BRIDGE_HOST, port=BRIDGE_PORT)
        try:
            self._bridge_server.start()
        except Exception as exc:
            carb.log_error(f"[bridge] bridge server on port {BRIDGE_PORT} failed to start: {exc}")

        # Clear the bridge's device registry whenever the stage is replaced, and restore
        # the devices a stage that is opened stores. This subscription is tied to the
        # server (always active while the extension is loaded), not to the UI panel --
        # the _on_window stage subscription only exists while the window is visible, but
        # the bridge runs regardless.
        self._bridge_stage_event_sub = (
            self._usd_context.get_stage_event_stream().create_subscription_to_pop(
                self._on_bridge_stage_event,
                name="telekinesis bridge device-registry reset",
            )
        )

        # Drive the same service instances as the HTTP routes, so GUI and clients behave alike.
        self._add_robot_window = AddRobotWindow(
            self._bridge_server.asset_library_service,
            self._bridge_server.articulation_service,
        )
        self._add_tool_window = AddToolWindow(
            self._bridge_server.asset_library_service,
            self._bridge_server.articulation_service,
            self._bridge_server.surface_gripper_service,
            self._bridge_server.pose_service,
        )
        self._add_mesh_window = AddMeshWindow(
            self._bridge_server.asset_library_service,
            self._bridge_server.articulation_service,
            self._bridge_server.stage_service,
        )
        self._add_camera_window = AddCameraWindow(
            self._bridge_server.asset_library_service,
            self._bridge_server.articulation_service,
            self._bridge_server.stage_service,
        )
        self._pose_service = self._bridge_server.pose_service
        self._isaacsim_server_window = IsaacSimServerWindow(
            self._pose_service, self._bridge_server.isaacsim_server_client
        )
        # Opened by the menu item of the same action id, added above.
        action_registry.register_action(
            ext_id,
            "ShowIsaacSimServerWindow",
            self._isaacsim_server_window.show,
            description="Show the Telekinesis Isaac Sim Server window",
        )
        self._robot_overlay = RobotOverlay(self._pose_service)
        self._add_pose_from_joint_positions_window = AddPoseFromJointPositionsWindow(
            self._pose_service, self._robot_overlay, self._bridge_server.isaacsim_server_client
        )
        self._add_pose_from_tcp_window = AddPoseFromTcpWindow(
            self._pose_service, self._robot_overlay
        )
        self._add_tcp_window = AddTcpWindow(self._pose_service)

        # Shown at the top of the right-click menu in the Stage panel and viewport. The returned
        # subscription keeps the entry alive; dropping it in on_shutdown removes it.
        extension_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)
        self._create_context_menu = omni.kit.context_menu.add_menu(
            {
                "name": {
                    "Telekinesis Isaac Sim Bridge": [
                        {
                            "name": "Add Robot",
                            # A robot cannot be loaded inside another robot.
                            "show_fn": lambda objects: (
                                not self._clicked_prim_is_robot_part(objects)
                            ),
                            "onclick_fn": lambda objects: self._show_add_asset_window(
                                self._add_robot_window, objects
                            ),
                        },
                        {
                            "name": "Add Tool",
                            "onclick_fn": lambda objects: self._show_add_asset_window(
                                self._add_tool_window, objects
                            ),
                        },
                        {
                            "name": "Add Mesh",
                            "onclick_fn": lambda objects: self._show_add_asset_window(
                                self._add_mesh_window, objects
                            ),
                        },
                        {
                            "name": "Add Camera",
                            "onclick_fn": lambda objects: self._show_add_asset_window(
                                self._add_camera_window, objects
                            ),
                        },
                        {
                            "name": "Add TCP",
                            "show_fn": self._clicked_prim_is_robot_part,
                            "onclick_fn": lambda objects: self._add_tcp_window.show(
                                self._clicked_prim_path(objects)
                            ),
                        },
                        {
                            # Kit ignores a submenu's show_fn and hides the submenu when none
                            # of its entries is shown, so each entry decides for itself.
                            "name": {
                                "Add Pose": [
                                    {
                                        "name": "From Tool",
                                        "show_fn": self._clicked_robot_has_tool,
                                        "onclick_fn": lambda objects: asyncio.ensure_future(
                                            self._create_pose_from_tool(objects)
                                        ),
                                    },
                                    {
                                        "name": "From Joint Positions",
                                        "show_fn": self._clicked_prim_is_robot_part,
                                        "onclick_fn": lambda objects: (
                                            self._add_pose_from_joint_positions_window.show(
                                                self._clicked_prim_path(objects)
                                            )
                                        ),
                                    },
                                    {
                                        "name": "From TCP",
                                        "show_fn": self._clicked_prim_is_robot_part,
                                        "onclick_fn": lambda objects: (
                                            self._add_pose_from_tcp_window.show(
                                                self._clicked_prim_path(objects)
                                            )
                                        ),
                                    },
                                ],
                            },
                        },
                    ],
                },
                "glyph": f"{extension_path}/data/icon.png",
            },
            "MENU",
        )

    def on_shutdown(self):
        """Stop the bridge server and release all subscriptions, menu items, and UI resources."""
        self._create_context_menu = None
        self._add_robot_window.destroy()
        self._add_tool_window.destroy()
        self._add_mesh_window.destroy()
        self._add_camera_window.destroy()
        self._isaacsim_server_window.destroy()
        self._add_pose_from_joint_positions_window.destroy()
        self._add_pose_from_tcp_window.destroy()
        self._add_tcp_window.destroy()
        self._robot_overlay.destroy()
        self._pose_service.follow_stage(None)
        self._bridge_stage_event_sub = None
        if getattr(self, "_bridge_server", None) is not None:
            self._bridge_server.stop()
            self._bridge_server = None
        remove_menu_items(self._menu_items, EXTENSION_TITLE)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")
        action_registry.deregister_action(self.ext_id, "ShowIsaacSimServerWindow")

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible):
        """Subscribe/unsubscribe stage and timeline events as the panel is shown or hidden."""
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        """Rebuild the extension panel in the ScrollingWindow and dock it left of the Viewport."""
        with self._window.frame, ui.VStack(spacing=5, height=0):
            self._build_extension_ui()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self):
        """Toggle the extension window visibility and notify UIBuilder."""
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_event(self, event):
        """Subscribe physics steps on play; unsubscribe on stop; forward to UIBuilder."""
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                self._physx_subscription = self._physx_iface.subscribe_physics_step_events(
                    self._on_physics_step
                )
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None

        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step):
        """Forward each physics step to UIBuilder (only fires while the timeline is playing)."""
        self.ui_builder.on_physics_step(step)

    def _on_bridge_stage_event(self, event):
        """Rebuild the bridge's device registry, follow the opened stage's poses and
        close the Add windows when the stage is opened or closed.

        The device handles held by the bridge belong to the stage that was active
        when they were created; replacing the stage strands them. The registry is
        cleared rather than the server restarted, so the port stays bound, and a
        stage that is opened gets back the devices it stores (see
        ``BridgeServer.restore_devices``). The Add windows' Attaching Prim names a
        prim of the old stage too.
        """
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            if getattr(self, "_bridge_server", None) is not None:
                if event.type == int(StageEventType.OPENED):
                    self._bridge_server.restore_devices()
                else:
                    self._bridge_server.reset_devices()
            self._pose_service.follow_stage(
                omni.usd.get_context().get_stage()
                if event.type == int(StageEventType.OPENED)
                else None
            )
            for window in (
                self._add_robot_window,
                self._add_tool_window,
                self._add_mesh_window,
                self._add_camera_window,
                self._add_pose_from_joint_positions_window,
                self._add_pose_from_tcp_window,
                self._add_tcp_window,
            ):
                window.close()

    def _show_add_asset_window(self, window: AddAssetWindow, objects: dict):
        """Open ``window`` with the right-clicked prim as its Attaching Prim.

        Args:
            window: The Add Robot, Add Tool, Add Mesh or Add Camera window the menu entry opens.
            objects: Kit context-menu payload. ``prim_list`` holds the right-clicked prims, as
                ``Usd.Prim`` or ``Sdf.Path`` depending on the window, and is empty when the click
                was on empty space; the stage's default prim is then the Attaching Prim, or the
                stage root when the stage has none.
        """
        window.show(self._clicked_prim_path(objects))

    async def _create_pose_from_tool(self, objects: dict):
        """Create a pose from the tool of the right-clicked robot, select the new pose, and
        notify the outcome, a failure as a warning that stays until dismissed.

        Args:
            objects: Kit context-menu payload, as for :meth:`_show_add_asset_window`.
        """
        try:
            pose = await self._pose_service.create_pose_from_tool(self._clicked_prim_path(objects))
        except HTTPException as error:
            notification_manager.post_notification(
                f"Adding a pose failed: {error.detail}",
                hide_after_timeout=False,
                status=notification_manager.NotificationStatus.WARNING,
                button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
            )
            return
        omni.usd.get_context().get_selection().set_selected_prim_paths([pose["prim_path"]], True)
        reach = "reachable" if pose["reachable"] else "not reachable by the robot"
        notification_manager.post_notification(f"Added pose {pose['prim_path']} ({reach}).")

    @staticmethod
    def _clicked_prim_path(objects: dict) -> str:
        """Return the path of the right-clicked prim in a Kit context-menu payload.

        ``prim_list`` holds the right-clicked prims, as ``Usd.Prim`` or ``Sdf.Path``, and is
        empty when the click was on empty space; the stage's default prim is then returned, or
        the stage root when the stage has none.
        """
        prim_list = objects.get("prim_list") or []
        if not prim_list:
            default_prim = omni.usd.get_context().get_stage().GetDefaultPrim()
            return str(default_prim.GetPath()) if default_prim else "/"
        if isinstance(prim_list[0], Usd.Prim):
            return str(prim_list[0].GetPath())
        return str(prim_list[0])

    @staticmethod
    def _clicked_robot_prim_path(objects: dict) -> str | None:
        """Return the path of the robot the right-clicked prim in a Kit context-menu payload is
        part of, or ``None`` when it is part of no robot.

        A robot part is the robot prim, which stores its Synapse class name, or a prim under it,
        or a robot's assembly Xform from Add Robot and anything in it, such as the robot's tool;
        the nearest of them decides. Read from the stage only, since Kit asks this synchronously
        every time the menu opens.
        """
        stage = omni.usd.get_context().get_stage()
        for ancestor_path in Sdf.Path(Extension._clicked_prim_path(objects)).GetAncestorsRange():
            ancestor = stage.GetPrimAtPath(ancestor_path)
            if ancestor.HasAttribute(ROBOT_CLASS_NAME_ATTRIBUTE):
                return ancestor_path.pathString
            if ancestor.HasRelationship(ASSEMBLY_ROBOT_RELATIONSHIP):
                robot_targets = ancestor.GetRelationship(ASSEMBLY_ROBOT_RELATIONSHIP).GetTargets()
                if robot_targets:
                    return robot_targets[0].pathString
        return None

    @staticmethod
    def _clicked_prim_is_robot_part(objects: dict) -> bool:
        """Return whether the right-clicked prim in a Kit context-menu payload is part of a
        robot, as :meth:`_clicked_robot_prim_path` defines it."""
        return Extension._clicked_robot_prim_path(objects) is not None

    def _clicked_robot_has_tool(self, objects: dict) -> bool:
        """Return whether the right-clicked prim in a Kit context-menu payload is part of a robot
        carrying a tool added with Add Tool.

        Read without awaiting, since Kit asks this synchronously every time the menu opens: a
        robot the bridge has not registered yet, such as one of a stage saved before the bridge
        stored device ids, counts as carrying none until it is first used.
        """
        robot_prim_path = self._clicked_robot_prim_path(objects)
        if robot_prim_path is None:
            return False
        articulation_service = self._bridge_server.articulation_service
        robot_id = articulation_service.find_device_id(robot_prim_path)
        return (
            robot_id is not None and articulation_service.find_tool_on_robot(robot_id) is not None
        )

    def _on_stage_event(self, event):
        """Clean up physics subscription on stage open/close; forward event to UIBuilder."""
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            self._physx_subscription = None
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        """Delegate UI construction to UIBuilder.build_ui()."""
        self.ui_builder.build_ui()
