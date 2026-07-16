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
"""The extension's toolbar panel: a minimal status display, built and torn down by
Extension (see .extension.py) as the panel is shown/hidden."""

import omni.timeline
from omni import ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style

from .global_variables import BRIDGE_HOST, BRIDGE_PORT


class UIBuilder:
    """Minimal status panel for the bridge. Robot UI lands in task 2."""

    def __init__(self):
        """Initialise frame/element lists and capture the timeline interface."""
        # Frames are sub-windows that can contain multiple UI elements.
        self.frames = []
        # UI elements created using a UIElementWrapper instance.
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically.
        self._timeline = omni.timeline.get_timeline_interface()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar."""

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)."""

    def on_physics_step(self, step: float):
        """Callback for Physics Step. Physics steps only occur when the timeline is playing."""

    def on_stage_event(self, event):
        """Callback for Stage Events (Open, Close)."""

    def cleanup(self):
        """Called when the stage is closed or the extension is hot reloaded."""
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """Build the bridge status panel. Called whenever the UI window is reopened."""
        status_frame = CollapsableFrame("Bridge Status", collapsed=False)
        with status_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Label("Telekinesis Isaac Sim Bridge")
                ui.Label(f"Bridge server: {BRIDGE_HOST}:{BRIDGE_PORT}")
