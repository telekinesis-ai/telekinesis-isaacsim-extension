# SPDX-License-Identifier: Apache-2.0
"""The Add Tool window opened from the right-click Create menu."""

import pathlib

from ..comm.services.articulations import ArticulationService
from ..comm.services.asset_library import AssetLibraryService
from ..comm.services.surface_grippers import SurfaceGripperService
from .add_asset_window import AddAssetWindow
from .mount_offset_fields import MountOffsetFields

PARALLEL_GRIPPER_CATEGORY = "urdf/tools/parallel_grippers"
SUCTION_GRIPPER_CATEGORY = "usd/tools/suction_grippers"


class AddToolWindow(AddAssetWindow):
    """A window that loads a gripper from the Telekinesis asset library, attaching it to a robot.

    A parallel gripper is registered as an articulation (``PUT /articulations``), a
    suction gripper as a surface gripper (``PUT /surface_grippers``). When the
    attaching prim is part of a robot, the gripper is loaded beside the robot, outside
    any rigid body the robot stands in, and assembled onto the robot's stored mount
    link at the entered offset, as ``assemble_robot`` does: a parallel gripper is
    merged into the robot's articulation, a suction gripper is bolted on and keeps its
    own id. A robot that stores no mount link refuses the add before anything is
    loaded. Otherwise the gripper is only loaded under the attaching prim. An assembly
    cannot be undone.

    Example:
        window = AddToolWindow(bridge_server.asset_library_service,
                               bridge_server.articulation_service,
                               bridge_server.surface_gripper_service)
        window.show("/World/ur10e/wrist_3_link")
    """

    def __init__(
        self,
        asset_library_service: AssetLibraryService,
        articulation_service: ArticulationService,
        surface_gripper_service: SurfaceGripperService,
    ):
        self._articulation_service = articulation_service
        self._surface_gripper_service = surface_gripper_service
        self._mount_offset_fields: MountOffsetFields
        super().__init__("Add Tool", asset_library_service)

    def asset_categories(self) -> list[str]:
        """Offer the parallel and suction grippers of the catalogue."""
        return [PARALLEL_GRIPPER_CATEGORY, SUCTION_GRIPPER_CATEGORY]

    def build_extra_fields(self):
        """Add the offset the gripper is attached at, used only when attaching to a robot."""
        self._mount_offset_fields = MountOffsetFields()

    async def add_fetched_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Load the gripper and, when ``attaching_prim_path`` is part of a robot, assemble it on."""
        robot_id = await self._articulation_service.find_robot_for_prim(attaching_prim_path)
        robot_prim_path = None
        if robot_id is not None:
            # Checked before anything is loaded, so a robot that stores no mount link
            # fails the add without leaving a gripper behind.
            self._articulation_service.resolve_arm_mount_link(robot_id)
            robot_prim_path = self._articulation_service.get_device(robot_id).prim_path
        tool_path = self._free_destination_path(attaching_prim_path, asset["model"], robot_prim_path)

        if asset["category"] == SUCTION_GRIPPER_CATEGORY:
            tool = await self._surface_gripper_service.create_surface_gripper(
                tool_path, asset["local_path"]
            )
            tool_id = tool["surface_gripper_id"]
        else:
            is_urdf = pathlib.Path(asset["local_path"]).suffix.lower() == ".urdf"
            tool = await self._articulation_service.create_articulation(
                tool_path,
                urdf_path=asset["local_path"] if is_urdf else None,
                usd_path=None if is_urdf else asset["local_path"],
            )
            tool_id = tool["articulation_id"]

        if robot_id is None:
            return f"Added {tool_id} at {tool['prim_path']}."
        assembly = await self._articulation_service.assemble_robot(
            robot_id, tool_id, None, None, self._mount_offset_fields.transformation(), True
        )
        return (
            f"Added {tool_id} at {tool['prim_path']} and attached it to {robot_id} "
            f"at {assembly['arm_mount_link']}."
        )
