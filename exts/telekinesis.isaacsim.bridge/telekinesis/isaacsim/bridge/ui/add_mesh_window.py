# SPDX-License-Identifier: Apache-2.0
"""The Add Mesh window opened from the right-click Create menu."""

from ..comm.services.articulations import ArticulationService
from ..comm.services.asset_library import AssetLibraryService
from ..comm.services.stage import StageService
from .add_asset_window import AddAssetWindow
from .mount_offset_fields import MountOffsetFields

ATTACHMENT_CATEGORY = "usd/attachments"


class AddMeshWindow(AddAssetWindow):
    """A window that loads a mesh from the Telekinesis asset library, mounting it on a robot.

    The mesh is added as ``POST /stage/scene/reference`` would. When the attaching prim
    is part of a robot, the mesh is placed beside the robot, outside any rigid body
    the robot stands in, and fixed to the robot's stored mount link at the entered
    offset, as ``mount_prim`` does, so it moves with the arm. A robot that stores no
    mount link refuses the add before anything is loaded. Otherwise the mesh is added
    under the attaching prim as it is: an OBJ, GLB or glTF mesh arrives without
    physics, a USD asset with whatever physics it authors.

    Example:
        window = AddMeshWindow(bridge_server.asset_library_service,
                               bridge_server.articulation_service,
                               bridge_server.stage_service)
        window.show("/World/Table")
    """

    def __init__(
        self,
        asset_library_service: AssetLibraryService,
        articulation_service: ArticulationService,
        stage_service: StageService,
    ):
        self._articulation_service = articulation_service
        self._stage_service = stage_service
        self._mount_offset_fields: MountOffsetFields
        super().__init__("Add Mesh", asset_library_service)

    def asset_categories(self) -> list[str]:
        """Offer the attachment meshes of the catalogue."""
        return [ATTACHMENT_CATEGORY]

    def build_extra_fields(self):
        """Add the offset the mesh is mounted at, used only when mounting on a robot."""
        self._mount_offset_fields = MountOffsetFields()

    async def add_fetched_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Add the mesh and, when ``attaching_prim_path`` is part of a robot, mount it there."""
        robot_id = await self._articulation_service.find_robot_for_prim(attaching_prim_path)
        robot_prim_path = None
        if robot_id is not None:
            # Checked before anything is loaded, so a robot that stores no mount link
            # fails the add without leaving a mesh behind.
            self._articulation_service.resolve_arm_mount_link(robot_id)
            robot_prim_path = self._articulation_service.get_device(robot_id).prim_path
        mesh_path = await self._stage_service.add_to_scene(
            asset["local_path"],
            self._free_destination_path(attaching_prim_path, asset["model"], robot_prim_path),
        )

        if robot_id is None:
            return f"Added {mesh_path}."
        mount = await self._articulation_service.mount_prim(
            robot_id, mesh_path, None, self._mount_offset_fields.transformation(), True
        )
        return f"Added {mesh_path} and mounted it on {robot_id} at {mount['arm_mount_link']}."
