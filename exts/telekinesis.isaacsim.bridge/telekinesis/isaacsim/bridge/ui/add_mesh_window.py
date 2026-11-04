# SPDX-License-Identifier: Apache-2.0
"""The Add Mesh window opened from the right-click Create menu."""

import omni.usd
from pxr import Tf

from ..comm.services.articulations import ArticulationService
from ..comm.services.asset_library import AssetLibraryService
from ..comm.services.stage import StageService
from ..core import asset_loader
from .add_asset_window import AddAssetWindow
from .mount_offset_fields import MountOffsetFields

ATTACHMENT_CATEGORY = "usd/attachments"


class AddMeshWindow(AddAssetWindow):
    """A window that loads a mesh from the Telekinesis asset library, mounting it on a robot.

    The mesh is added as ``POST /stage/scene/reference`` would. When the attaching prim
    is part of a robot, the mesh is placed beside the robot, outside any rigid body
    the robot stands in, and fixed to the robot's
    mount link at the entered offset, as ``mount_prim`` does, so it moves with the
    arm. Otherwise it is added under the attaching prim as a static, visual-only
    prim.

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
        mesh_name = Tf.MakeValidIdentifier(asset["model"])
        # Joined by a fixed joint, a mounted mesh sits beside the robot: a body nested
        # under a link would not follow it.
        mesh_path = await self._stage_service.add_to_scene(
            asset["local_path"],
            (
                f"{attaching_prim_path.rstrip('/')}/{mesh_name}"
                if robot_id is None
                else asset_loader.path_beside_clear_of_rigid_bodies(
                    omni.usd.get_context().get_stage(),
                    self._articulation_service.get_device(robot_id).prim_path,
                    mesh_name,
                )
            ),
        )

        if robot_id is None:
            return f"Added {mesh_path}."
        mount = await self._articulation_service.mount_prim(
            robot_id, mesh_path, None, self._mount_offset_fields.transformation(), True
        )
        return f"Added {mesh_path} and mounted it on {robot_id} at {mount['arm_mount_link']}."
