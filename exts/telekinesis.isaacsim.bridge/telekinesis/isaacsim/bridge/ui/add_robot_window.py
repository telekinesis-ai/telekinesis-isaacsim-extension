# SPDX-License-Identifier: Apache-2.0
"""The Add Robot window opened from the right-click Create menu."""

import omni.usd
from pxr import Tf

from ..comm.services.articulations import (
    ASSEMBLY_ROBOT_RELATIONSHIP,
    ArticulationService,
)
from ..comm.services.asset_library import AssetLibraryService
from ..core import asset_loader
from .add_asset_window import AddAssetWindow

ROBOT_CATEGORY = "robots/manipulators"


class AddRobotWindow(AddAssetWindow):
    """A window that loads a robot arm from the Telekinesis asset library.

    The robot is registered as an articulation at
    ``<attaching prim>/<model>_assembly/<model>`` (the assembly Xform gets a numeric
    suffix when that name is taken), exactly as ``PUT /articulations`` would, so HTTP
    clients can address it by the returned articulation id. The assembly Xform is
    what is parked clear of other robots, and it holds everything Add Tool and Add
    Mesh later attach to the robot, so moving or deleting it takes them along. It
    targets the robot through ``telekinesis:assemblyRobot``, so Add Tool and Add Mesh
    on the assembly or on anything in it attach to the robot. The catalogue's
    ``simulation_mount_link`` becomes the link Add Tool and Add Mesh attach to; a robot whose catalogue entry names none stores no mount link, and Add
    Tool and Add Mesh refuse to attach to it. When the attaching prim is, or lies
    inside, a rigid body, the robot is fixed to that body with a joint and moves with
    it, and the success message warns about it; what is then attached to the robot
    lands outside that body, and so outside the assembly. A robot cannot be added
    inside another articulation.

    Example:
        window = AddRobotWindow(bridge_server.asset_library_service,
                                bridge_server.articulation_service)
        window.show("/World")
    """

    def __init__(
        self,
        asset_library_service: AssetLibraryService,
        articulation_service: ArticulationService,
    ):
        self._articulation_service = articulation_service
        super().__init__("Add Robot", asset_library_service)

    def asset_categories(self) -> list[str]:
        """Offer the robot arms of the catalogue."""
        return [ROBOT_CATEGORY]

    async def add_selected_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Register the robot as an articulation under ``attaching_prim_path``."""
        robot_name = Tf.MakeValidIdentifier(asset["model"])
        assembly_path = self._free_destination_path(
            attaching_prim_path, f"{robot_name}_assembly"
        )
        articulation = await self._articulation_service.create_articulation(
            f"{assembly_path}/{robot_name}", None, asset_info=asset
        )
        stage = omni.usd.get_context().get_stage()
        stage.GetPrimAtPath(assembly_path).CreateRelationship(
            ASSEMBLY_ROBOT_RELATIONSHIP
        ).SetTargets([articulation["prim_path"]])
        status = (
            f"Added {articulation['articulation_id']} at {articulation['prim_path']}."
        )
        rigid_body_path = asset_loader.find_rigid_body_ancestor(
            stage, articulation["prim_path"]
        )
        if rigid_body_path is not None:
            status += (
                f" Warning: {rigid_body_path} is a rigid body, so the robot is fixed to it with "
                "a joint and moves with it instead of being fixed to the world."
            )
        return status
