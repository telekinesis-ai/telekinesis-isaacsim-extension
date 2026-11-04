# SPDX-License-Identifier: Apache-2.0
"""The Add Robot window opened from the right-click Create menu."""

import math
import pathlib

import omni.usd
from pxr import Tf

from ..comm.services.articulations import ArticulationService
from ..comm.services.asset_library import AssetLibraryService
from ..core import asset_loader
from .add_asset_window import AddAssetWindow

ROBOT_CATEGORY = "urdf/robots/manipulators"


class AddRobotWindow(AddAssetWindow):
    """A window that loads a robot arm from the Telekinesis asset library.

    The robot is registered as an articulation at ``<attaching prim>/<model>``, exactly
    as ``PUT /articulations`` would, so HTTP clients can address it by the returned
    articulation id. The catalogue's ``default_joint_configuration`` (degrees) becomes
    its start and reset pose, and its ``simulation_mount_link`` the link Add Tool and
    Add Mesh attach to (the arm's last link when the catalogue names none). When the
    attaching prim is, or lies inside, a rigid body, the robot is fixed to that body
    with a joint and moves with it, and the status line warns about it. A robot cannot
    be added inside another articulation.

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

    async def add_fetched_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Register the robot as an articulation under ``attaching_prim_path``."""
        default_joint_configuration = asset.get("default_joint_configuration")
        is_urdf = pathlib.Path(asset["local_path"]).suffix.lower() == ".urdf"
        articulation = await self._articulation_service.create_articulation(
            f"{attaching_prim_path.rstrip('/')}/{Tf.MakeValidIdentifier(asset['model'])}",
            urdf_path=asset["local_path"] if is_urdf else None,
            usd_path=None if is_urdf else asset["local_path"],
            default_joint_positions=(
                [math.radians(degrees) for degrees in default_joint_configuration]
                if default_joint_configuration is not None
                else None
            ),
            mount_link=asset.get("simulation_mount_link"),
        )
        status = f"Added {articulation['articulation_id']} at {articulation['prim_path']}."
        # Only a robot this add loaded was fixed to the body; one already there is used as is.
        if articulation["prim_source"] == "isaac_usd":
            return status
        rigid_body_path = asset_loader.find_rigid_body_ancestor(
            omni.usd.get_context().get_stage(), articulation["prim_path"]
        )
        if rigid_body_path is not None:
            status += (
                f" Warning: {rigid_body_path} is a rigid body, so the robot is fixed to it with "
                "a joint and moves with it instead of being fixed to the world."
            )
        return status
