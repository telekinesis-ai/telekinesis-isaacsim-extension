# SPDX-License-Identifier: Apache-2.0
"""The Add Robot window opened from the right-click Create menu."""

import omni.usd
from fastapi import HTTPException
from omni import ui
from pxr import Gf, Tf, UsdGeom

from ..comm import models
from ..comm.services.articulations import (
    ASSEMBLY_ROBOT_RELATIONSHIP,
    ArticulationService,
)
from ..comm.services.asset_library import AssetLibraryService
from ..core import asset_loader, robot_assembler
from .add_asset_window import BUTTON_WIDTH, AddAssetWindow

ROBOT_CATEGORY = "robots/manipulators"


class AddRobotWindow(AddAssetWindow):
    """A window that loads a robot arm from the Telekinesis asset library.

    The robot is registered as an articulation at
    ``<attaching prim>/<model>_assembly/<model>`` (the assembly Xform gets a numeric
    suffix when that name is taken), exactly as ``PUT /articulations`` would, so HTTP
    clients can address it by the returned articulation id. The assembly Xform is
    placed at the entered offset (meters) and rotation (XYZ Euler degrees), read in
    the attaching prim's frame, where the robot's ghost shows. Auto-calculate
    Placement parks the ghost where loading the robot with no placement would: moved
    along +X clear of the robots beside it and lifted onto the colliders of the prop
    it stands in. It enters the result in the fields, which stay editable. Add puts
    the robot where the ghost stands, except that a robot that would stand inside a
    collider of the prop is still lifted onto it. The assembly holds everything Add
    Tool and Add Mesh later attach to the robot, so moving or deleting it takes them
    along. It targets the robot through ``telekinesis:assemblyRobot``, so Add Tool
    and Add Mesh on the assembly or on anything in it attach to the robot. The
    catalogue's ``simulation_mount_link`` becomes the link Add Tool and Add Mesh
    attach to; a robot whose catalogue entry names none stores no mount link, and Add
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
        super().__init__("Add Robot", asset_library_service, articulation_service)

    def asset_categories(self) -> list[str]:
        """Offer the robot arms of the catalogue."""
        return [ROBOT_CATEGORY]

    def build_extra_buttons(self):
        """Add Auto-calculate Placement, left of Add."""
        ui.Button(
            "Auto-calculate Placement",
            width=BUTTON_WIDTH,
            clicked_fn=self._on_auto_placement_clicked,
        )

    async def transformation_frame(self, attaching_prim_path: str) -> Gf.Matrix4d:
        """Read the robot's placement in the attaching prim's frame."""
        return self._world_transform(attaching_prim_path)

    def place_ghost(self, attaching_prim_path: str, transformation_frame: Gf.Matrix4d):
        """Stand the robot's assembly at the entered placement in the attaching prim's frame."""
        self._ghost.place_under(
            robot_assembler.offset_matrix(self._transformation_fields.transformation())
            * transformation_frame
        )

    async def add_selected_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Register the robot as an articulation in an assembly placed under ``attaching_prim_path``."""
        robot_name = Tf.MakeValidIdentifier(asset["model"])
        assembly_path = self._free_destination_path(
            attaching_prim_path, f"{robot_name}_assembly"
        )
        placement = self._transformation_fields.transformation()
        stage = omni.usd.get_context().get_stage()
        # Placed before the robot is loaded into it, so the robot binds where the ghost
        # stood; the load parks only what it creates, which is now the robot alone.
        assembly = UsdGeom.Xform.Define(stage, assembly_path)
        assembly.AddTranslateOp().Set(Gf.Vec3d(*placement.translation))
        assembly.AddRotateXYZOp().Set(Gf.Vec3f(*placement.rotation))
        try:
            articulation = await self._articulation_service.create_articulation(
                f"{assembly_path}/{robot_name}", None, asset_info=asset
            )
        except (HTTPException, RuntimeError):
            # A load that failed before the robot arrived leaves nothing to hold.
            if not stage.GetPrimAtPath(assembly_path).GetChildren():
                stage.RemovePrim(assembly_path)
            raise
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

    def _on_auto_placement_clicked(self):
        """Park the ghost as loading the robot would, and enter where it ends up in the fields.

        The parking starts from the entered placement, so pressing it again on a
        parked ghost changes nothing. Needs the ghost of the selected robot to be
        shown and up to date; otherwise the status line says what to do.
        """
        if self._ghost_asset is None or not self._ghost_refresh.done():
            self._status_label.text = (
                "Auto-calculate Placement places the preview: press it once the robot "
                "shows."
            )
            return
        attaching_prim_path = self._attaching_prim_model.get_value_as_string()
        assembly_path = self._free_destination_path(
            attaching_prim_path,
            f"{Tf.MakeValidIdentifier(self._ghost_asset['model'])}_assembly",
        )
        shift = asset_loader.parking_shift(
            omni.usd.get_context().get_stage(),
            self._ghost.world_bounds(),
            assembly_path,
        )
        # The parking shift is in world axes; the fields are read in the attaching
        # prim's frame.
        attaching_shift = (
            self._world_transform(attaching_prim_path).GetInverse().TransformDir(shift)
        )
        placement = self._transformation_fields.transformation()
        self._transformation_fields.set_transformation(
            models.Transformation(
                translation=[
                    value + delta
                    for value, delta in zip(placement.translation, attaching_shift)
                ],
                rotation=placement.rotation,
            )
        )
        self._status_label.text = (
            f"Auto-calculate Placement moved the robot {shift[0]:.3f} m along +X and "
            f"{shift[2]:.3f} m along +Z."
        )
