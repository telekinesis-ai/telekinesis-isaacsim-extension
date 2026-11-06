# SPDX-License-Identifier: Apache-2.0
"""The Add Tool window opened from the right-click Create menu."""

import asyncio

import omni.usd
from fastapi import HTTPException
from omni import ui
from pxr import Gf, Sdf

from ..comm import models
from ..comm.services.articulations import ArticulationService
from ..comm.services.asset_library import AssetLibraryService
from ..comm.services.surface_grippers import SurfaceGripperService
from ..core import robot_assembler
from .add_asset_window import BUTTON_WIDTH, AddAssetWindow

PARALLEL_GRIPPER_CATEGORY = "tools/parallel_grippers"
SUCTION_GRIPPER_CATEGORY = "tools/suction_grippers"


class AddToolWindow(AddAssetWindow):
    """A window that loads a gripper from the Telekinesis asset library, attaching it to a robot.

    A parallel gripper is registered as an articulation (``PUT /articulations``), a
    suction gripper as a surface gripper (``PUT /surface_grippers``). When the
    attaching prim is part of a robot, the gripper is loaded beside the robot -- in
    the robot's ``<model>_assembly`` Xform when Add Robot created one -- outside any
    rigid body the robot stands in, and assembled onto the robot's stored mount
    link at the entered offset, as ``assemble_robot`` does: a parallel gripper is
    merged into the robot's articulation, a suction gripper is bolted on and keeps its
    own id. A robot that stores no mount link refuses the add before anything is
    loaded. Otherwise the gripper is only loaded under the attaching prim. An assembly
    cannot be undone. The window's ghost shows the gripper on the mount link at the
    entered offset, following every edit of it, or at the attaching prim off a robot.

    Auto-calculate Placement enters the offset that stands the gripper in front of
    the robot's flange: the gripper's Z axis along the flange's Z axis, its X axis
    along the flange's X axis, its base origin on the flange axis, and its backmost
    point on the flange face. The flange is the robot's ``tool0`` under the mount
    link (the ROS-Industrial tool flange, Z pointing out of the face) or, when it
    has none, the mount link itself. It assumes the gripper's own +Z points from
    its base towards its fingers; a gripper authored otherwise comes out turned and
    needs the Rotation fixed by hand.

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
        self._surface_gripper_service = surface_gripper_service
        super().__init__("Add Tool", asset_library_service, articulation_service)

    def asset_categories(self) -> list[str]:
        """Offer the parallel and suction grippers of the catalogue."""
        return [PARALLEL_GRIPPER_CATEGORY, SUCTION_GRIPPER_CATEGORY]

    def build_extra_buttons(self):
        """Add Auto-calculate Placement, left of Add."""
        ui.Button(
            "Auto-calculate Placement",
            width=BUTTON_WIDTH,
            clicked_fn=lambda: asyncio.ensure_future(self._auto_calculate_placement()),
        )

    async def _auto_calculate_placement(self):
        """Enter the offset that stands the gripper in front of the robot's flange.

        Needs the ghost of the selected gripper to be shown and up to date, and the
        Attaching Prim to be part of a robot; otherwise the status line says what is
        missing. The result does not depend on the entered offset, so pressing it
        again changes nothing.
        """
        if self._ghost_asset is None or not self._ghost_refresh.done():
            self._status_label.text = (
                "Auto-calculate Placement places the preview: press it once the "
                "gripper shows."
            )
            return
        robot_id = await self._articulation_service.find_robot_for_prim(
            self._attaching_prim_model.get_value_as_string()
        )
        if robot_id is None:
            self._status_label.text = (
                "Auto-calculate Placement needs an Attaching Prim that is part of a robot."
            )
            return
        try:
            arm_mount_path = self._arm_mount_path(robot_id)
        except HTTPException as exc:
            self._status_label.text = f"Auto-calculate Placement failed: {exc.detail}"
            return
        except RuntimeError as exc:
            self._status_label.text = f"Auto-calculate Placement failed: {exc}"
            return

        flange_path = robot_assembler.find_flange_frame(
            omni.usd.get_context().get_stage(), arm_mount_path
        )
        flange_in_mount = (
            self._world_transform(flange_path)
            * self._world_transform(arm_mount_path).GetInverse()
        )
        # Moves the gripper's backmost point onto the flange face.
        distance = -self._ghost.bounds_in_mount_frame().GetMin()[2]
        # Gf composes row-vector first: the gripper in the flange's frame, then the
        # flange in the mount link's frame, where the offset is read.
        placement = (
            Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, distance)) * flange_in_mount
        )
        translation, rotation = robot_assembler.matrix_to_translation_rotation(
            placement
        )
        self._transformation_fields.set_transformation(
            models.Transformation(translation=translation, rotation=rotation)
        )
        self._status_label.text = (
            f"Auto-calculate Placement stood the gripper {distance:.3f} m along the Z "
            f"axis of {Sdf.Path(flange_path).name}."
        )

    async def add_selected_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Load the gripper and, when ``attaching_prim_path`` is part of a robot, assemble it on."""
        robot_id = await self._articulation_service.find_robot_for_prim(
            attaching_prim_path
        )
        robot_prim_path = None
        if robot_id is not None:
            # Checked before anything is loaded, so a robot that stores no mount link
            # fails the add without leaving a gripper behind.
            self._articulation_service.resolve_arm_mount_link(robot_id)
            robot_prim_path = self._articulation_service.get_device(robot_id).prim_path
        tool_path = self._free_destination_path(
            attaching_prim_path, asset["model"], robot_prim_path
        )

        if asset["category"] == SUCTION_GRIPPER_CATEGORY:
            tool = await self._surface_gripper_service.create_surface_gripper(
                tool_path, asset_info=asset
            )
            tool_id = tool["surface_gripper_id"]
        else:
            tool = await self._articulation_service.create_articulation(
                tool_path, None, asset_info=asset
            )
            tool_id = tool["articulation_id"]

        if robot_id is None:
            return f"Added {tool_id} at {tool['prim_path']}."
        assembly = await self._articulation_service.assemble_robot(
            robot_id,
            tool_id,
            None,
            None,
            self._transformation_fields.transformation(),
            True,
        )
        return (
            f"Added {tool_id} at {tool['prim_path']} and attached it to {robot_id} "
            f"at {assembly['arm_mount_link']}."
        )
