# SPDX-License-Identifier: Apache-2.0
"""The Add Camera window opened from the right-click Create menu."""

import asyncio

import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from omni import ui
from pxr import Gf, Sdf, UsdPhysics

from ..comm import models
from ..comm.services.articulations import (
    ASSEMBLY_ROBOT_RELATIONSHIP,
    ArticulationService,
)
from ..comm.services.asset_library import AssetLibraryService
from ..comm.services.stage import StageService
from ..core import robot_assembler
from .add_asset_window import BUTTON_WIDTH, AddAssetWindow

CAMERA_CATEGORY = "cameras"


class AddCameraWindow(AddAssetWindow):
    """A window that loads a camera from the Telekinesis asset library onto a body or robot.

    The camera is added as a child of the prim it is to ride on, with no rigid body or
    joint of its own, so it follows that prim while simulating and can be moved at any
    time, the timeline playing or not. That prim is the rigid body the attaching prim
    is or lies in, such as a robot link or a mount fixed to one; else, when the
    attaching prim is a robot or its ``<model>_assembly`` Xform, the robot's stored
    mount link, and a robot that stores none refuses the add; else the attaching prim
    itself, where the camera stays put. Position and Rotation are the camera's pose
    in that prim's frame. The window's gizmo stands on the asset's first camera, which
    takes its pictures looking along its own -Z axis, opposite the gizmo's blue Z arrow,
    as a hint below the move/rotate hint says.

    Auto-calculate Placement enters the Rotation that turns that camera to look the
    way the robot's flange points: its Z axis opposite the flange's Z axis and its X
    axis along the flange's X axis. Position is kept, so a camera standing off the
    asset's origin moves as it turns. The flange is found as Add Tool finds it.

    Every mesh of the camera without a collider collides as its convex hull, as part of
    the body it rides on. It adds to that body's mass unless the body sets its own
    mass, as a robot link does: a camera that is to weigh on the arm is mounted with
    ``mount_prim`` instead. The camera is not registered for capture: register its
    camera prim with ``PUT /cameras`` (``GET /cameras/discover`` on the added prim
    lists it). Adding stops the timeline; a timeline that was playing is played again.

    Example:
        window = AddCameraWindow(bridge_server.asset_library_service,
                                 bridge_server.articulation_service,
                                 bridge_server.stage_service)
        window.show("/World/ur10e_assembly/ur10e")
    """

    def __init__(
        self,
        asset_library_service: AssetLibraryService,
        articulation_service: ArticulationService,
        stage_service: StageService,
    ):
        self._stage_service = stage_service
        super().__init__("Add Camera", asset_library_service, articulation_service)

    def asset_categories(self) -> list[str]:
        """Offer the cameras of the catalogue."""
        return [CAMERA_CATEGORY]

    def build_extra_buttons(self):
        """Add Auto-calculate Placement, left of Add."""
        ui.Button(
            "Auto-calculate Placement",
            width=BUTTON_WIDTH,
            clicked_fn=lambda: asyncio.ensure_future(self._auto_calculate_placement()),
        )

    def build_extra_hints(self):
        """Say which way the camera looks."""
        with self._hint_callout():
            ui.Label(
                "The camera looks the opposite way to the gizmo's blue Z arrow.",
                width=0,
            )

    def gizmo_pose_in_asset(self) -> Gf.Matrix4d:
        """Stand the gizmo on the asset's first camera, or on its root when it has none."""
        camera_in_asset = self._ghost.camera_in_mount_frame()
        return Gf.Matrix4d(1.0) if camera_in_asset is None else camera_in_asset

    async def transformation_frame(self, attaching_prim_path: str) -> Gf.Matrix4d:
        """Read Position and Rotation in the frame of the prim the camera is added under."""
        return self._world_transform(
            await self._camera_parent_path(attaching_prim_path)
        )

    async def add_selected_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Add the camera under the prim it rides on, at the entered pose, with colliders."""
        parent_path = await self._camera_parent_path(attaching_prim_path)
        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        # Stops the timeline, so the colliders below are added while PhysX is idle.
        camera_path = await self._stage_service.add_to_scene(
            None, self._free_destination_path(parent_path, asset["model"]), asset
        )
        camera = omni.usd.get_context().get_stage().GetPrimAtPath(camera_path)
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=camera.GetPath(),
            new_transform_matrix=robot_assembler.offset_matrix(
                self._transformation_fields.transformation()
            ),
            old_transform_matrix=omni.usd.get_local_transform_matrix(camera),
        )
        robot_assembler.add_convex_hull_colliders(camera)

        if was_playing:
            timeline.play()
            app = omni.kit.app.get_app()
            await app.next_update_async()
            await app.next_update_async()
            await self._articulation_service.rebind_stale_devices()
        return f"Added {camera_path}."

    async def _auto_calculate_placement(self):
        """Enter the Rotation that turns the asset's first camera to look the way the
        robot's flange points, keeping Position.

        Needs the ghost of the selected camera to be shown and up to date, the asset
        to hold a camera, and the Attaching Prim to be part of a robot; otherwise the
        status line says what is missing. The result does not depend on the entered
        Rotation, so pressing it again changes nothing.
        """
        arm_mount_path = await self._mount_path_for_auto_placement()
        if arm_mount_path is None:
            return
        camera_in_asset = self._ghost.camera_in_mount_frame()
        if camera_in_asset is None:
            self._status_label.text = (
                "Auto-calculate Placement needs a model that holds a camera."
            )
            return

        flange_path = robot_assembler.find_flange_frame(
            omni.usd.get_context().get_stage(), arm_mount_path
        )
        # Half a turn about the flange's X axis turns its Z axis around; a camera looks
        # along its -Z axis, so it then looks out of the flange.
        camera_world = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), 180.0)
        ) * self._world_transform(flange_path)
        transformation_frame = await self.transformation_frame(
            self._attaching_prim_model.get_value_as_string()
        )
        # Gf composes row-vector first: the camera's pose in the asset is divided out of
        # the camera's world pose, and the rest read in the transformation frame.
        placement = (
            camera_in_asset.GetInverse()
            * camera_world
            * transformation_frame.GetInverse()
        ).RemoveScaleShear()
        _, rotation = robot_assembler.matrix_to_translation_rotation(placement)
        self._transformation_fields.set_transformation(
            models.Transformation(
                translation=self._transformation_fields.transformation().translation,
                rotation=rotation,
            )
        )
        self._status_label.text = (
            "Auto-calculate Placement turned the camera to look along the Z axis of "
            f"{Sdf.Path(flange_path).name}."
        )

    async def _camera_parent_path(self, attaching_prim_path: str) -> str:
        """Return the path of the prim a camera added at ``attaching_prim_path`` goes under.

        That is the nearest rigid body at or above the attaching prim, looked for no
        higher than the robot or ``<model>_assembly`` Xform holding it, since a body
        around a robot does not carry what is fixed to the robot's links. Without one,
        it is the stored mount link of the robot the attaching prim is part of, and
        otherwise the attaching prim itself.

        Raises:
            HTTPException: 422 if the attaching prim is part of a robot that stores no
                mount link and lies in no rigid body.
            RuntimeError: If that robot has no prim of its mount link's name.
        """
        stage = omni.usd.get_context().get_stage()
        for path in Sdf.Path(attaching_prim_path).GetAncestorsRange():
            prim = stage.GetPrimAtPath(path)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return path.pathString
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI) or prim.HasRelationship(
                ASSEMBLY_ROBOT_RELATIONSHIP
            ):
                break
        robot_id = await self._articulation_service.find_robot_for_prim(
            attaching_prim_path
        )
        if robot_id is not None:
            return self._arm_mount_path(robot_id)
        return attaching_prim_path
