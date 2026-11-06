# SPDX-License-Identifier: Apache-2.0
"""A see-through, physics-free preview of an asset, shown where an Add window would put it."""

from pathlib import Path

import omni.kit.app
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from ..core import asset_loader

GHOST_ROOT_PATH = "/TelekinesisGhost"
GHOST_ASSET_PATH = f"{GHOST_ROOT_PATH}/Asset"
GHOST_MATERIAL_PATH = f"{GHOST_ROOT_PATH}/Material"


class AssetGhost:
    """A see-through copy of one asset that shows where it would land, without being part of the scene.

    The ghost is authored only in the stage's session layer, under
    :data:`GHOST_ROOT_PATH`, so it is never saved, never marks the stage modified and
    stays out of undo. The Stage window does not list it and the viewport cannot select
    it. Its physics is switched off, so a playing timeline neither moves it nor lets
    anything collide with it, and its cameras are switched off, so the viewport draws
    none of them. At most one asset is shown at a time.

    Example:
        ghost = AssetGhost()
        ghost.show(Path("C:/assets/ur10e/ur10e.usd"))
        ghost.place_under(Gf.Matrix4d(1.0))
        ghost.remove()
    """

    def __init__(self):
        extension_path = (
            omni.kit.app.get_app()
            .get_extension_manager()
            .get_extension_path_by_module("telekinesis.isaacsim.bridge")
        )
        self._material_url = Path(extension_path, "data", "ghost.mdl").as_posix()

    def show(self, usd_path: Path):
        """Show the USD asset at ``usd_path`` as the ghost, replacing the one shown.

        The new asset keeps the placement of the one it replaces, and stands at the
        world origin when none was shown.
        """
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            root = UsdGeom.Xform.Define(stage, GHOST_ROOT_PATH)
            omni.usd.editor.set_hide_in_stage_window(root.GetPrim(), True)

            # Authored with UsdShade rather than the CreateMdlMaterialPrim command, which
            # would record the material in the undo history.
            material = UsdShade.Material.Define(stage, GHOST_MATERIAL_PATH)
            shader = UsdShade.Shader.Define(stage, f"{GHOST_MATERIAL_PATH}/Shader")
            shader.SetSourceAsset(Sdf.AssetPath(self._material_url), "mdl")
            shader.SetSourceAssetSubIdentifier("voltest_02", "mdl")
            # Tinted and glowing #41d1ff; MDL reads colours as linear, the hex is sRGB.
            # The glow strength is the one Isaac Sim's conveyor builder gives its preview.
            ghost_color = Gf.ConvertDisplayToLinear(Gf.Vec3f(0x41, 0xD1, 0xFF) / 255.0)
            shader.CreateInput("transmission_color", Sdf.ValueTypeNames.Color3f).Set(
                ghost_color
            )
            shader.CreateInput("emission_color", Sdf.ValueTypeNames.Color3f).Set(
                ghost_color
            )
            shader.CreateInput("emissive_scale", Sdf.ValueTypeNames.Float).Set(300.0)
            shader_output = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
            material.CreateSurfaceOutput("mdl").ConnectToSource(shader_output)
            material.CreateVolumeOutput("mdl").ConnectToSource(shader_output)
            UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(
                material, UsdShade.Tokens.strongerThanDescendants
            )

            stage.RemovePrim(GHOST_ASSET_PATH)
            asset = stage.DefinePrim(GHOST_ASSET_PATH, "Xform")
            asset.GetReferences().AddReference(
                usd_path.as_posix(), asset_loader.asset_root_prim_path(usd_path)
            )
            self._switch_off_physics(asset)
            # The viewport draws every camera with its own frame, which would stand
            # beside the window's gizmo; switching a camera off removes it from the viewport.
            for camera in [prim for prim in Usd.PrimRange(asset) if prim.IsA(UsdGeom.Camera)]:
                camera.SetActive(False)
        omni.usd.get_context().set_pickable(GHOST_ROOT_PATH, False)

    def place_under(self, parent_world: Gf.Matrix4d):
        """Place the ghost as an asset loaded under a prim whose world transform is ``parent_world``.

        The asset keeps the transform it authors, read relative to that prim, exactly as
        when it is loaded there.
        """
        self._set_root_world_transform(parent_world)

    def place_on_mount(self, mount_world: Gf.Matrix4d):
        """Move the ghost so its mount body stands at the world transform ``mount_world``.

        The mount body is the asset's first rigid body -- the base link of an
        articulated gripper, the body of a suction gripper -- or the asset itself when
        it has none, the body a tool or mesh is fixed to a robot's mount link by.
        """
        stage = omni.usd.get_context().get_stage()
        mount_body = self._mount_body(stage.GetPrimAtPath(GHOST_ASSET_PATH))
        root_world = omni.usd.get_world_transform_matrix(
            stage.GetPrimAtPath(GHOST_ROOT_PATH)
        )
        # Gf matrices compose row-vector first: the mount body's pose in the ghost root's
        # frame is divided out of the pose the mount body is to take.
        mount_in_root = (
            omni.usd.get_world_transform_matrix(mount_body) * root_world.GetInverse()
        )
        self._set_root_world_transform(mount_in_root.GetInverse() * mount_world)

    def world_bounds(self) -> Gf.Range3d:
        """Return the axis-aligned world bounds of the shown asset, in metres."""
        stage = omni.usd.get_context().get_stage()
        return (
            UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            .ComputeWorldBound(stage.GetPrimAtPath(GHOST_ASSET_PATH))
            .ComputeAlignedRange()
        )

    def bounds_in_mount_frame(self) -> Gf.Range3d:
        """Return the bounds of the shown asset in its mount body's frame, in metres.

        The mount body is the one :meth:`place_on_mount` places. The result does not
        depend on where the ghost stands.
        """
        stage = omni.usd.get_context().get_stage()
        asset = stage.GetPrimAtPath(GHOST_ASSET_PATH)
        bounds = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
        ).ComputeWorldBound(asset)
        bounds.Transform(
            omni.usd.get_world_transform_matrix(self._mount_body(asset)).GetInverse()
        )
        return bounds.ComputeAlignedRange()

    def camera_in_mount_frame(self) -> Gf.Matrix4d | None:
        """Return the pose of the shown asset's first camera in its mount body's frame,
        or ``None`` when the asset has no camera.

        The camera looks along its own -Z axis. The mount body is the one
        :meth:`place_on_mount` places. The result does not depend on where the ghost stands.
        """
        stage = omni.usd.get_context().get_stage()
        asset = stage.GetPrimAtPath(GHOST_ASSET_PATH)
        # The ghost's cameras are inactive, which the default traversal skips.
        camera = next(
            (
                prim
                for prim in Usd.PrimRange(asset, Usd.PrimAllPrimsPredicate)
                if prim.IsA(UsdGeom.Camera)
            ),
            None,
        )
        if camera is None:
            return None
        return (
            omni.usd.get_world_transform_matrix(camera)
            * omni.usd.get_world_transform_matrix(self._mount_body(asset)).GetInverse()
        )

    def remove(self):
        """Remove the ghost from the stage. Does nothing when none is shown."""
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetSessionLayer().GetPrimAtPath(GHOST_ROOT_PATH):
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.RemovePrim(GHOST_ROOT_PATH)

    @staticmethod
    def _mount_body(asset: Usd.Prim) -> Usd.Prim:
        """Return the asset's first rigid body, or the asset itself when it has none."""
        return next(
            (
                prim
                for prim in Usd.PrimRange(asset)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ),
            asset,
        )

    @staticmethod
    def _set_root_world_transform(world: Gf.Matrix4d):
        """Set the ghost root's world transform; the root sits directly under the stage root."""
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdGeom.Xform(stage.GetPrimAtPath(GHOST_ROOT_PATH)).MakeMatrixXform().Set(
                world
            )

    @staticmethod
    def _switch_off_physics(asset: Usd.Prim):
        """Disable every rigid body, collider, joint and articulation in ``asset``.

        Overrides cannot be authored on instance proxies, and a URDF import instances
        its colliders, so the subtree is un-instanced first; un-instancing a prim can
        reveal instances nested in it, hence the repeat.
        """
        while instances := [prim for prim in Usd.PrimRange(asset) if prim.IsInstance()]:
            for instance in instances:
                instance.SetInstanceable(False)
        for prim in Usd.PrimRange(asset):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            if prim.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                PhysxSchema.PhysxArticulationAPI.Apply(
                    prim
                ).CreateArticulationEnabledAttr(False)
