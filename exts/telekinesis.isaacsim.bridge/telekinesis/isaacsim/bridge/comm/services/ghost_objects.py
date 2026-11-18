# SPDX-License-Identifier: Apache-2.0
"""Ghost objects: see-through copies of a robot's tool that mark where the tool should go."""

import math

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

from ...core import asset_loader, robot_assembler
from ..isaacsim_server_client import IsaacSimServerClient, IsaacSimServerError
from .articulations import ROBOT_ID_ATTRIBUTE, ArticulationService

# Properties of a ghost object prim. The relationship targets the robot prim whose default TCP
# the ghost places; the joint attributes hold the robot's inverse-kinematics solution for the
# ghost, in radians; ``reachable`` says whether there is one.
GHOST_ROBOT_RELATIONSHIP = "telekinesis:ghost:robot"
GHOST_JOINT_NAMES_ATTRIBUTE = "telekinesis:ghost:joint_names"
GHOST_JOINT_POSITIONS_ATTRIBUTE = "telekinesis:ghost:joint_positions"
GHOST_REACHABLE_ATTRIBUTE = "telekinesis:ghost:reachable"

# Name of the Xform beside the robot that holds its ghost objects.
GHOSTS_GROUP_NAME = "ghosts"
# Look of a ghost object: Wandelbots' reachability tints on Kit's SimPBR_Translucent material,
# the material their ghosts use. It is in Kit's core MDL library, so a saved stage finds it by
# name on any Isaac Sim install.
REACHABLE_COLOR = Gf.Vec3f(0.15, 0.8, 0.25)
UNREACHABLE_COLOR = Gf.Vec3f(0.9, 0.15, 0.15)
GHOST_MDL = "SimPBR_Translucent.mdl"


class GhostObjectError(Exception):
    """Raised when a ghost object cannot be created. The message is meant for the user."""


class GhostObjectService:
    """Creates ghost objects of the tools mounted on robots connected to Telekinesis.

    A ghost object is a single ``Mesh`` prim: the tool's visible meshes merged into the frame of
    the robot's default TCP, so the ghost's origin is where that TCP has to go for the real tool
    to cover the ghost. Ghosts live in a ``ghosts`` Xform beside the robot, in its assembly when
    it has one, carry no physics, and are saved with the stage. Each ghost stores the robot's
    inverse-kinematics solution for it, solved by the Telekinesis Isaac Sim Server, and is tinted
    green when the robot can reach it and red when it cannot.

    Example:
        service = GhostObjectService(bridge_server.articulation_service, IsaacSimServerClient())
        ghost = await service.create_ghost_object(
            "/World/vention_table/robot_mount/UR10e_assembly/Robotiq_Hand_E"
        )
        ghost["reachable"]  # True
    """

    def __init__(
        self, articulation_service: ArticulationService, server_client: IsaacSimServerClient
    ):
        self._articulation_service = articulation_service
        self._server_client = server_client

    async def create_ghost_object(self, tool_prim_path: str) -> dict:
        """Create a ghost object of the tool at ``tool_prim_path``, where the tool stands now.

        The tool must be mounted on a robot connected to Telekinesis. The robot's inverse
        kinematics for the ghost is solved before anything is written, so a failure leaves the
        stage unchanged. It is seeded with the robot's current joints, so the solution keeps the
        robot's current arm configuration.

        Args:
            tool_prim_path: The tool's prim, or any prim of it.

        Returns:
            ``{"prim_path": ..., "reachable": ...}`` of the new ghost object.

        Raises:
            GhostObjectError: If the stage is not in metres, the prim is not a tool mounted on a
                robot, the robot is not connected or no longer known to the server, the robot does
                not match its connected model, the tool has no meshes, or the server cannot be
                reached.
        """
        stage = omni.usd.get_context().get_stage()
        if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
            # The bridge loads robots at metre scale whatever the stage unit, so a pose read in
            # another unit would not match the robot.
            raise GhostObjectError("Ghost objects need a stage in metres (metersPerUnit = 1).")

        articulation_id = await self._articulation_service.find_robot_for_prim(tool_prim_path)
        robot_prim_path = (
            None
            if articulation_id is None
            else self._articulation_service.get_device(articulation_id).prim_path
        )
        # The clicked prim must be the tool: not the robot or inside it, and not a prim holding
        # the robot, such as its assembly.
        if (
            robot_prim_path is None
            or Sdf.Path(tool_prim_path).HasPrefix(Sdf.Path(robot_prim_path))
            or Sdf.Path(robot_prim_path).HasPrefix(Sdf.Path(tool_prim_path))
        ):
            raise GhostObjectError(
                f"'{tool_prim_path}' is not a tool mounted on a robot. Right-click a tool added "
                "with Add Tool."
            )
        robot_id = stage.GetPrimAtPath(robot_prim_path).GetAttribute(ROBOT_ID_ATTRIBUTE).Get()
        if not robot_id:
            raise GhostObjectError(
                f"{robot_prim_path} is not connected. Right-click the robot and choose Connect to "
                "Telekinesis first."
            )
        robot = await self._call_server(self._server_client.describe_robot(robot_id))

        default_tcp_world = self._world_transform_of_robot_frame(
            stage, robot_prim_path, robot["default_tcp"], robot["class_name"]
        )
        base_world = self._world_transform_of_robot_frame(
            stage, robot_prim_path, robot["base_frame"], robot["class_name"]
        )
        points, face_vertex_counts, face_vertex_indices = _merge_meshes_in_frame(
            stage, tool_prim_path, default_tcp_world
        )
        if not face_vertex_counts:
            raise GhostObjectError(f"'{tool_prim_path}' has no visible meshes to show as a ghost.")

        translation, rotation = robot_assembler.matrix_to_translation_rotation(
            default_tcp_world * base_world.GetInverse()
        )
        solution = await self._call_server(
            self._server_client.inverse_kinematics(
                robot_id,
                [translation + rotation],
                self._current_joints(stage, articulation_id, robot_prim_path),
            )
        )
        joint_positions = solution["joint_positions"][0]

        ghosts_path = asset_loader.path_beside_clear_of_rigid_bodies(
            stage, robot_prim_path, GHOSTS_GROUP_NAME
        )
        UsdGeom.Xform.Define(stage, ghosts_path)
        ghost_path = omni.usd.get_stage_next_free_path(
            stage,
            f"{ghosts_path}/{Tf.MakeValidIdentifier(Sdf.Path(tool_prim_path).name)}_ghost",
            False,
        )
        ghost = UsdGeom.Mesh.Define(stage, ghost_path)
        ghost.CreatePointsAttr(points)
        ghost.CreateFaceVertexCountsAttr(face_vertex_counts)
        ghost.CreateFaceVertexIndicesAttr(face_vertex_indices)
        local_transform = Gf.Transform(
            default_tcp_world
            * omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(ghosts_path)).GetInverse()
        )
        ghost.AddTranslateOp().Set(local_transform.GetTranslation())
        ghost.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(local_transform.GetRotation().GetQuat())
        )
        ghost.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(local_transform.GetScale())

        ghost_prim = ghost.GetPrim()
        ghost_prim.CreateRelationship(GHOST_ROBOT_RELATIONSHIP).SetTargets([robot_prim_path])
        ghost_prim.CreateAttribute(GHOST_JOINT_NAMES_ATTRIBUTE, Sdf.ValueTypeNames.StringArray).Set(
            solution["joint_names"]
        )
        ghost_prim.CreateAttribute(
            GHOST_JOINT_POSITIONS_ATTRIBUTE, Sdf.ValueTypeNames.DoubleArray
        ).Set(joint_positions or [])
        reachable = joint_positions is not None
        ghost_prim.CreateAttribute(GHOST_REACHABLE_ATTRIBUTE, Sdf.ValueTypeNames.Bool).Set(
            reachable
        )
        _set_ghost_look(stage, ghost_prim, reachable)
        return {"prim_path": ghost_path, "reachable": reachable}

    def _current_joints(self, stage: Usd.Stage, articulation_id: str, robot_prim_path: str) -> dict:
        """Return the robot's current joints in radians, as an IK seed.

        The seed keeps the solution on the robot's current arm configuration; without it the
        solver may return another configuration reaching the same pose. While the simulation
        runs, the joints are read from it. Otherwise they are the joint states stored in the
        stage, which is where the robot stands while the simulation is stopped: PhysX's
        ``state:<axis>:physics:position`` on every joint under the robot, in degrees for a
        revolute joint (USD physics' angular unit) and metres for a prismatic one, and 0, the
        schema's fallback, where none is stored. The server ignores names that are not joints of
        its robot.
        """
        state = self._articulation_service.get_articulation_state(articulation_id)
        if state is not None:
            return {
                "joint_names": list(
                    self._articulation_service.get_device(articulation_id).dof_names
                ),
                "joint_positions": state["joint_positions"],
            }
        joint_names, joint_positions = [], []
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
            if prim.IsA(UsdPhysics.RevoluteJoint):
                position = prim.GetAttribute("state:angular:physics:position").Get()
                joint_positions.append(math.radians(position or 0.0))
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                position = prim.GetAttribute("state:linear:physics:position").Get()
                joint_positions.append(position or 0.0)
            else:
                continue
            joint_names.append(prim.GetName())
        return {"joint_names": joint_names, "joint_positions": joint_positions}

    @staticmethod
    def _world_transform_of_robot_frame(
        stage: Usd.Stage, robot_prim_path: str, frame_name: str, class_name: str
    ) -> Gf.Matrix4d:
        """Return the world transform of the robot's prim named ``frame_name``.

        Raises:
            GhostObjectError: If the robot has no such prim, so it does not match ``class_name``.
        """
        try:
            frame_path = robot_assembler.find_prim_by_name(stage, robot_prim_path, frame_name)
        except RuntimeError as error:
            raise GhostObjectError(
                f"{robot_prim_path} has no '{frame_name}', so it does not match its connected "
                f"model {class_name}. Connect it to Telekinesis with the right model."
            ) from error
        return omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(frame_path))

    @staticmethod
    async def _call_server(request):
        """Await a server request, turning its failure into a :class:`GhostObjectError`.

        A 404 means the server no longer has the robot, e.g. because it restarted.
        """
        try:
            return await request
        except IsaacSimServerError as error:
            if error.status_code == 404:
                raise GhostObjectError(
                    "The Telekinesis Isaac Sim Server no longer has this robot, e.g. because it "
                    "restarted. Right-click the robot and choose Connect to Telekinesis again."
                ) from error
            raise GhostObjectError(str(error)) from error


def _merge_meshes_in_frame(
    stage: Usd.Stage, root_path: str, frame_world: Gf.Matrix4d
) -> tuple[Vt.Vec3fArray, Vt.IntArray, Vt.IntArray]:
    """Merge every visible ``Mesh`` under ``root_path`` into one mesh in the frame ``frame_world``.

    Meshes inside instances are included. Meshes that are invisible or whose purpose is guide or
    proxy, such as collision meshes, are skipped, as are gprims that are not meshes.

    Returns:
        The merged mesh's points, face vertex counts and face vertex indices.
    """
    xform_cache = UsdGeom.XformCache()
    frame_from_world = np.array(frame_world.GetInverse())
    points, face_vertex_counts, face_vertex_indices = [], [], []
    point_count = 0
    for prim in Usd.PrimRange(
        stage.GetPrimAtPath(root_path), Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
    ):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        imageable = UsdGeom.Imageable(prim)
        if (
            imageable.ComputeVisibility() == UsdGeom.Tokens.invisible
            or imageable.ComputePurpose()
            not in (
                UsdGeom.Tokens.default_,
                UsdGeom.Tokens.render,
            )
        ):
            continue
        mesh = UsdGeom.Mesh(prim)
        mesh_points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not mesh_points or not counts or not indices:
            continue
        # Gf matrices act on row vectors: mesh frame -> world -> the target frame.
        mesh_to_frame = np.array(xform_cache.GetLocalToWorldTransform(prim)) @ frame_from_world
        homogeneous = np.hstack(
            [np.array(mesh_points, dtype=float), np.ones((len(mesh_points), 1))]
        )
        points.append((homogeneous @ mesh_to_frame)[:, :3])
        face_vertex_counts.append(np.array(counts))
        face_vertex_indices.append(np.array(indices) + point_count)
        point_count += len(mesh_points)
    if not points:
        return Vt.Vec3fArray(), Vt.IntArray(), Vt.IntArray()
    return (
        Vt.Vec3fArray.FromNumpy(np.concatenate(points).astype(np.float32)),
        Vt.IntArray.FromNumpy(np.concatenate(face_vertex_counts).astype(np.int32)),
        Vt.IntArray.FromNumpy(np.concatenate(face_vertex_indices).astype(np.int32)),
    )


def _set_ghost_look(stage: Usd.Stage, ghost_prim: Usd.Prim, reachable: bool):
    """Give ``ghost_prim`` its own see-through material, green if ``reachable`` and red if not.

    The material lives under the ghost, at ``<ghost>/Looks/GhostMaterial``, so a duplicated
    ghost brings its own; it is bound again on every call, so a duplicate whose binding still
    targets the original's material gets its own.
    """
    material_path = ghost_prim.GetPath().AppendPath("Looks/GhostMaterial")
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path.AppendChild("Shader"))
    shader.SetSourceAsset(Sdf.AssetPath(GHOST_MDL), "mdl")
    shader.SetSourceAssetSubIdentifier("SimPBR_Translucent", "mdl")
    color = REACHABLE_COLOR if reachable else UNREACHABLE_COLOR
    # Wandelbots' ghost material settings (their wb_ghost.mdl), with our reachability colour
    # for both the see-through tint and the glow.
    shader.CreateInput("ior_constant", Sdf.ValueTypeNames.Float).Set(1.1)
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("transmittance_color", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("transmittance_measurement_distance", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("enable_thin_walled", Sdf.ValueTypeNames.Bool).Set(False)
    shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
    shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(5169.5)
    shader_output = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader_output)
    material.CreateVolumeOutput("mdl").ConnectToSource(shader_output)
    UsdShade.MaterialBindingAPI.Apply(ghost_prim).Bind(
        material, UsdShade.Tokens.strongerThanDescendants
    )
