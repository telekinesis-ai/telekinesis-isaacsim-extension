"""Build the pick-and-place scene of the Isaac Sim quickstart without the GUI.

Writes ``pick_and_place_scene.usda`` next to this script: the sample scene (Vention
table and three cubes) with a UR10e on the table's robot mount, a Robotiq UR coupler
and a Robotiq Hand-E on its flange. The result is the scene the quickstart builds with
Add Robot, Add Mesh and Add Tool, so the quickstart's pick-and-place example runs on it.
It also holds the poses taught in the quickstart scene with Add Pose > From Tool (three
pick poses on the cubes and a place start pose), so the teach pick-and-place example runs
on it too.

Every asset comes from the Telekinesis asset library and is downloaded once into
``~/.cache/telekinesis/usd``. The output scene links to those cached files, so it opens
on the machine that built it.

The robot and the gripper carry the bridge's tags with an empty id, and the robot its Synapse
class name and base frame: the bridge registers them under fresh ids when the scene is opened,
and drives the Hand-E's left finger joint only. The tags, the assembly and the poses follow the
extension's Add Robot, Add Mesh, Add Tool and Add Pose, and must be kept in step with them.

Example:
    In the SDK environment::

        python examples/scenes/pick_and_place_scene.py

    Then open ``pick_and_place_scene.usda`` in Isaac Sim with the bridge enabled and run
    ``examples/ur10e_hande_isaacsim_pick_place.py`` or
    ``examples/ur10e_hande_teach_pick_place.py``.
"""

import pathlib
import shutil
import urllib.request
import zipfile

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

ASSET_ROOT = "https://assets.telekinesis.ai"
CACHE_DIRECTORY = pathlib.Path.home() / ".cache" / "telekinesis"
OUTPUT_PATH = pathlib.Path(__file__).resolve().parent / "pick_and_place_scene.usda"

ASSEMBLY_PATH = "/World/vention_table/robot_mount/UR10e_assembly"
ROBOT_PATH = f"{ASSEMBLY_PATH}/UR10e"
COUPLER_PATH = f"{ASSEMBLY_PATH}/Robotiq_UR_Coupler"
GRIPPER_PATH = f"{ASSEMBLY_PATH}/Hand_E"
MOUNT_LINK = "wrist_3_link"
# The robot mount's origin is 7 mm above the base plate's top face; this lowers the robot onto it.
ROBOT_OFFSET_Z = -0.007  # m, in the robot mount's frame
# Hand-E origin above the mount link, as in the quickstart scene assembled with Add Tool.
GRIPPER_OFFSET_Z = 0.0098  # m

# The extension's names for the joint that holds a mounted prim and for its tags.
MOUNT_JOINT_NAME = "AssemblerFixedJoint"
ASSEMBLY_ROBOT_RELATIONSHIP = "telekinesis:assemblyRobot"
ARTICULATION_ID_ATTRIBUTE = "telekinesis:articulation:id"
DRIVEN_JOINTS_ATTRIBUTE = "telekinesis:articulation:driven_joints"
MOUNT_LINK_ATTRIBUTE = "telekinesis:mountLink"
# The Synapse class the Telekinesis Isaac Sim Server loads the robot as, for its poses.
ROBOT_CLASS_NAME_ATTRIBUTE = "telekinesis:robot:class_name"
ROBOT_CLASS_NAME = "universalrobotsur10e"
# The robot frame that class measures poses from, and the one it places them at. The base
# frame is stored on the robot, so the bridge lists the poses without the server.
ROBOT_BASE_FRAME_ATTRIBUTE = "telekinesis:robot:base_frame"
ROBOT_BASE_FRAME = "base_link"
ROBOT_DEFAULT_TCP = "tool0"

# The poses taught in the quickstart scene. "pose" is the robot's default TCP in its base
# frame, [x, y, z, rx, ry, rz] in metres and extrinsic XYZ Euler degrees, and
# "joint_positions" the joint positions that reach it, in radians, in POSE_JOINT_NAMES order.
POSES_PATH = f"{ASSEMBLY_PATH}/poses"
POSE_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
TAUGHT_POSES = {
    "pick_pose": {
        "pose": [0.088433, 0.498376, 0.142902, 180.0, 0.0, -0.012],
        "joint_positions": [1.746539, -1.771143, -2.52046, -0.420786, 1.570796, 0.175952],
    },
    "pick_pose_01": {
        "pose": [-0.016451, 0.498376, 0.142902, 180.0, 0.0, -0.012],
        "joint_positions": [1.960722, -1.761941, -2.534385, -0.416064, 1.570796, 0.390136],
    },
    "pick_pose_02": {
        "pose": [-0.134993, 0.498376, 0.142902, 180.0, 0.0, -0.012],
        "joint_positions": [2.179209, -1.783456, -2.50157, -0.427363, 1.570796, 0.608623],
    },
    "start_place_pose": {
        "pose": [-0.683706, -0.060906, 0.142902, 180.0, 0.0, 90.0],
        "joint_positions": [3.486991, -1.974982, -2.178495, -0.558912, 1.570796, 0.345399],
    },
}
# The extension's pose tags, and the see-through blue of a reachable pose's ghost.
POSE_ROBOT_RELATIONSHIP = "telekinesis:pose:robot"
POSE_IN_BASE_FRAME_ATTRIBUTE = "telekinesis:pose:pose_in_base_frame"
POSE_JOINT_NAMES_ATTRIBUTE = "telekinesis:pose:joint_names"
POSE_JOINT_POSITIONS_ATTRIBUTE = "telekinesis:pose:joint_positions"
POSE_REACHABLE_ATTRIBUTE = "telekinesis:pose:reachable"
GHOST_COLOR = Gf.ConvertDisplayToLinear(Gf.Vec3f(0x41, 0xD1, 0xFF) / 255.0)


def download_asset(key: str) -> pathlib.Path:
    """Download an asset of the Telekinesis asset library into the cache, once.

    ``key`` is the asset's path in the library, such as ``usd/couplers/x.usda``. A ZIP
    bundle is extracted into a folder named after it, and that folder is returned; any
    other file is stored as it is, and its path is returned. Later calls return the
    cached copy without downloading. A download is written under a temporary name and
    renamed at the end, so an interrupted run never leaves a partial copy behind.

    Raises:
        urllib.error.URLError: If the asset cannot be downloaded.
    """
    target = CACHE_DIRECTORY / key
    if key.endswith(".zip"):
        target = target.with_suffix("")
    if target.exists():
        return target

    print(f"Downloading {ASSET_ROOT}/{key}")
    target.parent.mkdir(parents=True, exist_ok=True)
    download_path = target.with_name(f".{target.name}.download")
    urllib.request.urlretrieve(f"{ASSET_ROOT}/{key}", download_path)
    if key.endswith(".zip"):
        staging_directory = target.with_name(f".{target.name}.staging")
        shutil.rmtree(staging_directory, ignore_errors=True)
        with zipfile.ZipFile(download_path) as archive:
            archive.extractall(staging_directory)
        download_path.unlink()
        download_path = staging_directory
    download_path.rename(target)
    return target


def place_prim_at(prim: Usd.Prim, world_transform: Gf.Matrix4d) -> None:
    """Author ``prim``'s transform so that it lands at ``world_transform`` in the stage.

    The transform is written relative to the prim's parent, as a translation and an
    orientation, on a prim that authors no transform yet.
    """
    parent_world = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    local = Gf.Transform(world_transform * parent_world.GetInverse())
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(local.GetTranslation())
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(local.GetRotation().GetQuat())


def create_mount_joint(
    stage: Usd.Stage, mount_link_path: str, body_path: str, offset: Gf.Vec3d
) -> None:
    """Fix the rigid body at ``body_path`` to the arm's mount link, as the extension does.

    The joint is authored beneath the body, holds the body's origin at ``offset`` in the
    mount link's frame with the same orientation, and stays in the arm's articulation, so
    the body becomes one more link of the arm without adding a degree of freedom.
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{body_path}/{MOUNT_JOINT_NAME}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(mount_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(offset))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0))


def find_prim_named(root: Usd.Prim, name: str) -> Usd.Prim:
    """Return the first prim named ``name`` at or under ``root``, as the bridge finds a
    robot's frames."""
    return next(prim for prim in Usd.PrimRange(root) if prim.GetName() == name)


def merge_meshes_in_frame(
    root: Usd.Prim, frame_world: Gf.Matrix4d
) -> tuple[Vt.Vec3fArray, list[int], list[int]]:
    """Merge every visible mesh under ``root`` into one mesh in the frame ``frame_world``.

    Meshes inside instances are included; invisible meshes and guide or proxy meshes, such
    as colliders, are skipped, as the bridge does when it draws a pose's ghost. Returns the
    merged points, face vertex counts and face vertex indices.
    """
    xform_cache = UsdGeom.XformCache()
    frame_from_world = frame_world.GetInverse()
    points, face_vertex_counts, face_vertex_indices = [], [], []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        imageable = UsdGeom.Imageable(prim)
        if (
            not prim.IsA(UsdGeom.Mesh)
            or imageable.ComputeVisibility() == UsdGeom.Tokens.invisible
            or imageable.ComputePurpose() not in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render)
        ):
            continue
        mesh = UsdGeom.Mesh(prim)
        mesh_points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not mesh_points or not counts or not indices:
            continue
        # Gf matrices act on row vectors: mesh frame -> world -> the target frame.
        mesh_to_frame = xform_cache.GetLocalToWorldTransform(prim) * frame_from_world
        face_vertex_indices.extend(index + len(points) for index in indices)
        face_vertex_counts.extend(counts)
        points.extend(Gf.Vec3f(mesh_to_frame.Transform(Gf.Vec3d(point))) for point in mesh_points)
    return Vt.Vec3fArray(points), face_vertex_counts, face_vertex_indices


def define_ghost_material(stage: Usd.Stage, material_path: str) -> UsdShade.Material:
    """Define the see-through, glowing blue material the bridge gives a reachable pose's
    ghost: Kit's ``SimPBR_Translucent`` with the bridge's ghost settings."""
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.SetSourceAsset(Sdf.AssetPath("SimPBR_Translucent.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("SimPBR_Translucent", "mdl")
    shader.CreateInput("ior_constant", Sdf.ValueTypeNames.Float).Set(1.1)
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("transmittance_color", Sdf.ValueTypeNames.Color3f).Set(GHOST_COLOR)
    shader.CreateInput("transmittance_measurement_distance", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("enable_thin_walled", Sdf.ValueTypeNames.Bool).Set(False)
    shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
    shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(GHOST_COLOR)
    shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(5169.5)
    shader_output = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader_output)
    material.CreateVolumeOutput("mdl").ConnectToSource(shader_output)
    return material


def create_taught_poses(stage: Usd.Stage) -> None:
    """Create the poses of ``TAUGHT_POSES`` in the ``poses`` Xform beside the robot, as Add
    Pose > From Tool creates them.

    Each pose is a see-through blue mesh of the Hand-E, placed at the robot's default TCP
    for that pose, and stores the bridge's pose tags, so the bridge treats it as a pose
    taught in Isaac Sim. Call it with the robot at its rest pose and the Hand-E mounted.
    """
    robot = stage.GetPrimAtPath(ROBOT_PATH)
    base_world = UsdGeom.Xformable(
        find_prim_named(robot, ROBOT_BASE_FRAME)
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    default_tcp_world = UsdGeom.Xformable(
        find_prim_named(robot, ROBOT_DEFAULT_TCP)
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # The Hand-E is fixed to the flange, so its meshes in the default TCP's frame are the
    # same for every pose.
    points, face_vertex_counts, face_vertex_indices = merge_meshes_in_frame(
        stage.GetPrimAtPath(GRIPPER_PATH), default_tcp_world
    )

    UsdGeom.Xform.Define(stage, POSES_PATH)
    for pose_name, taught_pose in TAUGHT_POSES.items():
        pose_path = f"{POSES_PATH}/{pose_name}"
        pose_mesh = UsdGeom.Mesh.Define(stage, pose_path)
        pose_mesh.CreatePointsAttr(points)
        pose_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        pose_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)

        x, y, z, rotation_x, rotation_y, rotation_z = taught_pose["pose"]
        # Rotations about the fixed x, then y, then z axes: extrinsic XYZ.
        pose_in_base = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d.XAxis(), rotation_x)
            * Gf.Rotation(Gf.Vec3d.YAxis(), rotation_y)
            * Gf.Rotation(Gf.Vec3d.ZAxis(), rotation_z)
        )
        pose_in_base.SetTranslateOnly(Gf.Vec3d(x, y, z))
        place_prim_at(pose_mesh.GetPrim(), pose_in_base * base_world)

        pose_prim = pose_mesh.GetPrim()
        pose_prim.CreateRelationship(POSE_ROBOT_RELATIONSHIP).SetTargets([Sdf.Path(ROBOT_PATH)])
        pose_prim.CreateAttribute(
            POSE_IN_BASE_FRAME_ATTRIBUTE, Sdf.ValueTypeNames.DoubleArray
        ).Set(taught_pose["pose"])
        pose_prim.CreateAttribute(POSE_JOINT_NAMES_ATTRIBUTE, Sdf.ValueTypeNames.StringArray).Set(
            POSE_JOINT_NAMES
        )
        pose_prim.CreateAttribute(
            POSE_JOINT_POSITIONS_ATTRIBUTE, Sdf.ValueTypeNames.DoubleArray
        ).Set(taught_pose["joint_positions"])
        pose_prim.CreateAttribute(POSE_REACHABLE_ATTRIBUTE, Sdf.ValueTypeNames.Bool).Set(True)
        UsdShade.MaterialBindingAPI.Apply(pose_prim).Bind(
            define_ghost_material(stage, f"{pose_path}/Looks/GhostMaterial"),
            UsdShade.Tokens.strongerThanDescendants,
        )


def main() -> None:
    """Download the assets, assemble the robot on the sample scene, add the taught poses and
    save the result."""
    scene_file = (
        download_asset("usd/environments/pick_and_place/pick_and_place_scene.zip")
        / "pick_and_place_scene.usd"
    )
    robot_file = download_asset("usd/robots/manipulators/universal_robots/ur10e.zip") / "ur10e.usda"
    gripper_file = (
        download_asset("usd/tools/parallel_grippers/robotiq/robotiqhande.zip")
        / "Robotiq_Hand_E.usda"
    )
    coupler_file = download_asset("usd/couplers/Robotiq_UR_Coupler.usda")

    # The sample scene is layered beneath the output rather than copied into it, so its
    # links to the table, which are relative to its own folder, keep resolving.
    stage = Usd.Stage.CreateNew(str(OUTPUT_PATH))
    stage.GetRootLayer().subLayerPaths.append(scene_file.as_posix())
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    # Timing is read from the root layer only, so the sample scene's would be lost.
    scene_layer = Sdf.Layer.FindOrOpen(scene_file.as_posix())
    stage.SetTimeCodesPerSecond(scene_layer.timeCodesPerSecond)
    stage.SetStartTimeCode(scene_layer.startTimeCode)
    stage.SetEndTimeCode(scene_layer.endTimeCode)

    # The assembly Xform makes everything mounted in it part of the robot, as Add Robot
    # builds it.
    assembly_xform = UsdGeom.Xform.Define(stage, ASSEMBLY_PATH)
    assembly_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, ROBOT_OFFSET_Z))
    assembly = assembly_xform.GetPrim()
    assembly.CreateRelationship(ASSEMBLY_ROBOT_RELATIONSHIP).SetTargets([Sdf.Path(ROBOT_PATH)])

    robot = stage.DefinePrim(ROBOT_PATH)
    robot.GetReferences().AddReference(robot_file.as_posix(), "/ur10e_robot")
    robot.CreateAttribute(ARTICULATION_ID_ATTRIBUTE, Sdf.ValueTypeNames.String).Set("")
    robot.CreateAttribute(MOUNT_LINK_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(MOUNT_LINK)
    robot.CreateAttribute(ROBOT_CLASS_NAME_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(
        ROBOT_CLASS_NAME
    )
    robot.CreateAttribute(ROBOT_BASE_FRAME_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(
        ROBOT_BASE_FRAME
    )

    mount_link_path = f"{ROBOT_PATH}/{MOUNT_LINK}"
    mount_link_world = UsdGeom.Xformable(
        stage.GetPrimAtPath(mount_link_path)
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    # The coupler is mounted as Add Mesh mounts a prim: a dynamic rigid body colliding as
    # convex hulls, fixed to the mount link, and not colliding with the robot it sits on.
    coupler = stage.DefinePrim(COUPLER_PATH)
    coupler.GetReferences().AddReference(coupler_file.as_posix(), "/Robotiq_UR_coupler")
    place_prim_at(coupler, mount_link_world)
    UsdPhysics.RigidBodyAPI.Apply(coupler).CreateKinematicEnabledAttr(False)
    for descendant in Usd.PrimRange(coupler):
        if descendant.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(descendant)
            # PhysX cannot simulate a triangle mesh on a moving body.
            UsdPhysics.MeshCollisionAPI.Apply(descendant).CreateApproximationAttr(
                UsdPhysics.Tokens.convexHull
            )
    UsdPhysics.FilteredPairsAPI.Apply(coupler).CreateFilteredPairsRel().SetTargets(
        [Sdf.Path(ROBOT_PATH)]
    )
    create_mount_joint(stage, mount_link_path, COUPLER_PATH, Gf.Vec3d(0.0))

    # The Hand-E is merged into the robot's articulation as Add Tool merges it: its own
    # articulation root is removed, its weld to the world is deactivated, and its base
    # link is fixed to the mount link on top of the coupler.
    gripper = stage.DefinePrim(GRIPPER_PATH)
    gripper.GetReferences().AddReference(gripper_file.as_posix(), "/Robotiq_Hand_E")
    gripper_offset = Gf.Vec3d(0.0, 0.0, GRIPPER_OFFSET_Z)
    place_prim_at(gripper, Gf.Matrix4d().SetTranslate(gripper_offset) * mount_link_world)
    stage.GetPrimAtPath(f"{GRIPPER_PATH}/Robotiq_Hand_E").RemoveAPI(UsdPhysics.ArticulationRootAPI)
    stage.GetPrimAtPath(f"{GRIPPER_PATH}/Robotiq_Hand_E/root_joint").SetActive(False)
    create_mount_joint(
        stage, mount_link_path, f"{GRIPPER_PATH}/Robotiq_Hand_E/base_link", gripper_offset
    )
    gripper.CreateAttribute(ARTICULATION_ID_ATTRIBUTE, Sdf.ValueTypeNames.String).Set("")
    gripper.CreateAttribute(DRIVEN_JOINTS_ATTRIBUTE, Sdf.ValueTypeNames.TokenArray).Set(
        ["left_finger_joint"]
    )

    create_taught_poses(stage)

    stage.GetRootLayer().Save()
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
