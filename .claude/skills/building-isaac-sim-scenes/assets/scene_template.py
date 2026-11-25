"""Build <describe the scene in one sentence> without the GUI.

Writes ``<script name>.usda`` next to this script: <what is in it, where, and why the
numbers are what they are>. Every asset comes from the Telekinesis asset library and is
downloaded once into ``~/.cache/telekinesis``; the scene links to those cached files, so
it opens on the machine that built it.

Robots and grippers carry the bridge's tags with an empty id: the bridge registers them
under fresh ids when the scene is opened in Isaac Sim with the bridge enabled. The
assembly follows the extension's Add Robot, Add Mesh and Add Tool.

Example:
    In the SDK environment (needs ``usd-core``)::

        python <path to this script>
"""

import pathlib
import shutil
import urllib.request
import zipfile

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

ASSET_ROOT = "https://assets.telekinesis.ai"
CACHE_DIRECTORY = pathlib.Path.home() / ".cache" / "telekinesis"
OUTPUT_PATH = pathlib.Path(__file__).resolve().with_suffix(".usda")

# --- Scene layout. World frame: Z up, metres; rotations are extrinsic XYZ Euler degrees.
# Every number here comes from the request or from inspect_asset.py, never from a guess.

# An environment scene to build on (usd/environments/...), or None to start from an
# empty stage with a floor, lights and a physics scene.
ENVIRONMENT_KEY = None
ENVIRONMENT_FILE_NAME = None  # e.g. "pick_and_place_scene.usd", the bundle's main file

TABLE_KEY = "usd/props/mounts/table_1400x800x900mm.usd"
TABLE_ROOT = "/TableAsset"  # the asset's default prim (inspect_asset.py)
TABLE_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # x, y, z, rx, ry, rz of the table's origin
TABLE_TOP_HEIGHT = 0.9  # m, top of the table (inspect_asset.py bounds)

ASSEMBLY_PATH = "/World/UR10e_assembly"
ROBOT_PATH = f"{ASSEMBLY_PATH}/UR10e"
ROBOT_KEY = "usd/robots/manipulators/universal_robots/ur10e.zip"
ROBOT_FILE_NAME = "ur10e.usda"
ROBOT_ROOT = "/ur10e_robot"
ROBOT_CLASS_NAME = "universalrobotsur10e"  # find_assets.py
MOUNT_LINK = "wrist_3_link"  # find_assets.py
DEFAULT_TCP = "tool0"  # find_assets.py; Add TCP places TCPs under the prim of this name
# Robot base on the table, 0.4 m in from its -X edge. The UR's arm reaches along its own
# -X at zero pan, so a half turn points it at the parts on +X.
ROBOT_POSE = (-0.3, 0.0, TABLE_TOP_HEIGHT, 0.0, 0.0, 180.0)
# Joint angles in degrees, in the order the asset's joints chain from the base, or None to
# leave the asset's zero pose. Play starts here, so the arm does not sweep through the scene.
HOME_JOINTS_DEG = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -90.0,
    "elbow_joint": 90.0,
    "wrist_1_joint": -90.0,
    "wrist_2_joint": -90.0,
    "wrist_3_joint": 0.0,
}

COUPLER_PATH = f"{ASSEMBLY_PATH}/Robotiq_UR_Coupler"
COUPLER_KEY = "usd/couplers/Robotiq_UR_Coupler.usda"
COUPLER_ROOT = "/Robotiq_UR_coupler"
COUPLER_TOP = 0.014  # m above the flange: the coupler's top face (bounds max z)

GRIPPER_PATH = f"{ASSEMBLY_PATH}/Hand_E"
GRIPPER_KEY = "usd/tools/parallel_grippers/robotiq/robotiqhande.zip"
GRIPPER_FILE_NAME = "Robotiq_Hand_E.usda"
GRIPPER_ROOT = "/Robotiq_Hand_E"
GRIPPER_BASE_LINK = "Robotiq_Hand_E/base_link"  # below GRIPPER_PATH
GRIPPER_DRIVER_JOINT = "left_finger_joint"  # the non-mimic joint with a drive

PART_SIZE = (0.05, 0.05, 0.05)  # m
PART_MASS = 0.2  # kg
PART_FRICTION = 0.8  # static and dynamic
PART_POSITIONS = [(0.25, -0.15), (0.25, 0.0), (0.25, 0.15)]  # x, y on the table top

# Bridge camera looking down at the parts from 1 m above the table.
CAMERA_PATH = "/World/Cameras/top"
CAMERA_EYE = (0.25, 0.0, TABLE_TOP_HEIGHT + 1.0)
CAMERA_TARGET = (0.25, 0.0, TABLE_TOP_HEIGHT)
CAMERA_FOCAL_LENGTH = 18.0  # mm
CAMERA_RESOLUTION = (1280, 720)

# --- The extension's names. They are an on-disk format: do not change them.
MOUNT_JOINT_NAME = "AssemblerFixedJoint"
ASSEMBLY_ROBOT_RELATIONSHIP = "telekinesis:assemblyRobot"
ARTICULATION_ID_ATTRIBUTE = "telekinesis:articulation:id"
DRIVEN_JOINTS_ATTRIBUTE = "telekinesis:articulation:driven_joints"
MOUNT_LINK_ATTRIBUTE = "telekinesis:mountLink"
ROBOT_CLASS_NAME_ATTRIBUTE = "telekinesis:robot:class_name"
ROBOT_DEFAULT_TCP_ATTRIBUTE = "telekinesis:robot:default_tcp"
CAMERA_ID_ATTRIBUTE = "telekinesis:camera:id"
CAMERA_RESOLUTION_ATTRIBUTE = "telekinesis:camera:resolution"
# The cinematic camera tool's 16:9 sensor, in mm.
CAMERA_SENSOR = (20.955, 11.787)


def download_asset(key: str) -> pathlib.Path:
    """Download an asset of the Telekinesis asset library into the cache, once.

    ``key`` is the asset's path in the library, such as ``usd/couplers/x.usda``. A ZIP
    bundle is extracted into a folder named after it, and that folder is returned; any
    other file is stored as it is, and its path is returned. Later calls return the
    cached copy. A download is written under a temporary name and renamed at the end, so
    an interrupted run never leaves a partial copy behind.

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


def pose_matrix(pose: tuple[float, float, float, float, float, float]) -> Gf.Matrix4d:
    """Matrix of an (x, y, z, rx, ry, rz) pose: metres and extrinsic XYZ Euler degrees."""
    x, y, z, rx, ry, rz = pose
    # Extrinsic X, then Y, then Z: Gf composes row-vector matrices left to right.
    rotation = (
        Gf.Rotation(Gf.Vec3d(1, 0, 0), rx)
        * Gf.Rotation(Gf.Vec3d(0, 1, 0), ry)
        * Gf.Rotation(Gf.Vec3d(0, 0, 1), rz)
    )
    return Gf.Matrix4d().SetRotate(rotation) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, z))


def world_transform(prim: Usd.Prim) -> Gf.Matrix4d:
    """The prim's local-to-world matrix as currently authored."""
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def place_prim_at(prim: Usd.Prim, world: Gf.Matrix4d) -> None:
    """Author ``prim``'s transform so that it lands at ``world`` in the stage.

    Replaces whatever transform the prim had, the referenced asset root's own ops
    included, with a translation and an orientation relative to the prim's parent. Add a
    scale op afterwards if the asset needs one, such as 0.01 for an asset in centimetres.
    """
    local = Gf.Transform(world * world_transform(prim.GetParent()).GetInverse())
    xformable = UsdGeom.Xformable(prim)
    # The asset's op attributes keep their authored precision: re-adding them with another raises.
    precisions = {op.GetOpType(): op.GetPrecision() for op in xformable.GetOrderedXformOps()}
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp(precisions.get(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble))
    translate_op.Set(Gf.Vec3f(local.GetTranslation()) if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else local.GetTranslation())
    orient_op = xformable.AddOrientOp(precisions.get(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble))
    orient_op.Set(Gf.Quatf(local.GetRotation().GetQuat()) if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else local.GetRotation().GetQuat())


def reference_asset(stage: Usd.Stage, path: str, asset_file: pathlib.Path, asset_root: str) -> Usd.Prim:
    """Define ``path`` as a reference to ``asset_root`` in ``asset_file`` and return it."""
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(asset_file.as_posix(), asset_root)
    return prim


def set_home_pose(stage: Usd.Stage, robot_path: str, angles_deg: dict[str, float]) -> None:
    """Pose the arm's links at ``angles_deg`` and make Play start there.

    ``angles_deg`` maps revolute joint names to degrees and must list them base first:
    each joint's child link is moved to where PhysX would put it at that angle relative
    to its parent link as already posed, so the authored stage and the simulation agree. The
    joint's drive target and initial joint state are set to the same angle, so the drive
    holds the pose instead of pulling the arm back to zero.

    Raises:
        KeyError: If a named joint is not under ``robot_path``.
    """
    joints = {
        prim.GetName(): prim
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_path))
        if prim.IsA(UsdPhysics.RevoluteJoint)
    }
    for name, angle in angles_deg.items():
        joint = UsdPhysics.RevoluteJoint(joints[name])
        frame0 = Gf.Matrix4d().SetRotate(Gf.Quatd(joint.GetLocalRot0Attr().Get())) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
        frame1 = Gf.Matrix4d().SetRotate(Gf.Quatd(joint.GetLocalRot1Attr().Get())) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(joint.GetLocalPos1Attr().Get()))
        axis = {"X": Gf.Vec3d.XAxis(), "Y": Gf.Vec3d.YAxis(), "Z": Gf.Vec3d.ZAxis()}[joint.GetAxisAttr().Get()]
        parent_link = stage.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
        child_link = stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
        child_world = frame1.GetInverse() * Gf.Matrix4d().SetRotate(Gf.Rotation(axis, angle)) * frame0 * world_transform(parent_link)
        place_prim_at(child_link, child_world)
        joint_prim = joint.GetPrim()
        UsdPhysics.DriveAPI.Apply(joint_prim, "angular").CreateTargetPositionAttr().Set(angle)
        joint_prim.AddAppliedSchema("PhysicsJointStateAPI:angular")
        joint_prim.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(angle)
        joint_prim.CreateAttribute("state:angular:physics:velocity", Sdf.ValueTypeNames.Float).Set(0.0)


def mount_on_link(stage: Usd.Stage, root: Usd.Prim, body_path: str, mount_link_path: str, offset: Gf.Matrix4d) -> None:
    """Move ``root`` so its rigid body at ``body_path`` sits at ``offset`` in the mount link's frame, and fix it there.

    The fixed joint is authored beneath the body and stays in the arm's articulation, so
    the body becomes one more link of the arm without adding a degree of freedom. Its two
    frames coincide at the authored pose, so nothing moves on Play.
    """
    body_in_root = world_transform(stage.GetPrimAtPath(body_path)) * world_transform(root).GetInverse()
    place_prim_at(root, body_in_root.GetInverse() * offset * world_transform(stage.GetPrimAtPath(mount_link_path)))
    mount = Gf.Transform(offset)
    joint = UsdPhysics.FixedJoint.Define(stage, f"{body_path}/{MOUNT_JOINT_NAME}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(mount_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(mount.GetTranslation()))
    joint.CreateLocalRot0Attr(Gf.Quatf(mount.GetRotation().GetQuat()))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0))


def flange_in_mount_link(stage: Usd.Stage, mount_link_path: str) -> Gf.Matrix4d:
    """The flange frame (Z out of the flange face) in the mount link's frame.

    That is ``tool0`` under the mount link when the arm has one: on ABB, KUKA and Fanuc
    arms the mount link's own X runs along the flange axis. Otherwise the mount link
    itself, as the extension's Add Tool assumes.
    """
    mount_link = stage.GetPrimAtPath(mount_link_path)
    tool0 = next((prim for prim in Usd.PrimRange(mount_link) if prim.GetName() == "tool0"), mount_link)
    return world_transform(tool0) * world_transform(mount_link).GetInverse()


def define_physics_material(stage: Usd.Stage, path: str, friction: float) -> UsdShade.Material:
    """A physics material with equal static and dynamic ``friction`` and no bounce."""
    material = UsdShade.Material.Define(stage, path)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(friction)
    physics.CreateDynamicFrictionAttr(friction)
    physics.CreateRestitutionAttr(0.0)
    return material


def add_box(
    stage: Usd.Stage, path: str, size: tuple[float, float, float], bottom_centre: tuple[float, float, float],
    color: tuple[float, float, float], material: UsdShade.Material | None = None, mass: float | None = None,
    yaw_deg: float = 0.0,
) -> Usd.Prim:
    """A box collider standing on ``bottom_centre``: a rigid body of ``mass`` kg, or static without one."""
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    box.AddTranslateOp().Set(Gf.Vec3d(bottom_centre[0], bottom_centre[1], bottom_centre[2] + size[2] / 2.0))
    box.AddRotateZOp().Set(yaw_deg)
    box.AddScaleOp().Set(Gf.Vec3f(*size))
    prim = box.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return prim


def look_at(eye, target) -> Gf.Matrix4d:
    """Camera matrix at ``eye`` looking at ``target``, world Z up in the image.

    A USD camera looks along its -Z with +Y up in the image.
    """
    eye, target = Gf.Vec3d(*eye), Gf.Vec3d(*target)
    back = (eye - target).GetNormalized()
    up = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(back * up) > 0.999:  # straight down: keep the image's up along world +Y
        up = Gf.Vec3d(0.0, 1.0, 0.0)
    right = (up ^ back).GetNormalized()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRow(0, Gf.Vec4d(*right, 0.0))
    matrix.SetRow(1, Gf.Vec4d(*(back ^ right), 0.0))
    matrix.SetRow(2, Gf.Vec4d(*back, 0.0))
    matrix.SetRow(3, Gf.Vec4d(*eye, 1.0))
    return matrix


def add_camera(stage: Usd.Stage, path: str, transform: Gf.Matrix4d, focal_length_mm: float, resolution: tuple[int, int] | None) -> None:
    """A 16:9 camera; with a ``resolution`` it is also registered as a bridge camera when the scene opens."""
    camera = UsdGeom.Camera.Define(stage, path)
    camera.AddTransformOp().Set(transform)
    camera.CreateFocalLengthAttr(focal_length_mm)
    camera.CreateHorizontalApertureAttr(CAMERA_SENSOR[0])
    camera.CreateVerticalApertureAttr(CAMERA_SENSOR[1])
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.02, 100.0))
    if resolution is not None:
        prim = camera.GetPrim()
        prim.CreateAttribute(CAMERA_ID_ATTRIBUTE, Sdf.ValueTypeNames.String).Set("")
        prim.CreateAttribute(CAMERA_RESOLUTION_ATTRIBUTE, Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(*resolution))


def create_stage() -> Usd.Stage:
    """A new stage at OUTPUT_PATH: over the environment scene, or empty with floor, lights and physics."""
    if OUTPUT_PATH.exists():
        OUTPUT_PATH.unlink()
    stage = Usd.Stage.CreateNew(str(OUTPUT_PATH))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    if ENVIRONMENT_KEY is not None:
        scene_file = download_asset(ENVIRONMENT_KEY) / ENVIRONMENT_FILE_NAME
        # Layered beneath rather than copied, so its links relative to its own folder resolve.
        stage.GetRootLayer().subLayerPaths.append(scene_file.as_posix())
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        # Timing is read from the root layer only, so the environment's would be lost.
        scene_layer = Sdf.Layer.FindOrOpen(scene_file.as_posix())
        stage.SetTimeCodesPerSecond(scene_layer.timeCodesPerSecond)
        stage.SetStartTimeCode(scene_layer.startTimeCode)
        stage.SetEndTimeCode(scene_layer.endTimeCode)
        return stage

    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    add_box(stage, "/World/Floor", (30.0, 30.0, 0.02), (0.0, 0.0, -0.02), (0.45, 0.45, 0.45))
    UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr(1000.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))
    return stage


def main() -> None:
    """Download the assets, assemble the scene and save it."""
    stage = create_stage()

    table = reference_asset(stage, "/World/table", download_asset(TABLE_KEY), TABLE_ROOT)
    place_prim_at(table, pose_matrix(TABLE_POSE))

    # The assembly Xform makes everything mounted in it part of the robot, as Add Robot
    # builds it. The robot keeps its own transform, so the assembly carries the pose.
    assembly = UsdGeom.Xform.Define(stage, ASSEMBLY_PATH).GetPrim()
    assembly.CreateRelationship(ASSEMBLY_ROBOT_RELATIONSHIP).SetTargets([Sdf.Path(ROBOT_PATH)])
    place_prim_at(assembly, pose_matrix(ROBOT_POSE))
    robot = reference_asset(stage, ROBOT_PATH, download_asset(ROBOT_KEY) / ROBOT_FILE_NAME, ROBOT_ROOT)
    robot.CreateAttribute(ARTICULATION_ID_ATTRIBUTE, Sdf.ValueTypeNames.String).Set("")
    robot.CreateAttribute(MOUNT_LINK_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(MOUNT_LINK)
    robot.CreateAttribute(ROBOT_CLASS_NAME_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(ROBOT_CLASS_NAME)
    robot.CreateAttribute(ROBOT_DEFAULT_TCP_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(DEFAULT_TCP)
    if HOME_JOINTS_DEG:
        set_home_pose(stage, ROBOT_PATH, HOME_JOINTS_DEG)

    # Tools are placed from the mount link's pose, so after the home pose.
    mount_link_path = f"{ROBOT_PATH}/{MOUNT_LINK}"
    flange = flange_in_mount_link(stage, mount_link_path)

    # The coupler is mounted as Add Mesh mounts a prim: a dynamic rigid body colliding as
    # convex hulls, fixed to the mount link, and not colliding with the robot it sits on.
    coupler = reference_asset(stage, COUPLER_PATH, download_asset(COUPLER_KEY), COUPLER_ROOT)
    UsdPhysics.RigidBodyAPI.Apply(coupler).CreateKinematicEnabledAttr(False)
    for descendant in Usd.PrimRange(coupler):
        if descendant.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(descendant)
            # PhysX cannot simulate a triangle mesh on a moving body.
            UsdPhysics.MeshCollisionAPI.Apply(descendant).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
    UsdPhysics.FilteredPairsAPI.Apply(coupler).CreateFilteredPairsRel().SetTargets([Sdf.Path(ROBOT_PATH)])
    mount_on_link(stage, coupler, COUPLER_PATH, mount_link_path, flange)

    # The gripper is merged into the robot's articulation as Add Tool merges it: its own
    # articulation root is removed, its welds to the world are deactivated, and its base
    # link is fixed to the mount link on top of the coupler.
    gripper = reference_asset(stage, GRIPPER_PATH, download_asset(GRIPPER_KEY) / GRIPPER_FILE_NAME, GRIPPER_ROOT)
    rigid_bodies = {prim.GetPath() for prim in Usd.PrimRange(gripper) if prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    for prim in list(Usd.PrimRange(gripper)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        joint = UsdPhysics.Joint(prim)
        if joint and sum(body in rigid_bodies for body in [*joint.GetBody0Rel().GetTargets(), *joint.GetBody1Rel().GetTargets()]) < 2:
            prim.SetActive(False)
    base_link_path = f"{GRIPPER_PATH}/{GRIPPER_BASE_LINK}"
    mount_on_link(stage, gripper, base_link_path, mount_link_path, Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, COUPLER_TOP)) * flange)
    gripper.CreateAttribute(ARTICULATION_ID_ATTRIBUTE, Sdf.ValueTypeNames.String).Set("")
    gripper.CreateAttribute(DRIVEN_JOINTS_ATTRIBUTE, Sdf.ValueTypeNames.TokenArray).Set([GRIPPER_DRIVER_JOINT])
    # Add Tool's gripping friction: colliders without a physics material grip at 1.0, the
    # higher friction of the two surfaces winning, so parts do not slip out of the fingers.
    grip_material = define_physics_material(stage, f"{GRIPPER_PATH}/PhysicsMaterial", 1.0)
    grip_material.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
    grip_material.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("max")
    for prim in Usd.PrimRange(gripper):
        bound, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
        if prim.HasAPI(UsdPhysics.CollisionAPI) and not (bound and bound.GetPrim().HasAPI(UsdPhysics.MaterialAPI)):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(grip_material, UsdShade.Tokens.weakerThanDescendants, "physics")

    part_material = define_physics_material(stage, "/World/PhysicsMaterials/part", PART_FRICTION)
    for index, (x, y) in enumerate(PART_POSITIONS):
        add_box(stage, f"/World/part_{index}", PART_SIZE, (x, y, TABLE_TOP_HEIGHT), (0.8, 0.3, 0.1), part_material, PART_MASS)

    UsdGeom.Xform.Define(stage, "/World/Cameras")
    add_camera(stage, CAMERA_PATH, look_at(CAMERA_EYE, CAMERA_TARGET), CAMERA_FOCAL_LENGTH, CAMERA_RESOLUTION)

    stage.GetRootLayer().Save()
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
