"""Print what a scene builder needs to know about a USD asset, so nothing is guessed.

    python inspect_asset.py usd/robots/manipulators/universal_robots/ur10e.zip
    python inspect_asset.py usd/tools/parallel_grippers/robotiq/robotiqhande.zip --frame base_link
    python inspect_asset.py usd/environments/pick_and_place/pick_and_place_scene.zip --depth 3
    python inspect_asset.py path/to/local_asset.usd

The argument is an asset key from ``find_assets.py`` (downloaded once into
``~/.cache/telekinesis/<key>``, the same cache the builder template uses) or a local USD
file. Prints the main file and the prim to reference, units, the root's own transform
ops, bounds, the prim tree, articulation roots, rigid bodies, joints (with the welds that
hold the asset to the world), named frames, cameras, surface grippers and lights.

Paths are printed relative to the default prim: referenced at ``/World/X``, the prim
``a/b`` is ``/World/X/a/b``. ``--frame NAME`` adds the asset's bounds in the frame of the
first prim named NAME, and that prim's pose in the asset frame (XYZ metres, extrinsic
XYZ Euler degrees): for a gripper, ``--frame base_link`` gives the distance from its
mount face to its fingertips; for an arm, ``--frame tool0`` gives the flange frame.

Needs ``usd-core`` (``pip install usd-core``); Isaac Sim need not run.
"""

import argparse
import math
import pathlib
import shutil
import urllib.request
import zipfile

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

ASSET_ROOT = "https://assets.telekinesis.ai"
CACHE_DIRECTORY = pathlib.Path.home() / ".cache" / "telekinesis"
USD_SUFFIXES = {".usd", ".usda", ".usdc", ".usdz"}
# Frames worth knowing for mounting tools and cameras.
FRAME_NAMES = {"tool0", "flange", "tcp", "TCP", "base", "base_link", "world"}
LIST_LIMIT = 150  # lines per section, so a whole environment scene stays readable


def download_asset(key: str) -> pathlib.Path:
    """Download an asset of the Telekinesis asset library into the cache, once; return its path."""
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


def resolve_main_file(argument: str) -> pathlib.Path:
    """Return the USD file to open: a local file, or the shallowest USD file of a downloaded bundle."""
    local = pathlib.Path(argument)
    if local.is_file():
        return local.resolve()
    downloaded = download_asset(argument)
    if downloaded.is_file():
        return downloaded
    # The bridge's rule: the main file sits above the layers and meshes it references.
    candidates = [path for path in downloaded.rglob("*") if path.suffix.lower() in USD_SUFFIXES]
    if not candidates:
        raise SystemExit(f"{downloaded} holds no USD file.")
    return min(candidates, key=lambda path: (len(path.parts), path.name))


def euler_xyz_degrees(rotation: Gf.Rotation) -> tuple[float, float, float]:
    """Extrinsic XYZ Euler angles in degrees (scipy "xyz"), the convention of the bridge's offsets."""
    matrix = Gf.Matrix3d(rotation)
    # Row-vector Gf matrix: transpose to the column-vector R where R = Rz * Ry * Rx.
    r = [[matrix[column][row] for column in range(3)] for row in range(3)]
    ry = math.asin(max(-1.0, min(1.0, -r[2][0])))
    if abs(math.cos(ry)) > 1e-9:
        rx = math.atan2(r[2][1], r[2][2])
        rz = math.atan2(r[1][0], r[0][0])
    else:  # gimbal lock: fold all of the remaining turn into X
        rx = math.atan2(-r[1][2], r[1][1])
        rz = 0.0
    return tuple(round(math.degrees(angle), 3) for angle in (rx, ry, rz))


def format_vector(vector) -> str:
    return "(" + ", ".join(f"{value:.4f}" for value in vector) + ")"


def format_range(bounds: Gf.Range3d) -> str:
    if bounds.IsEmpty():
        return "no extent"
    return (
        f"min {format_vector(bounds.GetMin())} max {format_vector(bounds.GetMax())} "
        f"size {format_vector(bounds.GetSize())}"
    )


def print_section(title: str, lines: list[str]) -> None:
    print(f"\n== {title} ({len(lines)})")
    for line in lines[:LIST_LIMIT]:
        print(f"  {line}")
    if len(lines) > LIST_LIMIT:
        print(f"  ... {len(lines) - LIST_LIMIT} more")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("asset", help="asset key (usd/...) or local USD file")
    parser.add_argument("--depth", type=int, default=2, help="levels of the prim tree to print")
    parser.add_argument("--frame", help="also print bounds in the frame of the first prim with this name")
    arguments = parser.parse_args()

    main_file = resolve_main_file(arguments.asset)
    stage = Usd.Stage.Open(str(main_file))
    root = stage.GetDefaultPrim()
    if not root:
        root = next(iter(stage.GetPseudoRoot().GetChildren()))
        print(f"WARNING: no default prim; reference the asset with an explicit prim path ({root.GetPath()}).")
    root_path = root.GetPath()

    def relative(prim_or_path) -> str:
        path = prim_or_path.GetPath() if hasattr(prim_or_path, "GetPath") else prim_or_path
        return "." if path == root_path else str(path.MakeRelativePath(root_path))

    print(f"Main file: {main_file.as_posix()}")
    print(f"Reference: prim.GetReferences().AddReference('{main_file.as_posix()}', '{root_path}')")
    print(f"Units: metersPerUnit={UsdGeom.GetStageMetersPerUnit(stage)} upAxis={UsdGeom.GetStageUpAxis(stage)}")
    root_ops = [op.GetOpName() for op in UsdGeom.Xformable(root).GetOrderedXformOps()]
    if root_ops:
        values = {op.GetOpName(): op.Get() for op in UsdGeom.Xformable(root).GetOrderedXformOps()}
        print(f"Root transform ops: {values}")
        print("  These compose onto the prim that references the asset: call ClearXformOpOrder() before")
        print("  adding your own ops (or AddTranslateOp raises), and re-add any scale you need to keep.")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    print(f"Bounds, root's own frame: {format_range(bbox_cache.ComputeUntransformedBound(root).ComputeAlignedRange())}")
    if root_ops:
        print(f"Bounds with root ops:     {format_range(bbox_cache.ComputeWorldBound(root).ComputeAlignedRange())}")

    tree = []

    def walk(prim: Usd.Prim, level: int) -> None:
        for child in prim.GetChildren():
            flags = []
            if child.IsInstanceable():
                flags.append("instanceable")
            if not child.IsActive():
                flags.append("inactive")
            bounds = bbox_cache.ComputeWorldBound(child).ComputeAlignedRange()
            tree.append(
                f"{'  ' * level}{child.GetName()} [{child.GetTypeName() or 'over'}]"
                f"{' ' + ','.join(flags) if flags else ''}  {format_range(bounds)}"
            )
            if level + 1 < arguments.depth:
                walk(child, level + 1)

    walk(root, 0)
    print_section(f"Prim tree under the root, depth {arguments.depth}, bounds in the asset frame", tree)

    prims = list(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
    rigid_bodies = {prim.GetPath() for prim in prims if prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    print_section(
        "Articulation roots (a gripper merged into an arm must lose this API)",
        [relative(prim) for prim in prims if prim.HasAPI(UsdPhysics.ArticulationRootAPI)],
    )

    body_lines = []
    for prim in prims:
        if prim.GetPath() not in rigid_bodies:
            continue
        mass = prim.GetAttribute("physics:mass").Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
        kinematic = prim.GetAttribute("physics:kinematicEnabled").Get()
        body_lines.append(
            f"{prim.GetName()}  path={relative(prim)}"
            f"{f'  mass={mass}' if mass else ''}{'  KINEMATIC' if kinematic else ''}"
        )
    print_section("Rigid bodies (links)", body_lines)

    collider_lines = []
    for prim in prims:
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            approximation = prim.GetAttribute("physics:approximation").Get() if prim.IsA(UsdGeom.Mesh) else prim.GetTypeName()
            collider_lines.append(f"{relative(prim)}  ({approximation or 'triangle mesh: static only'})")
    print_section("Colliders (none: things fall through it; add CollisionAPI)", collider_lines)

    joint_lines, weld_lines = [], []
    for prim in prims:
        joint = UsdPhysics.Joint(prim)
        if not joint:
            continue
        bodies = [*joint.GetBody0Rel().GetTargets(), *joint.GetBody1Rel().GetTargets()]
        details = [f"{prim.GetName()} [{prim.GetTypeName()}]"]
        details.append(" -> ".join(relative(body) if body.HasPrefix(root_path) else str(body) for body in bodies) or "no bodies")
        if prim.GetAttribute("physics:axis").Get():
            details.append(f"axis={prim.GetAttribute('physics:axis').Get()}")
        low, high = prim.GetAttribute("physics:lowerLimit").Get(), prim.GetAttribute("physics:upperLimit").Get()
        if low is not None or high is not None:
            details.append(f"limits=[{low}, {high}]")
        schemas = prim.GetMetadata("apiSchemas")
        schema_names = list(schemas.GetAddedOrExplicitItems()) if schemas else []
        if any(name.startswith("PhysicsDriveAPI") for name in schema_names):
            details.append("drive")
        if any("Mimic" in name for name in schema_names):
            details.append("MIMIC")
        if prim.GetAttribute("physics:jointEnabled").Get() is False:
            details.append("disabled")
        if prim.GetAttribute("physics:excludeFromArticulation").Get():
            details.append("excludedFromArticulation")
        joint_lines.append(f"{'  '.join(details)}  path={relative(prim)}")
        enabled = prim.GetAttribute("physics:jointEnabled").Get() is not False
        if enabled and prim.IsActive() and sum(body in rigid_bodies for body in bodies) < 2:
            weld_lines.append(f"{relative(prim)}  ({prim.GetTypeName()}, bodies {[relative(body) for body in bodies]})")
    print_section("Joints", joint_lines)
    print_section(
        "World welds: enabled joints with fewer than two rigid bodies. An arm's grounds its base: keep it. "
        "A gripper merged into an arm: deactivate it. A surface gripper's attachment points look like this "
        "too: re-park them instead",
        weld_lines,
    )

    frame_lines = []
    for prim in prims:
        if prim.GetName() in FRAME_NAMES:
            transform = Gf.Transform(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
            frame_lines.append(
                f"{relative(prim)}  at {format_vector(transform.GetTranslation())} "
                f"rot xyz {euler_xyz_degrees(transform.GetRotation())} (asset frame)"
            )
    print_section("Named frames", frame_lines)

    print_section("Cameras", [relative(prim) for prim in prims if prim.IsA(UsdGeom.Camera)])
    gripper_lines = []
    for prim in prims:
        if prim.GetTypeName() == "IsaacSurfaceGripper":
            attributes = {attribute.GetName(): attribute.Get() for attribute in prim.GetAttributes() if attribute.GetName().startswith("isaac:")}
            relationships = {relationship.GetName(): [str(target) for target in relationship.GetTargets()] for relationship in prim.GetRelationships()}
            gripper_lines.append(f"{relative(prim)}  {attributes}  {relationships}")
    print_section("Surface grippers", gripper_lines)
    print_section("Lights", [relative(prim) for prim in prims if prim.HasAPI(UsdLux.LightAPI) or prim.IsA(UsdLux.BoundableLightBase) or prim.IsA(UsdLux.NonboundableLightBase)])
    print_section("Physics scenes", [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)])

    if arguments.frame:
        frame = next((prim for prim in prims if prim.GetName() == arguments.frame), None)
        if frame is None:
            raise SystemExit(f"No prim named {arguments.frame!r} under {root_path}.")
        frame_world = UsdGeom.Xformable(frame).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        transform = Gf.Transform(frame_world)
        bounds = bbox_cache.ComputeWorldBound(root)
        bounds.Transform(frame_world.GetInverse())
        print(f"\n== Frame {relative(frame)}")
        print(f"  pose in asset frame: xyz {format_vector(transform.GetTranslation())} rot xyz {euler_xyz_degrees(transform.GetRotation())}")
        print(f"  asset bounds in this frame: {format_range(bounds.ComputeAlignedRange())}")


if __name__ == "__main__":
    main()
