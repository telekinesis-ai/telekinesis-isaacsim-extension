# SPDX-License-Identifier: Apache-2.0
"""
Import a URDF as a self-contained USD asset and reference that asset into the open
stage at an exact prim path. Shared by the robot and gripper connect paths.
"""

import hashlib
import shutil
from pathlib import Path

import carb
import omni.kit.app
import omni.kit.commands
import omni.timeline
from isaacsim.asset.importer import urdf as urdf_importer
from pxr import Sdf, UsdGeom

# Where imported robots are written. One directory per URDF, so the layers the
# importer writes alongside the asset cannot collide with another robot's.
ASSET_ROOT = Path.home() / ".telekinesis" / "isaacsim-bridge" / "urdf_assets"


async def import_urdf_at(stage, urdf_path, dest_prim_path):
    """Import ``urdf_path`` and place it exactly at ``dest_prim_path``.

    The URDF is imported into its own USD asset under :data:`ASSET_ROOT` and that
    asset is referenced onto ``dest_prim_path``, so the robot arrives as one prim with
    everything it owns below it. Every call re-imports, so an edited URDF or an edited
    mesh always reaches the stage.

    Importing straight into the open stage is the other way to do this, and it is the
    wrong way whenever the stage has never been saved: the importer then has nowhere
    to write the robot's mesh prototypes and drops them into the open stage as scopes
    at the world root, pointing each link at them through internal references. The
    collider prototypes among them carry collision APIs and belong to no rigid body,
    which leaves a full set of static colliders sitting at the world origin. Importing
    to an asset keeps all of it inside the asset's own layers, where referencing brings
    in the robot subtree and nothing else.

    Returns the container path (``dest_prim_path`` itself), NOT the deeper
    articulation-root prim inside the asset. The container is what callers want:
    SingleArticulation resolves the articulation root under it automatically, while the
    rigid-body links the assembler mounts to (``wrist_3_link`` etc.) sit below the
    container too.

    Async on purpose: after the reference is added the stage must be pumped a couple of
    frames so it recomposes *before* the caller plays the timeline and binds the
    articulation. Skipping this leaves the articulation half-composed, which froze the
    bind in the live extension. The timeline must be stopped while importing (PhysX
    requirement) and is left stopped for the caller.
    """
    app = omni.kit.app.get_app()
    omni.timeline.get_timeline_interface().stop()

    asset_path = _import_asset(urdf_path, _asset_directory_for(urdf_path))

    parent = Sdf.Path(dest_prim_path).GetParentPath()
    if parent.pathString not in ("", "/") and not stage.GetPrimAtPath(parent).IsValid():
        UsdGeom.Xform.Define(stage, parent.pathString)

    UsdGeom.Xform.Define(stage, dest_prim_path)
    omni.kit.commands.execute(
        "AddReference",
        stage=stage,
        prim_path=Sdf.Path(dest_prim_path),
        reference=Sdf.Reference(asset_path.as_posix(), _asset_root_prim_path(asset_path)),
    )

    # Let the stage recompose before the caller plays the timeline and binds.
    await app.next_update_async()
    await app.next_update_async()

    return dest_prim_path


def _asset_directory_for(urdf_path):
    """Directory holding the USD asset built from ``urdf_path``.

    Keyed on the URDF's absolute path, so two robots that happen to share a file name
    get their own directory, and re-importing the same URDF always lands on the same
    one.
    """
    resolved = Path(urdf_path).resolve()
    digest = hashlib.sha1(resolved.as_posix().lower().encode()).hexdigest()[:8]
    return ASSET_ROOT / f"{resolved.stem}-{digest}"


def _asset_root_prim_path(asset_path):
    """Path of the robot prim inside the asset at ``asset_path``.

    The reference has to name that prim, because a reference with no prim path
    resolves to the asset's default prim and not every URDF importer sets one. When
    the asset names a default prim that is the robot; otherwise the asset holds exactly
    one root prim, which is the robot.
    """
    layer = Sdf.Layer.FindOrOpen(asset_path.as_posix())
    if layer is None:
        raise RuntimeError(f"cannot open the urdf asset at '{asset_path}'")

    if layer.defaultPrim:
        return Sdf.Path.absoluteRootPath.AppendChild(layer.defaultPrim)

    root_names = [prim.name for prim in layer.rootPrims]
    if len(root_names) != 1:
        raise RuntimeError(
            f"expected one root prim in the urdf asset '{asset_path}', found {root_names}"
        )
    return Sdf.Path.absoluteRootPath.AppendChild(root_names[0])


def _import_asset(urdf_path, asset_directory):
    """Build the USD asset for ``urdf_path`` in ``asset_directory``, returning its path.

    Anything already in the directory is removed rather than imported over: depending
    on the Isaac Sim version, importing into an occupied directory either writes into
    the previous asset or writes a second copy of the robot beside it.

    Raises ``RuntimeError`` when the directory cannot be prepared or the import
    produces no asset, so the caller reports a failed import rather than an internal
    error.
    """
    try:
        if asset_directory.exists():
            carb.log_info(f"[bridge] replacing the previous urdf asset in {asset_directory}")
            shutil.rmtree(asset_directory)
        asset_directory.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise RuntimeError(
            f"cannot prepare the urdf asset directory '{asset_directory}': {exc}"
        ) from exc

    # Which URDF importer the running Isaac Sim ships: newer ones expose the importer
    # as a class and no longer register the import commands at all, so the two paths
    # are told apart by what the importer provides rather than by a version number.
    if hasattr(urdf_importer, "URDFImporter"):
        asset_path = _import_with_importer_class(urdf_path, asset_directory)
    else:
        asset_path = _import_with_importer_commands(urdf_path, asset_directory)

    if not asset_path.is_file():
        raise RuntimeError(f"URDF import of '{urdf_path}' wrote no asset at '{asset_path}'")

    carb.log_info(f"[bridge] imported '{urdf_path}' to asset {asset_path}")
    return asset_path


def _import_with_importer_class(urdf_path, asset_directory):
    """Import ``urdf_path`` into ``asset_directory`` with the URDF importer class.

    The importer names the asset itself and writes its layers beside it, so the path it
    reports back is the one to reference.
    """
    config = urdf_importer.URDFImporterConfig(
        urdf_path=str(urdf_path),
        usd_path=str(asset_directory),
        merge_fixed_joints=False,
        fix_base=True,
    )
    try:
        return Path(urdf_importer.URDFImporter(config).import_urdf())
    except Exception as exc:  # an unreadable or malformed URDF raises ValueError
        raise RuntimeError(f"failed to import URDF at '{urdf_path}': {exc}") from exc


def _import_with_importer_commands(urdf_path, asset_directory):
    """Import ``urdf_path`` into ``asset_directory`` with the URDF import commands.

    Used by Isaac Sim versions whose URDF importer is driven by commands instead of by
    the importer class. There the asset's name is ours to pick.
    """
    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.distance_scale = 1.0
    import_config.parse_mimic = True  # let single-input grippers' mimic joints follow

    asset_path = asset_directory / f"{Path(urdf_path).resolve().stem}.usd"

    # get_articulation_root must stay False: with a destination the importer authors
    # the asset's variant sets and payloads on the prim path it returns, so asking for
    # the articulation root would bolt them onto the root joint instead of onto the
    # asset's root prim.
    success, imported = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        dest_path=str(asset_path),
        get_articulation_root=False,
    )
    if not success or not imported:
        raise RuntimeError(f"failed to import URDF at '{urdf_path}' (check the path and file)")

    return asset_path
