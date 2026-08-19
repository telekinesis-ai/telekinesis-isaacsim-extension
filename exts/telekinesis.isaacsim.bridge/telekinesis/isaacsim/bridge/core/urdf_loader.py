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
from pxr import Sdf, UsdGeom

# Where imported robots are written. One directory per URDF, so the
# ``configuration/`` subdirectory the importer writes next to the asset cannot
# collide with another robot's.
ASSET_ROOT = Path.home() / ".telekinesis" / "isaacsim-bridge" / "urdf_assets"


async def import_urdf_at(stage, urdf_path, dest_prim_path):
    """Import ``urdf_path`` and place it exactly at ``dest_prim_path``.

    The URDF is imported into its own USD asset under :data:`ASSET_ROOT` and that
    asset is referenced onto ``dest_prim_path``, so the robot arrives as one prim
    with everything it owns below it. Every call re-imports, so an edited URDF or an
    edited mesh always reaches the stage.

    Importing straight into the open stage is the other way to do this, and it is
    the wrong way whenever the stage has never been saved: the importer then has
    nowhere to write the asset's layers, so it drops the robot's mesh prototypes
    into the open stage as ``/visuals``, ``/colliders`` and ``/meshes`` scopes at
    the world root and points each link at them through internal references. The
    prototypes under ``/colliders`` carry collision APIs and belong to no rigid
    body, which leaves a full set of static colliders sitting at the world origin;
    the importer filters those against the robot it just imported, but not against
    anything mounted onto that robot afterwards. Importing to an asset keeps all of
    it inside the asset's own layers, where referencing brings in the robot subtree
    and nothing else.

    Returns the container path (``dest_prim_path`` itself), NOT the deeper
    articulation-root prim inside the asset (e.g. ``.../root_joint``). The container
    is what callers want: SingleArticulation resolves the articulation root under it
    automatically (the Articulation view runs get_articulation_root_api_prim_path),
    while the rigid-body links the assembler mounts to (``wrist_3_link`` etc.) are
    children of the container, not of the nested root joint.

    Async on purpose: after the reference is added the stage must be pumped a couple
    of frames so it recomposes *before* the caller plays the timeline and binds the
    articulation. Skipping this leaves the articulation half-composed, which froze
    the bind in the live extension. The timeline must be stopped while importing
    (PhysX requirement) and is left stopped for the caller.
    """
    app = omni.kit.app.get_app()
    omni.timeline.get_timeline_interface().stop()

    asset_path = _asset_path_for(urdf_path)
    _import_asset(urdf_path, asset_path)

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


def _asset_path_for(urdf_path):
    """Path of the USD asset built from ``urdf_path``.

    Keyed on the URDF's absolute path, so two robots that happen to share a file
    name get their own asset, and re-importing the same URDF always lands on the
    same asset.
    """
    resolved = Path(urdf_path).resolve()
    digest = hashlib.sha1(resolved.as_posix().lower().encode()).hexdigest()[:8]
    return ASSET_ROOT / f"{resolved.stem}-{digest}" / f"{resolved.stem}.usd"


def _asset_root_prim_path(asset_path):
    """Path of the robot prim inside the asset built at ``asset_path``.

    The reference has to name that prim: the importer does not set a default prim on
    the asset it writes, and a reference with no prim path resolves to the default
    prim. The asset holds exactly one root prim, the robot.
    """
    layer = Sdf.Layer.FindOrOpen(asset_path.as_posix())
    if layer is None:
        raise RuntimeError(f"cannot open the urdf asset at '{asset_path}'")

    root_names = [prim.name for prim in layer.rootPrims]
    if len(root_names) != 1:
        raise RuntimeError(
            f"expected one root prim in the urdf asset '{asset_path}', found {root_names}"
        )
    return Sdf.Path.absoluteRootPath.AppendChild(root_names[0])


def _import_asset(urdf_path, asset_path):
    """Build the USD asset for ``urdf_path`` at ``asset_path``.

    Any asset already there is removed rather than imported over: the importer opens
    an existing destination and writes into it, which would leave prims from the
    previous import of the same robot behind.

    Raises ``RuntimeError`` when the directory cannot be prepared or the importer
    writes no asset, so the caller reports a failed import rather than an internal
    error.
    """
    try:
        if asset_path.parent.exists():
            carb.log_info(f"[bridge] replacing the previous urdf asset at {asset_path.parent}")
            shutil.rmtree(asset_path.parent)
        asset_path.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise RuntimeError(
            f"cannot prepare the urdf asset directory '{asset_path.parent}': {exc}"
        ) from exc

    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.distance_scale = 1.0
    import_config.parse_mimic = True  # let single-input grippers' mimic joints follow

    # get_articulation_root must stay False: with a destination the importer authors
    # the asset's variant sets and payloads on the prim path it returns, so asking
    # for the articulation root would bolt them onto the root joint instead of onto
    # the asset's root prim.
    success, imported = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        dest_path=str(asset_path),
        get_articulation_root=False,
    )
    if not success or not imported:
        raise RuntimeError(f"failed to import URDF at '{urdf_path}' (check the path and file)")

    if not asset_path.is_file():
        raise RuntimeError(f"URDF import of '{urdf_path}' wrote no asset at '{asset_path}'")

    carb.log_info(f"[bridge] imported '{urdf_path}' to asset {asset_path} ({imported})")
