# SPDX-License-Identifier: Apache-2.0
"""
Import a URDF into the open stage at an exact prim path, returning the
articulation-root path. Adapted from the prototyping examples
(`*_urdf_loading_extension.py`); shared by the robot and gripper connect paths.
"""

import omni.kit.app
import omni.kit.commands
import omni.timeline
from pxr import Sdf, UsdGeom


async def import_urdf_at(stage, urdf_path, dest_prim_path):
    """Import ``urdf_path`` and place it exactly at ``dest_prim_path``.

    The importer parents the result under the default prim, so we import then move
    it. The timeline must be stopped while importing (PhysX requirement). Returns
    the container path (``dest_prim_path`` itself), NOT the deeper articulation-root
    prim the importer reports (e.g. ``/World/ur10e/root_joint``). The container is
    what callers want: SingleArticulation resolves the articulation root under it
    automatically (the Articulation view runs get_articulation_root_api_prim_path),
    while the rigid-body links the assembler mounts to (``wrist_3_link`` etc.) are
    children of the container, not of the nested root joint.

    Async on purpose: after the import + move the stage must be pumped a couple of
    frames so it recomposes *before* the caller plays the timeline and binds the
    articulation. Skipping this (importing synchronously, then calling play()
    immediately) leaves the articulation half-composed -- which froze the bind in
    the live extension. Mirrors the known-good urdf_loading prototyping examples.
    """
    app = omni.kit.app.get_app()
    omni.timeline.get_timeline_interface().stop()

    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.distance_scale = 1.0
    import_config.parse_mimic = True  # let single-input grippers' mimic joints follow

    parent = Sdf.Path(dest_prim_path).GetParentPath()
    if parent.pathString not in ("", "/") and not stage.GetPrimAtPath(parent).IsValid():
        UsdGeom.Xform.Define(stage, parent.pathString)

    _, imported = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        get_articulation_root=True,
    )

    # Climb to the top-level imported prim (importer may return a nested root).
    top = Sdf.Path(imported)
    while top.GetParentPath() not in (parent, Sdf.Path("/"), Sdf.Path()):
        top = top.GetParentPath()

    if top.pathString != dest_prim_path:
        omni.kit.commands.execute("MovePrim", path_from=top.pathString, path_to=dest_prim_path)

    # Let the stage recompose after the import/move before the caller plays the
    # timeline and binds (timeline is still stopped here, matching the examples).
    await app.next_update_async()
    await app.next_update_async()

    return dest_prim_path
