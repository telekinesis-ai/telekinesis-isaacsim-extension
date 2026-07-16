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

    Leaves the stage with a VALID default prim. The importer would otherwise point
    the default prim at the imported robot (``make_default_prim``); our MovePrim
    then relocates that prim, leaving the default-prim metadata dangling, which
    crashes anything that later reads ``GetDefaultPrim().GetPath()`` -- e.g. the
    viewport's drag-drop handler. We disable that flag and repair the default prim
    to a top-level container so dropped assets land under it.
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
    import_config.make_default_prim = (
        False  # don't hijack the default prim; MovePrim would dangle it
    )

    parent = Sdf.Path(dest_prim_path).GetParentPath()
    if parent.pathString not in ("", "/") and not stage.GetPrimAtPath(parent).IsValid():
        UsdGeom.Xform.Define(stage, parent.pathString)

    success, imported = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        get_articulation_root=True,
    )
    if not success or not imported:
        raise RuntimeError(f"failed to import URDF at '{urdf_path}' (check the path and file)")

    # Climb to the top-level imported prim (importer may return a nested root).
    top = Sdf.Path(imported)
    while top.GetParentPath() not in (parent, Sdf.Path("/"), Sdf.Path()):
        top = top.GetParentPath()

    if top.pathString != dest_prim_path:
        omni.kit.commands.execute("MovePrim", path_from=top.pathString, path_to=dest_prim_path)

    # Guarantee a valid default prim. With make_default_prim disabled a pre-existing
    # default prim survives the import; if there is none (or the importer left a
    # dangling one), point it at a top-level container so the viewport drag-drop
    # handler (make_prim_path -> GetDefaultPrim().GetPath()) never reads an empty
    # path. SetDefaultPrim requires a root-level prim, which `container` always is.
    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        container = parent if parent.pathString not in ("", "/") else Sdf.Path(dest_prim_path)
        container_prim = stage.GetPrimAtPath(container)
        if container_prim and container_prim.IsValid():
            stage.SetDefaultPrim(container_prim)

    # Let the stage recompose after the import/move before the caller plays the
    # timeline and binds (timeline is still stopped here, matching the examples).
    await app.next_update_async()
    await app.next_update_async()

    return dest_prim_path
