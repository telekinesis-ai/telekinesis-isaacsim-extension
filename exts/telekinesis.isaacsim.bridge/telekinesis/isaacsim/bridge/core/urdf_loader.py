# SPDX-License-Identifier: Apache-2.0
"""
Import a URDF into the open stage at an exact prim path, returning the
articulation-root path. Adapted from the prototyping examples
(`*_urdf_loading_extension.py`); shared by the robot and gripper connect paths.
"""

import omni.kit.commands
import omni.timeline
from pxr import Sdf, UsdGeom


def import_urdf_at(stage, urdf_path, dest_prim_path):
    """Import ``urdf_path`` and place it exactly at ``dest_prim_path``.

    The importer parents the result under the default prim, so we import then move
    it. The timeline must be stopped while importing (PhysX requirement). Returns
    the articulation-root path (preserving any nested suffix the importer adds).
    """
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

    return dest_prim_path + imported[len(top.pathString):]
