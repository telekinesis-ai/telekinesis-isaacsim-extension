# SPDX-License-Identifier: Apache-2.0
"""
Assemble a gripper onto an arm with ``RobotAssembler`` and bind the result as one
articulation. Adapted from the prototyping example
(`manipulator_gripper_extension_assemble.py`); used by the robot device's
``attach_tool`` path.

After assembly the arm + gripper are a SINGLE articulation rooted at the arm
(n arm DOF + m gripper DOF). One ``SingleArticulation`` handle drives both; the
two devices keep that *same* handle and each addresses only its own columns
(resolved by joint name) -- see ``adopt_shared`` on the device classes.

omni/isaac imports live at module top (only imported inside Isaac Sim), matching
:mod:`.urdf_loader`. ``RobotAssembler`` ships as its own Kit extension, so it is
enabled + imported lazily inside :func:`assemble_tool`.
"""

import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation
from pxr import Gf, Usd

_BIND_RETRIES = 60


def find_prim_by_name(stage, root_path, name):
    """Full path of the prim named ``name`` under ``root_path`` (handles nesting)."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if prim.GetName() == name:
            return prim.GetPath().pathString
    raise RuntimeError(f"prim named {name!r} not found under {root_path!r}")


def get_articulation_base_link_name(articulation):
    """Root/base link name of an already-initialized ``SingleArticulation``.

    Used to auto-discover the gripper's mount link for ``attach_tool``: the gripper
    must be joined at its *base* link (the root of its kinematic tree), not an
    internal frame. Isaac orders an articulation's links root-first, so the base
    link is ``body_names[0]``. The name is read off the physics view (the wrapper
    does not expose it), the same path we use for joint limits.

    Why the physics view and not the robot schema: ``body_names`` comes straight
    from PhysX (``link_names``), so it is authoritative for the actual simulated
    articulation root and works even when an asset has not authored the robot
    schema. An explicit override may still be passed (a Robot Link or a Site).

    References:
    - isaacsim.core.prims Articulation.body_names (root-first link ordering;
      backed by PhysX ``link_names``).
    - Robot schema "Attach Point" (a Link or Site) and ``isaac:physics:robotLinks``
      ("ordered list of links ... starting with the base link"):
      https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/assemble_robots.html
      https://docs.isaacsim.omniverse.nvidia.com/latest/omniverse_usd/robot_schema.html
    """
    return articulation._articulation_view.body_names[0]


def apply_attach_offset(prim_path, translation_m, rotation_deg):
    """Nudge the attach prim by a custom offset in its own (mount-local) frame.

    Applied between ``begin_assembly`` and ``assemble`` so the fixed joint is baked
    at the adjusted relative pose. No-op when both offsets are zero.
    """
    if not any(translation_m) and not any(rotation_deg):
        return

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    old_mat = omni.usd.get_local_transform_matrix(prim)

    rot = (
        Gf.Rotation(Gf.Vec3d(1, 0, 0), rotation_deg[0])
        * Gf.Rotation(Gf.Vec3d(0, 1, 0), rotation_deg[1])
        * Gf.Rotation(Gf.Vec3d(0, 0, 1), rotation_deg[2])
    )
    offset = Gf.Matrix4d().SetRotate(rot)
    offset.SetTranslateOnly(Gf.Vec3d(*translation_m))

    # offset * old_mat -> offset expressed in the prim's local frame.
    new_mat = offset * old_mat
    omni.kit.commands.execute(
        "TransformPrimCommand",
        path=prim.GetPath(),
        new_transform_matrix=new_mat,
        old_transform_matrix=old_mat,
    )


async def assemble_tool(
    stage,
    arm_prim,
    arm_mount_link,
    gripper_prim,
    gripper_mount_link,
    offset,
    namespace,
    variant,
):
    """Fix the gripper's mount link to the arm's mount link via ``RobotAssembler``.

    Edits USD, so the timeline is stopped throughout. ``begin_assembly`` positions
    the gripper at the mount; ``assemble`` creates the fixed joint and removes the
    gripper's own articulation root (so the merged tree has ONE root at the arm).
    ``offset`` (a ``models.Transformation`` or ``None``) is baked in between the
    two; ``None`` => flush attach. Pumps a couple of frames so the stage recomposes
    before the caller plays the timeline and binds the merged articulation.
    """
    # RobotAssembler is packaged as an extension; enable it before importing.
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("isaacsim.robot_setup.assembler", True)
    from isaacsim.robot_setup.assembler import RobotAssembler

    omni.timeline.get_timeline_interface().stop()

    if not stage.GetPrimAtPath(arm_prim).IsValid():
        raise RuntimeError(f"arm prim {arm_prim!r} not found in the open stage")
    if not stage.GetPrimAtPath(gripper_prim).IsValid():
        raise RuntimeError(f"gripper prim {gripper_prim!r} not found in the open stage")

    arm_mount = find_prim_by_name(stage, arm_prim, arm_mount_link)
    gripper_mount = find_prim_by_name(stage, gripper_prim, gripper_mount_link)

    assembler = RobotAssembler()
    assembler.begin_assembly(
        stage,
        arm_prim,
        arm_mount,
        gripper_prim,
        gripper_mount,
        namespace,
        variant,
    )

    if offset is not None:
        apply_attach_offset(gripper_prim, offset.translation, offset.rotation)

    assembler.assemble()
    assembler.finish_assemble()

    app = omni.kit.app.get_app()
    await app.next_update_async()
    await app.next_update_async()


async def bind_shared_articulation(prim_path, name):
    """Play the timeline and return the single, initialized articulation at ``prim_path``.

    Same retry shape as ``RobotArticulation.bind``: physics may need a few frames
    to stabilize the merged topology after assembly. This one handle is shared by
    both the arm and the gripper devices.
    """
    omni.timeline.get_timeline_interface().play()
    app = omni.kit.app.get_app()
    await app.next_update_async()
    await app.next_update_async()

    for _ in range(_BIND_RETRIES):
        try:
            articulation = SingleArticulation(prim_path=prim_path, name=name)
            articulation.initialize()
            if articulation.num_dof and articulation.get_joint_positions() is not None:
                print(f"[bridge] bound shared articulation {prim_path}: {articulation.num_dof} dof {list(articulation.dof_names)}")
                return articulation
        except Exception:
            pass
        await app.next_update_async()

    raise RuntimeError(f"shared articulation at {prim_path} did not become valid after assembly")
