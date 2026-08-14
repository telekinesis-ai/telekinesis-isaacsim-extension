# SPDX-License-Identifier: Apache-2.0
"""
Assemble a gripper onto an arm with ``RobotAssembler``. Adapted from the
prototyping example (`manipulator_gripper_extension_assemble.py`); used by the
articulation service's ``assemble_robot`` path.

Two kinds of gripper, two paths:

* An **articulated** gripper (:func:`assemble_tool`) is merged into the arm, so
  afterwards the arm + gripper are a SINGLE articulation rooted at the arm (n arm
  DOF + m gripper DOF). One ``SingleArticulation`` handle drives both; the two
  devices keep that *same* handle and each addresses only its own columns
  (resolved by joint name) -- see ``adopt_shared_articulation`` on the device class.
* A **suction** gripper (:func:`attach_surface_gripper`) has no joints and no
  articulation to merge, so it is bolted on with a fixed joint that is excluded
  from the articulation, and its attachment points are re-parked onto the arm. The
  arm's articulation is unchanged; the gripper stays its own device.

omni/isaac imports live at module top (only imported inside Isaac Sim), matching
:mod:`.urdf_loader`. ``RobotAssembler`` ships as its own Kit extension, so it is
enabled + imported lazily inside the two assembly functions.
"""

import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation
import carb
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

_BIND_RETRIES = 60


def find_prim_by_name(stage, root_path, name):
    """Full path of the prim named ``name`` under ``root_path`` (handles nesting)."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if prim.GetName() == name:
            return prim.GetPath().pathString
    raise RuntimeError(f"prim named {name!r} not found under {root_path!r}")


def get_articulation_base_link_name(articulation):
    """Root/base link name of an already-initialized ``SingleArticulation``.

    Used to auto-discover the gripper's mount link for ``assemble_robot``: the gripper
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


async def attach_surface_gripper(
    stage,
    arm_prim,
    arm_mount_link,
    gripper_prim,
    gripper_mount_path,
    attachment_point_paths,
    offset,
    namespace,
    variant,
    mask_collisions,
):
    """Fix a suction gripper to the arm's mount link and re-park its attachment points.

    The suction counterpart to :func:`assemble_tool`. A suction gripper is not an
    articulation, so the two cannot share ``RobotAssembler.assemble()`` -- that path
    begins by stripping the attachment's articulation root, which a suction gripper
    does not have. The steps a suction gripper needs instead:

    1. ``begin_assembly`` places the gripper at the arm's mount link (the same
       placement the articulated path uses; it does not touch articulations). When
       the mount frame is the gripper prim itself, its transform is cleared first --
       see :func:`_reset_local_transform`.
    2. ``offset`` is baked in on top of that placement, then the gripper is freed
       from the stage -- any joint holding it to the world is disabled and it stops
       being kinematic -- so the new fixed joint is the only thing constraining it.
    3. A fixed joint excluded from the articulation joins the arm's mount link to
       the gripper's mount body, leaving the arm's articulation topology unchanged.
    4. Every attachment point is re-parked onto the arm's mount link: its
       ``physics:body1`` is re-pointed there and its local frame recomputed so the
       parked joint holds nothing. Until this is done the attachment points are
       still parked against whatever body the asset shipped them against, and the
       gripper cannot grip.
    5. Collisions between the arm and the gripper are masked (``mask_collisions``)
       -- the cups sit against the flange, and unlike the articulated path there is
       no articulation merge to sort this out.

    Edits USD, so the timeline is stopped throughout; the caller plays it again and
    re-binds both devices, since stopping invalidates the arm's articulation handle
    and the gripper's component.
    """
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("isaacsim.robot_setup.assembler", True)
    from isaacsim.robot_setup.assembler import robot_assembler as isaac_assembler

    omni.timeline.get_timeline_interface().stop()

    if not stage.GetPrimAtPath(arm_prim).IsValid():
        raise RuntimeError(f"arm prim {arm_prim!r} not found in the open stage")
    if not stage.GetPrimAtPath(gripper_prim).IsValid():
        raise RuntimeError(f"gripper prim {gripper_prim!r} not found in the open stage")

    arm_mount = find_prim_by_name(stage, arm_prim, arm_mount_link)

    if gripper_mount_path == gripper_prim:
        _reset_local_transform(stage, gripper_prim)

    assembler = isaac_assembler.RobotAssembler()
    assembler.begin_assembly(
        stage,
        arm_prim,
        arm_mount,
        gripper_prim,
        gripper_mount_path,
        namespace,
        variant,
    )

    if offset is not None:
        apply_attach_offset(gripper_prim, offset.translation, offset.rotation)

    _release_from_stage(stage, assembler, gripper_prim, attachment_point_paths)

    fixed_joint = assembler.create_fixed_joint(
        gripper_mount_path, target0=arm_mount, target1=gripper_mount_path
    )
    fixed_joint.CreateExcludeFromArticulationAttr(True)

    _park_attachment_points(stage, isaac_assembler, attachment_point_paths, arm_mount)

    if mask_collisions:
        assembler.mask_collisions(arm_prim, gripper_prim)

    assembler.finish_assemble()

    app = omni.kit.app.get_app()
    await app.next_update_async()
    await app.next_update_async()

    return fixed_joint.GetPath().pathString


def _reset_local_transform(stage, prim_path):
    """Put a prim back at its parent's origin, so ``begin_assembly`` can place it.

    ``begin_assembly`` reads the mount frame's transform *relative to the prim being
    attached* and cancels it out, which is right when the mount frame is a link
    inside the gripper. When the mount frame IS the gripper prim -- the default for a
    suction gripper, which is one rigid body with no internal mount link -- what it
    reads instead is the gripper's own placement in the stage, and the gripper ends
    up displaced by exactly that. Clearing the transform first makes the two
    readings the same, and costs nothing: the placement is about to be overwritten.
    """
    prim = stage.GetPrimAtPath(prim_path)
    omni.kit.commands.execute(
        "TransformPrimCommand",
        path=prim.GetPath(),
        new_transform_matrix=Gf.Matrix4d(1.0),
        old_transform_matrix=omni.usd.get_local_transform_matrix(prim),
    )


def _release_from_stage(stage, assembler, gripper_prim, attachment_point_paths):
    """Stop anything but the new fixed joint from holding the gripper in place.

    A gripper that sits in the stage on its own is usually held there by a joint to
    the world, by being kinematic, or both. Left in place, either fights the fixed
    joint to the arm: the gripper stays where it was while the arm moves away from
    it, and PhysX resolves the contradiction violently.

    A joint with only one body counts as a joint to the world, which is exactly what
    an attachment point looks like in an asset whose attachment points were never
    parked. Those are the joints assembly is about to park, not disable, so they are
    left alone here.
    """
    gripper = stage.GetPrimAtPath(gripper_prim)
    for joint in Usd.PrimRange(gripper):
        if joint.GetPath().pathString in attachment_point_paths:
            continue
        if assembler.is_root_joint(joint):
            UsdPhysics.Joint(joint).CreateJointEnabledAttr(False)
    if gripper.HasAttribute("physics:kinematicEnabled"):
        gripper.GetAttribute("physics:kinematicEnabled").Set(False)


def _park_attachment_points(stage, isaac_assembler, attachment_point_paths, arm_mount):
    """Re-park the gripper's attachment-point joints against the arm's mount link.

    An attachment point is a D6 joint between the suction cup and a second body.
    The gripper replaces that second body with the object it grips and restores it
    on release, so what the asset authored there is only where the joint rests
    while the gripper is open -- but it has to be a real rigid body, or PhysX never
    creates the joint and the gripper silently never grips.

    Pointing it at the arm's mount link and recomputing the joint's local frame to
    match leaves the resting joint holding the cup exactly where it already is, so
    parking it changes nothing until the gripper closes.
    """
    cache = UsdGeom.XformCache()
    for path in attachment_point_paths:
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(path))
        joint.GetBody1Rel().SetTargets([Sdf.Path(arm_mount)])
        joint.CreateExcludeFromArticulationAttr(True)
        isaac_assembler.set_opposite_body_transform(
            stage, cache, joint.GetPrim(), body0base=True, fixpos=True, fixrot=True
        )


async def bind_shared_articulation(prim_path, name):
    """Play the timeline and return the single, initialized articulation at ``prim_path``.

    Same retry shape as ``core.articulation.SingleArticulation.bind``: physics may need a few frames
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
                carb.log_info(
                    f"[bridge] bound shared articulation {prim_path}: "
                    f"{articulation.num_dof} dof {list(articulation.dof_names)}"
                )
                return articulation
        except Exception:
            pass
        await app.next_update_async()

    raise RuntimeError(f"shared articulation at {prim_path} did not become valid after assembly")
