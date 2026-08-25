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
  articulation to merge, so it is placed at the mount directly, bolted on with a
  fixed joint that is excluded from the articulation, and its attachment points are
  re-parked onto the arm. The arm's articulation is unchanged; the gripper stays its
  own device.

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
# Name of the fixed joint the suction path authors, matched on to clear out the ones
# an earlier assembly of the same gripper left behind.
_MOUNT_JOINT_NAME = "AssemblerFixedJoint"


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


def offset_matrix(offset):
    """Matrix form of a ``models.Transformation`` mount offset, identity for ``None``.

    ``offset.translation`` is in meters and ``offset.rotation`` is XYZ Euler degrees,
    both read in the mount frame.
    """
    if offset is None:
        return Gf.Matrix4d(1.0)

    rotation_deg = offset.rotation
    rot = (
        Gf.Rotation(Gf.Vec3d(1, 0, 0), rotation_deg[0])
        * Gf.Rotation(Gf.Vec3d(0, 1, 0), rotation_deg[1])
        * Gf.Rotation(Gf.Vec3d(0, 0, 1), rotation_deg[2])
    )
    matrix = Gf.Matrix4d().SetRotate(rot)
    matrix.SetTranslateOnly(Gf.Vec3d(*offset.translation))
    return matrix


def apply_attach_offset(prim_path, offset):
    """Nudge the attach prim by a custom offset in its own (mount-local) frame.

    Applied between ``begin_assembly`` and ``assemble`` so the fixed joint is baked
    at the adjusted relative pose. No-op without an offset.
    """
    if offset is None:
        return

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    old_mat = omni.usd.get_local_transform_matrix(prim)

    # offset * old_mat -> offset expressed in the prim's local frame.
    new_mat = offset_matrix(offset) * old_mat
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

    apply_attach_offset(gripper_prim, offset)

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
    mask_collisions,
):
    """Fix a suction gripper to the arm's mount link and re-park its attachment points.

    The suction counterpart to :func:`assemble_tool`. A suction gripper is not an
    articulation, so the two cannot share ``RobotAssembler``'s assembly path -- that
    one begins by stripping the attachment's articulation root, which a suction
    gripper does not have, and it authors the result as an asset variant through a
    session sublayer that is torn down afterwards. The steps a suction gripper needs
    instead:

    1. Any fixed joint an earlier assembly left on the gripper is removed, so the
       only thing holding the gripper afterwards is the joint this call creates.
    2. The gripper is moved so its mount body sits on the arm's mount link (see
       :func:`_place_at_mount`), in the stage's own edit layer, so the pose the
       gripper is bolted at is the pose it keeps.
    3. The gripper is freed from the stage -- any joint holding it to the world is
       disabled and it stops being kinematic -- so the new fixed joint is the only
       thing constraining it.
    4. A fixed joint excluded from the articulation joins the arm's mount link to
       the gripper's mount body, leaving the arm's articulation topology unchanged.
    5. Every attachment point is re-parked onto the arm's mount link: its
       ``physics:body1`` is re-pointed there and its local frame recomputed so the
       parked joint holds nothing. Until this is done the attachment points are
       still parked against whatever body the asset shipped them against, and the
       gripper cannot grip.
    6. Collisions between the arm and the gripper are masked (``mask_collisions``)
       -- the cups sit against the flange, and unlike the articulated path there is
       no articulation merge to sort this out.

    Edits USD, so the timeline is stopped throughout; the caller plays it again and
    re-binds both devices, since stopping invalidates the arm's articulation handle
    and the gripper's component.
    """
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("isaacsim.robot_setup.assembler", True)
    from isaacsim.robot_setup.assembler import robot_assembler as isaac_assembler

    app = omni.kit.app.get_app()
    omni.timeline.get_timeline_interface().stop()
    # Stopping restores the arm's joints to their authored pose, but not before the
    # next updates land. Every transform below is read off the arm, so waiting here is
    # what keeps the gripper from being mounted against the last simulated pose and
    # then having the arm move out from under it when the timeline plays again.
    await app.next_update_async()
    await app.next_update_async()

    if not stage.GetPrimAtPath(arm_prim).IsValid():
        raise RuntimeError(f"arm prim {arm_prim!r} not found in the open stage")
    if not stage.GetPrimAtPath(gripper_prim).IsValid():
        raise RuntimeError(f"gripper prim {gripper_prim!r} not found in the open stage")

    arm_mount = find_prim_by_name(stage, arm_prim, arm_mount_link)

    _remove_previous_mount_joints(stage, gripper_prim)
    _place_at_mount(stage, arm_mount, gripper_prim, gripper_mount_path, offset)

    assembler = isaac_assembler.RobotAssembler()
    _release_from_stage(stage, assembler, gripper_prim, attachment_point_paths)

    await app.next_update_async()
    await app.next_update_async()
    fixed_joint = _create_mount_joint(stage, arm_mount, gripper_mount_path, offset)

    # Parking reads the placement back off the stage, so let the move above compose.
    await app.next_update_async()
    _park_attachment_points(stage, isaac_assembler, attachment_point_paths, arm_mount)

    if mask_collisions:
        # The pair filter is authored on the GRIPPER, not on the arm: it is symmetric,
        # and applying an API schema to the arm's root prim resyncs the arm's whole
        # subtree. A robot imported from URDF carries its link visuals as instanceable
        # prims referencing shared prototypes, and that resync is what drops them --
        # the links lose their meshes while the frames keep moving. Nothing else in
        # this function writes to the arm, so the attach leaves it composed as it was.
        assembler.mask_collisions(gripper_prim, arm_prim)

    await app.next_update_async()
    await app.next_update_async()

    return fixed_joint.GetPath().pathString


def _remove_previous_mount_joints(stage, gripper_prim):
    """Delete the mount joints an earlier assembly of this gripper left behind.

    Each assembly authors its joint at a fresh path rather than reusing one, so an
    old joint would survive a re-assembly -- still enabled, and still holding the
    gripper at whatever pose it was authored for. That fights the new joint for the
    same body, which PhysX resolves by dragging the arm.

    A re-assembly happens whenever the bridge's assembly record is dropped but the
    stage is not: re-registering the arm or the gripper, or deleting and re-creating
    either of them, is enough.
    """
    stale = [
        prim.GetPath().pathString
        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_prim))
        if prim.GetName().startswith(_MOUNT_JOINT_NAME)
    ]
    if stale:
        carb.log_info(f"[bridge] removing stale mount joints before re-assembly: {stale}")
        omni.kit.commands.execute("DeletePrims", paths=stale, destructive=True)


def _place_at_mount(stage, arm_mount_path, gripper_prim, gripper_mount_path, offset):
    """Move the gripper so its mount body lands on the arm's mount link.

    The pose asked for is ``w_T_g = w_T_m . m_T_g``: the arm mount link's own world
    transform, with the requested offset ``m_T_g`` (identity when ``offset`` is
    ``None``) applied in the mount frame. Gf matrices multiply row-vector-first, so
    that composition is spelled ``m_T_g * w_T_m`` below.

    The gripper's *root* is what carries a transform, so the gripper mount body's
    pose within the gripper is divided back out; when the mount body is the root
    itself that factor is the identity. The result is written as the root's transform
    relative to its parent -- the offset replaces the gripper's placement in the
    stage rather than adding to it.
    """
    gripper = stage.GetPrimAtPath(gripper_prim)
    gripper_world = omni.usd.get_world_transform_matrix(gripper)
    parent_world = omni.usd.get_world_transform_matrix(gripper.GetParent())
    arm_mount_world = omni.usd.get_world_transform_matrix(
        stage.GetPrimAtPath(arm_mount_path)
    )
    gripper_mount_world = omni.usd.get_world_transform_matrix(
        stage.GetPrimAtPath(gripper_mount_path)
    )

    # Mount body expressed in the gripper root's frame, so it can be divided out.
    mount_in_gripper = gripper_mount_world * gripper_world.GetInverse()
    target_world = mount_in_gripper.GetInverse() * offset_matrix(offset) * arm_mount_world

    omni.kit.commands.execute(
        "TransformPrimCommand",
        path=gripper.GetPath(),
        new_transform_matrix=target_world * parent_world.GetInverse(),
        old_transform_matrix=omni.usd.get_local_transform_matrix(gripper),
    )


def _create_mount_joint(stage, arm_mount_path, gripper_mount_path, offset):
    """Bolt the gripper's mount body to the arm's mount link at exactly ``m_T_g``.

    The joint's frame on the arm's mount link is the requested offset, and its frame
    on the gripper is the gripper mount body's own origin -- which is the pose
    :func:`_place_at_mount` just moved the gripper to, so the joint is satisfied the
    moment it is created and there is nothing for physics to correct.

    Both frames are written from the offset rather than measured back off the stage.
    Measuring is what the assembler does, and it makes the joint only as good as the
    poses the arm and the gripper happen to hold at that instant; writing the offset
    means the joint says what was asked for regardless.

    The joint is excluded from the articulation, so the arm's DOF are unchanged.
    """
    joint = UsdPhysics.FixedJoint.Define(
        stage, f"{gripper_mount_path}/{_MOUNT_JOINT_NAME}"
    )
    joint.CreateBody0Rel().SetTargets([Sdf.Path(arm_mount_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(gripper_mount_path)])

    mount_pose = Gf.Transform(offset_matrix(offset))
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(mount_pose.GetTranslation()))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(mount_pose.GetRotation().GetQuat()))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateExcludeFromArticulationAttr(True)
    return joint


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
