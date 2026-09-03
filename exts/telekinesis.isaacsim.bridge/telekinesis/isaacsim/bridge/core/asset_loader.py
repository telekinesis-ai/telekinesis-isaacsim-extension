# SPDX-License-Identifier: Apache-2.0
"""
Bring a robot or tool into the open stage at an exact prim path, from either of the
two descriptions a client can hand the bridge: a URDF, which is imported to a
self-contained USD asset first (:func:`import_urdf_at`), or a USD asset that is
already prepared (:func:`reference_usd_at`). Both end up referencing one asset onto
one prim path, so the device arrives as a single prim with everything it owns below
it. Shared by the robot and gripper connect paths.
"""

import hashlib
import shutil
from pathlib import Path

import carb
import omni.kit.app
import omni.kit.commands
import omni.timeline
from isaacsim.asset.importer import urdf as urdf_importer
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

# Where imported robots are written. One directory per URDF, so the layers the
# importer writes alongside the asset cannot collide with another robot's.
ASSET_ROOT = Path.home() / ".telekinesis" / "isaacsim-bridge" / "urdf_assets"


async def import_urdf_at(stage, urdf_path, dest_prim_path):
    """Import ``urdf_path`` and place it exactly at ``dest_prim_path``.

    The URDF is imported into its own USD asset under :data:`ASSET_ROOT` and that
    asset is referenced onto ``dest_prim_path``, so the robot arrives as one prim with
    everything it owns below it. Every call re-imports, so an edited URDF or an edited
    mesh always reaches the stage.

    The robot arrives at the world origin unless another robot is already standing
    there, in which case it is parked beside that one instead (see
    :func:`_park_clear_of_existing_robots`).

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
    asset_path = _import_asset(urdf_path, _asset_directory_for(urdf_path))
    _author_missing_drives(asset_path)
    return await _reference_asset_at(stage, asset_path, dest_prim_path)


async def reference_usd_at(stage, usd_path, dest_prim_path):
    """Reference the prepared USD asset at ``usd_path`` onto ``dest_prim_path``.

    The USD counterpart of :func:`import_urdf_at`, for a robot or tool that is
    modelled as a USD asset instead of described by a URDF -- a welding gun, a
    surface gripper. The asset is referenced where it lies rather than copied: a
    client fetches its bundle into a stable cache directory, and the layers the asset
    references resolve relative to it from there.

    Nothing in the asset is edited on the way in. Unlike a URDF import there is no
    drive to complete: a modelled asset authors its own drives, and rewriting them
    would override what its author chose.

    Returns ``dest_prim_path``, and shares every other property with
    :func:`import_urdf_at`: the asset arrives as one prim, parked clear of the robots
    already in the stage, with the timeline stopped and the stage recomposed.
    """
    usd_path = Path(usd_path)
    if not usd_path.is_file():
        raise RuntimeError(f"cannot find the usd asset '{usd_path}'")

    return await _reference_asset_at(stage, usd_path, dest_prim_path)


async def _reference_asset_at(stage, asset_path, dest_prim_path):
    """Reference one USD asset onto ``dest_prim_path`` and park it clear of the robots.

    The step both loaders end in, whether the asset was built from a URDF a moment ago
    or shipped as USD. Stops the timeline (a PhysX requirement while the stage is
    rebuilt) and leaves it stopped, defines the destination and any missing parent, adds
    the reference, then pumps the app so the stage recomposes before the caller measures,
    plays or binds anything.
    """
    app = omni.kit.app.get_app()
    omni.timeline.get_timeline_interface().stop()

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

    # Let the stage recompose before the robot is measured against what is already
    # there, and before the caller plays the timeline and binds.
    await app.next_update_async()
    await app.next_update_async()

    _park_clear_of_existing_robots(stage, dest_prim_path)

    return dest_prim_path


def _park_clear_of_existing_robots(stage, dest_prim_path):
    """Offset a freshly referenced robot so it does not arrive inside another one.

    An import carries no pose of its own, so it lands at the world origin -- which is
    where the last import landed too. Two robots sharing that space are each held
    there by their own fixed base, so physics cannot separate them and resolves the
    overlap with forces large enough to throw either of them across the stage.
    Offsetting the new robot along +X until it clears the robots already beside it
    keeps an import from disturbing them, and a robot later assembled onto an arm is
    moved to its mount anyway, so the parking spot lasts only until it is given a
    real one.

    Only a robot counts as an occupant. A ground plane or a stage prop shares the
    origin with every robot by design, and counting those would push the first import
    off the origin for no reason.
    """
    gap_metres = 0.1

    prim = stage.GetPrimAtPath(dest_prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()

    occupied = Gf.Range3d()
    for sibling in stage.GetPrimAtPath(prim.GetPath().GetParentPath()).GetChildren():
        # Measured whole rather than from the articulation root down: which prim of a
        # robot carries the root API is up to whoever authored it, and on an asset
        # that puts it on the root joint that prim bounds nothing.
        if sibling == prim or not any(
            p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in Usd.PrimRange(sibling)
        ):
            continue
        occupied = Gf.Range3d.GetUnion(
            occupied, cache.ComputeWorldBound(sibling).ComputeAlignedRange()
        )

    if bounds.IsEmpty() or occupied.IsEmpty():
        return
    if Gf.Range3d.GetIntersection(bounds, occupied).IsEmpty():
        return

    shift = occupied.GetMax()[0] - bounds.GetMin()[0] + gap_metres

    # Added to whatever translation the asset authors rather than authored fresh: a
    # modelled asset arrives with its own xform ops, and adding a second translate op
    # to a prim that already has one is an error.
    xformable = UsdGeom.Xformable(prim)
    translate = next(
        (
            op
            for op in xformable.GetOrderedXformOps()
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
        ),
        None,
    ) or xformable.AddTranslateOp()
    authored = translate.Get()
    if authored is None:
        translate.Set(Gf.Vec3d(shift, 0.0, 0.0))
    else:
        translate.Set(type(authored)(authored[0] + shift, authored[1], authored[2]))
    carb.log_info(
        f"[bridge] parked {dest_prim_path} {shift:.3f} m along +X, clear of the robot(s) "
        "already on the stage"
    )


def _author_missing_drives(asset_path):
    """Author position-drive values into the asset for joints that arrived without any.

    A joint moves only through its position drive, and the drive works only when the
    asset authors its stiffness, damping and effort limit. URDF has no notion of drive
    stiffness, so the importer cannot fill it in from the file, and what it does author
    varies: a URDF with a ``<dynamics damping="...">`` tag yields a drive with damping
    and no stiffness, one without the tag yields no drive values at all. This completes
    such joints in the asset itself, where the values are part of the robot: they
    survive every timeline stop, re-import into any stage, and need no code to
    re-apply them.

    The values follow NVIDIA's published robot assets. Stiffness defaults to their
    manipulator scale, and damping is authored at 0.004 of the stiffness whenever the
    stiffness was, replacing any authored damping -- the two describe one drive, and a
    damping written for a drive that had no stiffness describes nothing. The ratio is
    the drive's slew: damping this small lets a joint snap to a new target and turns an
    effort-limit saturation into a transient instead of a sustained shove. The effort
    limit is taken from the URDF's own effort limit where the importer preserved it and
    the drive arrived without one; torque clamping is what keeps a drive with these
    gains from overpowering the joint, so a URDF with a wrong effort limit shows up as
    misbehaviour of that joint rather than being compensated here.

    A robot whose joints drive a mimic linkage -- a parallel gripper, where one driver
    joint moves the whole finger mechanism through mimic constraints -- gets a far
    smaller stiffness and a far heavier damping. A gripper is commanded in steps, not
    trajectories, so its driver sees its entire stroke as one error; at manipulator
    stiffness that demands hundreds of thousands of newton-metres, the drive rides its
    effort limit for the whole stroke, and the reaction through the flange is violent
    enough to blow up the simulation of whatever the gripper is bolted to. The gripper
    stiffness is taken from a working NVIDIA-style asset (OnRobot RG6: 3.75), some
    eight hundred times softer. The mimic joints are what mark the asset as such a
    mechanism.

    The gripper's damping ratio is not NVIDIA's. At their ratio the driver is close to
    undamped, and an undamped drive on gram-scale links inside a closed mimic loop
    diverges -- oscillation the loop cannot absorb, growing until the state is NaN and
    the simulation is gone. The 0.5 ratio caps the finger's approach speed at twice its
    remaining error per second, which is the behaviour every working close in this
    bridge has had; an arm needs no such cap because it is fed a trajectory whose error
    is always small.

    A gripper driver's stiffness is additionally capped so that its whole stroke, seen
    as one error, demands no more torque than the joint's effort limit allows. A drive
    pushed past its limit saturates, and a saturated drive in a closed mimic loop is
    the other way the loop diverges -- an open() across the full stroke NaN'd the
    simulation this way while a shorter close() survived. Below the cap the drive is
    linear everywhere: motion speed is set by the damping ratio alone, and the force on
    a gripped object stays within the effort limit the URDF declares.

    A gripper's joints also get their velocity limits lifted, to the 10000 degrees per
    second NVIDIA's own gripper assets author. The physics enforces the URDF's velocity
    limit on every joint, mimic followers included, and a linkage whose followers must
    track the driver exactly but are pinned to a velocity wall tears the solve apart:
    an open() whose commanded speed crossed the limit NaN'd this way while a slower
    close() stayed under it and survived. The damping ratio still bounds the real
    motion, at twice the remaining error per second, so lifting the wall changes what
    the solver is allowed to correct, not how fast the gripper moves.

    Only joints whose composed stiffness is zero or absent are touched, so an asset
    that authors real drives keeps them. Mimic joints are skipped: the mimic constraint
    moves them, and a drive would fight it. Written as overs on the asset's root layer,
    which outweighs the importer's internal layers regardless of the asset's variant
    structure.
    """
    stage = Usd.Stage.Open(asset_path.as_posix())
    if stage is None:
        raise RuntimeError(f"cannot open the urdf asset at '{asset_path}'")

    if any(_is_mimic(prim) for prim in stage.Traverse()):
        # A mimic linkage: gram-scale links behind one driver commanded in steps.
        angular_stiffness = 3.75  # USD degrees; the OnRobot RG6's authored figure
        linear_stiffness = 1.0e4  # per metre
        damping_ratio = 0.5  # heavy: caps the step response a closed loop must absorb
    else:
        angular_stiffness = 3000.0  # NVIDIA's manipulator scale, USD degrees
        linear_stiffness = 1.0e5  # per metre
        damping_ratio = 0.004  # NVIDIA's ratio: snappy tracking of a streamed trajectory

    completed = []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            kind, stiffness = "angular", angular_stiffness
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            kind, stiffness = "linear", linear_stiffness
        else:
            continue

        if damping_ratio == 0.5:
            # A gripper joint must never hit a velocity wall (see the docstring).
            PhysxSchema.PhysxJointAPI.Apply(prim)
            prim.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(
                10000.0
            )

        if _is_mimic(prim):
            continue

        drive = UsdPhysics.DriveAPI.Get(prim, kind)
        authored = drive.GetStiffnessAttr().Get() if drive else None
        if authored:
            continue

        drive = UsdPhysics.DriveAPI.Apply(prim, kind)

        max_force = drive.GetMaxForceAttr().Get()
        if max_force is None or not max_force < 1.0e37:
            effort = prim.GetAttribute("urdf:limit:effort").Get()
            if effort and effort > 0.0:
                max_force = float(effort)
                drive.CreateMaxForceAttr(max_force)
            else:
                max_force = None

        if damping_ratio == 0.5 and max_force is not None:
            # A gripper driver: keep the full-stroke demand inside the effort limit,
            # so the drive never saturates (see the docstring).
            joint = (
                UsdPhysics.RevoluteJoint(prim)
                if kind == "angular"
                else UsdPhysics.PrismaticJoint(prim)
            )
            lower = joint.GetLowerLimitAttr().Get()
            upper = joint.GetUpperLimitAttr().Get()
            if lower is not None and upper is not None and upper > lower:
                stiffness = min(stiffness, max_force / float(upper - lower))

        drive.CreateStiffnessAttr(stiffness)
        drive.CreateDampingAttr(stiffness * damping_ratio)
        completed.append(prim.GetName())

    if completed:
        stage.GetRootLayer().Save()
        carb.log_warn(
            f"[bridge] asset {asset_path}: authored position-drive values for joint(s) the "
            f"import left without any: {completed}. The drives clamp at the URDF's effort "
            "limits, so a joint that misbehaves under these gains has a wrong effort limit "
            "in its URDF."
        )


def _is_mimic(prim):
    """Whether ``prim`` carries a mimic schema, read from the raw metadata.

    Raw metadata rather than ``GetAppliedSchemas()``, which silently drops schema names
    whose plugin is not loaded -- and the mimic schema is the physics backend's own.
    """
    schemas = prim.GetMetadata("apiSchemas")
    return bool(schemas) and any("Mimic" in name for name in schemas.GetAddedOrExplicitItems())


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
    """Path of the device prim inside the asset at ``asset_path``.

    The reference has to name that prim, because a reference with no prim path
    resolves to the asset's default prim and neither every URDF importer nor every
    hand-modelled asset sets one. When the asset names a default prim that is the
    device; otherwise the asset holds exactly one root prim, which is the device.
    """
    layer = Sdf.Layer.FindOrOpen(asset_path.as_posix())
    if layer is None:
        raise RuntimeError(f"cannot open the usd asset at '{asset_path}'")

    if layer.defaultPrim:
        return Sdf.Path.absoluteRootPath.AppendChild(layer.defaultPrim)

    root_names = [prim.name for prim in layer.rootPrims]
    if len(root_names) != 1:
        raise RuntimeError(
            f"expected one root prim in the usd asset '{asset_path}', found {root_names}"
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
