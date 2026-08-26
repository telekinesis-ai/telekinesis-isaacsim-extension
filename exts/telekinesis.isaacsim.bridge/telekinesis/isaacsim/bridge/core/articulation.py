# SPDX-License-Identifier: Apache-2.0
"""
Generic articulation device: binds one isaacsim.core.prims.SingleArticulation
handle and drives a chosen subset of its joints in joint space.

This is the single device the bridge exposes -- there is no separate robot vs
gripper class. The extension stays generic: it applies an ``ArticulationAction``
to whatever DOF indices are currently the device's "driven subset" and reports
whether the move reached its target or stalled. All robot/gripper *semantics*
live in the client:

* A robot drives every DOF: leave the driven subset at its default (all joints)
  and send raw joint angles (radians) to :meth:`move_j` (drive to the target over
  time) or :meth:`set_j` (teleport there instantly).
* A gripper drives one actuated joint: the client first reads the driver joint
  (:func:`find_driver_joint`, exposed as a getter), narrows the driven subset to
  it via :meth:`set_driven_joints`, then sends a single angle. The
  fraction<->radians math and open/close convenience are the client's job; the
  joint limits it needs come from :meth:`get_dof_limits`.

Everything here is native Isaac units (radians). The request handler calls these
coroutines directly and awaits completion (the ``async def main()`` blocking
style) -- no background worker, no command queue.
"""

import asyncio
import time

import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.core import prims
from isaacsim.core.utils.types import ArticulationAction
import carb
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from .robot_assembler import get_articulation_base_link_name

# A PhysX position drive settles with a small steady-state offset (gravity, drive
# stiffness, the URDF importer's default gains), so an over-tight position
# tolerance is never satisfied and a move looks "never done". A move is done when
# the joints are within _REACH_TOLERANCE_RAD of target (reached cleanly), OR when
# they have stopped moving for _SETTLED_FRAMES consecutive frames (stalled --
# either settled at a steady-state offset, or blocked by a grasped object). The
# consecutive-frame requirement keeps the stall test from firing before the drive
# has accelerated the joints. The frame cap is a ~30 s backstop at 60 fps.
_REACH_TOLERANCE_RAD = 5e-3  # ~0.3 deg: target reached cleanly
_SETTLED_VELOCITY_RAD_S = 5e-3  # joints no longer moving
_SETTLED_FRAMES = 5  # consecutive low-velocity frames => stalled
_BIND_RETRIES = 60
_MOTION_MAX_FRAMES = 1800

# Position-drive values substituted for a driven joint that reports none, so that
# every robot the bridge is asked to position-control can actually be positioned.
# Three ways a joint arrives unusable: a URDF that leaves its joint effort limit
# at zero (a great many do), which carries through to a drive permitted no torque;
# a USD authored with no drive gains, which the bridge uses as it finds it rather
# than re-authoring, unlike a URDF it imports itself; and a drive given stiffness
# but no damping, which does respond to commands but rings.
#
# Zero is a missing value here rather than a deliberate one, so it is filled in.
# Each quantity is filled in only where the drive reports zero -- an authored
# value is never overridden. That restraint matters beyond tidiness: the drive's
# damping is the gain on its *velocity* term, and this class only ever writes
# position targets (ArticulationAction(joint_positions=...), leaving
# joint_velocities None), so a drive's target velocity is whatever its USD
# authored and is never cleared. Supplying damping to a joint that did not ask
# for it hands a gain to a stale target velocity the bridge does not control,
# and the joint rotates continuously.
#
# The stiffness is sized from the load the joint actually moves -- its effective
# inertia times a fixed square frequency -- because no single stiffness serves the
# whole size range the bridge sees. One firm enough to keep a thirty-kilogram arm
# from sagging asks a fifty-gram gripper finger for thousands of newton-metres,
# which the drive happily supplies up to its effort ceiling; the reaction then
# throws whatever the gripper is bolted to. Scaling with inertia makes the torque a
# command asks for proportional to what is being moved, so the same rule gives an
# arm's shoulder a drive that holds it up and a finger one that does not shove the
# arm. The effort is a ceiling high enough not to constrain any of them
# (newton-metres), bounding the drive's output without shaping it. Override any of
# them with set_dof_gains.
#
# The frequency is squared because a position drive behaves as a second-order system
# in the load it moves: stiffness = inertia x frequency^2. It sets how briskly a joint
# answers a command, and raising it asks more torque of everything -- which is what has
# to stay small at the gripper end, where a full stroke of an RG2 finger should ask a
# few newton-metres of a wrist rated for fifty-six, not the thousand its own asset
# declares itself good for.
#
# The damping is derived from the stiffness rather than chosen separately, because
# the two only mean something together: a position drive travels at roughly
# (stiffness / damping) x position error, so their ratio -- not the stiffness on its
# own -- is what decides how fast a substituted drive moves. Fixing that ratio at a
# settling time makes a substituted drive's speed a property of this bridge instead
# of a property of whatever velocity ceiling the joint's asset happens to declare:
# a joint one radian from its target closes it at about 2 rad/s either way. Left
# unrelated, a high stiffness against a low damping asks for a speed no manipulator
# should move at, and the joint crosses its whole range within a frame or two --
# visibly a teleport rather than a move.
#
# Inertia alone is not the whole demand on a stiffness: it says how hard a joint is to
# accelerate, not how hard it is to hold up. Sizing from inertia fixes the joint's natural
# frequency, and a joint with almost no inertia of its own therefore gets almost no
# stiffness -- a UR10e wrist_3 sized this way came out at 0.7 N.m/rad, too soft to hold the
# 0.01 N.m it was actually carrying. So the stiffness also has a floor: the one that keeps
# the joint within _FALLBACK_SAG_RAD of the pose it was told to hold, under the heaviest
# gravity load that joint can be asked to carry.
#
# _FALLBACK_SAG_RAD has to stay comfortably tighter than the tracking error a caller
# accepts, because a joint sized to the allowance sits AT the allowance in the worst pose.
#
# _FALLBACK_STIFFNESS is used only when the physics backend reports neither figure and
# there is nothing to size the drive against. It suits a manipulator, which is the case
# where getting it wrong means an arm on the floor.
_FALLBACK_FREQUENCY_RAD_S = 60.0
_FALLBACK_SAG_RAD = 3.5e-3  # ~0.2 deg of droop allowed under the joint's heaviest load
_FALLBACK_STIFFNESS = 1.0e7
_FALLBACK_SETTLING_TIME_S = 0.5
_FALLBACK_MAX_EFFORT = 1.0e6
_GRAVITY_M_S2 = 9.81


def joint_prims(stage, prim_path):
    """Map joint name -> prim for every UsdPhysics.Joint at or under ``prim_path``.

    Keyed by name because that is what a DOF is identified by; the joints
    themselves sit at whatever depth the asset (or the URDF importer) put them.
    """
    root = stage.GetPrimAtPath(prim_path)
    if root.IsA(UsdPhysics.Joint):  # importer may return the root joint, not the container
        root = root.GetParent()
    return {p.GetName(): p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)}


def is_mimic_joint(prim):
    """Whether ``prim`` is a joint driven by a mimic constraint following another joint.

    Matches both spellings the physics backends use, since the applied schema is
    named after whichever one authored the asset: ``PhysxMimicJointAPI:rot*``
    (PhysX) and ``NewtonMimicAPI`` (Newton).
    """
    return any("Mimic" in schema for schema in prim.GetAppliedSchemas())


def find_driver_joint(stage, prim_path, dof_names):
    """Return the gripper's actuated driver joint name from ``dof_names``.

    A single-input gripper has one actuated joint; the rest are mimic joints that
    follow it. The driver is the non-mimic joint, preferring one with a
    UsdPhysics.DriveAPI. Falls back to the first DOF if detection is inconclusive.
    This is a USD/PhysX schema walk a pure HTTP client cannot do, so it stays in
    the extension (exposed as the ``/driver_joint`` getter).
    """
    if not dof_names:
        raise ValueError(f"articulation at {prim_path} has no driven joints to search for a driver")

    joints = joint_prims(stage, prim_path)

    fallback = None
    for name in dof_names:
        prim = joints.get(name)
        if prim is None or is_mimic_joint(prim):
            continue
        if prim.HasAPI(UsdPhysics.DriveAPI):
            return name
        fallback = fallback or name
    return fallback or dof_names[0]


class SingleArticulation:
    """Binds a single articulation at ``prim_path`` and drives its driven joints
    (radians).

    Named to match the isaacsim.core.prims.SingleArticulation handle it wraps
    (imported as the ``prims`` module, not the bare class, so it doesn't shadow
    this one) -- this class adds the bridge's own driven-subset/retry/HTTP-facing
    behavior on top of that single Isaac Sim handle.
    """

    def __init__(self, prim_path, name="articulation", joint_names=None):
        """Store prim path and initialise joint state; call `bind()` before use."""
        self.prim_path = prim_path
        self._name = name
        self._articulation = None
        self.num_dof = 0
        self.dof_names = []
        # The DOFs this device actually drives, as names. None => all of them (the
        # default; a robot owns every joint). The client narrows this to one joint
        # for a gripper via set_driven_joints; the assembly step overwrites it with
        # just the arm's (or gripper's) joints. bind/_resolve_driven_joints turns
        # the names into joint_indices so moves address only those columns.
        self.joint_names = joint_names
        self.joint_indices = []
        self._move_lock = asyncio.Lock()
        self._start_time = time.monotonic()
        self._target = None  # last commanded joint target (rad)
        # Drive values as they stood before _backfill_missing_drive_gains substituted
        # them, by joint name, so narrowing the driven subset can put back what the
        # asset authored on the joints being dropped. See set_driven_joints.
        self._pre_substitution_gains = {}

    async def bind(self):
        """(Re)initialize the articulation against the current physics view.

        Starts the timeline if it is not already running, since an articulation has
        no physics handle to bind to while it is stopped.

        Safe to call repeatedly: reuses the handle and just re-initializes, which
        is needed when the timeline was stopped/replayed between client sessions
        (a stale handle returns None from get_joint_positions). Only reports bound
        once joint state actually reads back non-None.
        """
        omni.timeline.get_timeline_interface().play()
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()

        last_exc = None
        for _ in range(_BIND_RETRIES):
            try:
                if self._articulation is None:
                    self._articulation = prims.SingleArticulation(
                        prim_path=self.prim_path, name=self._name
                    )
                self._articulation.initialize()
                if (
                    self._articulation.num_dof
                    and self._articulation.get_joint_positions() is not None
                ):
                    # Order matters, and all of these run within this one frame: the
                    # drive needs usable gains, and a speed it can reach them at,
                    # before it is asked to hold a pose, and the pose it is aligned
                    # to is the one worth recording as the reset state.
                    self._resolve_driven_joints()
                    self._backfill_missing_drive_gains()
                    self._correct_imported_prismatic_max_velocities()
                    self._align_drive_to_current_pose()
                    self._ensure_default_state_populated()
                    carb.log_info(
                        f"[bridge] bound articulation {self.prim_path}: "
                        f"{self.num_dof} dof {self.dof_names}"
                    )
                    return
            except Exception as exc:
                last_exc = exc
            await app.next_update_async()

        detail = f"articulation at {self.prim_path} did not become valid"
        if last_exc is not None:
            detail += f" (last error: {last_exc!r})"
        raise RuntimeError(detail)

    def _align_drive_to_current_pose(self):
        """Point the position drive at the pose the joints are already in.

        A freshly imported articulation carries whatever drive targets the importer
        authored -- normally zero for every joint, regardless of where the joints
        were posed on the stage. Starting the timeline therefore makes the drives
        pull the articulation toward that authored target, which shows up as the
        arm snapping away from the pose it was set up in the moment physics begins;
        the lightest, widest-range joints (a wrist) travel the furthest before
        anything corrects it.

        Aligning the target with the measured positions here, in the same frame the
        handle becomes valid, leaves the articulation standing where it was. The
        positions are written back as well, not just commanded, so the alignment
        holds immediately and without depending on the drive: a joint the drive
        cannot move (no stiffness) would otherwise keep falling, and any joint that
        drifted in the few frames the handle took to become valid is put back rather
        than pulled back. The velocities are zeroed so nothing carries over from a
        previous run's motion.

        This covers every DOF of the underlying handle rather than just this
        device's driven subset: the stale targets are a property of the
        articulation, and the joints this device does not drive would otherwise
        still be pulled.
        """
        positions = self._articulation.get_joint_positions()
        self._articulation.set_joint_positions(positions)
        self._articulation.set_joint_velocities(np.zeros_like(positions))
        self._articulation.apply_action(ArticulationAction(joint_positions=positions))

    def _effective_joint_inertias(self):
        """Effective inertia of every DOF of the underlying handle, or ``None``.

        The diagonal of the articulation's generalized mass matrix, in the handle's own
        DOF order: how much inertia each joint has to move on its own, with the rest of
        the rig held still. Kilogram metres squared for a revolute joint, kilograms for
        a prismatic one. Used to size a substituted position drive to its load.

        The figure depends on the pose it is read in -- an outstretched arm's shoulder
        carries more than a folded one's -- so it describes the articulation as the
        bridge found it, which is the pose a substituted drive is first asked to hold.

        ``None`` when the physics backend offers no mass matrix, or offers one whose
        shape does not line up with the DOF order (a floating-base rig's carries six
        root rows and columns ahead of the joints). The caller then falls back on fixed
        gains.
        """
        # Broad on purpose: a backend that cannot supply a mass matrix, or supplies it
        # as a tensor type this does not convert, must not take the bind down with it --
        # sizing the drive is an improvement on fixed gains, not a precondition for
        # having any.
        try:
            matrices = self._articulation._articulation_view.get_mass_matrices()
            if matrices is None:
                return None
            matrix = np.asarray(matrices[0], dtype=float)
        except Exception as exc:
            carb.log_warn(
                f"[bridge] articulation {self.prim_path}: no usable mass matrix from the physics "
                f"backend ({exc!r}); substituted drives fall back on fixed gains, which suit a "
                "manipulator and are far too strong for a gripper"
            )
            return None

        total = len(self._articulation.dof_names)
        if matrix.shape != (total, total):
            return None
        return np.diag(matrix)

    def _effective_joint_gravity_torques(self):
        """Gravity torque on every DOF of the underlying handle, or ``None``.

        What each joint's drive has to supply just to keep the rig where it is, in the
        handle's own DOF order: newton-metres for a revolute joint, newtons for a
        prismatic one. Used to give a substituted position drive enough stiffness that
        the joint does not hang below the pose it was told to hold.

        Pose-dependent in the same way as :meth:`_effective_joint_inertias`, and read in
        the same pose: an outstretched arm's shoulder carries far more than a folded
        one's.

        ``None`` when the physics backend offers no gravity forces, or offers a row whose
        length does not match the DOF count. The caller then falls back on sizing the
        drive from inertia alone.
        """
        # Broad for the same reason as the mass matrix: a better-sized drive is an
        # improvement, never a precondition for the bind succeeding.
        try:
            forces = self._articulation._articulation_view.get_generalized_gravity_forces()
            if forces is None:
                return None
            torques = np.asarray(forces[0], dtype=float)
        except Exception as exc:
            carb.log_warn(
                f"[bridge] articulation {self.prim_path}: no usable gravity forces from the "
                f"physics backend ({exc!r}); substituted drives fall back on the load bounded "
                "from the mass hanging below each joint, which misses whatever is not held on "
                "by a joint"
            )
            return None

        if torques.shape != (len(self._articulation.dof_names),):
            return None
        return np.abs(torques)

    def _gravity_torque_bounds(self, joints):
        """Heaviest gravity torque each joint in ``joints`` can be asked to hold, by name.

        ``get_generalized_gravity_forces`` reports the load a joint carries in the pose the
        robot is standing in, which is the wrong quantity to size a drive against: the gains
        are fixed for the session while the load is not, so a joint whose axis carries little
        in that one pose is sized for the lighter case and sags once the arm turns. This
        bounds the load instead, from quantities that do not depend on the pose -- the mass
        hanging below the joint and how far from its axis that mass can reach.

        The bound is ``mass * g * lever``, with the mass summed over the rigid bodies below
        the joint and the lever taken as the furthest the bounding box of those bodies gets
        from the joint. Not projected onto the joint's axis, so it is an over-estimate rather
        than an under-estimate: sizing a drive slightly too stiff costs tracking crispness,
        sizing it too soft leaves the joint somewhere other than where it was sent.

        "Below the joint" follows the joints rather than the USD hierarchy: every body
        reachable from the joint's second body by walking joints in the parent-to-child
        direction. That is what makes a welded-on tool count. A gripper assembled onto a
        flange is a sibling prim held there by a fixed joint, so reading the arm's own USD
        subtree would size its wrist for the wrist alone and leave it far too soft for what
        the gripper does to it -- which is how a gripper opening came to throw the arm.
        Walking joints also terminates on a closed linkage, such as the mimic-driven loop in
        a two-finger gripper, because each body is counted once.

        The whole stage is scanned for joints, once, because the tool that welds itself to
        this articulation is not under its prim path. A joint whose bound cannot be worked
        out is left out, and falls back on the load sampled at the current pose.
        """
        stage = self._articulation.prim.GetStage()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

        children = {}
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            joint = UsdPhysics.Joint(prim)
            if not joint:
                continue
            parents = joint.GetBody0Rel().GetTargets()
            offspring = joint.GetBody1Rel().GetTargets()
            if parents and offspring:
                children.setdefault(parents[0], []).append(offspring[0])

        bounds = {}
        for name, prim in joints.items():
            targets = UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()
            if not targets:
                continue
            root = stage.GetPrimAtPath(targets[0])
            if not root.IsValid():
                continue

            reached = {targets[0]}
            frontier = [targets[0]]
            while frontier:
                for path in children.get(frontier.pop(), []):
                    if path not in reached:
                        reached.add(path)
                        frontier.append(path)

            mass = 0.0
            box = Gf.Range3d()
            for path in reached:
                body = stage.GetPrimAtPath(path)
                if not body.IsValid():
                    continue
                if body.HasAPI(UsdPhysics.MassAPI):
                    reported = UsdPhysics.MassAPI(body).GetMassAttr().Get()
                    mass += float(reported) if reported else 0.0
                box = Gf.Range3d.GetUnion(
                    box, cache.ComputeWorldBound(body).ComputeAlignedRange()
                )
            if mass <= 0.0 or box.IsEmpty():
                continue

            origin = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            ).ExtractTranslation()
            lever = max((box.GetCorner(corner) - origin).GetLength() for corner in range(8))
            if lever <= 0.0:
                continue

            bounds[name] = mass * _GRAVITY_M_S2 * lever

        return bounds

    def _backfill_missing_drive_gains(self):
        """Give driven joints whose position drive reports no gains usable ones.

        Every move this device performs is a position-drive command, so the gains
        decide both whether it works at all and whether it is stable. Two ways a
        joint arrives unusable: no stiffness, so the drive generates no torque,
        ignores every position command, and its links sag under gravity; or
        stiffness with no damping, so the drive does respond but rings --
        overshooting, returning, overshooting -- which on load, with every joint
        doing it at once, reads as the articulation dancing.

        The stiffness and the effort ceiling are filled in only where the drive
        reports zero, so an authored value is never overridden. Damping is supplied
        whenever the stiffness was, authored or not: the two describe one drive, and
        a damping meant for a joint that had no position drive says nothing about how
        one this stiff should behave. Pairing a substituted stiffness with an
        unrelated damping is what makes a drive travel at a speed nothing chose.

        A substituted stiffness is sized from two demands, whichever is larger. Its
        effective inertia keeps the torque a command asks for proportional to the load
        being moved, so a light joint does not shove whatever it is bolted to; the gravity
        load it carries sets a floor stiff enough that the joint does not hang below the
        pose it was told to hold. Neither covers both cases on its own, and the inertia
        term cannot cover the second one at all: it fixes the joint's natural frequency,
        which says nothing about how hard the joint resists being pushed off its target,
        so a joint with almost no inertia gets almost no stiffness however much it carries.

        The gravity load is taken as the larger of what the joint carries right now and
        what it could ever carry -- see :meth:`_gravity_torque_bounds`. The first figure
        alone is what the robot is standing in when this runs and changes as it moves, so
        a joint unloaded in that one pose would be sized for the lighter case and would sag
        once the arm turned. Sizing once, against the heavier case, is what keeps the gains
        fixed for the whole session, which is what makes a motion repeatable.

        A joint that still stops short of its target wants its real figures through
        :meth:`set_dof_gains`; the substituted ones are chosen to work, not to describe the
        robot.

        The joint's authored effort ceiling cannot serve as either scale: an asset is
        free to declare a figure that bears no relation to what the joint carries, and
        the grippers the bridge loads declare larger ceilings for a finger than a UR10e
        declares for its shoulder.

        What was substituted is logged: the values are chosen to work rather than to
        describe the real robot, so torque readings from an affected joint do not
        report its true limits. Filling in the URDF's effort limits, or authoring the
        USD's drives, is the proper fix; :meth:`set_dof_gains` overrides the
        substituted values in the meantime.

        Mimic joints are exempt: a mimic constraint is what moves them to follow the
        driver joint, so drive values they leave at zero are authored that way rather
        than missing. Substituting gains there pins each mimic joint at its current
        pose and fights the constraint the moment the driver joint moves.

        So are joints that belong to another device. On a merged rig every DOF of the
        assembled arm-and-gripper is visible here, and a device that drives all of
        them -- an arm bound to an already-assembled prim -- would otherwise reach
        across and substitute gains on the gripper's mimic and linkage joints, which
        is the pinning above by another route. A joint outside this device's own prim
        path is not this device's to correct.
        """
        joints = joint_prims(self._articulation.prim.GetStage(), self.prim_path)
        inertias = self._effective_joint_inertias()
        gravity_torques = self._effective_joint_gravity_torques()
        gravity_bounds = self._gravity_torque_bounds(joints)

        substituted = {}
        sizing = {}
        self._pre_substitution_gains = {}
        for name, index, props in zip(self.dof_names, self.joint_indices, self.dof_properties()):
            prim = joints.get(name)
            if prim is None or is_mimic_joint(prim):
                continue
            gains = {}
            if props["stiffness"] == 0.0:
                inertia_demand = 0.0
                if inertias is not None and inertias[index] > 0.0:
                    inertia_demand = inertias[index] * _FALLBACK_FREQUENCY_RAD_S**2
                # The sampled load and the bound are both lower bounds on what the joint
                # will be asked to hold -- the sample because it describes one pose, the
                # bound because it counts only what hangs off a joint. Whichever is larger
                # is the better figure to size against.
                load = 0.0
                if gravity_torques is not None:
                    load = gravity_torques[index]
                load = max(load, gravity_bounds.get(name, 0.0))
                gravity_demand = load / _FALLBACK_SAG_RAD if load > 0.0 else 0.0
                demand = max(inertia_demand, gravity_demand)
                gains["stiffness"] = demand if demand > 0.0 else _FALLBACK_STIFFNESS
                gains["damping"] = gains["stiffness"] * _FALLBACK_SETTLING_TIME_S
                # Which demand won, and by how much, is what tells a joint that stops
                # short of its target apart from one that was never sized for the load
                # it turned out to carry.
                sizing[name] = {
                    "inertia": round(inertia_demand, 1),
                    "gravity": round(gravity_demand, 1),
                    "load_n_m": round(load, 3),
                    "chose": "gravity" if gravity_demand > inertia_demand else "inertia",
                }
            if props["max_effort"] == 0.0:
                gains["max_effort"] = _FALLBACK_MAX_EFFORT
            if gains:
                self._pre_substitution_gains[name] = {key: props[key] for key in gains}
                self.set_dof_gains(indices=[index], **gains)
                substituted[name] = gains

        if substituted:
            carb.log_warn(
                f"[bridge] articulation {self.prim_path}: substituted position-drive values for "
                f"joint(s) that reported none: {substituted}. Without them these joints ignore "
                "position commands and sag under gravity; fill in the URDF's effort limits or "
                "author the USD's drives to control them with the robot's own figures. Each "
                f"substituted stiffness was sized from {sizing}, where the gravity figure is the "
                "load the joint carries in the pose the robot is standing in now."
            )

    def _correct_imported_prismatic_max_velocities(self):
        """Undo the degree conversion the URDF importer applies to a prismatic joint's
        velocity limit.

        The importer converts each imported joint's URDF velocity limit from radians
        per second to degrees per second. That is right for a revolute joint and wrong
        for a prismatic one, whose limit is already a linear speed and needs no
        conversion, so an imported prismatic joint arrives permitted to travel 180/pi
        times faster than its URDF asked for. Since a position drive is only ever
        commanded to a target, not at a speed, that ceiling is what a large move
        actually travels at: a gripper finger given roughly fifty-seven times its
        intended speed crosses its whole stroke inside a frame or two, which reads as
        the joint teleporting rather than closing.

        The importer records the URDF's own figure next to the converted one, so this
        is a comparison rather than a guess: a prismatic joint whose limit is exactly
        the degree conversion of its recorded ``urdf:limit:velocity`` is put back to
        that recorded value. Any other joint is left alone -- a revolute one, a limit
        that was authored rather than imported, an asset carrying no record to compare
        against -- so this corrects itself out of the way once the importer is fixed.
        """
        joints = joint_prims(self._articulation.prim.GetStage(), self.prim_path)

        corrected = {}
        for name, index, props in zip(self.dof_names, self.joint_indices, self.dof_properties()):
            prim = joints.get(name)
            if prim is None or not prim.IsA(UsdPhysics.PrismaticJoint):
                continue
            attribute = prim.GetAttribute("urdf:limit:velocity")
            urdf_velocity = attribute.Get() if attribute else None
            if not urdf_velocity or urdf_velocity <= 0.0:
                continue
            if not np.isclose(props["max_velocity"], np.degrees(urdf_velocity), rtol=1e-3):
                continue
            self._articulation._articulation_view.set_max_joint_velocities(
                np.array([[urdf_velocity]], dtype=float), joint_indices=[index]
            )
            corrected[name] = {"was": props["max_velocity"], "now": urdf_velocity}

        if corrected:
            carb.log_warn(
                f"[bridge] articulation {self.prim_path}: the URDF importer converted these "
                f"prismatic joint(s) velocity limit to degrees per second, which does not apply "
                f"to a linear axis; restored the limit their URDF asked for: {corrected}"
            )

    def _ensure_default_state_populated(self):
        """Isaac reports no stored default (reset) pose until something sets one --
        freshly imported/opened, ``get_joints_default_state()`` is ``None``. Seed it
        from the current joint positions/velocities on first bind so
        ``joints_default_state`` is never empty for a freshly bound device, and a
        Stop+Play returns to where it was when the bridge found it rather than to
        an arbitrary/unset pose. A no-op once a default is already stored (e.g. set
        explicitly via ``set_joints_default_state``, or from a prior bind).
        """
        if self._articulation.get_joints_default_state() is not None:
            return
        self._articulation.set_joints_default_state(
            positions=self._articulation.get_joint_positions(),
            velocities=self._articulation.get_joint_velocities(),
        )

    def _resolve_driven_joints(self):
        """Work out which columns of ``self._articulation`` this device drives.

        By joint NAME, so it survives a merged (assembled) rig's DOF ordering.
        ``joint_names is None`` => drive every DOF. The reported
        ``dof_names``/``num_dof`` are the driven subset, so ``info``, ``get_joints_state``,
        and ``get_dof_limits`` all match what ``move_j`` moves.
        """
        all_names = list(self._articulation.dof_names)
        if self.joint_names is None:
            self.joint_indices = list(range(len(all_names)))
        else:
            if not self.joint_names:
                # Narrowing to zero joints is never legitimate -- catch it here,
                # at the point the client actually made the mistake, rather than
                # leaving a zero-DOF device that crashes some other call later
                # (e.g. find_driver_joint's dof_names[0] fallback).
                raise ValueError(
                    f"cannot narrow the driven joints of articulation at {self.prim_path} "
                    "to an empty set"
                )
            missing = [n for n in self.joint_names if n not in all_names]
            if missing:
                raise ValueError(
                    f"unknown joint name(s) {missing} for articulation at {self.prim_path}; "
                    f"available: {all_names}"
                )
            self.joint_indices = [all_names.index(n) for n in self.joint_names]
        self.dof_names = [all_names[i] for i in self.joint_indices]
        self.num_dof = len(self.joint_indices)

    def _validate_indices(self, idx):
        """Raise ValueError if any of ``idx`` falls outside the underlying
        articulation's actual DOF range. ``indices`` may target any DOF of the
        (possibly merged) rig, not just this device's driven subset, so this
        checks against the full DOF count rather than ``self.num_dof``. Without
        this, an out-of-range index reaches PhysX directly and throws whatever
        low-level error it throws, instead of a clean client-facing message.
        """
        total = len(self._articulation.dof_names)
        bad = [i for i in idx if i < 0 or i >= total]
        if bad:
            raise ValueError(
                f"index/indices {bad} out of range for articulation with {total} DOF(s)"
            )

    def set_driven_joints(self, joint_names):
        """Narrow (or widen) the set of joints this device drives, by name.

        The "SET the driving joints" step: a gripper client first reads the driver
        joint (``/driver_joint``) and passes ``[driver_name]`` here, after which the
        device behaves exactly like a robot with one joint. Re-resolves against the
        current handle; forgets any pending move target.

        Any drive values :meth:`bind` had to substitute on a joint that is no longer
        driven are restored to what the asset authored. Until the driven subset is
        known, bind has to cover the whole articulation, which on a gripper includes
        the passive joints of its linkage: those carry no drive on purpose, and a
        substituted position drive pins each one at its current pose and locks the
        mechanism. Narrowing is the point at which they are known not to be driven.
        """
        self.joint_names = list(joint_names)
        self._resolve_driven_joints()

        all_names = list(self._articulation.dof_names)
        dropped = [name for name in self._pre_substitution_gains if name not in self.dof_names]
        for name in dropped:
            self.set_dof_gains(
                indices=[all_names.index(name)], **self._pre_substitution_gains.pop(name)
            )
        if dropped:
            carb.log_info(
                f"[bridge] articulation {self.prim_path}: restored the authored drive values "
                f"on no-longer-driven joint(s) {dropped}"
            )

        self._target = None
        carb.log_info(
            f"[bridge] articulation {self.prim_path} drives "
            f"{self.dof_names} at {self.joint_indices}"
        )

    def adopt_shared_articulation(self, articulation, joint_names):
        """Re-point this device at a shared (assembled) articulation.

        After ``assemble_robot`` the arm and gripper are one articulation; both
        devices hold this same handle. Each keeps driving only its own joints
        (``joint_names``), resolved into the merged DOF order. No rebind needed --
        the handle is already initialized; a later reconnect re-resolves by name.

        The drive setup :meth:`bind` performs is redone here, over this device's own
        joints. Assembly edits USD, which means stopping and replaying the timeline,
        and physics rebuilds itself from the stage across that: the gains, effort
        ceilings, velocity limits and drive targets the bridge wrote when it first
        bound are all gone by the time the merged articulation arrives. A robot whose
        asset authors no drive stiffness -- as an imported URDF's does not -- comes
        out of an assembly with damping and no position drive otherwise, ignoring
        every target and sagging under gravity, and its gripper stops closing.
        """
        self._articulation = articulation
        self.joint_names = list(joint_names)
        self._resolve_driven_joints()
        self._backfill_missing_drive_gains()
        self._correct_imported_prismatic_max_velocities()
        self._align_drive_to_current_pose()
        self._ensure_default_state_populated()
        self._target = None
        carb.log_info(
            f"[bridge] articulation {self.prim_path} on shared articulation: "
            f"drives {self.dof_names} at {self.joint_indices}"
        )

    def base_link_name(self):
        """Root/base link name of this device's underlying handle. Used by
        ``assemble_robot`` to auto-discover the gripper's mount link -- kept as a
        device method (rather than the caller reaching into ``self._articulation``
        directly) so the service layer never depends on what that handle is. See
        :func:`..core.robot_assembler.get_articulation_base_link_name`.
        """
        return get_articulation_base_link_name(self._articulation)

    def shared_info(self):
        """Total DOF count and full DOF-name list of the underlying handle --
        every DOF on the (possibly merged) rig, not just this device's driven
        subset. Used to describe the merged articulation after ``assemble_robot``,
        again so the caller never reaches into ``self._articulation`` directly.
        """
        return {
            "num_dof": self._articulation.num_dof,
            "dof_names": list(self._articulation.dof_names),
        }

    async def move_j(self, positions, indices=None, asynchronous=False):
        """Move ``positions`` (radians) onto the chosen joint ``indices`` via the drive.

        ``indices`` defaults to this device's driven subset (``joint_indices``), so
        a robot client sends all joint angles and a gripper client (narrowed to its
        driver) sends one -- neither needs to know the merged DOF order.

        ``asynchronous=True`` applies the action and returns immediately (the client
        polls ``get_joints_state`` / decides "done" itself). ``asynchronous=False`` blocks,
        awaiting ``next_update_async`` in a loop until the joints reach the target or
        stall, then returns the final status with ``reached`` telling the two apart.
        Each ``next_update_async`` steps one physics frame and yields Isaac Sim's
        loop, so other requests keep being served while a blocking move waits. The
        ``_move_lock`` serializes repeat commands to this one device.
        """
        target = np.asarray(positions, dtype=float)
        idx = list(self.joint_indices) if indices is None else list(indices)
        if target.shape != (len(idx),):
            raise ValueError(f"expected {len(idx)} joint positions, got {target.shape[0]}")
        self._validate_indices(idx)

        app = omni.kit.app.get_app()
        async with self._move_lock:
            self._target = target
            self._articulation.apply_action(
                ArticulationAction(joint_positions=target, joint_indices=idx)
            )

            if asynchronous:
                # Fire-and-forget: the action is queued; the client owns "done".
                return {"done": False, "reached": False, "applied": True, "target": target.tolist()}

            reached = False
            stalled_frames = 0
            max_error = float("inf")
            q = self._articulation.get_joint_positions()[idx]
            for _ in range(_MOTION_MAX_FRAMES):
                await app.next_update_async()
                q = self._articulation.get_joint_positions()[idx]
                max_error = float(np.max(np.abs(q - target)))
                if max_error < _REACH_TOLERANCE_RAD:
                    reached = True
                    break
                velocities = self._articulation.get_joint_velocities()
                max_speed = (
                    float(np.max(np.abs(velocities[idx]))) if velocities is not None else 0.0
                )
                stalled_frames = stalled_frames + 1 if max_speed < _SETTLED_VELOCITY_RAD_S else 0
                if stalled_frames >= _SETTLED_FRAMES:
                    break

        return {
            "done": True,
            "reached": reached,
            "max_error": max_error,
            "joint_positions": q.tolist(),
            "target": target.tolist(),
        }

    async def set_j(self, positions, indices=None):
        """Teleport the chosen joint ``indices`` directly to ``positions`` (radians).

        Unlike :meth:`move_j`, this does not drive the joints to the
        target over time: it writes the DOF state immediately, so the joints jump
        to ``positions`` in a single step. It also zeros those joints' velocities
        (so the teleport does not carry over any motion) and retargets the position
        drive to the same values, so the controller holds the new pose instead of
        pulling the joints back toward the previous target.

        ``indices`` defaults to this device's driven subset (``joint_indices``),
        matching :meth:`move_j`. The ``_move_lock`` serializes this
        against any in-flight move on the same device.
        """
        target = np.asarray(positions, dtype=float)
        idx = list(self.joint_indices) if indices is None else list(indices)
        if target.shape != (len(idx),):
            raise ValueError(f"expected {len(idx)} joint positions, got {target.shape[0]}")
        self._validate_indices(idx)

        async with self._move_lock:
            self._articulation.set_joint_positions(target, joint_indices=idx)
            self._articulation.set_joint_velocities(
                np.zeros(len(idx), dtype=float), joint_indices=idx
            )
            self._articulation.apply_action(
                ArticulationAction(joint_positions=target, joint_indices=idx)
            )
            self._target = target
            q = self._articulation.get_joint_positions()[idx]

        return {
            "done": True,
            "teleported": True,
            "joint_positions": q.tolist(),
            "target": target.tolist(),
        }

    def stream_joint_positions(self, positions, indices=None):
        """Retarget the position drive for high-rate streaming: issue the drive command
        and nothing else -- no wait for the joints to arrive, no completion read-back.

        Intended to be called repeatedly (e.g. from a WebSocket) with a target that
        moves a little each frame, so the joints follow the stream continuously. The
        joints are driven there rather than placed there, so they track the stream
        with the drive's own response: expect the measured pose to trail the
        commanded one slightly, and gravity sag or a steady-state offset to remain
        while the drive holds against a load. How closely the joints follow is a
        property of their drive gains, which :meth:`set_dof_gains` retunes.

        Because the drive target is the command, stopping the stream holds the last
        streamed pose.

        ``indices`` defaults to this device's driven subset (``joint_indices``),
        matching :meth:`set_j`. This is synchronous and lock-free: it performs no
        ``await``, so it runs atomically with respect to other coroutines on the
        loop; the ``_move_lock`` only needs to guard the awaiting :meth:`move_j`.
        """
        target = np.asarray(positions, dtype=float)
        idx = list(self.joint_indices) if indices is None else list(indices)
        if target.shape != (len(idx),):
            raise ValueError(f"expected {len(idx)} joint positions, got {target.shape[0]}")
        self._validate_indices(idx)

        self._articulation.apply_action(
            ArticulationAction(joint_positions=target, joint_indices=idx)
        )
        self._target = target

    def set_joint_velocities(self, velocities, indices=None):
        """Drive ``velocities`` (rad/s) onto the chosen joint ``indices``.

        The velocity counterpart to :meth:`set_joint_positions`, for driving joints
        at a target rate rather than to a pose (drive wheels, a spinning tool, a
        conveyor, or slewing any joint at a controlled speed). PhysX velocity drive
        holds the commanded speeds until the next command, so this is fire-and-forget:
        there is no reach/stall loop and nothing to await (a velocity-driven joint
        never "arrives"). ``{0, ...}`` stops the joints.

        ``indices`` defaults to this device's driven subset. The client owns any
        higher-level kinematics (e.g. a nav stack resolving a twist into per-wheel
        velocities) and sends the resolved values to their DOF indices -- the
        extension stays joint-space and device-agnostic, exactly like
        ``set_joint_positions``.
        """
        target = np.asarray(velocities, dtype=float)
        idx = list(self.joint_indices) if indices is None else list(indices)
        if target.shape != (len(idx),):
            raise ValueError(f"expected {len(idx)} joint velocities, got {target.shape[0]}")
        self._validate_indices(idx)

        self._articulation.apply_action(
            ArticulationAction(joint_velocities=target, joint_indices=idx)
        )
        return {"applied": True, "joint_velocities": target.tolist(), "indices": idx}

    def get_joints_state(self):
        """Snapshot of the driven joints' positions / velocities / torques (rad) +
        a timestamp. Reports only the driven columns (``joint_indices``), so a
        merged rig still presents just this device's joints."""
        positions = self._articulation.get_joint_positions()
        velocities = self._articulation.get_joint_velocities()
        efforts = self._articulation.get_measured_joint_efforts()
        return {
            "joint_positions": positions[self.joint_indices].tolist(),
            "joint_velocities": (
                velocities[self.joint_indices].tolist()
                if velocities is not None
                else [0.0] * self.num_dof
            ),
            "joint_efforts": (
                efforts[self.joint_indices].tolist()
                if efforts is not None
                else [0.0] * self.num_dof
            ),
            "timestamp": time.monotonic() - self._start_time,
        }

    def get_articulation_state(self):
        """Every per-frame quantity this device can report, in one snapshot (radians).

        Everything that changes as the simulation runs: the driven joints' state and
        efforts, the measured joint reaction forces, the last applied action, and the
        root link's world pose and velocity. The static description (DOF count and
        names, drive properties, stored reset pose, solver settings) is not repeated
        here -- :meth:`info` carries that, and it is returned once when the
        articulation is registered.

        Returns ``None`` when there is nothing to report, rather than raising: a
        handle invalidated by a timeline stop or a stage change reads back as no
        joint positions at all, and a caller sampling this every frame wants to skip
        such a frame and carry on.
        """
        if not self.handles_initialized() or self._articulation.get_joint_positions() is None:
            return None

        position, orientation = self._articulation.get_world_pose()
        return {
            **self.get_joints_state(),
            "applied_joint_efforts": self.get_applied_joint_efforts(),
            "measured_joint_forces": self.get_measured_joint_forces(),
            "applied_action": self.get_applied_action(),
            "world_pose": {
                "position": position.tolist(),
                "orientation": orientation.tolist(),
            },
            "world_velocity": self.get_world_velocity(),
        }

    def get_dof_limits(self):
        """``[lower, upper]`` radian limits for each driven joint, in ``get_joints_state``'s
        ``joint_positions`` order.

        prims.SingleArticulation exposes no ``get_dof_limits()``; we read it off the
        physics view, whose array carries a leading per-environment batch dimension
        (``(num_envs, num_dof, 2)``), so squeeze it to index by DOF directly. For a
        gripper narrowed to its driver this is one ``(open, closed)`` pair the client
        uses to map a fraction to an angle.
        """
        limits = self._articulation._articulation_view.get_dof_limits()
        if hasattr(limits, "ndim") and limits.ndim == 3:
            limits = limits[0]
        return [[float(limits[i][0]), float(limits[i][1])] for i in self.joint_indices]

    def info(self):
        """Static description of this device: DOF count/names, per-joint drive
        properties, stored reset pose, and the PhysX solver/collision settings
        that were baked in when the articulation was authored (or set via the
        setters below). A solver setting the articulation does not carry is
        reported as null, PhysX simulating it with its own default. The bind
        happens in the service's PUT /articulations.
        """
        return {
            "num_dof": self.num_dof,
            "dof_names": self.dof_names,
            "num_bodies": self.num_bodies(),
            "dof_properties": self.dof_properties(),
            "joints_default_state": self.get_joints_default_state(),
            "solver_position_iteration_count": self.get_solver_position_iteration_count(),
            "solver_velocity_iteration_count": self.get_solver_velocity_iteration_count(),
            "stabilization_threshold": self.get_stabilization_threshold(),
            "sleep_threshold": self.get_sleep_threshold(),
            "enabled_self_collisions": self.get_enabled_self_collisions(),
        }

    # -- extended introspection / physics tuning ---------------------------
    #
    # The rest of prims.SingleArticulation's surface: joint-level extras (efforts,
    # forces, default state, applied action), floating-base motion (gravity,
    # world/linear/angular velocity -- meaningless for a fixed-base arm, only
    # relevant once this bridge drives a mobile base or humanoid), and PhysX
    # solver tuning. Kept below the core move/state methods since these are
    # secondary to ordinary joint-space control.

    def handles_initialized(self):
        """Whether the underlying handle is currently valid, without a re-bind."""
        return bool(self._articulation is not None and self._articulation.handles_initialized)

    def num_bodies(self):
        """Number of rigid-body links in the underlying articulation (the whole
         rig, not just this device's driven subset -- gravity/velocity are body-
        -level properties with no notion of a driven subset)."""
        return self._articulation.num_bodies

    def dof_index(self, joint_name):
        """DOF index of ``joint_name`` within this device's driven subset (i.e. the
        order ``get_joints_state``'s ``joint_positions`` and friends use), not the
        underlying articulation's full DOF order. Raises ValueError if it isn't
        one of this device's driven joints."""
        if joint_name not in self.dof_names:
            raise ValueError(
                f"'{joint_name}' is not a driven joint of {self.prim_path}; "
                f"driven: {self.dof_names}"
            )
        return self.dof_names.index(joint_name)

    def dof_properties(self):
        """Per-driven-joint drive properties (type, hasLimits, lower/upper limit,
        driveMode, maxVelocity, maxEffort, stiffness, damping), in ``get_joints_state``
        order. See ``prims.SingleArticulation.dof_properties``."""
        props = self._articulation.dof_properties
        return [
            {
                "type": int(props["type"][i]),
                "has_limits": bool(props["hasLimits"][i]),
                "lower": float(props["lower"][i]),
                "upper": float(props["upper"][i]),
                "drive_mode": int(props["driveMode"][i]),
                "max_velocity": float(props["maxVelocity"][i]),
                "max_effort": float(props["maxEffort"][i]),
                "stiffness": float(props["stiffness"][i]),
                "damping": float(props["damping"][i]),
            }
            for i in self.joint_indices
        ]

    def set_dof_gains(self, stiffness=None, damping=None, max_effort=None, indices=None):
        """Set the position drive's gains and effort ceiling on the chosen joint
        ``indices`` (defaults to this device's driven subset).

        ``stiffness`` and ``damping`` are the drive's proportional and derivative
        gains; ``max_effort`` is the largest torque/force the drive may apply. Each
        may be a single value applied to every addressed joint, or one value per
        joint in ``indices`` order. Omitted quantities are left untouched.

        Use this to retune how a robot tracks a commanded pose without re-importing
        it, and to replace any value :meth:`bind` had to substitute because the
        drive reported none, with the figures the real robot's drives use. Higher
        stiffness tracks a commanded pose more closely at the cost of stiffer, less
        stable contacts; damping suppresses the resulting overshoot.

        The change applies to the running simulation only; it is not written back
        to the stage.
        """
        idx = list(self.joint_indices) if indices is None else list(indices)
        self._validate_indices(idx)

        def as_row(values, quantity):
            if values is None:
                return None
            values = np.asarray(values, dtype=float)
            try:
                row = np.broadcast_to(values, (len(idx),))
            except ValueError as exc:
                raise ValueError(
                    f"expected 1 or {len(idx)} value(s) for {quantity}, got {values.size}"
                ) from exc
            if np.any(row < 0.0):
                raise ValueError(f"{quantity} cannot be negative, got {row.tolist()}")
            return row.reshape(1, len(idx))

        stiffness_row = as_row(stiffness, "stiffness")
        damping_row = as_row(damping, "damping")
        effort_row = as_row(max_effort, "max_effort")

        # The physics view is the only place these are writable --
        # prims.SingleArticulation exposes no setter, the same gap get_dof_limits
        # works around. Its arrays carry a leading per-environment dimension.
        view = self._articulation._articulation_view
        if stiffness_row is not None or damping_row is not None:
            view.set_gains(kps=stiffness_row, kds=damping_row, joint_indices=idx)
        if effort_row is not None:
            view.set_max_efforts(effort_row, joint_indices=idx)

        return {"applied": True, "indices": idx, "dof_properties": self.dof_properties()}

    def get_applied_joint_efforts(self):
        """Efforts last commanded via :meth:`set_joint_efforts` on the driven
        joints (get_joints_state order) -- what was asked for, not what was measured."""
        efforts = self._articulation.get_applied_joint_efforts()
        return efforts[self.joint_indices].tolist()

    def get_measured_joint_forces(self):
        """Measured 6-axis joint reaction force/torque ``[fx, fy, fz, tx, ty, tz]``
        per driven joint, in ``get_joints_state`` order.

        Isaac reports one row per articulation body (row 0 is the base link's
        incoming "joint"; every other row is offset by 1 from its DOF index), so
        this looks up each driven joint's row via the view's own name->index
        metadata rather than assuming row order matches ``dof_names`` order --
        same private-attribute workaround as :meth:`get_dof_limits`, for the
        same reason (no public accessor for this mapping yet).
        """
        forces = self._articulation.get_measured_joint_forces()
        joint_indices = self._articulation._articulation_view._metadata.joint_indices
        try:
            return [forces[1 + joint_indices[name]].tolist() for name in self.dof_names]
        except KeyError as exc:
            raise RuntimeError(
                f"joint {exc} on articulation at {self.prim_path} has no entry in the "
                f"underlying articulation view's metadata (possible inconsistency after assembly)"
            ) from exc

    def get_joints_default_state(self):
        """Stored joint-space home pose (positions + velocities) for the driven
        joints, in ``get_joints_state`` order. ``None`` per field if never set --
        NOT zeros, which would be indistinguishable from a default explicitly
        set to zero."""
        state = self._articulation.get_joints_default_state()
        if state is None:
            return {"joint_positions": None, "joint_velocities": None}
        return {
            "joint_positions": np.asarray(state.positions)[self.joint_indices].tolist(),
            "joint_velocities": np.asarray(state.velocities)[self.joint_indices].tolist(),
        }

    def set_joints_default_state(
        self, joint_positions=None, joint_velocities=None, joint_efforts=None
    ):
        """Set the driven joints' stored home pose, applied on the next
        ``post_reset()`` / Stop+Play. Any field left ``None`` is not touched.

        The underlying API sets defaults for the WHOLE articulation at once, not
        just a subset, so this reads the current full-articulation defaults first
        and overwrites only the driven joints' columns -- otherwise an arm's call
        would clobber a gripper's stored defaults on a shared (assembled) rig.
        """
        num_all = len(self._articulation.dof_names)
        current = self._articulation.get_joints_default_state()

        def _merged(values, current_values):
            if values is None:
                return None
            target = np.asarray(values, dtype=float)
            if target.shape != (len(self.joint_indices),):
                raise ValueError(
                    f"expected {len(self.joint_indices)} values, got {target.shape[0]}"
                )
            full = (
                np.array(current_values, dtype=float)
                if current_values is not None
                else np.zeros(num_all)
            )
            full[self.joint_indices] = target
            return full

        positions = _merged(joint_positions, current.positions if current is not None else None)
        velocities = _merged(joint_velocities, current.velocities if current is not None else None)
        efforts = _merged(joint_efforts, None)  # Isaac never reports a stored default effort back
        self._articulation.set_joints_default_state(
            positions=positions, velocities=velocities, efforts=efforts
        )

    def get_applied_action(self):
        """Last ``ArticulationAction`` PhysX actually received for the WHOLE
        articulation (every DOF, not just this device's driven subset -- reflects
        whatever any device sharing this articulation last commanded). ``None``
        for every field if nothing has been applied yet -- not an error, just the
        normal starting state of a freshly bound device."""
        action = self._articulation.get_applied_action()
        if action is None:
            return {"joint_positions": None, "joint_velocities": None, "joint_efforts": None}
        return {
            "joint_positions": action.joint_positions.tolist()
            if action.joint_positions is not None
            else None,
            "joint_velocities": action.joint_velocities.tolist()
            if action.joint_velocities is not None
            else None,
            "joint_efforts": action.joint_efforts.tolist()
            if action.joint_efforts is not None
            else None,
        }

    def set_joint_efforts(self, efforts, indices=None):
        """Command raw torque/force directly on the chosen joint ``indices``
        (defaults to this device's driven subset), bypassing the position/
        velocity drive entirely. Only takes effect if that joint's drive
        stiffness and damping are zero (or it has no drive at all) -- see
        ``prims.SingleArticulation.set_joint_efforts``.
        """
        target = np.asarray(efforts, dtype=float)
        idx = list(self.joint_indices) if indices is None else list(indices)
        if target.shape != (len(idx),):
            raise ValueError(f"expected {len(idx)} joint efforts, got {target.shape[0]}")
        self._validate_indices(idx)
        self._articulation.set_joint_efforts(target, joint_indices=idx)
        return {"applied": True, "joint_efforts": target.tolist(), "indices": idx}

    def enable_gravity(self):
        """Gravity affects every body in this articulation. Gravity is a body-level
        property (no per-joint notion), so this always affects the whole rig, not
        just this device's driven subset. No getter exists on
        prims.SingleArticulation for the current state, so only enable/disable
        are exposed -- matching prims.SingleArticulation's own two-method shape
        rather than a single boolean-flag setter.
        """
        self._articulation.enable_gravity()

    def disable_gravity(self):
        """Gravity no longer affects any body in this articulation. See
        :meth:`enable_gravity`."""
        self._articulation.disable_gravity()

    def get_world_velocity(self):
        """Root link's full 6-DOF world-space velocity ``[vx, vy, vz, wx, wy, wz]``.
        Only meaningful for a floating-base articulation (mobile base, humanoid);
        always zero for a fixed-base arm bolted to the world."""
        velocity = self._articulation.get_world_velocity()
        return velocity.tolist() if velocity is not None else [0.0] * 6

    def set_world_velocity(self, velocity):
        """Set the root link's full 6-DOF world-space velocity. See
        :meth:`get_world_velocity`."""
        target = np.asarray(velocity, dtype=float)
        if target.shape != (6,):
            raise ValueError(f"expected 6 values (linear xyz + angular xyz), got {target.shape[0]}")
        self._articulation.set_world_velocity(target)

    def get_linear_velocity(self):
        """Root link's linear (translational) velocity ``[vx, vy, vz]``."""
        velocity = self._articulation.get_linear_velocity()
        return velocity.tolist() if velocity is not None else [0.0, 0.0, 0.0]

    def set_linear_velocity(self, velocity):
        """Set the root link's linear velocity only (leaves angular untouched)."""
        target = np.asarray(velocity, dtype=float)
        if target.shape != (3,):
            raise ValueError(f"expected 3 values (x, y, z), got {target.shape[0]}")
        self._articulation.set_linear_velocity(target)

    def get_angular_velocity(self):
        """Root link's angular (rotational) velocity ``[wx, wy, wz]``."""
        velocity = self._articulation.get_angular_velocity()
        return velocity.tolist() if velocity is not None else [0.0, 0.0, 0.0]

    def set_angular_velocity(self, velocity):
        """Set the root link's angular velocity only (leaves linear untouched)."""
        target = np.asarray(velocity, dtype=float)
        if target.shape != (3,):
            raise ValueError(f"expected 3 values (x, y, z), got {target.shape[0]}")
        self._articulation.set_angular_velocity(target)

    def get_solver_position_iteration_count(self):
        """PhysX position-solver iteration count for this articulation, ``None`` when the
        articulation does not carry one."""
        count = self._physx_setting("solverPositionIterationCount")
        return None if count is None else int(count)

    def set_solver_position_iteration_count(self, count):
        """Set the PhysX position-solver iteration count (accuracy vs. perf)."""
        if count < 0:
            raise ValueError(f"solver position iteration count must be >= 0, got {count}")
        self._set_physx_setting("solverPositionIterationCount", int(count))

    def get_solver_velocity_iteration_count(self):
        """PhysX velocity-solver iteration count for this articulation, ``None`` when the
        articulation does not carry one."""
        count = self._physx_setting("solverVelocityIterationCount")
        return None if count is None else int(count)

    def set_solver_velocity_iteration_count(self, count):
        """Set the PhysX velocity-solver iteration count (accuracy vs. perf)."""
        if count < 0:
            raise ValueError(f"solver velocity iteration count must be >= 0, got {count}")
        self._set_physx_setting("solverVelocityIterationCount", int(count))

    def get_stabilization_threshold(self):
        """Mass-normalized kinetic energy below which PhysX may stabilize this
        articulation (settle small residual jitter), ``None`` when the articulation does
        not carry a threshold."""
        threshold = self._physx_setting("stabilizationThreshold")
        return None if threshold is None else float(threshold)

    def set_stabilization_threshold(self, threshold):
        """Set the stabilization threshold. See :meth:`get_stabilization_threshold`."""
        if threshold < 0:
            raise ValueError(f"stabilization threshold must be >= 0, got {threshold}")
        self._set_physx_setting("stabilizationThreshold", float(threshold))

    def get_enabled_self_collisions(self):
        """Whether this articulation's own links can collide with each other, ``None``
        when the articulation does not carry the flag."""
        enabled = self._physx_setting("enabledSelfCollisions")
        return None if enabled is None else bool(enabled)

    def set_enabled_self_collisions(self, enabled):
        """Enable/disable self-collision between this articulation's own links."""
        self._set_physx_setting("enabledSelfCollisions", bool(enabled))

    def get_sleep_threshold(self):
        """Velocity threshold below which PhysX lets this articulation sleep
        (skip simulation) to save performance, ``None`` when the articulation does not
        carry a threshold."""
        threshold = self._physx_setting("sleepThreshold")
        return None if threshold is None else float(threshold)

    def set_sleep_threshold(self, threshold):
        """Set the sleep threshold. See :meth:`get_sleep_threshold`."""
        if threshold < 0:
            raise ValueError(f"sleep threshold must be >= 0, got {threshold}")
        self._set_physx_setting("sleepThreshold", float(threshold))

    def _physx_setting(self, name):
        """Value of PhysX's ``name`` setting on this articulation, or ``None`` when the
        articulation does not carry it.

        These settings are optional: an articulation that leaves one out is simulated
        with PhysX's own default for it and has no value to report. Read off the prim
        PhysX resolved as the articulation's root -- which is not necessarily the prim
        path this device was registered at -- rather than through the articulation
        handle, whose reader turns an absent setting into a type error or a NaN.
        """
        prim = self._physx_articulation_prim()
        attribute = prim.GetAttribute(f"physxArticulation:{name}")
        return attribute.Get() if attribute else None

    def _set_physx_setting(self, name, value):
        """Write PhysX's ``name`` setting on this articulation.

        An articulation carrying none of these settings has no attribute to write to, so
        they are added to it first, every one of them at the default PhysX was already
        simulating it with; only ``name`` is then given the requested value.
        """
        prim = self._physx_articulation_prim()
        if not prim.GetAttribute(f"physxArticulation:{name}"):
            PhysxSchema.PhysxArticulationAPI.Apply(prim)
        prim.GetAttribute(f"physxArticulation:{name}").Set(value)

    def _physx_articulation_prim(self):
        """Prim PhysX simulates as this articulation's root, which is where its solver
        settings live. ``prim_path`` is the handle's resolved root, not the path the
        device was registered at (a container prim holding the articulation below it).
        """
        return self._articulation.prim.GetStage().GetPrimAtPath(self._articulation.prim_path)
