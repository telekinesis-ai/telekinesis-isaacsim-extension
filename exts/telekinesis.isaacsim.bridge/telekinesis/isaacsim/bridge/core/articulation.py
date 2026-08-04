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
from pxr import Usd, UsdPhysics

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

    root = stage.GetPrimAtPath(prim_path)
    if root.IsA(UsdPhysics.Joint):  # importer may return the root joint, not the container
        root = root.GetParent()
    joints = {p.GetName(): p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)}

    fallback = None
    for name in dof_names:
        prim = joints.get(name)
        if prim is None or any("MimicJoint" in s for s in prim.GetAppliedSchemas()):
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

    async def bind(self):
        """(Re)initialize the articulation against the current physics view.

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
                    self._resolve_driven_joints()
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
        """
        self.joint_names = list(joint_names)
        self._resolve_driven_joints()
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
        """
        self._articulation = articulation
        self.joint_names = list(joint_names)
        self._resolve_driven_joints()
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
        """Teleport joints for high-rate streaming: write the DOF state directly, with
        no drive command, no velocity zeroing, and no completion read-back.

        Intended to be called repeatedly (e.g. from a WebSocket): the direct state
        write is the whole operation, so consecutive updates flow smoothly and
        cheaply. Because it issues no position-drive command, keep streaming to hold
        a pose -- if updates stop, the position drive settles toward its last
        commanded target.

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

        self._articulation.set_joint_positions(target, joint_indices=idx)
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
        setters below). The bind happens in the service's PUT /articulations.
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
        """PhysX position-solver iteration count for this articulation."""
        return int(self._articulation.get_solver_position_iteration_count())

    def set_solver_position_iteration_count(self, count):
        """Set the PhysX position-solver iteration count (accuracy vs. perf)."""
        if count < 0:
            raise ValueError(f"solver position iteration count must be >= 0, got {count}")
        self._articulation.set_solver_position_iteration_count(int(count))

    def get_solver_velocity_iteration_count(self):
        """PhysX velocity-solver iteration count for this articulation."""
        return int(self._articulation.get_solver_velocity_iteration_count())

    def set_solver_velocity_iteration_count(self, count):
        """Set the PhysX velocity-solver iteration count (accuracy vs. perf)."""
        if count < 0:
            raise ValueError(f"solver velocity iteration count must be >= 0, got {count}")
        self._articulation.set_solver_velocity_iteration_count(int(count))

    def get_stabilization_threshold(self):
        """Mass-normalized kinetic energy below which PhysX may stabilize this
        articulation (settle small residual jitter)."""
        return float(self._articulation.get_stabilization_threshold())

    def set_stabilization_threshold(self, threshold):
        """Set the stabilization threshold. See :meth:`get_stabilization_threshold`."""
        if threshold < 0:
            raise ValueError(f"stabilization threshold must be >= 0, got {threshold}")
        self._articulation.set_stabilization_threshold(float(threshold))

    def get_enabled_self_collisions(self):
        """Whether this articulation's own links can collide with each other."""
        return bool(self._articulation.get_enabled_self_collisions())

    def set_enabled_self_collisions(self, enabled):
        """Enable/disable self-collision between this articulation's own links."""
        self._articulation.set_enabled_self_collisions(bool(enabled))

    def get_sleep_threshold(self):
        """Velocity threshold below which PhysX lets this articulation sleep
        (skip simulation) to save performance."""
        return float(self._articulation.get_sleep_threshold())

    def set_sleep_threshold(self, threshold):
        """Set the sleep threshold. See :meth:`get_sleep_threshold`."""
        if threshold < 0:
            raise ValueError(f"sleep threshold must be >= 0, got {threshold}")
        self._articulation.set_sleep_threshold(float(threshold))
