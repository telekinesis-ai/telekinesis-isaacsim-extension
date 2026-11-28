# SPDX-License-Identifier: Apache-2.0
"""Move to: drive a pose's robot to the pose in Isaac Sim, along a motion the Telekinesis Isaac
Sim Server plans."""

import asyncio
import dataclasses
import enum

import carb
import numpy as np
import omni.kit.app
import omni.timeline
from fastapi import HTTPException

from .articulations import ArticulationService
from .poses import PoseService

# Frames to wait, after the simulation starts, for the robot to report its joints: the physics
# handles are bound a few frames after the timeline plays.
ROBOT_READY_FRAMES = 60


class MotionCommand(enum.Enum):
    """How Move to brings the robot to the pose; the value is the name the tool bar shows.

    Attributes:
        JOINT_P2P: All joints move together from where they are to the pose's joints (see
            :meth:`PoseService.plan_joint_trajectory_to_pose`); the TCP's path is not a line.
        LINE: The TCP moves along a straight line to the pose (see
            :meth:`PoseService.plan_line_to_pose`).
    """

    JOINT_P2P = "Joint P2P"
    LINE = "Line"


@dataclasses.dataclass
class PlannedMotion:
    """A motion planned for a robot's articulation, played by :meth:`MoveToService.execute`.

    Attributes:
        articulation_id: Id of the robot's articulation.
        joint_indices: Articulation joint index of each planned joint, as
            ``stream_joint_positions`` takes them.
        state_indices: Position of each planned joint in the articulation's joint state.
        times: Sample times in seconds from the start of the motion.
        joint_positions: Planned joint positions in radians, one row per sample time.
    """

    articulation_id: str
    joint_indices: list[int]
    state_indices: list[int]
    times: np.ndarray
    joint_positions: np.ndarray


class MoveToService:
    """Moves a pose's robot to the pose in Isaac Sim, with all joints moving together (Joint
    P2P) or with the TCP along a straight line (Line).

    :meth:`plan_move_to_pose` has the Telekinesis Isaac Sim Server plan the motion from the
    robot's current joints to the pose, starting the simulation first when it is not playing,
    so the motion starts from where the robot really is (see :class:`MotionCommand`). Either
    motion is planned as joint positions over time. :meth:`execute` plays the motion by
    simulation time: every frame, the robot's joints are driven to the planned joints at the
    simulation time since the motion began, interpolated between the planned samples, and then
    held at the last ones until the robot reaches them or stops moving, where it is then held.
    Only the planned joints are driven, so a gripper merged into the robot is left alone, and a
    paused simulation pauses the motion. Nothing checks the motion for collisions.

    Cancelling a running :meth:`execute` stops the robot where it is: its joints are driven to
    the positions they measure at that moment rather than to the last planned ones. Cancelling
    :meth:`plan_move_to_pose` never moves the robot.

    Example:
        service = MoveToService(pose_service, articulation_service)
        motion = await service.plan_move_to_pose(
            "/World/UR10e_assembly/poses/Pose", MotionCommand.JOINT_P2P, 0.7
        )
        await service.execute(motion)
    """

    def __init__(self, pose_service: PoseService, articulation_service: ArticulationService):
        self._pose_service = pose_service
        self._articulation_service = articulation_service

    async def plan_move_to_pose(
        self, pose_path: str, motion_command: MotionCommand, speed: float | None = None
    ) -> PlannedMotion:
        """Plan the motion of the robot of the pose at ``pose_path`` from its current joints to
        the pose, starting the simulation first when it is not playing.

        Args:
            pose_path: The pose prim.
            motion_command: How the robot moves to the pose.
            speed: For Joint P2P, the speed in rad/s of the joint that moves furthest; for Line,
                the TCP's speed along the line in m/s. ``None`` for the server robot's default.

        Raises:
            HTTPException: 422 if the pose cannot be planned for (see
                :meth:`PoseService.plan_joint_trajectory_to_pose` and
                :meth:`PoseService.plan_line_to_pose`), its robot has no articulation, or the
                plan names joints the robot does not have; 503 if the server cannot be reached
                or the robot does not report its joints after the simulation starts.
        """
        robot_prim_path = self._pose_service.robot_of_pose(pose_path)
        articulation_id = await self._ready_articulation(robot_prim_path)
        if motion_command is MotionCommand.LINE:
            plan = await self._pose_service.plan_line_to_pose(pose_path, speed)
        else:
            plan = await self._pose_service.plan_joint_trajectory_to_pose(pose_path, speed)
        device = self._articulation_service.get_device(articulation_id)
        missing_joints = [name for name in plan["joint_names"] if name not in device.dof_names]
        if missing_joints:
            raise HTTPException(
                422, f"The robot at {robot_prim_path} has no joints {missing_joints}."
            )
        # The articulation's joint indices differ from the robot's joint order when a gripper is
        # merged into it.
        state_indices = [device.dof_names.index(name) for name in plan["joint_names"]]
        return PlannedMotion(
            articulation_id=articulation_id,
            joint_indices=[device.joint_indices[index] for index in state_indices],
            state_indices=state_indices,
            times=np.asarray(plan["times"], dtype=float),
            joint_positions=np.asarray(plan["joint_positions"], dtype=float),
        )

    async def execute(self, motion: PlannedMotion):
        """Drive the robot along ``motion`` by simulation time, every frame, then hold the last
        sample's joints and return once the robot reaches them.

        The robot has reached them when every joint is within the bridge's arrival tolerance of
        its target, 0.01 rad for a revolute joint. A robot that stops moving short of them, e.g.
        because something blocks it, is held where it stopped, and a warning in the log says how
        far it is from them.

        Raises:
            HTTPException: 422 if the simulation stops during the motion.
        """
        from isaacsim.core.simulation_manager import SimulationManager

        app = omni.kit.app.get_app()
        timeline = omni.timeline.get_timeline_interface()
        start_time = SimulationManager.get_simulation_time()
        try:
            while True:
                if timeline.is_stopped():
                    raise HTTPException(422, "The simulation stopped during the motion.")
                elapsed = SimulationManager.get_simulation_time() - start_time
                target = [
                    np.interp(elapsed, motion.times, motion.joint_positions[:, joint])
                    for joint in range(motion.joint_positions.shape[1])
                ]
                self._articulation_service.stream_joint_positions(
                    motion.articulation_id, target, motion.joint_indices
                )
                if elapsed >= motion.times[-1]:
                    break
                await app.next_update_async()
            # The joint drives trail the streamed targets, so the robot is still moving when the
            # last one is sent.
            arrival = await self._articulation_service.move_j(
                motion.articulation_id,
                motion.joint_positions[-1],
                motion.joint_indices,
                asynchronous=False,
            )
            if not arrival["reached"]:
                self._hold_measured_joints(motion)
                carb.log_warn(
                    f"[bridge] Move to: the robot stopped {arrival['max_error']:.3f} short of the "
                    "pose (radians, or metres for a sliding joint), e.g. because something "
                    "blocks it."
                )
        except asyncio.CancelledError:
            self._hold_measured_joints(motion)
            raise

    def _hold_measured_joints(self, motion: PlannedMotion):
        """Drive the joints of ``motion`` to the positions they measure now, so the robot stays
        where it is instead of moving on toward the motion's target later, e.g. once whatever
        blocked it is gone. Does nothing while the robot reports no joints."""
        state = self._articulation_service.get_articulation_state(motion.articulation_id)
        if state is not None:
            measured = np.asarray(state["joint_positions"], dtype=float)[motion.state_indices]
            self._articulation_service.stream_joint_positions(
                motion.articulation_id, measured, motion.joint_indices
            )

    async def _ready_articulation(self, robot_prim_path: str) -> str:
        """Start the simulation when it is not playing, and return the id of the robot's
        articulation once the robot reports its joints.

        A robot whose physics handle a stopped simulation invalidated is bound again, whether
        Move to or the user played the simulation since.

        Raises:
            HTTPException: 422 if the robot has no articulation; 503 if it does not report its
                joints within :data:`ROBOT_READY_FRAMES` frames.
        """
        articulation_id = await self._articulation_service.find_robot_for_prim(robot_prim_path)
        if articulation_id is None:
            raise HTTPException(422, f"{robot_prim_path} is not a robot the bridge can drive.")
        app = omni.kit.app.get_app()
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            timeline.play()
            await app.next_update_async()
            await app.next_update_async()
        if self._articulation_service.get_articulation_state(articulation_id) is None:
            # A stopped timeline invalidates the robots' physics handles, and playing it again
            # does not bind them.
            await self._articulation_service.rebind_stale_devices()
        for _ in range(ROBOT_READY_FRAMES):
            if self._articulation_service.get_articulation_state(articulation_id) is not None:
                return articulation_id
            await app.next_update_async()
        raise HTTPException(
            503, f"{robot_prim_path} did not report its joints after the simulation started."
        )
