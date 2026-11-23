# SPDX-License-Identifier: Apache-2.0
"""Poses a robot is taught, shown as ghosts: see-through copies of the robot's tool that mark
where the tool should go."""

import asyncio
import math
from collections.abc import Callable

import carb
import numpy as np
import omni.usd
from fastapi import HTTPException
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade

from ...core import asset_loader, meshes, robot_assembler
from .. import models
from ..isaacsim_server_client import IsaacSimServerClient, IsaacSimServerError
from .articulations import ROBOT_CLASS_NAME_ATTRIBUTE, ROBOT_ID_ATTRIBUTE, ArticulationService

# Properties of a pose prim. The relationship targets the robot prim whose default TCP
# the pose places; the joint attributes hold the robot's inverse-kinematics solution for the
# pose, in radians; ``reachable`` says whether there is one.
POSE_ROBOT_RELATIONSHIP = "telekinesis:pose:robot"
POSE_JOINT_NAMES_ATTRIBUTE = "telekinesis:pose:joint_names"
POSE_JOINT_POSITIONS_ATTRIBUTE = "telekinesis:pose:joint_positions"
POSE_REACHABLE_ATTRIBUTE = "telekinesis:pose:reachable"

# Name of the Xform beside the robot that holds its poses, and the suffix of a new
# pose's name after its tool's: the default pose path is <tool parent>/poses/<tool>_Pose.
POSES_GROUP_NAME = "poses"
POSE_NAME_SUFFIX = "_Pose"
# Look of a pose's ghost: reachability tints on Kit's SimPBR_Translucent material. It is in
# Kit's core MDL library, so a saved stage finds it by name on any Isaac Sim install.
REACHABLE_COLOR = Gf.Vec3f(0.15, 0.8, 0.25)
UNREACHABLE_COLOR = Gf.Vec3f(0.9, 0.15, 0.15)
GHOST_MDL = "SimPBR_Translucent.mdl"
# A moved pose is solved again only when it moved more than this since it was last solved,
# as PrimPoseWatcher does: metres of translation and radians of XYZ Euler rotation.
MIN_TRANSLATION_CHANGE = 0.001
MIN_ROTATION_CHANGE = 0.01


class PoseService:
    """Creates the poses a robot is taught, and solves them again when they are moved.

    A pose is shown as a ghost of the tool. Add Pose ▸ From Tool creates one from a tool mounted on
    a robot, where the tool stands now.

    A pose is a single ``Mesh`` prim: the tool's visible meshes merged into the frame of
    the robot's default TCP, so the pose's origin is where that TCP has to go for the real tool
    to cover the pose. Poses live in a ``poses`` Xform beside the robot, in its assembly when
    it has one, carry no physics, and are saved with the stage. Each pose stores the robot's
    inverse-kinematics solution for it, solved by the Telekinesis Isaac Sim Server, and is tinted
    green when the robot can reach it and red when it cannot.

    While the service follows a stage, a pose that is moved, directly or with one of its parent
    prims, is solved again once it has moved more than :data:`MIN_TRANSLATION_CHANGE` or
    :data:`MIN_ROTATION_CHANGE` since it was last solved; so is a pose that appears, such as a
    duplicate. One solve runs per pose at a time, and a move during a solve leads to one more
    solve, so the stored solution always ends at the pose's latest placement. A pose moved out of
    reach keeps its last solution, which seeds the next solve. A solve that fails, e.g. because
    the server is not running, logs a warning and leaves the pose as it was.

    The robots are loaded on the Telekinesis Isaac Sim Server from the Synapse class name each
    robot prim stores, which Add Robot writes. A robot is loaded the first time one of its poses
    needs it, and again for every robot of the stage by :meth:`connect_stage_robots`; the server's
    id is stored on the robot prim.

    Example:
        service = PoseService(bridge_server.articulation_service, IsaacSimServerClient())
        pose = await service.create_pose_from_tool(
            "/World/vention_table/robot_mount/UR10e_assembly/UR10e"
        )
        pose["reachable"]  # True
    """

    def __init__(
        self, articulation_service: ArticulationService, server_client: IsaacSimServerClient
    ):
        self._articulation_service = articulation_service
        self._server_client = server_client
        # Isaac Sim's loop. A stage notice arrives on the thread that made the change, which
        # need not be this one.
        self._event_loop = asyncio.get_event_loop()
        # Server descriptions of the robots, by robot id. Ids are never reused, so an entry
        # never goes stale.
        self._robot_descriptions: dict[str, dict] = {}
        # The stage whose poses are followed and its change listener; see follow_stage.
        self._stage: Usd.Stage | None = None
        self._objects_changed_listener = None
        # Pose prims of the followed stage, and the pose each was last solved at, in its
        # robot's base frame.
        self._pose_paths: set[Sdf.Path] = set()
        self._solved_poses: dict[Sdf.Path, list[float]] = {}
        # The solve running for each pose, and the poses moved while theirs ran.
        self._solve_tasks: dict[Sdf.Path, asyncio.Task] = {}
        self._moved_while_solving: set[Sdf.Path] = set()
        # Serializes loading robots on the server, so concurrent solves of a robot that is not
        # loaded yet load it once.
        self._connect_lock = asyncio.Lock()
        # Called on Isaac Sim's loop with a pose's path whenever its solution was written, so
        # the robot overlay can follow the selected pose.
        self.on_pose_solved: Callable[[Sdf.Path], None] | None = None
        self.follow_stage(omni.usd.get_context().get_stage())

    def follow_stage(self, stage: Usd.Stage | None):
        """Solve the poses of ``stage`` again whenever they are moved; with ``None``,
        stop following any stage.

        Called when the extension starts, with the open stage, and whenever a stage is opened or
        closed. Solves still running for the previously followed stage are cancelled.
        """
        if self._objects_changed_listener is not None:
            self._objects_changed_listener.Revoke()
            self._objects_changed_listener = None
        for task in self._solve_tasks.values():
            task.cancel()
        self._solve_tasks.clear()
        self._moved_while_solving.clear()
        self._solved_poses.clear()
        self._stage = stage
        self._pose_paths = set() if stage is None else _find_poses(stage.GetPseudoRoot())
        if stage is not None:
            self._objects_changed_listener = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._on_objects_changed, stage
            )

    async def connect_stage_robots(self) -> dict[str, str]:
        """Load every robot of the followed stage on the server under a new id.

        A robot is a prim storing a Synapse class name, which Add Robot writes. Each robot's new
        id and exact class name are stored on its prim, and the server robot of the id it stored
        before is deleted. Called when the user connects to the server, which also repairs ids a
        restarted server no longer knows.

        Returns:
            The class name of each loaded robot, by prim path.

        Raises:
            HTTPException: 422 if the server does not know a robot's class name or answers with
                another error; 503 if the server cannot be reached.
        """
        stage = self._stage
        if stage is None:
            return {}
        robot_paths = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.GetAttribute(ROBOT_CLASS_NAME_ATTRIBUTE).Get()
        ]
        class_names = {}
        async with self._connect_lock:
            for robot_prim_path in robot_paths:
                await self._load_robot_on_server(stage, robot_prim_path)
                class_names[robot_prim_path] = (
                    stage.GetPrimAtPath(robot_prim_path)
                    .GetAttribute(ROBOT_CLASS_NAME_ATTRIBUTE)
                    .Get()
                )
        return class_names

    async def create_pose_from_tool(self, robot_part_prim_path: str) -> dict:
        """Create a pose from the tool mounted on a robot: a pose of the whole tool,
        where the tool stands now.

        The robot must be added with Add Robot and carry a tool added with Add Tool. The robot's
        inverse kinematics for the pose is solved before anything is written, so a failure
        leaves the stage unchanged. It is seeded with the robot's current joints, so the
        solution keeps the robot's current arm configuration.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.

        Returns:
            ``{"prim_path": ..., "reachable": ...}`` of the new pose.

        Raises:
            HTTPException: 422 if the stage is not in metres, the prim is part of no robot, the
                robot has no tool, the robot stores no class name or is no longer known to the
                server, the robot does not match its class, or the tool has no meshes; 503 if
                the server cannot be reached.
        """
        stage = omni.usd.get_context().get_stage()
        if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
            # The bridge loads robots at metre scale whatever the stage unit, so a pose read in
            # another unit would not match the robot.
            raise HTTPException(422, "Poses need a stage in metres (metersPerUnit = 1).")

        articulation_id = await self._articulation_service.find_robot_for_prim(robot_part_prim_path)
        if articulation_id is None:
            raise HTTPException(
                422,
                f"'{robot_part_prim_path}' is part of no robot. Right-click a robot added with "
                "Add Robot, or its tool.",
            )
        robot_prim_path = self._articulation_service.get_device(articulation_id).prim_path
        tool_prim_path = self._articulation_service.find_tool_on_robot(articulation_id)
        if tool_prim_path is None:
            raise HTTPException(
                422, f"'{robot_prim_path}' has no tool. Attach one with Add Tool first."
            )
        robot_id = await self._connected_robot_id(stage, robot_prim_path)
        robot = await self._describe_robot(robot_id)

        default_tcp_world = _world_transform_of_robot_frame(
            stage, robot_prim_path, robot["default_tcp"], robot["class_name"]
        )
        base_world = _world_transform_of_robot_frame(
            stage, robot_prim_path, robot["base_frame"], robot["class_name"]
        )
        points, face_vertex_counts, face_vertex_indices = meshes.merge_meshes_in_frame(
            stage, tool_prim_path, default_tcp_world
        )
        if not face_vertex_counts:
            raise HTTPException(
                422, f"'{tool_prim_path}' has no visible meshes to show as a ghost."
            )

        pose = _pose_in_frame(default_tcp_world, base_world)
        solution = await _call_server(
            self._server_client.inverse_kinematics(
                robot_id, [pose], self._current_joints(stage, articulation_id, robot_prim_path)
            )
        )

        poses_path = asset_loader.path_beside_clear_of_rigid_bodies(
            stage, robot_prim_path, POSES_GROUP_NAME
        )
        UsdGeom.Xform.Define(stage, poses_path)
        pose_name = Tf.MakeValidIdentifier(Sdf.Path(tool_prim_path).name) + POSE_NAME_SUFFIX
        pose_path = omni.usd.get_stage_next_free_path(stage, f"{poses_path}/{pose_name}", False)
        pose_mesh = UsdGeom.Mesh.Define(stage, pose_path)
        pose_mesh.CreatePointsAttr(points)
        pose_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        pose_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
        local_transform = Gf.Transform(
            default_tcp_world
            * omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(poses_path)).GetInverse()
        )
        pose_mesh.AddTranslateOp().Set(local_transform.GetTranslation())
        pose_mesh.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(local_transform.GetRotation().GetQuat())
        )
        pose_mesh.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(local_transform.GetScale())

        pose_prim = pose_mesh.GetPrim()
        pose_prim.CreateRelationship(POSE_ROBOT_RELATIONSHIP).SetTargets([robot_prim_path])
        reachable = _write_pose_solution(
            stage, pose_prim, solution["joint_names"], solution["joint_positions"][0]
        )
        # Recorded before the creation's own change notice is handled, so that notice does not
        # solve the new pose a second time.
        self._solved_poses[pose_prim.GetPath()] = pose
        if self.on_pose_solved is not None:
            self.on_pose_solved(pose_prim.GetPath())
        return {"prim_path": pose_path, "reachable": reachable}

    async def list_poses(self, robot_prim_path: str | None = None) -> list[dict]:
        """Return the poses of the open stage, sorted by prim path.

        Each entry is ``{"name", "prim_path", "robot_prim_path", "pose", "joint_names",
        "joint_positions", "reachable"}``: ``pose`` is the pose's ``[x, y, z, rx, ry, rz]`` in
        its robot's base frame, metres and extrinsic XYZ Euler degrees, the pose a program moves
        the robot's default TCP to; ``joint_positions`` is the stored solution in radians.

        Args:
            robot_prim_path: Only the poses of this robot, or every pose when ``None``.

        Raises:
            HTTPException: 422 if a listed pose's robot stores no class name or is no longer
                known to the server, does not match its class, or the stage changes while the
                poses are listed; 503 if the server cannot be reached.
        """
        stage = self._stage
        if stage is None:
            return []
        poses = []
        for pose_path in sorted(self._pose_paths):
            robot_targets = (
                stage.GetPrimAtPath(pose_path)
                .GetRelationship(POSE_ROBOT_RELATIONSHIP)
                .GetTargets()
            )
            if not robot_targets:
                continue
            pose_robot_path = robot_targets[0].pathString
            if robot_prim_path is not None and pose_robot_path != robot_prim_path:
                continue
            base_world = await self._base_frame_world(stage, pose_robot_path)
            if stage is not self._stage:
                raise HTTPException(422, "The stage changed while the poses were listed.")
            pose_prim = stage.GetPrimAtPath(pose_path)
            poses.append(
                {
                    "name": pose_path.name,
                    "prim_path": pose_path.pathString,
                    "robot_prim_path": pose_robot_path,
                    "pose": _pose_in_frame(
                        omni.usd.get_world_transform_matrix(pose_prim), base_world
                    ),
                    "joint_names": list(
                        pose_prim.GetAttribute(POSE_JOINT_NAMES_ATTRIBUTE).Get() or []
                    ),
                    "joint_positions": [
                        float(position)
                        for position in pose_prim.GetAttribute(POSE_JOINT_POSITIONS_ATTRIBUTE).Get()
                        or []
                    ],
                    "reachable": bool(pose_prim.GetAttribute(POSE_REACHABLE_ATTRIBUTE).Get()),
                }
            )
        return poses

    async def link_world_transforms(
        self, pose_path: Sdf.Path
    ) -> tuple[str, dict[str, Gf.Matrix4d]] | None:
        """Return the pose's robot and the world transform of each of its links in the pose's
        stored solution, by link name, or ``None`` when the pose has no solution.

        A pose has no solution when it is unreachable or was never solved. The transforms place
        the robot where its arm reaches the pose, relative to where the robot stands now.

        Raises:
            HTTPException: 422 if the pose names no robot in the stage, the robot stores no
                class name or is no longer known to the server, does not match its class, or the
                stage changes meanwhile; 503 if the server cannot be reached.
        """
        stage = self._stage
        pose_prim = stage.GetPrimAtPath(pose_path) if stage is not None else None
        if not pose_prim or not pose_prim.GetAttribute(POSE_REACHABLE_ATTRIBUTE).Get():
            return None
        joint_positions = pose_prim.GetAttribute(POSE_JOINT_POSITIONS_ATTRIBUTE).Get()
        if not joint_positions:
            return None
        robot_targets = pose_prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
        if not robot_targets or not stage.GetPrimAtPath(robot_targets[0]):
            raise HTTPException(422, f"The robot of {pose_path} is not in the stage.")
        robot_prim_path = robot_targets[0].pathString
        robot_id = await self._connected_robot_id(stage, robot_prim_path)
        joints = {
            "joint_names": list(pose_prim.GetAttribute(POSE_JOINT_NAMES_ATTRIBUTE).Get()),
            "joint_positions": [float(position) for position in joint_positions],
        }
        base_world = await self._base_frame_world(stage, robot_prim_path)
        link_poses = await _call_server(self._server_client.link_poses(robot_id, joints))
        if stage is not self._stage:
            raise HTTPException(422, "The stage changed while the robot overlay was computed.")
        return robot_prim_path, {
            link_name: robot_assembler.offset_matrix(
                models.Transformation(translation=pose[:3], rotation=pose[3:])
            )
            * base_world
            for link_name, pose in link_poses["link_poses"].items()
        }

    async def _base_frame_world(self, stage: Usd.Stage, robot_prim_path: str) -> Gf.Matrix4d:
        """Return the world transform of the base frame of the robot at ``robot_prim_path``.

        Raises:
            HTTPException: 422 if the robot stores no class name or is no longer known to the
                server, or does not match its class; 503 if the server cannot be reached.
        """
        robot_id = await self._connected_robot_id(stage, robot_prim_path)
        robot = await self._describe_robot(robot_id)
        return _world_transform_of_robot_frame(
            stage, robot_prim_path, robot["base_frame"], robot["class_name"]
        )

    def _on_objects_changed(self, notice, sender):
        """Hand the prims a change to the followed stage moved or re-created to Isaac Sim's loop.

        USD calls this while it is still processing the change, on whichever thread made it, so
        the poses are looked up and solved on the loop instead. A move is a change to an
        ``xformOp`` property or to ``xformOpOrder``; a re-created prim, such as a new or
        duplicated pose or one whose transform ops were replaced, is a resync.
        """
        moved_paths = [
            path.GetPrimPath()
            for path in notice.GetChangedInfoOnlyPaths()
            if path.IsPropertyPath()
            and (path.name.startswith("xformOp:") or path.name == "xformOpOrder")
        ]
        resynced_paths = [path.GetPrimPath() for path in notice.GetResyncedPaths()]
        if moved_paths or resynced_paths:
            self._event_loop.call_soon_threadsafe(
                self._on_prims_changed, sender, moved_paths, resynced_paths
            )

    def _on_prims_changed(
        self, stage: Usd.Stage, moved_paths: list[Sdf.Path], resynced_paths: list[Sdf.Path]
    ):
        """Update the known poses under the re-created prims, then solve every pose that is or
        lies under a moved or re-created prim. Runs on Isaac Sim's loop."""
        if stage is not self._stage:
            return
        for resynced_path in resynced_paths:
            self._pose_paths = {
                path for path in self._pose_paths if not path.HasPrefix(resynced_path)
            }
            resynced_prim = stage.GetPrimAtPath(resynced_path)
            if resynced_prim:
                self._pose_paths |= _find_poses(resynced_prim)
        for path in [path for path in self._solved_poses if path not in self._pose_paths]:
            del self._solved_poses[path]
        changed_paths = set(moved_paths) | set(resynced_paths)
        for pose_path in self._pose_paths:
            if any(pose_path.HasPrefix(changed_path) for changed_path in changed_paths):
                self._schedule_solve(pose_path)

    def _schedule_solve(self, pose_path: Sdf.Path):
        """Solve the pose at ``pose_path`` again, or once more after its running solve."""
        if pose_path in self._solve_tasks:
            self._moved_while_solving.add(pose_path)
            return
        self._solve_tasks[pose_path] = asyncio.ensure_future(self._solve_until_unmoved(pose_path))

    async def _solve_until_unmoved(self, pose_path: Sdf.Path):
        """Solve the pose at ``pose_path`` until it was not moved during the last solve."""
        task = asyncio.current_task()
        try:
            while True:
                self._moved_while_solving.discard(pose_path)
                try:
                    await self._solve_moved_pose(pose_path)
                except HTTPException as error:
                    carb.log_warn(
                        f"[bridge] could not solve pose {pose_path}: {error.detail}"
                    )
                if pose_path not in self._moved_while_solving:
                    return
        finally:
            # A stage change replaces the tasks, so only this task's own entry is removed.
            if self._solve_tasks.get(pose_path) is task:
                del self._solve_tasks[pose_path]

    async def _solve_moved_pose(self, pose_path: Sdf.Path):
        """Solve the pose at ``pose_path`` for its current pose and store the solution.

        Nothing happens when the pose is gone or has moved less than the thresholds since it
        was last solved. The solve is seeded with the pose's stored solution, so it stays on the
        same arm configuration, or with the robot's current joints when it has none. The
        solution is written only if the pose is still on the followed stage.

        Raises:
            HTTPException: 422 if the pose names no robot, the robot stores no class name or is
                no longer known to the server, or the robot does not match its class; 503 if the
                server cannot be reached.
        """
        stage = self._stage
        pose_prim = stage.GetPrimAtPath(pose_path)
        if not pose_prim:
            return
        robot_targets = pose_prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
        if not robot_targets or not stage.GetPrimAtPath(robot_targets[0]):
            raise HTTPException(422, "its robot is not in the stage.")
        robot_prim_path = robot_targets[0].pathString
        robot_id = await self._connected_robot_id(stage, robot_prim_path)
        base_world = await self._base_frame_world(stage, robot_prim_path)
        if stage is not self._stage or not pose_prim.IsValid():
            return
        pose = _pose_in_frame(omni.usd.get_world_transform_matrix(pose_prim), base_world)
        solved_pose = self._solved_poses.get(pose_path)
        if solved_pose is not None and not _moved_enough(solved_pose, pose):
            return

        stored_positions = pose_prim.GetAttribute(POSE_JOINT_POSITIONS_ATTRIBUTE).Get()
        if stored_positions:
            seed = {
                "joint_names": list(pose_prim.GetAttribute(POSE_JOINT_NAMES_ATTRIBUTE).Get()),
                "joint_positions": list(stored_positions),
            }
        else:
            articulation_id = await self._articulation_service.find_robot_for_prim(robot_prim_path)
            seed = self._current_joints(stage, articulation_id, robot_prim_path)
        solution = await _call_server(
            self._server_client.inverse_kinematics(robot_id, [pose], seed)
        )
        if stage is not self._stage or not pose_prim.IsValid():
            return
        _write_pose_solution(
            stage, pose_prim, solution["joint_names"], solution["joint_positions"][0]
        )
        self._solved_poses[pose_path] = pose
        if self.on_pose_solved is not None:
            self.on_pose_solved(pose_path)

    async def _connected_robot_id(self, stage: Usd.Stage, robot_prim_path: str) -> str:
        """Return the server robot id stored on the robot prim, loading the robot on the server
        first when it stores none.

        Raises:
            HTTPException: 422 if the robot stores no class name, or the server does not know it
                or answers with another error; 503 if the server cannot be reached.
        """
        async with self._connect_lock:
            robot_id = stage.GetPrimAtPath(robot_prim_path).GetAttribute(ROBOT_ID_ATTRIBUTE).Get()
            return robot_id or await self._load_robot_on_server(stage, robot_prim_path)

    async def _load_robot_on_server(self, stage: Usd.Stage, robot_prim_path: str) -> str:
        """Load the robot at ``robot_prim_path`` on the server from its stored class name, store
        the new id and the exact class name on the prim, and return the id.

        The server robot of the id the prim stored before is deleted. Failing to delete it only
        logs a warning: the robot is loaded either way, and a server that restarted no longer
        has the old robot (404). Called with ``_connect_lock`` held.

        Raises:
            HTTPException: 422 if the robot stores no class name, or the server does not know it
                or answers with another error; 503 if the server cannot be reached.
        """
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        class_name = robot_prim.GetAttribute(ROBOT_CLASS_NAME_ATTRIBUTE).Get()
        if not class_name:
            raise HTTPException(
                422,
                f"{robot_prim_path} has no robot class name. Add the robot again with Add Robot.",
            )
        robot = await _call_server(self._server_client.create_robot(class_name))
        previous_robot_id = robot_prim.GetAttribute(ROBOT_ID_ATTRIBUTE).Get()
        robot_prim.CreateAttribute(ROBOT_ID_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(robot["id"])
        robot_prim.CreateAttribute(ROBOT_CLASS_NAME_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(
            robot["class_name"]
        )
        self._robot_descriptions[robot["id"]] = robot
        if previous_robot_id:
            try:
                await self._server_client.delete_robot(previous_robot_id)
            except IsaacSimServerError as error:
                if error.status_code != 404:
                    carb.log_warn(
                        f"[bridge] could not delete the previous server robot "
                        f"{previous_robot_id} of {robot_prim_path}: {error}"
                    )
        return robot["id"]

    async def _describe_robot(self, robot_id: str) -> dict:
        """Return the server's description of robot ``robot_id``, asking the server only once.

        Raises:
            HTTPException: 422 if the server no longer knows the robot; 503 if it cannot be
                reached.
        """
        if robot_id not in self._robot_descriptions:
            self._robot_descriptions[robot_id] = await _call_server(
                self._server_client.describe_robot(robot_id)
            )
        return self._robot_descriptions[robot_id]

    def _current_joints(
        self, stage: Usd.Stage, articulation_id: str | None, robot_prim_path: str
    ) -> dict:
        """Return the robot's current joints in radians, as an IK seed.

        The seed keeps the solution on the robot's current arm configuration; without it the
        solver may return another configuration reaching the same pose. While the simulation
        runs, the joints of the robot's registered articulation are read from it. Otherwise they
        are the joint states stored in the stage, which is where the robot stands while the
        simulation is stopped: PhysX's ``state:<axis>:physics:position`` on every joint under the
        robot, in degrees for a revolute joint (USD physics' angular unit) and metres for a
        prismatic one, and 0, the schema's fallback, where none is stored. The server ignores
        names that are not joints of its robot.
        """
        state = (
            None
            if articulation_id is None
            else self._articulation_service.get_articulation_state(articulation_id)
        )
        if state is not None:
            return {
                "joint_names": list(
                    self._articulation_service.get_device(articulation_id).dof_names
                ),
                "joint_positions": state["joint_positions"],
            }
        joint_names, joint_positions = [], []
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
            if prim.IsA(UsdPhysics.RevoluteJoint):
                position = prim.GetAttribute("state:angular:physics:position").Get()
                joint_positions.append(math.radians(position or 0.0))
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                position = prim.GetAttribute("state:linear:physics:position").Get()
                joint_positions.append(position or 0.0)
            else:
                continue
            joint_names.append(prim.GetName())
        return {"joint_names": joint_names, "joint_positions": joint_positions}


def _world_transform_of_robot_frame(
    stage: Usd.Stage, robot_prim_path: str, frame_name: str, class_name: str
) -> Gf.Matrix4d:
    """Return the world transform of the robot's prim named ``frame_name``.

    Raises:
        HTTPException: 422 if the robot has no such prim, so it does not match ``class_name``.
    """
    try:
        frame_path = robot_assembler.find_prim_by_name(stage, robot_prim_path, frame_name)
    except RuntimeError as error:
        raise HTTPException(
            422,
            f"{robot_prim_path} has no '{frame_name}', so it does not match its robot class "
            f"{class_name}. Add the robot again with Add Robot.",
        ) from error
    return omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(frame_path))


def _pose_in_frame(world: Gf.Matrix4d, frame_world: Gf.Matrix4d) -> list[float]:
    """Return the pose of the world transform ``world`` in the frame ``frame_world`` as
    ``[x, y, z, rx, ry, rz]``: metres, then extrinsic XYZ Euler angles in degrees."""
    # Gf matrices act on row vectors: the pose in world, then world into the frame.
    translation, rotation = robot_assembler.matrix_to_translation_rotation(
        world * frame_world.GetInverse()
    )
    return translation + rotation


def _moved_enough(previous_pose: list[float], pose: list[float]) -> bool:
    """Return whether ``pose`` differs from ``previous_pose`` by more than
    :data:`MIN_TRANSLATION_CHANGE` or :data:`MIN_ROTATION_CHANGE`."""
    translation_change = np.linalg.norm(np.subtract(pose[:3], previous_pose[:3]))
    rotation_change = np.linalg.norm(np.radians(np.subtract(pose[3:], previous_pose[3:])))
    return translation_change > MIN_TRANSLATION_CHANGE or rotation_change > MIN_ROTATION_CHANGE


def _find_poses(root: Usd.Prim) -> set[Sdf.Path]:
    """Return the paths of the poses at or under ``root``."""
    return {
        prim.GetPath()
        for prim in Usd.PrimRange(root)
        if prim.HasAttribute(POSE_REACHABLE_ATTRIBUTE)
    }


def _write_pose_solution(
    stage: Usd.Stage,
    pose_prim: Usd.Prim,
    joint_names: list[str],
    joint_positions: list[float] | None,
) -> bool:
    """Store an inverse-kinematics solution on a pose and tint it; return whether it is
    reachable.

    ``joint_positions`` is ``None`` when the pose is out of reach: the pose is then marked
    unreachable and keeps the joint positions it stored before, empty for a new pose.
    """
    pose_prim.CreateAttribute(POSE_JOINT_NAMES_ATTRIBUTE, Sdf.ValueTypeNames.StringArray).Set(
        joint_names
    )
    positions_attribute = pose_prim.CreateAttribute(
        POSE_JOINT_POSITIONS_ATTRIBUTE, Sdf.ValueTypeNames.DoubleArray
    )
    if joint_positions is not None:
        positions_attribute.Set(joint_positions)
    elif positions_attribute.Get() is None:
        positions_attribute.Set([])
    reachable = joint_positions is not None
    pose_prim.CreateAttribute(POSE_REACHABLE_ATTRIBUTE, Sdf.ValueTypeNames.Bool).Set(reachable)
    _set_ghost_look(stage, pose_prim, reachable)
    return reachable


async def _call_server(request):
    """Await a server request, turning its failure into an ``HTTPException`` for the user.

    A 404 from the server means it no longer has the robot, e.g. because it restarted; that
    and any other server error is answered with 422, a server that cannot be reached with 503.
    """
    try:
        return await request
    except IsaacSimServerError as error:
        if error.status_code == 404:
            raise HTTPException(
                422,
                "The Telekinesis Isaac Sim Server no longer has this robot, e.g. because it "
                "restarted. Press Connect in Telekinesis Isaac Sim Bridge ▸ Telekinesis Isaac "
                "Sim Server.",
            ) from error
        raise HTTPException(422 if error.status_code is not None else 503, str(error)) from error


def define_ghost_material(
    stage: Usd.Stage, material_path: Sdf.Path, color: Gf.Vec3f
) -> UsdShade.Material:
    """Define the see-through, glowing material ghosts use, tinted ``color``, at
    ``material_path`` in the stage's current edit target.

    It is Kit's ``SimPBR_Translucent`` with ghost material settings (their
    ``wb_ghost.mdl``), ``color`` being both the see-through tint and the glow. Defining it again
    at the same path only updates its colour.
    """
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path.AppendChild("Shader"))
    shader.SetSourceAsset(Sdf.AssetPath(GHOST_MDL), "mdl")
    shader.SetSourceAssetSubIdentifier("SimPBR_Translucent", "mdl")
    shader.CreateInput("ior_constant", Sdf.ValueTypeNames.Float).Set(1.1)
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("transmittance_color", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("transmittance_measurement_distance", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("enable_thin_walled", Sdf.ValueTypeNames.Bool).Set(False)
    shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
    shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(5169.5)
    shader_output = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader_output)
    material.CreateVolumeOutput("mdl").ConnectToSource(shader_output)
    return material


def _set_ghost_look(stage: Usd.Stage, pose_prim: Usd.Prim, reachable: bool):
    """Give ``pose_prim`` its own see-through material, green if ``reachable`` and red if not.

    The material lives under the pose, at ``<pose>/Looks/GhostMaterial``, so a duplicated
    pose brings its own; it is bound again on every call, so a duplicate whose binding still
    targets the original's material gets its own.
    """
    material = define_ghost_material(
        stage,
        pose_prim.GetPath().AppendPath("Looks/GhostMaterial"),
        REACHABLE_COLOR if reachable else UNREACHABLE_COLOR,
    )
    UsdShade.MaterialBindingAPI.Apply(pose_prim).Bind(
        material, UsdShade.Tokens.strongerThanDescendants
    )
