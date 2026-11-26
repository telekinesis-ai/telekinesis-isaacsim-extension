# SPDX-License-Identifier: Apache-2.0
"""Poses a robot is taught, shown as ghosts, see-through copies of the robot's tool that mark
where the tool should go, or as red/green/blue frames marking where a TCP should go."""

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
from .articulations import (
    ROBOT_BASE_FRAME_ATTRIBUTE,
    ROBOT_CLASS_NAME_ATTRIBUTE,
    ROBOT_DEFAULT_TCP_ATTRIBUTE,
    ROBOT_ID_ATTRIBUTE,
    ArticulationService,
)

# Properties of a pose prim. The relationship targets the robot prim whose TCP the pose
# places, and ``tcp`` names that TCP; poses taught before every pose stored it lack it, which
# means the robot's default TCP. The joint
# attributes hold the robot's inverse-kinematics solution for the pose, in radians;
# ``reachable`` says whether there is one.
POSE_ROBOT_RELATIONSHIP = "telekinesis:pose:robot"
POSE_TCP_ATTRIBUTE = "telekinesis:pose:tcp"
POSE_JOINT_NAMES_ATTRIBUTE = "telekinesis:pose:joint_names"
POSE_JOINT_POSITIONS_ATTRIBUTE = "telekinesis:pose:joint_positions"
POSE_REACHABLE_ATTRIBUTE = "telekinesis:pose:reachable"
POSE_IN_BASE_FRAME_ATTRIBUTE = "telekinesis:pose:pose_in_base_frame"
# Relationship on a TCP prim targeting its robot prim; it also marks the prim as a TCP. The
# TCP's name is the prim's name, and its pose in the robot's default TCP frame is its offset,
# as Synapse's add_tcp takes it.
TCP_ROBOT_RELATIONSHIP = "telekinesis:tcp:robot"
# Name of the TCP at the base of a robot's tool, which Add Pose ▸ From Tool teaches. Synapse's
# attach_tool adds a TCP of the same name at the tool mount it is given, so a program on the real
# robot reaches a taught pose with its own measured tool mount.
TOOL_BASE_TCP_NAME = "tool_base"

# Name of the Xform beside the robot that holds its poses, and the suffix of a new
# pose's name after its tool's: the default pose path is <tool parent>/poses/<tool>_Pose.
POSES_GROUP_NAME = "poses"
POSE_NAME_SUFFIX = "_Pose"
# Look of a pose's ghost: reachability tints on Kit's SimPBR_Translucent material. It is in
# Kit's core MDL library, so a saved stage finds it by name on any Isaac Sim install.
# Reachable is the #41d1ff of the Add Tool preview; MDL reads colours as linear, the hex is sRGB.
REACHABLE_COLOR = Gf.ConvertDisplayToLinear(Gf.Vec3f(0x41, 0xD1, 0xFF) / 255.0)
UNREACHABLE_COLOR = Gf.Vec3f(0.9, 0.15, 0.15)
GHOST_MDL = "SimPBR_Translucent.mdl"
# A moved pose is solved again only when it moved more than this since it was last solved,
# as PrimPoseWatcher does: metres of translation and radians of XYZ Euler rotation.
MIN_TRANSLATION_CHANGE = 0.001
MIN_ROTATION_CHANGE = 0.01


class PoseService:
    """Creates the poses a robot is taught, and solves them again when they are moved.

    Add Pose ▸ From Tool creates a pose of the robot's ``tool_base`` TCP, the base of the tool
    mounted on the robot, where the tool stands now, shown as a ghost of the tool; Add Pose ▸
    From Joint Positions creates one where the robot's default TCP is when the robot stands at
    given joint positions, shown as a ghost of the whole robot at those joints and its tool, if
    it has one; Add Pose ▸ From TCP creates one where a TCP should go, given in the robot's base
    frame, shown as that TCP's frame. Every pose stores the name of the TCP it places.

    A ghost pose is a single ``Mesh`` prim: the ghost's visible meshes merged into the frame of
    the TCP it places, so the pose's origin is where that TCP has to go for the real tool to
    cover the pose. A pose of the tool base is therefore solved for wherever the tool sits on
    the flange: correcting the ``tool_base`` TCP keeps the taught tool placement, and a pose of
    the default TCP, such as one from joint positions, is never solved again because a TCP
    changed. A frame pose is an Xform showing its axes as red/green/blue arrows. Moving
    a pose moves it rigidly; the robot reaching the pose is drawn by the robot overlay. Poses
    live in a ``poses`` Xform beside the robot, in its assembly when it has one, carry no
    physics, and are saved with the stage. Each pose stores the robot's inverse-kinematics
    solution for it, solved by the Telekinesis Isaac Sim Server; a ghost pose is tinted blue
    when the robot can reach it and red when it cannot. Each pose also stores itself in its
    robot's base frame, the ``pose`` :meth:`list_poses` returns, so a program can be written
    from the values the Property window shows.

    While the service follows a stage, a pose that is moved, directly or with one of its parent
    prims, stores its new pose in the base frame at once, and is solved again once it has moved
    more than :data:`MIN_TRANSLATION_CHANGE` or :data:`MIN_ROTATION_CHANGE` since it was last
    solved; so is a pose that appears, such as a duplicate. One solve runs per pose at a time,
    and a move during a solve leads to one more solve, so the stored solution always ends at the
    pose's latest placement. A pose moved out of
    reach keeps its last solution, which seeds the next solve. A solve that fails, e.g. because
    the server is not running, logs a warning and leaves the pose as it was.

    A robot can have TCPs besides its default TCP, created with :meth:`create_tcp`: Xforms under
    the default TCP frame, each placed at its offset from that frame. Attaching a tool with Add
    Tool places the hidden ``tool_base`` TCP at the tool's base with :meth:`place_tool_base_tcp`;
    a robot has one tool, so attaching another moves it. A pose made from a TCP, by
    right-clicking the TCP, places that TCP instead and is solved for it.

    The robots are loaded on the Telekinesis Isaac Sim Server from the Synapse class name each
    robot prim stores, which Add Robot writes. A robot is loaded the first time one of its poses
    needs it, and again for every robot of the stage by :meth:`connect_stage_robots`; the server's
    id is stored on the robot prim. Loading a robot also gives the server the robot's TCPs, and
    while the service follows a stage, a TCP that is created, moved, renamed or deleted gives
    the server its robot's TCPs again and solves the poses of the robot's TCPs again. The stage
    holds the TCPs; a TCP change the server misses, e.g. because it is not running, reaches it
    with the next load.

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
        # TCP prims of the followed stage and the robot prim each belongs to, and the robots
        # whose TCPs are waiting to be given to the server. One TCP sync runs at a time.
        self._tcp_robots: dict[Sdf.Path, str] = {}
        self._tcp_syncs_pending: set[str] = set()
        self._tcp_sync_lock = asyncio.Lock()
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
        closed. Solves still running for the previously followed stage are cancelled. Each pose
        whose stored pose in the base frame is missing or does not match its place, such as a
        pose taught before it was stored, is given the right one.
        """
        if self._objects_changed_listener is not None:
            self._objects_changed_listener.Revoke()
            self._objects_changed_listener = None
        for task in self._solve_tasks.values():
            task.cancel()
        self._solve_tasks.clear()
        self._moved_while_solving.clear()
        self._solved_poses.clear()
        self._tcp_syncs_pending.clear()
        self._stage = stage
        self._pose_paths = set() if stage is None else _find_poses(stage.GetPseudoRoot())
        self._tcp_robots = {} if stage is None else _find_tcps(stage.GetPseudoRoot())
        if stage is not None:
            self._store_poses_in_base_frame(stage)
            self._objects_changed_listener = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._on_objects_changed, stage
            )

    def _store_poses_in_base_frame(self, stage: Usd.Stage):
        """Store each pose of ``stage`` in its robot's base frame where the stored one is missing
        or differs, leaving matching ones untouched so an unchanged stage is not marked modified.

        Needs no server: a pose whose robot stores no base frame yet is skipped and gets it with
        its next solve, and one whose robot does not match its class is skipped with a warning.
        """
        for pose_path in self._pose_paths:
            pose_prim = stage.GetPrimAtPath(pose_path)
            robot_targets = pose_prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
            robot_prim = stage.GetPrimAtPath(robot_targets[0]) if robot_targets else None
            if not robot_prim or not robot_prim.GetAttribute(ROBOT_BASE_FRAME_ATTRIBUTE).Get():
                continue
            try:
                base_world = _world_transform_of_robot_frame(
                    stage,
                    robot_prim.GetPath().pathString,
                    robot_prim.GetAttribute(ROBOT_BASE_FRAME_ATTRIBUTE).Get(),
                    robot_prim.GetAttribute(ROBOT_CLASS_NAME_ATTRIBUTE).Get(),
                )
            except HTTPException as error:
                carb.log_warn(f"[bridge] could not store pose {pose_path}: {error.detail}")
                continue
            pose = _pose_in_frame(omni.usd.get_world_transform_matrix(pose_prim), base_world)
            stored_pose = pose_prim.GetAttribute(POSE_IN_BASE_FRAME_ATTRIBUTE).Get()
            if stored_pose is None or list(stored_pose) != pose:
                _write_pose_in_base_frame(pose_prim, pose)

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

    async def find_robot_and_tool(self, robot_part_prim_path: str) -> tuple[str, str | None]:
        """Return the prim paths of the robot ``robot_part_prim_path`` is part of and of its
        tool, as ``(robot_prim_path, tool_prim_path)``; ``tool_prim_path`` is ``None`` when the
        robot carries no tool added with Add Tool.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.

        Raises:
            HTTPException: 422 if the prim is part of no robot added with Add Robot.
        """
        articulation_id = await self._articulation_service.find_robot_for_prim(robot_part_prim_path)
        if articulation_id is None:
            raise HTTPException(
                422,
                f"'{robot_part_prim_path}' is part of no robot. Right-click a robot added with "
                "Add Robot, or its tool.",
            )
        return (
            self._articulation_service.get_device(articulation_id).prim_path,
            self._articulation_service.find_tool_on_robot(articulation_id),
        )

    async def find_default_tcp(self, robot_part_prim_path: str) -> tuple[str, str]:
        """Return the prim paths of the robot ``robot_part_prim_path`` is part of and of its
        default TCP frame, such as ``tool0``, as ``(robot_prim_path, default_tcp_prim_path)``.

        The server is needed only for a robot that stores no default TCP yet, which is then
        loaded there to store it.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.

        Raises:
            HTTPException: 422 if the prim is part of no robot, the robot stores neither a
                default TCP nor a class name, the server does not know its class, or the robot
                does not match its class; 503 if the server is needed and cannot be reached.
        """
        stage = omni.usd.get_context().get_stage()
        robot_prim_path, _ = await self.find_robot_and_tool(robot_part_prim_path)
        default_tcp_prim_path = await self._robot_frame_path(
            stage, robot_prim_path, ROBOT_DEFAULT_TCP_ATTRIBUTE
        )
        return robot_prim_path, default_tcp_prim_path

    async def find_base_frame(self, robot_part_prim_path: str) -> tuple[str, str]:
        """Return the prim paths of the robot ``robot_part_prim_path`` is part of and of its
        base frame, the link poses are measured from, as ``(robot_prim_path,
        base_frame_prim_path)``.

        The server is needed only for a robot that stores no base frame yet, which is then
        loaded there to store it.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.

        Raises:
            HTTPException: 422 if the prim is part of no robot, the robot stores neither a
                base frame nor a class name, the server does not know its class, or the robot
                does not match its class; 503 if the server is needed and cannot be reached.
        """
        stage = omni.usd.get_context().get_stage()
        robot_prim_path, _ = await self.find_robot_and_tool(robot_part_prim_path)
        base_frame_prim_path = await self._robot_frame_path(
            stage, robot_prim_path, ROBOT_BASE_FRAME_ATTRIBUTE
        )
        return robot_prim_path, base_frame_prim_path

    def robot_tcp_paths(self, robot_prim_path: str) -> dict[str, str]:
        """Return the prim path of each TCP of the robot at ``robot_prim_path`` created with
        :meth:`create_tcp`, by TCP name, sorted by name; empty while no stage is followed."""
        return dict(
            sorted(
                (tcp_path.name, tcp_path.pathString)
                for tcp_path, tcp_robot_path in self._tcp_robots.items()
                if tcp_robot_path == robot_prim_path
            )
        )

    async def solve_tcp_pose(
        self,
        robot_prim_path: str,
        tcp_name: str | None,
        pose: models.Transformation,
        seed: dict | None,
    ) -> dict | None:
        """Return the joint positions that bring a TCP of the robot to ``pose``, or ``None``
        when the robot cannot reach it. Writes nothing to the stage.

        Args:
            robot_prim_path: The robot prim, which stores its Synapse class name.
            tcp_name: Name of a TCP of the robot, or ``None`` for its default TCP.
            pose: The TCP's pose in the robot's base frame, metres and XYZ Euler degrees.
            seed: ``{"joint_names": [...], "joint_positions": [...]}`` in radians to start the
                search from, keeping the solution on that arm configuration; ``None`` starts
                from the robot's current joints.

        Returns:
            ``{"joint_names": [...], "joint_positions": [...]}`` in radians, or ``None``.

        Raises:
            HTTPException: 422 if the robot stores no class name or is no longer known to the
                server, or the server has no TCP ``tcp_name``; 503 if the server cannot be
                reached.
        """
        stage = omni.usd.get_context().get_stage()
        robot_id = await self._connected_robot_id(stage, robot_prim_path)
        solution = await _call_server(
            self._server_client.inverse_kinematics(
                robot_id,
                [[*pose.translation, *pose.rotation]],
                seed or await self.current_joint_positions(robot_prim_path),
                tcp_name,
            )
        )
        if solution["joint_positions"][0] is None:
            return None
        return {
            "joint_names": solution["joint_names"],
            "joint_positions": solution["joint_positions"][0],
        }

    async def create_tcp(
        self,
        robot_part_prim_path: str,
        name: str,
        offset: models.Transformation,
        visible: bool = False,
    ) -> str:
        """Create a TCP of the robot ``robot_part_prim_path`` is part of and return its prim path.

        The TCP is an Xform named ``name`` under the robot's default TCP frame, at ``offset`` in
        that frame: the offset Synapse's ``add_tcp`` takes. It shows its axes as red/green/blue
        arrows, which the Stage window's visibility toggle shows when the TCP is created hidden.
        The Telekinesis Isaac Sim Server gets it when the robot is next loaded there, or at once
        when the robot is loaded already and the service follows the stage; the TCP is created
        without the server. The server is needed only for a robot that stores no default TCP
        yet, which is then loaded there to store it.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.
            name: The TCP's name; it is made a valid prim name.
            offset: The TCP's pose in the default TCP frame, metres and XYZ Euler degrees.
            visible: Whether the TCP's arrows are shown in the viewport and in camera images.

        Raises:
            HTTPException: 422 if the prim is part of no robot, ``name`` is empty or names a
                prim of the robot already, the robot stores neither a default TCP nor a class
                name, the server does not know its class, or the robot does not match its class;
                503 if the server is needed and cannot be reached.
        """
        stage = omni.usd.get_context().get_stage()
        if not name.strip():
            raise HTTPException(422, "Enter a name for the TCP.")
        robot_prim_path, default_tcp_path = await self.find_default_tcp(robot_part_prim_path)
        tcp_name = Tf.MakeValidIdentifier(name.strip())
        # Frames are found by name under the robot, so a TCP must not share a prim's name. The
        # markers Synapse draws for its active TCP are no frames, so their names are free.
        robot_prims = iter(Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)))
        for prim in robot_prims:
            if prim.GetName() == tcp_name:
                raise HTTPException(
                    422, f"{robot_prim_path} already has a prim named '{tcp_name}'."
                )
            if prim.GetName() == robot_assembler.TCP_MARKER_GROUP_NAME:
                robot_prims.PruneChildren()
        tcp_path = Sdf.Path(default_tcp_path).AppendChild(tcp_name)
        marker = robot_assembler.define_tcp_marker(stage, tcp_path, offset)
        marker.GetPrim().CreateRelationship(TCP_ROBOT_RELATIONSHIP).SetTargets([robot_prim_path])
        if not visible:
            marker.MakeInvisible()
        if stage is self._stage:
            # Known at once, so a pose can be made for the TCP before the creation's change
            # notice is handled.
            self._tcp_robots[tcp_path] = robot_prim_path
        return tcp_path.pathString

    async def place_tool_base_tcp(self, robot_part_prim_path: str) -> str:
        """Place the ``tool_base`` TCP of the robot ``robot_part_prim_path`` is part of at the
        base of the robot's tool, where the tool stands now, and return the TCP's prim path.

        The tool base is the tool's body fixed to the robot: an articulated tool's base link, or
        a suction gripper's mounted body. The TCP is created hidden when the robot has none, and
        moved otherwise, so it follows a tool attached in place of another. While the service
        follows the stage, a moved TCP is given to the server again and the poses of the tool
        base are solved again, as for any moved TCP. The server is needed only for a robot that
        stores no default TCP yet, which is then loaded there to store it.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.

        Raises:
            HTTPException: 422 if the prim is part of no robot, the robot has no tool, the robot
                stores neither a default TCP nor a class name, the server does not know its
                class, or the robot does not match its class; 503 if the server is needed and
                cannot be reached.
        """
        stage = omni.usd.get_context().get_stage()
        robot_prim_path, default_tcp_path = await self.find_default_tcp(robot_part_prim_path)
        robot_id = await self._articulation_service.find_robot_for_prim(robot_prim_path)
        tool_base_path = self._articulation_service.find_tool_base_on_robot(robot_id)
        if tool_base_path is None:
            raise HTTPException(
                422, f"'{robot_prim_path}' has no tool. Attach one with Add Tool first."
            )
        tool_base_world = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(tool_base_path))
        tcp_path = self.robot_tcp_paths(robot_prim_path).get(TOOL_BASE_TCP_NAME)
        if tcp_path is None:
            offset = _pose_in_frame(
                tool_base_world,
                omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(default_tcp_path)),
            )
            return await self.create_tcp(
                robot_prim_path,
                TOOL_BASE_TCP_NAME,
                models.Transformation(translation=offset[:3], rotation=offset[3:]),
            )
        tcp = UsdGeom.Xformable(stage.GetPrimAtPath(tcp_path))
        translation, rotation = robot_assembler.matrix_to_translation_rotation(
            tool_base_world
            * omni.usd.get_world_transform_matrix(tcp.GetPrim().GetParent()).GetInverse()
        )
        # The translate and rotateXYZ ops create_tcp authors, so the Property window keeps
        # showing the TCP's offset.
        tcp.ClearXformOpOrder()
        tcp.AddTranslateOp().Set(Gf.Vec3d(*translation))
        tcp.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
        return tcp_path

    async def create_pose_from_tool(self, robot_part_prim_path: str) -> dict:
        """Create a pose from the tool mounted on a robot: a pose of the whole tool,
        where the tool stands now.

        The robot must be added with Add Robot and carry a tool added with Add Tool. The pose
        places the robot's ``tool_base`` TCP, which is placed at the tool's base first when the
        robot has none, e.g. because a program attached the tool; or the TCP
        ``robot_part_prim_path`` is. It is named after the tool, or after that TCP. The robot's
        inverse kinematics for the pose is solved before the pose is written, so a failure
        leaves no pose behind. It is seeded with the robot's current joints, so the solution
        keeps the robot's current arm configuration.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool; a TCP of the
                robot to make the pose for that TCP.

        Returns:
            ``{"prim_path": ..., "reachable": ...}`` of the new pose.

        Raises:
            HTTPException: 422 if the stage is not in metres, the prim is part of no robot, the
                robot has no tool, the robot stores no class name or is no longer known to the
                server, the robot does not match its class, or the tool has no meshes; 503 if
                the server cannot be reached.
        """
        return await self._create_pose(robot_part_prim_path, None)

    async def create_pose_from_joint_positions(
        self, robot_part_prim_path: str, joints: dict
    ) -> dict:
        """Create a pose where the robot's default TCP is when the robot stands at ``joints``.

        The pose places the robot's default TCP whichever prim it is made from, so ``joints``
        stay its solution whatever is later done to the robot's tool or TCPs. It is placed at
        the default TCP in the forward kinematics of ``joints``, computed by the Telekinesis
        Isaac Sim Server, and stores ``joints`` as its solution, so it is reachable. Its ghost is
        the whole robot standing at ``joints`` together with its tool, when it carries one added
        with Add Tool; the pose is named after the tool, or the robot when it has none. The
        forward kinematics is computed before anything is written, so a failure leaves the
        stage unchanged.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.
            joints: ``{"joint_names": [...], "joint_positions": [...]}`` in radians, naming
                every joint of the robot.

        Returns:
            ``{"prim_path": ..., "reachable": True}`` of the new pose.

        Raises:
            HTTPException: 422 if the stage is not in metres, the prim is part of no robot, the
                robot stores no class name or is no longer known to the server, the robot does
                not match its class, the robot and its tool have no meshes, or the server
                rejects ``joints``, e.g. for a joint missing or outside its limits; 503 if the
                server cannot be reached.
        """
        return await self._create_pose(robot_part_prim_path, joints)

    async def create_pose_from_tcp(
        self,
        robot_part_prim_path: str,
        tcp_name: str | None,
        pose: models.Transformation,
    ) -> dict:
        """Create a pose of a TCP of the robot at ``pose`` in the robot's base frame.

        The pose is an Xform showing its axes as red/green/blue arrows, named after the TCP
        or, for the default TCP, after the default TCP frame. Its inverse kinematics is solved,
        seeded with the robot's current joints, before anything is written, so a failure leaves
        the stage unchanged; a pose out of reach is still created, marked unreachable.

        Args:
            robot_part_prim_path: Any prim of the robot, its assembly, or its tool.
            tcp_name: Name of a TCP of the robot, or ``None`` for its default TCP.
            pose: Where the TCP should go, in the robot's base frame: metres and XYZ Euler
                degrees.

        Returns:
            ``{"prim_path": ..., "reachable": ...}`` of the new pose.

        Raises:
            HTTPException: 422 if the stage is not in metres, the prim is part of no robot, the
                robot stores no class name or is no longer known to the server, the robot does
                not match its class, or the server has no TCP ``tcp_name``; 503 if the server
                cannot be reached.
        """
        stage = omni.usd.get_context().get_stage()
        _require_metres(stage)
        robot_prim_path, default_tcp_prim_path = await self.find_default_tcp(robot_part_prim_path)
        if tcp_name is not None:
            # Given again in case the TCP changed since the server last got it.
            robot_id = await self._connected_robot_id(stage, robot_prim_path)
            await _call_server(
                self._server_client.set_tcps(robot_id, self._robot_tcps(stage, robot_prim_path))
            )
        solution = await self.solve_tcp_pose(robot_prim_path, tcp_name, pose, None)
        base_world = await self._base_frame_world(stage, robot_prim_path)

        poses_path = asset_loader.path_beside_clear_of_rigid_bodies(
            stage, robot_prim_path, POSES_GROUP_NAME
        )
        UsdGeom.Xform.Define(stage, poses_path)
        pose_name = (tcp_name or Sdf.Path(default_tcp_prim_path).name) + POSE_NAME_SUFFIX
        pose_path = omni.usd.get_stage_next_free_path(stage, f"{poses_path}/{pose_name}", False)
        # Gf composes row-vector first: the pose in the base frame, then the base frame.
        pose_world = robot_assembler.offset_matrix(pose) * base_world
        translation, rotation = robot_assembler.matrix_to_translation_rotation(
            pose_world
            * omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(poses_path)).GetInverse()
        )
        pose_prim = robot_assembler.define_tcp_marker(
            stage,
            pose_path,
            models.Transformation(translation=translation, rotation=rotation),
        ).GetPrim()
        reachable = self._tag_new_pose(
            stage,
            pose_prim,
            robot_prim_path,
            tcp_name,
            _pose_in_frame(pose_world, base_world),
            solution or {"joint_names": [], "joint_positions": None},
        )
        return {"prim_path": pose_path, "reachable": reachable}

    async def _create_pose(self, robot_part_prim_path: str, joints: dict | None) -> dict:
        """Create a pose of a robot's TCP, as :meth:`create_pose_from_tool` when ``joints`` is
        ``None`` and as :meth:`create_pose_from_joint_positions` otherwise."""
        stage = omni.usd.get_context().get_stage()
        _require_metres(stage)
        robot_prim_path, tool_prim_path = await self.find_robot_and_tool(robot_part_prim_path)
        if tool_prim_path is None and joints is None:
            raise HTTPException(
                422, f"'{robot_prim_path}' has no tool. Attach one with Add Tool first."
            )
        tcp_name, tcp_prim_path = None, None
        if joints is None:
            tcp_name = find_tcp_name(stage.GetPrimAtPath(robot_part_prim_path), robot_prim_path)
            tcp_prim_path = robot_part_prim_path
            if tcp_name is None:
                tcp_name = TOOL_BASE_TCP_NAME
                tcp_prim_path = self.robot_tcp_paths(robot_prim_path).get(
                    TOOL_BASE_TCP_NAME
                ) or await self.place_tool_base_tcp(robot_prim_path)
        robot_id = await self._connected_robot_id(stage, robot_prim_path)
        robot = await self._describe_robot(robot_id)
        if tcp_name is not None:
            # Given again in case the TCP changed since the server last got it.
            await _call_server(
                self._server_client.set_tcps(robot_id, self._robot_tcps(stage, robot_prim_path))
            )

        default_tcp_world = _world_transform_of_robot_frame(
            stage, robot_prim_path, robot["default_tcp"], robot["class_name"]
        )
        base_world = _world_transform_of_robot_frame(
            stage, robot_prim_path, robot["base_frame"], robot["class_name"]
        )
        tcp_world = (
            default_tcp_world
            if tcp_prim_path is None
            else omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(tcp_prim_path))
        )
        ghost_meshes = []
        if tool_prim_path is not None:
            # The tool is fixed to the robot, so its meshes in the current TCP's frame are also
            # its meshes in the TCP's frame at any joint positions.
            ghost_meshes.append(meshes.merge_meshes_in_frame(stage, tool_prim_path, tcp_world))

        if joints is None:
            pose_world = tcp_world
            pose = _pose_in_frame(pose_world, base_world)
            solution = await _call_server(
                self._server_client.inverse_kinematics(
                    robot_id,
                    [pose],
                    await self.current_joint_positions(robot_prim_path),
                    tcp_name,
                )
            )
            joint_names, joint_positions = (
                solution["joint_names"],
                solution["joint_positions"][0],
            )
        else:
            link_world_transforms = await self.robot_link_world_transforms(robot_prim_path, joints)
            pose_world = link_world_transforms[robot["default_tcp"]]
            pose = _pose_in_frame(pose_world, base_world)
            joint_names, joint_positions = joints["joint_names"], joints["joint_positions"]
            link_paths = meshes.find_link_paths(stage, robot_prim_path, link_world_transforms)
            for link_name, link_path in link_paths.items():
                link_world_now = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(link_path))
                # Merged in this frame, the link's meshes land where the link stands at
                # ``joints``, expressed in the pose's frame.
                merge_frame_world = (
                    pose_world * link_world_transforms[link_name].GetInverse() * link_world_now
                )
                ghost_meshes.append(
                    meshes.merge_meshes_in_frame(
                        stage,
                        link_path.pathString,
                        merge_frame_world,
                        excluded_paths=set(link_paths.values()) - {link_path},
                    )
                )
        points, face_vertex_counts, face_vertex_indices = meshes.concatenate_meshes(ghost_meshes)
        if not face_vertex_counts:
            raise HTTPException(
                422,
                f"'{tool_prim_path or robot_prim_path}' has no visible meshes to show as a ghost.",
            )

        poses_path = asset_loader.path_beside_clear_of_rigid_bodies(
            stage, robot_prim_path, POSES_GROUP_NAME
        )
        UsdGeom.Xform.Define(stage, poses_path)
        # A pose of the tool base is named after the tool it shows.
        pose_name = (
            tcp_name
            if tcp_name not in (None, TOOL_BASE_TCP_NAME)
            else Tf.MakeValidIdentifier(Sdf.Path(tool_prim_path or robot_prim_path).name)
        ) + POSE_NAME_SUFFIX
        pose_path = omni.usd.get_stage_next_free_path(stage, f"{poses_path}/{pose_name}", False)
        pose_mesh = UsdGeom.Mesh.Define(stage, pose_path)
        pose_mesh.CreatePointsAttr(points)
        pose_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        pose_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
        local_transform = Gf.Transform(
            pose_world
            * omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(poses_path)).GetInverse()
        )
        pose_mesh.AddTranslateOp().Set(local_transform.GetTranslation())
        pose_mesh.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(local_transform.GetRotation().GetQuat())
        )
        pose_mesh.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(local_transform.GetScale())

        reachable = self._tag_new_pose(
            stage,
            pose_mesh.GetPrim(),
            robot_prim_path,
            tcp_name,
            pose,
            {"joint_names": joint_names, "joint_positions": joint_positions},
        )
        return {"prim_path": pose_path, "reachable": reachable}

    def _tag_new_pose(
        self,
        stage: Usd.Stage,
        pose_prim: Usd.Prim,
        robot_prim_path: str,
        tcp_name: str | None,
        pose: list[float],
        solution: dict,
    ) -> bool:
        """Mark the new ``pose_prim`` as a pose of the robot's TCP ``tcp_name`` (``None`` for
        the default TCP, whose name is stored), store ``pose`` in the base frame and its
        ``solution``, and return whether it is reachable.

        ``solution`` is ``{"joint_names": [...], "joint_positions": [...]}`` in radians, with
        ``joint_positions`` ``None`` for a pose out of reach.
        """
        pose_prim.CreateRelationship(POSE_ROBOT_RELATIONSHIP).SetTargets([robot_prim_path])
        pose_prim.CreateAttribute(POSE_TCP_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(
            tcp_name
            or stage.GetPrimAtPath(robot_prim_path).GetAttribute(ROBOT_DEFAULT_TCP_ATTRIBUTE).Get()
        )
        _write_pose_in_base_frame(pose_prim, pose)
        reachable = _write_pose_solution(
            stage, pose_prim, solution["joint_names"], solution["joint_positions"]
        )
        # Recorded before the creation's own change notice is handled, so that notice does not
        # solve the new pose a second time.
        self._solved_poses[pose_prim.GetPath()] = pose
        if self.on_pose_solved is not None:
            self.on_pose_solved(pose_prim.GetPath())
        return reachable

    async def list_poses(self, robot_prim_path: str | None = None) -> list[dict]:
        """Return the poses of the open stage, sorted by prim path.

        Each entry is ``{"name", "prim_path", "robot_prim_path", "tcp", "pose", "joint_names",
        "joint_positions", "reachable"}``: ``tcp`` is the name of the TCP the pose places, such
        as ``tool_base`` for a pose from a tool or the default TCP's name for a pose from joint
        positions, and ``None`` for a pose taught before poses stored it, which places the
        default TCP; ``pose`` is the pose's ``[x, y, z, rx, ry, rz]`` in
        its robot's base frame, metres and extrinsic XYZ Euler degrees, the pose a program moves
        that TCP to; ``joint_positions`` is the stored solution in radians.

        Args:
            robot_prim_path: Only the poses of this robot, or every pose when ``None``.

        The poses are read from the stage. The Telekinesis Isaac Sim Server is needed only for a
        robot that stores no base frame yet, one never loaded on the server since the bridge
        started storing it.

        Raises:
            HTTPException: 422 if a listed pose's robot stores neither a base frame nor a class
                name, the server does not know its class, the robot does not match its class, or
                the stage changes while the poses are listed; 503 if the server is needed and
                cannot be reached.
        """
        stage = self._stage
        if stage is None:
            return []
        poses = []
        for pose_path in sorted(self._pose_paths):
            robot_targets = (
                stage.GetPrimAtPath(pose_path).GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
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
                    "tcp": pose_prim.GetAttribute(POSE_TCP_ATTRIBUTE).Get() or None,
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
        joints = {
            "joint_names": list(pose_prim.GetAttribute(POSE_JOINT_NAMES_ATTRIBUTE).Get()),
            "joint_positions": [float(position) for position in joint_positions],
        }
        return robot_prim_path, await self.robot_link_world_transforms(robot_prim_path, joints)

    async def robot_link_world_transforms(
        self, robot_prim_path: str, joints: dict
    ) -> dict[str, Gf.Matrix4d]:
        """Return the world transform of each link of the robot at ``robot_prim_path`` when it
        stands at ``joints``, by link name, relative to where the robot stands now.

        Args:
            robot_prim_path: The robot prim, which stores its Synapse class name.
            joints: ``{"joint_names": [...], "joint_positions": [...]}`` in radians, naming
                every joint of the robot.

        Raises:
            HTTPException: 422 if the robot stores no class name or is no longer known to the
                server, does not match its class, the server rejects ``joints``, or the stage
                changes meanwhile; 503 if the server cannot be reached.
        """
        stage = self._stage
        robot_id = await self._connected_robot_id(stage, robot_prim_path)
        base_world = await self._base_frame_world(stage, robot_prim_path)
        link_poses = await _call_server(self._server_client.link_poses(robot_id, joints))
        if stage is not self._stage:
            raise HTTPException(422, "The stage changed while the robot's links were computed.")
        return {
            link_name: robot_assembler.offset_matrix(
                models.Transformation(translation=pose[:3], rotation=pose[3:])
            )
            * base_world
            for link_name, pose in link_poses["link_poses"].items()
        }

    async def _base_frame_world(self, stage: Usd.Stage, robot_prim_path: str) -> Gf.Matrix4d:
        """Return the world transform of the base frame of the robot at ``robot_prim_path``.

        The base frame's name is read from the robot prim, so the server is contacted only for
        a robot that stores none yet, which is then loaded on the server to store it.

        Raises:
            HTTPException: 422 if the robot stores neither a base frame nor a class name, the
                server does not know its class, or the robot does not match its class; 503 if
                the server cannot be reached.
        """
        frame_path = await self._robot_frame_path(
            stage, robot_prim_path, ROBOT_BASE_FRAME_ATTRIBUTE
        )
        return omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(frame_path))

    async def _robot_frame_path(
        self, stage: Usd.Stage, robot_prim_path: str, frame_attribute: str
    ) -> str:
        """Return the path of the robot's prim named by the robot prim's ``frame_attribute``,
        such as its base frame or default TCP.

        The server is contacted only for a robot that stores no name in ``frame_attribute``
        yet, which is then loaded on the server to store it.

        Raises:
            HTTPException: 422 if the robot stores neither the name nor a class name, the server
                does not know its class, or the robot does not match its class; 503 if the
                server cannot be reached.
        """
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        async with self._connect_lock:
            if not robot_prim.GetAttribute(frame_attribute).Get():
                await self._load_robot_on_server(stage, robot_prim_path)
        return _find_robot_frame(
            stage,
            robot_prim_path,
            robot_prim.GetAttribute(frame_attribute).Get(),
            robot_prim.GetAttribute(ROBOT_CLASS_NAME_ATTRIBUTE).Get(),
        )

    def _robot_tcps(self, stage: Usd.Stage, robot_prim_path: str) -> dict[str, list[float]]:
        """Return each TCP of the robot at ``robot_prim_path`` by name: its pose
        ``[x, y, z, rx, ry, rz]`` in the robot's default TCP frame, metres and extrinsic XYZ Euler
        degrees, read from where its prim stands, so a TCP moved anywhere fixed to the default
        TCP keeps its meaning.

        Raises:
            HTTPException: 422 if the robot has TCPs and its stored default TCP is not one of its
                prims, so it does not match its class.
        """
        tcp_prims = [
            stage.GetPrimAtPath(tcp_path)
            for tcp_path, tcp_robot_path in self._tcp_robots.items()
            if tcp_robot_path == robot_prim_path and stage.GetPrimAtPath(tcp_path)
        ]
        if not tcp_prims:
            return {}
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        default_tcp_world = _world_transform_of_robot_frame(
            stage,
            robot_prim_path,
            robot_prim.GetAttribute(ROBOT_DEFAULT_TCP_ATTRIBUTE).Get(),
            robot_prim.GetAttribute(ROBOT_CLASS_NAME_ATTRIBUTE).Get(),
        )
        return {
            tcp_prim.GetName(): _pose_in_frame(
                omni.usd.get_world_transform_matrix(tcp_prim), default_tcp_world
            )
            for tcp_prim in tcp_prims
        }

    def _schedule_tcp_sync(self, robot_prim_path: str):
        """Give the server the TCPs of the robot at ``robot_prim_path`` again, unless that is
        already waiting to run."""
        if robot_prim_path in self._tcp_syncs_pending:
            return
        self._tcp_syncs_pending.add(robot_prim_path)
        asyncio.ensure_future(self._sync_robot_tcps(robot_prim_path))

    async def _sync_robot_tcps(self, robot_prim_path: str):
        """Give the server the TCPs of the robot at ``robot_prim_path`` as the stage holds them
        now, then solve the robot's poses of a TCP again.

        Only a robot loaded on the server gets its TCPs; any other gets them when it is loaded.
        A failure logs a warning and leaves the poses as they are. Runs one at a time, and reads
        the stage only once it runs, so the last sync gives the server the latest TCPs.
        """
        async with self._tcp_sync_lock:
            self._tcp_syncs_pending.discard(robot_prim_path)
            stage = self._stage
            robot_prim = stage.GetPrimAtPath(robot_prim_path) if stage is not None else None
            if not robot_prim:
                return
            robot_id = robot_prim.GetAttribute(ROBOT_ID_ATTRIBUTE).Get()
            if robot_id:
                try:
                    await _call_server(
                        self._server_client.set_tcps(
                            robot_id, self._robot_tcps(stage, robot_prim_path)
                        )
                    )
                except HTTPException as error:
                    carb.log_warn(
                        f"[bridge] could not give the TCPs of {robot_prim_path} to the "
                        f"Telekinesis Isaac Sim Server: {error.detail}"
                    )
                    return
            if stage is not self._stage:
                return
            default_tcp = robot_prim.GetAttribute(ROBOT_DEFAULT_TCP_ATTRIBUTE).Get()
            for pose_path in self._pose_paths:
                pose_prim = stage.GetPrimAtPath(pose_path)
                robot_targets = pose_prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
                pose_tcp = pose_prim.GetAttribute(POSE_TCP_ATTRIBUTE).Get()
                if (
                    pose_tcp
                    and pose_tcp != default_tcp
                    and robot_targets
                    and robot_targets[0].pathString == robot_prim_path
                ):
                    # Solved again even though the pose did not move: its TCP did.
                    self._solved_poses.pop(pose_path, None)
                    self._schedule_solve(pose_path)

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
        """Update the known poses and TCPs under the re-created prims, then solve every pose
        that is or lies under a moved or re-created prim, and give the server the TCPs of every
        robot with a TCP that was moved itself, re-created or removed. Runs on Isaac Sim's loop.

        A TCP moved only with its parent prims, such as the robot's links while the simulation
        runs, keeps its offset from the default TCP, so it is not given to the server again.
        """
        if stage is not self._stage:
            return
        changed_tcp_robots = {
            self._tcp_robots[path] for path in moved_paths if path in self._tcp_robots
        }
        for resynced_path in resynced_paths:
            self._pose_paths = {
                path for path in self._pose_paths if not path.HasPrefix(resynced_path)
            }
            removed_tcps = {
                path: robot_path
                for path, robot_path in self._tcp_robots.items()
                if path.HasPrefix(resynced_path)
            }
            for path in removed_tcps:
                del self._tcp_robots[path]
            changed_tcp_robots |= set(removed_tcps.values())
            resynced_prim = stage.GetPrimAtPath(resynced_path)
            if resynced_prim:
                self._pose_paths |= _find_poses(resynced_prim)
                found_tcps = _find_tcps(resynced_prim)
                self._tcp_robots |= found_tcps
                changed_tcp_robots |= set(found_tcps.values())
        for robot_path in changed_tcp_robots:
            self._schedule_tcp_sync(robot_path)
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
                    carb.log_warn(f"[bridge] could not solve pose {pose_path}: {error.detail}")
                if pose_path not in self._moved_while_solving:
                    return
        finally:
            # A stage change replaces the tasks, so only this task's own entry is removed.
            if self._solve_tasks.get(pose_path) is task:
                del self._solve_tasks[pose_path]

    async def _solve_moved_pose(self, pose_path: Sdf.Path):
        """Store the pose at ``pose_path`` in its robot's base frame, then solve it for that pose
        and its TCP and store the solution.

        Nothing happens when the pose is gone, and only the pose in the base frame is stored when
        it has moved less than the thresholds since it was last solved. The solve is seeded
        with the pose's stored solution, so it stays on the same arm configuration, or with the
        robot's current joints when it has none. The solution is written only if the pose is
        still on the followed stage.

        Raises:
            HTTPException: 422 if the pose names no robot, the robot stores no class name or is
                no longer known to the server, the robot does not match its class, or the server
                has no TCP of the pose's TCP name, e.g. because the TCP was renamed or deleted;
                503 if the server cannot be reached.
        """
        stage = self._stage
        pose_prim = stage.GetPrimAtPath(pose_path)
        if not pose_prim:
            return
        robot_targets = pose_prim.GetRelationship(POSE_ROBOT_RELATIONSHIP).GetTargets()
        if not robot_targets or not stage.GetPrimAtPath(robot_targets[0]):
            raise HTTPException(422, "its robot is not in the stage.")
        robot_prim_path = robot_targets[0].pathString
        base_world = await self._base_frame_world(stage, robot_prim_path)
        if stage is not self._stage or not pose_prim.IsValid():
            return
        pose = _pose_in_frame(omni.usd.get_world_transform_matrix(pose_prim), base_world)
        # Stored on every move, also one too small to solve again, so it always matches the
        # pose's place.
        _write_pose_in_base_frame(pose_prim, pose)
        solved_pose = self._solved_poses.get(pose_path)
        if solved_pose is not None and not _moved_enough(solved_pose, pose):
            return

        robot_id = await self._connected_robot_id(stage, robot_prim_path)
        stored_positions = pose_prim.GetAttribute(POSE_JOINT_POSITIONS_ATTRIBUTE).Get()
        if stored_positions:
            seed = {
                "joint_names": list(pose_prim.GetAttribute(POSE_JOINT_NAMES_ATTRIBUTE).Get()),
                "joint_positions": list(stored_positions),
            }
        else:
            seed = await self.current_joint_positions(robot_prim_path)
        solution = await _call_server(
            self._server_client.inverse_kinematics(
                robot_id, [pose], seed, pose_prim.GetAttribute(POSE_TCP_ATTRIBUTE).Get() or None
            )
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
        the new id, the exact class name, the base frame and the default TCP on the prim, give
        the server the robot's TCPs, and return the id.

        The server robot of the id the prim stored before is deleted. Failing to delete it only
        logs a warning: the robot is loaded either way, and a server that restarted no longer
        has the old robot (404). Called with ``_connect_lock`` held.

        Raises:
            HTTPException: 422 if the robot stores no class name, the server does not know it,
                refuses a TCP or answers with another error, or the robot does not match its
                class; 503 if the server cannot be reached.
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
        robot_prim.CreateAttribute(ROBOT_BASE_FRAME_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(
            robot["base_frame"]
        )
        robot_prim.CreateAttribute(ROBOT_DEFAULT_TCP_ATTRIBUTE, Sdf.ValueTypeNames.String).Set(
            robot["default_tcp"]
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
        tcps = self._robot_tcps(stage, robot_prim_path)
        if tcps:
            await _call_server(self._server_client.set_tcps(robot["id"], tcps))
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

    async def current_joint_positions(self, robot_prim_path: str) -> dict:
        """Return the current joints of the robot at ``robot_prim_path`` as
        ``{"joint_names": [...], "joint_positions": [...]}``, in radians and metres.

        Used as an IK seed, which keeps the solution on the robot's current arm configuration;
        without it the solver may return another configuration reaching the same pose. While
        the simulation runs, the joints of the robot's registered articulation are read from it,
        which include those of a gripper merged into it. Otherwise they
        are the joint states stored in the stage, which is where the robot stands while the
        simulation is stopped: PhysX's ``state:<axis>:physics:position`` on every joint under the
        robot, in degrees for a revolute joint (USD physics' angular unit) and metres for a
        prismatic one, and 0, the schema's fallback, where none is stored. The server ignores
        names that are not joints of its robot.
        """
        stage = self._stage
        articulation_id = await self._articulation_service.find_robot_for_prim(robot_prim_path)
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


def _require_metres(stage: Usd.Stage):
    """Raise ``HTTPException`` 422 unless ``stage`` is in metres.

    The bridge loads robots at metre scale whatever the stage unit, so a pose read in another
    unit would not match the robot.
    """
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        raise HTTPException(422, "Poses need a stage in metres (metersPerUnit = 1).")


def _find_robot_frame(
    stage: Usd.Stage, robot_prim_path: str, frame_name: str, class_name: str
) -> str:
    """Return the path of the robot's prim named ``frame_name``.

    Raises:
        HTTPException: 422 if the robot has no such prim, so it does not match ``class_name``.
    """
    try:
        return robot_assembler.find_prim_by_name(stage, robot_prim_path, frame_name)
    except RuntimeError as error:
        raise HTTPException(
            422,
            f"{robot_prim_path} has no '{frame_name}', so it does not match its robot class "
            f"{class_name}. Add the robot again with Add Robot.",
        ) from error


def _world_transform_of_robot_frame(
    stage: Usd.Stage, robot_prim_path: str, frame_name: str, class_name: str
) -> Gf.Matrix4d:
    """Return the world transform of the robot's prim named ``frame_name``.

    Raises:
        HTTPException: 422 if the robot has no such prim, so it does not match ``class_name``.
    """
    frame_path = _find_robot_frame(stage, robot_prim_path, frame_name, class_name)
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


def _find_tcps(root: Usd.Prim) -> dict[Sdf.Path, str]:
    """Return the paths of the TCPs at or under ``root``, each with its robot prim's path."""
    tcps = {}
    for prim in Usd.PrimRange(root):
        robot_targets = prim.GetRelationship(TCP_ROBOT_RELATIONSHIP).GetTargets()
        if robot_targets:
            tcps[prim.GetPath()] = robot_targets[0].pathString
    return tcps


def find_tcp_name(prim: Usd.Prim, robot_prim_path: str) -> str | None:
    """Return the name of the TCP ``prim`` is when it is a TCP of the robot at
    ``robot_prim_path``, otherwise ``None``."""
    robot_targets = prim.GetRelationship(TCP_ROBOT_RELATIONSHIP).GetTargets() if prim else []
    if robot_targets and robot_targets[0].pathString == robot_prim_path:
        return prim.GetName()
    return None


def _write_pose_in_base_frame(pose_prim: Usd.Prim, pose: list[float]):
    """Store ``pose``, the pose's ``[x, y, z, rx, ry, rz]`` in its robot's base frame in metres
    and extrinsic XYZ Euler degrees, on the pose, where the Property window shows it."""
    pose_prim.CreateAttribute(POSE_IN_BASE_FRAME_ATTRIBUTE, Sdf.ValueTypeNames.DoubleArray).Set(
        pose
    )


def _write_pose_solution(
    stage: Usd.Stage,
    pose_prim: Usd.Prim,
    joint_names: list[str],
    joint_positions: list[float] | None,
) -> bool:
    """Store an inverse-kinematics solution on a pose and, when it is a ghost, tint it; return
    whether it is reachable.

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
    # A pose made from a TCP is a frame of red/green/blue arrows, which keeps its colours.
    if pose_prim.IsA(UsdGeom.Mesh):
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
    """Give ``pose_prim`` its own see-through material, blue if ``reachable`` and red if not.

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
