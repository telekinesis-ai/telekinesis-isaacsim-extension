# SPDX-License-Identifier: Apache-2.0
"""Prim service: the orchestration behind the ``/prims`` routes.

``PrimService`` reads and edits individual prims on the open stage -- poses
(world/local, rotation-vector or quaternion), the stored "default pose" used to
reset a prim, user metadata, visibility, physics joint enable, and colliders. It
needs the open stage, so it is composed with :class:`StageService` (one shared
instance, injected at construction) and reuses that single "409 if no stage"
rule rather than duplicating it.

The pose math is kept as pure module-level helpers (no stage, no ``self``) so the
class stays focused on prim lookup + edits. omni/pxr imports are lazy so this
module imports outside Isaac Sim.
"""

import math

from fastapi import HTTPException

# customData namespace where we persist a prim's "default pose" (local-space,
# 6-float rotation-vector form). Mirrors how the extension remembers a pose to
# reset prims back to.
_DEFAULT_POSE_KEY = "telekinesis:default_pose"


def _matrix_to_pose(matrix, rotation_type):
    """Gf.Matrix4d -> {"pose": [...]} in cartesian (rotvec) or quaternion form."""
    translation = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotation()
    head = [translation[0], translation[1], translation[2]]
    if rotation_type == "quaternions":
        quat = rotation.GetQuat()
        imaginary = quat.GetImaginary()
        return {"pose": head + [quat.GetReal(), imaginary[0], imaginary[1], imaginary[2]]}
    # cartesian: rotation vector = axis * angle (radians)
    axis = rotation.GetAxis()
    angle = math.radians(rotation.GetAngle())
    return {"pose": head + [axis[0] * angle, axis[1] * angle, axis[2] * angle]}


def _wspose_to_matrix(pose):
    """6-float rotation-vector pose -> Gf.Matrix4d."""
    from pxr import Gf

    if len(pose) != 6:
        raise HTTPException(status_code=400, detail=f"expected 6 pose values (x,y,z,rx,ry,rz), got {len(pose)}")
    x, y, z, rx, ry, rz = pose
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle:
        rotation = Gf.Rotation(Gf.Vec3d(rx / angle, ry / angle, rz / angle), math.degrees(angle))
    else:
        rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotateOnly(rotation)
    matrix.SetTranslateOnly(Gf.Vec3d(x, y, z))
    return matrix


def _world_matrix(prim):
    from pxr import Usd, UsdGeom

    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def _local_matrix(prim):
    from pxr import Usd, UsdGeom

    return UsdGeom.Xformable(prim).GetLocalTransformation(Usd.TimeCode.Default())


def _set_local_matrix(prim, local_matrix):
    from pxr import UsdGeom

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(local_matrix)


def _set_world_matrix(prim, world_matrix):
    """Set a prim's pose given a world-space matrix (converts through the parent)."""
    from pxr import Usd, UsdGeom

    parent = prim.GetParent()
    if parent and parent.IsValid() and parent.IsA(UsdGeom.Xformable):
        parent_world = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local_matrix = world_matrix * parent_world.GetInverse()
    else:
        local_matrix = world_matrix
    _set_local_matrix(prim, local_matrix)


class PrimService:
    """Read and edit individual prims on the open stage."""

    def __init__(self, stage_service):
        """``stage_service`` supplies the open stage (and the 409-if-none rule)."""
        self._stage_service = stage_service

    def _prim_or_404(self, prim_path):
        """Resolve a prim path on the open stage, or raise 404."""
        prim = self._stage_service.stage().GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            raise HTTPException(status_code=404, detail=f"prim '{prim_path}' not found")
        return prim

    def get_pose(self, prim_path, coordinate_system, rotation_type):
        """Pose of a prim, in world or local space, as rotvec or quaternion."""
        prim = self._prim_or_404(prim_path)
        matrix = _world_matrix(prim) if coordinate_system == "world" else _local_matrix(prim)
        return _matrix_to_pose(matrix, rotation_type)

    def update_pose(self, prim_path, pose):
        """Set a prim's world-space pose from a rotation-vector pose (6 floats)."""
        prim = self._prim_or_404(prim_path)
        _set_world_matrix(prim, _wspose_to_matrix(pose))

    def get_relative_pose(self, prim_path_1, prim_path_2, mode, rotation_type):
        """Pose of prim 2 expressed in prim 1's frame.

        ``mode`` optionally inverts either world transform before composing:
        ``relative = world2 * inverse(world1)``.
        """
        a = _world_matrix(self._prim_or_404(prim_path_1))
        b = _world_matrix(self._prim_or_404(prim_path_2))
        if mode in ("inverse_first", "inverse_both"):
            a = a.GetInverse()
        if mode in ("inverse_second", "inverse_both"):
            b = b.GetInverse()
        return _matrix_to_pose(b * a.GetInverse(), rotation_type)

    def apply_relative_pose(self, prim_path, relative_pose, object_first):
        """Pre/post-multiply a prim's world pose by a relative pose (6 floats).

        ``object_first`` chooses the multiplication order: ``world * relative`` when
        true (move in the prim's own frame), ``relative * world`` otherwise (move in
        world frame).
        """
        prim = self._prim_or_404(prim_path)
        relative = _wspose_to_matrix(relative_pose)
        current = _world_matrix(prim)
        new_world = (current * relative) if object_first else (relative * current)
        _set_world_matrix(prim, new_world)

    def list_default_poses(self):
        """Map every prim that has a stored default pose to that pose (local rotvec)."""
        from pxr import Usd

        poses = {}
        for prim in Usd.PrimRange(self._stage_service.stage().GetPseudoRoot()):
            stored = prim.GetCustomDataByKey(_DEFAULT_POSE_KEY)
            if stored is not None:
                poses[prim.GetPath().pathString] = {"pose": list(stored)}
        return poses

    def assign_default_pose(self, prim_path):
        """Record the prim's current local pose as its default (for later reset)."""
        prim = self._prim_or_404(prim_path)
        prim.SetCustomDataByKey(_DEFAULT_POSE_KEY, _matrix_to_pose(_local_matrix(prim), "cartesian")["pose"])

    def clear_default_poses(self):
        """Forget every stored default pose."""
        from pxr import Usd

        for prim in Usd.PrimRange(self._stage_service.stage().GetPseudoRoot()):
            if prim.GetCustomDataByKey(_DEFAULT_POSE_KEY) is not None:
                prim.ClearCustomDataByKey(_DEFAULT_POSE_KEY)

    def reset_to_default_pose(self, prim_path):
        """Restore a prim to its stored default pose (404 if none was assigned)."""
        prim = self._prim_or_404(prim_path)
        stored = prim.GetCustomDataByKey(_DEFAULT_POSE_KEY)
        if stored is None:
            raise HTTPException(status_code=404, detail=f"prim '{prim_path}' has no default pose")
        _set_local_matrix(prim, _wspose_to_matrix(list(stored)))

    def set_metadata(self, prim_path, metadata):
        """Store user metadata (category/type, a plain dict) on a prim under customData."""
        prim = self._prim_or_404(prim_path)
        prim.SetCustomDataByKey("telekinesis:metadata", metadata)

    def remove_metadata(self, prim_path):
        """Remove the user metadata previously stored on a prim."""
        prim = self._prim_or_404(prim_path)
        prim.ClearCustomDataByKey("telekinesis:metadata")

    def set_visibility(self, prim_path, visible):
        """Show (``visible=True``) or hide a prim (UsdGeom imageable visibility)."""
        from pxr import UsdGeom

        imageable = UsdGeom.Imageable(self._prim_or_404(prim_path))
        if visible:
            imageable.MakeVisible()
        else:
            imageable.MakeInvisible()

    def set_joint_state(self, prim_path, enable):
        """Enable or disable a physics joint."""
        from pxr import UsdPhysics

        prim = self._prim_or_404(prim_path)
        if not prim.IsA(UsdPhysics.Joint):
            raise HTTPException(status_code=400, detail=f"prim '{prim_path}' is not a physics joint")
        UsdPhysics.Joint(prim).CreateJointEnabledAttr(enable)

    def update_colliders(self, prim_path, enable):
        """Enable or disable collision on a prim (applies the CollisionAPI as needed)."""
        from pxr import UsdPhysics

        prim = self._prim_or_404(prim_path)
        collision = UsdPhysics.CollisionAPI(prim)
        if not collision:
            collision = UsdPhysics.CollisionAPI.Apply(prim)
        collision.CreateCollisionEnabledAttr(enable)
