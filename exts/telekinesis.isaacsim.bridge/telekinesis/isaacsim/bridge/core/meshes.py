# SPDX-License-Identifier: Apache-2.0
"""Merging a prim's meshes into one mesh, as poses and the robot overlay draw them."""

from collections.abc import Collection, Iterable

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, Vt


def find_link_paths(
    stage: Usd.Stage, robot_prim_path: str, link_names: Collection[str]
) -> dict[str, Sdf.Path]:
    """Return the prim of each link of the robot at ``robot_prim_path`` named in
    ``link_names``, by link name; names without a prim under the robot are left out.

    A link may hold a child of its own name, such as an ABB link_1's visual "link_1"; the
    traversal reaches the link before its children, so the first prim of a name is the link.
    """
    link_paths = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
        if prim.GetName() in link_names and prim.GetName() not in link_paths:
            link_paths[prim.GetName()] = prim.GetPath()
    return link_paths


def concatenate_meshes(
    meshes: Iterable[tuple[Vt.Vec3fArray, Vt.IntArray, Vt.IntArray]],
) -> tuple[Vt.Vec3fArray, Vt.IntArray, Vt.IntArray]:
    """Join meshes given as (points, face vertex counts, face vertex indices), all in the same
    frame, into one mesh of that form; all empty when there are none."""
    points, face_vertex_counts, face_vertex_indices = [], [], []
    point_count = 0
    for mesh_points, counts, indices in meshes:
        points.append(np.array(mesh_points, dtype=np.float32).reshape(-1, 3))
        face_vertex_counts.append(np.array(counts, dtype=np.int32))
        face_vertex_indices.append(np.array(indices, dtype=np.int32) + point_count)
        point_count += len(mesh_points)
    if not points:
        return Vt.Vec3fArray(), Vt.IntArray(), Vt.IntArray()
    return (
        Vt.Vec3fArray.FromNumpy(np.concatenate(points)),
        Vt.IntArray.FromNumpy(np.concatenate(face_vertex_counts)),
        Vt.IntArray.FromNumpy(np.concatenate(face_vertex_indices)),
    )


def merge_meshes_in_frame(
    stage: Usd.Stage,
    root_path: str,
    frame_world: Gf.Matrix4d,
    excluded_paths: Collection[Sdf.Path] = (),
) -> tuple[Vt.Vec3fArray, Vt.IntArray, Vt.IntArray]:
    """Merge every visible ``Mesh`` under ``root_path`` into one mesh in the frame ``frame_world``.

    Meshes inside instances are included. Meshes that are invisible or whose purpose is guide or
    proxy, such as collision meshes, are skipped, as are gprims that are not meshes and the whole
    subtree of every prim in ``excluded_paths``.

    Args:
        stage: Stage holding the meshes.
        root_path: Prim whose meshes are merged.
        frame_world: World transform of the frame the merged points are expressed in.
        excluded_paths: Prims under ``root_path`` whose subtrees are left out, e.g. the child
            links of a robot link.

    Returns:
        The merged mesh's points, face vertex counts and face vertex indices; all empty when
        there are no meshes.
    """
    xform_cache = UsdGeom.XformCache()
    frame_from_world = np.array(frame_world.GetInverse())
    points, face_vertex_counts, face_vertex_indices = [], [], []
    point_count = 0
    prims = iter(
        Usd.PrimRange(
            stage.GetPrimAtPath(root_path), Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
        )
    )
    for prim in prims:
        if prim.GetPath() in excluded_paths:
            prims.PruneChildren()
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        imageable = UsdGeom.Imageable(prim)
        if (
            imageable.ComputeVisibility() == UsdGeom.Tokens.invisible
            or imageable.ComputePurpose()
            not in (
                UsdGeom.Tokens.default_,
                UsdGeom.Tokens.render,
            )
        ):
            continue
        mesh = UsdGeom.Mesh(prim)
        mesh_points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not mesh_points or not counts or not indices:
            continue
        # Gf matrices act on row vectors: mesh frame -> world -> the target frame.
        mesh_to_frame = np.array(xform_cache.GetLocalToWorldTransform(prim)) @ frame_from_world
        homogeneous = np.hstack(
            [np.array(mesh_points, dtype=float), np.ones((len(mesh_points), 1))]
        )
        points.append((homogeneous @ mesh_to_frame)[:, :3])
        face_vertex_counts.append(np.array(counts))
        face_vertex_indices.append(np.array(indices) + point_count)
        point_count += len(mesh_points)
    if not points:
        return Vt.Vec3fArray(), Vt.IntArray(), Vt.IntArray()
    return (
        Vt.Vec3fArray.FromNumpy(np.concatenate(points).astype(np.float32)),
        Vt.IntArray.FromNumpy(np.concatenate(face_vertex_counts).astype(np.int32)),
        Vt.IntArray.FromNumpy(np.concatenate(face_vertex_indices).astype(np.int32)),
    )
