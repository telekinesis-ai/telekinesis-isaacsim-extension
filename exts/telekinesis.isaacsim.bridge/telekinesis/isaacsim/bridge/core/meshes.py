# SPDX-License-Identifier: Apache-2.0
"""Merging a prim's meshes into one mesh, as poses and the robot overlay draw them."""

from collections.abc import Collection

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, Vt


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
