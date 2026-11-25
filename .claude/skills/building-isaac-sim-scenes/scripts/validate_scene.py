"""Check a built scene before it is handed over, and print its layout for comparison with the request.

    python validate_scene.py examples/scenes/my_cell_scene.usda
    python validate_scene.py my_cell_scene.usda --floor 0.0

Errors (exit code 1) are scenes that open wrong in Isaac Sim or that the bridge cannot
restore: unresolved files, malformed or incomplete ``telekinesis:`` tags, a gripper merged
into an arm that still has its own articulation root or world weld, nested articulation
roots, mount joints whose bodies are missing or not rigid.

Warnings are physics that will move on Play: joints whose two frames do not coincide at
the authored pose (PhysX snaps the bodies together), free bodies without colliders, below
the floor, or overlapping another body or a static collider. Overlap is tested on
axis-aligned bounds, so a part resting inside a concave tray or bin can be reported
although it is clear; judge those from the layout.

The layout lists every prim two levels under the default prim with its world bounds,
every free body with its mass, and every camera with its position and view direction.

Needs ``usd-core``; Isaac Sim need not run.
"""

import argparse
import re

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

DEVICE_KINDS = ("articulation", "camera", "lidar", "lightbeam", "surface_gripper", "conveyor")
MOUNT_JOINT_NAME = "AssemblerFixedJoint"
JOINT_POSITION_TOLERANCE = 0.001  # m; below what a viewer or a grasp notices
JOINT_ANGLE_TOLERANCE = 0.5  # degrees
OVERLAP_TOLERANCE = 0.003  # m; convex colliders stand a few mm proud of their meshes
FLOOR_TOLERANCE = 0.005  # m


class Report:
    """Collected errors and warnings, printed at the end."""

    def __init__(self):
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)


def format_vector(vector) -> str:
    return "(" + ", ".join(f"{value:.3f}" for value in vector) + ")"


def world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def joint_frame_world(joint: UsdPhysics.Joint, body_index: int, stage: Usd.Stage) -> Gf.Matrix4d | None:
    """World matrix of the joint's frame on body 0 or 1; None if that body is the world."""
    relationship = joint.GetBody0Rel() if body_index == 0 else joint.GetBody1Rel()
    targets = relationship.GetTargets()
    position = (joint.GetLocalPos0Attr() if body_index == 0 else joint.GetLocalPos1Attr()).Get() or Gf.Vec3f(0.0)
    rotation = (joint.GetLocalRot0Attr() if body_index == 0 else joint.GetLocalRot1Attr()).Get() or Gf.Quatf(1.0)
    local = Gf.Matrix4d().SetRotate(Gf.Quatd(rotation)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(position))
    if not targets:
        return local
    body = stage.GetPrimAtPath(targets[0])
    if not body:
        return None
    return local * world_matrix(body)


def check_dependencies(scene_path: str, report: Report) -> None:
    _, _, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(scene_path))
    for path in unresolved:
        report.error(f"Unresolved file: {path}")


def check_stage_metadata(stage: Usd.Stage, report: Report) -> None:
    if not stage.GetDefaultPrim():
        report.error("No default prim: set it to /World.")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        report.warn(f"metersPerUnit is {UsdGeom.GetStageMetersPerUnit(stage)}, not 1.0.")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        report.warn(f"upAxis is {UsdGeom.GetStageUpAxis(stage)}, not Z.")
    if not any(prim.IsA(UsdPhysics.Scene) for prim in stage.Traverse()):
        report.warn("No PhysicsScene: Isaac Sim adds a default one, without the scene's own settings.")


def find_by_name(prim: Usd.Prim, name: str) -> Usd.Prim | None:
    return next((descendant for descendant in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()) if descendant.GetName() == name), None)


def has_active_articulation_root(prim: Usd.Prim) -> list[str]:
    return [
        str(descendant.GetPath())
        for descendant in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
        if descendant.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]


def check_tags(stage: Usd.Stage, prims: list[Usd.Prim], rigid_bodies: set, report: Report) -> None:
    seen_ids: dict[str, str] = {}
    for prim in prims:
        path = str(prim.GetPath())
        for kind in DEVICE_KINDS:
            attribute = prim.GetAttribute(f"telekinesis:{kind}:id")
            if not attribute or not attribute.HasAuthoredValue():
                continue
            device_id = attribute.Get()
            if device_id != "" and not re.fullmatch(rf"{kind}[1-9][0-9]*", str(device_id)):
                report.error(f"{path}: telekinesis:{kind}:id is {device_id!r}; use \"\" or {kind}<n>.")
            elif device_id:
                if device_id in seen_ids:
                    report.warn(f"{path} and {seen_ids[device_id]} both store {device_id}; the second gets a fresh id.")
                seen_ids.setdefault(device_id, path)

        if prim.HasAttribute("telekinesis:camera:id"):
            resolution = prim.GetAttribute("telekinesis:camera:resolution")
            value = resolution.Get() if resolution else None
            if value is None or str(resolution.GetTypeName()) != "int2" or min(value) <= 0:
                report.error(f"{path}: a camera device needs telekinesis:camera:resolution as int2, e.g. (1280, 720).")
            if not prim.IsA(UsdGeom.Camera):
                report.error(f"{path}: telekinesis:camera:id is on a {prim.GetTypeName() or 'typeless'} prim, not a Camera.")

        mount_link = prim.GetAttribute("telekinesis:mountLink")
        if mount_link and mount_link.HasAuthoredValue() and not find_by_name(prim, mount_link.Get()):
            report.error(f"{path}: telekinesis:mountLink {mount_link.Get()!r} names no prim under it.")

        class_name = prim.GetAttribute("telekinesis:robot:class_name")
        if class_name and class_name.HasAuthoredValue():
            if not re.fullmatch(r"[A-Za-z0-9]+", class_name.Get() or ""):
                report.error(f"{path}: telekinesis:robot:class_name {class_name.Get()!r} must be the catalogue identifier without '/' and '_'.")
            if not prim.HasAttribute("telekinesis:articulation:id"):
                report.warn(f"{path}: has a robot class name but no telekinesis:articulation:id, so the bridge never registers it.")

        relationship = prim.GetRelationship("telekinesis:assemblyRobot")
        if relationship:
            targets = relationship.GetTargets()
            if len(targets) != 1 or not stage.GetPrimAtPath(targets[0]) or not targets[0].HasPrefix(prim.GetPath()):
                report.error(f"{path}: telekinesis:assemblyRobot must target one robot prim inside this assembly, not {[str(t) for t in targets]}.")

        driven = prim.GetAttribute("telekinesis:articulation:driven_joints")
        if driven and driven.HasAuthoredValue():
            for name in driven.Get() or []:
                joint = find_by_name(prim, name)
                if joint is None or not UsdPhysics.Joint(joint):
                    report.error(f"{path}: driven joint {name!r} is not a joint under it.")

        if prim.HasAttribute("telekinesis:articulation:id"):
            mount_joints = [
                descendant for descendant in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
                if descendant.GetName().startswith(MOUNT_JOINT_NAME)
                and any(not target.HasPrefix(prim.GetPath()) for target in UsdPhysics.Joint(descendant).GetBody0Rel().GetTargets())
            ]
            roots = has_active_articulation_root(prim)
            if mount_joints:
                if roots:
                    report.error(f"{path}: merged into an arm by {mount_joints[0].GetPath()} but still has ArticulationRootAPI on {roots}; remove it.")
                for descendant in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
                    joint = UsdPhysics.Joint(descendant)
                    if not joint or descendant.GetAttribute("physics:jointEnabled").Get() is False:
                        continue
                    bodies = [*joint.GetBody0Rel().GetTargets(), *joint.GetBody1Rel().GetTargets()]
                    if sum(body in rigid_bodies for body in bodies) < 2:
                        report.error(f"{path}: merged into an arm but {descendant.GetPath()} still welds it to the world; deactivate it.")
            elif not roots:
                report.error(f"{path}: telekinesis:articulation:id on a prim with no ArticulationRootAPI at or below it.")


def check_articulation_nesting(prims: list[Usd.Prim], report: Report) -> None:
    roots = [prim.GetPath() for prim in prims if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    for root in roots:
        for other in roots:
            if other != root and root.HasPrefix(other):
                report.error(f"Articulation root {root} is nested under articulation root {other}.")


def check_joints(stage: Usd.Stage, prims: list[Usd.Prim], rigid_bodies: set, report: Report) -> set:
    """Check mount joints and joint frames; return the bodies held by an enabled joint."""
    jointed = set()
    for prim in prims:
        joint = UsdPhysics.Joint(prim)
        if not joint or prim.GetAttribute("physics:jointEnabled").Get() is False:
            continue
        bodies = [*joint.GetBody0Rel().GetTargets(), *joint.GetBody1Rel().GetTargets()]
        jointed.update(bodies)
        if prim.GetName().startswith(MOUNT_JOINT_NAME):
            for body in bodies:
                if not stage.GetPrimAtPath(body):
                    report.error(f"{prim.GetPath()}: body {body} does not exist.")
                elif body not in rigid_bodies:
                    report.error(f"{prim.GetPath()}: body {body} is not a rigid body; a mount joint needs one on both ends.")
        if prim.GetAttribute("physics:excludeFromArticulation").Get():
            continue  # surface gripper attachment points rest unsatisfied by design
        frame0 = joint_frame_world(joint, 0, stage)
        frame1 = joint_frame_world(joint, 1, stage)
        if frame0 is None or frame1 is None:
            continue
        gap = (frame0.ExtractTranslation() - frame1.ExtractTranslation()).GetLength()
        if gap > JOINT_POSITION_TOLERANCE:
            report.warn(f"{prim.GetPath()}: joint frames are {gap * 1000:.1f} mm apart at the authored pose; PhysX will pull the bodies together on Play.")
        elif prim.IsA(UsdPhysics.FixedJoint):
            relative = Gf.Rotation(Gf.Quatd(frame0.ExtractRotationQuat()).GetInverse() * Gf.Quatd(frame1.ExtractRotationQuat()))
            angle = abs(relative.GetAngle())
            angle = min(angle, 360.0 - angle)
            if angle > JOINT_ANGLE_TOLERANCE:
                report.warn(f"{prim.GetPath()}: fixed joint frames differ by {angle:.1f} degrees at the authored pose.")
    return jointed


def check_free_bodies(prims: list[Usd.Prim], rigid_bodies: set, jointed: set, floor: float, report: Report) -> list[str]:
    """Check bodies no joint holds; return their layout lines."""
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def bounds(prim: Usd.Prim) -> Gf.Range3d:
        return bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()

    def under_rigid_body(prim: Usd.Prim) -> bool:
        return any(path in rigid_bodies for path in prim.GetPath().GetAncestorsRange())

    free = [
        prim for prim in prims
        if prim.GetPath() in rigid_bodies and prim.GetPath() not in jointed
        and not prim.GetAttribute("physics:kinematicEnabled").Get()
    ]
    obstacles = [(prim, bounds(prim)) for prim in prims if prim.GetPath() in rigid_bodies and prim.GetPath() not in {f.GetPath() for f in free}]
    obstacles += [(prim, bounds(prim)) for prim in prims if prim.HasAPI(UsdPhysics.CollisionAPI) and not under_rigid_body(prim)]
    free_bounds = [(prim, bounds(prim)) for prim in free]

    lines = []
    for index, (prim, box) in enumerate(free_bounds):
        has_collider = any(descendant.HasAPI(UsdPhysics.CollisionAPI) for descendant in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
        mass = prim.GetAttribute("physics:mass").Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
        lines.append(f"{prim.GetPath()}  min {format_vector(box.GetMin())} max {format_vector(box.GetMax())}  mass {mass if mass else 'from colliders'}")
        if not has_collider:
            report.warn(f"{prim.GetPath()}: free rigid body without a collider; it falls through everything.")
        if not box.IsEmpty() and box.GetMin()[2] < floor - FLOOR_TOLERANCE:
            report.warn(f"{prim.GetPath()}: reaches {box.GetMin()[2]:.3f} m, below the floor at {floor} m.")
        others = obstacles + free_bounds[index + 1:]
        for other, other_box in others:
            if other.GetPath().HasPrefix(prim.GetPath()) or prim.GetPath().HasPrefix(other.GetPath()) or box.IsEmpty() or other_box.IsEmpty():
                continue
            overlap = Gf.Range3d.GetIntersection(box, other_box)
            if overlap.IsEmpty() or min(overlap.GetSize()) <= OVERLAP_TOLERANCE:
                continue
            if overlap == box or overlap == other_box:
                continue  # one inside the other's bounds: usually a part in a tray or bin
            report.warn(f"{prim.GetPath()} overlaps {other.GetPath()} by {format_vector(overlap.GetSize())} m (bounds).")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scene")
    parser.add_argument("--floor", type=float, default=0.0, help="height of the floor, m")
    arguments = parser.parse_args()

    report = Report()
    check_dependencies(arguments.scene, report)
    stage = Usd.Stage.Open(arguments.scene)
    check_stage_metadata(stage, report)
    prims = [prim for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()) if prim.IsActive()]
    rigid_bodies = {prim.GetPath() for prim in prims if prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    check_tags(stage, prims, rigid_bodies, report)
    check_articulation_nesting(prims, report)
    jointed = check_joints(stage, prims, rigid_bodies, report)
    free_lines = check_free_bodies(prims, rigid_bodies, jointed, arguments.floor, report)

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    print("== Layout: world bounds (m) two levels under the default prim")
    for child in root.GetChildren():
        for prim in [child, *child.GetChildren()]:
            if not prim.IsActive():
                continue
            box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            indent = "  " if prim == child else "    "
            extent = "no extent" if box.IsEmpty() else f"min {format_vector(box.GetMin())} max {format_vector(box.GetMax())}"
            print(f"{indent}{prim.GetPath()} [{prim.GetTypeName() or 'over'}]  {extent}")
    tagged = [prim for prim in prims if any(name.startswith("telekinesis:") for name in prim.GetPropertyNames())]
    print(f"\n== Tagged prims ({len(tagged)})")
    for prim in tagged:
        box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        extent = "no extent" if box.IsEmpty() else f"min {format_vector(box.GetMin())} max {format_vector(box.GetMax())}"
        tags = {}
        for name in prim.GetPropertyNames():
            if name.startswith("telekinesis:"):
                prop = prim.GetProperty(name)
                tags[name] = [str(target) for target in prop.GetTargets()] if isinstance(prop, Usd.Relationship) else prop.Get()
        print(f"  {prim.GetPath()}  {extent}\n    {tags}")
    print(f"\n== Free bodies ({len(free_lines)})")
    for line in free_lines:
        print(f"  {line}")
    cameras = [prim for prim in prims if prim.IsA(UsdGeom.Camera)]
    print(f"\n== Cameras ({len(cameras)})")
    for camera in cameras:
        matrix = world_matrix(camera)
        forward = -Gf.Vec3d(matrix.GetRow3(2)).GetNormalized()  # a USD camera looks along its -Z
        tags = " (bridge camera device)" if camera.HasAttribute("telekinesis:camera:id") else ""
        print(f"  {camera.GetPath()}  at {format_vector(matrix.ExtractTranslation())} looking {format_vector(forward)}{tags}")

    print(f"\n== Warnings ({len(report.warnings)})")
    for message in report.warnings:
        print(f"  {message}")
    print(f"\n== Errors ({len(report.errors)})")
    for message in report.errors:
        print(f"  {message}")
    raise SystemExit(1 if report.errors else 0)


if __name__ == "__main__":
    main()
