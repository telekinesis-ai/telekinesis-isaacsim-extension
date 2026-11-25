# Bridge tags and robot assembly

How to author a scene so the Telekinesis bridge registers its devices when the scene is
opened, and so robots and tools are assembled exactly as the extension's Add Robot,
Add Mesh, Add Tool and Add Camera windows leave them. `assets/scene_template.py` implements
the arm, coupler, articulated gripper and camera recipes. Copy from it.

## Contents

- Device tags: what makes the bridge restore a device
- Arm
- Mounted rigid prim (coupler, adapter plate, camera that weighs on the arm)
- Articulated gripper (parallel grippers)
- Surface (suction) gripper
- Cameras
- Tool offsets
- Tags a builder never writes

## Device tags

On stage open, the bridge restores a device for every prim that has an **authored**
`telekinesis:<kind>:id` string. Write `""`: the bridge assigns `<kind><n>` itself. A
non-empty id must match `<kind>[1-9][0-9]*`. Kinds: `articulation`, `camera`, `lidar`,
`lightbeam`, `surface_gripper`, `conveyor`. These names are an on-disk format. Never
rename them.

| Prim | Must have | Optional |
|---|---|---|
| Arm (the referencing prim) | `telekinesis:articulation:id` = `""`, `telekinesis:mountLink` (string, link name), `telekinesis:robot:class_name` (string) | `telekinesis:robot:default_tcp` (string, frame name, e.g. `tool0`) |
| Articulated gripper (referencing prim) | `telekinesis:articulation:id` = `""` | `telekinesis:articulation:driven_joints` (token[]): the driver joint only |
| Assembly Xform around arm and tools | relationship `telekinesis:assemblyRobot` → the arm prim | |
| Camera device (a `Camera` prim) | `telekinesis:camera:id` = `""`, `telekinesis:camera:resolution` (int2) | `telekinesis:camera:frequency` (float, Hz) |
| Surface gripper (asset root) | `telekinesis:surface_gripper:id` = `""` | |
| Conveyor, lidar, light beam | `telekinesis:<kind>:id` = `""` | Lidar scan settings live on the lidar prim itself |

- Without `telekinesis:camera:resolution` a camera device fails to restore.
- `class_name` is the catalogue identifier without `/` and `_` (`universal_robots/ur10e` →
  `universalrobotsur10e`). `find_assets.py` prints it. Without it the robot cannot be
  loaded on the Telekinesis Isaac Sim Server, and Add Pose fails.
- `mountLink` is the catalogue's `simulation_mount_link`, also printed by `find_assets.py`.
- `default_tcp` is the catalogue's `default_tcp`, also printed by `find_assets.py`; Add Robot
  writes it too. Add TCP and Add Pose ▸ From TCP then find the default TCP without the
  Telekinesis Isaac Sim Server. Without it they load the robot on the server first.
- Without `driven_joints` a gripper drives all its joints, mimic followers included. Give
  the one joint that is not marked `MIMIC` and has a `drive` in `inspect_asset.py`.

## Arm

1. Define the assembly Xform (e.g. `/World/<Robot>_assembly`). Give it the
   `telekinesis:assemblyRobot` relationship and the robot's world pose. Everything mounted
   on the robot goes under this Xform. Add Camera and Add Tool find the robot through it.
2. Define the arm prim under it as a reference to the asset's default prim (path from
   `inspect_asset.py`) and write the three tags above.
3. Keep the asset's world weld (the joint from the asset root to the base link). It grounds
   the base.
4. Optional: set a home pose with `set_home_pose` (template). Joints must be listed base
   first. Set it before mounting tools, because tools are placed from the mount link's
   pose. Pick angles that keep the arm clear of the scene and point the tool at the work.
   Check the result in `validate_scene.py`'s layout.

## Mounted rigid prim

As Add Mesh mounts a prim (`mount_prim` in `core/robot_assembler.py`):

- Apply `RigidBodyAPI` with `kinematicEnabled = False`. The fixed joint must be what holds it.
- Give every `Mesh` without a collider `CollisionAPI` and `MeshCollisionAPI` with
  approximation `convexHull`. PhysX cannot simulate a triangle mesh on a moving body.
- If it has neither mass nor colliders, author `MassAPI`. Otherwise PhysX assumes 1 kg.
- Apply `FilteredPairsAPI` on the prim, targeting the arm prim. Author it on the
  mounted prim, never on the arm: applying a schema to the arm's root resyncs its
  instanced link visuals, and they vanish.
- Fix it with `mount_on_link` (template): a `PhysicsFixedJoint` named
  `AssemblerFixedJoint` **beneath the mounted body**, `body0` = mount link, `body1` =
  body, `localPos0/localRot0` = the offset, `localPos1/localRot1` = identity. The body is
  placed so both joint frames coincide. Leave `excludeFromArticulation` unset, so the body
  becomes a link of the arm.

## Articulated gripper

As Add Tool merges it (`assemble_tool` in `core/robot_assembler.py`):

1. Reference it under the assembly Xform.
2. `RemoveAPI(UsdPhysics.ArticulationRootAPI)` on every prim of the gripper that has it,
   so the merged tree has one root, the arm's.
3. Deactivate (`SetActive(False)`) every joint of the gripper with fewer than two rigid
   bodies: the weld to the world (`root_joint` on the Robotiq Hand-E). Left enabled, it
   chains the arm to the world through the gripper.
4. `mount_on_link` its **base link** (the body the gripper's articulation is rooted at,
   from `inspect_asset.py`) at the tool offset.
5. Tag `telekinesis:articulation:id = ""` and `driven_joints = [<driver>]` on the
   gripper's referencing prim.
6. Bind the gripping friction material (template), as Add Tool does: friction 1.0,
   restitution 0, `physxMaterial:frictionCombineMode = max`, bound for purpose `physics`
   to every collider without a physics material.

## Surface gripper

Suction grippers (`usd/tools/suction_grippers/...`) have no articulation. As Add Tool
attaches one (`attach_surface_gripper` in `core/robot_assembler.py`):

1. Reference the asset under the assembly Xform. Tag its root
   `telekinesis:surface_gripper:id = ""`.
2. Find in `inspect_asset.py`: the `IsaacSurfaceGripper` prim, its attachment-point joints
   (the D6 joints its `isaac:attachmentPoints` relationship targets), and its rigid mount
   body.
3. Deactivate (`physics:jointEnabled = False`) any joint holding the gripper to the world
   **except the attachment points**, and set `physics:kinematicEnabled = False` on the root.
4. `mount_on_link` the mount body at the tool offset.
5. **Re-park every attachment point** on the arm's mount link. Set `physics:body1` to
   the mount link and `excludeFromArticulation = True`, and recompute `localPos1/localRot1`
   so the joint holds the cup where it already is:
   `parked = Gf.Transform(joint_frame0 * world(body0) * world(mount_link).GetInverse())`,
   where `joint_frame0` is the matrix of `localRot0/localPos0`. Write
   `localPos1 = parked.GetTranslation()` and `localRot1 = parked.GetRotation().GetQuat()`.
   With no second rigid body PhysX never creates the joint, and the gripper silently
   never grips.
6. Apply `FilteredPairsAPI` on the gripper, targeting the arm.

`build_euro_pallet_cell_scene.py` (applications repo, `add_welded_gripper`) is a worked
offline example. It also raises `isaac:coaxialForceLimit`/`isaac:shearForceLimit` to
1e9 N (see physics-pitfalls.md).

## Cameras

- **Fixed camera device**: a `Camera` with the two camera tags. Use `add_camera` (template).
- **Fixed camera not used by the robot** (cinematic or observer views): the same, without tags.
- **Wrist camera that weighs on the arm**: a camera asset (`usd/cameras/RealSense/D405.zip`)
  mounted as a rigid prim at an offset on the mount link. Tag its `Camera` prim, not the
  asset root.
- **Wrist camera that rides with a tool**: a `Camera` defined under the tool's rigid
  body (not a rigid body itself), placed in the tool's frame.

## Tool offsets

Offsets are in the **mount link's frame**. Compose them as
`Gf.Matrix4d().SetTranslate((0, 0, d)) * flange_in_mount_link(...)`. The flange frame is
`tool0` under the mount link if the arm has one (ABB, KUKA, Fanuc: the mount link's X runs
along the flange axis), else the mount link itself (UR: `wrist_3_link` Z points out of
the flange).

`d` is the distance from the flange face to the tool's mating face:

- Tool straight on the flange: `d = 0` when the tool's origin is on its mating face.
  Add Tool's *Auto-calculate Placement* instead puts the tool's backmost point on the
  flange: `d = -min_z` of `inspect_asset.py <tool> --frame <base_link>`.
- Tool on a coupler: `d` = the coupler's top face, its bounds max z (the Robotiq UR
  coupler: 0.014 m).

Rotations of the user's description are extrinsic XYZ Euler degrees. Build them with
`pose_matrix`.

## Tags a builder never writes

`telekinesis:robot:id` and `telekinesis:robot:base_frame` are written when the robot is
loaded on the Telekinesis Isaac Sim Server. `telekinesis:pose:*` tags are written by Add
Pose (From Tool, From Joint Positions, From TCP), and `telekinesis:tcp:robot` by Add TCP.
`telekinesis:metadata` customData is written by the `/prims` routes.
