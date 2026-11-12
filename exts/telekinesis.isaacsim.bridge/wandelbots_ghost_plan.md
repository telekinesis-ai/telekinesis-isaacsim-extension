# Wandelbots Isaac Sim — How “Create Ghost Object” Works

## Goal

Understand and replicate how the Wandelbots Isaac Sim extension creates a movable, translucent ghost of a tool or gripper, without duplicating its rigid bodies, physics joints, or articulation.

**Repository:** https://github.com/wandelbotsgmbh/wandelbots-isaacsim-extension

## 1. Entry Point: Right-Click → Create → Wandelbots NOVA → Ghost Object

The context-menu action is registered in:

`ui/schema/schema_extension_ui.py`

It calls:

```python
GhostObjectApiSchema.create_ghost_object_from_payload(payload)
```

The handler is implemented in:

`ui/schema/schema_components.py`

### Preconditions

- Exactly one prim must be selected.
- The selected prim must have Wandelbots `ToolAPI`.
- The tool must contain at least one valid TCP (Tool Center Point).
- If multiple TCPs exist, the user can select which one to use.

The handler ultimately calls:

```python
await GhostObjectUtils.add_ghost_object(
    source_prim=tool_prim,
    tcp_world_pose=pose,
    tcp_prim=tcp_prim,
)
```

## 2. Ghost Creation

**Main implementation:** `utils/teaching.py`

Functions:

- `GhostObjectUtils.add_ghost_object()`
- `GhostObjectUtils._convert_prim_to_ghost_prim()`

### Step-by-step

1. Identify the original tool/gripper USD prim.
2. Determine its selected TCP transform.
3. Generate a new USD prim path, normally under a sibling `poses/` hierarchy.
4. Create a new `UsdGeom.Mesh` at that path.
5. Collect the original tool's visual meshes, including descendant meshes.
6. Transform mesh vertices into the selected TCP coordinate frame.
7. Merge the geometry into a single standalone mesh.
8. Add translation, orientation, and scale Xform operations.
9. Apply a custom translucent MDL material.
10. Apply Wandelbots `GhostObjectAPI` and link the original TCP.
11. Position the ghost so its origin coincides with the selected TCP's world pose.

### Simplified pseudocode

```python
async def create_ghost(tool_prim, tcp_prim):
    tcp_pose = get_world_pose(tcp_prim)

    ghost_path = get_unique_path(
        f"{tool_prim.GetParent().GetPath()}/poses/"
        f"{tool_prim.GetName()}_Pose"
    )

    # Create only a visual mesh — not a copied articulation
    ghost_mesh = UsdGeom.Mesh.Define(stage, ghost_path)

    # Transform and merge source meshes into TCP coordinates
    vertices, indices, counts = await build_ghost_mesh(
        source_prim=tool_prim,
        tcp_prim=tcp_prim,
    )

    ghost_mesh.CreatePointsAttr().Set(vertices)
    ghost_mesh.CreateFaceVertexIndicesAttr().Set(indices)
    ghost_mesh.CreateFaceVertexCountsAttr().Set(counts)

    add_transform_ops(ghost_mesh)
    apply_translucent_ghost_material(ghost_mesh)

    # Link ghost back to its original TCP
    ghost_api = GhostObjectAPI.Apply(ghost_mesh.GetPrim())
    ghost_api.GetSourceTcpRel().AddTarget(tcp_prim.GetPath())

    set_world_pose(ghost_mesh.GetPrim(), tcp_pose)

    return ghost_mesh
```

This is illustrative pseudocode, not a verbatim implementation.

## 3. Mesh Processing

**Relevant files:**

- `utils/mesh.py`
- `utils/watertight_mesh.py`
- `utils/ghost_mesh_cache.py`

Wandelbots does not simply duplicate the entire tool USD hierarchy.

Instead, it generates a standalone visual representation.

### Geometry pipeline

```text
Original tool/gripper USD
    |
    +-- Visual Mesh A
    +-- Visual Mesh B
    +-- Visual Mesh C
    +-- TCP
    |
    v
Read descendant UsdGeom.Mesh prims
    |
    v
Collect vertices and triangle indices
    |
    v
Apply transforms relative to TCP
    |
    v
Generate fast convex-hull preview
    |
    v
Refine geometry asynchronously
    |
    v
Merge into watertight mesh
    |
    v
Write result into a single UsdGeom.Mesh
```

The refinement pipeline uses:

- Convex-hull generation for an initial preview.
- CoACD for convex decomposition.
- `manifold3d` for mesh operations.
- Mesh optimization/decimation.

The expensive refinement runs in a separate subprocess to avoid blocking Isaac Sim.

### Mesh caching

Completed meshes are cached using:

- Original source prim path.
- A geometry-content hash.
- Source-to-TCP relative transform.

Creating multiple ghosts from the same unchanged tool can therefore reuse the previously generated mesh.

## 4. Ghost Material

**Relevant code:** `GhostObjectUtils.add_material_to_prim()` in `utils/teaching.py`

**Shader asset:** `assets/wb_ghost.mdl`

The extension:

1. Copies `wb_ghost.mdl` into the scene project's material directory.
2. Creates a `UsdShade.Material`.
3. Creates a `UsdShade.Shader` referencing the MDL.
4. Connects the material's surface, displacement, and volume outputs.
5. Binds the material to the ghost mesh using `UsdShade.MaterialBindingAPI`.

The material is translucent and emissive.

The original tool's materials are not preserved as separate materials on the ghost.

## 5. Physics and Articulation Handling

**Critical: the generated ghost is not another physical robot or gripper.**

The creation function uses:

```python
UsdGeom.Mesh.Define(stage, target_path)
```

It does **not** copy the original gripper's:

- `UsdPhysics.RigidBodyAPI`
- `UsdPhysics.ArticulationRootAPI`
- Fixed joints
- Prismatic joints
- Joint drives
- Original articulated hierarchy

It also does not call Isaac Sim's `RobotAssembler`.

Although `utils/teaching.py` contains a helper named `remove_physics_attributes()`, the main ghost creation path does not need to invoke it because it constructs a fresh mesh instead of cloning the physics hierarchy.

### Result

```text
/World
  |
  +-- Robot
  |    +-- Robot articulation and physics links
  |    +-- Attached original gripper
  |         +-- Gripper rigid bodies
  |         +-- Finger joints
  |         +-- TCP
  |
  +-- poses
       +-- Gripper_Pose
            +-- UsdGeom.Mesh
            +-- GhostObjectAPI
            +-- GhostMaterial
            +-- SourceTcp relationship
```

The exact destination hierarchy depends on the source tool's parent.

The ghost can be translated and rotated freely in the Stage because it is a visual pose target, not a simulated rigid body.

## 6. How the Robot Preview Is Connected

**Relevant file:** `ui/overlay/ghost_teaching/ghost_teaching_overlay.py`

When the user selects or moves a ghost:

1. Wandelbots identifies the linked TCP through `GhostObjectAPI`.
2. It resolves the corresponding robot motion group.
3. It interprets the ghost's transform as a desired tool/TCP pose.
4. It requests inverse-kinematics solutions from NOVA.
5. It draws possible robot configurations in a viewport overlay.

The preview robot is also **not another USD articulation**.

Its geometry comes from:

```python
wb.MotionGroupModelsApi(
    client
).get_motion_group_collision_model(
    motion_group_model=model_name
)
```

Wandelbots converts NOVA collision hulls into `omni.ui.scene` rendering primitives and positions them using forward kinematics.

This is separate from the USD ghost object's visual mesh.

## 7. Implementation Guidance for Our Isaac Sim Project

To reproduce Wandelbots-style ghosts:

1. Accept an existing gripper or tool prim.
2. Identify a TCP or reference-frame prim.
3. Create a separate `UsdGeom.Mesh` or visual-only hierarchy.
4. Read the tool's visual mesh geometry.
5. Bake source transforms into TCP-relative coordinates.
6. Apply a translucent material.
7. Position the ghost at the desired TCP world pose.
8. Keep the original robot/gripper articulation untouched.
9. Avoid adding physics schemas or joints to the ghost.
10. Optionally implement caching and asynchronous mesh refinement.

### Important architectural recommendation

Start with a simple implementation:

- Copy and merge the original visual geometry.
- Apply transforms correctly.
- Assign a translucent USD material.
- Keep the result physics-free.

Do **not** implement CoACD, watertight Boolean unions, or background subprocesses unless the application actually needs high-quality merged ghost geometry.

Those are optimizations, not prerequisites for ghost visualization.

## 8. Exact Source Files

Repository path prefix:

`packages/linux_x86_64/cp311/exts/wandelbots.omni/wandelbots/omni/`

| File | Purpose |
|---|---|
| `ui/schema/schema_extension_ui.py` | Registers the right-click Ghost Object action |
| `ui/schema/schema_components.py` | Validates the tool and dispatches ghost creation |
| `utils/teaching.py` | Creates the ghost USD mesh, applies material, links TCP |
| `utils/mesh.py` | Reads and processes source geometry |
| `utils/watertight_mesh.py` | Refines and merges geometry |
| `utils/ghost_mesh_cache.py` | Caches generated ghost meshes |
| `assets/wb_ghost.mdl` | Custom translucent ghost shader (located under the same extension assets tree) |
| `ui/overlay/ghost_teaching/ghost_teaching_overlay.py` | Computes/displays robot IK previews |
| `ui/overlay/manipulators/motion_group_mesh.py` | Retrieves NOVA robot collision geometry |

### Direct Links

- Context-menu registration: https://github.com/wandelbotsgmbh/wandelbots-isaacsim-extension/blob/main/packages/linux_x86_64/cp311/exts/wandelbots.omni/wandelbots/omni/ui/schema/schema_extension_ui.py#L95-L109
- Ghost creation handler: https://github.com/wandelbotsgmbh/wandelbots-isaacsim-extension/blob/main/packages/linux_x86_64/cp311/exts/wandelbots.omni/wandelbots/omni/ui/schema/schema_components.py#L233-L295
- Core ghost conversion: https://github.com/wandelbotsgmbh/wandelbots-isaacsim-extension/blob/main/packages/linux_x86_64/cp311/exts/wandelbots.omni/wandelbots/omni/utils/teaching.py#L440-L500
- Mesh processing: https://github.com/wandelbotsgmbh/wandelbots-isaacsim-extension/blob/main/packages/linux_x86_64/cp311/exts/wandelbots.omni/wandelbots/omni/utils/mesh.py
- Ghost teaching overlay: https://github.com/wandelbotsgmbh/wandelbots-isaacsim-extension/blob/main/packages/linux_x86_64/cp311/exts/wandelbots.omni/wandelbots/omni/ui/overlay/ghost_teaching/ghost_teaching_overlay.py

## Final Takeaway

Wandelbots implements the right-click **Ghost Object** action by generating a new visual-only, TCP-aligned mesh from an existing tool/gripper.

It does not duplicate or assemble physics articulations, does not create rigid bodies or joints, and does not change the original gripper's physics.

The movable gripper ghost is a **visual representation of a target tool pose**, while the robot configurations displayed around it are separate, lightweight kinematic overlays.

**Task for coding agent:** Implement an Isaac Sim function that accepts a tool USD prim and TCP prim and creates an independently movable, transparent, physics-free ghost representation, while preserving the original tool and robot articulation unchanged.