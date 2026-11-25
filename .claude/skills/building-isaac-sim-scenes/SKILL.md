---
name: building-isaac-sim-scenes
description: Builds Isaac Sim robot-cell scenes (USD) from a user's description by writing a Python builder script that assembles Telekinesis asset-library robots, grippers, couplers, cameras, tables, pallets and parts, tags them for the Telekinesis bridge, and validates the result offline. Use when the user describes a scene, cell or workstation to create, asks to place or mount robots, tools, cameras or objects in a USD stage from code, or wants a scene built without the Isaac Sim GUI.
---

# Building Isaac Sim scenes

The user describes a scene. You write a self-contained builder script that writes a
`.usda` the user opens in Isaac Sim with the Telekinesis bridge enabled. The bridge then
registers the robot, gripper and cameras by the tags the script wrote.
`examples/scenes/pick_and_place_scene.py` is a finished example.

Precision comes from one rule: **every prim path, link name, joint name, size and height
comes from a script's output, never from memory or a guess.** Asset root prims are
not named after their files (the 1400 x 800 table's root is `/TableAsset`), and a UR
arm reaches along its own -X at zero pan.

Requires Python with `usd-core` (the Telekinesis SDK environment has it). Isaac Sim does
not need to run. Run the scripts below from this skill's folder or by full path.

## Workflow

Copy this checklist into your reply and tick it off:

```
- [ ] 1. Scene spec written and assumptions listed
- [ ] 2. Assets found
- [ ] 3. Assets inspected
- [ ] 4. Layout computed
- [ ] 5. Builder written from the template
- [ ] 6. Built and validated with no errors; layout checked against the spec
- [ ] 7. Reported
```

### 1. Write the scene spec

Restate the description as a table: each object, its asset (or primitive), where it
goes, what it rests on or is mounted to, and its role (robot, tool, part, fixture,
camera, dressing). Fill gaps with defaults and list each default as an assumption:

- Environment: none (empty stage with floor, lights, physics scene).
- Robot: UR10e. Tool: Robotiq Hand-E on the Robotiq UR coupler, as in the quickstart.
- Robot faces +X; "left" is +Y.
- Parts: 50 mm boxes of 0.2 kg, friction 0.8.
- Output: `examples/scenes/<scene_name>_scene.py`, writing `<scene_name>_scene.usda` beside it.

Ask the user (all questions at once) only when a missing answer changes what gets built
and no default fits, e.g. "a welding cell" with no robot payload or part size.

### 2. Find assets

```
python scripts/find_assets.py <words>              # e.g. ur10e, robotiq, pallet
python scripts/find_assets.py --category usd/tools
python scripts/find_assets.py --list-categories
```

Prints each asset's key (path on the asset server) and, for arms, the mount link, the
Synapse class name and the default TCP to tag. Use `usd/` keys. Environment scenes are under
`usd/environments`. If nothing fits, build the object from primitives (`add_box` in
the template) and say so. NVIDIA SimReady assets (pallets, containers) are another source:
see `build_euro_pallet_cell_scene.py` in the applications repo.

### 3. Inspect every asset

```
python scripts/inspect_asset.py <key>                       # robot, prop, scene
python scripts/inspect_asset.py <gripper key> --frame base_link
python scripts/inspect_asset.py <environment key> --depth 3
```

Read from the output: the main file and the root prim to reference, units and root
transform ops, bounds, articulation roots, rigid bodies, colliders, joints (driver vs
`MIMIC`), world welds, `tool0`/flange frames, surface grippers and cameras.

### 4. Compute the layout

Turn every placement into arithmetic on inspected numbers. Rules for "on", "next to",
"facing", rows and grids, reach limits, and the known environment prims are in
[references/layout-and-cameras.md](references/layout-and-cameras.md). Check reach before
writing code. If the described layout is out of reach, tell the user.

### 5. Write the builder from the template

Copy [assets/scene_template.py](assets/scene_template.py) to the output path. It is a
runnable builder: table, UR10e at a home pose, coupler, Hand-E, three parts, one bridge
camera. Then:

- Replace the module docstring and the layout constants. Delete what the scene does not
  need. Keep the helper functions as they are. They mirror the extension's Add Robot,
  Add Mesh and Add Tool.
- For an environment scene, set `ENVIRONMENT_KEY` and `ENVIRONMENT_FILE_NAME`. Put the
  robot where the environment provides a mount.
- Several robots: one assembly Xform per robot, each with its own tags.
- Suction grippers, wrist cameras, conveyors, mounted prims: follow
  [references/bridge-tags-and-assembly.md](references/bridge-tags-and-assembly.md).
- Containers, stacks, pallets, grasped parts, drive tuning: read
  [references/physics-pitfalls.md](references/physics-pitfalls.md) first.
- Follow the repository's code conventions: descriptive names, docstrings, a comment
  for each non-obvious number saying where it comes from.

### 6. Build and validate

```
python <builder>.py
python scripts/validate_scene.py <builder>.usda
```

Fix every error and rerun until there are none. Errors mean the scene opens wrong or the
bridge cannot restore a device. For each warning, fix it or explain why it is harmless.
A part inside a concave tray can trip the bounds-based overlap test.

Then compare the printed layout with the spec, object by object:

- Every object is where the spec says, within millimetres, and rests on its support:
  its min z equals the support's top.
- Nothing overhangs a support or pierces another object.
- The robot's tool (gripper bounds) is above and toward the work area.
- Every camera's position and view direction point at what it should watch.

Fix mismatches in the constants and rebuild. Do not open Isaac Sim, run simulations or
tune gains yourself.

### 7. Report

Tell the user:

- The builder and scene paths, and the one-line command to rebuild.
- What is in the scene: a short table of objects and poses.
- Every assumption from step 1, and every warning left with its reason.
- How to use it: open the `.usda` in Isaac Sim with the bridge enabled. The bridge
  registers the tagged devices. The scene links the asset cache by absolute path, so it
  opens only on the machine that built it.

## Rules that are easy to get wrong

- Write tag ids as `""`. The bridge assigns `<kind><n>` when the scene opens.
- A camera device needs `telekinesis:camera:resolution` (int2), or it is not restored.
- A gripper merged into an arm keeps no `ArticulationRootAPI` and no enabled world weld.
  Its `AssemblerFixedJoint` sits beneath its base link.
- `FilteredPairsAPI` goes on the mounted prim, never on the arm.
- Set the home pose before mounting tools. Tools are placed from the mount link's pose.
- `ClearXformOpOrder()` before posing a referenced asset. `place_prim_at` does it. Re-add
  a scale the asset needs.
- Rotations are extrinsic XYZ Euler degrees, built with `pose_matrix`.
