# Restore bridge devices from USD when a saved stage is reopened

## Context

The bridge keeps all six device registries (articulation, surface gripper, camera, conveyor,
lidar, lightbeam) only in memory. `Extension._on_bridge_stage_event` → `BridgeServer.reset_devices()`
empties them on every stage OPENED/CLOSED, and `clear()` resets each id counter. After reopening a
saved scene:

- **Nothing is registered.** The list endpoints are empty and the UI windows find no robot. Only a
  `telekinesis:mountLink` arm gets auto-registered, inside `find_robot_for_prim`.
- **Ids restart at 1 in PUT order.** An id a client still holds (Synapse C++ `articulation_id_`,
  `_articulation_id`, `_surface_gripper_id`) returns 404, or now points at another prim.
- **A merged robot+tool comes back wrong.** The merge itself is rebuilt from the stage:
  `_find_arm_merged_into` reads the `AssemblerFixedJoint` that `RobotAssembler` saved under the
  gripper. Three things are lost:
  - The gripper's driven-joint narrowing. It comes back driving all its joints, mimics included,
    instead of `[driver]`.
  - The arm's explicit joint list.
  - Order: a gripper PUT first fails with 422 when the arm has no `mountLink`.
- **Surface-gripper merges are never detected.** `assemble_robot` re-runs the whole attach.
- **Sensor settings are lost:**
  - Camera resolution and frequency.
  - Conveyor `cargo_root` and its captured nominal velocity. A belt saved running reversed comes back
    flipped.
  - Lidar `data_types`.
  - A re-PUT of a lidar overwrites its saved config with request defaults, and a re-PUT of a
    lightbeam un-pauses it.

Goal: the stage itself records which prims are bridge devices, under which id, with which settings
memory alone holds. On stage open the bridge rebuilds every registry from that record, merges
included, before any client connects.

Constraints:
- Synapse keeps working unchanged. It always PUTs by prim path on connect, relies on "PUT of a
  registered prim returns its id", and narrows the gripper itself.
- The in-construction UI (Add Robot / Tool / Mesh / Camera) keeps working.
- Scenes saved before this change still load as they do today.

### Answers to the questions asked

**Is a merged robot+tool recreated correctly?** Not today, as listed above. With this plan, yes:
- Each device's driven joints are saved, and the arm-before-gripper order is fixed.
- The merge is rebuilt through the same `_bind_assembly` that `assemble_robot` uses. That means one
  shared `isaacsim.core.prims.SingleArticulation` rooted at the arm, with each device resolving its
  saved joint names to indices in the merged DOF order (`core/articulation.py:291-334`).
- The `_assemblies` record is rebuilt too, for both kinds of gripper.

**Isaac Sim's robot schema (checked locally).** I checked the 5.1 source build at `D:\isaacsim`, the
pip 6.0.1 install and the pip 6.2 install. `isaacsim.robot.schema` is codeless, and the URDF and MJCF
importers apply it on every import. It defines:
- `IsaacRobotAPI` (`isaac:robotType`, `isaac:physics:robotLinks` / `robotJoints`)
- `IsaacLinkAPI` / `IsaacJointAPI`
- `IsaacSiteAPI` (named `IsaacReferencePointAPI` in 5.1)
- `IsaacNamedPose` (6.x only)
- the typed `IsaacSurfaceGripper`, which the bridge already uses

It describes the robot. It holds nothing about bridge state: device ids, driven-joint subsets,
sensor stream settings. Isaac's `RobotAssembler` nests the gripper in the arm's `robotLinks`, but
very likely only in its session sublayer. Its attribute names also differ between 5.1 and 6.x, and
DEVELOPMENT.md supports both versions. So:
- **Robot description:** leave it to Isaac's schema. Don't duplicate it, and don't make restore
  depend on it.
- **Bridge state:** keep it in `telekinesis:` attributes, the pattern `telekinesis:mountLink`
  already uses.
- **Why not our own codeless schema:** a codeless schema must be registered with Plug before the
  schema registry is first built, and the bridge can be enabled after a stage is already open. Plain
  attributes have no such constraint.
- **Wandelbots:** used for ideas only. It follows the same "mark the prim, rediscover on load"
  pattern.

**Why "Play, bind, then Stop" at open is the right call.** Stopping the timeline kills every physics
handle (that is why `rebind_stale_devices` exists), so the devices end up stale rather than live.
That is exactly the state every existing flow already handles after a user presses Stop:
- `create_*` re-binds.
- assemble, Add Camera and prim delete call `rebind_stale_devices`.
- The cached `dof_names` / `num_dof` that `assemble_robot` and `mount_prim` read stay filled in.

So restored devices satisfy the invariant every registered device has today: it was bound at least
once. The bind also checks each restored prim and logs one that fails. Stop resets the physics
transforms to their authored values, so the scene is left unchanged.

## Design

### Saved in USD on the device prim

The device prim is the requested prim, i.e. the `_id_by_prim` key, the path clients PUT. Restore
uses that same path, so a later PUT finds the restored id.

| Attribute | Type | Written by |
|---|---|---|
| `telekinesis:<kind>:id` (e.g. `telekinesis:articulation:id`, `telekinesis:surface_gripper:id`) | string | every `create_*`; removed by `delete_*` |
| `telekinesis:articulation:drivenJoints` | string[] | `set_driven_joints`; the assembly steps (arm and gripper names); removed when reset to own joints |
| `telekinesis:camera:resolution` / `telekinesis:camera:frequency` | int2 / float (absent = `None`) | `create_camera`, `set_resolution`, `set_frequency` |
| `telekinesis:conveyor:cargoRoot` | relationship | `create_conveyor` |
| conveyor's captured nominal drive (direction + nominal speed, from `core/conveyor.py:116-130`) | float attrs | first bind of a conveyor |
| `telekinesis:lidar:dataTypes` | token[] | `create_lidar` / data-type changes |

`telekinesis:mountLink`, `telekinesis:assemblyRobot` and the customData keys are kept as they are.

### `DeviceRegistry` base class (new, `comm/services/device_registry.py`)

It owns the flow all six services repeat today:
- `_devices`, `_id_by_prim`, `_count`, per-prim `_create_locks`
- `clear()`, `list_*`, `get_device` (404 text built from the kind), unregister on delete
- the new parts: stable id allocation, the id marker, and the restore loop

Class attribute `device_kind` (`"articulation"`, `"surface_gripper"`, `"camera"`, `"conveyor"`,
`"lidar"`, `"lightbeam"`) gives both the id prefix and the marker name. Every method gets a docstring
stating what it does, when it is called and what it returns.

Concrete methods:

- **`allocate_device_id(prim_path)`**, called inside the create lock before the bind. The id is:
  1. the registered id, if the prim is already registered;
  2. else the prim's stored marker, when no other prim in this stage holds it (a copy-pasted prim
     carries its source's id and gets a fresh one);
  3. else `<kind><_count+1>`.

  The id is reserved synchronously, as today.
- **`commit_device(prim_path, device_id, device)`** registers the device and writes the id marker.
  It runs only after a successful bind, keeping today's "a failed create registers nothing".
  Writing the marker on a PUT of an existing prim dirties the stage; that is expected.
- **`unregister_device(device_id)`** drops the table entries and the lock and removes the marker.
  Used by every `delete_*`.
- **`async restore_from_stage(stage)`**:
  - Walks `Usd.PrimRange` for prims with the kind's id marker, ordered by id number.
  - Sets `_count` to the highest stored number, including prims that fail to restore, so they can
    never be handed a duplicate.
  - For each prim, inside its create lock, `await self.restore_device(prim_path, device_id)` and
    commits the result.
  - A failure is logged with `carb.log_warn` and skipped, leaving the marker so a later PUT keeps
    the id.

Abstract hooks, each with a docstring stating its contract:
- **`async restore_device(prim_path, device_id) -> device`**: build and bind the device for a
  marked prim from what the prim stores, without changing its saved configuration. Called once per
  marked prim on stage open, with the timeline playing. Raises `RuntimeError` / `ValueError` when it
  cannot be bound.
- **`release_device(device)`**: free what the device holds when it is dropped (the existing
  `_safe_destroy` / `stop_velocity_drive`).

### Per service

The `create_*` request semantics, status codes and response shapes stay as they are. Each create's
own id/lock/commit code is swapped for the base calls.

- **Articulation (`comm/services/articulations.py`):**
  - `restore_device` builds `SingleArticulation(prim, name=id, joint_names=<saved drivenJoints>)`.
    The constructor already takes `joint_names`.
  - `restore_from_stage` overrides the base:
    1. Restore non-merged articulations first, with `bind()`.
    2. Then devices whose prim holds a merge. Found with `_find_arm_merged_into`, generalized so it
       also serves a surface gripper prim and never auto-registers during restore.
    3. Bind each merged pair through `_bind_assembly(arm, gripper, arm.joint_names,
       gripper.joint_names)`.
    4. Rebuild `_assemblies` for both `articulated` and `surface` merges. The `fixed_joint` is the
       joint found.
  - `set_driven_joints` and `_merge_articulated_gripper` / `_attach_surface_gripper` write
    `drivenJoints`.
- **Surface gripper:**
  - `restore_device` is `SurfaceGripper(prim)` plus `bind()`.
  - Restored before articulations, so the articulation restore can link surface merges.
- **Camera:** `restore_device` applies the saved resolution and frequency through the existing
  `_configure`. The paused state stays in memory only; restored cameras come back running.
- **Conveyor:** `restore_device` passes the saved `cargo_root` and the saved nominal drive (the
  constructor already takes `authored`). Without them, it re-captures the drive from the current
  value.
- **Lidar:** `restore_device` reads the config the prim already stores instead of the request
  defaults, plus the saved `dataTypes`.
- **Lightbeam:** `restore_device` keeps the saved `enabled` state. Construction re-enables the
  sensor (`core/lightbeam.py:123-125`), so it pauses the sensor again after the bind when it was
  saved disabled.

### Wiring

- `BridgeServer.restore_devices()` (`comm/server.py`):
  1. `await` surface grippers, then articulations, then the four sensor services.
  2. Then stop the timeline once.
  3. Log a one-line summary.
- `Extension._on_bridge_stage_event` (`extension.py:268`):
  - OPENED: `reset_devices()` then `asyncio.ensure_future(self._bridge_server.restore_devices())`.
  - CLOSED: unchanged.

  A client PUT that arrives during the restore waits on the same per-prim lock, and the stable id
  makes the result the same either way.

### Compatibility

- **Synapse:** no change needed. PUT by path returns the restored id. `set_driven_joints` and
  `assemble_robot` stay idempotent: the latter now returns `already_assembled` for both gripper kinds
  after a reopen. Ids held across a reopen keep pointing at the same prim.
- **Scenes saved before this change:** they carry no markers. Restore finds nothing and behavior is
  exactly today's, including the `mountLink` auto-register.
- **UI:** list endpoints and `find_robot_for_prim` see restored devices, so opening Add Tool / Add
  Mesh on a reopened robot no longer auto-registers and plays the timeline.

### Out of scope (named, not done)

- Rebinding stale devices automatically when the user presses Play by hand. This gap exists today
  too.
- Registering cameras added by Add Camera (TODO.md).
- Applying `IsaacSiteAPI` to TCP prims.
- Camera paused state.

## Docs

- Docstrings for the base class and every changed service method describe the saved attributes and
  restore behavior.
- `docs/CHANGELOG.md`, under `[0.9.1]`:
  - **Added:** devices restored on stage open, stable ids.
  - **Fixed:** merged gripper driven joints, surface merge detection, lidar/lightbeam re-PUT
    side effects on restore.
- `docs/README.md`: replace "clients re-PUT after a stage change" with the restore behavior.

## Verification

The user runs the builds and lints. I don't run tests as the implementation agent. Manual
end-to-end checks in Isaac Sim, on 5.1 and 6.0.1:

1. `examples/assemble_robot_and_gripper.py`, then save and reopen:
   - `GET /articulations` lists the same ids.
   - The arm's `move_j` moves only arm joints.
   - The gripper drives only its driver joint.
   - `assemble_robot` again returns `already_assembled`.
2. The same with `examples/assemble_robot_and_suction_gripper.py`: the surface record is restored,
   and close/open works.
3. Synapse robot + Robotiq connect against the reopened scene: same ids, no errors.
4. Add Robot + Add Tool in the UI, then save and reopen. Add Mesh on the robot works without
   re-registering.
5. Camera with a non-default resolution, a lidar with custom config, a paused lightbeam, and a
   conveyor running reversed: save, reopen, and all four come back as they were saved.
6. A scene saved before this change opens with an empty registry and behaves as today.
7. Copy-paste a registered robot, save and reopen: the two prims get distinct ids.
