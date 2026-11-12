# Teach poses with a synapse server: two plans

## Context
To teach poses in Isaac Sim, the bridge needs IK/FK, and kinematics lives in synapse. We are
starting a NOVA-like **synapse server** that owns kinematics, cells (workspaces for an
application or project) and motion groups (robots plus their TCPs). The bridge becomes its
client.

This slice builds only what is needed to:
- register a robot,
- teach a pose (to a frame, to joint positions, or to a tool placement),
- re-solve the pose when it is edited,
- move the sim robot to it.

The layering (cell → motion group → TCP, a kinematic model per robot type) stays, so later
work can be added rather than redesigned: cell layout, controllers/execution, several IK
solutions, and the applications loading from a cell.

Answers to the open questions in the notes:
- **Synapse robot, or assemble our own model?** Use the synapse robot. Build it through
  `synapse.factory.create_robot` and never call `connect()`. It is only a kinematic model.
- **Does the server register the original robot?** It stores a *motion group record*
  (robot identity + TCPs) in the cell and builds one kinematic model *per robot type*, shared
  by every motion group of that type. It does not create a controller, and it does not connect
  to hardware.
- **Robot deleted in Isaac Sim?** The bridge sends `DELETE` for the motion group, from the
  existing removed-prims listener. If that request is lost, nothing breaks: the record is inert,
  models are per type, and the next registration with the same id overwrites it.
- **ID.** The robot type is the catalogue **bundle** (e.g.
  `urdf/robots/manipulators/abb/crb15000595.zip`). The motion group id is the bridge
  articulation id (`articulation1`, stable across scene reopen). The bundle is exact. Display
  names like "IRB 120-3/0.58" would need fuzzy matching. synapse already derives the same bundle
  from module + class name (`abstract_robot.py:360` `_asset_info`), so the server can index
  `MANIPULATORS` by bundle without importing any driver.
- **Frames.** The client sends poses in the robot's **base frame**, because the bridge reads the
  live USD transforms anyway. The server stores no world transforms, so editing the cell in
  Isaac Sim can never leave it with stale ones.
- **TCP, NOVA-style.** TCPs belong to the motion group: a name, a parent link and an offset.
  The server applies the offset with its own matrix math, so the shared per-type model is never
  mutated (`add_tcp` would mutate it).

---

## Plan A: synapse server (new repo `D:\Telekinesis\Code\telekinesis-synapse-server`)

```
pyproject.toml   # telekinesis-synapse-server 0.1.0: fastapi, uvicorn, pydantic>=2,
                 # telekinesis-synapse, telekinesis-datatypes, numpy, scipy
README.md, CHANGELOG.md (0.1.0 / Added)
src/telekinesis/synapse_server/
  __init__.py
  __main__.py     # python -m telekinesis.synapse_server --cells-dir DIR --host 127.0.0.1 --port 8770
  models.py       # wire + stored models
  cell_store.py   # CellStore
  kinematics.py   # KinematicModelCache
  routers.py      # FastAPI routes, services via Depends (bridge comm/routers.py pattern)
```
The server runs in its own process. The bridge serves on Kit's main loop, so IK must never run
there.

### Models (`models.py`)
- `Tcp`: `parent_frame: str` (URDF link name, e.g. `wrist_3_link`) and `pose: [x,y,z,rx,ry,rz]`
  (m, extrinsic `xyz` degrees, the wire convention).
- `MotionGroup`: `bundle: str` and `tcps: dict[str, Tcp]`.
- `Cell`: `schema_version: int = 1`, `name` and `motion_groups: dict[str, MotionGroup]`.
  Cell layout (frames, tools, cameras) comes later.
- Kinematics requests and responses. Joints are **radians**, sent with `joint_names`, to match
  the bridge's wire units. The server converts to synapse degrees internally.
- Validation at the boundary:
  - Cell and motion-group ids match `^[A-Za-z0-9_-]+$`, which also makes them safe file names.
  - Poses have length 6 and finite values.
  - `schema_version` is 1.
  - `bundle` is in the `MANIPULATORS` bundle index; otherwise the request fails with 422.

### CellStore (`cell_store.py`)
- `CellStore(cells_directory)` keeps one `<cell>.json` per cell.
- Methods: `list_cell_names`, `read_cell` (raises `KeyError`), `write_cell`, `delete_cell`.
- `write_cell` writes a temp file and then calls `os.replace`, so a crash leaves no half-written
  cell. A `threading.Lock` serializes read-modify-write, for example two motion-group PUTs on
  one cell.
- On restart the cells are simply there again. No code path connects to hardware.

### KinematicModelCache (`kinematics.py`)
- `KinematicModelCache()` holds `{bundle: entry}`, where an entry is a synapse robot plus its
  own `threading.Lock`.
- **Creation is single-flight**: it runs under the cache lock, so two concurrent first requests
  build one robot. The cache is built from `MANIPULATORS` without imports: the bundle
  `urdf/robots/manipulators/<module>/<class lower minus prefix>.zip` maps to
  `(module, class)`, using the same rule as `_asset_info`. Construction goes through
  `factory.create_robot(datatypes.ManipulatorConfig(name=bundle, class_name=...))`
  (`synapse/factory.py:20`).
- **No eviction while running**, so no request can use a robot that another one shut down.
  `shutdown_all()` runs from the FastAPI lifespan when the server stops.
- Every IK/FK call on a model holds that model's lock, because one synapse instance is not
  thread-safe.
- `inverse_kinematics(bundle, tcp, base_poses, seed, elbow_up)`:
  - For each pose: `parent_target = base_T_tcp_target @ inv(parent_T_tcp)`.
  - Then `robot.inverse_kinematics(parent_targets, frame_name=tcp.parent_frame, q_inits=seed, elbow_up=elbow_up)`
    (`abstract_robot.py:3959`).
  - It returns radians per pose, or `null`. With no TCP, it solves for the robot's
    `default_tcp`.
- `forward_kinematics(bundle, tcp, joint_positions)` calls `forward_kinematics(q, frame_name=tcp.parent_frame) @ parent_T_tcp`
  (`:4300`).
- Pose ↔ matrix conversion uses `scipy.spatial.transform` with `"xyz"` degrees, in one place.
- Joint mapping: synapse's actuated joint names in model order are returned as `joint_names`.

### Routes (`routers.py`), sync `def` so they run in FastAPI's threadpool
| Method | Path | Meaning |
|---|---|---|
| GET | `/version` | Health and version; the bridge uses it to check the connection |
| GET | `/cells` | Cell names |
| GET / PUT / DELETE | `/cells/{cell}` | Export / create-or-replace (import) / remove |
| GET | `/cells/{cell}/motion-groups` | Motion groups of a cell |
| PUT / DELETE | `/cells/{cell}/motion-groups/{motion_group}` | Register-or-replace / remove (idempotent) |
| POST | `/cells/{cell}/motion-groups/{motion_group}/inverse_kinematics` | `{poses (base frame), tcp?, seed_joint_positions?, elbow_up?}` → `{joint_names, joint_positions: [[rad]\|null]}` |
| POST | `/cells/{cell}/motion-groups/{motion_group}/forward_kinematics` | `{joint_positions, tcp?}` → `{poses}` |

Errors: `KeyError` → 404, validation → 422, synapse `ValueError` (limits, elbow on fewer than
6 axes) → 400 with its message. URDF fetch failure → 503.

---

## Plan B: bridge extension (`exts/telekinesis.isaacsim.bridge/telekinesis/isaacsim/bridge`)

### B1. Robot identity on the stage
Write `telekinesis:articulation:bundle` (string) on the robot prim wherever a robot is loaded
from a catalogue entry: the asset load in `ArticulationService` (~`articulations.py:1204`), and
`ui/add_robot_window.py` `add_selected_asset` if it doesn't go through that path. Robots without
the attribute (scenes made before this change) are not offered in the pose windows, which say
why.

### B2. Multiple named TCPs (NOVA-like)
- `core/robot_assembler.py:436` `place_tcp` replaces only the marker with the same name, instead
  of emptying `<mount link>/tcp/`.
- Add `ArticulationService.list_tcps(id)`. It returns the name, the parent link (the mount link)
  and the offset from `matrix_to_translation_rotation` (`:108`).
- Add `ArticulationService.delete_tcp(id, name)`.
- New routes: `GET /articulations/{id}/tcps` and `DELETE /articulations/{id}/tcps/{name}`.
- This changes the behaviour of `PUT /tcp`, so it goes in the CHANGELOG. synapse's caller
  (`abstract_robot.py:613`) still works, and its earlier TCPs now stay visible.
- Pose windows offer "Flange" (no TCP) plus these markers.

### B3. `comm/synapse_server_client.py` `SynapseServerClient`
- Owned by `BridgeServer`: created in its constructor, started and stopped with it.
- Uses `httpx.AsyncClient`, so it doesn't block the Kit loop. Add `httpx` to `[python.pipapi]`
  in `config/extension.toml`.
- New constants in `global_variables.py`: `SYNAPSE_SERVER_URL = "http://127.0.0.1:8770"` and
  `SYNAPSE_SERVER_CELL = "default"`.
- The `connected: bool` property and `subscribe_connection_changed(callback)` are what the
  windows read.
- A connection-watch task on the Kit loop calls `GET /version` every 2 s. When it connects, it
  sends `PUT /cells/{cell}` if the cell is missing. When it disconnects, it forgets which records
  it has sent. This handles a server started late or restarted with no manual reconnect step.
- `sync_motion_group(articulation_id)` builds `{bundle, tcps}` from the stage. It sends `PUT` only
  when the record differs from the last one sent, and is called before every IK/FK call. That
  keeps TCP edits and reconnects correct with no extra wiring.
- `remove_motion_group(articulation_id)` is called next to `unregister_removed_devices` in
  `BridgeServer._on_stage_objects_changed` (`comm/server.py:219`).
- `inverse_kinematics(...)` and `forward_kinematics(...)` pass through. Failures raise one
  `SynapseServerError` that carries the server's detail.

### B4. `comm/services/poses.py` `PoseService` (teach poses in USD)
- **Pose prim:** an Xform at `<assembly>/poses/<name>`. The `poses` Xform is created if it is
  missing, and the assembly is found with `find_robot_for_prim` / `telekinesis:assemblyRobot`.
  Attributes:
  - `telekinesis:pose:robot`: relationship to the robot prim
  - `telekinesis:pose:tcp`: string, `""` meaning the flange
  - `telekinesis:pose:elbowUp`: bool, authored only when restricted
  - `telekinesis:pose:jointNames` and `telekinesis:pose:jointPositions`: string[] and double[],
    in radians
  - `telekinesis:pose:reachable`: bool

  Deleting the assembly deletes its poses as well.
- **Frame math:** `base_T_target = inv(world_T_base_link) @ world_T_pose`, using USD world
  transforms. The base link comes from `get_articulation_base_link_name` (`robot_assembler.py:63`).
- **Methods:**
  - `create_pose_from_world(robot, name, world_matrix, tcp, elbow_up)` runs IK, seeded with the
    current joints.
  - `create_pose_from_joints(robot, name, joint_positions, tcp)` runs FK and sets the Xform.
  - `move_robot_to_pose(pose_path)` maps the stored joints by name to the driven-joint order
    and calls the existing `ArticulationService.move_j` (`articulations.py:371`).
- **Automatic update:**
  - Extend `_on_stage_objects_changed` to also forward `GetChangedInfoOnlyPaths()` that are
    `xformOp` properties of prims carrying `telekinesis:pose:robot`.
  - For each such pose, `PoseService` keeps one pending task, and the latest change wins, so a
    gizmo drag doesn't flood the server.
  - The task re-runs IK seeded with the stored joints and rewrites `jointPositions` /
    `reachable`.
  - Its own writes touch only attributes, never `xformOp`, so they don't trigger the task again.
  - While disconnected, it marks `reachable = false`, and the reconnect callback re-solves
    every pose.

### B5. Add Pose windows (`ui/add_pose_window.py` + three subclasses)
- **`AddPoseWindow` base** owns the shared flow:
  - a robot dropdown (registered articulations that carry `bundle`), pre-selected from the
    right-clicked prim
  - a TCP dropdown (B2)
  - a **"Restrict elbow up"** checkbox, which sends `elbow_up=True` when ticked and `None`
    otherwise
  - a pose name field
  - a server status line: red "Synapse server not connected" with **Add disabled**, updated
    live through `subscribe_connection_changed`
  - an Add button, which calls `PoseService`, and a status label for errors
- **Abstract hooks.** Each has a docstring stating when it is called and what it returns.
  - `build_pose_inputs()`: build the mode's widgets
  - `async add_pose(robot, tcp, elbow_up, name) -> str`: return the pose path
  - `on_closed()`: release whatever the mode shows
- **Subclasses:**
  - `AddPoseToFrameWindow`: a frame prim field (or the selection); its world pose goes to
    `create_pose_from_world`.
  - `AddPoseToJointPositionsWindow`: one field per joint, pre-filled from `get_joints_state`;
    the values go to `create_pose_from_joints`.
  - `AddPoseToToolPlacementWindow`: a **translucent tool ghost** plus `PlacementGizmo`
    (`ui/placement_gizmo.py`).
    - The ghost is the mounted tool, shown with `AssetGhost.show_stage_prim(tool_prim)` (B6).
    - When the window opens, it reads `mount_in_tcp = world(mount body) * world(tcp).GetInverse()`.
    - On each drag it calls `ghost.place_on_mount(mount_in_tcp * gizmo_world)`
      (`asset_ghost.py:94`), so the TCP lands on the gizmo.
    - Add sends the gizmo pose to `create_pose_from_world`.
- **Menu:** the right-click "Telekinesis Isaac Sim Bridge" context menu (`extension.py:150-183`)
  gets "Add Pose → To Frame / To Joint Positions / To Tool Placement" on robot prims, and
  "Move To Pose" on pose prims. The windows close on stage open/close, as the existing ones do.

### B6. `ui/asset_ghost.py`: `show_stage_prim(prim_path)`
`show(usd_path)` references a file. A mounted tool already lives in the stage, so add a sibling
method that adds an internal reference to the stage prim instead. The ghost root, the material,
`_switch_off_physics` and the not-pickable setup move into one private method that both use. The
tool's fixed mount joint is copied with the reference and disabled by `_switch_off_physics`.

### Docs
Update the bridge README (pose windows, TCP routes, synapse server URL) and the CHANGELOG
(Added: poses, TCP routes, server client; Changed: `PUT /tcp` keeps other TCPs).

---

## Order
A (server) → B1, B2 → B3 → B4 → B6 → B5.

## Verification (you run installs, builds and lints)
**Server**
1. `python -m telekinesis.synapse_server --cells-dir %TEMP%\cells`, then open `/docs`.
2. `PUT /cells/demo`, then `PUT .../motion-groups/articulation1` with a UR10e bundle and one TCP.
   Check that `demo.json` appears.
3. FK at home joints, then IK on that pose seeded at home. FK of the result should match.
4. Fire two IK requests at once for a not-yet-built type. The log should show one URDF fetch.
5. Restart the server. The cell and motion group are still listed, and nothing connects.

**Bridge**
6. With the server off, Add Pose windows show red and Add is disabled. Start the server; within
   2 s they turn ready without reopening.
7. Add a UR10e, set a TCP, and add a pose with each of the three windows. Check the pose prims
   and attributes under `<assembly>/poses`, and that the ghost moves with the gizmo.
8. Drag a pose Xform. `jointPositions` updates once the drag settles; an unreachable spot gives
   `reachable = false`. Run "Move To Pose" and the robot drives there.
9. Delete the robot. The server's motion group is removed, and the next IK for a remaining robot
   still works.
