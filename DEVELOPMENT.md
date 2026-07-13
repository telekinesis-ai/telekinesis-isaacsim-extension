# Development

How to run the bridge extension from this source tree (your local dev copy) instead
of the published version from the community registry.

## Prerequisites
- It is recommended to create a conda environment.
  ```bash
  conda create -n telekinesis-isaacsim-bridge python=3.12
  ```

- Install NVIDIA Isaac Sim 5.1.0 or later. The required Python version depends on the
  Isaac Sim release, match it before installing. Tested against **5.1.0.0**
  and **6.0.0.1**:

  | Isaac Sim | Python | Install |
  |-----------|--------|---------|
  | 5.1.0.0 | 3.11 | `pip install isaacsim[all,extscache]==5.1.0.0 --extra-index-url https://pypi.nvidia.com` |
  | 6.0.0.1 | 3.12 | `pip install isaacsim[all,extscache]==6.0.0.1 --extra-index-url https://pypi.nvidia.com` |

- The Kit extensions `isaacsim.core.api`, `isaacsim.gui.components`, and
  `omni.kit.uiapp` must be available (they're pulled in automatically as
  dependencies, but if your Isaac Sim install is missing them, enabling
  `telekinesis.isaacsim.bridge` will fail)

The extension lives at:

```
exts/telekinesis.isaacsim.bridge
```

Isaac Sim discovers extensions by **search path**, and the path you register is the
**parent `exts/` directory**, not the extension folder itself.

> Use forward slashes in paths — Kit is unreliable with Windows backslashes.

---

## Option A — Extension Manager UI (quickest)

1. Open Isaac Sim → **Window ▸ Extensions**.
2. Click the hamburger/gear menu (top-left of the Extensions panel) → **Settings**.
3. Under **Extension Search Paths**, click **+** and add the absolute path to this
   repo's `exts` folder, e.g.:

   ```
   c:/Users/<you>/Documents/workspace/telekinesis-isaacsim-extension/exts
   ```

4. Back in the extension list, search for `telekinesis.isaacsim.bridge`. It now
   appears under the **THIRD PARTY / user** extensions.
5. Toggle it **ON**. Tick **Autoload** to enable it on every launch.

Your local copy takes priority over the registry version, so your source edits are
what run. Enable hot-reload (⟳) while developing to pick up changes without a restart.

---

## Option B — Command line (`--ext-folder`)

Launch Isaac Sim pointing at the folder:

```powershell
# from your Isaac Sim install dir
.\isaac-sim.bat --ext-folder "c:/Users/<you>/Documents/workspace/telekinesis-isaacsim-extension/exts" --enable telekinesis.isaacsim.bridge
```

---

## Option C — Persist it in a kit/config file

Add to your app's `.kit` file (or `user.config.json`) so it's always available:

```toml
[settings.app.exts]
folders.'++' = ["c:/Users/<you>/Documents/workspace/telekinesis-isaacsim-extension/exts"]

[dependencies]
"telekinesis.isaacsim.bridge" = {}
```

The `'++'` suffix appends to the existing search paths rather than overwriting them.

---

## Verify

On first enable, Kit auto-installs `fastapi`, `uvicorn`, `pydantic`, and `websockets`
via `pipapi` (needs internet — if you're offline, pre-install
`fastapi uvicorn pydantic websockets` into Isaac's Python instead).

Open a stage and start playing the simulation, then:

```bash
curl http://127.0.0.1:8766/status
# {"status":"OK"}
```

---

## Examples

All examples are in the [`examples/`](examples/) directory and only require the `requests` package (plus `numpy` for unit conversion).

| File | What it demonstrates |
|------|----------------------|
| `robot_set_joint_position.py` | Register a robot, drive it through multiple joint targets (blocking moves) |
| `robot_load_from_urdf.py` | Import a URDF into the stage via the bridge, then move the arm |
| `robot_async_set_joint_position.py` | Send a joint target with `asynchronous=true` and poll for completion client-side |
| `gripper_control.py` | Register a gripper, discover its driver joint, narrow driven joints, open/close |
| `gripper_load_from_urdf.py` | Import a gripper URDF, narrow to driver joint, open/close via joint limits |
| `assemble_robot_and_gripper.py` | Assemble a gripper onto an arm, then drive both through the shared articulation |
| `robot_stream_joint_positions.py` | Stream a joint trajectory over the WebSocket (`stream_joint_positions`) for fast, continuous updates |
| `extension_client.py` | Exercise all implemented General, Stage, and Prims routes end-to-end |

In addition, [`examples/articulations/`](examples/articulations/), [`examples/stage/`](examples/stage/), [`examples/prims/`](examples/prims/), and [`examples/general/`](examples/general/) each hold one small, self-contained script per remaining endpoint in the API Overview below:

| Folder | File | Endpoint |
|--------|------|----------|
| `articulations/` | `put_articulation.py` | `PUT /articulations` |
| `articulations/` | `get_articulations_list.py` | `GET /articulations` |
| `articulations/` | `get_articulation.py` | `GET /articulations/{id}` |
| `articulations/` | `delete_articulation.py` | `DELETE /articulations/{id}` |
| `articulations/` | `articulation_teleport_joint_positions.py` | `POST /articulations/{id}/set_j` |
| `articulations/` | `articulation_set_joint_velocities.py` | `POST /articulations/{id}/joint_velocities` |
| `articulations/` | `articulation_get_joint_state.py` | `GET /articulations/{id}/joint_state` |
| `articulations/` | `articulation_get_joint_limits.py` | `GET /articulations/{id}/joint_limits` |
| `stage/` | `get_stage_scene.py` | `GET /stage/scene` |
| `stage/` | `open_stage_scene.py` | `PUT /stage/scene` |
| `stage/` | `list_stage_motion_groups.py` | `GET /stage/motion-groups` |
| `stage/` | `get_stage_units.py` | `GET /stage/units` |
| `stage/` | `update_stage_units.py` | `PUT /stage/units` |
| `stage/` | `stage_timeline_control.py` | `PATCH /stage/simulation/timeline/{action}` |
| `stage/` | `get_stage_simulation_state.py` | `GET /stage/simulation` |
| `prims/` | `get_prim_pose.py` | `GET /prims/poses` |
| `prims/` | `update_prim_pose.py` | `PUT /prims/poses` |
| `prims/` | `get_prim_relative_pose.py` | `GET /prims/poses/relative` |
| `prims/` | `apply_prim_relative_pose.py` | `POST /prims/poses/relative` |
| `prims/` | `list_prim_default_poses.py` | `GET /prims/poses/default` |
| `prims/` | `assign_prim_default_pose.py` | `PUT /prims/poses/default` |
| `prims/` | `clear_prim_default_poses.py` | `DELETE /prims/poses/default` |
| `prims/` | `reset_prim_to_default_pose.py` | `POST /prims/poses/default/reset` |
| `prims/` | `set_prim_metadata.py` | `PUT /prims/metadata` |
| `prims/` | `remove_prim_metadata.py` | `DELETE /prims/metadata` |
| `prims/` | `set_prim_visibility.py` | `PATCH /prims/visibility` |
| `prims/` | `set_prim_joint_state.py` | `PATCH /prims/physics/joints` |
| `prims/` | `update_prim_colliders.py` | `PATCH /prims/physics/colliders/` |
| `general/` | `get_status.py` | `GET /status` |
| `general/` | `get_version.py` | `GET /version` |

---

## Merge to main

Before opening a PR against `main`, run `ruff` and `pylint` and fix anything they flag.
Both are run with a 100-character line length, matching the convention used throughout
`exts/` and `examples/`.

```bash
pip install ruff pylint
ruff check --line-length 100 .
pylint --max-line-length 100 exts/telekinesis.isaacsim.bridge/telekinesis examples
```

`ruff` covers style, unused imports, and import ordering; `pylint` catches a broader set of
correctness issues. Neither is wired into CI yet — run them locally until they are.

---

## API Overview

All endpoints accept and return JSON. Successful responses use `2xx`; errors use `4xx`/`5xx` with a detail message.

### Articulations

The core resource. One articulation maps to a USD prim path and drives a subset of its joints.

| Method | Path | Description |
|--------|------|-------------|
| `PUT` | `/articulations` | Register (or re-bind) an articulation; optionally imports a URDF |
| `GET` | `/articulations` | List all registered articulation IDs and their prim paths |
| `GET` | `/articulations/{id}` | Get info and current state for one articulation |
| `DELETE` | `/articulations/{id}` | Unregister (USD prim stays in the stage) |
| `POST` | `/articulations/{id}/move_j` | Move to joint targets (radians); blocks until reached or stalled |
| `POST` | `/articulations/{id}/set_j` | Teleport directly to joint targets (radians); immediate, no blocking |
| `WS` | `/articulations/{id}/stream_joint_positions` | Stream teleport targets (radians) over a WebSocket; fire-and-forget, no reply |
| `POST` | `/articulations/{id}/joint_velocities` | Drive the joints at a velocity (rad/s); fire-and-forget, holds until the next call |
| `GET` | `/articulations/{id}/joint_state` | Current positions, velocities, and efforts |
| `GET` | `/articulations/{id}/joint_limits` | Per-joint position limits (radians) |
| `GET` | `/articulations/{id}/driver_joint` | Discover a gripper's single actuated joint |
| `PUT` | `/articulations/{id}/driven_joints` | Narrow which joints this articulation drives |
| `POST` | `/articulations/{id}/assemble_robot` | Attach a gripper articulation to this arm's flange |

#### Joint positions request

```json
{
  "joint_positions": [-1.57, -1.57, 0.0, 0.0, 1.57, 0.0],
  "indices": null,
  "asynchronous": false
}
```

`joint_positions` is in **radians**. `indices` restricts which joints to move (null = all driven joints). When `asynchronous` is false (default), the call blocks until the move completes.

#### Joint positions response (blocking)

```json
{
  "done": true,
  "reached": true,
  "max_error": 0.002,
  "joint_positions": [-1.57, -1.57, 0.0, 0.0, 1.57, 0.0],
  "target": [-1.57, -1.57, 0.0, 0.0, 1.57, 0.0]
}
```

`reached=true` means the arm hit the target within 5 mrad. `reached=false` means it stalled (e.g. joint limit or contact). The server times out after ~30 s (1800 physics frames).

### Stage

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/stage/scene` | URI of the open USD stage |
| `PUT` | `/stage/scene` | Open a USD stage |
| `GET` | `/stage/motion-groups` | Articulation root prims in the stage |
| `GET/PUT` | `/stage/units` | Stage meters-per-unit scale |
| `PATCH` | `/stage/simulation/timeline/{action}` | `play`, `pause`, or `stop` the timeline |
| `GET` | `/stage/simulation` | Current timeline state |

### Prims

| Method | Path | Description |
|--------|------|-------------|
| `GET/PUT` | `/prims/poses` | Get or set a prim's world pose |
| `GET/POST` | `/prims/poses/relative` | Get or apply a relative pose between two prims |
| `GET/PUT/DELETE` | `/prims/poses/default` | List, save, and clear default poses |
| `POST` | `/prims/poses/default/reset` | Restore a prim to its stored default pose |
| `PUT/DELETE` | `/prims/metadata` | Store or remove `{category, type}` metadata on a prim |
| `PATCH` | `/prims/visibility` | Show or hide a prim |
| `PATCH` | `/prims/physics/joints` | Enable or disable a physics joint |
| `PATCH` | `/prims/physics/colliders/` | Enable or disable collision on a prim |

**Pose format:** `[x, y, z, rx, ry, rz]` - position in meters, rotation as axis×angle in radians (rotation-vector). Pass `rotation_type=quaternion` to use `[x, y, z, qw, qx, qy, qz]` instead.

### General

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/status` | Liveness check - returns `{"status": "OK"}` |
| `GET` | `/version` | Installed Kit extension names and versions |
