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

## Option A — Extension Manager UI (Recommended)

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
| `articulations/` | `teleport_joint_positions.py` | `POST /articulations/{id}/set_j` |
| `articulations/` | `set_joint_velocities.py` | `POST /articulations/{id}/joint_velocities` |
| `articulations/` | `get_joints_state.py` | `GET /articulations/{id}/joints_state` |
| `articulations/` | `get_dof_limits.py` | `GET /articulations/{id}/dof_limits` |
| `articulations/` | `get_handles_initialized.py` | `GET /articulations/{id}/handles_initialized` |
| `articulations/` | `get_num_bodies.py` | `GET /articulations/{id}/num_bodies` |
| `articulations/` | `get_dof_properties.py` | `GET /articulations/{id}/dof_properties` |
| `articulations/` | `get_dof_index.py` | `GET /articulations/{id}/dof_index/{joint_name}` |
| `articulations/` | `get_applied_joint_efforts.py` | `GET /articulations/{id}/applied_joint_efforts` |
| `articulations/` | `get_measured_joint_forces.py` | `GET /articulations/{id}/measured_joint_forces` |
| `articulations/` | `get_joints_default_state.py` | `GET /articulations/{id}/joints_default_state` |
| `articulations/` | `set_joints_default_state.py` | `PUT /articulations/{id}/joints_default_state` |
| `articulations/` | `get_applied_action.py` | `GET /articulations/{id}/applied_action` |
| `articulations/` | `set_joint_efforts.py` | `POST /articulations/{id}/joint_efforts` |
| `articulations/` | `enable_gravity.py` | `POST /articulations/{id}/enable_gravity` |
| `articulations/` | `disable_gravity.py` | `POST /articulations/{id}/disable_gravity` |
| `articulations/` | `get_world_velocity.py` | `GET /articulations/{id}/world_velocity` |
| `articulations/` | `set_world_velocity.py` | `PUT /articulations/{id}/world_velocity` |
| `articulations/` | `get_linear_velocity.py` | `GET /articulations/{id}/linear_velocity` |
| `articulations/` | `set_linear_velocity.py` | `PUT /articulations/{id}/linear_velocity` |
| `articulations/` | `get_angular_velocity.py` | `GET /articulations/{id}/angular_velocity` |
| `articulations/` | `set_angular_velocity.py` | `PUT /articulations/{id}/angular_velocity` |
| `articulations/` | `get_solver_position_iteration_count.py` | `GET /articulations/{id}/solver/position_iteration_count` |
| `articulations/` | `set_solver_position_iteration_count.py` | `PUT /articulations/{id}/solver/position_iteration_count` |
| `articulations/` | `get_solver_velocity_iteration_count.py` | `GET /articulations/{id}/solver/velocity_iteration_count` |
| `articulations/` | `set_solver_velocity_iteration_count.py` | `PUT /articulations/{id}/solver/velocity_iteration_count` |
| `articulations/` | `get_stabilization_threshold.py` | `GET /articulations/{id}/solver/stabilization_threshold` |
| `articulations/` | `set_stabilization_threshold.py` | `PUT /articulations/{id}/solver/stabilization_threshold` |
| `articulations/` | `get_enabled_self_collisions.py` | `GET /articulations/{id}/enabled_self_collisions` |
| `articulations/` | `set_enabled_self_collisions.py` | `PUT /articulations/{id}/enabled_self_collisions` |
| `articulations/` | `get_sleep_threshold.py` | `GET /articulations/{id}/solver/sleep_threshold` |
| `articulations/` | `set_sleep_threshold.py` | `PUT /articulations/{id}/solver/sleep_threshold` |
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

The full endpoint list is generated from the code, not hand-maintained here -- see
[README.md#api-reference](README.md#api-reference) for the live (`/docs`, `/redoc`) and static
(GitHub Pages) references. What follows is this project's own convention for *which* status
code a given failure gets; it's not derivable from the schema itself.

### Error Handling

Errors always come back as `{"detail": "..."}` with one of the status codes below. Every failure the bridge anticipates raises `fastapi.HTTPException` with one of these codes (at the point in `comm/services/*.py` where the failure is detected); anything unanticipated falls through to a global handler in `comm/server.py` that still returns this same JSON shape at `500`, instead of a bare-text crash.

This follows FastAPI's own conventions rather than a separate error-code standard: `fastapi.HTTPException` is used directly wherever a service detects a specific failure, and `422` is left as FastAPI/pydantic's automatic default for request-body validation errors (missing/wrong-typed fields) rather than overridden. The specific case-to-code choices below (e.g. "no stage open" -> `409`, "bind/import failed" -> `422`) are this app's own convention, informed by common REST practice -- neither RFC 9110 nor the FastAPI docs prescribe a decision rule for picking among codes for a given case, they only define what each code generically means. See [FastAPI's error-handling docs](https://fastapi.tiangolo.com/tutorial/handling-errors/) and [RFC 9110 §15](https://www.rfc-editor.org/rfc/rfc9110.html#name-status-codes) for those definitions.

| Status | When we use it |
|---|---|
| `400` | The client sent a specific value that is invalid  |
| `404` | The referenced resource — an articulation_id or prim_path — is not currently registered/present. Nothing wrong with the request itself, the thing it points at just doesn't exist. |
| `409` | A prerequisite isn't met for this operation right now (no USD stage open) |
| `422` | The request was well-formed but semantically failed -- either FastAPI/pydantic's own automatic validation (missing/wrong-typed field), or an operation that couldn't complete for a deeper reason (a prim that won't bind, a URDF that fails to import) |
| `500` | The global exception-handler backstop for anything not explicitly translated |
| `501` | Routes mirrored from the spec but not wired up yet (`not_implemented()`) |
