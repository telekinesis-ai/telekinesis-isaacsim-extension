# Changelog

## [1.5.0] - 2026-07-14
### Added
- ~25 new `/articulations/{id}/...` endpoints covering the rest of Isaac Sim's
  `SingleArticulation` surface: joint efforts (`set_joint_efforts`,
  `applied_joint_efforts`), `measured_joint_forces`, `joints_default_state`
  (get/set), `applied_action`, `enable_gravity`/`disable_gravity`,
  `world_velocity`/`linear_velocity`/`angular_velocity` (get/set, floating-base
  only), PhysX solver tuning (`solver/position_iteration_count`,
  `solver/velocity_iteration_count`, `solver/stabilization_threshold`,
  `solver/sleep_threshold`, `enabled_self_collisions`), plus introspection
  (`handles_initialized`, `num_bodies`, `dof_properties`, `dof_index`).
- One example script per new endpoint under `examples/articulations/`.
- `tests/test_bridge_smoke.py`: a minimal Isaac-Sim-free smoke test suite for
  the request models and the parts of the service layer that don't need omni.
### Changed
- Renamed several endpoints/fields to match Isaac Sim's own method names
  exactly (no back-compat alias): `joint_state`→`joints_state`,
  `joint_limits`→`dof_limits`, `joint_properties`→`dof_properties`,
  `joint_index`→`dof_index`, `joint_forces`→`measured_joint_forces`,
  `default_joint_state`→`joints_default_state`, `body_count`→`num_bodies`,
  `initialized`→`handles_initialized`, solver `*_iterations`→`*_iteration_count`,
  `self_collisions`→`enabled_self_collisions`.
- Split `PUT .../gravity {enabled}` into `POST .../enable_gravity` and
  `POST .../disable_gravity`, matching Isaac Sim's own two-method shape (it has
  no boolean gravity toggle).
- Renamed the bridge's internal `core.articulation.Articulation` class to
  `SingleArticulation`, matching the `isaacsim.core.prims.SingleArticulation`
  handle it wraps.
- Renamed example scripts under `examples/articulations/` to drop the redundant
  `articulation_` prefix, and split each get/set pair into separate files.
- `PUT /prims/poses/default` and `POST /prims/poses/default/reset` now take a
  named `{"prim_path": ...}` body instead of a bare JSON string, for
  consistency with every other body-carrying route.
### Fixed
- The server now returns a JSON body with `detail` for every failure -- a
  global exception handler replaces Starlette's bare-text 500 for anything an
  endpoint doesn't translate itself, and `bind()`/URDF-import failures now
  report the real underlying error instead of a generic message.
- `PUT /articulations` no longer races when two clients register the same prim
  concurrently (was possible to double-allocate an `articulation_id`).
- `assemble_robot` no longer races when two clients assemble the same arm+
  gripper pair concurrently (could previously build a second fixed joint).
- `PUT .../driven_joints` now rejects unknown joint names with 400 instead of
  silently narrowing to fewer DOFs.
- `move_j`/`set_j`/`set_joint_velocities`/`set_joint_efforts` now validate that
  `indices` are in range and return 400 instead of a raw PhysX error.
- The `stream_joint_positions` WebSocket no longer crashes the connection if
  the articulation is deleted while the stream is open.
- `PUT /prims/poses` and `POST /prims/poses/relative` now reject a malformed
  pose array (wrong length) with 400 instead of an unhandled exception.
### Security
- Documented in the README that the bridge has no authentication and is
  intended for a trusted localhost client only.

## [1.4.0] - 2026-07-13
### Fixed
- `SetJointPositionsRequest` (`set_j` body) was missing `indices`, causing an
  `AttributeError` on every `POST /articulations/{id}/set_j` call.
### Changed
- Rename wire fields for consistency with Isaac Sim/ROS naming:
  `positions`→`joint_positions`, `velocities`→`joint_velocities` (requests,
  incl. the `stream_joint_positions` WS frame); `q`→`joint_positions`,
  `dq`→`joint_velocities`, `torque`→`joint_efforts` (responses).
### Added
- `examples/articulations/`, `examples/stage/`, `examples/prims/`,
  `examples/general/`: one script per remaining API endpoint.
- Root `DEVELOPMENT.md`: source-install guide, Isaac Sim/Python compatibility
  table, and a merge-to-main `ruff`/`pylint` check section.

## [1.3.0] - 2026-07-10
### Changed
- Rename `POST /articulations/{id}/joint_positions` to
  `POST /articulations/{id}/move_j` (drive the joints to the target over time;
  keeps the `asynchronous` flag). No alias for the old path.
### Added
- `POST /articulations/{id}/set_j`: teleport the driven joints directly to the
  target (radians). Writes the DOF state immediately, zeros those joints'
  velocities, and retargets the position drive so the controller holds the new pose
  instead of pulling the joints back toward the previous target.
- `WS /articulations/{id}/stream_joint_positions`: stream teleport targets
  (radians) over a WebSocket for fast, high-rate updates. Each JSON frame
  (`{"positions": [...], "indices": [...]?}`) writes the DOF state directly;
  fire-and-forget (no per-frame reply). Adds the `websockets` runtime dependency
  (uvicorn needs it to serve the WebSocket protocol).

## [1.2.0] - 2026-06-25
### Changed
- Collapse the separate robot and gripper devices into one generic `Articulation`
  (`core/articulation.py`); delete `core/robot_articulation.py` and
  `core/gripper_articulation.py`. The bridge is now device-agnostic: it applies a
  joint-position `ArticulationAction` to a chosen joint subset and reports
  reached/stalled, using one blocking detector for both (reach OR stall).
- Replace the `/robot/*` and `/gripper/*` routes with generic articulation routes:
  `POST /articulations/{id}/joint_positions` (with an `asynchronous` flag),
  `GET /articulations/{id}/joint_state`, `GET .../joint_limits`,
  `GET .../driver_joint` (gripper driver discovery), `PUT .../driven_joints`
  (narrow the driven joints), and `POST .../assemble_robot` (assemble a gripper
  onto an arm; moved up from `/robot/attach_tool`). The service records each
  completed assembly so re-assembling the same pair is a no-op.
- `PUT /articulations` no longer takes `device_type`; ids are `articulation{N}`.
- Robot/gripper semantics (fraction<->radians, open/close, client-side "done")
  move to the examples: new `examples/dummy_devices.py` (`DummyRobot`/`DummyGripper`)
  and `examples/robot_async.py` (async vs blocking). No synapse changes.

## [1.1.0] - 2026-06-23
### Changed
- Migrate the bridge transport from a raw asyncio TCP / newline-JSON server to FastAPI
  served by uvicorn on Isaac Sim's existing asyncio loop (still single-threaded, still
  direct articulation access -- no thread marshalling).
- Per-device servers now expose typed REST routes (`POST /move_j`, `GET /state`,
  `POST /gripper/move|open|close`, `GET /gripper/state`) with pydantic models instead of
  a single `{"type": ...}` envelope. Success is HTTP 2xx; failures are 4xx with `{"error"}`.
- Per-device-port topology preserved: `POST /connect` on 8766 still returns a dedicated port.
### Added
- `fastapi`, `uvicorn`, `pydantic` declared via `[python.pipapi]`.

## [1.0.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings


## [0.1.0] - 2026-06-22

### Added

- Initial version of TelekinesisIsaacSimBridge Extension
