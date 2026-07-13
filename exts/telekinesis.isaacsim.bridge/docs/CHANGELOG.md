# Changelog

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
