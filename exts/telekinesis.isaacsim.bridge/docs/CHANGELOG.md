# Changelog

## [0.1.3] - 2026-08-04
### Added
- `GET /articulations/{id}/articulation_state`: every per-frame quantity of one articulation in a
  single snapshot (joint state and efforts, measured joint forces, last applied action, root world
  pose and velocity).
- `ws /articulations/{id}/stream_articulation_state`: the same snapshot pushed once per simulator
  update, so a client can follow an articulation without polling.

### Changed
- `ws /articulations/{id}/stream_joint_positions` now applies only the newest queued frame per
  simulator update instead of every frame received. A client streaming faster than the simulator
  updates gets coarser motion rather than motion that lags behind.

## [0.1.1] - 2026-07-22
### Changed
- Improved the extension and repository README content for better clarity for first-time users and GitHub visitors.

## [0.1.0] - 2026-07-16
### Added
- Initial version of TelekinesisIsaacSimBridge Extension
