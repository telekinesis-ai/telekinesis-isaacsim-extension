# Changelog

## [0.3.0] - 2026-08-14
### Added
- Suction (surface) gripper support as its own device registry, mirroring articulations and
  cameras: `PUT`/`GET`/`DELETE /surface_grippers` and `GET /surface_grippers` to register, look up,
  drop and list them.
- `POST /surface_grippers/{id}/close` and `.../open`: actuate the gripper. Both block by default
  and return the settled status, so callers never have to sleep before reading it — the gripper
  acts on the next physics step, and a status read taken straight after the command still reports
  the previous value. `{"asynchronous": true}` issues the command and returns immediately.
- `GET /surface_grippers/{id}/status`: status (`Open`/`Closing`/`Closed`), gripped objects and grip
  distance.
- `GET`/`PATCH /surface_grippers/{id}/properties`: the gripper's force limits, reach, retry
  interval and forward axis. The rotation and translation limits are accepted here too and written
  to every attachment point, since USD stores them per attachment point rather than on the gripper.
- `GET`/`PATCH /surface_grippers/{id}/attachment_points`: per-suction-cup local poses, Z axis
  translation drive gains, rotation and translation limits, clearance offset and forward axis.
- `POST /articulations/{id}/assemble_robot` now accepts a suction gripper. It places the gripper at
  the arm's flange, joins the two with a fixed joint excluded from the articulation, and re-parks
  every attachment point onto the arm's mount link so the gripper can grip. Nothing is merged: the
  arm's DOF are unchanged and the gripper keeps its own id and its own close/open routes. The
  response reports which path was taken as `gripper_kind`, plus the `fixed_joint` it created.

### Changed
- **Breaking:** `POST /articulations/{id}/assemble_robot`'s `gripper_articulation_id` field is now
  `gripper_id`, because it accepts either an `articulation_id` or a `surface_gripper_id`. The old
  name is not accepted; update clients when upgrading.
- `assemble_robot` gained `mask_collisions` (default true), which stops the arm and the gripper
  colliding with each other. It only applies to a suction gripper — an articulated one settles
  collisions through the articulation merge.

### Fixed
- `assemble_robot` with a suction gripper no longer snaps the gripper onto the arm and drags the
  arm with it when the timeline plays. Three things fed into it:
  - The placement and the collision mask were authored through `RobotAssembler`'s variant machinery,
    into a session sublayer that was torn down again, leaving the fixed joint and the attachment
    points baked against a pose the gripper no longer had. The gripper is now placed at the mount
    pose directly, in the stage's own edit layer.
  - Assembly read the arm's pose in the same update that stopped the timeline, before stopping had
    restored the arm's joints, so the gripper was mounted against the last simulated pose and the
    arm then moved out from under it. Assembly now waits for the stop to land.
  - Re-assembling a gripper left the previous assembly's fixed joint in place, so two joints fought
    over the same body. Stale mount joints are now removed first.
- The mount joint's local frames are written from the requested offset rather than measured back off
  the stage, so the joint holds the pose that was asked for regardless of where the arm and the
  gripper stood when it was created.

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
