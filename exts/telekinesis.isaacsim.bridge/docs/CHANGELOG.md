# Changelog

## [0.4.0] - 2026-08-18
### Changed
- **Breaking:** the camera image routes (`POST /cameras/{id}/capture`, `GET /cameras/{id}/rgb`,
  `.../rgba`, `.../depth`, `.../pointcloud`) answer `application/octet-stream` instead of JSON. One
  response is a binary frame: a `TKB1` magic, a length-prefixed JSON manifest naming each array's
  shape, dtype and byte range, then the arrays' raw bytes. Every other camera route stays JSON, and
  errors still answer `{"detail": ...}`, so a client picks how to read a response by its content
  type. `docs/README.md` documents the layout and `examples/cameras/capture.py` has a decoder.
  Encoding one 1280x720 RGB frame went from 4.8 s and 58 MB to 0.45 ms and 2.8 MB; because the
  bridge serves requests on Isaac Sim's own main-thread loop, that time was time in which nothing
  rendered or stepped, so a capture no longer stalls the simulation.
- Depth and pointcloud values that are `inf` now arrive as `inf` rather than `null`. A depth pixel
  that hit nothing was indistinguishable from missing data before, because JSON has no `inf`.
- **Breaking:** `PUT /cameras` no longer accepts `data_types`. Render outputs are activated on
  demand: a new camera produces `rgb`/`rgba`, and asking `POST /cameras/{id}/capture` for any other
  supported output activates it, at the cost of a few frames of annotator warmup on that one call.
  Every attached annotator costs a render pass per frame for as long as it stays attached, so this
  stops a camera paying for outputs nobody reads. The camera's response reports `active_data_types`
  (produced now) and `supported_data_types` (everything `capture` accepts) in place of
  `data_types`.
- **Breaking:** a pointcloud now comes back relative to the camera unless `world_frame` is set,
  matching the frame a physical depth camera reports in. `POST /cameras/{id}/capture` accepts
  `world_frame` for the same reason, and `GET /cameras/{id}/pointcloud?world_frame` defaults to
  false rather than true.
- `POST /cameras/{id}/capture` answers 422 when an output it just activated never produced data,
  mirroring a failed bind.

### Fixed
- `PUT /cameras` for a prim that is already registered now keeps and reconfigures the camera
  bound to it instead of building a second one. Isaac Sim hands a second camera over the same
  prim the same render product, so freeing the first one afterwards took the second one's
  annotators with it and every following capture answered `Annotator rgb is not attached to any
  render products.` Connecting to the same simulated camera again -- a second client session, or
  a second run of a script -- hit this on its first capture.
- An empty pointcloud reports shape `(0, 3)` float32, the shape and type a pointcloud has. Isaac Sim
  answers a flat empty array whenever it has no points to give, whatever the reason, so a client had
  to special-case a `(0,)` float64 result.
- Activating an output whose first result is legitimately empty no longer spends the whole warmup
  budget waiting for it, which delayed that capture by 60 render frames and filled the log with
  Isaac Sim's "a few render frames may be required" warning. Warmup now asks whether the annotator
  has produced a result rather than whether the result has anything in it — for a pointcloud those
  differ, since a camera pointed at empty space produces an empty one every frame.

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
