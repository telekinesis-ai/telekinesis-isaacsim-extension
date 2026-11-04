# Changelog
## [0.8.1] - 2026-11-02
### Added
- Right-click Create → Telekinesis Isaac Sim Bridge → Add Robot in the Stage panel and viewport
  opens a window to pick a robot (Manufacturer, Model) from the Telekinesis asset library and
  load it under the right-clicked prim. The robot is registered as an articulation, as with
  `PUT /articulations`. When the right-clicked prim is, or lies inside, a rigid body, the
  success message warns that the robot is fixed to that body.
- The Add Robot, Add Tool and Add Mesh windows close when Add is pressed and report progress
  and the outcome as notifications; an error stays until dismissed. Add is refused, with the
  window left open, when no model is selected or the Attaching Prim is not a prim in the stage.
  Every Add creates a new prim: a name already taken gets a numeric suffix.
- `GET /asset_library/{category}` lists the assets under a bucket prefix (e.g.
  `urdf/robots/manipulators`): each manufacturer folder and the model ZIPs in it.
  `POST /asset_library/fetch` downloads an asset bundle into the bridge's local cache once and
  returns its entry with the local path of the bundle's main file (its URDF, or its USD, OBJ,
  GLB or glTF file).
- Add Robot uses a catalogue entry's `default_joint_configuration` (degrees) as the start pose and
  its `simulation_mount_link` as the robot's mount link.
- Right-click Create → Telekinesis Isaac Sim Bridge → Add Tool loads a parallel or suction gripper.
  On a robot, it is loaded beside the robot, outside any rigid body the robot stands in, and
  assembled onto the robot's mount link at the entered offset, as `assemble_robot` does;
  elsewhere it is only loaded. A robot that stores no mount link refuses the add before
  anything is loaded.
- Right-click Create → Telekinesis Isaac Sim Bridge → Add Mesh adds a mesh from `usd/attachments`.
  On a robot, it is placed beside the robot, outside any rigid body the robot stands in, and
  mounted on the robot's mount link at the entered offset, as `mount_prim` does; elsewhere it is
  added under the right-clicked prim as it is. A robot that stores no mount link refuses the
  add before anything is loaded.
- Right-clicking empty space for Add Robot, Add Tool or Add Mesh adds the asset under the
  stage's default prim, or under the stage root when the stage has none.
- `PUT /articulations` takes an optional `mount_link`, stored on the articulation prim as the
  `telekinesis:mountLink` attribute. Without it, nothing is stored and no link is guessed.

### Changed
- `arm_mount_link` is optional on `POST /articulations/{id}/assemble_robot` and
  `POST /articulations/{id}/mount_prim`; omitted, the robot's stored mount link is used, and a
  robot that stores none is refused with 422.
- `PUT /articulations` refuses with 422 to load a robot inside an articulation. A robot loaded
  inside a rigid body keeps that place in the hierarchy, gets `!resetXformStack!` so PhysX
  treats it as standalone, and is fixed to the nearest rigid body with a fixed joint, so it
  moves with that body. It is no longer fixed to the world.
- `PUT /surface_grippers` refuses with 422 to load a gripper inside a rigid body or an
  articulation, where PhysX would not move it with its parent.
- A robot, gripper or prop loaded under a prim other than the stage root or the default prim,
  such as a box, that would land inside one of that prim's colliders is lifted along world +Z to
  stand on it, after the existing +X parking clear of other robots.
- `PUT /prims/physics/rigid_body` makes each mesh collide as its convex hull instead of a
  convex decomposition. Decomposing open, multi-shell CAD meshes left thin hulls that gripper
  fingers were pushed through. A hull is solid, but fills holes and concave pockets.
- `PUT /prims/physics/rigid_body` binds the prop's meshes to a physics material with a static
  and dynamic friction of 1.0 and the `max` friction combine mode. At the PhysX default of 0.5,
  small parts slid out of parallel-jaw grippers.

### Fixed
- `POST /articulations/{id}/mount_prim` mounted a prim with no mass and no colliders, such as a
  camera, as a body PhysX weighs at its 1 kg default with an inertia unrelated to the prim, which
  shook the arm. Such a prim now gets the mass and inertia of a solid box filling its bounds at
  1000 kg/m³; a prim with no extent is refused with 422.
- `PUT /stage/physics/solve_articulation_contact_last` failed on an Isaac Sim whose
  `PhysxSceneAPI` has no `CreateSolveArticulationContactLastAttr`, after already writing the
  setting as a custom attribute.
- Blocking joint moves on prismatic joints (the Robotiq Hand-E fingers) reported `reached`
  while still up to 10 mm from target, so a gripper `close` returned mid-stroke. Reach and
  settle thresholds were radian values applied to metres; prismatic joints now use 0.5 mm
  and 0.5 mm/s.

## [0.7.0] - 2026-10-20
### Added
- `GET /prims/scale` and `PUT /prims/scale` read and set a prim's local scale, keeping its pose.
- `POST /stage/scene/reference` accepts an OBJ, GLB or glTF mesh, converted to USD in metres with
  `omni.kit.asset_converter`. The converted asset carries no colliders or rigid body.
- `PUT /prims/physics/rigid_body {prim_path, mass}` makes a prop one dynamic rigid body that
  collides through convex decompositions of its meshes.
- `DELETE /prims?prim_path=` removes a prim and everything beneath it; a missing path succeeds.

### Fixed
- Pose edits (`PUT /prims/poses`, `POST /prims/poses/relative`, `POST /prims/poses/default/reset`)
  wiped the prim's scale. They now keep it.
- Poses read from a scaled prim, or a prim under a scaled parent, reported a rotation
  distorted by the scale.

## [0.6.0] - 2026-10-19
### Added
- Support for editing physics of the scene

## [0.5.1] - 2026-09-04
### Fixed
- Registering a conveyor that was already registered re-read the belt's velocity attribute,
  which is the same attribute `POST /conveyors/{id}/start` writes to. A belt that had been
  started in reverse therefore came back with its travel direction flipped, and one whose
  velocity had been overwritten with zero could not be registered again at all. The travel
  direction and authored speed are now captured on a belt's first registration in the open
  stage and reused, so only `DELETE /conveyors/{id}` or a stage change reads the belt again.

### Added
- A belt whose velocity reads zero is now registered anyway when a weaker USD layer still
  holds the velocity the scene authored, which is what a zeroing `over` or a session-layer
  edit leaves behind. The layer it was recovered from is logged.
- The refusal for a belt that is not provisioned as a conveyor says how to provision it --
  which attribute to author on which prim, and the Conveyor Belt Utility documentation --
  because a travel direction cannot be inferred: a belt has no natural speed, one end of it
  is not distinguishable from the other, and a curved belt has no single travel axis.

## [0.5.0] - 2026-09-02
### Added
- Conveyor belts are a device registry of their own: `PUT /conveyors {prim_path, cargo_root?}`
  hands back a `conveyor_id`, then `POST /conveyors/{id}/start` and `.../stop` run and stop the
  belt, `GET /conveyors/{id}` reports its speed and whether it is running, and
  `DELETE /conveyors/{id}` unregisters it. A belt runs at a signed speed along the travel direction
  its scene authored, so no direction is sent: a negative velocity reverses it and an omitted one
  runs it at the authored speed. Both authorings are driven -- a `PhysxSurfaceVelocityAPI` on the
  belt, or an `IsaacConveyor` node, whose graph velocity variable is written because the node
  overwrites the belt's own attribute every tick. `cargo_root` names the prim whose sleeping rigid
  bodies each start wakes, without which a belt cannot pick up a box that came to rest while it was
  stopped. Registering a conveyor plays the timeline, like every other device's bind: a belt
  carries nothing while physics is stopped, and `POST .../start` logs a warning if it is issued
  then, since the speed is written but moves nothing until the timeline plays.
- Lightbeam sensors are a device registry of their own: `PUT /lightbeams {prim_path}` hands back a
  `lightbeam_id`, then `GET /lightbeams/{id}/reading` answers
  `{num_rays, broken, beam_hit, linear_depth, hit_pos}`,
  `PATCH /lightbeams/{id}/configuration` sets the beam layout and detection range,
  `POST /lightbeams/{id}/pause` / `.../resume` switch the sensor's computation, and
  `DELETE /lightbeams/{id}` unregisters it. The reading route answers 409 while the timeline is
  stopped, because the sensor's buffers then hold a stale reading that looks like a clear beam.
  Registering a sensor enables it if the scene left it disabled, since a disabled sensor never
  reports a hit.
- `examples/conveyors/` and `examples/lightbeams/`, one runnable script per route plus a
  `smoke_all.py` for each.

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
