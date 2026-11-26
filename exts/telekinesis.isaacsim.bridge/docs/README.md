# Telekinesis Isaac Sim Bridge

Telekinesis Isaac Sim Bridge lets you connect NVIDIA Isaac Sim to local applications over HTTP and WebSocket. It is designed for developers who want to inspect articulation state, send commands, and stream real-time data without writing custom simulation-side integrations.

## What this extension provides

- Register and interact with any Isaac Sim articulation
- Register and actuate suction (surface) grippers
- Start and stop conveyor belts, and read lightbeam sensors
- Attach a gripper to an arm, articulated or suction
- Send motion commands and read articulation state
- Teach robot poses, solved by the Telekinesis Isaac Sim Server
- Stream updates in real time over WebSocket
- Explore the API interactively through Swagger UI

## Run the bridge

The bridge runs at `http://127.0.0.1:8766`.

It is localhost-only, uses no authentication, and is intended for trusted local clients.

## Enable the extension

Enable the extension under **Window ▸ Extensions** if it is not already enabled.

## Quick start

1. Add a Universal Robots UR10e to the stage from the Isaac Sim asset store or import urdf.
2. Note its prim path in the Stage panel (for example, `/World/ur10e`).
3. Install the Python dependencies you need before running examples:

```bash
pip install -r requirements.txt
```

4. Run the following example from a local Python client:

```python
import requests

base = "http://127.0.0.1:8766"
robot = requests.put(f"{base}/articulations", json={"prim_path": "/World/ur10e"}).json()

requests.post(
    f"{base}/articulations/{robot['articulation_id']}/move_j",
    json={"joint_positions": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]},
)
```

## API reference

The endpoint list is generated from the implementation and is kept up to date automatically:

- **Bridge running:** open `http://127.0.0.1:8766/docs` (Swagger UI) or `/redoc`
- **Without Isaac Sim running:** visit the [API reference](https://telekinesis-ai.github.io/telekinesis-isaacsim-extension/)

Use **radians** for joint values and **meters** for length-related values. Angular limits on a
suction gripper's attachment points are the exception: those are **degrees**, the native unit of
`UsdPhysics` limits.

### Camera image payloads

Five camera routes answer `application/octet-stream` instead of JSON, because a 1280x720 RGB frame
is 2.8 MB of raw bytes but 58 MB as JSON nested lists, and building those lists takes seconds on the
same thread that renders and steps physics:

- `POST /cameras/{id}/capture`
- `GET /cameras/{id}/rgb`, `.../rgba`, `.../depth`, `.../pointcloud`

Every other camera route — optics, poses, the registry — is unchanged JSON.

One response is one self-contained frame:

| offset | size | contents |
| --- | --- | --- |
| 0 | 4 | magic `TKB1`, ASCII — identifies the format and its version |
| 4 | 4 | manifest length `N`, uint32 little-endian |
| 8 | `N` | manifest, UTF-8 JSON |
| `8+N` | rest | every array's raw bytes, concatenated |

```json
{
  "structure": {"rgb": {"__ndarray__": 0}, "rendering_frame": 412, "timestamp": 12.35},
  "arrays": [{"shape": [720, 1280, 3], "dtype": "uint8", "offset": 0, "nbytes": 2764800}]
}
```

`structure` is the body the route would otherwise have returned as JSON, with every array replaced
by `{"__ndarray__": <index into arrays>}`. Nesting is preserved, so an output that is itself a dict
(`semantic_segmentation` is `{"data": ..., "info": {...}}`) needs no special handling, and an output
that is not ready is still `null`. Each `arrays` entry gives one array's slice of the array region,
with `offset` counted from `8+N` rather than from the start of the frame. Arrays are C-contiguous in
the host's native byte order, so a client reinterprets the bytes in place — no copy, and the arrays
a decoder hands back are read-only views into the response body. `dtype` is a numpy dtype name; for
the record-dtype outputs (`occlusion`, the `bounding_box_*` family) it is numpy's list of field
descriptors instead, since a name does not describe those.

Depth is float32 in stage units and pixels that hit nothing are `inf` — the JSON responses reported
those as `null`, since JSON has no `inf`.

A pointcloud carries only the pixels that hit geometry, so it holds at most one point per pixel and
shrinks as more of the image is background. A camera whose view holds no geometry reports `(0, 3)`:
an empty pointcloud, not a missing one.

Errors keep the `{"detail": ...}` JSON contract on these routes too, so read a response by its
content type rather than by its status alone. `examples/cameras/capture.py` has a decoder to copy.

### Suction (surface) grippers

A suction gripper is a registered device with an id, like an articulation or a camera:
`PUT /surface_grippers {prim_path}` hands back a `surface_gripper_id` that every other
`/surface_grippers/...` route addresses.

It is not an articulation, so there is nothing to drive with `move_j`. Instead:

- `POST /surface_grippers/{id}/close` and `.../open` actuate it. Both block by default and return
  the settled status, because the gripper only acts on the next physics step — a `.../status` read
  taken straight after the command still reports the previous value. Pass
  `{"asynchronous": true}` to issue the command and poll `.../status` yourself.
- `GET`/`PATCH .../properties` tune how it grips: the force limits that break a grip, how far it
  reaches, how long a close retries, and which axis it grips along.
- `GET`/`PATCH .../attachment_points` tune the individual suction cups: the D6 joint each grips
  with, its drive, its rotation and translation limits, and its clearance offset.

The asset has to be a prepared USD gripper carrying an `IsaacSurfaceGripper` prim with at least one
attachment point. There is no `urdf_path` counterpart to the articulation route — a suction gripper
has no URDF representation.

Instead of a file, `PUT /articulations`, `PUT /surface_grippers` and `POST /stage/scene/reference`
also take `asset_info` (`{"category", "manufacturer", "model"}`, as `GET /asset_library` lists
it): the bridge downloads that catalogue asset into its cache once and loads it, and an articulation
loaded this way gets the catalogue's default joint configuration and mount link unless the request
gives its own. Nothing is downloaded when a prim is already at the path.

### Attaching a gripper to an arm

`POST /articulations/{arm_id}/assemble_robot` takes one `gripper_id`, which may be either an
`articulation_id` or a `surface_gripper_id`; the bridge tells them apart and attaches accordingly.

- An **articulated** gripper is merged into the arm, so afterwards the two share one articulation
  and each device keeps driving only its own joints.
- A **suction** gripper is bolted on with a fixed joint and stays a device of its own: the arm's
  DOF are unchanged and the gripper keeps its own close/open routes. Its attachment points are
  re-parked onto the arm's mount link as part of the attach, which is what lets it grip at all.

The response's `gripper_kind` reports which happened.

### Conveyors

A conveyor is a registered device with an id, like an articulation or a camera:
`PUT /conveyors {prim_path, cargo_root?}` hands back a `conveyor_id` that every other
`/conveyors/...` route addresses. The prim path may be the conveyor asset's root, the belt rigid
body itself, or any prim in between.

It is neither an articulation nor a sensor, so there is nothing to drive with `move_j` and nothing
to `capture`. Instead:

- `POST /conveyors/{id}/start` runs the belt and `.../stop` stops it. A belt runs at a **signed
  speed along the travel direction its scene authored**, so no direction is ever sent — reverse a
  belt by starting it with a negative `velocity`, and omit `velocity` to run it at the speed the
  scene authored. A stop switches the belt's drive off rather than zeroing it, so that authored
  speed stays on the stage and the belt can be restarted without it being sent again.
- `GET /conveyors/{id}` reports the belt's current speed and whether it is running.

The belt has to be provisioned in the stage already, because its travel direction is the one thing
that cannot be guessed. Either authoring works: a `PhysxSurfaceVelocityAPI` with a non-zero
velocity on the belt, or an `IsaacConveyor` OmniGraph node driving it (the bridge then writes the
graph's velocity variable, since the node overwrites the belt's own attribute every tick).

Registering a conveyor plays the timeline, the same way registering an articulation or a sensor
does: a stopped simulation is the one state in which a belt silently does nothing, because the
velocity reaches cargo through a contact-modify callback. For the same reason it only works under
**CPU physics** — the GPU pipeline does not run that callback, and the bridge will report the belt
running regardless. And PhysX leaves sleeping
bodies out of the contact solve, so a belt cannot pick up a box that came to rest while it was
stopped: name a `cargo_root` on create and every start wakes the rigid bodies under it. Narrow it
to the prims the belt actually carries; waking a whole warehouse costs a pass over every prim in
it.

Deleting a conveyor leaves the prim in the stage and does **not** stop the belt.

### Lightbeam sensors

A lightbeam sensor is a registered device with an id, like a lidar: `PUT /lightbeams {prim_path}`
hands back a `lightbeam_id`. It wraps the other legacy PhysX range sensor, so it reads like a lidar
with a handful of rays instead of a sweep.

- `GET /lightbeams/{id}/reading` answers `{num_rays, broken, beam_hit, linear_depth, hit_pos}`.
  `broken` is true when any one beam is broken, which is the whole output of the photoelectric
  switch the sensor stands in for. `linear_depth` is per beam in meters and reads back as
  `max_range` for a beam that is **not** broken; `hit_pos` is per beam in the sensor's own frame,
  not the stage's.
- `PATCH /lightbeams/{id}/configuration` sets the beam layout and range. More than one beam spreads
  the beams evenly over `curtain_length` along the curtain axis, which is what lets the sensor
  detect an object of unknown height. `min_range` is a blind zone the beams start beyond, so an
  object closer than it is not seen at all. This takes effect on the next physics step, including
  while the timeline plays.
- `POST /lightbeams/{id}/pause` and `.../resume` switch PhysX's computation of the sensor off and
  on, which is what a registered sensor costs while nothing is reading it.

The prim has to exist and be an `IsaacLightBeamSensor`: a lightbeam's placement and aim are the
whole sensor, so unlike a lidar the bridge does not create one. Registering it enables the sensor
if the scene left it disabled, because a disabled sensor never reports a hit.

The sensor is sampled rather than queried — a reading is whatever the last physics step left behind
— so the reading route answers **409 while the timeline is stopped** rather than a beam that looks
unbroken. Only prims with a collider break the beam; purely visual geometry is invisible to it.

`linear_depth` is a raycast distance rather than something a physical light barrier measures. It is
useful as simulation ground truth, and wrong to build a controller on that has to run on hardware.

### Poses

A pose marks where a robot's tool should go. It is shown as a ghost: a see-through copy of the tool
that you place in the stage, for example around an object to pick, and that stores the robot's
joint positions for it. A program then reads the poses and moves the robot to them. The
robot's inverse kinematics is solved by the **Telekinesis Isaac Sim Server**, which must be
running at `http://127.0.0.1:6000`.

1. **Connect to the server.** Open **Telekinesis Isaac Sim Bridge ▸ Telekinesis Isaac Sim Server**
   in the menu bar. The light is red until you press **Connect**, then green when the server
   answers and red with the reason beside it when it does not. **Connect** loads every robot of the stage on the server and lists them. A
   robot added with Add Robot stores its Synapse class name as `telekinesis:robot:class_name`,
   and the server loads it as that class; no model has to be picked. The robot prim stores the
   id the server gave it as `telekinesis:robot:id`, and the link the class measures poses from
   as `telekinesis:robot:base_frame`. A robot that is not loaded yet is loaded the
   first time a pose needs it, so Connect is only needed after the server restarted, since the
   server keeps robots only while it runs.
2. **Add a pose.** Right-click the robot, any part of it, or its tool, and choose **Telekinesis
   Isaac Sim Bridge ▸ Add Pose ▸ From Tool**. The robot needs a tool attached with Add Tool. A
   ghost of the whole tool appears where the tool stands, as `<tool>_Pose` in a `poses` Xform
   beside the robot, and is selected. It is blue (`#41d1ff`, as the Add Tool preview) when the
   robot can reach it and red when it cannot.

   To teach a pose by joint positions instead, choose **Add Pose ▸ From Joint Positions**. A
   window opens with a slider and a value field per joint (degrees, or metres for a prismatic
   joint), starting at the robot's current joints; a typed value outside the joint limits is
   clamped to the nearest limit. Changing a joint draws the robot and its tool see-through at
   those joints. The joints and **Add** stay locked, with a notice saying how to connect, while
   the Telekinesis Isaac Sim Server cannot be reached, and unlock once **Connect** reaches it.
   **Add** creates a pose at the default TCP for those joints and stores them as its solution.
   Its ghost is the whole robot at those joints together with its tool, named `<tool>_Pose`, or
   `<robot>_Pose` for a robot without a tool. The new pose is hidden (its eye icon off) unless
   **Pose Visible After Add** is checked. The window stays open for the next pose. Moving the
   pose later moves its ghost as one piece; select it to see the robot reaching it.
   **From Tool** is offered only for a robot carrying a tool.

   To teach a pose for another TCP than the default one, such as the gripper's fingertip, add
   the TCP first (see **TCPs** below), then right-click the TCP and choose **Add Pose ▸ From
   Tool** or **From Joint Positions**. The pose is placed at that TCP, named `<tcp>_Pose`, and
   solved for it.

   To teach a pose by placing a frame where a TCP should go, choose **Add Pose ▸ From TCP**.
   Pick the TCP in the **TCP** dropdown: the default TCP or one added with Add TCP (right-click
   a TCP to pick it). A red/green/blue frame appears where that TCP stands, with the move/rotate
   gizmo of the Add windows (W/E); Position (m) and Rotation (XYZ Euler degrees) are the frame
   in the robot's base frame, the pose a program moves that TCP to. While you drag, the robot
   and its tool are drawn see-through reaching the frame with the picked TCP, and a line above
   **Add** says whether the robot reaches it. Picking another TCP keeps the frame where it is.
   **Add** creates the pose as that frame's arrows, named `<tcp>_Pose` (`tool0_Pose` for the
   default TCP), also when it is out of reach. A frame pose keeps its colours: select it to see
   whether the robot reaches it. Solving the frame needs the server.
3. **Place it.** Rename the pose, move it with the viewport gizmo or the Transform fields, or
   duplicate it. While one pose is selected and reachable, the robot is drawn see-through in
   the pose's solution, reaching it; the drawing is never saved and disappears when the pose
   is out of reach. Every pose that moves more than 1 mm or 0.01 rad is solved again, starting from
   its stored joint positions so the arm keeps its configuration; a pose moved out of reach turns
   red and keeps its last solution.
4. **Read the poses.** `GET /teaching/poses?robot_prim_path=<robot>` lists them with their
   `name`, `tcp`, `pose` and stored `joint_positions`. The pose is `[x, y, z, rx, ry, rz]` of
   the pose's TCP in the robot's base frame, in metres and extrinsic XYZ Euler degrees: the
   robot's default TCP (`tool0` on a Universal Robots arm) when `tcp` is `null`, so it can be
   passed to a Synapse robot's `pick` and `place` unchanged. For a pose of another TCP, give the
   Synapse robot that TCP first with `robot.add_tcp(name, offset)`, the offset shown below. Reading the poses needs no server once the robot stores its base frame. The same
   pose is stored on the pose as `telekinesis:pose:pose_in_base_frame`, so it can also be read
   in the Property window and written into a program that runs without Isaac Sim.
   `examples/ur10e_hande_teach_pick_place.py` picks at poses named `pick_pose*` and places in a
   row from the pose named `start_place_pose`, reading them from Isaac Sim or, on the real robot,
   from values written into the script.

A pose is a single `Mesh` with no physics, saved with the stage. Its origin is the robot's default
TCP, or the TCP it was made from, and it stores `telekinesis:pose:robot` (its robot),
`telekinesis:pose:tcp` (that TCP's name, for a pose made from one),
`telekinesis:pose:pose_in_base_frame`
(the `pose` above), `telekinesis:pose:joint_names`, `telekinesis:pose:joint_positions` (radians)
and `telekinesis:pose:reachable`. Poses need a
stage in metres. When the server is not running, adding a pose reports it and a moved pose keeps
its last solution, with a warning in the console. A robot added before the bridge stored class
names, and never connected, has no class name; add it again with Add Robot.

### TCPs

A TCP is a frame fixed to the robot's default TCP, such as a gripper's fingertip. Right-click the
robot, any part of it, or its tool, and choose **Telekinesis Isaac Sim Bridge ▸ Add TCP**. Enter a
name and the TCP's Position (m) and Rotation (XYZ Euler degrees) in the default TCP's frame
(`tool0` on a Universal Robots arm). While the window is open, a preview of the TCP's
red/green/blue X/Y/Z arrows stands where the entered offset puts it, with a move/rotate gizmo on
it, as in Add Tool: drag it to enter the offset (**W** to move, **E** to rotate). **Add** creates
the TCP as an Xform with those arrows under the default TCP, e.g.
`…/wrist_3_link/flange/tool0/fingertip`; the arrows are ordinary geometry, so they also show in
camera images. Once the window is closed, move the TCP with Kit's gizmo or the
Property window: its translate and rotate there are the offset Synapse's
`robot.add_tcp(name, [x, y, z, rx, ry, rz])` takes. A TCP must sit under the default TCP to move
with the robot, since prims follow their parents in the stage.

Adding a TCP needs no server once the robot stores its default TCP. The Telekinesis Isaac Sim Server gets the robot's TCPs whenever the
robot is loaded there, e.g. on **Connect**, and again whenever a TCP is created, moved, renamed or
deleted; the poses of a moved TCP are solved again. A TCP stores `telekinesis:tcp:robot`, and the
robot its default TCP's name as `telekinesis:robot:default_tcp`. Renaming or deleting a TCP
leaves its poses without it: their solves then fail with a warning in the console.

### Saved stages

Registered devices are saved with the stage. Each device prim stores its id, such as
`telekinesis:articulation:id`, next to what the bridge would otherwise keep only in memory:

| Device | Stored on its prim |
|---|---|
| Articulation | `driven_joints`, once narrowed or picked by `assemble_robot` |
| Camera | `resolution`, and `frequency` once one is requested |
| Lidar | `data_types`; the scan configuration is the prim's own |
| Conveyor | `cargo_root`, and the travel direction and speed captured on its first registration |

When a stage is opened, the bridge registers every device it stores under its stored id, as it
was saved: a gripper merged onto an arm or a suction gripper bolted onto one counts as assembled
with it, a paused lightbeam stays paused, and a conveyor saved running in reverse keeps its
direction. A camera comes back running even if it was paused. Restoring binds the devices, so the
timeline plays while it runs and is stopped again afterwards. Requests wait until the restore is
done. A device that cannot be restored is logged and skipped, and keeps its id for a later `PUT`.

A `PUT` of a restored device returns its stored id and otherwise behaves as any repeated `PUT`: a
lidar takes the request's configuration and a lightbeam is enabled again. A copy of a device prim
gets an id of its own when it is registered, and `DELETE` removes what the prim stores. A stage
saved before this release stores nothing and opens with no device registered.

Removing a device's prim from the stage — in the Stage window, from a script, or with
`DELETE /prims` — unregisters the device, and so does renaming or moving the prim. An assembly the
device took part in is forgotten with it, so a gripper that is added again is assembled again. A
prim brought back by undo still stores its id, and a `PUT` of it returns that id.

Isaac Sim does not support removing a physics prim — a link or collider of a bound articulation,
as an assembled gripper has — while the timeline plays: it breaks the physics view every
articulation reads through. When that happens, the bridge repairs it as soon as the prim is
removed. It stops and plays the timeline, which puts the scene back as it was before the timeline
was played, and rebinds every articulation. Open streams skip the frames in between; a request
in flight at that moment fails. An arm whose merged gripper was removed drives its own joints
afterwards; if the gripper was removed while the timeline was stopped, the arm's next
`PUT /articulations` rebinds it. `DELETE /prims` stops the timeline around the removal, as Isaac
Sim recommends, so the physics view never breaks.

### WebSocket routes

WebSocket routes cannot be described in OpenAPI, so Swagger UI does not list them. There are two,
both bound to a single articulation:

- `ws /articulations/{articulation_id}/stream_joint_positions` — the client pushes
  `{"joint_positions": [...], "indices": [...]?}` frames and each one retargets the articulation's
  position drive, so the joints are driven toward the stream rather than placed on it.
  Only the newest frame is applied per simulator update, so a client streaming faster than the
  simulator updates gets coarser motion rather than motion that lags behind.
- `ws /articulations/{articulation_id}/stream_articulation_state` — the server pushes one frame per
  simulator update, identical in shape to the `articulation_state` getter's response. Nothing is
  sent while the timeline is stopped.

## License

Proprietary. Copyright (c) 2024-2026 Telekinesis. All rights reserved.
Unauthorized copying, distribution, modification, or use is prohibited without
prior written permission. See [LICENSE](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/LICENSE).
