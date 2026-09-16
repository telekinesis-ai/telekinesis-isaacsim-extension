# Telekinesis Isaac Sim Bridge

Telekinesis Isaac Sim Bridge lets you connect NVIDIA Isaac Sim to local applications over HTTP and WebSocket. It is designed for developers who want to inspect articulation state, send commands, and stream real-time data without writing custom simulation-side integrations.

## What this extension provides

- Register and interact with any Isaac Sim articulation
- Register and actuate suction (surface) grippers
- Start and stop conveyor belts, and read lightbeam sensors
- Attach a gripper to an arm, articulated or suction
- Send motion commands and read articulation state
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
