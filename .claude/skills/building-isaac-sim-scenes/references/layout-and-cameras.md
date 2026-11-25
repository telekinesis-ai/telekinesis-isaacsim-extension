# Layout and cameras

How to turn a description's words into numbers, what the environment scenes contain, and
how to place cameras.

## Contents

- Conventions
- From words to coordinates
- Reach
- Environment scenes
- Cameras

## Conventions

- World: Z up, metres, `metersPerUnit = 1`, default prim `/World`.
- Poses are `(x, y, z, rx, ry, rz)`: metres, then **extrinsic XYZ Euler degrees** (scipy
  `"xyz"`, the bridge's convention). Never quaternions in anything the user reads.
- A pallet or a box is posed by the centre of its underside. A robot is posed by its
  base origin, which for catalogue arms is the centre of the base's underside (check
  `inspect_asset.py` bounds: min z = 0).
- A USD camera looks along its own -Z, with +Y up in the image.

## From words to coordinates

Write every placement as arithmetic on inspected numbers. Never estimate:

- **"on" a surface**: `z = surface_top - asset_min_z`. `surface_top` is the support's
  world bounds max z, or its top collider's top. `asset_min_z` comes from
  `inspect_asset.py` bounds in the asset's own frame. Free parts get 0.5 to 1 mm more.
- **"next to" / "left of"**: edge to edge. Centre distance = half size A + gap + half
  size B. Default gap: 0.05 m between objects the robot handles, more around a robot.
  "Left", "front" and similar words have no world meaning until the user says which way
  the robot faces. State the reading in the report. Default: the robot faces +X, so
  left is +Y.
- **"facing"**: yaw so the object's working side points at the target. For an arm, yaw
  so the home-pose tool points toward the work area. A UR arm reaches along its own -X
  at zero pan.
- **"in a row / grid"**: compute the slots from the count, item size and gap. Check that
  the total footprint fits the support.
- Put every number in a named constant with a comment saying where it comes from.

## Reach

- Keep everything the robot must touch within about 80 % of its catalogue reach (the
  model name often carries it, e.g. `IRB 1600-6/1.2` reaches 1.2 m, KUKA `KR 120 R2500`
  reaches 2.5 m). Distances count from the shoulder axis, not the base footprint.
- Floor-level and very near targets are hard too. A target almost below the shoulder
  puts the wrist straight, near a singularity.
- Leave about 0.5 m from a target the robot sets things down on to the robot's own base
  and links. At 0.42 m a UR20 knocked a box off with its own arm.
- If the description's layout cannot be reached, say so. Do not silently move things.

## Environment scenes

`find_assets.py --category usd/environments` lists them. Inspect before use. Known prims:

| Scene | Main file | Key prims |
|---|---|---|
| pick and place | `pick_and_place_scene.usd` | `/World/vention_table/robot_mount` at (-0.537, 0.382, 0.884): put the robot assembly under it with no transform. `/World/Cube`, `Cube_01`, `Cube_02` (30 mm, 0.3 kg) at z 0.86. `/World/GroundPlane`. |
| CNC machine tending | `cnc_machine_tending.usd` | Robot mount top (-2.693, -1.559, 1.23). Infeed tray (-3.10, -0.83, 1.05). Workholding (-2.737, -2.643, 1.44). Blanks `/World/part_*` ship without mass or friction. The outfeed tray's pockets are holes. |
| palletizing | `palletizing_rough_scene.usd` | MAiRA 7M on `/World/ur10_mount` (0.405 m square, 0.773 m high, top centre (1.305, 4.396)). Conveyor pick end (0.6, 3.9). Pallets `/World/pallet` and `/World/IndustrialPlasticPallet_A01_PR_V_NVD_01`. Fence 1.6 m in front of the robot, 1.7 m high. |
| bin picking, spot welding | `bin_picking_scene.usd`, `stationary_spot_welding_scene.usd` | Inspect first. |

## Cameras

Use `look_at(eye, target)` and `add_camera` from the template. Sensor 20.955 x 11.787 mm
(16:9), so focal lengths read as on full frame: 14 to 20 mm wide, 24 to 35 mm normal,
50+ mm close-up.

- **Dataset or observation cameras**: fixed third-person views under
  `/World/DatasetCameras` or `/World/Cameras`. Register them as bridge cameras
  (resolution tag) if code will read them. A `front` view from about 2 m away and 2 m up,
  aimed at the work area at table height, plus a `top` view straight down (14 to 18 mm),
  cover most cells.
- **Cinematic shots**: `cinematic_cameras.add_cinematic_cameras(stage, centre, radius,
  height, enclosure=(half_width, fence_height), ceiling=...)` in the applications repo
  frames six standard shots (establishing, hero, over_shoulder, plan, profile,
  close_up) around the action. Put `centre` between the robot and its work, not on the
  base. Without a sky dome, low shots looking up film black, so aim them slightly
  down. Inside a fence, keep low shots inside it, or above the fence by 0.6 m or more.
  Leave depth of field off: the real-time renderer blurs every shot.
- **Wrist cameras**: see bridge-tags-and-assembly.md. Place them diagonally clear of the
  tool's outline and outside a held part's footprint, or they film the tool's own
  bracket or the held part. Give camera housings box colliders.
- Check each camera in `validate_scene.py`'s camera list: position and view direction
  must point at what the description says it watches.
