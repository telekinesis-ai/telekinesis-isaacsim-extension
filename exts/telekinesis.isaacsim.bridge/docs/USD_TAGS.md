# Telekinesis tags written into USD

Every `telekinesis:` property the bridge authors on the stage. A device is restored on
stage open only if its prim has `telekinesis:<kind>:id`, so that attribute is the one a
scene-assembly script must write.

## Per-device state

Attributes named `telekinesis:<kind>:<name>` on the prim that is PUT, written by
`DeviceRegistry.write_stored_value` (`comm/services/device_registry.py`). The id must match
`<kind><n>` with n >= 1, e.g. `camera1`.

| Property | USD type | Written when | Required for restore? |
|---|---|---|---|
| `telekinesis:articulation:id` | `string` | `PUT` articulation, on register | Yes (restore trigger) |
| `telekinesis:articulation:driven_joints` | `token[]` | `set_driven_joints`, and `assemble_robot` for the gripper | No. Without it the articulation drives all its own joints. |
| `telekinesis:camera:id` | `string` | `PUT` camera | Yes |
| `telekinesis:camera:resolution` | `int2` | `PUT` camera, and when the resolution is set | Effectively yes. Restore passes it to `create_camera`, which calls `tuple(resolution)`; `None` is expected to fail (not run). |
| `telekinesis:camera:frequency` | `float` | `PUT` camera when a frequency is requested, and when frequency is set | No |
| `telekinesis:lidar:id` | `string` | `PUT` lidar | Yes |
| `telekinesis:lidar:data_types` | `token[]` | `PUT` lidar | Probably. Restore passes it through; handling of `None` not checked. |
| `telekinesis:lightbeam:id` | `string` | `PUT` lightbeam | Yes |
| `telekinesis:surface_gripper:id` | `string` | `PUT` surface gripper | Yes |
| `telekinesis:conveyor:id` | `string` | `PUT` conveyor | Yes |
| `telekinesis:conveyor:authored_drive` | `token` | `PUT` conveyor | No. If absent, the belt is read from the stage. |
| `telekinesis:conveyor:authored_attribute` | `token` | `PUT` conveyor | No |
| `telekinesis:conveyor:authored_nominal_speed` | `double` | `PUT` conveyor | No |
| `telekinesis:conveyor:authored_direction` | `float3` | `PUT` conveyor; removed if direction is `None` | No |
| `telekinesis:conveyor:cargo_root` | relationship (target = cargo prim path) | `PUT` conveyor when `cargo_root` is given; removed otherwise | No |

Lidar scan settings (range, fov, ...) are not stored under `telekinesis:`. They live on the
lidar prim itself and are read by `read_scan_config`.

## Assembly tags

Not under a device namespace.

| Property | Type | On which prim | Written when | Purpose |
|---|---|---|---|---|
| `telekinesis:mountLink` | `string` attribute | Articulation prim | `PUT` articulation with `mount_link`, or a catalogue asset's `simulation_mount_link` | Default link that `assemble_robot` and `mount_prim` attach to |
| `telekinesis:assemblyRobot` | relationship to the robot prim | The assembly Xform | Add Robot window (`ui/add_robot_window.py`) | Makes the Xform and everything under it count as part of that robot. Add Camera and Add Tool use it to find the robot. |

## Telekinesis connection tags

Written on the robot prim (the articulation) by Add Robot and by loading the robot on the
Telekinesis Isaac Sim Server (`PoseService`, `comm/services/poses.py`). Not a device kind: they
store no device id, so the device restore never reads them, and `DELETE` of an articulation does
not remove them. `examples/scenes/pick_and_place_scene.py` writes `telekinesis:robot:class_name`
(lowercase), `telekinesis:robot:base_frame` and `telekinesis:robot:default_tcp` itself.

| Property | Type | Written when | Purpose |
|---|---|---|---|
| `telekinesis:robot:id` | `string` attribute | The robot is loaded on the server: Connect in the Telekinesis Isaac Sim Server window, or the first pose that needs it | Id (UUID) of the robot loaded for this robot on the Telekinesis Isaac Sim Server. Valid until the server stops; afterwards the server answers 404 until Connect loads the robot again |
| `telekinesis:robot:base_frame` | `string` attribute | The robot is loaded on the server, as for `telekinesis:robot:id` | Name of the link the Synapse class measures poses from, e.g. `base_link`. Lets `GET /teaching/poses` read poses without the server |
| `telekinesis:robot:default_tcp` | `string` attribute | A catalogue robot whose index entry names a `default_tcp` is loaded (Add Robot, `PUT` articulation with `asset_info`); rewritten when the robot is loaded on the server, as for `telekinesis:robot:id` | Name of the robot's default TCP frame, e.g. `tool0`. Add TCP places TCPs under the prim of that name |
| `telekinesis:robot:class_name` | `string` attribute | A catalogue robot is loaded (Add Robot, `PUT` articulation with `asset_info`), in lowercase; rewritten with the server's exact casing when the robot is loaded on the server | Synapse class the server loads the robot as, e.g. `universalrobotsur10e`, then `UniversalRobotsUR10E` |

## Pose tags

Written on a pose (a `Mesh` ghost named `<tcp>_Pose` for a pose made from a right-clicked TCP,
otherwise `<tool>_Pose`, or `<robot>_Pose` for a robot without a tool, in the `poses` Xform beside
the robot; its origin is the TCP it places, the `tool_base` TCP for a From Tool pose) by **Add
Pose ▸ From Tool** and **Add Pose ▸ From Joint Positions**, and on a frame pose (an `Xform` of red/green/blue axis arrows named `<tcp>_Pose`,
or `<default TCP>_Pose` such as `tool0_Pose`, in the same `poses` Xform) by **Add Pose ▸ From
TCP**; rewritten by `PoseService` whenever the pose moves.
`examples/scenes/pick_and_place_scene.py` writes the same tags on the poses it creates.

| Property | Type | Purpose |
|---|---|---|
| `telekinesis:pose:robot` | relationship to the robot prim | The robot whose TCP the pose places |
| `telekinesis:pose:tcp` | `string` attribute | Name of the robot's TCP the pose places, written on every new pose: `tool_base` for From Tool, the right-clicked TCP for From Tool on a TCP, the picked TCP for From TCP, and the default TCP's name (e.g. `tool0`) for From Joint Positions and the default TCP in From TCP. Absent on poses taught before every pose stored it, which place the default TCP. The pose is solved for this TCP, and solved again when that TCP changes unless it is the default TCP; renaming or deleting the TCP makes its solves fail |
| `telekinesis:pose:pose_in_base_frame` | `double[6]`: metres, then extrinsic XYZ Euler degrees | `[x, y, z, rx, ry, rz]` of the pose's TCP in the robot's base frame, the `pose` `GET /teaching/poses` returns. Written on creation, on every move (including moves too small to solve again), and when the stage is opened wherever it is missing or differs, e.g. on poses taught before 0.11.2; not for a robot that stores no `telekinesis:robot:base_frame` yet. A mirror for reading: `GET /teaching/poses` still computes the pose from the prim's transform |
| `telekinesis:pose:joint_names` | `string[]` | The robot's joint names, in the order of `joint_positions` |
| `telekinesis:pose:joint_positions` | `double[]`, radians | The robot's inverse-kinematics solution for the pose; empty when unreachable |
| `telekinesis:pose:reachable` | `bool` | Whether the robot can reach the pose; also marks the prim as a pose |

## TCP tags

Written on a TCP, an `Xform` that **Add TCP** places under the robot's default TCP frame
(`<default TCP>/<name>`, e.g. `…/wrist_3_link/flange/tool0/fingertip`), with red/green/blue
axis arrows: one `Mesh` at `<tcp>/frame` in the default purpose, coloured per face by
`primvars:displayColor`, drawn without guides. The TCP's name is its prim's name. Its offset is not stored separately: it is
the prim's pose in the default TCP frame, read from where the prim stands, so its own translate
and rotateXYZ ops show it while it sits directly under the default TCP: metres, then extrinsic XYZ
Euler degrees, as Synapse's `add_tcp` takes it. **Add TCP** creates a visible TCP. **Add Tool**
places a hidden TCP named `tool_base` (its Xform has `visibility = invisible`) at the base
of the tool it attaches, and moves it there when the robot already has one; **Add Pose ▸ From
Tool** places it first when the robot has none, e.g. for a tool a program attached. Synapse's
`attach_tool` adds a TCP of the same name at the tool mount it is given. The bridge gives a robot's TCPs to the
Telekinesis Isaac Sim Server when the robot is loaded there, and again whenever a TCP is
created, moved, renamed or deleted.

| Property | Type | Purpose |
|---|---|---|
| `telekinesis:tcp:robot` | relationship to the robot prim | Marks the prim as a TCP of that robot |

## Prim customData

Set by the `/prims` routes (`comm/services/prims.py`).

| Key | Value | Written when |
|---|---|---|
| `telekinesis:metadata` | plain dict | `set_metadata`; removed by `remove_metadata` |

## Notes

- `DELETE` of a device removes every property in its `telekinesis:<kind>` namespace from the
  prim. The prim stays.
- The `telekinesis:<kind>:<name>` names and the `<kind><n>` id format are an on-disk format.
  Do not rename them.
- A prim with `telekinesis:mountLink` but no id is treated as an old-style articulation by
  `find_robot_for_prim`.
- A copied prim carries its source's id; the first prim to take an id keeps it.
- Lightbeam and surface-gripper services were only grepped (no extra stored properties beyond
  `id`); their restore code was not read. The "required" column comes from reading
  `restore_device`, not from running a restore.
