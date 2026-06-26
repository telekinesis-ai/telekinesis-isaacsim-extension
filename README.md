# Telekinesis Isaac Sim Bridge

An HTTP/REST bridge between external clients and Isaac Sim physics articulations. It runs a [FastAPI](https://fastapi.tiangolo.com/) server (uvicorn) inside the Isaac Sim process and exposes joints, stage control, and prim manipulation.

The bridge is **device-agnostic**: a robot arm and a gripper are both just articulations with joint positions in radians. All higher-level semantics (open/close fractions, trajectory interpolation, "done" detection) live in the client.

**Server:** `http://127.0.0.1:8766`

---

## Getting Started

### Prerequisites

- NVIDIA Isaac Sim (5.1.0 or later)
- Python 3.10+ with `pip install requests numpy` (client side only)

### Step 1 - Enable the extension in Isaac Sim

1. Open Isaac Sim
2. Click on Window -> Extensions
3. Click on the burge menu (3 dots) -> Settings -> Extension Registries -> Check kit/community  
3. Clear the serch bar from other word and then search for telekinesis.isaacsim.bridge

Isaac Sim will install `fastapi`, `uvicorn`, and `pydantic` automatically on first load via `pipapi`.

### Step 2 - Open a stage and start the simulation

In the Isaac Sim GUI:

1. Open a USD stage that contains your robot (e.g. `File > Open`).
2. Add a robot to the scene from usd or by running python examples/robot_load_from_urdf.py

The bridge is ready as soon as the simulation is running. You can verify it:

```bash
curl http://127.0.0.1:8766/status
# {"status":"OK"}
```

### Step 3 - Run the robot joint position example

With Isaac Sim playing and a 6-DOF arm in the stage (e.g. a Kuka KR210 at `/World/kuka_kr210`):

```bash
cd examples
pip install requests numpy
python robot_set_joint_position.py
```

To target a different prim:

```bash
python robot_set_joint_position.py --prim /World/ur10e
```

**What this script does:**

1. Registers the robot: `PUT /articulations` with the prim path.
2. Receives back an `articulation_id` (`"articulation1"`), the number of DOFs, and joint names.
3. Sends three joint targets in sequence (blocking): each `POST /articulations/articulation1/joint_positions` waits server-side until the arm reaches the target (or stalls), then returns.
4. Prints the result - `done`, `reached`, and the final position error in radians.

Expected output:

```
bridge: http://127.0.0.1:8766  articulations: {}
created robot: articulation_id=articulation1 prim_path=/World/.../kuka_kr210
  num_dof=6 dof_names=['joint_a1', 'joint_a2', ...]
move target (deg): [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0]
  done=True reached=True (max_error=2.31e-03 rad)
move target (deg): [-20.0, 20.0, 0.0, 0.0, 80.0, 90.0]
  done=True reached=True (max_error=1.87e-03 rad)
move target (deg): [-90.0, -90.0, 0.0, 0.0, 90.0, 0.0]
  done=True reached=True (max_error=2.12e-03 rad)
```

---

## Examples

All examples are in the [`examples/`](../../../../examples/) directory and only require the `requests` package (plus `numpy` for unit conversion).

| File | What it demonstrates |
|------|----------------------|
| `robot_set_joint_position.py` | Register a robot, drive it through multiple joint targets (blocking moves) |
| `robot_load_from_urdf.py` | Import a URDF into the stage via the bridge, then move the arm |
| `robot_asynchronous_run.py` | Send a joint target with `asynchronous=true` and poll for completion client-side |
| `gripper_control.py` | Register a gripper, discover its driver joint, narrow driven joints, open/close |
| `gripper_load_from_urdf.py` | Import a gripper URDF, narrow to driver joint, open/close via joint limits |
| `assemble_robot.py` | Assemble a gripper onto an arm, then drive both through the shared articulation |
| `robot_and_gripper.py` | Control an arm and a gripper as two independent articulations (no assembly) |
| `extension_client.py` | Exercise all implemented General, Stage, and Prims routes end-to-end |

---

## API Overview

All endpoints accept and return JSON. Successful responses use `2xx`; errors use `4xx`/`5xx` with a detail message.

### Articulations

The core resource. One articulation maps to a USD prim path and drives a subset of its joints.

| Method | Path | Description |
|--------|------|-------------|
| `PUT` | `/articulations` | Register (or re-bind) an articulation; optionally imports a URDF |
| `GET` | `/articulations` | List all registered articulation IDs and their prim paths |
| `GET` | `/articulations/{id}` | Get info and current state for one articulation |
| `DELETE` | `/articulations/{id}` | Unregister (USD prim stays in the stage) |
| `POST` | `/articulations/{id}/joint_positions` | Move to joint targets (radians); blocks until reached or stalled |
| `GET` | `/articulations/{id}/joint_state` | Current positions, velocities, and efforts |
| `GET` | `/articulations/{id}/joint_limits` | Per-joint position limits (radians) |
| `GET` | `/articulations/{id}/driver_joint` | Discover a gripper's single actuated joint |
| `PUT` | `/articulations/{id}/driven_joints` | Narrow which joints this articulation drives |
| `POST` | `/articulations/{id}/assemble_robot` | Attach a gripper articulation to this arm's flange |

#### Joint positions request

```json
{
  "positions": [-1.57, -1.57, 0.0, 0.0, 1.57, 0.0],
  "indices": null,
  "asynchronous": false
}
```

`positions` is in **radians**. `indices` restricts which joints to move (null = all driven joints). When `asynchronous` is false (default), the call blocks until the move completes.

#### Joint positions response (blocking)

```json
{
  "done": true,
  "reached": true,
  "max_error": 0.002,
  "q": [-1.57, -1.57, 0.0, 0.0, 1.57, 0.0],
  "target": [-1.57, -1.57, 0.0, 0.0, 1.57, 0.0]
}
```

`reached=true` means the arm hit the target within 5 mrad. `reached=false` means it stalled (e.g. joint limit or contact). The server times out after ~30 s (1800 physics frames).

### Stage

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/stage/scene` | URI of the open USD stage |
| `PUT` | `/stage/scene` | Open a USD stage |
| `GET` | `/stage/motion-groups` | Articulation root prims in the stage |
| `GET/PUT` | `/stage/units` | Stage meters-per-unit scale |
| `PATCH` | `/stage/simulation/timeline/{action}` | `play`, `pause`, or `stop` the timeline |
| `GET` | `/stage/simulation` | Current timeline state |

### Prims

| Method | Path | Description |
|--------|------|-------------|
| `GET/PUT` | `/prims/poses` | Get or set a prim's world pose |
| `GET/POST` | `/prims/poses/relative` | Get or apply a relative pose between two prims |
| `GET/PUT/DELETE/POST` | `/prims/poses/default` | Save, list, clear, and restore default poses |
| `PUT/DELETE` | `/prims/metadata` | Store or remove `{category, type}` metadata on a prim |
| `PATCH` | `/prims/visibility` | Show or hide a prim |
| `PATCH` | `/prims/physics/joints` | Enable or disable a physics joint |
| `PATCH` | `/prims/physics/colliders/` | Enable or disable collision on a prim |

**Pose format:** `[x, y, z, rx, ry, rz]` - position in meters, rotation as axis×angle in radians (rotation-vector). Pass `rotation_type=quaternion` to use `[x, y, z, qw, qx, qy, qz]` instead.

### General

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/status` | Liveness check - returns `{"status": "OK"}` |
| `GET` | `/version` | Installed Kit extension names and versions |

---

## Notes

- **Single-threaded on Isaac's loop.** All requests run on Isaac Sim's own asyncio loop - no extra threads. Blocking moves yield with `next_update_async()` so the server stays responsive to other requests while a move is in progress.
- **Articulation IDs are stable.** IDs are 1-based (`articulation1`, `articulation2`, …) and the same prim path always gets the same ID across repeated `PUT` calls.
- **Assembly is idempotent.** `POST /articulations/{id}/assemble_robot` for the same arm+gripper pair is a no-op; it returns `already_assembled=true`. The registry is cleared when the stage changes.
- **URDF import.** Pass `urdf_path` in the `PUT /articulations` body to have the bridge import the URDF and place it at `prim_path` automatically.

---

## License

Proprietary. Copyright (c) 2024-2026 Telekinesis. All rights reserved.
Unauthorized copying, distribution, modification, or use is prohibited without
prior written permission. See [LICENSE](LICENSE).
