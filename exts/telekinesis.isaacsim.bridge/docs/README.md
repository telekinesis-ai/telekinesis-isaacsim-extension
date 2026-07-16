# Telekinesis Isaac Sim Bridge

An HTTP/REST and WebSocket bridge between external clients and Isaac Sim physics articulations. It runs a [FastAPI](https://fastapi.tiangolo.com/) server (uvicorn) inside the Isaac Sim process and exposes joints, stage control, and prim manipulation, plus a WebSocket stream for high-rate joint teleport updates.

The bridge is **device-agnostic**: a robot (arm, mobile base, humanoid, ...) and an end-effector tool (e.g. a gripper) are both just articulations with joint positions in radians. All higher-level semantics (open/close fractions, trajectory interpolation, "done" detection) live in the client.

**Server:** `http://127.0.0.1:8766`

> **Security:** the bridge binds to `127.0.0.1` only and has **no authentication or authorization** -- any process that can reach that port can register articulations, drive joints, open arbitrary USD stages (`PUT /stage/scene`), and import arbitrary URDF files from the local filesystem. It is designed for a **trusted local client only** (a script or tool running on the same machine as Isaac Sim). Do not port-forward, reverse-proxy, or otherwise expose this port beyond localhost without adding your own auth layer in front of it.

---

## Getting Started

### Prerequisites

- NVIDIA Isaac Sim 5.1.0 or later, already installed and runnable. Tested
  against **5.1.0.0** (Python 3.11) and **6.0.0.1** (Python 3.12).
- Python 3.10+ with `pip install requests numpy` (client side only)

### Step 1 - Load the extension in Isaac Sim

You're reading this because the extension is already installed (this file ships inside the
package itself) -- enable it under **Window ▸ Extensions** if you haven't. Isaac Sim will
install `fastapi`, `uvicorn`, `pydantic`, and `websockets` automatically on first load via
`pipapi`.

Building from source instead (contributing, or testing an unreleased change)? See
[DEVELOPMENT.md](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/DEVELOPMENT.md)
in the GitHub repo -- it isn't bundled in this installed package.

### Step 2 - Open a stage and start the simulation

In the Isaac Sim GUI:

1. Open a USD stage that contains your robot (e.g. `File > Open`).
2. Add a robot to the scene from USD, or by running the
   [`robot_load_from_urdf.py`](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/examples/robot_load_from_urdf.py)
   example (see Examples below -- examples live in the GitHub repo, not in this installed package).

The bridge is ready as soon as the simulation is running. You can verify it:

```bash
curl http://127.0.0.1:8766/status
# {"status":"OK"}
```

### Step 3 - Run the robot joint position example

With Isaac Sim playing and a 6-DOF arm in the stage (e.g. a Kuka KR210 at `/World/kuka_kr210`),
clone the repo (or just download
[`robot_set_joint_position.py`](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/examples/robot_set_joint_position.py))
to get the example script -- it isn't bundled in this installed package:

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
3. Sends three joint targets in sequence (blocking): each `POST /articulations/articulation1/move_j` waits server-side until the arm reaches the target (or stalls), then returns.
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

All examples live in the
[`examples/`](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/tree/main/examples)
directory of the GitHub repo (not bundled in this installed package) and only require the
`requests` package (plus `numpy` for unit conversion).

| File | What it demonstrates |
|------|----------------------|
| `robot_set_joint_position.py` | Register a robot, drive it through multiple joint targets (blocking moves) |
| `robot_load_from_urdf.py` | Import a URDF into the stage via the bridge, then move the arm |
| `robot_async_set_joint_position.py` | Send a joint target with `asynchronous=true` and poll for completion client-side |
| `gripper_control.py` | Register a gripper, discover its driver joint, narrow driven joints, open/close |
| `gripper_load_from_urdf.py` | Import a gripper URDF, narrow to driver joint, open/close via joint limits |
| `assemble_robot_and_gripper.py` | Assemble a gripper onto an arm, then drive both through the shared articulation |
| `robot_stream_joint_positions.py` | Stream a joint trajectory over the WebSocket (`stream_joint_positions`) for fast, continuous updates |
| `extension_client.py` | Exercise all implemented General, Stage, and Prims routes end-to-end |

---

## API Reference

The full, always-current endpoint list is generated straight from the code:

- **Bridge running:** open `http://127.0.0.1:8766/docs` (Swagger UI) or `/redoc`.
- **Browsing without Isaac Sim running:** [API reference](https://telekinesis-ai.github.io/telekinesis-isaacsim-extension/).

Wire units throughout: **radians** for joints, **meters** for lengths. Every endpoint accepts
and returns JSON; successful responses use `2xx`, errors use `4xx`/`5xx` with a `detail` message.

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
prior written permission. See
[LICENSE](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/LICENSE).
