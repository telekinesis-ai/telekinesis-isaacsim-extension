# Telekinesis Isaac Sim Bridge

Telekinesis Isaac Sim Bridge lets you connect NVIDIA Isaac Sim to local applications over HTTP and WebSocket. It is designed for developers who want to inspect articulation state, send commands, and stream real-time data without writing custom simulation-side integrations.

## What this extension provides

- Register and interact with any Isaac Sim articulation
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

Use **radians** for joint values and **meters** for length-related values.

## License

Proprietary. Copyright (c) 2024-2026 Telekinesis. All rights reserved.
Unauthorized copying, distribution, modification, or use is prohibited without
prior written permission. See [LICENSE](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/LICENSE).
