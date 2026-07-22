# Telekinesis Isaac Sim Bridge

[![Isaac Sim 6.0](https://img.shields.io/badge/Isaac_Sim-6.0-green)](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html) [![Isaac Sim 5.1](https://img.shields.io/badge/Isaac_Sim-5.1-green)](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html) [![Linux platform](https://img.shields.io/badge/platform-linux--64-blue.svg)](https://releases.ubuntu.com/22.04/) [![Windows platform](https://img.shields.io/badge/platform-windows--64-blue.svg)](https://www.microsoft.com/en-us/) [![License](https://img.shields.io/badge/license-Proprietary-red.svg)](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/LICENSE)

Telekinesis Isaac Sim Bridge connects NVIDIA Isaac Sim to local applications over HTTP and WebSocket. It is a developer-facing extension for controlling articulations, reading state, and streaming real-time data from Isaac Sim.

## Enable the Extension

Enable the extension under **Window ▸ Extensions** if you haven't.

## Features

- Control and inspect Isaac Sim articulations
- Send commands through a simple HTTP API
- Stream updates in real time with WebSocket
- Explore the interface through Swagger UI

## Quick start

1. Install and enable the extension in Isaac Sim under **Window ▸ Extensions**.
2. Launch the bridge. It runs locally at `http://127.0.0.1:8766`.
3. Install the Python dependencies you need before running examples:

```bash
pip install -r requirements.txt
```

4. Connect from a local Python client or any HTTP/WebSocket-capable application.

Example:

```python
import requests

base = "http://127.0.0.1:8766"
robot = requests.put(f"{base}/articulations", json={"prim_path": "/World/ur10e"}).json()

requests.post(
    f"{base}/articulations/{robot['articulation_id']}/move_j",
    json={"joint_positions": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]},
)
```

## Documentation

- Extension README: [exts/telekinesis.isaacsim.bridge/docs/README.md](exts/telekinesis.isaacsim.bridge/docs/README.md)
- Interactive API docs: `http://127.0.0.1:8766/docs` or `http://127.0.0.1:8766/redoc`
- Static API reference: [https://telekinesis-ai.github.io/telekinesis-isaacsim-extension/](https://telekinesis-ai.github.io/telekinesis-isaacsim-extension/)

## Notes

- The bridge is localhost-only.
- It has no authentication and is intended for trusted local clients.
- Use **radians** for joint values and **meters** for length-related values.

## License

Proprietary. Copyright (c) 2024-2026 Telekinesis. All rights reserved.
Unauthorized copying, distribution, modification, or use is prohibited without
prior written permission. See [LICENSE](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/LICENSE).
