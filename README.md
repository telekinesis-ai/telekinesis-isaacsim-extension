# Telekinesis Isaac Sim Bridge

[![Isaac Sim 6.0](https://img.shields.io/badge/Isaac_Sim-6.0-green)](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html) [![Isaac Sim 5.1](https://img.shields.io/badge/Isaac_Sim-5.1-green)](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html) [![Linux platform](https://img.shields.io/badge/platform-linux--64-blue.svg)](https://releases.ubuntu.com/22.04/) [![Windows platform](https://img.shields.io/badge/platform-windows--64-blue.svg)](https://www.microsoft.com/en-us/) [![License](https://img.shields.io/badge/license-Proprietary-red.svg)](LICENSE)

This extension is used to connect and communicate with NVIDIA Isaac Sim.
Runs at `http://127.0.0.1:8766`. Localhost-only, no authentication, for trusted local clients.

## Install

Open **Window ▸ Extensions** in Isaac Sim, search for `telekinesis.isaacsim.bridge`, and enable it.

## Usage

1. Add a Universal Robots UR10e to the stage from the Isaac Sim asset store.
2. Note its prim path in the Stage panel (e.g. `/World/ur10e`).
3. Make sure the extansion in enables, and run:

```python
import requests

base = "http://127.0.0.1:8766"
robot = requests.put(f"{base}/articulations", json={"prim_path": "/World/ur10e"}).json()

requests.post(
    f"{base}/articulations/{robot['articulation_id']}/move_j",
    json={"joint_positions": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]},
)
```

## API Reference

The endpoint list is generated straight from the code, not hand-maintained:

- **Bridge running:** open `http://127.0.0.1:8766/docs` (Swagger UI) or `/redoc`.
- **Browsing without Isaac Sim running:** [API reference](https://telekinesis-ai.github.io/telekinesis-isaacsim-extension/).

Wire units throughout: **radians** for joints, **meters** for lengths.

## License

Proprietary. Copyright (c) 2024-2026 Telekinesis. All rights reserved.
Unauthorized copying, distribution, modification, or use is prohibited without
prior written permission. See [LICENSE](LICENSE).
