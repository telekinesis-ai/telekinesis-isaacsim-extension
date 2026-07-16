# Telekinesis Isaac Sim Bridge

[![Isaac Sim 6.0](https://img.shields.io/badge/Isaac_Sim-6.0-green)](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html) [![Isaac Sim 5.1](https://img.shields.io/badge/Isaac_Sim-5.1-green)](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html) [![Linux platform](https://img.shields.io/badge/platform-linux--64-blue.svg)](https://releases.ubuntu.com/22.04/) [![Windows platform](https://img.shields.io/badge/platform-windows--64-blue.svg)](https://www.microsoft.com/en-us/) [![License](https://img.shields.io/badge/license-Proprietary-red.svg)](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/LICENSE)

Control Isaac Sim robots over HTTP.

Any robot -- arm, mobile base, gripper, or humanoid -- is just an articulation with joint positions. Control logic stays in your client.

Runs at `http://127.0.0.1:8766`. Localhost-only, no authentication, for trusted local clients.

## Install

You're reading this because the extension is already installed (this file ships inside the package itself) -- enable it under **Window ▸ Extensions** if you haven't.

## Usage

1. Add a Universal Robots UR10e to the stage from the Isaac Sim asset store (**Create ▸ Isaac ▸ Robots ▸ Universal Robots ▸ UR10e**, or drag it in from the Assets browser).
2. Note its prim path in the Stage panel (e.g. `/World/ur10e`).
3. Press **Play**, then register it and move its joints:

```python
import requests

base = "http://127.0.0.1:8766"
robot = requests.put(f"{base}/articulations", json={"prim_path": "/World/ur10e"}).json()

requests.post(
    f"{base}/articulations/{robot['articulation_id']}/move_j",
    json={"joint_positions": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]},
)
```

More complete, runnable examples (URDF import, grippers, streaming, async moves) are in [`examples/`](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/tree/main/examples) in the GitHub repo -- not bundled in this installed package.

## API Reference

The endpoint list is generated straight from the code, not hand-maintained:

- **Bridge running:** open `http://127.0.0.1:8766/docs` (Swagger UI) or `/redoc`.
- **Browsing without Isaac Sim running:** [API reference](https://telekinesis-ai.github.io/telekinesis-isaacsim-extension/).

Wire units throughout: **radians** for joints, **meters** for lengths.

## License

Proprietary. Copyright (c) 2024-2026 Telekinesis. All rights reserved.
Unauthorized copying, distribution, modification, or use is prohibited without
prior written permission. See [LICENSE](https://github.com/telekinesis-ai/telekinesis-isaacsim-extension/blob/main/LICENSE).
