# Examples

Standalone Python scripts that call the Telekinesis Isaac Sim Bridge.

- `articulations/`, `cameras/`, `lidars/`, `conveyors/`, `lightbeams/`, `surface_grippers/`:
  one script per route of that device
- `general/`: bridge status and version
- `prims/`: prim poses, metadata, visibility and colliders
- `stage/`: open the scene, stage units and timeline
- `scenes/`: builds the quickstart's pick-and-place scene with pxr; see
  [scenes/README.md](scenes/README.md)
- Top-level scripts: complete workflows, such as assembling a gripper onto a robot,
  streaming joint positions, and the quickstart's pick-and-place example
  `ur10e_hande_isaacsim_pick_place.py`

## Setup

Start Isaac Sim with the bridge extension enabled. The bridge listens on
`http://127.0.0.1:8766`.

### Installing required packages

| Examples | Packages |
|-|-|
| Device folders, `general/`, `prims/`, `stage/` and top-level scripts | `pip install -r requirements.txt` |
| `ur10e_hande_isaacsim_pick_place.py` | The Telekinesis SDK packages pinned in the [Isaac Sim installation guide](https://docs.telekinesis.ai/getting-started/advanced-installation/install-isaacsim) |
| `scenes/` | `usd-core` |

## Run the example

Every example is a standalone script. Run it from the repository root:

```
python examples/general/get_status.py
```

The docstring at the top of each script describes what it does and how to run it.
