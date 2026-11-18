# Pick-and-place scene from a script

`pick_and_place_scene.py` builds the quickstart's pick-and-place scene without the GUI:
the sample scene with a UR10e on the Vention table, a Robotiq UR coupler and a Robotiq
Hand-E, assembled and tagged as Add Robot, Add Mesh and Add Tool leave them. The
quickstart example runs on it unchanged.

## Setup

Python with `usd-core`. The Telekinesis SDK environment includes it. Isaac Sim does not
need to be running.

## Generate

```
python examples/scenes/pick_and_place_scene.py
```

The first run downloads the assets (about 200 MB) from the Telekinesis asset library
into `~/.cache/telekinesis/usd`; later runs reuse them. The scene is written to
`examples/scenes/pick_and_place_scene.usda`.

## Run

1. Open `pick_and_place_scene.usda` in Isaac Sim with the bridge enabled. The bridge
   registers the robot and the gripper.
2. Run `python examples/ur10e_hande_isaacsim_pick_place.py`.

The scene can be saved under another name and reopened; the example runs on the reopened
scene too.

## What gets generated

```
/World/vention_table/robot_mount/UR10e_assembly    telekinesis:assemblyRobot -> UR10e
├─ UR10e                telekinesis:articulation:id = "", telekinesis:mountLink = wrist_3_link
├─ Robotiq_UR_Coupler   rigid body fixed to wrist_3_link
└─ Hand_E               telekinesis:articulation:id = "", driven_joints = [left_finger_joint]
                        merged into the UR10e's articulation, 14 mm above wrist_3_link
```

The bridge assigns the empty ids when the scene is opened.

## Known limitations

- The scene links to the cached assets by their full path, so it opens only on the
  computer that generated it. On another computer, run the script there.
- Cached assets are not updated. Delete their folder under `~/.cache/telekinesis/usd`
  to download the current version.
- The assembly follows the extension's Add Robot, Add Mesh and Add Tool. A change to
  those, such as a new tag, needs the same change in the script.
