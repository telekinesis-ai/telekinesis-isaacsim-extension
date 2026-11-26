# telekinesis-isaacsim-extensions

## USD tags

`exts/telekinesis.isaacsim.bridge/docs/USD_TAGS.md` lists every `telekinesis:` property and
customData key the bridge authors on the stage. Whenever a change adds, removes, renames, or
changes the type, units, meaning, or write timing of such a property, update that file in the
same change, and record the change in the extension's `docs/CHANGELOG.md`.

## Pick-and-place examples

`examples/scenes/pick_and_place_scene.py` builds a stage the way the bridge would (robot
assembly, taught poses and their `telekinesis:` tags), and
`examples/ur10e_hande_teach_pick_place.py` runs against that stage through the bridge's routes.
Whenever a change affects what the bridge authors on the stage (prim structure, tags, materials)
or the routes, request or response bodies those examples use, check both examples and update them
in the same change if they no longer match.
1