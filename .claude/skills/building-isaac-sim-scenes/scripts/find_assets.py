"""Search the Telekinesis asset library for scene-building assets.

Reads the catalogue the bridge ships (``assets_index.json``) plus the environment scenes,
which the catalogue does not list. Every word of the query must appear in an entry's
category, manufacturer, model or identifier (case-insensitive).

    python find_assets.py ur10e
    python find_assets.py robotiq --category usd/tools
    python find_assets.py --category usd/environments
    python find_assets.py --list-categories

Each hit prints its asset key (the path under https://assets.telekinesis.ai, the argument
``inspect_asset.py`` and the template's ``download_asset`` take) and, for a robot arm, its
mount link and the Synapse class name to tag it with.
"""

import argparse
import json
import pathlib

# The skill lives in <repo>/.claude/skills/<skill>/scripts/.
REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[4]
INDEX_PATH = (
    REPOSITORY_ROOT
    / "exts/telekinesis.isaacsim.bridge/telekinesis/isaacsim/bridge/assets/assets_index.json"
)
# Not in the catalogue. Each is a ZIP whose main file is <name>.usd; all are Z-up, metres.
ENVIRONMENT_SCENES = [
    ("pick and place: Vention table with robot_mount, three cubes", "usd/environments/pick_and_place/pick_and_place_scene.zip"),
    ("CNC machine tending: lathe, tray table, 16 round blanks", "usd/environments/machine_tending/cnc_machine_tending.zip"),
    ("palletizing: MAiRA 7M, conveyors, eight boxes, fence, pallet", "usd/environments/palletizing/palletizing_rough_scene.zip"),
    ("bin picking", "usd/environments/bin_picking/bin_picking_scene.zip"),
    ("stationary spot welding", "usd/environments/welding/stationary_spot_welding_scene.zip"),
]


def load_entries() -> list[dict]:
    """Return every catalogue entry and environment scene, each with its ``category``."""
    index = json.loads(INDEX_PATH.read_text(encoding="utf-8"))
    entries = [
        {**entry, "category": category}
        for category, category_entries in index.items()
        for entry in category_entries
    ]
    entries += [
        {"category": "usd/environments", "manufacturer": None, "model": description,
         "identifier": pathlib.PurePosixPath(key).stem, "bundle": key}
        for description, key in ENVIRONMENT_SCENES
    ]
    return entries


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("query", nargs="*", help="words that must all match")
    parser.add_argument("--category", help="category prefix, e.g. usd/robots or usd/tools/parallel_grippers")
    parser.add_argument("--list-categories", action="store_true")
    parser.add_argument("--include-urdf", action="store_true",
                        help="also list urdf/ bundles; scenes built offline reference usd/ bundles")
    arguments = parser.parse_args()

    entries = load_entries()
    if not arguments.include_urdf:
        entries = [entry for entry in entries if not entry["category"].startswith("urdf/")]
    if arguments.list_categories:
        for category in sorted({entry["category"] for entry in entries}):
            count = sum(entry["category"] == category for entry in entries)
            print(f"{category}  ({count})")
        return

    words = [word.lower() for word in arguments.query]
    hits = []
    for entry in entries:
        if arguments.category and not entry["category"].startswith(arguments.category):
            continue
        haystack = " ".join(
            str(entry.get(field) or "") for field in ("category", "manufacturer", "model", "identifier")
        ).lower()
        if all(word in haystack for word in words):
            hits.append(entry)

    if not hits:
        print("No match. Try fewer words, or --list-categories.")
        return
    for entry in hits:
        name = " ".join(part for part in (entry.get("manufacturer"), entry["model"]) if part)
        print(f"{name}\n  key: {entry['bundle']}")
        if entry.get("simulation_mount_link"):
            # The class name the bridge stores for a catalogue arm (articulations.py).
            class_name = entry["identifier"].replace("/", "").replace("_", "")
            print(
                f"  mount link: {entry['simulation_mount_link']}  class name: {class_name}"
                f"  default TCP: {entry.get('default_tcp') or '-'}"
            )
    print(f"\n{len(hits)} match(es).")


if __name__ == "__main__":
    main()
