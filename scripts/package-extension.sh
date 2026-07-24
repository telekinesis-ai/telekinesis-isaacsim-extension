#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
extension_dir="$repo_root/exts/telekinesis.isaacsim.bridge"
toml_path="$extension_dir/config/extension.toml"
output_dir="$repo_root/packages"
platform="${1:-windows}"

if [[ ! -f "$toml_path" ]]; then
  echo "extension.toml not found: $toml_path" >&2
  exit 1
fi

mkdir -p "$output_dir"

version="$(python - "$toml_path" <<'PY'
import sys
import tomllib
from pathlib import Path

with Path(sys.argv[1]).open("rb") as fh:
    config = tomllib.load(fh)

print(config["package"]["version"])
PY
)"

zip_name="telekinesis-ai-telekinesis-isaacsim-extension-${platform}-x86_64-v${version}.zip"
zip_path="$output_dir/$zip_name"

python - "$extension_dir" "$zip_path" "telekinesis.isaacsim.bridge-${version}" <<'PY'
import sys
import zipfile
from pathlib import Path

source_dir = Path(sys.argv[1]).resolve()
archive_path = Path(sys.argv[2]).resolve()
version_folder = sys.argv[3]
archive_path.parent.mkdir(parents=True, exist_ok=True)

with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
    for path in sorted(source_dir.rglob("*")):
        if path.is_dir():
            continue
        archive.write(path, arcname=f"{version_folder}/{path.relative_to(source_dir)}")

print(archive_path)
PY

echo "Created package: $zip_path"
