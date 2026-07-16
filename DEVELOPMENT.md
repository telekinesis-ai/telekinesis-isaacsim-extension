# Development

This is a developer's guide to run the bridge extension from this source tree (your local dev copy) instead of the published version from the community registry, and also to contribute.

## Prerequisites
- It is recommended to create a conda environment.
  ```bash
  conda create -n telekinesis-isaacsim-bridge python=3.12
  ```

- Install NVIDIA Isaac Sim 5.1.0 or later. The required Python version depends on the Isaac Sim release, match it before installing. Tested against **5.1.0.0** and **6.0.0.1**:

  | Isaac Sim | Python | Install |
  |-----------|--------|---------|
  | 5.1.0.0 | 3.11 | `pip install isaacsim[all,extscache]==5.1.0.0 --extra-index-url https://pypi.nvidia.com` |
  | 6.0.0.1 | 3.12 | `pip install isaacsim[all,extscache]==6.0.0.1 --extra-index-url https://pypi.nvidia.com` |

- The Kit extensions `isaacsim.core.api`, `isaacsim.gui.components`, and `omni.kit.uiapp` must be available (they're pulled in automatically as dependencies, but if your Isaac Sim install is missing them, enabling `telekinesis.isaacsim.bridge` will fail)

The extension lives at:
```
exts/telekinesis.isaacsim.bridge
```

Isaac Sim discovers extensions by **search path**, and the path you register is the **parent `exts/` directory**, not the extension folder itself.

> Use forward slashes in paths — Kit is unreliable with Windows backslashes.

---

## Enable the Extension via the Extension Manager UI

1. Open Isaac Sim → **Window ▸ Extensions**.
2. Click the hamburger/gear menu (top-left of the Extensions panel) → **Settings**.
3. Under **Extension Search Paths**, click **+** and add the absolute path to this repo's `exts` folder, e.g.:

   ```
   c:/Users/<you>/<path>/telekinesis-isaacsim-extension/exts
   ```

4. Back in the extension list, search for `telekinesis.isaacsim.bridge`. It now appears under the **THIRD PARTY / user** extensions.
5. Toggle it **ON**. Tick **Autoload** to enable it on every launch.

Your local copy takes priority over the registry version, so your source edits are
what run. Enable hot-reload (⟳) while developing to pick up changes without a restart.


## Verify

On first enable, Kit auto-installs `fastapi`, `uvicorn`, `pydantic`, and `websockets` via `pipapi` (needs internet — if you're offline, pre-install `fastapi uvicorn pydantic websockets` into Isaac's Python instead).

Open a stage and start playing the simulation, then:

```bash
curl http://127.0.0.1:8766/status
# {"status":"OK"}
```

## Merge to main

Before opening a PR against `main`, run `ruff` and `pylint` and fix anything they flag. Both are run with a 100-character line length, matching the convention used throughout `exts/` and `examples/`.

```bash
pip install ruff pylint
ruff check --line-length 100 .
ruff format --line-length 100 .
pylint --max-line-length 100 --ignored-modules=omni,carb,isaacsim,pxr --disable=duplicate-code,attribute-defined-outside-init,import-outside-toplevel,unused-argument,broad-exception-caught,protected-access,too-many-arguments,too-many-positional-arguments,too-many-locals,too-many-public-methods,too-many-instance-attributes,too-many-lines exts/telekinesis.isaacsim.bridge/telekinesis examples
```

`--ignored-modules` tells pylint not to try to fully resolve `omni`/`carb`/`isaacsim`/`pxr` -- those are Isaac Sim's own packages, only present when Isaac Sim is installed, so without this flag pylint reports every import from them as unresolved and can't check the members of `pxr` types either. Without Isaac Sim installed, this is expected and not a real code issue.

Every disabled check is a deliberate, documented pattern rather than a real issue:
- `duplicate-code`: `examples/` is one small, self-contained script per endpoint (see Examples above) -- the shared `_request()` helper and argparse boilerplate are copy-pasted on purpose so each script can be grabbed and run standalone.
- `attribute-defined-outside-init`: only fires in `extension.py`, where Kit's own convention uses `on_startup` as the real constructor instead of `__init__`.
- `import-outside-toplevel`: only fires in the service/core modules that lazily import `omni`/`pxr` inside their methods on purpose, so those modules still import outside Isaac Sim.
- `unused-argument`: only fires on 501 stub routes (the path parameter has to exist for FastAPI's route to match, even though the stub body ignores it) and on Kit event-callback signatures whose shape is mandated by the framework.
- `broad-exception-caught`: the codebase's own "catch broad, translate to a typed error at the boundary" pattern -- bind-retry loops, the bridge server's must-never-break-startup guard, and a couple of example scripts' top-level fallback.
- `protected-access`: the handful of `_articulation_view`/`_metadata` reads are already commented in the code as "no public accessor exists yet."
- `too-many-arguments` / `too-many-positional-arguments`: fire on `assemble_robot`-shaped functions (assembling a gripper onto an arm genuinely needs many parameters -- stage, prims, mounts, offset, namespace); the sibling `telekinesis` repo's own `.pylintrc` relaxes these same two checks.
- `too-many-locals`: the same assemble/orchestration functions, for the same reason.
- `too-many-public-methods` / `too-many-instance-attributes`: `ArticulationService`/`SingleArticulation` deliberately mirror Isaac Sim's own large `SingleArticulation` API surface; `Extension` deliberately holds one attribute per Kit subscription/callback per Kit's own convention.
- `too-many-lines`: `routers.py` holds every route for the whole API in one file by design (see the Merge to main section above); splitting it up would fight that design, not fix a real problem.

`ruff check` covers style, unused imports, and import ordering; `ruff format` auto-formats to match (run it after `check` so anything `check` flagged is already fixed first); `pylint` catches a broader set of correctness issues. There's no CI running these yet -- run all three by hand before every PR.

---

## Manual Deployment

Follow the below steps to release a new version of the extension. 
### 1. Verify

Covered by [Merge to main](#merge-to-main) (ruff, pylint), plus:

```bash
python scripts/generate_openapi.py   # should exit 0 and write public/openapi.json
```

### 2. Check the version hasn't already been released

```bash
VERSION=$(python -c "import tomllib; print(tomllib.load(open('exts/telekinesis.isaacsim.bridge/config/extension.toml','rb'))['package']['version'])")
echo "$VERSION"
git tag -l "v$VERSION"        # must print nothing -- if it prints a tag, bump the version first
```

PowerShell (bare `python` may resolve to a different/older interpreter than your active conda env
-- use the full path to that env's `python.exe` if `tomllib` isn't found):

```powershell
$VERSION = python -c "import tomllib; print(tomllib.load(open('exts/telekinesis.isaacsim.bridge/config/extension.toml','rb'))['package']['version'])"
Write-Host "VERSION=$VERSION"
git tag -l "v$VERSION"        # must print nothing -- if it prints a tag, bump the version first
```

### 3. Build the release zip

```bash
cd exts
zip -r "../telekinesis.isaacsim.bridge-v$VERSION.zip" telekinesis.isaacsim.bridge
cd ..
unzip -l "telekinesis.isaacsim.bridge-v$VERSION.zip" | grep "telekinesis.isaacsim.bridge/config/extension.toml"
```

The `grep` should find a match -- confirms the zip contains a real, valid extension, not an empty or broken folder.

PowerShell (`zip`/`unzip` don't exist on Windows -- use `Compress-Archive` and .NET's `ZipFile`
instead; `Set-Location (git rev-parse --show-toplevel)` first so this works regardless of your
current directory):

```powershell
Set-Location (git rev-parse --show-toplevel)
$PY = "C:\Users\<you>\miniconda3\envs\<your-env>\python.exe"
$VERSION = & $PY -c "import tomllib; print(tomllib.load(open('exts/telekinesis.isaacsim.bridge/config/extension.toml','rb'))['package']['version'])"
Write-Host "VERSION=$VERSION"
Compress-Archive -Path "exts/telekinesis.isaacsim.bridge" -DestinationPath "telekinesis.isaacsim.bridge-v$VERSION.zip" -Force
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zip = [System.IO.Compression.ZipFile]::OpenRead((Resolve-Path "telekinesis.isaacsim.bridge-v$VERSION.zip"))
$zip.Entries.FullName | Where-Object { $_ -like "*extension.toml" }
$zip.Dispose()
```

The last line should print `telekinesis.isaacsim.bridge\config\extension.toml` (backslashes --
`Compress-Archive` stores Windows-style paths, unlike `zip`).

### 4. Tag and push

```bash
git tag -a "v$VERSION" -m "Release v$VERSION"
git push origin "v$VERSION"
```

### 5. Create the GitHub Release

On GitHub: **Releases ▸ Draft a new release**, pick the tag from step 4, title it `v$VERSION`, paste the matching section of `exts/telekinesis.isaacsim.bridge/docs/CHANGELOG.md` as the description, attach the zip from step 3, and publish.

### 6. Bump `develop`'s version

```bash
git checkout develop
git pull
# edit exts/telekinesis.isaacsim.bridge/config/extension.toml: bump the patch version
git add exts/telekinesis.isaacsim.bridge/config/extension.toml
git commit -m "chore: bump version after release v$VERSION"
git push origin develop
```

### 7. Publish the API reference

GitHub Pages is set to **Settings ▸ Pages ▸ Source: Deploy from a branch ▸ Branch: `main`,
folder: `/public`** (one-time setting) -- no GitHub Actions involved. GitHub serves whatever
is committed under `public/` on `main` directly, so publishing is just: regenerate, commit,
push.

```bash
python scripts/generate_openapi.py
git add public/openapi.json
git commit -m "chore: regenerate API reference for v$VERSION"
git push origin main
```

GitHub picks up the change and republishes within a minute or two of the push -- check
`https://telekinesis-ai.github.io/telekinesis-isaacsim-extension/` after pushing to confirm.

---

## API Overview

All endpoints accept and return JSON. Successful responses use `2xx`; errors use `4xx`/`5xx` with a detail message.

The full endpoint list is generated from the code, not hand-maintained here -- see [README.md#api-reference](README.md#api-reference) for the live (`/docs`, `/redoc`) and static (GitHub Pages) references. What follows is this project's own convention for *which* status code a given failure gets; it's not derivable from the schema itself.

### Error Handling

Errors always come back as `{"detail": "..."}` with one of the status codes below. Every failure the bridge anticipates raises `fastapi.HTTPException` with one of these codes (at the point in `comm/services/*.py` where the failure is detected); anything unanticipated falls through to a global handler in `comm/server.py` that still returns this same JSON shape at `500`, instead of a bare-text crash.

This follows FastAPI's own conventions rather than a separate error-code standard: `fastapi.HTTPException` is used directly wherever a service detects a specific failure, and `422` is left as FastAPI/pydantic's automatic default for request-body validation errors (missing/wrong-typed fields) rather than overridden. The specific case-to-code choices below (e.g. "no stage open" -> `409`, "bind/import failed" -> `422`) are this app's own convention, informed by common REST practice -- neither RFC 9110 nor the FastAPI docs prescribe a decision rule for picking among codes for a given case, they only define what each code generically means. See [FastAPI's error-handling docs](https://fastapi.tiangolo.com/tutorial/handling-errors/) and [RFC 9110 §15](https://www.rfc-editor.org/rfc/rfc9110.html#name-status-codes) for those definitions.

| Status | When we use it |
|---|---|
| `400` | The client sent a specific value that is invalid  |
| `404` | The referenced resource — an articulation_id or prim_path — is not currently registered/present. Nothing wrong with the request itself, the thing it points at just doesn't exist. |
| `409` | A prerequisite isn't met for this operation right now (no USD stage open) |
| `422` | The request was well-formed but semantically failed -- either FastAPI/pydantic's own automatic validation (missing/wrong-typed field), or an operation that couldn't complete for a deeper reason (a prim that won't bind, a URDF that fails to import) |
| `500` | The global exception-handler backstop for anything not explicitly translated |
| `501` | Routes mirrored from the spec but not wired up yet (`not_implemented()`) |

---

## Implementation Notes

- **Single-threaded on Isaac's loop.** All requests run on Isaac Sim's own asyncio loop - no extra threads. Blocking moves yield with `next_update_async()` so the server stays responsive to other requests while a move is in progress.
- **Articulation IDs are stable.** IDs are 1-based (`articulation1`, `articulation2`, …) and the same prim path always gets the same ID across repeated `PUT` calls.
- **Assembly is idempotent.** `POST /articulations/{id}/assemble_robot` for the same arm+gripper pair is a no-op; it returns `already_assembled=true`. The registry is cleared when the stage changes.
- **URDF import.** Pass `urdf_path` in the `PUT /articulations` body to have the bridge import the URDF and place it at `prim_path` automatically.
