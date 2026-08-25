# Development

This is a developer's guide to run and contribute to the extension.

## Prerequisites
- It is recommended to create a conda environment.
  ```bash
  conda create -n telekinesis-isaacsim-bridge python=3.12
  ```

- Install NVIDIA Isaac Sim 5.1.0 or later. The required Python version depends on the Isaac Sim release, match it before installing. Tested against **5.1.0.0** and **6.0.1.0**:

  | Isaac Sim | Python | Install |
  |-----------|--------|---------|
  | 5.1.0.0 | 3.11 | `pip install isaacsim[all,extscache]==5.1.0.0 --extra-index-url https://pypi.nvidia.com` |
  | 6.0.1.0 | 3.12 | `pip install isaacsim[all,extscache]==6.0.1.0 --extra-index-url https://pypi.nvidia.com` |

  6.0.1.0 is a bug-fix release over 6.0.0.x and is the recommended 6.x version to develop against.
  Note that 6.0.0 deprecated `isaacsim.sensors.camera` (the module `core/camera.py` wraps) in
  favour of `isaacsim.sensors.experimental.rtx`; it still works on 6.0.1.0 but will need migrating
  in a future Isaac Sim release.

The extension lives at:
```
exts/telekinesis.isaacsim.bridge
```

## Enable the Extension via the Extension Manager UI

1. Start Isaac Sim (type isaacsim in terminal)
2.  Open **Window ▸ Extensions ▸ Settings**.
3. Under **Extension Search Paths**, click **+** and add the absolute path to this repo's `exts` folder, e.g.:

   ```
   c:/Users/<you>/<path>/telekinesis-isaacsim-extension/exts
   ```
> Use forward slashes in paths — Kit is unreliable with Windows backslashes.
4. Search for `telekinesis.isaacsim.bridge` (Under Third Party) and enable it. Tick **Autoload** to enable it on every launch.

## Verify

Install the client-side dependencies used by the examples before running them:

```bash
pip install -r requirements.txt
```

The extension itself declares the server-side runtime dependencies in `exts/telekinesis.isaacsim.bridge/config/extension.toml`, including `fastapi`, `uvicorn`, `pydantic`, and `websockets`.

```bash
curl http://127.0.0.1:8766/status
# {"status":"OK"}
```

## Manual Deployment

Follow the steps below to release a new version of the extension.

### 1. Linting, Static Checks and API Docs Generation

Run the following before merging to main.
```bash
pip install ruff pylint
ruff check --line-length 100 .
ruff format --line-length 100 .
pylint --max-line-length 100 --ignored-modules=omni,carb,isaacsim,pxr,usd --disable=duplicate-code,attribute-defined-outside-init,import-outside-toplevel,unused-argument,broad-exception-caught,protected-access,too-many-arguments,too-many-positional-arguments,too-many-locals,too-many-public-methods,too-many-instance-attributes,too-many-lines exts/telekinesis.isaacsim.bridge/telekinesis examples
python scripts/generate_openapi.py # should exit 0 and write public/openapi.json
```
On succesful completion, merge to main, and proceed to release.
<details>
<summary>Why these Pylint checks are ignored</summary>

`--ignored-modules` tells Pylint not to fully resolve
`omni`, `carb`, `isaacsim`, `pxr`, and `usd` (the Isaac robot schema lives in
`usd.schema.isaac`).

These packages are provided by Isaac Sim. Without Isaac Sim installed,
Pylint may report unresolved imports or fail to inspect `pxr` members.
That is expected and is not necessarily a real code issue.

Every disabled check is intentional:

- `duplicate-code`: example scripts intentionally repeat small helpers and
  argument-parsing boilerplate so each example remains standalone.
- `attribute-defined-outside-init`: Kit initializes extension state in
  `on_startup()` rather than `__init__()`.
- `import-outside-toplevel`: Isaac Sim modules are imported lazily so some
  modules can still be imported outside Isaac Sim.
- `unused-argument`: required by FastAPI route signatures and Kit callbacks.
- `broad-exception-caught`: broad exceptions are translated into typed errors
  at application boundaries.
- `protected-access`: used where Isaac Sim currently provides no public accessor.
- `too-many-arguments` and `too-many-positional-arguments`: assembly functions
  genuinely require several robot, stage, mount, and offset parameters.
- `too-many-locals`: applies to the same orchestration functions.
- `too-many-public-methods` and `too-many-instance-attributes`: service classes
  intentionally mirror Isaac Sim APIs and Kit subscription state.
- `too-many-lines`: `routers.py` intentionally keeps the full API route surface
  in one file.

`ruff check` handles style, unused imports, and import ordering.
`ruff format` formats the code.
`pylint` performs broader static checks.

There is currently no CI for these checks, so run them manually before each PR.

</details>


### 2. Prepare the release

#### 2.1. One-time repository setup

The repository must be public and carry the `omniverse-kit-extension` topic — that topic is how
NVIDIA's nightly crawler finds the extension at all.

```bash
gh repo edit --visibility public --accept-visibility-change-consequences
gh repo edit --add-topic omniverse-kit-extension

# Verify both landed.
gh repo view --json visibility,repositoryTopics
```

Then check the metadata in `exts/telekinesis.isaacsim.bridge/config/extension.toml`:

- package name and version are correct;
- repository URL is correct;
- supported Kit, Python, and platform targets are declared.

#### 2.2. Sync `main` and `develop`

Do this before bumping anything, so the release is cut from an up-to-date branch and any
hotfix that landed on `main` is already in `develop`.

```bash
git fetch origin

git checkout main
git pull --ff-only origin main

git checkout develop
git pull --ff-only origin develop

# Bring any hotfixes/changes on main into develop first.
git merge main
git push origin develop
```

`--ff-only` is deliberate: it fails instead of creating a surprise merge commit when your
local branch has drifted, so you find out before the release rather than during it.

#### 2.3. Bump the version and package

1. Bump `version` in `exts/telekinesis.isaacsim.bridge/config/extension.toml`, then read it
   back so the rest of the steps can reuse it:

```bash
VERSION=$(python -c "import tomllib; print(tomllib.load(open('exts/telekinesis.isaacsim.bridge/config/extension.toml','rb'))['package']['version'])")
echo "VERSION=$VERSION"
```

```powershell
$VERSION = python -c "import tomllib; print(tomllib.load(open('exts/telekinesis.isaacsim.bridge/config/extension.toml','rb'))['package']['version'])"
Write-Host "VERSION=$VERSION"
```

2. Add a `## [$VERSION]` section to `exts/telekinesis.isaacsim.bridge/docs/CHANGELOG.md`, and
   document any new routes in `exts/telekinesis.isaacsim.bridge/docs/README.md` (WebSocket
   routes especially — they never appear in the generated OpenAPI spec).

3. Commit the bump on `develop` and push it, so the squash below picks it up:

```bash
git add exts/telekinesis.isaacsim.bridge/config/extension.toml exts/telekinesis.isaacsim.bridge/docs
git commit -m "Release v$VERSION"
git push origin develop
```

4. Package the extension from the repository root using the helper script (use git bash within
   the conda environment if PowerShell fails):

```bash
bash -x scripts/package-extension.sh
```

The script creates a ZIP archive under `packages/` using the release naming convention:

```text
telekinesis-ai-telekinesis-isaacsim-extension-windows-x86_64-v<VERSION>.zip
```

#### 2.4. Release `develop` into `main`, then tag

```bash
git checkout main
git merge --squash develop
git commit -m "Merged develop into main"
git push origin main

# Tag the release commit that now exists ON MAIN.
git tag -a "v$VERSION" -m "Release v$VERSION"
git push origin "v$VERSION"

# IMPORTANT: reconnect the histories.
git checkout develop
git merge main
git push origin develop
```

Two things here are easy to get wrong:

- **Tag after the squash, not before.** `git merge --squash` creates a *new* commit on `main`
  rather than replaying develop's. A tag created on `develop` beforehand points at a commit
  that is not the released one, and the GitHub release then attaches to the wrong revision.
- **Reconnect the histories afterwards.** A squash merge leaves no merge commit, so git still
  considers `develop` unmerged and the next release re-offers every commit already on `main`.
  The closing `git checkout develop && git merge main` is what prevents that.

#### 2.5. Publish the GitHub release

The tag must already be pushed (previous step) — `gh` attaches the release to the existing tag.
Requires the [GitHub CLI](https://cli.github.com/), authenticated once with `gh auth login`.

```bash
gh release create "v$VERSION" \
  "packages/telekinesis-ai-telekinesis-isaacsim-extension-windows-x86_64-v$VERSION.zip" \
  --title "v$VERSION" \
  --notes-file exts/telekinesis.isaacsim.bridge/docs/CHANGELOG.md
```

Verify the release and its asset:

```bash
gh release view "v$VERSION"
```

Sanity-check that the tag exists on the remote and points where you expect:

```bash
git ls-remote --tags origin | grep "v$VERSION"
git log -1 --oneline "v$VERSION"
```

NVIDIA's publishing pipeline runs nightly. Check the Community Registry the following day and search for:

```text
telekinesis.isaacsim.bridge
```

### 2.6. Official NVIDIA references

- Publishing: https://docs.omniverse.nvidia.com/kit/docs/kit-extension-template-cpp/latest/index.html#publishing
- Community Registry: https://docs.omniverse.nvidia.com/kit/docs/kit-registry-reference/latest/community/extensions.html
- Extension metadata and compatibility: https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/extensions_advanced.html

## Error Handling

Errors always come back as `{"detail": "..."}` with one of the status codes below. That holds on the camera image routes too, which answer `application/octet-stream` on success (see **Camera image payloads** in the extension's `docs/README.md`) but JSON on failure, so a client picks how to read a response by its content type rather than by its status alone. Every failure the bridge anticipates raises `fastapi.HTTPException` with one of these codes (at the point in `comm/services/*.py` where the failure is detected); anything unanticipated falls through to a global handler in `comm/server.py` that still returns this same JSON shape at `500`, instead of a bare-text crash.

This follows FastAPI's own conventions rather than a separate error-code standard: `fastapi.HTTPException` is used directly wherever a service detects a specific failure, and `422` is left as FastAPI/pydantic's automatic default for request-body validation errors (missing/wrong-typed fields) rather than overridden. The specific case-to-code choices below (e.g. "no stage open" -> `409`, "bind/import failed" -> `422`) are this app's own convention, informed by common REST practice -- neither RFC 9110 nor the FastAPI docs prescribe a decision rule for picking among codes for a given case, they only define what each code generically means. See [FastAPI's error-handling docs](https://fastapi.tiangolo.com/tutorial/handling-errors/) and [RFC 9110 §15](https://www.rfc-editor.org/rfc/rfc9110.html#name-status-codes) for those definitions.

| Status | When we use it |
|---|---|
| `400` | The client sent a specific value that is invalid  |
| `404` | The referenced resource — an articulation_id or prim_path — is not currently registered/present. Nothing wrong with the request itself, the thing it points at just doesn't exist. |
| `409` | A prerequisite isn't met for this operation right now (no USD stage open) |
| `422` | The request was well-formed but semantically failed -- either FastAPI/pydantic's own automatic validation (missing/wrong-typed field), or an operation that couldn't complete for a deeper reason (a prim that won't bind, a URDF that fails to import) |
| `500` | The global exception-handler backstop for anything not explicitly translated |
| `501` | Routes mirrored from the spec but not wired up yet (`not_implemented()`) |



## Implementation Notes

- **Camera images are binary.** `POST /cameras/{id}/capture` and the `rgb`/`rgba`/`depth`/`pointcloud` getters answer one binary frame (`comm/binary.py`) rather than JSON nested lists. Because requests run on Isaac's own loop, the seconds a JSON encode of a 720p frame took were seconds in which nothing rendered or stepped. `core/camera.py` therefore hands render outputs back as C-contiguous host numpy, and the routers encode them; every other camera read stays JSON-ready Python.
- **Single-threaded on Isaac's loop.** All requests run on Isaac Sim's own asyncio loop - no extra threads. Blocking moves yield with `next_update_async()` so the server stays responsive to other requests while a move is in progress.
- **Articulation IDs are stable.** IDs are 1-based (`articulation1`, `articulation2`, …) and the same prim path always gets the same ID across repeated `PUT` calls.
- **Assembly is idempotent.** `POST /articulations/{id}/assemble_robot` for the same arm+gripper pair is a no-op; it returns `already_assembled=true`. The registry is cleared when the stage changes.
- **URDF import.** Pass `urdf_path` in the `PUT /articulations` body to have the bridge import the URDF and place it at `prim_path` automatically.
