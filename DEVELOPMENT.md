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
   c:/Users/<you>/Documents/workspace/telekinesis-isaacsim-extension/exts
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
pylint --max-line-length 100 exts/telekinesis.isaacsim.bridge/telekinesis examples
```

`ruff check` covers style, unused imports, and import ordering; `ruff format` auto-formats to match (run it after `check` so anything `check` flagged is already fixed first); `pylint` catches a broader set of correctness issues. `ruff check` and `pylint` also run in CI (see `.github/workflows/verify.yml`) and block merging into `main`/`develop` if they fail -- `ruff format` is local-only for now, so run it before every PR to keep formatting consistent.

---

## Manual Release (dry run before enabling CI/CD)

`.github/workflows/release.yml` automates everything below and fires for real on every push to `main` -- a public GitHub Release, a public GitHub Pages doc site. Run the sequence by hand once first to verify each step actually works before trusting the automation with a real release.

### 1. Verify

Covered by [Merge to main](#merge-to-main) (ruff, pylint), plus:

```bash
pytest tests/ -v
python scripts/generate_openapi.py   # should exit 0 and write public/openapi.json
```

### 2. Check the version hasn't already been released

```bash
VERSION=$(python -c "import tomllib; print(tomllib.load(open('exts/telekinesis.isaacsim.bridge/config/extension.toml','rb'))['package']['version'])")
echo "$VERSION"
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

```bash
python scripts/generate_openapi.py
```

Actually publishing `public/` to GitHub Pages needs a one-time repo setting (**Settings ▸ Pages ▸ Build and deployment ▸ Source: GitHub Actions**) that only takes effect once the `publish-docs` job in `release.yml` runs for real -- there's no clean manual equivalent for this one step. Verify the *contents* locally instead (open `public/index.html` in a browser next to the freshly generated `public/openapi.json`) and defer the actual publish to CI.

### Once every step above has been verified by hand

Push to `main` (or merge a PR into it) and let `release.yml` run steps 2–7 automatically on every future release.

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
