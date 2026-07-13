# Development

How to run the bridge extension from this source tree (your local dev copy) instead
of the published version from the community registry.

The extension lives at:

```
exts/telekinesis.isaacsim.bridge
```

Isaac Sim discovers extensions by **search path**, and the path you register is the
**parent `exts/` directory**, not the extension folder itself.

> Use forward slashes in paths — Kit is unreliable with Windows backslashes.

---

## Option A — Extension Manager UI (quickest)

1. Open Isaac Sim → **Window ▸ Extensions**.
2. Click the hamburger/gear menu (top-left of the Extensions panel) → **Settings**.
3. Under **Extension Search Paths**, click **+** and add the absolute path to this
   repo's `exts` folder, e.g.:

   ```
   c:/Users/<you>/Documents/workspace/telekinesis-isaacsim-extension/exts
   ```

4. Back in the extension list, search for `telekinesis.isaacsim.bridge`. It now
   appears under the **THIRD PARTY / user** extensions.
5. Toggle it **ON**. Tick **Autoload** to enable it on every launch.

Your local copy takes priority over the registry version, so your source edits are
what run. Enable hot-reload (⟳) while developing to pick up changes without a restart.

---

## Option B — Command line (`--ext-folder`)

Launch Isaac Sim pointing at the folder:

```powershell
# from your Isaac Sim install dir
.\isaac-sim.bat --ext-folder "c:/Users/<you>/Documents/workspace/telekinesis-isaacsim-extension/exts" --enable telekinesis.isaacsim.bridge
```

---

## Option C — Persist it in a kit/config file

Add to your app's `.kit` file (or `user.config.json`) so it's always available:

```toml
[settings]
exts.folders = ["c:/Users/<you>/Documents/workspace/telekinesis-isaacsim-extension/exts"]

[dependencies]
"telekinesis.isaacsim.bridge" = {}
```

---

## Verify

On first enable, Kit auto-installs `fastapi`, `uvicorn`, and `pydantic` via `pipapi`
(needs internet — if you're offline, pre-install them into Isaac's Python).

Once the extension is enabled and the simulation is playing:

```bash
curl http://127.0.0.1:8766/status
# {"status":"OK"}
```

See [README.md](README.md) for the full API and client examples.
