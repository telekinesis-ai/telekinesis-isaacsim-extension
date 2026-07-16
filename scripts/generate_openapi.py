"""
Generate the bridge's OpenAPI schema (public/openapi.json) straight from the real
route/model definitions -- no hand-maintained API table to keep in sync.

Runnable OUTSIDE Isaac Sim: only ``comm.routers``, ``comm.dependencies``, and
``comm.models`` are imported, none of which touch ``omni``/``carb`` (same
Isaac-free set ``tests/test_bridge_smoke.py`` relies on). The package's real
``__init__.py`` does `from .extension import *`, which DOES import omni, so a
stand-in package object is registered first, exactly as the smoke test does, to
reach the submodules without running that import.

Run:  python scripts/generate_openapi.py
"""

import json
import sys
import types
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_EXT_ROOT = _REPO_ROOT / "exts" / "telekinesis.isaacsim.bridge"
sys.path.insert(0, str(_EXT_ROOT))

_bridge_pkg = types.ModuleType("telekinesis.isaacsim.bridge")
_bridge_pkg.__path__ = [str(_EXT_ROOT / "telekinesis" / "isaacsim" / "bridge")]
sys.modules["telekinesis.isaacsim.bridge"] = _bridge_pkg

from fastapi import FastAPI  # noqa: E402
from telekinesis.isaacsim.bridge.comm.routers import ALL_ROUTERS  # noqa: E402


def build_schema():
    """Assemble just the route/model surface (no services) and return its OpenAPI schema."""
    app = FastAPI(title="Telekinesis Isaac Sim Bridge")
    for router in ALL_ROUTERS:
        app.include_router(router)
    return app.openapi()


def main():
    out_path = _REPO_ROOT / "public" / "openapi.json"
    out_path.parent.mkdir(exist_ok=True)
    out_path.write_text(json.dumps(build_schema(), indent=2))
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
