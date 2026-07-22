"""
Generate ``public/openapi.json`` from the bridge's FastAPI routers.

FastAPI references:
- Include routers in an application:
  https://fastapi.tiangolo.com/tutorial/bigger-applications/
- Generate the OpenAPI schema with ``app.openapi()``:
  https://fastapi.tiangolo.com/how-to/extending-openapi/

Python import reference for the temporary package workaround:
- ``sys.modules`` and package ``__path__``:
  https://docs.python.org/3/reference/import.html

The temporary package lets this script import the API routers without executing
``bridge/__init__.py``, which imports Isaac Sim modules such as ``omni``.

Run:

    python scripts/generate_openapi.py
"""

import json
import sys
import types
from pathlib import Path

from fastapi import FastAPI


repo_root = Path(__file__).resolve().parent.parent
ext_root = repo_root / "exts" / "telekinesis.isaacsim.bridge"
package_root = ext_root / "telekinesis" / "isaacsim" / "bridge"

sys.path.insert(0, str(ext_root))

# Skip bridge/__init__.py because it imports Isaac Sim modules.
bridge_package = types.ModuleType("telekinesis.isaacsim.bridge")
bridge_package.__path__ = [str(package_root)]
sys.modules["telekinesis.isaacsim.bridge"] = bridge_package

from telekinesis.isaacsim.bridge.comm import routers  # noqa: E402

# Generate OpenAPI schema from FastAPI routers and write to public/openapi.json
app = FastAPI(title="Telekinesis Isaac Sim Bridge")

# Include all routers
for router in routers.ALL_ROUTERS:
    app.include_router(router)

# Generate OpenAPI schema and write to public/openapi.json
output = repo_root / "public" / "openapi.json"
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(
    json.dumps(app.openapi(), indent=2),
    encoding="utf-8",
)

print(f"Wrote {output}")
