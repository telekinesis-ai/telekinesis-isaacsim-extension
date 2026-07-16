"""
Minimal smoke tests for the bridge, runnable OUTSIDE Isaac Sim.

Only two modules are genuinely import-safe without Isaac Sim present:
``comm.services.general`` (no omni import at all) and ``comm.models`` (pure
pydantic). Everything else -- ``comm.server``, ``comm.services.articulations``,
and every ``core.*`` module -- imports ``omni``/``isaacsim``/``carb`` eagerly,
directly or transitively (``services.articulations`` imports ``core.articulation``
at module level, which imports ``omni.kit.app`` at module level), despite several
of their docstrings claiming lazy-import safety. That claim only actually holds
for the methods' *own* omni/pxr usage, not for the module's own dependency chain.
So this suite is intentionally narrow -- it is NOT a substitute for testing
against a live Isaac Sim process, only a fast check that the two truly
Isaac-free modules still import and behave correctly.

Run:  cd <repo root> && python -m pytest tests/ -v
(needs the same env the bridge runs in: fastapi, uvicorn, pydantic, websockets --
see exts/telekinesis.isaacsim.bridge/config/extension.toml)
"""
import sys
import types
from pathlib import Path

import pytest

_EXT_ROOT = Path(__file__).resolve().parent.parent / "exts" / "telekinesis.isaacsim.bridge"
sys.path.insert(0, str(_EXT_ROOT))

# telekinesis.isaacsim.bridge's real __init__.py does `from .extension import *`,
# which imports `omni` (Kit-only) -- required for Isaac Sim's extension loader to
# find the Extension class, but it means the package can't be imported at all
# outside Isaac Sim as-is. Register a stand-in package object instead of letting
# that real __init__.py run, so we can still reach the two Isaac-free submodules.
_bridge_pkg = types.ModuleType("telekinesis.isaacsim.bridge")
_bridge_pkg.__path__ = [str(_EXT_ROOT / "telekinesis" / "isaacsim" / "bridge")]
sys.modules["telekinesis.isaacsim.bridge"] = _bridge_pkg

from telekinesis.isaacsim.bridge.comm.services.general import GeneralService  # noqa: E402
from telekinesis.isaacsim.bridge.comm import models  # noqa: E402


# -- GeneralService (no omni import at all) ----------------------------------

def test_general_service_status():
    assert GeneralService().status() == {"status": "OK"}


# -- pydantic request models --------------------------------------------------

def test_joint_positions_request_requires_joint_positions():
    with pytest.raises(Exception):
        models.JointPositionsRequest()


def test_joint_positions_request_defaults():
    req = models.JointPositionsRequest(joint_positions=[0.0, 1.0])
    assert req.indices is None
    assert req.asynchronous is False


def test_create_articulation_request_requires_prim_path():
    with pytest.raises(Exception):
        models.CreateArticulationRequest()


def test_create_articulation_request_urdf_path_optional():
    req = models.CreateArticulationRequest(prim_path="/World/ur10e")
    assert req.urdf_path is None


def test_set_driven_joints_request_requires_joint_names():
    with pytest.raises(Exception):
        models.SetDrivenJointsRequest()


def test_joint_efforts_request_defaults():
    req = models.JointEffortsRequest(joint_efforts=[1.0, 2.0])
    assert req.indices is None


def test_set_enabled_request_requires_enabled():
    with pytest.raises(Exception):
        models.SetEnabledRequest()
    req = models.SetEnabledRequest(enabled=True)
    assert req.enabled is True
