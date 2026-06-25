# SPDX-License-Identifier: Apache-2.0
"""FastAPI dependency providers for the bridge routers.

Reference:
- https://github.com/fastapi/full-stack-fastapi-template/blob/master/backend/app/api/deps.py
- https://github.com/zhanymkanov/fastapi-best-practices

Each provider pulls one service off ``app.state``, where :class:`BridgeServer`
stashed it when it built the app. The routers declare exactly the service they
need via ``Depends(get_<name>_service)``, which keeps them thin and free of any
closure over the server instance. These read ``app.state`` at call time, so this
module imports no ``services`` (no import cycle: comm -> services is one-way).
"""

from fastapi import Request


def get_articulation_service(request: Request):
    """The shared :class:`..services.articulations.ArticulationService`."""
    return request.app.state.articulation_service


def get_stage_service(request: Request):
    """The shared :class:`..services.stage.StageService`."""
    return request.app.state.stage_service


def get_prim_service(request: Request):
    """The shared :class:`..services.prims.PrimService`."""
    return request.app.state.prim_service


def get_general_service(request: Request):
    """The shared :class:`..services.general.GeneralService`."""
    return request.app.state.general_service
