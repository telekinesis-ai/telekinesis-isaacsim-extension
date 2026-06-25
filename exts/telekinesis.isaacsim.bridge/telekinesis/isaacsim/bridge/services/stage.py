# SPDX-License-Identifier: Apache-2.0
"""Stage service: the orchestration behind the ``/stage`` routes.

``StageService`` is the single source of the "current USD stage, or 409 if none"
rule (``stage()``); every other method builds on it to read/replace the open
stage, list articulation roots, get/set stage units, and drive the simulation
timeline. Stateless -- the open stage lives in ``omni.usd``, not here -- so one
shared instance serves all requests.

It stays free of :mod:`..comm.models`: units are passed as a plain float and the
router wraps/unwraps the ``StageUnits`` model, keeping the dependency arrow
one-directional (comm -> services). omni/pxr imports are lazy so this module
imports outside Isaac Sim.
"""

from fastapi import HTTPException


class StageService:
    """Read and drive the open USD stage (scene, units, timeline)."""

    def stage(self):
        """Current USD stage, or 409 if no stage is open.

        The one place the "no stage" rule lives; other stage/prim logic builds on it.
        """
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise HTTPException(status_code=409, detail="no USD stage is open")
        return stage

    def get_active_scene(self):
        """URI/identifier of the open USD stage (empty string if none)."""
        import omni.usd

        return omni.usd.get_context().get_stage_url() or ""

    async def open_scene(self, uri):
        """Open the USD stage at ``uri`` (replaces the current stage)."""
        import omni.usd

        success, error = await omni.usd.get_context().open_stage_async(uri)
        if not success:
            raise HTTPException(status_code=400, detail=f"could not open '{uri}': {error}")

    def list_motion_groups(self):
        """Prim paths of every articulation root in the stage (potential robots)."""
        from pxr import Usd, UsdPhysics

        stage = self.stage()
        return [
            prim.GetPath().pathString
            for prim in Usd.PrimRange(stage.GetPseudoRoot())
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]

    def get_units(self):
        """Linear scale of the stage in meters per unit."""
        from pxr import UsdGeom

        return UsdGeom.GetStageMetersPerUnit(self.stage())

    def update_units(self, meters_per_unit):
        """Set the stage's meters-per-unit scale."""
        from pxr import UsdGeom

        UsdGeom.SetStageMetersPerUnit(self.stage(), meters_per_unit)

    def timeline_action(self, action):
        """Drive the simulation timeline: play / pause / stop.

        ``action`` is a :class:`..comm.models.TimelineAction` (its values match the
        timeline interface method names).
        """
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        {
            "play": timeline.play,
            "pause": timeline.pause,
            "stop": timeline.stop,
        }[action.value]()

    def simulation_state(self):
        """Current timeline state: ``playing`` / ``paused`` / ``stopped``."""
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing():
            state = "playing"
        elif timeline.is_stopped():
            state = "stopped"
        else:
            state = "paused"
        return {"timeline": state}
