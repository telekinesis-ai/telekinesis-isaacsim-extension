# SPDX-License-Identifier: Apache-2.0
"""Stage service: the orchestration behind the ``/stage`` routes.

``StageService`` is the single source of the "current USD stage, or 409 if none"
rule (``stage()``); every other method builds on it to read/replace the open
stage, list articulation roots, get/set stage units, and drive the simulation
timeline. It holds no stage state -- the open stage lives in ``omni.usd``, not
here -- so one shared instance serves all requests.

It stays free of :mod:`..comm.models`: units are passed as a plain float and the
router wraps/unwraps the ``StageUnits`` model, keeping the dependency arrow
one-directional (comm -> services). omni/pxr imports are lazy so this module
imports outside Isaac Sim.
"""

from fastapi import HTTPException

from .asset_library import AssetLibraryService


class StageService:
    """Read and drive the open USD stage (scene, units, timeline).

    Catalogue assets added to the scene are downloaded through
    ``asset_library_service``.
    """

    def __init__(self, asset_library_service: AssetLibraryService):
        self._asset_library_service = asset_library_service

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

    async def add_to_scene(self, uri, prim_path, asset_info=None):
        """Reference the USD, OBJ, GLB or glTF asset at ``uri`` into the open stage.

        The asset is referenced onto ``prim_path``.

        The counterpart to :meth:`open_scene` for adding one device to a stage instead
        of replacing it: the asset arrives as one prim beside what is already there,
        so a cell someone has built survives bringing a conveyor or a sensor into it.
        An OBJ, GLB or glTF mesh is converted to USD first and arrives without physics.

        ``uri`` names an asset on the machine running Isaac Sim, not a URL.
        ``asset_info`` (``{"category", "manufacturer", "model"}``) instead names a
        catalogue asset, which is downloaded into the cache once and loaded from
        there. Exactly one of the two is given. Returns the prim path the asset
        arrived at, and leaves the timeline stopped.

        Raises:
            HTTPException: 400 unless exactly one of ``uri`` and ``asset_info`` is
                given; 404/502 from :meth:`AssetLibraryService.fetch_asset`; 422 if
                loading the asset fails.
        """
        from pathlib import Path

        from ...core.asset_loader import import_mesh_at, reference_usd_at

        if (uri is None) == (asset_info is None):
            raise HTTPException(
                status_code=400,
                detail="give either uri or asset_info, not both or neither",
            )
        if asset_info is not None:
            fetched = await self._asset_library_service.fetch_asset(
                asset_info["category"], asset_info["manufacturer"], asset_info["model"]
            )
            uri = fetched["local_path"]
        is_mesh = Path(uri).suffix.lower() in {".obj", ".glb", ".gltf"}
        load = import_mesh_at if is_mesh else reference_usd_at
        try:
            return await load(self.stage(), uri, prim_path)
        except RuntimeError as exc:
            # 422: well-formed request, but loading the asset itself failed.
            raise HTTPException(status_code=422, detail=str(exc)) from exc

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

    # Application settings that decide how much simulated time passes per second of wall clock.
    SPEED_SETTINGS = (
        "/app/runLoops/main/rateLimitEnabled",
        "/app/runLoops/main/rateLimitFrequency",
        "/app/runLoops/main/manualModeEnabled",
        "/app/player/useFixedTimeStepping",
        "/app/player/playSimulations",
        "/app/player/useFastMode",
        "/app/asyncRendering",
        "/app/asyncRenderingLowLatency",
        "/app/omni.usd/asyncHandshake",
        "/exts/isaacsim.core.throttling/enable_manualmode",
        "/exts/isaacsim.core.throttling/enable_async",
        "/rtx/rendermode",
        "/app/renderer/sleepMsOnFocus",
        "/app/renderer/sleepMsOutOfFocus",
        "/app/vsync",
        "/persistent/simulation/minFrameRate",
    )
    # A client may write settings under these paths only.
    SPEED_SETTING_PREFIXES = (
        "/app/runLoops/",
        "/app/player/",
        "/app/renderer/",
        "/app/vsync",
        "/app/asyncRendering",
        "/app/omni.usd/asyncHandshake",
        "/omni/replicator/asyncRendering",
        "/exts/isaacsim.core.throttling/",
        "/rtx/rendermode",
        "/persistent/simulation/",
    )

    async def simulation_speed(self, keys=(), measure=True) -> dict:
        """What decides the simulation's speed, and the speed measured now.

        Reads ``SPEED_SETTINGS`` and any further setting paths in ``keys``, the
        timeline's time codes per second, every physics scene's authored steps per
        second (``None`` if not authored), and whether the active viewport renders.
        It then waits for up to 30 application updates, at most one second, and
        reports ``update_hz``, the updates per second of wall clock, and
        ``simulation_rate``, the simulated seconds per second of wall clock
        (``None`` unless the timeline is playing). With ``measure`` false it
        returns at once and both are ``None``. Works without an open stage.
        """
        import time

        import carb.settings
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import PhysxSchema, UsdPhysics

        settings = carb.settings.get_settings()
        timeline = omni.timeline.get_timeline_interface()
        values = {key: settings.get(key) for key in (*self.SPEED_SETTINGS, *keys)}
        steps = {}
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            for prim in stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    attr = PhysxSchema.PhysxSceneAPI(prim).GetTimeStepsPerSecondAttr()
                    steps[str(prim.GetPath())] = attr.Get() if attr and attr.HasAuthoredValue() else None
        viewport = self._active_viewport()
        app = omni.kit.app.get_app()
        playing = timeline.is_playing()
        updates, started, simulated = 0, time.perf_counter(), SimulationManager.get_simulation_time()
        while measure and updates < 30 and time.perf_counter() - started < 1.0:
            await app.next_update_async()
            updates += 1
        elapsed = time.perf_counter() - started if measure else 0.0
        advanced = SimulationManager.get_simulation_time() - simulated
        return {
            "settings": values,
            "time_codes_per_second": timeline.get_time_codes_per_second(),
            "physics_steps_per_second": steps,
            "viewport_updates": None if viewport is None else bool(viewport.updates_enabled),
            "timeline": self.simulation_state()["timeline"],
            "update_hz": updates / elapsed if elapsed > 0 else None,
            # simulated time restarts from zero on play: a negative step is not a rate
            "simulation_rate": advanced / elapsed if playing and elapsed > 0 and advanced >= 0 else None,
        }

    async def set_simulation_speed(
        self,
        settings=None,
        time_codes_per_second=None,
        physics_steps_per_second=None,
        viewport_updates=None,
    ) -> None:
        """Change what decides the simulation's speed; an argument left ``None`` is not touched.

        ``settings`` are application settings under ``SPEED_SETTING_PREFIXES`` (422
        for any other path); those under ``/persistent/`` survive a restart, the rest
        last until Isaac Sim closes; ``/app/player/useFastMode`` is also set on the
        timeline, which otherwise restores it on the next play.
        ``time_codes_per_second`` is the timeline's rate,
        authored on the open stage. ``physics_steps_per_second`` is authored on every
        physics scene and takes effect on the next play. ``viewport_updates`` turns
        the active viewport's rendering on or off; camera render products are not
        affected. Stage changes are made in the current edit layer and not saved.
        Raises 409 for a stage change with no stage open.
        """
        import carb.settings
        import omni.timeline
        from pxr import PhysxSchema, UsdPhysics

        refused = [key for key in (settings or {}) if not key.startswith(self.SPEED_SETTING_PREFIXES)]
        if refused:
            raise HTTPException(
                status_code=422,
                detail=f"not a simulation speed setting: {', '.join(refused)}; "
                f"allowed paths start with {', '.join(self.SPEED_SETTING_PREFIXES)}",
            )
        if physics_steps_per_second is not None or time_codes_per_second is not None:
            stage = self.stage()
        for key, value in (settings or {}).items():
            carb.settings.get_settings().set(key, value)
        fast_mode = (settings or {}).get("/app/player/useFastMode")
        if fast_mode is not None:
            # The timeline keeps its own copy of this one and writes it back to the
            # setting on every play, so the setting alone comes back on the next play.
            timeline = omni.timeline.get_timeline_interface()
            set_fast_mode = getattr(timeline, "set_play_every_frame", None) or timeline.set_fast_mode
            set_fast_mode(bool(fast_mode))
        if time_codes_per_second is not None:
            omni.timeline.get_timeline_interface().set_time_codes_per_second(time_codes_per_second)
        if physics_steps_per_second is not None:
            scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
            if not scenes:
                scenes = [UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim()]
            for prim in scenes:
                PhysxSchema.PhysxSceneAPI.Apply(prim).CreateTimeStepsPerSecondAttr().Set(physics_steps_per_second)
        if viewport_updates is not None:
            viewport = self._active_viewport()
            if viewport is None:
                raise HTTPException(status_code=409, detail="no active viewport")
            viewport.updates_enabled = viewport_updates

    @staticmethod
    def _active_viewport():
        """The active viewport, or ``None`` when Isaac Sim runs without one."""
        try:
            from omni.kit.viewport import utility as viewport_utility

            return viewport_utility.get_active_viewport()
        except Exception:  # pylint: disable=broad-exception-caught
            return None

    async def set_solve_articulation_contact_last(self, enabled: bool) -> None:
        """Stop simulation and set contact ordering on every physics scene.

        Creates ``/physicsScene`` if none exists. The timeline remains stopped;
        its next play creates the solver with the new order. Changes are authored
        in the current edit layer without saving it. Raises 409 if no stage is open.
        """
        import omni.kit.app
        import omni.timeline
        from pxr import PhysxSchema, Sdf, UsdPhysics

        stage = self.stage()
        # PhysX reads this immutable scene flag when simulation is initialized.
        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if not scenes:
            scenes = [UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim()]
        for prim in scenes:
            scene_api = PhysxSchema.PhysxSceneAPI.Apply(prim)
            if hasattr(scene_api, "CreateSolveArticulationContactLastAttr"):
                scene_api.CreateSolveArticulationContactLastAttr(enabled)
            else:
                # Isaac Sim 5.1 exposes this setting as a custom USD attribute.
                prim.CreateAttribute(
                    "physxScene:solveArticulationContactLast",
                    Sdf.ValueTypeNames.Bool,
                    custom=True,
                ).Set(enabled)

    async def set_cpu_simulation(self) -> None:
        """Stop simulation and switch physics to the CPU on every physics scene.

        Selects the CPU simulation device, disables GPU dynamics and uses the MBP
        broadphase on every physics scene. Creates ``/physicsScene`` if none
        exists. The timeline remains stopped; its next play creates the solver
        with these settings. Scene changes are authored in the current edit layer
        without saving it. Raises 409 if no stage is open.
        """
        import omni.kit.app
        import omni.timeline
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import PhysxSchema, UsdPhysics

        stage = self.stage()
        # PhysX reads these scene settings when simulation is initialized.
        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if not scenes:
            scenes = [UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim()]
        SimulationManager.set_device("cpu")
        for prim in scenes:
            scene_api = PhysxSchema.PhysxSceneAPI.Apply(prim)
            scene_api.CreateEnableGPUDynamicsAttr().Set(False)
            scene_api.CreateBroadphaseTypeAttr().Set("MBP")

    def set_guides_visible(self, visible: bool) -> None:
        """Show or hide guide-purpose prims, such as colliders' guide meshes, in the viewport.

        This is the viewport's Display > Show By Purpose > Guide toggle, so it applies
        to every guide prim in the stage. The setting is persistent and survives an
        Isaac Sim restart.
        """
        import carb.settings

        carb.settings.get_settings().set("/persistent/app/hydra/displayPurpose/guide", visible)
