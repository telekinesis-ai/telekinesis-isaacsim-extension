# SPDX-License-Identifier: Apache-2.0
"""The shared flow of the Add Robot / Add Tool / Add Mesh windows."""

import abc
import asyncio
from pathlib import Path

import omni.usd
from fastapi import HTTPException
from omni import ui
from omni.kit import notification_manager
from pxr import Tf
from isaacsim.gui.components import element_wrappers

from ..comm import models
from ..comm.services.articulations import ArticulationService
from ..comm.services.asset_library import AssetLibraryService
from ..core import asset_loader, robot_assembler
from .asset_ghost import AssetGhost

# Shown in the Manufacturer dropdown for catalogue entries listed without a manufacturer.
GENERIC_MANUFACTURER_LABEL = "Generic"
# Width, in pixels, of every button in the windows' centered button rows.
BUTTON_WIDTH = 180


class AddAssetWindow(abc.ABC):
    """A window that adds one asset from the Telekinesis asset library to the stage.

    The user picks a manufacturer and model from the catalogue categories the window
    covers; the prim the asset is added at, the Attaching Prim, is the right-clicked
    prim and cannot be edited. While the window is open, a see-through ghost of the
    selected model shows where Add will put it and follows every change of the model
    and the window's other fields; the ghost is never saved, cannot be selected and
    takes no part in physics. A model
    that cannot be previewed leaves the reason in the status line and can still be
    added. The window closes when another stage is opened. Pressing Add checks that a model is
    selected and that the Attaching Prim exists; if not, the window stays open and
    its status line says why. Otherwise the window closes, which removes the ghost,
    and the selected catalogue entry is handed to :meth:`add_selected_asset`, which
    loads it through the bridge services by its ``asset_info`` -- the services
    download it into the local cache once -- so the result is the same as an HTTP
    client doing it. Progress and the outcome are shown as notifications, an error as
    a warning that stays until dismissed. Add is disabled while an add runs. Every add
    creates a new prim: a name already taken in the stage gets a numeric suffix.

    A subclass names its categories, may add input fields, places the ghost where its
    add puts the asset, and implements the add itself.

    Example:
        window = AddRobotWindow(bridge_server.asset_library_service,
                                bridge_server.articulation_service)
        window.show("/World")
    """

    def __init__(
        self,
        title: str,
        asset_library_service: AssetLibraryService,
        articulation_service: ArticulationService,
    ):
        self._asset_library_service = asset_library_service
        self._articulation_service = articulation_service
        # Entries of every category this window covers, each tagged with its "category".
        self._assets: list[dict] = []
        # The catalogue load running or last run; None until the window is first shown.
        self._catalogue_load: asyncio.Future | None = None
        self._ghost = AssetGhost()
        # Main file of a catalogue asset -> the USD its ghost shows, built once per session.
        self._preview_asset_paths: dict[str, Path] = {}
        # The catalogue entry the ghost shows, None while no ghost is shown.
        self._ghost_asset: dict | None = None
        # The ghost refresh running or last run, and whether a change arrived since it
        # last read the window.
        self._ghost_refresh: asyncio.Future | None = None
        self._ghost_outdated = False

        self._window = ui.Window(title, width=420, height=0, visible=False)
        self._window.set_visibility_changed_fn(self._on_visibility_changed)
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._manufacturer_dropdown = element_wrappers.DropDown(
                    "Manufacturer",
                    on_selection_fn=self._on_manufacturer_selected,
                )
                self._model_dropdown = element_wrappers.DropDown(
                    "Model",
                    populate_fn=self._models_of_selected_manufacturer,
                    on_selection_fn=lambda _: self.update_ghost(),
                )
                # Read-only: the right-clicked prim the window was opened on.
                self._attaching_prim_field = element_wrappers.StringField(
                    "Attaching Prim",
                    tooltip="Prim the asset is added at",
                    read_only=True,
                )
                self.build_extra_fields()
                # The button row is centered by a stretching spacer on either side.
                with ui.HStack(spacing=5, height=0):
                    ui.Spacer()
                    self.build_extra_buttons()
                    self._add_button = ui.Button(
                        "Add", width=BUTTON_WIDTH, clicked_fn=self._on_add_clicked
                    )
                    ui.Spacer()
                self._status_label = ui.Label("", word_wrap=True)

    @abc.abstractmethod
    def asset_categories(self) -> list[str]:
        """Return the catalogue categories this window offers, e.g. ``["robots/manipulators"]``.

        Called when the catalogue is first loaded. The entries of all returned
        categories share the Manufacturer and Model dropdowns.
        """

    def build_extra_fields(self):
        """Build input fields shown below Attaching Prim. Optional; the default builds none.

        Called once from the constructor, inside the window's vertical layout, so
        widgets created here land in it. Store anything :meth:`add_selected_asset`
        reads on ``self``. A field that changes where the asset lands must call
        :meth:`update_ghost` when its value changes.
        """

    def build_extra_buttons(self):
        """Build buttons shown left of Add, in the same row. Optional; the default
        builds none.

        Called once from the constructor, inside that centered horizontal row; give
        each button a width of :data:`BUTTON_WIDTH`.
        """

    def reset_extra_fields(self):
        """Set the fields built by :meth:`build_extra_fields` back to their defaults. Optional.

        Called each time the window is opened while no add is running, so every add
        starts from the defaults instead of the values of the previous one. The default
        resets nothing.
        """

    async def place_ghost(self, attaching_prim_path: str):
        """Move the ghost to where :meth:`add_selected_asset` would put the selected asset.

        Called on every ghost refresh, once the ghost shows the selected asset and
        ``attaching_prim_path`` names a prim in the stage. Must place the ghost through
        ``self._ghost`` (:meth:`AssetGhost.place_under` or
        :meth:`AssetGhost.place_on_mount`). May raise the services' ``HTTPException`` or
        the core layer's ``RuntimeError``; the ghost is then removed and the message
        shown as the reason there is no preview.

        Optional; the default stands the asset at the Attaching Prim's origin, as it
        is loaded under it.
        """
        self._ghost.place_under(self._world_transform(attaching_prim_path))

    @abc.abstractmethod
    async def add_selected_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Load the selected catalogue asset into the stage at ``attaching_prim_path``.

        Called when Add is pressed, with ``attaching_prim_path`` naming a prim in the
        stage. ``asset`` is the selected catalogue entry (``manufacturer``, ``model``,
        ``bundle``, ``category`` and any extra index fields). Must load it through the
        bridge services by passing ``asset`` as their ``asset_info``, so they download
        it and apply the catalogue's defaults, and place new prims at
        :meth:`_free_destination_path`. May raise the services' ``HTTPException`` or
        the core layer's ``RuntimeError``, whose message is shown to the user.

        Returns the message shown on success.
        """

    def show(self, attaching_prim_path: str):
        """Open the window with ``attaching_prim_path`` pre-filled as the Attaching Prim.

        The catalogue is loaded on the first open, and retried on later opens until
        it yields any asset. Opening the window while it loads starts no second load.
        The extra fields are reset to their defaults unless an add is running.
        """
        self._attaching_prim_field.set_value(attaching_prim_path)
        # Add is disabled exactly while an add runs, and that add still reads the fields.
        if self._add_button.enabled:
            self.reset_extra_fields()
        self._window.visible = True
        if not self._assets and (
            self._catalogue_load is None or self._catalogue_load.done()
        ):
            self._catalogue_load = asyncio.ensure_future(self._load_catalogue())
        self.update_ghost()

    def update_ghost(self):
        """Bring the ghost in line with the window: its model, Attaching Prim and fields.

        Returns at once; the ghost follows asynchronously. Changes that arrive while it
        is being brought up to date are applied together afterwards, so the ghost
        always ends at the latest values.
        """
        self._ghost_outdated = True
        if self._ghost_refresh is None or self._ghost_refresh.done():
            self._ghost_refresh = asyncio.ensure_future(self._refresh_ghost())

    def close(self):
        """Close the window, which removes the ghost. An add already running carries on."""
        self._window.visible = False

    def destroy(self):
        """Remove the ghost and release the window's widgets."""
        self._ghost.remove()
        self._window.destroy()

    def _on_visibility_changed(self, visible: bool):
        """Remove the ghost as soon as the window closes, before an add starts loading."""
        if not visible:
            self._ghost.remove()
            self._ghost_asset = None
        self.update_ghost()

    def _on_manufacturer_selected(self, _manufacturer: str):
        """Offer the manufacturer's models and show the selected one as the ghost."""
        self._model_dropdown.repopulate()
        # Repopulating calls the model callback only when the selected name changed.
        self.update_ghost()

    async def _refresh_ghost(self):
        """Bring the ghost up to date until no change is left unapplied.

        A model that cannot be fetched, previewed or placed removes the ghost and
        leaves the reason in the status line.
        """
        while self._ghost_outdated:
            self._ghost_outdated = False
            try:
                await self._bring_ghost_up_to_date()
            except HTTPException as exc:
                self._show_no_preview(exc.detail)
            except RuntimeError as exc:
                # The core layer's user-facing error, which the services translate to 422.
                self._show_no_preview(exc)

    async def _bring_ghost_up_to_date(self):
        """Show the selected asset as the ghost, then place it where Add would put it.

        No ghost is shown while the window is closed, no model is selected, or the
        Attaching Prim names no prim. A model is fetched and previewed only when it is
        not the one shown already.
        """
        asset = self._selected_asset()
        attaching_prim_path = self._attaching_prim_field.get_value()
        stage = omni.usd.get_context().get_stage()
        if (
            not self._window.visible
            or asset is None
            or not stage.GetPrimAtPath(attaching_prim_path)
        ):
            self._ghost.remove()
            self._ghost_asset = None
            return

        if asset is not self._ghost_asset:
            self._status_label.text = "Loading preview..."
            fetched = await self._asset_library_service.fetch_asset(
                asset["category"], asset["manufacturer"], asset["model"]
            )
            main_file = fetched["local_path"]
            if main_file not in self._preview_asset_paths:
                self._preview_asset_paths[
                    main_file
                ] = await asset_loader.build_preview_asset(main_file)
            if self._ghost_outdated:
                # The window changed while the preview loaded; the next pass shows the
                # latest selection instead of this one.
                return
            self._ghost.show(self._preview_asset_paths[main_file])
            self._ghost_asset = asset
            self._status_label.text = ""
        await self.place_ghost(attaching_prim_path)

    def _show_no_preview(self, reason):
        """Remove the ghost and say why the selected asset has no preview."""
        self._ghost.remove()
        self._ghost_asset = None
        self._status_label.text = f"No preview: {reason}"

    async def _place_ghost_on_robot_mount(
        self, attaching_prim_path: str, offset: models.Transformation
    ):
        """Place the ghost as a tool or mesh attached at ``offset`` to the robot the
        Attaching Prim is part of, or under the Attaching Prim when it is part of none.

        Mirrors the add: the robot's stored mount link is used, and ``offset`` is read
        in its frame.

        Raises:
            HTTPException: 422 if the robot stores no mount link.
            RuntimeError: If the robot has no prim of its mount link's name.
        """
        robot_id = await self._articulation_service.find_robot_for_prim(
            attaching_prim_path
        )
        if robot_id is None:
            self._ghost.place_under(self._world_transform(attaching_prim_path))
            return
        self._ghost.place_on_mount(
            robot_assembler.offset_matrix(offset)
            * self._world_transform(self._arm_mount_path(robot_id))
        )

    def _arm_mount_path(self, robot_id: str) -> str:
        """Return the prim path of the robot's stored mount link.

        Raises:
            HTTPException: 422 if the robot stores no mount link.
            RuntimeError: If the robot has no prim of its mount link's name.
        """
        return robot_assembler.find_prim_by_name(
            omni.usd.get_context().get_stage(),
            self._articulation_service.get_device(robot_id).prim_path,
            self._articulation_service.resolve_arm_mount_link(robot_id),
        )

    @staticmethod
    def _world_transform(prim_path: str):
        """Return the world transform of the prim at ``prim_path`` as a ``Gf.Matrix4d``."""
        return omni.usd.get_world_transform_matrix(
            omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
        )

    async def _load_catalogue(self):
        """Fill the dropdowns from every category; report categories that fail to load.

        A category that fails (e.g. one with no assets yet) does not hide the others.
        """
        self._status_label.text = "Loading asset catalogue..."
        assets = []
        errors = []
        for category in self.asset_categories():
            try:
                entries = await self._asset_library_service.list_assets(category)
            except HTTPException as exc:
                errors.append(str(exc.detail))
                continue
            assets.extend({**entry, "category": category} for entry in entries)
        self._assets = assets
        self._manufacturer_dropdown.set_items(
            sorted({self._manufacturer_label(asset) for asset in self._assets}),
            select_index=0,
        )
        self._model_dropdown.repopulate()
        self._status_label.text = "; ".join(errors)
        self.update_ghost()

    def _models_of_selected_manufacturer(self) -> list[str]:
        """Return the models of the manufacturer selected in the Manufacturer dropdown."""
        manufacturer = self._manufacturer_dropdown.get_selection()
        return sorted(
            asset["model"]
            for asset in self._assets
            if self._manufacturer_label(asset) == manufacturer
        )

    def _selected_asset(self) -> dict | None:
        """Return the catalogue entry selected in the dropdowns, or ``None`` when there is none."""
        manufacturer = self._manufacturer_dropdown.get_selection()
        model = self._model_dropdown.get_selection()
        return next(
            (
                candidate
                for candidate in self._assets
                if self._manufacturer_label(candidate) == manufacturer
                and candidate["model"] == model
            ),
            None,
        )

    def _on_add_clicked(self):
        """Check the selected asset and the Attaching Prim, then close the window and add it.

        A missing selection or an Attaching Prim that is not a prim in the stage keeps
        the window open with the reason in the status line, before anything is
        downloaded or loaded.
        """
        asset = self._selected_asset()
        if asset is None:
            self._status_label.text = "Select a manufacturer and model first."
            return
        attaching_prim_path = self._attaching_prim_field.get_value()
        if not omni.usd.get_context().get_stage().GetPrimAtPath(attaching_prim_path):
            self._status_label.text = (
                f"'{attaching_prim_path}' is not a prim in the stage."
            )
            return

        self._status_label.text = ""
        self._window.visible = False
        asyncio.ensure_future(self._add_asset(asset, attaching_prim_path))

    async def _add_asset(self, asset: dict, attaching_prim_path: str):
        """Pass ``asset`` to :meth:`add_selected_asset` and notify the outcome."""
        asset_label = f"{self._manufacturer_label(asset)} {asset['model']}"
        self._add_button.enabled = False
        notification_manager.post_notification(f"Adding {asset_label}...")
        try:
            success_message = await self.add_selected_asset(asset, attaching_prim_path)
        except HTTPException as exc:
            self._post_add_failure(asset_label, exc.detail)
        except RuntimeError as exc:
            # The core layer's user-facing error, which the services translate to 422.
            self._post_add_failure(asset_label, exc)
        else:
            notification_manager.post_notification(success_message)
        finally:
            self._add_button.enabled = True

    @staticmethod
    def _post_add_failure(asset_label: str, reason):
        """Notify that adding ``asset_label`` failed, as a warning that stays until dismissed."""
        notification_manager.post_notification(
            f"Adding {asset_label} failed: {reason}",
            hide_after_timeout=False,
            status=notification_manager.NotificationStatus.WARNING,
            button_infos=[
                notification_manager.NotificationButtonInfo("OK", on_complete=None)
            ],
        )

    @staticmethod
    def _free_destination_path(
        attaching_prim_path: str, asset_name: str, robot_prim_path: str | None = None
    ) -> str:
        """Return a prim path no prim in the stage uses yet, for a new asset named ``asset_name``.

        Without ``robot_prim_path`` the path lies under ``attaching_prim_path``. With
        it, the asset is to be joined to that robot with a fixed joint, and bodies
        nested under a link would not follow it, so the path lies beside the robot --
        in its ``<model>_assembly`` Xform when it has one -- outside any rigid body the
        robot stands in (see
        :func:`..core.asset_loader.path_beside_clear_of_rigid_bodies`). A name already
        taken gets a numeric suffix, so every add creates a new prim.
        """
        stage = omni.usd.get_context().get_stage()
        prim_name = Tf.MakeValidIdentifier(asset_name)
        if robot_prim_path is None:
            path = f"{attaching_prim_path.rstrip('/')}/{prim_name}"
        else:
            path = asset_loader.path_beside_clear_of_rigid_bodies(
                stage, robot_prim_path, prim_name
            )
        return omni.usd.get_stage_next_free_path(stage, path, False)

    @staticmethod
    def _manufacturer_label(asset: dict) -> str:
        """Return the asset's manufacturer as shown in the Manufacturer dropdown."""
        return asset["manufacturer"] or GENERIC_MANUFACTURER_LABEL
