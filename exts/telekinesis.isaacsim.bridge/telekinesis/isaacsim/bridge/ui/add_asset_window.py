# SPDX-License-Identifier: Apache-2.0
"""The shared flow of the Add Robot / Add Tool / Add Mesh windows."""

import abc
import asyncio

import omni.usd
from fastapi import HTTPException
from omni import ui
from omni.kit import notification_manager
from pxr import Tf
from isaacsim.gui.components import element_wrappers

from ..comm.services.asset_library import AssetLibraryService
from ..core import asset_loader

# Shown in the Manufacturer dropdown for catalogue entries listed without a manufacturer.
GENERIC_MANUFACTURER_LABEL = "Generic"


class AddAssetWindow(abc.ABC):
    """A window that adds one asset from the Telekinesis asset library to the stage.

    The user picks a manufacturer and model from the catalogue categories the window
    covers, and the prim the asset is added at (pre-filled with the right-clicked
    prim). Pressing Add checks that a model is selected and that the Attaching Prim
    exists; if not, the window stays open and its status line says why. Otherwise the
    window closes, the asset is downloaded into the local cache once and handed to
    :meth:`add_fetched_asset`, which loads it through the bridge services so the
    result is the same as an HTTP client doing it. Progress and the outcome are shown
    as notifications, an error as a warning that stays until dismissed. Add is
    disabled while an add runs. Every add creates a new prim: a name already taken in
    the stage gets a numeric suffix.

    A subclass names its categories, may add input fields, and implements the add
    itself.

    Example:
        window = AddRobotWindow(bridge_server.asset_library_service,
                                bridge_server.articulation_service)
        window.show("/World")
    """

    def __init__(self, title: str, asset_library_service: AssetLibraryService):
        self._asset_library_service = asset_library_service
        # Entries of every category this window covers, each tagged with its "category".
        self._assets: list[dict] = []
        # The catalogue load running or last run; None until the window is first shown.
        self._catalogue_load: asyncio.Future | None = None

        self._window = ui.Window(title, width=420, height=0, visible=False)
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._manufacturer_dropdown = element_wrappers.DropDown(
                    "Manufacturer",
                    on_selection_fn=lambda _: self._model_dropdown.repopulate(),
                )
                self._model_dropdown = element_wrappers.DropDown(
                    "Model", populate_fn=self._models_of_selected_manufacturer
                )
                self._attaching_prim_field = element_wrappers.StringField(
                    "Attaching Prim", tooltip="Prim the asset is added at"
                )
                self.build_extra_fields()
                self._add_button = element_wrappers.Button(
                    "", "Add", on_click_fn=self._on_add_clicked
                )
                self._status_label = ui.Label("", word_wrap=True)

    @abc.abstractmethod
    def asset_categories(self) -> list[str]:
        """Return the catalogue categories this window offers, e.g. ``["urdf/robots/manipulators"]``.

        Called when the catalogue is first loaded. The entries of all returned
        categories share the Manufacturer and Model dropdowns.
        """

    def build_extra_fields(self):
        """Build input fields shown below Attaching Prim. Optional; the default builds none.

        Called once from the constructor, inside the window's vertical layout, so
        widgets created here land in it. Store anything :meth:`add_fetched_asset`
        reads on ``self``.
        """

    def reset_extra_fields(self):
        """Set the fields built by :meth:`build_extra_fields` back to their defaults. Optional.

        Called each time the window is opened while no add is running, so every add
        starts from the defaults instead of the values of the previous one. The default
        resets nothing.
        """

    @abc.abstractmethod
    async def add_fetched_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Load the selected, downloaded asset into the stage at ``attaching_prim_path``.

        Called when Add is pressed, after the asset is in the local cache and with
        ``attaching_prim_path`` naming a prim in the stage. ``asset`` is its catalogue
        entry (``manufacturer``, ``model``, ``bundle``, ``category`` and any extra index
        fields) plus ``local_path``, the bundle's main file. Must go through the bridge
        services and place new prims at :meth:`_free_destination_path`. May raise the
        services' ``HTTPException`` or the core layer's ``RuntimeError``, whose message
        is shown to the user.

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
        if not self._assets and (self._catalogue_load is None or self._catalogue_load.done()):
            self._catalogue_load = asyncio.ensure_future(self._load_catalogue())

    def destroy(self):
        """Release the window's widgets."""
        self._window.destroy()

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
            sorted({self._manufacturer_label(asset) for asset in self._assets}), select_index=0
        )
        self._model_dropdown.repopulate()
        self._status_label.text = "; ".join(errors)

    def _models_of_selected_manufacturer(self) -> list[str]:
        """Return the models of the manufacturer selected in the Manufacturer dropdown."""
        manufacturer = self._manufacturer_dropdown.get_selection()
        return sorted(
            asset["model"] for asset in self._assets if self._manufacturer_label(asset) == manufacturer
        )

    def _on_add_clicked(self):
        """Check the selected asset and the Attaching Prim, then close the window and add it.

        A missing selection or an Attaching Prim that is not a prim in the stage keeps
        the window open with the reason in the status line, before anything is
        downloaded or loaded.
        """
        manufacturer = self._manufacturer_dropdown.get_selection()
        model = self._model_dropdown.get_selection()
        asset = next(
            (
                candidate
                for candidate in self._assets
                if self._manufacturer_label(candidate) == manufacturer and candidate["model"] == model
            ),
            None,
        )
        if asset is None:
            self._status_label.text = "Select a manufacturer and model first."
            return
        attaching_prim_path = self._attaching_prim_field.get_value()
        if not omni.usd.get_context().get_stage().GetPrimAtPath(attaching_prim_path):
            self._status_label.text = f"'{attaching_prim_path}' is not a prim in the stage."
            return

        self._status_label.text = ""
        self._window.visible = False
        asyncio.ensure_future(self._add_asset(asset, attaching_prim_path))

    async def _add_asset(self, asset: dict, attaching_prim_path: str):
        """Download ``asset``, pass it to :meth:`add_fetched_asset`, and notify the outcome."""
        asset_label = f"{self._manufacturer_label(asset)} {asset['model']}"
        self._add_button.enabled = False
        notification_manager.post_notification(f"Adding {asset_label}...")
        try:
            fetched = await self._asset_library_service.fetch_asset(
                asset["category"], asset["manufacturer"], asset["model"]
            )
            success_message = await self.add_fetched_asset(
                {**fetched, "category": asset["category"]}, attaching_prim_path
            )
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
            button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
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
            path = asset_loader.path_beside_clear_of_rigid_bodies(stage, robot_prim_path, prim_name)
        return omni.usd.get_stage_next_free_path(stage, path, False)

    @staticmethod
    def _manufacturer_label(asset: dict) -> str:
        """Return the asset's manufacturer as shown in the Manufacturer dropdown."""
        return asset["manufacturer"] or GENERIC_MANUFACTURER_LABEL
