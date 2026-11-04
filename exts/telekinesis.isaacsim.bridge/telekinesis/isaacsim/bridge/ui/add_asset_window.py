# SPDX-License-Identifier: Apache-2.0
"""The shared flow of the Add Robot / Add Tool / Add Mesh windows."""

import abc
import asyncio

from fastapi import HTTPException
from omni import ui
from isaacsim.gui.components import element_wrappers

from ..comm.services.asset_library import AssetLibraryService

# Shown in the Manufacturer dropdown for catalogue entries listed without a manufacturer.
GENERIC_MANUFACTURER_LABEL = "Generic"


class AddAssetWindow(abc.ABC):
    """A window that adds one asset from the Telekinesis asset library to the stage.

    The user picks a manufacturer and model from the catalogue categories the window
    covers, and the prim the asset is added at (pre-filled with the right-clicked
    prim). Pressing Add downloads the asset into the local cache once, then hands it
    to :meth:`add_fetched_asset`, which loads it through the bridge services so the
    result is the same as an HTTP client doing it. The status line shows the outcome
    or the service's error; the Add button is disabled while an add runs.

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
                    "", "Add", on_click_fn=lambda: asyncio.ensure_future(self._add_selected_asset())
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

    @abc.abstractmethod
    async def add_fetched_asset(self, asset: dict, attaching_prim_path: str) -> str:
        """Load the selected, downloaded asset into the stage at ``attaching_prim_path``.

        Called when Add is pressed, after the asset is in the local cache. ``asset`` is
        its catalogue entry (``manufacturer``, ``model``, ``bundle``, ``category`` and
        any extra index fields) plus ``local_path``, the bundle's main file. Must go
        through the bridge services, and may raise their ``HTTPException``, whose
        detail is shown to the user.

        Returns the status message shown on success.
        """

    def show(self, attaching_prim_path: str):
        """Open the window with ``attaching_prim_path`` pre-filled as the Attaching Prim.

        The catalogue is loaded on the first open, and retried on later opens until
        it yields any asset.
        """
        self._attaching_prim_field.set_value(attaching_prim_path)
        self._window.visible = True
        if not self._assets:
            asyncio.ensure_future(self._load_catalogue())

    def destroy(self):
        """Release the window's widgets."""
        self._window.destroy()

    async def _load_catalogue(self):
        """Fill the dropdowns from every category; report categories that fail to load.

        A category that fails (e.g. one with no assets yet) does not hide the others.
        """
        self._status_label.text = "Loading asset catalogue..."
        errors = []
        for category in self.asset_categories():
            try:
                entries = await self._asset_library_service.list_assets(category)
            except HTTPException as exc:
                errors.append(str(exc.detail))
                continue
            self._assets.extend({**entry, "category": category} for entry in entries)
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

    async def _add_selected_asset(self):
        """Download the selected asset and pass it to :meth:`add_fetched_asset`."""
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

        self._add_button.enabled = False
        self._status_label.text = f"Adding {manufacturer} {model}..."
        try:
            fetched = await self._asset_library_service.fetch_asset(
                asset["category"], asset["manufacturer"], asset["model"]
            )
            self._status_label.text = await self.add_fetched_asset(
                {**fetched, "category": asset["category"]}, self._attaching_prim_field.get_value()
            )
        except HTTPException as exc:
            self._status_label.text = str(exc.detail)
        finally:
            self._add_button.enabled = True

    @staticmethod
    def _manufacturer_label(asset: dict) -> str:
        """Return the asset's manufacturer as shown in the Manufacturer dropdown."""
        return asset["manufacturer"] or GENERIC_MANUFACTURER_LABEL
