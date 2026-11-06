# SPDX-License-Identifier: Apache-2.0
"""The shared flow of the Add Robot / Add Tool / Add Mesh windows."""

import abc
import asyncio
from pathlib import Path

import omni.usd
from fastapi import HTTPException
from omni import ui
from omni.kit import notification_manager
from omni.kit.window.property import templates
from pxr import Gf, Tf
from isaacsim.gui.components import element_wrappers

from ..comm import models
from ..comm.services.articulations import ArticulationService
from ..comm.services.asset_library import AssetLibraryService
from ..core import asset_loader, robot_assembler
from .asset_ghost import AssetGhost
from .placement_gizmo import PlacementGizmo
from .transformation_fields import TransformationFields

# Shown in the Manufacturer dropdown for catalogue entries listed without a manufacturer.
GENERIC_MANUFACTURER_LABEL = "Generic"
# First entry of the Manufacturer and Model dropdowns; it matches no catalogue entry, so
# nothing is loaded while it is selected.
NO_SELECTION_LABEL = "None"
# Width, in pixels, of every button in the windows' centered button rows.
BUTTON_WIDTH = 180
# Size, in pixels, every Add window opens at, so the Add windows all look alike.
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 280
# Extra gap, in pixels, between the input fields and the button row.
BUTTON_ROW_GAP = 15
# omni.ui colours are 0xAABBGGRR. The accent is the preview's #41d1ff; the hint's
# background is the same colour, mostly transparent.
ACCENT_COLOR = 0xFFFFD141
HINT_BACKGROUND_COLOR = 0x33FFD141
KEY_CAP_TEXT_COLOR = 0xFF1E1E1E


class AddAssetWindow(abc.ABC):
    """A window that adds one asset from the Telekinesis asset library to the stage.

    The user picks a manufacturer and model from the catalogue categories the window
    covers. Both read None whenever the window opens, and picking a manufacturer sets
    Model back to None, so no model is loaded until one is picked. The prim the asset is added at, the Attaching Prim, is the right-clicked
    prim and cannot be edited. While the window is open, a see-through ghost of the
    selected model shows where Add will put it and follows every change of the model
    and the window's other fields; the ghost is never saved, cannot be selected and
    takes no part in physics. Position and Rotation place the asset in a frame the
    window names, its transformation frame; while that frame exists, a move/rotate
    gizmo stands on the ghost in the viewport, and dragging it enters where it is
    dragged to in Position and Rotation. The gizmo turns about the world axes, offers
    rotation while the viewport's Rotate tool (E) is selected and translation
    otherwise, and edits nothing in the stage. Nothing in the stage can be selected
    while the window is open: a selection is cleared as soon as it is made, so Kit's
    own gizmo never shows beside it. A model
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

    A subclass names its categories, may add buttons, names its transformation frame,
    places the ghost where its add puts the asset, and implements the add itself.

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
        self._gizmo = PlacementGizmo(on_dragged_fn=self._on_gizmo_dragged)
        # The world transformation frame the gizmo was last shown in, which turns a
        # dragged pose back into Position and Rotation.
        self._gizmo_frame = Gf.Matrix4d(1.0)
        # Clears every selection while the window is open; None while it is closed.
        self._selection_subscription = None

        self._window = ui.Window(
            title, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, visible=False
        )
        self._window.set_visibility_changed_fn(self._on_visibility_changed)
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                # Every row is a label of the property panel's label width and a field
                # filling the rest, so the fields line up on both edges.
                self._manufacturer_dropdown = element_wrappers.DropDown(
                    "Manufacturer",
                    on_selection_fn=self._on_manufacturer_selected,
                    add_flourish=False,
                )
                self._model_dropdown = element_wrappers.DropDown(
                    "Model",
                    populate_fn=self._models_of_selected_manufacturer,
                    on_selection_fn=lambda _: self.update_ghost(),
                    add_flourish=False,
                )
                # Built directly because Isaac's StringField always ends its row with a
                # decoration the other rows do not have.
                with ui.HStack(height=0):
                    ui.Label(
                        "Attaching Prim",
                        width=templates.LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Prim the asset is added at",
                    )
                    # Read-only: the right-clicked prim the window was opened on.
                    self._attaching_prim_model = ui.StringField(read_only=True).model
                self._transformation_fields = TransformationFields(
                    on_changed_fn=self.update_ghost
                )
                # A callout in the preview's colour, always shown so the buttons below
                # it never move.
                with ui.ZStack(height=0):
                    ui.Rectangle(
                        style={
                            "background_color": HINT_BACKGROUND_COLOR,
                            "border_color": ACCENT_COLOR,
                            "border_width": 1,
                            "border_radius": 4,
                        }
                    )
                    with ui.VStack(height=0):
                        ui.Spacer(height=6)
                        with ui.HStack(height=0, spacing=6):
                            ui.Spacer()
                            ui.Label("Hover over the viewport and press", width=0)
                            self._build_key_cap("W")
                            ui.Label("for the move or", width=0)
                            self._build_key_cap("E")
                            ui.Label("for the rotate tool", width=0)
                            ui.Spacer()
                        ui.Spacer(height=6)
                ui.Spacer(height=BUTTON_ROW_GAP)
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

    def build_extra_buttons(self):
        """Build buttons shown left of Add, in the same row. Optional; the default
        builds none.

        Called once from the constructor, inside that centered horizontal row; give
        each button a width of :data:`BUTTON_WIDTH`.
        """

    async def transformation_frame(self, attaching_prim_path: str) -> Gf.Matrix4d | None:
        """Return the world frame Position and Rotation are read in, or ``None`` when
        they do not place the asset.

        Called on every ghost refresh, with ``attaching_prim_path`` naming a prim in
        the stage. While it returns a frame, the gizmo is shown on the ghost; ``None``
        hides it. May raise the services' ``HTTPException`` or the core layer's
        ``RuntimeError``; the ghost is then removed and the message shown as the reason
        there is no preview.

        Optional; the default is the stored mount link of the robot the Attaching
        Prim is part of, and ``None`` when it is part of no robot.
        """
        robot_id = await self._articulation_service.find_robot_for_prim(
            attaching_prim_path
        )
        if robot_id is None:
            return None
        return self._world_transform(self._arm_mount_path(robot_id))

    def place_ghost(
        self, attaching_prim_path: str, transformation_frame: Gf.Matrix4d | None
    ):
        """Move the ghost to where :meth:`add_selected_asset` would put the selected asset.

        Called on every ghost refresh, once the ghost shows the selected asset, with
        ``attaching_prim_path`` naming a prim in the stage and the frame
        :meth:`transformation_frame` returned. Must place the ghost through
        ``self._ghost`` (:meth:`AssetGhost.place_under` or
        :meth:`AssetGhost.place_on_mount`).

        Optional; the default stands the asset's mount body at the entered
        transformation in ``transformation_frame``, as a tool or mesh attached to a
        robot, and at the Attaching Prim's origin when the frame is ``None``.
        """
        if transformation_frame is None:
            self._ghost.place_under(self._world_transform(attaching_prim_path))
            return
        self._ghost.place_on_mount(
            robot_assembler.offset_matrix(self._transformation_fields.transformation())
            * transformation_frame
        )

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
        Manufacturer and Model are set to None, and Position and Rotation are reset to
        zero unless an add is running.
        """
        self._attaching_prim_model.set_value(attaching_prim_path)
        # Add is disabled exactly while an add runs, and that add still reads the fields.
        if self._add_button.enabled:
            self._transformation_fields.reset()
        if self._manufacturer_dropdown.get_items():
            # Selecting the manufacturer's None also sets Model to None.
            self._manufacturer_dropdown.set_selection(NO_SELECTION_LABEL)
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
        """Remove the ghost and the gizmo and release the window's widgets."""
        self._ghost.remove()
        self._gizmo.destroy()
        self._selection_subscription = None
        self._window.destroy()

    def _on_visibility_changed(self, visible: bool):
        """Keep the stage unselected while the window is open, and remove the ghost and
        the gizmo as soon as it closes, before an add starts loading."""
        if visible:
            # A selected prim gets Kit's own gizmo, which would stand beside the
            # window's; so nothing stays selected while the window is open.
            self._deselect_all()
            usd_context = omni.usd.get_context()
            self._selection_subscription = (
                usd_context.get_stage_event_stream().create_subscription_to_pop_by_type(
                    int(omni.usd.StageEventType.SELECTION_CHANGED),
                    lambda _: self._deselect_all(),
                    name="telekinesis add window selection lock",
                )
            )
        else:
            self._selection_subscription = None
            self._remove_ghost()
        self.update_ghost()

    @staticmethod
    def _deselect_all():
        """Clear the stage selection, unless nothing is selected."""
        selection = omni.usd.get_context().get_selection()
        if selection.get_selected_prim_paths():
            selection.clear_selected_prim_paths()

    def _remove_ghost(self):
        """Remove the ghost and the gizmo standing on it."""
        self._ghost.remove()
        self._ghost_asset = None
        self._gizmo.hide()

    @staticmethod
    def _build_key_cap(key: str):
        """Build a keyboard key labelled ``key``, as the gizmo hint names it."""
        with ui.ZStack(width=22, height=20):
            ui.Rectangle(
                style={"background_color": ACCENT_COLOR, "border_radius": 3}
            )
            ui.Label(
                key,
                alignment=ui.Alignment.CENTER,
                style={"color": KEY_CAP_TEXT_COLOR, "font_size": 14},
            )

    def _on_gizmo_dragged(self, pose: Gf.Matrix4d):
        """Enter the dragged world ``pose``, read in the transformation frame, in
        Position and Rotation, which moves the ghost after it."""
        translation, rotation = robot_assembler.matrix_to_translation_rotation(
            pose * self._gizmo_frame.GetInverse()
        )
        self._transformation_fields.set_transformation(
            models.Transformation(translation=translation, rotation=rotation)
        )

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
        attaching_prim_path = self._attaching_prim_model.get_value_as_string()
        stage = omni.usd.get_context().get_stage()
        if (
            not self._window.visible
            or asset is None
            or not stage.GetPrimAtPath(attaching_prim_path)
        ):
            self._remove_ghost()
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
        transformation_frame = await self.transformation_frame(attaching_prim_path)
        self.place_ghost(attaching_prim_path, transformation_frame)
        if transformation_frame is None:
            self._gizmo.hide()
            return
        self._gizmo_frame = transformation_frame
        self._gizmo.show(
            robot_assembler.offset_matrix(self._transformation_fields.transformation())
            * transformation_frame
        )

    def _show_no_preview(self, reason):
        """Remove the ghost and say why the selected asset has no preview."""
        self._remove_ghost()
        self._status_label.text = f"No preview: {reason}"

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
            [
                NO_SELECTION_LABEL,
                *sorted({self._manufacturer_label(asset) for asset in self._assets}),
            ],
            select_index=0,
        )
        self._model_dropdown.repopulate()
        self._status_label.text = "; ".join(errors)
        self.update_ghost()

    def _models_of_selected_manufacturer(self) -> list[str]:
        """Return None followed by the models of the manufacturer selected in the
        Manufacturer dropdown."""
        manufacturer = self._manufacturer_dropdown.get_selection()
        return [
            NO_SELECTION_LABEL,
            *sorted(
                asset["model"]
                for asset in self._assets
                if self._manufacturer_label(asset) == manufacturer
            ),
        ]

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
        attaching_prim_path = self._attaching_prim_model.get_value_as_string()
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
