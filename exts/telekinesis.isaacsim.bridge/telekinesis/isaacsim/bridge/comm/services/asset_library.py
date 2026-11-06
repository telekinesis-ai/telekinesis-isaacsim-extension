# SPDX-License-Identifier: Apache-2.0
"""Asset library service: the orchestration behind the ``/asset_library`` routes.

``AssetLibraryService`` lists the Telekinesis asset bucket and downloads asset
bundles into a local cache, so both HTTP clients and the extension's GUI load the
same assets the same way. omni imports are lazy so this module imports outside
Isaac Sim.
"""

import asyncio
import io
import json
import pathlib
import shutil
import zipfile

from fastapi import HTTPException

from ...global_variables import ASSET_LIBRARY_URL

# While ASSET_LIBRARY_URL is None, the catalogue is read from this file in the extension
# folder, and bundles are downloaded from the host that serves them today.
LOCAL_ASSET_INDEX_FILE = "data/assets_index.json"
LEGACY_BUNDLE_URL = "https://assets.telekinesis.ai"

# The bucket keeps one tree per description format, each holding the same categories.
# USD comes first: a model found in both trees is taken from its USD bundle.
FORMAT_PREFIXES = ("usd", "urdf")

# The main file of a bundle: a URDF in the "urdf" tree, else a USD or mesh file.
URDF_SUFFIXES = {".urdf"}
USD_SUFFIXES = {".usd", ".usda", ".usdc", ".usdz"}
USD_OR_MESH_SUFFIXES = USD_SUFFIXES | {".obj", ".glb", ".gltf"}

# Matches Isaac Sim's own omni.client listing timeout (isaacsim.storage.native).
LISTING_TIMEOUT_SECONDS = 10


class AssetLibraryService:
    """The Telekinesis asset catalogue and its local download cache.

    The catalogue is the layout of the asset bucket, which keeps one tree per
    description format, ``usd/`` and ``urdf/``. A category such as
    ``"robots/manipulators"`` is a folder path below a format tree and is listed from
    both trees; each folder below it is a manufacturer, and each ``.zip`` in a
    manufacturer folder is one model's bundle. A model found in both trees is taken
    from its USD bundle. An entry is ``{"manufacturer", "model", "bundle"}``, where
    ``model`` is the ZIP name without extension and ``bundle`` the bucket-relative key
    of the ZIP, format tree included. Each category is listed once per session;
    bundles are downloaded once and kept in the cache across sessions.

    The bucket must allow public listing. Kit's storage client also caches
    listings, so assets uploaded during a session may only appear after a restart.

    Example:
        service = AssetLibraryService()
        robot = await service.fetch_asset("robots/manipulators", "epson", "cx4a601s")
        robot["local_path"]  # .../telekinesis_assets/urdf/robots/manipulators/epson/cx4a601s/....urdf
    """

    def __init__(self):
        import carb.tokens

        self._cache_directory = pathlib.Path(
            carb.tokens.get_tokens_interface().resolve("${data}/telekinesis_assets")
        )
        self._entries_by_category: dict[str, list[dict]] = {}
        # bundle key -> asyncio.Lock, so concurrent fetches of one bundle download it once
        # and never extract into the same folder at the same time.
        self._fetch_locks: dict[str, asyncio.Lock] = {}

    async def list_assets(self, category: str) -> list[dict]:
        """Return the catalogue entries of ``category``, sorted by manufacturer and model.

        Raises:
            HTTPException: 404 if the category holds no assets; 502 if the bucket
                cannot be listed.
        """
        if category not in self._entries_by_category:
            entries_by_identity = {}
            for format_prefix in FORMAT_PREFIXES:
                tree_category = f"{format_prefix}/{category}"
                if ASSET_LIBRARY_URL is None:
                    tree_entries = self._read_local_asset_index(tree_category)
                else:
                    tree_entries = await self._list_bucket_category(tree_category)
                for entry in tree_entries:
                    entries_by_identity.setdefault(
                        (entry["manufacturer"], entry["model"]), entry
                    )
            entries = list(entries_by_identity.values())
            if not entries:
                raise HTTPException(
                    status_code=404,
                    detail=f"asset category '{category}' holds no assets",
                )
            self._entries_by_category[category] = sorted(
                entries, key=lambda entry: (entry["manufacturer"] or "", entry["model"])
            )
        return self._entries_by_category[category]

    async def fetch_asset(
        self, category: str, manufacturer: str | None, model: str
    ) -> dict:
        """Make the asset available locally and return its catalogue entry.

        The bundle is downloaded and extracted on first use only. The returned entry
        gains ``local_path``: the absolute path of the bundle's main file in the cache
        -- its URDF (or USD) for a bundle in the ``urdf/`` tree, otherwise its USD,
        OBJ, GLB or glTF file, the shallowest one when there are several. ``manufacturer`` is ``None`` for an
        entry listed without one.

        Raises:
            HTTPException: 404 if the catalogue has no such category or asset; 502 if
                the download or extraction fails, or the bundle holds no main file.
        """
        import omni.client

        entries = await self.list_assets(category)
        entry = next(
            (
                candidate
                for candidate in entries
                if candidate["manufacturer"] == manufacturer
                and candidate["model"] == model
            ),
            None,
        )
        if entry is None:
            raise HTTPException(
                status_code=404,
                detail=f"no '{manufacturer} {model}' in asset category '{category}'",
            )

        bundle_directory = self._cache_directory / pathlib.PurePosixPath(
            entry["bundle"]
        ).with_suffix("")
        async with self._fetch_locks.setdefault(entry["bundle"], asyncio.Lock()):
            if not bundle_directory.is_dir():
                url = f"{ASSET_LIBRARY_URL or LEGACY_BUNDLE_URL}/{entry['bundle']}"
                result, _, content = await omni.client.read_file_async(url)
                if result != omni.client.Result.OK:
                    raise HTTPException(
                        status_code=502,
                        detail=f"failed to download asset bundle {url}: {result.name}",
                    )
                await asyncio.to_thread(
                    self._extract_bundle,
                    memoryview(content).tobytes(),
                    bundle_directory,
                )

        main_file_suffixes = (
            URDF_SUFFIXES | USD_SUFFIXES
            if entry["bundle"].startswith("urdf/")
            else USD_OR_MESH_SUFFIXES
        )
        main_files = [
            path
            for path in bundle_directory.rglob("*")
            if path.suffix.lower() in main_file_suffixes
        ]
        if not main_files:
            raise HTTPException(
                status_code=502,
                detail=f"asset bundle '{entry['bundle']}' contains no {sorted(main_file_suffixes)} file",
            )
        # The main file sits above the layers, meshes and textures it references.
        main_file = min(main_files, key=lambda path: len(path.parts))
        return {**entry, "local_path": str(main_file)}

    async def _list_bucket_category(self, category: str) -> list[dict]:
        """List ``<ASSET_LIBRARY_URL>/<category>/<manufacturer>/<model>.zip`` into catalogue entries.

        Raises:
            HTTPException: 502 if a listing fails or times out. A missing category
                lists as empty, which ``list_assets`` reports as 404.
        """
        import omni.client

        manufacturer_folders = await self._list_bucket_folder(f"{category}/")
        entries = []
        for manufacturer_folder in manufacturer_folders:
            if not manufacturer_folder.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN:
                continue
            manufacturer = manufacturer_folder.relative_path.rstrip("/")
            for bundle_file in await self._list_bucket_folder(
                f"{category}/{manufacturer}/"
            ):
                if bundle_file.relative_path.endswith(".zip"):
                    entries.append(
                        {
                            "manufacturer": manufacturer,
                            "model": bundle_file.relative_path.removesuffix(".zip"),
                            "bundle": f"{category}/{manufacturer}/{bundle_file.relative_path}",
                        }
                    )
        return entries

    async def _list_bucket_folder(self, folder: str) -> list:
        """Return the ``omni.client.ListEntry`` items directly inside ``folder`` of the bucket.

        Raises:
            HTTPException: 502 if the listing fails or times out.
        """
        import omni.client

        url = f"{ASSET_LIBRARY_URL}/{folder}"
        try:
            result, entries = await asyncio.wait_for(
                omni.client.list_async(url), timeout=LISTING_TIMEOUT_SECONDS
            )
        except asyncio.TimeoutError as exc:
            raise HTTPException(
                status_code=502, detail=f"listing {url} timed out"
            ) from exc
        if result == omni.client.Result.ERROR_NOT_FOUND:
            return []
        if result != omni.client.Result.OK:
            raise HTTPException(
                status_code=502, detail=f"failed to list {url}: {result.name}"
            )
        return list(entries)

    @staticmethod
    def _read_local_asset_index(category: str) -> list[dict]:
        """Return ``category``'s entries from the extension's local ``assets_index.json``.

        The file maps a category to its entries in the same shape the bucket listing
        produces: ``{"<category>": [{"manufacturer", "model", "bundle"}, ...]}``, where
        ``manufacturer`` may be ``null``. A robot entry may also carry
        ``simulation_mount_link``, which is passed through unchanged.
        """
        import omni.kit.app

        extension_path = (
            omni.kit.app.get_app()
            .get_extension_manager()
            .get_extension_path_by_module("telekinesis.isaacsim.bridge")
        )
        index = json.loads(
            pathlib.Path(extension_path, LOCAL_ASSET_INDEX_FILE).read_text()
        )
        return index.get(category, [])

    @staticmethod
    def _extract_bundle(archive_bytes: bytes, bundle_directory: pathlib.Path):
        """Extract a bundle ZIP into ``bundle_directory``. Runs off the main thread.

        Extraction goes to a temporary sibling directory that is renamed into place,
        so an interrupted extraction never leaves a half-extracted bundle that looks
        complete on the next fetch. A temporary directory left by such an extraction is
        cleared first, so none of its files end up in the bundle.

        Raises:
            HTTPException: 502 if the archive is not a valid ZIP.
        """
        partial_directory = bundle_directory.with_name(
            bundle_directory.name + ".partial"
        )
        shutil.rmtree(partial_directory, ignore_errors=True)
        partial_directory.mkdir(parents=True)
        try:
            with zipfile.ZipFile(io.BytesIO(archive_bytes)) as archive:
                archive.extractall(partial_directory)
        except zipfile.BadZipFile as exc:
            raise HTTPException(
                status_code=502,
                detail=f"asset bundle for '{bundle_directory.name}' is not a ZIP: {exc}",
            ) from exc
        partial_directory.rename(bundle_directory)
