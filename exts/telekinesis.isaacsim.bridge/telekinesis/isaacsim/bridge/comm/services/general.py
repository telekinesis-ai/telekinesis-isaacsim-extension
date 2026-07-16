# SPDX-License-Identifier: Apache-2.0
"""General service: the orchestration behind the ``/status`` and ``/version`` routes.

``GeneralService`` answers service-health and environment queries -- a liveness
check and the list of enabled Kit extensions with their versions. Stateless, so
one shared instance serves all requests. omni imports are lazy so this module
imports outside Isaac Sim.
"""


class GeneralService:
    """Service health and environment introspection."""

    def status(self):
        """Service health. Returns ``OK`` while the bridge is running."""
        return {"status": "OK"}

    def versions(self):
        """Best-effort map of enabled Kit extensions -> version.

        Runs inside Isaac Sim, so ``omni.kit.app`` is importable (imported lazily so
        this module still loads outside Isaac). Defensive: the
        manager's summary dict shape varies across Kit releases, so a missing
        ``version`` is recovered from the id (``"<name>-<version>"``) rather than
        raising.
        """
        import omni.kit.app

        manager = omni.kit.app.get_app().get_extension_manager()
        versions = {}
        for ext in manager.get_extensions():
            if not ext.get("enabled"):
                continue
            name = ext.get("name", "")
            version = ext.get("version")
            if version is None:
                ext_id = ext.get("id", "")
                version = ext_id[len(name) + 1 :] or None  # strip the "<name>-" prefix
            versions[name] = version
        return versions
