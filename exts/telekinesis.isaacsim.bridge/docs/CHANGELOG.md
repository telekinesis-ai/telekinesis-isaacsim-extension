# Changelog

## [1.1.0] - 2026-06-23
### Changed
- Migrate the bridge transport from a raw asyncio TCP / newline-JSON server to FastAPI
  served by uvicorn on Isaac Sim's existing asyncio loop (still single-threaded, still
  direct articulation access -- no thread marshalling).
- Per-device servers now expose typed REST routes (`POST /move_j`, `GET /state`,
  `POST /gripper/move|open|close`, `GET /gripper/state`) with pydantic models instead of
  a single `{"type": ...}` envelope. Success is HTTP 2xx; failures are 4xx with `{"error"}`.
- Per-device-port topology preserved: `POST /connect` on 8766 still returns a dedicated port.
### Added
- `fastapi`, `uvicorn`, `pydantic` declared via `[python.pipapi]`.

## [1.0.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings


## [0.1.0] - 2026-06-22

### Added

- Initial version of TelekinesisIsaacSimBridge Extension
