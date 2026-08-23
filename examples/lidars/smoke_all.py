"""
THROWAWAY end-to-end smoke test for every /lidars route (delete when done).

Creates one lidar at --prim, then exercises every lidar endpoint the examples
cover -- GETs, capture, and round-trip SETs (read a value, write the same
value back, so the lidar isn't disturbed) -- printing PASS / FAIL / SKIP per
route, then deletes the lidar. Requires Isaac Sim running, the bridge
extension on, a stage open, and the timeline playing.

Run:
  python examples/lidars/smoke_all.py --prim /World/Lidar
  python examples/lidars/smoke_all.py --prim /World/Lidar --keep
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 60.0

_results = []  # (label, "PASS" | "FAIL" | "SKIP", detail)


def _shape(value):
    """Nested-list dimensions, so we log an array's shape instead of its values."""
    dims = []
    while isinstance(value, list):
        dims.append(len(value))
        value = value[0] if value else None
    return dims


def _brief(payload):
    """One-line summary of a response body (shapes for big arrays)."""
    if isinstance(payload, dict):
        return "{" + ", ".join(f"{k}: {_brief(v)}" for k, v in payload.items()) + "}"
    if isinstance(payload, list):
        return f"array{_shape(payload)}"
    return repr(payload)


def _call(base, method, path, body=None):
    """Return (ok, status, payload). payload is JSON on success, else the detail."""
    try:
        resp = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    except requests.exceptions.RequestException as exc:
        return False, "ERR", str(exc)
    payload = resp.json() if resp.content else None
    if resp.ok:
        return True, resp.status_code, payload
    detail = payload.get("detail") if isinstance(payload, dict) else payload
    return False, resp.status_code, detail


def _record(label, ok, status, payload):
    """Log one endpoint result and remember it for the summary."""
    verdict = "PASS" if ok else "FAIL"
    detail = _brief(payload) if ok else f"{payload}"
    _results.append((label, verdict, detail))
    print(f"[{verdict}] {status:>4}  {label}  ->  {detail[:120]}")
    return payload if ok else None


def _skip(label, reason):
    """Log one skipped route.

    Smoke tests use this when a route cannot be exercised because a
    prerequisite is missing or the returned value is empty.
    """
    _results.append((label, "SKIP", reason))
    print(f"[SKIP]   -   {label}  ->  {reason}")


def _get(base, label, path):
    ok, status, payload = _call(base, "GET", path)
    return _record(label, ok, status, payload)


def _post(base, label, path, body=None):
    ok, status, payload = _call(base, "POST", path, body)
    return _record(label, ok, status, payload)


def _roundtrip(base, label, base_path, key):
    """GET a single-value endpoint, then PUT the same value back."""
    got = _get(base, f"GET {label}", base_path)
    if not got or got.get(key) is None:
        _skip(f"PUT {label}", "no value to echo back")
        return
    ok, status, payload = _call(base, "PUT", base_path, {"value": got[key]})
    _record(f"PUT {label}", ok, status, payload)


def _exercise_registry_routes(base, root):
    """Exercise the lidar registry endpoints."""
    _get(base, "GET /lidars (list)", "/lidars")
    _get(base, f"GET {root}", root)


def _exercise_capture_routes(base, root):
    """Exercise capture and scan-data routes."""
    _post(
        base,
        f"POST {root}/capture",
        f"{root}/capture",
        {"data_types": ["point_cloud", "depth"]},
    )
    _get(base, f"GET {root}/depth", f"{root}/depth")
    _get(base, f"GET {root}/linear_depth", f"{root}/linear_depth")
    _get(base, f"GET {root}/intensity", f"{root}/intensity")
    _get(base, f"GET {root}/zenith", f"{root}/zenith")
    _get(base, f"GET {root}/azimuth", f"{root}/azimuth")
    _get(base, f"GET {root}/point_cloud", f"{root}/point_cloud")
    _get(base, f"GET {root}/semantic", f"{root}/semantic")


def _exercise_pose_routes(base, root):
    """Exercise pose GET/PUT routes with a round-trip where possible."""
    wp = _get(base, f"GET {root}/world_pose", f"{root}/world_pose")
    if wp and wp.get("position") is not None:
        _record(
            f"PUT {root}/world_pose",
            *_call(
                base,
                "PUT",
                f"{root}/world_pose",
                {"position": wp["position"], "orientation": wp["orientation"]},
            ),
        )
    else:
        _skip(f"PUT {root}/world_pose", "no pose to echo back")

    lp = _get(base, f"GET {root}/local_pose", f"{root}/local_pose")
    if lp and lp.get("translation") is not None:
        _record(
            f"PUT {root}/local_pose",
            *_call(
                base,
                "PUT",
                f"{root}/local_pose",
                {"translation": lp["translation"], "orientation": lp["orientation"]},
            ),
        )
    else:
        _skip(f"PUT {root}/local_pose", "no pose to echo back")


def _exercise_config_routes(base, root):
    """Exercise scan-configuration GET/PUT routes via round-trips."""
    _roundtrip(base, f"{root}/min_range", f"{root}/min_range", "min_range")
    _roundtrip(base, f"{root}/max_range", f"{root}/max_range", "max_range")
    _roundtrip(base, f"{root}/horizontal_fov", f"{root}/horizontal_fov", "horizontal_fov")
    _roundtrip(base, f"{root}/vertical_fov", f"{root}/vertical_fov", "vertical_fov")
    _roundtrip(
        base,
        f"{root}/horizontal_resolution",
        f"{root}/horizontal_resolution",
        "horizontal_resolution",
    )
    _roundtrip(
        base, f"{root}/vertical_resolution", f"{root}/vertical_resolution", "vertical_resolution"
    )
    _roundtrip(base, f"{root}/rotation_rate", f"{root}/rotation_rate", "rotation_rate")
    _roundtrip(base, f"{root}/yaw_offset", f"{root}/yaw_offset", "yaw_offset")
    _roundtrip(base, f"{root}/high_lod", f"{root}/high_lod", "high_lod")
    _roundtrip(base, f"{root}/draw_points", f"{root}/draw_points", "draw_points")
    _roundtrip(base, f"{root}/draw_lines", f"{root}/draw_lines", "draw_lines")
    _roundtrip(base, f"{root}/enable_semantics", f"{root}/enable_semantics", "enable_semantics")


def _exercise_introspection_routes(base, root):
    """Exercise introspection endpoints."""
    _get(base, f"GET {root}/num_rows", f"{root}/num_rows")
    _get(base, f"GET {root}/num_cols", f"{root}/num_cols")
    _get(base, f"GET {root}/num_cols_ticked", f"{root}/num_cols_ticked")
    _get(base, f"GET {root}/azimuth_range", f"{root}/azimuth_range")
    _get(base, f"GET {root}/zenith_range", f"{root}/zenith_range")
    _get(base, f"GET {root}/is_lidar_sensor", f"{root}/is_lidar_sensor")


def _exercise_collection_routes(base, root):
    """Exercise pause/resume collection-control routes."""
    _post(base, f"POST {root}/pause", f"{root}/pause")
    _get(base, f"GET {root}/is_paused", f"{root}/is_paused")
    _post(base, f"POST {root}/resume", f"{root}/resume")
    _get(base, f"GET {root}/is_paused (after resume)", f"{root}/is_paused")


def _exercise_routes(base, args):
    """Create a lidar and exercise the exposed lidar routes."""
    ok, status, payload = _call(
        base, "PUT", "/lidars", {"prim_path": args.prim, "data_types": ["point_cloud", "depth"]}
    )
    created = _record("PUT /lidars (create)", ok, status, payload)
    if not created or "lidar_id" not in created:
        print(
            "\nCreate failed -- cannot continue. Check the prim path and that the sim is playing."
        )
        _summary()
        return
    lid = created["lidar_id"]
    root = f"/lidars/{lid}"

    _exercise_registry_routes(base, root)
    _exercise_capture_routes(base, root)
    _exercise_pose_routes(base, root)
    _exercise_config_routes(base, root)
    _exercise_introspection_routes(base, root)
    _exercise_collection_routes(base, root)

    if args.keep:
        _skip(f"DELETE {root}", "--keep set")
    else:
        _record(f"DELETE {root}", *_call(base, "DELETE", root))


def main():
    """Smoke-test the lidar routes exposed by the bridge."""
    parser = argparse.ArgumentParser(description="Smoke-test every /lidars route.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="lidar prim path to register")
    parser.add_argument("--keep", action="store_true", help="don't delete the lidar at the end")
    args = parser.parse_args()
    base = f"http://{args.host}:{args.port}"
    _exercise_routes(base, args)
    _summary()


def _summary():
    counts = {"PASS": 0, "FAIL": 0, "SKIP": 0}
    for _, verdict, _detail in _results:
        counts[verdict] += 1
    print("\n" + "=" * 60)
    print(f"SUMMARY: {counts['PASS']} passed, {counts['FAIL']} failed, {counts['SKIP']} skipped")
    if counts["FAIL"]:
        print("\nFailures:")
        for label, verdict, detail in _results:
            if verdict == "FAIL":
                print(f"  {label}: {detail}")


if __name__ == "__main__":
    main()
