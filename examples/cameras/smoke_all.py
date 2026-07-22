"""
THROWAWAY end-to-end smoke test for every /cameras route (delete when done).

Creates one camera at --prim, then exercises every camera endpoint the examples
cover -- GETs, capture, and round-trip SETs (read a value, write the same value
back, so the camera isn't disturbed) -- printing PASS / FAIL / SKIP per route,
then deletes the camera. Requires Isaac Sim running (with --enable_cameras), the
bridge extension on, a stage open, and the timeline playing.

Run:
  python examples/cameras/smoke_all.py
      --prim /World/rsd455/RSD455/Camera_OmniVision_OV9782_Color
  python examples/cameras/smoke_all.py --prim /World/Camera --keep
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 60.0

_results = []  # (label, "PASS" | "FAIL" | "SKIP", detail)


def _shape(value):
    """Nested-list dimensions, so we log an image's shape instead of its pixels."""
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
    """Record a skipped route.

    Smoke tests use this when a route cannot be exercised because a prerequisite is
    missing or the returned value is empty.
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
    """Exercise the camera registry endpoints."""
    _get(base, "GET /cameras (list)", "/cameras")
    _get(base, f"GET {root}", root)


def _exercise_capture_routes(base, root):
    """Exercise capture and image-data routes."""
    _post(base, f"POST {root}/capture", f"{root}/capture", {"data_types": ["rgb", "depth"]})
    _get(base, f"GET {root}/rgb", f"{root}/rgb")
    _get(base, f"GET {root}/rgba", f"{root}/rgba")
    _get(base, f"GET {root}/depth", f"{root}/depth")
    _get(base, f"GET {root}/pointcloud", f"{root}/pointcloud")


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
                {
                    "position": wp["position"],
                    "orientation": wp["orientation"],
                    "camera_axes": "world",
                },
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
                {
                    "translation": lp["translation"],
                    "orientation": lp["orientation"],
                    "camera_axes": "world",
                },
            ),
        )
    else:
        _skip(f"PUT {root}/local_pose", "no pose to echo back")


def _exercise_property_routes(base, root):
    """Exercise property GET/PUT routes and scalar optics endpoints."""
    res = _get(base, f"GET {root}/resolution", f"{root}/resolution")
    if res and res.get("resolution"):
        w, h = res["resolution"]
        _record(
            f"PUT {root}/resolution",
            *_call(base, "PUT", f"{root}/resolution", {"width": w, "height": h}),
        )

    _roundtrip(base, f"{root}/focal_length", f"{root}/focal_length", "focal_length")
    _roundtrip(base, f"{root}/focus_distance", f"{root}/focus_distance", "focus_distance")
    _roundtrip(base, f"{root}/lens_aperture", f"{root}/lens_aperture", "lens_aperture")
    _roundtrip(
        base, f"{root}/horizontal_aperture", f"{root}/horizontal_aperture", "horizontal_aperture"
    )
    _roundtrip(base, f"{root}/vertical_aperture", f"{root}/vertical_aperture", "vertical_aperture")
    _roundtrip(base, f"{root}/frequency", f"{root}/frequency", "frequency")

    clip = _get(base, f"GET {root}/clipping_range", f"{root}/clipping_range")
    if clip and clip.get("clipping_range"):
        near, far = clip["clipping_range"]
        _record(
            f"PUT {root}/clipping_range",
            *_call(
                base, "PUT", f"{root}/clipping_range", {"near_distance": near, "far_distance": far}
            ),
        )

    _roundtrip(base, f"{root}/projection_mode", f"{root}/projection_mode", "projection_mode")
    _roundtrip(base, f"{root}/stereo_role", f"{root}/stereo_role", "stereo_role")
    _roundtrip(
        base,
        f"{root}/lens_distortion_model",
        f"{root}/lens_distortion_model",
        "lens_distortion_model",
    )


def _exercise_introspection_routes(base, root):
    """Exercise introspection endpoints."""
    _get(base, f"GET {root}/intrinsics_matrix", f"{root}/intrinsics_matrix")
    _get(base, f"GET {root}/fov", f"{root}/fov")
    _get(base, f"GET {root}/render_product_path", f"{root}/render_product_path")
    _get(base, f"GET {root}/supported_annotators", f"{root}/supported_annotators")


def _exercise_collection_routes(base, root):
    """Exercise pause/resume collection-control routes."""
    _post(base, f"POST {root}/pause", f"{root}/pause")
    _get(base, f"GET {root}/is_paused", f"{root}/is_paused")
    _post(base, f"POST {root}/resume", f"{root}/resume")
    _get(base, f"GET {root}/is_paused (after resume)", f"{root}/is_paused")


def _exercise_routes(base, args):
    """Create a camera and exercise the exposed camera routes."""
    ok, status, payload = _call(
        base, "PUT", "/cameras", {"prim_path": args.prim, "data_types": ["rgb", "depth"]}
    )
    created = _record("PUT /cameras (create)", ok, status, payload)
    if not created or "camera_id" not in created:
        print(
            "\nCreate failed -- cannot continue. Check the prim path and that the sim is playing."
        )
        _summary()
        return
    cid = created["camera_id"]
    root = f"/cameras/{cid}"

    _exercise_registry_routes(base, root)
    _exercise_capture_routes(base, root)
    _exercise_pose_routes(base, root)
    _exercise_property_routes(base, root)
    _exercise_introspection_routes(base, root)
    _exercise_collection_routes(base, root)

    if args.keep:
        _skip(f"DELETE {root}", "--keep set")
    else:
        _record(f"DELETE {root}", *_call(base, "DELETE", root))


def main():
    """Smoke-test the camera routes exposed by the bridge."""
    parser = argparse.ArgumentParser(description="Smoke-test every /cameras route.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="camera prim path to register")
    parser.add_argument(
        "--keep", action="store_true", help="don't delete the camera at the end"
    )
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
