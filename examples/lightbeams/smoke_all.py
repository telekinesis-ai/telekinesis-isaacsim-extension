"""
THROWAWAY end-to-end smoke test for every /lightbeams route (delete when done).

Registers one lightbeam sensor at --prim, then exercises every lightbeam
endpoint the examples cover -- the GETs, a reading, a round-trip PATCH (read the
layout, write the same layout back, so the sensor is not disturbed) and a
pause/resume cycle -- printing PASS / FAIL per route, then deletes the sensor.
Requires Isaac Sim running, the bridge extension on, an IsaacLightBeamSensor
prim in the stage, and the timeline playing (a stopped timeline makes the
reading answer 409).

Run:
  python examples/lightbeams/smoke_all.py --prim /World/LightBeam_Sensor
  python examples/lightbeams/smoke_all.py --prim /World/LightBeam_Sensor --keep
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
# Binding waits out the physics steps the sensor's first reading needs.
DEFAULT_TIMEOUT = 60.0

# The configuration fields the round-trip PATCH echoes back.
_CONFIGURATION_FIELDS = (
    "num_rays",
    "curtain_length",
    "forward_axis",
    "curtain_axis",
    "min_range",
    "max_range",
)

_results = []  # (label, "PASS" | "FAIL" | "SKIP", detail)


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
    _results.append((label, verdict, f"{payload}"))
    print(f"[{verdict}] {status:>4}  {label}  ->  {str(payload)[:120]}")
    return payload if ok else None


def _skip(label, reason):
    """Record a skipped route (a prerequisite is missing)."""
    _results.append((label, "SKIP", reason))
    print(f"[SKIP]   -   {label}  ->  {reason}")


def _exercise_routes(base, args):
    """Register a lightbeam sensor and exercise every exposed lightbeam route."""
    created = _record(
        "PUT /lightbeams (create)",
        *_call(base, "PUT", "/lightbeams", {"prim_path": args.prim}),
    )
    if not created or "lightbeam_id" not in created:
        print(
            "\nCreate failed -- cannot continue. Check the prim path holds an "
            "IsaacLightBeamSensor prim and that the timeline can play."
        )
        return
    root = f"/lightbeams/{created['lightbeam_id']}"

    _record("GET /lightbeams (list)", *_call(base, "GET", "/lightbeams"))
    _record(f"GET {root}", *_call(base, "GET", root))
    _record(f"GET {root}/reading", *_call(base, "GET", f"{root}/reading"))

    # Write the layout the sensor already has straight back, so the round trip
    # is covered without changing how the scene is set up.
    _record(
        f"PATCH {root}/configuration",
        *_call(
            base,
            "PATCH",
            f"{root}/configuration",
            {field: created[field] for field in _CONFIGURATION_FIELDS},
        ),
    )

    _record(f"POST {root}/pause", *_call(base, "POST", f"{root}/pause"))
    _record(f"GET {root}/is_paused (paused)", *_call(base, "GET", f"{root}/is_paused"))
    _record(f"POST {root}/resume", *_call(base, "POST", f"{root}/resume"))
    _record(f"GET {root}/is_paused (resumed)", *_call(base, "GET", f"{root}/is_paused"))

    if args.keep:
        _skip(f"DELETE {root}", "--keep set")
    else:
        _record(f"DELETE {root}", *_call(base, "DELETE", root))


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


def main():
    """Smoke-test the lightbeam routes exposed by the bridge."""
    parser = argparse.ArgumentParser(description="Smoke-test every /lightbeams route.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--prim", default="/World/LightBeam_Sensor", help="lightbeam sensor prim path to register"
    )
    parser.add_argument("--keep", action="store_true", help="don't delete the sensor at the end")
    args = parser.parse_args()
    base = f"http://{args.host}:{args.port}"
    _exercise_routes(base, args)
    _summary()


if __name__ == "__main__":
    main()
