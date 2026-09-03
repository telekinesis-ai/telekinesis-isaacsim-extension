"""
THROWAWAY end-to-end smoke test for every /conveyors route (delete when done).

Registers one conveyor at --prim, then exercises every conveyor endpoint the
examples cover -- the GETs, a start/stop cycle at the belt's own speed and then
reversed -- printing PASS / FAIL per route, then deletes the conveyor. Requires
Isaac Sim running, the bridge extension on, and the belt provisioned as a
conveyor in the stage. Registering it plays the timeline, so nothing has to be
started by hand first.

Run:
  python examples/conveyors/smoke_all.py --prim /World/ConveyorBelt_A08
  python examples/conveyors/smoke_all.py --prim /World/ConveyorBelt_A08 --keep
"""

import argparse
import time

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0
# Long enough to see the belt actually carry something between the two reads.
RUN_SECONDS = 2.0

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
    """Register a conveyor and exercise every exposed conveyor route."""
    created = _record(
        "PUT /conveyors (create)",
        *_call(base, "PUT", "/conveyors", {"prim_path": args.prim, "cargo_root": args.cargo_root}),
    )
    if not created or "conveyor_id" not in created:
        print(
            "\nCreate failed -- cannot continue. Check the prim carries a non-zero "
            "PhysxSurfaceVelocityAPI velocity, or is driven by an IsaacConveyor node."
        )
        return
    root = f"/conveyors/{created['conveyor_id']}"

    _record("GET /conveyors (list)", *_call(base, "GET", "/conveyors"))
    _record(f"GET {root}", *_call(base, "GET", root))

    # Run at the speed the scene authored, then reversed, so both the default
    # and the signed-velocity path are covered.
    _record(f"POST {root}/start (authored speed)", *_call(base, "POST", f"{root}/start", {}))
    time.sleep(RUN_SECONDS)
    _record(f"GET {root} (running)", *_call(base, "GET", root))

    reversed_speed = -abs(created["nominal_speed"]) or -0.5
    _record(
        f"POST {root}/start (reversed)",
        *_call(base, "POST", f"{root}/start", {"velocity": reversed_speed}),
    )
    time.sleep(RUN_SECONDS)

    _record(f"POST {root}/stop", *_call(base, "POST", f"{root}/stop"))
    _record(f"GET {root} (stopped)", *_call(base, "GET", root))

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
    """Smoke-test the conveyor routes exposed by the bridge."""
    parser = argparse.ArgumentParser(description="Smoke-test every /conveyors route.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--prim", default="/World/ConveyorBelt_A08", help="conveyor prim path to register"
    )
    parser.add_argument(
        "--cargo-root",
        default="/World",
        help="prim whose sleeping rigid bodies are woken when the belt starts",
    )
    parser.add_argument("--keep", action="store_true", help="don't delete the conveyor at the end")
    args = parser.parse_args()
    base = f"http://{args.host}:{args.port}"
    _exercise_routes(base, args)
    _summary()


if __name__ == "__main__":
    main()
