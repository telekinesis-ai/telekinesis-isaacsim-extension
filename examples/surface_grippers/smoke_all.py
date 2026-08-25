"""
THROWAWAY end-to-end smoke test for every /surface_grippers route (delete when done).

Registers one suction gripper at --prim, then exercises every surface gripper
endpoint the examples cover -- the GETs, a close/open cycle, and round-trip PATCHes
(read a value, write the same value back, so the gripper isn't disturbed) --
printing PASS / FAIL / SKIP per route, then deletes the gripper. Requires Isaac Sim
running, the bridge extension on, the gripper asset in the stage, and the timeline
playing (a stopped timeline makes close/open answer 409).

Run:
  python examples/surface_grippers/smoke_all.py
      --prim /World/defitech_modelled_surface_gripper
  python examples/surface_grippers/smoke_all.py --prim /World/gripper --keep
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
# A blocking close/open waits on the simulation, whose backstop is ~30 s.
DEFAULT_TIMEOUT = 60.0

_results = []  # (label, "PASS" | "FAIL" | "SKIP", detail)


def _brief(payload):
    """One-line summary of a response body (lengths for long lists)."""
    if isinstance(payload, dict):
        return "{" + ", ".join(f"{k}: {_brief(v)}" for k, v in payload.items()) + "}"
    if isinstance(payload, list):
        return f"list[{len(payload)}]" if len(payload) > 3 else repr(payload)
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
    """Record a skipped route (a prerequisite is missing, or there is nothing to echo)."""
    _results.append((label, "SKIP", reason))
    print(f"[SKIP]   -   {label}  ->  {reason}")


def _get(base, label, path):
    ok, status, payload = _call(base, "GET", path)
    return _record(label, ok, status, payload)


def _post(base, label, path, body=None):
    ok, status, payload = _call(base, "POST", path, body)
    return _record(label, ok, status, payload)


def _exercise_registry_routes(base, root):
    """Exercise the surface gripper registry endpoints."""
    _get(base, "GET /surface_grippers (list)", "/surface_grippers")
    _get(base, f"GET {root}", root)
    _get(base, f"GET {root}/status", f"{root}/status")


def _exercise_actuation_routes(base, root):
    """Close then open the gripper, blocking on each so the status is the settled one."""
    _post(base, f"POST {root}/close", f"{root}/close", {"asynchronous": False})
    _get(base, f"GET {root}/status (after close)", f"{root}/status")
    _post(base, f"POST {root}/open", f"{root}/open", {"asynchronous": False})
    _get(base, f"GET {root}/status (after open)", f"{root}/status")
    _post(base, f"POST {root}/open (async)", f"{root}/open", {"asynchronous": True})


def _exercise_property_routes(base, root):
    """Read the gripper's properties, then write the same values straight back."""
    properties = _get(base, f"GET {root}/properties", f"{root}/properties")
    if not properties:
        _skip(f"PATCH {root}/properties", "no properties to echo back")
        return
    _record(f"PATCH {root}/properties", *_call(base, "PATCH", f"{root}/properties", properties))


def _exercise_attachment_point_routes(base, root):
    """Read one attachment point's tunables, then write the same values back to it."""
    points = _get(base, f"GET {root}/attachment_points", f"{root}/attachment_points")
    if not points or not points.get("attachment_points"):
        _skip(f"PATCH {root}/attachment_points", "gripper reports no attachment points")
        return
    first = points["attachment_points"][0]
    _record(
        f"PATCH {root}/attachment_points",
        *_call(
            base,
            "PATCH",
            f"{root}/attachment_points",
            {
                "joint_paths": [first["prim_path"]],
                "z_axis_translation_drive_stiffness": first["z_axis_translation_drive_stiffness"],
                "z_axis_translation_drive_damping": first["z_axis_translation_drive_damping"],
                "rotation_limits": first["rotation_limits"] or None,
                "translation_limits": first["translation_limits"] or None,
                "clearance_offset": first["clearance_offset"],
                "forward_axis": first["forward_axis"],
            },
        ),
    )


def _exercise_routes(base, args):
    """Register a surface gripper and exercise every exposed surface gripper route."""
    ok, status, payload = _call(base, "PUT", "/surface_grippers", {"prim_path": args.prim})
    created = _record("PUT /surface_grippers (create)", ok, status, payload)
    if not created or "surface_gripper_id" not in created:
        print(
            "\nCreate failed -- cannot continue. Check the prim path holds an "
            "IsaacSurfaceGripper prim with at least one attachment point."
        )
        return
    root = f"/surface_grippers/{created['surface_gripper_id']}"

    _exercise_registry_routes(base, root)
    _exercise_property_routes(base, root)
    _exercise_attachment_point_routes(base, root)
    _exercise_actuation_routes(base, root)

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
    """Smoke-test the surface gripper routes exposed by the bridge."""
    parser = argparse.ArgumentParser(description="Smoke-test every /surface_grippers route.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--prim",
        default="/World/defitech_modelled_surface_gripper",
        help="suction gripper prim path to register",
    )
    parser.add_argument("--keep", action="store_true", help="don't delete the gripper at the end")
    args = parser.parse_args()
    base = f"http://{args.host}:{args.port}"
    _exercise_routes(base, args)
    _summary()


if __name__ == "__main__":
    main()
