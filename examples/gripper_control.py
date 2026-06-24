"""
Gripper-only example for the FastAPI bridge: create -> close -> open.

Replaces the old ``robot_gripper.py`` socket flow. The bridge no longer speaks
the per-device TCP protocol -- it's a single HTTP/JSON server on 127.0.0.1:8765
(see telekinesis_isaacsim_bridge).

Flow (all over HTTP):
  1. PUT  /articulations {prim_path, device_type:"gripper", urdf_path?} -> {articulation_id, ...}
  2. GET  /articulations/{id}/gripper/state -> {fraction}
  3. POST /articulations/{id}/gripper/close (fraction 1.0)
     -> blocks until the finger reaches the target or stalls, returns {done, reached, fraction}
  4. POST /articulations/{id}/gripper/open  (fraction 0.0) -> blocks, returns final state

The bridge runs each move to completion server-side, so the POST blocks until the
finger reaches the target or stalls against an object -- no client-side polling.

Run:  python gripper.py
      python gripper.py --prim /World/rg6
      python gripper.py --prim /World/robotiq --urdf C:/path/to/robotiq.urdf

Requires the ``requests`` package (``pip install requests``).
"""

import argparse

import requests

try:
    import telekinesis_urdfs
except ImportError as e:
    raise ImportError(
        "telekinesis-urdfs is not installed. "
        "Install it from: https://github.com/telekinesis-ai/telekinesis-urdfs"
    ) from e

# Path to the grippers's URDF to import. Left blank on purpose -- set it to the URDF
# you want to load (an absolute path or a path the bridge process can read).
try:
    robot_description = telekinesis_urdfs.load("OnRobotRG6")
    if not robot_description.urdf_path.is_file():
        raise ValueError(f"No urdf found, i.e. {robot_description.urdf_path} is not a file")

except Exception as e:
    raise RuntimeError(
        f"Failed to load robot description for '{__class__.__name__}'. "
        "Ensure telekinesis-urdfs is installed: "
        "https://github.com/telekinesis-ai/telekinesis-urdfs"
    ) from e

URDF_PATH = str(robot_description.urdf_path )

HOST = "127.0.0.1"
PORT = 8765
GRIPPER_PRIM_PATH = "/World/rg6"

# A move blocks server-side until it finishes, so allow well over the bridge's
# own motion cap (~30 s) before the HTTP client gives up.
DEFAULT_TIMEOUT = 120.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def run_gripper(base, prim_path, urdf_path):
    info = _request(
        base,
        "PUT",
        "/articulations",
        {"prim_path": prim_path, "device_type": "gripper", "urdf_path": urdf_path},
    )
    articulation_id = info["articulation_id"]
    print(f"created gripper: articulation_id={articulation_id} prim_path={info['prim_path']}")
    print(f"  fraction (start): {info['state']['fraction']:.3f}")

    for label in ("close", "open"):
        print(f"gripper {label}")
        status = _request(base, "POST", f"/articulations/{articulation_id}/gripper/{label}")
        print(f"  done (reached={status['reached']} fraction={status['fraction']:.3f})")


def main():
    parser = argparse.ArgumentParser(description="Gripper-only smoke test for the Isaac Sim bridge.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", default=GRIPPER_PRIM_PATH, help="prim path of the gripper in the stage")
    parser.add_argument("--urdf", default=URDF_PATH, help="optional URDF to import if the prim isn't in the stage")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"bridge: {base}  articulations: {_request(base, 'GET', '/articulations')}")
    run_gripper(base, args.prim, args.urdf)


if __name__ == "__main__":
    main()
