"""
PUT /surface_grippers {prim_path} ->
{surface_gripper_id, prim_path, gripper_prim_path, attachment_point_paths,
 properties, status, gripped_objects, grip_distance, simulated}

Registers one suction gripper with the bridge and hands back the id every other
surface gripper route takes -- including the ``gripper_id`` of
POST /articulations/{arm}/assemble_robot.

Unlike PUT /articulations there is no ``urdf_path``: a suction gripper has no URDF
representation (neither the ``IsaacSurfaceGripper`` prim nor its D6 attachment
points have a URDF equivalent), so it has to be a prepared USD asset already in the
stage. Import or reference the asset first, then register it here.

``prim_path`` is the gripper prim itself or any ancestor of it -- normally the
asset's root, which is also the prim assemble_robot attaches to the arm. The bridge
searches that subtree for the ``IsaacSurfaceGripper`` prim and reports it as
``gripper_prim_path``, along with the attachment points it grips with. Registering
the same prim path again returns the same id and re-binds.

Run:  python put_surface_gripper.py
      python put_surface_gripper.py --prim /World/defitech_modelled_surface_gripper
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0

# Default prim path of a suction gripper asset in the stage.
GRIPPER_PRIM_PATH = "/World/defitech_modelled_surface_gripper"


def _request(base, method, path, body=None):
    """Send one request; exit with a clear message on failure instead of a
    traceback."""
    try:
        response = requests.request(
            method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT
        )
        response.raise_for_status()
    except requests.exceptions.HTTPError as exc:
        # Bridge rejected the request.
        # Response JSON body always has a "detail" message.
        raise SystemExit(
            f"{method} {path} failed ({response.status_code}): {response.json()['detail']}"
        ) from exc
    except requests.exceptions.RequestException as exc:
        # No response at all -- most likely the bridge isn't running.
        raise SystemExit(
            f"Could not reach {base} -- "
            f"is Isaac Sim running with the bridge extension "
            f"loaded? ({exc})"
        ) from exc

    return response.json() if response.content else None


def main():
    """Register a suction gripper and print what the bridge learned about it."""
    parser = argparse.ArgumentParser(description="Register a suction gripper.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--prim",
        default=GRIPPER_PRIM_PATH,
        help="prim path of the gripper asset (its root, or the IsaacSurfaceGripper prim)",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(base, "PUT", "/surface_grippers", {"prim_path": args.prim})

    for key, value in response.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
