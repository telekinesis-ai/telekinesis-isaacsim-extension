"""
PATCH /surface_grippers/{id}/properties
{coaxial_force_limit?, shear_force_limit?, max_grip_distance?, retry_interval?,
 forward_axis?, rotation_limits?, translation_limits?} ->
{coaxial_force_limit, shear_force_limit, max_grip_distance, retry_interval,
 forward_axis}

Retunes the gripper's grip behaviour. Omit a flag to leave that property alone.
The running gripper picks the new values up on its next step -- no re-create and no
Stop+Play -- and they can equally be set while the simulation is stopped.

Units: newtons for the force limits (the loads that break a grip along and across
the forward axis), meters for the reach, seconds for how long a close keeps
retrying before it gives up on the cups that found nothing.

``rotation_limits`` (degrees) and ``translation_limits`` (meters) are how far a
gripped object may swivel and slide once held. USD stores those on each of the
gripper's attachment points rather than on the gripper, so setting them here writes
the same limits to all of them -- which is why they are not in the response; read
them back with get_attachment_points.py. Use
set_attachment_point_properties.py to give individual cups different limits.

Run:  python set_properties.py --id surface_gripper1 --max-grip-distance 0.02
      python set_properties.py --id surface_gripper1 --coaxial-force-limit 5e5
          --shear-force-limit 5e5 --retry-interval 0.5
      python set_properties.py --id surface_gripper1 --rotation-limits -3 3
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(method, base.rstrip("/") + path, json=body, timeout=DEFAULT_TIMEOUT)
    response.raise_for_status()
    return response.json() if response.content else None


def _axis_limits(bounds):
    """Same [minimum, maximum] on all three axes, or None to leave them untouched."""
    if bounds is None:
        return None
    minimum, maximum = bounds
    limit = {"minimum": minimum, "maximum": maximum}
    return {"x": limit, "y": limit, "z": limit}


def main():
    """Retune one surface gripper's properties and print the resulting values."""
    parser = argparse.ArgumentParser(description="Set a surface gripper's properties.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id",
        required=True,
        dest="surface_gripper_id",
        help="surface_gripper_id from put_surface_gripper.py",
    )
    parser.add_argument("--coaxial-force-limit", type=float, default=None, help="newtons")
    parser.add_argument("--shear-force-limit", type=float, default=None, help="newtons")
    parser.add_argument("--max-grip-distance", type=float, default=None, help="meters")
    parser.add_argument("--retry-interval", type=float, default=None, help="seconds")
    parser.add_argument("--forward-axis", default=None, choices=["X", "Y", "Z"])
    parser.add_argument(
        "--rotation-limits",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=None,
        help="degrees, applied to every attachment point on all three axes",
    )
    parser.add_argument(
        "--translation-limits",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=None,
        help="meters, applied to every attachment point on all three axes",
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PATCH",
        f"/surface_grippers/{args.surface_gripper_id}/properties",
        {
            "coaxial_force_limit": args.coaxial_force_limit,
            "shear_force_limit": args.shear_force_limit,
            "max_grip_distance": args.max_grip_distance,
            "retry_interval": args.retry_interval,
            "forward_axis": args.forward_axis,
            "rotation_limits": _axis_limits(args.rotation_limits),
            "translation_limits": _axis_limits(args.translation_limits),
        },
    )
    for key, value in response.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
