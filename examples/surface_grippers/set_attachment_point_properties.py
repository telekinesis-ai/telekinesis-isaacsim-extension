"""
PATCH /surface_grippers/{id}/attachment_points
{joint_paths?, local_pose_0?, local_pose_1?, z_axis_translation_drive_stiffness?,
 z_axis_translation_drive_damping?, rotation_limits?, translation_limits?,
 clearance_offset?, forward_axis?} -> {attachment_points: [...]}

Retunes individual suction cups. ``--joint`` selects which attachment points to
write (repeat it, or omit it for all of them); omit a value flag to leave that
property alone.

The Z axis translation drive's stiffness and damping govern how firmly a gripped
object is pulled in along the forward axis. ``--rotation-limits`` (degrees) and
``--translation-limits`` (meters) are how far it may then swivel and slide.
``--clearance-offset`` (meters) is how far ahead of the cup the search for a
surface begins.

``clearance_offset`` and ``forward_axis`` are read when the gripper starts, so a
change to either applies from the next Stop+Play rather than immediately. The rest
take effect on the gripper's next step.

This route also writes the joint's local frames, which is normally left to
assembly: attaching the gripper to an arm re-parks every attachment point and
recomputes ``local_pose_1`` to match. They are not exposed as flags here.

Run:  python set_attachment_point_properties.py --id surface_gripper1
          --drive-stiffness 5000 --drive-damping 100
      python set_attachment_point_properties.py --id surface_gripper1
          --joint /World/gripper/suction_joints/D6Joint --translation-limits 0 0.01
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
    """Retune a surface gripper's attachment points and print their new state."""
    parser = argparse.ArgumentParser(
        description="Set properties on a surface gripper's attachment points."
    )
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id",
        required=True,
        dest="surface_gripper_id",
        help="surface_gripper_id from put_surface_gripper.py",
    )
    parser.add_argument(
        "--joint",
        action="append",
        dest="joint_paths",
        default=None,
        help="attachment point to write (repeatable; omit for every attachment point)",
    )
    parser.add_argument("--drive-stiffness", type=float, default=None)
    parser.add_argument("--drive-damping", type=float, default=None)
    parser.add_argument(
        "--rotation-limits",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=None,
        help="degrees, applied to all three axes",
    )
    parser.add_argument(
        "--translation-limits",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=None,
        help="meters, applied to all three axes",
    )
    parser.add_argument("--clearance-offset", type=float, default=None, help="meters")
    parser.add_argument("--forward-axis", default=None, choices=["X", "Y", "Z"])
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(
        base,
        "PATCH",
        f"/surface_grippers/{args.surface_gripper_id}/attachment_points",
        {
            "joint_paths": args.joint_paths,
            "z_axis_translation_drive_stiffness": args.drive_stiffness,
            "z_axis_translation_drive_damping": args.drive_damping,
            "rotation_limits": _axis_limits(args.rotation_limits),
            "translation_limits": _axis_limits(args.translation_limits),
            "clearance_offset": args.clearance_offset,
            "forward_axis": args.forward_axis,
        },
    )
    for point in response["attachment_points"]:
        print(f"\n{point['prim_path']}")
        for key, value in point.items():
            if key != "prim_path":
                print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
