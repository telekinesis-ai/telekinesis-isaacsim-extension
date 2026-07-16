"""
Standalone bridge example: enable or disable a physics joint.

Flow (all over HTTP):
  1. PATCH /prims/physics/joints {prim_path, enable: false} -> disables the joint
  2. PATCH /prims/physics/joints {prim_path, enable: true}  -> re-enables it

Run:  python set_prim_joint_state.py --prim /World/ur10e/joint_a1

Requires the ``requests`` package (``pip install requests``).
"""

import argparse

import requests

HOST = "127.0.0.1"
PORT = 8766
DEFAULT_TIMEOUT = 30.0


def _request(base, method, path, params=None, body=None):
    """Send one request and return the decoded JSON (None for an empty body)."""
    response = requests.request(
        method,
        base.rstrip("/") + path,
        params=params,
        json=body,
        timeout=DEFAULT_TIMEOUT,
    )
    response.raise_for_status()
    return response.json() if response.content else None


def main():
    """Disable a physics joint, then re-enable it."""
    parser = argparse.ArgumentParser(description="Enable or disable a physics joint.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--prim", required=True, help="prim path of the joint")
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    _request(base, "PATCH", "/prims/physics/joints", body={"prim_path": args.prim, "enable": False})
    print(f"joint disabled: {args.prim}")

    _request(base, "PATCH", "/prims/physics/joints", body={"prim_path": args.prim, "enable": True})
    print(f"joint enabled: {args.prim}")


if __name__ == "__main__":
    main()
