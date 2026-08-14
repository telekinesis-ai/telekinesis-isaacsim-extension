"""
GET /surface_grippers -> {surface_gripper_id: prim_path}

Every suction gripper currently registered with the bridge. Empty until a
client PUTs one, and emptied again whenever a new stage is opened.

Run:  python get_surface_grippers_list.py
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


def main():
    """Print every registered surface gripper id and its prim path."""
    parser = argparse.ArgumentParser(description="List registered surface grippers.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)

    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    response = _request(base, "GET", "/surface_grippers")
    print(f"registered surface grippers: {response}")


if __name__ == "__main__":
    main()
