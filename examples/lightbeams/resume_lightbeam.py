"""
POST /lightbeams/{id}/resume -> {paused: false}

Resumes sensor computation after pause_lightbeam.py.

Requires the id to already be registered -- run put_lightbeam.py first.

Run:  python resume_lightbeam.py --id lightbeam1
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
    """Resume one lightbeam sensor's computation."""
    parser = argparse.ArgumentParser(description="Resume a lightbeam sensor.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--id", required=True, dest="lightbeam_id", help="lightbeam_id from put_lightbeam.py"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    print(f"response: {_request(base, 'POST', f'/lightbeams/{args.lightbeam_id}/resume')}")


if __name__ == "__main__":
    main()
