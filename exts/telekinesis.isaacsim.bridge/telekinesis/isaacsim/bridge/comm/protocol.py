# SPDX-License-Identifier: Apache-2.0
"""
Wire protocol for the Synapse <-> Isaac Sim bridge.

Transport: TCP, newline-delimited JSON, one request -> one response. Units on the
wire are native Isaac: radians for joints, meters for lengths. This module is the
shared mental model with the Synapse client -- keep it tiny and explicit.

    Request:  {"type": <str>, ...params}
    Response: {"ok": true, "result": {...}}  or  {"ok": false, "error": "<msg>"}
"""

import json

# Message-type strings. The Synapse client and the task-2 handlers route on these.
# CONNECT/DISCONNECT are handled by the single connection server; the rest are
# handled by the per-device articulation servers it spawns.
CONNECT = "connect"
DISCONNECT = "disconnect"
HANDSHAKE = "handshake"
MOVE_J = "move_j"
GET_STATE = "get_state"
GRIPPER_OPEN = "gripper_open"
GRIPPER_CLOSE = "gripper_close"
GRIPPER_MOVE = "gripper_move"
GRIPPER_STATE = "gripper_state"


def encode_request(message_type, **params):
    """Build a request line: {"type": message_type, **params} terminated by '\\n'."""
    return (json.dumps({"type": message_type, **params}) + "\n").encode()


def encode_response(result):
    """Build a success line: {"ok": true, "result": result}."""
    return (json.dumps({"ok": True, "result": result}) + "\n").encode()


def encode_error(message):
    """Build a failure line: {"ok": false, "error": message}."""
    return (json.dumps({"ok": False, "error": message}) + "\n").encode()


def decode_line(line):
    """Parse one newline-delimited JSON line (bytes) into a dict."""
    return json.loads(line.decode())
