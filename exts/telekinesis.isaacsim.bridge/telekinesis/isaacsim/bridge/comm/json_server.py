# SPDX-License-Identifier: Apache-2.0
"""
A generic newline-delimited-JSON TCP server that lives inside Isaac Sim's event
loop. Payload-agnostic: each request line is decoded and passed to an injected
async `handler(request) -> result_dict`, whose result is sent back as a structured
response. Used both for the single connection server and for each per-device
articulation server.

`port=0` lets the OS pick a free port; the actual bound port is available via the
`port` property after `start()`. Because the server runs on Isaac Sim's own
asyncio loop (the main thread), handlers may touch articulation / timeline APIs
directly -- no thread marshalling, no command queue.
"""

import asyncio

from .protocol import decode_line, encode_error, encode_response


class JsonLineServer:
    """TCP/JSON-line server; routes each request line to an async handler."""

    def __init__(self, handler, host="127.0.0.1", port=0, name=""):
        self._handler = handler
        self._host = host
        self._port = port  # 0 -> OS assigns; replaced with the real port on start()
        self._name = name
        self._server = None

    async def start(self):
        """Bind and start serving; returns the actual (possibly OS-assigned) port."""
        self._server = await asyncio.start_server(self._handle, self._host, self._port)
        self._port = self._server.sockets[0].getsockname()[1]
        print(f"[bridge] {self._name} server listening on {self._host}:{self._port}")
        return self._port

    @property
    def port(self):
        return self._port

    async def _handle(self, reader, writer):
        peer = writer.get_extra_info("peername")
        try:
            # One connection may carry many requests; read until the client closes.
            while True:
                line = await reader.readline()
                if not line:
                    break

                try:
                    request = decode_line(line)
                    result = await self._handler(request)
                    writer.write(encode_response(result))
                except Exception as exc:  # a bad request must never kill the server
                    print(f"[bridge] {self._name} error handling {peer}: {exc}")
                    writer.write(encode_error(str(exc)))
                await writer.drain()
        except Exception as exc:  # connection-level failure -- drop just this client
            print(f"[bridge] {self._name} connection error {peer}: {exc}")
        finally:
            writer.close()

    def stop(self):
        """Close the listener."""
        if self._server is not None:
            self._server.close()
            self._server = None
        print(f"[bridge] {self._name} server stopped.")
