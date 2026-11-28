"""Keep the bridge's port open when Windows closes it under the server.

On Windows, Isaac Sim's event loop accepts connections through asyncio's proactor.
When a client resets a connection the loop is in the middle of accepting (a client
process that was killed, say), the accept fails with ``OSError: [WinError 64] The
specified network name is no longer available``, and asyncio answers by logging
"Accept failed on a socket" and closing the listening socket. uvicorn is not told:
its serve loop keeps running with nothing listening, so every later request is
refused until the extension is reloaded.

:func:`keep_listening` watches for that and opens the port again on the same
uvicorn server, so the app, its services and the connections still open carry on.

Nothing here imports Isaac Sim, so it can be exercised outside it.
"""

import asyncio


def closed_listeners(server):
    """The asyncio servers of a started uvicorn ``server`` that no longer listen.

    A server whose socket asyncio closed after a failed accept still lists that
    socket, with a file descriptor of -1; one closed through its own ``close()``
    lists none.
    """
    closed = []
    for listener in getattr(server, "servers", None) or []:
        sockets = listener.sockets or ()
        if not sockets or any(sock.fileno() == -1 for sock in sockets):
            closed.append(listener)
    return closed


async def listen_again(server, host, port):
    """Open ``host:port`` again for a started uvicorn ``server`` whose listeners closed.

    Builds connections the way ``uvicorn.Server.startup`` does, on the same server
    state, so requests reach the same app. Returns the new asyncio server. Raises
    ``OSError`` when the port cannot be bound.
    """
    config = server.config

    def create_protocol(_loop=None):
        return config.http_protocol_class(
            config=config,
            server_state=server.server_state,
            app_state=server.lifespan.state,
            _loop=_loop,
        )

    for listener in closed_listeners(server):
        listener.close()
        server.servers.remove(listener)
    listener = await asyncio.get_running_loop().create_server(
        create_protocol, host=host, port=port, ssl=config.ssl, backlog=config.backlog
    )
    server.servers.append(listener)
    return listener


async def keep_listening(get_server, host, port, log=print, interval=1.0):
    """Reopen ``host:port`` whenever the uvicorn server ``get_server()`` returns has stopped listening.

    Checks every ``interval`` seconds for as long as ``get_server()`` returns a
    server, and returns once it returns ``None``. A server that has not started yet,
    or is shutting down, is left alone. A port that cannot be reopened is logged
    through ``log`` and tried again on the next check.
    """
    while True:
        await asyncio.sleep(interval)
        server = get_server()
        if server is None:
            return
        if not getattr(server, "started", False) or server.should_exit:
            continue
        if not closed_listeners(server):
            continue
        try:
            await listen_again(server, host, port)
        except OSError as error:
            log(f"[bridge] the server stopped listening on {host}:{port} and could not listen again: {error}")
        else:
            log(
                f"[bridge] the server had stopped listening on {host}:{port} (a failed accept closes "
                "the socket on Windows); listening again."
            )
