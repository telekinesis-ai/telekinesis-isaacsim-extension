# SPDX-License-Identifier: Apache-2.0
"""Client of the Telekinesis Isaac Sim Server, which loads robots for kinematics."""

import asyncio
import json
import urllib.error
import urllib.request

from ..global_variables import ISAACSIM_SERVER_URL

# Seconds to wait for an answer. Generous because the first robot of a class makes the server
# download that class's URDF before it answers.
REQUEST_TIMEOUT_SECONDS = 60


class IsaacSimServerError(Exception):
    """Raised when the Telekinesis Isaac Sim Server cannot be reached or answers with an error.

    The message is meant for the user. ``status_code`` is the server's HTTP status, or ``None``
    when the server was not reached.
    """

    def __init__(self, message: str, status_code: int | None = None) -> None:
        self.status_code = status_code
        super().__init__(message)


class IsaacSimServerClient:
    """Client of the Telekinesis Isaac Sim Server's HTTP API.

    Every method sends one request and returns the server's answer; nothing runs in the
    background. Requests are sent with the standard library on a worker thread, so they never
    block Isaac Sim's event loop.

    Example:
        client = IsaacSimServerClient()
        robot = await client.create_robot("UniversalRobotsUR10E")
        robot["id"]  # "6f1c2b0e-8d3a-4c5e-9f7b-2a1d0c9e8b7a"
    """

    def __init__(self, base_url: str = ISAACSIM_SERVER_URL):
        self._base_url = base_url

    async def list_robot_class_names(self) -> list[str]:
        """Return the Synapse robot classes a robot can be loaded from.

        Raises:
            IsaacSimServerError: If the server is not reachable or answers with an error.
        """
        return await self._request("GET", "/robot_class_names")

    async def create_robot(self, class_name: str) -> dict:
        """Load a new robot of ``class_name`` on the server and return its description.

        The description holds the ``id`` the server assigned, as in the server's
        ``POST /robots`` answer.

        Raises:
            IsaacSimServerError: If the server is not reachable or answers with an error, e.g.
                422 for an unknown class name.
        """
        return await self._request("POST", "/robots", {"class_name": class_name})

    async def describe_robot(self, robot_id: str) -> dict:
        """Return the description of the server's robot ``robot_id``: its ``base_frame``,
        ``default_tcp`` and the other fields of the server's ``GET /robots/{robot_id}`` answer.

        Raises:
            IsaacSimServerError: If the server is not reachable or answers with an error, e.g.
                404 when it has no robot of that id.
        """
        return await self._request("GET", f"/robots/{robot_id}")

    async def inverse_kinematics(
        self, robot_id: str, poses: list[list[float]], seed: dict | None = None
    ) -> dict:
        """Solve the joint positions that bring the robot's default TCP to each pose.

        Args:
            robot_id: Id of the server's robot.
            poses: Poses ``[x, y, z, rx, ry, rz]`` of the default TCP in the robot's base frame:
                metres, then extrinsic XYZ Euler angles in degrees.
            seed: ``{"joint_names": [...], "joint_positions": [...]}`` in radians to start the
                search from, or ``None``.

        Returns:
            ``{"joint_names": [...], "joint_positions": [...]}`` with one solution in radians
            per pose, ``None`` for a pose that has no solution.

        Raises:
            IsaacSimServerError: If the server is not reachable or answers with an error, e.g.
                404 when it has no robot of that id.
        """
        body = {"poses": poses} if seed is None else {"poses": poses, "seed": seed}
        return await self._request("POST", f"/robots/{robot_id}/inverse_kinematics", body)

    async def delete_robot(self, robot_id: str) -> None:
        """Shut the server's robot ``robot_id`` down and forget it.

        Raises:
            IsaacSimServerError: If the server is not reachable or answers with an error, e.g.
                404 when it has no robot of that id.
        """
        await self._request("DELETE", f"/robots/{robot_id}")

    async def _request(self, method: str, path: str, body: dict | None = None):
        """Send one request on a worker thread and return the decoded JSON answer, or ``None``
        for an empty one.

        Raises:
            IsaacSimServerError: If the server is not reachable or answers with an error.
        """
        return await asyncio.get_running_loop().run_in_executor(
            None, self._send_request, method, path, body
        )

    def _send_request(self, method: str, path: str, body: dict | None):
        """Send one request and wait for the answer. Runs on a worker thread.

        Raises:
            IsaacSimServerError: If the server is not reachable or answers with an error.
        """
        request = urllib.request.Request(
            self._base_url + path,
            data=None if body is None else json.dumps(body).encode(),
            method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=REQUEST_TIMEOUT_SECONDS) as response:
                content = response.read()
        except urllib.error.HTTPError as error:
            raise IsaacSimServerError(
                f"Telekinesis Isaac Sim Server: {_error_detail(error)}", status_code=error.code
            ) from error
        except (urllib.error.URLError, TimeoutError) as error:
            raise IsaacSimServerError(
                f"Telekinesis Isaac Sim Server is not reachable at {self._base_url}. Start it "
                "with: fastapi run --host 127.0.0.1 --port 8770"
            ) from error
        return json.loads(content) if content else None


def _error_detail(error: urllib.error.HTTPError) -> str:
    """Return the ``detail`` of a FastAPI error answer, or the HTTP reason if it has none."""
    try:
        return str(json.loads(error.read())["detail"])
    except (ValueError, KeyError, TypeError):
        return f"{error.code} {error.reason}"
