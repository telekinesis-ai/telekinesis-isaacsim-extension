# SPDX-License-Identifier: Apache-2.0
"""The Add Camera window opened from the right-click Create menu."""

from .add_mesh_window import AddMeshWindow

CAMERA_CATEGORY = "cameras"


class AddCameraWindow(AddMeshWindow):
    """A window that loads a camera from the Telekinesis asset library, mounting it on a robot.

    The camera is added as Add Mesh adds a mesh. When the attaching prim is part of a
    robot, the camera is placed beside the robot -- in the robot's ``<model>_assembly``
    Xform when Add Robot created one -- and fixed to the robot's stored mount link at the
    entered offset, as ``mount_prim`` does, so it moves with the arm. A robot that stores
    no mount link refuses the add before anything is loaded. Otherwise the camera is added
    under the attaching prim as it is. The camera is not registered for capture: register
    its camera prim with ``PUT /cameras`` (``GET /cameras/discover`` on the added prim
    lists it).

    Example:
        window = AddCameraWindow(bridge_server.asset_library_service,
                                 bridge_server.articulation_service,
                                 bridge_server.stage_service)
        window.show("/World/ur10e_assembly/ur10e")
    """

    # TODO: register the added camera with CameraService, as PUT /cameras does, once a
    # default resolution and frequency for catalogue cameras are decided (e.g. as catalogue
    # fields, like a robot's simulation_mount_link).
    window_title = "Add Camera"

    def asset_categories(self) -> list[str]:
        """Offer the cameras of the catalogue."""
        return [CAMERA_CATEGORY]
