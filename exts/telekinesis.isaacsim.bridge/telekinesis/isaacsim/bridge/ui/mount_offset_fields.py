# SPDX-License-Identifier: Apache-2.0
"""Input fields for the pose an asset is attached at on a robot's mount link."""

from isaacsim.gui.components import element_wrappers

from ..comm import models


class MountOffsetFields:
    """Six fields for an attach offset: translation in meters and rotation in XYZ Euler degrees.

    Both are read in the robot's mount-link frame; all zeros attaches the asset flush
    at the mount link's origin. Built into the enclosing ``omni.ui`` layout when
    constructed.

    Example:
        offset_fields = MountOffsetFields()
        offset_fields.transformation()  # Transformation(translation=[0, 0, 0], rotation=[0, 0, 0])
    """

    def __init__(self):
        self._translation_fields = [
            element_wrappers.FloatField(f"Offset {axis} (m)", format="%.3f", step=0.001)
            for axis in "XYZ"
        ]
        self._rotation_fields = [
            element_wrappers.FloatField(
                f"Rotation {axis} (deg)", format="%.1f", step=1.0
            )
            for axis in "XYZ"
        ]

    def reset(self):
        """Set every field back to zero, attaching flush at the mount link's origin."""
        for field in self._translation_fields + self._rotation_fields:
            field.set_value(0.0)

    def transformation(self) -> models.Transformation:
        """Return the entered offset as the mount transform the bridge routes take."""
        return models.Transformation(
            translation=[field.get_value() for field in self._translation_fields],
            rotation=[field.get_value() for field in self._rotation_fields],
        )
