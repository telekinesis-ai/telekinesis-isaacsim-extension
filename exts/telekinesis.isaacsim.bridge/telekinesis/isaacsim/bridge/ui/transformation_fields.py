# SPDX-License-Identifier: Apache-2.0
"""Input fields for the transformation an asset is placed at."""

import math
from collections.abc import Callable

from omni import ui
from omni.kit.window.property import templates

from ..comm import models


class TransformationFields:
    """Two rows of X, Y and Z fields for a transformation: Position in meters and
    Rotation in XYZ Euler degrees.

    The window that builds them says which frame they are read in: the robot's mount
    link for an attach offset, the Attaching Prim for a robot's placement. All zeros
    places the asset flush at that frame's origin. Built into the enclosing ``omni.ui``
    layout when constructed.

    Example:
        transformation_fields = TransformationFields(on_changed_fn=lambda: None)
        transformation_fields.transformation()
        # Transformation(translation=[0, 0, 0], rotation=[0, 0, 0])
    """

    def __init__(self, on_changed_fn: Callable[[], None]):
        """Build the fields; ``on_changed_fn`` is called whenever any of them changes value."""
        self._translation_models = self._build_xyz_row(
            "Position (m)", step=0.001, value_format="%.3f", on_changed_fn=on_changed_fn
        )
        self._rotation_models = self._build_xyz_row(
            "Rotation (deg)", step=1.0, value_format="%.2f", on_changed_fn=on_changed_fn
        )

    def reset(self):
        """Set every field back to zero, placing the asset flush at the frame's origin."""
        self.set_transformation(models.Transformation())

    def transformation(self) -> models.Transformation:
        """Return the entered transformation as the bridge routes take it."""
        return models.Transformation(
            translation=[model.get_value_as_float() for model in self._translation_models],
            rotation=[model.get_value_as_float() for model in self._rotation_models],
        )

    def set_transformation(self, transformation: models.Transformation):
        """Show ``transformation`` in the fields, as if the user had typed it."""
        for model, value in zip(
            self._translation_models + self._rotation_models,
            [*transformation.translation, *transformation.rotation],
        ):
            model.set_value(float(value))

    @staticmethod
    def _build_xyz_row(
        label: str, step: float, value_format: str, on_changed_fn: Callable[[], None]
    ) -> list[ui.AbstractValueModel]:
        """Build one row of ``label`` and an X, Y and Z drag field; return the fields' models.

        ``step`` is the change per dragged pixel and ``value_format`` how the values are
        shown; ``on_changed_fn`` is called whenever any of the three changes value.
        """
        # The fields start right after the label and run to the row's end, like the
        # Add windows' other rows, so both edges line up with them.
        with ui.HStack(height=0):
            ui.Label(
                label,
                width=templates.LABEL_WIDTH,
                alignment=ui.Alignment.LEFT_CENTER,
            )
            with ui.HStack(spacing=5):
                value_models = [
                    ui.FloatDrag(
                        min=-math.inf,
                        max=math.inf,
                        step=step,
                        format=value_format,
                        tooltip=axis,
                    ).model
                    for axis in "XYZ"
                ]
        for value_model in value_models:
            value_model.add_value_changed_fn(lambda _: on_changed_fn())
        return value_models
