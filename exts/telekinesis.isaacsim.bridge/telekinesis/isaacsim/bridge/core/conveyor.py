# SPDX-License-Identifier: Apache-2.0
"""
Conveyor belt device: binds one conveyor already in the stage and runs it.

A conveyor belt is a kinematic rigid body carrying ``PhysxSurfaceVelocityAPI``.
PhysX injects that velocity at the belt's contact points through a contact-modify
callback, so bodies resting on the belt are dragged along while the belt itself
never moves. Because the effect is a contact callback it needs CPU physics and
does nothing when the scene runs on the GPU pipeline.

Scenes author a belt's running velocity in one of two ways, and this class drives
whichever one it finds:

* directly on the belt, as a ``physxSurfaceVelocity:surfaceVelocity`` vector whose
  direction is the belt's travel direction and whose length is its running speed --
  how the NVIDIA warehouse conveyor props behave, since the props themselves are
  only rigid bodies;
* through an ``isaacsim.asset.gen.conveyor.IsaacConveyor`` OmniGraph node, which the
  Conveyor Belt Utility creates. The node recomputes ``direction * velocity`` every
  tick and writes it to the belt, so the belt's own velocity attribute cannot be set
  from outside -- the node overwrites it on the next tick. Such a belt is driven
  through the action graph's velocity variable instead, and only responds while the
  timeline plays.

Either way a belt runs at a *signed speed along the direction its scene authored*,
so reversing a belt means passing a negative speed and no direction is sent over the
bridge. The belt has to be provisioned in the stage already; nothing here creates a
conveyor, because the travel direction is the one thing that cannot be guessed.

Native Isaac units: meters per second for a straight belt, radians per second for a
curved one (which PhysX drives with an angular surface velocity instead).
"""

import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
import carb
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics

# node:type of the OmniGraph node the Conveyor Belt Utility builds for a belt.
CONVEYOR_NODE_TYPE = "isaacsim.asset.gen.conveyor.IsaacConveyor"

# What sets the bound belt's speed, as reported by info()["drive"].
SURFACE_VELOCITY_DRIVE = "surface_velocity"
CONVEYOR_NODE_DRIVE = "conveyor_node"

# Where the Conveyor Belt Utility is documented, named by the refusals below so
# that a belt which is not provisioned as a conveyor says how to provision it.
CONVEYOR_DOC_URL = (
    "https://docs.isaacsim.omniverse.nvidia.com/6.0.1/digital_twin/"
    "warehouse_logistics/ext_isaacsim_asset_gen_conveyor.html"
)

# How to author a belt's running velocity by hand, for the same refusals.
AUTHOR_VELOCITY_HINT = (
    "set physxSurfaceVelocity:surfaceVelocity on the belt to a vector pointing along its "
    "travel whose length is its running speed in meters per second (Property > Physics on "
    "the selected belt prim), or rebuild the belt with the Conveyor Belt Utility: "
    f"{CONVEYOR_DOC_URL}"
)


class Conveyor:
    """Binds one conveyor belt at ``prim_path`` and starts/stops it.

    The path may point at the conveyor asset's root, at the belt rigid body itself,
    or at any prim in between, so a path taken straight from the stage tree works.

    The belt's first binding in the open stage captures the velocity the scene
    authored as its travel direction and its default running speed, so bind against a
    belt that is at rest (or at its authored speed). Binding it again reuses that
    capture rather than re-reading an attribute this bridge has since written a
    command into, which is what ``authored`` carries.
    """

    def __init__(self, prim_path, name="conveyor", cargo_root=None, authored=None):
        """Resolve the belt and capture the running velocity its scene authored.

        ``authored`` is the capture a previous binding of the same belt produced (see
        the :attr:`authored` property), reused instead of reading the stage again;
        ``None`` reads the stage, which is what a belt's first binding does.

        ``cargo_root`` is the prim whose rigid bodies are woken when the belt starts;
        ``None`` wakes nothing. PhysX leaves sleeping bodies out of the contact solve,
        so a surface velocity cannot reach cargo that came to rest while the belt was
        stopped -- narrow this to the prims the belt actually carries, since waking a
        whole warehouse costs a pass over every prim in it.

        Raises ``ValueError`` if no stage is open, the path does not resolve, no rigid
        body is found at or below it, the belt has neither a surface velocity nor a
        conveyor node driving it, or its velocity is zero with no weaker layer holding
        the one the scene authored, so its travel direction is unknown.
        """
        self.prim_path = prim_path
        self.cargo_root = cargo_root
        self._name = name
        self._remembered = authored

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise ValueError("no USD stage is open")

        self._belt_prim = self._resolve_belt(stage, prim_path)
        node_prim = self._find_conveyor_node(stage, self._belt_prim)
        if node_prim is None:
            self.drive = SURFACE_VELOCITY_DRIVE
            self._connect_surface_velocity()
        else:
            self.drive = CONVEYOR_NODE_DRIVE
            self._connect_conveyor_node(stage, node_prim)
        carb.log_info(
            f"[bridge] bound conveyor {self.prim_path} ({self.drive}) at "
            f"{self._nominal_speed} m/s"
        )

    @property
    def authored(self):
        """What this belt's travel direction and running speed were captured as.

        The registry hands this back when the same belt is bound again, so that a
        re-registration reuses the capture instead of re-reading an attribute this
        bridge has since written a command into: a belt started in reverse would
        otherwise come back with its travel direction flipped.
        """
        return {
            "drive": self.drive,
            "attribute": self._velocity_attribute.GetName(),
            "direction": None if self._direction is None else tuple(self._direction),
            "nominal_speed": self._nominal_speed,
        }

    # -- bridge lifecycle (not part of the isaacsim surface) --------------------

    async def bind(self):
        """Play the timeline so the belt can move what it carries.

        There is nothing here to wait for -- resolving the belt is a stage read -- but a
        conveyor is brought up the same way an articulation or a sensor is, because a
        stopped simulation is the one state in which it silently does nothing: a surface
        velocity reaches cargo through a contact callback, and a sleeping body cannot be
        woken while nothing is stepping. Pumps a couple of frames so the simulation has
        stepped by the time the belt is started.

        Safe to call repeatedly.
        """
        omni.timeline.get_timeline_interface().play()
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()

    # -- control ----------------------------------------------------------------

    def start(self, velocity=None):
        """Run the belt at a signed speed, and wake the cargo it carries.

        ``velocity`` defaults to the speed the scene authored; a negative value
        reverses the belt against its authored direction. A belt driven by a conveyor
        node only starts moving once the timeline plays, because the node recomputes
        its velocity on tick.

        Raises ``ValueError`` if the belt has no authored speed to fall back on, or
        ``RuntimeError`` if the configured cargo root has gone from the stage.
        """
        self._write_velocity(velocity)
        if not omni.timeline.get_timeline_interface().is_playing():
            # The speed is written and stays written, so this is a belt that will run
            # the moment the timeline plays rather than a command that was dropped.
            carb.log_warn(
                f"[bridge] conveyor {self.prim_path} was started while the simulation "
                "is stopped; it will not move anything until the timeline plays"
            )
        # PhysX picks a surface velocity up reliably only when the API is re-enabled,
        # which is also how the conveyor node gets its own velocity applied on the
        # first simulation step (see OgnIsaacConveyor's "cycle the enabled attr").
        if self._enabled_attribute is not None:
            self._enabled_attribute.Set(False)
            self._enabled_attribute.Set(True)
        return self._wake_cargo()

    def stop(self):
        """Stop the belt.

        A belt driven by its own surface velocity is switched off rather than set to
        zero, so the running velocity the scene authored stays on the stage and the
        belt can be restarted at that speed without it having to be sent again.
        """
        if self._enabled_attribute is not None:
            self._enabled_attribute.Set(False)
        else:
            self._write_velocity(0.0)

    def info(self):
        """Static description plus the belt's current speed and whether it is running.

        ``direction`` is the unit travel vector the scene authored, in the belt's own
        frame, or ``null`` for a node-driven belt -- the node owns the direction and
        applies it itself, so the bridge only ever writes a scalar there.
        """
        return {
            "prim_path": self.prim_path,
            "belt_prim_path": self._belt_prim.GetPath().pathString,
            "drive": self.drive,
            "direction": None if self._direction is None else list(self._direction),
            "nominal_speed": self._nominal_speed,
            "velocity": self.get_velocity(),
            "running": self.is_running(),
        }

    def get_velocity(self):
        """The signed speed currently written to the belt, along its authored direction."""
        value = self._velocity_attribute.Get()
        if value is None:
            return 0.0
        if self._direction is None:
            return float(value)
        # The stage holds a vector; report the component along the authored direction,
        # so a reversed belt reads back negative rather than as a positive length.
        return float(Gf.Dot(Gf.Vec3f(value), self._direction))

    def is_running(self):
        """Whether the belt is set to move: a non-zero speed with its drive switched on."""
        if self._enabled_attribute is not None and not bool(self._enabled_attribute.Get()):
            return False
        return self.get_velocity() != 0.0

    def destroy(self):
        """Release the wrapper. Does not stop the belt and does not touch the stage.

        Called on unregister and on a stage change, where the prim this wrapper points
        at may already be gone -- so it deliberately writes nothing, and a belt left
        running keeps running. Stop a belt before unregistering it. Nothing else is
        held: the conveyor lives and dies with its USD prim, which the bridge leaves in
        the stage on delete, exactly like a camera's or a lidar's.
        """

    # -- internals --------------------------------------------------------------

    @staticmethod
    def _resolve_belt(stage, prim_path):
        """Return the rigid body prim that carries the conveyor's surface velocity.

        Conveyor assets nest the belt below a hierarchy root, so a path is resolved by
        walking up to the nearest rigid body ancestor and, failing that, down to the
        first rigid body descendant.
        """
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"prim {prim_path!r} not found in the open stage")

        ancestor = prim
        while ancestor.IsValid() and not ancestor.IsPseudoRoot():
            if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                return ancestor
            ancestor = ancestor.GetParent()

        for descendant in Usd.PrimRange(prim):
            if descendant.HasAPI(UsdPhysics.RigidBodyAPI):
                return descendant

        raise ValueError(
            f"no rigid body at or below {prim_path!r}; a conveyor belt has to be a "
            "rigid body for PhysX to apply a surface velocity to it"
        )

    @staticmethod
    def _find_conveyor_node(stage, belt_prim):
        """Return the ``IsaacConveyor`` node targeting ``belt_prim``, or ``None``.

        A conveyor graph is authored anywhere on the stage, not necessarily beside the
        belt it drives, so the whole stage is searched.
        """
        for prim in stage.Traverse():
            node_type = prim.GetAttribute("node:type")
            if not node_type.IsValid() or node_type.Get() != CONVEYOR_NODE_TYPE:
                continue
            relationship = prim.GetRelationship("inputs:conveyorPrim")
            if relationship and belt_prim.GetPath() in relationship.GetTargets():
                return prim
        return None

    def _connect_surface_velocity(self):
        """Bind a belt whose surface velocity is authored directly on it.

        A curved belt is driven by an angular surface velocity rather than a linear
        one, so whichever of the two the scene authored is the one written.
        """
        surface_velocity = PhysxSchema.PhysxSurfaceVelocityAPI(self._belt_prim)
        if not surface_velocity:
            raise ValueError(
                f"the belt at {self._belt_prim.GetPath()} has no surface velocity and no "
                "conveyor node driving it, so it is not provisioned as a conveyor: apply "
                f"PhysxSurfaceVelocityAPI to it, then {AUTHOR_VELOCITY_HINT}"
            )

        if self._adopt_remembered():
            self._enabled_attribute = surface_velocity.CreateSurfaceVelocityEnabledAttr()
            return

        attribute = surface_velocity.GetSurfaceVelocityAttr()
        angular_attribute = surface_velocity.GetSurfaceAngularVelocityAttr()
        authored = Gf.Vec3f(attribute.Get() or Gf.Vec3f(0.0))
        if authored.GetLength() == 0.0:
            angular_authored = Gf.Vec3f(angular_attribute.Get() or Gf.Vec3f(0.0))
            if angular_authored.GetLength() > 0.0:
                attribute = angular_attribute
                authored = angular_authored

        # A belt reading zero may still be authored in a weaker layer: what zeroes one
        # is usually a stronger `over` or a session-layer edit, which leaves the scene's
        # own opinion underneath. Recovering it beats refusing a belt whose direction is
        # still on the stage, and beats guessing one that is not.
        if authored.GetLength() == 0.0:
            for candidate in (attribute, angular_attribute):
                recovered, layer = self._weaker_opinion(candidate)
                if recovered is None:
                    continue
                attribute = candidate
                authored = recovered
                carb.log_warn(
                    f"[bridge] the belt at {self._belt_prim.GetPath()} reads a zero "
                    f"{candidate.GetName()}, so its travel direction was taken from the "
                    f"{tuple(recovered)} authored in {layer.identifier}"
                )
                break

        if authored.GetLength() == 0.0:
            raise ValueError(
                f"the belt at {self._belt_prim.GetPath()} has a zero "
                f"{attribute.GetName()} and no weaker layer holds the velocity it was "
                "authored with, so its travel direction is unknown -- an earlier run may "
                f"have overwritten it. To provision the belt, {AUTHOR_VELOCITY_HINT}"
            )

        self._velocity_attribute = attribute
        self._enabled_attribute = surface_velocity.CreateSurfaceVelocityEnabledAttr()
        self._direction = authored.GetNormalized()
        self._nominal_speed = float(authored.GetLength())

    def _connect_conveyor_node(self, stage, node_prim):
        """Bind a belt driven by a conveyor node, through the graph variable feeding it.

        The Conveyor Belt Utility feeds the node's velocity input from a graph variable
        so that several belts can share one speed. That variable is what has to be
        written; a value written to the node's input while it is connected is discarded.
        """
        enabled = node_prim.GetAttribute("inputs:enabled")
        if enabled.IsValid() and not enabled.Get():
            raise ValueError(
                f"the conveyor node at {node_prim.GetPath()} is disabled and ignores any "
                "velocity written to it"
            )

        attribute = node_prim.GetAttribute("inputs:velocity")
        connections = attribute.GetConnections()
        if connections:
            reader_prim = stage.GetPrimAtPath(connections[0].GetPrimPath())
            variable_name = reader_prim.GetAttribute("inputs:variableName").Get()
            graph_path = reader_prim.GetAttribute("inputs:graph").Get()
            graph_prim = stage.GetPrimAtPath(
                graph_path if graph_path else reader_prim.GetParent().GetPath()
            )
            attribute = graph_prim.GetAttribute(f"graph:variable:{variable_name}")
            if not attribute.IsValid():
                raise ValueError(
                    f"the conveyor node at {node_prim.GetPath()} reads a variable "
                    f"{variable_name!r} that {graph_prim.GetPath()} does not declare"
                )

        self._velocity_attribute = attribute
        # The node owns the belt's surface velocity, including when it is applied, so
        # the API must not be switched from outside.
        self._enabled_attribute = None
        self._direction = None
        # The graph variable read here is the one start() writes to, so a re-binding
        # reads back a commanded speed rather than the authored one unless the capture
        # from the first binding is reused.
        remembered = self._remembered
        if remembered is not None and remembered["drive"] == CONVEYOR_NODE_DRIVE:
            self._nominal_speed = remembered["nominal_speed"]
        else:
            self._nominal_speed = float(attribute.Get() or 0.0)

    def _adopt_remembered(self):
        """Take direction and speed from a previous binding's capture, if it still fits.

        For a belt driven by its own surface velocity; a node-driven belt adopts its
        speed where it resolves the graph variable, which is not the belt's attribute.
        Answers whether the capture was used. It is only reused for an attribute the
        belt still has, so a stage re-authored under the same prim path falls through
        to a fresh read rather than keeping a stale capture.
        """
        remembered = self._remembered
        if remembered is None or remembered["drive"] != SURFACE_VELOCITY_DRIVE:
            return False

        attribute = self._belt_prim.GetAttribute(remembered["attribute"])
        if not attribute.IsValid():
            return False

        self._velocity_attribute = attribute
        self._direction = Gf.Vec3f(*remembered["direction"])
        self._nominal_speed = remembered["nominal_speed"]
        return True

    @staticmethod
    def _weaker_opinion(attribute):
        """Return the strongest non-zero opinion on ``attribute``, with its layer.

        ``(None, None)`` when every layer holding the attribute holds a zero. The
        strongest opinion is the zero that sent us here, so it drops out on its own.
        """
        if not attribute.IsValid():
            return None, None

        for spec in attribute.GetPropertyStack(Usd.TimeCode.Default()):
            if spec.default is None:
                continue
            vector = Gf.Vec3f(spec.default)
            if vector.GetLength() > 0.0:
                return vector, spec.layer
        return None, None

    def _write_velocity(self, velocity):
        """Write a signed speed to the belt, defaulting to its authored running speed."""
        if velocity is None and self._nominal_speed == 0.0:
            raise ValueError(
                f"the conveyor at {self.prim_path} has no authored running speed to fall "
                "back on; send a velocity to start it"
            )
        speed = self._nominal_speed if velocity is None else float(velocity)
        if self._direction is None:
            self._velocity_attribute.Set(speed)
        else:
            self._velocity_attribute.Set(self._direction * speed)

    def _wake_cargo(self):
        """Wake the sleeping rigid bodies under ``cargo_root`` and report how many.

        Waking a body that is already awake has no effect, and an idle body falls
        asleep again on its own. Nothing is woken while the timeline is stopped,
        because no body is simulating then.
        """
        if self.cargo_root is None:
            return 0
        if not omni.timeline.get_timeline_interface().is_playing():
            return 0

        context = omni.usd.get_context()
        root_prim = context.get_stage().GetPrimAtPath(self.cargo_root)
        if not root_prim.IsValid():
            raise RuntimeError(f"cargo root prim {self.cargo_root!r} not found in the open stage")

        simulation = omni.physx.get_physx_simulation_interface()
        stage_id = context.get_stage_id()
        woken = 0
        for prim in Usd.PrimRange(root_prim):
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            simulation.wake_up(stage_id, PhysicsSchemaTools.sdfPathToInt(prim.GetPath()))
            woken += 1
        return woken
