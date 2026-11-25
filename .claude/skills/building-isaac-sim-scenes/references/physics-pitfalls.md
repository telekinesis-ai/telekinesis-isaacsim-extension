# Physics pitfalls

What made earlier scene builds misbehave in Isaac Sim, with the numbers that fixed them.
They come from the builders in the applications repo (`examples/build_*_scene.py`).
Apply the ones that match what the scene contains.

## Contents

- Composition
- Colliders
- Bodies at rest
- Grasped parts
- Robot drives and inertia
- Suction grippers

## Composition

- **Build on an environment as a sublayer, not a copy.** Its asset paths are relative to
  its own folder and keep resolving. Edits go to the new root layer. Copy the
  environment's `timeCodesPerSecond`, `startTimeCode` and `endTimeCode`: timing is read
  from the root layer only.
- **Remove an environment prim with `SetActive(False)`**, and move one by adding a
  translate op with an `opSuffix` in front of its existing ops
  (`build_palletizing_ur20_scene.py`). Never delete what a sublayer defines.
- **A referenced asset's root carries its own transform ops.** They compose onto your
  referencing prim, so `ClearXformOpOrder()` before adding your own ops. Re-add any
  scale you still need (e.g. `0.01` for an asset authored in centimetres: check
  `metersPerUnit` in `inspect_asset.py`).
- **Instanceable prims cannot be edited below.** `SetInstanceable(False)` on the prim
  first, e.g. to change a collider approximation.
- **Use absolute or layer-relative asset paths with forward slashes.** The template
  references the download cache by absolute path, so the scene opens only on the
  machine that built it. Say so in the report.

## Colliders

- **Moving bodies need convex colliders.** PhysX cannot simulate a triangle mesh on a
  dynamic body. Use `convexHull` for solid parts and `convexDecomposition` when pockets
  must stay open (a pallet's fork pockets: a single hull fills them).
- **Concave containers need exact box colliders.** A convex decomposition of an open
  container has hulls that bulge in from the walls, so items jam and lids slide off
  sloped rim hulls. Author a floor plate and four wall `Cube` colliders with
  `purpose = guide`, and turn off the mesh's collision (`collisionEnabled = False`). The
  mesh stays as the visual.
- **SDF colliders misbehave under non-uniform scale.** They launch bodies on Play.
  Switch to convex approximations on scaled assets.
- **A tray of holes needs a floor.** Parts placed in pockets that are holes over an open
  frame fall through or hang on the rim. Add a thin static plate under the pockets.
- **Visual-only dressing has no collider.** Walls, fences and props beyond the robot's
  reach and the cameras' depth range stay cheap that way.

## Bodies at rest

- **Start parts where their colliders rest, plus 0.5 to 1 mm.** Convex colliders stand a
  few mm proud of their meshes. Parts placed mesh-to-mesh start interpenetrating and
  jump apart on Play. Parts placed with a larger gap drop and bounce. Pallet colliders
  stood 6 mm above the deck.
- **Leave 5 cm between neighbouring containers** that the robot sets down. At 10 mm,
  every placement nudged its neighbours.
- **Make supports kinematic or static.** A dynamic pallet rocked a little each time a
  part was set down, and light lids on it crept by centimetres.
- **Thin or fast bodies need CCD.** Apply `PhysxRigidBodyAPI` with
  `physxRigidBody:enableCCD = True` and set `physxScene:enableCCD = True` on the physics
  scene. A 4 mm floor plate falling 9 mm per 60 Hz step passed through the pallet.
- **Cap depenetration**: `physxRigidBody:maxDepenetrationVelocity = 1.0` m/s. Uncapped,
  a 10 kg item overlapping its container was fired 3 m. Do not go much lower: at 0.1 m/s
  a stack sank 16 mm into itself.
- **Stacks need solver iterations**: `physxRigidBody:solverPositionIterationCount = 16`.
  At the default 4, stacked 10 kg items sank and sprang back by 30 to 40 mm.
- **Keep physics at 60 Hz for settling stacks**
  (`physxScene:timeStepsPerSecond`). At 120 Hz, packed items slid up to 52 mm as they
  settled.
- **Give friction explicitly.** Plastic lids and containers used 1.5/1.3 (static/dynamic),
  cardboard 0.6/0.5, restitution 0. Bind with purpose `physics`.

## Grasped parts

- **Every part has a mass and a physics material.** Reference scenes often ship parts
  with neither.
- **Moderate friction on parts in fixtures.** Friction 1.5 on a blank also held it to
  the tray, so the first jaw tipped it 7 to 10 degrees. 1.0 worked.
- **Two-jaw grippers need flat faces.** A round blank held on two points swung 6 degrees
  and dropped. A square bar held. A bar as wide as its pocket stood on the rim, so
  check part size against fixture size.
- **Hold long parts at their centre of mass**, or the arm's accelerations swing them.

## Robot drives and inertia

- **Drives too stiff act as locks.** Tune from `Kp = J wn^2` and `Kd = 2 zeta J wn`, with
  `J` the joint's largest effective inertia, `wn` about 30 Hz and `zeta = 1`. At 15 Hz
  joints sagged 3.5 mm at the tool. Wrist joints under load may need more stiffness,
  because PhysX drives have no integral term.
- **Author joint speed limits** (`physxJoint:maxJointVelocity`, deg/s from the datasheet).
  Without them a wrist swinging a 10 kg part a full turn at once blew up to NaN.
- **Placeholder URDF inertials** (often 2 kg, 0.01 kg m^2 per link) make the arm far too
  light. Author link `MassAPI` (mass, centre of mass, diagonal inertia, principal axes)
  scaled to the datasheet mass.
- **Author the home pose** (`set_home_pose`). Driving to home from the zero pose on Play
  can swing the arm through nearby objects.
- Do not tune gains or run Isaac Sim pipelines yourself. Give the user the command.

## Suction grippers

- **Raise the grip force limits out of reach** (`isaac:coaxialForceLimit` and
  `isaac:shearForceLimit` = 1e9 N). Stepped arm targets spiked the weld force past
  1000 N and 10 kN and dropped boxes mid-swing.
- **Stiffen the suction drive** (`transZ`: stiffness 1e6, damping 5000). At 1e4 N/m a
  10 kg item hung 10 mm below the pad and was reported lost.
- **Lock the suction joint's limits to zero** on all six axes when the held pose must
  not drift. The asset let parts slide and swivel 2 cm and 3.5 degrees.
- **Grip distance** (`isaac:maxGripDistance`): about 15 mm, as bellows cups reach.
- **Move the suction ray origin to the visible pad face** if the asset authors it
  elsewhere: set the attachment joint's `localPos0`.
