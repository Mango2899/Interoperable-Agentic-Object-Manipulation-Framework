# Fitting a gripper that is not the RG2

Everything about the tool the arm holds things with is per-robot configuration
under `runtime.gripper` in a robot's manifest. Four of those values were RG2
constants baked into the generator, and are now flags that survive
regeneration. This is what has to be measured for a new gripper, and why.

## Angles are radians, always

`modelOrientation` and `modelLocalRotation` are radians, because
`sim.setObjectOrientation` and `sim.buildMatrix` are. Only the generator flags
`--gripper-orientation-deg` and `--gripper-local-rotation-deg` take degrees, and
they convert on the way in.

Hand-editing the manifest bypasses that conversion, and the failure is quiet
rather than loud. `110` written for "110 degrees" is 110 radians: seventeen full
turns plus 182.5 degrees, so the gripper lands almost-but-not-quite flipped and
every measurement downstream is taken from that pose. Both fields are now
rejected outright above a full turn, in the pipeline and in the runtime driver,
with a message that names the field and does the conversion for you.

## The four values that are properties of the gripper

| Manifest field | Flag | RG2 | Baxter gripper |
| --- | --- | --- | --- |
| `modelPath` | `--gripper-model-path` | `RG2.ttm` | `Baxter gripper.ttm` |
| `graspPadAlias` | `--gripper-pad-alias` | `touch` | `pad` |
| `toolLengthEstimate` | `--gripper-tool-length` | `0.19` | `0.077` |
| `toolAxis` | `--gripper-tool-axis` | on the mount axis | 5.6 degrees off it |

`graspPadAlias` is a substring matched against the aliases of the shapes in the
gripper's tree, and their bounding-box midpoint is the tool control point: the
point the workspace is sampled at, the point IK steers, and the point a grasp is
measured from. It is a substring rather than a pair of names because model
authors are inconsistent -- the Baxter gripper's own pads are `leftPad` and
`righPad`, with the typo in the shipped model.

A substring that matches nothing is a **hard pipeline failure**, thrown as
`workspace_gripper_contact_pads_not_found: <alias>` before workspace estimation
runs. That is the whole error: nothing else in the message says "gripper", so it
reads like a workspace problem. The RG2 default `touch` matches nothing in the
Baxter gripper, which is exactly how this was found.

The match also skips render-only shapes -- anything whose alias contains
`visual` or `visible`. The Baxter gripper needs this: `pad` matches its
`leftPad_visible` and `rightPad_visible` meshes as readily as the real pads.

## Measuring them

Load the model in CoppeliaSim and read its tree, or mount it on the real flange
and measure. `npm run robot:inspect-tool -- --manifest <manifest>` reports the
mount, the parentage, and the distance from flange to pads once a scene exists.

For the Baxter gripper on Robot4's CR3, measured on the flange:

- pads found: `leftPad`, `righPad`, midpoint 0.0774 m from the flange
- with `modelOrientation` `[-pi/2, 0, 0]` the tool points along flange `+Y`,
  which is the mount every other arm in the cell uses
- the pad midpoint sits 7.6 mm off the mount axis, so `toolAxis` is
  `[0, 0.995, 0.098]` rather than a clean `[0, 1, 0]`

## What did not need adapting

- **Driven joints.** `womatDrivenGripperJoints` finds `closeJoint` on its own:
  the alias heuristic misses it, but the fallback -- a non-cyclic joint with a
  range under 0.2 m -- picks it out and rejects the `centerJoint` linkage.
  With a single configured position the runtime drives it to the joint's own
  interval bounds, so `openJointPositions` is not consulted.
- **The attach marker.** The Baxter gripper exposes `attachPoint` and
  `attachProxSensor` under the same names the RG2 does.

## What is still worth watching

- **The model's own script.** The Baxter gripper ships a simulation child script
  that sets a target velocity on `closeJoint` every step from the scene signal
  `<alias>_close`. The runtime already writes that signal, so the two agree on
  direction, but the runtime also writes the joint kinematically -- two
  controllers on one mechanism. If the jaws behave oddly, `activityDataBlock`
  is the existing opt-out: naming a block makes the runtime write it and leave
  the joints alone.
- **`applyBaseOrientationCorrection`.** Robot4 has it `true`, which is a no-op
  while it stands at the origin. Robot3, the other CR-family arm, sets it
  `false` because the gripper already inherits the corrected transform through
  the TCP hierarchy and applying it again double-rotates the tool. Expect to
  need the same once Robot4 is placed with a yaw.
- **Closed jaw gap.** The Baxter gripper's pads are 34.8 mm apart at
  `closeJoint = 0`, against a 45 mm cube, so they sink about 5 mm into it. The
  grasp is faked by parenting, so this is cosmetic -- but a custom closed
  position needs two or more entries in `closedJointPositions`, because a single
  entry is overridden by the joint's interval bound.

## Where the tool comes from: `runtime.gripper.source`

Not every arm needs a gripper bolted onto it. `source` says which of three
things this robot is, and the pipeline and the runtime read it through one
resolver so they cannot disagree about it.

| `source` | Means | Also needs |
| --- | --- | --- |
| `model` (default) | Load `modelPath` and bolt it to the flange | `modelPath` |
| `builtin` | The arm's own model already carries the tool | `sourceAlias` |
| `none` | No tool at all; the tool control point is the flange | — |

Flags: `--gripper-source`, `--gripper-source-alias`, `--gripper-tcp-offset`.
`shipsWithModel: true` is the older spelling of `builtin` and still resolves to
it.

**`builtin` never loads and never reparents.** That second half is the one that
matters: the tool is a *link of the arm's kinematic chain*, and bolting it onto
the flange dummy the way a loaded gripper is bolted lifts it out of that chain.
The arm then drives one link short while the tool is towed along by a dummy
nothing drives. The runtime's "replace a gripper that has no joints" repair is
skipped for the same reason -- for a built-in tool that repair deletes part of
the robot.

## Tools with nothing to measure: `toolControlPointOffset`

A jawed gripper's tool control point is *measured* -- the midpoint of the shapes
matching `graspPadAlias`. A fixed tool, a suction cup, or a gripper that is only
a housing mesh has no such shapes, so its business end is *declared* instead, in
the tool's own frame, and it still follows the wrist.

Measuring wins wherever there is anything to measure: pads move when the jaws
open and a declared offset does not. So the offset is consulted only when the
pad scan comes up empty, and for a jawed gripper an empty scan stays a hard
failure -- there it means the alias is wrong, and sampling the flange instead
looks like a good workspace and is wrong by a whole tool length.

## Robot5, the uArm SwiftPro

Its URDF ships an end effector: `Link8` wears `Gripper.STL`, a 122 x 52 x 85 mm
housing. So Robot5 is `source: "builtin"`, `sourceAlias: "Link8_respondable"`,
and the RG2 that earlier runs bolted on is gone.

What that housing is **not** is a working gripper. There are no contact pads, no
second jaw and no joint to open: the URDF gives `Link8` only `Joint9`, and that
drives `Link9`, a 171 x 2 x 12 mm tie-rod. So its tool control point is declared
-- `[0.063, 0, -0.049]`, the centroid of the mesh's downward-facing business end
in `Link8`'s frame -- and open/close is cosmetic. Grasping still works, because
a grasp has always been faked by parenting the cube to the tool.

`Suction.STL` sits unused beside the meshes, which says what the real end
effector was meant to be.
