# E15: a second cell, a smaller arm, and a route with no alternative

E11, E12, E13 and E14 all run on one cell. Its five arms, their mountings, its
two supports and its walls are the same inputs throughout, and E14 perturbs only
the obstacles inside it. So every number those experiments produce is a
statement about that cell, and the claim the approach makes — that it is driven
by the cell it observes rather than by the constants of one hand-tuned scene —
is not tested by any of them.

E15 builds a different cell. Four arms instead of five, different base
placements, different mountings, both supports somewhere else, and one of the
four arms replaced by a Dobot Magician: a 4-DOF desktop arm with a reach of
0.45 m, against the 0.86 m to 1.60 m of the others. Nothing is re-tuned for it.
The pipeline is run with new `--offsets`, `--orientations`, `--wall-offsets` and
`--sites`, every arm is resampled, and the cell is asked to deliver the cube.

The Dobot is the point of the experiment rather than a detail of it. It is the
one arm in the repository that does not come from a URDF import, its end
effector is a mechanism no URDF can describe, and its reach is small enough that
it changes the shape of the problem: it makes exactly one route through the cell
possible, so the search has nothing to choose between and everything to refuse.

## The cell

| | Base offset [m] | Mounting $(r,p,y)$ [deg] | Wall offset [m] | Arm | Reach [m] |
|---|---|---|---|---|---|
| Robot1 | `0, 0, 0` | `0, 0, -15` | `0, 0, 0` | UR10 | 1.597 |
| Robot2 | `1.5, 0.2, 0` | `0, 0, 180` | `0, 0, 0` | UR3 | 0.863 |
| Robot3 | `2.1, -1, 0` | `0, -90, 180` | `-0.42876, -0.25726, 0` | CR5 | 1.279 |
| Robot4 | `1.5, -2, 0` | `0, 0, 0` | `0.9, 0.5, 0` | **Dobot Magician** | **0.447** |

Support A stands at `(0.75, 0.40, 0.08)` holding the cube at `z = 0.185`;
support B stands at `(1.50, -2.20, 0.08)`, empty, 0.20 m in front of the Dobot.
Robot3 is the only arm mounted on its side (pitch $-90^\circ$), which is what
lets it reach down to the Dobot's working height.

Reach is the largest $\lVert p \rVert$ over the arm's own valid tip-frame
samples, and it is the number the neighbour test uses. It is also the number
that decides this cell: the Dobot's 0.447 m is less than a third of Robot1's.

## Where the Dobot comes from, and how its movement core was integrated

The Dobot is taken from RobWoT, and it is taken at three separate layers. Each
one is worth naming, because each is a different kind of reuse and only the
middle one is the "movement core".

### 1. The kinematics: a URDF, as for every other arm

`external/RobWoT/Assets/dobot_with_new_gripper/dobot/urdf/dobot1.urdf` supplies
the joint names, the chain and the limits, and those go into the manifest
exactly as a UR10's do:

| Joint | Interval [rad] |
|---|---|
| `dobot1/joint_1` | $[-2.25, 2.25]$ |
| `dobot1/joint_2` | $[0, 1.5708]$ |
| `dobot1/joint_3` | $[0, 1.5708]$ |
| `dobot1/joint_4` | $[-3.1416, 3.1416]$ |
| `dobot1/joint_grip` | $[-2.5, 2.5]$ |

Root link `dobot1/base_link`, tip link `dobot1/link_grip`. Note that
`joint_grip` is an *arm* joint: it is a link of the kinematic chain that carries
the tool, not a jaw. The jaws are elsewhere and are not in this list at all.

### 2. The geometry and the mechanism: adopted from a scene, not imported

The rod gripper is a four-bar linkage closed by dummy link pairs and driven by
two prismatic hand joints. A URDF describes trees, not closed loops, so
importing `dobot1.urdf` yields an arm with a gripper-shaped decoration that
cannot grip. The pipeline therefore has a second acquisition mode,
`--adopt-model`, which takes the arm out of a scene that already contains the
working mechanism:

```json
"sourceScene":  ".../Assets/dobot_with_new_gripper/dobot_magician_with_gripper.ttt",
"sourceModel":  "/dobot_normal",
"sourceGripper": "gripper_rod"
```

Adoption is not a copy. Four things happen to the model on the way out of that
scene, and each of them exists because the vendor scene is a *running demo*
rather than a component:

* **The arm's own controllers are removed.** RobWoT's scene drives the Dobot
  itself: an IK environment, a motion coroutine, and a `sysCall_actuation` that
  resumes it. Left in place, they run alongside the WoMAT driver on the same
  joints, and the arm is commanded by two controllers that do not know about
  each other. Every simulation and customization script in the arm subtree is
  stripped — **except the gripper's**, which is the mechanism rather than a
  controller, and which the runtime goes on to speak to.
* **The flange is re-established.** The source ships `tip` and `target` dummies
  out at the jaws, because they were the tip of a full-pose IK, not a mount. The
  driver's flange resolver would take them for the flange and bolt the tool onto
  it, putting the jaws at two tool lengths. Both dummies are deleted and a new
  pair is created at the gripper's own frame, which is where the tool actually
  bolts on.
* **The gripper subtree is renamed** to `Robot4_tcp_gripper`, so that everything
  downstream finds it by the same alias a loaded gripper would carry.
* **Everything else in the scene is dropped** — the table, the cubes and
  somebody else's robot — and only the adopted model is saved.

### 3. The movement core: RobWoT's driver Lua, with three edits

This is the part the question is really about. WoMAT does **not** reimplement
motion. Both of RobWoT's driver scripts are read from the submodule and used as
the base layer, and the WoMAT layer is appended after them:

| RobWoT file | Used for | Lines |
|---|---|---|
| `Library/Robot_WoT_Server/robot_driver.lua` | the runtime driver in the cell | 288 |
| `Library/GenerateRobotDescription/robot_driver_workspace.lua` | workspace sampling | 642 |

From the first come the primitives that actually write joints:
`moveToConfig` (CoppeliaSim's Ruckig profile via `sim.moveToConfig`),
`moveToPose_viaIK` (`simIK` against the `tip`/`target` pair), `movCallback`
(which chooses `setJointTargetPosition` for a dynamically enabled joint and
`setJointPosition` otherwise), `getJointposition`, `getCartesianposition` and
`robotInfo`. From the second comes `generateWorkingspace`, the random/divisions
sampler that produces the reachable cloud.

Three edits turn that into something a multi-agent cell can drive:

**The coroutine pump is switched off.** RobWoT's `moveTojoint` is fire and
forget: it stores the target, creates a coroutine, sets `rState`, and returns
`"success"` immediately, while `sysCall_actuation` resumes the coroutine each
step. A cell that has to reserve space, watch for a collision mid-motion and
abort needs a call that blocks and can be interrupted. So `sysCall_actuation`
is renamed to `sysCall_actuation_disabled`, `moveTojoint` to `moveTojoint_async`
and `moveToPosition` to `moveToPosition_async`, and the WoMAT layer paces the
motion itself, one sample per simulation step, calling the vendor primitives
underneath.

**The joint-selection cutoff is replaced.** RobWoT takes the first six joints it
finds in tree order:

```lua
newjointHandles = {}
for i=1,#jointHandles,1 do
    newjointHandles[i] = jointHandles[i]
    if i==6 then break end
end
```

with its own comment beside it: *"6 is quite special cases, because gripper has
joint can't be identify — modify this part later."* On a six-axis UR that is
right by accident. On the Dobot it is wrong in both directions at once: the arm
has five chain joints, not six, and the model also carries the gripper
mechanism's own prismatic joints, which tree order does not separate from the
arm's. WoMAT replaces the cutoff with a walk up the parents of `./tip`, keeping
only joints that lie on the kinematic path to the tool. The substitution is
applied to **both** vendor files from one function, because the two inherited
the same cutoff and a mis-selected arm corrupts the sampled cloud exactly as it
corrupts the live IK. Both substitutions are anchored: if the vendor text ever
changes, generation stops naming the anchor it could not find rather than
silently producing a driver with the patch missing.

**The tool control point is redirected.** The sampler is pointed at a dummy at
the contact-pad midpoint instead of at `./tip`, so the cloud describes the point
the arm actually grips with. `./tip` is never moved — it is the mount, in the
model, in every saved scene and in the runtime driver, and moving it would
permanently redefine the flange.

### 4. The gripper: one interface, written once

The adopted mechanism keeps its own script, which reads a custom data block on
the gripper root every step and sets a target **velocity** from it, so the jaws
close until they meet something and hold. Writing those joints from WoMAT as
well would be two controllers on one mechanism — ours setting a kinematic
position, its setting a velocity, each undoing the other every step. So where a
manifest names an activity block, that block is the only thing written:

```json
"activityDataBlock": "activity",
"activityClosedValue": "on",
"activityOpenValue": "off",
"graspPadAlias": "hand",
"source": "builtin",
"sourceAlias": "Robot4_tcp_gripper"
```

`source: "builtin"` is what keeps the tool from being loaded, reparented or
"repaired": it is a link of the arm's own chain, and bolting it onto the flange
dummy the way a loaded RG2 is bolted would lift it out of that chain — the arm
would then drive one link short while the tool is towed by a dummy nothing
drives.

RobWoT also ships a WoT-level client for this arm,
`Setups/IoT_Remote_Lab/virtualDevices.ts`, whose `virtualDobot` class drives
`/dobot_normal` by calling `fetchJointHandles`, `moveJoint`, `check` and
`setGripperstate` on the model's own script at 22 deg/s, 40 deg/s², 80 deg/s³.
That layer is *not* reused: it is the single-arm remote-lab client that the
generated Thing Description replaces. What is reused is everything below it.

## The interaction clouds

All five pairwise spaces were recomputed for this cell from the same estimator
and the same configuration, which is the cell descriptor's `interactionSpace`
block:

| Parameter | Value |
|---|---|
| `targetVoxelCount` | 1000 |
| `proximityFactor` | 1.25 |
| `sampleSize` | 20000 |
| `spacingPercentile` | 0.75 |
| `minVoxelSize` / `maxVoxelSize` | 0 / 0.2 m |
| `minimumSharedVoxels` | 10 |
| `bidirectional` | true |
| `seed` | 42 |
| `centralCore` quantiles | 0.2 / 0.8 |
| `centralCore` candidates | 10 min, 60 max |
| estimator mode | `kd_tree_proximity` |
| handover height band | 0.15–0.45 m |

The voxel size of a pair is not configured: it is the coarser of the two arms'
own sample spacings, and the proximity radius is that spacing times
`proximityFactor`. Per-arm spacings in this cell are 0.0491 m (Robot1),
0.0217 m (Robot2), 0.0323 m (Robot3) and 0.0083 m (Robot4), so every pair the
Dobot takes part in is voxelized at its partner's spacing rather than its own.

| Pair | Base sep. [m] | $r_a + r_b$ [m] | Slack [m] | Voxel size [m] | Shared voxels | In band | Verdict |
|---|---|---|---|---|---|---|---|
| Robot1 – Robot2 | 1.513 | 2.460 | $+0.947$ | 0.0491 | **371** | 129 | priced |
| Robot2 – Robot3 | 1.342 | 2.142 | $+0.800$ | 0.0323 | **202** | 76 | priced |
| Robot3 – Robot4 | 1.166 | 1.726 | $+0.560$ | 0.0323 | **25** | 15 | priced |
| Robot1 – Robot3 | 2.326 | 2.876 | $+0.550$ | 0.0491 | 7 | 7 | **refused: below floor of 10** |
| Robot1 – Robot4 | 2.500 | 2.044 | $-0.456$ | — | — | — | **never computed: no overlap** |
| Robot2 – Robot4 | 2.200 | 1.310 | $-0.890$ | — | — | — | not a neighbour |

Reachable sample counts behind those spaces: 25 626 of 35 106 for Robot1,
32 527 of 38 543 for Robot2, 37 903 of 39 710 for Robot3, and 32 212 of 39 341
for Robot4.

Two rows deserve reading carefully, because between them they are the reason
this cell has one route.

**Robot1 – Robot4 is discovered and never priced.** The cell descriptor lists it
as a pair, because pair discovery in the descriptor is an axis-aligned
bounding-box overlap and Robot1's box is large enough to reach the Dobot's. The
agents never compute it: their own neighbour test is by reach, and 2.500 m of
separation against 2.044 m of combined reach is not an overlap at any height.
No `space.json` for that pair exists, and it appears nowhere in
`performance_log/interaction-compute.jsonl`. This is the bounding box being an
over-approximation and the reach test settling it — exactly the division of
labour the neighbour test is for.

**Robot1 – Robot3 is priced-eligible and refused.** The two arms genuinely
overlap: 0.55 m of slack, and seven shared voxels all of which fall inside the
handover band. Seven is below the floor of ten, so the pair is refused as a
rendezvous and the arms are told so. This is the case the floor exists for — two
reach spheres grazing each other produce a handful of voxels at the far edge of
both workspaces, and one voxel prices exactly as confidently as three hundred.
Note that the verdict does not depend on the floor's value here: at the
five-robot cell's stricter floor of 20 the outcome is identical, since
Robot3 – Robot4 has 25.

**The Dobot's space is the smallest usable one in the repository.** Twenty-five
voxels, fifteen of them in the band, spanning $x \in [1.44, 1.76]$,
$y \in [-1.96, -1.70]$, $z \in [-0.02, 0.34]$. It sits low, because the Dobot is
short, and it is the reason the handover band matters: at the default band of
0.3–0.7 m only four of its voxels would qualify.

## Why there is exactly one route

Robot1 holds the cube and Robot4 owns support B. Of the six possible pairs, two
are not neighbours at all, one is refused for being too small, and three remain.
Those three form a path graph, so

$$\text{Robot1} \rightarrow \text{Robot2} \rightarrow \text{Robot3} \rightarrow \text{Robot4}$$

is not the cheapest route: it is the only one. Every shortcut is gone, and it is
gone for a different reason in each case — geometry for Robot1 – Robot4, the
voxel floor for Robot1 – Robot3.

That makes the cell a different kind of test from E12 and E13. There, the
contribution was that the cell chose correctly between two routes and repaired
its choice when one failed; the risk being measured was a wrong decision. Here
there is no decision to get wrong and the risk is the opposite one: that the
search fails to find the only route there is, or refuses an edge it needs. Three
of the five discovered pairs had to survive pricing for the delivery to be
possible at all, and a single spurious refusal anywhere on the chain would have
produced `noRoute` at the source rather than a dearer alternative.

## What was measured

One complete run, launched as a normal cell: scene, shared lower server, four
AgentThings, then `./gradlew -Pjcm=generated/robots/Robot1_Robot2_Robot3_Robot4.jcm`
and one `!deliverCube` at Robot1.

| Quantity | Value |
|---|---|
| Route discovery | **≈ 3 s**, settling on a complete tally |
| Routes discovered | **1** |
| Route cost | **2.29 m** |
| Chain execution | **≈ 41 s** of robot movement |
| Manual interventions | 0 |

The route cost is the sum defined in the thesis's route-cost equation and
computed by `pathCost`: the origin's own leg onto its first hop, plus, at every
step, the leg paid by the arm being handed *to*. A leg is the Euclidean distance
from that arm's cost origin — its current tool control point — to the
cheapest-ranked rendezvous candidate of the pair, in metres. Four legs are
summed for a three-hop route.

**These three numbers are a stopwatch on one run, not an instrumented series.**
E11, E12 and E13 each record their intervals from an instrumented
`routePlans.asl` into a JSONL trace against a monotonic clock inside the JaCaMo
JVM, over five or ten runs, and report a mean with a sample standard deviation.
E15 has no recorder: the convergence time is read from the Gradle console, and
the 41 s is the movement of the chain as observed in the simulator. They should
be read as an order of magnitude and as evidence that the cell runs, not as
comparable measurements. The three seconds is, however, consistent with E12's
instrumented $3.60 \pm 0.46$ s on a larger cell with two routes to price, which
is the direction one would expect.

Everything above the table is a different kind of evidence and is stronger:
voxel counts, voxel sizes, spacings, reaches, sample counts and the pair
verdicts are read directly out of the artifacts the run generated, and can be
re-derived from the repository without re-running the cell.

## Reproducing it

CoppeliaSim must be running with its websocket remote API on `localhost:23050`.
From the repository root:

```bash
# Robot4 is the Dobot, restored from the archive
npm run archive:robot -- --name Robot4 --as Robot4_CR3   # keep the CR3 first
rsync -a archive/Robot4_Dobot/files/ ./

npm run robot:pipeline -- \
  --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json,generated/robots/Robot3/manifest.json,generated/robots/Robot4/manifest.json \
  --generate-shared-cell \
  --offsets      '0,0,0;1.5,0.2,0;2.1,-1,0;1.5,-2,0' \
  --orientations '0,0,-15;0,0,180;0,-90,180;0,0,0' \
  --wall-offsets '0,0,0;0,0,0;-0.42876,-0.25726,0;0.9,0.5,0' \
  --sites 'Robot1_grip_support@0.75,0.4,0.08,0,Robot1_grip_cube;Robot4_grip_support_b@1.5,-2.2,0.08'
```

No `--resample` flag: every arm is resampled, because the walls move for all
four and workspace sampling culls the poses they block. Then:

```bash
npm run robot:serve -- --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json,generated/robots/Robot3/manifest.json,generated/robots/Robot4/manifest.json
npm run robot:agent -- --manifest generated/robots/Robot1/manifest.json   # and 2, 3, 4
./gradlew -Pjcm=generated/robots/Robot1_Robot2_Robot3_Robot4.jcm
```

To go back to the five-robot cell of E11–E14: `rsync -a archive/Robot4_CR3/files/ ./`,
which restores the CR3, the five-robot descriptor and scenes, and the
`Robot2__Robot4` and `Robot3__Robot4` interaction spaces that this cell
overwrote.

### Two things that will stop a fresh run

**`sourceAlias`.** The Dobot was archived on 2026-09-07 and
`runtime.gripper.source` / `sourceAlias` was introduced two days later. The
archived manifest uses the older spelling `shipsWithModel: true`, which still
resolves to `builtin`, but generation then refuses it for having no
`sourceAlias`. The value is `Robot4_tcp_gripper` — the alias adoption renames the
subtree to, not `gripper_rod`, which is only its name in the source scene.

**`minimumSharedVoxels`.** The floor is carried forward from the existing cell
descriptor and defaults to 20 when none is found. This cell inherits 10 from the
archived descriptor. Deleting `Robot1_Robot2_Robot3_Robot4.shared.json` before
regenerating would silently raise the floor — harmless here, since the one edge
that matters has 25 voxels, but worth knowing before it is not.

## What this does not establish

The thesis's original design for this experiment was ten seeded random
displacements of the five-robot cell, ten runs each, testing whether a selected
route follows the geometry when the geometry changes. This is not that
experiment, and it does not replace it: one hand-built cell with one possible
route cannot show that a *choice* tracks displacement, because there is no
choice in it.

What it does establish is the part the ten-layout design takes for granted —
that the chain runs end to end on a cell it has never seen, including one built
around an arm that was acquired differently from every other arm in the
repository, with no value re-tuned and no plan file changed. Whether the route
*decision* follows displacement is still open, and E12's margin of 0.187 m
between two routes is the number that experiment would have to resolve against.

The 41 s and the 3 s are one run each. A distribution would need the E12
recorder pointed at a four-robot cell, which is a small change to
`e12Principal.js` and was not made.
