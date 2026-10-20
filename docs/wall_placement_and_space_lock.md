# Wall placement and the initial space lock

The scene template puts one demo wall obstacle (`80cmHighWall50cm`) a fixed
vector from the template robot's base. Every robot added to a shared cell gets a
clone of it at the same vector in its own frame, so an arm mounted like the
template one is boxed in the same way. That is the intent: the wall is what the
collision check, the joint preflight and the OMPL detour are demonstrated
against.

It stops being harmless as soon as a robot has more than one neighbour.

## What went wrong for Robot3

In the four-robot cell, Robot3 (CR5, base at `2.1, -1.0`, yaw 180 degrees) sat
with its wall 0.195 m from its base, spanning world bearings 13 to 67 degrees.
Its neighbours lie at bearings -37 (Robot4), 117 (Robot2) and 155 (Robot1)
degrees, so the wall stood inside the arc between the Robot4 side of the cell
and the Robot1/Robot2 side.

A wall that close does not merely block the direction it stands in. It blocks
every base angle whose arm SWEEP crosses it, and for a wall 20 cm from the base
that is most of the circle. Sweeping Robot3's base joint through its full range
in six representative postures:

```
before:  ...################################################.....................
after:   ........................................................................
         j1 = -180 ................................................... +175
```

Only 120 degrees of base rotation were collision-free, and the free sector was
the one facing Robot4. The headings for Robot1 (`j1 = -1`) and Robot2
(`j1 = -39`) were both unreachable through any base rotation. Robot3 could not
turn from one partner to the other -- it was locked into whichever half of the
cell it started in.

The interaction spaces showed the same thing from the other side: 2 shared
voxels with Robot1 and 30 with Robot2, against 392 with Robot4.

## The fix

`--wall-offsets` displaces one robot's wall in that robot's own (yaw-rotated)
frame, on top of the placement inherited from the template. Robot3's wall was
pushed 0.5 m further out along the vector it already stood on:

```bash
npm run robot:pipeline -- \
  --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json,generated/robots/Robot3/manifest.json,generated/robots/Robot4/manifest.json \
  --generate-shared-cell \
  --offsets "0,0;1.5,0.2;2.1,-1.0;2.9,-1.6" \
  --orientations "0,0,0;0,0,180;0,-90,180;0,0,180" \
  --wall-offsets ";;-0.42876,-0.25726;" \
  --resample Robot3
```

Entries are positional and empty ones mean "no displacement", so only the third
robot moves. The wall center went from 0.437 m to 0.937 m from Robot3's base --
still inside the CR5's roughly 1.08 m reach, so it remains a real obstacle for
the planner, but far enough out that no base angle is blocked any more.

0.4 m is the knee: below it some base angles stay blocked, above it nothing
changes. 0.5 m was chosen for margin.

### Why only Robot3 was resampled

`--resample` limits which robots get a new workspace in the shared environment.
Two reasons it matters here:

- The workspace driver never seeds `math.random`, so sampling is not
  reproducible. Resampling a robot whose surroundings did not change still
  moves its reachable set and invalidates a calibration tuned against the old
  one.
- Robot3's wall, in both its old and new positions, lies outside every other
  robot's workspace bounding box, so no other workspace could have changed.

Robot1, Robot2 and Robot4 kept their CSVs byte for byte.

## Result

| | before | after |
|---|---|---|
| Robot3 reachable samples | 25342 | 39381 |
| Robot3 collision samples | -- | 531 |
| free base rotation | 120 deg | 360 deg |
| `Robot1__Robot3` voxels | 2 | 3 |
| `Robot2__Robot3` voxels | 30 | 132 |
| `Robot3__Robot4` voxels | 392 | 385 |
| `Robot1__Robot2` voxels | 343 | 343 (untouched) |

Commanded through the running cell, Robot3 now reaches all three neighbour
headings; before, two of them were geometrically unreachable.

## Two things this did not fix

- **`Robot1__Robot3` is still a degenerate pair at 3 voxels.** Robot1 and
  Robot3 stand 2.3 m apart, which is barely inside their combined reach, so the
  bounding-box discovery registers a handover pair that has no usable volume.
  It was 2 voxels before the wall moved; the wall was never the reason. Either
  place the two closer or configure the topology explicitly with
  `--interaction-pairs "Robot1:Robot2;Robot2:Robot3;Robot3:Robot4"`.
- **Robot3 reports `joint_target_not_reached` on every joint move.** Its
  shoulder settles 0.7 to 1.1 degrees below the commanded angle, scaling with
  how far the arm is extended, against a `pathTrackingToleranceDeg` of 0.5. It
  is identical at every base angle, including angles that were never blocked,
  so it is a steady-state sag under gravity rather than anything to do with an
  obstacle. The arm does arrive -- the tolerance is simply tighter than the
  CR5's static droop.
