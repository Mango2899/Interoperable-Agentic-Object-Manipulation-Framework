# Three-robot interaction chain

This setup uses three different fixed arms:

- Robot1: UR10
- Robot2: UR3
- Robot3: CR5

The configured topology contains exactly `Robot1__Robot2` and
`Robot2__Robot3`. Robot1 owns the first calculation and Robot2 owns the second.
Robot2 therefore never recalculates the Robot1-Robot2 space.

## 1. Generate Robot3

Run from the WoMAT repository root:

```bash
npm run generate:robot -- \
  --urdf external/RobWoT/Assets/URDFs/cr_description/urdf/cr5_robot.urdf \
  --name Robot3 \
  --platform fixed \
  --thing-port 8130 \
  --agent-port 8131 \
  --websocket-port 8231 \
  --home-position 0,0,0.30 \
  --horizontal-position 0.30,0,0.35 \
  --grip-position 0.45,0.20,0.19 \
  --carry-position 0.40,0.20,0.35 \
  --home-joints 0,0,0,0,0,0 \
  --gripper-orientation-deg "-90,0,0" \
  --gripper-local-rotation-deg "0,0,180" \
  --no-gripper-base-orientation-correction \
  --gripper-jaw-center-offset 0,-0.18,-0.005
```

The explicit TCP-local 180-degree Z rotation turns the RG2 outward on the CR5.
The base-orientation correction is disabled because the gripper already inherits
the CR5's corrected transform through the TCP hierarchy; applying it again would
double-rotate the tool. The jaw-center offset is mirrored accordingly so
handover target calculation continues to refer to the physical center between
the jaws.

This also adds the `Robot3` port entry to `config.json`. The generated manifest
contains `80cmHighWall50cm` in both workspace and runtime collision-obstacle
lists, so the normal pipeline creates a Robot3 wall automatically. No wall has
to be inserted manually in CoppeliaSim.

## 2. Generate the CR5 model and workspace

Start CoppeliaSim with its websocket remote API on port 23050, then run:

```bash
npm run robot:pipeline -- \
  --manifest generated/robots/Robot3/manifest.json \
  --full-pipeline
```

For thesis-quality workspace sampling, set `pipeline.workspaceRandomSteps` in
`generated/robots/Robot3/manifest.json` to `80000` before this command. The
generator default of `8000` is faster for an initial integration test.

## 3. Assemble the chain

The following placement preserves the existing Robot1-Robot2 calibration and
places the CR5 0.85 m beyond Robot2:

```bash
npm run robot:pipeline -- \
  --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json,generated/robots/Robot3/manifest.json \
  --generate-shared-cell \
  --offsets "0,0;1.25,0.20;2.10,0.20" \
  --orientations "0,0,0;0,0,180;0,-90,180" \
  --interaction-pairs "Robot1:Robot2;Robot2:Robot3"
```

The CR5 URDF/model frame needs the third entry's `-90` degree pitch to stand
upright. The final `180` degree value is its floor-plane yaw toward Robot2.
Only the robot and its workspace use the pitch correction; Robot3's cloned wall
uses the yaw and therefore remains upright.

The explicit pair list prevents an accidental Robot1-Robot3 interaction even
if their coarse workspace bounding boxes touch. If `Robot2__Robot3` reports
`no_geometric_overlap`, move Robot3 toward Robot2 in 0.05 m increments, for
example from `2.10,0.20` to `2.05,0.20`, and regenerate the shared cell. Do not
move Robot1 or Robot2, because that invalidates the existing handover
calibration.

The pipeline produces:

```text
generated/robots/Robot1_Robot2_Robot3.shared.json
generated/robots/Robot1_Robot2_Robot3_shared.ttt
generated/robots/Robot1_Robot2_Robot3.jcm
```

The scene contains one cloned wall for each robot. Its descriptor should show:

```text
Robot1__Robot2 owner=Robot1
Robot2__Robot3 owner=Robot2
```

## 4. Start the three-robot system

Terminal 1, shared lower runtime:

```bash
npm run robot:serve -- \
  --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json,generated/robots/Robot3/manifest.json
```

Terminals 2-4, one AgentThing per robot:

```bash
npm run robot:agent -- --manifest generated/robots/Robot1/manifest.json
```

```bash
npm run robot:agent -- --manifest generated/robots/Robot2/manifest.json
```

```bash
npm run robot:agent -- --manifest generated/robots/Robot3/manifest.json
```

Terminal 5, combined JaCaMo application:

```bash
./gradlew -Pjcm=generated/robots/Robot1_Robot2_Robot3.jcm
```

Robot1 and Robot2 calculate their owned spaces during agent initialization.
Expected AgentThing messages include:

```text
[AgentThing:Robot1] interaction Robot1__Robot2 ready ... cached=false
[AgentThing:Robot2] interaction Robot2__Robot3 ready ... cached=false
```

There must be no Robot2 calculation message for `Robot1__Robot2`.

## 5. Run the complete BDI handover chain

Trigger only Robot1's agent-facing action:

```bash
curl -X POST http://localhost:8111/robot1_agent/actions/handoverCube \
  -H 'content-type: application/json' \
  -d '{"targetAgent":"robot2_agent","targetRobot":"robot2"}'
```

The HTTP action only creates Robot1's initial achievement goal. The interaction
capability publishes a bounded central core from `Robot1__Robot2` as candidate
beliefs. Robot1 evaluates centrality, local density, and reachability support
for both robots, selects the maximum-scoring belief, and plans both approaches.
After Robot2 receives the cube, it backs off and tells Robot1. Robot1 then
retreats and immediately sends `giverRetreated`, allowing Robot2 to continue
without waiting for Robot1's home pose or a complete home-path search. A failed
retreat does not send that acknowledgement.

The giver follows one continuous, collision-checked home trajectory with smooth
timing. It reserves the swept rooms before starting, then atomically shrinks
that reservation while moving. A room is released only after both the measured
whole-arm footprint and every remaining path sample are clear of it. Dynamic
joint lag and paths that re-enter a room therefore cannot cause early release.
Only the final home configuration requires joint settling, avoiding repeated
five-degree stops and failures from gravity-induced error at intermediate poses.
A blocked direct home retains the established full-home planner as recovery,
with all affected regions reserved. This recovery can still serialize motion.

The retreat message permits a receiver to request movement; it does not bypass
the room lock. With whole-room reservations, simultaneous movement is possible
only while their claims are disjoint. A short tool retreat alone need not clear
the giver's elbow or its remaining home path from the shared room.

Smaller-zone mode is enabled in the five generated manifests:

```json
"handoverZone": { "enabled": true, "radius": 0.12 }
```

This setting belongs under `runtime`; radius is in metres. Both partners must
opt in. The giver passes the agent-selected `cubeWorld` point to its lower core,
which replicates the world-coordinate zone and the larger of the two configured
radii to the receiver. The zone follows the selected exchange point, not the
geometric center of the entire shared volume. Missing or invalid zone geometry
retains full-region claims. Set `enabled` to `false` to restore whole-region
handover locking.

Approaches, retreats, giver-home paths, and the receiver's first departure use
this mode. The first departure retains its incoming zone even after the agent
closes that session; a direct carrier can also have its outgoing zone during
that motion. Once the departure succeeds, the incoming zone is discarded.
Ordinary motions continue to use the normal region policy.

The small zone is exclusive whenever the arm's footprint touches it. Outside
it, the core reserves the checked whole-arm swept voxels, using the original
region's keys so these reservations still conflict with normal full-region
claims. Path claims include a 3 cm tracking allowance in addition to the usual
body radius and voxel dilation. Reservations shrink only when both the measured
arm and the remaining path clear the voxels. Leaving the reserved envelope
stops the motion. Overlapping exterior paths still serialize.

In simple Cartesian mode the existing incremental IK route is recorded before
movement, then those exact joint samples are checked and replayed. This avoids
running an unrecorded IK or Ruckig detour under a small claim. Robot2 retains its
receive → home → next-handover policy; Robot4 retains direct carrying. Pickup,
placement, unavailable path geometry, and recovery requiring an unknown route
use full-region reservations. A planning or preflight refusal before movement
can therefore fall back to the established execution path.

Route preparation restores the live posture between short IK calls (at most
eight solves per call, yielding earlier after 20 ms of CPU time). Geometry
sweeps default to 128 samples per call. Collision preflight uses a 50 ms CPU
budget per slice and resumes over separate simulator calls. This keeps the
simulator responsive; it does not bound total preparation to 50 ms. Hundreds
of samples can still take seconds of mesh checks and round trips before
playback starts. Contact and self-collision checks
remain on every sample; mesh clearance runs at the configured stride and at
the actual final goal, rather than treating every batch boundary as a goal.
Spatial lookup caches occupied voxel columns and repeated cells without
changing the claimed volume. These bounds reduce preparation stalls that can
otherwise delay the simulator and lock-release polling. Lua driver changes
take effect when the lower-core server is restarted.

The lower log reports `using handover zones with swept-path reservations`,
`Handover zone cleared`, or a reason for falling back to full-region locking.
`WOMAT_MOTION_LOG` adds `handover_zone_released` events and claim counts. Actual
simultaneous movement still depends on disjoint paths; shrinking the center
zone does not allow two arms to occupy the same exterior voxels.
For these paths, `trajectory_started` and `trajectory_finished` mark simulator
playback separately from lock acquisition. The start also reports geometry and
collision-preflight times, helping distinguish preparation stalls from room
waits. Overlapping reservation intervals alone do not prove overlapping motion.
The runtime announces when preflight starts and reports `claimWaitMs`,
`preflightRpcMs`, `preflightMaxCallMs`, and `preflightCpuMs`. RPC elapsed time
includes simulator queuing; CPU time is measured within the Lua slices and is
not a wall-clock measure. The simulation timer can advance between slices
while both arms wait for preflight completion.

Live reservation checks use capsules around the full joint-origin polyline,
with the configured body radius, rather than dilating its containing grid
cells again. This avoids false escapes at voxel boundaries. The original
conservative path reservation remains padded, and released keys must also be
outside the padded measured arm. A real escape still stops playback before
releasing the claim. Its diagnostic includes the path index, escaped voxel
keys, and how many of those keys were previously released.


Repeated lower-action invocations in older AgentThing logs can be permission
retries of one queued command. A `region_claimed` or `motion_busy` refusal occurs
before that attempt executes movement. The bridge now logs the invocation once,
reports the blocking robot and region when they change, and summarizes refused
attempts when the request finishes. The retry interval remains 250 ms; this
logging change does not bypass locks or duplicate a successful movement.

Set `runtime.returnHomeBetweenHandovers` in a robot's manifest:

- `true`: receive the cube, return to home joints while holding it, then approach
  the next handover. Motion safety is unchanged: admission, preflight, the
  Cartesian fallback and the watchdog all stay enabled. **Enabled for Robot2.**
- `false` or omitted: carriers without a reachable support continue directly;
  support-serving robots retain their upright reset.

The policy is read by the AgentThing bridge, so no agent regeneration is needed.
Short handover retreats are retained in both modes. `WOMAT_MOTION_LOG` records
`giverHome` intervals and `home_rooms_released` events for checking lock timing.
Use the motion observer to distinguish physical overlap from overlapping action
intervals, which also include preflight time.

Each core reloads shared regions before motion, including canonical pair files
exported by agents after startup and absent from the descriptor's pair list.
Restart the lower cores, AgentThings and JaCaMo to load these code changes;
robot regeneration is unnecessary. Use region arbitration (the default), since
`--exclusive-motion` deliberately serializes the cell.

Robot2 independently performs the same central-core evaluation for
`Robot2__Robot3` and plans the Robot2 and Robot3 approaches. Robot3 is the
terminal receiver and returns the cube to the shared support after the chain.

Expected BDI-stage messages include:

```text
Robot1: BDI selected central handover candidate ... score=...
Robot2: Continuing directly to the next handover with the cube
Robot2: Adopting downstream handover intention to robot3
Robot2: BDI selected central handover candidate ... score=...
Robot3: Terminal receiver returning cube to support
```

## 6. Verify caching and visualize both spaces

The pair-specific results are stored under:

```text
generated/robots/interactions/Robot1__Robot2/space.json
generated/robots/interactions/Robot1__Robot2/space.csv
generated/robots/interactions/Robot2__Robot3/space.json
generated/robots/interactions/Robot2__Robot3/space.csv
```

Restart the AgentThings and JaCaMo without regenerating a workspace. The same
messages should now end in `cached=true`, demonstrating that neither pair was
revoxelized.

To add both pair clouds to the saved shared scene, stop the runtime simulation
and run:

```bash
npm run robot:pipeline -- \
  --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json,generated/robots/Robot3/manifest.json \
  --update-shared-space-visual
```

The Robot1-Robot2 point cloud is green and Robot2-Robot3 is cyan.

Run the non-Coppelia regression suite with:

```bash
npm test
```
