# Plan: per-agent movement/safety cores and environment-aware BDI agents

Two goals, one architecture change:

1. **Every robot agent knows its environment as beliefs** — derived from its
   reachability point cloud (`*_data_point.csv`), the scene obstacle set, and
   the pairwise interaction spaces.
2. **Movement and safety cores run per agent**, so two robots can move at the
   same time.

They are the same change because concurrent motion is only safe if somebody
reasons about *space* instead of taking turns in *time* — and the thing that
should do that reasoning is the agent, using beliefs it does not currently have.

---

## 1. Where we are

```
CoppeliaSim  (one scene, one wsRemoteApi on :23050)
      ^
      | ONE websocket, ONE process
lowerThingServer.js         5510 lines, serveMultipleRobots()
      - per robot: own Lua script handle, own WoT HTTP server
      - SHARED: sim connection, motionCoordinator mutex, gripCoordinator
      ^
      | HTTP (WoT)
agentThingServer.js         one process per robot   <-- already split
      - InteractionSpaceController, belief publication
      ^
      | websocket -> wotCommunicator CArtAgO artifact
JaCaMo MAS                  robot1_agent / robot2_agent / robot3_agent .asl
```

### 1.1 What actually blocks concurrency

`serveMultipleRobots` builds one `motionCoordinator` (`lowerThingServer.js:3634`)
and every motion handler goes through `runExclusiveMotion`
(`lowerThingServer.js:5262`), which rejects with `reason: "motion_busy"` while
any other robot is moving.

The mutex is **not arbitrary**. The comment above it states the real constraint:
each robot's planner has the other robots in its obstacle collection, so a path
accepted by preflight becomes invalid the moment another arm moves. Deleting the
mutex without replacing that guarantee just converts a serialization into a race.

So the mutex has to be replaced, not removed: a **time lock becomes a space
lock**.

### 1.2 What is already good and must be kept

- The Lua in-motion watchdog (`womatInMotionUnsafe`, `lowerThingServer.js:900`)
  runs **per robot, every simulation step**, and already has the other robots in
  its obstacle collection. Physical safety under concurrent motion is already
  enforced; what is missing is *liveness* and *planning validity*.
- `womatBlockingObstacleAlias` / `womatObstacleOwnerAlias` already name which
  object blocked and which robot owns it, and `publishMotionFailure`
  (`agentThingServer.js:516`) already turns that into a `motionBlockedBy(Obj,
  Owner)` belief. The feedback channel for concurrency exists.
- `WorkspaceGrid` (`workspaceGrid.js`) is a client-side voxel hash over the CSV —
  admission is already O(1) and needs no round trip.
- Pairwise interaction spaces (`generated/robots/interactions/<pair>/space.json`,
  358 shared voxels for `Robot1__Robot2`) are exactly the geometry a space lock
  needs, already computed and cached with placement-hash invalidation.

### 1.3 What the agents currently know

Beliefs in `src/agt/generated/robot1_agent.asl`: `homePosition`,
`horizontalDemoPosition`, `homeJoints`, `gripTargetPosition`, `pickupHoverHeight`,
`carryPosition`, `agentIdentity`, `handoverHeightRange`, plus generated pair
topology.

Nothing about reachability, bounds, obstacles or shared space. The 35 131-point
cloud is consumed **only** inside the lower server. The agent asks for a move it
cannot evaluate, waits up to 60 s, and on failure sets `+myStatus(broken)` —
which has **no recovery plan anywhere** (`robot{1,2,3}_agent.asl`,
`handoverPlans.asl:150,175`). Under concurrency, transient blocks become normal,
so that alone would wedge the cell.

---

## 2. Target architecture

```
CoppeliaSim
   ^          ^          ^
   |          |          |            one RemoteAPIClient each
robotCore(R1) robotCore(R2) robotCore(R3)      <-- NEW: one process per robot
   |  motion + safety + claims           WoT peer mesh for claims & transfers
   ^          ^          ^
agentThing(R1) agentThing(R2) agentThing(R3)   <-- unchanged shape, richer beliefs
   ^          ^          ^
robot1_agent  robot2_agent  robot3_agent       <-- reasons over its environment

cellHost                                        <-- NEW: loads scene, spawns robots,
                                                    starts sim, forks the cores
```

Three separations:

| concern | today | target |
|---|---|---|
| scene lifecycle | inside `serveMultipleRobots` | `cellHost` only |
| motion + safety | one process, N robots, one mutex | one process per robot |
| cube ownership | `gripCoordinator` JS object | derived from the scene, peer transfer |
| conflict avoidance | global time mutex | per-pair voxel claims + agent reasoning |

---

## 3. The space lock

### 3.1 Claim computation

Every motion already produces a checked path before it executes:
`womatPreflightJointPath` (`:1179`) and `womatSweepCartesianPath` (`:1162`)
sweep the path in one batched Lua call.

Extend those sweeps to also return, per sample, the world positions of a small
set of arm frames (base, elbow, wrist, tip — resolvable from `jointHandles` and
`womatResolveFlange`). Voxelize those polylines into the **pair's shared grid**
(`origin`, `voxelSize` from `space.json`), dilate by one voxel, and the result is
the motion's **claim set**: the shared voxels this trajectory can occupy.

Cost: one extra array in a call that already happens. No new round trips.

### 3.2 Arbitration

- If the claim set is **empty** for every pair the robot participates in, execute
  immediately with no coordination. This is the common case (homing, own-side
  picks) and is where the throughput win comes from.
- Otherwise, request the claim from each affected pair's **owner**, which is
  already a deterministic belief (`interactionSpaceOwner(Robot1, Robot2)`), so no
  global coordinator is introduced — consistent with the design note at the top
  of `handoverPlans.asl`.
- Requests are issued in **lexicographic pair-key order**. Fixed resource
  ordering; no deadlock cycle is possible.
- The owner grants if the requested voxels are disjoint from live claims. Grants
  carry a **lease with a timeout**, released on motion completion, on watchdog
  abort, or on lease expiry.

New WoT affordances on the lower TD: `claimRegion`, `releaseRegion`,
`getRegionClaims`.

### 3.3 Abort semantics

Watchdog trip must stop being terminal. On trip: halt the coroutine, release all
claims, report `motionAborted(Reason, Obstacle, Owner)`. The agent gets a
recovery plan (§5.3) instead of `+myStatus(broken)`.

### 3.4 Coverage gap — closed

**Closed in Phase 3.** `--all-pairs` now scans every combination rather than the
configured handover pairs, and Robot1/Robot3 turns out to share exactly **one
voxel** at 0.108 m. Too small to hand over in, and exactly the kind of fact a
space lock must not assume away. Recorded in the descriptor as
`discovery: "pair_scan"`, `handoverPair: false`.

---

## 4. Splitting the cores

### 4.1 Module decomposition (no behaviour change)

`lowerThingServer.js` is 5510 lines, of which ~2900 are a Lua program inside a
template literal. Split into:

```
src/generator/core/
  cellHost.js          scene load, robot spawn, sim start, fork cores
  robotCore.js         per-robot process entry (connect, attach, serve)
  motion.js            planners, cartesian/joint moves, follow
  safety.js            admission, preflight, watchdog config, claims
  gripper.js           grip state, open/close, attach/detach
  handover.js          session, approach lock, transfer
  thing.js             WoT wiring (TD handlers)
  arbiter.js           claim table, leases, peer client
src/generator/lua/
  *.lua                the driver split into named chunks, concatenated
                       by prepareRuntimeDriverScript
```

Moving the Lua out of the template literal is worth doing on its own: it gets
syntax highlighting, a linter, and diffable history.

> Note: `docs/python_migration_plan.md` proposes porting this engine to Python.
> These are compatible but must not run concurrently. The module boundaries here
> are the prerequisite for that port either way — do this first, decide on Python
> after.

### 4.2 Removing the last shared JS state

`gripCoordinator` holds `owner`, `activeTransfer` and the robot map. Replace with:

- **Ownership derived from the scene.** Each core already has `getGripperState()`
  (`:1694`); the owner is the robot whose gripper reports holding the cube.
  `getCubeOwnership` becomes a scene query rather than a variable read, which is
  also more truthful when a grip slips.
- **Peer-to-peer transfer.** `transferCube` becomes: giver core calls the
  receiver core's `acceptTransfer` over WoT, then relinquishes. The Lua halves
  (`acceptTransferredGripObject:1790`, `relinquishTransferredGripObject:1849`)
  already exist. `agentThingServer` already keeps a `partnerLowerThings` map —
  the same pattern applies one layer down.

### 4.3 Process supervision

`npm run robot:serve -- --manifests a,b,c` keeps its current UX: it becomes
`cellHost`, which loads the shared scene, spawns robots, starts the simulation,
then forks one `robotCore` child per manifest and supervises them (restart on
crash, propagate SIGINT, aggregate logs with the existing `[LowerThing:<name>]`
prefix). `--single` runs one core directly for debugging.

`cleanupRobotPorts.js` needs to learn about the child PIDs.

---

## 5. Environment as beliefs

Three tiers. The agent believes a **conservative approximation** and reasons
locally; the safety core keeps the **exact** grid and remains the authority.
That split is deliberate: an agent that has to make a network call to know
whether a point is reachable is not reasoning, it is polling.

### 5.1 Static, generated into the `.asl`

A new unit-testable `src/generator/workspaceSummary.js` reduces the CSV to a
compact shell model — arm workspaces are approximately annular, so per-z-slab
radial bounds in base frame describe the cloud in ~12 facts instead of 35 131:

```prolog
workspaceVoxelSize(0.0488).
workspaceSampleCount(35131).
workspaceBounds(MinX, MinY, MinZ, MaxX, MaxY, MaxZ).
workspaceTolerance(0.146).            // spacingFactor * voxelSize
reachableShell(Index, ZLow, ZHigh, RMin, RMax, Fill).
```

with rules the agent can use directly:

```prolog
baseRadius(X, Y, R)  :- R = math.sqrt(X*X + Y*Y).
inWorkspace(X, Y, Z) :- baseRadius(X, Y, R) & reachableShell(_, ZLo, ZHi, RMin, RMax, _)
                        & Z >= ZLo & Z < ZHi & R >= RMin & R <= RMax.
nearWorkspaceEdge(X, Y, Z) :- baseRadius(X, Y, R) & reachableShell(_, ZLo, ZHi, _, RMax, _)
                        & Z >= ZLo & Z < ZHi & R > RMax - 0.1.
```

The shell model **over-approximates** (it cannot see interior voids or the
collision-state column). That is the correct direction: the agent may believe a
point reachable that the core then rejects, never the reverse.

`Collision_state` in the CSV is currently ignored by `WorkspaceGrid`; the shell
builder should emit `Fill` from collision-free samples only, and it is worth
checking whether the grid should filter on it too.

### 5.2 Semi-static, pushed by the AgentThing at init

```prolog
obstacleBox(Alias, Owner, MinX, MinY, MinZ, MaxX, MaxY, MaxZ).
sharedRegion(Neighbor, SpaceId, VoxelCount, MinX, MinY, MinZ, MaxX, MaxY, MaxZ, CX, CY, CZ).

inSharedRegion(X, Y, Z, N) :- sharedRegion(N, _, _, MinX, MinY, MinZ, MaxX, MaxY, MaxZ, _, _, _)
                              & X >= MinX & X <= MaxX & Y >= MinY & Y <= MaxY
                              & Z >= MinZ & Z <= MaxZ.
```

Sources already exist: `getCollisionObstacles` / `womatObstacleLocalBoxes`
(`:986`) and the pair `space.json`. This extends the existing
`interactionSpace(SpaceId, Target, VoxelSize, Ox, Oy, Oz, Count)` belief rather
than replacing it.

### 5.3 Dynamic, pushed on change

```prolog
regionClaimed(Neighbor, ClaimId, Holder).
neighborStatus(Neighbor, moving | idle | holding).
motionAborted(Reason, Obstacle, Owner).
```

New plans in `handoverPlans.asl` / the generated agents:

```prolog
// fail fast and locally, instead of a 60 s round trip to learn it
+!moveTo(X, Y, Z) : not inWorkspace(X, Y, Z) <-
    .print("Target outside my believed workspace: ", X, Y, Z);
    +unreachableTarget(X, Y, Z);
    .fail.

// coordinate only where coordination is actually needed
+!moveTo(X, Y, Z) : inSharedRegion(X, Y, Z, N) & interactionRobot(NAgent, N) <-
    .send(NAgent, tell, enteringSharedRegion(X, Y, Z));
    !requestMotion(X, Y, Z);
    .send(NAgent, tell, leftSharedRegion).

// replaces the terminal +myStatus(broken)
+!recoverFromAbort(Reason) : motionBlockedBy(_, Owner) <-
    .print("Yielding to ", Owner, " and retrying");
    .wait(neighborStatus(Owner, idle), 30000);
    !retryLastMotion.
```

The point is that `broken` stops being a state and becomes a decision.

---

## 6. Phasing

Each phase leaves the system runnable and `npm test` green.

| # | Phase | Deliverable | Rough size |
|---|---|---|---|
| 0 | Baseline | **DONE.** Per-motion instrumentation and a report that measures concurrent motion time. See §15.1. | S |
| 1 | Decompose | **DONE.** `lowerThingServer.js` → `core/*.js` + `lua/*.lua`. Driver output byte-identical. See §10. | L |
| 2 | Split processes | **DONE.** `cellHost` + `robotCore` behind `--isolated-cores`, one sim client each, mutex kept via the arbiter. See §11. | M |
| 3 | Unshare state | **DONE.** Scene-derived cube ownership; per-robot handover registries; reusable, order-independent interaction core. See §9. | M |
| 4 | Space lock | **DONE.** Conservative region claims, `RegionArbiter` with leases, `--exclusive-motion` baseline. **Three arms measured moving at once.** See §15.3. | L |
| 5 | Beliefs | `workspaceSummary.js`, generator emission, obstacle/shared-region/claim beliefs, Jason rules. | M |
| 6 | Agentic plans | Pre-move guards, region-aware coordination, `!recoverFromAbort`, negotiation instead of blind retry. Retire `+myStatus(broken)`. | M |
| 7 | Validate | Sim runs, measurements, docs. | M |

---

## 7. Validation

**Unit** (`node --test`, no simulator):
- shell model: over-approximation is provable — every collision-free CSV sample
  must satisfy `inWorkspace`; assert the false-positive rate against the grid.
- claim voxelization: a known polyline maps to a known voxel set.
- arbiter: grant, deny on overlap, lease expiry, and a two-robot cyclic request
  that must not deadlock under the fixed pair ordering.

**Integration** (fake cores, no CoppeliaSim): two cores, overlapping and
disjoint claims, crash-during-lease.

**Simulation**:
- *Concurrency*: Robot1 picks while Robot3 homes. Both arms move in the same
  window; wall clock materially below the Phase-0 serial baseline.
- *Contention*: both robots target the same shared voxel. One proceeds, the other
  waits and then succeeds — no abort, no `broken`.
- *Regression*: the existing R1→R2→R3 handover chain completes unchanged.
- *Belief check*: each agent logs its shells, obstacles and shared regions at
  init, and refuses an out-of-workspace target without contacting the core.

---

## 8. Risks

- **wsRemoteApi is not parallel.** N clients still serialize at the simulator's
  API boundary. The win comes from motion *execution* overlapping in
  `sysCall_actuation` coroutines, not from parallel API calls. With N cores,
  `waitForMotionComplete` polling load multiplies — replace the 50 ms poll with a
  signal, as `python_migration_plan.md` already recommends (~20 lines, and it
  removes up to 50 ms of slop per motion from handover choreography).
- **Concurrency makes the watchdog fire routinely.** Phase 6 (recovery plans) is
  not optional polish; without it Phase 4 makes the cell *less* reliable.
- **Handover choreography implicitly assumes serialization** in places. The
  handover sequence must be re-read as an explicitly claimed region, not as
  something the mutex happened to protect.
- **Belief-base growth.** Keep the shell model to ~12 facts and the region model
  to one fact per neighbor. Do not push per-voxel beliefs; that is what the
  queryable core is for.
- **Scope.** Phases 1–3 are refactors with a testable definition of done. Phase 4
  is the only phase that changes what the system guarantees. If time is short,
  stopping after Phase 3 + Phase 5 still yields per-agent cores and
  environment-aware agents, with the mutex retained.

---

## 9. Phase 3, as built

### 9.1 The interaction core is now reusable by any agent

Three defects stopped every agent from using the voxelisation core, and all
three are fixed in `src/generator/interactionSpace.js`:

**The estimator depended on which robot asked.** Spacing and proximity were
sampled with derived seeds (`seed`, `seed+1`, `seed+2`, `seed+3`), so swapping
the two clouds changed the estimate:

```
before:  Robot1/Robot2  358 voxels @ 0.048792     Robot2/Robot1  360 voxels @ 0.048874
after:   identical voxel set and voxel size either way round
```

Every seed is now the same seed. A cloud's subsample depends only on that cloud,
so swapping the arguments swaps the outputs and changes nothing else. Two robots
cannot coordinate in a space they do not agree on. `diagnostics.orderIndependent`
records the one configuration where this does not hold (`bidirectional: false`,
which reads only the first cloud).

Robot2/Robot3 moved 39 → 42 voxels: the old 39 was an artifact of argument order.

**Ownership was enforced as a geometric rule.** `build()` refused any robot that
was not the configured pair owner, so Robot2 could not answer a question about
its own shared space with Robot1. Ownership is a *protocol* rule — who may open
a handover session — and it stays enforced in `beginHandoverSession`, where
handovers are actually opened. Any participant may now compute the pair; the
version hash, the estimator call and the exported files are all keyed on the
canonical pair rather than on the caller, so whoever computes it computes the
same space and the second one to ask hits the cache.

**Concurrent use was unsafe.** Two agents computing the same pair wrote the same
two files with `writeFileSync`, letting one land inside the other. Exports now
go through `writeFileAtomic` (temp file + rename), and `loadCached` rejects a CSV
whose row count disagrees with its own metadata rather than serving a cloud with
holes in it.

Plus a shared per-process file cache: one cloud, one KD-tree and one digest per
CSV however many controllers ask. Three agents in one process previously loaded
and re-sorted the same 35 131-point cloud once per pair, and re-hashed the same
1.5 MB file on every version check.

### 9.2 Cube ownership comes from the scene

`gripCoordinator` held four different kinds of thing in one object: an address
book, a cached cube owner, a transfer lock and the live handover session. Only
the first is configuration.

`src/generator/cellPeers.js` keeps the address book and nothing else that
changes as the cell runs. Every question that *does* change is answered by
asking the scene — a robot holds the cube when the cube is parented to its
gripper, and a cached boolean that says otherwise is simply wrong. `closeGripper`
and `openGripper` no longer record anything on success.

Two consequences worth having:

- Two arms reporting a hold is reported as `contested`, not resolved by
  precedence. It means the scene has the cube parented twice, or a transfer tore.
- An arm that does not answer is reported in `unreachable`. Answering "nobody
  holds it" while one arm never replied is how a second robot gets cleared to
  pick the same cube.

### 9.3 Handover sessions are per robot

Each core owns a `HandoverSessions` registry. The giver creates the record and
replicates it to the receiver (`peers.openSessionOn`, an in-process hand-off
today and a WoT action once the cores split); the receiver holds its own copy and
closes it on its own schedule. The `approachEnabledRobots` reference count is
gone — it existed only because one object was standing in for two robots'
bookkeeping.

The transfer lock moved from the cell to the giver. Two unrelated pairs handing
over at once were previously refused by a single shared flag for no reason either
of them could observe.

### 9.4 The seam for Phase 2

A peer entry is `{ invoke, manifest, sessions }`. `invoke(code)` runs a driver
call against that robot and resolves with its result — today through the shared
simulator connection, and after the process split through an HTTP call to that
robot's own lower Thing. Nothing else in `cellPeers.js` changes. That is the
whole point of the abstraction: Phase 2 replaces one function per peer.

### 9.5 Toward agent-discovered neighbours

The descriptor now separates two things that were conflated:

- `interactionSpace.pairs` — the **handover topology**, a decision.
- `sharedSpaces` — the **geometry**, one entry per robot combination, each
  tagged `discovery` (`configured` | `pair_scan`) and `handoverPair`.

`robwotBridge.js` already carries an AABB-overlap discovery path
(`discoverInteractionPairs`, `discovery: "workspace_aabb_overlap"`) that this
cell bypassed by configuring its pairs explicitly. With the core now symmetric
and safe to run from several agents at once, an agent asking "who are my
neighbours?" is a scan it can run itself: compute against each other robot, keep
the pairs whose shared voxel count clears a threshold. Robot1/Robot3's single
voxel is exactly the case that threshold has to answer, and it is now a recorded
number rather than an omission.

---

## 10. Phase 1, as built

### 10.1 The Lua left the template literal

`lowerThingServer.js` was 5,510 lines, of which ~2,900 were a Lua program inside
a JavaScript template literal. It is now ten files in `src/generator/lua`,
assembled by `core/luaDriver.js`:

```
01-base-frame.lua       85   robot-local to world, async move overrides
02-simple-motion.lua   186   Ruckig-smoothed pose and joint motion
03-scene-objects.lua   135   alias lookup, what counts as an obstacle
04-safety.lua          538   collections, watchdog, swept preflight
05-gripper.lua         653   grip objects, TCP gripper, cube transfer
06-path-execution.lua  297   actuation loop, path following
07-tool-frame.lua      334   tool control point and approach frames
08-pick.lua            309   top-down pick planning
09-handover.lua        328   pose lock, standoff, approach planning
10-planning.lua        189   OMPL goal search and joint-path planning
```

The split was verified by reassembling the chunks and asserting the result was
byte-identical to what was extracted, and the rewiring by rendering all three
robots' drivers in two configurations each and diffing against a baseline
captured before the change. **All six are byte-identical.**

The JS-expression substitutions became named placeholders (`${luaArray(toolAxis)}`
→ `${toolAxisLua}`), so a chunk contains only `${identifier}`. That let `render()`
refuse a script it cannot fully substitute: a missing value used to render the
string `undefined` into the middle of the scene script and surface much later as
an unrelated failure. It now names the placeholder and the file it is in, before
the script reaches CoppeliaSim. An unused value is refused too — that is what a
half-finished rename looks like.

### 10.2 The JavaScript became a directory

```
core/context.js          128   CLI, manifests, paths          (no simulator)
core/simConnection.js     38   connect, execute in a script
core/driverScript.js     149   render one robot's driver
core/scene.js            216   load, spawn, attach, visuals
core/runtimeOptions.js    99   manifest tuning, workspace grid
core/safety.js           246   admission, preflight, sweeps
core/motion.js           790   planners and moves
core/handover.js         349   approach, session, transfer
core/thing.js            322   WoT wiring
core/cellHost.js         281   scene owner and supervisor
core/motionArbiter.js    132   who may move
core/robotCore.js         89   one robot, one process
lowerThingServer.js       99   the command line
```

The dependency graph is acyclic and each module states what it imports, so
"which parts of the runtime does motion actually need" is now answerable by
reading nine lines instead of grepping five thousand.

`core/runtimeSource.js` exists for the tests that assert on how the runtime is
written rather than what it returns — that a sweep runs inside a single script
call, that the watchdog tests contact every step. Those properties are not
observable from a return value. It joins the modules and the driver back into
the one surface those assertions used to read.

---

## 11. Phase 2, as built

### 11.1 What belongs to the cell, and what belongs to an arm

`--isolated-cores` splits them:

- **Cell host**: loads the shared scene, spawns every arm, attaches the driver
  scripts, starts the simulation, holds the motion lock, forks the cores and
  supervises them. Nothing else — it does not proxy for the cores.
- **Robot core**: one process per arm, with its own CoppeliaSim connection, its
  own workspace grid and preflight, its own gripper and its own handover
  sessions.

Script handles are scene-global integers, so the host passes each core the
handle it already attached rather than making every core re-attach its own
driver into the same scene.

Cores reach each other over their lower Things, never by executing in a
sibling's script through their own connection. The latter would work — handles
are global — and would put one robot's core straight back inside another's,
which is the thing being undone.

### 11.2 The peer seam turned

§9.4 said Phase 2 replaces one function per peer. That was optimistic: it is
four. A peer is now four named operations rather than a channel for arbitrary
Lua, which is what made a remote implementation possible at all — `remotePeer`
cannot accept a Lua string, and should not.

| operation | local | remote |
|---|---|---|
| `gripperState` | driver call | `GET /properties/getGripperState` |
| `acceptTransfer` | driver call | `POST /actions/acceptTransfer` |
| `openSession` | object reference | `POST /actions/registerHandoverSession` |
| `updateSession` | object reference | `POST /actions/updateHandoverSession` |

The last three are new TD actions, added in `ensureSharedCellAffordances` so an
already generated robot picks them up without a pipeline re-run. They are
peer-to-peer plumbing, not agent affordances: an agent never invokes them.

### 11.3 The motion lock survived the split

The cell still moves one arm at a time, and it still should — every planner has
the other robots in its obstacle collection, so a preflighted path stops being
valid the moment another arm moves. What changed is where the rule lives:
`LocalArbiter` when the cores share a process, `RemoteArbiter` → the host's
`ArbiterServer` when they do not.

Two properties worth stating:

- A core that cannot reach the arbiter is **refused**, never assumed free.
  Assuming the lock is available because the question could not be asked is the
  one failure mode this mechanism exists to prevent.
- A core that dies still holds the lock, and nothing else can release it, so the
  host does — then stops the cell. Restarting an arm halfway through a planned
  trajectory would resume a cell in a state nobody checked. The plan said
  "restart on crash"; that was wrong, and this is the correction.

This is the seam Phase 4 replaces. `acquire`/`release` keep their shape and
"one arm at a time" becomes "one arm per contested region" — which is what
finally lets two robots move at once. **Nothing moves concurrently yet.**

---

## 12. Measurements for the thesis

### 12.1 The headline number: contested workspace fraction

Reproducible from the repository with no simulator running
(`npm run interaction:overlap`), at a common 5 cm resolution and common origin
so the three robots are directly comparable:

| robot | reachable samples | workspace voxels | contested | share | uncontested |
|---|---|---|---|---|---|
| Robot1 | 25,717 | 19,633 | 356 | **1.81 %** | 98.19 % |
| Robot2 | 32,595 | 8,562 | 425 | **4.96 %** | 95.04 % |
| Robot3 | 25,300 | 15,513 | 69 | **0.44 %** | 99.56 % |

| pair | shared voxels | of each |
|---|---|---|
| Robot1__Robot2 | 356 | 1.81 % of Robot1, 4.16 % of Robot2 |
| Robot2__Robot3 | 69 | 0.81 % of Robot2, 0.44 % of Robot3 |
| Robot1__Robot3 | 0 at 5 cm | 1 voxel at the 10.8 cm its own estimator picks |

**Quote the figure to two significant digits, not five.** Voxel membership at a
boundary depends on where the grid is anchored, so shifting the common origin
moves these counts by a percent or two of themselves -- 356 against 355, 425
against 418 -- while leaving every conclusion intact. The claim the number
supports is "a few percent", and that is robust; "4.96 %" is not a constant of
the cell, it is one alignment's reading of it. Report the voxel size and the
origin rule alongside it, as the tool does.

This is an *architectural* measurement, not a benchmark, which is what makes it
worth leading with: **a global motion mutex serialises the whole cell in order
to protect between 1.8 % and 4.9 % of its motion space.** It bounds how often a
spatial claim scheme can ever block, and it is derived from the robots' own
measured reachability clouds rather than from a chosen workload.

Report Robot1/Robot3 honestly as *at or below the resolution the sampling
supports*. That is a more informative statement than either "they overlap" or
"they do not", and it is precisely the case a neighbour-discovery threshold has
to answer.

### 12.2 The number NOT to report

Wall-clock speedup on the Robot1 -> Robot2 -> Robot3 handover chain.

That workload is **sequential by construction**: the cube physically travels
down the line and Robot3 can do nothing useful until it holds it. Amdahl's law
applies to almost the entire chain, so a concurrency result measured on it will
be small and will invite the obvious question of why the machinery was built.

The overlap actually available in the chain is narrow and specific: the giver's
return-home leg against the receiver's return-upright leg, which today are
serialised by `.wait(giverHome(Session))` in `+!receiveCube`. Two such legs, a
few seconds each, in a chain of the order of a minute or two. Worth measuring
and worth reporting -- as a secondary figure, with the sequential structure
stated plainly.

### 12.3 The numbers worth reporting instead

- **Makespan on a workload that has parallelism** -- three independent
  pick-and-place tasks, one per station -- reported separately from the handover
  chain. The honest framing: coordination cost is bounded by the contested
  fraction; the handover chain realises little of it because it is sequential,
  while independent task loads realise the rest.
- **Coordination overhead**: arbiter round trip against mean motion duration.
  Expect single-digit milliseconds against multi-second motions, i.e. well under
  0.1 %.
- **Watchdog aborts under concurrency**, which should be **zero** if the claims
  are correct. This is a safety result, and for this thesis it is worth more
  than any speedup figure.
- **Behaviour preservation across the refactor**: the driver rendered for three
  robots in two configurations each, byte-identical before and after (§10.1).

### 12.4 Discovery cascade cost

Measured on the same clouds (§14.1 explains why the middle tier may not prune):

| pair | AABB overlap | sampled, 200 pts | full voxel intersection |
|---|---|---|---|
| Robot1__Robot2 | 0.089 ms | 3.4 ms (3 hits) | 923 ms -> 358 voxels |
| Robot1__Robot3 | 0.012 ms | 1.8 ms (0 hits) | 754 ms -> 1 voxel |
| Robot2__Robot3 | 0.007 ms | 1.7 ms (6 hits) | 879 ms -> 42 voxels |

One-time per process: 139 ms to load three clouds, 422 ms to build three
KD-trees -- both now paid once and shared (§9.1).

---

## 13. Why safety is decomposed by arity

The question "are independent safety cores better than one shared safety core?"
does not have a single answer, because safety here is not one property. It
separates cleanly by **arity**, and each tier wants a different home.

| tier | examples | belongs |
|---|---|---|
| single-body | workspace admission, self-collision, own swept preflight, joint limits | genuinely **per robot** |
| pairwise / n-body | arm-to-arm collision | irreducibly **relational** -- neither robot owns it |
| physical backstop | the in-motion watchdog, other arms in its obstacle collection | **needs no agreement at all** |

**Single-body checks favour independent cores.** Each core's grid describes one
arm. The three robots already estimate genuinely different voxel sizes --
0.0488, 0.0218 and 0.0355 m -- so per-robot cores make that heterogeneity
structural rather than a parameter table in a shared component. No queueing
behind another arm's checks, and no shared mutable state to reason about.

**Pairwise checks favour a single core, and this must be said plainly.** One
core sees every arm in a single consistent snapshot, needs no protocol, and can
permit interleavings a claim-based scheme must conservatively forbid -- a claim
covers the whole swept volume for the whole motion duration, whereas a central
checker could reason about *when* each arm occupies *which* part. Independent
cores buy that back with an arbiter, leases and stale-claim handling: more code
and more failure modes than a central checker has.

So the defensible claim is **not** "independent safety cores are safer". It is:

> Safety is decomposed by arity. Single-body safety lives in each robot's own
> core; pairwise safety lives in a shared artifact that both participants
> compute identically; and a physical watchdog backstops both without requiring
> anyone to agree on anything.

**What makes the middle tier viable is order-independence.** Two independent
cores may act on a relational fact without a central authority *only because
they provably derive the same one*. Before §9.1 they did not: the same pair came
out as 358 voxels or 360 depending on which robot asked. The order-independence
fix is therefore not a tidiness fix -- it is the **precondition for
decentralised pairwise safety**. Had the shared space been computed on the fly
and disagreed between participants, independent cores would have been strictly
worse than one, and arguably unsafe.

Two caveats to state rather than let a reader find:

1. **Fault isolation is not currently realised.** The cell host stops the whole
   cell when a core dies -- safety over availability, deliberately, because
   restarting an arm mid-trajectory resumes a state nobody checked. Do not claim
   isolation as a benefit of the split today.
2. **A legitimate middle design exists**: per-robot planning cores plus a
   central safety *monitor* that only vetoes and never plans, which is what
   industrial cells do with a safety PLC. Naming it and saying why this system
   chose otherwise is stronger than omitting it.

---

## 14. Phase 8: agents that discover their own neighbours and their own scene

The target: an agent finds its neighbours by a fast precompute, computes the
interaction space only with those, and knows what else is standing in its
workspace -- rather than being handed all three answers in a descriptor.

### 14.1 The discovery cascade, and the one rule it must obey

Four tiers, spanning four orders of magnitude in cost (§12.4):

```
tier 0   workspace signature      ~10 float ops, no I/O      may prune
tier 1   transformed AABB         ~0.01-0.09 ms              may prune
tier 2   sampled proximity        ~2-3 ms                    MUST NOT prune
tier 3   full voxel intersection  ~750-920 ms                the authority
```

The rule, which the repository's own data demonstrates:

> A discovery prefilter may only test **necessary** conditions. A false positive
> costs a little coordination. A false negative is a collision.

Robot1/Robot3 is the worked example. A 200-point proximity sample returns **zero
hits**, while the full intersection finds a real shared voxel. Sampling would
have pruned a genuine overlap and left two arms believing they could never meet.

Note also that AABB overlap does not *rank* usefully here -- Robot1/Robot3's
overlap volume (1.264) exceeds Robot2/Robot3's (1.015) while containing 350x
less shared space. That is the correct division of labour: **the cheap tiers
prune, the sampled tier orders, and only the interaction-space calculator
decides.**

### 14.2 The workspace signature -- BUILT

Implemented as `getWorkspaceSignature` on every lower Thing, derived entirely
from artifacts that already existed: the reachability cloud each core loads at
startup, and the live base pose it already reports.

```
Robot1   base=[0, 0, 0.03]      reach=1.6010  safe=1.6498  spacing=0.0488  n=25717
Robot2   base=[1.5, 0.2, 0.03]  reach=0.8664  safe=0.8882  spacing=0.0218  n=32595
Robot3   base=[2.1, -1, 0.03]   reach=1.2825  safe=1.3181  spacing=0.0355  n=25300
```

**`safeReachRadius` is the value to test against, and the widening is not
cosmetic.** `reachRadius` is the extreme of a finite Monte-Carlo sample, so the
farthest reachable point is almost never one that was drawn: the raw figure
under-states the true reach by up to the gap between samples. Understating it is
the dangerous direction -- the test is a necessary condition, so a radius that
is too small prunes a pair that really can collide. Widening by the measured
sample spacing (which `estimateSpacing` already computes per cloud) covers
exactly that gap.

**Why the radius and not the bounds.** A sphere centred on the base is invariant
under however the arm is mounted; axis-aligned bounds are not. Robot3 is pitched
90 degrees, and its local z-extent lands somewhere else entirely in world:

```
Robot3   local z[-1.075, 1.034]
         naive z[-1.075, 1.034]     <- offset[2] + local z
         TRUE  z[-1.273, 0.912]     <- wrong by ~20 cm at both ends
         rMax  1.2825 local, 1.2825 after rotation   <- invariant
```

So the tier-0 test is radial only. Adding a z-interval would make it tighter for
upright mountings and wrong for Robot3, and doing it correctly means rotating
the local AABB's eight corners -- which is tier 1, not tier 0.

An analytic radius from the URDF was considered and rejected: summing the arm
joint origins gives 1.5398 m for Robot1 against a sampled 1.6010 m, because the
chain stops at the last arm joint and misses the fixed joints and the tool. An
UNDER-estimate is exactly the false negative this test must never produce. A
sound analytic bound would have to sum the whole chain to the TCP.

### 14.2b Neighbours as a belief

`neighborTest.js` implements the comparison; the AgentThing runs it against
every other robot in the cell at agent start; `src/agt/neighborPlans.asl` holds
the plans, and `!discoverNeighbors` is bound into `+!init` for every generated
agent.

Both the evidence and the verdict are published, because an agent that believes
"robot3 is not my neighbour" and cannot say why has been handed an answer rather
than given a reason:

```prolog
workspaceReach(robot1, 1.601, 1.6498).
reachTest(robot2, 1.5133, 2.538, 1.0247, reach_spheres_overlap).
neighbor(robot2_agent, robot2).
reachTest(robot3, 2.3259, 2.9678, 0.6419, reach_spheres_overlap).
neighbor(robot3_agent, robot3).
neighborScanComplete(2, 2).
```

`reachTest(Other, Distance, CombinedReach, Margin, Verdict)` carries the whole
test. The margin is the term worth reading: a small positive margin means the
pair only just qualifies, which is precisely the case worth computing exactly.

The test is symmetric by construction -- Robot1 and Robot2 both derive distance
1.5133 against combined reach 2.538 -- so two agents never disagree about
whether they are neighbours, for the same reason the interaction-space estimator
had to be made order-independent.

On the shipped cell every pair qualifies, which is the correct answer: all three
genuinely share voxels, Robot1/Robot3 by exactly one. The belief flips where it
should:

```
Robot3 base x=2.60   d=2.7857  margin=+0.1821  ->  neighbor(robot3_agent, robot3)
Robot3 base x=2.90   d=3.0676  margin=-0.0997  ->  notNeighbor(robot3, 3.068, 2.968)
```

### 14.2c The workspace signature, as originally proposed

Tier 0 needs a record small enough to be a WoT property, so that "could we
possibly be neighbours?" is answered without anyone loading a 35k-point CSV:

```
robot            name
basePose         live, from the existing getBasePose property
reachShell       rMin, rMax, zMin, zMax   (base frame)
localBounds      the 8 numbers of the local AABB
sampleCount      point count
csvDigest        already computed and cached (reachableDigestShared)
```

The tier-0 test is `distance(baseA, baseB) <= rMaxA + rMaxB` together with a
z-interval overlap. Ten float operations, provably never false-negative, and it
needs nothing but two properties read over HTTP.

Tier 1 is `discoverInteractionPairs` / `transformedWorkspaceBounds` /
`boundsOverlap`, which already exist in `robwotBridge.js` and are currently
bypassed because this cell configured its pairs explicitly. Tier 2 is
`estimateCloseCount`, already exported. Tier 3 is `build()`, which now serves a
second asker from cache (§9.1).

**What is missing is only the signature record and publishing it.**

### 14.3 Rendering interaction spaces live

This already works, and the design is right for decentralised discovery:
`refreshInteractionVisual` is a TD action on every robot's lower Thing, and it
draws from the **sandbox script** rather than from any arm's driver, because the
cloud is scene furniture and not robot state. Whoever discovers a pair can draw
it. `agentThingServer` already redraws whenever a space is recomputed rather
than served from cache.

Four additions for a discovery flow:

1. **Removal.** Today the action removes-then-recreates; there is no "this pair
   is no longer a neighbour, erase it".
2. **A race.** Remove-then-create keyed on an alias is not atomic, so two agents
   drawing the same pair at once can leave two clouds or none. Either draw under
   the pair owner, or have the sandbox script check-and-claim the alias.
3. **Colour at request time.** The non-colliding palette assignment added in
   §9.5 lives in `buildEveryPair`; a live flow needs "a colour no live pair is
   using" as a call rather than a batch pass.
4. **Redraw gating**, which the version hash already provides.

### 14.4 Typed scene-object discovery

The agent should know that the walls, the support and the cube are in its
workspace -- and should know they are *different kinds of thing*, because the
planner already treats them differently.

Most of the mechanism exists and is gated behind one configured list:

| exists | where |
|---|---|
| classification by tree membership (mine / cube / tool / support) | `womatIsSceneObstacleShape` |
| naming the owning model of a hit link | `womatObstacleOwnerAlias` |
| local-frame AABBs for admission | `womatObstacleLocalBoxes` |
| alias inventory over WoT | `getCollisionObstacles` property |

Four things to add:

**a. Enumerate the scene, not a list.** `womatSceneObstacleShapes` starts from
`local obstacleNames=${obstacleLua}` -- a Lua literal baked in at attach time.
Replace the default with self-exclusion: every shape that is not in my own tree
and not the object I am holding. Keep the alias list as an override for the
non-shared-scene case, where "everything that is not me" would sweep in scenery
that is deliberately ignored.

**b. Give every discovered object a kind.** The distinctions the runtime already
makes, made explicit:

| kind | how it is recognised | how it is treated |
|---|---|---|
| `peer_robot` | tree root is another robot model | obstacle; **this is who to coordinate with** |
| `payload` | is the grip object | not an obstacle to its holder or during its own pick approach; obstacle to everyone else |
| `support` | is the grip support | obstacle, except during a pick approach |
| `fixture` | static respondable, everything else | obstacle; static, so it can be folded into the workspace grid once |

The cube being a `payload` rather than an obstacle is the point the current code
already encodes implicitly, in `womatSetPickupApproach` and in the held-object
branch of `womatIsSceneObstacleShape`. Making it an explicit kind is what lets
the agent reason about it -- "the cube is in my workspace and it is graspable"
is a different belief from "there is an obstacle at that point".

**c. Answer workspace membership.** Test a discovered object's local AABB
against this robot's own `WorkspaceGrid`. That is a handful of admission
lookups against a structure each core already holds.

**d. Publish it as beliefs**, extending §5.2:

```prolog
sceneObject(Alias, Kind, OwnerRobot).
objectBox(Alias, MinX, MinY, MinZ, MaxX, MaxY, MaxZ).   // this robot's base frame
inMyWorkspace(Alias).
graspable(Alias) :- sceneObject(Alias, payload, _) & inMyWorkspace(Alias).
obstacle(Alias)  :- sceneObject(Alias, Kind, _) & Kind \== payload & inMyWorkspace(Alias).
```

Two things fall out of this that were previously configuration. Neighbours are
discoverable from `sceneObject(_, peer_robot, Owner)` -- the arms whose shapes
are in my workspace are exactly the arms I might collide with, which is tier 0
of §14.1 arrived at from the other direction. And `obstacleBox` beliefs give the
agent the means to explain a refusal in its own terms rather than relaying
`motionBlockedBy` verbatim.

### 14.5 The safety architecture the dynamic case needs

Keep the arity decomposition of §13. Exactly one tier has to change.

**Per-robot tier -- unchanged.** Own admission, self-collision, own swept
preflight. Depends on one arm; discovery does not touch it.

**Relational tier -- already suits dynamic discovery.** The version hash is
keyed on placements *and* on the CSVs, so a robot that moves invalidates its own
pairwise spaces automatically. That was built for cache correctness and turns
out to be exactly what runtime discovery requires. Claims go against these.

**Physical backstop -- this is the change, and it is the important one.**

The driver's obstacle set is a Lua literal fixed when the script is attached
(`03-scene-objects.lua`, `local obstacleNames=${obstacleLua}`). With neighbours
discovered at runtime that is a real hazard: **discover a new neighbour and the
watchdog does not see that arm.** The backstop silently stops backstopping, and
it fails in the one direction that matters.

The backstop must therefore **not depend on the discovery result at all**. This
is the same change as §14.4a, and it is a safety improvement independent of
discovery: self-exclusion is conservative by construction, needs no
configuration, and cannot be invalidated by a discovery bug. Collections are
already cached and invalidated on demand (`womatInvalidateSafetyCollections`),
so enumeration is paid once per change rather than once per check.

The invariant to state in the thesis:

> Discovery and claims determine **who coordinates with whom**. They never
> determine **what the watchdog can see**. If discovery is wrong, motions are
> refused or blocked -- never uncontained.

### 14.6 Sequencing

- Make the backstop discovery-independent (§14.4a / §14.5) **before** discovery
  ships, not after. It is the piece that makes getting discovery wrong
  survivable.
- Dynamic discovery interacts with Phase 4: a newly discovered neighbour is a
  newly contested region, so the claim tier must accept regions appearing at
  runtime rather than being enumerated once at startup.
- Typed object discovery (§14.4) subsumes part of Phase 5: `obstacleBox` stops
  being a semi-static belief pushed at init and becomes a live one.

---

## 15. Phase 0, the backstop, and Phase 4, as built

### 15.1 Phase 0: the measurement

`core/motionLog.js` writes one JSON line per motion per robot -- requested,
granted or refused, finished -- with a wall-clock anchor plus a monotonic clock
so records from three separate core processes land on one timeline. Enabled by
`WOMAT_MOTION_LOG`, labelled by `WOMAT_RUN_LABEL`, and it disables itself rather
than ever being the reason an arm stops moving.

`motionReport.js` (`npm run motion:report`) sweeps the intervals once and
reports the figure the whole exercise exists to produce: **wall time during
which two or more arms were moving**. Under a global mutex that is zero by
construction, which is what makes it a usable baseline.

`requested` and `granted` are recorded separately on purpose. The gap between
them is arbitration wait, and it is the cost the space lock is meant to remove.

### 15.2 The backstop stopped depending on configuration

`womatSceneObstacleShapes` walked a Lua literal fixed when the driver was
attached. It now enumerates the scene and excludes what is provably this
robot's own, with the alias list surviving only as an override
(`runtime.obstacleDiscovery: "configured"`). Measured on the shipped cell:

```
configured   75 obstacle shapes
scene        77 obstacle shapes      <- the Floor, which nobody had listed
```

**And the Floor immediately broke every pose**, which is exactly why this needed
measuring rather than reasoning about:

```
configured   zero-config safe: true
scene        zero-config safe: false     <- before the fix
```

Not a contact -- `checkCollision` returned 0 throughout. A floor-mounted arm's
base is permanently inside any sensible **clearance** margin of the floor it
stands on, so asking "is anything within 2 cm" against the full obstacle set
refuses every pose the robot can adopt.

Contact and clearance are different questions, and the fix is to stop conflating
them. The surface a robot rests on is exempt from the clearance test and never
from the contact test, and it is **detected rather than configured**: at rest, an
obstacle inside the margin but not touching is something this arm is mounted on.

```
configured   exempt []          75 obstacles   poses safe
scene        exempt ["Floor"]   77 obstacles   poses safe
```

The Floor is now a contact obstacle that was previously invisible, and no pose
regressed.

`womatSceneObjectInventory` reports the cell as typed objects, and the types are
derived rather than declared -- an articulated body is a tree with joints in it,
so a robot nobody configured is still recognised as one:

```
self        Robot1                35 shapes
peer_robot  Robot2                35 shapes
peer_robot  Robot3                35 shapes
payload     Robot1_grip_cube       1 shape
support     Robot1_grip_support    1 shape
fixture     Robot1_wall / Robot2_wall / Robot3_wall / Floor
```

Exposed as the `getSceneObjects` property, which is the substrate §14.4 needs.

### 15.3 Phase 4: the space lock

`RegionArbiter` replaces "one arm at a time" with "one arm per contested
region". A motion arrives with the shared voxels it may occupy; an empty set is
granted unconditionally, a non-empty set is granted if it is disjoint from every
live claim. Claims carry a lease, so a core that dies cannot wedge the cell.

**Claims are region-granular, and that is a deliberate limitation.** Arbitration
happens before planning, so neither the swept path nor where the elbow travels
is known yet. A claim smaller than the motion is a collision, so if a motion
could enter a shared region at all, it claims that region whole. Coarse -- but it
cannot under-claim, and it still answers the question that decides everything,
because a motion that cannot enter any shared region claims nothing and waits
for nobody. `womatSweepFramePositions` already reports what a per-voxel swept
claim would need; using it means arbitrating after planning instead of before.

Measured at rest on the shipped cell:

```
Robot1  regions [Robot1__Robot2, Robot1__Robot3]  ->    0 voxels  contested=false
Robot2  regions [Robot1__Robot2, Robot2__Robot3]  ->  400 voxels  contested=true
Robot3  regions [Robot1__Robot3, Robot2__Robot3]  ->    0 voxels  contested=false
```

Which matches §12.1 exactly: Robot2 has the highest contested fraction (4.96 %)
and is the arm whose rest pose sits inside the shared space.

### 15.4 A correction: what the motion log does and does not measure

The first concurrency figure reported here was **wrong**, and the way it was
wrong is worth recording.

`motionReport` measures each motion from grant to completion. That is what the
ARBITER did, and it is not what the arms did. Two things occupy an identical
window in the motion log while nothing turns:

- a motion that spends seconds in preflight and is then refused
  (`joint_path_collision_detected`);
- a motion to the pose the arm is already standing in.

The first run had both. Of its "5.48 s with two or more arms moving", 1.32 s was
Robot3 in a failed preflight, 3.66 s was Robot2 in a failed preflight, and the
remainder was Robot3 "succeeding" at a move to all-zero from all-zero. Watching
the simulator, only Robot1 visibly moved -- and the simulator was right.

`motionObserver.js` (`npm run motion:observe`) exists because of this. It samples
every robot's joint angles directly, independently of the cores, and counts an
arm as moving only when its joints change by more than a threshold. It is the
measurement to report; the motion log is context for it.

The general lesson, which belongs in the thesis: **instrumenting the controller
measures the controller.** A claim about the robots has to be sampled from the
robots.

### 15.5 The result

Same three commands, both modes, measured from scene ground truth rather than
from the cores' own bookkeeping:

| | `--exclusive-motion` (baseline) | region claims (default) |
|---|---|---|
| any arm actually moving | 4.31 s | 22.55 s |
| **2+ arms actually moving** | **0.00 s** | **17.61 s** |
| peak arms moving | 1 | **3** |
| Robot1 joint travel | 59.9 deg | 132.5 deg |
| Robot2 joint travel | **0.0 deg** | 150.0 deg |
| Robot3 joint travel | **0.0 deg** | 59.9 deg |

Under the mutex two of the three arms never moved at all: refused with
`motion_busy` and never retried. Under region claims all three swept, and for
17.6 s of a 45 s observation two or more were turning at once.

### 15.6 A second correction: the backstop paralysed Robot2

§15.2 claimed "no pose regressed" on the strength of testing Robot1. That was
not enough. Robot2 could not move **at all** once the scene was enumerated:

```
robot2  j1 +30  -> REFUSED joint_path_collision_detected obstacle=Floor owner=Floor
robot2  j1 -30  -> REFUSED joint_path_collision_detected obstacle=Floor owner=Floor
robot2  j2 -20  -> REFUSED joint_path_collision_detected obstacle=Floor owner=Floor
```

Not clearance this time -- actual contact. Robot2's base rests **on** the floor,
so with the floor in the obstacle set every motion collided before it started.
The clearance exemption of §15.2 does not help, because this is contact, and
contact must never be exempted.

The fix is kinematic rather than a heuristic: **a link no joint moves cannot
collide with anything it is not already touching.** `womatIsJointDrivenShape`
walks from a shape to the model root and reports whether any joint lies on the
path; shapes where none does are dropped from the robot collision collection.
The base and its pedestal are exactly those shapes, they are rigidly fixed to
the world, and testing them every step tests a constant.

Nothing is lost by dropping them. A fixed base cannot move into anything, and
anything that moves into it is caught by that mover's own check -- which has
this base in its obstacle set.



The same three simultaneous joint commands, in both modes, on the live cell:

| | `--exclusive-motion` (baseline) | region claims (default) |
|---|---|---|
| motions completed | 1 | 5 |
| **wall time 2+ arms moving** | **0.00 s** | **5.48 s** |
| peak arms moving | 1 | **3** |
| concurrency factor | 1.000 | **1.512** |
| refusals | 2 (`motion_busy`) | **0** |

Under the mutex, two of the three arms were refused outright. Under region
claims all three proceeded, and all three were in motion at the same moment.

This is the figure to report, with §12.2's caveat attached: it is measured on a
workload that *has* parallelism. The handover chain is sequential by
construction and will not show this, and saying so is part of reporting it
honestly.

### 15.5 What is still open

- **Claim granularity.** Region-level, per §15.3. Refining it to the swept
  volume needs arbitration to move after planning.
- **`+myStatus(broken)` is still terminal.** Phase 6 remains a prerequisite for
  running the *agents* concurrently, as opposed to the cores. The measurements
  above drive the lower Things directly.
- **Two `joint_path_collision_detected` refusals** appeared in the concurrent
  run. Those are the preflight doing its job on poses that genuinely collide on
  the way to the goal -- not concurrency failures -- but they are the reason
  Phase 6 matters: an agent meeting one today has no recovery plan.
