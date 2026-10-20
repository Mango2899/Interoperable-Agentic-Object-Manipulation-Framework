# Migration Plan: Robotics Core to Python, WoT/Agent Layer stays in TS/JS

## 1. Why migrate

The problem is not TypeScript. It is that the safety and motion engine lives as a
**~2,925-line Lua program embedded in a JavaScript template literal**
(`prepareRuntimeDriverScript`, `lowerThingServer.js:180-3104`), plus another
~460 Lua lines in `robwotBridge.js:465-924`.

Consequences of that design:

- no syntax checking, no linting, no type information on the Lua
- no stack traces: a runtime error surfaces as a generic script failure
- no unit tests possible — the code only exists once injected into the scene
- no debugger, no breakpoints, no REPL
- every collision/preflight sample costs a remote-API round trip
- 130 `womat*` functions with no module boundaries

CoppeliaSim 4.10 ships `coppeliasim_zmqremoteapi_client` for Python
(`~/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/python`).
Verified capabilities: `client.require()` (so `simIK`, `simOMPL` are directly
callable), Python function **callbacks** (so `sim.moveToConfig`/`sim.moveToPose`
work), and `sim.setStepping()`.

This means every Lua line can become ordinary Python calling the same API —
testable, debuggable, roughly one third the length.

### Performance is NOT a reason to migrate

Do not justify this migration on speed. Measured against the actual code:

- workspace admission (`WorkspaceGrid.admitsPath`) is **already** a client-side
  spatial hash in Node — zero round trips, and correctly designed
- `preflightJointPath` **already** batches a whole path into one
  `womatPreflightJointPath(...)` call
- `preflightSimpleCartesianPath` **already** batches into one
  `womatSweepCartesianPath(...)` call

The only surviving per-sample loop is `preflightCartesianPath`
(`lowerThingServer.js:3602`), reached from a single fallback call site
(`:4420`), not the hot path.

Actual round-trip distribution for a typical motion: ~4 calls for
getCartesian/preflight/plan/follow, versus ~80 for `waitForMotionComplete`
polling `check()` every 50 ms. **~95% of remote-API traffic is polling**, costing
roughly 80 ms on a 4,000 ms motion — 1–2% overhead. There is no throughput
bottleneck to fix.

There is a *latency* issue: 50 ms poll granularity adds up to 50 ms of slop to
every motion-completion detection, which matters for handover choreography.

Both remaining fixes are language-independent and belong in the **current JS**,
independent of this migration:

1. batch `preflightCartesianPath` like the other two preflights (~20 lines)
2. replace the 50 ms poll with a signal + callback push (~20 lines)

`sim.checkCollision` round trips are irreducible in any language, because
obstacle geometry lives in the scene, not the client. Python's only real
advantage here would be mirroring collision meshes client-side (`trimesh` +
`python-fcl`, the same FCL that MoveIt uses — JS has no mature equivalent), and
at 1–2% overhead that is not worth the risk of a client-side collision model
silently disagreeing with the sim.

**The sole justification for this migration is code quality:** eliminating 2,925
lines of untestable, undebuggable Lua embedded in a JS template literal.

---

## 2. Target architecture

```
┌─────────────────────────────────────────────────────────┐
│  JaCaMo / Jason agents        (Java + .asl)   UNCHANGED │
└────────────────────────┬────────────────────────────────┘
                         │ CArtAgO artifact
┌────────────────────────┴────────────────────────────────┐
│  agentThingServer.js  — upper WoT Thing       STAYS TS  │
│  (agent reasoning bridge, AgentMessage)                 │
└────────────────────────┬────────────────────────────────┘
                         │ HTTP / WoT
┌────────────────────────┴────────────────────────────────┐
│  lowerThingServer.js  — node-wot servient     STAYS TS  │
│  ~300 lines: serve the TD, 13 handlers,                 │
│  each one an HTTP call downward. TD UNCHANGED.          │
└────────────────────────┬────────────────────────────────┘
                         │ localhost JSON REST  ← NEW SEAM
┌────────────────────────┴────────────────────────────────┐
│  womat_motion  — Python motion service          NEW     │
│  FastAPI + uvicorn, one process per robot               │
│  ├── safety/     collision, clearance, sweep, grid      │
│  ├── motion/     IK, OMPL planning, trajectories        │
│  ├── gripper/    attach/detach, joint drive             │
│  ├── handover/   standoff, approach axis, session       │
│  └── sim/        ZMQ client, scene setup, stepping      │
└────────────────────────┬────────────────────────────────┘
                         │ ZMQ Remote API (sim, simIK, simOMPL)
┌────────────────────────┴────────────────────────────────┐
│  CoppeliaSim 4.10                                       │
└─────────────────────────────────────────────────────────┘
```

**The Thing Description is the contract.** Because the TD stays byte-identical,
the agents, the `.asl` plans, and `agentThingServer.js` never learn that the
engine changed language. That is what makes this migration safe to do
incrementally.

### Why REST over stdin/stdout or ZMQ between TS and Python

- you already run HTTP servients, so the operational model is familiar
- `curl` becomes your debugging tool for the entire motion engine
- process isolation: a Python crash does not take down the WoT server
- trivially matches the existing per-robot port scheme in `manifest.json`

Add one port per robot, e.g. `ports.motionService = ports.lowerThing + 1000`.

### Concurrency rule

One Python process per robot owns exactly one ZMQ connection. The remote API
client is not thread-safe per connection, and you already have
`runExclusiveMotion` mutex semantics in `lowerThingServer.js:4837`. Keep that
mutex in Python (an `asyncio.Lock`) and let the TS side stay stateless.

---

## 3. What moves to Python

| Source | LOC | Becomes | Est. Python LOC |
|---|---|---|---|
| Lua safety kernel (`womatObstacleCollection`, `womat*Collision`, `womatClearanceUnsafe`, `womatSweep*`, `womatPreflight*`) | ~900 | `safety/` calling `sim.checkCollision` / `sim.checkDistance`, batched per path as today | ~250 |
| Lua motion (`womatPlanTopDownPick`, `womatPlanHandoverApproach`, `womatSolveHandoverStandoff`, `womatFindGoalConfig`, `womatPlanJointPath`) | ~1,200 | `motion/` calling `simIK` + `simOMPL` directly | ~350 |
| Lua gripper (`womatEnsureTcpGripperModel`, `closeGripper`, `openGripper`, `acceptTransferredGripObject`, ...) | ~600 | `gripper/` | ~180 |
| Lua path utilities (`womatSmoothFlatPath`, `womatShortcutFlatPath`, `womatDensifyFlatPath`) | ~200 | numpy vectorised, ~15 lines each | ~60 |
| Node orchestration (`smoothCartesianMove`, `planAndFollow`, `tryBaseJointFirst`, `cartesianPlannerFallback`, `preflight*`, `waitForMotionComplete`) | ~1,400 | `motion/strategies.py` — the retry ladder as a declarative list | ~300 |
| `interactionSpace.js` numerics | 825 | `analysis/interaction_space.py` with `cKDTree` + `ConvexHull` | ~200 |
| `workspaceGrid.js` | 244 | `safety/workspace.py`, numpy occupancy | ~80 |
| `handoverGeometry.js` | 200 | `spatialmath-python` SE3, mostly deleted | ~50 |
| `robwotBridge.js` Lua workspace sampler | ~460 | `pipeline/workspace_sampling.py` | ~150 |
| **Total** | **~6,030** | | **~1,620** |

### Library choices

| Need | Use | Replaces |
|---|---|---|
| Sim control | `coppeliasim_zmqremoteapi_client` (ships with 4.10) | the entire Lua-in-JS template |
| IK | `simIK` via ZMQ; optionally `roboticstoolbox-python` analytic IK for UR-family | `womatEnsureSimplePositionIk`, `womatFindGoalConfig` |
| Path planning | `simOMPL` via ZMQ | `womatPlanJointPath` |
| Frame math | `spatialmath-python` (SE3) | `handoverGeometry.js` in full |
| Spatial queries | `scipy.spatial.cKDTree`, `ConvexHull` | seeded sampling + percentile heuristics in `interactionSpace.js` |
| Trajectories | `sim.moveToConfig`/`moveToPose` with Python callbacks, **or** the `ruckig` pip package client-side | `womatSimpleMoveToPose`, `interpolate`, `*StepCount` |
| Offline collision (optional) | `trimesh` + `python-fcl` | swept-volume checks |

Note on `interactionSpace.js`: `estimateSpacing` currently approximates
nearest-neighbour distance by sampling 20,000 points and taking a percentile.
`cKDTree.query(points, k=2)` gives the exact answer over the full cloud. That is
a correctness improvement, and the one place where a Python library genuinely
beats what is there — but it is an offline pipeline path, not a runtime one.

There is no drop-in "robot safety engine" package on GitHub — be sceptical of
that framing. The value is `simIK` + `simOMPL` + numpy called from a real
language, not a magic dependency.

---

## 4. What stays in TS/JS

| Component | LOC | Why it stays |
|---|---|---|
| `lowerThingServer.js` (reduced to ~300) | 300 | node-wot servient. `wotpy` is unmaintained; node-wot is the reference implementation and your TD tooling depends on it. |
| `agentThingServer.js` | 689 | Bridges to JaCaMo via `dist/WoT/agentThing` + `AgentMessage`. This is JVM/Node interop — rewriting it in Python buys nothing and breaks the CArtAgO artifact. |
| `generateRobot.js` | 1,551 | Pure scaffolding: emits TDs, `.asl`, `.ts`, `.jcm`, README. Zero robotics. Rewriting it is churn. |
| `deleteRobot.js`, `cleanupRobotPorts.js` | 246 | Node process/port lifecycle management. |
| `urdfStructure.js` | 299 | Feeds `generateRobot.js`; keep it next to its only consumer. |
| `src/WoT/generated/*.ts`, TDs | 590 | Generated WoT artifacts. |
| `src/agt/*.asl`, `src/env/*.java` | — | Jason/JaCaMo. Untouched. |
| `robwotBridge.js` (reduced) | ~800 | Keep URDF import + model generation orchestration; move only the Lua sampler out. |

**Rule of thumb:** anything that speaks WoT, TD, or JaCaMo stays in TS.
Anything that computes geometry, collisions, or trajectories moves to Python.

`interactionSpace.js` is the one split case: the `InteractionSpaceController`
class (`interactionSpace.js:353`) is agent-facing session state and stays in TS;
the numerics beneath it move to Python behind a `/interaction-space/estimate`
endpoint.

---

## 4b. Planned future work — where it belongs

Two developments are planned: a neighbor-recognition algorithm over the existing
voxelization, and a distributed consensus system for transferring the cube along
a chain of robots. Neither is a reason to migrate, and one is a reason **not**
to migrate part of the stack.

### Neighbor recognition — already ~80% built, stays in TS

- `robwotBridge.js:194` `discoverInteractionPairs` — AABB-overlap neighbor
  discovery, tagged `workspace_aabb_overlap`
- `interactionSpace.js:207` `estimateInteractionSpace` — voxel intersection with
  adaptive voxel sizing

Missing is only the **graph layer**: transitive routing so an A→D goal derives
the chain A→B→C→D instead of relying on a hardcoded `handoverSuccessor` belief.
That is a BFS/Dijkstra over 3–10 nodes, roughly 40 lines. `networkx` would be
pleasant but is not worth a language boundary for a graph this size.

### Distributed consensus — stays in Jason/.asl, emphatically

`src/agt/handoverPlans.asl` (341 lines) is already a working distributed
protocol: peer-to-peer `.send(achieve/tell)` with no broker, a session-scoped
stage machine, `.wait` barriers with `[source(Agent)]` provenance, compensation
plans on failure, chain composition via `continueHandoverChain`, and
authoritative-state reconciliation through `refreshCubeOwner`.

Python alternatives are strictly worse here:

| Option | Verdict |
|---|---|
| SPADE (XMPP/FIPA) | thin, lightly maintained; means rewriting 1,084 lines of working `.asl` |
| Mesa | agent-based *simulation* for social/economic modeling — wrong tool category |
| any Python BDI engine | none approach Jason's maturity |

JaCaMo + WoT + robotics is also the novel combination for the thesis. Moving the
agent layer to Python reduces novelty.

### The actual gap: motion is globally serialized

`lowerThingServer.js:3409` creates **one** `motionCoordinator` object shared by
all robots, so `runExclusiveMotion` permits exactly one moving robot in the
entire cell. Coordination is distributed at the BDI layer but globally
serialized at the motion layer.

The natural contribution is to replace the global mutex with **voxel-overlap
mutual exclusion**: two robots may move concurrently unless their swept
interaction voxels intersect — using voxels that are already computed. The
geometry may live in Python; the negotiation protocol must stay in `.asl`.

### Caution on "path planning with distributed consensus"

MAPF libraries (CBS, ECBS, `libMultiRobotPlanning`) solve **mobile robots on a
shared grid**. These robots are **stationary arms**. That is multi-arm
coordination in a shared workspace — a different problem with a far thinner
library landscape. Adopting a MAPF package would be a category error.

Realistic ladder for stationary arms:

1. global mutex — **exists today**
2. geometry-derived mutual exclusion — **the right next step, no new libraries**
3. prioritized planning with peers as dynamic obstacles
4. composite configuration-space planning

Rung 2 is appropriately sized for a thesis contribution.

---

## 5. Migration phases

Strangler-fig order — the system is runnable and demoable after every phase.

### Phase 0 — Contract freeze and golden traces (0.5 day)
Before touching anything: script a scenario that exercises all 13 affordances
and record request/response JSON to `testdata/golden/`. This is your parity
oracle for every later phase. You already have `testdata/` staged in git — use it.
Freeze `Robot1.lower.td.json` and add a test asserting it does not change.

### Phase 1 — Python service skeleton + read-only parity (1–2 days)
Stand up `womat_motion` with FastAPI, ZMQ connection, manifest loading, health
endpoint. Port the three read-only properties: `getJointposition`,
`getCartesianposition`, `getGripperState`. Point those three TS handlers at
Python. Assert byte-parity against golden traces. **Milestone: proof the seam
works with zero behavioural risk.**

### Phase 2 — Gripper (2–3 days)
Port `closeGripper` / `openGripper` / `acceptTransferredGripObject` /
`relinquishTransferredGripObject` and the TCP gripper model setup. Self-contained,
and one of your stated pain points. `sim.setObjectParent` for attach beats the
current signal-and-script dance.

### Phase 3 — Safety kernel (3–4 days)
Port collision collections, self-collision, clearance, and sweep sampling.

`WorkspaceGrid` is a **straight port, not a redesign** — the existing JS spatial
hash is already client-side and correct. Reimplement the same algorithm with
numpy arrays; do not "improve" it into a KD-tree expecting a speedup, there is
none to get. The one genuine upgrade is `estimateSpacing`: replace the
20,000-point percentile approximation with `cKDTree.query(k=2)` for the exact
nearest-neighbour distance over the full cloud. That is a *correctness*
improvement, not a performance one.

Keep sweep checks batched into single script calls exactly as the JS does today.

### Phase 4 — Motion (5–7 days, the big one)
Port IK, OMPL joint planning, `topDownPick`, `handoverApproach`. Rewrite the
retry ladder (`tryBaseJointFirst` → `planAndFollow` → `cartesianPlannerFallback`)
as an ordered list of strategy objects, each with `applicable()` and `attempt()`.
Today that ladder is ~1,400 lines of nested `if (!result.success)`; it should be
~80 lines plus one strategy class per rung.

### Phase 5 — Analysis offline (2 days)
Port `interactionSpace.js` numerics and `handoverGeometry.js`. Easy to validate:
run both implementations on the same CSVs and diff the voxel keys. Zero sim
involvement, so this phase can be done in parallel with any other.

### Phase 6 — Pipeline sampler (2 days)
Move `robwotBridge.js`'s workspace-sampling Lua to Python. Pipeline-time only,
not runtime, so lowest risk — it is last on purpose.

### Phase 7 — Delete (0.5 day)
Remove `prepareRuntimeDriverScript` and the dead Node orchestration.
`lowerThingServer.js` drops from 5,068 to ~300 lines.

**Total: roughly 3–4 weeks.** Phase 5 is parallelisable; Phases 1–4 are strictly
sequential.

---

## 6. Risks

| Risk | Mitigation |
|---|---|
| Startup ordering: Python service must be up before the WoT servient serves the TD | Have the TS side poll `/health` before `servient.start()`; fail loudly with a clear message |
| Two processes per robot to supervise (3 robots = 6 processes) | Extend `cleanupRobotPorts.js` to cover motion-service ports; add one launcher script |
| ZMQ vs the existing websocket remote API in the same scene | They coexist — different ports, different client IDs. But keep `robwotBridge.js` on websockets until Phase 6 rather than mixing mid-migration. |
| `sim.moveToConfig` callbacks over ZMQ prove flaky | Fall back to the `ruckig` pip package: generate the trajectory client-side, `sim.setJointTargetPosition` per step under `setStepping(True)`. Fully in your control. |
| Thesis timeline pressure | Every phase leaves a working demo. If you run out of time after Phase 4 you still have the whole engine ported and a defensible result. |

---

## 7. What to do first

### Independent of this migration (do it anyway, ~40 lines of JS)

These two fixes are worth doing in the current codebase whether or not you
migrate, and they carry over to Python untouched:

- batch `preflightCartesianPath` (`lowerThingServer.js:3602`) into one script
  call, matching `preflightJointPath`
- replace `waitForMotionComplete`'s 50 ms poll with a signal + callback push,
  removing ~95% of remote-API traffic and up to 50 ms of completion latency

### Then the migration

1. `python -m venv .venv && pip install "fastapi[standard]" numpy scipy spatialmath-python`
2. `pip install -e ~/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/python`
3. Write the Phase 0 golden-trace capture script.
4. Phase 1 skeleton — three read-only endpoints, prove parity, then commit.

Do not start with the motion engine. Start with `getJointposition`, because
proving the seam is worth more than proving the math.
