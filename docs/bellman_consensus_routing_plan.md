# Distributed Bellman consensus routing for cube transfer

Replaces the statically generated handover chain with a route each cell
discovers for itself: agents detect the task sites in their own workspace,
flood a request for the goal site, and relax accumulated handover costs on the
way back until every agent on the path knows its own next hop.

Today the chain is a compile-time fact. `robwotBridge.js:1761-1787` writes
`handoverSuccessor(robotN_agent, robotN).` into every generated `.asl`, and
`+!continueHandoverChain` in `src/agt/handoverPlans.asl` simply follows it,
falling back to `!returnCubeToSupport` when a robot has no successor. Nothing in
the running system decides *where* the cube should go, or which of several
possible routes is cheaper. That is what this plan adds.

The 4-robot cell is already the right test bed: its neighbour graph is
`Robot1-Robot2`, `Robot1-Robot3`, `Robot2-Robot3`, `Robot3-Robot4` — a triangle
with a tail. `Robot1 -> Robot4` therefore has two genuinely different routes
(`1-3-4` and `1-2-3-4`), so "shortest hop count" and "cheapest" can be made to
disagree. Every optimality claim below is testable in that cell.

---

## 1. Algorithm specification

### 1.1 Per-agent state (all Jason beliefs)

| Belief | Meaning |
| --- | --- |
| `site(Alias, Kind, X, Y, Z)` | a support/payload the robot detected, in its own base frame |
| `siteReachable(Alias)` | that site lies inside this robot's sampled workspace |
| `siteOccupied(Alias, Cube)` / `siteEmpty(Alias)` | occupancy from the scene inventory |
| `hasSource(Alias, Cube)` | reachable, occupied — this robot is at **A** |
| `hasSink(Alias)` | reachable, empty — this robot is at **B** |
| `routeRequestFrom(Task, UpstreamAgent)` | who asked me (one per upstream, not just the first) |
| `routePending(Task, N)` | replies still outstanding from downstream |
| `edgeCost(Task, Neighbour, C)` | my own leg on that edge: `||tcp - p(me, Neighbour)||` |
| `routeCost(Task, Neighbour, V)` | total accumulated cost to B, as that neighbour reported it |
| `nextHop(Task, Agent, Robot, Total)` | the argmin — the only thing execution reads |

### 1.2 Cost, and how it accumulates

Each robot contributes **exactly one term**: the distance from its own TCP to
the handover point of the rendezvous it is answering across. It adds that term
to whatever total it received from downstream and passes the sum upstream.

```
value that i reports to its upstream requester u:

    V(i -> u) = || tcp_i  -  p(i, u) ||  +  min over downstream j of V(j -> i)

and at the robot holding B, the sum bottoms out:

    V(b -> u) = || tcp_b  -  p(b, u) ||
```

So the number arriving at `u` is already the complete cost of the whole
remaining route, and `u`'s decision is a plain `.min` over what its neighbours
sent — no further arithmetic at selection time. That is the property that makes
the choice cheap at every node, which is the point of accumulating upstream
rather than collecting paths centrally.

Two consequences worth being explicit about:

- A node answering several upstream requesters sends each of them a **different**
  number, because `p(i, u)` is a different rendezvous per edge. Only the
  downstream part (`min over j`) is shared between those replies.
- The one leg the sum does not contain is the origin's own approach to its first
  handover, since the origin has nobody upstream to report to. It can price that
  locally with the same query before taking its `.min` — one extra term per
  candidate first hop, no extra messages. Stage 2 leaves it out; Stage 5 adds it
  and reports whether it ever changes the choice.

The distance itself is already computable: `rank()`
(`interactionSpace.js:890`) returns candidates carrying `giver.localTcp` and
`receiver.localTcp` for the pair's best rendezvous, and `getCartesianposition`
(`core/thing.js:369`) gives the current TCP. Note that `rank()` always treats
the computing robot as `space.owner` (`interactionSpace.js:943`), so a robot
pricing an edge gets *its own* approach point to the shared rendezvous whether
it will end up giving or receiving there; the giver/receiver role shifts that
point only by the grasp offset. Good enough to price with, **not** to be reused
as the executed target — the executed target comes from the giver's own
`commitHandoverCandidate` at handover time.

**Assumption to state in the thesis:** `tcp_i` is sampled at search time, so the
term describes the cost from where the arm stands *now*, not from where it will
stand once the cube reaches it. Stage 6 offers the refinement (price the edge
from the inbound rendezvous point instead of the current TCP), which is still
purely local knowledge.

### 1.3 Messages (native Jason `.send`, no new transport)

| Message | Direction | Payload |
| --- | --- | --- |
| `findRoute(Task, Cube, TargetSite, TTL)` | downstream flood | goal identity, hop budget |
| `routeCost(Task, Epoch, Cost, Path)` | upstream reply | accumulated cost so far |
| `noRoute(Task)` | upstream reply | this branch is a dead end |

### 1.4 Protocol

1. **Origin** (the agent with `hasSource`) adopts `!transferCubeToGoal(Cube, B)`.
   If it also has `hasSink(B)` the route is empty and it just places the cube.
   Otherwise it sends `findRoute` to every `neighbor/2` and waits.
2. **On `findRoute` from `u`**: record `routeRequestFrom(Task, u)`. If already
   seen this Task, reply `noRoute` immediately — this is what breaks the cycle
   in the `1-2-3` triangle and prevents a flood storm. If `hasSink(TargetSite)`,
   price the edge back to `u` with `!estimateEdgeCost(u)` and reply
   `routeCost(Task, 0, ||tcp - p(me,u)||, [me])`. Otherwise forward to all
   neighbours except `u`, set `routePending(Task, N)`, and wait.
3. **On `routeCost(Task, E, V, Path)` from `j`**: store
   `routeCost(Task, j, V)` as it arrived — `j` already added its own leg, so
   nothing is added here — and decrement `routePending`. When it reaches zero
   (or the timeout fires), take `.min` over the stored values, assert
   `nextHop(Task, j*, V*)`, and reply to **each** `u` in
   `routeRequestFrom(Task, _)` with `routeCost(Task, E, C_u + V*, [me|Path])`,
   where `C_u = !estimateEdgeCost(u)` is priced separately per upstream edge.
4. **On `noRoute`**: decrement pending; if all children answered `noRoute`,
   propagate `noRoute` upstream.
5. **Origin** picks its own argmin, and the chain
   `nextHop -> nextHop -> ... -> hasSink` is the route. Because all edge costs
   are positive and the accumulated value strictly decreases along `nextHop`,
   the chain cannot loop.

### 1.5 Optimality — and the honest caveat

Steps 1-4 are a **request-tree** search: the "already seen, reply `noRoute`"
rule means a node answers only the first requester with real information, so
the answer is optimal over the paths in the flood tree, not over all paths in
the graph. On the triangle this can miss the cheaper route.

Stage 5 upgrades it to true Bellman-Ford by allowing **re-announcement**: when a
node's accumulated minimum improves after it has already replied, it bumps
`Epoch` and re-sends `routeCost` upstream; upstream relaxes again and propagates only if improved.
With positive costs this converges, and the origin declares the route stable
after a quiescence window with no improvement. Stage 5 is where the algorithm
earns the name, and its test is precisely the case where the tree answer and the
optimal answer differ.

---

## 2. Implementation stages

Each stage is independently runnable and independently testable. Nothing in an
earlier stage is thrown away by a later one.

### Stage 0 — Site detection: who has A, who has B — **implemented**

The detection path, from scene to belief:

- `src/generator/lua/03-scene-objects.lua` — `womatSceneObjectInventory` now
  reports geometry as well as kind: world position, position in the reading
  robot's own frame (`womatWorldToLocal`), and bounding-box size. A new
  `womatTaskSiteDefinitions` list lets an arm recognise a cube support that is
  not its own pick support.
- `src/generator/lua/05-gripper.lua` — `womatEnsureTaskSites()` builds the
  declared supports the same way the pick support is built, idempotent by alias,
  called from `womatEnsureGripObjects`.
- `src/generator/core/cellHost.js` — `collectCellTaskSites()` resolves every
  robot's declared sites into world coordinates and hands the **whole list to
  every driver**. An arm that only knew its own sites could never answer "no, I
  do not have that one" about a neighbour's, and that answer is what the search
  in Stage 2 is made of.
- `src/generator/taskSites.js` — the classifier, pure and unit-tested. Occupancy
  is footprint overlap plus a resting clearance, not centre proximity; the
  reachability question is asked of the **grasp point** (top face plus half a
  payload), never the support's own centre, which is buried inside a solid block.
- `src/generator/core/thing.js` — the `getTaskSites` lower-Thing property.
  Reachability is the one part the scene cannot answer, because it is a property
  of this arm's sampled workspace, which lives in the runtime and not in the
  simulator.
- `src/generator/agentThingServer.js` — the `detectTaskSites` keyword, emitting
  `site/4` (grasp point, robot-local), `siteReachable/1`, `siteOccupied/2`,
  `siteEmpty/1`, `taskSitesReady/2`. It keeps an atom-to-alias map so a later
  goal carrying the atom can still name the object.
- `src/agt/routePlans.asl` — `+!detectTaskSites` and the derivations
  `hasSource/2`, `hasSink/1`, `knownButUnreachable/1`. Included from every
  generated agent, and run from `!init` after `!shareInteractionSpaces`.

**The goal support.** `Robot4` declares one in `runtime.taskSites`, at
`(0.25, 0, 0.08)` in its own frame — world `(2.65, -1.6, 0.08)` — with the same
`0.14 x 0.14 x 0.16` footprint as the cell's loaded support, so the goal end of
the transfer is geometrically the same problem as the source end. Its grasp
point sits 1 cm from a sampled reachable point, well inside Robot4's 0.08 m
admission tolerance, and the hover points above it are reachable too.

**Tests** (`npm test`, all offline)
- `taskSites.test.js` — the classifier: occupancy by footprint overlap, the
  grasp point above the top face, an unmeasured support falling back to a
  nominal footprint, a carried cube resting on nothing, and only supports
  becoming sites.
- `cellTaskSites.test.js` — a site declared in one robot's frame is placed in
  the cell by that robot's base pose, and reaches *every* driver.
- `routePlans.test.js` — plan structure: bounded waits, stale beliefs abolished
  before a rescan, both derivations requiring reachability, and `site/4`
  asserted independently of it.
- `taskSiteDetection.test.js` — the real driver Lua running under `lua` against
  a stub cell (`testdata/cellSceneStub.lua`), its inventory classified with the
  arm's real sampled workspace. This is the one that matters: **exactly one arm
  reports A and exactly one reports B**, each names the site it cannot serve,
  and moving the cube in the scene moves who believes they are at A — with
  nothing regenerated.

**Live check.** Start the 4-robot cell and read each agent's startup print.
Expect `Robot1` to report the loaded support reachable and holding the cube,
`Robot4` to report the goal support reachable and empty, and both to report the
other's support as seen but out of reach.

**A WoT DataSchema cannot carry a null.** The first live run failed on every
agent with `Invalid value according to DataSchema`, because an empty support
reports `occupiedBy: null` and the property declares that field a string. A
consumed property is validated with ajv *before* the value reaches the caller,
so one null in one field fails the entire read and the only symptom upstream is
an agent that believes the cell has no supports at all. Absence is now expressed
the two ways the schema allows: the key is omitted when there is no value, and
occupancy uses the same `"none"` sentinel `getCubeOwnership` and
`getGripperState` already use. `taskSitesSchema.test.js` validates real
classifier output against the real served schema, and the offline detection test
now ends by putting the actual driver output through it -- the link that was
missing when detection passed every unit test and still failed on every agent.

**Known follow-up, not Stage 0.** The new support is a scene obstacle like any
other, and `womatIsSceneObstacleShape` exempts only the *configured* pick
support while an arm holds the cube. The terminal place-at-B in Stage 4 will
need declared task sites given the same exemption, or the final approach will
fail its collision preflight against the very support it is placing onto.

#### Where A and B are declared

Detection only means something if the sites are placed independently of the
arms. They were not. **A** lived in `Robot1.runtime.gripper` as
`supportPosition`, which `sharedGripConfig` read as world coordinates on the
strength of a comment — *"the scene owner sits at the origin"* — and **B** lived
in `Robot4.runtime.taskSites` in Robot4's own frame, rotated into the cell by
Robot4's yaw. Two sites, two frames, neither of them the cell's, and no way to
say either was turned.

A site is now declared once, in the cell's own coordinates, with a yaw:

```
npm run robot:pipeline -- --manifests ... --generate-shared-cell \
  --sites "Robot1_grip_support@0.9,0,0.08,Robot1_grip_cube; \
           Robot4_grip_support_b@1.25,-2,0.08"
```

`alias@x,y,z[,yawDeg][,payloadAlias]`, semicolon-separated, the same shape
`--offsets` and `--wall-offsets` already read. Yaw and payload are both optional
and both follow the coordinates; they are told apart by whether the token is a
number, so a site can carry a payload without stating a yaw it does not have.

Three things follow from the declaration rather than being written twice:

- **Which site is A.** The site carrying a payload is where the cube starts.
  Nothing is labelled source or goal — that stays what the arms read back out of
  the scene, which is what makes Stage 0 a detection rather than a lookup.
  Declaring two payloads is refused, because it would make the origin of the
  transfer a matter of configuration.
- **Where the cube stands.** Derived as the support's top face plus half the
  payload, the same arithmetic `graspPointOf` uses to read a loaded site back.
  Moving a support moves its cube. The derived height reproduces the 0.185 the
  manifest had been carrying by hand, which is the test that says this moved the
  declaration and not the furniture.
- **Who serves it.** Nobody says. A cell site has no declaring robot; the arms
  work out reachability from their own sampled workspaces, which is the fact the
  whole search is built on.

Yaw only, about the site's own centre: these are boxes on the floor, the grasp
point is the middle of the top face, and a level top face is what the descent
models. Rolling one onto its side needs the pick path revisited, not another
number in the declaration.

The robot-local form still resolves, so a manifest written before this places
its site exactly where it always did — and records which robot's frame it came
out of, which a cell site has no need for.

### Stage 1 — My leg on an edge, as a first-class query — **implemented**

- `src/generator/agentThingServer.js` — the `estimateEdgeCost` keyword. It
  ensures the pair's space exists (building with `synthesizePair: true` if this
  arm does not own it), ranks the rendezvous, and prices **every** viable one as
  the distance from this arm's own start to its own approach point
  (`candidate.giver.localTcp`). It emits `edgeCandidate/8` per rendezvous,
  `edgeCostOrigin/5`, and `edgeCostReady/2`.
- `src/agt/routePlans.asl` — `+!estimateEdgeCost/1` picks the best rendezvous
  with the **same weighting `!requestHandoverPlan` commits a real handover
  with**, and asserts `edgeCost(Neighbour, Distance, Candidate)`.
  `+!rankHandoverCosts` prices every neighbour and takes the `.min`, asserting
  `cheapestHandover/2`. `+!reportHandoverCosts` runs it at startup, but only for
  the arm that finds itself at the loaded support.

**Where the leg starts.** For an arm standing at A the answer is not its current
pose: it is going to pick the cube up first, so the leg it pays for runs from
the cube to the handover point. `edgeCostOrigin()` returns the grasp point of a
reachable, occupied site when there is one, and the live TCP otherwise. Both are
read, never assumed. This also settles the fourth decision below: the origin
does price its own first leg, and prices it from the right place.

**Why the weighting stays in the agent.** `interactionSpace.rank()` says in so
many words that "the weighted ranking itself deliberately remains in the BDI
agent". Pricing the best-*scoring* rendezvous rather than the nearest one is
what makes the cost describe the handover that will actually happen, so the
weights have to be applied where the handover chooses — above, not below.
`edgeCost.test.js` asserts the two copies of those weights have not drifted.

**Tests**
- `edgeCostArithmetic.test.js` — the real method against a stubbed lower layer:
  an arm at A pays from the cube (0.3 m) and not from its pose (1.2 m); an arm
  elsewhere pays from its pose; an unreachable loaded support is not a pickup;
  the distance is 3D; every rendezvous is priced and none is chosen; a failed
  ranking is a failed edge rather than a free one.
- `edgeCost.test.js` — the split holds: the bridge contains no weighting, the
  agent contains the same weighting as the handover, one unpriceable neighbour
  does not cost the arm the others, and no priced edge at all is a real outcome
  rather than a zero-cost one.

**A defect this found.** `Number(null)` is `0`, so an unreadable TCP was being
coerced into a perfectly finite point at the robot's own base — a plausible cost
for a rendezvous nobody would ever travel to, which is worse than no answer.
Absence now survives as absence up to the check that rejects it.

**Measured on the real cell.** From Robot1's cube grasp point at
`(0.9, 0, 0.1825)`, the best-scoring rendezvous with Robot2 is `candidate_18` at
`(1.128, 0.118, 0.414)` — **0.345 m**. That is the whole of Robot1's choice,
because of the next point.

**The cell is a chain, not a triangle.** Measured after the 2026-08-29
regeneration that moved Robot4 to `(1.5, -2)`, with all four workspaces
resampled, filtered to the configured `handoverHeightRange(0.3, 0.65)`:

| pair | voxels | ranked |
| --- | --- | --- |
| Robot1__Robot2 | 366 | 40 |
| Robot1__Robot3 | 1 | **none** |
| Robot2__Robot3 | 111 | 14 |
| Robot2__Robot4 | — | no geometric overlap |
| Robot3__Robot4 | 55 | **3** |

Robot1 and Robot3 are neighbours by the reach test but share a single voxel, so
the edge exists in the neighbour graph and carries no rendezvous a handover
could use. The usable topology is `1-2-3-4`, and the last hop has only three
viable rendezvous — thin, given `handoverRetriesLeft` starts at 2.

This removes the premise of the Stage 5 optimality demonstration, which assumed
two usable routes. A scan of Robot4 placements that would restore one:

| Robot4 at | Robot2 link | Robot3 link |
| --- | --- | --- |
| (1.8, -0.4) | 60 | 60 |
| (2.0, -0.2) | 60 | 60 |
| (1.6, -0.4) | 60 | 53 |

`(1.8, -0.4)` gives the diamond Stage 5 needs -- `1-2-4` at two hops against
`1-2-3-4` at three, both usable -- so cost can be made to disagree with hop
count. A placement decision, not a code one.

**Two arms holding A also mis-price one leg.** Robot2 has `hasSource`, so it
prices its leg from the cube rather than from its own pose -- but in execution
Robot2 is the RECEIVER from Robot1 and will never pick the cube up. Its leg
should be measured from where it stands. Harmless while both arms sit at the
same support and the numbers coincide; wrong as soon as they do not. Stage 3
should price from the cube only for the arm that will actually do the picking,
which is the arm the transfer goal was given to, not everyone who can reach A.

**Two arms can hold A.** Robot2 reaches the loaded support at 0.658 m with its
nearest sample 51 mm away, and its farthest reachable sample is 0.868 m, so this
is real reach and not a tolerance artifact. Stage 2 therefore cannot assume the
source is unique: whoever is given the transfer goal starts, and `hasSource` is
evidence about an arm rather than an election.

**Reachability is judged the way a pick actually approaches.** The resample left
Robot1's own grasp point 0.078 m from its nearest sample against a 0.08 m
tolerance -- just outside -- and Robot1 stopped recognising the support it had
been picking from all along. The grasp point is the wrong thing to test: samples
are sparse directly above a support precisely BECAUSE the support was a
collision obstacle while the workspace was sampled, and a pick never solves for
that point anyway. It arms the top-down planner, drops the support from the
obstacle set, solves a hover pose at `pickupApproachHeight` and descends
vertically. A site is now reachable if the grasp point is admitted **or** the
hover point is, reported as `admission: "hover_only"` so the weaker evidence
stays visible instead of being quietly folded into a wider tolerance.

**Task sites are obstacles for every arm.** Added in `cellHost.js` where the
obstacle list is assembled, not in the shared descriptor -- the pipeline rebuilds
`sharedEnvironment.collisionObstacles` from the wall aliases and the shared
support on every regeneration, so a descriptor edit would not survive one.

### Stage 2 — The flood — **implemented**

- `src/agt/routePlans.asl` — the protocol. `+!findRouteTo/1` starts a search,
  `+!answerFindRoute/5` answers one, `+routeCost/4` and `+noRoute/1` collect the
  replies, `+!settleRoute/1` decides, `+!answerUpstream/3` reports. Every arm
  ends up holding its own `nextHop/4`, which is what execution will read.
- `src/generator/agentThingServer.js` — `--mock-sites`, `--mock-costs` and
  `--mock-neighbors`, so the whole search runs on four bridges and a MAS with no
  simulator at all.
- `src/generator/routeSearch.js` — the same protocol as a deterministic message
  simulator, so its answers can be graded against a brute-force optimum on
  graphs nobody had to build.

**A protocol bug the model found before the cell could.** The arm holding the
goal was answering only the *first* arm to ask it, on the same already-seen rule
that stops the flood circling. But a goal-holder never forwards, so it cannot
loop — and if the first question happens to arrive across an edge it cannot hand
over on, it spends its one answer saying "no route", which was never a statement
about reachability. Every other arm that could have reached the goal through it
is then told there is no way there. The goal-holder clause now sits *above* the
already-seen clause in both the model and the plans, and
`routePlans.test.js` asserts that ordering.

**A second instance of the same mistake, found only by the live topology.** An
arm that cannot hand back to its requester -- because their shared space carries
no rendezvous at handover height -- was accepting the question anyway, consuming
its one-shot dedup, and then refusing the next arm to ask. In the real cell
Robot3 accepted Robot1 across the unusable `Robot1__Robot3` pair, then refused
Robot2's forward, and the route that exists (`R1-R2-R3-R4`) became invisible:
the cell reported *no route to a goal it can reach*. A refusal caused by an
unusable edge is a fact about that edge, never about the goal. The upstream edge
is now priced BEFORE the question is accepted, and the recovery leaves no
`routeSeen`, `routeRequestFrom` or `routeSettled` behind. `!ensureEdgePriced`
also stops an edge being re-ranked once per requester.

Two regression tests hold it: the live topology exactly, and its general form --
300 random graphs with about a fifth of the edges made unpriceable, asserting
that a reachable goal is never reported unreachable. That is the one failure
that cannot be excused as a heuristic limit, unlike the suboptimality below.

**Verified on the real cell.** Path `[robot2, robot3, robot4]`, total
`2.050052206486738`, accumulating one leg per arm:

| arm | own leg | reports upstream | own next hop |
| --- | --- | --- | --- |
| Robot4 (goal) | 0.50701 to R3 | 0.50701 | — |
| Robot3 | 0.80000 to R2 | 1.30701 | robot4 @ 0.50701 |
| Robot2 | 0.37152 to R1 | 1.67853 | robot3 @ 1.30701 |
| Robot1 (source) | 0.37152 to R2 | — | robot2, total 2.05005 |

Robot3 still answers Robot1 `none`, which is correct: that pair cannot carry a
handover. It simply no longer poisons the route through Robot2.

**What the model proves, and what it does not.** On trees the search finds the
true optimum, over 300 random topologies. On graphs with cycles it is never
below the optimum — a total lower than the true one would mean the accumulation
is wrong — but it is sometimes above it, and the test asserts that at least one
such case exists. That gap is the honest limit of first-seen dedup and exactly
what Stage 5's re-announcement is for; asserting it means implementing Stage 5
has to move it.

**Two bugs only a cold start could produce.** Everything above is about *what*
the arms say to each other. These two were about *when*, and both surfaced only
on a full restart of the whole cell, where all four arms flood at once and both
Robot1 and Robot2 believe they are standing at A — so two searches for the same
goal run concurrently and every arm is asked the same question twice, once
across an edge it can hand over on and once across one it cannot.

*The last resort settled a search it did not own.* An arm that cannot answer
must still answer, so `-!answerFindRoute` replies `noRoute` — but it also
asserted `routeSettled(Task)`, and that belief belongs to the *search*, not to
the requester. When Robot3's failed duplicate settled `route(robot1,…)`, the
sibling intention that was legitimately running that search lost its deadline,
`!settleIfComplete` stopped counting, and Robot4's answer of 0.506 — already
delivered, sitting in the belief base — was read by nobody. The cell reported no
route to the only empty support in it while holding the proof there was one. The
last resort now answers its requester and leaves the search alone.

*Pricing before accepting opened a window.* Priced-before-accepted is what keeps
an unusable edge from consuming an arm's one answer, but pricing takes seconds
of shared-space work and `routeSeen` is asserted only after it. A second copy of
the same question arriving inside that window found no `routeSeen` and started
its own search: two intentions forwarding, two `routeAsked` tallies counting
each other's answers, both racing to settle. A `routePricing(Task)` claim is now
taken before pricing; a duplicate waits for that attempt to resolve and then
asks again, so if the first attempt *refused* the second still gets its own
chance at the edge. The same window existed one layer down — two intentions
pricing the same edge and mode wipe each other's working beliefs on the way in,
which is how a perfectly usable edge got reported as unusable — and `edgePricing/2`
closes it the same way.

**`.wait` fails when it times out.** This one is worth writing down on its own,
because the whole cell is built on `.wait` and the assumption was backwards.
Jason 3.2.1's `.wait(Cond, Timeout)` does not resume the intention when the
deadline passes — it prepends `.fail` to the plan body (`jason.stdlib.wait$WaitEvent.resume`,
`wait_timeout`). Only the no-condition form `.wait(Time)` and the three-argument
form that reports elapsed time resume normally.

So `!routeDeadline` never worked. It reads

```
.wait(routeSettled(Task), Deadline);
if (not routeSettled(Task)) { !settleRoute(Task) }
```

and the `if` is unreachable: the only way past the wait is the belief arriving,
in which case the branch is false. A neighbour that went quiet failed
`!answerFindRoute` instead of settling — which is what produced Robot2's
otherwise inexplicable "Could not process the route question" on a search it
legitimately owned and had already been answered on. The deadline, the one
mechanism whose entire job is to cope with an answer that never comes, could
cope with everything except an answer that never came. The wait now sits in a
goal of its own whose failure handler does nothing, which turns the timeout back
into what it always meant.

The same shape was in the handover: the receiver's wait for the giver to report
parked took its whole handover down when the giver could not park — over a cube
that had already crossed. Every other `.wait` in the cell is paired with a
belief the runtime always asserts, success or failure, so a timeout there is a
genuine fault and failing is right.

#### How to test it

**1. Without a simulator or a MAS — `npm test`.** 317 tests. The ones that
matter here are `routeSearch.test.js` (protocol against brute force),
`routePlans.test.js` (plan structure: clause order, bounded waits, per-edge
pricing, one minimum per arm) and `mockMode.test.js`.

**2. Without a simulator, with a real MAS.** This is the level to use while
changing the protocol: four bridges and JaCaMo, no CoppeliaSim, no lower
runtime. The topology and the costs are whatever the test says they are.

A diamond where the cheap route and the short route are the same, and the
expensive one is 9x worse:

```bash
npm run robot:agent -- --manifest generated/robots/Robot1/manifest.json \
  --mock-neighbors "Robot2,Robot3" --mock-costs "Robot2=0.10,Robot3=0.90" \
  --mock-sites "Robot1_grip_support=holding:Robot1_grip_cube,Robot4_grip_support_b=out"

npm run robot:agent -- --manifest generated/robots/Robot2/manifest.json \
  --mock-neighbors "Robot1,Robot4" --mock-costs "Robot1=0.10,Robot4=0.10" \
  --mock-sites "Robot1_grip_support=out,Robot4_grip_support_b=out"

npm run robot:agent -- --manifest generated/robots/Robot3/manifest.json \
  --mock-neighbors "Robot1,Robot4" --mock-costs "Robot1=0.90,Robot4=0.90" \
  --mock-sites "Robot1_grip_support=out,Robot4_grip_support_b=out"

npm run robot:agent -- --manifest generated/robots/Robot4/manifest.json \
  --mock-neighbors "Robot2,Robot3" --mock-costs "Robot2=0.10,Robot3=0.90" \
  --mock-sites "Robot1_grip_support=out,Robot4_grip_support_b=empty"

./gradlew -Pjcm=generated/robots/Robot1_Robot2_Robot3_Robot4.jcm
```

Work the expected numbers out by hand first, then check them. Robot4 answers
Robot2 with its own leg 0.10 and Robot3 with 0.90 — *different numbers to
different requesters*, which is the per-edge pricing rule made visible. Robot2
adds its leg back to Robot1 and reports 0.20; Robot3 reports 1.80. Robot1 adds
its own first leg to each and compares 0.10 + 0.20 = **0.30 via Robot2** against
0.90 + 1.80 = 2.70 via Robot3.

Things worth deliberately breaking at this level, because each has a distinct
right answer:

| change | expected |
| --- | --- |
| drop `Robot4=...` from Robot2's costs | Robot2 answers `noRoute`; Robot1 routes the expensive way round |
| make Robot4's site `out` instead of `empty` | every branch returns `noRoute`; Robot1 reports no route rather than hanging |
| give Robot1 `--mock-neighbors ""` | "nobody to pass it to", not a stalled agent |
| give two arms an `empty` site | Robot1 refuses to guess and asks for `goalSite/1` |
| kill one bridge mid-search | the 90 s deadline fires and the search settles on what arrived |

**3. On the real cell.** Start the lower runtime, four bridges with no mock
flags, and the MAS. Read the AgentThing logs rather than gradle's stdout — agent
`.print` output goes to the MAS console. Confirm the search reaches Robot4 and
comes back, and compare the settled total against the edge costs the bridges
logged.

### Stage 3 — Execution follows `nextHop` — **implemented**

- `src/agt/handoverPlans.asl` — `+!continueHandoverChain` has three clauses in
  order: this arm's own `nextHop` (pass it on), `hasSink(Goal)` (place it), then
  the configured `handoverSuccessor` chain as a baseline. Each arm reads only
  its own next hop; nothing is ever sent the whole path.
- `+!placeCubeAt(Goal)` reads `site(Goal, X, Y, Z)` -- the position the arm
  detected for itself -- so moving the support moves the placement with it.
- `src/generator/lua/05-gripper.lua` + `03-scene-objects.lua` — the approach
  exemption is site-scoped: `womatSetPickupApproach(enabled, siteAlias)`.
- `src/generator/robwotBridge.js` — `--static-chain` gates the
  `handoverSuccessor` facts. Dynamic routing is the default.

**Placing is picking run backwards.** Same top-down planner: arm the approach,
descend vertically, release, disarm, retreat straight up. That is why the Lua
exemption had to become site-scoped -- a cube being lowered onto a support ends
in contact with it exactly as a freshly grasped one starts in contact with the
one it came off. Only the ONE block the approach is aimed at is dropped; every
other support in the cell stays hard, and the exemption is still gated on
actually holding the cube, so arming the flag alone permits nothing.

**The old terminal behaviour could not have worked.** `!returnCubeToSupport`
reads `supportApproachPosition/supportReleasePosition/supportRetreatPosition`,
and those facts exist in exactly one generated agent -- `robot2_agent.asl` --
because the two-robot demo only ever needed one. Robot4 has none, so the
terminal placement would have had no applicable plan at all.

**Two blockers the plan did not anticipate**, both found by reading rather than
running:

1. `+!handoverCubeTo` required `interactionSpaceOwner(MyRobot, ReceiverRobot)`.
   Ownership decides which arm BUILDS and caches a shared space -- once,
   deterministically, without negotiation. It was never meant to decide which
   arm may be the giver, and a route needing the cube passed toward a pair's
   non-owner needs exactly that. Replaced with
   `canGiveTo(Robot) :- neighbor(_, Robot)`; the space is refreshed inside the
   plan before any rendezvous is chosen, so requiring it in the guard only kept
   non-owners out.
2. The owner told its partner `interactionSpaceReady(Space, ReceiverRobot)`,
   which read in the partner's own frame says "the space with ME is ready" --
   true for the sender, useless to the receiver, and the receiver is the one who
   needs to know the space with the SENDER is ready before it can hand back.

**The pricing wrinkle is closed.** Only the arm that will actually pick the cube
up prices from the cube; every arm that RECEIVES it prices from where it stands.
`!ensureEdgePriced(Robot, Mode)` keys its cache on the mode, so a leg measured
from the cube is never handed back for a question about the arm's pose.

#### How to test it

**1. `npm test`** — 295 tests. `placeAtGoal.test.js` covers the placement path;
`routePlans.test.js` covers clause ordering and the opt-in baseline;
`handoverPlans.test.js` carries the rewritten ownership and frame invariants.

**2. Live, which is the level that matters** -- execution moves real arms and
cannot be mocked. Expect `Robot1` to pick and hand to `Robot2`, `Robot2` to go
upright with the cube and adopt the downstream intention, and so on to
`Robot4`, which places on `Robot4_grip_support_b`. Three things to watch:

| check | why it matters |
| --- | --- |
| the final descent onto the goal support survives preflight | the site-scoped exemption is the only thing making it possible |
| an intermediate arm goes upright with the cube BEFORE forwarding | `!handoverCubeTo` is guarded on `not handoverSession(_, _, _)`; a session left standing makes the forward inapplicable and it fails through without entering the plan |
| move the cube to another support and re-run | a different arm claims A and the route re-forms, with nothing regenerated |

**What the live cell does not yet exercise.** Its usable topology is the chain
`1-2-3-4`, and on that chain every giver happens to own its pair -- so the
non-owner giving path, which blocker 1 above exists for, is not tested by it.
A route running the other way, or the `(1.8, -0.4)` placement from Stage 1,
would exercise it.

#### What the cell taught us about leaving and arriving

Three faults the route search could never have shown, all of them about the
motion either side of a rendezvous rather than about which rendezvous.

**Nothing ranks the long leg.** `posturePenalty` scores the standoff leg into a
rendezvous — the last 15 cm along the pinned tool axis — and rejects candidates
whose approach crosses the arm's own base-joint axis. Nothing scores the journey
from wherever the arm happens to be parked. Robot4 refused its rendezvous at
sample 1 of 14, 3.6 cm into a 51 cm move, `baseVisible` against its own
`link_3`: a straight line out of a pose folded in near the base passes back over
the base. That is a fact about where the arm stood, not about the point, so
`!approachRendezvous` now resets the posture and asks the same point once more
before handing back a refusal — which otherwise costs the giver one of its two
candidates for a reason that was never the candidate's.

**Nothing let the giver out.** The giver leaves a rendezvous by interpolating in
joint space from the handover configuration to all-zero, the widest sweep either
arm makes, starting at the one point where the other arm is certain to be. The
receiver had a retreat; the giver had none. Both now back out by the same
`handoverBackoff`.

**A rendezvous is not retried after the cube has crossed.** The retry exists to
strike off a point the arms cannot make and take the next. Run after the
transfer, it reaches for a cube that is in somebody else's gripper, fails to
hold it, and reports "failed before session creation" over a handover that
physically succeeded — abandoning a delivered cube mid-chain. The post-transfer
case is now checked first and tells the receiver `giverStuck`, which is safe to
carry on from because the receiver's next move is preflighted against the giver
wherever it is actually standing.

### Stage 4 — End-to-end on the real cell

Run the full loop: detect A and B, discover the route, execute the handovers,
place the cube at B.

**Tests**
- Move `Robot1_grip_support` and the cube to a different robot's workspace in
  the scene and re-run **without regenerating anything**. A different agent must
  claim `hasSource` and the route must re-form. This is the single most
  convincing demonstration that the chain is no longer configured.
- Place B on `Robot2` instead of `Robot4`: route must shorten to `1-2`.
- Remove B entirely: every branch must answer `noRoute` and the origin must
  report "no route to goal" rather than hang. Check the timeouts fire.

### Stage 5 — True Bellman relaxation

Add `Epoch` to `routeCost`, re-announcement on improvement, and quiescence
detection at the origin.

**Tests**
- In `routeSearch.test.js`, construct graphs where the flood-tree answer is
  strictly worse than optimal and assert Stage 2 finds the worse one while
  Stage 5 finds the optimal one — the difference is the contribution.
- Live: pick costs in the `1-2-3` triangle so that `1-2-3-4` beats `1-3-4`
  despite being one hop longer, and confirm the cube physically takes the long
  way. Log both `D` values at each node so the relaxation is visible in the
  transcript.
- Bound the message count and assert it in the JS model; unbounded
  re-announcement is the classic failure mode here.

### Stage 6 — Re-planning on failure, and the TCP refinement

- When `-!handoverCubeTo` exhausts its retries, set `edgeCost(Task, J, inf)`
  for that neighbour and re-run the search from the current holder. A refused
  rendezvous then reroutes instead of aborting — the natural pay-off of having
  the costs distributed in the first place.
- Optional cost refinement: a node prices its outbound leg from its *inbound*
  rendezvous point rather than its current TCP, which removes the staleness
  noted in 1.2.

**Tests**
- Force a rendezvous refusal (shrink the `handoverHeightRange` for one pair, or
  drop a wall in the shared space) and assert the cube still reaches B by the
  other branch of the triangle.
- Compare total executed path length between the two cost models across ~5 runs
  and report the difference — a small, clean quantitative result.

---

## 3. Test ladder, summarised

| Level | Runs where | Covers |
| --- | --- | --- |
| `npm test` unit | no sim, no MAS | cost math, site classification, protocol model vs. Dijkstra |
| plan-text assertions | no sim, no MAS | ASL structure, guards, timeouts (pattern already used in `handoverPlans.test.js`) |
| mock-cost MAS | bridges + JaCaMo, no sim | the consensus protocol end to end, deterministically |
| live cell | CoppeliaSim + 4 bridges + JaCaMo | detection, real costs, physical execution, re-planning |

The mock-cost level is the one to build early. Every routing bug found there is
one not found at 40 minutes per simulator run.

## 4. Decisions

1. **Goal naming — settled.** B is a named support. `findRoute` carries
   `TargetSite` and the sink test is `hasSink(TargetSite)`, so "several empty
   supports exist" never makes the optimal-route claim ambiguous.
2. **Cost accumulation — settled.** One leg per robot, added to the total
   received from downstream and passed upstream, as specified in 1.2. Each node
   therefore decides by a bare `.min` over what arrived.
3. **Still open: whether to keep `handoverSuccessor` at all** once Stage 4
   passes, or keep it permanently behind `--static-chain` as the baseline for
   comparison runs. Recommendation: keep it — a configured chain and a
   discovered chain producing the same physical result is the cleanest evidence
   that the discovery is correct, and it costs one flag.
4. **Whether the origin prices its own first leg — settled.** It does, and from
   the cube rather than from its current pose, since it will pick the cube up
   before carrying it anywhere. Implemented in Stage 1 as `edgeCostOrigin()`.
