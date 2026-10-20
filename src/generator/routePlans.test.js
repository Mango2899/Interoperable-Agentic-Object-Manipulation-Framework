const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const plansPath = path.resolve(__dirname, "../agt/routePlans.asl");
const plans = fs.readFileSync(plansPath, "utf8");
const generator = fs.readFileSync(path.resolve(__dirname, "generateRobot.js"), "utf8");
const agentBridge = fs.readFileSync(path.resolve(__dirname, "agentThingServer.js"), "utf8");

test("detection is a lower-layer query the agent waits on, with a deadline", () => {
  assert.match(plans, /wotAchieve\("ownLayer1", "detectTaskSites", \[\]\)/);
  // Every .wait in this file must carry a deadline -- a literal or a variable
  // bound, either is a bound. An unbounded wait on a neighbour that never
  // answers is an agent that never reaches the rest of its init, and in the
  // search it is a branch that holds the whole cell open.
  const waits = plans.match(/\.wait\([^;]*?\);/gs) || [];
  assert.ok(waits.length > 0);
  for (const wait of waits) {
    assert.match(wait, /,\s*(\d+|[A-Z][A-Za-z0-9_]*)\s*\)/, `unbounded .wait: ${wait}`);
  }
});

test("a failed scan is reported rather than silently leaving the arm siteless", () => {
  assert.match(plans, /if \(taskSitesFailed\(Reason\)\)/);
  assert.match(plans, /-!detectTaskSites/);
});

test("stale beliefs are dropped before a rescan, so a moved cube cannot linger", () => {
  for (const belief of ["site\\(_, _, _, _\\)", "siteReachable\\(_\\)", "siteOccupied\\(_, _\\)", "siteEmpty\\(_\\)"]) {
    assert.match(plans, new RegExp(`\\.abolish\\(${belief}\\)`));
  }
});

test("A and B are derived, and both require this arm to actually reach the site", () => {
  assert.match(plans, /hasSource\(Site, Payload\) :- site\(Site, _, _, _\) & siteReachable\(Site\) & siteOccupied\(Site, Payload\)/);
  assert.match(plans, /hasSink\(Site\) :- site\(Site, _, _, _\) & siteReachable\(Site\) & siteEmpty\(Site\)/);
});

test("a site out of reach is still nameable, which is what a truthful 'not mine' needs", () => {
  assert.match(plans, /knownButUnreachable\(Site\) :- site\(Site, _, _, _\) & not siteReachable\(Site\)/);
  // The bridge asserts site/4 for every site it saw, gated on nothing.
  const siteBelief = agentBridge.indexOf('"site",');
  const reachableBelief = agentBridge.indexOf('if (site.reachable)');
  assert.notEqual(siteBelief, -1);
  assert.ok(siteBelief < reachableBelief, "site/4 must be asserted before, and independently of, reachability");
});

test("detection runs at startup, after the spaces the arm shares with its neighbours", () => {
  const shareSpaces = generator.indexOf("!shareInteractionSpaces;");
  const detect = generator.indexOf("!detectTaskSites;");
  const ready = generator.indexOf('.print("${name} agent ready")');
  assert.notEqual(detect, -1);
  assert.ok(shareSpaces < detect && detect < ready);
  assert.match(generator, /\{ include\("src\/agt\/routePlans\.asl"\) \}/);
});

test("every generated agent includes the route plans and runs the scan", () => {
  const directory = path.resolve(__dirname, "../agt/generated");
  const agents = fs.readdirSync(directory).filter((file) => file.endsWith("_agent.asl"));
  const wired = agents.filter((file) => {
    const source = fs.readFileSync(path.join(directory, file), "utf8");
    return source.includes('include("src/agt/routePlans.asl")') && source.includes("!detectTaskSites;");
  });
  // The four robots of the cell. YouBot predates the shared-cell init and is
  // deliberately left alone.
  for (const expected of ["robot1_agent.asl", "robot2_agent.asl", "robot3_agent.asl", "robot4_agent.asl"]) {
    assert.ok(wired.includes(expected), `${expected} is not wired for task-site detection`);
  }
});

// ---------------------------------------------------------------------------
// Stage 2: the search itself.
// ---------------------------------------------------------------------------

const search = fs.readFileSync(path.resolve(__dirname, "../agt/routePlans.asl"), "utf8");
const handoverPlans = fs.readFileSync(path.resolve(__dirname, "../agt/handoverPlans.asl"), "utf8");

function clauseOrder(head) {
  const positions = [];
  const pattern = new RegExp(`\\+!${head}\\([^)]*\\)[\\s\\S]*?\\n    : ([^<]*)<-`, "g");
  let match = pattern.exec(search);
  while (match) {
    positions.push(match[1].replace(/\s+/g, " ").trim());
    match = pattern.exec(search);
  }
  return positions;
}

test("holding the goal is checked before the already-seen rule", () => {
  // An arm holding the goal never forwards, so answering everyone cannot loop.
  // Answering only the first requester would let a question arriving across an
  // unhandoverable edge spend the goal-holder's one answer on a "no" that was
  // never about reachability. Found by routeSearch.test.js before it could be
  // found in the cell.
  const guards = clauseOrder("answerFindRoute");
  const sink = guards.findIndex((guard) => guard.includes("hasSink(Goal)"));
  const seen = guards.findIndex((guard) => guard.includes("routeSeen(Task)"));
  assert.ok(sink !== -1 && seen !== -1, `guards found: ${JSON.stringify(guards)}`);
  assert.ok(sink < seen, "the goal-holder clause must come before the already-seen clause");
});

test("a repeated question is refused, which is what bounds the flood", () => {
  assert.match(search, /\+!answerFindRoute\(Task, _, _, _, _, Upstream\)\s*\n\s*: routeSeen\(Task\)\s*<-\s*\n\s*\.send\(Upstream, tell, noRoute\(Task\)\)/);
  assert.match(search, /TTL <= 0/, "a hop budget must bound the flood even if dedup is defeated");
});

test("a question is never forwarded back to whoever asked", () => {
  assert.match(search, /neighbor\(A, R\) & A \\== Upstream/);
});

test("an arm that cannot answer still answers", () => {
  // Silence upstream is indistinguishable from a slow neighbour: the requester
  // would wait out its whole deadline for a branch that was never coming.
  assert.match(search, /-!answerFindRoute\([\s\S]*?\.send\(Upstream, tell, noRoute\(Task\)\)/);
  assert.match(search, /-!findRoute\([\s\S]*?\.send\(Upstream, tell, noRoute\(Task\)\)/);
});

test("a question arriving before this arm has looked at the cell waits", () => {
  assert.match(search, /not taskSitesReady\(_, _\)[\s\S]*?\.wait\(taskSitesReady\(_, _\), \d+\)/);
});

test("nothing is added to a value that arrives: the sender already added its leg", () => {
  const onArrival = search.slice(
    search.indexOf("+routeCost(Task, Options, Value, Path)[source(Downstream)]"),
    search.indexOf("+!settleIfComplete(Task)")
  );
  assert.match(onArrival, /\+routeAnswer\(Task, Downstream, Value, Path\)/);
  assert.match(onArrival, /\+routeOption\(Task, Downstream, Value, Path\)/,
    "and so must every alternative that travelled with it");
  assert.doesNotMatch(onArrival, /\+\s*Leg|edgeCost/, "the arriving total must be stored as it came");
});

test("the reply is priced per upstream edge, not once for everybody", () => {
  const answering = search.slice(search.indexOf("+!answerEach([Upstream | Rest], Task, Value, Path, Options)"));
  assert.match(answering, /!ensureEdgePriced\(UpstreamRobot, pose\)/);
  assert.match(answering, /Total = Leg \+ Value/);
  // Inside the per-upstream recursion, so two requesters get two numbers.
  assert.match(search, /!answerEach\(Rest, Task, Value, Path, Options\)/);
  // The alternatives are priced on the same edge, in the same place.
  assert.match(answering, /!priceOptions\(Options, Leg, \[\], Priced\)/);
});

test("only the arm that started the search adds its own first leg", () => {
  const origin = search.slice(
    search.indexOf("+!settleRoute(Task)\n    : routeOrigin(Task)"),
    search.indexOf("+!settleRoute(Task)\n    : not routeOrigin(Task)")
  );
  const carrier = search.slice(search.indexOf("+!settleRoute(Task)\n    : not routeOrigin(Task)"));
  assert.match(origin, /Total = Leg \+ Value/, "the origin has nobody upstream, so it pays its own first leg here");
  assert.doesNotMatch(carrier.slice(0, carrier.indexOf("+!settleRoute(_)")), /Total = Leg \+ Value/);
});

test("the decision is a minimum, at every arm", () => {
  assert.match(search, /\.min\(Totals, total\(Best, BestAgent, BestRobot, BestPath\)\)/);
  assert.match(search, /\.min\(Costs, cost\(BestValue, BestAgent, BestPath\)\)/);
});

test("every arm records its own next hop, not just the origin", () => {
  // Execution reads nextHop locally; a path held only by the origin would have
  // to be shipped back out to be acted on.
  assert.match(search, /\+nextHop\(Task, BestAgent, BestRobot, Best\)/);
  assert.match(search, /\+nextHop\(Task, BestAgent, BestRobot, BestValue\)/);
});

test("a branch that never answers cannot hold the search open forever", () => {
  assert.match(search, /\+!routeDeadline\(Task, Deadline\)[\s\S]*?\.wait\(routeSettled\(Task\), Deadline\)[\s\S]*?!settleRoute\(Task\)/);
  assert.match(search, /!routeDeadline\(Task, \d+\)/);
});

test("settling happens once, however it is triggered", () => {
  // The deadline and the last reply race each other; both call settleRoute.
  assert.match(search, /\+!settleRoute\(Task\)\s*\n\s*: routeOrigin\(Task\)[\s\S]*?not routeSettled\(Task\)/);
  assert.match(search, /\+!settleRoute\(Task\)\s*\n\s*: not routeOrigin\(Task\)\s*\n\s*& not routeSettled\(Task\)/);
  assert.match(search, /\+!settleRoute\(_\): true <- true\./, "a fallback so a second trigger is a no-op, not a failure");
});

test("stale route beliefs are dropped before a new search", () => {
  for (const belief of ["routeSeen", "routeRequestFrom", "routeAsked", "routeAnswer", "routeSettled", "nextHop", "noRouteTo"]) {
    assert.match(search, new RegExp(`\\.abolish\\(${belief}\\(`), `${belief} survives into the next search`);
  }
});

test("an ambiguous goal is refused rather than guessed", () => {
  // Two empty supports and no named goal is not a routing problem yet.
  assert.match(search, /\+!discoverRoute: hasSource\(_, _\) & goalSite\(Goal\)/);
  assert.match(search, /name the goal with goalSite\/1/);
});

test("the search runs at startup, after the arm knows what it can reach", () => {
  const detect = generator.indexOf("!detectTaskSites;");
  const price = generator.indexOf("!reportHandoverCosts;");
  const route = generator.indexOf("!discoverRoute;");
  assert.ok(detect < price && price < route, "sites, then prices, then the search");
  for (const agent of ["robot1_agent.asl", "robot2_agent.asl", "robot3_agent.asl", "robot4_agent.asl"]) {
    const source = fs.readFileSync(path.resolve(__dirname, "../agt/generated", agent), "utf8");
    assert.ok(source.includes("!discoverRoute;"), `${agent} does not run the search`);
  }
});

// One clause of the plan file, from its head to the '.' that closes it.
function clauseAt(head) {
  const start = search.indexOf(head);
  assert.ok(start >= 0, `no clause starting ${head}`);
  const end = search.indexOf("\n\n", start);
  return search.slice(start, end < 0 ? undefined : end);
}

test("a question is refused without being taken up when the reply could not be sent", () => {
  // The second instance of the same mistake as the goal-holder's, one hop out:
  // an arm that cannot hand back to its requester must say so WITHOUT consuming
  // its one-shot dedup, or an arm it could have served is refused next and a
  // real route disappears. Live cell: Robot3 accepted Robot1 across a pair with
  // no usable rendezvous, then refused Robot2's forward, and R1-R2-R3-R4 became
  // invisible.
  assert.match(
    search,
    /\+!takeUpRoute\(Task, Cube, Goal, TTL, Deadline, Upstream, _, UpstreamRobot\)\s*\n\s*: true\s*<-\s*\n\s*!ensureEdgePriced\(UpstreamRobot, pose\);\s*\n\s*\+routeSeen\(Task\)/,
    "the upstream edge must be priced before the question is accepted"
  );
  // And the recovery must leave no trace of a question it never took up.
  const untaken = clauseAt("-!takeUpRoute(Task, _, _, _, _, Upstream, _, UpstreamRobot)\n    : not routeSeen(Task)");
  assert.ok(untaken.length > 0, "there must be a clause for a question refused before it was accepted");
  assert.doesNotMatch(untaken, /\+routeSeen|\+routeSettled|\+routeRequestFrom/);
  assert.match(untaken, /\.abolish\(routePricing\(Task\)\)/, "the refusal must release the claim it took");
  assert.match(untaken, /\.abolish\(routeTakenUp\(Task, Upstream\)\)/, "and the claim token with it");
  assert.match(untaken, /\.send\(Upstream, tell, noRoute\(Task\)\)/);
  // Only the claimant may clear the claim. The old recovery hung off
  // -!answerFindRoute, so a sibling intention failing on its own edge wiped
  // routePricing out from under the arm that was mid-pricing -- reopening the
  // double take-up the claim exists to prevent.
  const lastResort = clauseAt("-!answerFindRoute(Task, _, _, _, _, Upstream): true");
  assert.doesNotMatch(lastResort, /\.abolish\(routePricing|\.abolish\(routeTakenUp/);
});

test("a second copy of one question does not start a second search", () => {
  // Pricing the upstream edge before accepting opens a window: routeSeen is not
  // asserted for seconds, so the same question arriving twice inside it was
  // taken up twice. Two intentions then forwarded the same question, counted
  // each other's answers against their own tally, and raced to settle.
  assert.match(
    search,
    /\+!answerFindRoute\(Task, Cube, Goal, TTL, Deadline, Upstream\)\s*\n\s*: routePricing\(Task\) & not routeDeferred\(Task, Upstream\)\s*<-\s*\n\s*\+routeDeferred\(Task, Upstream\);\s*\n\s*\.wait\(routeSeen\(Task\) \| not routePricing\(Task\), \d+\)/,
    "a duplicate must wait for the attempt already running"
  );
  // Waiting is not deciding: whichever way the first attempt goes, the second
  // asks again. If the first refused, this edge may still be priceable.
  const deferred = clauseAt("+!answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream)\n    : routePricing(Task)");
  assert.match(deferred, /!answerFindRoute\(Task, Cube, Goal, TTL, Deadline, Upstream\)\.$/m);
  // The dedup rule has to come first, or a settled task would be re-asked.
  assert.ok(
    search.indexOf("+!answerFindRoute(Task, _, _, _, _, Upstream)\n    : routeSeen(Task)") <
      search.indexOf("    : routePricing(Task) & not routeDeferred"),
    "already answered outranks still being weighed"
  );
});

test("taking a question up is an atomic claim, not a line of the plan that acts on it", () => {
  // Deferral only helps a copy that arrives after routePricing is set. Jason
  // checks a context in one reasoning step and runs the first body action in
  // the next, and interleaves freely between the two, so two copies arriving
  // in that gap both passed the context and both took the question up. Live
  // cell: robot4, asked by robot2 and by robot3 in the same instant, forwarded
  // route(robot1,..) twice and settled it on its siblings' refusals.
  const claim = clauseAt("@claimRoute[atomic]");
  assert.ok(claim.length > 0, "the claim has to be a plan of its own to be atomic");
  assert.match(
    claim,
    /: not routeSeen\(Task\)\s*\n\s*& not routePricing\(Task\)\s*\n\s*& not routeTakenUp\(Task, _\)/,
    "nothing may be claimed twice"
  );
  assert.match(claim, /\+routePricing\(Task\);\s*\n\s*\+routeTakenUp\(Task, Upstream\)\./);
  // Belief operations only: an atomic plan that waits on anything freezes the
  // arm for as long as it waits, and a subgoal is a wait in disguise.
  assert.doesNotMatch(claim.slice(claim.indexOf("<-")), /!\w|wotAchieve|\.wait|\.send/);
  // Losing the claim is not a failure, so the loser re-enters dispatch and is
  // caught by the deferral rule -- which is what should have happened to it.
  assert.match(search, /\+!claimRoute\(_, _\): true <- true\./);
  const dispatch = clauseAt("+!answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream)\n    : not routeTakenUp(Task, _)");
  assert.match(dispatch, /!claimRoute\(Task, Upstream\);\s*\n\s*!answerFindRoute\(Task, Cube, Goal, TTL, Deadline, Upstream\)\.$/m);
  // The winner comes back through dispatch holding routePricing, so its own
  // clause has to outrank the deferral rule or it would wait for itself.
  assert.ok(
    search.indexOf("+!answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream)\n    : routeTakenUp(Task, Upstream)") <
      search.indexOf("    : routePricing(Task) & not routeDeferred"),
    "the claimant must not be deferred behind its own claim"
  );
  // But not once the work is done: the token outlives the take-up, and a later
  // copy from the same upstream gets the ordinary already-seen answer.
  const winner = clauseAt("+!answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream)\n    : routeTakenUp(Task, Upstream)");
  assert.match(winner, /& not routeSeen\(Task\)/);
  // Dispatch is the last resort, so an exhausted or answered question never
  // reaches the claim at all.
  assert.ok(
    search.indexOf("    : not routeTakenUp(Task, _)") > search.indexOf("    : TTL <= 0"),
    "the hop and deadline budgets are checked before a question is claimed"
  );
});

test("a search is complete when every neighbour asked has answered, by name", () => {
  // Counting replies in bulk let one neighbour's two answers stand in for a
  // neighbour that had not answered at all -- and the branch still in flight
  // was always the slow one, because an already-seen refusal is instant while
  // a real leg costs the arm a rendezvous ranking first. Live cell: robot4
  // settled on three instant refusals while robot5, which held the goal, was
  // still pricing the leg that proved the route existed.
  // The question itself goes out through !askNeighbour, so that an arm nobody can
  // reach cannot stop the loop; the ORDER this protects is unchanged, and still
  // the point: written down first, asked second.
  assert.match(
    search,
    /\+!askNeighbours\(\[peer\(Agent, _\) \| Rest\], Task, Cube, Goal, TTL, Deadline\): true <-\s*\n\s*\+routeAskedPeer\(Task, Agent\);\s*\n\s*!askNeighbour\(Agent, Task, Cube, Goal, TTL, Deadline\)/,
    "a neighbour is written down before the question goes out, never after"
  );
  const settle = clauseAt("+!settleIfComplete(Task)\n    : routeAsked(Task, Asked)");
  assert.match(
    settle,
    /\.setof\(A, routeAskedPeer\(Task, A\) & routeAnswer\(Task, A, _, _\), Answered\)/,
    "the tally is over the neighbours asked, one entry each"
  );
  // How many were asked stays the number fixed before the first send. Deriving
  // it from the peers recorded so far moves the target: the first neighbour to
  // reply would find a list of one and settle the whole search on itself.
  assert.doesNotMatch(settle, /\.setof\(A, routeAskedPeer\(Task, A\), /);
  assert.match(settle, /if \(Got >= Asked\)/);
  assert.match(search, /\.abolish\(routeAskedPeer\(_, _\)\)/, "the tally is forgotten with the search");
  assert.match(search, /\.abolish\(routeTakenUp\(_, _\)\)/);
});

test("one neighbour is one verdict, and a leg outranks that neighbour's refusal", () => {
  // The refusal can come from an intention that had already spent its one
  // answer elsewhere, while the leg it was refusing was genuinely there.
  const upgrade = clauseAt("+routeCost(Task, Options, Value, Path)[source(Downstream)]\n    : routeAnswer(Task, Downstream, none, _)");
  assert.match(upgrade, /-routeAnswer\(Task, Downstream, none, _\);\s*\n\s*!acceptAnswer\(Task, Downstream, Options, Value, Path\)/);
  // One neighbour's answer replaces that neighbour's answer whole: keeping the
  // alternatives of a verdict that has been superseded would report routes the
  // arm no longer believes in.
  const accept = clauseAt("+!acceptAnswer(Task, Downstream, Options, Value, Path)");
  assert.match(accept, /\.abolish\(routeOption\(Task, Downstream, _, _\)\);\s*\n\s*\+routeAnswer\(Task, Downstream, Value, Path\)/);
  // A cheaper leg from the same neighbour replaces the dearer one; a dearer one
  // is dropped rather than counted twice.
  assert.match(search, /: routeAnswer\(Task, Downstream, Old, _\) & Old \\== none & Value < Old/);
  assert.match(
    search,
    /\+routeCost\(Task, _, _, _\)\[source\(Downstream\)\]\s*\n\s*: routeAnswer\(Task, Downstream, _, _\)\s*<-\s*\n\s*true\./,
    "a neighbour's second, worse answer is not a second answer"
  );
  // And a refusal never overwrites a leg already given.
  assert.match(
    search,
    /\+noRoute\(Task\)\[source\(Downstream\)\]\s*\n\s*: routeAnswer\(Task, Downstream, _, _\)\s*<-\s*\n\s*true\./,
    "a no from a neighbour that already answered changes nothing"
  );
});

test("failing to answer one requester does not settle everybody else's search", () => {
  // The arm that fails is usually not the arm that owns the search: the same
  // question reaches it twice, once across an edge it can hand over on and once
  // across one it cannot, and only the second copy fails. Live cell: Robot3
  // settled route(robot1,..) from its failed duplicate, so the deadline stopped
  // waiting and Robot4's answer of 0.506 was read by nobody.
  const lastResort = search.slice(search.indexOf("-!answerFindRoute(Task, _, _, _, _, Upstream): true"));
  assert.ok(lastResort.length > 0, "an arm that cannot answer must still answer");
  assert.doesNotMatch(
    lastResort.slice(0, lastResort.indexOf(".send(Upstream, tell, noRoute(Task)).")),
    /\+routeSettled/,
    "the last resort must not settle a task it may not own"
  );
  assert.match(lastResort, /\.send\(Upstream, tell, noRoute\(Task\)\)/);
});

test("an edge is priced once and reused, not re-ranked per requester", () => {
  assert.match(search, /\+!ensureEdgePriced\(Robot, Mode\): edgeCost\(Robot, Mode, _, _\) <- true\./);
  assert.match(search, /\+!ensureEdgePriced\(Robot, Mode\): true <-\s*\n\s*!estimateEdgeCost\(Robot, Mode\)\./);
  // A leg measured from the cube and one measured from the arm's pose are
  // different numbers, and an arm prices both concurrently -- once for its own
  // search, once to answer somebody else's. Every belief carries the mode so
  // one run's abolish cannot erase the other's result mid-flight.
  for (const belief of ["edgeCandidate", "edgeCostOrigin", "edgeCostReady", "edgeCostFailed", "edgeCost"]) {
    assert.match(search, new RegExp(`\\.abolish\\(${belief}\\(Neighbour, Mode,`), `${belief} is abolished across modes`);
  }
  assert.match(search, /\?edgeCost\(UpstreamRobot, pose, Leg, _\)/);
  assert.match(search, /edgeCost\(Robot, pickup, Leg, _\)/);
});

// ---------------------------------------------------------------------------
// Stage 3: the route drives execution.
// ---------------------------------------------------------------------------

test("the configured chain is opt-in, so a run that looks like routing is routing", () => {
  const bridgeSource = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  // handoverSuccessor facts would silently take over the moment a search came
  // back empty, and the run would look identical.
  assert.match(bridgeSource, /staticChain\s*\n?\s*\?\s*ownedPairs\.map/);
  assert.match(bridgeSource, /const staticChain = options\.staticChain === true/);
  assert.match(bridgeSource, /args\["static-chain"\] === true/);
});

test("no generated agent carries a configured successor by default", () => {
  const directory = path.resolve(__dirname, "../agt/generated");
  for (const agent of fs.readdirSync(directory).filter((file) => file.endsWith("_agent.asl"))) {
    const source = fs.readFileSync(path.join(directory, agent), "utf8");
    assert.doesNotMatch(source, /^handoverSuccessor\(/m, `${agent} still carries a configured chain`);
  }
});

test("the discovered route is consulted before the configured baseline", () => {
  const chain = handoverPlans.split("+!continueHandoverChain(Session, _)").slice(1);
  const guards = chain.map((clause) => clause.slice(0, clause.indexOf("<-")).replace(/\s+/g, " ").trim());
  const dynamic = guards.findIndex((guard) => guard.includes("nextHop("));
  const sink = guards.findIndex((guard) => guard.includes("hasSink(Goal)"));
  const configured = guards.findIndex((guard) => guard.includes("handoverSuccessor(NextAgent"));
  assert.ok(dynamic !== -1 && sink !== -1 && configured !== -1, JSON.stringify(guards));
  assert.ok(dynamic < sink, "an arm that can pass the cube on does so before checking if it holds the goal");
  assert.ok(sink < configured, "a discovered goal beats a configured successor");
});

test("every route found reaches the arm that decides, not just each hop's minimum", () => {
  // Every hop takes a minimum and forwards one value, so the arm holding the
  // cube heard about exactly one route however many the cell had found -- and
  // could not tell "this is the way" from "this is the cheapest of the ways".
  // Live cell: robot2 priced robot4 at 2.855 and robot3 at 3.042 and reported
  // only the first, so robot1 never saw that a second way round existed.
  const carrier = clauseAt("+!settleRoute(Task)\n    : not routeOrigin(Task)");
  assert.match(carrier, /\.findall\(option\(V, P\), openAnswer\(Task, V, _, P\), Direct\)/,
    "every neighbour's own verdict is a way through this arm");
  assert.match(carrier, /\.findall\(option\(V, P\),\s*\n\s*routeOption\(Task, A, V, P\) & not hopExcluded\(Task, A\)/,
    "and so is everything they reported from further out");
  // The answer itself is not also an alternative to itself.
  assert.match(carrier, /\.delete\(option\(BestValue, BestPath\), Every, Others\)/);
  assert.match(carrier, /!answerUpstream\(Task, BestValue, BestPath, Others\)/);
  // They travel inside the reply, so they cannot arrive after the answer that
  // completes the tally and settles the search upstream.
  assert.match(search, /\.send\(Upstream, tell, routeCost\(Task, Priced, Total, Path\)\)/);
  assert.match(search, /\.send\(Upstream, tell, routeCost\(Task, \[\], Leg, \[MyRobot\]\)\)/,
    "the arm holding the goal has no alternatives to report");
  // This arm joins every path it reports, not only the one it chose, and the
  // leg to a requester is added to all of them -- the same number for each, so
  // their order is untouched and the answer stays the cheapest.
  assert.match(search, /\+!prefixOptions\(\[option\(Value, Path\) \| Rest\], MyRobot, Acc, Out\)/);
  assert.match(search, /\+!priceOptions\(\[option\(Value, Path\) \| Rest\], Leg, Acc, Out\)/);
});

test("the origin reports what it found and did not choose, cheapest first", () => {
  const origin = search.slice(
    search.indexOf("+!settleRoute(Task)\n    : routeOrigin(Task)"),
    search.indexOf("+!settleRoute(Task)\n    : not routeOrigin(Task)")
  );
  // The decision is still made on the neighbours' own verdicts alone, and it is
  // the same decision: an alternative reported through a neighbour is never
  // cheaper than that neighbour's verdict, which is already in Totals.
  assert.match(origin, /\.min\(Totals, total\(Best, BestAgent, BestRobot, BestPath\)\)/);
  assert.ok(
    origin.indexOf("+nextHop(Task, BestAgent, BestRobot, Best)") < origin.indexOf("Alternatives"),
    "the alternatives must not be able to change the hop that was chosen"
  );
  // Each alternative is priced with this arm's own first leg across the
  // neighbour it came through, exactly as the chosen route is.
  assert.match(
    origin,
    /routeOption\(Task, Agent, Value, Path\)[\s\S]*?& edgeCost\(Robot, pickup, Leg, _\)\s*\n\s*& Total = Leg \+ Value/
  );
  assert.match(origin, /\.concat\(Totals, Alternatives, Every\);\s*\n\s*\.sort\(Every, Ranked\)/);
  assert.match(origin, /route\(s\) to the goal, cheapest first/);
  assert.match(search, /\+!printRoutes\(\[total\(Total, _, _, Path\) \| Rest\]\)/);
  assert.match(search, /\.abolish\(routeOption\(_, _, _, _\)\)/, "forgotten with the search");
});

test("an arm reads only its own next hop, never a path", () => {
  // Nothing is ever sent the whole route. Each arm was told one thing by its
  // downstream neighbour and worked out one thing for itself.
  assert.match(handoverPlans, /nextHop\(_, NextAgent, NextRobot, _\)/);
  assert.doesNotMatch(handoverPlans, /routeTotal|\[robot/);
});

test("acting on the route is separate from discovering it", () => {
  // The search settles in seconds; the transfer takes minutes of real motion.
  // The cell works the route out at startup and waits to be told to go.
  assert.match(search, /\+!deliverCube\s*\n\s*: nextHop\(_, NextAgent, NextRobot, Total\)[\s\S]*?!handoverCubeTo\(NextAgent, NextRobot\)/);
  assert.doesNotMatch(search, /!discoverRoute;[\s\S]{0,200}!deliverCube/);
});

test("delivery has an answer for every state the search can leave behind", () => {
  // A route, no route, already at the goal, and not searched yet are four
  // different situations and only one of them is an error.
  assert.match(search, /\+!deliverCube\s*\n\s*: hasSink\(Goal\)[\s\S]*?!placeCubeAt\(Goal\)/);
  assert.match(search, /\+!deliverCube: noRouteTo\(_\)/);
  assert.match(search, /\+!deliverCube: true <-[\s\S]*?run !discoverRoute first/);
  assert.match(search, /-!deliverCube: true <-/);
});

test("one call to the arm holding the cube moves it the whole way", () => {
  const bridgeSource = fs.readFileSync(path.resolve(__dirname, "agentThingServer.js"), "utf8");
  assert.match(bridgeSource, /setActionHandler\("deliverCube", async \(\) => \{\s*\n\s*this\.forwardAchieveToAgent\("deliverCube"\);/);
  assert.match(generator, /deliverCube: \{[\s\S]*?href: "\/actions\/deliverCube"/);
  // Declared on every generated agent, not just the one that happens to hold A
  // today -- moving the cube changes which arm that is.
  const directory = path.resolve(__dirname, "../td/generated");
  for (const file of fs.readdirSync(directory).filter((name) => name.endsWith(".agent.td.json"))) {
    const td = JSON.parse(fs.readFileSync(path.join(directory, file), "utf8"));
    assert.ok(td.actions.deliverCube, `${file} cannot be told to deliver`);
  }
});

test("an arm that can see the goal but not serve it says which of the two it is", () => {
  // knownButUnreachable/1 was defined, tested for, and used by nothing. The one
  // case it exists for is the one where it is most needed: the whole cell
  // answers "no route" because the goal is out of everybody's reach, and the
  // only trace is an arm reporting it has nobody to forward to -- true, and
  // silent about the reason. Live cell: Robot4's support sat 0.0866 m outside
  // its cloud against a 0.08 tolerance, and nothing said so.
  assert.match(
    search,
    /if \(knownButUnreachable\(Goal\) & site\(Goal, GX, GY, GZ\)\) \{\s*\n\s*\.print\("I can see ", Goal, " at \("/,
    "the arm has to name the site and where it thinks it is"
  );
  // In its OWN frame, which is the frame that decides reachability -- a world
  // coordinate would not show that the site is behind the base.
  assert.match(search, /\) in my own frame but cannot reach it"\)/);
  // Said before the forwarding decision, so it is reported whether the arm has
  // somewhere to pass the question or not.
  const accept = search.slice(search.indexOf("+!takeUpRoute(Task, Cube, Goal, TTL, Deadline, Upstream, _, UpstreamRobot)"));
  assert.ok(
    accept.indexOf("knownButUnreachable(Goal)") < accept.indexOf("if (Peers == []) {"),
    "the reason has to be printed whether or not the branch ends here"
  );
});

test("a price says how much choice there was behind it", () => {
  // A pair whose shared space is one voxel prices exactly as confidently as one
  // with three hundred, and the search reads the two numbers identically. Live
  // cell: the Robot1-Robot3 overlap came back as a SINGLE voxel, the route was
  // settled through it, and nothing in the log said the pair had one rendezvous
  // in the world -- with no second candidate for the retry to fall back on.
  assert.match(search, /\.length\(Rankings, Choices\);\s*\n\s*if \(Choices <= 2\) \{/);
  assert.match(search, /usable rendezvous in total; a route through ",\s*\n\s*Neighbour, " has nothing to fall back on/);
  // Said after the price, not instead of it: the edge is still usable and the
  // number is still real.
  assert.ok(
    search.indexOf("!reportEdgeCost(Neighbour, Mode, BestDistance, BestCandidate);") < search.indexOf(".length(Rankings, Choices)"),
    "the warning annotates the price rather than replacing it"
  );
});

// ---------------------------------------------------------------------------
// Rerouting: a hop that was settled on and then turned out to be unusable.
// ---------------------------------------------------------------------------

test("which answers an arm may still act on is one rule, used everywhere a hop is chosen", () => {
  // A hop is struck, not forgotten, so the reason survives in the transcript and
  // the same arm is not chosen again a minute later. Two kinds of striking,
  // because they are learnt in two places: this arm's own refusal is per search,
  // and an arm struck further up the route arrives with the cube.
  assert.match(plans, /openAnswer\(Task, Value, Agent, Path\) :-\s*\n\s*routeAnswer\(Task, Agent, Value, Path\)\s*\n\s*& Value \\== none\s*\n\s*& not hopExcluded\(Task, Agent\)\s*\n\s*& neighbor\(Agent, Robot\)\s*\n\s*& not outOfService\(Robot\)/);
  // Two reasons an arm is out, one rule, so no decision can honour one and miss
  // the other: a refused handover, and an arm that says it cannot serve at all.
  assert.match(plans, /outOfService\(Robot\) :- armUnfit\(Robot\)\./);
  assert.match(plans, /outOfService\(Robot\) :- armUnavailable\(Robot\)\./);
  // Every place a next hop is settled on reads it, or a struck hop is struck in
  // one of them and live in the next.
  const decisions = plans.match(/\.findall\(total\(|\.findall\(cost\(/g) || [];
  assert.ok(decisions.length >= 4, "the origin and every other arm, at settling and at rerouting");
  assert.doesNotMatch(
    plans,
    /routeAnswer\(Task, Agent, Value, Path\) & Value \\== none\s*\n\s*& neighbor/,
    "no decision may read routeAnswer directly and miss the strikes"
  );
  assert.doesNotMatch(plans, /not armUnfit\(R\w*\)[\s\S]{0,80}edgeCost/,
    "and none may check armUnfit alone where outOfService is meant");
});

test("a fresh search does not inherit the last one's strikes", () => {
  const forget = clauseAt("+!forgetRoutes: true <-");
  for (const belief of ["hopExcluded\\(_, _\\)", "armUnfit\\(_\\)", "rerouteAttempted\\(_\\)"]) {
    assert.match(forget, new RegExp(`\\.abolish\\(${belief}\\)`),
      "an arm refused in one delivery must be priceable again in the next");
  }
});

test("a struck hop is replaced from what the arm already collected, with no new messages", () => {
  const reroute = clauseAt("+!rerouteAround(FailedAgent, FailedRobot)");
  // The hop goes, the reason stays, and the choice is remade.
  assert.match(reroute, /\+hopExcluded\(Task, FailedAgent\)/);
  assert.match(reroute, /-nextHop\(Task, FailedAgent, FailedRobot, Total\)/);
  assert.match(reroute, /!chooseAnotherHop\(Task\)/);
  // Nothing is asked again. The alternatives were priced by the search that
  // chose the route and came home inside the same replies, so the recovery is
  // arithmetic on beliefs the arm already holds -- which is the whole pay-off of
  // having accumulated the costs instead of collecting paths centrally.
  const rerouting = plans.slice(plans.indexOf("// ---- rerouting"), plans.indexOf("// Price an edge unless"));
  assert.ok(rerouting.length > 0, "the rerouting section has to exist");
  assert.doesNotMatch(rerouting, /\.send\(/, "a reroute sends nothing");
  assert.doesNotMatch(rerouting, /findRoute|routeAsked|\.wait\(/, "and re-floods nothing");
});

test("the origin reprices its own first leg on a reroute; every other arm does not", () => {
  // The origin is the one arm that pays a leg no accumulated total contains, so
  // its re-decision has to add that leg back exactly as the settling did -- and
  // measured from the cube, which has not moved if the origin is still holding it.
  const asOrigin = clauseAt("+!chooseAnotherHop(Task)\n    : routeOrigin(Task)");
  assert.match(asOrigin, /!priceReachedEdges\(Reached\)/);
  assert.match(asOrigin, /edgeCost\(Robot, pickup, Leg, _\)\s*\n\s*& Total = Leg \+ Value/);
  // Everywhere else the number that arrived is already the whole remaining cost,
  // so re-deciding is a bare minimum over one fewer candidate.
  const asCarrier = clauseAt("+!chooseAnotherHop(Task): agentIdentity(_, MyRobot) <-");
  assert.match(asCarrier, /openAnswer\(Task, Value, Agent, Path\) & neighbor\(Agent, Robot\)/);
  assert.doesNotMatch(asCarrier, /edgeCost|Leg \+ Value/);
});

test("a reroute that found nothing leaves no next hop, and never fails", () => {
  // Execution reads nextHop and nothing else, so leaving none behind is how a
  // reroute says it found none -- and the arm falls through to its terminal
  // behaviour on its own. A failing reroute would instead take down the
  // intention that called it, which on the chain is the delivery itself.
  assert.match(plans, /\+!adoptCheapest\(_, MyRobot, \[\]\): true <-\s*\n\s*\.print\([^;]*?\)\.\s*\n/,
    "the empty case reports and stops; it does not .fail");
  const adopt = clauseAt("+!adoptCheapest(Task, MyRobot, Totals): true <-");
  assert.match(adopt, /\.min\(Totals, total\(Best, BestAgent, BestRobot, BestPath\)\)/);
  assert.match(adopt, /\+nextHop\(Task, BestAgent, BestRobot, Best\)/);
  // How many were left is part of what the choice means.
  assert.match(adopt, /\.length\(Totals, HowMany\)/);
  for (const goal of ["rerouteAround\\(_, FailedRobot\\)", "chooseAnotherHop\\(_\\)"]) {
    assert.match(plans, new RegExp(`-!${goal}: true <-`), "and every reroute goal has a contingency");
  }
});

test("an arm that cannot be asked does not take the other branches down with it", () => {
  // Jason FAILS .send when the receiver is not in the MAS, so the send sits in a
  // goal of its own: inline in the loop, one unaskable arm stopped the recursion
  // and the neighbours further down the list were never asked at all.
  assert.match(plans, /\+!askNeighbour\(Agent, Task, Cube, Goal, TTL, Deadline\): true <-\s*\n\s*\.send\(Agent, achieve, findRoute\(Task, Cube, Goal, TTL, Deadline\)\)\./);
  const loop = clauseAt("+!askNeighbours([peer(Agent, _) | Rest], Task, Cube, Goal, TTL, Deadline): true <-");
  assert.match(loop, /!askNeighbour\(Agent, Task, Cube, Goal, TTL, Deadline\);\s*\n\s*!askNeighbours\(Rest/);
  assert.doesNotMatch(loop, /\.send\(/, "the loop itself must not be able to fail on a send");

  const recovery = clauseAt("-!askNeighbour(Agent, Task, _, _, _, _): true <-");
  assert.match(recovery, /!refuseOnBehalfOf\(Agent, Task\)/);
  // Answering on the absent arm's behalf is what keeps the tally reachable: it
  // was counted into routeAsked before the send went out, so a branch left
  // without a verdict means the search can only end by running out of time.
  const refuse = clauseAt("+!refuseOnBehalfOf(Agent, Task): true <-");
  assert.match(refuse, /\+routeAnswer\(Task, Agent, none, \[\]\)/);
  assert.match(refuse, /!settleIfComplete\(Task\)/);
  assert.match(refuse, /\+routeUnreachable\(Task, Agent\)/, "and which arm it was stays in the record");
  assert.match(plans, /-!refuseOnBehalfOf\(Agent, _\): true <-/,
    "called from a failure handler, so it must not be able to fail");
  assert.match(clauseAt("+!forgetRoutes: true <-"), /\.abolish\(routeUnreachable\(_, _\)\)/);
});

test("an arm whose own layer never answered says so once, and is not asked again", () => {
  // Three timeouts used to stand in for one fact. An arm whose AgentThing is
  // down spent 30s failing to scan for neighbours, 20s failing to look for task
  // sites, and then made every neighbour that asked it about a route wait a
  // further 30s on taskSitesReady -- a belief that could no longer be asserted.
  // Nothing here names a robot: it is whichever arm finds its own layer silent.
  assert.match(plans, /cannotServe :- ownLayerUnresponsive\(_\)\./);

  // Announced, so the other arms never ask. Broadcast rather than sent to the
  // neighbours, because the neighbour scan is one of the things that failed.
  const announce = clauseAt("+!announceUnavailable(Step)\n    : agentIdentity(_, MyRobot) & not unavailabilityAnnounced");
  assert.match(announce, /\+ownLayerUnresponsive\(Step\)/);
  assert.match(announce, /\.broadcast\(tell, armUnavailable\(MyRobot\)\)/);
  assert.match(announce, /\+unavailabilityAnnounced/, "once, not once per failed step");
  // And the belief is recorded even when the announcement cannot be made, since
  // it is what stops this arm waiting on itself.
  assert.match(plans, /\+!announceUnavailable\(Step\): true <-\s*\n\s*\+ownLayerUnresponsive\(Step\)\./);
  assert.match(plans, /-!announceUnavailable\(_\): true <-/);
  // Both steps that talk to the layer report it.
  assert.match(plans, /!announceUnavailable\(task_sites\)/);
  const neighbours = fs.readFileSync(path.resolve(__dirname, "../agt/neighborPlans.asl"), "utf8");
  assert.match(neighbours, /!announceUnavailable\(neighbour_scan\)/);

  // Asked anyway -- by an arm that had already sent, or had not heard -- the
  // answer is immediate. Waiting on taskSitesReady when the layer is gone can
  // only end in the deadline, and spends the REQUESTER's budget doing it.
  const atOnce = clauseAt("+!findRoute(Task, _, Goal, _, _)[source(Upstream)]\n    : not routeSeen(Task) & cannotServe");
  assert.match(atOnce, /\.send\(Upstream, tell, noRoute\(Task\)\)/);
  assert.match(atOnce, /\+routeSeen\(Task\)/, "so a second copy is refused instantly too");
  assert.doesNotMatch(atOnce, /\.wait\(/);
  // And it is checked BEFORE the clause that waits, or it never runs.
  assert.ok(
    plans.indexOf("& not routeSeen(Task) & cannotServe") < plans.indexOf("waiting to answer honestly"),
    "the arm that cannot serve must answer before the arm that is merely slow waits"
  );

  // An arm known to be out is not sent the question at all.
  const skip = clauseAt("+!askNeighbour(Agent, Task, _, _, _, _)\n    : neighbor(Agent, Robot) & armUnavailable(Robot)");
  assert.match(skip, /!refuseOnBehalfOf\(Agent, Task\)/);
  assert.doesNotMatch(skip, /\.send\(/);
  assert.ok(
    plans.indexOf("& neighbor(Agent, Robot) & armUnavailable(Robot)")
      < plans.indexOf("+!askNeighbour(Agent, Task, Cube, Goal, TTL, Deadline): true <-"),
    "the skip must be checked before the send"
  );
});
