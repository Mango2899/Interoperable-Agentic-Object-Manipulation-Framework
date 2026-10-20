"use strict";

// What one handover edge costs the arm that would perform it.
//
// The geometry is priced below and the choice is made above, the same split the
// candidate ranking already uses. These tests hold both halves to that: the
// bridge must price every viable rendezvous without choosing one, and the agent
// must choose with the weighting it actually commits handovers with.

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const plans = fs.readFileSync(path.resolve(__dirname, "../agt/routePlans.asl"), "utf8");
const handoverPlans = fs.readFileSync(path.resolve(__dirname, "../agt/handoverPlans.asl"), "utf8");
const bridge = fs.readFileSync(path.resolve(__dirname, "agentThingServer.js"), "utf8");
const generator = fs.readFileSync(path.resolve(__dirname, "generateRobot.js"), "utf8");

function weightsIn(source) {
  const match = source.match(
    /Score = ([\d.]+) \* Centrality \+ ([\d.]+) \* Density\s*\n\s*\+ ([\d.]+) \* GiverSupport \+ ([\d.]+) \* ReceiverSupport \+ ([\d.]+) \* Posture/
  );
  assert.ok(match, "no candidate weighting found");
  return match.slice(1).map(Number);
}

test("an edge is priced with the same weighting a handover is committed with", () => {
  // Two copies of these numbers exist on purpose -- the ranking deliberately
  // lives in the BDI layer -- so drift between them is what this catches. A
  // cost computed for a rendezvous the handover would not select is a cost for
  // something that never happens.
  assert.deepEqual(weightsIn(plans), weightsIn(handoverPlans));
  assert.deepEqual(weightsIn(plans), [0.32, 0.2, 0.14, 0.14, 0.2]);
});

test("the bridge prices every candidate and chooses none", () => {
  const method = bridge.slice(bridge.indexOf("async estimateEdgeCost("), bridge.indexOf("async discoverNeighbors()"));
  assert.match(method, /for \(const candidate of ranked\.candidates\)/);
  assert.match(method, /"edgeCandidate"/);
  // The weighting must not appear below: choosing here would silently override
  // the agent's judgement with a second, invisible ranking.
  assert.doesNotMatch(method, /0\.32/);
  assert.doesNotMatch(method, /centrality \*/);
});

test("the cost is the distance from this arm's own start to its own approach point", () => {
  const method = bridge.slice(bridge.indexOf("async estimateEdgeCost("), bridge.indexOf("async discoverNeighbors()"));
  assert.match(method, /Math\.hypot\(/);
  assert.match(method, /candidate\.giver\?\.localTcp/);
  assert.match(method, /origin\.point\.x/);
});

test("only the arm that will pick the cube up prices from the cube", () => {
  const origin = bridge.slice(bridge.indexOf("async edgeCostOrigin(mode"), bridge.indexOf("async readTaskSites()"));
  // Reachability of the loaded support does NOT decide this. Two arms can reach
  // it; only one of them is going to pick the cube up, and every other arm on
  // the route receives it and pays from where it stands. The caller says which.
  assert.match(origin, /if \(mode === "pose"\)/);
  assert.ok(origin.indexOf('if (mode === "pose")') < origin.indexOf('kind: "pickup"'));
  // And the pickup still has to be a site this arm can serve that holds it.
  assert.match(origin, /site\.reachable/);
  assert.match(origin, /site\.occupiedBy !== "none"/);
});

test("any arm can price any edge it touches, owned pair or not", () => {
  const method = bridge.slice(bridge.indexOf("async estimateEdgeCost("), bridge.indexOf("async discoverNeighbors()"));
  // Ownership decides who builds and caches a space, never who may ask what an
  // edge would cost them.
  assert.match(method, /synthesizePair: true/);
  assert.match(method, /interaction_space_not_ready/);
  assert.doesNotMatch(method, /interactionSpaceOwner/);
});

test("pricing one neighbour cannot cost the arm the others", () => {
  assert.match(plans, /-!priceEdges\(\[Neighbour \| Rest\]\)[\s\S]*?!priceEdges\(Rest\)/);
});

test("the minimum is taken over the priced edges, and only over those", () => {
  assert.match(plans, /\.findall\(cost\(C, N\), edgeCost\(N, pickup, C, _\), Costs\)/);
  assert.match(plans, /\.min\(Costs, cost\(BestCost, BestNeighbour\)\)/);
  assert.match(plans, /\+cheapestHandover\(BestNeighbour, BestCost\)/);
  // No priced edge at all is a real outcome, not a zero-cost one.
  assert.match(plans, /if \(Costs == \[\]\)[\s\S]*?\.fail/);
});

test("stale prices are dropped before an edge is re-priced", () => {
  // Keyed by mode: an arm prices the same edge from the cube and from its pose
  // concurrently, and wiping the neighbour's other mode would erase a price
  // another intention is in the middle of reading.
  for (const belief of [
    "edgeCandidate\\(Neighbour, Mode, _, _, _, _, _, _, _\\)",
    "edgeCostOrigin\\(Neighbour, Mode, _, _, _, _\\)",
    "edgeCost\\(Neighbour, Mode, _, _\\)"
  ]) {
    assert.match(plans, new RegExp(`\\.abolish\\(${belief}\\)`));
  }
});

test("only the arm holding the cube prices its options at startup", () => {
  assert.match(plans, /\+!reportHandoverCosts: hasSource\(Site, Payload\)/);
  assert.match(plans, /\+!reportHandoverCosts: not hasSource\(_, _\) <- true\./);
  const detect = generator.indexOf("!detectTaskSites;");
  const price = generator.indexOf("!reportHandoverCosts;");
  assert.ok(detect !== -1 && price !== -1);
  assert.ok(detect < price, "sites must be known before edges can be priced from them");
});

test("every generated agent prices its handovers at startup", () => {
  const directory = path.resolve(__dirname, "../agt/generated");
  for (const agent of ["robot1_agent.asl", "robot2_agent.asl", "robot3_agent.asl", "robot4_agent.asl"]) {
    const source = fs.readFileSync(path.join(directory, agent), "utf8");
    assert.ok(source.includes("!reportHandoverCosts;"), `${agent} does not price its handovers`);
  }
});
