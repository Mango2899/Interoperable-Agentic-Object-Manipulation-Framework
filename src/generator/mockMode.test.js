"use strict";

// The simulator stand-in.
//
// The consensus protocol is agents talking to agents; none of it needs a robot.
// Reaching it through CoppeliaSim means a scene load and a full cell startup per
// protocol change, so these flags let the whole search run on four bridges and a
// MAS with nothing behind them. What the tests below protect is that mock mode
// is a substitution and not a second code path: the agent must weigh and choose
// exactly as it does for real, and mock mode must never silently activate.

const assert = require("node:assert/strict");
const test = require("node:test");

const { GeneratedRobotAgentThing, parseMockCosts, parseMockSites } = require("./agentThingServer");

test("a site spec names what each support is to this arm", () => {
  const sites = parseMockSites("Robot1_grip_support=holding:Robot1_grip_cube,Robot4_grip_support_b=out");
  assert.equal(sites.length, 2);
  assert.deepEqual(
    sites.map((site) => [site.alias, site.reachable, site.occupiedBy]),
    [["Robot1_grip_support", true, "Robot1_grip_cube"], ["Robot4_grip_support_b", false, "none"]]
  );
});

test("an empty support is reachable and holds nothing", () => {
  const [site] = parseMockSites("Robot4_grip_support_b=empty");
  assert.equal(site.reachable, true);
  assert.equal(site.occupiedBy, "none");
});

test("a state left off defaults to an empty, reachable support", () => {
  const [site] = parseMockSites("Robot4_grip_support_b");
  assert.equal(site.reachable, true);
  assert.equal(site.occupiedBy, "none");
});

test("costs are read per neighbour, and a bad one is dropped rather than guessed", () => {
  const costs = parseMockCosts("Robot2=0.4,robot3=0.9,Robot9=notanumber");
  assert.equal(costs.get("Robot2"), 0.4);
  assert.equal(costs.get("Robot3"), 0.9, "a lowercase name is the same robot");
  assert.equal(costs.has("Robot9"), false);
});

test("no flag means no mock: the simulator is the only source", () => {
  assert.equal(parseMockSites(undefined), null);
  assert.equal(parseMockCosts(undefined), null);
  // A bare flag with no value must not silently enable an empty mock.
  assert.equal(parseMockSites(true), null);
  assert.equal(parseMockCosts(true), null);
});

function mockBridge({ sites, costs }) {
  const beliefs = [];
  const bridge = Object.create(GeneratedRobotAgentThing.prototype);
  bridge.ctx = { manifest: { name: "Robot1" } };
  bridge.mockSites = sites ? parseMockSites(sites) : null;
  bridge.mockCosts = costs ? parseMockCosts(costs) : null;
  bridge.mockNeighbors = null;
  // Any mock flag means there is no cell behind this bridge, exactly as the
  // constructor decides it.
  bridge.mocked = Boolean(bridge.mockSites || bridge.mockCosts || bridge.mockNeighbors);
  bridge.readLowerProperty = async () => {
    throw new Error("the simulator must not be consulted in mock mode");
  };
  bridge.interactionSpace = {
    rank: () => {
      throw new Error("the interaction-space estimator must not be consulted in mock mode");
    }
  };
  bridge.addBelief2Agent = async (name, args) => beliefs.push({ name, args });
  return { bridge, beliefs };
}

test("mocked detection reaches the agent without touching the simulator", async () => {
  const { bridge, beliefs } = mockBridge({
    sites: "Robot1_grip_support=holding:Robot1_grip_cube,Robot4_grip_support_b=out"
  });
  await bridge.detectTaskSites();
  const named = (name) => beliefs.filter((belief) => belief.name === name).map((belief) => belief.args);
  // asJasonAtom prefixes a name that does not start lowercase, so the alias
  // Robot1_grip_support becomes the atom v_Robot1_grip_support. The bridge
  // keeps the mapping back, which is what lets a goal carrying the atom still
  // name the object.
  assert.deepEqual(named("site").map((args) => args[0]), ["v_Robot1_grip_support", "v_Robot4_grip_support_b"]);
  assert.deepEqual(named("siteReachable").map((args) => args[0]), ["v_Robot1_grip_support"]);
  assert.deepEqual(named("siteOccupied")[0], ["v_Robot1_grip_support", "v_Robot1_grip_cube"]);
  assert.deepEqual(named("siteEmpty").map((args) => args[0]), ["v_Robot4_grip_support_b"]);
  assert.deepEqual(named("taskSitesReady")[0], [2, 1]);
});

test("a mocked edge is still weighed and chosen by the agent, not by the flag", async () => {
  const { bridge, beliefs } = mockBridge({ costs: "Robot2=0.4" });
  await bridge.estimateEdgeCost("Robot2", 0.3, 0.65);
  const candidates = beliefs.filter((belief) => belief.name === "edgeCandidate");
  assert.equal(candidates.length, 1);
  assert.equal(candidates[0].args[3], 0.4);
  // Same eight-term shape the real path emits, so the agent's weighting runs
  // over it unchanged rather than taking a shortcut for mocked edges.
  assert.equal(candidates[0].args.length, 9);
  assert.equal(beliefs.filter((belief) => belief.name === "edgeCost").length, 0, "the bridge must not choose");
});

test("a neighbour with no mocked cost is an unpriceable edge, as in the real cell", async () => {
  const { bridge, beliefs } = mockBridge({ costs: "Robot2=0.4" });
  await bridge.estimateEdgeCost("Robot3", 0.3, 0.65);
  assert.deepEqual(
    beliefs.find((belief) => belief.name === "edgeCostFailed").args,
    ["robot3", "pickup", "no_mock_cost_for_neighbour"]
  );
  assert.equal(beliefs.filter((belief) => belief.name === "edgeCostReady").length, 0);
});

test("one mock flag means there is no cell behind the bridge at all", async () => {
  // Not a per-affordance switch. Reaching for a lower Thing that is not there
  // throws, and the agent then waits out a 60-second deadline for a belief that
  // was never coming -- so a bridge told to stand in for anything stands in for
  // everything, and an unmentioned site list is simply empty.
  const { bridge, beliefs } = mockBridge({ costs: "Robot2=0.4" });
  await bridge.detectTaskSites();
  assert.equal(beliefs.filter((belief) => belief.name === "taskSitesFailed").length, 0);
  assert.deepEqual(beliefs.find((belief) => belief.name === "taskSitesReady").args, [0, 0]);
});

test("a bridge standing in for a cell reports no sites rather than failing", async () => {
  const { bridge, beliefs } = mockBridge({ costs: "Robot2=0.4" });
  await bridge.detectTaskSites();
  assert.equal(beliefs.filter((belief) => belief.name === "site").length, 0);
});
