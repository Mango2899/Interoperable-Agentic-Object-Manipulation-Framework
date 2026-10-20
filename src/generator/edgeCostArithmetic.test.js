"use strict";

// The cost itself, not the wiring around it. The real method runs with a
// stubbed lower layer and a stubbed ranking, so what is under test is the
// arithmetic and the choice of origin -- the two things a structural assertion
// cannot check.

const assert = require("node:assert/strict");
const test = require("node:test");

const { GeneratedRobotAgentThing } = require("./agentThingServer");

function candidate(id, giverLocalTcp, scores = {}) {
  return {
    id,
    giver: { localTcp: giverLocalTcp },
    receiver: { localTcp: { x: 0, y: 0, z: 0 } },
    centrality: scores.centrality ?? 0.5,
    density: scores.density ?? 0.5,
    giverSupport: scores.giverSupport ?? 0.5,
    receiverSupport: scores.receiverSupport ?? 0.5,
    posture: scores.posture ?? 0.5
  };
}

// A bridge with nothing real behind it: no simulator, no interaction-space
// estimator, no agent attached. Only the method under test is genuine.
function bridgeWith({ sites, tcp, candidates, rankResult }) {
  const beliefs = [];
  const bridge = Object.create(GeneratedRobotAgentThing.prototype);
  bridge.ctx = { manifest: { name: "Robot1" } };
  bridge.taskSites = sites;
  bridge.interactionSpace = {
    rank: () => rankResult || { success: true, candidates, candidateCount: candidates.length }
  };
  bridge.readLowerProperty = async (property) => {
    if (property === "getCartesianposition") return tcp;
    if (property === "getTaskSites") return { sites: sites || [] };
    throw new Error(`unexpected property ${property}`);
  };
  bridge.addBelief2Agent = async (name, args) => {
    beliefs.push({ name, args });
  };
  return { bridge, beliefs };
}

const LOADED_SITE = {
  alias: "Robot1_grip_support",
  reachable: true,
  occupiedBy: "Robot1_grip_cube",
  grasp: { x: 0.9, y: 0, z: 0.1825 }
};
const EMPTY_SITE = {
  alias: "Robot4_grip_support_b",
  reachable: true,
  occupiedBy: "none",
  grasp: { x: 0.25, y: 0, z: 0.1825 }
};

function beliefsNamed(beliefs, name) {
  return beliefs.filter((belief) => belief.name === name);
}

test("an arm at the loaded support pays from the cube, not from where it stands", async () => {
  const { bridge, beliefs } = bridgeWith({
    sites: [LOADED_SITE],
    tcp: { x: 0, y: 0, z: 0.3 },
    candidates: [candidate("candidate_1", { x: 1.2, y: 0, z: 0.1825 })]
  });
  await bridge.estimateEdgeCost("Robot2", 0.3, 0.65);

  const [origin] = beliefsNamed(beliefs, "edgeCostOrigin");
  // (neighbour, mode, kind, x, y, z) -- the mode is part of the answer so two
  // concurrent prices of the same edge cannot overwrite each other.
  assert.equal(origin.args[1], "pickup");
  assert.equal(origin.args[2], "pickup");
  assert.deepEqual(origin.args.slice(3), [0.9, 0, 0.1825]);

  // 1.2 - 0.9 = 0.3 m along x, and nothing else. Measured from the cube, this
  // is 0.3; measured from the arm's current pose it would be 1.2.
  const [priced] = beliefsNamed(beliefs, "edgeCandidate");
  assert.ok(Math.abs(priced.args[3] - 0.3) < 1e-9, `cost was ${priced.args[3]}`);
});

test("an arm that is not at the loaded support pays from its current pose", async () => {
  const { bridge, beliefs } = bridgeWith({
    sites: [EMPTY_SITE],
    tcp: { x: 0.5, y: 0, z: 0.3 },
    candidates: [candidate("candidate_1", { x: 0.9, y: 0, z: 0.3 })]
  });
  await bridge.estimateEdgeCost("Robot3", 0.3, 0.65);

  const [origin] = beliefsNamed(beliefs, "edgeCostOrigin");
  assert.equal(origin.args[2], "tcp");
  const [priced] = beliefsNamed(beliefs, "edgeCandidate");
  assert.ok(Math.abs(priced.args[3] - 0.4) < 1e-9, `cost was ${priced.args[3]}`);
});

test("a support this arm cannot reach is not a pickup it can pay from", async () => {
  const unreachable = { ...LOADED_SITE, reachable: false };
  const { bridge, beliefs } = bridgeWith({
    sites: [unreachable],
    tcp: { x: 0, y: 0, z: 0.3 },
    candidates: [candidate("candidate_1", { x: 0, y: 0, z: 0.5 })]
  });
  await bridge.estimateEdgeCost("Robot2", 0.3, 0.65);
  assert.equal(beliefsNamed(beliefs, "edgeCostOrigin")[0].args[2], "tcp");
});

test("the cost is a true 3D distance, not a horizontal one", async () => {
  const { bridge, beliefs } = bridgeWith({
    sites: [],
    tcp: { x: 0, y: 0, z: 0 },
    candidates: [candidate("candidate_1", { x: 3, y: 4, z: 12 })]
  });
  await bridge.estimateEdgeCost("Robot2", 0.3, 0.65);
  assert.ok(Math.abs(beliefsNamed(beliefs, "edgeCandidate")[0].args[3] - 13) < 1e-9);
});

test("every viable rendezvous is priced, and its score travels with the price", async () => {
  const { bridge, beliefs } = bridgeWith({
    sites: [],
    tcp: { x: 0, y: 0, z: 0 },
    candidates: [
      candidate("candidate_1", { x: 1, y: 0, z: 0 }, { centrality: 0.9 }),
      candidate("candidate_2", { x: 2, y: 0, z: 0 }, { centrality: 0.1 })
    ]
  });
  await bridge.estimateEdgeCost("Robot2", 0.3, 0.65);
  const priced = beliefsNamed(beliefs, "edgeCandidate");
  assert.equal(priced.length, 2);
  // Neighbour, mode, candidate, distance and the five scoring factors, so the
  // agent can weigh them itself.
  assert.equal(priced[0].args.length, 9);
  assert.equal(priced[0].args[4], 0.9);
  assert.equal(priced[1].args[4], 0.1);
  // Nothing here declares a winner.
  assert.equal(beliefsNamed(beliefs, "edgeCost").length, 0);
  assert.equal(beliefsNamed(beliefs, "edgeCostReady")[0].args[2], 2);
});

test("a ranking that fails is reported as a failed edge, not as a free one", async () => {
  const { bridge, beliefs } = bridgeWith({
    sites: [],
    tcp: { x: 0, y: 0, z: 0 },
    candidates: [],
    rankResult: { success: false, reason: "no_height_filtered_interaction_voxel" }
  });
  await bridge.estimateEdgeCost("Robot2", 0.3, 0.65);
  assert.equal(beliefsNamed(beliefs, "edgeCostReady").length, 0);
  assert.equal(beliefsNamed(beliefs, "edgeCostFailed")[0].args[2], "no_height_filtered_interaction_voxel");
});

test("an unreadable pose is a failed edge rather than a cost measured from the origin", async () => {
  const { bridge, beliefs } = bridgeWith({
    sites: [],
    tcp: { x: null, y: null, z: null },
    candidates: [candidate("candidate_1", { x: 1, y: 0, z: 0 })]
  });
  await bridge.estimateEdgeCost("Robot2", 0.3, 0.65);
  assert.equal(beliefsNamed(beliefs, "edgeCostFailed")[0].args[2], "cost_origin_unreadable");
  assert.equal(beliefsNamed(beliefs, "edgeCandidate").length, 0);
});
