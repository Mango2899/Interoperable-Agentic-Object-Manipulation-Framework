const test = require("node:test");
const assert = require("node:assert/strict");

const {
  STRATEGIES,
  WEIGHTS,
  bboxCentre,
  boundaryDistance,
  densities,
  outsideCells,
  ordering,
  refusedHistogram,
  reverseView,
  strategySummary,
  walk,
  weightedScore
} = require("./e9HandoverPoint");

// A synthetic pool: five points on a line, so that every criterion has an
// unambiguous favourite and the walk's bookkeeping is checkable by hand.
function poolOf(specs) {
  const entries = specs.map((spec, index) => ({
    index,
    voxel: { center: { x: index * 0.1, y: 0, z: 0 }, index: [index, 0, 0] },
    centrality: spec.centrality,
    giverSupport: spec.giverSupport ?? 1,
    receiverSupport: spec.receiverSupport ?? 1,
    posture: spec.posture ?? 1,
    feasible: spec.feasible !== false
  }));
  return { entries, neighbours: specs.map((spec) => spec.neighbours) };
}

test("the weighted score is Eq. (handover_score) and its weights sum to one", () => {
  const total = Object.values(WEIGHTS).reduce((sum, value) => sum + value, 0);
  assert.ok(Math.abs(total - 1) < 1e-12);
  assert.equal(
    weightedScore({ centrality: 0.9, density: 0.6, giverSupport: 0.7, receiverSupport: 0.5, posture: 1 })
      .toFixed(3),
    "0.776"
  );
  assert.equal(
    weightedScore({ centrality: 0.6, density: 1, giverSupport: 0.9, receiverSupport: 0.9, posture: 0.4 })
      .toFixed(3),
    "0.724"
  );
});

test("density is a count over the survivors, so a strike stops propping up a neighbour", () => {
  const pool = poolOf([
    { centrality: 0.5, neighbours: [0, 1, 2] },
    { centrality: 0.5, neighbours: [1, 0, 2] },
    { centrality: 0.5, neighbours: [2, 0, 1] },
    { centrality: 0.5, neighbours: [3] }
  ]);
  assert.deepEqual(densities(pool, new Set()), [1, 1, 1, 1 / 3]);
  const afterStrike = densities(pool, new Set([0]));
  assert.equal(afterStrike[1], 1);
  assert.equal(afterStrike[3], 0.5);
});

test("each criterion orders the pool by itself and only density reacts to a strike", () => {
  const pool = poolOf([
    { centrality: 0.2, neighbours: [0, 1, 2] },
    { centrality: 0.9, neighbours: [1] },
    { centrality: 0.5, neighbours: [2, 0] }
  ]);
  const context = { shuffled: [2, 0, 1], bboxDistance: [0.5, 0.1, 0.9] };
  assert.equal(ordering("centrality", pool, new Set(), context)[0], 1);
  assert.equal(ordering("density", pool, new Set(), context)[0], 0);
  assert.deepEqual(ordering("random", pool, new Set(), context), [2, 0, 1]);
  // bbox offers the box centre, which is not one of the voxels, before them.
  assert.deepEqual(ordering("bbox", pool, new Set(), context), [-1, 1, 0, 2]);
  assert.deepEqual(ordering("bbox", pool, new Set([-1]), context), [1, 0, 2]);
  // Striking voxel 0 costs voxel 2 its only neighbour, so 2 stops leading.
  assert.notEqual(ordering("density", pool, new Set([0]), context)[0], 2);
});

test("the walk strikes what the cell refuses and counts the strikes", () => {
  const pool = poolOf([
    { centrality: 0.9, neighbours: [0], feasible: false },
    { centrality: 0.8, neighbours: [1], feasible: false },
    { centrality: 0.7, neighbours: [2], feasible: true }
  ]);
  const context = { shuffled: [], bboxDistance: [0, 1, 2], centreEntry: null, centreDensity: 0 };
  const result = walk("centrality", pool, context);
  assert.equal(result.refused, 2);
  assert.equal(result.selected.index, 2);
  assert.equal(result.firstChoice.index, 0, "the first choice is reported even when it was struck");
});

test("a pool with nothing feasible reports no point rather than throwing", () => {
  const pool = poolOf([{ centrality: 0.9, neighbours: [0], feasible: false }]);
  const result = walk("centrality", pool, { shuffled: [], bboxDistance: [0] });
  assert.equal(result.selected, null);
  assert.equal(result.refused, 1);
});

test("bbox falls back on the nearest voxel when the box centre itself is refused", () => {
  const pool = poolOf([
    { centrality: 0.4, neighbours: [0], feasible: true },
    { centrality: 0.9, neighbours: [1], feasible: true }
  ]);
  const centreEntry = {
    index: -1, voxel: { center: { x: 0, y: 0, z: 0 }, index: [0, 0, 0] },
    centrality: 1, giverSupport: 1, receiverSupport: 1, posture: 1, feasible: false
  };
  const result = walk("bbox", pool, { shuffled: [], bboxDistance: [0.9, 0.1], centreEntry, centreDensity: 0 });
  assert.equal(result.refused, 1);
  assert.equal(result.selected.index, 1, "the voxel nearest the centre is next in line");
});

// The interaction space is a sparse intersection of two sampled clouds, so its
// complement runs right through it. Only the cells a flood fill reaches from
// the outside count as boundary; an interior hole does not.
test("boundary distance is measured against the outside, not against every empty cell", () => {
  const voxels = [];
  for (let i = 0; i < 5; i += 1) {
    for (let j = 0; j < 5; j += 1) {
      for (let k = 0; k < 5; k += 1) {
        if (i === 2 && j === 2 && k === 2) continue; // the hole in the middle
        voxels.push({ index: [i, j, k], center: { x: i * 0.1, y: j * 0.1, z: k * 0.1 } });
      }
    }
  }
  const space = { voxels, voxelSize: 0.1, origin: { x: 0, y: 0, z: 0 } };
  const grid = outsideCells(space);
  assert.ok(!grid.outside.has("2,2,2"), "the enclosed hole is not outside");
  assert.ok(grid.outside.has("-1,2,2"), "the padded shell is");
  const corner = voxels.find(({ index }) => index.join(",") === "0,0,0");
  const nextToHole = voxels.find(({ index }) => index.join(",") === "2,2,1");
  assert.ok(Math.abs(boundaryDistance(space, grid, corner) - 0.1) < 1e-9);
  assert.ok(
    boundaryDistance(space, grid, nextToHole) > 0.1,
    "a point beside the interior hole is deeper in than a point on the face"
  );
});

test("the bounding-box centre is the centre of the voxel centres' box", () => {
  const voxels = [
    { index: [0, 0, 0], center: { x: 0, y: 0, z: 0 } },
    { index: [2, 4, 6], center: { x: 0.2, y: 0.4, z: 0.6 } }
  ];
  assert.deepEqual(bboxCentre(voxels), { x: 0.1, y: 0.2, z: 0.3 });
  // Restricted to a band, the box is the band's, not the whole space's.
  assert.deepEqual(bboxCentre(voxels.slice(0, 1)), { x: 0, y: 0, z: 0 });
});

test("the reverse direction swaps the roles and keeps the geometry", () => {
  const space = {
    voxels: [1], voxelSize: 0.03, targetRobot: "B",
    owner: { name: "A" }, partner: { name: "B" },
    ownerManifest: { name: "A" }, partnerManifest: { name: "B" },
    ownerTree: "treeA", partnerTree: "treeB",
    ownerBounds: "boundsA", partnerBounds: "boundsB"
  };
  const reversed = reverseView(space);
  assert.equal(reversed.owner.name, "B");
  assert.equal(reversed.partner.name, "A");
  assert.equal(reversed.targetRobot, "A");
  assert.equal(reversed.ownerTree, "treeB");
  assert.equal(reversed.ownerBounds, "boundsB");
  assert.equal(reversed.voxels, space.voxels, "the same voxels, not a recomputation");
});

test("the summary separates the first choice from what the walk ended on", () => {
  const rows = [
    { strategy: "random", repeat: 0, found: 1, refused: 0, firstChoiceFeasible: 1, boundaryDistance: 0.03, edgeCost: 1, score: 0.7 },
    { strategy: "random", repeat: 1, found: 1, refused: 3, firstChoiceFeasible: 0, boundaryDistance: 0.03, edgeCost: 1.2, score: 0.6 },
    { strategy: "bbox", repeat: 0, found: 1, refused: 0, firstChoiceFeasible: 1, boundaryDistance: 0, edgeCost: 1.1, score: 0.5 }
  ];
  const summary = strategySummary(rows);
  const random = summary.find((row) => row.strategy === "random");
  assert.equal(random.feasibilityRate, 0.5);
  assert.equal(random.refused_mean, 1.5);
  assert.equal(random.inSpaceRate, 1, "both random picks are voxels of the shared set");
  const bbox = summary.find((row) => row.strategy === "bbox");
  assert.equal(bbox.inSpaceRate, 0, "the box centre fell outside the shared set");
  assert.equal(summary.length, 2, "a strategy with no rows is left out");
});

test("the refusal histogram is a share per strategy and covers every strategy", () => {
  const rows = [
    { strategy: "weighted", found: 1, refused: 0 },
    { strategy: "weighted", found: 1, refused: 4 },
    { strategy: "random", found: 1, refused: 0 }
  ];
  const histogram = refusedHistogram(rows);
  assert.equal(histogram[0].weighted, 0.5);
  assert.equal(histogram.find((row) => row.from === 3).weighted, 0.5);
  assert.equal(histogram[0].random, 1);
  for (const strategy of STRATEGIES) assert.ok(strategy in histogram[0]);
  const shares = histogram.reduce((sum, row) => sum + (row.weighted || 0), 0);
  assert.ok(Math.abs(shares - 1) < 1e-12, "the shares of one strategy add up to one");
});

// ---------------------------------------------------------------------------
// The live half. The cell is not available in a unit test, so the two lower
// Things are faked and what is checked is the part that is this script's own:
// the order of the stages, where a failure is attributed, that the teardown
// runs whatever happened, and that the summary reads the records back.

const {
  LIVE_STRATEGIES,
  POINT_STAGES,
  STAGES,
  directionSummary,
  wilson,
  executeHandover,
  executionSummary,
  jointPayload,
  reasonOf,
  resetCell,
  resumeState,
  thingClient
} = require("./e9HandoverPoint");

function fakeClient(name, refusals = {}) {
  const calls = [];
  return {
    name,
    calls,
    post: async (action, body) => {
      calls.push({ action, body });
      const refusal = refusals[action];
      if (refusal) return { success: false, reason: refusal };
      if (action === "transferCube") return { success: true, graspDistance: 0.004, graspTolerance: 0.05 };
      return { success: true };
    },
    get: async () => ({ holding: false })
  };
}

function resetFixture(holders = []) {
  const holding = new Set(holders);
  const events = [];
  const clients = Array.from({ length: 5 }, (_, i) => {
    const name = `Robot${i + 1}`;
    return {
      name,
      get: async () => ({ holding: holding.has(name) }),
      post: async (action) => {
        events.push(`${name}:${action}`);
        if (action === "openGripper") holding.delete(name);
        return { success: true };
      }
    };
  });
  return {
    holding, events, clients,
    tds: Object.fromEntries(clients.map(({ name }) => [name, {}])),
    run: async () => {
      assert.equal(holding.size, 0, "release every claim before reparenting the shared cube");
      events.push("park");
      return { success: true };
    },
    parkAt: { x: 0, y: 0, z: 0.1 }, cubeName: "cube", settleMs: 0
  };
}

test("pair changes release the previous receiver, including robots outside the next pair", async () => {
  const cell = resetFixture(["Robot1"]); // a holder left by an earlier, filtered run
  for (const [giver, receiver] of [
    ["Robot1", "Robot2"], ["Robot2", "Robot3"], ["Robot2", "Robot4"],
    ["Robot3", "Robot5"], ["Robot4", "Robot5"]
  ]) {
    assert.equal((await resetCell(cell)).success, true);
    // The harness seeds the next giver. No previous receiver may still claim it.
    cell.holding.add(giver);
    assert.deepEqual([...cell.holding], [giver]);
    cell.holding.delete(giver);
    cell.holding.add(receiver);
  }
  assert.equal((await resetCell(cell)).success, true);
  assert.equal(cell.holding.size, 0);
});

test("a failed release or unreadable grip state stops setup before parking or homing", async () => {
  for (const mode of ["unreadable", "refused", "still_holding"]) {
    const cell = resetFixture(["Robot3"]);
    const client = cell.clients[2];
    if (mode === "unreadable") client.get = async () => ({ success: false, reason: "thing_unreachable" });
    else client.post = async () => ({ success: mode !== "refused", reason: "release_failed" });
    assert.equal((await resetCell(cell)).success, false, mode);
    assert.ok(cell.events.every((event) => event.endsWith(":openGripper")), "a bad setup must not proceed to parking or arm motion");
  }
});

test("a parking failure stops setup before motion", async () => {
  const cell = resetFixture();
  cell.run = async () => ({ success: false, reason: "grip_object_missing" });
  assert.equal((await resetCell(cell)).reason, "park_failed: grip_object_missing");
  assert.ok(cell.events.every((event) => event.endsWith(":openGripper")));
});

test("reset clears stale claims even when the old gripper state cannot be read", async () => {
  const cell = resetFixture(["Robot3"]);
  cell.clients[2].get = async () => cell.holding.has("Robot3")
    ? { success: false, reason: "thing_request_failed" }
    : { holding: false };
  assert.equal((await resetCell(cell)).success, true);
  assert.equal(cell.holding.size, 0);
});

test("reset clears the interrupted receiver before homing the giver and other arms", async () => {
  const cell = resetFixture(["Robot3"]);
  cell.homeFirst = [cell.clients[4], cell.clients[2]];
  assert.equal((await resetCell(cell)).success, true);
  assert.deepEqual(cell.events.filter((event) => event.endsWith(":moveTojointPosition")), [
    "Robot5:moveTojointPosition", "Robot3:moveTojointPosition", "Robot1:moveTojointPosition",
    "Robot2:moveTojointPosition", "Robot4:moveTojointPosition"
  ]);
});

test("legacy ownership failures resume and do not dilute the corrected handover rates", () => {
  const base = { pair: "Robot2__Robot4", giver: "Robot2", strategy: "weighted", attempt: 0, final: true };
  const invalid = { ...base, stage: "session", reason: "cube_held_by_multiple_robots", completed: 0, transferred: 0, durationMs: 30 };
  assert.equal(resumeState([invalid]).done.size, 0);
  assert.equal(resumeState([invalid]).finished.size, 0);
  const corrected = { ...base, stage: "complete", completed: 1, transferred: 1, durationMs: 9000 };
  const resumed = resumeState([invalid, corrected]);
  // A record written before --repeats existed is the zeroth round.
  assert.ok(resumed.done.has("Robot2__Robot4|Robot2|weighted|0|0"));
  assert.ok(resumed.finished.has("Robot2__Robot4|Robot2|weighted|0"));
  const [summary] = executionSummary([invalid, corrected]);
  assert.equal(summary.setupFailures, 1);
  assert.equal(summary.handovers, 1);
  assert.equal(summary.firstAttemptSuccess, 1);
  assert.equal(summary.successRate, 1);
  // Other session errors remain visible as failures, rather than being hidden.
  const [other] = executionSummary([{ ...invalid, reason: "not_interaction_space_owner" }]);
  assert.equal(other.setupFailures, 0);
  assert.equal(other.successRate, 0);
});

const CANDIDATE = {
  cubeWorld: { x: 1, y: 2, z: 0.3 },
  giverTarget: { localTcp: { x: 0.1, y: 0.2, z: 0.3 } },
  receiverTarget: { localTcp: { x: -0.1, y: 0.2, z: 0.3 } }
};

test("a clean handover runs the stages in order and tears the session down", async () => {
  const giver = fakeClient("A");
  const receiver = fakeClient("B");
  const result = await executeHandover({ giver, receiver, candidate: CANDIDATE, sessionId: "s1", log: () => {} });
  assert.equal(result.completed, true);
  assert.equal(result.transferred, true);
  assert.equal(result.stage, "complete");
  assert.equal(result.graspDistance, 0.004);
  assert.deepEqual(
    giver.calls.map((call) => call.action),
    ["beginHandoverSession", "moveTocartesianPosition", "transferCube", "retreatFromHandover",
      "returnHomeAfterHandover", "setHandoverApproach", "closeHandoverSession"]
  );
  assert.deepEqual(
    receiver.calls.map((call) => call.action),
    ["setHandoverApproach", "moveTocartesianPosition", "retreatFromHandover",
      "setHandoverApproach", "closeHandoverSession"]
  );
  // Local TCP, per robot -- the giver is not sent the receiver's target.
  assert.deepEqual(giver.calls[1].body, { x: 0.1, y: 0.2, z: 0.3 });
  assert.deepEqual(receiver.calls[1].body, { x: -0.1, y: 0.2, z: 0.3 });
  assert.equal(giver.calls[0].body.cubeWorld.z, 0.3);
});

test("the receiver's jaw gate is what a refused transfer reports", async () => {
  const giver = fakeClient("A", { transferCube: "receiver_not_at_cube" });
  const receiver = fakeClient("B");
  const result = await executeHandover({ giver, receiver, candidate: CANDIDATE, sessionId: "s2", log: () => {} });
  assert.equal(result.stage, "transfer");
  assert.equal(result.reason, "receiver_not_at_cube");
  assert.equal(result.transferred, false);
  assert.equal(result.completed, false);
  assert.ok(POINT_STAGES.has(result.stage), "a refused transfer strikes the candidate");
});

test("a failure stops the remaining stages but never the teardown", async () => {
  const giver = fakeClient("A", { moveTocartesianPosition: "workspace_admission_refused" });
  const receiver = fakeClient("B");
  const result = await executeHandover({ giver, receiver, candidate: CANDIDATE, sessionId: "s3", log: () => {} });
  assert.equal(result.stage, "giver_approach");
  assert.equal(result.reason, "workspace_admission_refused");
  assert.ok(!giver.calls.some((call) => call.action === "transferCube"), "nothing is attempted after a refusal");
  // Both arms are still told to disarm and drop the session.
  for (const client of [giver, receiver]) {
    assert.ok(client.calls.some((call) => call.action === "closeHandoverSession"));
    const disarm = client.calls.filter((call) => call.action === "setHandoverApproach").pop();
    assert.equal(disarm.body.enabled, false);
  }
});

test("a retreat failure is not a verdict on the point, so it does not strike it", async () => {
  const giver = fakeClient("A");
  const receiver = fakeClient("B", { retreatFromHandover: "no_standoff_recorded" });
  const result = await executeHandover({ giver, receiver, candidate: CANDIDATE, sessionId: "s4", log: () => {} });
  assert.equal(result.stage, "receiver_retreat");
  assert.equal(result.transferred, true, "the cube did cross");
  assert.equal(result.completed, false, "but the handover did not complete");
  assert.ok(!POINT_STAGES.has(result.stage));
});

test("every stage a handover can stop at is a known stage", async () => {
  for (const stage of POINT_STAGES) assert.ok(STAGES.includes(stage));
  const giver = fakeClient("A", { returnHomeAfterHandover: "motion_busy" });
  const result = await executeHandover({
    giver, receiver: fakeClient("B"), candidate: CANDIDATE, sessionId: "s5", log: () => {}
  });
  assert.ok(STAGES.includes(result.stage));
});

test("a refusal is read off whichever field the cell used to say no", () => {
  assert.equal(reasonOf({ success: true }), null);
  assert.equal(reasonOf({ success: false, reason: "not_interaction_space_owner" }), "not_interaction_space_owner");
  assert.equal(reasonOf({ success: false, message: "fetch failed" }), "fetch failed");
  assert.equal(reasonOf({ success: false }), "refused");
  assert.equal(reasonOf(null), "no_response");
});

test("joint targets are named after the TD's own joints", () => {
  const td = { actions: { moveTojointPosition: { input: { properties: { joint1: {}, joint2: {}, joint3: {} } } } } };
  assert.deepEqual(jointPayload(td, [0, 0, 0]), { joint1: 0, joint2: 0, joint3: 0 });
  assert.deepEqual(jointPayload(td, [10, 20]), { joint1: 10, joint2: 20, joint3: 0 });
});

test("an unreachable Thing is a refusal, not an exception", async () => {
  const client = thingClient({ name: "A", lowerLayerUrl: "http://127.0.0.1:1/thing" }, 50);
  const result = await client.post("beginHandoverSession", {});
  assert.equal(result.success, false);
  assert.ok(["thing_unreachable", "thing_timeout"].includes(result.reason));
});

test("the execution summary separates the first choice from the retried outcome", () => {
  const records = [
    // weighted: one direction, succeeded straight away.
    { strategy: "weighted", attempt: 0, final: true, completed: true, transferred: true, stage: "complete", durationMs: 9000, graspDistance: 0.003, struck: 0 },
    // random: refused once on the transfer, then completed.
    { strategy: "random", attempt: 0, final: false, completed: false, transferred: false, stage: "transfer", durationMs: 7000, graspDistance: 0.09, struck: 1 },
    { strategy: "random", attempt: 1, final: true, completed: true, transferred: true, stage: "complete", durationMs: 9500, graspDistance: 0.004, struck: 0 }
  ];
  const summary = executionSummary(records);
  const weighted = summary.find((row) => row.strategy === "weighted");
  assert.equal(weighted.firstAttemptSuccess, 1);
  assert.equal(weighted.successRate, 1);
  const random = summary.find((row) => row.strategy === "random");
  assert.equal(random.firstAttemptSuccess, 0, "its first choice did not work");
  assert.equal(random.successRate, 1, "the replacement did");
  assert.equal(random.liveRefused_mean, 1);
  assert.equal(random.commonFailure, "transfer");
  assert.equal(summary.length, 2, "a strategy that was never run is left out");
});

test("the live half also executes the deployed pipeline, which offline cannot fail", () => {
  assert.ok(LIVE_STRATEGIES.includes("deployed"));
  for (const strategy of STRATEGIES) assert.ok(LIVE_STRATEGIES.includes(strategy));
  const summary = executionSummary([
    { strategy: "deployed", attempt: 0, final: true, completed: true, transferred: true, stage: "complete", durationMs: 9000, graspDistance: 0.003, struck: 0 }
  ]);
  assert.equal(summary[0].strategy, "deployed");
  assert.equal(summary[0].successRate, 1);
});

test("a setup failure is reported apart and never averaged into a success rate", () => {
  const records = [
    { strategy: "weighted", attempt: 0, final: true, completed: false, transferred: false, stage: "seed", reason: "object_not_in_grip_range", durationMs: 0, graspDistance: null, struck: 0 },
    { strategy: "weighted", attempt: 0, final: true, completed: true, transferred: true, stage: "complete", durationMs: 9000, graspDistance: 0.003, struck: 0 }
  ];
  const summary = executionSummary(records);
  assert.equal(summary[0].setupFailures, 1);
  assert.equal(summary[0].handovers, 1, "only the trial that actually ran counts");
  assert.equal(summary[0].firstAttemptSuccess, 1, "the seed failure does not drag the rate to 0.5");
  assert.equal(summary[0].successRate, 1);
  assert.equal(summary[0].commonFailure, null, "a seed failure is not a stage a rendezvous broke at");
});

test("a strategy whose only record is a setup failure has no rate at all", () => {
  const summary = executionSummary([
    { strategy: "random", attempt: 0, final: true, completed: false, transferred: false, stage: "seed", reason: "object_not_in_grip_range", durationMs: 0, graspDistance: null, struck: 0 }
  ]);
  assert.equal(summary[0].setupFailures, 1);
  assert.equal(summary[0].handovers, 0);
  assert.equal(summary[0].firstAttemptSuccess, null);
  assert.equal(summary[0].successRate, null);
});

// rank() keeps returning a candidate the cell has refused -- it only stops
// counting it towards its neighbours' density. The agent is what removes it,
// so the live runner has to remove it too, or a retry re-offers the point that
// was just struck.
test("a refused candidate is not offered again by the deployed pipeline", () => {
  const { InteractionSpaceController } = require("../generator/interactionSpace");
  const controller = Object.create(InteractionSpaceController.prototype);
  controller.rejectedCandidates = new Map([["Robot5", new Set(["c7"])]]);
  const ranked = {
    success: true,
    candidates: [
      { id: "c7", centrality: 0.9, density: 1, giverSupport: 1, receiverSupport: 1, posture: 1 },
      { id: "c9", centrality: 0.8, density: 1, giverSupport: 1, receiverSupport: 1, posture: 1 }
    ]
  };
  const struck = controller.rejectedCandidates.get("Robot5");
  const open = ranked.candidates.filter((candidate) => !struck.has(candidate.id));
  assert.equal(open.length, 1);
  assert.equal(open[0].id, "c9", "the struck candidate is gone, not merely discounted");
  // And the highest-scoring open candidate is the one that would be offered.
  const best = open.slice().sort((left, right) => weightedScore(right) - weightedScore(left))[0];
  assert.equal(best.id, "c9");
});

// ---------------------------------------------------------------------------
// Repetitions.

test("a round is resumed on its own, so a finished round does not block the next", () => {
  const base = {
    pair: "Robot2__Robot4", giver: "Robot2", strategy: "weighted",
    attempt: 0, final: true, stage: "complete", completed: 1, transferred: 1, durationMs: 9000
  };
  const { done, finished } = resumeState([{ ...base, repeat: 0 }, { ...base, repeat: 1 }]);
  assert.ok(finished.has("Robot2__Robot4|Robot2|weighted|0"));
  assert.ok(finished.has("Robot2__Robot4|Robot2|weighted|1"));
  assert.ok(!finished.has("Robot2__Robot4|Robot2|weighted|2"), "the third round is still to run");
  assert.equal(done.size, 2, "the two rounds are separate trials, not one repeated key");
});

test("the success rate is counted over directions times rounds", () => {
  const rows = [];
  for (const pair of ["Robot1__Robot2", "Robot2__Robot3"]) {
    for (const repeat of [0, 1, 2]) {
      rows.push({
        pair, giver: "Robot1", receiver: "Robot2", strategy: "random", repeat, attempt: 0, final: true,
        stage: "complete", completed: pair === "Robot1__Robot2" || repeat === 0 ? 1 : 0,
        transferred: 1, durationMs: 9000, struck: 0, graspDistance: 0.004
      });
    }
  }
  const [summary] = executionSummary(rows);
  assert.equal(summary.rounds, 3);
  assert.equal(summary.directions, 2, "two pairs, whatever the round");
  assert.equal(summary.firstAttemptTrials, 6, "directions times rounds");
  assert.equal(summary.firstAttemptWon, 4);
  assert.ok(Math.abs(summary.firstAttemptSuccess - 4 / 6) < 1e-12);
});

test("the Wilson interval stays inside [0,1] and narrows as trials accumulate", () => {
  assert.deepEqual(wilson(0, 0), { low: null, high: null });
  const perfect = wilson(5, 5);
  assert.ok(perfect.low > 0.5 && perfect.low < 1, "five from five is not certainty");
  assert.equal(perfect.high, 1);
  const none = wilson(0, 5);
  assert.equal(none.low, 0);
  assert.ok(none.high > 0 && none.high < 0.6);
  const narrow = wilson(25, 25);
  assert.ok(narrow.low > perfect.low, "more trials, a tighter bound");
  const half = wilson(10, 20);
  assert.ok(half.low < 0.5 && half.high > 0.5);
});

test("a flaky direction is visible per direction, not only in the average", () => {
  const row = (pair, repeat, completed) => ({
    pair, giver: "Robot2", receiver: "Robot4", strategy: "random", repeat, attempt: 0,
    final: true, stage: completed ? "complete" : "receiver_approach", completed, durationMs: 9000
  });
  // Same rate, different stories: one direction always fails, or every round
  // loses a different one.
  const always = directionSummary([row("A", 0, 1), row("A", 1, 1), row("B", 0, 0), row("B", 1, 0)]);
  const scattered = directionSummary([row("A", 0, 1), row("A", 1, 0), row("B", 0, 0), row("B", 1, 1)]);
  assert.deepEqual(always.map((entry) => entry.completed).sort(), [0, 2]);
  assert.deepEqual(scattered.map((entry) => entry.completed).sort(), [1, 1]);
  assert.equal(always[0].rounds, 2);
  assert.equal(always[0].trials, 2);
});
