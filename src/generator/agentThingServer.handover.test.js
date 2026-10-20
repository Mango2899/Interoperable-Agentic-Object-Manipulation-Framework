const assert = require("node:assert/strict");
const test = require("node:test");

const { GeneratedRobotAgentThing } = require("./agentThingServer");

function mockBridge(actionResults = {}, propertyResults = {}, partnerProperties = null) {
  const bridge = Object.create(GeneratedRobotAgentThing.prototype);
  const beliefs = [];
  const invocations = [];
  const builds = [];
  bridge.ctx = { manifest: { name: "Robot1" } };
  bridge.interactionSpace = {
    build: (targetRobot, options = {}) => {
      builds.push([targetRobot, options]);
      return {
        success: true,
        id: "space-1",
        pairKey: "Robot1__Robot2",
        version: "abc123",
        cached: false,
        voxelSize: 0.04,
        voxelCount: 1000,
        origin: { x: -1, y: -1, z: -1 },
        placementSource: "scene"
      };
    },
    robot: () => null,
    visualFor: () => ({
      pairKey: "Robot1__Robot2",
      csv: "generated/robots/interactions/Robot1__Robot2/space.csv",
      voxelSize: 0.04,
      color: [0, 255, 0]
    }),
    select: () => ({
      success: true,
      id: "session-1",
      cubeWorld: { x: 1, y: 2, z: 3 },
      giver: { localTcp: { x: 4, y: 5, z: 6 } },
      receiver: { localTcp: { x: 7, y: 8, z: 9 } }
    })
  };
  bridge.ensureLowerThing = async () => {};
  bridge.addBelief2Agent = async (name, values) => beliefs.push([name, values]);
  bridge.lowerThing = {
    invokeAction: async (name, payload) => {
      invocations.push([name, payload]);
      return { value: async () => actionResults[name] };
    },
    readProperty: async (name) => ({ value: async () => propertyResults[name] })
  };
  bridge.partnerLowerThings = new Map();
  if (partnerProperties) {
    bridge.partnerLowerThings.set("Robot2", {
      readProperty: async (name) => ({ value: async () => partnerProperties[name] })
    });
  }
  return { bridge, beliefs, invocations, builds };
}

test("maps successful lower handover operations to session-correlated beliefs", async () => {
  const { bridge, beliefs, invocations } = mockBridge(
    {
      beginHandoverSession: { success: true, session: "session-1" },
      setHandoverApproach: { success: true },
      setPickupApproach: { success: true },
      transferCube: { success: true, sourceRobot: "Robot1", targetRobot: "Robot2" }
    },
    {
      getCubeOwnership: { owner: "Robot2", transferActive: false },
      getGripperState: { closed: true, holding: true, object: "cube" }
    }
  );

  const requests = [
    ["computeInteractionSpace", ["Robot2"]],
    ["planHandover", ["Robot2", 42, 0.3, 0.65]],
    ["setHandoverApproach", ["session-1", true]],
    ["setPickupApproach", [true]],
    ["transferCube", ["session-1", "Robot2"]],
    ["queryCubeOwnership", ["session-1"]],
    ["queryGripperState", ["session-1"]]
  ];
  for (const [keyword, value] of requests) {
    assert.equal(await bridge.handleCustomAgentRequests({
      targetAgentId: "ownLayer1",
      content: { keyword, value }
    }), true);
  }

  assert.deepEqual(beliefs, [
    ["interactionRobot", ["robot2_agent", "robot2"]],
    ["interactionSpace", ["space_1", "robot2", 0.04, -1, -1, -1, 1000]],
    ["interactionSpaceReady", ["space_1", "robot2"]],
    ["interactionSpaceCache", ["space_1", "robot1__robot2", "abc123", false]],
    ["handoverPlan", ["session_1", 1, 2, 3, 4, 5, 6, 7, 8, 9]],
    ["handoverApproachSet", ["session-1", true]],
    ["pickupApproachSet", [true]],
    ["cubeTransferSucceeded", ["session-1", "Robot1", "Robot2"]],
    ["cubeOwnership", ["session-1", "Robot2", false]],
    ["gripperState", ["session-1", true, true, "cube"]]
  ]);
  assert.deepEqual(invocations.filter(([name]) => ["beginHandoverSession", "setHandoverApproach", "setPickupApproach", "transferCube"].includes(name)), [
    ["beginHandoverSession", { session: "session-1", targetRobot: "Robot2", cubeWorld: { x: 1, y: 2, z: 3 } }],
    ["setHandoverApproach", { session: "session-1", enabled: true }],
    ["setPickupApproach", { enabled: true }],
    ["transferCube", { session: "session-1", targetRobot: "Robot2" }]
  ]);
});

test("leaving a rendezvous is one affordance, and a refusal is not a motion failure", async () => {
  const { bridge, beliefs, invocations } = mockBridge({
    retreatFromHandover: { success: true, state: "handover_retreat_complete", retreatStates: 8 }
  });
  bridge.motionQueue = Promise.resolve();

  assert.equal(await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "retreatFromHandover", value: ["session-1"] }
  }), true);

  // No point is sent: the lower layer holds the path the arm came in on, and
  // the agent asks it to leave rather than to move somewhere.
  assert.deepEqual(
    invocations.filter(([name]) => name === "retreatFromHandover"),
    [["retreatFromHandover", undefined]]
  );
  assert.deepEqual(beliefs, [["handoverRetreated", ["session-1"]]]);
});

test("a rendezvous with no recorded approach reports an unconfirmed retreat", async () => {
  const { bridge, beliefs } = mockBridge({
    retreatFromHandover: { success: false, reason: "handover_approach_path_stale" }
  });
  bridge.motionQueue = Promise.resolve();

  await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "retreatFromHandover", value: ["session-1"] }
  });

  // motionFailed would send the agent broken over an arm that has not moved.
  assert.deepEqual(beliefs, [
    ["handoverRetreatFailed", ["session-1", "handover_approach_path_stale"]]
  ]);
});

test("a retreat that moved but missed its endpoint never reports clearance or issues a Cartesian retry", async () => {
  const { bridge, beliefs, invocations } = mockBridge({
    retreatFromHandover: { success: false, reason: "handover_retreat_not_reached",
      actual: { x: 0.8, y: 0, z: 0.3 }, finalError: 0.03 }
  });
  bridge.motionQueue = Promise.resolve();
  await bridge.handleCustomAgentRequests({ targetAgentId: "ownLayer1",
    content: { keyword: "retreatFromHandover", value: ["session-1"] } });
  assert.deepEqual(invocations, [["retreatFromHandover", undefined]]);
  assert.deepEqual(beliefs, [["handoverRetreatFailed", ["session-1", "handover_retreat_not_reached"]]]);
});

test("does not publish ownership-changing success beliefs on lower failures", async () => {
  const { bridge, beliefs } = mockBridge({
    beginHandoverSession: { success: false, reason: "no_plan" },
    setHandoverApproach: { success: false, reason: "collision_mode_rejected" },
    transferCube: { success: false, reason: "receiver_not_ready" }
  });

  await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "planHandover", value: ["Robot2", 42, 0.3, 0.65] }
  });
  await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "setHandoverApproach", value: ["session-1", true] }
  });
  await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "transferCube", value: ["session-1", "Robot2"] }
  });

  assert.deepEqual(beliefs, [
    ["handoverPlanFailed", ["no_plan"]],
    ["handoverApproachFailed", ["session-1", true, "collision_mode_rejected"]],
    ["cubeTransferFailed", ["session-1", "receiver_not_ready"]]
  ]);
});

test("publishes central-core evidence and commits only the candidate selected by the agent", async () => {
  const { bridge, beliefs, invocations } = mockBridge({
    beginHandoverSession: { success: true, session: "session-central" }
  });
  const candidate = {
    id: "candidate_7",
    cubeWorld: { x: 1, y: 2, z: 3 },
    giver: { localTcp: { x: 4, y: 5, z: 6 } },
    receiver: { localTcp: { x: 7, y: 8, z: 9 } },
    centrality: 0.9,
    density: 0.8,
    giverSupport: 0.7,
    receiverSupport: 0.6,
    // How far each arm's straight approach leg stays clear of its own
    // base-joint axis: the term per-point reachability cannot supply.
    posture: 0.5
  };
  bridge.interactionSpace.rank = () => ({
    success: true,
    center: { x: 1.1, y: 2.1, z: 3.1 },
    bounds: { x: [0, 2], y: [1, 3], z: [2, 4] },
    heightCandidateCount: 100,
    coreCandidateCount: 30,
    candidateCount: 1,
    candidates: [candidate]
  });
  bridge.interactionSpace.commit = (_target, id) => ({
    success: id === candidate.id,
    ...candidate,
    id: "session-central"
  });

  await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "rankHandoverCandidates", value: ["Robot2", 0.3, 0.65] }
  });
  await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "commitHandoverCandidate", value: ["Robot2", "candidate_7"] }
  });

  assert.deepEqual(beliefs, [
    ["handoverCore", ["robot2", 1.1, 2.1, 3.1, 0, 2, 1, 3, 2, 4, 100, 30, 1]],
    ["handoverCandidate", ["candidate_7", "robot2", 1, 2, 3, 4, 5, 6, 7, 8, 9, 0.9, 0.8, 0.7, 0.6, 0.5]],
    ["handoverCandidatesReady", ["robot2", 1]],
    ["handoverPlan", ["session_central", 1, 2, 3, 4, 5, 6, 7, 8, 9]]
  ]);
  assert.deepEqual(invocations, [
    ["beginHandoverSession", { session: "session-central", targetRobot: "Robot2", cubeWorld: { x: 1, y: 2, z: 3 } }]
  ]);
});

test("recomputes the shared space from the poses the robots report, then redraws it", async () => {
  const { bridge, invocations, builds } = mockBridge(
    { refreshInteractionVisual: { success: true, pointCount: 881 } },
    { getBasePose: { position: [0, 0, 0.03], orientation: [0, 0, 0] } },
    { getBasePose: { position: [1.5, 0.2, 0.03], orientation: [0, 0, Math.PI] } }
  );

  assert.equal(await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "refreshInteractionSpace", value: ["Robot2"] }
  }), true);

  const [targetRobot, options] = builds[0];
  assert.equal(targetRobot, "Robot2");
  // Both arms are asked where they stand, and the pedestal each base is lifted
  // onto is removed again: the workspace cloud is written in the frame below it.
  assert.deepEqual(options.placements.Robot1.offset, [0, 0, 0]);
  assert.deepEqual(options.placements.Robot2.offset, [1.5, 0.2, 0]);
  assert.ok(Math.abs(options.placements.Robot2.orientationDeg[2] - 180) < 1e-9);

  // A space that was actually recomputed is worth nothing if the scene still
  // shows the cloud the pipeline baked for the previous layout.
  assert.deepEqual(invocations.filter(([name]) => name === "refreshInteractionVisual"), [
    ["refreshInteractionVisual", {
      pairKey: "Robot1__Robot2",
      csv: "generated/robots/interactions/Robot1__Robot2/space.csv",
      voxelSize: 0.04,
      color: [0, 255, 0]
    }]
  ]);
});

test("falls back to the descriptor placement when a robot cannot report its pose", async () => {
  const { bridge, builds } = mockBridge({}, {});

  await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1",
    content: { keyword: "computeInteractionSpace", value: ["Robot2"] }
  });

  assert.deepEqual(builds[0][1].placements, {});
});

test("release publishes the transformed retreat before waking the placement plan", async () => {
  const { bridge, beliefs } = mockBridge();
  bridge.enqueueMotion = async (action) => action();
  bridge.invokeLowerMove = async () => ({
    success: true,
    planner: "top_down_release",
    dropPoint: { x: -0.35, y: -0.8, z: 0.29 },
    retreatPoint: { x: -0.55, y: -0.8, z: 0.29 }
  });
  await bridge.handleCustomAgentRequests({
    targetAgentId: "ownLayer1", content: { keyword: "moveTo", value: [0, 0, 0] }
  });
  const retreat = beliefs.findIndex(([name]) => name === "placementRetreat");
  const moved = beliefs.findIndex(([name]) => name === "movedTo");
  assert.ok(retreat >= 0 && retreat < moved);
  assert.deepEqual(beliefs[retreat][1], [-0.55, -0.8, 0.29]);
});

test("clearance only publishes true for a successful positive core result", async () => {
  for (const result of [{ success: true, clear: true }, { success: true, clear: false }, { success: false, clear: true }]) {
    const { bridge, beliefs } = mockBridge({ checkHandoverClearance: result });
    await bridge.handleCustomAgentRequests({ targetAgentId: "ownLayer1", content: { keyword: "checkHandoverClearance", value: ["session-1"] } });
    assert.deepEqual(beliefs, [["handoverClearance", ["session-1", result.success && result.clear]]]);
  }
});

test("room contention retries one queued motion, reporting its blocker without repeated invocation logs", async (t) => {
  const output = t.mock.method(console, "log", () => {});
  const { bridge } = mockBridge();
  bridge.motionQueue = Promise.resolve();
  const order = [];
  let attempts = 0;
  const first = bridge.enqueueMotion(async () => {
    order.push("first");
    return ++attempts <= 2 ? { success: false, reason: "region_claimed",
      activeRobot: "Robot4", conflictRegion: "Robot2__Robot4" } : { success: true };
  }, { action: "moveTocartesianPosition" });
  const second = bridge.enqueueMotion(async () => { order.push("second"); return { success: true }; }, { action: "retreatFromHandover" });
  assert.equal((await first).success, true);
  assert.equal((await second).success, true);
  assert.deepEqual(order, ["first", "first", "first", "second"]);
  const logs = output.mock.calls.map((call) => call.arguments);
  assert.equal(logs.filter(([message]) => message.includes("invoking lower moveTocartesianPosition")).length, 1);
  const waiting = logs.filter(([message]) => message.includes("waiting for motion permission"));
  assert.equal(waiting.length, 1);
  assert.equal(waiting[0][1].activeRobot, "Robot4");
  assert.equal(waiting[0][1].conflictRegion, "Robot2__Robot4");
  assert.ok(logs.some(([message]) => message.includes("after 2 contention refusal(s)")));
});

test("a collision refusal is not retried as room contention", async () => {
  const { bridge } = mockBridge();
  bridge.motionQueue = Promise.resolve();
  let attempts = 0;
  const result = await bridge.enqueueMotion(async () => { attempts++; return { success: false, reason: "collision" }; });
  assert.equal(result.reason, "collision");
  assert.equal(attempts, 1);
});

test("the manifest selects the legacy home-between-handovers policy", async () => {
  for (const requested of [true, false, undefined]) {
    const { bridge, beliefs } = mockBridge();
    bridge.ctx.manifest.runtime = { returnHomeBetweenHandovers: requested };
    await bridge.handleCustomAgentRequests({ targetAgentId: "ownLayer1", content: {
      keyword: "getHandoverPolicy", value: []
    } });
    assert.deepEqual(beliefs, [["handoverReturnHome", [requested === true]]]);
  }
});

test("giver home uses the handover home action with the agent's home joints", async () => {
  const { bridge, invocations, beliefs } = mockBridge({ returnHomeAfterHandover: { success: true } });
  bridge.motionQueue = Promise.resolve();
  await bridge.handleCustomAgentRequests({ targetAgentId: "ownLayer1", content: {
    keyword: "goHomeAfterHandover", value: ["s1", { functor: "array", args: [0, 10, 0] }]
  } });
  assert.deepEqual(invocations, [["returnHomeAfterHandover", { session: "s1", homeJoints: [0, 10, 0] }]]);
  assert.deepEqual(beliefs, [["movedTo", ["home"]]]);
});

test("giver home accepts the nested array string that JaCaMo actually sends", async () => {
  const { bridge, invocations, beliefs } = mockBridge({ returnHomeAfterHandover: { success: true } });
  bridge.motionQueue = Promise.resolve();
  // JavaLibrary.termToObject([s1,array(...)]) returns an Object[] whose second
  // element is a String. The previous mock used JasonTermParser's different,
  // top-level representation and missed the live handover failure.
  await bridge.handleCustomAgentRequests({ targetAgentId: "ownLayer1", content: {
    keyword: "goHomeAfterHandover", value: ["s1", "array(0, -10, 2.5, 0, 1e-3, 0)"]
  } });
  assert.deepEqual(invocations, [["returnHomeAfterHandover", {
    session: "s1", homeJoints: [0, -10, 2.5, 0, 0.001, 0]
  }]]);
  assert.deepEqual(beliefs, [["movedTo", ["home"]]]);
});

test("malformed giver home joints report failure instead of issuing an invalid motion", async () => {
  for (const home of ["array(0,unknown,0)", "array(0,null,0)", "array()", "array(0,[1],0)"]) {
    const { bridge, invocations } = mockBridge();
    bridge.motionQueue = Promise.resolve();
    const failures = [];
    bridge.publishMotionFailure = async (result) => failures.push(result.reason);
    await bridge.handleCustomAgentRequests({ targetAgentId: "ownLayer1", content: {
      keyword: "goHomeAfterHandover", value: ["s1", home]
    } });
    assert.deepEqual(invocations, []);
    assert.deepEqual(failures, ["invalid_home_joints"]);
  }
});
