const assert = require("node:assert/strict");
const test = require("node:test");

const fs = require("node:fs");
const path = require("node:path");

const {
  baseJointFirstConfigs,
  beginHandoverSession,
  createContext,
  jointConfigStates,
  prepareRuntimeDriverScript
} = require("./lowerThingServer");
const { CellPeers, HandoverSessions, localPeer } = require("./cellPeers");

const { placeholderNames, render: renderDriver } = require("./core/luaDriver");
const { runtimeSource } = require("./core/runtimeSource");

// The runtime is the JavaScript that decides what to run plus the Lua it runs.
// The Lua used to be a template literal inside that JavaScript, so one read
// covered both; it now lives in src/generator/lua and is assembled here, so
// these assertions go on reading the runtime as a single surface -- and now
// read the driver as it is actually assembled rather than as it is spelled.
const lowerServerSource = runtimeSource();
const bridgeSource = fs.readFileSync(require.resolve("./robwotBridge"), "utf8");

// The driver is a Lua template embedded in a JS template literal, so these
// assertions read one function at a time rather than letting [\s\S]*? run on
// past its end.
function luaFunctionBody(source, name) {
  const start = source.indexOf(`function ${name}(`);
  assert.ok(start >= 0, `${name} not found in the runtime driver`);
  const end = source.indexOf("\nend", start);
  assert.ok(end > start, `${name} has no closing end`);
  return source.slice(start, end);
}

function context(name, owner = "Robot1") {
  return { manifest: { name, sharedEnvironment: { interactionSpaceOwner: owner } } };
}

// The cell address book, plus one session registry per robot. `holder` is who
// physically has the cube: ownership is answered by asking each arm's gripper,
// so the fixture models the scene rather than a cached owner field.
function cell(holder = "Robot1", names = ["Robot1", "Robot2"]) {
  const peers = new CellPeers({ cube: "grip_cube" });
  const sessions = new Map(names.map((name) => [name, new HandoverSessions(name)]));
  for (const name of names) {
    peers.register(name, localPeer({
      name,
      invoke: async (code) => (/getGripperState/.test(String(code))
        ? { holding: name === holder, object: "grip_cube" }
        : { success: true }),
      manifest: { name },
      sessions: sessions.get(name)
    }));
  }
  return { peers, sessions, sessionsFor: (name) => sessions.get(name) };
}

// beginHandoverSession arms the giver's handover pose lock in the driver, so it
// needs a scripting surface. These tests exercise the ownership rules; record
// the calls so the lock itself stays asserted.
function scripting() {
  const calls = [];
  return {
    calls,
    sim: {
      executeScriptString: async (code) => {
        calls.push(code);
        // The pose lock reports back the axis the robot will approach along.
        return [1, { success: true, axis: [0, 1, 0] }];
      }
    }
  };
}

function pairContext(name) {
  return {
    manifest: {
      name,
      sharedEnvironment: {
        interactionPairs: [
          { key: "Robot1__Robot2", robots: ["Robot1", "Robot2"], ownerRobot: "Robot1" },
          { key: "Robot2__Robot3", robots: ["Robot2", "Robot3"], ownerRobot: "Robot2" }
        ]
      }
    }
  };
}

test("only the configured interaction-space owner can register a physical session", async () => {
  const { peers, sessionsFor } = cell();
  const { sim } = scripting();
  assert.deepEqual(
    await beginHandoverSession(context("Robot2"), sim, 1, peers, sessionsFor("Robot2"), "session-1", "Robot1"),
    { success: false, reason: "not_interaction_space_owner" }
  );
  assert.equal(sessionsFor("Robot2").activeSession(), null);
});

test("a robot that is not physically holding the cube cannot open a handover", async () => {
  // Ownership comes from the scene, so a giver whose grip has slipped is
  // refused even though nothing in this process was told the grip had failed.
  const { peers, sessionsFor } = cell("Robot2");
  const { sim } = scripting();
  const result = await beginHandoverSession(
    context("Robot1"), sim, 1, peers, sessionsFor("Robot1"), "session-1", "Robot2"
  );
  assert.equal(result.success, false);
  assert.equal(result.reason, "source_not_cube_owner");
  assert.equal(result.owner, "Robot2");
});

test("two arms reporting a hold is a fault, not a tie to be broken", async () => {
  const peers = new CellPeers({ cube: "grip_cube" });
  const sessions = new HandoverSessions("Robot1");
  for (const name of ["Robot1", "Robot2"]) {
    peers.register(name, localPeer({
      name,
      invoke: async () => ({ holding: true, object: "grip_cube" }),
      manifest: { name },
      sessions: name === "Robot1" ? sessions : new HandoverSessions(name)
    }));
  }
  const { sim } = scripting();
  const result = await beginHandoverSession(context("Robot1"), sim, 1, peers, sessions, "session-1", "Robot2");
  assert.equal(result.success, false);
  assert.equal(result.reason, "cube_held_by_multiple_robots");
  assert.deepEqual(result.holders, ["Robot1", "Robot2"]);
});

test("session registration stores identities but no intersection geometry", async () => {
  const { peers, sessionsFor } = cell();
  const { sim, calls } = scripting();
  const result = await beginHandoverSession(
    context("Robot1"), sim, 1, peers, sessionsFor("Robot1"), "session-1", "Robot2"
  );
  assert.equal(result.success, true);
  // The giver's very next move is its leg of the rendezvous, so the tool axis
  // has to be pinned before it starts.
  assert.ok(
    calls.some((call) => /womatSetHandoverPoseLock\(true\)/.test(String(call))),
    "registering a session must arm the giver's handover pose lock"
  );
  const expected = {
    id: "session-1",
    sourceRobot: "Robot1",
    targetRobot: "Robot2",
    state: "registered",
    // The receiver's approach is defined relative to the giver's tool, so the
    // session has to carry it. Still no intersection geometry.
    giverAxis: [0, 1, 0]
  };
  assert.deepEqual(sessionsFor("Robot1").get("session-1"), expected);
  // The receiver holds its own copy, because it is the one that has to arm its
  // approach against the giver's axis. Nothing is shared between the two.
  assert.equal(result.receiverNotified, true);
  assert.deepEqual(sessionsFor("Robot2").get("session-1"), expected);
  assert.notEqual(sessionsFor("Robot1").get("session-1"), sessionsFor("Robot2").get("session-1"));
  assert.equal("cubeWorld" in sessionsFor("Robot1").get("session-1"), false);
});

test("a handover the giver closes does not close the receiver's copy", async () => {
  const { peers, sessionsFor } = cell();
  const { sim } = scripting();
  await beginHandoverSession(context("Robot1"), sim, 1, peers, sessionsFor("Robot1"), "session-1", "Robot2");
  sessionsFor("Robot1").close("session-1");
  assert.equal(sessionsFor("Robot1").get("session-1"), null);
  assert.ok(sessionsFor("Robot2").get("session-1"), "each robot retires its own session on its own schedule");
});

test("pair ownership allows Robot2 to coordinate Robot2-Robot3 but not Robot1-Robot2", async () => {
  const { peers, sessionsFor } = cell("Robot2", ["Robot1", "Robot2", "Robot3"]);
  const { sim } = scripting();
  const rejected = await beginHandoverSession(
    pairContext("Robot2"), sim, 1, peers, sessionsFor("Robot2"), "session-12", "Robot1"
  );
  assert.equal(rejected.success, false);
  assert.equal(rejected.reason, "not_interaction_space_owner");

  const accepted = await beginHandoverSession(
    pairContext("Robot2"), sim, 1, peers, sessionsFor("Robot2"), "session-23", "Robot3"
  );
  assert.equal(accepted.success, true);
  assert.equal(accepted.sourceRobot, "Robot2");
  assert.equal(accepted.targetRobot, "Robot3");
});

test("base-joint-first path holds the other joints until joint 1 is clear", () => {
  const options = {
    baseJointFirstMinDeltaDeg: 2,
    jointMaxVelocityDeg: 60,
    stepDelayMs: 25,
    jointSteps: 4
  };
  const result = baseJointFirstConfigs([0, 0, 0], [Math.PI / 2, 1, -1], options);
  assert.ok(result);
  const states = [];
  for (let index = 0; index < result.configs.length; index += 3) {
    states.push(result.configs.slice(index, index + 3));
  }
  assert.ok(states.slice(0, result.baseSteps).every((state) => state[1] === 0 && state[2] === 0));
  assert.ok(Math.abs(states[result.baseSteps - 1][0] - Math.PI / 2) < 1e-12);
  assert.deepEqual(states.at(-1), [Math.PI / 2, 1, -1]);
});

test("staged flat paths are converted to nested Lua preflight configurations", () => {
  assert.deepEqual(jointConfigStates([1, 2, 3, 4, 5, 6], 3), [[1, 2, 3], [4, 5, 6]]);
  assert.throws(() => jointConfigStates([1, 2, 3, 4], 3), /whole number of configurations/);
  assert.doesNotMatch(lowerServerSource, /womatPreflightJointPath\(\$\{luaArray\(staged\.configs\)\}/);
});

test("runtime records coroutine failures without opening CoppeliaSim's abort dialog", () => {
  assert.match(lowerServerSource, /womatRecordCoroutineFailure\(thread, ok, result\)/);
  assert.match(lowerServerSource, /if ok then\s*return false/);
  assert.doesNotMatch(lowerServerSource, /if err then error\(debug\.traceback\(corout_/);
});

test("Cartesian fallback uses position-only IK with the configured search budget", () => {
  assert.match(
    lowerServerSource,
    /womatFindGoalConfig\(x, y, z, maxTime\)[\s\S]*?simIK\.constraint_position[\s\S]*?simIK\.findConfig[\s\S]*?maxTime or 4/
  );
  assert.match(lowerServerSource, /reason: "cartesian_goal_config_unavailable"/);
  assert.match(
    lowerServerSource,
    /tryDirectJointTrajectory\(sim, scriptHandle, goal\.config\.map\(Number\), options\)/
  );
  assert.match(lowerServerSource, /planned = await planAndFollow\([\s\S]*?goal\.config\.map\(Number\)/);
  assert.match(lowerServerSource, /reached !== true && requireJointGoal/);
  assert.match(
    lowerServerSource,
    /pcall\(simIK\.handleGroup, env2, group2\)[\s\S]*?womatGoalConfigValid\(nearest\)[\s\S]*?if config == nil then[\s\S]*?simIK\.findConfig/
  );
});

test("joint-trajectory Cartesian mode plans before direct visible IK motion", () => {
  const plannedFirst = lowerServerSource.indexOf("if (options.preferJointTrajectoryForCartesian || options.preferOmplForCartesian)");
  const directStart = lowerServerSource.indexOf("const start = await getCartesian(sim, scriptHandle)", plannedFirst);
  assert.ok(plannedFirst >= 0);
  assert.ok(plannedFirst < directStart);
  assert.match(
    lowerServerSource.slice(plannedFirst, directStart),
    /cartesianPlannerFallback\(sim, scriptHandle, point, options[\s\S]*?ompl_first_preferred/
  );
});

test("planned Cartesian trajectories suppress modal errors and shorten RRT detours", () => {
  assert.match(lowerServerSource, /setBoolProperty\(await sim\.handle_scene, "pauseSimulationAtError", false\)/);
  assert.match(lowerServerSource, /womatShortcutFlatPath\(path, dof, robotCollection, obstacleCollection\)/);
  assert.match(lowerServerSource, /planner: "collision_checked_joint_trajectory"/);
});

test("simple smooth mode retains guarded execution and configured Cartesian planning", () => {
  assert.match(lowerServerSource, /simpleSmoothMotion: runtime\.simpleSmoothMotion === true/);
  assert.match(
    lowerServerSource,
    /options\.simpleSmoothMotion\s*\? simpleSmoothCartesianMove\(sim, scriptHandle, point, workspace, motionOptions\)/
  );
  assert.match(
    lowerServerSource,
    /options\.simpleSmoothMotion\s*\? simpleSmoothJointMove\(sim, scriptHandle, td, radians, motionOptions\)/
  );
  assert.match(lowerServerSource, /function womatStartSimpleCartesian\(x, y, z\)/);
  assert.match(lowerServerSource, /function womatStartSimpleJoint\(targetConf\)/);
  assert.match(
    lowerServerSource,
    /function womatEnsureSimplePositionIk\(\)[\s\S]*?simIK\.constraint_position/
  );
  assert.match(lowerServerSource, /const workspace = loadWorkspaceGrid\(ctx, options\)/);
  assert.doesNotMatch(lowerServerSource, /simpleSmoothMotion \? null : readWorkspaceSafety/);
  assert.match(lowerServerSource, /function womatAttachMarkerReference\(\)/);
  // No jaw correction any more: simTip is the tool control point, so the
  // commanded position is where the gripper goes.
  assert.match(
    lowerServerSource,
    /execution = await executeSimpleCartesianPose\(sim, scriptHandle, point, workspace, options\);/
  );
  assert.match(lowerServerSource, /function womatCalibratedAttachPosition\(\)/);
  assert.doesNotMatch(lowerServerSource, /motionTarget/);
  assert.doesNotMatch(lowerServerSource, /simple_gripper_approach_preplan/);
  assert.match(lowerServerSource, /function womatRobotSelfCollision\(\)/);
  assert.match(lowerServerSource, /womatRobotObstacleCollision\(\) or womatRobotSelfCollision\(\)/);
  assert.match(lowerServerSource, /if womatSimpleSafetyEnabled and womatCommandedPostureUnsafe\(womatCommandedConfig\(jointHandles\)\) then/);
  assert.doesNotMatch(lowerServerSource, /womatSimpleSafetyEnabled and womatRobotMotionUnsafe\(\)/);
  assert.match(lowerServerSource, /function womatGripperClearanceUnsafe\(obstacleCollection\)/);
  assert.match(lowerServerSource, /colliding=womatGripperClearanceUnsafe\(obstacleCollection\)/);
  assert.match(lowerServerSource, /simple_cartesian_path_collision_detected/);
  assert.match(lowerServerSource, /simple_joint_path_collision_detected/);
  assert.match(lowerServerSource, /if clearance > 0 and not womatPickupApproachEnabled then/);
  assert.doesNotMatch(lowerServerSource, /womatPickupApproachEnabled[\s\S]{0,250}womatIsDescendantOf\(shape, gripSupport\)[\s\S]{0,80}return false/);
  assert.doesNotMatch(lowerServerSource, /womatRemoveGripperScripts\(\)\s*\n\s*womatRemoveGeneratedVisualJaws/);
  assert.match(lowerServerSource, /if #existingJoints == 0 then[\s\S]*?sim\.loadModel/);
});

test("workspace sampling and runtime safety include the physical gripper envelope", () => {
  assert.match(bridgeSource, /womatCreateWorkspaceRobotCollisionCollection\(base\)/);
  assert.match(bridgeSource, /sim\.getObjectsInTree\(gripper, sim\.object_shape_type, 0\)/);
  assert.match(bridgeSource, /sim\.addItemToCollection\(womatWorkspaceGripperCollection/);
  assert.match(bridgeSource, /sim\.checkDistance,[\s\S]*?womatWorkspaceGripperCollection/);
  assert.match(lowerServerSource, /gripperCollisionClearance \?\? 0\.005/);
  assert.match(lowerServerSource, /function womatGripperClearanceUnsafe\(obstacleCollection\)/);
  assert.match(lowerServerSource, /if womatPickupApproachEnabled then\s*clearance=\$\{pickupGripperClearance\}/);
});

test("the robot's tool control point is its gripper's jaw midpoint", () => {
  // The change this locks in: simTip is rebound to a dummy at the contact-pad
  // midpoint, so a commanded Cartesian position IS where the object ends up.
  // Every offset, correction and calibration that used to bridge the gap
  // between the flange and the grasp point is gone with it.
  const tcp = luaFunctionBody(lowerServerSource, "womatEnsureToolControlPoint");
  assert.match(tcp, /simTip=dummy/);
  assert.match(tcp, /womatInvalidateIkEnvironments\(\)/);
  // The gripper bolts to the flange; parenting it to its own tool control point
  // would make it its own ancestor.
  assert.match(tcp, /sim\.setObjectParent\(handle, womatFlangeTip, true\)/);
  assert.match(
    luaFunctionBody(lowerServerSource, "womatEnsureTcpGripperModel"),
    /\n        sim\.setObjectParent\(tcpGripper, womatFlangeTip, false\)/
  );

  // None of the compensation machinery survives.
  for (const gone of [
    "womatGripAlignmentCorrection",
    "womatHandoverJawOffsetWorld",
    "womatJawCenterOffsetInTip",
    "womatSolveRestrictedIk",
    "womatSetHandoverCalibration"
  ]) {
    assert.doesNotMatch(lowerServerSource, new RegExp(gone), `${gone} should have been removed`);
  }
  assert.doesNotMatch(lowerServerSource, /jawCenterOffset/);

  // The pickup still biases the grasp up the cube's height; that is a real
  // physical choice, not offset compensation.
  assert.match(lowerServerSource, /graspHeightOffset/);
  const { createContext } = require("./lowerThingServer");
  for (const name of ["Robot1", "Robot2", "Robot3"]) {
    const grip = createContext(`generated/robots/${name}/manifest.json`).manifest.runtime.gripper;
    assert.equal(grip.jawCenterOffset, undefined, `${name} still carries a jaw offset`);
    assert.equal(grip.toolAxis.length, 3, `${name} needs a tool direction`);
    const length = Math.hypot(...grip.toolAxis);
    assert.ok(Math.abs(length - 1) < 1e-3, `${name} tool axis must be a direction, got length ${length}`);
  }
});


test("safety collections are cached and invalidated instead of rebuilt per check", () => {
  // Rebuilding these per call also discarded CoppeliaSim's per-collection
  // collision and distance caches, which is what made in-motion checking
  // cost more than the simulation step it ran in.
  assert.match(lowerServerSource, /function womatInvalidateSafetyCollections\(\)/);
  assert.match(lowerServerSource, /if womatCachedObstacleCollection ~= nil then\s*return womatCachedObstacleCollection/);
  assert.match(lowerServerSource, /if womatCachedRobotCollection ~= nil then\s*return womatCachedRobotCollection/);
  assert.match(lowerServerSource, /function womatPrepareSelfCollisionCollections\(\)/);
  assert.doesNotMatch(lowerServerSource, /sim\.destroyCollection\(robotCollection\)/);
  assert.doesNotMatch(lowerServerSource, /sim\.destroyCollection\(obstacleCollection\)/);
  for (const mutation of [
    /heldGripObject=object\s*\n\s*gripperClosed=true\s*\n\s*womatInvalidateSafetyCollections\(\)/,
    // Every field that changes what counts as an obstacle, then the
    // invalidation -- the peer's pad alias joined them when the receiver
    // started measuring which way the giver's jaws close.
    /womatHandoverIgnoredGripperAlias=alias\n(?:.*\n)*?\s*womatInvalidateSafetyCollections\(\)/
  ]) {
    assert.match(lowerServerSource, mutation);
  }
  // Both self-collision collections are built once and cached, like the others.
  assert.match(
    lowerServerSource,
    /if womatCachedToolCollection ~= nil then\s*return womatCachedToolCollection, womatCachedDistantLinkCollection/
  );
});

test("the in-motion watchdog tests contact every step, on cached collections", () => {
  // Striding contact is not a safe economy: at 0.175 m/s a stride of 4 leaves
  // 35 mm between tests, and both the wall and the cube are thinner than that,
  // so the arm passes through with no test landing inside.
  const options = require("./lowerThingServer").createRuntimeOptions(
    require("./lowerThingServer").createContext("generated/robots/Robot1/manifest.json")
  );
  assert.equal(options.inMotionCollisionStride, 1);
  const watchdog = luaFunctionBody(lowerServerSource, "womatInMotionUnsafe");
  assert.match(watchdog, /womatRobotSelfCollision\(\)/);
  assert.match(watchdog, /local robotCollection=womatRobotCollisionCollection\(\)/);
  assert.match(watchdog, /womatCollectionsTouch\(robotCollection, obstacleCollection\)/);
  // Box-culled, pair by pair -- and the whole-collection check whenever a
  // shape cannot be bounded or a pair cannot be asked on its own.
  const touch = luaFunctionBody(lowerServerSource, "womatCollectionsTouch");
  assert.match(touch, /return sim\.checkCollision\(collectionA, collectionB\) ~= 0/);
  assert.match(touch, /sim\.checkCollision\(a\.shapes\[i\]\.handle, b\.shapes\[j\]\.handle\)/);
  // The dear query stays off the per-step path unless explicitly enabled.
  assert.match(watchdog, /womatInMotionClearanceStride > 0/);
  assert.equal(options.inMotionClearanceStride, 0);
  // No clearance distance query on the executed trajectory: that is the
  // expensive part, and the swept preflight already covers it.
  assert.doesNotMatch(luaFunctionBody(lowerServerSource, "womatInMotionUnsafe"), /checkDistance/);
  assert.match(lowerServerSource, /function womatStartSimpleCartesian\(x, y, z\)\s*\n\s*womatMotionError=nil\s*\n\s*womatResetInMotionSafety\(\)/);
});

test("simple Cartesian commands are swept once, with the IK that will execute them", () => {
  // Preflight and execution must drive the IK through one entry point, or
  // the sweep validates postures the motion never takes.
  assert.match(luaFunctionBody(lowerServerSource, "womatSweepCartesianPath"), /womatApplyCartesianTarget/);
  assert.match(luaFunctionBody(lowerServerSource, "womatApplyCartesianTarget"), /womatSimpleIkEnv, womatSimpleIkGroup/);
  // General moves stay position-only with a free wrist. Orientation is
  // constrained only in the pick planner, and even there the roll is left
  // free -- constraining all three rotational DOF is what diverged at the
  // doubly singular home pose.
  assert.doesNotMatch(luaFunctionBody(lowerServerSource, "womatEnsureSimplePositionIk"), /constraint_alpha_beta|constraint_pose/);
  // Exactly one chunk of the driver builds a pose-constrained element, and it
  // is the handover approach, and only when a roll was asked for. Everything
  // else -- the pick included -- leaves the rotation about the tool free,
  // because pinning all three rotational DOF is what diverged at the doubly
  // singular home pose. A handover pays that risk knowingly: two grippers
  // closing on one cube have to place their fingers, and the roll is the only
  // thing that decides whether they interleave or collide.
  const driverTemplate = require("./core/luaDriver").driverTemplate();
  const poseConstrained = driverTemplate
    .split(/^function /m)
    .filter((chunk) => /simIK\.constraint_pose/.test(chunk))
    .map((chunk) => chunk.slice(0, chunk.indexOf("(")));
  assert.deepEqual(poseConstrained, ["womatPlanHandoverApproach"],
    "only the handover approach may constrain the full pose");
  // And there only behind the roll, so a pair that does not search one keeps
  // solving the weaker, better-conditioned problem.
  assert.match(
    luaFunctionBody(driverTemplate, "womatPlanHandoverApproach"),
    /if rollRad ~= nil then\s*\n\s*constraint=simIK\.constraint_pose/
  );
  assert.match(
    lowerServerSource,
    /const VENDOR_IK_ELEMENT =[\s\S]*?simIK\.constraint_pose\)";/
  );
  assert.match(
    luaFunctionBody(lowerServerSource, "womatPlanTopDownPick"),
    /simIK\.constraint_position \+ simIK\.constraint_alpha_beta/
  );
  assert.doesNotMatch(luaFunctionBody(lowerServerSource, "womatPlanTopDownPick"), /constraint_pose/);
  // The grasp is lifted off the cube centre because the gripper body hangs
  // ~26 mm below the jaw line while the cube centre is only ~25 mm above the
  // support top; aiming at the centre drives the body into the support.
  assert.match(lowerServerSource, /graspHeightOffset/);
  const gripper = require("./lowerThingServer")
    .createContext("generated/robots/Robot1/manifest.json").manifest.runtime.gripper;
  const jawZ = gripper.objectPosition[2] + gripper.graspHeightOffset;
  // 25.9 mm is the RG2's measured reach below its jaw centre.
  assert.ok(jawZ - 0.0259 > 0.160, "grasp height does not clear the support top");
  assert.ok(jawZ < gripper.objectPosition[2] + gripper.objectSize[2] / 2, "grasp is above the cube");
  assert.ok(Math.abs(gripper.graspHeightOffset) < gripper.gripThreshold, "grasp is outside gripThreshold");

  // The pick is two legs: hover above the grasp, then descend onto it.
  assert.match(lowerServerSource, /hoverWorld=\{graspWorld\[1\], graspWorld\[2\], graspWorld\[3\]\+approachHeight\}/);
  assert.match(
    lowerServerSource,
    /async function executeSimpleCartesianPose\(sim, scriptHandle, target, workspace, options\) \{[\s\S]*?guardSimpleCartesianPose\(sim, scriptHandle, target, workspace, options\)/
  );
  assert.match(lowerServerSource, /function womatObstacleLocalBoxes\(\)/);
  assert.match(lowerServerSource, /preflightJointPath\(sim, scriptHandle, start, targetRadians, options, steps\)[\s\S]*?womatStartSimpleJoint/);
});

test("the generated driver defines the safety functions it calls", () => {
  // Asserting on the assembled Lua rather than on the JS template catches a
  // helper that was renamed on one side of the string interpolation only.
  const { createContext, prepareRuntimeDriverScript } = require("./lowerThingServer");
  const ctx = createContext("generated/robots/Robot1/manifest.json");
  const driver = prepareRuntimeDriverScript(ctx, {
    baseOffset: [1.2, 0, 0],
    baseOrientation: [0, 0, 1.57],
    obstacleNames: ["Robot1_wall", "Robot2_wall", "Robot2", "Robot3"]
  });

  const defined = new Set([...driver.matchAll(/^function\s+([A-Za-z_][\w]*)\s*\(/gm)].map((m) => m[1]));
  for (const name of [
    "womatInvalidateSafetyCollections",
    "womatPrepareSelfCollisionCollections",
    "womatInMotionUnsafe",
    "womatResetInMotionSafety",
    "womatObstacleLocalBoxes",
    "womatSweepCartesianPath",
    "womatApplyCartesianTarget",
    "womatRunSweep",
    "womatSweepSampleUnsafe",
    "womatRestoreSweepPosture",
    "womatBlockingObstacleAlias",
    "womatSceneObstacleContext"
  ]) {
    assert.ok(defined.has(name), `${name} is called by the runtime but never defined`);
  }

  // Every womat* call in the driver must resolve to a definition in it.
  const called = new Set([...driver.matchAll(/\b(womat[A-Za-z_]\w*)\s*\(/g)].map((m) => m[1]));
  const assigned = new Set([...driver.matchAll(/^(womat[A-Za-z_]\w*)\s*=/gm)].map((m) => m[1]));
  const missing = [...called].filter((name) => !defined.has(name) && !assigned.has(name));
  assert.deepEqual(missing, [], `undefined helpers referenced by the driver: ${missing.join(", ")}`);
});

test("the pick is planned before it moves, and driven off the contact pads", () => {
  const plan = luaFunctionBody(lowerServerSource, "womatPlanTopDownPick");
  // Randomised search for the approach pose, not gradient descent: the home
  // pose is the wrist and elbow singularity together.
  assert.match(plan, /simIK\.findConfig/);
  // The IK plugin has its own world. Scene target/joint changes are invisible
  // to it until explicitly synchronized.
  assert.match(plan, /womatSetDownTarget\(hoverWorld\)\s*[\s\S]*?simIK\.syncFromSim\(env, \{group\}\)\s*[\s\S]*?simIK\.findConfig/);
  assert.match(
    plan,
    /sim\.setJointPosition\(jointHandles\[i\], hover\[i\]\)[\s\S]*?simIK\.syncFromSim\(env, \{group\}\)[\s\S]*?for step=1,steps,1 do/
  );
  assert.match(
    plan,
    /womatSetDownTarget\(\{[\s\S]*?simIK\.syncFromSim\(env, \{group\}\)\s*[\s\S]*?simIK\.handleGroup/
  );
  // The descent is solved continuously FROM the hover posture, so the wrist
  // cannot flip between waypoints.
  assert.match(plan, /simIK\.handleGroup/);
  assert.match(plan, /womatGoalConfigValid\(config\)/);
  // Nothing moves while planning.
  assert.match(plan, /sim\.setJointPosition\(jointHandles\[i\], saved\[i\]\)/);

  // The tool control point is the midpoint of the contact pads: the RG2 attach
  // point sits at the base of the fingers, ~43 mm short of where an object is
  // actually held.
  assert.match(luaFunctionBody(lowerServerSource, "womatToolControlPointInFlange"), /graspPadAlias/);
  assert.match(luaFunctionBody(lowerServerSource, "womatEnsureToolControlPoint"), /womatToolControlPointInFlange\(\)/);

  // A pickup routes to the planner instead of the general Cartesian move.
  assert.match(lowerServerSource, /if \(options\.enableTopDownPick\)[\s\S]*?topDownPick\(sim, scriptHandle, point, workspace, options\)/);
  // Leg 1 is a joint move (cannot diverge); leg 2 is the pre-validated descent.
  assert.match(lowerServerSource, /womatStartSimpleJoint\(\$\{luaArray\(hover\)\}\)/);
  assert.match(lowerServerSource, /womatFollowDescent\(\$\{luaArray\(descent\)\}\)/);

  // The shipped geometry must actually clear the support, measured from the
  // simulator: pads at +27.9..+57.4 mm and fingertips at +59.9 mm past the
  // attach point, support top at 0.160.
  const g = require("./lowerThingServer")
    .createContext("generated/robots/Robot1/manifest.json").manifest.runtime.gripper;
  const tcpFromAttach = (0.0279 + 0.0574) / 2;
  const tcpZ = g.objectPosition[2] + g.graspHeightOffset;
  const fingertipZ = tcpZ + tcpFromAttach - 0.0599;
  assert.ok(fingertipZ > 0.160, `fingertips at ${fingertipZ} do not clear the support top`);
  const padLow = tcpZ + tcpFromAttach - 0.0574;
  const padHigh = tcpZ + tcpFromAttach - 0.0279;
  const half = g.objectSize[2] / 2;
  const overlap = Math.min(padHigh, g.objectPosition[2] + half) - Math.max(padLow, g.objectPosition[2] - half);
  assert.ok(overlap > 0.015, `pads only overlap the object by ${overlap} m`);
});

test("the receiver has to physically reach the cube before the transfer", () => {
  const accept = luaFunctionBody(lowerServerSource, "acceptTransferredGripObject");
  // The cube is never moved to meet the gripper. Re-parent in place only.
  assert.doesNotMatch(
    accept,
    /sim\.setObjectPosition\(object, sim\.handle_world/,
    "the transfer must not teleport the cube into the receiver's jaws"
  );
  // Measured pad-to-cube distance, checked against a tolerance, refused above
  // it. The reference is the contact pads, not the RG2 attach point ~43 mm
  // behind them, which would reject a perfectly centred cube on its own.
  assert.match(accept, /womatDistance\(graspPoint, cubePosition\)/);
  assert.match(accept, /womatToolControlPointWorld\(\)/);
  assert.match(accept, /reason="receiver_not_at_cube"/);
  assert.match(
    accept,
    /reach > \$\{handoverGraspTolerance\} then[\s\S]*?return \{\s*success=false/
  );
  // The gate can be switched off for a robot that is not being steered onto the
  // cube at all, but absence of the flag means ON -- a missing manifest key must
  // never silently reinstate the teleport.
  assert.match(
    lowerServerSource,
    /enableHandoverGraspGate = ctx\.manifest\.runtime\?\.enableHandoverGraspGate !== false/
  );
  const { createContext } = require("./lowerThingServer");
  const disabled = ["Robot1", "Robot2", "Robot3"].filter((name) =>
    createContext(`generated/robots/${name}/manifest.json`).manifest.runtime
      .enableHandoverGraspGate === false);
  assert.deepEqual(disabled, [], `grasp gate is disabled for ${disabled.join(", ")}`);
  // The gate has to run before the cube changes hands.
  assert.ok(
    accept.indexOf("receiver_not_at_cube") < accept.indexOf("sim.setObjectParent(object, tcpGripper"),
    "the reach check must precede the re-parent"
  );
  // The reported result no longer advertises an unvalidated snap.
  assert.match(lowerServerSource, /validation: "receiver_jaw_distance"/);
  assert.doesNotMatch(lowerServerSource, /validation: "disabled"/);
});

test("both handover roles approach the rendezvous with the tool axis pinned", () => {
  const plan = luaFunctionBody(lowerServerSource, "womatPlanHandoverApproach");
  // Position alone is what let the wrist drift and invalidated the plan's
  // constant jaw offset. Pin the tool axis, leave the roll about it free.
  assert.match(plan, /simIK\.constraint_position \+ simIK\.constraint_alpha_beta/);
  // Seeded solve first, randomised restarts only as the escape hatch, and the
  // result verified against the scene -- same shape as the pick.
  const leg = luaFunctionBody(lowerServerSource, "womatSolveHandoverStandoff");
  assert.match(leg, /simIK\.handleGroup[\s\S]*?simIK\.findConfigs/);
  assert.match(leg, /womatPickConfigValid/);
  assert.match(plan, /sim\.setJointPosition\(jointHandles\[i\], saved\[i\]\)/);

  // The axis is the configured jaw offset as a direction: exactly the vector
  // interactionSpace.targetForRobot subtracts from the shared cube point.
  // A direction, not an offset: there is no length left to carry.
  const tool = luaFunctionBody(lowerServerSource, "womatToolAxisWorld");
  assert.match(tool, /womatLocalVectorToWorld\(configured\[1\], configured\[2\], configured\[3\]\)/);
  const axis = luaFunctionBody(lowerServerSource, "womatHandoverApproachAxisWorld");
  assert.match(axis, /womatToolAxisWorld\(\)/);

  // Driven off the same contact-pad dummy the pick uses, onto the cube point
  // recovered from the plan's tip target -- not onto the tip target itself,
  // which is measured to the attach point ~43 mm behind the pads.
  const rendezvous = luaFunctionBody(lowerServerSource, "womatPlanHandoverApproach");
  assert.match(rendezvous, /womatEnsureToolControlPoint\(\)/);
  // The commanded point IS the grasp point now.
  assert.match(rendezvous, /graspWorld=\{tipWorld\[1\], tipWorld\[2\], tipWorld\[3\]\}/);
  assert.doesNotMatch(
    rendezvous,
    /leg1\.world\[1\]\+\(tipWorld\[1\]/,
    "the approach must interpolate towards the cube point, not the raw tip target"
  );
  assert.match(rendezvous, /leg1\.world\[1\]\+\(graspWorld\[1\]-leg1\.world\[1\]\)\*alpha/);

  // A locked move routes to the planner instead of the free-wrist Cartesian move.
  assert.match(
    lowerServerSource,
    /womatHandoverPoseEnabled"\);\s*if \(handoverLocked === true\) \{\s*return handoverApproach\(/
  );
  // Receiver arms its lock in setHandoverApproach; giver arms it at session
  // registration, which is the step immediately before its own approach.
  assert.match(
    lowerServerSource,
    /womatSetHandoverApproachGripper\([\s\S]*?luaString\(giverGripper\)[\s\S]*?setHandoverPoseLock\(\s*\n?\s*sim, scriptHandle, true, candidates\[0\] \|\| null, candidates\.slice\(1\)/
  );
  assert.match(lowerServerSource, /if \(!enabled\) \{\s*await setHandoverPoseLock\(sim, scriptHandle, false\)/);
});

test("the handover rendezvous is a standoff plus a straight line in", () => {
  const leg1 = luaFunctionBody(lowerServerSource, "womatSolveHandoverStandoff");
  // Leg 1 solves a pose backed off along the approach axis, not the rendezvous
  // itself: a joint line all the way in sweeps the volume the giver occupies.
  assert.match(leg1, /graspWorld\[1\]-axis\[1\]\*distance/);
  // Shorter standoffs are tried before paying for a randomised search, and the
  // search takes the config CLOSEST to the current posture -- findConfig
  // returns an arbitrary branch, which is what produced the long contortions on
  // the way to the standoff.
  assert.match(leg1, /local fractions=\{1\.0, 0\.7, 0\.45\}/);
  assert.match(leg1, /simIK\.findConfigs/);
  assert.doesNotMatch(leg1, /simIK\.findConfig,/);

  const plan = luaFunctionBody(lowerServerSource, "womatPlanHandoverApproach");
  // Leg 2 interpolates the TIP in a straight line while re-applying the same
  // axis constraint at every waypoint, so the tool orientation is held.
  assert.match(
    plan,
    /for step=1,steps,1 do[\s\S]*?womatSetApproachTarget\(waypoint, approachAxis, rollRad\)[\s\S]*?simIK\.handleGroup/
  );
  assert.match(plan, /womatPickConfigFailure\(config\)/);
  // A blocked waypoint says which of the three things went wrong, not
  // "unreachable or blocked".
  assert.match(plan, /reason="handover_approach_ik_diverged"/);
  assert.match(plan, /reason="handover_approach_"\.\.failure/);
  assert.match(plan, /womatBlockingObstacleAlias\(\)/);
  // One direction per call: a loop in here multiplies the worst case of a
  // single blocking script call by the number of candidates.
  assert.doesNotMatch(plan, /for index=1,#candidates/);

  // Leg 1 falls back to the sampled planner rather than failing when the
  // straight joint line is blocked; leg 2 replays the pre-validated samples.
  assert.match(
    lowerServerSource,
    /if \(!safety\.ok\) \{[\s\S]*?womatPlanAndFollowJointPath\(\$\{luaArray\(standoff\)\}/
  );
  assert.match(lowerServerSource, /womatFollowDescent\(\$\{luaArray\(approach\)\}\)/);
});

test("the way out of a rendezvous is the way in, played backwards", () => {
  // The pose lock stays armed while the arm leaves -- it is also what keeps the
  // other robot's gripper out of the obstacle set -- and while it is armed
  // every Cartesian move is dispatched as an approach. A plain move out was
  // therefore planned as an arrival at the retreat point: standoff first, then
  // straight in, driving the arm forward through the rendezvous it was leaving.
  assert.match(lowerServerSource, /const executedRendezvousLeg = new Map\(\);/);
  // One entry per robot, and a failed or superseded approach leaves none.
  assert.match(
    lowerServerSource,
    /async function handoverApproach\(sim, scriptHandle, point, workspace, options\) \{[\s\S]*?executedRendezvousLeg\.delete\(scriptHandle\);/
  );
  assert.match(
    lowerServerSource,
    /executedRendezvousLeg\.set\(scriptHandle, \{\s*\n\s*kind: "pose_locked_approach",\s*\n\s*standoff,\s*\n\s*approach,/
  );
  // An arm that runs unlocked never plans two legs, so there is no joint path
  // to reverse -- but there is still a straight line it came in on, and local
  // -X is not it. Both kinds are recorded, and a joint move (which is how an
  // arm leaves for good) invalidates either.
  assert.match(
    lowerServerSource,
    /executedRendezvousLeg\.set\(scriptHandle, \{\s*\n\s*kind: "cartesian_leg",[\s\S]*?to: point/
  );
  assert.match(
    lowerServerSource,
    /async function simpleSmoothJointMove\([^)]*\) \{[\s\S]*?executedRendezvousLeg\.delete\(scriptHandle\);/
  );
  const cartesianReverse = lowerServerSource.slice(lowerServerSource.indexOf("async function reverseCartesianLeg("));
  const cartesianBody = cartesianReverse.slice(0, cartesianReverse.indexOf("\n}\n"));
  assert.match(cartesianBody, /reason: "handover_approach_path_stale"/);
  // Re-admitted and re-swept on the way out: a cube has changed hands since.
  assert.match(cartesianBody, /executeSimpleCartesianPose\(sim, scriptHandle, record\.from, workspace, options\)/);

  const retreat = lowerServerSource.slice(
    lowerServerSource.indexOf("async function handoverRetreat(")
  );
  const body = retreat.slice(0, retreat.indexOf("\n}\n"));
  // A jump into the first waypoint is the one thing nothing has checked, so the
  // replay is refused unless the arm is still standing where the approach left it.
  assert.match(body, /reason: "handover_approach_path_stale"/);
  assert.match(body, /options\.handoverRetreatDriftTolerance/);
  // Every state of the way in, last to first, and then the standoff -- which is
  // not in the approach array, because leg 2 was solved from it.
  assert.match(body, /for \(let state = states - 1; state >= 0; state -= 1\)/);
  assert.match(body, /reversed\.push\(record\.standoff\[joint\]\)/);
  assert.match(body, /womatFollowDescent\(\$\{luaArray\(reversed\)\}\)/);
  // Walked once. Replaying it a second time would drive the arm from wherever
  // it now stands back through poses it has left.
  assert.match(body, /executedRendezvousLeg\.delete\(scriptHandle\);\s*\n\s*return \{\s*\n\s*success: true/);
  // Nothing plans here: no IK, no standoff search, no axis choice.
  assert.doesNotMatch(body, /womatPlanHandoverApproach/);

  // Reachable as its own affordance, so the agent asks to LEAVE rather than
  // asking to move to a point that gets re-planned as an arrival.
  assert.match(lowerServerSource, /td\.actions\.retreatFromHandover = \{/);
  assert.match(
    lowerServerSource,
    /runHandoverMotion\(\s*\n?\s*"retreatFromHandover",\s*\n?\s*\(motionOptions\) => handoverRetreat\(sim, scriptHandle, workspace, motionOptions\)/
  );
});

test("a blocked path names which robot was hit, not just the link", () => {
  // "link4_respondable" is an alias every arm in the cell shares.
  const owner = luaFunctionBody(lowerServerSource, "womatObstacleOwnerAlias");
  assert.match(owner, /while current ~= -1 do[\s\S]*?root=current/);
  assert.match(
    luaFunctionBody(lowerServerSource, "womatBlockingObstacleAlias"),
    /return womatPlainAlias\(shapes\[i\]\), womatObstacleOwnerAlias\(shapes\[i\]\)/
  );
  assert.match(lowerServerSource, /obstacleOwner: swept\?\.obstacleOwner/);
  // Step 6 of 60 and step 6 of 6 mean different things; report which.
  assert.match(lowerServerSource, /blockedOfSamples: bounded\.length/);
});

test("every grip decision is referenced to the contact pads", () => {
  // The RG2 attach point sits ~43 mm behind the pads. Measuring from it
  // rejects a cube sitting dead centre between the fingers against a 40 mm
  // threshold, and offsets the handover rendezvous by the same amount on each
  // arm -- in opposite directions, so ~85 mm between them.
  const g = require("./lowerThingServer")
    .createContext("generated/robots/Robot1/manifest.json").manifest.runtime.gripper;
  const padOffset = (0.0279 + 0.0574) / 2;
  assert.ok(
    padOffset > g.gripThreshold,
    `a ${padOffset} m reference error alone exceeds the ${g.gripThreshold} m grip threshold`
  );

  for (const fn of ["closeGripper", "acceptTransferredGripObject", "womatPlanHandoverApproach"]) {
    const body = luaFunctionBody(lowerServerSource, fn);
    assert.match(
      body,
      /womatToolControlPointWorld\(\)|womatEnsurePickTipDummy\(\)/,
      `${fn} must measure from the contact pads, not the attach point`
    );
  }
  // closeGripper's range test in particular.
  assert.match(
    luaFunctionBody(lowerServerSource, "closeGripper"),
    /local distance=womatDistance\(graspPoint, objectPosition\)/
  );
  // The fallback is named rather than silent, so a gripper whose pads cannot be
  // identified is diagnosable instead of quietly 43 mm out.
  assert.match(lowerServerSource, /womatToolControlPointMode="contact_pad_midpoint"/);
  assert.match(lowerServerSource, /womatToolControlPointMode="attach_point_fallback"/);
});

test("the receiver can meet the giver head-on or at right angles", () => {
  const { receiverApproachAxis, handoverApproachMode } = require("./lowerThingServer");
  const round = (axis) => axis.map((value) => Number(value.toFixed(6)));
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

  // Giver presenting the cube along +Y, as Robot1 does.
  const giver = [0, 1, 0];

  // Opposed: nose to nose. This is what a 180-degree-mounted pair already did
  // from its own mounting direction, now stated explicitly.
  assert.deepEqual(round(receiverApproachAxis(giver, "opposed", [0, 0, -1])), [0, -1, 0]);

  // Perpendicular: exactly orthogonal, and it takes the preferred side.
  const perpendicular = receiverApproachAxis(giver, "perpendicular", [0, 0, -1]);
  assert.deepEqual(round(perpendicular), [0, 0, -1]);
  assert.ok(Math.abs(dot(perpendicular, giver)) < 1e-9, "must be orthogonal to the giver's tool");

  // Default preference is the line between the two bases, not a world axis.
  // For the shipped cell -- Robot1 at the origin, Robot2 at [1.25, 0.2, 0]
  // facing it -- that resolves to global -X once the giver's +Y tool is
  // projected out, which is the sideways approach across the cell.
  const { cellBaselineAxis } = require("./lowerThingServer");
  const baseline = cellBaselineAxis({ offset: [1.25, 0.2, 0] }, { offset: [0, 0, 0] });
  assert.deepEqual(round(receiverApproachAxis(giver, "perpendicular", baseline)), [-1, 0, 0]);
  // And it is a preference, not the answer: the +Y component is projected out
  // rather than tilting the approach.
  assert.ok(Math.abs(baseline[1]) > 0.1, "the raw baseline is not axis-aligned");
  assert.equal(cellBaselineAxis({}, { offset: [0, 0, 0] }), null);

  // The preference only picks WHICH perpendicular; orthogonality is exact even
  // when the preference is nowhere near orthogonal itself.
  const skewed = receiverApproachAxis(giver, "perpendicular", [0.3, 0.9, -0.3]);
  assert.ok(Math.abs(dot(skewed, giver)) < 1e-9, "orthogonality must not depend on the preference");
  assert.ok(Math.abs(Math.hypot(...skewed) - 1) < 1e-9, "axis must be a unit vector");

  // Degenerate case: the giver holds the cube vertically, so "from above" is
  // parallel to its tool and cannot be projected. Still returns a real axis.
  const vertical = receiverApproachAxis([0, 0, -1], "perpendicular", [0, 0, -1]);
  assert.ok(vertical, "a parallel preference must not yield a null axis");
  assert.ok(Math.abs(dot(vertical, [0, 0, -1])) < 1e-9, "must still be orthogonal");

  assert.equal(receiverApproachAxis([0, 0, 0], "opposed", [0, 0, -1]), null);

  // A pair entry beats the robot default, so one arm can receive head-on and
  // hand off at right angles.
  const ctx = {
    manifest: {
      name: "Robot2",
      runtime: { handoverApproachMode: "opposed" },
      sharedEnvironment: {
        interactionPairs: [
          { key: "Robot1__Robot2", robots: ["Robot1", "Robot2"] },
          { key: "Robot2__Robot3", robots: ["Robot2", "Robot3"], approachMode: "perpendicular" }
        ]
      }
    }
  };
  assert.equal(handoverApproachMode(ctx, "Robot1"), "opposed");
  assert.equal(handoverApproachMode({ ...ctx, manifest: { ...ctx.manifest, name: "Robot3" } }, "Robot2"), "perpendicular");
  // An unrecognised mode falls back rather than reaching the planner.
  assert.equal(
    handoverApproachMode({ manifest: { name: "R", runtime: { handoverApproachMode: "sideways" } } }, "G"),
    "opposed"
  );

  // The axis reaches the driver, and the driver prefers it over its own mounting.
  const axisFn = luaFunctionBody(lowerServerSource, "womatHandoverApproachAxisWorld");
  assert.match(axisFn, /if womatHandoverPoseAxis ~= nil then[\s\S]*?return \{a\[1\]\/given/);
  assert.match(
    luaFunctionBody(lowerServerSource, "womatSetHandoverPoseLock"),
    /womatHandoverPoseAxis=\{axis\[1\], axis\[2\], axis\[3\]\}/
  );
});

test("a named cell direction pins the receiver's tool regardless of the giver", () => {
  const {
    receiverApproachAxis, receiverApproachAxisCandidates, handoverApproachMode
  } = require("./lowerThingServer");
  const round = (axis) => axis.map((value) => Number(value.toFixed(6)));

  // Robot3 is mounted pitched -90 degrees, so its tool points horizontally
  // along +Y and a head-on exchange asks it to come in sideways. "vertical"
  // says: never mind the giver, bring the tool straight down onto the cube.
  assert.deepEqual(round(receiverApproachAxis([0, 1, 0], "vertical", [0, 0, -1])), [0, 0, -1]);

  // The giver is genuinely irrelevant -- that is the difference between this
  // and "opposed"/"perpendicular", both of which move when the giver moves.
  for (const giver of [[0, 1, 0], [1, 0, 0], [0, 0, -1], [-0.3, 0.9, 0.3]]) {
    assert.deepEqual(round(receiverApproachAxis(giver, "vertical", [0, 0, -1])), [0, 0, -1]);
  }
  // Including when there is no giver axis at all, where a relative mode has
  // nothing to work from.
  assert.deepEqual(round(receiverApproachAxis(null, "vertical", [0, 0, -1])), [0, 0, -1]);
  assert.equal(receiverApproachAxis(null, "opposed", [0, 0, -1]), null);

  // Down is unambiguous; the horizontal ones name a line, and the sense comes
  // from the preference so the tool points AT the cube rather than away from
  // it. Robot2 sits at +X of Robot1, so it reaches back along -X.
  const { cellBaselineAxis } = require("./lowerThingServer");
  const towardRobot1 = cellBaselineAxis({ offset: [1.5, 0.2, 0] }, { offset: [0, 0, 0] });
  assert.deepEqual(round(receiverApproachAxis([0, 1, 0], "horizontalX", towardRobot1)), [-1, 0, 0]);
  const towardRobot2 = cellBaselineAxis({ offset: [0, 0, 0] }, { offset: [1.5, 0.2, 0] });
  assert.deepEqual(round(receiverApproachAxis([0, 1, 0], "horizontalX", towardRobot2)), [1, 0, 0]);
  assert.deepEqual(round(receiverApproachAxis([0, 1, 0], "horizontalY", [0, -1, 0])), [0, -1, 0]);

  // The named direction is always tried first and exactly. The alternates
  // exist only so a rendezvous the arm cannot solve dead-on is not thrown away
  // over a few degrees -- a real failure on the shipped cell, which reported
  // no_collision_free_handover_standoff with the seed diverging.
  const cone = receiverApproachAxisCandidates([0, 1, 0], "vertical", [0, 0, -1], {
    count: 8, maxTurnDeg: 180, fixedMaxTurnDeg: 25
  });
  assert.deepEqual(round(cone[0]), [0, 0, -1], "the named direction is the first thing tried");
  assert.ok(cone.length > 1, "a named direction must offer alternates when it cannot be solved");
  for (const candidate of cone.slice(1)) {
    const tilt = Math.acos(-candidate[2]) * 180 / Math.PI;
    assert.ok(tilt > 1e-6 && tilt <= 25 + 1e-6, `alternate tilted ${tilt} degrees, bound is 25`);
    assert.ok(Math.abs(Math.hypot(...candidate) - 1) < 1e-9, "every alternate is a unit axis");
  }
  // Bound of zero, or a single candidate requested, means exactly the one.
  assert.equal(
    receiverApproachAxisCandidates([0, 1, 0], "vertical", [0, 0, -1], { fixedMaxTurnDeg: 0 }).length,
    1
  );

  // Spelled as a manifest would spell it, and still refusing a typo.
  const modeFor = (configured) => handoverApproachMode(
    { manifest: { name: "Robot3", runtime: { handoverApproachMode: configured } } },
    "Robot2"
  );
  assert.equal(modeFor("vertical"), "vertical");
  assert.equal(modeFor("Vertical"), "vertical");
  assert.equal(modeFor("horizontalX"), "horizontalX");
  assert.equal(modeFor("horizontalx"), "horizontalX");
  assert.equal(modeFor("vertically"), "vertical");
  assert.equal(modeFor("verticle"), "opposed");

  // A pair entry still wins over the robot default.
  assert.equal(handoverApproachMode({
    manifest: {
      name: "Robot3",
      runtime: { handoverApproachMode: "vertical" },
      sharedEnvironment: {
        interactionPairs: [{ key: "Robot2__Robot3", robots: ["Robot2", "Robot3"], approachMode: "opposed" }]
      }
    }
  }, "Robot2"), "opposed");
});

test("the giver presents along the cell direction its manifest names, not its mounting", () => {
  const { giverPresentAxisFor } = require("./lowerThingServer");
  const round = (axis) => axis.map((value) => Number(value.toFixed(6)));

  const cell = (name, offset) => ({
    ctx: { manifest: { name, sharedEnvironment: { offset, orientationDeg: [0, 0, 0] } } }
  });
  const coordinator = { robots: new Map([["Robot2", cell("Robot2", [1.5, 0.2, 0])]]) };
  const giver = (mode) => ({
    manifest: {
      name: "Robot1",
      runtime: { handoverPresentMode: mode },
      sharedEnvironment: { offset: [0, 0, 0], orientationDeg: [0, 0, 0] }
    }
  });

  // Held level and reaching across the cell: Robot2 sits at +Y of Robot1 as
  // well as +X, so a Y-parallel present points at it rather than away.
  assert.deepEqual(round(giverPresentAxisFor(giver("horizontalY"), coordinator, "Robot2")), [0, 1, 0]);
  assert.deepEqual(round(giverPresentAxisFor(giver("horizontalX"), coordinator, "Robot2")), [1, 0, 0]);
  assert.deepEqual(round(giverPresentAxisFor(giver("vertical"), coordinator, "Robot2")), [0, 0, -1]);

  // Unset keeps the old behaviour -- present along this robot's own mounting,
  // which is what the driver falls back to when handed no axis.
  assert.equal(giverPresentAxisFor(giver(undefined), coordinator, "Robot2"), null);
  // A typo must not silently become a direction either.
  assert.equal(giverPresentAxisFor(giver("horizontal"), coordinator, "Robot2"), null);
});

test("every mode the shipped cell configures is a mode that exists", () => {
  const fs = require("node:fs");
  const { handoverApproachMode, giverPresentAxisFor } = require("./lowerThingServer");

  // Deliberately not asserting WHICH mode each robot uses: that is a tuning
  // decision, and a test that pins it just breaks every time the cell is
  // retuned. What must hold is that a configured mode is a real one -- a typo
  // resolves to the "opposed" fallback, which looks like a working setting
  // right up until an arm comes in from the wrong side.
  for (const name of ["Robot1", "Robot2", "Robot3"]) {
    const manifest = JSON.parse(fs.readFileSync(`generated/robots/${name}/manifest.json`, "utf8"));
    const configured = manifest.runtime?.handoverApproachMode;
    if (configured !== undefined) {
      assert.equal(
        handoverApproachMode({ manifest }, "AnyGiver"),
        String(configured).toLowerCase() === "vertically" ? "vertical" : configured,
        `${name} declares handoverApproachMode ${JSON.stringify(configured)}, which did not resolve to itself`
      );
    }
    const presenting = manifest.runtime?.handoverPresentMode;
    if (presenting !== undefined) {
      assert.ok(
        giverPresentAxisFor({ manifest }, null, "AnyReceiver"),
        `${name} declares handoverPresentMode ${JSON.stringify(presenting)}, which names no direction`
      );
    }
  }
});

test("which perpendicular is a free parameter, so the planner may search it", () => {
  const { receiverApproachAxisCandidates } = require("./lowerThingServer");
  const giver = [0, 1, 0];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

  // Unbounded, the search spans the whole circle.
  const candidates = receiverApproachAxisCandidates(giver, "perpendicular", [-1, 0, 0], {
    count: 8, maxTurnDeg: 180
  });
  assert.equal(candidates.length, 8);

  // The point of the search: every alternative is EXACTLY as perpendicular as
  // the first. They are not degraded fallbacks, they are the same constraint
  // satisfied a different way.
  for (const axis of candidates) {
    assert.ok(Math.abs(dot(axis, giver)) < 1e-9, `${axis} is not perpendicular to the giver's tool`);
    assert.ok(Math.abs(Math.hypot(...axis) - 1) < 1e-9, `${axis} is not a unit vector`);
  }
  // The preference is honoured first, and alternatives are ordered by how far
  // they turn from it, so a workable direction stays close to what was asked.
  assert.deepEqual(candidates[0].map((v) => Number(v.toFixed(6))), [-1, 0, 0]);
  const turn = (axis) => Math.acos(Math.min(1, Math.max(-1, dot(axis, candidates[0]))));
  for (let i = 2; i < candidates.length; i += 2) {
    assert.ok(
      turn(candidates[i]) >= turn(candidates[i - 2]) - 1e-9,
      "alternatives must be ordered nearest-first"
    );
  }
  // The candidates span the circle rather than clustering on one side.
  assert.ok(candidates.some((a) => dot(a, candidates[0]) < -0.99), "must include the opposite side");

  // Bounded by default, because "sideways is blocked" must not be answered
  // with "then come up from underneath" -- still perpendicular, still collision
  // free, and not what was asked for.
  const bounded = receiverApproachAxisCandidates(giver, "perpendicular", [-1, 0, 0], {
    count: 8, maxTurnDeg: 60
  });
  assert.equal(bounded.length, 3, "60 degrees of turn allows the preferred direction and +/-45");
  for (const axis of bounded) {
    assert.ok(
      dot(axis, bounded[0]) > Math.cos((60 + 1e-6) * Math.PI / 180),
      `${axis} turns further from the requested direction than the bound allows`
    );
  }
  assert.ok(
    !bounded.some((a) => Math.abs(a[2]) > 0.9),
    "a 60 degree bound must exclude approaching from directly below or above"
  );

  // "Opposed" names exactly one direction, so there is nothing to search and
  // the planner must not pretend otherwise.
  assert.equal(receiverApproachAxisCandidates(giver, "opposed", [-1, 0, 0], { count: 8 }).length, 1);
  assert.deepEqual(receiverApproachAxisCandidates([0, 0, 0], "perpendicular", [-1, 0, 0]), []);

  // The candidates are tried one script call each, from the Node driver. A loop
  // inside the driver script would multiply one blocking call's worst case by
  // the number of candidates, which is what raises CoppeliaSim's abort dialog.
  assert.match(
    lowerServerSource,
    /for \(let index = 0; index < candidates\.length[\s\S]*?womatPlanHandoverApproach\(/
  );
  assert.match(lowerServerSource, /womatHandoverApproachAxisCandidates\(\)/);
  assert.match(lowerServerSource, /rejectedAxes: rejected/);
});

test("the roll about the pinned axis is searched, against a check that can see the giver", () => {
  // Two tool control points meeting at one cube puts both grippers around the
  // same 45 mm block. Which way each is ROLLED about its own tool axis is the
  // only thing left deciding whether their fingers interleave or drive through
  // each other, and the approach constraint leaves exactly that free.
  //
  // Live cell: Robot4's Baxter gripper reaching into Robot2's RG2 had a window
  // roughly ten degrees wide, at 265 and 275 degrees, out of a full turn.
  const { approachRollCandidates } = require("./handoverGeometry");
  const rolls = approachRollCandidates({ rollStepDeg: 5 });
  assert.equal(rolls.length, 72, "a five degree grid is one full turn");
  assert.equal(rolls[0], 0);
  assert.equal(rolls[rolls.length - 1], 355);
  // Coarse enough to miss that window is the same as not searching.
  assert.ok(approachRollCandidates({ rollStepDeg: 15 }).length === 24);

  const plan = luaFunctionBody(
    require("./core/luaDriver").driverTemplate(), "womatPlanHandoverApproach"
  );
  // Choosing a roll that misses the giver's gripper is only possible while the
  // giver's gripper is still an obstacle. The free-roll approach cannot avoid
  // it and so has it dropped; a searched roll takes it back for the plan, and
  // hands it back afterwards.
  assert.match(plan, /womatHandoverIgnoredGripper=-1/);
  assert.match(plan, /local function restoreIgnoredGripper\(\)/);
  assert.match(plan, /if rollRad ~= nil and womatHandoverIgnoredGripper ~= -1 then/);
  // Both legs are solved at the same roll, or the wrist turns on the way in and
  // the pose that was checked is not the pose that arrives.
  assert.match(plan, /womatSetApproachTarget\(waypoint, approachAxis, rollRad\)/);
  const standoff = luaFunctionBody(
    require("./core/luaDriver").driverTemplate(), "womatSolveHandoverStandoff"
  );
  assert.match(standoff, /womatSetApproachTarget\(standoffWorld, axis, rollRad\)/);
});

test("the roll is guessed from where the giver's jaws are, then searched around that", () => {
  const { driverTemplate } = require("./core/luaDriver");
  const template = driverTemplate();

  // A gripper's orientation has two halves. The tool axis says which way it
  // faces; the closing axis says which pair of the cube's faces it is holding.
  // Only the second decides whether two grippers meeting at one cube take
  // different faces or fight over the same ones, and nothing used to read it.
  const closing = luaFunctionBody(template, "womatClosingAxisOf");
  assert.match(closing, /womatIsDecorativeShape/, "a duplicate render mesh is not a contact pad");
  assert.match(closing, /if length > bestDistance then/, "measure across the jaws, not within one");
  // The giver's pad naming is its own -- RG2s call them "touch", the Baxter
  // gripper "pad" -- so the receiver is told rather than left to guess.
  assert.match(luaFunctionBody(template, "womatSetHandoverApproachGripper"), /padAlias/);
  const handoverSource = fs.readFileSync(path.join(__dirname, "core", "handover.js"), "utf8");
  assert.match(handoverSource, /giverPadAlias = giverManifest\?\.runtime\?\.gripper\?\.graspPadAlias/);

  const predicted = luaFunctionBody(template, "womatPredictedHandoverRoll");
  assert.match(predicted, /peerAngle \+ 90 - ownOffset/, "across the giver's jaws, not along them");

  // ONE return value. These go straight to the Node search as a list of angles,
  // so a second return reads as one more angle to try -- and did.
  const candidates = luaFunctionBody(template, "womatHandoverApproachRollCandidates");
  assert.doesNotMatch(candidates, /return rolls\s*,/);
  assert.match(candidates, /return rolls\s*$/m);
  // Jaws are a line, not an arrow, so a roll and its opposite place the fingers
  // the same way and must rank the same.
  assert.match(candidates, /% 180/);
});

test("the giver can leave a rendezvous it never handed over in", () => {
  // A rendezvous ENDS with the two grippers interleaved around one cube, so
  // from the receiver's arrival the giver is in contact with it and can plan
  // nothing, including its own way out. Arming the ignore only on a successful
  // transfer meant the one case where the giver most needs to leave -- a
  // handover that did not complete -- was the one case where it could not.
  const handoverSource = fs.readFileSync(path.join(__dirname, "core", "handover.js"), "utf8");
  const begin = handoverSource.slice(
    handoverSource.indexOf("async function beginHandoverSession"),
    handoverSource.indexOf("async function transferCubeOwnership")
  );
  assert.match(begin, /womatSetHandoverApproachGripper\(/,
    "the giver arms the peer-gripper ignore when the session opens");
  assert.match(begin, /receiverGripper/);
  // And it is still cleared in exactly one place, so nothing leaks past the end
  // of the session.
  assert.match(handoverSource, /womatClearHandoverApproachGripper/);
});

test("a searched roll is bounded, and a robot that does not search one is unchanged", () => {
  const { driverTemplate } = require("./core/luaDriver");
  const standoff = luaFunctionBody(driverTemplate(), "womatSolveHandoverStandoff");
  // The randomised standoff search is the expensive half, and it is the wrong
  // fallback here: the driver is already walking a ring of rolls about this
  // same axis, so paying its full time budget on every roll that fails turns a
  // couple of dozen cheap attempts into a minute of them.
  assert.match(standoff, /if rollRad ~= nil then[\s\S]*?no_collision_free_handover_standoff[\s\S]*?simIK\.findConfigs/);

  // Nothing changes for a robot that asks for no rolls: no roll list reaches
  // the driver, so no roll reaches the planner, so the constraint, the obstacle
  // set and the search are all the ones every pair used before.
  const { setHandoverApproach } = require("./core/handover");
  assert.equal(typeof setHandoverApproach, "function");
  const roll = luaFunctionBody(driverTemplate(), "womatHandoverApproachRollCandidates");
  assert.match(roll, /if womatHandoverPoseRolls == nil then\s*\n\s*return \{\}/);
  const lock = luaFunctionBody(driverTemplate(), "womatSetHandoverPoseLock");
  assert.match(lock, /if womatHandoverPoseEnabled and rolls ~= nil and #rolls > 0 then/);
});

test("an unlocked giver reports where its tool is pointing, not how it is bolted down", () => {
  // A position-only move leaves the wrist free, so the mounting direction and
  // the direction the cube is actually held out along are unrelated -- and the
  // receiver's opposed and perpendicular modes are defined against the second.
  // Live cell: Robot2 stood at the rendezvous with its tool straight up while
  // its mounting axis was horizontal and aimed AWAY from Robot4, so Robot4's
  // opposed approach put its standoff on the far side of the cube and no
  // collision-free standoff existed at all.
  const handoverSource = fs.readFileSync(
    path.join(__dirname, "core", "handover.js"), "utf8"
  );
  assert.match(
    handoverSource,
    /womatToolDirectionWorld\(\)"\)\s*\n\s*\?\?\s*await executeInScript\([\s\S]*?womatHandoverApproachAxisWorld/,
    "the live tool direction must be preferred, with the mounting as last resort"
  );
  const direction = luaFunctionBody(
    require("./core/luaDriver").driverTemplate(), "womatToolDirectionWorld"
  );
  // Read off the frame the IK steers, whose Z runs down the tool.
  assert.match(direction, /womatIkTip\(tip\), sim\.handle_world/);
  assert.match(direction, /\{matrix\[3\], matrix\[7\], matrix\[11\]\}/);
});

test("the pair a rendezvous belongs to may name how it is approached, and keep it", () => {
  // Robot4 receives from Robot2 relative to its live tool frame: which way one arm has to come at another depends on both their
  // mountings and both their grippers, so it is a property of the PAIR and not
  // of either robot's default.
  const { handoverApproachMode } = require("./core/handover");
  const load = (name) => JSON.parse(fs.readFileSync(
    path.join(__dirname, "..", "..", "generated", "robots", name, "manifest.json"), "utf8"
  ));
  const robot4 = { manifest: load("Robot4") };
  assert.equal(handoverApproachMode(robot4, "Robot2"), "opposed");
  robot4.manifest.runtime.handoverApproachMode = "horizontalX";
  assert.equal(handoverApproachMode(robot4, "Robot2"), "opposed", "pair overrides robot default");

  // And it survives a rebuild of the cell. Nothing in the generator can derive
  // it -- it is found by trying it in the scene -- so regenerating used to drop
  // it and quietly put the pair back on the robot default.
  assert.match(
    bridgeSource,
    /interactionPairs: pairs\.map\(\(\{ key, robots, ownerRobot \}\) => \{[\s\S]*?approachMode = previous\?\.approachMode/
  );
});

test("no single planning call can run long enough to raise the abort dialog", () => {
  // The driver script is synchronous inside CoppeliaSim: whatever one
  // executeScriptString does, the simulator is blocked for. Past a few seconds
  // it offers to abort the script, which reads as the robot counting forever.
  const plan = luaFunctionBody(lowerServerSource, "womatPlanHandoverApproach");
  const standoff = luaFunctionBody(lowerServerSource, "womatSolveHandoverStandoff");

  // Exactly one randomised search per call, and its budget is the caller's,
  // not the caller's multiplied by anything.
  assert.equal((plan.match(/simIK\.findConfig/g) || []).length, 0);
  assert.equal((standoff.match(/pcall\(simIK\.findConfigs/g) || []).length, 1);
  assert.match(standoff, /maxTime=budget/);
  assert.match(plan, /graspWorld, approachAxis, standoffDistance or 0, saved, maxTime or 2\.5/);

  // The expensive obstacle-naming walk builds a collection per scene shape, so
  // it must stay on the failure path only.
  assert.match(plan, /if failure == "collision" then\s*\n\s*local obstacle, owner=womatBlockingObstacleAlias\(\)/);

  // And the per-direction budget is well under the pick's, because the search
  // now costs one call per direction instead of one call for all of them.
  const gripper = require("./lowerThingServer")
    .createContext("generated/robots/Robot1/manifest.json").manifest.runtime.gripper;
  const options = require("./lowerThingServer")
    .createRuntimeOptions(require("./lowerThingServer")
      .createContext("generated/robots/Robot1/manifest.json"));
  assert.ok(
    options.handoverPlanningMaxTime <= options.pickPlanningMaxTime,
    "a handover direction may not be given a longer blocking budget than the whole pick"
  );
  assert.ok(options.handoverPlanningMaxTime > 0);
  assert.ok(gripper);
});

test("a robot can opt out of handover planning and just move", () => {
  const { createContext } = require("./lowerThingServer");

  // Both arming sites are gated, so the switch covers the robot in either role:
  // giver (armed at session registration) and receiver (armed just before its
  // approach). Absent flag means planning stays on. Each site names its role,
  // so a robot in the middle of a chain can be pinned as one and free as the
  // other -- see handoverPoseLockRole.test.js for the resolution itself.
  assert.match(
    lowerServerSource,
    /return runtime\.enableHandoverPoseLock !== false;/
  );
  assert.match(lowerServerSource, /const poseLocked = handoverPoseLockEnabled\(ctx, "giver"\);/);
  assert.match(lowerServerSource, /if \(!handoverPoseLockEnabled\(ctx, "receiver"\)\) \{[\s\S]*?approachMode: "simple_cartesian"/);

  // Opting out must not touch the driver flag the move routes on, or the move
  // would still be diverted into the planner.
  assert.match(
    lowerServerSource,
    /womatHandoverPoseEnabled"\);\s*if \(handoverLocked === true\) \{\s*\n\s*return handoverApproach\(/
  );

  // Which arms run free is a tuning decision and deliberately not asserted --
  // an arm that cannot converge on the constrained pose gets opted out, and
  // pinning the roster here just breaks the suite when one is. What must hold
  // is that opting out gates PLANNING and nothing else: basic safety stays on
  // for every robot, opted out or not.
  for (const name of ["Robot1", "Robot2", "Robot3"]) {
    const runtime = createContext(`generated/robots/${name}/manifest.json`).manifest.runtime;
    for (const guard of [
      "enableCollisionCheck",
      "enableJointCollisionPreflight",
      "enableInMotionCollisionCheck",
      "enableWorkspaceAdmission"
    ]) {
      assert.equal(runtime[guard], true, `${name}: ${guard} must stay on for an unplanned approach`);
    }
  }

  // And an arm that opts out must not also be carrying an approach modifier,
  // which cannot fire once the wrist is free and only reads as if it does.
  for (const name of ["Robot1", "Robot2", "Robot3"]) {
    const runtime = createContext(`generated/robots/${name}/manifest.json`).manifest.runtime;
    if (runtime.enableHandoverPoseLock === false) {
      for (const modifier of ["handoverApproachMode", "handoverPresentMode"]) {
        assert.equal(
          runtime[modifier], undefined,
          `${name} runs free, so ${modifier} can never take effect`
        );
      }
    }
  }
});


test("the tool control point is a visible object, and the one the solver uses", () => {
  const marker = luaFunctionBody(lowerServerSource, "womatEnsureToolControlPoint");
  // Sized and coloured to be found in the scene, not a 1 cm invisible helper.
  assert.match(marker, /sim\.createDummy\(\$\{tcpMarkerSize\}\)/);
  assert.match(marker, /sim\.setObjectColor/);
  assert.match(marker, /sim\.setObjectAlias\(handle, \$\{tcpAliasLua\}\)/);
  // Built from the measured contact-pad midpoint, so what is on screen is what
  // every offset in the driver is measured to.
  assert.match(marker, /womatToolControlPointInFlange\(\)/);
  // Created with the gripper rather than lazily on the first pick, so it is
  // visible before anything moves.
  assert.match(
    luaFunctionBody(lowerServerSource, "womatEnsureTcpGripperModel"),
    /womatEnsureToolControlPoint\(\)/
  );
  // And it is the same object the pick IK is driven through.
  assert.match(
    luaFunctionBody(lowerServerSource, "womatPlanTopDownPick"),
    /local pickTip=womatEnsureToolControlPoint\(\)/
  );
});

test("the runtime adds a tool control point, it never moves the flange", () => {
  // A child follows its parent. The gripper is posed against the flange, so
  // anything that MOVES the flange moves the gripper too -- and the workspace
  // scene is saved afterwards, which makes that permanent.
  const tcp = luaFunctionBody(lowerServerSource, "womatEnsureToolControlPoint");
  assert.match(tcp, /sim\.setObjectParent\(handle, womatFlangeTip, true\)/);
  assert.doesNotMatch(tcp, /sim\.setObjectPosition\(womatFlangeTip/);
  assert.doesNotMatch(tcp, /sim\.setObjectMatrix\(womatFlangeTip/);

  // Script globals reset on re-init while scene objects persist. Measuring a
  // fresh pad offset from a tip already sitting on the pads is the same
  // doubling by another route, so an existing marker is adopted instead.
  // Looked up through the robot's own tree: the marker hangs off the flange, and
  // '/alias' is a root-level lookup that would miss it and build a second one.
  assert.match(tcp, /local existingMarker=womatObjectByAliasInRobot/);
  assert.ok(
    tcp.indexOf("existingMarker") < tcp.indexOf("womatToolControlPointInFlange()"),
    "an existing marker must be adopted before any offset is measured"
  );

  // A cloud sampled from the flange looks exactly like a good one and is wrong
  // by a whole tool length, so the failure stops generation.
  const bridgeSrc = fs.readFileSync(require.resolve("./robwotBridge"), "utf8");
  assert.match(bridgeSrc, /error\('workspace_gripper_contact_pads_not_found/);
});

test("the flange stays the flange, in the model and in every saved scene", () => {
  const bridgeSrc = fs.readFileSync(require.resolve("./robwotBridge"), "utf8");

  // Workspace generation saves the scene afterwards, so anything it does to
  // './tip' is permanent. Moving it redefines the gripper's mount point for
  // every later run: the runtime then bolts the gripper one tool length out and
  // the jaws land at two. Sample a separate dummy instead.
  assert.doesNotMatch(bridgeSrc, /sim\.setObjectPosition\(flange, sim\.handle_world/);
  assert.match(bridgeSrc, /simTip=womatWorkspaceToolControlPoint\(\)/);
  assert.match(bridgeSrc, /local tip=womatWorkspaceToolControlPoint\(\)/);
  // The gripper is bolted to the flange, exactly as the runtime bolts it, and
  // not through a pcall: a failed mount must not pass for success.
  assert.match(bridgeSrc, /"    sim\.setObjectParent\(gripper, flange, false\)"/);
  assert.doesNotMatch(bridgeSrc, /pcall\(sim\.setObjectParent, gripper/);
  // And the sampled dummy carries the same alias the runtime looks for, so the
  // two agree on which object is the tool control point.
  assert.match(bridgeSrc, /const tcpAlias = `\$\{ctx\.manifest\.name\}_tcp`/);
  // Generation resolves the flange the same way the runtime does, by searching
  // the tree: the path form is a direct-child lookup and this dummy hangs off
  // the last link.
  assert.match(bridgeSrc, /womatWorkspacePlainAlias\(candidates\[i\]\) == 'tip'/);

  // Self-heal: a scene whose tip already sits on the pads must be adopted, not
  // offset a second time.
  const tcp = luaFunctionBody(lowerServerSource, "womatEnsureToolControlPoint");
  assert.match(tcp, /if length < 0\.002 then/);
  assert.match(tcp, /womatPickTipDummy=simTip/);
  assert.match(tcp, /tip_already_at_pads/);
});

test("the flange is resolved by name, never from the tip variable", () => {
  // simTip is mutable -- it BECOMES the tool control point. Deriving the flange
  // from it is circular, and any code path that reads it after the rebinding
  // mounts the gripper one tool length out, which puts the jaws at two. This is
  // order-dependent, so it survived until a scene happened to hit the bad order.
  const resolve = luaFunctionBody(lowerServerSource, "womatResolveFlange");
  assert.match(resolve, /sim\.getObject\('\.\/tip', \{noError=true\}\)/);
  assert.doesNotMatch(lowerServerSource, /womatFlangeTip=simTip/);

  const attach = luaFunctionBody(lowerServerSource, "womatEnsureTcpGripperModel");
  assert.match(attach, /local flange=womatResolveFlange\(\)/);
  // Verified on every call: a stale scene can arrive already wired wrongly, and
  // the runtime is the thing that has to repair it.
  assert.match(attach, /local mountWrong=.*sim\.getObjectParent\(tcpGripper\) ~= flange/);
  assert.match(attach, /if not womatGripperConfiguredOnce or mountWrong then/);
  // A marker measured through a wrong mount is wrong too, so it is rebuilt.
  assert.match(attach, /womatPickTipDummy=-1/);
  assert.match(attach, /womatToolControlPoint=nil/);

  // And a marker that hangs off anything but the flange describes a tool frame
  // this robot no longer has: rebuild rather than adopt it.
  const tcp = luaFunctionBody(lowerServerSource, "womatEnsureToolControlPoint");
  assert.match(tcp, /sim\.getObjectParent\(existingMarker\) == flange/);
  assert.match(tcp, /pcall\(sim\.removeObject, existingMarker\)/);
});

test("an unresolvable flange fails loudly instead of leaving the gripper adrift", () => {
  const resolve = luaFunctionBody(lowerServerSource, "womatResolveFlange");
  // './tip' is a direct-child lookup and the dummy hangs off the last LINK, so
  // the path form can return -1. Search the tree instead.
  assert.match(resolve, /sim\.getObjectsInTree\(simBase, sim\.object_dummy_type, 0\)/);
  assert.match(resolve, /womatPlainAlias\(candidates\[i\]\) == "tip"/);

  const attach = luaFunctionBody(lowerServerSource, "womatEnsureTcpGripperModel");
  // Refused rather than attempted: mounting against an unresolved flange leaves
  // the gripper wherever it was, which reads as success everywhere downstream.
  assert.match(attach, /reason="flange_dummy_not_found"/);
  assert.ok(
    attach.indexOf("flange_dummy_not_found") < attach.indexOf("setObjectParent(tcpGripper"),
    "the flange must be resolved before anything is mounted to it"
  );
  // The three calls that decide where the gripper ends up are not swallowed by
  // pcall; a failure there must not pass for success.
  for (const call of ["setObjectParent", "setObjectPosition", "setObjectOrientation"]) {
    assert.doesNotMatch(
      attach,
      new RegExp(`pcall\\(sim\\.${call}, tcpGripper, womatFlangeTip`),
      `${call} on the gripper mount must not be silently swallowed`
    );
  }
});


test("every driver placeholder is supplied, and the alias reaches the scene named", () => {
  // A mistyped substitution used to render the string "undefined" into the
  // middle of the scene script and surface much later as an unrelated failure.
  const ctx = createContext("generated/robots/Robot1/manifest.json");
  const script = prepareRuntimeDriverScript(ctx, {});
  assert.doesNotMatch(script, /\$\{/u, "no placeholder survives into the scene script");
  // Word-bounded: the driver legitimately contains reasons like
  // "handover_approach_axis_undefined". What must never appear is a bare
  // `undefined` where a substituted value belongs.
  assert.doesNotMatch(script, /\bundefined\b/, "no substitution renders as undefined");
  assert.match(script, /sim\.setObjectAlias\(handle, \[\[Robot1_tcp\]\]\)/);
});

test("the renderer refuses a driver it cannot fully substitute", () => {
  assert.throws(
    () => renderDriver({ robotNameLua: "[[Robot1]]" }),
    /missing substitutions/
  );
  const complete = Object.fromEntries([...placeholderNames()].map((name) => [name, "0"]));
  assert.throws(
    () => renderDriver({ ...complete, somethingNobodyUses: "0" }),
    /nothing uses/
  );
  assert.equal(typeof renderDriver(complete), "string");
});

test("the backstop sees the scene, not a list somebody remembered to write", () => {
  const source = lowerServerSource;
  // A neighbour discovered at runtime, or an object dropped into the scene by
  // hand, is an obstacle whether or not it was configured. Enumerating the
  // scene and excluding what is provably mine cannot be invalidated by a
  // discovery bug; walking a configured list can.
  assert.match(source, /womatObstacleDiscovery == "scene"/);
  assert.match(source, /sim\.getObjectsInTree\(sim\.handle_scene, sim\.object_shape_type, 0\)/);
  assert.match(source, /obstacleDiscovery === "configured" \? "configured" : "scene"/);
});

test("the surface a robot stands on is exempt from clearance, never from contact", () => {
  // A floor-mounted arm's base is permanently inside any sensible clearance
  // margin of its floor, so asking "is anything within 2cm" against the full
  // obstacle set refuses every pose the robot can adopt. Contact still answers
  // correctly, and must keep answering.
  const clearance = luaFunctionBody(lowerServerSource, "womatClearanceUnsafe");
  assert.match(clearance, /womatSweepClearanceCollection\(\)/);
  assert.doesNotMatch(clearance, /checkDistance, robotCollection, obstacleCollection/);
  // One call deeper since ordered sweeps can exempt a followed arm: the
  // collection is still the floor-exempt one, with or without that exemption.
  const sweepClearance = luaFunctionBody(lowerServerSource, "womatSweepClearanceCollection");
  assert.match(sweepClearance, /return womatClearanceObstacleCollection\(\)/);
  assert.match(sweepClearance, /not womatIsClearanceExempt\(shape\)/);

  const both = luaFunctionBody(lowerServerSource, "womatCollectionsUnsafe");
  assert.match(both, /womatCollectionsTouch\(robotCollection, obstacleCollection\)/);
  assert.match(both, /checkDistance, robotCollection, clearanceCollection/);

  // Detected at rest, not configured: inside the margin but not touching.
  const detect = luaFunctionBody(lowerServerSource, "womatDetectClearanceExempt");
  assert.match(detect, /not touching and ok and near ~= nil and near > 0/);
});

test("scene objects are typed by what they are, not by what a manifest calls them", () => {
  const inventory = luaFunctionBody(lowerServerSource, "womatSceneObjectInventory");
  // An articulated body is a tree with joints in it. That is a property of the
  // object, so a robot nobody configured is still recognised as a robot.
  assert.match(inventory, /sim\.object_joint_type[\s\S]*?kind="peer_robot"/);
  assert.match(inventory, /kind="payload"/);
  assert.match(inventory, /kind="support"/);
  assert.match(inventory, /kind="self"/);
});

test("Robot2's receiving approach is pinned to a named axis, and only as receiver", () => {
  // The pose lock was off entirely on this arm, so there was no axis to set --
  // the wrist ran free and the approach mode would have been inert.
  //
  // WHICH axis is deliberately not asserted: that is a tuning decision, and
  // pinning it here breaks the suite the moment the cell is retuned. What must
  // hold is that a named axis is in force, that it resolves to a single
  // direction rather than a circle of them, and that it is signed toward the arm
  // handing over -- a tool that points away from the cube is not an approach.
  const { handoverApproachMode, handoverPoseLockEnabled } = require("./core/handover");
  const { cellBaselineAxis, receiverApproachAxisCandidates } = require("./handoverGeometry");
  const load = (name) => JSON.parse(fs.readFileSync(
    path.join(__dirname, "..", "..", "generated", "robots", name, "manifest.json"), "utf8"
  ));
  const robot2 = load("Robot2");
  const ctx = { manifest: robot2 };

  assert.equal(handoverPoseLockEnabled(ctx, "receiver"), true);
  assert.equal(handoverPoseLockEnabled(ctx, "giver"), false, "the giver side was not asked for");

  for (const partner of ["Robot1", "Robot3"]) {
    const mode = handoverApproachMode(ctx, partner);
    assert.match(mode, /^horizontal[XY]$|^vertical$/, `receiving from ${partner} is not pinned to a named axis: ${mode}`);
    const preferred = cellBaselineAxis(robot2.sharedEnvironment, load(partner).sharedEnvironment) || [0, 0, -1];
    const [primary] = receiverApproachAxisCandidates([0, 1, 0], mode, preferred, {
      count: robot2.runtime.handoverApproachCandidates ?? 8,
      maxTurnDeg: robot2.runtime.handoverApproachMaxTurnDeg ?? 60,
      fixedMaxTurnDeg: robot2.runtime.handoverFixedApproachMaxTurnDeg ?? 25
    });
    // One axis, not a circle: exactly one component carries the direction.
    const carrying = primary.filter((value) => Math.abs(value) > 1e-9);
    assert.equal(carrying.length, 1, `not a single named direction: ${primary}`);
    assert.ok(Math.abs(Math.abs(carrying[0]) - 1) < 1e-9);
    // A horizontal axis names a line, not a sense, so the sense has to come from
    // the baseline toward the partner.
    const axis = primary.findIndex((value) => Math.abs(value) > 1e-9);
    if (axis !== 2) {
      assert.ok(primary[axis] * preferred[axis] > 0, `approach points away from ${partner}`);
    }
  }
});

test("an opted-in handover replicates the same selected world zone to both robot cores", async () => {
  const { peers, sessionsFor } = cell();
  const ctx = context("Robot1");
  ctx.manifest.runtime = { handoverZone: { enabled: true, radius: 0.12 } };
  peers.manifestOf("Robot2").runtime = { handoverZone: { enabled: true, radius: 0.16 } };
  const { sim } = scripting();
  const center = { x: 1.8, y: -0.3, z: 0.28 };
  const result = await beginHandoverSession(ctx, sim, 1, peers, sessionsFor("Robot1"), "zone-session", "Robot2", center);
  assert.equal(result.success, true);
  assert.deepEqual(sessionsFor("Robot1").get("zone-session").handoverZone,
    { pairKey: "Robot1__Robot2", center, radius: 0.16 });
  assert.deepEqual(sessionsFor("Robot2").get("zone-session").handoverZone,
    sessionsFor("Robot1").get("zone-session").handoverZone);
});
