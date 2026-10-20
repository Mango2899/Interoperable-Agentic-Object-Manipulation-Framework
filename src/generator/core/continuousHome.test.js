"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const { continuousHome, homeTrajectory } = require("./continuousHome");
const { RegionArbiter } = require("./motionArbiter");
const { SharedRegion } = require("./claims");
const { createRuntimeOptions } = require("./runtimeOptions");
const region = new SharedRegion({ pairKey: "A__B", robots: ["A", "B"],
  voxelSize: 0.1, origin: { x: 0, y: 0, z: 0 }, voxelKeys: ["5,5,5"] });
const options = createRuntimeOptions({ manifest: { runtime: {} } });
// The flow used when the arbiter cannot queue a claim: the path reserved whole.
const reservedWhole = { ...options, orderedReservations: false };
const log = { begin: () => ({ granted() {}, refused() {}, finished() {} }) };
const inside = [0, 0, 0, 0.55, 0.55, 0.55];
const outside = [0, 0, 0, 0, 0, 0.1];
const rad = Math.PI / 180;

function simulator({ blocked = false, lag = false, reenter = false,
  selfCollision = false, blockedSweeps = 0 } = {}) {
  const calls = [];
  let path;
  let polls = 0;
  let remaining = 0;
  let sweeps = 0;
  return { calls, executeScriptString: async (code) => {
    calls.push(code);
    if (code.includes("sim.getSimulationTimeStep")) return 0.05;
    if (code.includes("womatCurrentConfig")) return [0, [20 * rad]];
    if (code.includes("womatSweepFramePositions")) {
      const values = code.match(/\{([^}]+)\}/)[1].split(",").map(Number);
      return { ok: true, frames: values.map((value) => value > 10 * rad || (reenter && value < rad) ? inside : outside) };
    }
    if (code.includes("womatSceneRootHandle")) return 7;
    if (code.includes("womatBeginPreflight")) {
      const samples = code.match(/\{[^{}]+\}/g) || [];
      assert.ok(samples.length > 0, "preflight must receive the complete sample list");
      assert.doesNotMatch(code, /NaN|undefined|Infinity/);
      remaining = samples.length;
      sweeps++;
      // A posture that collides with itself is the same verdict whoever else
      // is in the cell; a peer arm standing in the way is not, and clears once
      // the rooms this path reserved are actually held.
      if (selfCollision) return { ok: false, mode: "self_collision", reason: "joint_path_collision_detected" };
      if (sweeps <= blockedSweeps) {
        return { ok: false, mode: "obstacle_contact", reason: "joint_path_collision_detected", obstacleOwner: "B" };
      }
    }
    if (code.includes("womatBeginPreflight") || code.includes("womatContinuePreflight")) {
      remaining = Math.max(0, remaining - 4);
      return blocked ? { ok: false, reason: "wall" } : { ok: true, pending: remaining > 0 };
    }
    if (code.includes("womatPreflightJointPath")) {
      return blocked ? { ok: false, reason: "wall" } : { ok: true, checked: 1 };
    }
    if (code.includes("womatFollowConfigs")) {
      path = code.match(/\{([^}]+)\}/)[1].split(",").map(Number);
      return "started";
    }
    if (code.includes("womatHomePathStatus")) {
      polls++;
      const index = Math.ceil(path.length * 0.8);
      return polls < 3 ? { state: 4, index, config: [path[index - 1] + 0.88 * rad],
        frames: lag && polls === 1 ? inside : outside }
        : { state: 3, index: path.length, config: [0], frames: outside };
    }
    throw new Error(`Unexpected call: ${code}`);
  } };
}

test("one continuous home tolerates intermediate dynamic lag and frees the room before completion", async () => {
  const arbiter = new RegionArbiter();
  const sim = simulator();
  const narrow = arbiter.narrow.bind(arbiter);
  let receiverMoved = false;
  arbiter.narrow = async (robot, voxels) => {
    const result = await narrow(robot, voxels);
    if (!receiverMoved) {
      receiverMoved = (await arbiter.acquire("B", region.allKeys())).granted;
      assert.deepEqual(arbiter.activeRobots, ["A", "B"]);
    }
    return result;
  };
  const result = await continuousHome(sim, 1, "A", [0],
    { ...options, homeGeometryChunk: 16 }, arbiter, () => [region], log);
  assert.equal(result.success, true);
  assert.equal(result.planner, "continuous_joint_home");
  assert.equal(receiverMoved, true);
  const begins = sim.calls.filter((code) => code.includes("womatBeginPreflight"));
  assert.equal(begins.length, 1, "the full path is submitted once");
  assert.match(begins[0], /, true, 0, 0\.05\)$/);
  const continuations = sim.calls.filter((code) => code.includes("womatContinuePreflight"));
  assert.ok(continuations.length > 0, "pending preflight is completed before playback");
  const playback = sim.calls.findIndex((code) => code.includes("womatFollowConfigs"));
  assert.ok(playback > sim.calls.lastIndexOf(continuations.at(-1)));
  assert.equal(sim.calls.filter((code) => code.includes("womatFollowConfigs")).length, 1);
  const checked = (begins[0].match(/\{[^{}]+\}/g) || []).map((row) => Number(row.slice(1, -1)));
  assert.equal(checked[0], 20 * rad);
  assert.equal(checked.at(-1), 0);
  assert.ok(checked.slice(1).every((value, i) => value <= checked[i] && checked[i] - value <= 0.5 * rad + 1e-12),
    "the complete dense sweep is retained across batches");
  assert.deepEqual(arbiter.activeRobots, ["B"]);
});

test("runtime preflight options have valid defaults and preserve valid overrides", () => {
  assert.equal(options.collisionCheckStepDeg, 0.5);
  assert.equal(options.preflightBudgetMs, 50);
  const configured = createRuntimeOptions({ manifest: { runtime: {
    collisionCheckStepDeg: "0.25", preflightBudgetMs: "20"
  } } });
  assert.equal(configured.collisionCheckStepDeg, 0.25);
  assert.equal(configured.preflightBudgetMs, 20);
  for (const value of [0, -1, NaN, Infinity, "invalid", null]) {
    const invalid = createRuntimeOptions({ manifest: { runtime: {
      collisionCheckStepDeg: value, preflightBudgetMs: value
    } } });
    assert.equal(invalid.collisionCheckStepDeg, 0.5);
    assert.equal(invalid.preflightBudgetMs, 50);
  }
});

test("invalid preflight options cannot start or reserve an unchecked motion", async () => {
  for (const key of ["collisionCheckStepDeg", "preflightBudgetMs"]) {
    const sim = simulator();
    const arbiter = new RegionArbiter();
    const result = await continuousHome(sim, 1, "A", [0], { ...options, [key]: undefined }, arbiter, () => [region], log);
    assert.equal(result.reason, "invalid_preflight_options");
    assert.deepEqual(sim.calls, []);
    assert.deepEqual(arbiter.activeRobots, []);
  }
});

test("command progress alone does not release a room while the actual arm still occupies it", async () => {
  const arbiter = new RegionArbiter();
  const sim = simulator({ lag: true });
  const narrow = arbiter.narrow.bind(arbiter);
  const attempts = [];
  arbiter.narrow = async (robot, voxels) => {
    const result = await narrow(robot, voxels);
    attempts.push((await arbiter.acquire("B", region.allKeys())).granted);
    return result;
  };
  assert.equal((await continuousHome(sim, 1, "A", [0], options, arbiter, () => [region], log)).success, true);
  assert.deepEqual(attempts, [false, true]);
});

test("a room needed again later stays reserved even after the arm temporarily clears it", async () => {
  const arbiter = new RegionArbiter();
  const sim = simulator({ reenter: true });
  const narrow = arbiter.narrow.bind(arbiter);
  arbiter.narrow = async (robot, voxels) => {
    const result = await narrow(robot, voxels);
    assert.equal((await arbiter.acquire("B", region.allKeys())).granted, false);
    return result;
  };
  assert.equal((await continuousHome(sim, 1, "A", [0], options, arbiter, () => [region], log)).success, true);
});

test("a blocked home path never starts and releases its claim for recovery", async () => {
  const arbiter = new RegionArbiter();
  const sim = simulator({ blocked: true });
  const result = await continuousHome(sim, 1, "A", [0], reservedWhole, arbiter, () => [region], log);
  assert.equal(result.success, false);
  assert.equal(result.safeToReplan, true);
  assert.equal(sim.calls.some((code) => code.includes("womatFollowConfigs")), false);
  assert.deepEqual(arbiter.activeRobots, []);
  assert.equal(sim.calls.filter((code) => code.includes("womatBeginPreflight")).length, 2,
    "a rejection the scene could lift is re-swept once the rooms are held");
});

test("the path is swept before its rooms are claimed, and only the goal pose is re-tested under them", async () => {
  const arbiter = new RegionArbiter();
  const sim = simulator();
  const preflights = () => sim.calls.filter((code) => /womat(Begin|Continue)Preflight/.test(code)).length;
  const acquire = arbiter.acquire.bind(arbiter);
  let sweptBeforeClaim = null;
  arbiter.acquire = async (robot, voxels) => {
    if (sweptBeforeClaim === null) sweptBeforeClaim = preflights();
    return acquire(robot, voxels);
  };
  assert.equal((await continuousHome(sim, 1, "A", [0], reservedWhole, arbiter, () => [region], log)).success, true);
  assert.ok(sweptBeforeClaim > 1, "the whole dense sweep runs off the critical path");
  assert.equal(preflights(), sweptBeforeClaim, "no room is held across the dense sweep");
  const rechecks = sim.calls.filter((code) => code.includes("womatPreflightJointPath"));
  assert.equal(rechecks.length, 1);
  // The goal alone, carrying the sample index the full sweep would have given
  // it so the clearance stride keeps counting from the same place.
  assert.match(rechecks[0], /^return womatPreflightJointPath\(\{\{0\}\}, true, \d+\)$/);
  assert.ok(sim.calls.indexOf(rechecks[0]) < sim.calls.findIndex((code) => code.includes("womatFollowConfigs")));
});

test("a rejection a peer could have caused is re-swept under the claim rather than replanned", async () => {
  const arbiter = new RegionArbiter();
  const sim = simulator({ blockedSweeps: 1 });
  const events = [];
  const result = await continuousHome(sim, 1, "A", [0], reservedWhole, arbiter, () => [region],
    { ...log, record: (event, details) => events.push({ event, details }) });
  assert.equal(result.success, true);
  assert.equal(sim.calls.filter((code) => code.includes("womatBeginPreflight")).length, 2);
  assert.equal(sim.calls.some((code) => code.includes("womatPreflightJointPath")), false,
    "a path re-swept in full needs no separate goal recheck");
  assert.equal(events.find((e) => e.event === "trajectory_started").details.resweptUnderClaim, true);
});

test("a self-collision is reported without ever claiming a room", async () => {
  const arbiter = new RegionArbiter();
  const sim = simulator({ selfCollision: true });
  const result = await continuousHome(sim, 1, "A", [0], reservedWhole, arbiter, () => [region], log);
  assert.equal(result.success, false);
  assert.equal(result.mode, "self_collision");
  assert.equal(result.safeToReplan, true);
  assert.deepEqual(arbiter.activeRobots, []);
  assert.equal(sim.calls.filter((code) => code.includes("womatBeginPreflight")).length, 1,
    "a verdict no peer can lift is not paid for twice");
  assert.equal(sim.calls.some((code) => code.includes("womatFollowConfigs")), false);
});

test("lost motion status stops the trajectory before releasing its claims", async () => {
  const arbiter = new RegionArbiter();
  const sim = simulator();
  const execute = sim.executeScriptString;
  let stopped = false;
  sim.executeScriptString = async (code) => {
    if (code.includes("womatHomePathStatus")) return { state: 4 };
    if (code.includes("womatStopPath")) { stopped = true; return { ok: true }; }
    return execute(code);
  };
  const release = arbiter.release.bind(arbiter);
  arbiter.release = async (robot) => { assert.equal(stopped, true); return release(robot); };
  assert.equal((await continuousHome(sim, 1, "A", [0], options, arbiter, () => [region], log)).reason, "home_path_status_invalid");
});

test("continuous trajectory respects velocity, acceleration and jerk bounds", () => {
  const dt = 0.05;
  const path = homeTrajectory([3, -1], [0, 0], options, dt);
  assert.deepEqual(path[0], [3, -1]);
  assert.deepEqual(path.at(-1), [0, 0]);
  let differences = path;
  for (const bound of [options.simpleJointVelocityDeg, options.simpleJointAccelerationDeg, options.simpleJointJerkDeg]) {
    differences = differences.slice(1).map((row, i) => row.map((value, j) => (value - differences[i][j]) / dt));
    assert.ok(Math.max(...differences.flat().map(Math.abs)) <= bound * rad + 1e-8);
  }
});

test("Lua batched sweeps retain contact checks, clearance strides and scene restoration", (t) => {
  const fs = require("node:fs");
  const path = require("node:path");
  const { execFileSync } = require("node:child_process");
  const source = fs.readFileSync(path.join(__dirname, "../lua/04-safety.lua"), "utf8");
  const driver = source.slice(source.indexOf("function womatSweepSampleUnsafe("), source.indexOf("function womatPointObstacleCollision("));
  const assertions = `
jointHandles={1}
womatPreflightClearanceStride=6
local position=0
local contacts={}
local clearances={}
local blocked=false
sim={}
function sim.setJointPosition(_, value) position=value end
function sim.checkCollision()
    contacts[#contacts+1]=position
    return (blocked and position == 5) and 1 or 0
end
function womatObstacleCollection() return 1 end
function womatRobotCollisionCollection() return 2 end
function womatRobotSelfCollision() return false end
function womatClearanceUnsafe() clearances[#clearances+1]=position; return false end
function womatCaptureSweepPosture() return position end
function womatRestoreSweepPosture(saved) position=saved end
function womatBlockingObstacleAlias() return 'wall', 'wall' end
assert(womatPreflightJointPath({{1},{2},{3},{4}}, false, 0).ok)
assert(position == 0)
assert(womatPreflightJointPath({{5},{6},{7},{8}}, false, 4).ok)
assert(position == 0)
assert(womatPreflightJointPath({{9}}, true, 8).ok)
assert(position == 0)
assert(#contacts == 9)
for i=1,9 do assert(contacts[i] == i) end
assert(#clearances == 2 and clearances[1] == 6 and clearances[2] == 9)
blocked=true
local rejected=womatPreflightJointPath({{5},{6}}, false, 4)
assert(not rejected.ok and rejected.mode == 'obstacle_contact')
assert(position == 0)
-- Existing unbatched callers still check their endpoint by default.
blocked=false
assert(womatPreflightJointPath({{1}}).ok)
assert(clearances[#clearances] == 1)
-- Resumable preflight retains every contact check and restores the arm
-- between slices. A deterministic clock makes this independent of CPU speed.
contacts={}
clearances={}
local clock=os.clock
local elapsed=0
os.clock=function() elapsed=elapsed+0.03; return elapsed end
local result=womatBeginPreflight({{1},{2},{3},{4},{5},{6},{7},{8},{9}}, true, 0, 0.05)
local slices=1
assert(result.pending and result.checked == 2)
while result.pending do
    assert(position == 0)
    result=womatContinuePreflight()
    assert(result.ok)
    slices=slices+1
end
assert(slices == 5 and position == 0)
assert(#contacts == 9)
for i=1,9 do assert(contacts[i] == i) end
assert(#clearances == 2 and clearances[1] == 6 and clearances[2] == 9)
assert(womatPreflightJob == nil)
blocked=true
result=womatBeginPreflight({{1},{2},{5},{6}}, true, 0, 0.05)
assert(result.ok and result.pending and position == 0)
result=womatContinuePreflight()
assert(not result.ok and result.blockedAtStep == 3)
assert(position == 0 and womatPreflightJob == nil)
os.clock=clock
`;
  const dir = fs.mkdtempSync(path.join(require("node:os").tmpdir(), "womat-sweep-lua-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  const script = path.join(dir, "sweep.lua");
  fs.writeFileSync(script, driver + assertions);
  execFileSync("lua", [script], { encoding: "utf8", timeout: 5000 });
});

test("the actual Lua coroutine runs through sagged intermediate poses and settles only at home", (t) => {
  const fs = require("node:fs");
  const path = require("node:path");
  const { execFileSync } = require("node:child_process");
  const driver = fs.readFileSync(path.join(__dirname, "../lua/06-path-execution.lua"), "utf8")
    .replace(/\$\{pathTrackingToleranceDeg\}/g, "0.5")
    .replace(/\$\{pathSettleMaxSteps\}/g, "100")
    .replace(/\$\{collisionCheckStepDeg\}/g, "2");
  const setup = `
jointHandles={1}
local position=0.3
local target=position
local unsafe=false
sim={}
function sim.isDynamicallyEnabled() return true end
function sim.setJointTargetPosition(_, value) target=value end
function sim.getJointPosition() return position end
function sim.switchThread() coroutine.yield() end
function womatResetInMotionSafety() end
function womatInMotionUnsafe() return unsafe end
function womatCommandedPostureUnsafe() return womatInMotionUnsafe() end
function womatCurrentFramePositions() return {points={0,0,0,0,0,0.1}} end
`;
  const assertions = `
womatFollowConfigs({0.3,0.2,0.1,0}, true)
local ticks=0
while rState ~= 3 do
    sysCall_actuation()
    -- Reproduce the live shoulder error at each intermediate posture.
    position=target + (target ~= 0 and 0.88*math.pi/180 or 0)
    ticks=ticks+1
    assert(ticks < 20, 'must not settle at intermediate poses')
end
assert(womatPathReached)
assert(womatPathIndex == 4)
assert(position == 0)
assert(womatHomePathStatus().state == 3)
-- The watchdog must stop playback and hold the measured arm on contact.
womatFollowConfigs({0,0.1,0.2}, true)
unsafe=true
sysCall_actuation()
assert(rState == 3)
assert(womatMotionError == 'home_path_collision_detected')
assert(target == position)
`;
  const dir = fs.mkdtempSync(path.join(require("node:os").tmpdir(), "womat-home-lua-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  const script = path.join(dir, "home.lua");
  fs.writeFileSync(script, setup + driver + assertions);
  execFileSync("lua", [script], { encoding: "utf8", timeout: 5000 });
});

test("release retains padding around the measured arm through a small controller correction", async () => {
  const key = "A__B:5,5,5";
  const frame = (x) => [0, 0, 0, x, 0, 0.1];
  const claim = (voxels) => ({ voxels });
  let polls = 0;
  const arbiter = new RegionArbiter();
  const narrow = arbiter.narrow.bind(arbiter);
  arbiter.narrow = async (robot, voxels) => {
    assert.ok(voxels.includes(key), "padding is not released while the measured arm can correct into it");
    return narrow(robot, voxels);
  };
  const sim = { executeScriptString: async (code) => {
    if (code.includes("womatSweepFramePositions")) return { ok: true,
      frames: code.match(/\{([^}]+)\}/)[1].split(",").map((v) => frame(Number(v))) };
    if (code.includes("womatSceneRootHandle")) return 7;
    if (code.includes("womatBeginPreflight")) return { ok: true, sliceCpuMs: 2 };
    if (code.includes("womatPreflightJointPath")) return { ok: true, checked: 1 };
    if (code.includes("womatFollowConfigs")) return "started";
    if (code.includes("womatHomePathStatus")) return { state: ++polls > 2 ? 3 : 4,
      index: 3, config: [0.002], frames: frame(polls === 1 ? 0.0004 : 0) };
    throw new Error(code);
  } };
  const events = [];
  const result = await continuousHome(sim, 1, "A", [0.002], {
    ...options, reservedPath: [[0], [0.001], [0.002]],
    pathClaim: (frames) => claim(frames.some((f) => f[3] < 0.0005) ? [key] : []),
    occupancyClaim: (frames) => claim(frames.some((f) => f[3] < 0.0001) ? [key] : [])
  }, arbiter, () => [region], { ...log, record: (event, details) => events.push({ event, details }) });
  assert.equal(result.success, true);
  const timing = events.find((e) => e.event === "trajectory_started").details;
  assert.equal(timing.preflightCpuMs, 2);
  assert.ok(timing.claimWaitMs >= 0 && timing.preflightRpcMs >= timing.preflightMaxCallMs);
  assert.equal(timing.resweptUnderClaim, false);
  assert.ok(timing.preflightMs >= 0 && timing.recheckMs >= 0);
});

// Playback as the coroutine does it: never past the hold point it was given.
function followingSimulator({ resweep = () => ({ ok: true, sliceCpuMs: 1 }) } = {}) {
  const calls = [];
  const limits = [];
  let path = [];
  let index = 0;
  let limit = null;
  let begins = 0;
  const frameOf = (angle) => (angle > 10 * rad ? inside : outside);
  return { calls, limits, get index() { return index; }, executeScriptString: async (code) => {
    calls.push(code);
    if (code.includes("sim.getSimulationTimeStep")) return 0.05;
    if (code.includes("womatSceneRootHandle")) return 42;
    if (code.includes("womatCurrentConfig")) return [0, [20 * rad]];
    if (code.includes("womatSweepFramePositions")) {
      const values = code.match(/\{([^}]+)\}/)[1].split(",").map(Number);
      return { ok: true, frames: values.map(frameOf) };
    }
    if (code.includes("womatBeginPreflight")) return ++begins === 1 ? { ok: true, sliceCpuMs: 1 } : resweep(code);
    if (code.includes("womatFollowConfigs")) {
      path = code.match(/\{([^}]+)\}/)[1].split(",").map(Number);
      const given = code.match(/, true, (-?\d+)\)$/);
      limit = given && Number(given[1]) >= 0 ? Number(given[1]) : null;
      limits.push(limit);
      return "started";
    }
    if (code.includes("womatHomePathStatus")) {
      const given = code.match(/womatHomePathStatus\((-?\d+)\)/);
      if (given) limit = Number(given[1]) < 0 ? null : Number(given[1]);
      if (limits.at(-1) !== limit) limits.push(limit);
      index = Math.min(path.length, limit === null ? index + 8 : Math.max(index, Math.min(limit, index + 8)));
      if (index >= path.length) return { state: 3, index, config: [path.at(-1)], frames: outside };
      const angle = path[Math.max(0, index - 1)];
      return { state: 4, index, config: [angle], frames: frameOf(angle) };
    }
    if (code.includes("womatStopPath")) return { ok: true };
    throw new Error(`Unexpected call: ${code}`);
  } };
}

async function until(condition, timeoutMs = 3000) {
  const deadline = Date.now() + timeoutMs;
  while (!condition()) {
    if (Date.now() > deadline) throw new Error("condition not reached");
    await new Promise((resolve) => setTimeout(resolve, 10));
  }
}

test("a follower holds behind the arm ahead and moves as soon as that arm has left what it needs", async () => {
  const arbiter = new RegionArbiter();
  assert.equal((await arbiter.acquire("A", region.allKeys(), { root: 99 })).granted, true);
  const sim = followingSimulator();
  const home = continuousHome(sim, 1, "B", [0], options, arbiter, () => [region], log);
  // Granted at once, behind A -- not refused and retried.
  await until(() => sim.calls.filter((code) => code.includes("womatHomePathStatus")).length >= 3);
  assert.deepEqual(arbiter.activeRobots, ["A", "B"]);
  assert.equal(sim.index, 0, "never commanded into space A still holds");
  assert.deepEqual(sim.limits, [0]);
  // Swept with A exempt: where A stands now is somewhere it is leaving.
  const sweep = sim.calls.find((code) => code.includes("womatBeginPreflight"));
  assert.match(sweep, /, \{99\}\)$/);
  // A leaves the shared voxel without finishing its motion.
  await arbiter.narrow("A", []);
  const result = await home;
  assert.equal(result.success, true);
  assert.deepEqual(sim.limits, [0, null], "the hold is lifted, not stepped through");
  assert.equal(sim.calls.filter((code) => code.includes("womatBeginPreflight")).length, 1,
    "an arm that moved on is not re-checked against");
  assert.deepEqual(arbiter.activeRobots, ["A"]);
});

test("an arm that parks in the rest of the path is checked against before the path goes on", async () => {
  for (const clear of [true, false]) {
    const arbiter = new RegionArbiter();
    await arbiter.acquire("A", region.allKeys(), { root: 99 });
    const resweeps = [];
    const sim = followingSimulator({ resweep: (code) => {
      resweeps.push(code);
      return clear ? { ok: true, sliceCpuMs: 1 }
        : { ok: false, mode: "obstacle_contact", obstacle: "link3", obstacleOwner: "A", blockedAtStep: 1 };
    } });
    const home = continuousHome(sim, 1, "B", [0], options, arbiter, () => [region], log);
    await until(() => sim.calls.filter((code) => code.includes("womatHomePathStatus")).length >= 2);
    // A finishes: its claim is gone, and it is standing wherever it stopped.
    await arbiter.release("A");
    const result = await home;
    assert.equal(resweeps.length, 1);
    assert.doesNotMatch(resweeps[0], /\{99\}\)$/, "the parked arm is no longer exempt");
    if (clear) {
      assert.equal(result.success, true);
    } else {
      assert.equal(result.reason, "followed_arm_parked_in_path");
      assert.equal(result.parkedRobot, "A");
      assert.equal(result.safeToReplan, true, "nothing had moved yet");
      assert.equal(sim.index, 0);
      assert.deepEqual(arbiter.activeRobots, []);
    }
  }
});

test("a follower that is never let through gives up instead of holding its claim forever", async () => {
  const arbiter = new RegionArbiter();
  await arbiter.acquire("A", region.allKeys(), { root: 99 });
  const sim = followingSimulator();
  const result = await continuousHome(sim, 1, "B", [0], { ...options, followWaitMs: 1000 },
    arbiter, () => [region], log);
  assert.equal(result.reason, "reservation_follow_timeout");
  assert.deepEqual(result.behind, ["A"]);
  assert.equal(result.safeToReplan, true);
  assert.deepEqual(arbiter.activeRobots, ["A"]);
});

test("the Lua path holds at its limit, still watched, and resumes when it is lifted", (t) => {
  const fs = require("node:fs");
  const path = require("node:path");
  const { execFileSync } = require("node:child_process");
  const driver = fs.readFileSync(path.join(__dirname, "../lua/06-path-execution.lua"), "utf8")
    .replace(/\$\{pathTrackingToleranceDeg\}/g, "0.5")
    .replace(/\$\{pathSettleMaxSteps\}/g, "100")
    .replace(/\$\{collisionCheckStepDeg\}/g, "2");
  const script = `
jointHandles={1}
local position=0.3
local watched=0
sim={}
function sim.isDynamicallyEnabled() return false end
function sim.setJointPosition(_, value) position=value end
function sim.setJointTargetPosition(_, value) position=value end
function sim.getJointPosition() return position end
function sim.switchThread() coroutine.yield() end
function womatResetInMotionSafety() end
function womatInMotionUnsafe() watched=watched+1; return false end
function womatCommandedPostureUnsafe() return womatInMotionUnsafe() end
function womatCurrentFramePositions() return {points={0,0,0,0,0,0.1}} end
${driver}
womatFollowConfigs({0.3,0.2,0.1,0}, true, 1)
for i=1,10 do sysCall_actuation() end
assert(womatPathIndex == 1 and position == 0.3, 'held after the last permitted sample')
assert(watched >= 10, 'the watchdog keeps running while held')
assert(womatHomePathStatus().limit == 1, 'a bare status poll leaves the hold alone')
womatHomePathStatus(2)
for i=1,10 do sysCall_actuation() end
assert(womatPathIndex == 2 and position == 0.2)
womatHomePathStatus(-1)
local ticks=0
while rState ~= 3 do sysCall_actuation(); ticks=ticks+1; assert(ticks < 50) end
assert(womatPathReached and position == 0)
-- A new path never inherits the previous one's hold.
womatFollowConfigs({0.1,0.2}, true)
assert(womatPathLimit == nil)
`;
  const dir = fs.mkdtempSync(path.join(require("node:os").tmpdir(), "womat-hold-lua-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  fs.writeFileSync(path.join(dir, "hold.lua"), script);
  execFileSync("lua", [path.join(dir, "hold.lua")], { encoding: "utf8", timeout: 5000 });
});

test("an ordered sweep ignores only the arms it follows, and only for the length of the call", (t) => {
  const fs = require("node:fs");
  const path = require("node:path");
  const { execFileSync } = require("node:child_process");
  const source = fs.readFileSync(path.join(__dirname, "../lua/04-safety.lua"), "utf8");
  const driver = source.slice(source.indexOf("function womatSweepSampleUnsafe("), source.indexOf("function womatPointObstacleCollision("));
  const script = `${driver}
jointHandles={1}
womatPreflightClearanceStride=0
local parents={[1]=10, [2]=20, [3]=-1, [10]=-1, [20]=-1}
local members={}
local nextCollection=100
local checkedAgainst={}
sim={}
function sim.getObjectParent(handle) return parents[handle] or -1 end
function sim.setJointPosition() end
function sim.createCollection() nextCollection=nextCollection+1; members[nextCollection]={}; return nextCollection end
function sim.addItemToCollection(collection, _, shape) table.insert(members[collection], shape) end
function sim.destroyCollection() end
function sim.checkCollision(_, obstacles) checkedAgainst[#checkedAgainst+1]=obstacles; return 0 end
function womatSceneObstacleShapes() return {1, 2, 3} end
function womatIsRenderOnlyShape() return false end
function womatObstacleCollection() return 1 end
function womatRobotCollisionCollection() return 2 end
function womatRobotSelfCollision() return false end
function womatClearanceUnsafe() return false end
function womatCaptureSweepPosture() return 0 end
function womatRestoreSweepPosture() end
local result=womatBeginPreflight({{1},{2}}, true, 0, 1, {20})
assert(result.ok and not result.pending)
local scoped=checkedAgainst[1]
assert(scoped ~= 1 and #members[scoped] == 2 and members[scoped][1] == 1 and members[scoped][2] == 3,
  'arm 20 is out of the contact set; the rest, idle arms included, stay in')
assert(womatSweepExemptKey == nil and womatSweepExemptRoots == nil, 'cleared before the call returns')
checkedAgainst={}
assert(womatBeginPreflight({{1}}, true, 0, 1).ok)
assert(checkedAgainst[1] == 1, 'no exemption means the whole cell')
checkedAgainst={}
assert(womatPreflightJointPath({{1}}, true, 0, {20}).ok)
assert(checkedAgainst[1] == scoped, 'the same exemption is served from cache')
assert(womatSweepExemptKey == nil)
`;
  const dir = fs.mkdtempSync(path.join(require("node:os").tmpdir(), "womat-exempt-lua-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  fs.writeFileSync(path.join(dir, "exempt.lua"), script);
  execFileSync("lua", [path.join(dir, "exempt.lua")], { encoding: "utf8", timeout: 5000 });
});

test("a follower is held by what its body would reach, not by the pocket its reservation arms", async () => {
  // Two voxels: 5,5,5 on this arm's path, and 9,9,9 standing for the edge of a
  // handover pocket -- claimed by the reservation because the tool is near the
  // rendezvous, reached by no link of the arm at any sample.
  const cell = new SharedRegion({ pairKey: "A__B", robots: ["A", "B"],
    voxelSize: 0.1, origin: { x: 0, y: 0, z: 0 }, voxelKeys: ["5,5,5", "9,9,9"] });
  const arbiter = new RegionArbiter();
  // The arm ahead is parked beside the pocket for its whole motion.
  await arbiter.acquire("A", ["A__B:9,9,9"], { root: 99 });
  const sim = followingSimulator();
  const result = await continuousHome(sim, 1, "B", [0], {
    ...options, pathClaim: () => ({ voxels: ["A__B:9,9,9"] })
  }, arbiter, () => [cell], log);
  assert.equal(result.success, true);
  assert.deepEqual(sim.limits, [null], "never held: no link of this arm goes near what A holds");
  assert.deepEqual(arbiter.activeRobots, ["A"], "and it did not wait for A to finish");
});
