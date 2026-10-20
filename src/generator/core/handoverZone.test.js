"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const { SharedRegion } = require("./claims");
const { RegionArbiter } = require("./motionArbiter");
const { sessionZone, handoverPathClaims, zoneKeys } = require("./handoverZone");
const { continuousHome } = require("./continuousHome");
const { createRuntimeOptions } = require("./runtimeOptions");
const keys = [];
for (let x = 0; x < 40; x++) for (let y = 0; y < 10; y++) for (let z = 0; z < 10; z++) keys.push(`${x},${y},${z}`);
const region = new SharedRegion({ pairKey: "A__B", robots: ["A", "B"],
  voxelSize: 0.1, origin: { x: 0, y: 0, z: 0 }, voxelKeys: keys });
const zone = { pairKey: "A__B", center: { x: 2, y: 0.5, z: 0.5 }, radius: 0.15 };
const left = [0.3, 0.4, 0.5, 0.4, 0.5, 0.5];
const right = [3.5, 0.4, 0.5, 3.6, 0.5, 0.5];
const claims = (robot, zones = [zone]) => handoverPathClaims(robot, [region], zones, 0);
const manifest = (name, radius = 0.12) => ({ name, runtime: { handoverZone: { enabled: true, radius } } });

test("a session zone uses the selected world point and the larger of the peers' radii", () => {
  assert.deepEqual(sessionZone(manifest("B", 0.2), manifest("A"), zone.center), { ...zone, radius: 0.2 });
  assert.equal(sessionZone(manifest("A"), { name: "B" }, zone.center), null);
  assert.equal(sessionZone(manifest("A"), manifest("B"), { x: 0, y: NaN, z: 0 }), null);
});

test("outside the handover zone, disjoint arm paths can share the same larger region", async () => {
  const arbiter = new RegionArbiter();
  const a = claims("A")([left]);
  const b = claims("B")([right]);
  assert.ok(a.voxels.length && b.voxels.length);
  assert.equal((await arbiter.acquire("A", a.voxels)).granted, true);
  assert.equal((await arbiter.acquire("B", b.voxels)).granted, true);
  assert.deepEqual(arbiter.activeRobots, ["A", "B"]);
});

test("the arm whose tool is in the pocket locks the whole pocket, and normal claims still conflict", async () => {
  // A's tool sits on the pocket boundary; B's is 0.2 m out, its links reaching in.
  const a = claims("A")([[1.8, 0.5, 0.5, 1.85, 0.5, 0.5]]);
  const b = claims("B")([[2.15, 0.5, 0.5, 2.2, 0.5, 0.5]]);
  const zoneVoxels = zoneKeys(region, zone);
  assert.ok(zoneVoxels.every((key) => a.voxels.includes(key)), "the tool inside locks the whole pocket");
  assert.ok(!zoneVoxels.every((key) => b.voxels.includes(key)), "a tool outside it does not");
  const arbiter = new RegionArbiter();
  await arbiter.acquire("A", a.voxels);
  // Not locking the pocket is not permission to move through it: B's own swept
  // path still overlaps what A holds.
  assert.equal((await arbiter.acquire("B", b.voxels)).granted, false);
  assert.equal((await arbiter.acquire("B", region.allKeys())).granted, false);
});

test("two arms retreating from one rendezvous stop contending as soon as their tools leave the pocket", async () => {
  const arbiter = new RegionArbiter();
  // Both tools 0.25 m out, on opposite sides, links trailing away from each other.
  const a = claims("A")([[1.55, 0.5, 0.5, 1.75, 0.5, 0.5]]);
  const b = claims("B")([[2.45, 0.5, 0.5, 2.25, 0.5, 0.5]]);
  assert.ok(a.voxels.length && b.voxels.length);
  assert.equal((await arbiter.acquire("A", a.voxels)).granted, true);
  assert.equal((await arbiter.acquire("B", b.voxels)).granted, true);
  assert.deepEqual(arbiter.activeRobots, ["A", "B"]);
});

test("an overlapping elbow outside the zone remains a conflict", async () => {
  const arbiter = new RegionArbiter();
  await arbiter.acquire("A", claims("A")([left]).voxels);
  assert.equal((await arbiter.acquire("B", claims("B")([left]).voxels)).granted, false);
});

test("a rendezvous region whose pocket does not resolve retains a full-region claim", () => {
  // The zone names this pair but lands nowhere in it: the geometry the motion
  // is being arbitrated around is wrong, so stay coarse.
  assert.deepEqual(claims("A", [{ ...zone, center: { x: 90, y: 0, z: 0 } }])([left]).voxels, region.allKeys());
});

test("regions this handover is not about are claimed from the path, not taken whole", () => {
  // Robot A also shares space with C. Leaving the A/B rendezvous must not
  // reserve the whole of A__C and stall an arm that is nowhere near it.
  const other = new SharedRegion({ pairKey: "A__C", robots: ["A", "C"],
    voxelSize: 0.1, origin: { x: 0, y: 0, z: 0 }, voxelKeys: keys });
  const claim = handoverPathClaims("A", [region, other], [zone], 0)([left]);
  const inOther = claim.voxels.filter((key) => key.startsWith("A__C:"));
  assert.ok(inOther.length > 0, "the path through A__C is still claimed");
  assert.ok(inOther.length < other.allKeys().length / 4,
    `claimed ${inOther.length} of ${other.allKeys().length} voxels of a region this handover is not about`);
});

test("two continuous trajectories run concurrently within one region when their reserved paths are disjoint", async () => {
  const arbiter = new RegionArbiter();
  const options = createRuntimeOptions({ manifest: { runtime: {} } });
  const log = { begin: () => ({ granted() {}, refused() {}, finished() {} }) };
  let started = 0;
  let bothStarted;
  const rendezvous = new Promise((resolve) => { bothStarted = resolve; });
  const simul = (frame) => ({ executeScriptString: async (code) => {
    if (code.includes("womatCurrentConfig")) return [0, [0.05]];
    if (code.includes("sim.getSimulationTimeStep")) return 0.05;
    if (code.includes("womatSweepFramePositions")) return { ok: true,
      frames: code.match(/\{([^}]+)\}/)[1].split(",").map(() => frame) };
    if (code.includes("womatSceneRootHandle")) return 7;
    if (code.includes("womatBeginPreflight")) return { ok: true };
    if (code.includes("womatPreflightJointPath")) return { ok: true, checked: 1 };
    if (code.includes("womatFollowConfigs")) {
      if (++started === 2) bothStarted();
      await rendezvous;
      assert.deepEqual(arbiter.activeRobots, ["A", "B"]);
      return "started";
    }
    if (code.includes("womatHomePathStatus")) return { state: 3, index: 20, config: [0], frames: frame };
    throw new Error(code);
  } });
  const results = await Promise.all(["A", "B"].map((robot, i) => continuousHome(simul(i ? right : left), 1,
    robot, [0], { ...options, pathClaim: claims(robot) }, arbiter, () => [region], log)));
  assert.ok(results.every((result) => result.success));
  assert.equal(started, 2);
});

test("the real lower core retains the incoming zone for one departure, then restores normal room locking", async (t) => {
  const fs = require("node:fs");
  const os = require("node:os");
  const path = require("node:path");
  const vm = require("node:vm");
  const { createRequire } = require("node:module");
  const filename = path.join(__dirname, "thing.js");
  const requireCore = createRequire(filename);
  const repoRoot = fs.mkdtempSync(path.join(os.tmpdir(), "womat-zone-core-"));
  t.after(() => fs.rmSync(repoRoot, { recursive: true, force: true }));
  fs.writeFileSync(path.join(repoRoot, "cell.json"), JSON.stringify({ robots: [{ name: "A" }, { name: "B" }] }));
  const dir = path.join(repoRoot, "generated/robots/interactions/A__B");
  fs.mkdirSync(dir, { recursive: true });
  fs.writeFileSync(path.join(dir, "space.json"), JSON.stringify({ pairKey: region.pairKey,
    voxelSize: region.voxelSize, origin: region.origin }));
  fs.writeFileSync(path.join(dir, "space.csv"), "X,Y,Z,RobotCount\n" + keys.map((key) =>
    key.split(",").map((v) => (Number(v) + 0.5) * 0.1).join(",") + ",2").join("\n"));
  const ctx = { repoRoot, paths: { td: "test-td" }, manifest: {
    ...manifest("A"), ports: { lowerThing: 8110 }, sharedEnvironment: { descriptor: "cell.json" }
  } };
  Object.assign(ctx.manifest.runtime, { simpleSmoothMotion: true, returnHomeBetweenHandovers: true });
  const handlers = new Map();
  const thing = { setPropertyReadHandler() {}, setActionHandler(name, handler) { handlers.set(name, handler); }, async expose() {} };
  const substitutes = {
    "@node-wot/core": { Servient: class { addServer() {} async start() { return { produce: async () => thing }; } } },
    "@node-wot/binding-http": { HttpServer: class {} },
    "./context": { readJson: () => ({ title: "A", properties: {}, actions: {
      moveTojointPosition: { input: { properties: { joint1: {} } } },
      setHandoverApproach: {}
    } }) },
    "./runtimeOptions": { ...requireCore("./runtimeOptions"), loadWorkspaceGrid: () => null },
    "./motionLog": { motionLogFor: () => ({ begin: () => ({ refused() {}, granted() {}, finished() {} }) }) }
  };
  const module = { exports: {} };
  vm.runInNewContext(fs.readFileSync(filename, "utf8"), { module,
    require: (name) => substitutes[name] || requireCore(name), console }, { filename });
  let joints = [0.05];
  let tcp = { x: 0.3, y: 0.5, z: 0.5 };
  let starts = 0;
  const arbiter = new RegionArbiter();
  const sim = { executeScriptString: async (code) => {
    if (code.includes("womatPickupApproachEnabled") || code === "return womatHandoverPoseEnabled") return false;
    if (code.includes("getCartesianposition")) return tcp;
    if (code.includes("womatCurrentConfig")) return [0, joints];
    if (code.includes("sim.getSimulationTimeStep")) return 0.05;
    if (code.includes("womatCurrentFramePositions")) return { points: left };
    if (code.includes("womatSimpleCartesianRouteSetup")) return { ok: true, from: tcp, dt: 0.05 };
    if (code.includes("womatPrepareSimpleCartesianRoute")) return { ok: true, dof: 1, path: [0.05, 0.02] };
    if (code.includes("womatSweepFramePositions")) return { ok: true,
      frames: code.match(/\{([^}]+)\}/)[1].split(",").map(() => left) };
    if (code.includes("womatSceneRootHandle")) return 7;
    if (code.includes("womatBeginPreflight")) return { ok: true };
    if (code.includes("womatPreflightJointPath")) return { ok: true, checked: 1 };
    if (code.includes("womatFollowConfigs")) {
      starts++;
      assert.deepEqual(arbiter.activeRobots, ["B", "A"]);
      joints = [0.02]; tcp = { x: 0.4, y: 0.5, z: 0.5 };
      return "started";
    }
    if (code.includes("womatHomePathStatus")) return { state: 3, index: 2, config: joints, frames: left };
    if (code.includes("womatStartSimple")) throw new Error("An unchecked motion must not run under the smaller claim");
    return { success: true };
  } };
  await module.exports.startThing(ctx, sim, 1, arbiter, {});
  await handlers.get("registerHandoverSession")({ value: async () => ({ session: {
    id: "received", sourceRobot: "B", targetRobot: "A", state: "transferred", handoverZone: zone
  } }) });
  await handlers.get("setHandoverApproach")({ value: async () => ({ session: "received", enabled: false }) });
  await arbiter.acquire("B", claims("B")([right]).voxels);
  const move = () => handlers.get("moveTocartesianPosition")({ value: async () => ({ x: 0.4, y: 0.5, z: 0.5 }) });
  assert.equal((await move()).success, true);
  assert.equal(starts, 1);
  const normal = await move();
  assert.equal(normal.reason, "region_claimed");
  assert.equal(starts, 1, "the second move must use normal whole-region locking");
});

test("the Lua Cartesian proposal records the legacy IK route and restores the scene on success and failure", (t) => {
  const fs = require("node:fs");
  const os = require("node:os");
  const path = require("node:path");
  const { execFileSync } = require("node:child_process");
  const source = fs.readFileSync(path.join(__dirname, "../lua/02-simple-motion.lua"), "utf8");
  const proposal = source.slice(source.indexOf("womatCartesianRouteJob=nil"));
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-zone-lua-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  const script = path.join(dir, "route.lua");
  fs.writeFileSync(script, `
jointHandles={1}
womatSimpleCartesianVelocity=0.175
womatSimpleCartesianAcceleration=0.25
womatSimpleCartesianJerk=0.75
local joint=0.1
local fail=false
local restored=0
sim={getSimulationTimeStep=function() return 0.05 end, getJointPosition=function() return joint end,
    setJointPosition=function(_, value) joint=value end}
function womatEnsureSimplePositionIk() return true end
function womatCaptureSweepPosture() return joint end
function womatRestoreSweepPosture(saved) joint=saved; restored=restored+1 end
function getCartesianposition() return {x=joint,y=0,z=0} end
function womatApplyCartesianTarget(x,y,z)
    joint=x
    if fail then error('IK failed') end
end
` + proposal + `
-- The core owns the time law; this stands in for the schedule it hands over.
local schedule={}
for i=1,65 do schedule[i]=i/65 end
local result=womatPrepareSimpleCartesianRoute(0.4,0,0,0.025,schedule)
assert(result.ok)
assert(result.dof == 1)
assert(result.path[1] == 0.1)
local previous=0.1
local chunks=0
while true do
    chunks=chunks+1
    assert(#result.path <= 9, 'each call must bound the number of IK solves')
    for i=1,#result.path do assert(result.path[i] >= previous); previous=result.path[i] end
    assert(joint == 0.1 and restored == chunks, 'every chunk restores the live arm')
    if not result.pending then break end
    result=womatContinueSimpleCartesianRoute()
    assert(result.ok)
end
assert(chunks > 1)
assert(math.abs(previous-0.4) < 1e-8)
assert(womatCartesianRouteJob == nil)
fail=true
result=womatPrepareSimpleCartesianRoute(0.4,0,0,0.025,schedule)
assert(not result.ok and result.reason == 'cartesian_route_ik_failed')
assert(joint == 0.1 and restored == chunks+1, 'failed proposal must restore the arm too')
assert(womatCartesianRouteJob == nil)
-- A route with no schedule has no pace, so it is refused rather than guessed at.
result=womatPrepareSimpleCartesianRoute(0.4,0,0,0.025)
assert(not result.ok and result.reason == 'cartesian_route_schedule_missing')
assert(restored == chunks+1, 'a refused route touches nothing')
`);
  execFileSync("lua", [script], { encoding: "utf8", timeout: 5000 });
});

// Reproduce a sub-millimetre crossing of a 48 mm grid boundary. Comparing two
// cell-dilated claims used to report 38 escaped voxels despite the 30 mm margin.
function boundaryFixture() {
  const keys = [];
  for (let x = -6; x <= 6; x++) for (let y = -6; y <= 6; y++) for (let z = -6; z <= 6; z++) keys.push(`${x},${y},${z}`);
  const region = new SharedRegion({ pairKey: "A__B", robots: ["A", "B"], voxelSize: 0.048,
    origin: { x: 0, y: 0, z: 0 }, voxelKeys: keys });
  const planned = [-0.0001, -0.0001, -0.0001, -0.0001, -0.0001, 0.01];
  const actual = planned.map((v, i) => i % 3 === 2 ? v : v + 0.0002);
  return { region, planned, actual };
}

test("live occupancy stays inside the padded reservation across voxel boundaries", () => {
  const { region, planned, actual } = boundaryFixture();
  const reserved = new Set(handoverPathClaims("A", [region], [], 0.15)([planned]).voxels);
  const old = handoverPathClaims("A", [region], [], 0.12)([actual]);
  assert.ok(old.voxels.some((key) => !reserved.has(key)), "the old comparison reproduces the false escape");
  const measured = handoverPathClaims("A", [region], [], 0.12, { occupancy: true })([actual]);
  assert.ok(measured.voxels.length > 0);
  assert.ok(measured.voxels.every((key) => reserved.has(key)));
  // A tool in a handover pocket still reserves that pocket whole.
  const pocket = { pairKey: "A__B", center: { x: 0, y: 0, z: 0 }, radius: 0.12 };
  const inZone = handoverPathClaims("A", [region], [pocket], 0.12, { occupancy: true })([actual]);
  assert.ok(zoneKeys(region, pocket).every((key) => inZone.voxels.includes(key)));
});

test("reserved playback tolerates grid jitter but stops a real escape before releasing the claim", async () => {
  const { region, planned, actual } = boundaryFixture();
  for (const escape of [false, true]) {
    const measured = escape ? actual.map((v, i) => i % 3 === 0 ? v + 0.25 : v) : actual;
    let polls = 0, stopped = false;
    const arbiter = new RegionArbiter();
    const release = arbiter.release.bind(arbiter);
    arbiter.release = async (robot) => {
      if (escape) assert.equal(stopped, true);
      return release(robot);
    };
    const sim = { executeScriptString: async (code) => {
      if (code.includes("womatSweepFramePositions")) return { ok: true,
        frames: code.match(/\{([^}]+)\}/)[1].split(",").map(() => planned) };
      if (code.includes("womatSceneRootHandle")) return 7;
      if (code.includes("womatBeginPreflight")) return { ok: true };
      if (code.includes("womatPreflightJointPath")) return { ok: true, checked: 1 };
      if (code.includes("womatFollowConfigs")) return "started";
      if (code.includes("womatHomePathStatus")) return { state: ++polls > 1 ? 3 : 4,
        index: 2, config: [0.02], frames: measured };
      if (code.includes("womatStopPath")) { stopped = true; return { ok: true }; }
      throw new Error(code);
    } };
    const result = await continuousHome(sim, 1, "A", [0.02], {
      ...createRuntimeOptions({ manifest: {} }), reservedPath: [[0], [0.01], [0.02]],
      pathClaim: handoverPathClaims("A", [region], [], 0.15),
      occupancyClaim: handoverPathClaims("A", [region], [], 0.12, { occupancy: true })
    }, arbiter, () => [region], { begin: () => ({ granted() {}, refused() {}, finished() {} }) });
    assert.equal(result.success, !escape);
    if (escape) {
      assert.equal(result.reason, "handover_path_left_reservation");
      assert.ok(result.outsideVoxelCount > 0);
      assert.equal(result.previouslyReleasedCount, 0);
    }
    assert.deepEqual(arbiter.activeRobots, []);
  }
});
