"use strict";

const assert = require("node:assert/strict");
const test = require("node:test");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { loadSharedRegions } = require("./runtimeOptions");
const { handoverClearance } = require("./thing");
const { SharedRegion, conservativeClaim } = require("./claims");
const { RegionArbiter } = require("./motionArbiter");

const region = new SharedRegion({ pairKey: "A__B", robots: ["A", "B"],
  origin: { x: 0, y: 0, z: 0 }, voxelSize: 0.1, voxelKeys: ["5,5,5"] });
const options = { claimBodyRadius: 0.01 };

test("clearance checks the elbow as well as the tool and fails closed without geometry", () => {
  assert.equal(handoverClearance("A", [region], [0, 0, 0, 0.55, 0.55, 0.55, 0, 0, 0.2], options).clear, false);
  assert.equal(handoverClearance("A", [region], [0, 0, 0, 0, 0, 0.2], options).clear, true);
  for (const points of [[], [0, 0, 0], [0, 0, 0, NaN, 0, 0]]) {
    assert.equal(handoverClearance("A", [region], points, options).clear, false);
  }
  assert.equal(handoverClearance("A", [], [0, 0, 0, 0, 0, 0.2], options).clear, false);
});

test("after retreat releases the room, giver and receiver can hold disjoint motion claims together", async () => {
  const arbiter = new RegionArbiter();
  assert.equal((await arbiter.acquire("A", region.allKeys())).granted, true);
  assert.equal((await arbiter.acquire("B", region.allKeys())).granted, false);
  const frames = [0, 0, 0, 0, 0, 0.2];
  assert.equal(handoverClearance("A", [region], frames, options).clear, true);
  await arbiter.release("A");
  const home = conservativeClaim("A", [region], [frames], { bodyRadius: 0.01 });
  assert.equal((await arbiter.acquire("A", home.voxels)).granted, true);
  assert.equal((await arbiter.acquire("B", region.allKeys())).granted, true);
  assert.deepEqual(arbiter.activeRobots, ["A", "B"]);
});

test("regions exported by agents after core startup are loaded without a descriptor pair entry", (t) => {
  const repoRoot = fs.mkdtempSync(path.join(os.tmpdir(), "womat-live-regions-"));
  t.after(() => fs.rmSync(repoRoot, { recursive: true, force: true }));
  fs.writeFileSync(path.join(repoRoot, "cell.json"), JSON.stringify({ robots: [{ name: "A" }, { name: "B" }] }));
  const ctx = { repoRoot, manifest: { name: "A", sharedEnvironment: { descriptor: "cell.json" } } };
  assert.deepEqual(loadSharedRegions(ctx), []);
  const dir = path.join(repoRoot, "generated/robots/interactions/A__B");
  fs.mkdirSync(dir, { recursive: true });
  fs.writeFileSync(path.join(dir, "space.json"), JSON.stringify({ pairKey: "A__B", voxelSize: 0.1, origin: { x: 0, y: 0, z: 0 } }));
  fs.writeFileSync(path.join(dir, "space.csv"), "X,Y,Z,RobotCount\n0.55,0.55,0.55,2\n");
  assert.deepEqual(loadSharedRegions(ctx)[0].allKeys(), region.allKeys());
  fs.writeFileSync(path.join(dir, "space.csv"), "X,Y,Z,RobotCount\n0.85,0.55,0.55,2\n");
  assert.deepEqual(loadSharedRegions(ctx)[0].allKeys(), ["A__B:8,5,5"]);
});

test("a core exposes its Thing before agents export regions and uses their later export for motion locking", async (t) => {
  const vm = require("node:vm");
  const { createRequire } = require("node:module");
  const filename = path.join(__dirname, "thing.js");
  const requireCore = createRequire(filename);
  const repoRoot = fs.mkdtempSync(path.join(os.tmpdir(), "womat-core-startup-"));
  t.after(() => fs.rmSync(repoRoot, { recursive: true, force: true }));
  fs.writeFileSync(path.join(repoRoot, "cell.json"), JSON.stringify({ robots: [{ name: "A" }, { name: "B" }] }));
  const ctx = { repoRoot, paths: { td: "test-td" }, manifest: {
    name: "A", ports: { lowerThing: 8110 }, sharedEnvironment: { descriptor: "cell.json" }
  } };
  const handlers = new Map();
  const logs = [];
  let exposed = false;
  let producedTD;
  let motions = 0;
  const thing = {
    setPropertyReadHandler() {},
    setActionHandler(name, handler) { handlers.set(name, handler); },
    async expose() { exposed = true; }
  };
  // Run the complete startup and real action handlers. Substitute only the
  // server/simulator boundaries, workspace admission and logging sinks.
  const substitutes = {
    "@node-wot/core": { Servient: class {
      addServer() {}
      async start() { return { produce: async (td) => { producedTD = td; return thing; } }; }
    } },
    "@node-wot/binding-http": { HttpServer: class {} },
    "./context": { readJson: () => ({ title: "A", properties: {}, actions: {
      moveTojointPosition: { input: { properties: { joint1: {}, joint2: {} } } }
    } }) },
    "./runtimeOptions": { ...requireCore("./runtimeOptions"), loadWorkspaceGrid: () => null },
    "./motionLog": { motionLogFor: () => ({ enabled: false,
      begin: () => ({ refused() {}, granted() {}, finished() {} }) }) },
    "./motion": { handoverRetreat: async () => { motions++; return { success: true }; } }
  };
  const module = { exports: {} };
  vm.runInNewContext(fs.readFileSync(filename, "utf8"), {
    module, require: (name) => substitutes[name] || requireCore(name),
    console: { log: (line) => logs.push(line), warn() {}, error() {} }
  }, { filename });
  const sim = { executeScriptString: async (code) => code.includes("womatCurrentFramePositions")
    ? { points: [0.55, 0.55, 0.55, 0.56, 0.55, 0.55] }
    : { success: true } };
  const arbiter = new RegionArbiter();
  await module.exports.startThing(ctx, sim, 1, arbiter);
  assert.equal(exposed, true, "startup must reach expose even with no region artifacts");
  assert.ok(logs.some((line) => line.includes("regions created later by agents")));

  const dir = path.join(repoRoot, "generated/robots/interactions/A__B");
  fs.mkdirSync(dir, { recursive: true });
  fs.writeFileSync(path.join(dir, "space.json"), JSON.stringify({ pairKey: "A__B", voxelSize: 0.1, origin: { x: 0, y: 0, z: 0 } }));
  fs.writeFileSync(path.join(dir, "space.csv"), "X,Y,Z,RobotCount\n0.55,0.55,0.55,2\n");
  await arbiter.acquire("B", region.allKeys());
  const retreat = handlers.get("retreatFromHandover");
  const refused = await retreat();
  assert.equal(refused.reason, "region_claimed");
  assert.equal(refused.activeRobot, "B");
  assert.equal(refused.conflictRegion, "A__B");
  assert.equal(motions, 0, "a region exported after startup must protect the motion");
  await arbiter.release("B");
  assert.equal((await retreat()).success, true);
  assert.equal(motions, 1);

  // Reproduce the live boundary, not just a bridge with an always-successful
  // invokeAction stub: validate the advertised TD input and execute the actual
  // lower handler, session checks, room arbitration and continuous home execution.
  const Ajv = require("ajv");
  const validate = new Ajv().compile(producedTD.actions.returnHomeAfterHandover.input);
  assert.equal(validate({ session: "s1", homeJoints: "array(0,0)" }), false);
  await handlers.get("registerHandoverSession")({ value: async () => ({ session: {
    id: "s1", sourceRobot: "A", targetRobot: "B", state: "transferred"
  } }) });
  let joints = [10 * Math.PI / 180, 10 * Math.PI / 180];
  const movingTogether = [];
  const originalNarrow = arbiter.narrow.bind(arbiter);
  let pathStates = 0;
  let polls = 0;
  arbiter.narrow = async (robot, voxels) => {
    const result = await originalNarrow(robot, voxels);
    if (!voxels.length) {
      assert.equal((await arbiter.acquire("B", region.allKeys())).granted, true);
      movingTogether.push(arbiter.activeRobots.slice());
    }
    return result;
  };
  sim.executeScriptString = async (code) => {
    if (code.includes("womatCurrentConfig")) return [0, joints];
    if (code.includes("sim.getSimulationTimeStep")) return 0.05;
    if (code.includes("womatSweepFramePositions")) {
      const flat = code.match(/\{([^}]+)\}/)[1].split(",").map(Number);
      const frames = [];
      for (let i = 0; i < flat.length; i += 2) frames.push(flat[i] > 5 * Math.PI / 180 + 1e-8
        ? [0, 0, 0, 0.55, 0.55, 0.55] : [0, 0, 0, 0, 0, 0.1]);
      return { ok: true, frames };
    }
    if (code.includes("womatSceneRootHandle")) return 7;
    if (code.includes("womatBeginPreflight")) return { ok: true };
    if (code.includes("womatPreflightJointPath")) return { ok: true, checked: 1 };
    if (code.includes("womatFollowConfigs")) {
      pathStates = code.match(/\{([^}]+)\}/)[1].split(",").length / 2;
      movingTogether.push(arbiter.activeRobots.slice());
      return "started";
    }
    if (code.includes("womatHomePathStatus")) {
      if (++polls === 1) return { state: 4, index: Math.ceil(pathStates * 0.9),
        config: [0.01, 0.01], frames: [0, 0, 0, 0, 0, 0.1] };
      joints = [0, 0];
      return { state: 3, index: pathStates, config: joints, frames: [0, 0, 0, 0, 0, 0.1] };
    }
    if (code === "return check()") return 3;
    if (code.includes("womatGetMotionError")) return null;
    throw new Error(code);
  };
  const { GeneratedRobotAgentThing } = require("../agentThingServer");
  const bridge = Object.create(GeneratedRobotAgentThing.prototype);
  bridge.ctx = ctx;
  bridge.motionQueue = Promise.resolve();
  bridge.ensureLowerThing = async () => {};
  bridge.lowerThing = { invokeAction: async (name, payload) => {
    assert.equal(validate(payload), true, JSON.stringify(validate.errors));
    const result = await handlers.get(name)({ value: async () => payload });
    return { value: async () => result };
  } };
  const beliefs = [];
  bridge.addBelief2Agent = async (name, values) => beliefs.push([name, values]);
  await bridge.handleCustomAgentRequests({ targetAgentId: "ownLayer1", content: {
    keyword: "goHomeAfterHandover", value: ["s1", "array(0,0)"]
  } });
  assert.deepEqual(joints, [0, 0]);
  assert.deepEqual(beliefs, [["movedTo", ["home"]]]);
  assert.deepEqual(movingTogether, [["A"], ["A", "B"]]);
});
