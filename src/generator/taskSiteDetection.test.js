"use strict";

// Stage 0 end to end, without a simulator: the real driver Lua runs against a
// stub cell, and its inventory is classified with the arm's real sampled
// workspace. What this pins down is the thing no unit test can -- that the same
// scene produces DIFFERENT beliefs in two arms, and that which arm holds A and
// which holds B falls out of where they stand.

const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");
const { execFileSync, spawnSync } = require("node:child_process");

const { createContext } = require("./core/context");
const { prepareRuntimeDriverScript } = require("./core/driverScript");
const { sharedGripConfig } = require("./core/scene");
const { classifyTaskSites, taskSitesPayload } = require("./taskSites");
const { ensureSharedCellAffordances } = require("./core/thing");
const Ajv = require("ajv");
const { WorkspaceGrid } = require("./workspaceGrid");
const { checkWorkspace, createRuntimeOptions } = require("./core/runtimeOptions");

const luaAvailable = spawnSync("lua", ["-v"], { stdio: "ignore" }).status === 0;
const root = path.resolve(__dirname, "..", "..");
const stub = path.join(root, "src/generator/testdata/cellSceneStub.lua");

// The cell as the stub builds it, in world coordinates. A synthetic two-arm
// cell, deliberately independent of wherever the real Robot4 currently stands:
// what is under test is the frame arithmetic, not the live layout.
const CELL_TASK_SITES = [
  { alias: "Robot4_grip_support_b", size: [0.14, 0.14, 0.16], position: [2.65, -1.6, 0.08] }
];

function contextFor(name) {
  return createContext(path.join("generated", "robots", name, "manifest.json"));
}

// Run the robot's own driver against the stub scene and read back what it saw.
function inventoryAs(robot, { baseOffset, baseOrientation, stubBaseKey }) {
  const ctx = contextFor(robot);
  const script = prepareRuntimeDriverScript(ctx, {
    sharedGrip: sharedGripConfig(contextFor("Robot1")),
    taskSites: CELL_TASK_SITES,
    baseOffset,
    baseYaw: baseOrientation[2],
    baseOrientation
  });
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-sites-"));
  const driver = path.join(dir, "driver.lua");
  fs.writeFileSync(driver, script);
  const runner = path.join(dir, "run.lua");
  fs.writeFileSync(runner, [
    `local env = dofile(${JSON.stringify(stub)})`,
    `simBase = env.${stubBaseKey}`,
    `assert(loadfile(${JSON.stringify(driver)}))()`,
    "for _, o in ipairs(womatSceneObjectInventory().objects) do",
    "  local s = o.size or {x=0, y=0, z=0}",
    "  print(table.concat({o.alias, o.kind, o.world.x, o.world.y, o.world.z,",
    "    o.localPosition.x, o.localPosition.y, o.localPosition.z, s.x, s.y, s.z}, '|'))",
    "end"
  ].join("\n"));

  const output = execFileSync("lua", [runner], { cwd: root, encoding: "utf8" }).trim();
  fs.rmSync(dir, { recursive: true, force: true });
  return output.split("\n").filter(Boolean).map((line) => {
    const [alias, kind, wx, wy, wz, lx, ly, lz, sx, sy, sz] = line.split("|");
    return {
      alias,
      kind,
      world: { x: Number(wx), y: Number(wy), z: Number(wz) },
      local: { x: Number(lx), y: Number(ly), z: Number(lz) },
      size: { x: Number(sx), y: Number(sy), z: Number(sz) }
    };
  });
}

// The same admission test the lower Thing applies, against the arm's own cloud.
function detect(robot, placement) {
  const ctx = contextFor(robot);
  const options = createRuntimeOptions(ctx);
  const grid = WorkspaceGrid.fromCsv(ctx.paths.robotDataPoints, {
    voxelSize: options.workspaceVoxelSize,
    tolerance: options.workspaceTolerance,
    spacingFactor: options.workspaceToleranceSpacingFactor
  });
  return classifyTaskSites(inventoryAs(robot, placement), {
    admits: (point) => checkWorkspace(point, grid, options),
    payloadHalfHeight: Number(ctx.manifest.runtime?.gripper?.objectSize?.[2] || 0.045) / 2,
    fallbackSize: ctx.manifest.runtime?.gripper?.supportSize
  });
}

const ROBOT1 = { baseOffset: [0, 0, 0], baseOrientation: [0, 0, 0], stubBaseKey: "robot1" };
const ROBOT4 = { baseOffset: [2.9, -1.6, 0], baseOrientation: [0, 0, Math.PI], stubBaseKey: "robot4" };

const skip = !luaAvailable && "lua interpreter not installed";

test("the driver classifies both supports and the cube from the scene alone", { skip }, () => {
  const seen = inventoryAs("Robot4", ROBOT4);
  const kinds = Object.fromEntries(seen.map((entry) => [entry.alias, entry.kind]));
  assert.equal(kinds.Robot1_grip_support, "support");
  assert.equal(kinds.Robot4_grip_support_b, "support");
  assert.equal(kinds.Robot1_grip_cube, "payload");
  assert.equal(kinds.Robot1, "peer_robot");
  assert.equal(kinds.Robot4, "self");
  assert.equal(kinds.Robot1_wall, "fixture");
});

test("a site declared in a robot's own frame comes back in that frame", { skip }, () => {
  const site = inventoryAs("Robot4", ROBOT4).find((entry) => entry.alias === "Robot4_grip_support_b");
  // Declared at (0.25, 0, 0.08) in Robot4's manifest, placed in the cell at
  // (2.65, -1.6, 0.08) by the host, and read back through a base yawed 180
  // degrees. A sign error anywhere in that chain shows up here.
  assert.ok(Math.abs(site.local.x - 0.25) < 1e-6, `local x was ${site.local.x}`);
  assert.ok(Math.abs(site.local.y - 0) < 1e-6);
  assert.ok(Math.abs(site.local.z - 0.08) < 1e-6);
});

test("Robot4 holds the empty goal support and cannot serve the loaded one", { skip }, () => {
  const { sites } = detect("Robot4", ROBOT4);
  const goal = sites.find((site) => site.alias === "Robot4_grip_support_b");
  const loaded = sites.find((site) => site.alias === "Robot1_grip_support");

  assert.equal(goal.reachable, true, `goal support admission: ${goal.admission}`);
  assert.equal(goal.occupiedBy, null);

  // Seen and named, but three metres away: this is the belief that lets Robot4
  // answer "not mine" about A truthfully rather than by not knowing it exists.
  assert.ok(loaded, "the loaded support is still reported");
  assert.equal(loaded.reachable, false);
  assert.equal(loaded.occupiedBy, "Robot1_grip_cube");
});

test("Robot1 holds the loaded support and cannot serve the goal", { skip }, () => {
  const { sites } = detect("Robot1", ROBOT1);
  const goal = sites.find((site) => site.alias === "Robot4_grip_support_b");
  const loaded = sites.find((site) => site.alias === "Robot1_grip_support");

  assert.equal(loaded.reachable, true, `loaded support admission: ${loaded.admission}`);
  assert.equal(loaded.occupiedBy, "Robot1_grip_cube");
  assert.equal(goal.reachable, false);
  assert.equal(goal.occupiedBy, null);
});

test("exactly one arm is at A and exactly one at B", { skip }, () => {
  const cell = { Robot1: detect("Robot1", ROBOT1), Robot4: detect("Robot4", ROBOT4) };
  const sources = [];
  const sinks = [];
  for (const [robot, { sites }] of Object.entries(cell)) {
    for (const site of sites) {
      if (!site.reachable) continue;
      (site.occupiedBy ? sources : sinks).push(`${robot}:${site.alias}`);
    }
  }
  assert.deepEqual(sources, ["Robot1:Robot1_grip_support"]);
  assert.deepEqual(sinks, ["Robot4:Robot4_grip_support_b"]);
});

test("moving the cube moves who believes they are at A", { skip }, () => {
  // Nothing regenerated, nothing reconfigured: the cube is simply standing on
  // the other support now, and the beliefs follow.
  const inventory = inventoryAs("Robot4", ROBOT4).map((entry) => (entry.alias === "Robot1_grip_cube"
    ? { ...entry, world: { x: 2.65, y: -1.6, z: 0.185 }, local: { x: 0.25, y: 0, z: 0.185 } }
    : entry));
  const ctx = contextFor("Robot4");
  const options = createRuntimeOptions(ctx);
  const grid = WorkspaceGrid.fromCsv(ctx.paths.robotDataPoints, {
    voxelSize: options.workspaceVoxelSize,
    tolerance: options.workspaceTolerance,
    spacingFactor: options.workspaceToleranceSpacingFactor
  });
  const { sites } = classifyTaskSites(inventory, {
    admits: (point) => checkWorkspace(point, grid, options),
    payloadHalfHeight: 0.0225,
    fallbackSize: ctx.manifest.runtime?.gripper?.supportSize
  });
  const goal = sites.find((site) => site.alias === "Robot4_grip_support_b");
  assert.equal(goal.occupiedBy, "Robot1_grip_cube");
  assert.equal(goal.reachable, true);
  // Robot4 is now the one holding A, and the support at the far end is the
  // empty one -- but not one Robot4 can serve, so it has no B of its own.
  assert.equal(sites.find((site) => site.alias === "Robot1_grip_support").occupiedBy, null);
});

test("what the real driver sees survives the property's own DataSchema", { skip }, () => {
  // The full path in one assertion: the real driver Lua reads the scene, the
  // real classifier turns it into sites, and the result is validated against
  // the schema a consumer will hold it to. Skipping this link is how detection
  // passed every unit test and still failed on every agent at startup.
  const payload = taskSitesPayload("Robot4", detect("Robot4", ROBOT4));
  const td = ensureSharedCellAffordances(
    { title: "Robot4", properties: {}, actions: {} },
    { manifest: { ports: { lowerThing: 8140 } } }
  );
  const schema = { ...td.properties.getTaskSites };
  delete schema.forms;
  delete schema.description;
  const check = new Ajv({ strict: false }).compile(schema);
  const ok = check(JSON.parse(JSON.stringify(payload)));
  assert.ok(ok, `driver output rejected by its own schema: ${JSON.stringify(check.errors)}`);
  assert.equal(payload.sites.find((site) => site.alias === "Robot4_grip_support_b").occupiedBy, "none");
});
