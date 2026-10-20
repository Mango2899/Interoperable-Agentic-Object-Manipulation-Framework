"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");

const {
  CONFIGS,
  OUTCOMES,
  classifyOutcome,
  configOrder,
  floorHeight,
  sampleTargets,
  stageSwitches,
  summaryRows,
  thesisFigure
} = require("./e3SafetyPipeline");

const bounds = { min: { x: -0.5, y: -0.4, z: -0.3 }, max: { x: 0.5, y: 0.4, z: 0.8 } };
// A regular cloud over the bounds, part of it below the floor as a sampler
// that ignores the floor produces.
const points = [];
for (let x = -0.5; x <= 0.5; x += 0.1) {
  for (let y = -0.4; y <= 0.4; y += 0.1) {
    for (let z = -0.3; z <= 0.8; z += 0.1) points.push({ x, y, z });
  }
}
const floor = { alias: "Floor", min: { x: -5, y: -5, z: -1 }, max: { x: 5, y: 5, z: 0 } };
const wall = { alias: "Robot2_wall", min: { x: 0.45, y: -0.4, z: 0 }, max: { x: 0.5, y: 0.4, z: 0.8 } };
const cell = { points, bounds, obstacleBoxes: [floor, wall] };
const options = { margin: 0.2, jitter: 0.05, nearObstacle: 0.15 };

test("targets are reproducible and come in the three kinds", () => {
  const a = sampleTargets(cell, 60, options, 7);
  assert.deepEqual(a, sampleTargets(cell, 60, options, 7));
  assert.notDeepEqual(a, sampleTargets(cell, 60, options, 8));
  const count = (kind) => a.filter((target) => target.kind === kind).length;
  assert.deepEqual([count("near"), count("obstacle"), count("box")], [30, 15, 15]);
  assert.deepEqual(a.map((target) => target.index), a.map((_, index) => index + 1));
  // Interleaved, so a run cut short still has every kind.
  assert.ok(new Set(a.slice(0, 20).map((target) => target.kind)).size === 3);
});

test("no target lies below the floor, and each kind is where it should be", () => {
  const targets = sampleTargets(cell, 200, options, 3);
  assert.equal(floorHeight(cell.obstacleBoxes), 0);
  for (const target of targets) {
    assert.ok(target.z >= 0, `target ${target.index} at z=${target.z}`);
    const nearest = Math.min(...points.map((p) => Math.hypot(p.x - target.x, p.y - target.y, p.z - target.z)));
    if (target.kind === "near") assert.ok(nearest <= options.jitter + 1e-9);
    if (target.kind === "obstacle") assert.ok(target.x >= 0.45 - options.nearObstacle - options.jitter);
    if (target.kind === "box") {
      assert.ok(target.x >= bounds.min.x - 0.2 && target.x <= bounds.max.x + 0.2);
      assert.ok(target.z <= bounds.max.z + 0.2);
    }
  }
});

test("without an obstacle nearby, its share goes to targets near the workspace", () => {
  const targets = sampleTargets({ ...cell, obstacleBoxes: [floor] }, 60, options, 1);
  assert.equal(targets.filter((target) => target.kind === "obstacle").length, 0);
  assert.equal(targets.filter((target) => target.kind === "near").length, 45);
  // With no Floor box, the floor is the base's height above the world floor.
  assert.equal(floorHeight([], 0.1), -0.1);
});

test("every target runs every configuration once, in rotated order", () => {
  for (let index = 1; index <= 7; index += 1) {
    assert.deepEqual([...configOrder(index)].sort(), [...CONFIGS].sort());
  }
  assert.equal(configOrder(1)[0], "full");
  assert.equal(configOrder(2)[0], "no_admission");
  assert.equal(configOrder(6)[0], "full");
});

test("each ablation switches off exactly one stage", () => {
  assert.deepEqual(stageSwitches("full"), { admission: true, preflight: true, fallback: true, watchdog: true });
  for (const config of CONFIGS.slice(1)) {
    assert.equal(Object.values(stageSwitches(config)).filter((on) => !on).length, 1, config);
  }
});

test("outcomes are classified by what actually happened", () => {
  const base = { contactSteps: 0, success: false, startKind: null };
  assert.equal(classifyOutcome({ ...base, success: true, startKind: "cartesian" }), "reached_direct");
  assert.equal(classifyOutcome({ ...base, success: true, startKind: "path" }), "reached_fallback");
  assert.equal(classifyOutcome({ ...base, success: true, startKind: "path", contactSteps: 3 }), "collision");
  assert.equal(classifyOutcome({ ...base, startKind: "cartesian", reason: "motion_execution_failed",
    plannerMessage: "...: simple_cartesian_path_collision_detected" }), "aborted_watchdog");
  assert.equal(classifyOutcome({ ...base, startKind: "cartesian", reason: "cartesian_target_not_reached" }), "not_reached");
  assert.equal(classifyOutcome({ ...base, admissionOk: false, linePreflightOk: null }), "rejected_admission");
  assert.equal(classifyOutcome({ ...base, admissionOk: true, linePreflightOk: false }), "rejected_preflight");
  assert.equal(classifyOutcome({ ...base, admissionOk: true, linePreflightOk: true }), "rejected_other");
});

test("summary pools the robots and the figure lists every outcome", () => {
  const records = [];
  for (const robot of ["Robot2", "Robot4"]) {
    for (const config of CONFIGS) {
      records.push({ robot, config, outcome: "reached_direct", startKind: "cartesian", decisionMs: 40, totalMs: 3000, contactSteps: 0 });
      records.push({ robot, config, outcome: "rejected_admission", startKind: null, decisionMs: 5, totalMs: 5, contactSteps: 0 });
    }
  }
  const rows = summaryRows(records);
  const pooled = rows.find((row) => row.robot === "all" && row.config === "full");
  assert.equal(pooled.commands, 4);
  assert.equal(pooled.reached_direct, 2);
  assert.equal(pooled.decisionMsMedianMoved, 40);
  const figure = thesisFigure(rows);
  assert.equal((figure.match(/\\addplot/g) || []).length, OUTCOMES.length);
  assert.match(figure, /\(50\.0,Complete\)/);
});
