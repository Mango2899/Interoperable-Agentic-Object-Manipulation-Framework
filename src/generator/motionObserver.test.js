"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const { movementReport } = require("./motionObserver");

function samples(root, rows) {
  const file = path.join(root, "observed.jsonl");
  fs.writeFileSync(file, rows.map((row) => JSON.stringify(row)).join("\n"));
  return file;
}

test("an arm is moving only when its joints change", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-observe-"));
  // A motion that spends its whole window in preflight and is then refused, or
  // a move to the pose the arm is already standing in, occupies exactly the
  // same window in the motion log as a real sweep. Only the joints tell them
  // apart, and overstating concurrency is the one error that matters here.
  const file = samples(root, [
    { t: 1000, robot: "A", joints: [0, 0] },
    { t: 1100, robot: "A", joints: [0, 0] },
    { t: 1200, robot: "A", joints: [0, 0] }
  ]);
  const report = movementReport(file);
  assert.equal(report.anyMovingMs, 0);
  assert.equal(report.perRobot[0].travelDeg, 0);
  fs.rmSync(root, { recursive: true, force: true });
});

test("two arms turning in the same window is what concurrency means", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-observe-2-"));
  const file = samples(root, [
    { t: 1000, robot: "A", joints: [0] }, { t: 1000, robot: "B", joints: [0] },
    { t: 1100, robot: "A", joints: [5] }, { t: 1100, robot: "B", joints: [0] },
    { t: 1200, robot: "A", joints: [10] }, { t: 1200, robot: "B", joints: [5] },
    { t: 1300, robot: "A", joints: [10] }, { t: 1300, robot: "B", joints: [10] }
  ]);
  const report = movementReport(file);
  assert.equal(report.peakMovingArms, 2);
  assert.equal(report.concurrentMs, 100, "only the window where both were turning");
  assert.equal(report.anyMovingMs, 300);
  fs.rmSync(root, { recursive: true, force: true });
});

test("servo jitter below the threshold is holding position, not moving", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-observe-3-"));
  const file = samples(root, [
    { t: 1000, robot: "A", joints: [0] },
    { t: 1100, robot: "A", joints: [0.01] },
    { t: 1200, robot: "A", joints: [0.02] }
  ]);
  assert.equal(movementReport(file, { thresholdDeg: 0.05 }).anyMovingMs, 0);
  assert.ok(movementReport(file, { thresholdDeg: 0.001 }).anyMovingMs > 0);
  fs.rmSync(root, { recursive: true, force: true });
});
