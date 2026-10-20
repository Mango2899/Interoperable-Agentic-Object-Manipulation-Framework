"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const { ComputeLog, computeReport } = require("./computeLog");
const { paletteColor } = require("./interactionSpace");

function logFile(root) {
  return path.join(root, "compute.jsonl");
}

test("the log shows who did the work and who was served from cache", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-compute-"));
  const file = logFile(root);
  new ComputeLog({ agent: "Robot3", file }).record({ pairKey: "Robot3__Robot4", cached: false, voxelCount: 392, durationMs: 1198 });
  new ComputeLog({ agent: "Robot4", file }).record({ pairKey: "Robot3__Robot4", cached: true, voxelCount: 392, durationMs: 14 });
  const report = computeReport(file);
  const pair = report.pairs[0];
  assert.deepEqual(pair.computedBy, ["Robot3"]);
  assert.deepEqual(pair.servedFromCache, ["Robot4"]);
  // Two agents asked, one did the work: the cache is what keeps the second ask
  // from being a second voxelisation.
  assert.equal(pair.attempts, 2);
  assert.equal(pair.duplicated, false);
  assert.equal(report.duplicatedPairs.length, 0);
  fs.rmSync(root, { recursive: true, force: true });
});

test("two agents both doing the work is reported as duplication", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-compute-dup-"));
  const file = logFile(root);
  for (const agent of ["Robot1", "Robot2"]) {
    new ComputeLog({ agent, file }).record({ pairKey: "Robot1__Robot2", cached: false, voxelCount: 343, durationMs: 900 });
  }
  const report = computeReport(file);
  assert.equal(report.pairs[0].duplicated, true);
  assert.deepEqual(report.duplicatedPairs, ["Robot1__Robot2"]);
  assert.equal(report.totalWorkMs, 1800);
  fs.rmSync(root, { recursive: true, force: true });
});

test("a refusal carries the numbers behind it, on the console and in the file", () => {
  // A pair that shares nothing writes no space file, so the log line is the
  // only record it ever leaves. A bare reason atom cannot tell "these two arms
  // genuinely never meet" apart from "something went wrong in here", and that
  // ambiguity sent the cell hunting for a bug in the estimator when the answer
  // was that Robot1 and Robot3 are 0.029m apart at their closest.
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-compute-why-"));
  const file = logFile(root);
  const printed = [];
  const original = console.log;
  console.log = (line) => printed.push(line);
  try {
    new ComputeLog({ agent: "Robot1", file }).record({
      pairKey: "Robot1__Robot3",
      targetRobot: "Robot3",
      durationMs: 1349,
      reason: "voxel_intersection_empty",
      diagnostics: { nearestApproach: 0.0292, voxelSize: 0.0486 }
    });
  } finally {
    console.log = original;
  }
  assert.match(printed[0], /voxel_intersection_empty/);
  assert.match(printed[0], /nearestApproach=0\.0292/);
  assert.match(printed[0], /voxelSize=0\.0486/);
  const [row] = fs.readFileSync(file, "utf8").trim().split("\n").map((line) => JSON.parse(line));
  assert.deepEqual(row.diagnostics, { nearestApproach: 0.0292, voxelSize: 0.0486 });
  fs.rmSync(root, { recursive: true, force: true });
});

test("every drawn pair gets its own colour, however many pairs there are", () => {
  // Four palette entries covered a three-robot cell with two pairs. Four robots
  // make six pairs, and two clouds in one scene drawn the same colour cannot be
  // told apart.
  const colors = new Set();
  for (let index = 0; index < 24; index += 1) {
    colors.add(JSON.stringify(paletteColor(index)));
  }
  assert.ok(colors.size >= 12, `expected distinct colours well past the palette, got ${colors.size}`);
  assert.deepEqual(paletteColor(0), [0, 255, 0], "the shipped palette still leads");
});

test("a test run never writes to the cell's real compute log", () => {
  // The suite's stub controllers report invented versions and voxel counts.
  // Once those rows are in the shared file they are indistinguishable from a
  // real run: six phantom computes of a pair no agent had touched, reported as
  // duplication that never happened.
  const { computeLogFor } = require("./computeLog");
  const log = computeLogFor({ manifest: { name: "Robot1" } });
  assert.equal(log.enabled, false, "file logging is off under node --test");
  assert.equal(log.silent, true, "and so is the console line");
  assert.equal(fs.existsSync(path.resolve(__dirname, "..", "..", "performance_log/interaction-compute.jsonl")) &&
    fs.readFileSync(path.resolve(__dirname, "..", "..", "performance_log/interaction-compute.jsonl"), "utf8")
      .includes("abc123"), false, "no stub version reaches the real log");
});
