"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const { overlapReport } = require("./workspaceOverlap");
const { REPO_ROOT } = require("./core/context");

function cell(root, robots) {
  for (const [name, shift] of Object.entries(robots)) {
    const points = [];
    for (let x = 0; x < 8; x += 1) {
      for (let y = 0; y < 8; y += 1) {
        for (let z = 0; z < 8; z += 1) points.push(`${x * 0.1 + shift},${y * 0.1},${z * 0.1},1`);
      }
    }
    fs.writeFileSync(path.join(root, `${name}.csv`), ["X,Y,Z,Reachable", ...points].join("\n"));
  }
  const descriptor = path.join(root, "cell.json");
  fs.writeFileSync(descriptor, JSON.stringify({
    robots: Object.keys(robots).map((name) => ({
      name,
      workspaceCsv: path.relative(REPO_ROOT, path.join(root, `${name}.csv`)),
      offset: [0, 0, 0],
      rotationDeg: 0
    }))
  }));
  return descriptor;
}

test("the contested fraction is what a motion lock actually protects", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-overlap-"));
  // Two arms half a workspace apart, one well clear of both.
  const report = overlapReport(cell(root, { A: 0, B: 0.4, C: 9 }), { voxelSize: 0.1 });
  const of = (name) => report.robots.find((robot) => robot.robot === name);

  assert.ok(of("A").contestedFraction > 0 && of("A").contestedFraction < 1);
  assert.equal(of("C").contestedVoxels, 0, "an arm nobody can reach contests nothing");
  const ac = report.pairs.find((pair) => pair.pairKey === "A__C");
  assert.equal(ac.sharedVoxels, 0);
  assert.equal(ac.boundingBoxOverlapVolume, 0);
  fs.rmSync(root, { recursive: true, force: true });
});

test("every robot is measured on the same grid, or the fractions mean nothing", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-overlap-grid-"));
  const report = overlapReport(cell(root, { A: 0, B: 0.4 }), { voxelSize: 0.1 });
  // A shared voxel is one cell of space, so it counts once for each side and
  // the pair's own count is that same number -- not two different ones.
  const ab = report.pairs.find((pair) => pair.pairKey === "A__B");
  for (const name of ["A", "B"]) {
    assert.equal(report.robots.find((robot) => robot.robot === name).contestedVoxels, ab.sharedVoxels);
  }
  fs.rmSync(root, { recursive: true, force: true });
});
