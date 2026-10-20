"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const { basePosition, couldOverlap, radiusOf } = require("./neighborTest");

const signature = (position, reach, spacing = 0.05) => ({
  basePose: { position, orientation: [0, 0, 0] },
  reachRadius: reach,
  safeReachRadius: reach + spacing,
  spacing
});

test("two arms whose reach spheres meet are neighbours", () => {
  const verdict = couldOverlap(signature([0, 0, 0], 1.0), signature([1.5, 0, 0], 0.8));
  assert.equal(verdict.neighbor, true);
  assert.equal(verdict.reason, "reach_spheres_overlap");
  assert.equal(verdict.distance, 1.5);
  // 1.05 + 0.85, both widened by their own sample spacing.
  assert.ok(Math.abs(verdict.combinedReach - 1.9) < 1e-9);
  assert.ok(verdict.margin > 0);
});

test("two arms too far apart to touch are not", () => {
  const verdict = couldOverlap(signature([0, 0, 0], 1.0), signature([5, 0, 0], 0.8));
  assert.equal(verdict.neighbor, false);
  assert.equal(verdict.reason, "reach_spheres_disjoint");
  assert.ok(verdict.margin < 0);
});

test("the radius tested is widened by the sample spacing", () => {
  // reachRadius is the extreme of a finite Monte-Carlo sample, so the farthest
  // reachable point is almost never one that was drawn. Understating it prunes
  // a pair that really can collide, which is the one error that matters.
  assert.equal(radiusOf({ reachRadius: 1.0, spacing: 0.05 }), 1.05);
  assert.equal(radiusOf({ reachRadius: 1.0, safeReachRadius: 1.05 }), 1.05);

  const bare = { basePose: { position: [0, 0, 0] }, reachRadius: 1.0, spacing: 0.05 };
  const far = { basePose: { position: [2.06, 0, 0] }, reachRadius: 1.0, spacing: 0.05 };
  // 2.06 exceeds 1.0 + 1.0 but not 1.05 + 1.05: the widening is what keeps the
  // pair in play.
  assert.equal(couldOverlap(bare, far).neighbor, true);
});

test("a robot that will not say where it is cannot be ruled out", () => {
  const verdict = couldOverlap(signature([0, 0, 0], 1.0), null);
  assert.equal(verdict.neighbor, true);
  assert.equal(verdict.reason, "signature_incomplete");
  assert.equal(basePosition(null), null);
});

test("the test is invariant to how a robot is mounted", () => {
  // A sphere around the base does not change when the arm is pitched. Bounds do
  // -- Robot3 is mounted at -90 degrees and its local z-extent lands somewhere
  // else entirely in world -- which is why the test is radial and not boxed.
  const upright = { basePose: { position: [0, 0, 0], orientation: [0, 0, 0] }, reachRadius: 1.0, spacing: 0 };
  const pitched = { basePose: { position: [0, 0, 0], orientation: [0, -Math.PI / 2, Math.PI] }, reachRadius: 1.0, spacing: 0 };
  const other = signature([1.5, 0, 0], 0.8);
  assert.deepEqual(couldOverlap(upright, other).neighbor, couldOverlap(pitched, other).neighbor);
});

test("the cell's real robots all qualify, and the ranking is honest about why", () => {
  const root = path.resolve(__dirname, "..", "..");
  const descriptorPath = path.join(root, "generated/robots/Robot1_Robot2_Robot3.shared.json");
  if (!fs.existsSync(descriptorPath)) return;
  const descriptor = JSON.parse(fs.readFileSync(descriptorPath, "utf8"));
  const { loadReachableCsv } = require("./interactionSpace");
  const signatures = descriptor.robots.map((robot) => {
    const points = loadReachableCsv(path.join(root, robot.workspaceCsv));
    const reach = Math.max(...points.map((point) => Math.hypot(point.x, point.y, point.z)));
    return { name: robot.name, basePose: { position: robot.offset }, reachRadius: reach, spacing: 0.05 };
  });
  // Passing this test is not a promise that a pair shares anything. The sphere
  // test is a necessary condition and nothing more, and Robot1/Robot3 are the
  // pair that shows the difference: they clear it with 0.63m to spare and their
  // sampled workspaces still come no closer than 0.029m, which is less than one
  // voxel, so their interaction space is empty. Pruning them here would be
  // wrong for a different reason -- the sphere test cannot know that, and a
  // cell that guessed instead of measuring is what this test guards against.
  for (let first = 0; first < signatures.length; first += 1) {
    for (let second = first + 1; second < signatures.length; second += 1) {
      const verdict = couldOverlap(signatures[first], signatures[second]);
      assert.equal(verdict.neighbor, true,
        `${signatures[first].name}/${signatures[second].name} must not be pruned`);
    }
  }
});
