"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const plans = fs.readFileSync(path.resolve(__dirname, "../agt/neighborPlans.asl"), "utf8");

test("a pair that shares nothing is reported as an answer, not as a breakdown", () => {
  // The reach-sphere scan is a necessary condition, so a pair it passes can
  // still voxelise to an empty intersection -- Robot1 and Robot3 clear it with
  // 0.63m to spare and their clouds never come within one voxel of each other.
  // "Could not build the shared space with robot3", with nothing after it, reads
  // as a fault in the estimator and sent the cell looking for one. The reason
  // the estimator gave has to reach the console.
  for (const reason of ["voxel_intersection_empty", "no_geometric_overlap", "insufficient_reachable_points"]) {
    assert.match(plans, new RegExp(`^sharesNoSpace\\(${reason}\\)\\.$`, "m"),
      `${reason} is a fact about the geometry, not a failure to compute`);
  }
  assert.match(
    plans,
    /-!buildSharedSpaceWith\(Other\): interactionSpaceFailed\(Other, Reason\) & sharesNoSpace\(Reason\) <-/
  );
  // Any other reason is still a failure, and still names itself.
  assert.match(
    plans,
    /-!buildSharedSpaceWith\(Other\): interactionSpaceFailed\(Other, Reason\) <-\s*\n\s*\.print\([^\n]*Reason\)/
  );
  // The bare handler is last, and no longer claims to know why.
  const ordered = [
    "-!buildSharedSpaceWith(Other): interactionSpaceFailed(Other, Reason) & sharesNoSpace(Reason)",
    "-!buildSharedSpaceWith(Other): interactionSpaceFailed(Other, Reason) <-",
    "-!buildSharedSpaceWith(Other): true <-"
  ].map((fragment) => {
    const index = plans.indexOf(fragment);
    assert.notEqual(index, -1, `missing failure handler: ${fragment}`);
    return index;
  });
  assert.deepEqual(ordered, [...ordered].sort((left, right) => left - right),
    "Jason takes the first applicable plan, so the specific handlers must precede the catch-all");
});
