"use strict";

const assert = require("node:assert/strict");
const test = require("node:test");

const { SharedRegion, claimFor, framePolyline, segmentPoints } = require("./claims");

// A single shared voxel column around the origin, 10 cm voxels.
function region(pairKey, robots, keys) {
  return new SharedRegion({
    pairKey,
    robots,
    voxelSize: 0.1,
    origin: { x: 0, y: 0, z: 0 },
    voxelKeys: new Set(keys)
  });
}

function frameAt(points) {
  return points.flatMap((point) => [point.x, point.y, point.z]);
}

test("a motion nowhere near the shared space claims nothing", () => {
  const shared = region("A__B", ["A", "B"], ["50,50,50"]);
  // An arm sweeping near its own base, metres from the shared column.
  const claim = claimFor("A", [shared], [frameAt([
    { x: 0, y: 0, z: 0 }, { x: 0.2, y: 0, z: 0.3 }
  ])], { bodyRadius: 0.05 });
  assert.equal(claim.contested, false);
  assert.deepEqual(claim.voxels, []);
});

test("a motion through the shared space claims the voxels it passes", () => {
  const shared = region("A__B", ["A", "B"], ["5,5,5", "6,5,5", "7,5,5"]);
  const claim = claimFor("A", [shared], [frameAt([
    { x: 0.55, y: 0.55, z: 0.55 }, { x: 0.75, y: 0.55, z: 0.55 }
  ])], { bodyRadius: 0.01 });
  assert.equal(claim.contested, true);
  assert.deepEqual(claim.regions, ["A__B"]);
  assert.ok(claim.voxels.includes("A__B:5,5,5"));
  assert.ok(claim.voxels.includes("A__B:7,5,5"));
});

test("the claim follows the links, not only the tool", () => {
  // The elbow bows out well clear of the line the tool travels. A claim built
  // from the tip alone would miss the space the arm actually sweeps, and let
  // another robot reserve it.
  const shared = region("A__B", ["A", "B"], ["0,5,0"]);
  const base = { x: 0.05, y: 0.05, z: 0.05 };
  const elbow = { x: 0.05, y: 0.55, z: 0.05 };
  const tip = { x: 0.05, y: 0.05, z: 0.35 };

  const wholeArm = claimFor("A", [shared], [frameAt([base, elbow, tip])], { bodyRadius: 0.01 });
  const tipOnly = claimFor("A", [shared], [frameAt([tip])], { bodyRadius: 0.01 });
  assert.equal(wholeArm.contested, true, "the elbow reaches the shared voxel");
  assert.equal(tipOnly.contested, false, "the tool never does");
});

test("a robot only claims in regions it belongs to", () => {
  const mine = region("A__B", ["A", "B"], ["5,5,5"]);
  const theirs = region("B__C", ["B", "C"], ["5,5,5"]);
  const claim = claimFor("A", [mine, theirs], [frameAt([
    { x: 0.55, y: 0.55, z: 0.55 }, { x: 0.56, y: 0.55, z: 0.55 }
  ])], { bodyRadius: 0.01 });
  assert.deepEqual(claim.regions, ["A__B"]);
  assert.ok(claim.voxels.every((voxel) => voxel.startsWith("A__B:")));
});

test("a claim is an over-approximation, because the other direction is a collision", () => {
  // A 7x7x7 block of shared voxels, so the body radius has room to show.
  const keys = [];
  for (let x = 2; x <= 8; x += 1) {
    for (let y = 2; y <= 8; y += 1) {
      for (let z = 2; z <= 8; z += 1) keys.push(`${x},${y},${z}`);
    }
  }
  const shared = region("A__B", ["A", "B"], keys);
  const point = frameAt([{ x: 0.55, y: 0.55, z: 0.55 }, { x: 0.55, y: 0.55, z: 0.56 }]);
  const thin = claimFor("A", [shared], [point], { bodyRadius: 0.01 });
  const thick = claimFor("A", [shared], [point], { bodyRadius: 0.25 });
  assert.ok(thick.voxels.length > thin.voxels.length, "a wider body claims more space");
  assert.ok(thick.voxels.length <= keys.length, "and never more than the region holds");
  // Even the thinnest body claims more than the voxel it is standing in: a link
  // near a boundary is about to cross it.
  assert.ok(thin.voxels.length > 1);
});

test("frames are read as polylines through the joints", () => {
  assert.deepEqual(framePolyline([1, 2, 3, 4, 5, 6]), [
    { x: 1, y: 2, z: 3 }, { x: 4, y: 5, z: 6 }
  ]);
  assert.equal(segmentPoints({ x: 0, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }, 0.25).length, 5);
});

const { conservativeClaim } = require("./claims");

// Brute-force reference for the claimed volume, written the slow obvious way,
// so the indexed and cached lookup cannot silently shrink it at grid boundaries
// or when cached cells are revisited. A voxel is claimed when it is within the
// body radius of the sample's OWN voxel, which is where the one-voxel dilation
// comes from.
function referenceTouching(shared, point, radius) {
  const reach = Math.floor(radius / shared.voxelSize) + 1;
  const base = ["x", "y", "z"].map((axis) => Math.floor((point[axis] - shared.origin[axis]) / shared.voxelSize));
  const gap = (offset) => Math.max(Math.abs(offset) - 1, 0) * shared.voxelSize;
  const hits = [];
  for (let x = -reach; x <= reach; x++) {
    for (let y = -reach; y <= reach; y++) {
      for (let z = -reach; z <= reach; z++) {
        const key = `${base[0] + x},${base[1] + y},${base[2] + z}`;
        if (gap(x) ** 2 + gap(y) ** 2 + gap(z) ** 2 > radius * radius) continue;
        if (shared.voxelKeys.has(key)) hits.push(key);
      }
    }
  }
  return hits;
}

// A claim may never be smaller than the voxels genuinely within the body radius
// of the sample point itself. Losing that is a collision, not a missed
// optimisation, so it is asserted separately from the reference comparison.
function pointSphere(shared, point, radius) {
  const reach = Math.ceil(radius / shared.voxelSize) + 1;
  const base = ["x", "y", "z"].map((axis) => Math.floor((point[axis] - shared.origin[axis]) / shared.voxelSize));
  const hits = [];
  for (let x = -reach; x <= reach; x++) {
    for (let y = -reach; y <= reach; y++) {
      for (let z = -reach; z <= reach; z++) {
        const index = [base[0] + x, base[1] + y, base[2] + z];
        const key = index.join(",");
        if (!shared.voxelKeys.has(key)) continue;
        const distanceSquared = ["x", "y", "z"].reduce((total, axis, position) => {
          const low = shared.origin[axis] + index[position] * shared.voxelSize;
          const away = Math.max(low - point[axis], 0, point[axis] - (low + shared.voxelSize));
          return total + away * away;
        }, 0);
        if (distanceSquared <= radius * radius) hits.push(key);
      }
    }
  }
  return hits;
}

test("indexed and cached claims match the reference volume across sparse grids and boundaries", () => {
  let seed = 17;
  const random = () => ((seed = (1664525 * seed + 1013904223) >>> 0) / 2 ** 32);
  const keys = new Set();
  for (let i = 0; i < 180; i++) keys.add(Array.from({ length: 3 }, () => Math.floor(random() * 21) - 10).join(","));
  for (const voxelSize of [0.1, 0.0489]) {
    const shared = new SharedRegion({ pairKey: "A__B", robots: ["A", "B"], voxelSize,
      origin: { x: -0.37, y: 0.13, z: -0.21 }, voxelKeys: keys });
    const frames = [];
    for (let i = 0; i < 150; i++) {
      const point = Object.fromEntries(["x", "y", "z"].map((axis) => [axis,
        shared.origin[axis] + (Math.floor(random() * 35) - 17) * voxelSize
          + [0, 1e-12, -1e-12, voxelSize * 0.5][i % 4]]));
      for (const radius of [0, 0.01, 0.12, 0.18]) {
        const expected = referenceTouching(shared, point, radius);
        const actual = shared.touching(point, radius);
        assert.deepEqual(actual, expected);
        assert.deepEqual(shared.touching(point, radius), expected, "revisited cells keep every voxel");
        const reachable = new Set(actual);
        for (const key of pointSphere(shared, point, radius)) {
          assert.ok(reachable.has(key), `claim dropped ${key}, which is within ${radius} m of the sample`);
        }
      }
      if (i < 12) frames.push(frameAt([point, { ...point, x: point.x + 0.15 }]));
    }
    const reference = new SharedRegion({ pairKey: shared.pairKey, robots: shared.robots,
      voxelSize, origin: shared.origin, voxelKeys: keys });
    reference.touching = (point, radius) => referenceTouching(reference, point, radius);
    assert.deepEqual(claimFor("A", [shared], frames), claimFor("A", [reference], frames));
  }
});

test("a motion that cannot enter any shared region claims nothing and waits for nobody", () => {
  const shared = region("A__B", ["A", "B"], ["50,50,50"]);
  const claim = conservativeClaim("A", [shared], [frameAt([
    { x: 0, y: 0, z: 0 }, { x: 0.2, y: 0, z: 0.3 }
  ])], { bodyRadius: 0.05, target: { x: 0.2, y: 0, z: 0.3 }, reachMargin: 0.35 });
  assert.equal(claim.contested, false);
  assert.deepEqual(claim.voxels, []);
});

test("a motion that could enter a region claims that region whole", () => {
  // Arbitration happens before planning, so neither the path nor where the
  // elbow goes on the way is known. Claiming the region whole cannot
  // under-claim; claiming a guessed swept volume could.
  const keys = [];
  for (let x = 4; x <= 6; x += 1) keys.push(`${x},5,5`);
  const shared = region("A__B", ["A", "B"], keys);
  const claim = conservativeClaim("A", [shared], [frameAt([
    { x: 0.05, y: 0.05, z: 0.05 }, { x: 0.06, y: 0.05, z: 0.05 }
  ])], { bodyRadius: 0.01, target: { x: 0.55, y: 0.55, z: 0.55 }, reachMargin: 0.35 });
  assert.equal(claim.contested, true);
  assert.equal(claim.granularity, "region");
  assert.equal(claim.voxels.length, keys.length, "the whole region, not a guess at part of it");
});

test("a target beyond reach of a region does not claim it", () => {
  const shared = region("A__B", ["A", "B"], ["5,5,5"]);
  const far = conservativeClaim("A", [shared], [frameAt([{ x: 0, y: 0, z: 0 }, { x: 0.01, y: 0, z: 0 }])], {
    bodyRadius: 0.01, target: { x: 5, y: 5, z: 5 }, reachMargin: 0.35
  });
  assert.equal(far.contested, false);
});

test("live occupancy follows complete link capsules, including interior and tangential contact", () => {
  const { occupancyFor, segmentBoxDistanceSquared } = require("./claims");
  const min = { x: 0, y: 0, z: 0 }, max = { x: 1, y: 1, z: 1 };
  for (const [from, to, expected] of [
    [[-1, 0.5, 0.5], [2, 0.5, 0.5], 0],
    [[-1, 2, 0.5], [2, 2, 0.5], 1],
    [[2, 2, 2], [2, 2, 2], 3],
    [[-1, 1, 0.5], [1, -1, 0.5], 0],
    [[-1, 2, 0.5], [2, -1, 0.5], 0],
    [[-1, 3, 0.5], [3, -1, 0.5], 0],
    [[-1, 4, 0.5], [4, -1, 0.5], 0.5]
  ]) {
    const [a, b] = [from, to].map((p) => ({ x: p[0], y: p[1], z: p[2] }));
    assert.ok(Math.abs(segmentBoxDistanceSquared(a, b, min, max) - expected) < 1e-10);
  }
  const shared = region("A__B", ["A", "B"], ["5,5,5"]);
  assert.equal(occupancyFor("A", [shared], [[0, 0.55, 0.55, 1, 0.55, 0.55]], { bodyRadius: 0 }).contested, true);
  assert.equal(occupancyFor("A", [shared], [[0, 0.4, 0.55, 1, 0.4, 0.55]], { bodyRadius: 0.1 }).contested, true);
  assert.equal(occupancyFor("A", [shared], [[0, 0.39, 0.55, 1, 0.39, 0.55]], { bodyRadius: 0.1 }).contested, false);
  assert.equal(occupancyFor("C", [shared], [[0, 0.55, 0.55, 1, 0.55, 0.55]], { bodyRadius: 0 }).contested, false);
});

test("capsule distance agrees with independently sampled segments", () => {
  const { segmentBoxDistanceSquared } = require("./claims");
  let seed = 31415;
  const random = () => ((seed = (1664525 * seed + 1013904223) >>> 0) / 2 ** 32);
  const axes = ["x", "y", "z"];
  const point = () => Object.fromEntries(axes.map((axis) => [axis, random() * 4 - 2]));
  for (let i = 0; i < 100; i++) {
    const from = point(), to = point(), min = point();
    const max = Object.fromEntries(axes.map((axis) => [axis, min[axis] + random() + 0.01]));
    let sampled = Infinity;
    for (let j = 0; j <= 2000; j++) {
      const squared = axes.reduce((sum, axis) => {
        const value = from[axis] + (to[axis] - from[axis]) * j / 2000;
        return sum + Math.max(min[axis] - value, 0, value - max[axis]) ** 2;
      }, 0);
      sampled = Math.min(sampled, Math.sqrt(squared));
    }
    const actual = Math.sqrt(segmentBoxDistanceSquared(from, to, min, max));
    const spacing = Math.hypot(...axes.map((axis) => to[axis] - from[axis])) / 2000;
    assert.ok(actual <= sampled + 1e-10);
    assert.ok(actual >= sampled - spacing - 1e-10);
  }
});
