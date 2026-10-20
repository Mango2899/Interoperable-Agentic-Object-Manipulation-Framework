const assert = require("node:assert/strict");
const test = require("node:test");

const { WorkspaceGrid, segmentSamples } = require("./workspaceGrid");

function shell(count = 2000) {
  // A hollow spherical shell: reachable near radius 1, empty at the centre.
  // This is the shape a real arm's tip cloud has, and the case a bounding-box
  // test alone gets wrong.
  const points = [];
  for (let index = 0; index < count; index += 1) {
    const phi = Math.acos(1 - 2 * ((index + 0.5) / count));
    const theta = Math.PI * (1 + Math.sqrt(5)) * index;
    points.push({
      x: Math.sin(phi) * Math.cos(theta),
      y: Math.sin(phi) * Math.sin(theta),
      z: Math.cos(phi)
    });
  }
  return points;
}

test("admission accepts reachable points and rejects the hollow centre", () => {
  const grid = new WorkspaceGrid(shell(), { voxelSize: 0.1 });
  assert.equal(grid.admits({ x: 1, y: 0, z: 0 }, 0.08).ok, true);
  const inside = grid.admits({ x: 0, y: 0, z: 0 }, 0.08);
  assert.equal(inside.ok, false);
  assert.equal(inside.reason, "target_not_near_workspace_samples");
  const outside = grid.admits({ x: 4, y: 0, z: 0 }, 0.08);
  assert.equal(outside.ok, false);
  assert.equal(outside.reason, "target_outside_workspace_bounds");
});

test("voxel lookup reports the same nearest distance a full scan would", () => {
  const points = shell(500);
  const grid = new WorkspaceGrid(points, { voxelSize: 0.1 });
  const probe = { x: 0.62, y: 0.55, z: 0.53 };
  const scanned = points.reduce(
    (best, point) => Math.min(best, Math.hypot(probe.x - point.x, probe.y - point.y, probe.z - point.z)),
    Infinity
  );
  const looked = grid.nearestWithin(probe, 0.5);
  assert.ok(Math.abs(looked - scanned) < 1e-12, `${looked} vs ${scanned}`);
  // Nothing is reported once the sample falls outside the query radius.
  assert.equal(grid.nearestWithin(probe, scanned / 2), null);
});

test("obstacle boxes reject a target and pinpoint where a path enters one", () => {
  const grid = new WorkspaceGrid(shell(), { voxelSize: 0.1 });
  grid.setObstacleBoxes([
    { alias: "wall", min: { x: -0.2, y: -1.5, z: -1.5 }, max: { x: 0.2, y: 1.5, z: 1.5 } }
  ]);
  const blocked = grid.admits({ x: 0, y: 0.98, z: 0 }, 0.08);
  assert.equal(blocked.ok, false);
  assert.equal(blocked.reason, "target_inside_obstacle");
  assert.equal(blocked.obstacle, "wall");

  // A straight line between two reachable points crosses the hollow centre,
  // so it is refused for leaving the workspace before it ever reaches a wall.
  const throughCentre = grid.admitsPath(segmentSamples({ x: -1, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }, 0.02), 0.08);
  assert.equal(throughCentre.ok, false);
  assert.equal(throughCentre.reason, "target_not_near_workspace_samples");

  // Arcing over the top stays reachable throughout and is refused by the wall.
  const overTheTop = [];
  for (let step = 0; step <= 60; step += 1) {
    const angle = Math.PI * (step / 60);
    overTheTop.push({ x: Math.cos(angle), y: 0, z: Math.sin(angle) });
  }
  const arcing = grid.admitsPath(overTheTop, 0.08);
  assert.equal(arcing.ok, false);
  assert.equal(arcing.reason, "target_inside_obstacle");
  assert.equal(arcing.obstacle, "wall");
  assert.ok(Math.abs(arcing.blockedPoint.x) <= 0.2 + 1e-9);
});

test("a path staying on the reachable shell is admitted end to end", () => {
  const grid = new WorkspaceGrid(shell(), { voxelSize: 0.1 });
  grid.setObstacleBoxes([]);
  const arc = [];
  for (let step = 0; step <= 40; step += 1) {
    const angle = (Math.PI / 4) * (step / 40);
    arc.push({ x: Math.cos(angle), y: Math.sin(angle), z: 0 });
  }
  assert.deepEqual(grid.admitsPath(arc, 0.08), { ok: true, samples: arc.length });
});

test("segment sampling honours the requested spatial resolution", () => {
  const samples = segmentSamples({ x: 0, y: 0, z: 0 }, { x: 0.1, y: 0, z: 0 }, 0.01);
  assert.equal(samples.length, 10);
  assert.deepEqual(samples.at(-1), { x: 0.1, y: 0, z: 0 });
  // A zero-length move still produces the target itself, never an empty path.
  assert.deepEqual(segmentSamples({ x: 1, y: 1, z: 1 }, { x: 1, y: 1, z: 1 }, 0.01), [{ x: 1, y: 1, z: 1 }]);
});

test("an empty cloud is refused rather than admitting everything", () => {
  assert.throws(() => new WorkspaceGrid([]), /at least one reachable sample/);
});

// A short reachable segment along +x, used to check what a straight-line path
// is allowed to do relative to it.
const segment = () => [0, 0.1, 0.2, 0.3, 0.4].map((x) => ({ x, y: 0, z: 0 }));

test("an arm standing outside its own sampled cloud can still move back into it", () => {
  // Random joint sampling misses full extension -- a singular pose at the very
  // edge of the reachable set -- so a robot whose rest pose is fully extended
  // holds a pose its own cloud does not cover. Judging the sample it already
  // occupies refuses every motion it could ever make, including this one.
  const grid = new WorkspaceGrid(segment(), { voxelSize: 0.1 });
  const inward = grid.admitsPath([
    { x: -0.5, y: 0, z: 0 },   // where the arm stands: outside the cloud
    { x: -0.25, y: 0, z: 0 },  // still outside, on its way in
    { x: 0.1, y: 0, z: 0 },
    { x: 0.3, y: 0, z: 0 }
  ], 0.05);
  assert.equal(inward.ok, true, "a path that reaches the workspace is admitted");
});

test("leaving the workspace once inside it is still refused", () => {
  const grid = new WorkspaceGrid(segment(), { voxelSize: 0.1 });
  const outward = grid.admitsPath([
    { x: 0.1, y: 0, z: 0 },
    { x: 0.3, y: 0, z: 0 },
    { x: 2.5, y: 0, z: 0 }
  ], 0.05);
  assert.equal(outward.ok, false, "a path that leaves the reachable set is refused");
  assert.equal(outward.blockedAtSample, 3);
});

test("a path that never reaches the workspace at all is refused", () => {
  const grid = new WorkspaceGrid(segment(), { voxelSize: 0.1 });
  const nowhere = grid.admitsPath([
    { x: -3, y: 0, z: 0 },
    { x: -2.5, y: 0, z: 0 },
    { x: -2, y: 0, z: 0 }
  ], 0.05);
  assert.equal(nowhere.ok, false);
  assert.equal(nowhere.blockedAtSample, 1, "reported at the first sample, as before");
});
