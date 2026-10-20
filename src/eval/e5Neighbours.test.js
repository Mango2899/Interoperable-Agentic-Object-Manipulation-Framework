const test = require("node:test");
const assert = require("node:assert/strict");

const { couldOverlap } = require("../generator/neighborTest");
const {
  cellPairs,
  classify,
  confusion,
  confusionTable,
  descriptorForCell,
  parseIntegerList,
  parseSpan,
  pgfplotsFigures,
  placeCloud,
  signatureAt,
  signatureOf,
  spinAboutZ,
  summarizePairs
} = require("./e5Neighbours");

// A cloud whose farthest point sits at a known radius, dense enough for the
// spacing estimate to have more than one nearest-neighbour distance to work on.
function cloudOf(radius, count = 200) {
  const points = [];
  for (let index = 0; index < count; index += 1) {
    const alpha = (index / (count - 1)) * radius;
    points.push({ x: alpha, y: 0, z: 0 });
  }
  return points;
}

function runOf(overrides) {
  return { cell: 5, seed: 1, pair: "Robot1__Robot2", neighbor: true, sharedVoxels: 0, ...overrides };
}

test("signatureOf measures the reach from the base and widens it by the spacing", () => {
  const signature = signatureOf(cloudOf(1.5));
  assert.ok(Math.abs(signature.reachRadius - 1.5) < 1e-12);
  assert.ok(signature.spacing > 0);
  assert.equal(signature.safeReachRadius, signature.reachRadius + signature.spacing);
  assert.equal(signature.sampleCount, 200);
  assert.throws(() => signatureOf([]), /at least one reachable sample/);
});

test("a signature at a base position is what the deployed test consumes", () => {
  const left = signatureAt(signatureOf(cloudOf(1)), [0, 0, 0]);
  const right = signatureAt(signatureOf(cloudOf(1)), [1.9, 0, 0]);
  const verdict = couldOverlap(left, right);
  assert.equal(verdict.neighbor, true);
  assert.ok(Math.abs(verdict.distance - 1.9) < 1e-12);
  assert.ok(verdict.margin > 0);
  // Pull them apart beyond both widened radii and the pair is rejected.
  const far = couldOverlap(left, signatureAt(signatureOf(cloudOf(1)), [2.5, 0, 0]));
  assert.equal(far.neighbor, false);
  assert.ok(far.margin < 0);
});

test("spinAboutZ turns the cloud about the world z axis and leaves the height alone", () => {
  const spun = spinAboutZ({ x: 1, y: 0, z: 0.3 }, Math.PI / 2);
  assert.ok(Math.abs(spun.x) < 1e-12);
  assert.ok(Math.abs(spun.y - 1) < 1e-12);
  assert.equal(spun.z, 0.3);
});

test("placeCloud mounts, spins and translates, in that order", () => {
  // A quarter turn of the mounting, then a quarter turn of the layout, then the
  // base offset: (1,0,0) ends up at (0,0,1)+(-1,0,0) = (-1,0,1)... in x,y only.
  const placement = { offset: [2, 0, 0], orientationDeg: [0, 0, 90] };
  const [mounted] = placeCloud([{ x: 1, y: 0, z: 0 }], placement);
  assert.ok(Math.abs(mounted.x - 2) < 1e-12);
  assert.ok(Math.abs(mounted.y - 1) < 1e-12);
  const [spun] = placeCloud([{ x: 1, y: 0, z: 0 }], placement, Math.PI / 2);
  assert.ok(Math.abs(spun.x - 1) < 1e-12);
  assert.ok(Math.abs(spun.y) < 1e-12);
});

test("classify names the four cases against the given voxel floor", () => {
  assert.equal(classify(runOf({ neighbor: true, sharedVoxels: 500 }), 1), "tp");
  assert.equal(classify(runOf({ neighbor: true, sharedVoxels: 0 }), 1), "fp");
  assert.equal(classify(runOf({ neighbor: false, sharedVoxels: 3 }), 1), "fn");
  assert.equal(classify(runOf({ neighbor: false, sharedVoxels: 0 }), 1), "tn");
  // The usable floor turns a pair that shares a voxel or two into a negative.
  assert.equal(classify(runOf({ neighbor: true, sharedVoxels: 3 }), 20), "fp");
  assert.equal(classify(runOf({ neighbor: true, sharedVoxels: 20 }), 20), "tp");
});

test("confusion counts the cases and keeps the false negatives", () => {
  const runs = [
    runOf({ neighbor: true, sharedVoxels: 500 }),
    runOf({ neighbor: true, sharedVoxels: 0 }),
    runOf({ neighbor: true, sharedVoxels: 0 }),
    runOf({ neighbor: false, sharedVoxels: 0 }),
    runOf({ neighbor: false, sharedVoxels: 4, pair: "Robot1__Robot5" })
  ];
  const result = confusion(runs, 1);
  assert.deepEqual([result.tp, result.fp, result.fn, result.tn], [1, 2, 1, 1]);
  assert.equal(result.n, 5);
  assert.ok(Math.abs(result.precision - 1 / 3) < 1e-12);
  assert.ok(Math.abs(result.recall - 0.5) < 1e-12);
  assert.ok(Math.abs(result.wastedComputations - 2 / 3) < 1e-12);
  assert.equal(result.falseNegatives.length, 1);
  assert.equal(result.falseNegatives[0].pair, "Robot1__Robot5");
});

test("a test that admits everything has perfect recall and no false negative", () => {
  const runs = [
    runOf({ neighbor: true, sharedVoxels: 100 }),
    runOf({ neighbor: true, sharedVoxels: 1 }),
    runOf({ neighbor: true, sharedVoxels: 0 })
  ];
  const result = confusion(runs, 1);
  assert.equal(result.fn, 0);
  assert.equal(result.recall, 1);
  assert.equal(result.specificity, 0);
});

test("summarizePairs reports one row per cell and pair, with the seeds aggregated", () => {
  const runs = [
    { cell: 5, seed: 1, pair: "A__B", neighbor: true, sharedVoxels: 100, distance: 1, combinedReach: 2, margin: 1, voxelSize: 0.03, voxelizeMs: 10, testUs: 0.1 },
    { cell: 5, seed: 2, pair: "A__B", neighbor: true, sharedVoxels: 300, distance: 1, combinedReach: 2, margin: 1, voxelSize: 0.03, voxelizeMs: 12, testUs: 0.1 },
    { cell: 5, seed: 1, pair: "A__C", neighbor: false, sharedVoxels: 0, distance: 3, combinedReach: 2, margin: -1, voxelSize: null, voxelizeMs: 1, testUs: 0.1 }
  ];
  const rows = summarizePairs(runs, 20);
  assert.equal(rows.length, 2);
  const [first, second] = rows;
  assert.equal(first.pair, "A__B");
  assert.equal(first.n, 2);
  assert.equal(first.neighbor, true);
  assert.equal(first.usableEverySeed, true);
  assert.equal(first.sharedVoxels_median, 200);
  assert.equal(second.pair, "A__C");
  assert.equal(second.neighbor, false);
  assert.equal(second.sharesEverySeed, false);
  assert.equal(second.falseNegativeSeeds, 0);
});

test("a seed in which a rejected pair shares space is counted as a false negative", () => {
  const rows = summarizePairs([
    { cell: 5, seed: 1, pair: "A__C", neighbor: false, sharedVoxels: 7, distance: 3, combinedReach: 2.9, margin: -0.1, voxelSize: 0.04, voxelizeMs: 5, testUs: 0.1 }
  ], 20);
  assert.equal(rows[0].falseNegativeSeeds, 1);
  assert.equal(rows[0].usableEverySeed, false);
});

test("cellPairs lists every unordered pair once, in canonical order", () => {
  const pairs = cellPairs(["Robot2", "Robot1", "Robot3"]);
  assert.deepEqual(pairs.map((pair) => pair.key), ["Robot1__Robot2", "Robot1__Robot3", "Robot2__Robot3"]);
  assert.equal(cellPairs(["Robot1", "Robot2", "Robot3", "Robot4", "Robot5"]).length, 10);
});

test("descriptorForCell names the descriptor of each cell", () => {
  assert.equal(descriptorForCell(2), "generated/robots/Robot1_Robot2.shared.json");
  assert.equal(descriptorForCell(5), "generated/robots/Robot1_Robot2_Robot3_Robot4_Robot5.shared.json");
});

test("the CLI list and span parsers reject what they cannot use", () => {
  assert.deepEqual(parseIntegerList("5,2,2,3", "Cell sizes"), [2, 3, 5]);
  assert.throws(() => parseIntegerList("2,x", "Cell sizes"), /positive integers/);
  assert.deepEqual(parseSpan("0.55,1.25"), [0.55, 1.25]);
  assert.throws(() => parseSpan("1.25,0.55"), /increasing/);
  assert.throws(() => parseSpan("0.5"), /two increasing/);
});

test("the emitted figures and table are self-contained LaTeX", () => {
  const tex = pgfplotsFigures({ withLayouts: true, axisMax: 3.6, seeds: 10, cells: [2, 3, 4, 5], minimumSharedVoxels: 20 });
  assert.match(tex, /\\providecommand\{\\EFiveData\}/);
  assert.match(tex, /e5_scatter_cells\.csv/);
  assert.match(tex, /e5_scatter_layouts\.csv/);
  assert.match(tex, /label\{fig:e5-cells\}/);
  assert.equal((tex.match(/\\begin\{figure\}/g) || []).length, 2);
  assert.equal(pgfplotsFigures({ withLayouts: false, axisMax: 3.6, seeds: 10, cells: [5], minimumSharedVoxels: 20 })
    .includes("e5_scatter_layouts.csv"), false);

  const table = confusionTable({
    rows: [{ population: "cells", scope: "all", threshold: "shares", n: 20, tp: 12, fp: 7, fn: 0, tn: 1, precision: 0.63, recall: 1, specificity: 0.125, wastedComputations: 0.37 }],
    minimumSharedVoxels: 20,
    seeds: 10
  });
  assert.match(table, /\\begin\{table\}/);
  assert.match(table, /label\{tab:exp_e5\}/);
  assert.match(table, /all cells & \$\\geq 1\$ & 20 & 12 & 7 & 0 & 1 & 0\.630 & 1\.000 & 0\.370/);
});
