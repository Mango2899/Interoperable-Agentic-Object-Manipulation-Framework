const test = require("node:test");
const assert = require("node:assert/strict");

const {
  cellBudget,
  classifyLive,
  costClasses,
  costTable,
  lineCell,
  lineCurve,
  liveRows,
  liveSaving,
  nearestNeighbourPitch,
  normalizeRun,
  parseCsv,
  scalingFigure
} = require("./e6ComputationSaved");

function runOf(overrides) {
  return normalizeRun({
    cell: 5, seed: 1, pair: "Robot1__Robot2", neighbor: 1, sharedVoxels: 100,
    reason: "", voxelizeMs: 2000, testUs: 0.1, ...overrides
  });
}

const RADII = [{ robot: "A", safeReachRadius: 0.6 }, { robot: "B", safeReachRadius: 0.55 }];

test("parseCsv undoes what toCsv wrote, including the 1/0 booleans", () => {
  const rows = parseCsv("cell,neighbor,voxelizeMs\n5,1,2000\n5,0,110\n");
  assert.equal(rows.length, 2);
  assert.equal(rows[0].neighbor, "1");
  assert.equal(normalizeRun(rows[0]).neighbor, true);
  assert.equal(normalizeRun(rows[1]).neighbor, false);
  assert.throws(() => parseCsv("cell,neighbor\n"), /no rows/);
});

test("a run is classed by the verdict and by what the pair turned out to share", () => {
  assert.equal(runOf({}).class, "admitted_shares");
  assert.equal(runOf({ sharedVoxels: 0 }).class, "admitted_empty");
  assert.equal(runOf({ neighbor: 0, sharedVoxels: 0 }).class, "rejected");
  // A rejected pair that shares space would be the false negative of E5; E6
  // still has to class it as rejected, since that is what the pipeline did.
  assert.equal(runOf({ neighbor: 0, sharedVoxels: 50 }).class, "rejected");
});

test("the cost classes keep the three prices apart", () => {
  const classes = costClasses([
    runOf({ voxelizeMs: 1900 }),
    runOf({ voxelizeMs: 2000 }),
    runOf({ sharedVoxels: 0, voxelizeMs: 24000 }),
    runOf({ neighbor: 0, sharedVoxels: 0, voxelizeMs: 110 })
  ]);
  const byName = new Map(classes.map((row) => [row.class, row]));
  assert.equal(byName.get("admitted_shares").n, 2);
  assert.equal(byName.get("admitted_shares").voxelizeMs_median, 1950);
  assert.equal(byName.get("admitted_empty").voxelizeMs_median, 24000);
  assert.equal(byName.get("rejected").voxelizeMs_median, 110);
  // The verdict is the same price in every class, so it is quoted per class.
  assert.equal(byName.get("rejected").testUs_median, 0.1);
});

test("an empty class is left out rather than reported as zero", () => {
  const classes = costClasses([runOf({}), runOf({})]);
  assert.equal(classes.length, 1);
  assert.equal(classes[0].class, "admitted_shares");
});

test("the cell budget prices the filter against computing every pair", () => {
  const runs = [
    runOf({ cell: 2, pair: "A__B", voxelizeMs: 1000 }),
    runOf({ cell: 2, pair: "A__C", neighbor: 0, sharedVoxels: 0, voxelizeMs: 100 })
  ];
  const [row] = cellBudget(runs);
  assert.equal(row.allPairs, 2);
  assert.equal(row.admittedPairs, 1);
  assert.equal(row.rejectedPairs, 1);
  assert.equal(row.withoutFilterMs_median, 1100);
  // 1000 ms of computation plus two verdicts of 0.1 us.
  assert.ok(Math.abs(row.withFilterMs_median - 1000.0002) < 1e-6);
  assert.ok(Math.abs(row.savedMs_median - 99.9998) < 1e-6);
  assert.ok(row.savedShare_median > 0.09 && row.savedShare_median < 0.1);
});

test("the cell budget medians over the seeds and keeps the cells apart", () => {
  const runs = [
    runOf({ cell: 4, seed: 1, voxelizeMs: 1000 }),
    runOf({ cell: 4, seed: 2, voxelizeMs: 3000 }),
    runOf({ cell: 5, seed: 1, voxelizeMs: 500 })
  ];
  const rows = cellBudget(runs);
  assert.deepEqual(rows.map((row) => row.cell), [4, 5]);
  assert.equal(rows[0].seeds, 2);
  assert.equal(rows[0].withoutFilterMs_median, 2000);
  assert.equal(rows[1].withoutFilterMs_median, 500);
});

test("a live entry without a cache flag is a computation that produced nothing", () => {
  assert.equal(classifyLive({ cached: true }), "cached");
  assert.equal(classifyLive({ cached: false }), "cold");
  assert.equal(classifyLive({ reason: "voxel_intersection_empty" }), "failed_empty");
  assert.equal(classifyLive({ reason: "no_geometric_overlap" }), "failed_bbox");
});

test("the live rows summarize per pair and over all of them", () => {
  const entries = [
    { pairKey: "A__B", cached: false, durationMs: 1000, voxelCount: 300 },
    { pairKey: "A__B", cached: true, durationMs: 60, voxelCount: 300 },
    { pairKey: "A__C", reason: "voxel_intersection_empty", durationMs: 3000, voxelCount: 0 }
  ];
  const rows = liveRows(entries);
  const all = rows.filter((row) => row.pair === "all pairs");
  assert.deepEqual(all.map((row) => row.class).sort(), ["cached", "cold", "failed_empty"]);
  assert.equal(all.find((row) => row.class === "cold").durationMs_median, 1000);
  assert.equal(rows.find((row) => row.pair === "A__C").n, 1);
});

test("the live saving counts the rejected pairs that never asked for a cloud", () => {
  const entries = [
    { pairKey: "Robot1__Robot2", cached: false, durationMs: 1400, voxelCount: 300 },
    { pairKey: "Robot1__Robot2", cached: false, durationMs: 1600, voxelCount: 300 }
  ];
  const runs = [
    runOf({ cell: 5, pair: "Robot1__Robot2" }),
    runOf({ cell: 5, pair: "Robot1__Robot5", neighbor: 0, sharedVoxels: 0, voxelizeMs: 110 })
  ];
  const saving = liveSaving(entries, runs, 5);
  assert.equal(saving.pairs, 2);
  assert.equal(saving.rejectedPairs, 1);
  assert.equal(saving.rejectedNeverRequested, 1);
  assert.equal(saving.rejectedPairKeys, "Robot1__Robot5");
  assert.equal(saving.coldMs_median, 1500);
  assert.equal(saving.savedMs_median, 1500);
});

test("a rejected pair that does appear in the log is not counted as saved", () => {
  const entries = [{ pairKey: "Robot1__Robot5", cached: false, durationMs: 1400, voxelCount: 0 }];
  const runs = [runOf({ cell: 5, pair: "Robot1__Robot5", neighbor: 0, sharedVoxels: 0 })];
  const saving = liveSaving(entries, runs, 5);
  assert.equal(saving.rejectedPairs, 1);
  assert.equal(saving.rejectedNeverRequested, 0);
  assert.equal(saving.savedMs_median, 0);
});

test("a line cell admits only the pairs within the two widened radii", () => {
  // Radii 0.6 and 0.55 alternate, so a pair one pitch apart is admitted
  // (1.15 > 1.0) and a pair two pitches apart is not (1.2 < 2.0).
  const row = lineCell(4, 1.0, RADII);
  assert.equal(row.allPairs, 6);
  assert.equal(row.admittedPairs, 3);
  assert.equal(row.rejectedPairs, 3);
  assert.equal(row.admittedShare, 0.5);
  assert.equal(row.admittedPerRobot, 1.5);
  assert.ok(row.margin_median > 0);
});

test("the admitted pairs of a line grow linearly while the pairs grow quadratically", () => {
  const rows = lineCurve({ sizes: [5, 10, 20], pitch: 1.0, radii: RADII, coldMs: 1000 });
  const at = (n) => rows.find((row) => row.robots === n);
  assert.equal(at(20).allPairs, 190);
  // Each robot keeps one neighbour on each side however long the line gets.
  assert.equal(at(20).admittedPairs, 19);
  assert.equal(at(10).admittedPairs, 9);
  assert.ok(at(20).admittedPerRobot < 2.01);
  assert.deepEqual(rows.filter((row) => row.requested).map((row) => row.robots), [5, 10, 20]);
  // The times are the pair counts at the price of one live computation.
  assert.equal(at(20).withFilterS, 19);
  assert.equal(at(20).withoutFilterS, 190);
});

test("the line curve is a curve even when only large cells were requested", () => {
  const rows = lineCurve({ sizes: [20], pitch: 1.0, radii: RADII, coldMs: null });
  assert.equal(rows[0].robots, 2);
  assert.equal(rows[rows.length - 1].robots, 20);
  assert.equal(rows[0].withFilterS, null);
});

test("the pitch is the median nearest-neighbour base distance, not the span", () => {
  const descriptor = {
    robots: [
      { name: "A", offset: [0, 0, 0] },
      { name: "B", offset: [1, 0, 0] },
      { name: "C", offset: [5, 0, 0] }
    ]
  };
  // Nearest distances are 1, 1 and 4, so the median is 1 and the far robot
  // does not stretch the line.
  assert.equal(nearestNeighbourPitch(descriptor), 1);
});

test("the generated LaTeX carries both populations and points at the data", () => {
  const classes = costClasses([runOf({}), runOf({ neighbor: 0, sharedVoxels: 0, voxelizeMs: 110 })]);
  const scaling = lineCurve({ sizes: [5], pitch: 1.0, radii: RADII, coldMs: 1000 });
  const budget = cellBudget([runOf({ cell: 5 })]);
  const table = costTable({ budget, classes, scaling, lineSizes: [5], coldMs: 1000 });
  assert.match(table, /label\{tab:exp_e6\}/);
  assert.match(table, /5-robot cell/);
  assert.match(table, /line of 5/);
  assert.match(table, /admitted, shares space/);
  const figure = scalingFigure({ lineSizes: [5, 10, 20], pitch: 1.24 });
  assert.match(figure, /label\{fig:exp_e6\}/);
  assert.match(figure, /e6_scaling\.csv/);
  assert.match(figure, /n\(n-1\)\/2/);
});
