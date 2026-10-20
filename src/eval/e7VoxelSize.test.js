const test = require("node:test");
const assert = require("node:assert/strict");

const {
  agreementAt,
  bounds,
  boundingBoxSize,
  boxIntersection,
  decodeReference,
  dilationOffsets,
  evaluateRun,
  fixedSize,
  gridOrigin,
  methodSummary,
  methodTable,
  jaccardOf,
  occupiedKeys,
  parseSweep,
  partnerSeeds,
  poolOccupancy,
  pooledReference,
  referenceCode,
  relativeDeviation,
  resolutionDiagnostics,
  resolutionRatios,
  scatterRows,
  sharedKeysAt,
  sweepCurves,
  sweepFigure,
  sweepPair,
  sweepSizes,
  sweepSummary,
  trimToOverlap
} = require("./e7VoxelSize");

const ORIGIN = { x: 0, y: 0, z: 0 };

// A cube of points on a regular lattice, so that every voxel count in a test is
// countable by hand.
function lattice(from, to, step, offset = { x: 0, y: 0, z: 0 }) {
  const points = [];
  for (let x = from; x <= to + 1e-9; x += step) {
    for (let y = from; y <= to + 1e-9; y += step) {
      for (let z = from; z <= to + 1e-9; z += step) {
        points.push({ x: x + offset.x, y: y + offset.y, z: z + offset.z });
      }
    }
  }
  return points;
}

test("the sweep is the exact list of sizes, without accumulated drift", () => {
  const sizes = sweepSizes({ from: 0.01, to: 0.2, step: 0.005 });
  assert.equal(sizes.length, 39);
  assert.equal(sizes[0], 0.01);
  assert.equal(sizes[6], 0.04);
  assert.equal(sizes[sizes.length - 1], 0.2);
  assert.deepEqual(sweepSizes({ from: 0.05, to: 0.05, step: 0.01 }), [0.05]);
  assert.throws(() => sweepSizes({ from: 0, to: 0.1, step: 0.01 }), /0 < from/);
  assert.throws(() => sweepSizes({ from: 0.1, to: 0.01, step: 0.01 }), /0 < from/);
});

test("--sweep takes from,to,step and nothing else", () => {
  assert.deepEqual(parseSweep("0.01,0.1,0.005"), { from: 0.01, to: 0.1, step: 0.005 });
  assert.throws(() => parseSweep("0.01,0.1"), /from,to,step/);
  assert.throws(() => parseSweep("a,b,c"), /from,to,step/);
});

test("reference codes survive a round trip and refuse a cell off the lattice", () => {
  for (const cell of [[0, 0, 0], [1, -1, 2], [-1023, 1022, 7], [400, -200, 130]]) {
    assert.deepEqual(decodeReference(referenceCode(...cell)), cell);
  }
  assert.throws(() => referenceCode(1024, 0, 0), /outside/);
  assert.throws(() => referenceCode(0, -1025, 0), /outside/);
});

test("the dilation ball holds the offsets within its radius and no others", () => {
  assert.equal(dilationOffsets(0).length, 1);
  // Radius one cell: the cell itself and its six faces, not its edges.
  assert.equal(dilationOffsets(1).length, 7);
  // Radius sqrt(2): the twelve edge neighbours join.
  assert.equal(dilationOffsets(1.5).length, 19);
  assert.equal(dilationOffsets(2).length, 33);
  for (const offset of dilationOffsets(2)) {
    assert.ok(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2 <= 4);
  }
});

test("the pooled occupancy is the union of the seeds' cells", () => {
  const first = [{ x: 0.005, y: 0.005, z: 0.005 }, { x: 0.015, y: 0.005, z: 0.005 }];
  const second = [{ x: 0.015, y: 0.005, z: 0.005 }, { x: 0.025, y: 0.005, z: 0.005 }];
  const pooled = poolOccupancy([first, second], 0.01);
  assert.equal(pooled.size, 3);
  assert.ok(pooled.has(referenceCode(0, 0, 0)));
  assert.ok(pooled.has(referenceCode(2, 0, 0)));
  // One seed alone knows two of the three cells.
  assert.equal(poolOccupancy([first], 0.01).size, 2);
});

test("the reference is what both clouds reach, and dilation 0 is the literal intersection", () => {
  const first = poolOccupancy([[{ x: 0.005, y: 0.005, z: 0.005 }, { x: 0.015, y: 0.005, z: 0.005 }]], 0.01);
  const second = poolOccupancy([[{ x: 0.015, y: 0.005, z: 0.005 }, { x: 0.025, y: 0.005, z: 0.005 }]], 0.01);

  const strict = pooledReference(first, second, 0);
  assert.equal(strict.size, 1);
  assert.ok(strict.has(referenceCode(1, 0, 0)));

  // With one cell of slack the two cells between the clouds join it: cell 0 is
  // reached by the first cloud directly and by the second through cell 1.
  const dilated = pooledReference(first, second, 1);
  assert.ok(dilated.size > strict.size);
  assert.ok(dilated.has(referenceCode(0, 0, 0)));
  assert.ok(dilated.has(referenceCode(2, 0, 0)));

  // Clouds that never come within the radius share nothing.
  const far = poolOccupancy([[{ x: 5.005, y: 0.005, z: 0.005 }]], 0.01);
  assert.equal(pooledReference(first, far, 1).size, 0);
});

test("occupied and shared voxels are the deployed hash sets", () => {
  const first = [{ x: 0.01, y: 0.01, z: 0.01 }, { x: 0.06, y: 0.01, z: 0.01 }];
  const second = [{ x: 0.02, y: 0.02, z: 0.02 }, { x: 0.2, y: 0.01, z: 0.01 }];
  assert.equal(occupiedKeys(first, 0.05, ORIGIN).size, 2);
  const shared = sharedKeysAt(first, second, 0.05, ORIGIN);
  assert.deepEqual([...shared], ["0,0,0"]);
  assert.equal(sharedKeysAt(first, [{ x: 9, y: 9, z: 9 }], 0.05, ORIGIN).size, 0);
});

test("relative deviation is signed against the target", () => {
  assert.equal(relativeDeviation(1000, 1000), 0);
  assert.equal(relativeDeviation(2000, 1000), 1);
  assert.equal(relativeDeviation(500, 1000), -0.5);
});

test("the fixed baseline costs nothing to choose, the bounding-box baseline reads only the boxes", () => {
  assert.deepEqual(fixedSize(0.05), { name: "fixed_0.05", estimateMs: 0, voxelSize: 0.05, mode: "fixed" });

  // Two unit cubes overlapping in a 0.5 x 1 x 1 slab: V = 0.5, s = cbrt(0.5/1000).
  const first = [{ x: 0, y: 0, z: 0 }, { x: 1, y: 1, z: 1 }];
  const second = [{ x: 0.5, y: 0, z: 0 }, { x: 1.5, y: 1, z: 1 }];
  const chosen = boundingBoxSize(first, second, 1000);
  assert.equal(chosen.name, "bbox");
  assert.ok(Math.abs(chosen.overlapVolume - 0.5) < 1e-12);
  assert.ok(Math.abs(chosen.voxelSize - Math.cbrt(0.0005)) < 1e-12);
  assert.ok(chosen.estimateMs >= 0);
});

test("bounds and the shared grid origin are the corner both clouds start from", () => {
  const box = bounds([{ x: -1, y: 0, z: 2 }, { x: 3, y: 1, z: 2 }]);
  assert.deepEqual(box.min, { x: -1, y: 0, z: 2 });
  assert.deepEqual(box.max, { x: 3, y: 1, z: 2 });
  const origin = gridOrigin(box, bounds([{ x: 0, y: -5, z: 0 }]));
  assert.deepEqual(origin, { x: -1, y: -5, z: 0 });
  assert.equal(boxIntersection(bounds([{ x: 0, y: 0, z: 0 }]), bounds([{ x: 9, y: 9, z: 9 }])), null);
});

test("seeds are paired off, each with exactly one partner that is not itself", () => {
  const partner = partnerSeeds([1, 2, 3, 4]);
  assert.equal(partner.get(1), 2);
  assert.equal(partner.get(2), 1);
  assert.equal(partner.get(4), 3);
  for (const [seed, other] of partner) assert.notEqual(seed, other);
  // An odd count repeats a comparison rather than inventing a run.
  const odd = partnerSeeds([1, 2, 3]);
  assert.equal(odd.get(3), 2);
  assert.equal(partnerSeeds([1]).size, 0);
});

test("the agreement of two sets is symmetric and reads zero when they are disjoint", () => {
  const left = new Set(["a", "b", "c"]);
  const right = new Set(["b", "c", "d"]);
  const forward = jaccardOf(left, right);
  assert.equal(forward.common, 2);
  assert.ok(Math.abs(forward.jaccard - 0.5) < 1e-12);
  assert.ok(Math.abs(jaccardOf(right, left).jaccard - forward.jaccard) < 1e-12);
  assert.ok(Math.abs(forward.coverage - 2 / 3) < 1e-12);
  assert.equal(jaccardOf(new Set(["a"]), new Set(["z"])).jaccard, 0);
  assert.equal(jaccardOf(new Set(), new Set()).jaccard, null);
});

test("trimming to the overlap leaves the shared voxels untouched", () => {
  const first = lattice(0, 0.3, 0.02);
  const second = lattice(0, 0.3, 0.02, { x: 0.2, y: 0, z: 0 });
  const origin = { x: 0, y: 0, z: 0 };
  const whole = sharedKeysAt(first, second, 0.05, origin);
  const trimmed = trimToOverlap(first, second, bounds(first), bounds(second), 0.05);
  const cut = sharedKeysAt(trimmed.first, trimmed.second, 0.05, origin);
  assert.ok(trimmed.first.length < first.length, "the trim did not remove anything");
  assert.deepEqual([...cut].sort(), [...whole].sort());
});

// Two runs of the sampler are two INDEPENDENT random draws over the same
// geometry, which is what makes a fine grid irreproducible and a coarse one
// reproducible. A deterministic generator keeps the test from flaking.
function randomCloud(count, box, seed) {
  let state = seed;
  const next = () => {
    state = (state * 1103515245 + 12345) % 2147483648;
    return state / 2147483648;
  };
  return Array.from({ length: count }, () => ({
    x: box.x[0] + next() * (box.x[1] - box.x[0]),
    y: box.y[0] + next() * (box.y[1] - box.y[0]),
    z: box.z[0] + next() * (box.z[1] - box.z[0])
  }));
}

test("reproducibility rises with the voxel size and is 1 for a run against itself", () => {
  const box = { x: [0, 0.4], y: [0, 0.4], z: [0, 0.4] };
  const perSeed = new Map([
    [1, { first: randomCloud(4000, box, 11), second: randomCloud(4000, box, 22) }],
    [2, { first: randomCloud(4000, box, 33), second: randomCloud(4000, box, 44) }]
  ]);
  const origin = { x: 0, y: 0, z: 0 };
  const coarse = agreementAt({ perSeed, seed: 1, partner: 2, size: 0.1, origin, withPooled: false });
  const fine = agreementAt({ perSeed, seed: 1, partner: 2, size: 0.02, origin, withPooled: false });
  assert.ok(coarse.reproJaccard > fine.reproJaccard,
    `coarse ${coarse.reproJaccard} should beat fine ${fine.reproJaccard}`);
  assert.ok(coarse.reproJaccard <= 1 && fine.reproJaccard >= 0);

  // A run compared with a copy of itself agrees completely, which is the only
  // case in which the measure may read 1.
  const clone = new Map([[1, perSeed.get(1)], [2, perSeed.get(1)]]);
  assert.equal(agreementAt({ perSeed: clone, seed: 1, partner: 2, size: 0.05, origin, withPooled: false }).reproJaccard, 1);

  // The pooled comparison is against the other runs, never against itself.
  const pooled = agreementAt({ perSeed, seed: 1, partner: 2, size: 0.05, origin, withPooled: true });
  assert.equal(pooled.pooledVoxels,
    agreementAt({ perSeed, seed: 2, partner: 1, size: 0.05, origin, withPooled: false }).sharedVoxels);
  assert.ok(pooled.pooledCoverage > 0 && pooled.pooledCoverage <= 1);
});

test("the sweep walks every size for every run and scores each against its partner", () => {
  const first = lattice(0, 0.2, 0.02);
  const second = lattice(0, 0.2, 0.02, { x: 0.1, y: 0, z: 0 });
  const perSeed = new Map([[1, { first, second }], [2, { first, second }]]);
  const sizes = sweepSizes({ from: 0.02, to: 0.06, step: 0.02 });
  const rows = sweepPair({
    perSeed,
    partner: partnerSeeds([1, 2]),
    origin: { x: 0, y: 0, z: 0 },
    sizes,
    target: 100,
    spacingBySeed: new Map([[1, 0.02], [2, 0.02]])
  });
  assert.equal(rows.length, sizes.length * perSeed.size);
  for (const row of rows) {
    assert.ok(row.sharedVoxels > 0);
    assert.equal(row.reproJaccard, 1, "identical runs must agree completely");
    assert.ok(Math.abs(row.sizeOverSpacing - row.voxelSize / 0.02) < 1e-12);
    assert.ok(row.voxelizeMs >= 0);
  }
});

test("one run yields every method, and the offline oracles come out of the sweep", () => {
  const first = lattice(0, 0.2, 0.02);
  const second = lattice(0, 0.2, 0.02, { x: 0.1, y: 0, z: 0 });
  const perSeed = new Map([[1, { first, second }], [2, { first, second }]]);
  const origin = { x: 0, y: 0, z: 0 };
  const sizes = sweepSizes({ from: 0.02, to: 0.06, step: 0.02 });
  const sweep = sweepPair({
    perSeed,
    partner: partnerSeeds([1, 2]),
    origin,
    sizes,
    target: 100,
    spacingBySeed: new Map([[1, 0.02], [2, 0.02]])
  });
  const result = evaluateRun({
    clouds: perSeed.get(1),
    full: { first, second },
    perSeed,
    seed: 1,
    partner: 2,
    origin,
    sizes,
    sweep,
    options: { targetVoxelCount: 100, sampleSize: 500, seed: 42, maxVoxelSize: 0.2, reproThreshold: 0.75 },
    target: 100,
    fixed: 0.05,
    minimumSharedVoxels: 20,
    effectiveSpacing: 0.02
  });

  assert.deepEqual(result.methods.map((method) => method.name),
    ["fixed_0.05", "bbox", "proposed", "sweep_target", "sweep_repro"]);
  assert.ok(result.usable);
  assert.ok(result.effectiveSpacing > 0);
  for (const method of result.methods) {
    assert.ok(method.sharedVoxels > 0, `${method.name} found no shared voxel`);
    assert.ok(method.reproJaccard >= 0 && method.reproJaccard <= 1);
    assert.ok(method.pooledCoverage >= 0 && method.pooledCoverage <= 1);
  }
  const byTarget = result.methods.find((method) => method.name === "sweep_target");
  assert.ok(sizes.includes(byTarget.voxelSize));
  const closest = sweep.filter((step) => step.seed === 1)
    .reduce((best, step) => (Math.abs(step.sharedVoxels - 100) < Math.abs(best.sharedVoxels - 100) ? step : best));
  assert.equal(byTarget.voxelSize, closest.voxelSize);
  // Identical runs reproduce everything, so the finest swept size qualifies.
  assert.equal(result.methods.find((method) => method.name === "sweep_repro").voxelSize, sizes[0]);
});

test("a size below the native spacing shows up as one sample per voxel", () => {
  const first = lattice(0, 0.1, 0.02);
  const second = lattice(0, 0.1, 0.02, { x: 0.02, y: 0, z: 0 });
  const origin = { x: 0, y: 0, z: 0 };
  const fine = sharedKeysAt(first, second, 0.005, origin);
  const coarse = sharedKeysAt(first, second, 0.04, origin);
  const fineStats = resolutionDiagnostics({
    shared: fine, first, second, occupiedFirst: first.length, occupiedSecond: second.length,
    size: 0.005, spacing: 0.02
  });
  const coarseStats = resolutionDiagnostics({
    shared: coarse, first, second, occupiedFirst: occupiedKeys(first, 0.04, origin).size,
    occupiedSecond: occupiedKeys(second, 0.04, origin).size, size: 0.04, spacing: 0.02
  });
  assert.equal(fineStats.samplesPerVoxelFirst, 1);
  assert.ok(coarseStats.samplesPerVoxelFirst > 1);
  assert.ok(Math.abs(fineStats.sizeOverSpacing - 0.25) < 1e-12);
  assert.ok(Math.abs(coarseStats.sizeOverSpacing - 2) < 1e-12);
  assert.ok(coarseStats.fill > 0 && coarseStats.fill <= 1);
});

test("the resolution ratio says how far a size sits from the finest reproducible one", () => {
  const runs = [
    { cell: 5, seed: 1, pair: "Robot1__Robot2", method: "sweep_repro", voxelSize: 0.08 },
    { cell: 5, seed: 1, pair: "Robot1__Robot2", method: "proposed", voxelSize: 0.04 },
    { cell: 5, seed: 1, pair: "Robot1__Robot2", method: "fixed_0.05", voxelSize: 0.16 },
    // A run whose sweep never reached the threshold has no ratio at all.
    { cell: 4, seed: 2, pair: "Robot1__Robot2", method: "proposed", voxelSize: 0.04 }
  ];
  const rows = resolutionRatios(runs);
  assert.ok(Math.abs(rows[1].sizeOverReproducible - 0.5) < 1e-12);
  assert.ok(Math.abs(rows[2].sizeOverReproducible - 2) < 1e-12);
  assert.equal(rows[3].sizeOverReproducible, null);
  assert.equal(rows[0].reproducibleSize, 0.08);
});

test("the method summary keeps one row per method and counts the near misses", () => {
  const runs = [
    { method: "proposed", voxelSize: 0.05, sharedVoxels: 1000, relDeviation: 0, absDeviation: 0, sharedVolume: 1, jaccard: 0.5, coverage: 0.6, precision: 0.7, estimateMs: 10, voxelizeMs: 5, totalMs: 15 },
    { method: "proposed", voxelSize: 0.06, sharedVoxels: 400, relDeviation: -0.6, absDeviation: 600, sharedVolume: 1, jaccard: 0.4, coverage: 0.5, precision: 0.6, estimateMs: 10, voxelizeMs: 5, totalMs: 15 },
    { method: "fixed_0.05", voxelSize: 0.05, sharedVoxels: 0, relDeviation: -1, absDeviation: 1000, sharedVolume: 0, jaccard: 0, coverage: 0, precision: null, estimateMs: 0, voxelizeMs: 5, totalMs: 5 }
  ];
  const rows = methodSummary(runs, 1000);
  assert.deepEqual(rows.map((row) => row.method), ["fixed_0.05", "proposed"]);
  const proposed = rows.find((row) => row.method === "proposed");
  assert.equal(proposed.n, 2);
  assert.equal(proposed.hitsTarget, 1);
  assert.equal(rows.find((row) => row.method === "fixed_0.05").empty, 1);
  assert.ok(Math.abs(proposed.sharedVoxels_mean - 700) < 1e-9);
});

test("the scatter compares each method with the offline oracle of the same run", () => {
  const runs = [
    { method: "sweep_target", cell: 5, pair: "Robot1__Robot2", seed: 1, sharedVoxels: 900, jaccard: 0.5, totalMs: 300 },
    { method: "proposed", cell: 5, pair: "Robot1__Robot2", seed: 1, sharedVoxels: 1600, jaccard: 0.3, totalMs: 1500 },
    // A run without an oracle row cannot be placed on the diagonal and is dropped.
    { method: "proposed", cell: 4, pair: "Robot1__Robot2", seed: 2, sharedVoxels: 100, jaccard: 0.1, totalMs: 40 }
  ];
  const rows = scatterRows(runs);
  assert.equal(rows.length, 1);
  assert.equal(rows[0].method, "proposed");
  assert.equal(rows[0].reference, 900);
  assert.equal(rows[0].achieved, 1600);
});

test("the sweep summary and the curves keep the sizes in order and mark the chosen one", () => {
  const rows = [0.01, 0.02, 0.03].flatMap((voxelSize) => [1, 2].map((seed) => ({
    cell: 5,
    seed,
    pair: "Robot1__Robot2",
    voxelSize,
    sharedVoxels: voxelSize * 10000,
    relDeviation: 0,
    sharedVolume: 1,
    partnerVoxels: voxelSize * 10000,
    reproJaccard: voxelSize === 0.02 ? 0.9 : 0.2,
    sizeOverSpacing: voxelSize / 0.02,
    voxelizeMs: 10
  })));
  const perSize = sweepSummary(rows);
  assert.deepEqual(perSize.map((row) => row.voxelSize), [0.01, 0.02, 0.03]);
  assert.equal(perSize[1].n, 2);

  // 0.021 lies inside half a step of 0.02 and nowhere else.
  const curves = sweepCurves(rows, new Map([["5|Robot1__Robot2", 0.021]]));
  assert.equal(curves.length, 3);
  assert.deepEqual(curves.map((row) => row.isChosen), [false, true, false]);
  assert.equal(curves[1].reproJaccard, 0.9);
  // Without a chosen size nothing is marked, and the curve still comes out.
  assert.deepEqual(sweepCurves(rows).map((row) => row.isChosen), [false, false, false]);
});

test("the E7 table lists the methods in a fixed order and says what the reference is", () => {
  const tex = methodTable({
    rows: methodSummary([
      { method: "proposed", voxelSize: 0.05, sharedVoxels: 1000, relDeviation: 0, absDeviation: 0, sharedVolume: 1, reproJaccard: 0.22, pooledCoverage: 0.3, pooledPrecision: 0.9, sizeOverSpacing: 1.4, estimateMs: 10, voxelizeMs: 5, totalMs: 15 },
      { method: "fixed_0.05", voxelSize: 0.05, sharedVoxels: 500, relDeviation: -0.5, absDeviation: 500, sharedVolume: 1, reproJaccard: 0.2, pooledCoverage: 0.25, pooledPrecision: 0.8, sizeOverSpacing: 1.2, estimateMs: 0, voxelizeMs: 5, totalMs: 5 },
      { method: "sweep_repro", voxelSize: 0.1, sharedVoxels: 300, relDeviation: -0.7, absDeviation: 700, sharedVolume: 1, reproJaccard: 0.78, pooledCoverage: 0.6, pooledPrecision: 0.95, sizeOverSpacing: 2.8, estimateMs: 300, voxelizeMs: 5, totalMs: 305 }
    ], 1000),
    target: 1000,
    seeds: 10,
    pairs: 120,
    reproThreshold: 0.75
  });
  assert.ok(tex.includes("\\label{tab:exp_e7}"));
  assert.ok(tex.indexOf("Fixed size") < tex.indexOf("Proposed estimator"));
  assert.ok(tex.indexOf("Proposed estimator") < tex.indexOf("finest reproducible"));
  assert.ok(tex.includes("J_\\mathrm{repro} \\geq 0.75"));
  assert.ok(tex.includes("10 seeds"));
  // The medians of the summary, not the means, are what the table prints.
  assert.ok(tex.includes("0.220"), "the reproducibility of the proposed row is missing");
});

test("the E8 figure marks the estimator's size and the threshold it is judged against", () => {
  const curves = [0.02, 0.04, 0.06].map((voxelSize) => ({
    cell: 5,
    pair: "Robot1__Robot2",
    voxelSize,
    reproJaccard: voxelSize * 10,
    sharedVoxels: 1000,
    voxelizeMs: 60,
    chosenByEstimator: 0.04,
    isChosen: voxelSize === 0.04
  }));
  const tex = sweepFigure({ curves, sizes: [0.01, 0.015, 0.2], reproThreshold: 0.75 });
  assert.ok(tex.includes("\\label{fig:exp_e7e8}"));
  assert.ok(tex.includes("discard if not={isChosen}{1}"), "the chosen size is not marked");
  assert.ok(tex.includes("{0.75}"), "the threshold line is missing");
  assert.ok(tex.includes("Robot1--Robot2"), "the legend entry is not readable");
  assert.ok(tex.includes("J_\\mathrm{repro}"));
});
