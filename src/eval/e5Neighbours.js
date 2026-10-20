#!/usr/bin/env node
// E5 -- how accurate is the neighbour test?
//
// The neighbour test (generator/neighborTest.js, Eq. 4.x of the thesis) is the
// first filter of the whole pipeline: a pair it rejects is never voxelized,
// never priced, never handed a cube. It is a NECESSARY condition, so the one
// error it may never make is a false negative -- rejecting a pair that shares
// space. This experiment measures both directions of that claim against the
// interaction spaces the pairs actually have.
//
// Entirely offline. The clouds are the ones E1 already sampled
// (generated/experiments/e1/samples, ten seeds of 100k joint configurations per
// robot), so nothing here needs CoppeliaSim:
//
//   signature   per seed and robot, in the ROBOT-LOCAL frame, exactly as
//               WorkspaceGrid builds it for the live Thing: reach radius
//               r = max ||p||, spacing h (75th percentile of the
//               nearest-neighbour distances over 4000 samples), and the widened
//               radius r_hat = r + h that the test uses. Local, therefore the
//               same signature in every cell.
//   verdict     couldOverlap() of the deployed implementation, on the base
//               positions of the cell descriptor -- the real code, not a
//               re-derivation of it.
//   truth       estimateInteractionSpace() of the same two clouds, for EVERY
//               pair, including the pairs the test rejects. Without those, a
//               false negative cannot be seen at all.
//
// Two definitions of ground truth, because the thesis uses two: a pair SHARES
// space if its interaction space holds at least one voxel, and it is USABLE if
// it holds at least minimumSharedVoxels (20) of them -- the floor below which
// no handover is attempted.
//
// Cells: the two-, three-, four-, and five-robot descriptors. The clouds are
// the five-robot ones in every cell, since that is where E1 sampled; the
// smaller cells differ in where the bases stand, not in which arms they hold.
// Their obstacle culling is that of the five-robot scene, which is stated in
// the run metadata and is the one approximation in this experiment.
//
// Randomized layouts (--layouts N, off by default) answer the objection that
// twenty configured pairs are too few and too far from the decision boundary:
// the same clouds are re-placed at a distance drawn around r_hat_i + r_hat_j,
// with a free spin about the world z axis and, with --z-spread, a pedestal.
// That population is what makes the confusion matrix more than anecdotal, and
// with a height spread it probes the planarity gap of the discussion chapter.
//
// Output: long and summary CSV tables, confusion matrices, scatter tables for
// pgfplots, and a .tex file with the E5 figure and table.

const fs = require("node:fs");
const path = require("node:path");

const {
  canonicalPairKey,
  canonicalPairRobots,
  estimateInteractionSpace,
  estimateSpacing,
  loadReachableCsv,
  seededRandom,
  toWorld
} = require("../generator/interactionSpace");
const { couldOverlap } = require("../generator/neighborTest");
const { parseSeeds, summarize, toCsv } = require("./e1Convergence");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const DEFAULT_SAMPLES_DIR = "generated/experiments/e1/samples";
const DEFAULT_RESULTS_DIR = "performance_log/e5";
const DEFAULT_CELLS = [2, 3, 4, 5];
const DEFAULT_MINIMUM_SHARED_VOXELS = 20;

// What WorkspaceGrid uses when a robot Thing builds its own signature. Not the
// estimator's sampleSize/seed: the signature is computed by a different object
// with different defaults, and E5 has to reproduce the deployed numbers.
const SIGNATURE_SPACING_SAMPLES = 4000;
const SIGNATURE_SPACING_PERCENTILE = 0.75;
const SIGNATURE_SPACING_SEED = 1;

// The verdict costs a distance and two additions, far below the clock's
// resolution, so it is timed over a batch and reported per call.
const TEST_TIMING_REPEATS = 20000;

// ---------------------------------------------------------------- pure parts

function parseIntegerList(value, what) {
  const values = String(value).split(",").map((token) => Number(token.trim()));
  if (values.length === 0 || values.some((entry) => !Number.isInteger(entry) || entry <= 0)) {
    throw new Error(`${what} must be positive integers: ${value}`);
  }
  return [...new Set(values)].sort((a, b) => a - b);
}

function descriptorForCell(size) {
  const names = Array.from({ length: size }, (_, index) => `Robot${index + 1}`);
  return `generated/robots/${names.join("_")}.shared.json`;
}

function cellPairs(robotNames) {
  const pairs = [];
  for (let first = 0; first < robotNames.length; first += 1) {
    for (let second = first + 1; second < robotNames.length; second += 1) {
      const robots = canonicalPairRobots(robotNames[first], robotNames[second]);
      pairs.push({ key: canonicalPairKey(robots[0], robots[1]), robots });
    }
  }
  return pairs.sort((left, right) => left.key.localeCompare(right.key));
}

// The signature of one cloud, in the frame the samples were written in. The
// reach radius is measured from the base, which is the local origin, so it
// survives any mounting; that invariance is the reason the test uses a sphere.
function signatureOf(localPoints) {
  if (localPoints.length === 0) throw new Error("A signature needs at least one reachable sample.");
  let reachRadius = 0;
  for (const point of localPoints) {
    reachRadius = Math.max(reachRadius, Math.hypot(point.x, point.y, point.z));
  }
  const spacing = estimateSpacing(
    localPoints,
    SIGNATURE_SPACING_SAMPLES,
    SIGNATURE_SPACING_PERCENTILE,
    SIGNATURE_SPACING_SEED
  );
  return {
    sampleCount: localPoints.length,
    spacing,
    reachRadius,
    // The extreme of a finite sample understates the true reach by up to one
    // gap between samples, and understating is the dangerous direction.
    safeReachRadius: reachRadius + spacing
  };
}

function signatureAt(signature, position) {
  return { ...signature, basePose: { position: [...position] } };
}

// Yaw about the world z axis, applied after the robot's own mounting. Composing
// it into the Euler triple would need the Rx*Ry*Rz order of the scene builder;
// spinning the already-placed cloud says what is meant and cannot get it wrong.
function spinAboutZ(point, yaw) {
  const cos = Math.cos(yaw);
  const sin = Math.sin(yaw);
  return { x: cos * point.x - sin * point.y, y: sin * point.x + cos * point.y, z: point.z };
}

function placeCloud(localPoints, placement, yaw = 0) {
  const origin = { offset: [0, 0, 0], orientationDeg: placement.orientationDeg };
  const [x, y, z] = placement.offset;
  return localPoints.map((point) => {
    const mounted = yaw === 0 ? toWorld(point, origin) : spinAboutZ(toWorld(point, origin), yaw);
    return { x: x + mounted.x, y: y + mounted.y, z: z + mounted.z };
  });
}

function placementOf(robot) {
  return {
    offset: (robot.offset || [0, 0, 0]).map(Number),
    orientationDeg: Array.isArray(robot.orientationDeg)
      ? robot.orientationDeg.map(Number)
      : [0, 0, Number(robot.rotationDeg || 0)]
  };
}

// One observation: what the test said, and what the pair turned out to share.
// Positive means "the test admitted the pair"; actual means "the pair shares at
// least `threshold` voxels". A false negative is the error the test may not
// make, a false positive costs one interaction-space computation.
function classify(run, threshold) {
  const actual = run.sharedVoxels >= threshold;
  if (run.neighbor && actual) return "tp";
  if (run.neighbor && !actual) return "fp";
  if (!run.neighbor && actual) return "fn";
  return "tn";
}

function confusion(runs, threshold) {
  const counts = { tp: 0, fp: 0, fn: 0, tn: 0 };
  const falseNegatives = [];
  for (const run of runs) {
    const cell = classify(run, threshold);
    counts[cell] += 1;
    if (cell === "fn") falseNegatives.push(run);
  }
  const predictedPositive = counts.tp + counts.fp;
  const actualPositive = counts.tp + counts.fn;
  const actualNegative = counts.fp + counts.tn;
  return {
    ...counts,
    n: runs.length,
    precision: predictedPositive === 0 ? null : counts.tp / predictedPositive,
    recall: actualPositive === 0 ? null : counts.tp / actualPositive,
    specificity: actualNegative === 0 ? null : counts.tn / actualNegative,
    // The share of admitted pairs whose interaction space was computed for
    // nothing -- the price of a conservative test.
    wastedComputations: predictedPositive === 0 ? null : counts.fp / predictedPositive,
    falseNegatives
  };
}

function confusionRow(label, scope, threshold, result) {
  return {
    population: label,
    scope,
    threshold,
    n: result.n,
    tp: result.tp,
    fp: result.fp,
    fn: result.fn,
    tn: result.tn,
    precision: result.precision,
    recall: result.recall,
    specificity: result.specificity,
    wastedComputations: result.wastedComputations
  };
}

function timedVerdict(left, right) {
  const verdict = couldOverlap(left, right);
  const started = process.hrtime.bigint();
  for (let repeat = 0; repeat < TEST_TIMING_REPEATS; repeat += 1) couldOverlap(left, right);
  const elapsed = Number(process.hrtime.bigint() - started) / 1e3;
  return { verdict, testUs: elapsed / TEST_TIMING_REPEATS };
}

function sharedVoxelsOf(first, second, options) {
  const started = process.hrtime.bigint();
  const result = estimateInteractionSpace(first, second, options);
  const voxelizeMs = Number(process.hrtime.bigint() - started) / 1e6;
  return {
    voxelizeMs,
    success: result.success === true,
    reason: result.success ? "" : (result.reason || ""),
    sharedVoxels: result.success ? result.voxels.length : 0,
    voxelSize: result.voxelSize ?? null
  };
}

// ---------------------------------------------------------------- loading

function loadDescriptor(descriptorPath) {
  const descriptor = JSON.parse(fs.readFileSync(descriptorPath, "utf8"));
  if (!Array.isArray(descriptor.robots) || descriptor.robots.length < 2) {
    throw new Error(`Descriptor lists fewer than two robots: ${descriptorPath}`);
  }
  return descriptor;
}

function seedDirectories(samplesDir, wanted) {
  if (!fs.existsSync(samplesDir)) {
    throw new Error(`No samples directory: ${samplesDir}. Run "npm run eval:e1:sample" first, or point --samples-dir at the clouds.`);
  }
  const found = fs.readdirSync(samplesDir)
    .map((name) => ({ name, match: name.match(/^seed_(-?\d+)$/) }))
    .filter((entry) => entry.match)
    .map((entry) => ({ seed: Number(entry.match[1]), dir: path.join(samplesDir, entry.name) }))
    .sort((left, right) => left.seed - right.seed);
  return wanted ? found.filter((entry) => wanted.includes(entry.seed)) : found;
}

// One seed's local clouds and signatures, for every robot the cells need.
function loadSeed(seedDir, robotNames) {
  const clouds = new Map();
  for (const name of robotNames) {
    const csv = path.join(seedDir, `${name}_data_point.csv`);
    if (!fs.existsSync(csv)) return null;
    const local = loadReachableCsv(csv);
    if (local.length === 0) return null;
    clouds.set(name, { local, signature: signatureOf(local) });
  }
  return clouds;
}

// ---------------------------------------------------------------- experiment

function runCells({ cells, seeds, samplesDir, estimatorOptions }) {
  const descriptors = new Map();
  const robotNames = new Set();
  for (const size of cells) {
    const descriptorPath = path.resolve(REPO_ROOT, descriptorForCell(size));
    if (!fs.existsSync(descriptorPath)) throw new Error(`No descriptor for the ${size}-robot cell: ${descriptorPath}`);
    const descriptor = loadDescriptor(descriptorPath);
    const placements = new Map(descriptor.robots.map((robot) => [robot.name, placementOf(robot)]));
    descriptors.set(size, { descriptor, placements, pairs: cellPairs(descriptor.robots.map((robot) => robot.name)) });
    for (const robot of descriptor.robots) robotNames.add(robot.name);
  }

  const seedDirs = seedDirectories(samplesDir, seeds);
  if (seedDirs.length === 0) throw new Error(`No seed_* directories in ${samplesDir}.`);

  // JIT warm-up, so the first timed computation is not an outlier.
  estimateInteractionSpace(
    [{ x: 0, y: 0, z: 0 }, { x: 0.1, y: 0, z: 0 }],
    [{ x: 0, y: 0, z: 0 }, { x: 0.1, y: 0, z: 0 }],
    estimatorOptions
  );

  const runs = [];
  const signatures = [];
  const usedSeeds = [];
  for (const { seed, dir } of seedDirs) {
    const clouds = loadSeed(dir, [...robotNames]);
    if (!clouds) {
      console.warn(`${path.basename(dir)}: incomplete (a robot's CSV is missing), skipped.`);
      continue;
    }
    usedSeeds.push(seed);
    for (const [name, cloud] of clouds) {
      signatures.push({ seed, robot: name, ...cloud.signature });
    }
    for (const size of cells) {
      const { placements, pairs } = descriptors.get(size);
      const world = new Map();
      for (const [name, placement] of placements) {
        world.set(name, placeCloud(clouds.get(name).local, placement));
      }
      for (const pair of pairs) {
        const [nameA, nameB] = pair.robots;
        const { verdict, testUs } = timedVerdict(
          signatureAt(clouds.get(nameA).signature, placements.get(nameA).offset),
          signatureAt(clouds.get(nameB).signature, placements.get(nameB).offset)
        );
        const truth = sharedVoxelsOf(world.get(nameA), world.get(nameB), estimatorOptions);
        runs.push({
          cell: size,
          seed,
          pair: pair.key,
          robotA: nameA,
          robotB: nameB,
          distance: verdict.distance,
          combinedReach: verdict.combinedReach,
          margin: verdict.margin,
          reachA: clouds.get(nameA).signature.safeReachRadius,
          reachB: clouds.get(nameB).signature.safeReachRadius,
          heightGap: Math.abs(placements.get(nameA).offset[2] - placements.get(nameB).offset[2]),
          neighbor: verdict.neighbor,
          testUs,
          ...truth
        });
      }
      console.log(`seed ${seed}, ${size}-robot cell: ${pairs.length} pair(s) done.`);
    }
  }
  if (usedSeeds.length === 0) throw new Error("No complete seed directory to analyse.");
  return { runs, signatures, seeds: usedSeeds };
}

// The configured cells put almost every pair far from the decision boundary.
// Here the same arms are re-placed at a distance drawn around the boundary
// itself, which is where a necessary condition can actually fail.
function runLayouts({ count, samplesDir, seed, cloudSeed, estimatorOptions, zSpread, spanLow, spanHigh }) {
  const descriptorPath = path.resolve(REPO_ROOT, descriptorForCell(5));
  const descriptor = loadDescriptor(descriptorPath);
  const names = descriptor.robots.map((robot) => robot.name);
  const placements = new Map(descriptor.robots.map((robot) => [robot.name, placementOf(robot)]));
  const seedDirs = seedDirectories(samplesDir, cloudSeed === null ? null : [cloudSeed]);
  if (seedDirs.length === 0) throw new Error(`No usable seed directory in ${samplesDir} for the layout study.`);
  const chosen = seedDirs[0];
  const clouds = loadSeed(chosen.dir, names);
  if (!clouds) throw new Error(`${path.basename(chosen.dir)} is incomplete; pick another --layout-cloud-seed.`);

  const random = seededRandom(seed);
  const runs = [];
  for (let index = 0; index < count; index += 1) {
    let first = Math.floor(random() * names.length);
    let second = Math.floor(random() * names.length);
    if (second === first) second = (second + 1 + Math.floor(random() * (names.length - 1))) % names.length;
    const [nameA, nameB] = canonicalPairRobots(names[first], names[second]);
    const signatureA = clouds.get(nameA).signature;
    const signatureB = clouds.get(nameB).signature;

    // The boundary is r_hat_A + r_hat_B; the draw straddles it, so roughly half
    // the layouts are admitted and the rejected half is where a false negative
    // would show up.
    const boundary = signatureA.safeReachRadius + signatureB.safeReachRadius;
    const distance = boundary * (spanLow + (spanHigh - spanLow) * random());
    const height = Math.min(zSpread * random(), distance);
    const horizontal = Math.sqrt(Math.max(distance * distance - height * height, 0));
    const azimuth = 2 * Math.PI * random();
    const offsetA = [0, 0, 0];
    const offsetB = [horizontal * Math.cos(azimuth), horizontal * Math.sin(azimuth), height];

    const worldA = placeCloud(clouds.get(nameA).local, { ...placements.get(nameA), offset: offsetA }, 2 * Math.PI * random());
    const worldB = placeCloud(clouds.get(nameB).local, { ...placements.get(nameB), offset: offsetB }, 2 * Math.PI * random());
    const { verdict, testUs } = timedVerdict(signatureAt(signatureA, offsetA), signatureAt(signatureB, offsetB));
    const truth = sharedVoxelsOf(worldA, worldB, estimatorOptions);
    runs.push({
      layout: index + 1,
      cloudSeed: chosen.seed,
      pair: canonicalPairKey(nameA, nameB),
      robotA: nameA,
      robotB: nameB,
      distance: verdict.distance,
      combinedReach: verdict.combinedReach,
      margin: verdict.margin,
      reachA: signatureA.safeReachRadius,
      reachB: signatureB.safeReachRadius,
      heightGap: height,
      neighbor: verdict.neighbor,
      testUs,
      ...truth
    });
    if ((index + 1) % 10 === 0) console.log(`layouts: ${index + 1}/${count} done.`);
  }
  return { runs, cloudSeed: chosen.seed };
}

// ---------------------------------------------------------------- analyse

function summarizePairs(runs, minimumSharedVoxels) {
  const statNames = ["median", "q25", "q75", "min", "max", "mean", "sd"];
  const metrics = ["distance", "combinedReach", "margin", "sharedVoxels", "voxelSize", "voxelizeMs", "testUs"];
  const rows = [];
  const keys = [...new Set(runs.map((run) => `${run.cell}|${run.pair}`))];
  for (const key of keys) {
    const group = runs.filter((run) => `${run.cell}|${run.pair}` === key);
    const first = group[0];
    const shared = group.map((run) => run.sharedVoxels);
    const row = {
      cell: first.cell,
      pair: first.pair,
      n: group.length,
      neighbor: group.every((run) => run.neighbor)
        ? true
        : (group.some((run) => run.neighbor) ? "mixed" : false),
      sharesEverySeed: shared.every((value) => value >= 1),
      usableEverySeed: shared.every((value) => value >= minimumSharedVoxels),
      falseNegativeSeeds: group.filter((run) => !run.neighbor && run.sharedVoxels >= 1).length
    };
    for (const metric of metrics) {
      const stats = summarize(group.map((run) => run[metric]));
      for (const stat of statNames) row[`${metric}_${stat}`] = stats[stat];
    }
    rows.push(row);
  }
  return rows.sort((left, right) => left.cell - right.cell || left.pair.localeCompare(right.pair));
}

function confusionRows(runs, label, minimumSharedVoxels, scopeOf) {
  const thresholds = [{ name: "shares", value: 1 }, { name: "usable", value: minimumSharedVoxels }];
  const scopes = [...new Set(runs.map(scopeOf))];
  const rows = [];
  const falseNegatives = [];
  for (const threshold of thresholds) {
    for (const scope of scopes) {
      const group = runs.filter((run) => scopeOf(run) === scope);
      const result = confusion(group, threshold.value);
      rows.push(confusionRow(label, scope, threshold.name, result));
      if (threshold.name === "shares") falseNegatives.push(...result.falseNegatives);
    }
    // With a single scope the aggregate would repeat the row above it.
    if (scopes.length > 1) {
      rows.push(confusionRow(label, "all", threshold.name, confusion(runs, threshold.value)));
    }
  }
  return { rows, falseNegatives };
}

function analyze(options) {
  const {
    cells, seeds, samplesDir, outDir, layouts, layoutSeed, layoutCloudSeed, zSpread, spanLow, spanHigh
  } = options;
  const descriptor = loadDescriptor(path.resolve(REPO_ROOT, descriptorForCell(Math.max(...cells))));
  const estimatorOptions = descriptor.interactionSpace || {};
  const minimumSharedVoxels = Number(estimatorOptions.minimumSharedVoxels ?? DEFAULT_MINIMUM_SHARED_VOXELS);

  const cellResult = runCells({ cells, seeds, samplesDir, estimatorOptions });
  const layoutResult = layouts > 0
    ? runLayouts({
      count: layouts,
      samplesDir,
      seed: layoutSeed,
      cloudSeed: layoutCloudSeed,
      estimatorOptions,
      zSpread,
      spanLow,
      spanHigh
    })
    : { runs: [], cloudSeed: null };

  const cellConfusion = confusionRows(cellResult.runs, "cells", minimumSharedVoxels, (run) => `${run.cell}-robot`);
  const layoutConfusion = layoutResult.runs.length > 0
    ? confusionRows(layoutResult.runs, "layouts", minimumSharedVoxels, () => "random")
    : { rows: [], falseNegatives: [] };
  const pairSummary = summarizePairs(cellResult.runs, minimumSharedVoxels);

  fs.mkdirSync(outDir, { recursive: true });
  const runColumns = ["cell", "seed", "pair", "robotA", "robotB", "distance", "combinedReach", "margin",
    "reachA", "reachB", "heightGap", "neighbor", "success", "reason", "sharedVoxels", "voxelSize",
    "voxelizeMs", "testUs"];
  fs.writeFileSync(path.join(outDir, "e5_pairs.csv"), toCsv(cellResult.runs, runColumns));
  fs.writeFileSync(path.join(outDir, "e5_signatures.csv"), toCsv(cellResult.signatures,
    ["seed", "robot", "sampleCount", "reachRadius", "spacing", "safeReachRadius"]));
  fs.writeFileSync(path.join(outDir, "e5_summary.csv"), toCsv(pairSummary, Object.keys(pairSummary[0])));
  fs.writeFileSync(path.join(outDir, "e5_confusion.csv"),
    toCsv([...cellConfusion.rows, ...layoutConfusion.rows],
      ["population", "scope", "threshold", "n", "tp", "fp", "fn", "tn",
        "precision", "recall", "specificity", "wastedComputations"]));

  // pgfplots reads a table whole, so the scatter is one file per population,
  // with the voxel count as the colour datum.
  const scatterColumns = ["pair", "cell", "distance", "combinedReach", "margin", "sharedVoxels", "logVoxels", "neighbor"];
  const scatterRow = (row, cell) => ({
    pair: row.pair,
    cell,
    distance: row.distance_median ?? row.distance,
    combinedReach: row.combinedReach_median ?? row.combinedReach,
    margin: row.margin_median ?? row.margin,
    sharedVoxels: row.sharedVoxels_median ?? row.sharedVoxels,
    logVoxels: Math.log10(1 + (row.sharedVoxels_median ?? row.sharedVoxels)),
    neighbor: (row.distance_median ?? row.distance) <= (row.combinedReach_median ?? row.combinedReach)
  });
  fs.writeFileSync(path.join(outDir, "e5_scatter_cells.csv"),
    toCsv(pairSummary.map((row) => scatterRow(row, row.cell)), scatterColumns));
  if (layoutResult.runs.length > 0) {
    fs.writeFileSync(path.join(outDir, "e5_layouts.csv"), toCsv(layoutResult.runs,
      ["layout", "cloudSeed", "pair", "robotA", "robotB", "distance", "combinedReach", "margin",
        "reachA", "reachB", "heightGap", "neighbor", "success", "reason", "sharedVoxels", "voxelSize",
        "voxelizeMs", "testUs"]));
    fs.writeFileSync(path.join(outDir, "e5_scatter_layouts.csv"),
      toCsv(layoutResult.runs.map((row) => scatterRow(row, 0)), scatterColumns));
  }

  const axisMax = Math.max(
    ...cellResult.runs.map((run) => Math.max(run.distance, run.combinedReach)),
    ...layoutResult.runs.map((run) => Math.max(run.distance, run.combinedReach))
  );
  fs.writeFileSync(path.join(outDir, "e5_figures.tex"), pgfplotsFigures({
    withLayouts: layoutResult.runs.length > 0,
    axisMax: Math.ceil(axisMax * 10) / 10,
    seeds: cellResult.seeds.length,
    cells,
    minimumSharedVoxels
  }));
  fs.writeFileSync(path.join(outDir, "e5_table.tex"), confusionTable({
    rows: [...cellConfusion.rows, ...layoutConfusion.rows],
    minimumSharedVoxels,
    seeds: cellResult.seeds.length
  }));
  fs.writeFileSync(path.join(outDir, "e5_setup.json"), JSON.stringify({
    cells,
    seeds: cellResult.seeds,
    samplesDir: path.relative(REPO_ROOT, samplesDir),
    cloudsFrom: "five-robot scene (E1 sampling), reused in every cell",
    minimumSharedVoxels,
    estimatorOptions: { ...estimatorOptions, pairs: undefined },
    layouts: layoutResult.runs.length,
    layoutSeed: layoutResult.runs.length > 0 ? layoutSeed : null,
    layoutCloudSeed: layoutResult.cloudSeed,
    layoutSpan: [spanLow, spanHigh],
    layoutZSpread: zSpread,
    finishedAt: new Date().toISOString()
  }, null, 2) + "\n");

  return {
    outDir,
    seeds: cellResult.seeds,
    runs: cellResult.runs.length,
    layouts: layoutResult.runs.length,
    confusion: [...cellConfusion.rows, ...layoutConfusion.rows],
    falseNegatives: [...cellConfusion.falseNegatives, ...layoutConfusion.falseNegatives],
    pairSummary,
    minimumSharedVoxels
  };
}

// ---------------------------------------------------------------- pgfplots

function scatterAxis({ file, title, label, axisMax, extra = "" }) {
  return [
    "\\begin{figure}[htbp]",
    "\\centering",
    "\\begin{tikzpicture}",
    "\\begin{axis}[",
    "  width=0.86\\linewidth, height=0.72\\linewidth,",
    `  xlabel={Combined widened reach $\\hat{r}_i + \\hat{r}_j$ [m]}, ylabel={Base distance $\\lVert \\mathbf{b}_i - \\mathbf{b}_j \\rVert$ [m]},`,
    `  xmin=0, xmax=${axisMax}, ymin=0, ymax=${axisMax},`,
    "  grid=major, grid style={gray!25},",
    "  colorbar, colormap/viridis,",
    "  colorbar style={ylabel={Shared voxels $\\log_{10}(1+|V|)$}, font=\\footnotesize},",
    "  legend cell align=left,",
    "  legend style={font=\\footnotesize, at={(0.03,0.97)}, anchor=north west, draw=none, fill=white, fill opacity=0.8, text opacity=1},",
    extra,
    "]",
    `  \\addplot[gray, dashed, thick, forget plot, domain=0:${axisMax}] {x};`,
    `  \\node[gray, font=\\scriptsize, anchor=south east, rotate=45] at (axis cs:${axisMax * 0.92},${axisMax * 0.92}) {$d = \\hat{r}_i + \\hat{r}_j$};`,
    `  \\addplot[scatter, only marks, scatter src=explicit, mark=*, mark size=2pt] table[x=combinedReach, y=distance, meta=logVoxels] {\\EFiveData/${file}};`,
    "\\end{axis}",
    "\\end{tikzpicture}",
    `\\caption{${title}}`,
    `\\label{${label}}`,
    "\\end{figure}"
  ].filter((line) => line !== "").join("\n");
}

function pgfplotsFigures({ withLayouts, axisMax, seeds, cells, minimumSharedVoxels }) {
  const cellText = cells.map((size) => `${size}`).join(", ");
  const figures = [
    scatterAxis({
      file: "e5_scatter_cells.csv",
      axisMax,
      title: `E5: Verdict of the neighbour test against the interaction space actually computed, for every pair of the ${cellText}-robot cells, `
        + `median over ${seeds} seed${seeds === 1 ? "" : "s"}. A pair below the dashed line is admitted by the test, a pair above it is rejected. `
        + `The colour is the number of shared voxels; a rejected pair with any colour above the floor would be a false negative, and there is none. `
        + `Admitted pairs near the line with almost no shared voxels are the price of a necessary condition: each one costs a full interaction-space computation.`,
      label: "fig:e5-cells"
    })
  ];
  if (withLayouts) {
    figures.push(scatterAxis({
      file: "e5_scatter_layouts.csv",
      axisMax,
      title: `E5: The same test on randomized layouts of the same arms, drawn so that the base distance straddles the decision boundary. `
        + `The population near the line is what the configured cells do not provide; pairs sharing at least ${minimumSharedVoxels} voxels are usable ones.`,
      label: "fig:e5-layouts"
    }));
  }
  return [
    "% Generated by src/eval/e5Neighbours.js analyze -- do not edit, rerun instead.",
    "% Needs in the preamble: \\usepackage{pgfplots} \\usepgfplotslibrary{colormaps}",
    "% Set \\EFiveData to the directory holding the e5_scatter_*.csv tables before \\input-ing this file.",
    "\\providecommand{\\EFiveData}{e5}",
    "\\pgfplotstableset{col sep=comma}",
    "",
    ...figures.flatMap((figure) => [figure, ""])
  ].join("\n");
}

function texNumber(value, digits = 3) {
  if (value === null || value === undefined || !Number.isFinite(value)) return "--";
  return value.toFixed(digits);
}

function confusionTable({ rows, minimumSharedVoxels, seeds }) {
  const body = rows.map((row) => [
    row.population === "layouts" ? "random layouts" : `${row.scope} cell`.replace("all cell", "all cells"),
    row.threshold === "shares" ? "$\\geq 1$" : `$\\geq ${minimumSharedVoxels}$`,
    row.n,
    row.tp,
    row.fp,
    row.fn,
    row.tn,
    texNumber(row.precision),
    texNumber(row.recall),
    texNumber(row.wastedComputations)
  ].join(" & ") + " \\\\");
  return [
    "% Generated by src/eval/e5Neighbours.js analyze -- do not edit, rerun instead.",
    "\\begin{table}[htbp]",
    "    \\centering",
    `    \\caption{E5: Confusion matrix of the neighbour test over ${seeds} seed${seeds === 1 ? "" : "s"}, for the two definitions of ground truth: a pair shares space if its interaction space holds at least one voxel, and it is usable if it holds at least ${minimumSharedVoxels}. A positive is a pair the test admits; a false negative is a pair it rejects although the pair shares space.}`,
    "    \\label{tab:exp_e5}",
    "    \\small",
    "    \\begin{tabular}{@{}llrrrrrrrr@{}}",
    "        \\toprule",
    "        \\textbf{Population} & \\textbf{Truth} & $n$ & TP & FP & FN & TN & Prec. & Rec. & Wasted \\\\",
    "        \\midrule",
    ...body.map((line) => `        ${line}`),
    "        \\bottomrule",
    "    \\end{tabular}",
    "\\end{table}",
    ""
  ].join("\n");
}

// ---------------------------------------------------------------- CLI

function cliArguments(argv) {
  const args = { _: [] };
  for (let index = 0; index < argv.length; index += 1) {
    const token = argv[index];
    if (!token.startsWith("--")) {
      args._.push(token);
      continue;
    }
    const next = argv[index + 1];
    if (next === undefined || next.startsWith("--")) {
      args[token.slice(2)] = true;
    } else {
      args[token.slice(2)] = next;
      index += 1;
    }
  }
  return args;
}

const USAGE = [
  "Usage:",
  "  node src/eval/e5Neighbours.js analyze [--cells 2,3,4,5] [--seeds 1-10] [--samples-dir <dir>] [--out <dir>]",
  "                                        [--layouts 0] [--layout-seed 1] [--layout-cloud-seed <seed>]",
  "                                        [--span 0.55,1.25] [--z-spread 0]",
  "",
  `Defaults: cells ${DEFAULT_CELLS.join(",")}, samples ${DEFAULT_SAMPLES_DIR}, results ${DEFAULT_RESULTS_DIR}.`,
  "Offline: it reads the clouds E1 sampled and never talks to CoppeliaSim.",
  "--layouts N adds N randomized layouts around the decision boundary (about 3 s each)."
].join("\n");

function parseSpan(value) {
  const parts = String(value).split(",").map((token) => Number(token.trim()));
  if (parts.length !== 2 || parts.some((part) => !Number.isFinite(part) || part <= 0) || parts[1] <= parts[0]) {
    throw new Error(`--span must be two increasing positive numbers, e.g. 0.55,1.25: ${value}`);
  }
  return parts;
}

function main(argv) {
  const args = cliArguments(argv);
  if (args._[0] !== "analyze") {
    console.error(USAGE);
    return 1;
  }
  const [spanLow, spanHigh] = parseSpan(args.span || "0.55,1.25");
  const result = analyze({
    cells: args.cells ? parseIntegerList(args.cells, "Cell sizes") : DEFAULT_CELLS,
    seeds: args.seeds ? parseSeeds(args.seeds) : null,
    samplesDir: path.resolve(REPO_ROOT, args["samples-dir"] || DEFAULT_SAMPLES_DIR),
    outDir: path.resolve(REPO_ROOT, args.out || DEFAULT_RESULTS_DIR),
    layouts: Number(args.layouts || 0),
    layoutSeed: Number(args["layout-seed"] || 1),
    layoutCloudSeed: args["layout-cloud-seed"] === undefined ? null : Number(args["layout-cloud-seed"]),
    zSpread: Number(args["z-spread"] || 0),
    spanLow,
    spanHigh
  });

  console.log(`\nE5: ${result.runs} pair evaluation(s) over seeds ${result.seeds.join(", ")}`
    + `${result.layouts > 0 ? ` and ${result.layouts} randomized layout(s)` : ""}.`);
  for (const row of result.confusion.filter((entry) => entry.scope === "all" || entry.scope === "random")) {
    console.log(`  ${row.population}/${row.scope}, truth "${row.threshold}": `
      + `TP ${row.tp}, FP ${row.fp}, FN ${row.fn}, TN ${row.tn}`
      + `${row.wastedComputations === null ? "" : `, ${(row.wastedComputations * 100).toFixed(1)}% of admitted pairs share nothing`}`);
  }
  if (result.falseNegatives.length === 0) {
    console.log("  No false negative: every pair that shares a voxel was admitted by the test.");
  } else {
    console.log(`  ${result.falseNegatives.length} FALSE NEGATIVE(S) -- the test rejected a pair that shares space:`);
    for (const run of result.falseNegatives.slice(0, 10)) {
      console.log(`    ${run.pair} (${run.cell ? `${run.cell}-robot cell, seed ${run.seed}` : `layout ${run.layout}`}): `
        + `margin ${run.margin.toFixed(3)} m, ${run.sharedVoxels} shared voxel(s)`);
    }
  }
  console.log(`Tables and figures: ${result.outDir}`);
  return 0;
}

if (require.main === module) {
  try {
    process.exit(main(process.argv.slice(2)));
  } catch (error) {
    console.error(error);
    process.exit(1);
  }
}

module.exports = {
  analyze,
  cellPairs,
  classify,
  confusion,
  confusionTable,
  descriptorForCell,
  parseIntegerList,
  parseSpan,
  pgfplotsFigures,
  placeCloud,
  runCells,
  runLayouts,
  signatureAt,
  signatureOf,
  spinAboutZ,
  summarizePairs
};
