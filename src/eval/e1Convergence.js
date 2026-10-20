#!/usr/bin/env node
// E1 -- convergence of the shared workspace with the number of samples.
//
// Two stages, because they need different things:
//
//   sample   drives CoppeliaSim. For every seed it samples every robot of the
//            cell ONCE, with the largest sample count, into its own directory.
//            The live cell (CSVs, TDs, scenes, descriptor) is never touched.
//
//   analyze  is offline. Every smaller sample count is the first N samples of
//            the one long run -- the prefix of an i.i.d. stream is itself an
//            i.i.d. sample of size N -- so 5k/10k/25k/50k/100k cost one 100k
//            run per seed instead of five runs. The sampler's progress sidecar
//            says how many CSV rows the first N samples produced, because
//            colliding samples below the floor are not written at all.
//
// Metrics per seed, sample count and robot pair:
//
//   as deployed  voxel count, voxel size and volume from the real estimator,
//                which picks a new voxel size (and grid origin) for every
//                input. Volume = count * size^3 is comparable across N; the
//                voxel sets themselves are not, they sit on different grids.
//
//   fixed grid   every run of a pair on ONE grid: voxel size s_ref (the median
//                deployed size at the largest N) and origin at the world
//                origin. V_N is the pair's shared-voxel set at N samples, V_ref
//                the shared set of all seeds' full runs pooled. Reported:
//                Jaccard |V_N & V_ref| / |V_N | V_ref|, coverage |V_N & V_ref| /
//                |V_ref| and precision |V_N & V_ref| / |V_N|.
//
// Output (analyze): long and summary CSV tables, one summary file per pair for
// pgfplots, and a .tex file with the three E1 figures (log x axis, median line,
// band over seeds).

const fs = require("node:fs");
const path = require("node:path");

const {
  canonicalPairKey,
  canonicalPairRobots,
  estimateInteractionSpace,
  percentile,
  toWorld
} = require("../generator/interactionSpace");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const DEFAULT_DESCRIPTOR = "generated/robots/Robot1_Robot2_Robot3_Robot4_Robot5.shared.json";
const DEFAULT_SAMPLES_DIR = "generated/experiments/e1/samples";
const DEFAULT_RESULTS_DIR = "performance_log/e1";
const DEFAULT_COUNTS = [5000, 10000, 25000, 50000, 100000];
const ORIGIN = { x: 0, y: 0, z: 0 };

// ---------------------------------------------------------------- pure parts

function parseSeeds(value) {
  const seeds = [];
  for (const part of String(value).split(",").map((token) => token.trim()).filter(Boolean)) {
    const range = part.match(/^(-?\d+)-(-?\d+)$/);
    if (range) {
      const [from, to] = [Number(range[1]), Number(range[2])];
      if (to < from) throw new Error(`Seed range ${part} runs backwards.`);
      for (let seed = from; seed <= to; seed += 1) seeds.push(seed);
    } else if (/^-?\d+$/.test(part)) {
      seeds.push(Number(part));
    } else {
      throw new Error(`Not a seed or seed range: ${part}`);
    }
  }
  if (seeds.length === 0) throw new Error("No seeds given.");
  return [...new Set(seeds)];
}

function parseCounts(value) {
  const counts = String(value).split(",").map((token) => Number(token.trim()));
  if (counts.length === 0 || counts.some((count) => !Number.isInteger(count) || count <= 0)) {
    throw new Error(`Sample counts must be positive integers: ${value}`);
  }
  return [...new Set(counts)].sort((a, b) => a - b);
}

function seedDirectoryName(seed) {
  return `seed_${String(seed).padStart(2, "0")}`;
}

function parseProgress(text) {
  const lines = String(text).trim().split(/\r?\n/);
  if (lines[0] !== "Samples,Rows,Seconds") throw new Error("Not a sampling progress file.");
  return lines.slice(1).filter(Boolean).map((line) => {
    const [samples, rows, seconds] = line.split(",").map(Number);
    return { samples, rows, seconds };
  });
}

// The checkpoint for exactly N samples. Interpolating would invent rows that
// were never written, so a count between checkpoints is an error.
function checkpointAt(progress, samples) {
  const found = progress.find((entry) => entry.samples === samples);
  if (!found) {
    const available = progress.map((entry) => entry.samples);
    throw new Error(`No progress checkpoint at ${samples} samples (have ${available[0]}..${available[available.length - 1]}, `
      + "every workspaceProgressEvery samples).");
  }
  return found;
}

// Every row of the workspace CSV in the order it was written, colliding rows
// included -- the prefix is taken over rows, the reachable filter after.
function parseWorkspaceRows(text) {
  const lines = String(text).trim().split(/\r?\n/).slice(1);
  const rows = [];
  for (const line of lines) {
    const values = line.split(",").map(Number);
    if (values.length < 4 || !values.slice(0, 4).every(Number.isFinite)) continue;
    rows.push({ x: values[0], y: values[1], z: values[2], reachable: values[3] === 1 });
  }
  return rows;
}

function reachablePrefix(rows, rowCount) {
  if (rowCount > rows.length) {
    throw new Error(`Progress names ${rowCount} rows but the CSV holds ${rows.length}.`);
  }
  const points = [];
  for (let index = 0; index < rowCount; index += 1) {
    if (rows[index].reachable) points.push({ x: rows[index].x, y: rows[index].y, z: rows[index].z });
  }
  return points;
}

function fixedGridKey(point, voxelSize, origin = ORIGIN) {
  return `${Math.floor((point.x - origin.x) / voxelSize)},${Math.floor((point.y - origin.y) / voxelSize)},${Math.floor((point.z - origin.z) / voxelSize)}`;
}

function occupiedVoxels(points, voxelSize, origin = ORIGIN) {
  const keys = new Set();
  for (const point of points) keys.add(fixedGridKey(point, voxelSize, origin));
  return keys;
}

// Occupied-voxel sets of every prefix of one cloud, built in one pass. The
// points must be in sampling order; boundaries are counts of points.
function nestedOccupiedVoxels(points, boundaries, voxelSize, origin = ORIGIN) {
  const sorted = [...boundaries].sort((a, b) => a - b);
  const snapshots = new Map();
  const keys = new Set();
  let next = 0;
  for (let index = 0; index <= points.length && next < sorted.length; index += 1) {
    while (next < sorted.length && sorted[next] === index) {
      snapshots.set(sorted[next], new Set(keys));
      next += 1;
    }
    if (index < points.length) keys.add(fixedGridKey(points[index], voxelSize, origin));
  }
  if (next < sorted.length) throw new Error(`Prefix boundary ${sorted[next]} lies beyond ${points.length} points.`);
  return snapshots;
}

function intersection(left, right) {
  const [small, large] = left.size <= right.size ? [left, right] : [right, left];
  const result = new Set();
  for (const key of small) if (large.has(key)) result.add(key);
  return result;
}

function overlapMetrics(candidate, reference) {
  const common = intersection(candidate, reference).size;
  const union = candidate.size + reference.size - common;
  return {
    intersection: common,
    union,
    jaccard: union === 0 ? 1 : common / union,
    coverage: reference.size === 0 ? null : common / reference.size,
    precision: candidate.size === 0 ? null : common / candidate.size
  };
}

function summarize(values) {
  const finite = values.filter((value) => value !== null && Number.isFinite(value));
  if (finite.length === 0) {
    return { n: 0, mean: null, sd: null, median: null, q25: null, q75: null, min: null, max: null };
  }
  const mean = finite.reduce((sum, value) => sum + value, 0) / finite.length;
  const variance = finite.length > 1
    ? finite.reduce((sum, value) => sum + (value - mean) ** 2, 0) / (finite.length - 1)
    : 0;
  return {
    n: finite.length,
    mean,
    sd: Math.sqrt(variance),
    median: percentile(finite, 0.5),
    q25: percentile(finite, 0.25),
    q75: percentile(finite, 0.75),
    min: Math.min(...finite),
    max: Math.max(...finite)
  };
}

function median(values) {
  const finite = values.filter((value) => value !== null && Number.isFinite(value));
  return finite.length === 0 ? null : percentile(finite, 0.5);
}

function csvCell(value) {
  if (value === null || value === undefined || (typeof value === "number" && !Number.isFinite(value))) return "nan";
  if (typeof value === "number") return Number.isInteger(value) ? String(value) : value.toPrecision(8);
  if (typeof value === "boolean") return value ? "1" : "0";
  const text = String(value);
  return /[",\n]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
}

function toCsv(rows, columns) {
  return [columns.join(","), ...rows.map((row) => columns.map((column) => csvCell(row[column])).join(","))].join("\n") + "\n";
}

// ---------------------------------------------------------------- analyze

function loadDescriptor(descriptorPath) {
  const descriptor = JSON.parse(fs.readFileSync(descriptorPath, "utf8"));
  if (!Array.isArray(descriptor.robots) || descriptor.robots.length < 2) {
    throw new Error(`Descriptor lists fewer than two robots: ${descriptorPath}`);
  }
  return descriptor;
}

function allPairs(robotNames) {
  const pairs = [];
  for (let first = 0; first < robotNames.length; first += 1) {
    for (let second = first + 1; second < robotNames.length; second += 1) {
      const robots = canonicalPairRobots(robotNames[first], robotNames[second]);
      pairs.push({ key: canonicalPairKey(robots[0], robots[1]), robots });
    }
  }
  return pairs.sort((left, right) => left.key.localeCompare(right.key));
}

function seedDirectories(samplesDir) {
  if (!fs.existsSync(samplesDir)) throw new Error(`No samples directory: ${samplesDir}. Run the sample stage first.`);
  return fs.readdirSync(samplesDir)
    .map((name) => ({ name, match: name.match(/^seed_(-?\d+)$/) }))
    .filter((entry) => entry.match)
    .map((entry) => ({ seed: Number(entry.match[1]), dir: path.join(samplesDir, entry.name) }))
    .sort((left, right) => left.seed - right.seed);
}

// One seed's clouds: world-frame reachable points of every robot, in sampling
// order, and the checkpoint for every requested count.
function loadSeed(seedDir, descriptor, counts) {
  const clouds = new Map();
  for (const robot of descriptor.robots) {
    const csv = path.join(seedDir, `${robot.name}_data_point.csv`);
    const progressFile = path.join(seedDir, `${robot.name}_sampling_progress.csv`);
    if (!fs.existsSync(csv) || !fs.existsSync(progressFile)) return null;
    const rows = parseWorkspaceRows(fs.readFileSync(csv, "utf8"));
    const progress = parseProgress(fs.readFileSync(progressFile, "utf8"));
    const total = progress[progress.length - 1];
    const usable = counts.filter((count) => count <= total.samples);
    const world = reachablePrefix(rows, total.rows).map((point) => toWorld(point, robot));
    const prefixes = new Map();
    for (const count of usable) {
      const checkpoint = checkpointAt(progress, count);
      const reachable = reachablePrefix(rows, checkpoint.rows).length;
      prefixes.set(count, { rows: checkpoint.rows, reachable, seconds: checkpoint.seconds });
    }
    clouds.set(robot.name, { world, prefixes, totalSamples: total.samples });
  }
  return clouds;
}

function analyze({ descriptorPath, samplesDir, outDir, counts, band }) {
  const descriptor = loadDescriptor(descriptorPath);
  const estimatorOptions = descriptor.interactionSpace || {};
  const minimumSharedVoxels = Number(estimatorOptions.minimumSharedVoxels ?? 20);
  const robotNames = descriptor.robots.map((robot) => robot.name);
  const pairs = allPairs(robotNames);
  const seeds = seedDirectories(samplesDir);
  if (seeds.length === 0) throw new Error(`No seed_* directories in ${samplesDir}.`);

  const runs = [];
  const samplingRuns = [];
  const usedSeeds = [];
  let maxCount = 0;

  // Pass 1: the deployed estimator on every prefix.
  estimateInteractionSpace(
    [{ x: 0, y: 0, z: 0 }, { x: 0.1, y: 0, z: 0 }],
    [{ x: 0, y: 0, z: 0 }, { x: 0.1, y: 0, z: 0 }],
    estimatorOptions
  ); // JIT warm-up so the first timed call is not an outlier
  for (const { seed, dir } of seeds) {
    const clouds = loadSeed(dir, descriptor, counts);
    if (!clouds) {
      console.warn(`${path.basename(dir)}: incomplete (a robot's CSV or progress file is missing), skipped.`);
      continue;
    }
    usedSeeds.push({ seed, dir });
    const seedCounts = counts.filter((count) => robotNames.every((name) => clouds.get(name).prefixes.has(count)));
    maxCount = Math.max(maxCount, ...seedCounts);
    for (const name of robotNames) {
      for (const [count, prefix] of clouds.get(name).prefixes) {
        samplingRuns.push({ seed, robot: name, samples: count, rows: prefix.rows, reachable: prefix.reachable, seconds: prefix.seconds });
      }
    }
    for (const count of seedCounts) {
      for (const pair of pairs) {
        const [first, second] = pair.robots.map((name) => {
          const cloud = clouds.get(name);
          return cloud.world.slice(0, cloud.prefixes.get(count).reachable);
        });
        const started = process.hrtime.bigint();
        const result = estimateInteractionSpace(first, second, estimatorOptions);
        const voxelizeMs = Number(process.hrtime.bigint() - started) / 1e6;
        const voxelSize = result.voxelSize ?? null;
        const voxelCount = result.success ? result.voxels.length : 0;
        runs.push({
          seed,
          samples: count,
          pair: pair.key,
          success: result.success,
          reason: result.success ? "" : result.reason,
          reachableA: first.length,
          reachableB: second.length,
          samplingSeconds: clouds.get(pair.robots[0]).prefixes.get(count).seconds
            + clouds.get(pair.robots[1]).prefixes.get(count).seconds,
          voxelizeMs,
          voxelCountAuto: voxelCount,
          voxelSizeAuto: voxelSize,
          volumeAuto: voxelSize === null ? 0 : voxelCount * voxelSize ** 3
        });
      }
    }
    console.log(`${path.basename(dir)}: deployed estimator done for ${seedCounts.length} sample count(s).`);
  }
  if (usedSeeds.length === 0) throw new Error("No complete seed directory to analyze.");

  // One grid per pair, shared by every seed and every N.
  const reference = new Map();
  for (const pair of pairs) {
    const atMax = runs.filter((run) => run.pair === pair.key && run.samples === maxCount && run.success);
    const any = runs.filter((run) => run.pair === pair.key && run.success);
    const voxelSize = median((atMax.length ? atMax : any).map((run) => run.voxelSizeAuto));
    reference.set(pair.key, { voxelSize, pooled: new Map(pair.robots.map((name) => [name, new Set()])), perRun: new Map() });
  }

  // Pass 2: fixed-grid voxel sets, per prefix and pooled over all seeds.
  for (const { seed, dir } of usedSeeds) {
    const clouds = loadSeed(dir, descriptor, counts);
    for (const pair of pairs) {
      const ref = reference.get(pair.key);
      if (ref.voxelSize === null) continue;
      const nested = pair.robots.map((name) => {
        const cloud = clouds.get(name);
        const boundaries = [...cloud.prefixes.values()].map((prefix) => prefix.reachable);
        const snapshots = nestedOccupiedVoxels(cloud.world, boundaries, ref.voxelSize);
        for (const key of occupiedVoxels(cloud.world, ref.voxelSize)) ref.pooled.get(name).add(key);
        return { cloud, snapshots };
      });
      for (const count of counts) {
        if (!nested.every(({ cloud }) => cloud.prefixes.has(count))) continue;
        const [a, b] = nested.map(({ cloud, snapshots }) => snapshots.get(cloud.prefixes.get(count).reachable));
        ref.perRun.set(`${seed}:${count}`, intersection(a, b));
      }
    }
  }

  const referenceRows = [];
  for (const pair of pairs) {
    const ref = reference.get(pair.key);
    ref.shared = ref.voxelSize === null
      ? new Set()
      : intersection(ref.pooled.get(pair.robots[0]), ref.pooled.get(pair.robots[1]));
    ref.degenerate = ref.shared.size < minimumSharedVoxels;
    referenceRows.push({
      pair: pair.key,
      seeds: usedSeeds.length,
      samplesPerRobot: usedSeeds.length * maxCount,
      voxelSizeRef: ref.voxelSize,
      voxelCountRef: ref.shared.size,
      volumeRef: ref.voxelSize === null ? 0 : ref.shared.size * ref.voxelSize ** 3,
      degenerate: ref.degenerate
    });
  }

  for (const run of runs) {
    const ref = reference.get(run.pair);
    const shared = ref.perRun.get(`${run.seed}:${run.samples}`) || new Set();
    const metrics = ref.voxelSize === null ? null : overlapMetrics(shared, ref.shared);
    Object.assign(run, {
      voxelSizeRef: ref.voxelSize,
      voxelCountFixed: shared.size,
      volumeFixed: ref.voxelSize === null ? 0 : shared.size * ref.voxelSize ** 3,
      jaccard: metrics ? metrics.jaccard : null,
      coverage: metrics ? metrics.coverage : null,
      precision: metrics ? metrics.precision : null,
      degenerate: ref.degenerate
    });
  }

  const summaryMetrics = [
    "volumeAuto", "voxelCountAuto", "voxelSizeAuto", "volumeFixed", "voxelCountFixed",
    "jaccard", "coverage", "precision", "voxelizeMs", "samplingSeconds"
  ];
  const statNames = ["median", "q25", "q75", "min", "max", "mean", "sd"];
  const summaryColumns = ["pair", "samples", "n", "degenerate",
    ...summaryMetrics.flatMap((metric) => statNames.map((stat) => `${metric}_${stat}`))];
  const summaryRows = [];
  for (const pair of pairs) {
    for (const count of counts) {
      const group = runs.filter((run) => run.pair === pair.key && run.samples === count);
      if (group.length === 0) continue;
      const row = { pair: pair.key, samples: count, n: group.length, degenerate: reference.get(pair.key).degenerate };
      for (const metric of summaryMetrics) {
        const stats = summarize(group.map((run) => run[metric]));
        for (const stat of statNames) row[`${metric}_${stat}`] = stats[stat];
      }
      summaryRows.push(row);
    }
  }

  const samplingColumns = ["robot", "samples", "n",
    ...["seconds", "reachable", "rows"].flatMap((metric) => statNames.map((stat) => `${metric}_${stat}`))];
  const samplingSummary = [];
  for (const name of robotNames) {
    for (const count of counts) {
      const group = samplingRuns.filter((run) => run.robot === name && run.samples === count);
      if (group.length === 0) continue;
      const row = { robot: name, samples: count, n: group.length };
      for (const metric of ["seconds", "reachable", "rows"]) {
        const stats = summarize(group.map((run) => run[metric]));
        for (const stat of statNames) row[`${metric}_${stat}`] = stats[stat];
      }
      samplingSummary.push(row);
    }
  }

  fs.mkdirSync(outDir, { recursive: true });
  const runColumns = ["seed", "samples", "pair", "success", "reason", "degenerate", "reachableA", "reachableB",
    "samplingSeconds", "voxelizeMs", "voxelCountAuto", "voxelSizeAuto", "volumeAuto",
    "voxelSizeRef", "voxelCountFixed", "volumeFixed", "jaccard", "coverage", "precision"];
  fs.writeFileSync(path.join(outDir, "e1_runs.csv"), toCsv(runs, runColumns));
  fs.writeFileSync(path.join(outDir, "e1_summary.csv"), toCsv(summaryRows, summaryColumns));
  fs.writeFileSync(path.join(outDir, "e1_reference.csv"), toCsv(referenceRows,
    ["pair", "seeds", "samplesPerRobot", "voxelSizeRef", "voxelCountRef", "volumeRef", "degenerate"]));
  fs.writeFileSync(path.join(outDir, "e1_sampling_runs.csv"), toCsv(samplingRuns,
    ["seed", "robot", "samples", "rows", "reachable", "seconds"]));
  fs.writeFileSync(path.join(outDir, "e1_sampling_summary.csv"), toCsv(samplingSummary, samplingColumns));

  // One file per pair and per robot: pgfplots reads a table whole, so a
  // series is a file.
  const pairDir = path.join(outDir, "pairs");
  const robotDir = path.join(outDir, "robots");
  fs.mkdirSync(pairDir, { recursive: true });
  fs.mkdirSync(robotDir, { recursive: true });
  const plottedPairs = pairs.filter((pair) => reference.get(pair.key).voxelSize !== null);
  for (const pair of plottedPairs) {
    fs.writeFileSync(path.join(pairDir, `${pair.key}.csv`),
      toCsv(summaryRows.filter((row) => row.pair === pair.key), summaryColumns));
  }
  for (const name of robotNames) {
    fs.writeFileSync(path.join(robotDir, `${name}.csv`),
      toCsv(samplingSummary.filter((row) => row.robot === name), samplingColumns));
  }
  fs.writeFileSync(path.join(outDir, "e1_figures.tex"), pgfplotsFigures({
    pairs: plottedPairs.map((pair) => ({ key: pair.key, degenerate: reference.get(pair.key).degenerate })),
    robots: robotNames,
    band
  }));

  return { outDir, seeds: usedSeeds.map((entry) => entry.seed), maxCount, pairs: referenceRows };
}

// ---------------------------------------------------------------- pgfplots

const SERIES_COLORS = ["e1c1", "e1c2", "e1c3", "e1c4", "e1c5", "e1c6", "e1c7", "e1c8", "e1c9", "e1c10"];
const SERIES_RGB = ["31,119,180", "255,127,14", "44,160,44", "214,39,40", "148,103,189",
  "140,86,75", "227,119,194", "127,127,127", "188,189,34", "23,190,207"];

function bandColumns(metric, band) {
  return band === "minmax" ? [`${metric}_min`, `${metric}_max`] : [`${metric}_q25`, `${metric}_q75`];
}

function seriesBlock({ file, metric, color, legend, dashed, band, scale = 1 }) {
  const [low, high] = bandColumns(metric, band);
  const id = `${legend}${metric}`.replace(/[^A-Za-z0-9]/g, "");
  const y = (column) => (scale === 1 ? `y=${column}` : `y expr=\\thisrow{${column}}*${scale}`);
  return [
    `  \\addplot[draw=none, forget plot, name path=${id}hi] table[x=samples, ${y(high)}] {\\EOneData/${file}};`,
    `  \\addplot[draw=none, forget plot, name path=${id}lo] table[x=samples, ${y(low)}] {\\EOneData/${file}};`,
    `  \\addplot[${color}, fill opacity=0.18, forget plot] fill between[of=${id}hi and ${id}lo];`,
    `  \\addplot[${color}, thick, mark=*, mark size=1.2pt${dashed ? ", dashed" : ""}] table[x=samples, ${y(`${metric}_median`)}] {\\EOneData/${file}};`,
    `  \\addlegendentry{${legend.replace(/__/g, "--").replace(/_/g, "\\_")}}`
  ].join("\n");
}

function figure({ title, ylabel, ymode, series, label, extraAxis = "" }) {
  return [
    "\\begin{figure}[htbp]",
    "\\centering",
    "\\begin{tikzpicture}",
    "\\begin{axis}[",
    "  width=\\linewidth, height=0.62\\linewidth,",
    "  xmode=log, log basis x=10,",
    ymode ? `  ymode=${ymode},` : "",
    "  xlabel={Joint samples per robot $N$}, " + `ylabel={${ylabel}},`,
    "  xtick={5000,10000,25000,50000,100000}, xticklabels={5k,10k,25k,50k,100k},",
    "  grid=major, grid style={gray!25},",
    "  legend cell align=left, legend columns=3,",
    "  legend style={font=\\footnotesize, at={(0.5,-0.2)}, anchor=north, draw=none},",
    "  unbounded coords=jump,",
    extraAxis,
    "]",
    series.join("\n"),
    "\\end{axis}",
    "\\end{tikzpicture}",
    `\\caption{${title}}`,
    `\\label{${label}}`,
    "\\end{figure}"
  ].filter((line) => line !== "").join("\n");
}

function pgfplotsFigures({ pairs, robots, band }) {
  const bandText = band === "minmax" ? "minimum to maximum" : "interquartile range";
  const colors = SERIES_RGB.map((rgb, index) => `\\definecolor{${SERIES_COLORS[index]}}{RGB}{${rgb}}`);
  const pairSeries = (metric, scale) => pairs.map((pair, index) => seriesBlock({
    file: `pairs/${pair.key}.csv`,
    metric,
    color: SERIES_COLORS[index % SERIES_COLORS.length],
    legend: pair.key,
    dashed: pair.degenerate,
    band,
    scale
  }));
  const robotSeries = robots.map((robot, index) => seriesBlock({
    file: `robots/${robot}.csv`,
    metric: "seconds",
    color: SERIES_COLORS[index % SERIES_COLORS.length],
    legend: robot,
    dashed: false,
    band
  }));
  return [
    "% Generated by src/eval/e1Convergence.js analyze -- do not edit, rerun instead.",
    "% Needs in the preamble: \\usepackage{pgfplots} \\usepgfplotslibrary{fillbetween}",
    "% Set \\EOneData to the directory holding pairs/ and robots/ before \\input-ing this file.",
    "\\providecommand{\\EOneData}{e1}",
    "\\pgfplotstableset{col sep=comma}",
    ...colors,
    "",
    figure({
      title: `Shared volume per robot pair over the number of joint samples, as computed by the deployed estimator (median over seeds, band: ${bandText}; dashed: degenerate pair).`,
      ylabel: "Shared volume [dm$^3$]",
      series: pairSeries("volumeAuto", 1000),
      label: "fig:e1-volume"
    }),
    "",
    figure({
      title: `Jaccard similarity of each pair's shared-voxel set to the pooled reference on a fixed grid (median over seeds, band: ${bandText}; dashed: degenerate pair).`,
      ylabel: "Jaccard similarity $J(V_N, V_{\\mathrm{ref}})$",
      series: pairSeries("jaccard", 1),
      label: "fig:e1-jaccard",
      extraAxis: "  ymin=0, ymax=1,"
    }),
    "",
    figure({
      title: `Workspace sampling time per robot over the number of joint samples (median over seeds, band: ${bandText}).`,
      ylabel: "Sampling time [s]",
      ymode: "log",
      series: robotSeries,
      label: "fig:e1-time"
    }),
    ""
  ].join("\n");
}

// ---------------------------------------------------------------- sample

async function sample({ descriptorPath, samplesDir, seeds, samples, robots, force, progressEvery }) {
  const bridge = require("../generator/robwotBridge");
  const descriptor = loadDescriptor(descriptorPath);
  const environmentScene = path.resolve(REPO_ROOT, descriptor.environmentScene);
  if (!fs.existsSync(environmentScene)) {
    throw new Error(`Environment scene missing: ${environmentScene}. Build the cell with --generate-shared-cell first.`);
  }
  if (samples % progressEvery !== 0) {
    throw new Error(`--samples ${samples} must be a multiple of the checkpoint interval ${progressEvery}.`);
  }
  const selected = robots ? new Set(robots) : null;
  const entries = descriptor.robots.filter((robot) => !selected || selected.has(robot.name));
  if (selected && entries.length !== selected.size) {
    throw new Error(`--robots names a robot that is not in the cell: ${[...selected].join(", ")}`);
  }
  const contexts = new Map(entries.map((robot) => [robot.name, bridge.createContext(path.resolve(REPO_ROOT, robot.manifest))]));
  const owner = bridge.createContext(path.resolve(REPO_ROOT, descriptor.robots[0].manifest));
  const sharedConfig = {
    collisionObstacles: descriptor.collisionObstacles || [],
    sharedGrip: bridge.gripSceneConfig(owner, descriptor.taskSites || [])
  };
  const { sim } = await bridge.connect(owner);

  for (const seed of seeds) {
    const seedDir = path.resolve(samplesDir, seedDirectoryName(seed));
    fs.mkdirSync(seedDir, { recursive: true });
    for (const robot of entries) {
      const runFile = path.join(seedDir, `${robot.name}_run.json`);
      if (!force && fs.existsSync(runFile)) {
        const previous = JSON.parse(fs.readFileSync(runFile, "utf8"));
        if (previous.samples === samples) {
          console.log(`seed ${seed} ${robot.name}: already sampled, skipped (--force to redo).`);
          continue;
        }
      }
      const ctx = contexts.get(robot.name);
      const experimentCtx = {
        ...ctx,
        infoDir: seedDir,
        paths: {
          ...ctx.paths,
          robotInfo: path.join(seedDir, `${robot.name}_info.json`),
          robotShape: path.join(seedDir, `${robot.name}_shape.stl`),
          robotDataPoints: path.join(seedDir, `${robot.name}_data_point.csv`)
        }
      };
      const orientation = (robot.orientationDeg || [0, 0, Number(robot.rotationDeg || 0)]).map((value) => Number(value) * Math.PI / 180);
      console.log(`seed ${seed} ${robot.name}: sampling ${samples} configurations.`);
      const info = await bridge.calculateRobotInSharedEnvironment(
        sim, experimentCtx, robot.offset || [0, 0, 0], orientation, environmentScene, sharedConfig,
        {
          generateTd: false,
          samplingOverrides: {
            workspaceRandomSteps: samples,
            workspaceSeed: seed,
            workspaceProgressLog: true,
            workspaceProgressEvery: progressEvery
          }
        }
      );
      // The shape export is a by-product of the vendor sampler and 10-40 MB a run.
      fs.rmSync(experimentCtx.paths.robotShape, { force: true });
      fs.writeFileSync(runFile, JSON.stringify({
        seed,
        samples,
        robot: robot.name,
        offset: robot.offset,
        orientationDeg: robot.orientationDeg,
        wallSeconds: info.womatSampling?.seconds,
        obstacles: info.womatSampling?.culling,
        finishedAt: new Date().toISOString()
      }, null, 2) + "\n");
    }
  }
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
  "  node src/eval/e1Convergence.js sample  [--seeds 1-10] [--samples 100000] [--robots Robot1,Robot2]",
  "                                         [--descriptor <cell>.shared.json] [--samples-dir <dir>] [--force]",
  "  node src/eval/e1Convergence.js analyze [--counts 5000,10000,25000,50000,100000] [--band iqr|minmax]",
  "                                         [--descriptor <cell>.shared.json] [--samples-dir <dir>] [--out <dir>]",
  "",
  `Defaults: descriptor ${DEFAULT_DESCRIPTOR}, samples ${DEFAULT_SAMPLES_DIR}, results ${DEFAULT_RESULTS_DIR}.`,
  "sample needs CoppeliaSim running; it resumes, skipping robots a seed already has."
].join("\n");

async function main(argv) {
  const args = cliArguments(argv);
  const command = args._[0];
  const descriptorPath = path.resolve(REPO_ROOT, args.descriptor || DEFAULT_DESCRIPTOR);
  const samplesDir = path.resolve(REPO_ROOT, args["samples-dir"] || DEFAULT_SAMPLES_DIR);
  if (command === "sample") {
    await sample({
      descriptorPath,
      samplesDir,
      seeds: parseSeeds(args.seeds || "1-10"),
      samples: Number(args.samples || 100000),
      robots: args.robots ? String(args.robots).split(",").map((name) => name.trim()).filter(Boolean) : null,
      force: args.force === true,
      progressEvery: Number(args["progress-every"] || 1000)
    });
    console.log("E1 sampling complete.");
    return 0;
  }
  if (command === "analyze") {
    const band = args.band || "iqr";
    if (!["iqr", "minmax"].includes(band)) throw new Error("--band must be iqr or minmax.");
    const result = analyze({
      descriptorPath,
      samplesDir,
      outDir: path.resolve(REPO_ROOT, args.out || DEFAULT_RESULTS_DIR),
      counts: parseCounts(args.counts || DEFAULT_COUNTS.join(",")),
      band
    });
    console.log(`Seeds analysed: ${result.seeds.join(", ")}; reference built from ${result.maxCount} samples per robot per seed.`);
    for (const pair of result.pairs) {
      console.log(`  ${pair.pair}: ${pair.voxelSizeRef === null ? "never shares a voxel" : `s_ref ${pair.voxelSizeRef.toFixed(4)} m, `
        + `|V_ref| ${pair.voxelCountRef}${pair.degenerate ? " (degenerate)" : ""}`}`);
    }
    console.log(`Tables and figures: ${result.outDir}`);
    return 0;
  }
  console.error(USAGE);
  return 1;
}

if (require.main === module) {
  main(process.argv.slice(2)).then((code) => process.exit(code), (error) => {
    console.error(error);
    process.exit(1);
  });
}

module.exports = {
  allPairs,
  analyze,
  checkpointAt,
  fixedGridKey,
  intersection,
  nestedOccupiedVoxels,
  occupiedVoxels,
  overlapMetrics,
  parseCounts,
  parseProgress,
  parseSeeds,
  parseWorkspaceRows,
  pgfplotsFigures,
  reachablePrefix,
  seedDirectoryName,
  summarize,
  toCsv
};
