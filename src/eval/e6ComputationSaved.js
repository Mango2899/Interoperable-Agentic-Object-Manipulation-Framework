#!/usr/bin/env node
// E6 -- how much computation does the neighbour test save?
//
// E5 (e5Neighbours.js) asked whether the test is CORRECT: it never rejects a
// pair that shares space, and it pays for that with false positives. E6 asks
// what the filter is WORTH: how many interaction spaces are never computed,
// and what that costs or saves in seconds.
//
// The answer has two halves, and they disagree, which is the point:
//
//   offline   The estimator itself is cheap on a rejected pair -- it finds the
//             bounding boxes disjoint and returns before it voxelizes anything
//             (~0.11 s). The expensive computations, the empty ones at tens of
//             seconds, are exactly the pairs the test ADMITS. Measured in pure
//             computation the filter therefore saves little.
//   live      In the running cell the same computation is preceded by a cloud
//             request and the transfer of two point clouds over HTTP, and that
//             is what the verdict actually avoids: a rejected pair never asks.
//             performance_log/interaction-compute.jsonl holds those end-to-end
//             times from the deployed system.
//
// Both are reported, because quoting only the first understates the filter and
// quoting only the second overstates it.
//
// Three populations:
//
//   cells       the two-, three-, four-, and five-robot cells, from the per-pair
//               measurements E5 already made (performance_log/e5/e5_pairs.csv,
//               ten seeds, every pair including the rejected ones -- without the
//               rejected ones the counterfactual "no filter" cannot be priced).
//               --measure re-runs those measurements here instead of reading them.
//   live        the interaction-space computations the running cell logged,
//               split into cold (cloud request + transfer + voxelization),
//               cached, and failed.
//   lines       synthetic cells of n robots on a line at the pitch of the real
//               cell. No voxelization: the admitted pairs follow from the base
//               positions and the widened radii alone, which is what makes
//               n = 20 affordable. This is the scaling claim -- n(n-1)/2 pairs
//               grow quadratically, the admitted ones roughly linearly.
//
// Output: CSV tables, a pgfplots figure and a LaTeX table for the thesis.

const fs = require("node:fs");
const path = require("node:path");

const { couldOverlap } = require("../generator/neighborTest");
const { parseSeeds, summarize, toCsv } = require("./e1Convergence");
const {
  descriptorForCell,
  parseIntegerList,
  runCells,
  signatureAt
} = require("./e5Neighbours");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const DEFAULT_PAIRS_CSV = "performance_log/e5/e5_pairs.csv";
const DEFAULT_LIVE_LOG = "performance_log/interaction-compute.jsonl";
const DEFAULT_SAMPLES_DIR = "generated/experiments/e1/samples";
const DEFAULT_RESULTS_DIR = "performance_log/e6";
const DEFAULT_CELLS = [2, 3, 4, 5];
const DEFAULT_LINE_SIZES = [5, 10, 20];
// The figure wants a curve, not three points, so every n up to the largest
// requested line is evaluated and the requested ones are merely marked.
const LINE_CURVE_MAX = 20;

// ---------------------------------------------------------------- pure parts

function isTruthy(value) {
  const token = String(value).trim().toLowerCase();
  return token === "1" || token === "true" || token === "yes";
}

// toCsv writes booleans as 1/0 and nulls as empty, so the reader has to undo
// exactly that and nothing more.
function parseCsv(text) {
  const lines = text.split("\n").filter((line) => line.trim() !== "");
  if (lines.length < 2) throw new Error("The pair table has no rows.");
  const columns = lines[0].split(",").map((name) => name.trim());
  return lines.slice(1).map((line) => {
    const cells = line.split(",");
    const row = {};
    columns.forEach((name, index) => { row[name] = cells[index] === undefined ? "" : cells[index]; });
    return row;
  });
}

function numberOf(value) {
  if (value === "" || value === undefined || value === null) return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

// One pair evaluation, however it was obtained: from the E5 table or from a
// fresh measurement. `class` is what E6 prices things by.
function normalizeRun(row) {
  const neighbor = typeof row.neighbor === "boolean" ? row.neighbor : isTruthy(row.neighbor);
  const sharedVoxels = Number(row.sharedVoxels) || 0;
  return {
    cell: Number(row.cell),
    seed: Number(row.seed),
    pair: row.pair,
    neighbor,
    sharedVoxels,
    reason: row.reason || "",
    voxelizeMs: numberOf(row.voxelizeMs),
    testUs: numberOf(row.testUs),
    class: !neighbor ? "rejected" : (sharedVoxels >= 1 ? "admitted_shares" : "admitted_empty")
  };
}

function loadPairRuns(pairsCsv) {
  if (!fs.existsSync(pairsCsv)) {
    throw new Error(`No pair table at ${pairsCsv}. Run "npm run eval:e5" first, or pass --measure to measure the pairs here.`);
  }
  const runs = parseCsv(fs.readFileSync(pairsCsv, "utf8")).map(normalizeRun);
  if (runs.some((run) => !Number.isFinite(run.cell) || run.voxelizeMs === null)) {
    throw new Error(`${pairsCsv} is not an E5 pair table (cell/voxelizeMs missing).`);
  }
  return runs;
}

// ---------------------------------------------------------------- cost classes

// What one pair costs, by the class the test put it in. The rejected class is
// the counterfactual: it is what the pair WOULD have cost had nobody filtered.
function costClasses(runs) {
  const labels = {
    admitted_shares: "admitted, shares space",
    admitted_empty: "admitted, empty",
    rejected: "rejected (never computed in operation)"
  };
  return Object.keys(labels).map((name) => {
    const group = runs.filter((run) => run.class === name);
    const stats = summarize(group.map((run) => run.voxelizeMs));
    const verdict = summarize(runs.map((run) => run.testUs));
    return {
      class: name,
      description: labels[name],
      n: group.length,
      voxelizeMs_median: stats.median,
      voxelizeMs_q25: stats.q25,
      voxelizeMs_q75: stats.q75,
      voxelizeMs_min: stats.min,
      voxelizeMs_max: stats.max,
      voxelizeMs_mean: stats.mean,
      testUs_median: verdict.median
    };
  }).filter((row) => row.n > 0);
}

// The budget of one cell in one seed: what the pipeline pays with the filter,
// and what it would have paid without it. The verdicts are paid in both, since
// the pipeline runs the test on every pair either way; they are microseconds
// against seconds, which is the result.
function cellBudget(runs) {
  const cells = [...new Set(runs.map((run) => run.cell))].sort((left, right) => left - right);
  const rows = [];
  for (const cell of cells) {
    const cellRuns = runs.filter((run) => run.cell === cell);
    const seeds = [...new Set(cellRuns.map((run) => run.seed))].sort((left, right) => left - right);
    const perSeed = seeds.map((seed) => {
      const group = cellRuns.filter((run) => run.seed === seed);
      const admitted = group.filter((run) => run.neighbor);
      const verdictMs = group.reduce((sum, run) => sum + (run.testUs || 0) / 1000, 0);
      const withFilter = admitted.reduce((sum, run) => sum + run.voxelizeMs, 0) + verdictMs;
      const withoutFilter = group.reduce((sum, run) => sum + run.voxelizeMs, 0);
      return {
        seed,
        pairs: group.length,
        admitted: admitted.length,
        rejected: group.length - admitted.length,
        verdictMs,
        withFilter,
        withoutFilter,
        savedMs: withoutFilter - withFilter,
        savedShare: withoutFilter === 0 ? null : (withoutFilter - withFilter) / withoutFilter
      };
    });
    const first = perSeed[0];
    const stat = (key) => summarize(perSeed.map((entry) => entry[key]));
    rows.push({
      cell,
      robots: cell,
      seeds: perSeed.length,
      // n(n-1)/2 -- what a cell without a filter computes, by definition.
      allPairs: first.pairs,
      admittedPairs: first.admitted,
      rejectedPairs: first.rejected,
      admittedShare: first.admitted / first.pairs,
      verdictMs_median: stat("verdictMs").median,
      withFilterMs_median: stat("withFilter").median,
      withoutFilterMs_median: stat("withoutFilter").median,
      savedMs_median: stat("savedMs").median,
      savedShare_median: stat("savedShare").median
    });
  }
  return rows;
}

// ---------------------------------------------------------------- live log

// The deployed cell's own record. A line without `cached` is a computation that
// did not produce an interaction space: the pair was admitted by the test and
// the estimator then found nothing, which is the expensive failure mode.
function classifyLive(entry) {
  if (entry.cached === true) return "cached";
  if (entry.cached === false) return "cold";
  if (entry.reason === "no_geometric_overlap") return "failed_bbox";
  return "failed_empty";
}

function loadLive(logPath) {
  if (!fs.existsSync(logPath)) return null;
  const entries = fs.readFileSync(logPath, "utf8")
    .split("\n")
    .filter((line) => line.trim() !== "")
    .map((line) => { try { return JSON.parse(line); } catch { return null; } })
    .filter((entry) => entry && typeof entry.pairKey === "string" && Number.isFinite(entry.durationMs));
  return entries.length === 0 ? null : entries;
}

function liveRows(entries) {
  const rows = [];
  const classes = ["cold", "cached", "failed_empty", "failed_bbox"];
  const pairs = [...new Set(entries.map((entry) => entry.pairKey))].sort();
  for (const pair of [...pairs, "__all__"]) {
    for (const name of classes) {
      const group = entries.filter((entry) =>
        (pair === "__all__" || entry.pairKey === pair) && classifyLive(entry) === name);
      if (group.length === 0) continue;
      const stats = summarize(group.map((entry) => entry.durationMs));
      rows.push({
        pair: pair === "__all__" ? "all pairs" : pair,
        class: name,
        n: group.length,
        durationMs_median: stats.median,
        durationMs_q25: stats.q25,
        durationMs_q75: stats.q75,
        durationMs_min: stats.min,
        durationMs_max: stats.max,
        voxels_median: summarize(group.map((entry) => Number(entry.voxelCount) || 0)).median
      });
    }
  }
  return rows;
}

// What the test saved in the running cell: the pairs of the largest cell that
// never appear in the log at all, priced at the median cold computation. A pair
// the test rejects never asks its neighbour for a cloud, so its absence from
// the log IS the saving.
function liveSaving(entries, runs, largestCell) {
  const seen = new Set(entries.map((entry) => entry.pairKey));
  const cellRuns = runs.filter((run) => run.cell === largestCell);
  const pairs = [...new Set(cellRuns.map((run) => run.pair))];
  const rejected = pairs.filter((pair) => cellRuns.some((run) => run.pair === pair && !run.neighbor));
  const coldMs = summarize(entries.filter((entry) => classifyLive(entry) === "cold").map((entry) => entry.durationMs));
  const failedMs = summarize(entries.filter((entry) => classifyLive(entry) === "failed_empty").map((entry) => entry.durationMs));
  return {
    cell: largestCell,
    pairs: pairs.length,
    rejectedPairs: rejected.length,
    rejectedNeverRequested: rejected.filter((pair) => !seen.has(pair)).length,
    rejectedPairKeys: rejected.join(" "),
    coldMs_median: coldMs.median,
    failedMs_median: failedMs.median,
    // Priced at the cold median, because that is what a first request costs.
    savedMs_median: rejected.filter((pair) => !seen.has(pair)).length * (coldMs.median || 0)
  };
}

// ---------------------------------------------------------------- scaling

// The widened radii of the real arms, median over the seeds E5 wrote. Reading
// them from the E5 signature table keeps E6 offline and consistent with E5.
function reachRadii(signaturesCsv) {
  const rows = parseCsv(fs.readFileSync(signaturesCsv, "utf8"));
  const byRobot = new Map();
  for (const row of rows) {
    if (!byRobot.has(row.robot)) byRobot.set(row.robot, []);
    byRobot.get(row.robot).push(Number(row.safeReachRadius));
  }
  return [...byRobot.entries()]
    .sort((left, right) => left[0].localeCompare(right[0]))
    .map(([robot, values]) => ({ robot, safeReachRadius: summarize(values).median }));
}

// The pitch of the real cell: the median distance from a base to its nearest
// neighbour. A line at that pitch is as dense as the cell that was actually
// built, so the scaling curve is not an arbitrary geometry.
function nearestNeighbourPitch(descriptor) {
  const bases = descriptor.robots.map((robot) => (robot.offset || [0, 0, 0]).map(Number));
  const nearest = bases.map((here, index) => Math.min(...bases
    .filter((_, other) => other !== index)
    .map((there) => Math.hypot(here[0] - there[0], here[1] - there[1], here[2] - there[2]))));
  return summarize(nearest).median;
}

// n robots on a line at `pitch`, radii cycled through the measured arms so the
// mix of long and short reaches is the real one. The verdict is the deployed
// couldOverlap, so this is the same test, only on a bigger cell.
function lineCell(n, pitch, radii) {
  const signatures = Array.from({ length: n }, (_, index) => signatureAt({
    ...radii[index % radii.length],
    reachRadius: radii[index % radii.length].safeReachRadius,
    spacing: 0
  }, [index * pitch, 0, 0]));
  let admitted = 0;
  let margins = [];
  for (let first = 0; first < n; first += 1) {
    for (let second = first + 1; second < n; second += 1) {
      const verdict = couldOverlap(signatures[first], signatures[second]);
      if (verdict.neighbor) {
        admitted += 1;
        margins.push(verdict.margin);
      }
    }
  }
  const allPairs = (n * (n - 1)) / 2;
  return {
    robots: n,
    pitch,
    allPairs,
    admittedPairs: admitted,
    rejectedPairs: allPairs - admitted,
    admittedShare: allPairs === 0 ? null : admitted / allPairs,
    // The quantity the scaling claim is about: a robot has a bounded number of
    // neighbours within twice its reach, so this settles instead of growing.
    admittedPerRobot: n === 0 ? null : (2 * admitted) / n,
    margin_median: summarize(margins).median
  };
}

function lineCurve({ sizes, pitch, radii, coldMs }) {
  const largest = Math.max(LINE_CURVE_MAX, ...sizes);
  const rows = [];
  for (let n = 2; n <= largest; n += 1) {
    const row = lineCell(n, pitch, radii);
    rows.push({
      ...row,
      requested: sizes.includes(n),
      // What the two policies would cost at the measured price of one live
      // computation. Extrapolation, and labelled as such in the table.
      withFilterS: coldMs === null ? null : (row.admittedPairs * coldMs) / 1000,
      withoutFilterS: coldMs === null ? null : (row.allPairs * coldMs) / 1000
    });
  }
  return rows;
}

// ---------------------------------------------------------------- analyse

function analyze(options) {
  const {
    cells, seeds, pairsCsv, signaturesCsv, liveLog, samplesDir, outDir, lineSizes, pitchOverride, measure
  } = options;

  const descriptorPath = path.resolve(REPO_ROOT, descriptorForCell(Math.max(...cells)));
  const descriptor = JSON.parse(fs.readFileSync(descriptorPath, "utf8"));

  let runs;
  let source;
  if (measure) {
    const measured = runCells({
      cells,
      seeds,
      samplesDir,
      estimatorOptions: descriptor.interactionSpace || {}
    });
    runs = measured.runs.map(normalizeRun);
    source = `measured here over seeds ${measured.seeds.join(", ")}`;
  } else {
    runs = loadPairRuns(pairsCsv).filter((run) => cells.includes(run.cell)
      && (seeds === null || seeds.includes(run.seed)));
    if (runs.length === 0) throw new Error(`${pairsCsv} holds no rows for cells ${cells.join(",")}.`);
    source = path.relative(REPO_ROOT, pairsCsv);
  }

  const classes = costClasses(runs);
  const budget = cellBudget(runs);
  const entries = loadLive(liveLog);
  const live = entries ? liveRows(entries) : [];
  const saving = entries ? liveSaving(entries, runs, Math.max(...cells)) : null;
  const coldMs = entries
    ? summarize(entries.filter((entry) => classifyLive(entry) === "cold").map((entry) => entry.durationMs)).median
    : null;

  const radii = reachRadii(signaturesCsv);
  const pitch = pitchOverride === null ? nearestNeighbourPitch(descriptor) : pitchOverride;
  const scaling = lineCurve({ sizes: lineSizes, pitch, radii, coldMs });

  fs.mkdirSync(outDir, { recursive: true });
  fs.writeFileSync(path.join(outDir, "e6_cost_classes.csv"), toCsv(classes, Object.keys(classes[0])));
  fs.writeFileSync(path.join(outDir, "e6_cells.csv"), toCsv(budget, Object.keys(budget[0])));
  fs.writeFileSync(path.join(outDir, "e6_scaling.csv"), toCsv(scaling,
    ["robots", "pitch", "allPairs", "admittedPairs", "rejectedPairs", "admittedShare",
      "admittedPerRobot", "margin_median", "requested", "withFilterS", "withoutFilterS"]));
  if (live.length > 0) {
    fs.writeFileSync(path.join(outDir, "e6_live.csv"), toCsv(live, Object.keys(live[0])));
  }
  if (saving) {
    fs.writeFileSync(path.join(outDir, "e6_live_saving.csv"), toCsv([saving], Object.keys(saving)));
  }
  fs.writeFileSync(path.join(outDir, "e6_figure.tex"), scalingFigure({ lineSizes, pitch }));
  fs.writeFileSync(path.join(outDir, "e6_table.tex"), costTable({ budget, classes, scaling, lineSizes, coldMs }));
  fs.writeFileSync(path.join(outDir, "e6_setup.json"), JSON.stringify({
    cells,
    seeds: [...new Set(runs.map((run) => run.seed))].sort((left, right) => left - right),
    pairSource: source,
    liveLog: entries ? path.relative(REPO_ROOT, liveLog) : null,
    liveEntries: entries ? entries.length : 0,
    lineSizes,
    linePitch: pitch,
    linePitchSource: pitchOverride === null ? "median nearest-neighbour base distance of the largest cell" : "--pitch",
    reachRadii: radii,
    finishedAt: new Date().toISOString()
  }, null, 2) + "\n");

  return { outDir, runs: runs.length, classes, budget, live, saving, scaling, pitch, coldMs, source };
}

// ---------------------------------------------------------------- pgfplots

function scalingFigure({ lineSizes, pitch }) {
  return [
    "% Generated by src/eval/e6ComputationSaved.js analyze -- do not edit, rerun instead.",
    "% Needs in the preamble: \\usepackage{pgfplots}",
    "% Set \\ESixData to the directory holding e6_scaling.csv before \\input-ing this file.",
    "\\providecommand{\\ESixData}{e6}",
    "\\pgfplotstableset{col sep=comma}",
    "",
    "\\begin{figure}[htbp]",
    "\\centering",
    "\\begin{tikzpicture}",
    "\\begin{axis}[",
    "  width=0.86\\linewidth, height=0.6\\linewidth,",
    "  xlabel={Robots in the cell $n$}, ylabel={Interaction spaces computed},",
    `  xmin=2, xmax=${Math.max(LINE_CURVE_MAX, ...lineSizes)}, ymin=0,`,
    "  grid=major, grid style={gray!25},",
    "  legend cell align=left,",
    "  legend style={font=\\footnotesize, at={(0.03,0.97)}, anchor=north west, draw=none, fill=white, fill opacity=0.8, text opacity=1},",
    "]",
    "  \\addplot[black, thick, mark=none] table[x=robots, y=allPairs] {\\ESixData/e6_scaling.csv};",
    "  \\addlegendentry{without the test, $n(n-1)/2$}",
    "  \\addplot[blue, thick, mark=*, mark size=1.5pt] table[x=robots, y=admittedPairs] {\\ESixData/e6_scaling.csv};",
    "  \\addlegendentry{admitted by the test}",
    "\\end{axis}",
    "\\end{tikzpicture}",
    `\\caption{E6: Interaction-space computations with and without the neighbour test, for cells of $n$ robots on a line at the pitch of the five-robot cell (${pitch.toFixed(2)}\\,m). The pairs to compute grow as $n(n-1)/2$; the pairs the test admits grow roughly linearly, because a robot has a bounded number of neighbours within twice its reach.}`,
    "\\label{fig:exp_e6}",
    "\\end{figure}",
    ""
  ].join("\n");
}

function texNumber(value, digits = 2) {
  if (value === null || value === undefined || !Number.isFinite(value)) return "--";
  // A cell in which the test rejects nothing still pays for the verdicts, so
  // the saving comes out at -1e-7 rather than 0. Printing "-0.00" would claim a
  // loss the numbers do not support.
  const rounded = Number(value.toFixed(digits));
  return (rounded === 0 ? 0 : rounded).toFixed(digits);
}

function costTable({ budget, classes, scaling, lineSizes, coldMs }) {
  const cellRows = budget.map((row) => [
    `${row.robots}-robot cell`,
    row.allPairs,
    row.admittedPairs,
    row.rejectedPairs,
    texNumber(row.withoutFilterMs_median / 1000),
    texNumber(row.withFilterMs_median / 1000),
    texNumber(row.savedShare_median === null ? null : row.savedShare_median * 100, 1)
  ].join(" & ") + " \\\\");
  const lineRows = scaling
    .filter((row) => lineSizes.includes(row.robots))
    .map((row) => [
      `line of ${row.robots}`,
      row.allPairs,
      row.admittedPairs,
      row.rejectedPairs,
      texNumber(row.withoutFilterS),
      texNumber(row.withFilterS),
      texNumber(row.admittedShare === null ? null : (1 - row.admittedShare) * 100, 1)
    ].join(" & ") + " \\\\");
  const classRows = classes.map((row) => [
    row.description,
    row.n,
    texNumber(row.voxelizeMs_median / 1000),
    texNumber(row.voxelizeMs_q25 / 1000),
    texNumber(row.voxelizeMs_q75 / 1000)
  ].join(" & ") + " \\\\");
  return [
    "% Generated by src/eval/e6ComputationSaved.js analyze -- do not edit, rerun instead.",
    "\\begin{table}[htbp]",
    "    \\centering",
    `    \\caption{E6: Interaction-space computations avoided by the neighbour test. The configured cells are measured over the seeds of E5; the lines are synthetic cells of $n$ robots at the pitch of the five-robot cell, whose admitted pairs follow from the base positions and the widened radii alone and whose times are extrapolated from the median cold computation of the running cell (${texNumber(coldMs === null ? null : coldMs / 1000)}\\,s). One verdict costs ${texNumber((classes[0] ? classes[0].testUs_median : null), 3)}\\,\\textmu s.}`,
    "    \\label{tab:exp_e6}",
    "    \\small",
    "    \\begin{tabular}{@{}lrrrrrr@{}}",
    "        \\toprule",
    "        \\textbf{Cell} & \\textbf{Pairs} & \\textbf{Admitted} & \\textbf{Rejected} & \\textbf{No test [s]} & \\textbf{With test [s]} & \\textbf{Saved [\\%]} \\\\",
    "        \\midrule",
    ...cellRows.map((line) => `        ${line}`),
    "        \\midrule",
    ...lineRows.map((line) => `        ${line}`),
    "        \\bottomrule",
    "    \\end{tabular}",
    "",
    "    \\vspace{1em}",
    "    \\begin{tabular}{@{}lrrrr@{}}",
    "        \\toprule",
    "        \\textbf{Interaction space of a pair the test} & $n$ & \\textbf{Median [s]} & \\textbf{Q25 [s]} & \\textbf{Q75 [s]} \\\\",
    "        \\midrule",
    ...classRows.map((line) => `        ${line}`),
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
  "  node src/eval/e6ComputationSaved.js analyze [--cells 2,3,4,5] [--seeds 1-10] [--lines 5,10,20]",
  "                                              [--pairs <e5_pairs.csv>] [--signatures <e5_signatures.csv>]",
  "                                              [--live <interaction-compute.jsonl>] [--pitch <m>]",
  "                                              [--measure] [--samples-dir <dir>] [--out <dir>]",
  "",
  `Defaults: cells ${DEFAULT_CELLS.join(",")}, lines ${DEFAULT_LINE_SIZES.join(",")}, pairs ${DEFAULT_PAIRS_CSV}, live ${DEFAULT_LIVE_LOG}, results ${DEFAULT_RESULTS_DIR}.`,
  "Offline and fast: it reuses the per-pair times E5 measured and the log the running cell wrote.",
  "--measure re-measures the pairs instead of reading them (needs the E1 clouds, ~10 min)."
].join("\n");

function main(argv) {
  const args = cliArguments(argv);
  if (args._[0] !== "analyze") {
    console.error(USAGE);
    return 1;
  }
  const pairsCsv = path.resolve(REPO_ROOT, args.pairs || DEFAULT_PAIRS_CSV);
  const result = analyze({
    cells: args.cells ? parseIntegerList(args.cells, "Cell sizes") : DEFAULT_CELLS,
    seeds: args.seeds ? parseSeeds(args.seeds) : null,
    lineSizes: args.lines ? parseIntegerList(args.lines, "Line sizes") : DEFAULT_LINE_SIZES,
    pairsCsv,
    signaturesCsv: path.resolve(REPO_ROOT, args.signatures
      || path.join(path.dirname(pairsCsv), "e5_signatures.csv")),
    liveLog: path.resolve(REPO_ROOT, args.live || DEFAULT_LIVE_LOG),
    samplesDir: path.resolve(REPO_ROOT, args["samples-dir"] || DEFAULT_SAMPLES_DIR),
    outDir: path.resolve(REPO_ROOT, args.out || DEFAULT_RESULTS_DIR),
    pitchOverride: args.pitch === undefined ? null : Number(args.pitch),
    measure: args.measure === true
  });

  console.log(`\nE6: ${result.runs} pair evaluation(s) from ${result.source}.`);
  console.log("  Cost of one interaction space, by what the test said:");
  for (const row of result.classes) {
    console.log(`    ${row.description.padEnd(38)} n=${String(row.n).padStart(3)}  `
      + `median ${(row.voxelizeMs_median / 1000).toFixed(2)} s `
      + `[${(row.voxelizeMs_q25 / 1000).toFixed(2)}, ${(row.voxelizeMs_q75 / 1000).toFixed(2)}]`);
  }
  console.log(`  One verdict: ${result.classes[0].testUs_median.toFixed(3)} us.`);
  console.log("  Configured cells (offline computation only):");
  for (const row of result.budget) {
    console.log(`    ${row.robots}-robot: ${row.admittedPairs}/${row.allPairs} pairs computed, `
      + `${(row.withoutFilterMs_median / 1000).toFixed(1)} s -> ${(row.withFilterMs_median / 1000).toFixed(1)} s `
      + `(${texNumber((row.savedShare_median || 0) * 100, 1)}% saved)`);
  }
  if (result.saving) {
    console.log(`  Running cell: ${result.saving.rejectedNeverRequested} of ${result.saving.pairs} pairs never `
      + `asked for a cloud at all (${result.saving.rejectedPairKeys || "none"}), `
      + `worth ${(result.saving.savedMs_median / 1000).toFixed(1)} s at the median cold computation `
      + `of ${(result.saving.coldMs_median / 1000).toFixed(2)} s.`);
  }
  console.log(`  Lines at ${result.pitch.toFixed(2)} m pitch:`);
  for (const row of result.scaling.filter((entry) => entry.requested)) {
    console.log(`    n=${String(row.robots).padStart(2)}: ${row.admittedPairs}/${row.allPairs} pairs admitted `
      + `(${((1 - row.admittedShare) * 100).toFixed(1)}% avoided, ${row.admittedPerRobot.toFixed(1)} neighbours per robot)`);
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
  reachRadii,
  scalingFigure
};
