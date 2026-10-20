"use strict";

// Who computed which interaction space, and whether they had to.
//
// Once neighbours are discovered rather than configured and every agent could
// in principle compute any pair it takes part in, "was this pair computed once
// or twice?" stops being obvious from watching the console. This records it:
// one line per attempt, from whichever agent made it, with whether the answer
// came from cache or from work.
//
// Appended as JSON lines so several agent processes can write one file; a short
// line to an O_APPEND descriptor is not interleaved on Linux.

const fs = require("fs");
const path = require("path");

const { REPO_ROOT } = require("./core/context");

const DEFAULT_FILE = "performance_log/interaction-compute.jsonl";

class ComputeLog {
  constructor(options = {}) {
    const { agent, file = null, enabled = true } = options;
    this.agent = agent;
    this.file = file;
    this.enabled = Boolean(enabled && file);
    this.silent = Boolean(options.silent);
    if (this.enabled) fs.mkdirSync(path.dirname(file), { recursive: true });
  }

  record(entry) {
    const line = { t: Date.now(), agent: this.agent, ...entry };
    if (this.silent) return line;
    // Always visible next to the MAS output, whether or not a file is
    // configured: the point is to be able to watch who did what.
    const detail = entry.cached === undefined
      ? ""
      : ` ${entry.cached ? "CACHED" : "COMPUTED"} ${entry.voxelCount ?? "?"} voxels in ${Math.round(entry.durationMs || 0)}ms`;
    const why = entry.diagnostics
      ? ` [${Object.entries(entry.diagnostics).map(([key, value]) => `${key}=${value}`).join(" ")}]`
      : "";
    console.log(`[Interaction:${this.agent}] ${entry.pairKey || entry.targetRobot}${detail}${entry.reason ? ` (${entry.reason})` : ""}${why}`);
    if (!this.enabled) return line;
    try {
      fs.appendFileSync(this.file, `${JSON.stringify(line)}\n`);
    } catch (error) {
      this.enabled = false;
      console.warn(`[Interaction:${this.agent}] compute log disabled: ${error?.message || error}`);
    }
    return line;
  }
}

function computeLogFor(ctx) {
  // A test that exercises a compute path must not append to the cell's real
  // log. The suite's stub controllers report invented versions and voxel counts
  // ("abc123", 1000 voxels), and those rows are indistinguishable from a real
  // run once they are in the file -- they showed up as six duplicate computes
  // of a pair no agent had touched.
  if (process.env.NODE_TEST_CONTEXT && !process.env.WOMAT_COMPUTE_LOG) {
    return new ComputeLog({ agent: ctx?.manifest?.name || "test", file: null, enabled: false, silent: true });
  }
  const configured = process.env.WOMAT_COMPUTE_LOG || DEFAULT_FILE;
  // A context assembled by hand for a test carries no repoRoot; the canonical
  // one is correct in every case and never undefined.
  const root = ctx?.repoRoot || REPO_ROOT;
  const file = path.isAbsolute(configured) ? configured : path.resolve(root, configured);
  return new ComputeLog({ agent: ctx?.manifest?.name || "unknown", file, enabled: true });
}

// Who did the work, and how much of it was duplicated.
function computeReport(file) {
  const rows = fs.readFileSync(file, "utf8").split("\n").filter((line) => line.trim())
    .map((line) => JSON.parse(line));
  const byPair = new Map();
  for (const row of rows) {
    if (!row.pairKey) continue;
    if (!byPair.has(row.pairKey)) byPair.set(row.pairKey, []);
    byPair.get(row.pairKey).push(row);
  }
  const pairs = [...byPair].map(([pairKey, entries]) => {
    const computed = entries.filter((entry) => entry.cached === false);
    return {
      pairKey,
      attempts: entries.length,
      computedBy: computed.map((entry) => entry.agent),
      servedFromCache: entries.filter((entry) => entry.cached === true).map((entry) => entry.agent),
      voxelCount: entries.find((entry) => entry.voxelCount !== undefined)?.voxelCount ?? null,
      workMs: computed.reduce((total, entry) => total + (entry.durationMs || 0), 0),
      // More than one agent doing the work for the same pair is the thing this
      // log exists to make visible.
      duplicated: computed.length > 1
    };
  }).sort((left, right) => left.pairKey.localeCompare(right.pairKey));
  return {
    file,
    pairs,
    totalWorkMs: pairs.reduce((total, pair) => total + pair.workMs, 0),
    duplicatedPairs: pairs.filter((pair) => pair.duplicated).map((pair) => pair.pairKey)
  };
}

function formatComputeReport(report) {
  const lines = [`Interaction-space computation: ${report.file}`, "",
    "pair                 voxels  attempts  computed by        served from cache"];
  for (const pair of report.pairs) {
    lines.push(
      `${pair.pairKey.padEnd(20)} ${String(pair.voxelCount ?? "-").padStart(6)} `
      + `${String(pair.attempts).padStart(9)}  ${(pair.computedBy.join(", ") || "-").padEnd(18)} `
      + `${pair.servedFromCache.join(", ") || "-"}`
    );
  }
  lines.push("", `  total computation ${(report.totalWorkMs / 1000).toFixed(2)}s`);
  lines.push(report.duplicatedPairs.length === 0
    ? "  no pair was computed twice"
    : `  DUPLICATED: ${report.duplicatedPairs.join(", ")}`);
  return lines.join("\n");
}

if (require.main === module) {
  const args = process.argv.slice(2);
  const index = args.indexOf("--file");
  const file = path.resolve(process.cwd(), index === -1 ? DEFAULT_FILE : args[index + 1]);
  console.log(args.includes("--json")
    ? JSON.stringify(computeReport(file), null, 2)
    : formatComputeReport(computeReport(file)));
}

module.exports = { ComputeLog, DEFAULT_FILE, computeLogFor, computeReport, formatComputeReport };
