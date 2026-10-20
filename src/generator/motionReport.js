#!/usr/bin/env node
"use strict";

// What a run actually did, from the motion log.
//
// The number this exists to produce is CONCURRENT MOTION TIME: how long two or
// more arms were moving at once. Under a global mutex it is zero by
// construction, which is the baseline Phase 4 has to beat. Everything else here
// is context for it -- makespan, per-arm busy time, and how long arms spent
// waiting for permission rather than moving.

const fs = require("fs");
const path = require("path");

function readRecords(file) {
  return fs.readFileSync(file, "utf8").split("\n")
    .filter((line) => line.trim().length > 0)
    .map((line) => {
      try {
        return JSON.parse(line);
      } catch (error) {
        return null;
      }
    })
    .filter(Boolean);
}

// Motions become half-open intervals. A "finished" without a start is a motion
// that was refused before it moved, and contributes wait but no motion time.
function intervalsOf(records, run = null) {
  const scoped = run ? records.filter((record) => record.run === run) : records;
  return scoped
    .filter((record) => record.event === "motion_finished" && record.startedAt !== null)
    .map((record) => ({
      robot: record.robot,
      action: record.action,
      start: record.startedAt,
      end: record.startedAt + record.durationMs,
      success: record.success,
      reason: record.reason
    }))
    .sort((left, right) => left.start - right.start);
}

// Sweep the interval endpoints once, carrying how many arms are moving. The
// union of "two or more moving" is the concurrency figure; the union of "one or
// more" is how much of the makespan the cell spent moving at all.
function overlapProfile(intervals) {
  const events = [];
  for (const interval of intervals) {
    events.push({ at: interval.start, delta: 1 });
    events.push({ at: interval.end, delta: -1 });
  }
  events.sort((left, right) => left.at - right.at || left.delta - right.delta);
  let active = 0;
  let previous = null;
  let movingMs = 0;
  let concurrentMs = 0;
  let peak = 0;
  for (const event of events) {
    if (previous !== null && event.at > previous) {
      const span = event.at - previous;
      if (active >= 1) movingMs += span;
      if (active >= 2) concurrentMs += span;
    }
    active += event.delta;
    peak = Math.max(peak, active);
    previous = event.at;
  }
  return { movingMs, concurrentMs, peakConcurrentArms: peak };
}

function motionReport(file, options = {}) {
  const records = readRecords(file);
  if (records.length === 0) throw new Error(`No motion records in ${file}`);
  const runs = [...new Set(records.map((record) => record.run))];
  const run = options.run || (runs.length === 1 ? runs[0] : null);
  const scoped = run ? records.filter((record) => record.run === run) : records;
  const intervals = intervalsOf(scoped);
  if (intervals.length === 0) throw new Error(`No completed motions in ${file}${run ? ` for run ${run}` : ""}`);

  const first = Math.min(...scoped.map((record) => record.t));
  const last = Math.max(...scoped.map((record) => record.t));
  const makespanMs = last - first;
  const profile = overlapProfile(intervals);

  const robots = [...new Set(intervals.map((interval) => interval.robot))].sort();
  const perRobot = robots.map((robot) => {
    const own = intervals.filter((interval) => interval.robot === robot);
    const busyMs = own.reduce((total, interval) => total + (interval.end - interval.start), 0);
    const waits = scoped.filter((record) => record.robot === robot && record.event === "motion_started");
    return {
      robot,
      motions: own.length,
      busyMs,
      failed: own.filter((interval) => !interval.success).length,
      arbitrationWaitMs: waits.reduce((total, record) => total + (record.waitMs || 0), 0)
    };
  });

  const refusals = scoped.filter((record) => record.event === "motion_refused");
  const summedBusyMs = perRobot.reduce((total, robot) => total + robot.busyMs, 0);

  return {
    file,
    run,
    runs,
    makespanMs,
    motions: intervals.length,
    summedBusyMs,
    movingMs: profile.movingMs,
    concurrentMs: profile.concurrentMs,
    peakConcurrentArms: profile.peakConcurrentArms,
    // 1.0 means the arms never overlapped: the cell was fully serialised.
    // Higher means motion time was compressed into less wall clock.
    concurrencyFactor: profile.movingMs > 0 ? summedBusyMs / profile.movingMs : 0,
    concurrentFraction: profile.movingMs > 0 ? profile.concurrentMs / profile.movingMs : 0,
    idleMs: makespanMs - profile.movingMs,
    refusals: refusals.length,
    refusalReasons: [...new Set(refusals.map((record) => record.reason).filter(Boolean))],
    perRobot
  };
}

function formatReport(report) {
  const seconds = (ms) => `${(ms / 1000).toFixed(2)}s`;
  const lines = [
    `Motion report: ${report.file}${report.run ? ` (run ${report.run})` : ""}`,
    "",
    `  makespan                ${seconds(report.makespanMs)}`,
    `  motions                 ${report.motions}`,
    `  summed arm-motion time  ${seconds(report.summedBusyMs)}`,
    `  wall time any arm moved ${seconds(report.movingMs)}`,
    `  wall time 2+ arms moved ${seconds(report.concurrentMs)}   <- zero under a global mutex`,
    `  peak arms moving        ${report.peakConcurrentArms}`,
    `  concurrency factor      ${report.concurrencyFactor.toFixed(3)}   (1.000 = fully serialised)`,
    `  cell idle               ${seconds(report.idleMs)}`,
    `  refusals                ${report.refusals}${report.refusalReasons.length ? ` (${report.refusalReasons.join(", ")})` : ""}`,
    "",
    "robot        motions    busy    waiting   failed"
  ];
  for (const robot of report.perRobot) {
    lines.push(
      `${robot.robot.padEnd(12)} ${String(robot.motions).padStart(7)} `
      + `${seconds(robot.busyMs).padStart(8)} ${seconds(robot.arbitrationWaitMs).padStart(10)} `
      + `${String(robot.failed).padStart(8)}`
    );
  }
  return lines.join("\n");
}

if (require.main === module) {
  const args = process.argv.slice(2);
  const valueOf = (flag) => {
    const index = args.indexOf(flag);
    return index === -1 ? null : args[index + 1];
  };
  const file = valueOf("--file") || process.env.WOMAT_MOTION_LOG;
  if (!file) {
    console.error("Usage: node src/generator/motionReport.js --file performance_log/motions.jsonl [--run <label>] [--json]");
    process.exit(1);
  }
  const report = motionReport(path.resolve(process.cwd(), file), { run: valueOf("--run") });
  console.log(args.includes("--json") ? JSON.stringify(report, null, 2) : formatReport(report));
}

module.exports = { formatReport, intervalsOf, motionReport, overlapProfile, readRecords };
