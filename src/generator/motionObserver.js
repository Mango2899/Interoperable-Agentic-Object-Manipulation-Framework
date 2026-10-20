#!/usr/bin/env node
"use strict";

// Ground truth: which arms were actually moving, sampled from the scene.
//
// The motion log records grant-to-completion, which is what the ARBITER did.
// That is not the same as what the arms did: a motion that spends four seconds
// in preflight and is then refused occupies the same window as one that sweeps
// the cell, and a move to a pose the arm is already standing in occupies a
// window while nothing turns at all. Reporting either as "moving" overstates
// concurrency, which is the one direction this measurement must not err in.
//
// So this samples every robot's joint angles directly and independently of the
// cores. An arm is moving when its joints are changing. Nothing else counts.

const fs = require("fs");
const path = require("path");

const DEFAULT_INTERVAL_MS = 100;
// Below this, a joint is holding position rather than moving: servo jitter and
// dynamic settling both sit well under a tenth of a degree per sample.
const DEFAULT_MOVING_THRESHOLD_DEG = 0.05;

function lowerThingUrl(manifest) {
  const url = manifest.lowerLayerUrl;
  if (!url) throw new Error(`${manifest.name} has no lowerLayerUrl`);
  // node-wot serves the Thing on a lowercased route.
  const marker = url.lastIndexOf("/");
  return `${url.slice(0, marker)}/${url.slice(marker + 1).toLowerCase()}`;
}

async function sampleJoints(url, timeoutMs = 2000) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(`${url}/properties/getJointposition`, { signal: controller.signal });
    if (!response.ok) return null;
    const payload = await response.json();
    const values = Array.isArray(payload) ? payload : Object.values(payload || {});
    return values.map(Number).filter(Number.isFinite);
  } catch (error) {
    return null;
  } finally {
    clearTimeout(timer);
  }
}

async function observe({ robots, file, durationMs, intervalMs = DEFAULT_INTERVAL_MS, onSample = null }) {
  const stream = fs.createWriteStream(file, { flags: "a" });
  const started = Date.now();
  const targets = robots.map((manifest) => ({ name: manifest.name, url: lowerThingUrl(manifest) }));
  while (Date.now() - started < durationMs) {
    const at = Date.now();
    const samples = await Promise.all(targets.map(async (target) => ({
      robot: target.name,
      joints: await sampleJoints(target.url)
    })));
    for (const sample of samples) {
      if (!sample.joints) continue;
      stream.write(`${JSON.stringify({ t: at, robot: sample.robot, joints: sample.joints })}\n`);
    }
    if (onSample) onSample(at, samples);
    const elapsed = Date.now() - at;
    if (elapsed < intervalMs) await new Promise((resolve) => setTimeout(resolve, intervalMs - elapsed));
  }
  await new Promise((resolve) => stream.end(resolve));
  return { file, samples: Math.floor(durationMs / intervalMs) };
}

// Turn samples into per-robot moving intervals, then measure their overlap.
function movementReport(file, options = {}) {
  const threshold = Number(options.thresholdDeg ?? DEFAULT_MOVING_THRESHOLD_DEG);
  const rows = fs.readFileSync(file, "utf8").split("\n").filter((line) => line.trim())
    .map((line) => JSON.parse(line));
  const byRobot = new Map();
  for (const row of rows) {
    if (!byRobot.has(row.robot)) byRobot.set(row.robot, []);
    byRobot.get(row.robot).push(row);
  }

  const intervals = [];
  const perRobot = [];
  for (const [robot, samples] of byRobot) {
    samples.sort((left, right) => left.t - right.t);
    let movingMs = 0;
    let travelDeg = 0;
    for (let index = 1; index < samples.length; index += 1) {
      const previous = samples[index - 1].joints;
      const current = samples[index].joints;
      if (!previous || !current || previous.length !== current.length) continue;
      const delta = Math.max(...current.map((value, axis) => Math.abs(value - previous[axis])));
      if (delta > threshold) {
        movingMs += samples[index].t - samples[index - 1].t;
        travelDeg += delta;
        intervals.push({ robot, start: samples[index - 1].t, end: samples[index].t });
      }
    }
    perRobot.push({ robot, samples: samples.length, movingMs, travelDeg });
  }

  // Merge each robot's adjacent moving samples, then sweep for overlap.
  const events = [];
  for (const interval of intervals) {
    events.push({ at: interval.start, delta: 1 });
    events.push({ at: interval.end, delta: -1 });
  }
  events.sort((left, right) => left.at - right.at || left.delta - right.delta);
  let active = 0;
  let previous = null;
  let anyMovingMs = 0;
  let concurrentMs = 0;
  let peak = 0;
  const concurrentWindows = [];
  let windowStart = null;
  for (const event of events) {
    if (previous !== null && event.at > previous) {
      if (active >= 1) anyMovingMs += event.at - previous;
      if (active >= 2) concurrentMs += event.at - previous;
    }
    const before = active;
    active += event.delta;
    if (before < 2 && active >= 2) windowStart = event.at;
    if (before >= 2 && active < 2 && windowStart !== null) {
      concurrentWindows.push({ start: windowStart, end: event.at });
      windowStart = null;
    }
    peak = Math.max(peak, active);
    previous = event.at;
  }

  const first = rows.length ? Math.min(...rows.map((row) => row.t)) : 0;
  return {
    file,
    thresholdDeg: threshold,
    observedMs: rows.length ? Math.max(...rows.map((row) => row.t)) - first : 0,
    anyMovingMs,
    concurrentMs,
    peakMovingArms: peak,
    concurrentWindows: concurrentWindows.map((window) => ({
      startMs: window.start - first,
      durationMs: window.end - window.start
    })),
    perRobot: perRobot.sort((left, right) => left.robot.localeCompare(right.robot))
  };
}

function formatMovement(report) {
  const seconds = (ms) => `${(ms / 1000).toFixed(2)}s`;
  const lines = [
    `Observed motion: ${report.file}`,
    `  sampled over            ${seconds(report.observedMs)} (moving threshold ${report.thresholdDeg} deg/sample)`,
    `  any arm actually moving ${seconds(report.anyMovingMs)}`,
    `  2+ arms actually moving ${seconds(report.concurrentMs)}`,
    `  peak arms moving        ${report.peakMovingArms}`,
    "",
    "robot        samples   moving   joint travel"
  ];
  for (const robot of report.perRobot) {
    lines.push(
      `${robot.robot.padEnd(12)} ${String(robot.samples).padStart(7)} `
      + `${seconds(robot.movingMs).padStart(8)} ${`${robot.travelDeg.toFixed(1)} deg`.padStart(14)}`
    );
  }
  if (report.concurrentWindows.length > 0) {
    lines.push("", "windows with 2+ arms moving:");
    for (const window of report.concurrentWindows) {
      lines.push(`  at +${(window.startMs / 1000).toFixed(2)}s for ${(window.durationMs / 1000).toFixed(2)}s`);
    }
  }
  return lines.join("\n");
}

if (require.main === module) {
  const args = process.argv.slice(2);
  const valueOf = (flag, fallback = null) => {
    const index = args.indexOf(flag);
    return index === -1 ? fallback : args[index + 1];
  };
  const file = path.resolve(process.cwd(), valueOf("--file", "performance_log/observed.jsonl"));
  if (args.includes("--report")) {
    console.log(formatMovement(movementReport(file, { thresholdDeg: valueOf("--threshold") })));
  } else {
    const manifests = String(valueOf("--manifests", "")).split(",").map((part) => part.trim()).filter(Boolean);
    if (manifests.length === 0) {
      console.error("Usage: node src/generator/motionObserver.js --manifests <a,b,c> [--seconds 60] [--file f]");
      console.error("       node src/generator/motionObserver.js --report [--file f] [--threshold 0.05]");
      process.exit(1);
    }
    const robots = manifests.map((manifest) => JSON.parse(fs.readFileSync(path.resolve(process.cwd(), manifest), "utf8")));
    fs.mkdirSync(path.dirname(file), { recursive: true });
    fs.writeFileSync(file, "");
    const durationMs = Number(valueOf("--seconds", "60")) * 1000;
    console.log(`Observing ${robots.map((robot) => robot.name).join(", ")} for ${durationMs / 1000}s -> ${file}`);
    observe({ robots, file, durationMs, intervalMs: Number(valueOf("--interval", DEFAULT_INTERVAL_MS)) })
      .then(() => console.log(formatMovement(movementReport(file))))
      .catch((error) => { console.error(error); process.exit(1); });
  }
}

module.exports = { DEFAULT_MOVING_THRESHOLD_DEG, formatMovement, movementReport, observe };
