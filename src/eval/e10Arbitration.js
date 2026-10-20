#!/usr/bin/env node
"use strict";
// E10: the normal BDI delivery, exclusive versus region arbitration.
// Evaluation copies add markers and the terminal receiver's return home.
const fs = require("node:fs");
const path = require("node:path");
const net = require("node:net");
const { spawn } = require("node:child_process");
const { nowMs } = require("../generator/core/motionLog");
const { toCsv } = require("./e2CollisionCost");
const { readRecords } = require("./e9HandoverPoint");
const ROOT = path.resolve(__dirname, "../..");
const NAMES = [1, 2, 3, 4, 5].map(i => "Robot" + i);
const MODES = ["exclusive", "region"];
const LABEL = { exclusive: "Exclusive", region: "Region-based" };
const CELL = NAMES.join("_");
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
const json = file => JSON.parse(fs.readFileSync(file, "utf8"));
const writeJson = (file, value) => fs.writeFileSync(file, JSON.stringify(value, null, 2) + "\n");

function replaceOnce(source, from, to) {
  if (source.split(from).length !== 2) throw new Error("Instrumentation anchor changed: " + from.slice(0, 90));
  return source.replace(from, to);
}

function prepareAgents(directory) {
  fs.mkdirSync(directory, { recursive: true });
  let plans = fs.readFileSync(path.join(ROOT, "src/agt/handoverPlans.asl"), "utf8");
  plans = replaceOnce(plans, "    if (not handoverRetriesLeft(_))",
    '    .print("E10|begin|", MyRobot);\n    if (not handoverRetriesLeft(_))');
  plans = replaceOnce(plans, "    +handoverStage(Session, receiver_approaching);",
    '    .print("E10|begin|", MyRobot);\n    +handoverStage(Session, receiver_approaching);');
  plans = replaceOnce(plans, '    .print("Giver reached all-zero joint pose: ", Session);',
    '    .print("Giver reached all-zero joint pose: ", Session);\n    .print("E10|home|", MyRobot);');
  plans = replaceOnce(plans, '    .print("Cube placed on ", Goal, " by ", MyRobot).',
    '    .print("Cube placed on ", Goal, " by ", MyRobot);\n'
    + '    .print("E10|placed|", MyRobot);\n    !goHome;\n    .print("E10|home|", MyRobot).');
  fs.writeFileSync(path.join(directory, "handoverPlans.asl"), plans);
  for (const name of NAMES) {
    const file = name.toLowerCase() + "_agent.asl";
    let agent = fs.readFileSync(path.join(ROOT, "src/agt/generated", file), "utf8");
    agent = replaceOnce(agent, 'include("src/agt/handoverPlans.asl")',
      "include(" + JSON.stringify(path.join(directory, "handoverPlans.asl")) + ")");
    fs.writeFileSync(path.join(directory, file), agent);
  }
  let jcm = fs.readFileSync(path.join(ROOT, "generated/robots/" + CELL + ".jcm"), "utf8");
  for (const name of NAMES) {
    const file = name.toLowerCase() + "_agent.asl";
    // JaCaMo's source token retains quote characters; use its unquoted path
    // syntax (as the generated production JCM does).
    const source = path.relative(ROOT, path.join(directory, file));
    if (/\s/.test(source)) throw new Error("The E10 output path must not contain whitespace (JaCaMo source syntax).");
    jcm = replaceOnce(jcm, "generated/" + file, source);
  }
  const file = path.join(directory, "e10.jcm");
  fs.writeFileSync(file, jcm);
  fs.writeFileSync(path.join(directory, "logging.properties"),
    "handlers=java.util.logging.ConsoleHandler\n.level=INFO\n"
    + "java.util.logging.ConsoleHandler.level=ALL\n"
    + "java.util.logging.ConsoleHandler.formatter=jason.runtime.MASConsoleLogFormatter\n");
  return file;
}

function parseMarker(line) {
  const match = /E10\|(begin|home|placed)\|\s*(robot[1-5])\b/i.exec(line);
  return match ? { event: match[1], robot: "Robot" + match[2].slice(5) } : null;
}

function terminalFailure(line) {
  return /Could not place the cube on |Giver handover failed|Giver home motion failed|Home motion failed for |only this arm's own exit failed/.test(line)
    ? line.trim() : null;
}

function scheduledTrials(options, records = []) {
  // A failed execution consumes its scheduled trial; never replace failures
  // with extra successes. Setup-only failures require separate diagnosis.
  const done = new Set(records.filter(r => r.startedAt != null).map(r => r.mode + ":" + r.repetition));
  const remaining = [];
  for (let repetition = 1; repetition <= options.repetitions; repetition++) {
    for (const mode of repetition % 2 ? options.modes : [...options.modes].reverse()) {
      if (!done.has(mode + ":" + repetition)) remaining.push({ mode, repetition });
    }
  }
  return remaining;
}

// A final-role marker, emitted AFTER the agent became idle, arms completion.
// Intermediate homes while carrying the cube and unused arms do not count.
function updateCompletion(state, sample, { toleranceDeg, settleMs }) {
  for (const [name, robot] of Object.entries(state.robots)) {
    if (robot.homeRequestedAt == null || robot.endAt != null) continue;
    const joints = sample.joints[name];
    const valid = Array.isArray(joints) && joints.length === 6 && joints.every(Number.isFinite);
    if (!valid || !joints.every(q => Math.abs(q) <= toleranceDeg)) {
      robot.stableSince = null;
      continue;
    }
    if (robot.stableSince == null) robot.stableSince = sample.t;
    if (sample.t - robot.stableSince >= settleMs) {
      robot.endAt = sample.t; // confirmation includes the settling window
      robot.finalJointsDeg = joints;
    }
  }
}

function taskComplete(state) {
  return state.placedAt != null && state.selectedRoute?.length >= 2
    && state.selectedRoute.every(name => Number.isFinite(state.robots[name]?.endAt))
    && Object.values(state.robots).every(robot => Number.isFinite(robot.endAt));
}

function stats(values) {
  const xs = values.filter(Number.isFinite);
  if (!xs.length) return { n: 0, mean: null, sd: null };
  const mean = xs.reduce((a, b) => a + b, 0) / xs.length;
  return { n: xs.length, mean, sd: xs.length > 1
    ? Math.sqrt(xs.reduce((s, x) => s + (x - mean) ** 2, 0) / (xs.length - 1)) : null };
}

function summarize(records, robotOrigin = "task") {
  if (!["task", "participation"].includes(robotOrigin)) throw new Error("Robot timing origin must be task or participation.");
  return MODES.flatMap(mode => {
    const attempted = records.filter(r => r.mode === mode && r.startedAt != null);
    const completed = attempted.filter(r => r.success);
    return ["delivery", "task", ...NAMES].map(metric => {
      const values = completed.map(r => metric === "delivery" ? (r.placedAt - r.startedAt) / 1000
        : metric === "task" ? (r.endedAt - r.startedAt) / 1000
          : r.robots[metric]?.endAt != null
            ? (r.robots[metric].endAt - (robotOrigin === "task" ? r.startedAt : r.robots[metric].beginAt)) / 1000 : null);
      return { mode, metric, origin: NAMES.includes(metric) ? robotOrigin : "task",
        attempted: attempted.length, succeeded: completed.length,
        failed: attempted.length - completed.length,
        setupFailures: records.filter(r => r.mode === mode && r.startedAt == null).length, ...stats(values) };
    });
  });
}

function figures(records, summary) {
  const good = records.filter(r => r.success);
  const metrics = ["delivery", "task", ...NAMES];
  const labels = ["Delivery", "All home", ...NAMES.map(n => n.replace("Robot", "R"))];
  const duration = ["% E10 means and sample standard deviations; successful runs only.",
    "\\begin{tikzpicture}", "\\begin{axis}[ybar,bar width=7pt,width=\\linewidth,height=6.5cm,",
    "symbolic x coords={" + labels.join(",") + "},xtick=data,ymin=0,ylabel={Time [s]},",
    "legend style={at={(0.5,1.03)},anchor=south,legend columns=2},enlarge x limits=0.1,ymajorgrids=true]"];
  for (const mode of MODES) {
    const coords = metrics.map((metric, i) => {
      const s = summary.find(row => row.mode === mode && row.metric === metric);
      return s.n ? "(" + labels[i] + "," + s.mean.toFixed(4) + ") +- (0," + (s.sd ?? 0).toFixed(4) + ")" : null;
    }).filter(Boolean);
    if (coords.length) duration.push("\\addplot+[draw=" + (mode === "exclusive" ? "blue!65" : "orange!85!black")
      + ",fill=" + (mode === "exclusive" ? "blue!30" : "orange!45")
      + ",error bars/.cd,y dir=both,y explicit] coordinates {" + coords.join(" ") + "};",
    "\\addlegendentry{" + LABEL[mode] + "}");
  }
  duration.push("\\end{axis}", "\\end{tikzpicture}");
  const timeline = ["% Representative: lower median successful makespan in each mode. Bars are participation, NOT motion."];
  const representatives = Object.fromEntries(MODES.map(mode => {
    const sorted = good.filter(r => r.mode === mode).sort((a, b) => (a.endedAt - a.startedAt) - (b.endedAt - b.startedAt));
    return [mode, sorted.length ? sorted[Math.floor((sorted.length - 1) / 2)] : null];
  }));
  const timelineMax = Math.max(...Object.values(representatives).filter(Boolean).map(r => (r.endedAt - r.startedAt) / 1000)) * 1.04;
  for (const mode of MODES) {
    const r = representatives[mode];
    if (!r) continue;
    timeline.push("% " + mode + ", repetition " + r.repetition, "\\begin{tikzpicture}",
      "\\begin{axis}[width=\\linewidth,height=4.5cm,title={" + LABEL[mode] + "},xmin=0,xmax="
        + timelineMax.toFixed(3) + ",",
      "ymin=0.5,ymax=5.5,ytick={1,2,3,4,5},yticklabels={R1,R2,R3,R4,R5},xlabel={Time since delivery request [s]},xmajorgrids=true]");
    for (const [name, robot] of Object.entries(r.robots)) {
      if (robot.endAt == null) continue;
      const y = Number(name.slice(5));
      timeline.push("\\addplot+[no marks,line width=7pt," + (mode === "exclusive" ? "blue!65" : "orange!85!black")
        + "] coordinates {(" + ((robot.beginAt - r.startedAt) / 1000).toFixed(3) + "," + y + ") ("
        + ((robot.endAt - r.startedAt) / 1000).toFixed(3) + "," + y + ")};");
    }
    const delivery = ((r.placedAt - r.startedAt) / 1000).toFixed(3);
    timeline.push("\\addplot[black,dashed] coordinates {(" + delivery + ",0.5) (" + delivery + ",5.5)};",
      "\\end{axis}", "\\end{tikzpicture}\\par");
  }
  return { duration: duration.join("\n") + "\n", timeline: timeline.join("\n") + "\n" };
}

// Home endpoints were recorded on a joint sample, so their simulation times
// are exact sample values. Agent print events lie between samples: use the
// following sample and retain both bounds rather than converting wall seconds
// with an assumed constant playback rate.
function inSimulationTime(record, samples) {
  if (record.startedAt == null) return { mode: record.mode, repetition: record.repetition,
    success: false, startedAt: null, error: record.error, robots: {}, clock: "simulation" };
  const origin = record.startedSimTime;
  if (!Number.isFinite(origin)) throw new Error("Missing simulation-time origin.");
  const trace = [{ t: record.startedAt, simTime: origin }, ...samples];
  const exact = new Map(samples.map(sample => [sample.t, sample.simTime]));
  const bounds = timestamp => {
    if (timestamp == null) return { lower: null, upper: null };
    const index = trace.findIndex(sample => sample.t >= timestamp);
    if (index < 0) {
      if (!record.success) return { lower: (trace.at(-1).simTime - origin) * 1000, upper: null };
      throw new Error("No simulation sample brackets an agent event.");
    }
    return { lower: (trace[Math.max(0, index - 1)].simTime - origin) * 1000,
      upper: (trace[index].simTime - origin) * 1000 };
  };
  const robots = {};
  for (const [name, robot] of Object.entries(record.robots)) {
    const begin = bounds(robot.beginAt);
    const home = robot.endAt == null ? null : exact.get(robot.endAt);
    if (robot.endAt != null && !Number.isFinite(home)) throw new Error("Missing exact joint sample for " + name + "'s home confirmation.");
    robots[name] = { beginAt: begin.upper, beginBounds: begin, endAt: home == null ? null : (home - origin) * 1000,
      finalJointsDeg: robot.finalJointsDeg };
  }
  const delivery = bounds(record.placedAt);
  return { mode: record.mode, repetition: record.repetition, success: record.success, error: record.error,
    selectedRoute: record.selectedRoute, clock: "simulation", originSimTime: origin, startedAt: 0,
    placedAt: delivery.upper, deliveryBounds: delivery,
    endedAt: record.success ? (record.endedSimTime - origin) * 1000 : null, robots };
}

function durationTable(summary, robotOrigin, clock) {
  const value = (mode, metric) => {
    const s = summary.find(r => r.mode === mode && r.metric === metric);
    return s.n ? s.mean.toFixed(2) + (s.sd == null ? " (n=1)" : " $\\pm$ " + s.sd.toFixed(2) + " (" + s.n + ")") : "--";
  };
  return ["% E10 mean +/- sample SD; successful observations. Clock: " + clock,
    "% Per-robot timing origin: " + robotOrigin,
    "\\begin{tabular}{@{}lrr@{}}", "\\toprule", "Metric & Exclusive & Region-based \\\\", "\\midrule",
    "Completed trials & " + MODES.map(mode => {
      const row = summary.find(r => r.mode === mode && r.metric === "task");
      return row.succeeded + "/" + row.attempted;
    }).join(" & ") + " \\\\", "\\midrule",
    ...["delivery", "task", ...NAMES].map(metric =>
      (({ delivery: "Delivery [s]", task: "All home and idle [s]" })[metric] || metric + " [s]")
      + " & " + value("exclusive", metric) + " & " + value("region", metric) + " \\\\"),
    "\\bottomrule", "\\end{tabular}"].join("\n") + "\n";
}

function comparisons(summary) {
  return Object.fromEntries(["delivery", "task", ...NAMES].map(metric => {
    const ex = summary.find(r => r.mode === "exclusive" && r.metric === metric);
    const re = summary.find(r => r.mode === "region" && r.metric === metric);
    return [metric, { exclusiveN: ex.n, regionN: re.n,
      speedup: ex.mean > 0 && re.mean > 0 ? ex.mean / re.mean : null,
      reductionPercent: ex.mean > 0 && re.mean > 0 ? 100 * (ex.mean - re.mean) / ex.mean : null }];
  }));
}

function analyze(outDir, robotOrigin = "task") {
  const records = readRecords(path.join(outDir, "e10_runs.jsonl"));
  const summary = summarize(records, robotOrigin);
  fs.mkdirSync(outDir, { recursive: true });
  const columns = ["mode", "metric", "origin", "attempted", "succeeded", "failed", "setupFailures", "n", "mean", "sd"];
  fs.writeFileSync(path.join(outDir, "e10_summary.csv"), toCsv(summary,
    columns));
  fs.writeFileSync(path.join(outDir, "e10_participation_summary.csv"), toCsv(summarize(records, "participation").filter(r => NAMES.includes(r.metric)), columns));
  writeJson(path.join(outDir, "e10_analysis.json"), { robotOrigin, durationUnit: "wall-clock seconds",
    successfulTrialsOnly: true, standardDeviation: "sample", representativeTimeline: "lower median makespan",
    simulationOutputs: { durationUnit: "simulation seconds", homeEndpoints: "exact recorded joint sample",
      agentEvents: "following sample; lower and upper bounds retained", start: "last scene sample before delivery request" } });
  const rows = records.flatMap(r => NAMES.map(robot => ({ mode: r.mode, repetition: r.repetition,
    success: r.success, robot, participated: !!r.robots?.[robot],
    beginS: r.robots?.[robot] ? (r.robots[robot].beginAt - r.startedAt) / 1000 : null,
    homeS: r.robots?.[robot]?.endAt != null ? (r.robots[robot].endAt - r.startedAt) / 1000 : null,
    participationS: r.robots?.[robot]?.endAt != null ? (r.robots[robot].endAt - r.robots[robot].beginAt) / 1000 : null,
    deliveryS: r.placedAt != null ? (r.placedAt - r.startedAt) / 1000 : null,
    taskS: r.success ? (r.endedAt - r.startedAt) / 1000 : null })));
  fs.writeFileSync(path.join(outDir, "e10_robots.csv"), toCsv(rows,
    ["mode", "repetition", "success", "robot", "participated", "beginS", "homeS", "participationS", "deliveryS", "taskS"]));
  fs.writeFileSync(path.join(outDir, "e10_trials.csv"), toCsv(records.map(r => ({
    mode: r.mode, repetition: r.repetition, started: r.startedAt != null, success: r.success,
    route: (r.selectedRoute || Object.keys(r.robots || {})).join(">"),
    deliveryS: r.placedAt != null ? (r.placedAt - r.startedAt) / 1000 : null,
    taskS: r.success ? (r.endedAt - r.startedAt) / 1000 : null
  })), ["mode", "repetition", "started", "success", "route", "deliveryS", "taskS"]));
  fs.writeFileSync(path.join(outDir, "e10_table.tex"), durationTable(summary, robotOrigin, "wall"));
  let simulation = null;
  if (records.some(r => r.success && Number.isFinite(r.startedSimTime))) {
    const converted = records.map(r => inSimulationTime(r, readRecords(path.join(outDir, "trials", r.mode + "_" + r.repetition, "samples.jsonl"))));
    const simSummary = summarize(converted, robotOrigin);
    simulation = { records: converted, summary: simSummary };
    fs.writeFileSync(path.join(outDir, "e10_simulation_runs.jsonl"), converted.map(r => JSON.stringify(r)).join("\n") + "\n");
    fs.writeFileSync(path.join(outDir, "e10_simulation_summary.csv"), toCsv(simSummary, columns));
    writeJson(path.join(outDir, "e10_simulation_comparison.json"), comparisons(simSummary));
    fs.writeFileSync(path.join(outDir, "e10_simulation_table.tex"), durationTable(simSummary, robotOrigin, "simulation"));
    const simPlots = figures(converted, simSummary);
    simPlots.duration = simPlots.duration.replace("ylabel={Time [s]}", "ylabel={Simulation time [s]}");
    simPlots.timeline = simPlots.timeline.replaceAll("Time since delivery request [s]", "Simulation time since delivery request [s]");
    fs.writeFileSync(path.join(outDir, "e10_figure_simulation_durations.tex"), simPlots.duration);
    fs.writeFileSync(path.join(outDir, "e10_figure_simulation_timeline.tex"), simPlots.timeline);
    simulation.plots = simPlots;
    const clockRows = records.map(r => ({ mode: r.mode, repetition: r.repetition, success: r.success,
      wallS: r.success ? (r.endedAt - r.startedAt) / 1000 : null,
      simulationS: r.success ? r.endedSimTime - r.startedSimTime : null,
      simulationPerWall: r.success ? (r.endedSimTime - r.startedSimTime) / ((r.endedAt - r.startedAt) / 1000) : null }));
    fs.writeFileSync(path.join(outDir, "e10_clocks.csv"), toCsv(clockRows, ["mode", "repetition", "success", "wallS", "simulationS", "simulationPerWall"]));
  }
  if (records.some(r => r.success)) {
    const plots = figures(records, summary);
    fs.writeFileSync(path.join(outDir, "e10_figure_durations.tex"), plots.duration);
    fs.writeFileSync(path.join(outDir, "e10_figure_timeline.tex"), plots.timeline);
    fs.writeFileSync(path.join(outDir, "e10_figures.tex"),
      "\\documentclass{article}\n\\usepackage[margin=15mm]{geometry}\n\\usepackage{pgfplots}\n\\pgfplotsset{compat=1.18}\n\\pagestyle{empty}\n\\begin{document}\n"
      + (simulation ? simulation.plots.duration + "\\clearpage\n" + simulation.plots.timeline + "\\clearpage\n" : "")
      + plots.duration.replace("ylabel={Time [s]}", "ylabel={Wall time [s]}")
      + "\\clearpage\n" + plots.timeline.replaceAll("Time since delivery request [s]", "Wall time since delivery request [s]")
      + "\\end{document}\n");
  }
  const comparison = comparisons(summary);
  writeJson(path.join(outDir, "e10_comparison.json"), comparison);
  return { records, summary, comparison, simulation };
}

function launch(command, args, file, onLine, env = {}) {
  const child = spawn(command, args, { cwd: ROOT, env: { ...process.env, ...env },
    detached: true, stdio: ["ignore", "pipe", "pipe"] });
  child.on("error", error => { child.launchError = error; });
  for (const stream of [child.stdout, child.stderr]) {
    let pending = "";
    stream.on("data", chunk => {
      fs.appendFileSync(file, chunk);
      pending += chunk.toString();
      const lines = pending.split(/\r?\n/); pending = lines.pop();
      for (const line of lines) onLine?.(line);
    });
  }
  return child;
}
async function stop(child) {
  if (!child?.pid) return;
  try { process.kill(-child.pid, "SIGTERM"); } catch (e) { if (e.code !== "ESRCH") throw e; }
  await delay(700);
  try { process.kill(-child.pid, "SIGKILL"); } catch (e) { if (e.code !== "ESRCH") throw e; }
}
async function until(predicate, timeoutMs, children = []) {
  const start = nowMs();
  while (!await predicate()) {
    for (const child of children) if (child.launchError || child.exitCode != null || child.signalCode != null)
      throw new Error("Runtime exited: " + (child.launchError?.message || child.exitCode));
    if (nowMs() - start > timeoutMs) throw new Error("Runtime readiness timeout; inspect trial logs.");
    await delay(250);
  }
}
async function request(url, body) {
  const response = await fetch(url, { signal: AbortSignal.timeout(5000), ...(body === undefined ? {} : {
    method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(body) }) });
  if (!response.ok) throw new Error("HTTP " + response.status + ": " + url);
  const content = await response.text();
  return content ? JSON.parse(content) : null;
}
async function assertPortsFree(manifests) {
  for (const port of manifests.flatMap(m => Object.values(m.ports))) {
    const occupied = await new Promise(resolve => {
      const socket = net.connect({ host: "127.0.0.1", port: Number(port) });
      socket.setTimeout(1000);
      socket.once("connect", () => { socket.destroy(); resolve(true); });
      socket.once("error", () => resolve(false));
      socket.once("timeout", () => { socket.destroy(); resolve(false); });
    });
    if (occupied) throw new Error("Port " + port + " is occupied. Stop the existing robot runtime before E10; the runner manages a fresh cell for each trial.");
  }
}

async function trial(options, mode, repetition, manifests, jcm) {
  const directory = path.join(options.outDir, "trials", mode + "_" + repetition);
  if (fs.existsSync(directory)) throw new Error("Trial directory already exists; preserve it and use a fresh --out.");
  fs.mkdirSync(directory, { recursive: true });
  const state = { mode, repetition, robots: {}, success: false, startedAt: null, placedAt: null };
  const children = [];
  const append = row => fs.appendFileSync(path.join(directory, "events.jsonl"), JSON.stringify(row) + "\n");
  const ready = new Set();
  let routeReady = false;
  let armed = false;
  let fatal = null;
  let connection;
  const logLine = line => {
    if (/URISyntaxException|Error parsing|SEVERE.*not found/.test(line))
      fatal = new Error("Agent startup failed: " + line);
    const readyMatch = /Robot([1-5]) agent ready/.exec(line);
    if (readyMatch) ready.add("Robot" + readyMatch[1]);
    if (/Route settled:/.test(line)) {
      routeReady = true;
      append({ t: nowMs(), event: "route", line });
      const route = /reaching the goal via \[([^\]]+)\]/.exec(line);
      if (route) state.selectedRoute = ["Robot1", ...route[1].split(",").map(name => name.trim().replace(/^robot/, "Robot"))];
    }
    if (armed && terminalFailure(line)) {
      if (!fatal) append({ t: nowMs(), event: "terminal_failure", line });
      fatal ||= new Error(terminalFailure(line));
    }
    const marker = parseMarker(line);
    if (!armed || !marker) return;
    const t = nowMs();
    append({ t, ...marker });
    if (marker.event === "begin") state.robots[marker.robot] ||= { beginAt: t, homeRequestedAt: null, endAt: null };
    else if (marker.event === "placed") { state.placedAt = t; state.sink = marker.robot; }
    else if (state.robots[marker.robot]) state.robots[marker.robot].homeRequestedAt = t;
  };
  const interrupt = () => { fatal = new Error("Evaluation interrupted"); };
  process.on("SIGINT", interrupt);
  process.on("SIGTERM", interrupt);
  try {
    const manifestPaths = NAMES.map(name => "generated/robots/" + name + "/manifest.json").join(",");
    let modeConfirmed = false;
    children.push(launch(process.execPath, ["src/generator/lowerThingServer.js", "--manifests", manifestPaths,
      ...(mode === "exclusive" ? ["--exclusive-motion"] : [])], path.join(directory, "lower.log"), line => {
      if (line.includes("motion arbitration: " + (mode === "exclusive" ? "EXCLUSIVE" : "REGION"))) modeConfirmed = true;
    }, { WOMAT_MOTION_LOG: path.join(directory, "motions.jsonl"), WOMAT_RUN_LABEL: mode + "_" + repetition }));
    await until(() => { if (fatal) throw fatal; return modeConfirmed; }, options.startupMs, children);
    for (const name of NAMES) children.push(launch(process.execPath,
      ["src/generator/agentThingServer.js", "--manifest", "generated/robots/" + name + "/manifest.json"],
      path.join(directory, name + ".log")));
    await until(async () => {
      if (fatal) throw fatal;
      const results = await Promise.allSettled(manifests.map(m => request("http://localhost:" + m.ports.agentThing + "/" + m.name.toLowerCase() + "_agent")));
      return results.every(r => r.status === "fulfilled");
    }, options.startupMs, children);
    children.push(launch(path.join(ROOT, "gradlew"),
      ["--offline", "--no-daemon", "--console=plain", "-Pheadless", "-Pjcm=" + jcm,
        "run", "--args=" + jcm + " --log-conf " + path.join(path.dirname(jcm), "logging.properties")],
      path.join(directory, "agents.log"), logLine));
    await until(() => { if (fatal) throw fatal; return ready.size === NAMES.length && routeReady; }, options.startupMs, children);
    if (!state.selectedRoute || state.selectedRoute.length < 2 || state.selectedRoute.some(name => !NAMES.includes(name)))
      throw new Error("Cannot identify the complete selected handover route.");
    const { connect, executeInScript, sandboxScriptHandle } = require("../generator/core/simConnection");
    connection = await connect({ robwotRoot: path.join(ROOT, "external/RobWoT") });
    const { sim } = connection;
    const sandbox = await sandboxScriptHandle(sim);
    const handles = {};
    for (const name of NAMES) {
      const first = x => Array.isArray(x) ? x[0] : x;
      const object = first(await sim.getObject("/" + name));
      const script = first(await sim.getScript(await sim.scripttype_customizationscript, object, "/" + name));
      handles[name] = await executeInScript(sim, script, "return jointHandles");
      if (!Array.isArray(handles[name]) || handles[name].length !== 6) throw new Error("Cannot sample all six joints of " + name);
    }
    const code = "local result={joints={},simTime=sim.getSimulationTime()}; " + NAMES.map(name =>
      "result.joints." + name + "={}; for i,h in ipairs({" + handles[name].join(",")
      + "}) do result.joints." + name + "[i]=sim.getJointPosition(h)*180/math.pi end; ").join("") + "return result";
    const sample = async () => ({ ...await executeInScript(sim, sandbox, code), t: nowMs() });
    state.simulationSettings = await executeInScript(sim, sandbox,
      "return {speedModifier=sim.getIntProperty(sim.handle_scene,'speedModifier'),"
      + "stepsPerRendering=sim.getIntProperty(sim.handle_scene,'stepsPerRendering'),"
      + "realtime=sim.getBoolProperty(sim.handle_scene,'realtimeSimulation'),"
      + "realtimeModifier=sim.getFloatProperty(sim.handle_scene,'realtimeModifier'),"
      + "timeStep=sim.getSimulationTimeStep()}");
    const initial = await sample();
    if (!NAMES.every(name => initial.joints[name].every(q => Number.isFinite(q) && Math.abs(q) <= options.toleranceDeg)))
      throw new Error("The fresh scene did not start at zero joints.");
    append({ event: "initial", ...initial });
    state.startedAt = nowMs();
    state.startedSimTime = initial.simTime;
    armed = true;
    append({ t: state.startedAt, event: "task_begin" });
    await request("http://localhost:" + manifests[0].ports.agentThing + "/robot1_agent/actions/deliverCube", {});
    while (nowMs() - state.startedAt <= options.timeoutMs) {
      if (fatal) throw fatal;
      for (const child of children) if (child.exitCode != null || child.signalCode != null || child.launchError) throw new Error("A runtime process exited during the task.");
      const reading = await sample();
      fs.appendFileSync(path.join(directory, "samples.jsonl"), JSON.stringify(reading) + "\n");
      updateCompletion(state, reading, options);
      if (taskComplete(state)) {
        const grip = await request(manifests.find(m => m.name === state.sink).lowerLayerUrl + "/properties/getGripperState");
        if (grip.holding !== false) throw new Error("Placement marker arrived but final gripper is not empty.");
        state.endedAt = Math.max(...Object.values(state.robots).map(robot => robot.endAt));
        state.endedSimTime = reading.simTime;
        state.success = true;
        break;
      }
      await delay(options.intervalMs);
    }
    if (!state.success) throw new Error("Task timeout: delivery and every participating robot's final home must both be confirmed.");
  } catch (error) {
    state.error = error.message;
    fatal = error;
  } finally {
    state.observationEndedAt = nowMs();
    if (connection) await connection.client.websocket.close().catch(() => {});
    for (const child of [...children].reverse()) await stop(child);
    process.removeListener("SIGINT", interrupt);
    process.removeListener("SIGTERM", interrupt);
    writeJson(path.join(directory, "result.json"), state);
    fs.appendFileSync(path.join(options.outDir, "e10_runs.jsonl"), JSON.stringify(state) + "\n");
    analyze(options.outDir);
  }
  if (fatal) throw fatal;
  return state;
}

async function run(options) {
  const manifests = NAMES.map(name => json(path.join(ROOT, "generated/robots/" + name + "/manifest.json")));
  await assertPortsFree(manifests);
  fs.mkdirSync(options.outDir, { recursive: true });
  const setupFile = path.join(options.outDir, "e10_setup.json");
  const descriptor = json(path.join(ROOT, "generated/robots/" + CELL + ".shared.json"));
  let jcm;
  if (options.resume) {
    const setup = json(setupFile);
    for (const key of ["modes", "repetitions", "timeoutMs", "startupMs", "intervalMs", "toleranceDeg", "settleMs", "manifests", "descriptor"]) {
      const current = key === "manifests" ? manifests : key === "descriptor" ? descriptor : options[key];
      if (JSON.stringify(current) !== JSON.stringify(setup[key])) throw new Error("Resume setup differs: " + key);
    }
    jcm = path.join(options.outDir, "agent_sources/e10.jcm");
    if (!fs.existsSync(jcm)) throw new Error("Missing original evaluation agent copies.");
    fs.appendFileSync(path.join(options.outDir, "e10_resumptions.jsonl"), JSON.stringify({
      at: new Date().toISOString(), continueOnFailure: options.continueOnFailure
    }) + "\n");
  } else {
    if (fs.existsSync(setupFile)) throw new Error("Output contains an E10 setup; use --resume or a fresh --out.");
    writeJson(setupFile, { createdAt: new Date().toISOString(), ...options, manifests, descriptor,
      timing: "Host monotonic wall time. Task begins at deliverCube request; robot begins on first giver/receiver entry. Home requires final-role idle marker and measured zero joints for settleMs. Delivery includes placement retreat. Startup and route discovery excluded.",
      independentMotions: false });
    jcm = prepareAgents(path.join(options.outDir, "agent_sources"));
  }
  for (const { mode, repetition } of scheduledTrials(options, readRecords(path.join(options.outDir, "e10_runs.jsonl")))) {
    console.log("E10 " + mode + ", repetition " + repetition + "/" + options.repetitions);
    try {
      const result = await trial(options, mode, repetition, manifests, jcm);
      console.log("  delivery " + ((result.placedAt - result.startedAt) / 1000).toFixed(2)
        + " s; all home " + ((result.endedAt - result.startedAt) / 1000).toFixed(2) + " s");
    } catch (error) {
      const last = readRecords(path.join(options.outDir, "e10_runs.jsonl")).at(-1);
      if (!options.continueOnFailure || last?.startedAt == null || last.mode !== mode
        || last.repetition !== repetition || last.success || error.message === "Evaluation interrupted") throw error;
      console.log("  FAILED (retained): " + error.message);
    }
  }
}

async function main(argv) {
  if (argv.includes("--help")) {
    console.log([
      "E10: normal five-robot BDI delivery; no independent motion scenario.",
      "run [--repetitions 10] [--modes exclusive,region] [--out performance_log/e10]",
      "    [--resume] [--continue-on-failure]",
      "analyze [--out performance_log/e10] [--robot-origin task|participation]",
      "Run requires CoppeliaSim on port 23050 and free robot/agent ports.",
      "Both wall-clock and simulation-clock data and LaTeX figures are exported.",
      "Compile performance_log/e10/e10_figures.tex for the figure PDF."
    ].join("\n"));
    return;
  }
  const args = {};
  const command = argv[0] || "analyze";
  for (let i = 1; i < argv.length; i++) {
    if (["--resume", "--continue-on-failure"].includes(argv[i])) { args[argv[i].slice(2)] = true; continue; }
    if (!argv[i].startsWith("--") || !argv[i + 1] || argv[i + 1].startsWith("--")) throw new Error("Expected --option value: " + argv[i]);
    const key = argv[i].slice(2);
    args[key] = argv[++i];
  }
  const outDir = path.resolve(ROOT, args.out || "performance_log/e10");
  if (command === "analyze") { console.log(JSON.stringify(analyze(outDir, args["robot-origin"] || "task").summary, null, 2)); return; }
  if (command !== "run") throw new Error("Usage: e10Arbitration.js run|analyze [--out directory] [--repetitions 10] [--modes exclusive,region]");
  const number = (name, fallback, integer = false) => {
    const n = Number(args[name] ?? fallback);
    if (!(n > 0) || !Number.isFinite(n) || (integer && !Number.isInteger(n))) throw new Error("Invalid --" + name);
    return n;
  };
  const modes = (args.modes || "exclusive,region").split(",");
  if (new Set(modes).size !== modes.length || modes.some(mode => !MODES.includes(mode))) throw new Error("Modes must be exclusive and/or region");
  await run({ outDir, modes, resume: args.resume === true, continueOnFailure: args["continue-on-failure"] === true,
    repetitions: number("repetitions", 10, true), timeoutMs: number("timeout", 900000),
    startupMs: number("startup-timeout", 300000), intervalMs: number("interval", 100),
    toleranceDeg: number("home-tolerance", 1), settleMs: number("settle", 500) });
}
if (require.main === module) main(process.argv.slice(2)).catch(error => { console.error(error.message); process.exitCode = 1; });
module.exports = { prepareAgents, parseMarker, terminalFailure, scheduledTrials, updateCompletion, taskComplete, inSimulationTime, stats, summarize, figures, analyze, main };
