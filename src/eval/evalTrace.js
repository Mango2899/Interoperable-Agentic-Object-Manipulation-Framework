"use strict";

// Shared measurement plumbing for the trace-based evaluations (E11, E12).
//
// Extracted from e11Scaling.js so that E12 measures the same way E11 does
// rather than re-deriving it: one JSONL trace per run, written from inside the
// JaCaMo JVM by e11.Trace, with System.nanoTime() as the clock. Every duration
// in both experiments is a difference of two `mono_ms` fields from that one
// clock, so it is immune to wall-clock adjustment and comparable across agents
// -- all five agents share the JVM the Gradle run starts.
const fs = require("fs");
const path = require("path");
const { spawn } = require("child_process");

const ROOT = path.resolve(__dirname, "../..");

const read = p => JSON.parse(fs.readFileSync(p, "utf8"));
const relative = p => path.relative(ROOT, p);
const append = (p, v) => fs.appendFileSync(p, JSON.stringify(v) + "\n");
const quote = s => "'" + String(s).replace(/'/g, "'\\''") + "'";

function write(p, v) {
  fs.mkdirSync(path.dirname(p), { recursive: true });
  const temporary = p + `.${process.pid}.tmp`;
  fs.writeFileSync(temporary, JSON.stringify(v, null, 2) + "\n");
  fs.renameSync(temporary, p);
}

// Instrumentation edits production plans by exact-text substitution. A plan
// that has been reworded must fail loudly here rather than silently produce a
// run with a missing boundary, which analysis would report as a null duration.
function replaceOnce(source, before, after) {
  if (source.split(before).length !== 2) throw Error(`Instrumentation anchor changed: ${before}`);
  return source.replace(before, after);
}

function readLines(file, offset = 0) {
  if (!fs.existsSync(file)) return [];
  const data = fs.readFileSync(file).subarray(offset).toString("utf8");
  // An append in progress may leave a partial final record; retry it next poll.
  return data.split("\n").slice(0, -1).filter(Boolean).map(line => JSON.parse(line));
}

const csv = rows => {
  if (!rows.length) return "";
  const keys = Object.keys(rows[0]), enc = v => JSON.stringify(v == null ? "" : typeof v === "object" ? JSON.stringify(v) : String(v));
  return keys.map(enc).join(",") + "\n" + rows.map(r => keys.map(k => enc(r[k])).join(",")).join("\n") + "\n";
};

function meanAndSampleSd(values) {
  if (!values.length) return { mean: null, sd: null };
  const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
  const sd = values.length > 1
    ? Math.sqrt(values.reduce((sum, value) => sum + (value - mean) ** 2, 0) / (values.length - 1))
    : null;
  return { mean, sd };
}

function texNumber(value, digits = 3) {
  return Number(value).toFixed(digits);
}

// The route search is instrumented identically in both experiments, so the
// route-discovery duration E12 reports is the quantity E11 plots against cell
// size and not a second, subtly different definition of it.
function instrumentRoutes(source) {
  source = replaceOnce(source, "    !forgetRoutes;\n    if (hasSink(Goal))", "    e11.event(phase_start, route_discovery, Goal);\n    !forgetRoutes;\n    if (hasSink(Goal))");
  source = replaceOnce(source, "+routeIsLocal(Goal)", "+routeIsLocal(Goal);\n        e11.event(route_end, route_discovery, local, Goal)");
  source = replaceOnce(source, "+noRouteTo(Task)\n        } else", "+noRouteTo(Task);\n            e11.event(route_end, route_discovery, no_route, Task)\n        } else");
  source = replaceOnce(source, '        .print("No route from ", MyRobot, " to the goal: every branch came back empty")',
    '        .print("No route from ", MyRobot, " to the goal: every branch came back empty");\n        e11.event(route_end, route_discovery, no_route, Task)');
  // routeSettled is asserted BEFORE final source-edge pricing, so that belief is
  // not a valid timing endpoint. Capture the actual decision after routeTotal.
  return replaceOnce(source, "+routeTotal(Task, Best);", "+routeTotal(Task, Best);\n        e11.event(route_end, route_discovery, success, Task, Best, [MyRobot|BestPath]);");
}

const LOGGING = "handlers=java.util.logging.ConsoleHandler\n.level=INFO\n"
  + "java.util.logging.ConsoleHandler.level=ALL\n"
  + "java.util.logging.ConsoleHandler.formatter=jason.runtime.MASConsoleLogFormatter\n";

// Only Gradle is started, and only when the user asks for it. Scene generation,
// the lower servers and the AgentThings stay under the user's control in both
// experiments, so a measurement never silently moves a robot.
async function runGradle({ jcm, directory, trace, env = {}, onLine = () => {} }) {
  const args = ["--console=plain", "-Pheadless", "run",
    "--args=" + relative(jcm) + " --log-conf " + relative(path.join(path.dirname(jcm), "logging.properties"))];
  const child = spawn(path.join(ROOT, "gradlew"), args, {
    cwd: ROOT, env: { ...process.env, WOMAT_E11_TRACE: trace, ...env }, stdio: ["inherit", "pipe", "pipe"]
  });
  const log = fs.createWriteStream(path.join(directory, "gradle.log"));
  let carry = "";
  for (const stream of [child.stdout, child.stderr]) stream.on("data", chunk => {
    log.write(chunk);
    process.stdout.write(chunk);
    carry += chunk.toString("utf8");
    const lines = carry.split("\n");
    carry = lines.pop();
    for (const line of lines) onLine(line);
  });
  const forward = signal => child.kill(signal);
  const interrupt = () => forward("SIGINT"), terminate = () => forward("SIGTERM");
  process.on("SIGINT", interrupt); process.on("SIGTERM", terminate);
  child.on("error", error => append(trace, { event: "launch_error", t: Date.now(), error: error.message }));
  const exit = await new Promise(resolve => child.on("close", (code, signal) => {
    append(trace, { event: "gradle_exit", t: Date.now(), code, signal });
    log.end(); resolve({ code, signal });
  }));
  process.off("SIGINT", interrupt); process.off("SIGTERM", terminate);
  return { exit, args };
}

module.exports = { ROOT, read, write, append, relative, quote, replaceOnce, readLines, csv, meanAndSampleSd, texNumber, instrumentRoutes, runGradle, LOGGING };
