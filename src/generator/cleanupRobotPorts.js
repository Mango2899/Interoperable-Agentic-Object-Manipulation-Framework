#!/usr/bin/env node

// Frees every port and process the generated robot stack occupies.
//
// The port list is DISCOVERED from the generated manifests rather than written
// down here. A hardcoded list silently stops covering the stack the moment a
// robot is generated or renumbered, and the failure mode is a port that is
// still held by a stale process -- which surfaces much later as an
// EADDRINUSE, or worse, as a server that looks healthy while talking to a
// CoppeliaSim scene from a previous run.

const fs = require("fs");
const path = require("path");
const { execFileSync } = require("child_process");

const repoRoot = path.resolve(__dirname, "..", "..");

// CoppeliaSim's WebSocket remote API. The add-on binds 23050 + process index,
// so a second instance lands on 23051 and so on; a small span is checked
// because a stale second instance holds the port the bridge would otherwise
// have taken.
const SIM_PORTS = [23050, 23051, 23052];

// Matched against full command lines. Script PATHS, not bare words: "pgrep -f
// lowerThingServer" also matches the shell that happens to mention it in its
// own argv -- including the shell running this cleanup.
const ROBOT_PROCESS_PATTERNS = [
  "src/generator/lowerThingServer.js",
  "src/generator/agentThingServer.js"
];
const MAS_PROCESS_PATTERNS = ["jacamo.infra.JaCaMoLauncher"];
const SIM_PROCESS_PATTERNS = ["CoppeliaSim_Edu", "coppeliaSim"];

function parseArgs(argv) {
  return {
    dryRun: argv.includes("--dry-run"),
    keepSim: argv.includes("--keep-sim"),
    help: argv.includes("--help") || argv.includes("-h")
  };
}

function usage() {
  return [
    "Usage:",
    "  npm run robot:cleanup [-- --keep-sim] [-- --dry-run]",
    "",
    "Stops every generated lower Thing and AgentThing, the JaCaMo MAS, and",
    "CoppeliaSim, then verifies their ports are free. Ports are read from",
    "generated/robots/*/manifest.json and config.json, so a newly generated",
    "robot is covered without editing this script.",
    "",
    "  --keep-sim   Leave CoppeliaSim running. Restarting it reloads the scene",
    "               and takes a while, so this is the usual choice when only",
    "               the Node servers need recycling.",
    "  --dry-run    Report what would be stopped, kill nothing."
  ].join("\n");
}

function run(command, args) {
  try {
    return execFileSync(command, args, { encoding: "utf8", stdio: ["ignore", "pipe", "pipe"] });
  } catch (error) {
    // pgrep/fuser/ss all exit non-zero for "nothing matched", which is a normal
    // result here rather than a failure.
    return error.stdout ? String(error.stdout) : "";
  }
}

function unique(values) {
  return [...new Set(values.filter(Boolean))];
}

function readJson(file) {
  try {
    return JSON.parse(fs.readFileSync(file, "utf8"));
  } catch (error) {
    return null;
  }
}

// Every port the generated stack binds, grouped by robot so the report says
// which robot a busy port belongs to.
function discoverRobotPorts() {
  const robotsDir = path.join(repoRoot, "generated", "robots");
  const config = readJson(path.join(repoRoot, "config.json")) || {};
  if (!fs.existsSync(robotsDir)) return [];

  return fs.readdirSync(robotsDir)
    .map((entry) => path.join(robotsDir, entry, "manifest.json"))
    .filter((file) => fs.existsSync(file))
    .map((file) => readJson(file))
    .filter(Boolean)
    .map((manifest) => {
      const ports = manifest.ports || {};
      // config.json is the AgentThing's own source of truth for its two ports;
      // read both so a manifest that has drifted from it still gets cleaned.
      const thingConfig = config[manifest.configKey || manifest.name] || {};
      return {
        name: manifest.name,
        ports: unique([
          ports.lowerThing,
          ports.agentThing,
          ports.websocket,
          thingConfig.TDPort,
          thingConfig.websocketPort
        ].map(Number).filter((port) => Number.isInteger(port) && port > 0)).sort((a, b) => a - b)
      };
    })
    .filter((robot) => robot.ports.length > 0)
    .sort((a, b) => String(a.name).localeCompare(String(b.name)));
}

// Never kill this process, the shell that launched it, or that shell's parent:
// a full command line containing "lowerThingServer.js" matches the invoking
// shell just as well as the server itself.
function protectedPids() {
  const pids = new Set([String(process.pid), String(process.ppid)]);
  const parentOfParent = run("ps", ["-o", "ppid=", "-p", String(process.ppid)]).trim();
  if (parentOfParent) pids.add(parentOfParent);
  return pids;
}

function pidsListening(port) {
  // ss is authoritative for a LISTENing socket and, unlike fuser, distinguishes
  // a listener from a client that merely connected to that port.
  const output = run("ss", ["-ltnpH", `sport = :${port}`]);
  const pids = [...output.matchAll(/pid=(\d+)/g)].map((match) => match[1]);
  if (pids.length > 0) return unique(pids);
  return unique(run("fuser", [`${port}/tcp`]).trim().split(/\s+/));
}

function pidsForPatterns(patterns, protect) {
  return unique(patterns.flatMap((pattern) => run("pgrep", ["-f", pattern]).trim().split(/\s+/)))
    .filter((pid) => !protect.has(pid));
}

function describe(pid) {
  const line = run("ps", ["-o", "args=", "-p", pid]).trim().split("\n")[0] || "";
  return line.length > 90 ? `${line.slice(0, 90)}...` : line;
}

function killPids(pids, signal) {
  for (const pid of pids) {
    run("kill", [`-${signal}`, pid]);
  }
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function main() {
  const args = parseArgs(process.argv.slice(2));
  if (args.help) {
    console.log(usage());
    return;
  }

  const robots = discoverRobotPorts();
  const robotPorts = unique(robots.flatMap((robot) => robot.ports));
  const simPorts = args.keepSim ? [] : SIM_PORTS;
  const allPorts = [...robotPorts, ...simPorts];

  console.log(`Generated robots: ${robots.length > 0 ? robots.map((r) => `${r.name} (${r.ports.join(", ")})`).join("; ") : "none found"}`);
  if (!args.keepSim) console.log(`CoppeliaSim remote API: ${SIM_PORTS.join(", ")}`);

  const protect = protectedPids();
  const patterns = [
    ...ROBOT_PROCESS_PATTERNS,
    ...MAS_PROCESS_PATTERNS,
    ...(args.keepSim ? [] : SIM_PROCESS_PATTERNS)
  ];

  const portPids = unique(allPorts.flatMap(pidsListening)).filter((pid) => !protect.has(pid));
  const patternPids = pidsForPatterns(patterns, protect);
  const pids = unique([...portPids, ...patternPids]);

  if (pids.length === 0) {
    console.log(`Nothing to stop. Ports ${allPorts.join(", ")} are clear.`);
    return;
  }

  console.log(args.dryRun ? "Would stop:" : "Stopping:");
  for (const pid of pids) console.log(`  ${pid}  ${describe(pid)}`);

  if (args.dryRun) {
    console.log("\nDry run: nothing was killed.");
    return;
  }

  killPids(pids, "TERM");
  await delay(1200);

  const remaining = unique([
    ...allPorts.flatMap(pidsListening),
    ...pidsForPatterns(patterns, protect)
  ]).filter((pid) => !protect.has(pid));
  if (remaining.length > 0) {
    console.log(`Force-stopping: ${remaining.join(", ")}`);
    killPids(remaining, "KILL");
    await delay(600);
  }

  const stillBusy = allPorts.filter((port) => pidsListening(port).some((pid) => !protect.has(pid)));
  if (stillBusy.length > 0) {
    console.error(`Ports still busy: ${stillBusy.join(", ")}`);
    process.exit(1);
  }

  console.log(`Ports ${allPorts.join(", ")} are clear.`);
}

if (require.main === module) {
  main().catch((error) => {
    console.error(error.message || error);
    process.exit(1);
  });
}

module.exports = { discoverRobotPorts, SIM_PORTS };
