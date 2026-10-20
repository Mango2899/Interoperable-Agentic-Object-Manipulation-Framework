"use strict";

// Command line, manifest resolution, and the paths a robot's runtime needs.
// Nothing here talks to CoppeliaSim.

const fs = require("fs");
const path = require("path");

// This file is src/generator/core/context.js, so the repository root is three
// levels up. Resolved once, and checked: every generated artifact path in the
// manifests is relative to it, so getting it wrong turns every one of them into
// a "file not found" that names a plausible-looking path in the wrong place.
const REPO_ROOT = path.resolve(__dirname, "..", "..", "..");

if (!fs.existsSync(path.join(REPO_ROOT, "package.json"))) {
  throw new Error(
    `Repository root resolved to ${REPO_ROOT}, which has no package.json. `
    + "core/context.js has moved and REPO_ROOT no longer points at the repository."
  );
}

const LEGACY_ROBWOT_ROOT = "/home/maxchrs/Desktop/MA/RobWoT";

function defaultRobwotRoot() {
  const localSubmodule = path.join(REPO_ROOT, "external", "RobWoT");
  return fs.existsSync(localSubmodule) ? localSubmodule : LEGACY_ROBWOT_ROOT;
}

function parseArgs(argv) {
  const args = {};
  for (let i = 0; i < argv.length; i += 1) {
    const token = argv[i];
    if (!token.startsWith("--")) {
      continue;
    }
    const key = token.slice(2);
    const next = argv[i + 1];
    if (!next || next.startsWith("--")) {
      args[key] = true;
    } else {
      args[key] = next;
      i += 1;
    }
  }
  return args;
}

function usage() {
  return [
    "Usage:",
    "  npm run robot:serve -- --manifest generated/robots/<name>/manifest.json",
    "  npm run robot:serve -- --manifests generated/robots/<a>/manifest.json,generated/robots/<b>/manifest.json [--offsets \"0,0;2,0\"]",
    "",
    "Multi mode loads the first robot's workspace scene and spawns every further robot's",
    "generated model into the same scene at its offset (default 2m apart on x; pass",
    "--offsets so the workspaces intersect, e.g. \"0,0;1.2,0\" for UR10+UR3). Each robot",
    "gets its own mirrored wall clone, gripper, workspace visuals, driver script, and",
    "lower Thing port, and treats the other robots as collision obstacles. All robots",
    "share the first robot's pick cube and support unless --per-robot-grip is passed.",
    "",
    "--exclusive-motion forces the cell back to one arm at a time, whatever the",
    "motions would have shared. This is the serialised baseline to measure against;",
    "without it a motion only waits for arms whose paths claim the same shared space.",
    "",
    "--isolated-cores gives every robot its own process: the cell host keeps the scene",
    "and the motion lock, forks one movement/safety core per arm, and the cores reach",
    "each other over their lower Things. Without it every core is served from the host",
    "process, which needs no extra port and is easier to attach a debugger to.",
    "",
    "Requires CoppeliaSim websocket remote API on localhost:23050 and phase-two artifacts."
  ].join("\n");
}

function fail(message) {
  console.error(message);
  console.error("");
  console.error(usage());
  process.exit(1);
}

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, "utf8"));
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function resolveFromRobwot(moduleName, robwotRoot) {
  const resolved = require.resolve(moduleName, { paths: [robwotRoot] });
  return require(resolved);
}

function createContext(manifestFile) {
  const repoRoot = REPO_ROOT;
  const manifestPath = path.resolve(process.cwd(), manifestFile);
  if (!fs.existsSync(manifestPath)) {
    fail(`Manifest file does not exist: ${manifestPath}`);
  }

  const manifest = readJson(manifestPath);
  const robwotRoot = manifest.pipeline?.robwotRoot || manifest.robwotRoot || defaultRobwotRoot();
  const resolveGenerated = (key) => {
    const relative = manifest.generatedFiles?.[key];
    return relative ? path.join(repoRoot, relative) : null;
  };

  return {
    repoRoot,
    manifestPath,
    manifest,
    robwotRoot,
    paths: {
      scene: resolveGenerated("workspaceScene"),
      sharedScene: manifest.sharedEnvironment?.scene ? path.join(repoRoot, manifest.sharedEnvironment.scene) : null,
      td: resolveGenerated("lowerTd"),
      model: resolveGenerated("model"),
      robotDataPoints: resolveGenerated("robotDataPoints"),
      driver: manifest.pipeline?.runtimeDriver || path.join(robwotRoot, "Library", "Robot_WoT_Server", "robot_driver.lua")
    }
  };
}

function uniqueItems(values) {
  return [...new Set(values.filter(Boolean))];
}

function parseOffsets(value, count) {
  const defaults = Array.from({ length: count }, (_, index) => [index * 2, 0, 0]);
  if (!value) {
    return defaults;
  }
  const parts = String(value).split(";").map((part) => part.trim()).filter((part) => part.length > 0);
  return defaults.map((fallback, index) => {
    if (!parts[index]) {
      return fallback;
    }
    const numbers = parts[index].split(",").map((token) => Number(token.trim()));
    if (numbers.length < 2 || numbers.some((number) => Number.isNaN(number))) {
      fail(`--offsets entry ${index + 1} must be x,y[,z] (got "${parts[index]}").`);
    }
    return [numbers[0], numbers[1], numbers[2] || 0];
  });
}

function assertRuntimeInputs(ctx) {
  const required = [
    ["workspace scene", ctx.paths.scene],
    ["lower TD", ctx.paths.td],
    ["workspace CSV", ctx.paths.robotDataPoints],
    ["runtime driver", ctx.paths.driver]
  ];
  for (const [label, file] of required) {
    if (!file || !fs.existsSync(file)) {
      fail(`${label} not found: ${file}`);
    }
  }
}

module.exports = {
  LEGACY_ROBWOT_ROOT,
  REPO_ROOT,
  assertRuntimeInputs,
  createContext,
  defaultRobwotRoot,
  delay,
  fail,
  parseArgs,
  parseOffsets,
  readJson,
  resolveFromRobwot,
  uniqueItems,
  usage
};
