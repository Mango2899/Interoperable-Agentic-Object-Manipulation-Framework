#!/usr/bin/env node

const fs = require("fs");
const path = require("path");

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
    "  npm run delete:robot -- --name <robotName> [--dry-run]",
    "",
    "Deletes generated pipeline files for one robot and removes its config.json entry."
  ].join("\n");
}

function fail(message) {
  console.error(message);
  console.error("");
  console.error(usage());
  process.exit(1);
}

function validateRobotName(name) {
  if (!name || !/^[A-Za-z][A-Za-z0-9_]*$/.test(name)) {
    fail("Robot name must start with a letter and contain only letters, numbers, and underscores.");
  }
}

function readJsonIfExists(file) {
  if (!fs.existsSync(file)) {
    return null;
  }
  return JSON.parse(fs.readFileSync(file, "utf8"));
}

function writeJson(file, value) {
  fs.writeFileSync(file, JSON.stringify(value, null, 2) + "\n", "utf8");
}

function removeIfExists(target, dryRun, removed) {
  if (!fs.existsSync(target)) {
    return;
  }
  removed.push(target);
  if (!dryRun) {
    fs.rmSync(target, { recursive: true, force: true });
  }
}

function removeEmptyDir(dir, dryRun) {
  if (!fs.existsSync(dir)) {
    return;
  }
  if (fs.readdirSync(dir).length === 0 && !dryRun) {
    fs.rmdirSync(dir);
  }
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  if (args.help) {
    console.log(usage());
    return;
  }

  const name = args.name;
  validateRobotName(name);

  const dryRun = args["dry-run"] === true;
  const repoRoot = path.resolve(__dirname, "..", "..");
  const robotRoot = path.join(repoRoot, "generated", "robots", name);
  const manifestFile = path.join(robotRoot, "manifest.json");
  const manifest = readJsonIfExists(manifestFile);
  const configKey = manifest?.configKey || name;

  const manifestTargets = Object.values(manifest?.generatedFiles || {})
    .filter((target) => typeof target === "string")
    .map((target) => path.join(repoRoot, target));

  const fallbackTargets = [
    robotRoot,
    path.join(repoRoot, "src", "WoT", "generated", `${name}_agent_thing.ts`),
    path.join(repoRoot, "src", "WoT", "generated", `${name}_lower_thing.ts`),
    path.join(repoRoot, "src", "agt", "generated", `${name}_agent.asl`),
    path.join(repoRoot, "src", "agt", "generated", `${name.toLowerCase()}_agent.asl`),
    path.join(repoRoot, "src", "td", "generated", `${name}.agent.td.json`)
  ];
  const generatedTargets = Array.from(new Set([...manifestTargets, ...fallbackTargets]));

  const removed = [];
  for (const target of generatedTargets) {
    removeIfExists(target, dryRun, removed);
  }

  const configFile = path.join(repoRoot, "config.json");
  const config = readJsonIfExists(configFile);
  const removedConfigKeys = [];
  if (config) {
    for (const key of new Set([configKey, name])) {
      if (Object.prototype.hasOwnProperty.call(config, key)) {
        removedConfigKeys.push(key);
        if (!dryRun) {
          delete config[key];
        }
      }
    }
    if (removedConfigKeys.length > 0 && !dryRun) {
      writeJson(configFile, config);
    }
  }

  const generatedDirs = [
    path.join(repoRoot, "src", "WoT", "generated"),
    path.join(repoRoot, "src", "agt", "generated"),
    path.join(repoRoot, "src", "td", "generated"),
    path.join(repoRoot, "generated", "robots"),
    path.join(repoRoot, "generated")
  ];
  for (const dir of generatedDirs) {
    removeEmptyDir(dir, dryRun);
  }

  console.log(`${dryRun ? "Would delete" : "Deleted"} generated robot: ${name}`);
  if (removed.length > 0) {
    console.log("");
    console.log("Generated paths:");
    for (const target of removed) {
      console.log(`  ${path.relative(repoRoot, target)}`);
    }
  } else {
    console.log("");
    console.log("Generated paths: none found");
  }

  console.log("");
  if (removedConfigKeys.length > 0) {
    console.log(`${dryRun ? "Would remove" : "Removed"} config.json entries: ${removedConfigKeys.join(", ")}`);
  } else {
    console.log("config.json entries: none found");
  }
}

if (require.main === module) {
  main();
}
