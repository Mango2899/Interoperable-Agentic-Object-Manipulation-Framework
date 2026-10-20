#!/usr/bin/env node

// Takes a copy of everything one generated robot consists of, before it is
// regenerated into something else.
//
// Regenerating a robot rewrites its manifest, its plans, its Things and its
// CoppeliaSim model in place, and rebuilds the cell descriptor and scene around
// it. None of that is recoverable from the URDF alone: the hand-tuned runtime
// values, the joint classification, the ports, the agent defaults and the
// baked model are all products of the run that produced them. A copy taken
// first is the difference between swapping an arm and losing one.
//
// What belongs to a robot is READ from its manifest, the same way deleteRobot
// works it out, rather than listed here -- a hardcoded list stops covering the
// stack the moment the generator emits one more file, and the failure mode is
// an archive that looks complete and is not.
//
// The archive lives OUTSIDE generated/robots. Anything under that directory
// with a manifest.json in it is a robot as far as the rest of the repo is
// concerned: cleanupRobotPorts would read the copy's ports and kill the live
// robot holding them, and the plan tests would assert against a robot that is
// not in the cell.

const fs = require("fs");
const path = require("path");
const crypto = require("crypto");

const repoRoot = path.resolve(__dirname, "..", "..");

function usage() {
  return [
    "Usage:",
    "  npm run archive:robot -- --name <robotName> [--as <folderName>] [--into archive] [--dry-run] [--force]",
    "",
    "  --name    the generated robot to copy (e.g. Robot4)",
    "  --as      the folder to copy it into (default: the robot's name)",
    "  --into    where that folder is created (default: archive/, relative to the repo root)",
    "  --dry-run list what would be copied and write nothing",
    "  --force   overwrite an archive folder that already exists",
    "",
    "The cell files a robot takes part in -- the shared descriptor, the .jcm, the",
    "combined scenes and the pair interaction spaces -- are copied alongside it,",
    "because regenerating the robot rebuilds all of them."
  ].join("\n");
}

function parseArgs(argv) {
  const args = { help: argv.includes("--help") || argv.includes("-h") };
  for (let index = 0; index < argv.length; index += 1) {
    const token = argv[index];
    if (!token.startsWith("--")) continue;
    const key = token.slice(2);
    const next = argv[index + 1];
    if (!next || next.startsWith("--")) {
      args[key] = true;
    } else {
      args[key] = next;
      index += 1;
    }
  }
  return args;
}

function fail(message) {
  console.error(message);
  console.error("");
  console.error(usage());
  process.exit(1);
}

function readJsonIfExists(file) {
  if (!fs.existsSync(file)) return null;
  return JSON.parse(fs.readFileSync(file, "utf8"));
}

// Every path a copy of this robot has to carry, repo-relative so the archive
// mirrors the tree and restoring is a copy back rather than a reconstruction.
function robotPaths(name, manifest) {
  const fromManifest = Object.values(manifest?.generatedFiles || {})
    .filter((target) => typeof target === "string");
  // The same fallbacks deleteRobot uses, for a robot whose manifest predates a
  // generatedFiles entry or is missing entirely.
  const fallbacks = [
    path.join("generated", "robots", name),
    path.join("src", "WoT", "generated", `${name}_agent_thing.ts`),
    path.join("src", "WoT", "generated", `${name}_lower_thing.ts`),
    path.join("src", "agt", "generated", `${name}_agent.asl`),
    path.join("src", "agt", "generated", `${name.toLowerCase()}_agent.asl`),
    path.join("src", "td", "generated", `${name}.agent.td.json`)
  ];
  return [...new Set([...fromManifest, ...fallbacks])];
}

// The cell is rebuilt around a regenerated robot, so the arrangement this robot
// was part of has to be copied with it. Discovered by name rather than listed:
// a cell file is named after every robot in it.
function cellPaths(root, name, manifest) {
  const found = [];
  const shared = manifest?.sharedEnvironment || {};
  for (const value of [shared.descriptor, shared.scene, shared.environmentScene]) {
    if (typeof value === "string") found.push(value);
  }
  const robotsDir = path.join(root, "generated", "robots");
  if (fs.existsSync(robotsDir)) {
    for (const entry of fs.readdirSync(robotsDir)) {
      const isCellFile = entry.includes("_") && entry.includes(name) && !fs.statSync(path.join(robotsDir, entry)).isDirectory();
      if (isCellFile) found.push(path.join("generated", "robots", entry));
    }
  }
  const interactionsDir = path.join(robotsDir, "interactions");
  if (fs.existsSync(interactionsDir)) {
    for (const entry of fs.readdirSync(interactionsDir)) {
      if (entry.split("__").includes(name)) {
        found.push(path.join("generated", "robots", "interactions", entry));
      }
    }
  }
  return [...new Set(found)].filter((relative) => fs.existsSync(path.join(root, relative)));
}

function walkFiles(absolute, relative, out) {
  const stats = fs.statSync(absolute);
  if (stats.isDirectory()) {
    for (const entry of fs.readdirSync(absolute).sort()) {
      walkFiles(path.join(absolute, entry), path.posix.join(relative, entry), out);
    }
    return;
  }
  out.push({ absolute, relative, size: stats.size });
}

function digest(file) {
  return crypto.createHash("sha256").update(fs.readFileSync(file)).digest("hex");
}

function humanSize(bytes) {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

// The copy itself, with the repository root passed in rather than read from
// this file's location -- a test drives it against a tree it built, and an
// archiver that can only ever run against one directory cannot be tested
// without writing into the real one.
function archiveRobot(options) {
  const { root, name, folder = name, into = "archive", dryRun = false, force = false } = options;
  const manifestFile = path.join(root, "generated", "robots", name, "manifest.json");
  const manifest = readJsonIfExists(manifestFile);
  if (!manifest) throw new Error(`No generated robot named ${name}: ${path.relative(root, manifestFile)} does not exist.`);

  const archiveRoot = path.resolve(root, into, folder);
  if (fs.existsSync(archiveRoot) && !force && !dryRun) {
    throw new Error(`${path.relative(root, archiveRoot)} already exists. Pass --force to overwrite it.`);
  }
  // An archive under generated/robots would BE a robot: anything there with a
  // manifest.json in it is one, so cleanupRobotPorts would read the copy's
  // ports and kill the live robot holding them.
  const robotsDir = path.join(root, "generated", "robots");
  if (archiveRoot === robotsDir || archiveRoot.startsWith(`${robotsDir}${path.sep}`)) {
    throw new Error("An archive cannot live under generated/robots; everything there is treated as a live robot.");
  }

  const sources = [
    ...robotPaths(name, manifest).map((relative) => ({ relative, kind: "robot" })),
    ...cellPaths(root, name, manifest).map((relative) => ({ relative, kind: "cell" }))
  ].filter((entry, index, all) => all.findIndex((other) => other.relative === entry.relative) === index)
    .filter((entry) => fs.existsSync(path.join(root, entry.relative)));

  // A manifest names files that also live inside the robot's own directory, and
  // that directory is a source in its own right. Walking both reaches the same
  // file twice, which copies it twice and lists it twice in SHA256SUMS -- a
  // checksum file with duplicate lines still verifies, so the inflated count
  // would be the only sign anything was wrong.
  const walked = [];
  for (const source of sources) {
    walkFiles(path.join(root, source.relative), source.relative, walked);
  }
  const files = walked.filter(
    (file, index) => walked.findIndex((other) => other.relative === file.relative) === index
  );
  const total = files.reduce((sum, file) => sum + file.size, 0);
  const result = { archiveRoot, sources, files, total, manifest };
  if (dryRun) return result;

  fs.rmSync(archiveRoot, { recursive: true, force: true });
  const lines = [];
  for (const file of files) {
    const target = path.join(archiveRoot, "files", file.relative);
    fs.mkdirSync(path.dirname(target), { recursive: true });
    fs.copyFileSync(file.absolute, target);
    lines.push(`${digest(file.absolute)}  ${file.relative}`);
  }
  fs.writeFileSync(path.join(archiveRoot, "SHA256SUMS"), `${lines.join("\n")}\n`, "utf8");

  // config.json is shared by every robot, so the archive keeps this robot's
  // entry alone rather than a copy of the whole file -- restoring the whole
  // file would take every other robot's ports back to whatever they were today.
  const configKey = manifest.configKey || name;
  const config = readJsonIfExists(path.join(root, "config.json")) || {};
  fs.writeFileSync(
    path.join(archiveRoot, "config.entry.json"),
    `${JSON.stringify({ [configKey]: config[configKey] ?? null }, null, 2)}\n`,
    "utf8"
  );
  fs.writeFileSync(path.join(archiveRoot, "README.md"), readmeFor(root, archiveRoot, folder, name, manifest, configKey), "utf8");
  return result;
}

function readmeFor(root, archiveRoot, folder, name, manifest, configKey) {
  const relative = path.relative(root, archiveRoot).split(path.sep).join("/");
  return [
    `# ${folder}`,
    "",
    `A copy of the generated robot **${name}** as it stood on ${new Date().toISOString().slice(0, 10)},`,
    "taken before regenerating it from a different URDF.",
    "",
    "## What it was built from",
    "",
    `- URDF: \`${manifest.urdfPath || "(not recorded in the manifest)"}\``,
    `- Model adopted from: \`${manifest.pipeline?.sourceScene || "(none -- imported from the URDF)"}\``,
    `- Platform: ${manifest.mobility?.type || "unknown"}, base link \`${manifest.mobility?.baseLink || "?"}\``,
    `- Arm joints: ${(manifest.mobility?.armJoints || []).join(", ") || "-"}`,
    `- Gripper joints: ${(manifest.mobility?.gripperJoints || []).join(", ") || "-"}`,
    `- Ports: lower ${manifest.ports?.lowerThing}, agent ${manifest.ports?.agentThing}, websocket ${manifest.ports?.websocket}`,
    `- Cell: \`${manifest.sharedEnvironment?.descriptor || "-"}\``,
    "",
    "## Layout",
    "",
    "- `files/` mirrors the repository tree, so every path here is where it came from.",
    "- `SHA256SUMS` covers `files/`; verify with `cd files && sha256sum -c ../SHA256SUMS`.",
    `- \`config.entry.json\` holds only this robot's \`config.json\` entry (\`${configKey}\`).`,
    "  The rest of that file belongs to the other robots and is deliberately not copied.",
    "",
    "## Restoring it",
    "",
    "```bash",
    "# from the repository root",
    `rsync -a ${relative}/files/ ./`,
    "# then put the config.json entry back by hand from config.entry.json",
    "```",
    "",
    "The cell files copied alongside the robot describe the arrangement it was part",
    "of. Restoring the robot without them leaves a descriptor that names an arm the",
    "scene no longer contains.",
    ""
  ].join("\n");
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  if (args.help) {
    console.log(usage());
    return;
  }
  const name = args.name;
  if (!name || typeof name !== "string" || !/^[A-Za-z][A-Za-z0-9_]*$/.test(name)) {
    fail("Robot name must start with a letter and contain only letters, numbers, and underscores.");
  }
  const dryRun = args["dry-run"] === true;
  const folder = typeof args.as === "string" ? args.as : name;

  let result;
  try {
    result = archiveRobot({
      root: repoRoot,
      name,
      folder,
      into: typeof args.into === "string" ? args.into : "archive",
      dryRun,
      force: args.force === true
    });
  } catch (error) {
    fail(error.message);
    return;
  }

  console.log(`${dryRun ? "Would archive" : "Archiving"} ${name} to ${path.relative(repoRoot, result.archiveRoot)}`);
  console.log("");
  for (const source of result.sources) {
    console.log(`  ${source.kind === "cell" ? "cell " : "robot"}  ${source.relative}`);
  }
  console.log("");
  console.log(`  ${result.files.length} files, ${humanSize(result.total)}`);
  if (dryRun) return;
  console.log("");
  console.log(`  wrote ${path.relative(repoRoot, result.archiveRoot)}/README.md, SHA256SUMS, config.entry.json`);
}

if (require.main === module) {
  main();
}

module.exports = { archiveRobot, cellPaths, robotPaths };
