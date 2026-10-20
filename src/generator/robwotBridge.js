#!/usr/bin/env node

const fs = require("fs");
const path = require("path");

const { BASE_PEDESTAL_Z } = require("./sharedCell");
const { rotateLocal, rotateWorldToLocal } = require("./handoverGeometry");
const {
  gripFromSite,
  loadedSite,
  parseCellSites,
  payloadSpawnPosition,
  resolveCellSites
} = require("./cellSites");
const {
  chainJointIntervals,
  gripperSource,
  lockedChainPositions,
  radiansTriple,
  selectArmChainJoints,
  toolControlPointOffset
} = require("./core/luaDriver");

const LEGACY_ROBWOT_ROOT = "/home/maxchrs/Desktop/MA/RobWoT";

function defaultRobwotRoot() {
  const localSubmodule = path.resolve(__dirname, "..", "..", "external", "RobWoT");
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
    if (next === undefined || next.startsWith("--")) {
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
    "  npm run robot:pipeline -- --manifest generated/robots/<name>/manifest.json --dry-run",
    "  npm run robot:pipeline -- --manifest generated/robots/<name>/manifest.json --generate-model",
    "  npm run robot:pipeline -- --manifest generated/robots/<name>/manifest.json --adopt-model",
    "    --adopt-model takes the robot from an already-built scene instead of importing its URDF, for an arm whose",
    "    end effector is a mechanism a URDF cannot describe. Needs pipeline.sourceScene, .sourceModel and .sourceGripper.",
    "  npm run robot:pipeline -- --manifest generated/robots/<name>/manifest.json --generate-td",
    "  npm run robot:pipeline -- --manifest generated/robots/<name>/manifest.json --full-pipeline",
    "  npm run robot:pipeline -- --manifests generated/robots/<a>/manifest.json,generated/robots/<b>/manifest.json --generate-shared-cell [--offsets \"0,0;2,0\"] [--rotations \"0;180\"] [--orientations \"0,0,0;0,0,180\"] [--wall-offsets \"0,0;0.5,0\"] [--wall-yaws \";90;\"] [--resample \"Robot3\"] [--interaction-pairs \"Robot1:Robot2\"] [--static-chain]",
    "  --wall-yaws turns a wall to an absolute WORLD yaw in degrees instead of the one its robot's stance gives it;",
    "    the panel's 0.5 m side lies along its own +Y, so 0 puts it along world Y and 90 along world X. Empty entries keep the stance heading.",
    "  --static-chain emits handoverSuccessor facts: the configured chain, kept as the baseline a discovered route is compared against.",
    "  --sites \"a@0.9,0,0.08,0,cube_a; b@1.25,-2,0.08,90\" places the cube supports in the CELL's frame: alias@x,y,z[,yawDeg][,payloadAlias].",
    "    The site carrying a payload is where the cube starts; every other site is an empty destination. Which arm serves which is discovered, never declared.",
    "  npm run robot:pipeline -- --manifests generated/robots/<a>/manifest.json,generated/robots/<b>/manifest.json --update-shared-space-visual",
    "",
    "CoppeliaSim must be running with its websocket remote API on localhost:23050 for generation and visualization modes."
  ].join("\n");
}

function fail(message) {
  console.error(message);
  console.error("");
  console.error(usage());
  process.exit(1);
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, "utf8"));
}

function writeJson(file, value) {
  ensureDir(path.dirname(file));
  fs.writeFileSync(file, JSON.stringify(value, null, 2) + "\n", "utf8");
}

function ensureDir(dir) {
  fs.mkdirSync(dir, { recursive: true });
}

function copyFileIfExists(source, target) {
  if (fs.existsSync(source)) {
    ensureDir(path.dirname(target));
    fs.copyFileSync(source, target);
    return true;
  }
  return false;
}

function luaString(value) {
  return `[[${String(value).replace(/\]\]/g, "]]..']]'..[[")}]]`;
}

function luaArray(values) {
  return `{${values.join(",")}}`;
}

// One number, safe to drop into generated Lua whatever its sign.
//
// Lua reads "--" as a line comment. An expression built as `a-${b}` with a
// negative b therefore emits `a--0.9063...` and everything after it on that
// line -- including the closing brace of the table being built -- becomes a
// comment, so the chunk fails to compile and executeScriptString reports a
// bare "operation failed". Parenthesising negatives keeps the arithmetic
// readable and the sign intact. This stayed hidden for as long as every robot
// in a cell was yawed 0 or 180 degrees, whose sines are 0 and +1.2e-16.
function luaNumber(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) {
    throw new Error(`Refusing to emit a non-finite number into Lua: ${value}`);
  }
  return number < 0 ? `(${number})` : String(number);
}

function parseOffsets(value, count) {
  const defaults = Array.from({ length: count }, (_, index) => [index * 2, 0, 0]);
  if (!value) {
    return defaults;
  }
  const parts = String(value).split(";").map((part) => part.trim()).filter(Boolean);
  return defaults.map((fallback, index) => {
    if (!parts[index]) {
      return fallback;
    }
    const numbers = parts[index].split(",").map((token) => Number(token.trim()));
    if (numbers.length < 2 || numbers.some((number) => Number.isNaN(number))) {
      throw new Error(`--offsets entry ${index + 1} must be x,y[,z] (got "${parts[index]}").`);
    }
    return [numbers[0], numbers[1], numbers[2] || 0];
  });
}

function parseRotations(value, count) {
  const parts = value
    ? String(value).split(";").map((part) => part.trim()).filter(Boolean)
    : [];
  return Array.from({ length: count }, (_, index) => {
    const degrees = parts[index] === undefined ? 0 : Number(parts[index]);
    if (Number.isNaN(degrees)) {
      throw new Error(`--rotations entry ${index + 1} must be a yaw angle in degrees.`);
    }
    return degrees * Math.PI / 180;
  });
}

// Where each robot's wall obstacle stands, relative to where the scene template
// puts it.
//
// The template wall sits a fixed vector from the template robot's base, and
// every cloned wall inherits that vector, so an arm mounted like the template
// one is equally boxed in by it. That is fine until a robot has to face several
// neighbours: a wall 20 cm from the base does not just block the direction it
// stands in, it blocks every base angle whose SWEEP crosses it, which for a
// close wall is most of the circle. The robot is then locked into whichever
// sector it starts in and can never turn to its other partners.
//
// So the displacement is per robot, in that robot's own (yaw-rotated) frame,
// and defaults to zero -- an unlisted robot keeps the template placement.
function parseWallOffsets(value, count) {
  // Empty entries are kept, not dropped: the usual reason to reach for this
  // option is that one robot in the middle of the cell needs its wall moved,
  // and collapsing ";;" would hand that displacement to the first robot
  // instead -- a misplaced wall nobody notices until a full resample later.
  const parts = value === undefined || value === true
    ? []
    : String(value).split(";").map((part) => part.trim());
  if (parts.length > count) {
    throw new Error(`--wall-offsets lists ${parts.length} entries for ${count} robots; entries are positional and must line up with --manifests.`);
  }
  return Array.from({ length: count }, (_, index) => {
    if (!parts[index]) {
      return [0, 0, 0];
    }
    const numbers = parts[index].split(",").map((token) => Number(token.trim()));
    if (numbers.length < 2 || numbers.some((number) => !Number.isFinite(number))) {
      throw new Error(`--wall-offsets entry ${index + 1} must be dx,dy[,dz] in the robot's own frame (got "${parts[index]}").`);
    }
    return [numbers[0], numbers[1], numbers[2] || 0];
  });
}

// Which way each robot's wall FACES, when the stance it inherits is the wrong
// way round.
//
// The template wall is cloned upright and turned with the robot, so its heading
// is whatever the mount happens to give: an arm standing at yaw 50 gets a wall
// at 140. That is the right default -- the wall is meant to box an arm in the
// same way the template wall boxes the template arm -- but it is not a heading
// anything can choose. An experiment that needs a wall ACROSS a handover
// direction needs the panel turned to the CELL's frame, not to the arm's.
//
// So an entry here is the wall's absolute world yaw in degrees, and the panel's
// long side lies along its own +Y: yaw 0 puts the 0.5 m side along world Y, yaw
// 90 puts it along world X. An empty entry keeps the inherited heading. The
// DISPLACEMENT stays in the robot's own frame either way, so --wall-offsets
// means the same thing whether or not the wall was turned.
function parseWallYaws(value, count) {
  const parts = value === undefined || value === true
    ? []
    : String(value).split(";").map((part) => part.trim());
  if (parts.length > count) {
    throw new Error(`--wall-yaws lists ${parts.length} entries for ${count} robots; entries are positional and must line up with --manifests.`);
  }
  return Array.from({ length: count }, (_, index) => {
    if (!parts[index]) {
      return null;
    }
    const degrees = Number(parts[index]);
    if (!Number.isFinite(degrees)) {
      throw new Error(`--wall-yaws entry ${index + 1} must be a world yaw in degrees (got "${parts[index]}").`);
    }
    return degrees * Math.PI / 180;
  });
}

// Which robots are re-sampled in the shared environment.
//
// Sampling is what makes a shared-cell run expensive, and it is also not
// reproducible: the workspace driver never seeds math.random, so re-sampling a
// robot whose surroundings did not change still moves its reachable set and
// invalidates a calibration that was tuned against the old one. Only a robot
// whose obstacles actually moved needs a new workspace, so the run can be told
// to leave the others' CSVs alone.
function parseResampleSelection(value, contexts) {
  if (value === undefined || value === true) {
    return contexts.map(() => true);
  }
  const known = new Set(contexts.map((ctx) => ctx.manifest.name));
  const requested = String(value).split(",").map((name) => name.trim()).filter(Boolean);
  for (const name of requested) {
    if (!known.has(name)) {
      throw new Error(`--resample names ${name}, which is not one of the robots in this cell: ${[...known].join(", ")}.`);
    }
  }
  const selected = new Set(requested);
  return contexts.map((ctx) => selected.has(ctx.manifest.name));
}

function parseOrientations(value, rotations, count) {
  const parts = value
    ? String(value).split(";").map((part) => part.trim()).filter(Boolean)
    : [];
  return Array.from({ length: count }, (_, index) => {
    if (!parts[index]) return [0, 0, rotations[index]];
    const degrees = parts[index].split(",").map((token) => Number(token.trim()));
    if (degrees.length !== 3 || degrees.some((number) => !Number.isFinite(number))) {
      throw new Error(`--orientations entry ${index + 1} must be roll,pitch,yaw in degrees.`);
    }
    return degrees.map((number) => number * Math.PI / 180);
  });
}

// The one rotation this file is allowed to know about.
//
// CoppeliaSim's alpha-beta-gamma Euler matrix is Rx(alpha)*Ry(beta)*Rz(gamma).
// This file used to carry its own Rz*Ry*Rx copy, which agrees with the
// simulator only while at most one angle is non-zero -- true of every cell
// built so far, because a robot was either upright at [0,0,yaw] or the CR5 at
// its single working [0,-90,180]. The moment a mount uses two angles the two
// conventions diverge, and the workspace CSV, the pair bounding boxes and the
// drawn shared space are all silently expressed in a frame the scene does not
// use. handoverGeometry already had the simulator's composition; there is no
// second convention to keep in step now.
const rotatePoint = rotateLocal;
const inverseRotatePoint = rotateWorldToLocal;

// The angle a robot's stance turns the floor plane through.
//
// The wall is cloned upright and spun about world Z, so it needs a heading, not
// an Euler triple. Reading orientation[2] for it only works when the mount is a
// bare yaw: in Rx*Ry*Rz the LAST angle is the innermost rotation, applied in the
// model's own frame, so for a CR arm standing at [-90, beta, 90] it is a
// constant 90 and every wall in the cell would end up parallel.
//
// Model +Y is the axis both arm families keep pointing along their own left,
// horizontal whichever way the model is mounted, so crossing its world image
// with world Z recovers the stance's heading without the pipeline having to
// know which axis a given URDF stands on. For a bare-yaw mount this returns
// exactly orientation[2], so cells built before this reproduce unchanged.
function floorPlaneYaw(orientation) {
  const left = rotatePoint({ x: 0, y: 1, z: 0 }, orientation);
  const forward = { x: left.y, y: -left.x };
  if (Math.hypot(forward.x, forward.y) < 1e-6) {
    // Model +Y points straight up or down, so it says nothing about heading.
    // Nothing in the cell is mounted that way; fall back rather than guess.
    return orientation[2];
  }
  return Math.atan2(forward.y, forward.x);
}

function pairKey(first, second) {
  return [String(first), String(second)]
    .sort((left, right) => left.localeCompare(right, undefined, { numeric: true, sensitivity: "base" }))
    .join("__");
}

function parseInteractionPairs(value, contexts) {
  if (!value) return null;
  const known = new Set(contexts.map((ctx) => ctx.manifest.name));
  const seen = new Set();
  return String(value).split(";").map((entry) => entry.trim()).filter(Boolean).map((entry) => {
    const robots = entry.split(":").map((name) => name.trim()).filter(Boolean);
    if (robots.length !== 2 || robots[0] === robots[1] || robots.some((name) => !known.has(name))) {
      throw new Error(`Invalid --interaction-pairs entry "${entry}"; use two different generated robot names, e.g. Robot1:Robot2.`);
    }
    const key = pairKey(robots[0], robots[1]);
    if (seen.has(key)) throw new Error(`Duplicate interaction pair: ${entry}`);
    seen.add(key);
    const ordered = key.split("__");
    return { key, robots: ordered, ownerRobot: ordered[0], discovery: "configured" };
  });
}

function transformedWorkspaceBounds(ctx, offset, orientation) {
  const [minX, maxX, minY, maxY, minZ, maxZ] = csvExtremes(ctx.paths.robotDataPoints);
  const corners = [minX, maxX].flatMap((x) => [minY, maxY].flatMap((y) => [minZ, maxZ].map((z) => {
    const rotated = rotatePoint({ x, y, z }, orientation);
    return { x: offset[0] + rotated.x, y: offset[1] + rotated.y, z: offset[2] + rotated.z };
  })));
  return {
    min: { x: Math.min(...corners.map((p) => p.x)), y: Math.min(...corners.map((p) => p.y)), z: Math.min(...corners.map((p) => p.z)) },
    max: { x: Math.max(...corners.map((p) => p.x)), y: Math.max(...corners.map((p) => p.y)), z: Math.max(...corners.map((p) => p.z)) }
  };
}

function boundsOverlap(left, right) {
  return ["x", "y", "z"].every((axis) => Math.min(left.max[axis], right.max[axis]) > Math.max(left.min[axis], right.min[axis]));
}

function discoverInteractionPairs(contexts, offsets, orientations, configuredPairs) {
  const explicit = parseInteractionPairs(configuredPairs, contexts);
  if (explicit) return explicit;
  const worldBounds = contexts.map((ctx, index) => transformedWorkspaceBounds(ctx, offsets[index], orientations[index]));
  const pairs = [];
  for (let first = 0; first < contexts.length; first += 1) {
    for (let second = first + 1; second < contexts.length; second += 1) {
      if (!boundsOverlap(worldBounds[first], worldBounds[second])) continue;
      const key = pairKey(contexts[first].manifest.name, contexts[second].manifest.name);
      const robots = key.split("__");
      pairs.push({
        key,
        robots,
        ownerRobot: robots[0],
        discovery: "workspace_aabb_overlap"
      });
    }
  }
  return pairs;
}

function parsePositiveNumber(value, fallback, optionName) {
  if (value === undefined) {
    return fallback;
  }
  const number = Number(value);
  if (!Number.isFinite(number) || number <= 0) {
    throw new Error(`--${optionName} must be a positive number (got "${value}").`);
  }
  return number;
}

function inferPackageReplacement(urdfPath) {
  const urdfDir = path.dirname(urdfPath);
  const packageDir = path.dirname(urdfDir);
  return path.dirname(packageDir) + path.sep;
}

function resolveFromRobwot(moduleName, robwotRoot) {
  const resolved = require.resolve(moduleName, { paths: [robwotRoot] });
  return require(resolved);
}

function createContext(manifestFile) {
  const repoRoot = path.resolve(__dirname, "..", "..");
  const manifestPath = path.resolve(process.cwd(), manifestFile);
  if (!fs.existsSync(manifestPath)) {
    fail(`Manifest file does not exist: ${manifestPath}`);
  }

  const manifest = readJson(manifestPath);
  const robwotRoot = manifest.pipeline?.robwotRoot || manifest.robwotRoot || defaultRobwotRoot();
  const robotRoot = path.dirname(manifestPath);
  const modelDir = path.join(robotRoot, "model");
  const infoDir = path.join(robotRoot, "info");
  const tdDir = path.join(robotRoot, "td");
  const sceneDir = path.join(robotRoot, "scene");

  return {
    repoRoot,
    manifestPath,
    manifest,
    robwotRoot,
    robotRoot,
    modelDir,
    infoDir,
    tdDir,
    sceneDir,
    paths: {
      urdf: manifest.urdfPath,
      urdfLoadScene: manifest.pipeline?.urdfLoadScene || path.join(robwotRoot, "Coppeliasim scene", "robot_urdf_load_scene.ttt"),
      workspaceSceneTemplate: manifest.pipeline?.workspaceSceneTemplate || path.join(robwotRoot, "Coppeliasim scene", "robot_virtual_workspace.ttt"),
      workspaceDriver: manifest.pipeline?.workspaceDriver || path.join(robwotRoot, "Library", "GenerateRobotDescription", "robot_driver_workspace.lua"),
      robotTemplate: manifest.pipeline?.robotTemplate || path.join(robwotRoot, "Library", "GenerateRobotDescription", "robot_template.json"),
      generatedModel: path.join(modelDir, `${manifest.name}.ttm`),
      generatedWorkspaceScene: path.join(sceneDir, `${manifest.name}_workspace.ttt`),
      generatedLowerTd: path.join(tdDir, `${manifest.name}.lower.td.json`),
      robotInfo: path.join(infoDir, `${manifest.name}_info.json`),
      robotShape: path.join(infoDir, `${manifest.name}_shape.stl`),
      robotDataPoints: path.join(infoDir, `${manifest.name}_data_point.csv`)
    }
  };
}

function validateContext(ctx) {
  const checks = [
    ["URDF", ctx.paths.urdf],
    ["RobWoT root", ctx.robwotRoot],
    ["URDF load scene", ctx.paths.urdfLoadScene],
    ["Workspace scene template", ctx.paths.workspaceSceneTemplate],
    ["Workspace driver", ctx.paths.workspaceDriver],
    ["Robot TD template", ctx.paths.robotTemplate]
  ];

  const missing = checks.filter(([, file]) => !fs.existsSync(file));
  if (missing.length > 0) {
    missing.forEach(([label, file]) => console.error(`${label} not found: ${file}`));
    return false;
  }

  try {
    require.resolve("coppelia-ws-api", { paths: [ctx.robwotRoot] });
  } catch (error) {
    console.error(`coppelia-ws-api not resolvable from RobWoT root: ${ctx.robwotRoot}`);
    return false;
  }

  return true;
}

async function connect(ctx) {
  const { RemoteAPIClient } = resolveFromRobwot("coppelia-ws-api", ctx.robwotRoot);
  const client = new RemoteAPIClient("localhost", 23050, "json");
  try {
    await client.websocket.open();
    const sim = await client.getObject("sim");
    return { client, sim };
  } catch (error) {
    const message = [
      "Could not connect to CoppeliaSim websocket remote API at localhost:23050.",
      "Start CoppeliaSim, enable/load the websocket remote API, and then rerun this command.",
      `Original error: ${error.message || error}`
    ].join("\n");
    throw new Error(message);
  }
}

async function stopSimulationIfNeeded(sim) {
  const state = Number(await sim.getSimulationState());
  if (state !== Number(await sim.simulation_stopped)) {
    await sim.stopSimulation();
  }

  for (let attempt = 0; attempt < 50; attempt += 1) {
    const currentState = Number(await sim.getSimulationState());
    if (currentState === Number(await sim.simulation_stopped)) {
      return;
    }
    await delay(200);
  }

  throw new Error("CoppeliaSim simulation did not stop in time.");
}

async function getSandboxScriptHandle(sim) {
  const handle = await sim.getScript(await sim.scripttype_sandboxscript);
  return Array.isArray(handle) ? handle[0] : handle;
}

async function importUrdfThroughSandbox(sim, ctx) {
  const sandboxHandle = await getSandboxScriptHandle(sim);
  const replacement = ctx.manifest.pipeline?.packageReplacement || inferPackageReplacement(ctx.paths.urdf);
  const script = [
    "local simURDF = require('simURDF')",
    `return simURDF.import(${luaString(ctx.paths.urdf)}, 516, ${luaString(replacement)})`
  ].join("\n");
  const result = await sim.executeScriptString(script, sandboxHandle);
  const executionResult = Array.isArray(result) ? result[1] : result;
  console.log(`URDF import returned: ${executionResult}`);
  return executionResult;
}

async function configureImportedRobot(sim, robotName, lockedPositions, options = {}) {
  // Objects the caller has asked to be left exactly as they are. A URDF import
  // has none: every joint in the tree is the arm's, so aliasing all of them
  // "joint" and recomputing every link's mass is right. An ADOPTED model does:
  // its gripper is a working mechanism with named joints and a script that
  // reads them, and renaming those joints to "joint" would put seven more
  // objects into the arm chain and leave the gripper unable to find its own.
  const preserved = new Set((options.preserve || []).map(Number));
  // Positions in the arm chain, not names: this function is the reason names
  // cannot be used. Every joint is aliased "joint" a few lines below, so by the
  // time anything downstream looks at the model there is nothing to match on.
  const locked = new Map((lockedPositions || []).map((entry) => [entry.index, entry.position]));
  const declaredIntervals = options.jointIntervals || [];
  const intervals = new Map(declaredIntervals.map((entry) => [entry.index, entry]));
  const treeResult = await sim.getObjectsInTree(await sim.handle_scene, await sim.handle_all, 2);
  const treeObjects = Array.isArray(treeResult[0]) ? treeResult[0] : treeResult;
  const robotHandles = [];

  for (const curHandle of treeObjects) {
    const jointResult = await sim.getObjectsInTree(curHandle, await sim.object_joint_type, 0);
    const joints = Array.isArray(jointResult[0]) ? jointResult[0] : jointResult;
    if (!joints || joints.length === 0) {
      continue;
    }

    robotHandles.push(curHandle);
    // Chain positions are counted over the arm alone. A preserved joint is not
    // part of the arm, so counting it would shift every lock index past it.
    //
    // Which is exactly why the interval table is checked against the chain
    // BEFORE any of it is applied. A position-addressed limit is only as good
    // as the agreement between the manifest's chain and the scene's, and where
    // they disagree the damage is silent: Robot2's manifest lists five arm
    // joints for a six-joint UR3, so applying its table by position would put
    // joint_2's limits on joint_1 and leave the wrist unbounded, with every
    // joint still inside a plausible range and nothing to show for it. A
    // mismatch is a manifest to repair, not a table to apply partially.
    const armJointCount = joints.filter((joint) => !preserved.has(Number(joint))).length;
    const applyIntervals = intervals.size > 0 && intervals.size === armJointCount;
    const appliedIntervals = [];
    if (intervals.size > 0 && !applyIntervals) {
      console.warn(
        `${robotName}: manifest declares ${intervals.size} arm joint limit(s) but the scene chain has `
        + `${armJointCount} joint(s); leaving the model's own joint intervals alone. `
        + "Fix mobility.armJoints/armJointLimits to match the arm before relying on them."
      );
    }
    let chainIndex = 0;
    for (let index = 0; index < joints.length; index += 1) {
      const joint = joints[index];
      if (preserved.has(Number(joint))) {
        continue;
      }
      chainIndex += 1;
      // Baked into the saved model, not just into the driver that reads it: the
      // GUI will happily drag a dynamic joint the driver has filtered out of its
      // own list.
      //
      // Set up exactly like a driven joint, then parked on a target it is never
      // asked to leave. A fixed axis is a motor holding a position -- it is NOT
      // a joint taken out of the dynamic chain, which detaches every link below
      // it and drops the end link and its gripper on the floor at simulation
      // start. Only the alias marks it, so it is findable in the scene tree.
      await sim.setJointMode(joint, await sim.jointmode_dynamic, 0);
      await sim.setObjectInt32Param(joint, await sim.jointintparam_dynctrlmode, await sim.jointdynctrl_position);
      await sim.setObjectFloatParam(joint, await sim.jointfloatparam_maxvel, 50);
      await sim.setJointTargetForce(joint, 1000);
      if (locked.has(chainIndex)) {
        const held = Number(locked.get(chainIndex) || 0);
        await sim.setJointPosition(joint, held);
        await sim.setJointTargetPosition(joint, held);
        await sim.setObjectAlias(joint, "joint_locked");
        continue;
      }
      // After the lock branch on purpose: a locked axis is parked on a target
      // it never leaves, so its interval decides nothing, and narrowing it
      // around a held position is a way to make the two disagree for no gain.
      const interval = applyIntervals ? intervals.get(chainIndex) : null;
      if (interval) {
        await sim.setJointInterval(joint, false, [interval.low, interval.high - interval.low]);
        appliedIntervals.push(
          `${interval.name} -> [${interval.low.toFixed(4)}, ${interval.high.toFixed(4)}]`
        );
      }
      await sim.setObjectAlias(joint, "joint");
    }
    if (locked.size > 0) {
      console.log(`Locked ${locked.size} joint(s) in the model: chain position(s) ${[...locked.keys()].join(", ")} of ${joints.length}.`);
    }
    if (appliedIntervals.length > 0) {
      console.log(`Set ${appliedIntervals.length} joint interval(s) from the manifest: ${appliedIntervals.join(", ")}.`);
    }

    const linkResult = await sim.getObjectsInTree(curHandle, await sim.object_shape_type, 0);
    const allLinks = Array.isArray(linkResult[0]) ? linkResult[0] : linkResult;
    const links = allLinks.filter((link) => !preserved.has(Number(link)));
    for (let i = 0; i < links.length; i += 1) {
      if (i === 0) {
        await sim.setObjectInt32Param(links[i], await sim.shapeintparam_respondable, 1);
        await sim.setObjectInt32Param(links[i], await sim.shapeintparam_static, 1);
      } else if (i % 2 === 0) {
        await sim.setObjectInt32Param(links[i], await sim.shapeintparam_respondable, 1);
        await sim.setObjectInt32Param(links[i], await sim.shapeintparam_static, 0);
        await sim.computeMassAndInertia(links[i], 1200);
      } else {
        await sim.setObjectInt32Param(links[i], await sim.shapeintparam_respondable, 0);
      }
    }
  }

  if (robotHandles.length === 0) {
    throw new Error("No imported robot with joints was found in the CoppeliaSim scene.");
  }

  const rootHandle = robotHandles[0];
  let properties = await sim.getModelProperty(rootHandle);
  properties = Array.isArray(properties) ? properties[0] : properties;
  const modelProperty = await sim.modelproperty_not_model;
  properties = (await sim.boolOr32(properties, modelProperty)) - modelProperty;
  await sim.setModelProperty(rootHandle, properties);
  await sim.setObjectAlias(rootHandle, robotName);

  return rootHandle;
}

async function generateModel(ctx) {
  ensureDir(ctx.modelDir);
  const { sim } = await connect(ctx);

  console.log(`Loading URDF scene: ${ctx.paths.urdfLoadScene}`);
  await stopSimulationIfNeeded(sim);
  await sim.loadScene(ctx.paths.urdfLoadScene);

  console.log(`Importing URDF: ${ctx.paths.urdf}`);
  try {
    await importUrdfThroughSandbox(sim, ctx);
  } catch (error) {
    if (String(error).includes("simURDF") || String(error).includes("module")) {
      throw new Error([
        "CoppeliaSim could not load or execute simURDF.import from the sandbox script.",
        "Install/enable the CoppeliaSim URDF plugin, then restart CoppeliaSim and rerun this command.",
        "The bridge cannot create a model from URDF until that plugin function is available."
      ].join("\n"));
    }
    console.error("CoppeliaSim reported an error while importing the URDF. Continuing to inspect the scene.", error);
  }

  await delay(1000);
  const robotHandle = await configureImportedRobot(
    sim,
    ctx.manifest.name,
    lockedChainPositions(ctx.manifest.mobility),
    { jointIntervals: chainJointIntervals(ctx.manifest.mobility) }
  );
  await sim.saveModel(robotHandle, ctx.paths.generatedModel);
  console.log(`Saved generated model: ${ctx.paths.generatedModel}`);

  return ctx.paths.generatedModel;
}

// A robot that already exists, taken as it stands.
//
// The pipeline's normal first step imports a URDF: a description of links and
// joints, from which CoppeliaSim builds a tree and configureImportedRobot makes
// it a robot. That is the right step for an arm whose end effector is bolted on
// afterwards from a model file -- which is what --generate-model plus the
// gripper mount has always assumed.
//
// It is the wrong step for an arm whose end effector is a MECHANISM. The
// Dobot's rod gripper is a four-bar linkage closed by dummy link pairs, driven
// by two prismatic hand joints, with its own script reading a custom data
// block; none of that survives a URDF, where link_grip is a single rigid mesh.
// The working version exists only as a built scene, so this takes it from
// there: the arm, its gripper and the relationship between them, already
// assembled, and turns it into the model the rest of the pipeline consumes.
//
// What still has to be done to it is what the URDF path would have done later:
// the gripper is renamed to the alias the runtime looks for, the flange dummy
// is put where the gripper bolts on rather than out at the jaws, the source
// scene's own control scripts are removed so they do not run alongside the
// driver, and the arm's joints and masses are configured the way an imported
// robot's are.
async function adoptSceneModel(ctx) {
  const source = ctx.manifest.pipeline?.sourceScene;
  const sourceModel = ctx.manifest.pipeline?.sourceModel;
  if (!source || !sourceModel) {
    fail(
      `${ctx.manifest.name} has no model to adopt. Set pipeline.sourceScene and pipeline.sourceModel `
      + "in its manifest, or use --generate-model to import its URDF instead."
    );
  }
  const scenePath = path.isAbsolute(source) ? source : path.resolve(ctx.repoRoot, source);
  if (!fs.existsSync(scenePath)) {
    fail(`Source scene does not exist: ${scenePath}`);
  }
  const grip = ctx.manifest.runtime?.gripper || {};
  const gripperAlias = grip.modelName || `${ctx.manifest.name}_tcp_gripper`;
  const sourceGripper = ctx.manifest.pipeline?.sourceGripper;
  if (!sourceGripper) {
    fail(
      `${ctx.manifest.name} adopts a model but does not say which subtree is its gripper. `
      + "Set pipeline.sourceGripper to the gripper's alias in the source scene."
    );
  }

  ensureDir(ctx.modelDir);
  const { sim } = await connect(ctx);
  await stopSimulationIfNeeded(sim);
  console.log(`Loading source scene: ${scenePath}`);
  await sim.loadScene(scenePath);
  await delay(500);

  const sandboxHandle = await getSandboxScriptHandle(sim);
  const prepared = await executeInScript(
    sim,
    sandboxHandle,
    [
      `local robot=sim.getObject(${luaString(sourceModel)}, {noError=true})`,
      "if robot == -1 then return {ok=false, reason='source_model_not_found'} end",
      `local gripper=sim.getObject(${luaString(sourceModel + "/" + sourceGripper)}, {noError=true})`,
      "if gripper == -1 then return {ok=false, reason='source_gripper_not_found'} end",
      "",
      "-- The source scene drives this arm itself: an IK environment, a motion",
      "-- coroutine and a sysCall_actuation that resumes it. Left in place they",
      "-- run alongside the WoMAT driver on the same joints, and the arm is",
      "-- commanded by two controllers that do not know about each other. The",
      "-- gripper's own script is deliberately NOT removed here -- it is the",
      "-- mechanism, not a controller, and the runtime speaks to it.",
      "local gripperTree={}",
      "gripperTree[gripper]=true",
      "for _, h in ipairs(sim.getObjectsInTree(gripper, sim.handle_all, 0)) do gripperTree[h]=true end",
      "local stripped=0",
      "local arm=sim.getObjectsInTree(robot, sim.handle_all, 0)",
      "arm[#arm+1]=robot",
      "for i=1,#arm,1 do",
      "    if not gripperTree[arm[i]] then",
      "        for _, kind in ipairs({sim.scripttype_simulation, sim.scripttype_customization}) do",
      "            local ok, script=pcall(sim.getScript, kind, arm[i])",
      "            if ok and script ~= nil and script ~= -1 then",
      "                pcall(sim.removeScript, script)",
      "                stripped=stripped+1",
      "            end",
      "        end",
      "    end",
      "end",
      "",
      "-- The flange is where the gripper BOLTS ON, and in this scene that is",
      "-- exactly the gripper's own frame. The tip dummy the source ships with",
      "-- sits out at the jaws instead -- it was the tip of a full-pose IK, not a",
      "-- mount -- and womatResolveFlange would take it for the flange and mount",
      "-- the gripper on it, putting the jaws at two tool lengths. So the source",
      "-- dummies go and a new pair is made where the mount actually is.",
      "for _, alias in ipairs({'tip', 'target'}) do",
      "    for _, h in ipairs(sim.getObjectsInTree(robot, sim.object_dummy_type, 0)) do",
      "        if sim.getObjectAlias(h) == alias then pcall(sim.removeObject, h) end",
      "    end",
      "end",
      "local mount=sim.getObjectPose(gripper, sim.handle_world)",
      "local flange=sim.createDummy(0.01)",
      "sim.setObjectAlias(flange, 'tip')",
      "sim.setObjectParent(flange, sim.getObjectParent(gripper), true)",
      "sim.setObjectPose(flange, sim.handle_world, mount)",
      "-- The gripper hangs off the flange from here on, exactly as a mounted one",
      "-- does, so womatWorkspaceMountGripper's zero offset is a no-op and the",
      "-- assembly does not move.",
      "sim.setObjectParent(gripper, flange, true)",
      "local target=sim.createDummy(0.01)",
      "sim.setObjectAlias(target, 'target')",
      "sim.setObjectPose(target, sim.handle_world, mount)",
      "sim.setObjectParent(target, robot, true)",
      `sim.setObjectAlias(gripper, ${luaString(gripperAlias)})`,
      "",
      "-- Everything else in the source scene is somebody else's robot, table and",
      "-- cubes. Only the adopted model is saved, so the rest is removed rather",
      "-- than carried into a model file that is loaded into every cell.",
      "local kept={}",
      "kept[robot]=true",
      "for _, h in ipairs(sim.getObjectsInTree(robot, sim.handle_all, 0)) do kept[h]=true end",
      "local removed=0",
      "for _, h in ipairs(sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 2)) do",
      "    if not kept[h] then",
      "        local alias=sim.getObjectAlias(h)",
      "        if alias ~= 'Floor' and not string.find(alias, 'Camera') and not string.find(alias, 'Light') then",
      "            pcall(sim.removeModel, h)",
      "            removed=removed+1",
      "        end",
      "    end",
      "end",
      "",
      "local gripperHandles={}",
      "for h in pairs(gripperTree) do gripperHandles[#gripperHandles+1]=h end",
      "gripperHandles[#gripperHandles+1]=flange",
      "gripperHandles[#gripperHandles+1]=target",
      "return {ok=true, robot=robot, gripper=gripper, flange=flange,",
      "        preserve=gripperHandles, strippedScripts=stripped, removedModels=removed}"
    ].join("\n")
  );
  if (!prepared || prepared.ok === false) {
    throw new Error(`Could not adopt ${sourceModel} from ${scenePath}: ${prepared?.reason || "unknown"}`);
  }
  console.log(
    `Adopted ${sourceModel}: gripper ${sourceGripper} -> ${gripperAlias}, `
    + `${prepared.strippedScripts} source control script(s) removed, `
    + `${prepared.removedModels} other scene model(s) dropped.`
  );

  const robotHandle = await configureImportedRobot(
    sim,
    ctx.manifest.name,
    lockedChainPositions(ctx.manifest.mobility),
    {
      // The websocket JSON bridge hands a Lua array back as either a JS array or
      // an object keyed by index, depending on how sparse it was.
      preserve: Object.values(prepared.preserve || {}).map(Number).filter(Number.isFinite),
      // The source scene's own limits are whoever built it; the manifest's come
      // from the URDF that describes the same arm. This is the path that needs
      // them -- see chainJointIntervals.
      jointIntervals: chainJointIntervals(ctx.manifest.mobility)
    }
  );
  await sim.saveModel(robotHandle, ctx.paths.generatedModel);
  console.log(`Saved adopted model: ${ctx.paths.generatedModel}`);
  return ctx.paths.generatedModel;
}

async function loadWorkspaceDriver(sim, ctx, robotName) {
  const script = prepareWorkspaceDriverScript(ctx);
  const objectName = `/${robotName}`;
  const scriptHandle = Number(await sim.addScript(await sim.scripttype_customizationscript));
  const objectHandle = Number(await sim.getObject(objectName));
  const existing = await sim.getScript(await sim.scripttype_customizationscript, objectHandle, objectName);

  if (existing[0] !== -1) {
    await sim.removeScript(existing[0]);
  }

  await sim.setScriptStringParam(scriptHandle, Number(await sim.scriptstringparam_text), script);
  await sim.associateScriptWithObject(scriptHandle, objectHandle);
  await sim.initScript(scriptHandle);
  return scriptHandle;
}

// The sampled cloud has to describe the point the robot actually grips with,
// or every admission check downstream compares a jaw target against a flange
// cloud. So the gripper is attached before sampling and a tool-control-point
// dummy is added at the midpoint of its contact pads.
//
// './tip' is NOT moved. It is the flange -- the mount the gripper bolts to --
// in the model, in every saved scene, and in the runtime driver. Moving it both
// drags the gripper along and, because the scene is saved afterwards,
// permanently redefines the mount. The sampling is pointed at the new dummy
// instead.
function buildWorkspaceGripperPreamble({
  tcpAlias,
  gripperModelName,
  gripperModelPath,
  gripperModelPosition,
  gripperModelOrientation,
  gripperModelLocalRotation,
  graspPadAlias,
  applyBaseOrientationCorrection = false,
  squareToWristJoint = false,
  source = "model",
  sourceAlias = "",
  tcpOffset = null
}) {
  return [
    fs.readFileSync(path.join(__dirname, "lua/00-tool-frame-math.lua"), "utf8"),
    "womatWorkspaceTcpDummy=nil",
    "womatWorkspaceGripper=nil",
    "-- Rotation of the robot's base in the cell. Zero while the robot stands at",
    "-- the origin; set by the pipeline before it re-mounts the gripper, because",
    "-- the correction turns the tool and therefore moves the contact pads.",
    "womatWorkspaceBaseOrientation={0,0,0}",
    "",
    "function womatWorkspacePlainAlias(handle)",
    "    local alias=sim.getObjectAlias(handle, 4)",
    "    if alias == nil then return '' end",
    "    return (string.gsub(alias, '__%d+__$', ''))",
    "end",
    "",
    "function womatWorkspaceFindByAlias(alias)",
    "    -- Tree search first, '/alias' only as a fallback. The path form is a",
    "    -- root-level lookup, and everything searched for here -- the gripper,",
    "    -- the tool-control-point dummy -- hangs deep off the flange once it has",
    "    -- been mounted. A miss does not fail loudly: it silently loads or",
    "    -- creates a SECOND one, and the scene then holds two objects with the",
    "    -- same alias, of which the wrong one is the one that gets measured.",
    "    local candidates=sim.getObjectsInTree(simBase, sim.handle_all, 0)",
    "    for i=1,#candidates,1 do",
    "        if womatWorkspacePlainAlias(candidates[i]) == alias then",
    "            return candidates[i]",
    "        end",
    "    end",
    "    return sim.getObject('/'..alias, {noError=true})",
    "end",
    "",
    "function womatWorkspaceWristJoint(flange)",
    "    -- The axis the wrist rolls about, found up the flange's own parent",
    "    -- chain so nothing in the scene has to be moved to get it.",
    "    if flange == nil or flange == -1 then return -1 end",
    "    local node=sim.getObjectParent(flange)",
    "    local hops=0",
    "    while node ~= -1 and hops < 12 do",
    "        if sim.getObjectType(node) == sim.object_joint_type then return node end",
    "        node=sim.getObjectParent(node)",
    "        hops=hops+1",
    "    end",
    "    return -1",
    "end",
    "",
    "function womatWorkspaceFlange()",
    "    -- Found by searching the tree, not by path. './tip' is a direct-child",
    "    -- lookup and this dummy hangs off the last LINK, so the path form can",
    "    -- return -1 -- and a workspace sampled from the wrong point looks",
    "    -- exactly like a good one and is wrong by a whole tool length.",
    "    local candidates=sim.getObjectsInTree(simBase, sim.object_dummy_type, 0)",
    "    for i=1,#candidates,1 do",
    "        if womatWorkspacePlainAlias(candidates[i]) == 'tip' then",
    "            return candidates[i]",
    "        end",
    "    end",
    "    return sim.getObject('./tip', {noError=true})",
    "end",
    "",
    "function womatWorkspaceToolControlPoint()",
    "    -- No fallback to the flange on purpose: sampling the wrong point is a",
    "    -- worse outcome than failing to sample at all.",
    "    if womatWorkspaceTcpDummy == nil or womatWorkspaceTcpDummy == -1 then",
    `        error('workspace_tool_control_point_missing: ${tcpAlias}')`,
    "    end",
    "    return womatWorkspaceTcpDummy",
    "end",
    "",
    "-- Mirrors womatIsDecorativeShape in 04-safety, which the runtime's own pad",
    "-- scan applies. The two have to agree: graspPadAlias is a SUBSTRING match, so",
    "-- a gripper whose pads are 'leftPad'/'rightPad' also matches the render-only",
    "-- 'leftPad_visible'/'rightPad_visible' hanging off them. Folding those into",
    "-- the bounding box moves the midpoint -- and this midpoint is the point the",
    "-- whole workspace is sampled at, while the runtime steers the other one.",
    "function womatWorkspaceIsDecorative(alias)",
    "    return string.find(alias, 'visual', 1, true) ~= nil",
    "        or string.find(alias, 'visible', 1, true) ~= nil",
    "end",
    "",
    "function womatWorkspacePadMidpoint(gripper)",
    "    local low={math.huge, math.huge, math.huge}",
    "    local high={-math.huge, -math.huge, -math.huge}",
    "    local found=false",
    "    for _, shape in ipairs(sim.getObjectsInTree(gripper, sim.object_shape_type, 0)) do",
    "        local alias=string.lower(womatWorkspacePlainAlias(shape))",
    `        if string.find(alias, ${luaString(graspPadAlias)}, 1, true) ~= nil`,
    "            and not womatWorkspaceIsDecorative(alias) then",
    "            found=true",
    "            local matrix=sim.getObjectMatrix(shape, sim.handle_world)",
    "            local e={",
    "                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_x), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_x)},",
    "                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_y), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_y)},",
    "                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_z), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_z)}",
    "            }",
    "            for a=1,2,1 do",
    "                for b=1,2,1 do",
    "                    for c=1,2,1 do",
    "                        local w=sim.multiplyVector(matrix, {e[1][a], e[2][b], e[3][c]})",
    "                        for k=1,3,1 do",
    "                            if w[k] < low[k] then low[k]=w[k] end",
    "                            if w[k] > high[k] then high[k]=w[k] end",
    "                        end",
    "                    end",
    "                end",
    "            end",
    "        end",
    "    end",
    "    if not found then return nil end",
    "    return {(low[1]+high[1])/2, (low[2]+high[2])/2, (low[3]+high[3])/2}",
    "end",
    "",
    ...(source === "builtin" ? [
      "-- Adopts the tool the arm's own model already carries, and returns it.",
      "-- Nothing is loaded and, above all, NOTHING IS REPARENTED: this subtree is",
      "-- a link of the kinematic chain, and bolting it onto the flange dummy the",
      "-- way a loaded gripper is bolted takes it out of that chain -- so the arm",
      "-- would be sampled with its own end effector hanging off a dummy.",
      "function womatWorkspaceMountGripper()",
      "    local flange=womatWorkspaceFlange()",
      "    if flange == nil or flange == -1 then",
      "        error('workspace_flange_dummy_not_found')",
      "    end",
      `    local gripper=womatWorkspaceFindByAlias(${luaString(gripperModelName)})`,
      "    if gripper == -1 then",
      `        gripper=womatWorkspaceFindByAlias(${luaString(sourceAlias)})`,
      "        if gripper == -1 then",
      `            error('workspace_builtin_gripper_not_found: ${sourceAlias}')`,
      "        end",
      "        -- Renamed, not moved: from here on the rest of the pipeline finds",
      "        -- it by the same alias a loaded gripper would carry.",
      `        sim.setObjectAlias(gripper, ${luaString(gripperModelName)})`,
      "    end",
      "    womatWorkspaceGripper=gripper",
      "    return gripper, flange",
      "end"
    ] : [
      "-- Bolts the gripper to the FLANGE and returns it. The single place in the",
      "-- pipeline that decides where the gripper hangs: mounting it anywhere else",
      "-- -- on simTip in particular, which has already become the tool control",
      "-- point by the time anything else runs -- puts the jaws at two tool lengths.",
      "function womatWorkspaceMountGripper()",
      "    local flange=womatWorkspaceFlange()",
      "    if flange == nil or flange == -1 then",
      "        error('workspace_flange_dummy_not_found')",
      "    end",
      `    local gripper=womatWorkspaceFindByAlias(${luaString(gripperModelName)})`,
      "    if gripper == -1 then",
      `        local okLoad, loaded=pcall(sim.loadModel, ${luaString(gripperModelPath)})`,
      "        if not okLoad or loaded == nil or loaded == -1 then",
      `            error('workspace_gripper_model_load_failed: ${gripperModelPath}')`,
      "        end",
      `        sim.setObjectAlias(loaded, ${luaString(gripperModelName)})`,
      "        gripper=loaded",
      "    end",
      "    -- Bolted to the flange, exactly as the runtime bolts it, so the arm and",
      "    -- the gripper are one body for the whole of sampling: the collision",
      "    -- collection is built from this tree, and the point that is sampled is",
      "    -- measured through this mount.",
      "    sim.setObjectParent(gripper, flange, false)",
      `    sim.setObjectPosition(gripper, flange, ${luaArray(gripperModelPosition)})`,
      "    -- Orientation against the WRIST JOINT when the manifest asks for it, and",
      "    -- for the same reason the runtime does: the 'tip' dummy is a mount POINT",
      "    -- whose orientation is whatever the last link's mesh origin left it.",
      "    -- Sampling measures the workspace THROUGH this mount, so a tool bolted",
      "    -- on sideways here is a whole workspace sampled at the wrong point --",
      "    -- which is what put Robot1's control point 0.18 m off its flange axis.",
      "    -- Position still comes off the flange; that part of the dummy is right.",
      `      local squareTo=${squareToWristJoint ? "true" : "false"} and womatWorkspaceWristJoint(flange) or -1`,
      "    if squareTo ~= -1 then",
      `        sim.setObjectOrientation(gripper, squareTo, ${luaArray(gripperModelOrientation)})`,
      "    else",
      `        sim.setObjectOrientation(gripper, flange, ${luaArray(gripperModelOrientation)})`,
      "    end",
      `    local localRotation=${luaArray(gripperModelLocalRotation)}`,
      "    if math.abs(localRotation[1])+math.abs(localRotation[2])+math.abs(localRotation[3]) > 0.0000001 then",
      "        local currentLocal=sim.getObjectMatrix(gripper, flange)",
      "        local extraLocal=sim.buildMatrix({0,0,0},localRotation)",
      "        sim.setObjectMatrix(gripper,flange,sim.multiplyMatrices(currentLocal,extraLocal))",
      "    end",
      `    if ${applyBaseOrientationCorrection ? "true" : "false"} and math.abs(womatWorkspaceBaseOrientation[1])+math.abs(womatWorkspaceBaseOrientation[2])+math.abs(womatWorkspaceBaseOrientation[3]) > 0.0000001 then`,
      "        local current=sim.getObjectMatrix(gripper, sim.handle_world)",
      "        local worldPosition={current[4],current[8],current[12]}",
      "        local rotation=sim.buildMatrix({0,0,0},womatWorkspaceBaseOrientation)",
      "        local rotated=sim.multiplyMatrices(rotation,current)",
      "        rotated[4]=worldPosition[1]",
      "        rotated[8]=worldPosition[2]",
      "        rotated[12]=worldPosition[3]",
      "        sim.setObjectMatrix(gripper,sim.handle_world,rotated)",
      "    end",
      "    womatWorkspaceGripper=gripper",
      "    return gripper, flange",
      "end"
    ]),
    "",
    "-- Re-measures the sampled point from the gripper's CURRENT pose. Separate",
    "-- from the mount on purpose: the base-orientation correction and the jaw",
    "-- opening both move the pads, and both happen after the first mount.",
    "function womatWorkspaceRetargetTip()",
    "    local flange=womatWorkspaceFlange()",
    "    if flange == nil or flange == -1 then",
    "        error('workspace_flange_dummy_not_found')",
    "    end",
    `    local gripper=womatWorkspaceGripper or womatWorkspaceFindByAlias(${luaString(gripperModelName)})`,
    "    if gripper == nil or gripper == -1 then",
    `        error('workspace_gripper_model_load_failed: ${gripperModelPath}')`,
    "    end",
    "    local pads=womatWorkspacePadMidpoint(gripper)",
    ...(tcpOffset
      ? [
        "    if pads == nil then",
        "        -- A tool with no contact pads to measure -- a fixed tool, a",
        "        -- suction cup, a gripper that is only a housing mesh. Its",
        "        -- business end is declared instead, in the TOOL's own frame, so",
        "        -- it still follows the wrist. Measuring is preferred wherever",
        "        -- there is anything to measure: pads move when the jaws open and",
        "        -- a declared offset does not, so this runs only on an empty scan.",
        `        pads=sim.multiplyVector(sim.getObjectMatrix(gripper, sim.handle_world), ${luaArray(tcpOffset)})`,
        "    end"
      ]
      : [
        "    if pads == nil then",
        `        error('workspace_gripper_contact_pads_not_found: ${graspPadAlias}')`,
        "    end"
      ]),
    "    local dummy=womatWorkspaceTcpDummy",
    "    if dummy == nil or dummy == -1 or not sim.isHandle(dummy) then",
    `        dummy=womatWorkspaceFindByAlias(${luaString(tcpAlias)})`,
    "    end",
    "    if dummy == nil or dummy == -1 then",
    "        dummy=sim.createDummy(0.02)",
    `        sim.setObjectAlias(dummy, ${luaString(tcpAlias)})`,
    "    end",
    "    -- Parented to the flange, never the other way round: the gripper is",
    "    -- measured THROUGH its mount, so a gripper hanging off its own tool",
    "    -- control point reports jaws at twice the tool length.",
    "    sim.setObjectParent(dummy, flange, true)",
    "    sim.setObjectPosition(dummy, sim.handle_world, pads)",
    "    local tcp=sim.getObjectPosition(dummy, flange)",
    "    local frame=womatToolFrameMatrix(tcp)",
    "    if frame == nil then error('workspace_tool_frame_degenerate') end",
    "    sim.setObjectMatrix(dummy, flange, frame)",
    "    womatWorkspaceTcpDummy=dummy",
    "    local flangePosition=sim.getObjectPosition(flange, sim.handle_world)",
    "    sim.addLog(sim.verbosity_scriptinfos, string.format(",
    "        'WoMAT: sampling tool control point %.4f m out from the flange',",
    "        math.sqrt((pads[1]-flangePosition[1])^2+(pads[2]-flangePosition[2])^2+(pads[3]-flangePosition[3])^2)))",
    "    return dummy",
    "end",
    "",
    "function womatWorkspaceAttachGripperAndRetargetTip()",
    "    womatWorkspaceMountGripper()",
    "    womatWorkspaceRetargetTip()",
    "    return true",
    "end",
    ""
  ].join("\n");
}

function replaceExactlyOnce(source, anchor, replacement, label) {
  const first = source.indexOf(anchor);
  if (first === -1 || source.indexOf(anchor, first + anchor.length) !== -1) {
    throw new Error(`Workspace driver: the anchor for ${label} must occur exactly once; the vendor driver has changed.`);
  }
  return source.slice(0, first) + replacement + source.slice(first + anchor.length);
}

// Two things the vendor sampler cannot do on its own, both needed to repeat an
// experiment rather than merely rerun it.
//
// A seed: math.random is never seeded, so two runs of the same cell sample two
// different workspaces. With one, a run is reproducible and a set of seeds is a
// set of independent repetitions.
//
// A progress record: the CSV holds one row per KEPT sample -- colliding samples
// below the floor are dropped -- so its first N rows are not its first N
// samples. The sidecar says how many rows the first N samples produced and how
// long they took, which makes every prefix of one long run an honest shorter
// run. Anchored with replaceExactlyOnce so a changed vendor file fails loudly
// instead of silently sampling unseeded.
function instrumentSamplingLoop(source, { seed, progressEvery = 0 } = {}) {
  let result = source;
  if (seed !== undefined && seed !== null) {
    const value = Number(seed);
    if (!Number.isInteger(value)) {
      throw new Error(`pipeline.workspaceSeed must be an integer, got ${JSON.stringify(seed)}.`);
    }
    result = replaceExactlyOnce(
      result,
      "    if generateRandom then\n",
      `    if generateRandom then\n        math.randomseed(${value})\n`,
      "the sampling seed"
    );
  }
  if (progressEvery > 0) {
    if (!Number.isInteger(progressEvery)) {
      throw new Error("pipeline.workspaceProgressEvery must be a positive integer.");
    }
    result = replaceExactlyOnce(
      result,
      "        for cnt=1,randomSteps,1 do\n",
      "        for cnt=1,randomSteps,1 do\n            womatSamplingCheckpoint(cnt-1, (#points+#collisionpoints)//3, false)\n",
      "the sampling progress checkpoint"
    );
    result = replaceExactlyOnce(
      result,
      "-- only in generate random try to save data points in csv file\n",
      "-- only in generate random try to save data points in csv file\n        womatSamplingCheckpoint(randomSteps, (#points+#collisionpoints)//3, true)\n",
      "the final sampling progress checkpoint"
    );
    result += `

womatSamplingProgressEvery=${progressEvery}
womatSamplingProgressFile=nil
womatSamplingStartTime=nil

function womatNow()
    if sim.getSystemTime ~= nil then
        return sim.getSystemTime()
    end
    return os.clock()
end

function womatSamplingCheckpoint(samples, rows, final)
    if womatSamplingProgressFile == nil then
        local name=sim.getObjectAlias(simBase)
        womatSamplingProgressFile=io.open(rootPath.."/"..name.."_sampling_progress.csv", "w+")
        womatSamplingProgressFile:write("Samples,Rows,Seconds\\n")
        womatSamplingStartTime=womatNow()
    end
    if final or samples % womatSamplingProgressEvery == 0 then
        womatSamplingProgressFile:write(string.format("%d,%d,%.4f\\n", samples, rows, womatNow()-womatSamplingStartTime))
    end
    if final then
        womatSamplingProgressFile:close()
        womatSamplingProgressFile=nil
    end
end
`;
  }
  return result;
}

function prepareWorkspaceDriverScript(ctx) {
  const randomSteps = Number(ctx.manifest.pipeline?.workspaceRandomSteps || 8000);
  const gripperCollisionClearance = Number(
    ctx.manifest.pipeline?.gripperCollisionClearance
      ?? ctx.manifest.runtime?.gripperCollisionClearance
      ?? 0.005
  );
  const obstacleNames = ctx.manifest.pipeline?.collisionObstacles || ctx.manifest.runtime?.collisionObstacles || ["80cmHighWall50cm"];
  const obstacleLua = `{${obstacleNames.map(luaString).join(",")}}`;
  const obstacleCulling = ctx.manifest.pipeline?.samplingObstacleCulling !== false;
  const obstacleCullMargin = Number(ctx.manifest.pipeline?.samplingCullMargin ?? 0.25);
  if (!(obstacleCullMargin >= 0)) {
    throw new Error(`${ctx.manifest.name}: pipeline.samplingCullMargin must be a non-negative number of metres.`);
  }
  const grip = ctx.manifest.runtime?.gripper || {};
  const cubeName = grip.objectName || `${ctx.manifest.name}_grip_cube`;
  const gripperModelName = grip.modelName || `${ctx.manifest.name}_tcp_gripper`;
  const gripperModelPath = grip.modelPath || "";
  const gripperModelPosition = grip.modelPosition || [0, 0, 0];
  const gripperModelOrientation = radiansTriple(
    grip.modelOrientation || [-1.57079632679, 0, 0],
    "runtime.gripper.modelOrientation"
  );
  const gripperModelLocalRotation = radiansTriple(
    grip.modelLocalRotation || [0, 0, 0],
    "runtime.gripper.modelLocalRotation"
  );
  const graspPadAlias = String(grip.graspPadAlias ?? "touch").toLowerCase();
  const source = gripperSource(grip);
  const sourceAlias = String(grip.sourceAlias ?? "");
  if (source === "builtin" && sourceAlias === "") {
    throw new Error(
      `${ctx.manifest.name} declares runtime.gripper.source "builtin" but no sourceAlias. `
      + "Set it to the alias of the end-effector subtree inside the robot's own model."
    );
  }
  const tcpOffset = toolControlPointOffset(grip);

  // The sampled cloud has to describe the point the robot actually grips with,
  // or every admission check downstream compares a jaw target against a flange
  // cloud. So the gripper is attached before sampling and a tool-control-point
  // dummy is added at the midpoint of its contact pads.
  //
  // './tip' is NOT moved. It is the flange -- the mount the gripper bolts to --
  // in the model, in every saved scene, and in the runtime driver. Moving it
  // both drags the gripper along and, because the scene is saved afterwards,
  // permanently redefines the mount, so the runtime then bolts the gripper one
  // tool length out and the jaws land at two. The sampling is pointed at the
  // new dummy instead.
  const tcpAlias = `${ctx.manifest.name}_tcp`;
  const gripperAwareTipPreamble = buildWorkspaceGripperPreamble({
    tcpAlias,
    gripperModelName,
    gripperModelPath,
    gripperModelPosition,
    gripperModelOrientation,
    gripperModelLocalRotation,
    graspPadAlias,
    // Off unless a manifest asks for it. This re-rotates an already-mounted
    // gripper in the WORLD by the robot's base orientation, but the gripper
    // hangs off the flange and so has already inherited that rotation through
    // the hierarchy -- so on any arm that is not standing square at the origin
    // it double-rotates the tool by exactly the base heading. It read as a
    // no-op for as long as every arm stood unrotated; docs/fitting_a_different
    // _gripper.md called the failure before it happened. Defaulting it on made
    // every placed arm wrong by default.
    applyBaseOrientationCorrection: grip.applyBaseOrientationCorrection === true,
    // Mount the tool against the axis the wrist rolls about, not against the
    // 'tip' dummy's inherited orientation. The runtime reads the same flag, so
    // the arm is sampled through the mount the cell will actually run with.
    squareToWristJoint: grip.squareToWristJoint === true,
    source,
    sourceAlias,
    tcpOffset
  });

  const vendorDriver = instrumentSamplingLoop(fs.readFileSync(ctx.paths.workspaceDriver, "utf8"), {
    seed: ctx.manifest.pipeline?.workspaceSeed,
    progressEvery: ctx.manifest.pipeline?.workspaceProgressLog === true
      ? Number(ctx.manifest.pipeline?.workspaceProgressEvery || 1000)
      : 0
  });
  return gripperAwareTipPreamble + selectArmChainJoints(vendorDriver, lockedChainPositions(ctx.manifest.mobility))
    .replace("checkCollision=true", "checkCollision=true")
    .replace("randomSteps=80000", `randomSteps=${randomSteps}`)
    // After the joint list is fixed (so gripper joints are never taken for arm
    // joints) and before the IK environment is built against './tip'.
    // After the joint list is fixed (so gripper joints are never taken for arm
    // joints) and before the IK environment is built.
    .replace(
      "    jointHandles = newjointHandles",
      "    jointHandles = newjointHandles\n    womatWorkspaceAttachGripperAndRetargetTip()"
    )
    // The two places the vendor driver resolves the point it solves for and
    // samples. Redirecting these -- rather than moving './tip' -- is what keeps
    // the flange meaning the flange everywhere else.
    .replace("    simTip=sim.getObject('./tip')", "    simTip=womatWorkspaceToolControlPoint()")
    .replace("    local tip=sim.getObject('./tip')", "    local tip=womatWorkspaceToolControlPoint()")
    .replace(/rootPath\.\."\\\\\"\.\./g, 'rootPath.."/"..')
    .replace(
      [
        "    local robotCollection=sim.createCollection(0)",
        "    sim.addItemToCollection(robotCollection,sim.handle_tree,base,0)"
      ].join("\n"),
      "    local robotCollection=womatCreateWorkspaceRobotCollisionCollection(base)"
    )
    .replace(
      "local collisionpoints={}",
      [
        "local collisionpoints={}",
        "    womatCollisionPointsByObstacle={}",
        "    womatCollisionObstacleOrder={}",
        "    womatLastCollisionObstacle=nil"
      ].join("\n")
    )
    .split("if sim.checkCollision(robotCollection,sim.handle_all)~=0 then")
    .join([
      "local collisionAlias=womatRobotObstacleCollisionAlias(robotCollection)",
      "                                            if collisionAlias ~= nil then",
      "                                                womatLastCollisionObstacle=collisionAlias"
    ].join("\n"))
    .split("collisionpoints[#collisionpoints+1]=matrix[12]")
    .join("collisionpoints[#collisionpoints+1]=matrix[12]\n                womatAddCollisionPoint(womatLastCollisionObstacle, matrix[4], matrix[8], matrix[12])")
    .replace(
      [
        "        local collPtcld=sim.createPointCloud(0.05,1,0,2)",
        "        if #collisionpoints>0 then",
        "            sim.insertPointsIntoPointCloud(collPtcld,0,collisionpoints,{255,0,0})",
        "        end"
      ].join("\n"),
      "        womatCreateCollisionPointClouds(collisionpoints)"
    )
    .replace(
      [
        "    if usePointCloud then",
        "        local ptcld=sim.createPointCloud(0.05,1,0,2)",
        "        sim.insertPointsIntoPointCloud(ptcld,0,points)",
        "        womatCreateCollisionPointClouds(collisionpoints)",
        "    end"
      ].join("\n"),
      [
        "    if usePointCloud then",
        "        local ptcld=sim.createPointCloud(0.05,1,0,2)",
        "        pcall(sim.setObjectAlias, ptcld, \"workspace_reachable_points\")",
        "        sim.insertPointsIntoPointCloud(ptcld,0,points)",
        "        womatCreateCollisionPointClouds(collisionpoints)",
        "    end"
      ].join("\n")
    )
    .replace(
      "sim.setObjectParent(shape,simBase,true)",
      [
        "sim.setObjectParent(shape,simBase,true)",
        "        pcall(sim.setObjectAlias, shape, \"workspace_hull\")"
      ].join("\n")
    ) + `

function womatPointObstacleCollision(x, y, z)
    return womatPointObstacleCollisionAlias(x, y, z) ~= nil
end

function womatPointObstacleCollisionAlias(x, y, z)
    local obstacleShapes=womatSceneObstacleShapes()
    for i=1,#obstacleShapes,1 do
        local obstacle=obstacleShapes[i]
        local minX=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_x)
        local maxX=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_x)
        local minY=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_y)
        local maxY=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_y)
        local minZ=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_z)
        local maxZ=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_z)
        local m=sim.getObjectMatrix(obstacle, sim.handle_world)
        local inv=sim.getMatrixInverse(m)
        local p=sim.multiplyVector(inv, {x, y, z})
        if p[1] >= minX and p[1] <= maxX and p[2] >= minY and p[2] <= maxY and p[3] >= minZ and p[3] <= maxZ then
            return sim.getObjectAlias(obstacle, 4)
        end
    end
    return nil
end

function womatRobotObstacleCollisionAlias(robotCollection)
    -- The full robot is checked for contact. The gripper gets an additional
    -- proximity envelope so thin fingers cannot scrape an obstacle between
    -- sampled configurations without producing a collision contact.
    local obstacleCache=womatBuildObstacleCollisionCache()
    for i=1,#obstacleCache,1 do
        local obstacle=obstacleCache[i]
        if sim.checkCollision(robotCollection, obstacle.collection)~=0 then
            return obstacle.alias
        end
        if womatWorkspaceGripperCollection ~= nil and ${gripperCollisionClearance} > 0 then
            local ok, distance=pcall(
                sim.checkDistance,
                womatWorkspaceGripperCollection,
                obstacle.collection,
                ${gripperCollisionClearance}
            )
            if ok and distance ~= nil and distance > 0 then
                return obstacle.alias
            end
        end
    end
    return nil
end

womatWorkspaceGripperCollection=nil

function womatCreateWorkspaceRobotCollisionCollection(base)
    if womatWorkspaceGripperCollection ~= nil then
        pcall(sim.destroyCollection, womatWorkspaceGripperCollection)
    end
    local robotCollection=sim.createCollection(0)
    womatWorkspaceGripperCollection=sim.createCollection(0)
    local gripper=womatObjectByAlias(${luaString(gripperModelName)})
    if gripper == -1 then
        error("workspace_gripper_missing: ${gripperModelName}")
    end

    local gripperShapes=sim.getObjectsInTree(gripper, sim.object_shape_type, 0)
    if #gripperShapes == 0 then
        error("workspace_gripper_has_no_collision_shapes: ${gripperModelName}")
    end
    for i=1,#gripperShapes,1 do
        sim.addItemToCollection(womatWorkspaceGripperCollection, sim.handle_single, gripperShapes[i], 0)
    end

    -- Enumerate concrete shapes only after installing the gripper. This
    -- guarantees the RG2 body and fingers are part of workspace sampling.
    local robotShapes=sim.getObjectsInTree(base, sim.object_shape_type, 0)
    local included={}
    for i=1,#robotShapes,1 do
        sim.addItemToCollection(robotCollection, sim.handle_single, robotShapes[i], 0)
        included[robotShapes[i]]=true
    end
    for i=1,#gripperShapes,1 do
        if included[gripperShapes[i]] ~= true then
            sim.addItemToCollection(robotCollection, sim.handle_single, gripperShapes[i], 0)
        end
    end
    return robotCollection
end

function womatObjectByAlias(alias)
    if alias == nil or alias == "" then
        return -1
    end
    return sim.getObject('/'..alias, {noError=true})
end

function womatIsDescendantOf(object, root)
    if object == -1 or root == -1 then
        return false
    end
    local current=object
    while current ~= -1 do
        if current == root then
            return true
        end
        current=sim.getObjectParent(current)
    end
    return false
end

function womatIsSceneObstacleShape(shape)
    local pickObject=womatObjectByAlias(${luaString(cubeName)})
    local tcpTool=womatObjectByAlias(${luaString(gripperModelName)})
    if womatIsDescendantOf(shape, simBase) then
        return false
    end
    if womatIsDescendantOf(shape, pickObject) then
        return false
    end
    if womatIsDescendantOf(shape, tcpTool) then
        return false
    end
    return true
end

function womatSceneObstacleShapes()
    local obstacleNames=${obstacleLua}
    local obstacles={}
    for i=1,#obstacleNames,1 do
        local obstacle=womatObjectByAlias(obstacleNames[i])
        if obstacle ~= -1 and not womatIsDescendantOf(obstacle, womatObjectByAlias(${luaString(cubeName)})) then
            local shapes=sim.getObjectsInTree(obstacle, sim.object_shape_type, 0)
            for j=1,#shapes,1 do
                if womatIsSceneObstacleShape(shapes[j]) then
                    obstacles[#obstacles+1]=shapes[j]
                end
            end
        end
    end
    return obstacles
end

womatObstacleCollisionCache=nil
womatObstacleCulling=${obstacleCulling ? "true" : "false"}
womatObstacleCullMargin=${luaNumber(obstacleCullMargin)}
womatObstacleCullReport=nil

function womatDistance(a, b)
    return math.sqrt((a[1]-b[1])^2+(a[2]-b[2])^2+(a[3]-b[3])^2)
end

-- An upper bound on how far any part of the arm can get from its base. The
-- distance between two consecutive joint origins of a serial chain is the same
-- in every configuration, so their sum -- base, every joint, tool control
-- point -- bounds the reach whatever pose the arm is in. The margin covers what
-- the chain of origins does not: link thickness and the fingers past the TCP.
function womatRobotReachBound()
    local base=sim.getObject('.')
    local basePosition=sim.getObjectPosition(base, sim.handle_world)
    local previous=basePosition
    local length=0
    for i=1,#jointHandles,1 do
        local position=sim.getObjectPosition(jointHandles[i], sim.handle_world)
        length=length+womatDistance(previous, position)
        previous=position
    end
    local tcp=womatWorkspaceToolControlPoint()
    length=length+womatDistance(previous, sim.getObjectPosition(tcp, sim.handle_world))
    return basePosition, length
end

-- World-axis box around every shape of one obstacle, from the shapes' mesh
-- vertices. Not objfloatparam_objbbox_*: those are in the shape's bounding-box
-- frame, which for an imported mesh is turned against the frame
-- getObjectMatrix places, so a box built from them need not contain the
-- shape. Returns nil when a shape has no readable mesh; the caller then keeps
-- the obstacle rather than cull something it cannot bound.
function womatObstacleWorldBox(shapes)
    local low={math.huge, math.huge, math.huge}
    local high={-math.huge, -math.huge, -math.huge}
    for i=1,#shapes,1 do
        local shape=shapes[i]
        local ok, vertices=pcall(sim.getShapeMesh, shape)
        if not ok or type(vertices) ~= "table" or #vertices < 3 then
            return nil, nil
        end
        local m=sim.getObjectMatrix(shape, sim.handle_world)
        for j=1,#vertices,3 do
            local p=sim.multiplyVector(m, {vertices[j], vertices[j+1], vertices[j+2]})
            for axis=1,3,1 do
                if p[axis] < low[axis] then low[axis]=p[axis] end
                if p[axis] > high[axis] then high[axis]=p[axis] end
            end
        end
    end
    return low, high
end

function womatPointToBoxDistance(point, low, high)
    local sum=0
    for axis=1,3,1 do
        local gap=math.max(low[axis]-point[axis], 0, point[axis]-high[axis])
        sum=sum+gap*gap
    end
    return math.sqrt(sum)
end

function womatBuildObstacleCollisionCache()
    if womatObstacleCollisionCache ~= nil then
        return womatObstacleCollisionCache
    end

    womatObstacleCollisionCache={}
    womatObstacleCullReport={enabled=womatObstacleCulling, margin=womatObstacleCullMargin, kept={}, culled={}}
    -- Sampling checks every sample against every listed obstacle, and a check
    -- costs the same whether the obstacle is beside the arm or across the
    -- cell. An obstacle wholly outside the reach bound can never be touched,
    -- so leaving it out changes no sample's verdict -- only the time taken.
    local basePosition=nil
    local cullRadius=nil
    if womatObstacleCulling then
        local ok, position, reach=pcall(womatRobotReachBound)
        if ok then
            basePosition=position
            cullRadius=reach+womatObstacleCullMargin
            womatObstacleCullReport.reach=reach
            womatObstacleCullReport.radius=cullRadius
        else
            womatObstacleCullReport.error=tostring(position)
        end
    end
    local obstacleNames=${obstacleLua}
    local pickObject=womatObjectByAlias(${luaString(cubeName)})
    for i=1,#obstacleNames,1 do
        local obstacle=womatObjectByAlias(obstacleNames[i])
        if obstacle ~= -1 and not womatIsDescendantOf(obstacle, pickObject) then
            local collection=-1
            local shapes=sim.getObjectsInTree(obstacle, sim.object_shape_type, 0)
            local obstacleShapes={}
            for j=1,#shapes,1 do
                if womatIsSceneObstacleShape(shapes[j]) then
                    obstacleShapes[#obstacleShapes+1]=shapes[j]
                end
            end
            local distance=nil
            if cullRadius ~= nil and #obstacleShapes > 0 then
                local low, high=womatObstacleWorldBox(obstacleShapes)
                if low ~= nil then
                    distance=womatPointToBoxDistance(basePosition, low, high)
                end
            end
            local alias=sim.getObjectAlias(obstacle, 4)
            if distance ~= nil and distance > cullRadius then
                womatObstacleCullReport.culled[#womatObstacleCullReport.culled+1]={alias=alias, distance=distance}
                obstacleShapes={}
            elseif #obstacleShapes > 0 then
                womatObstacleCullReport.kept[#womatObstacleCullReport.kept+1]={alias=alias, distance=distance or -1}
            end
            local shapeCount=0
            for j=1,#obstacleShapes,1 do
                if collection == -1 then
                    collection=sim.createCollection(0)
                end
                sim.addItemToCollection(collection, sim.handle_single, obstacleShapes[j], 0)
                shapeCount=shapeCount+1
            end
            if collection ~= -1 then
                womatObstacleCollisionCache[#womatObstacleCollisionCache+1]={
                    alias=sim.getObjectAlias(obstacle, 4),
                    collection=collection,
                    shapeCount=shapeCount
                }
            end
        end
    end
    return womatObstacleCollisionCache
end

function womatClearObstacleCollisionCache()
    if womatObstacleCollisionCache == nil then
        return
    end
    for i=1,#womatObstacleCollisionCache,1 do
        local collection=womatObstacleCollisionCache[i].collection
        if collection ~= nil and collection ~= -1 then
            pcall(sim.destroyCollection, collection)
        end
    end
    womatObstacleCollisionCache=nil
end

function womatAddCollisionPoint(alias, x, y, z)
    if alias == nil then
        alias="unknown_collision_object"
    end
    if womatCollisionPointsByObstacle[alias] == nil then
        womatCollisionPointsByObstacle[alias]={}
        womatCollisionObstacleOrder[#womatCollisionObstacleOrder+1]=alias
    end
    local points=womatCollisionPointsByObstacle[alias]
    points[#points+1]=x
    points[#points+1]=y
    points[#points+1]=z
end

function womatCreateCollisionPointClouds(fallbackCollisionPoints)
    if #womatCollisionObstacleOrder == 0 then
        return
    end

    for i=1,#womatCollisionObstacleOrder,1 do
        local alias=womatCollisionObstacleOrder[i]
        local points=womatCollisionPointsByObstacle[alias]
        if points ~= nil and #points > 0 then
            local collPtcld=sim.createPointCloud(0.05,1,0,2)
            pcall(sim.setObjectAlias, collPtcld, "collision_"..alias)
            sim.insertPointsIntoPointCloud(collPtcld,0,points,{255,0,0})
        end
    end
end
`;
}

async function waitForScriptInitialization(sim, scriptHandle) {
  for (let attempt = 0; attempt < 30; attempt += 1) {
    try {
      const countResult = await sim.getScriptExecutionCount(scriptHandle);
      const count = Array.isArray(countResult) ? countResult[0] : countResult;
      if (Number(count) > 0) {
        return;
      }
    } catch (error) {
      // Some CoppeliaSim versions report execution count only after the first simulation step.
    }
    await delay(200);
  }
  await sim.initScript(scriptHandle);
}

async function executeInScript(sim, scriptHandle, code) {
  const result = await sim.executeScriptString(code, scriptHandle);
  if (!Array.isArray(result)) {
    return result;
  }
  return result.length > 2 ? result.slice(1) : result[1];
}

function gripperRuntimeConfig(ctx) {
  const grip = ctx.manifest.runtime?.gripper || {};
  return {
    modelName: grip.modelName || `${ctx.manifest.name}_tcp_gripper`,
    modelPath: grip.modelPath || "/home/maxchrs/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/models/components/grippers/RG2.ttm",
    modelPosition: grip.modelPosition || [0, 0, 0],
    modelOrientation: radiansTriple(grip.modelOrientation || [-1.57079632679, 0, 0], "runtime.gripper.modelOrientation"),
    modelLocalRotation: radiansTriple(grip.modelLocalRotation || [0, 0, 0], "runtime.gripper.modelLocalRotation"),
    // Off unless a manifest asks for it. This re-rotates an already-mounted
    // gripper in the WORLD by the robot's base orientation, but the gripper
    // hangs off the flange and so has already inherited that rotation through
    // the hierarchy -- so on any arm that is not standing square at the origin
    // it double-rotates the tool by exactly the base heading. It read as a
    // no-op for as long as every arm stood unrotated; docs/fitting_a_different
    // _gripper.md called the failure before it happened. Defaulting it on made
    // every placed arm wrong by default.
    applyBaseOrientationCorrection: grip.applyBaseOrientationCorrection === true,
    openJointPositions: grip.openJointPositions || [0.06],
    closedJointPositions: grip.closedJointPositions || [0],
    // An adopted model brings its own end effector, so there is no file to
    // load and no mount to make -- only the tool control point to measure.
    source: gripperSource(grip),
    sourceAlias: String(grip.sourceAlias ?? ""),
    toolControlPointOffset: toolControlPointOffset(grip),
    shipsWithModel: gripperSource(grip) === "builtin"
  };
}

function gripSceneConfig(ctx, cellSites) {
  // A is whichever site starts loaded. Only when no site says so does the pick
  // scene fall back to this robot's own grip config -- coordinates that are
  // world coordinates only because the scene owner sits at the origin.
  const loaded = loadedSite(cellSites);
  if (loaded) return gripFromSite(loaded);
  const grip = ctx.manifest.runtime?.gripper || {};
  return {
    objectName: grip.objectName || `${ctx.manifest.name}_grip_cube`,
    supportName: grip.supportName || `${ctx.manifest.name}_grip_support`,
    objectSize: grip.objectSize || [0.045, 0.045, 0.045],
    objectPosition: grip.objectPosition || [0.45, 0.2, 0.325],
    supportSize: grip.supportSize || [0.18, 0.18, 0.24],
    supportPosition: grip.supportPosition || [0.45, 0.2, 0.18],
    yawDeg: Number(grip.supportYawDeg || 0)
  };
}

function uniqueItems(values) {
  return [...new Set(values.filter(Boolean))];
}

// Finishes the gripper the driver's preamble already bolted on: applies the
// cell's base-orientation correction, makes its shapes static and collidable,
// parks the jaws open, and then re-measures the sampled point.
//
// The mount itself is deliberately NOT done here. This used to parent the
// gripper to simTip, which was the flange when it was written -- but the
// preamble rebinds simTip to the tool control point during script init, which
// runs first, so by the time this executed simTip was the jaw midpoint and the
// gripper was bolted one whole tool length past the flange, landing exactly on
// the TCP marker. Mounting goes through womatWorkspaceMountGripper(), which
// resolves the flange itself and cannot be pointed at a moving target.
function buildPipelineGripperLua({ modelName, openJointPositions, baseOrientation }) {
  return [
    "local ok, result=pcall(function()",
    `local name=${luaString(modelName)}`,
    `local openPositions=${luaArray(openJointPositions)}`,
    `womatWorkspaceBaseOrientation=${luaArray(baseOrientation)}`,
    "-- Leftovers from an earlier jaw-visualisation pass; they are not part of",
    "-- the model and would be swept into the collision collection.",
    "local oldLeft=sim.getObject('/'..name..'_visual_jaw_left', {noError=true})",
    "if oldLeft ~= -1 then pcall(sim.removeObject, oldLeft) end",
    "local oldRight=sim.getObject('/'..name..'_visual_jaw_right', {noError=true})",
    "if oldRight ~= -1 then pcall(sim.removeObject, oldRight) end",
    "local handle=womatWorkspaceMountGripper()",
    "local shapes=sim.getObjectsInTree(handle, sim.object_shape_type, 0)",
    "for i=1,#shapes,1 do",
    "    sim.setObjectInt32Param(shapes[i], sim.shapeintparam_static, 1)",
    "    sim.setObjectInt32Param(shapes[i], sim.shapeintparam_respondable, 0)",
    "    local okProperty, property=pcall(sim.getObjectSpecialProperty, shapes[i])",
    "    if okProperty and property ~= nil then",
    "        pcall(sim.setObjectSpecialProperty, shapes[i], property | sim.objectspecialproperty_collidable)",
    "    end",
    "    pcall(sim.resetDynamicObject, shapes[i])",
    "end",
    "local joints=sim.getObjectsInTree(handle, sim.object_joint_type, 0)",
    "for i=1,#joints,1 do",
    "    local target=openPositions[((i-1) % #openPositions) + 1]",
    "    local okInterval, cyclic, interval=pcall(sim.getJointInterval, joints[i])",
    "    if okInterval and not cyclic and interval ~= nil and math.abs(interval[2]) > 0.0001 and #openPositions == 1 then",
    "        target=interval[1]+interval[2]",
    "    end",
    "    pcall(sim.setJointMode, joints[i], sim.jointmode_kinematic, 0)",
    "    pcall(sim.setJointTargetForce, joints[i], 100)",
    "    pcall(sim.setJointTargetPosition, joints[i], target)",
    "    pcall(sim.setJointPosition, joints[i], target)",
    "end",
    "-- Last, because everything above moves the pads: the correction turns the",
    "-- tool and opening the jaws spreads them. The dummy is repositioned rather",
    "-- than replaced, so simTip and the IK environment built against it stay",
    "-- valid.",
    "womatWorkspaceRetargetTip()",
    "simTip=womatWorkspaceToolControlPoint()",
    "return {handle, #joints}",
    "end)",
    "if not ok then return false, tostring(result) end",
    "return true, result[1], result[2]"
  ].join("\n");
}

async function installPipelineGripper(sim, scriptHandle, ctx, baseOrientation = [0, 0, 0]) {
  const grip = gripperRuntimeConfig(ctx);
  const orientation = Array.isArray(baseOrientation) ? baseOrientation : [0, 0, Number(baseOrientation || 0)];
  // The file check guards the LOAD, and an adopted model has nothing to load:
  // its gripper came with it, and womatWorkspaceMountGripper skips the load
  // when the alias is already in the tree. Skipping the whole install for a
  // missing file would also skip the retarget, which is the part that puts the
  // tool control point on the jaw pads -- so the arm would be sampled, and then
  // driven, at its flange.
  if (grip.source === "none") {
    // No tool at all: the tool control point is the flange, and there is
    // nothing to mount, measure or open. Sampling still runs -- it just
    // describes where the flange reaches.
    console.log(`${ctx.manifest.name} declares no gripper; skipping tool install.`);
    return;
  }

  if (grip.source === "model" && !fs.existsSync(grip.modelPath)) {
    console.warn(`Skipping gripper install; model not found: ${grip.modelPath}`);
    return;
  }

  const result = await executeInScript(
    sim,
    scriptHandle,
    buildPipelineGripperLua({
      modelName: grip.modelName,
      openJointPositions: grip.openJointPositions,
      baseOrientation: orientation
    })
  );

  const ok = Array.isArray(result) ? result[0] : result;
  if (!ok) {
    const message = Array.isArray(result) ? result[1] : result;
    throw new Error(`Pipeline gripper install failed: ${message}`);
  }

  const handle = Array.isArray(result) ? result[1] : null;
  const jointCount = Array.isArray(result) ? result[2] : null;
  console.log(`Installed TCP gripper ${grip.modelName} into workspace scene (handle ${handle}, joints ${jointCount}).`);
}

async function installPipelineGripObjects(sim, scriptHandle, ctx, cellSites) {
  const sites = cellSites || [];
  // Declared sites ARE the scene. Falling back to this robot's own grip config
  // when a declaration exists but nothing on it is loaded would install a second
  // support and a second cube nobody asked for, at coordinates belonging to a
  // cell this run is not building -- which is how a single-arm scene ended up
  // with Robot4_grip_support standing next to the site that replaced it.
  const grip = sites.length > 0 ? (loadedSite(sites) && gripFromSite(loadedSite(sites))) : gripSceneConfig(ctx);
  // Every empty site is built the same way a loaded one is, in the same pass and
  // in the same frame. They used to be created later, by whichever robot's
  // driver started first, which meant the saved environment scene did not
  // contain the cell's own furniture.
  const empties = sites.filter((site) => !site.payload);
  if (!grip && empties.length === 0) return;
  const result = await executeInScript(
    sim,
    scriptHandle,
    [
      "local function setShapePhysics(root, isStatic, respondable)",
      "    local shapes=sim.getObjectsInTree(root, sim.object_shape_type, 0)",
      "    for i=1,#shapes,1 do",
      "        sim.setObjectInt32Param(shapes[i], sim.shapeintparam_static, isStatic and 1 or 0)",
      "        sim.setObjectInt32Param(shapes[i], sim.shapeintparam_respondable, respondable and 1 or 0)",
      "        pcall(sim.resetDynamicObject, shapes[i])",
      "    end",
      "end",
      "local function makeCuboid(alias, size, position, color, isStatic, respondable, yaw)",
      "    local existing=sim.getObject('/'..alias, {noError=true})",
      "    if existing ~= -1 then pcall(sim.removeObject, existing) end",
      "    local ok, handle=pcall(sim.createPrimitiveShape, sim.primitiveshape_cuboid, size, 0)",
      "    if not ok or handle == nil or handle == -1 then",
      "        handle=sim.createPureShape(0, 16, size, 0.05, nil)",
      "    end",
      "    sim.setObjectAlias(handle, alias)",
      "    sim.setObjectPosition(handle, sim.handle_world, position)",
      "    sim.setObjectOrientation(handle, sim.handle_world, {0, 0, yaw or 0})",
      "    setShapePhysics(handle, isStatic, respondable)",
      "    pcall(sim.setShapeColor, handle, nil, sim.colorcomponent_ambient_diffuse, color)",
      "    return handle",
      "end",
      ...(grip ? [
        `local support=makeCuboid(${luaString(grip.supportName)}, ${luaArray(grip.supportSize)}, ${luaArray(grip.supportPosition)}, {0.35,0.35,0.35}, true, true, ${radians(grip.yawDeg)})`,
        `local object=makeCuboid(${luaString(grip.objectName)}, ${luaArray(grip.objectSize)}, ${luaArray(grip.objectPosition)}, {0.9,0.25,0.1}, false, true, ${radians(grip.yawDeg)})`
      ] : ["local support, object"]),
      ...empties.map((site) =>
        `makeCuboid(${luaString(site.alias)}, ${luaArray(site.size)}, ${luaArray(site.position)}, {0.35,0.35,0.35}, true, true, ${radians(site.yawDeg)})`),
      "return true, support, object"
    ].join("\n")
  );

  const ok = Array.isArray(result) ? result[0] : result;
  if (!ok) {
    const message = Array.isArray(result) ? result[1] : result;
    throw new Error(`Pipeline grip object install failed: ${message}`);
  }

  if (grip) {
    console.log(`Installed pick cube ${grip.objectName} and support ${grip.supportName} into workspace scene.`);
  } else {
    console.log("No site carries a payload, so no pick cube was installed.");
  }
  if (empties.length > 0) {
    console.log(`Installed ${empties.length} empty cell site(s): ${empties.map((site) => site.alias).join(", ")}.`);
  }
}

function radians(degrees) {
  return Number(degrees || 0) * Math.PI / 180;
}

async function cleanupWorkspaceHelperShapes(sim, scriptHandle) {
  // Workspace visuals (workspace_reachable_points, collision_* point clouds and
  // the workspace_hull shape) stay in the saved scene; the lower thing server
  // strips them at load time when runtime.workspaceVisuals is false.
  const result = await executeInScript(
    sim,
    scriptHandle,
    [
      "local removed={helperShapes=0}",
      "local shapes=sim.getObjectsInTree(simBase, sim.object_shape_type, 1)",
      "for i=1,#shapes,1 do",
      "    local alias=sim.getObjectAlias(shapes[i], 4)",
      "    local parent=sim.getObjectParent(shapes[i])",
      "    if parent == simBase and string.match(alias, '^shape') then",
      "        if pcall(sim.removeObject, shapes[i]) then removed.helperShapes=removed.helperShapes+1 end",
      "    end",
      "end",
      "return true, removed"
    ].join("\n")
  );

  const ok = Array.isArray(result) ? result[0] : result;
  if (!ok) {
    console.warn("Workspace helper shape cleanup did not complete:", result);
    return;
  }

  const removed = Array.isArray(result) ? result[1] : result;
  console.log(`Removed workspace helper shapes: ${JSON.stringify(removed)}.`);
}

async function generateWorkspaceData(ctx, cellSites) {
  if (!fs.existsSync(ctx.paths.generatedModel)) {
    throw new Error(`Generated model not found. Run --generate-model first: ${ctx.paths.generatedModel}`);
  }
  // Sampling one arm in its own scene is still sampling it against the cell's
  // furniture, if the cell has any. A support the arm will have to reach around
  // is an obstacle here for the same reason it is one in the shared cell -- and
  // the arm stands at the origin in this scene, so the cell's coordinates are
  // already this robot's.
  const sites = cellSites || [];
  const samplingCtx = sites.length === 0 ? ctx : {
    ...ctx,
    manifest: {
      ...ctx.manifest,
      pipeline: {
        ...(ctx.manifest.pipeline || {}),
        collisionObstacles: uniqueItems([
          ...(ctx.manifest.pipeline?.collisionObstacles || []),
          ...sites.map((site) => site.alias)
        ])
      }
    }
  };

  ensureDir(ctx.infoDir);
  ensureDir(ctx.sceneDir);
  copyFileIfExists(ctx.paths.workspaceSceneTemplate, ctx.paths.generatedWorkspaceScene);

  const { sim } = await connect(ctx);
  await stopSimulationIfNeeded(sim);
  await sim.loadScene(ctx.paths.generatedWorkspaceScene);

  let robotHandle = await sim.loadModel(ctx.paths.generatedModel);
  robotHandle = Array.isArray(robotHandle) ? robotHandle[0] : robotHandle;
  await sim.setObjectPosition(robotHandle, await sim.handle_world, [0, 0, 0.03]);
  const robotNameResult = await sim.getObjectAlias(robotHandle);
  const robotName = Array.isArray(robotNameResult) ? robotNameResult[0] : robotNameResult;

  const robotScriptHandle = await loadWorkspaceDriver(sim, samplingCtx, robotName);
  await waitForScriptInitialization(sim, robotScriptHandle);
  await installPipelineGripper(sim, robotScriptHandle, ctx);
  await installPipelineGripObjects(sim, robotScriptHandle, ctx, sites);
  const divisions = Number(ctx.manifest.pipeline?.workspaceDivisions || 8);

  console.log(`Generating workspace data in ${ctx.infoDir} with ${divisions} divisions.`);
  const calculationResult = await executeInScript(
    sim,
    robotScriptHandle,
    [
      `generateWorkingspace(true, ${divisions}, ${luaString(ctx.infoDir)}, ${luaArray([1, 1, 1])}, ${luaArray([0, 0, 0])})`,
      "local cacheOk, cacheOrError=pcall(womatBuildObstacleCollisionCache)",
      "if not cacheOk then",
      "    womatClearObstacleCollisionCache()",
      "    return false, 'obstacle_cache_failed: '..tostring(cacheOrError)",
      "end",
      "local ok, err = pcall(doCalculation)",
      "womatClearObstacleCollisionCache()",
      "if not ok then return false, tostring(err) end",
      "local info=robotInfo()",
      "info.womatObstacleCulling=womatObstacleCullReport",
      "return true, info"
    ].join("\n")
  );
  const calculationOk = Array.isArray(calculationResult) ? calculationResult[0] : calculationResult;
  const robotInfo = Array.isArray(calculationResult) ? calculationResult[1] : null;
  if (!calculationOk) {
    throw new Error(`Workspace calculation failed in CoppeliaSim: ${robotInfo}`);
  }
  takeObstacleCullReport(ctx.manifest.name, robotInfo);
  await stopSimulationIfNeeded(sim);
  // Rebuilt after the sweep, because sampling leaves the scene mid-posture --
  // and THIS is the pass that gets saved. Called without the declared sites it
  // fell back to this robot's own grip config, so every run ended by putting the
  // default support and cube into the scene that was about to be written to
  // disk, whatever the sampling pass had been asked to build.
  await installPipelineGripObjects(sim, robotScriptHandle, ctx, sites);
  await cleanupWorkspaceHelperShapes(sim, robotScriptHandle);
  await sim.saveScene(ctx.paths.generatedWorkspaceScene);

  const rawRobotName = robotInfo.robotName || robotName;
  const originalInfo = path.join(ctx.infoDir, `${rawRobotName}_info.json`);
  writeJson(originalInfo, robotInfo);
  normalizeWorkspaceFiles(ctx, rawRobotName);

  console.log(`Saved workspace scene: ${ctx.paths.generatedWorkspaceScene}`);
  console.log(`Saved robot info: ${ctx.paths.robotInfo}`);

  return robotInfo;
}

function normalizeWorkspaceFiles(ctx, rawRobotName) {
  const candidates = [
    [`${rawRobotName}_info.json`, ctx.paths.robotInfo],
    [`${rawRobotName}_shape.stl`, ctx.paths.robotShape],
    [`${rawRobotName}_data_point.csv`, ctx.paths.robotDataPoints]
  ];

  for (const [sourceName, target] of candidates) {
    const source = path.join(ctx.infoDir, sourceName);
    if (source !== target && fs.existsSync(source)) {
      fs.copyFileSync(source, target);
    }
  }
}

function sharedSamplingContext(ctx, collisionObstacles, sharedGrip) {
  return {
    ...ctx,
    manifest: {
      ...ctx.manifest,
      pipeline: {
        ...(ctx.manifest.pipeline || {}),
        collisionObstacles
      },
      runtime: {
        ...(ctx.manifest.runtime || {}),
        collisionObstacles,
        gripper: {
          ...(ctx.manifest.runtime?.gripper || {}),
          objectName: sharedGrip.objectName,
          supportName: sharedGrip.supportName,
          objectPosition: sharedGrip.objectPosition,
          supportPosition: sharedGrip.supportPosition,
          objectSize: sharedGrip.objectSize,
          supportSize: sharedGrip.supportSize,
          // Nothing here builds the support -- sampling loads the environment
          // scene, which already has it standing and turned. Carried so the
          // copy of the shared grip is a faithful one.
          supportYawDeg: Number(sharedGrip.yawDeg || 0)
        }
      }
    }
  };
}

async function createSharedEnvironmentScene(sim, contexts, offsets, rotations, environmentScene, wallOffsets, wallYaws, cellSites) {
  const owner = contexts[0];
  const sites = cellSites || [];
  const sharedGrip = gripSceneConfig(owner, sites);
  const wallAliases = contexts.map((ctx) => `${ctx.manifest.name}_wall`);

  // The template robot stands at the origin, so the template wall's world
  // position IS the placement every robot inherits, expressed in its own frame.
  // Rotating that by the robot's yaw and adding its base offset puts the wall
  // where it belongs; adding this robot's displacement first moves it.
  const wallPlacement = (index) => {
    const cosine = Math.cos(rotations[index]);
    const sine = Math.sin(rotations[index]);
    const [dx, dy, dz] = wallOffsets[index];
    const localX = `(sourcePosition[1]+${luaNumber(dx)})`;
    const localY = `(sourcePosition[2]+${luaNumber(dy)})`;
    return [
      `{${luaNumber(offsets[index][0])}+${luaNumber(cosine)}*${localX}-${luaNumber(sine)}*${localY}, ${luaNumber(offsets[index][1])}+${luaNumber(sine)}*${localX}+${luaNumber(cosine)}*${localY}, sourcePosition[3]+${luaNumber(offsets[index][2])}+${luaNumber(dz)}}`,
      // The heading is the stance's unless the cell asked for one: an absolute
      // world yaw replaces it outright, the template wall standing upright with
      // its first two Euler angles zero.
      wallYaws[index] === null || wallYaws[index] === undefined
        ? `{sourceOrientation[1],sourceOrientation[2],sourceOrientation[3]+${luaNumber(rotations[index])}}`
        : `{sourceOrientation[1],sourceOrientation[2],${luaNumber(wallYaws[index])}}`
    ];
  };

  await stopSimulationIfNeeded(sim);
  await sim.loadScene(owner.paths.workspaceSceneTemplate);
  await stopSimulationIfNeeded(sim);

  const sandboxHandle = await getSandboxScriptHandle(sim);
  const wallResult = await executeInScript(
    sim,
    sandboxHandle,
    [
      "local source=sim.getObject('/80cmHighWall50cm', {noError=true})",
      "if source == -1 then return false, 'template_wall_missing' end",
      `sim.setObjectAlias(source, ${luaString(wallAliases[0])})`,
      "local sourcePosition=sim.getObjectPosition(source, sim.handle_world)",
      "local sourceOrientation=sim.getObjectOrientation(source, sim.handle_world)",
      "local handles={source}",
      // The clones are placed from sourcePosition, so the template wall is only
      // moved to its own placement after that vector has been read.
      ...contexts.slice(1).map((ctx, relativeIndex) => {
        const index = relativeIndex + 1;
        const [position, orientation] = wallPlacement(index);
        return [
          "do",
          "    local tree=sim.getObjectsInTree(source, sim.handle_all, 0)",
          "    local copies=sim.copyPasteObjects(tree, 0)",
          "    local wall=copies[1]",
          `    sim.setObjectAlias(wall, ${luaString(wallAliases[index])})`,
          `    sim.setObjectPosition(wall, sim.handle_world, ${position})`,
          `    sim.setObjectOrientation(wall, sim.handle_world, ${orientation})`,
          "    handles[#handles+1]=wall",
          "end"
        ].join("\n");
      }),
      `sim.setObjectPosition(source, sim.handle_world, ${wallPlacement(0)[0]})`,
      `sim.setObjectOrientation(source, sim.handle_world, ${wallPlacement(0)[1]})`,
      "return true, handles"
    ].join("\n")
  );
  if (!(Array.isArray(wallResult) ? wallResult[0] : wallResult)) {
    throw new Error(`Shared wall creation failed: ${JSON.stringify(wallResult)}`);
  }

  await installPipelineGripObjects(sim, sandboxHandle, owner, sites);
  ensureDir(path.dirname(environmentScene));
  await sim.saveScene(environmentScene);
  console.log(`Saved shared static environment: ${environmentScene}`);

  return {
    wallAliases,
    wallOffsets,
    wallYaws,
    sharedGrip,
    cellSites: sites,
    // Every site is a solid block whoever is standing next to it, including the
    // arm that will eventually place onto it -- which is the arm most likely to
    // hit it.
    collisionObstacles: uniqueItems([
      ...wallAliases,
      sharedGrip.supportName,
      ...sites.map((site) => site.alias)
    ])
  };
}

function transformWorkspaceCsvToLocal(csvFile, offset, orientation) {
  const lines = fs.readFileSync(csvFile, "utf8").trim().split(/\r?\n/);
  const transformed = [lines[0]];
  for (const line of lines.slice(1)) {
    const values = line.split(",").map(Number);
    if (values.length < 4 || values.some((value) => Number.isNaN(value))) {
      continue;
    }
    const local = inverseRotatePoint({
      x: values[0] - offset[0],
      y: values[1] - offset[1],
      z: values[2] - offset[2]
    }, orientation);
    transformed.push(`${local.x},${local.y},${local.z},${values[3]}`);
  }
  fs.writeFileSync(csvFile, transformed.join("\n") + "\n", "utf8");
}

// Which obstacles a robot is sampled against in the shared cell. By default
// every wall and every site in the cell -- the culling in the sampler then drops
// whatever lies out of the arm's reach. pipeline.sharedSamplingObstacles replaces
// that list for one robot. It is a manual override and it is trusted: an
// obstacle left out of it is never checked, and poses through it are sampled as
// reachable. Anything it leaves out is therefore named at every run.
function sharedSamplingObstacles(ctx, cellObstacles) {
  const override = ctx.manifest.pipeline?.sharedSamplingObstacles;
  if (override === undefined || override === null) return cellObstacles;
  if (!Array.isArray(override) || override.some((alias) => typeof alias !== "string" || alias === "")) {
    throw new Error(`${ctx.manifest.name}: pipeline.sharedSamplingObstacles must be a list of object aliases.`);
  }
  const omitted = cellObstacles.filter((alias) => !override.includes(alias));
  if (omitted.length > 0) {
    console.warn(`${ctx.manifest.name}: pipeline.sharedSamplingObstacles leaves out ${omitted.join(", ")}; `
      + "those are NOT checked while sampling this robot.");
  }
  return uniqueItems(override);
}

function takeObstacleCullReport(robotName, robotInfo) {
  const report = robotInfo?.womatObstacleCulling;
  if (robotInfo) delete robotInfo.womatObstacleCulling;
  if (!report) return null;
  const kept = (report.kept || []).map((entry) => entry.alias);
  const culled = (report.culled || []).map((entry) => `${entry.alias} (${Number(entry.distance).toFixed(2)} m)`);
  if (report.enabled && report.error) {
    console.warn(`${robotName}: obstacle culling disabled for this run, reach bound failed: ${report.error}`);
  } else if (report.enabled) {
    console.log(`${robotName}: sampling against ${kept.length} obstacle(s) [${kept.join(", ")}] within `
      + `${Number(report.radius).toFixed(2)} m (reach bound ${Number(report.reach).toFixed(2)} m + margin ${Number(report.margin).toFixed(2)} m); `
      + `culled ${culled.length}${culled.length ? ` [${culled.join(", ")}]` : ""}.`);
  } else {
    console.log(`${robotName}: obstacle culling off, sampling against ${kept.length} obstacle(s).`);
  }
  return { kept, culled: (report.culled || []).map((entry) => entry.alias), radius: report.radius, reach: report.reach };
}

// options.generateTd: false leaves the robot's TD alone -- for experiment runs
// that sample into their own directory and must not touch the live cell.
// options.samplingOverrides: merged into manifest.pipeline for this run only
// (seed, sample count, progress log).
async function calculateRobotInSharedEnvironment(sim, ctx, offset, orientation, environmentScene, sharedConfig, options = {}) {
  await stopSimulationIfNeeded(sim);
  await sim.loadScene(environmentScene);
  await stopSimulationIfNeeded(sim);

  let robotHandle = await sim.loadModel(ctx.paths.generatedModel);
  robotHandle = Number(Array.isArray(robotHandle) ? robotHandle[0] : robotHandle);
  await sim.setObjectOrientation(robotHandle, await sim.handle_world, orientation);
  await sim.setObjectPosition(robotHandle, await sim.handle_world, [
    offset[0],
    offset[1],
    BASE_PEDESTAL_Z + offset[2]
  ]);

  const baseSamplingCtx = sharedSamplingContext(ctx, sharedSamplingObstacles(ctx, sharedConfig.collisionObstacles), sharedConfig.sharedGrip);
  const samplingCtx = options.samplingOverrides
    ? { ...baseSamplingCtx, manifest: { ...baseSamplingCtx.manifest, pipeline: { ...baseSamplingCtx.manifest.pipeline, ...options.samplingOverrides } } }
    : baseSamplingCtx;
  const scriptHandle = await loadWorkspaceDriver(sim, samplingCtx, ctx.manifest.name);
  await waitForScriptInitialization(sim, scriptHandle);
  await installPipelineGripper(sim, scriptHandle, samplingCtx, orientation);

  const divisions = Number(ctx.manifest.pipeline?.workspaceDivisions || 8);
  const startedAt = Date.now();
  console.log(`Sampling ${ctx.manifest.name} alone at shared-cell offset [${offset.join(", ")}], orientation [${orientation.map((value) => value * 180 / Math.PI).join(", ")}]deg.`);
  const calculationResult = await executeInScript(
    sim,
    scriptHandle,
    [
      `generateWorkingspace(true, ${divisions}, ${luaString(ctx.infoDir)}, ${luaArray([1, 1, 1])}, ${luaArray([0, 0, 0])})`,
      "local cacheOk, cacheOrError=pcall(womatBuildObstacleCollisionCache)",
      "if not cacheOk then",
      "    womatClearObstacleCollisionCache()",
      "    return false, 'obstacle_cache_failed: '..tostring(cacheOrError)",
      "end",
      "local ok, err=pcall(doCalculation)",
      "womatClearObstacleCollisionCache()",
      "if not ok then return false, tostring(err) end",
      "local info=robotInfo()",
      "info.womatObstacleCulling=womatObstacleCullReport",
      "return true, info"
    ].join("\n")
  );
  const calculationOk = Array.isArray(calculationResult) ? calculationResult[0] : calculationResult;
  const robotInfo = Array.isArray(calculationResult) ? calculationResult[1] : null;
  if (!calculationOk) {
    throw new Error(`Shared-cell workspace calculation failed for ${ctx.manifest.name}: ${robotInfo}`);
  }
  const culling = takeObstacleCullReport(ctx.manifest.name, robotInfo);
  const seconds = (Date.now() - startedAt) / 1000;
  console.log(`Sampled ${ctx.manifest.name} in ${seconds.toFixed(1)} s.`);

  const rawRobotName = robotInfo.robotName || ctx.manifest.name;
  writeJson(path.join(ctx.infoDir, `${rawRobotName}_info.json`), robotInfo);
  normalizeWorkspaceFiles(ctx, rawRobotName);
  transformWorkspaceCsvToLocal(ctx.paths.robotDataPoints, offset, orientation);
  if (options.generateTd !== false) generateLowerTd(ctx);
  return { ...robotInfo, womatSampling: { seconds, culling } };
}

async function installSharedWorkspaceVisuals(sim, ctx, offset, orientation) {
  const xAxis = rotatePoint({ x: 1, y: 0, z: 0 }, orientation);
  const yAxis = rotatePoint({ x: 0, y: 1, z: 0 }, orientation);
  const zAxis = rotatePoint({ x: 0, y: 0, z: 1 }, orientation);
  const sandboxHandle = await getSandboxScriptHandle(sim);
  const result = await executeInScript(
    sim,
    sandboxHandle,
    [
      `local csvPath=${luaString(ctx.paths.robotDataPoints)}`,
      `local robot=sim.getObject('/'..${luaString(ctx.manifest.name)}, {noError=true})`,
      "if robot == -1 then return false, 'robot_missing' end",
      "local file=io.open(csvPath, 'r')",
      "if file == nil then return false, 'csv_unreadable' end",
      "local reachable={}",
      "local blocked={}",
      "local first=true",
      "for line in file:lines() do",
      "    if first then first=false else",
      "        local x,y,z,state=string.match(line, '([^,]+),([^,]+),([^,]+),([^,]+)')",
      "        if state ~= nil then",
      `            local lx,ly,lz=tonumber(x),tonumber(y),tonumber(z)`,
      `            local p={${offset[0]}+${xAxis.x}*lx+${yAxis.x}*ly+${zAxis.x}*lz,${offset[1]}+${xAxis.y}*lx+${yAxis.y}*ly+${zAxis.y}*lz,${offset[2]}+${xAxis.z}*lx+${yAxis.z}*ly+${zAxis.z}*lz}`,
      "            local destination=tonumber(state)==1 and reachable or blocked",
      "            destination[#destination+1]=p[1]",
      "            destination[#destination+1]=p[2]",
      "            destination[#destination+1]=p[3]",
      "        end",
      "    end",
      "end",
      "file:close()",
      "local cloud=sim.createPointCloud(0.05,1,0,2)",
      `sim.setObjectAlias(cloud, ${luaString(`workspace_${ctx.manifest.name}_reachable_points`)})`,
      "sim.insertPointsIntoPointCloud(cloud,0,reachable)",
      "if #blocked>0 then",
      "    local collisionCloud=sim.createPointCloud(0.05,1,0,2)",
      `    sim.setObjectAlias(collisionCloud, ${luaString(`collision_${ctx.manifest.name}_workspace_points`)})`,
      "    sim.insertPointsIntoPointCloud(collisionCloud,0,blocked,{255,0,0})",
      "end",
      "local ok,vertices,indices=pcall(sim.getQHull,reachable)",
      "if ok then",
      "    local hull=sim.createMeshShape(3,0,vertices,indices)",
      "    pcall(sim.reorientShapeBoundingBox,hull,-1)",
      "    sim.setShapeColor(hull,nil,0,{1,0,1})",
      "    sim.setShapeColor(hull,nil,4,{0.2})",
      "    sim.setObjectAlias(hull,'workspace_hull')",
      "    sim.setObjectParent(hull,robot,true)",
      "end",
      "return true,#reachable/3,#blocked/3"
    ].join("\n")
  );
  if (!(Array.isArray(result) ? result[0] : result)) {
    throw new Error(`Workspace visual installation failed for ${ctx.manifest.name}: ${JSON.stringify(result)}`);
  }
  console.log(`Installed shared-cell visuals for ${ctx.manifest.name}: ${result[1]} reachable, ${result[2]} collision.`);
}

async function installSharedSpaceVisual(sim, csvPath, voxelSize, alias = "workspace_shared_reachable_points", color = [0, 255, 0]) {
  const sandboxHandle = await getSandboxScriptHandle(sim);
  const result = await executeInScript(
    sim,
    sandboxHandle,
    [
      `local csvPath=${luaString(csvPath)}`,
      `local alias=${luaString(alias)}`,
      "local existing=sim.getObject('/'..alias, {noError=true})",
      "if existing ~= -1 then pcall(sim.removeObject, existing) end",
      "local file=io.open(csvPath, 'r')",
      "if file == nil then return false, 'shared_csv_unreadable' end",
      "local points={}",
      "local first=true",
      "for line in file:lines() do",
      "    if first then first=false else",
      "        local x,y,z=string.match(line, '([^,]+),([^,]+),([^,]+)')",
      "        if z ~= nil then",
      "            points[#points+1]=tonumber(x)",
      "            points[#points+1]=tonumber(y)",
      "            points[#points+1]=tonumber(z)",
      "        end",
      "    end",
      "end",
      "file:close()",
      "if #points == 0 then return true,0 end",
      `local cloud=sim.createPointCloud(${voxelSize},1,0,4)`,
      "sim.setObjectAlias(cloud,alias)",
      `sim.insertPointsIntoPointCloud(cloud,0,points,{${color.map(Number).join(",")}})`,
      "return true,#points/3"
    ].join("\n")
  );
  if (!(Array.isArray(result) ? result[0] : result)) {
    throw new Error(`Shared-space visual installation failed: ${JSON.stringify(result)}`);
  }
  const pointCount = Array.isArray(result) ? result[1] : 0;
  console.log(`Installed ${alias} visual: ${pointCount} voxel centers.`);
}

async function updateSharedSpaceVisualization(manifestFiles) {
  const contexts = manifestFiles.map(createContext);
  const cellName = contexts.map((ctx) => ctx.manifest.name).join("_");
  const outputDir = path.join(contexts[0].repoRoot, "generated", "robots");
  const descriptorPath = path.join(outputDir, `${cellName}.shared.json`);
  const defaultScenePath = path.join(outputDir, `${cellName}_shared.ttt`);
  const descriptor = fs.existsSync(descriptorPath) ? readJson(descriptorPath) : { name: cellName };
  const sharedScene = descriptor.scene
    ? path.resolve(contexts[0].repoRoot, descriptor.scene)
    : defaultScenePath;
  if (!fs.existsSync(sharedScene)) {
    throw new Error(`Shared scene missing: ${sharedScene}`);
  }
  const configuredPairs = descriptor.interactionSpace?.pairs;
  const outputs = Array.isArray(configuredPairs)
    ? configuredPairs.map((pair) => ({ key: pair.key, ...pair.output }))
    : [{
        key: `${cellName}`,
        csv: path.relative(contexts[0].repoRoot, path.join(outputDir, `${cellName}.shared-space.csv`)),
        metadata: path.relative(contexts[0].repoRoot, path.join(outputDir, `${cellName}.shared-space.json`))
      }];
  for (const output of outputs) {
    for (const [label, relative] of [["CSV", output.csv], ["metadata", output.metadata]]) {
      const file = path.resolve(contexts[0].repoRoot, relative);
      if (!fs.existsSync(file)) throw new Error(`Shared-space ${label} missing for ${output.key}: ${file}`);
    }
  }

  const { sim } = await connect(contexts[0]);
  await stopSimulationIfNeeded(sim);

  let loadedScene = "";
  try {
    const scenePathResult = await sim.getStringParam(await sim.stringparam_scene_path_and_name);
    loadedScene = String(Array.isArray(scenePathResult) ? scenePathResult[0] : scenePathResult);
  } catch (error) {
    // Older remote API versions might not expose the current scene path.
  }
  if (!loadedScene || path.resolve(loadedScene) !== path.resolve(sharedScene)) {
    await sim.loadScene(sharedScene);
    await stopSimulationIfNeeded(sim);
  }

  const palette = [[0, 255, 0], [0, 210, 255], [255, 210, 0], [210, 0, 255]];
  const sharedSpaces = [];
  for (let index = 0; index < outputs.length; index += 1) {
    const output = outputs[index];
    const csvPath = path.resolve(contexts[0].repoRoot, output.csv);
    const metadataPath = path.resolve(contexts[0].repoRoot, output.metadata);
    const metadata = readJson(metadataPath);
    const voxelSize = parsePositiveNumber(metadata.voxelSize, 0.05, "interaction-voxel-size");
    const alias = `interaction_${output.key}`.replace(/[^A-Za-z0-9_]/g, "_");
    const color = palette[index % palette.length];
    await installSharedSpaceVisual(sim, csvPath, voxelSize, alias, color);
    const previous = (descriptor.sharedSpaces || []).find((entry) => entry.pairKey === output.key) || {};
    sharedSpaces.push({
      ...previous,
      pairKey: output.key,
      version: metadata.version,
      voxelSize,
      voxelCount: Number(metadata.sharedVoxelCount || 0),
      csv: output.csv,
      metadata: output.metadata,
      visualObject: alias,
      color
    });
  }

  // Only the configured handover pairs get a cloud drawn in the scene -- a pair
  // that shares one voxel is worth knowing about and not worth rendering. Their
  // entries are carried through untouched rather than dropped, because this
  // step redraws visuals; it does not decide which pairs the cell knows about.
  for (const entry of descriptor.sharedSpaces || []) {
    if (!sharedSpaces.some((kept) => kept.pairKey === entry.pairKey)) sharedSpaces.push(entry);
  }
  await sim.saveScene(sharedScene);

  descriptor.scene = path.relative(contexts[0].repoRoot, sharedScene);
  descriptor.sharedSpaces = sharedSpaces;
  const drawn = sharedSpaces.filter((entry) => entry.visualObject && entry.version);
  if (drawn.length === 1) descriptor.sharedSpace = drawn[0];
  else delete descriptor.sharedSpace;
  writeJson(descriptorPath, descriptor);
  console.log(`Updated ${sharedSpaces.length} pairwise interaction-space visual(s) in: ${sharedScene}`);
  return sharedSpaces;
}

async function assembleSharedRobotScene(
  sim,
  contexts,
  offsets,
  orientations,
  environmentScene,
  sharedScene,
  sharedConfig
) {
  await stopSimulationIfNeeded(sim);
  await sim.loadScene(environmentScene);
  await stopSimulationIfNeeded(sim);

  for (let index = 0; index < contexts.length; index += 1) {
    const ctx = contexts[index];
    let robotHandle = await sim.loadModel(ctx.paths.generatedModel);
    robotHandle = Number(Array.isArray(robotHandle) ? robotHandle[0] : robotHandle);
    await sim.setObjectOrientation(robotHandle, await sim.handle_world, orientations[index]);
    await sim.setObjectPosition(robotHandle, await sim.handle_world, [
      offsets[index][0],
      offsets[index][1],
      BASE_PEDESTAL_Z + offsets[index][2]
    ]);
    const samplingCtx = sharedSamplingContext(ctx, sharedConfig.collisionObstacles, sharedConfig.sharedGrip);
    const scriptHandle = await loadWorkspaceDriver(sim, samplingCtx, ctx.manifest.name);
    await waitForScriptInitialization(sim, scriptHandle);
    await installPipelineGripper(sim, scriptHandle, samplingCtx, orientations[index]);
    await sim.removeScript(scriptHandle);
    await installSharedWorkspaceVisuals(sim, ctx, offsets[index], orientations[index]);
  }

  ensureDir(path.dirname(sharedScene));
  await sim.saveScene(sharedScene);
  console.log(`Saved assembled shared robot scene: ${sharedScene}`);
}

// The manifest is what a regeneration reads; the .asl is what the agent running
// right now reads. Writing only the first leaves the pick point one
// regeneration away from moving, which is the same stale constant with a longer
// story.
function writeGripTargetFact(ctx, gripPosition) {
  const aslRelative = ctx.manifest.generatedFiles?.agentSpeak;
  if (!aslRelative) return;
  const aslPath = path.resolve(ctx.repoRoot, aslRelative);
  if (!fs.existsSync(aslPath)) return;
  const fact = `(${gripPosition.join(", ")}).`;
  const source = fs.readFileSync(aslPath, "utf8")
    .replace(/^gripTargetPosition\([^\n]*\)\.$/m, `gripTargetPosition${fact}`)
    // The mobile variant names the same point differently: its arm is commanded
    // relative to a base that moves, so the fact is armGripPosition.
    .replace(/^armGripPosition\([^\n]*\)\.$/m, `armGripPosition${fact}`);
  fs.writeFileSync(aslPath, source, "utf8");
}

// Where the cube stands, in one robot's own frame.
//
// The site is declared in the CELL's frame and the arm is commanded in its own,
// so the two are the same place written two ways: subtract where the robot's
// base stands, undo its yaw. The height is the part that is easy to leave out
// and the part that puts the gripper through the support -- the declared z is
// the support's CENTRE, so the cube sits half a support and half a cube above
// it, plus the sliver it is spawned with so it settles onto the top face rather
// than starting inside it. Exactly the arithmetic cellSites uses to place the
// cube, because it has to describe the same cube.
function cellSiteGraspInRobotFrame(site, offset, orientation) {
  const world = payloadSpawnPosition(site.position, site.size, site.payload.size);
  const local = inverseRotatePoint(
    {
      x: world[0] - Number(offset[0] || 0),
      y: world[1] - Number(offset[1] || 0),
      z: world[2] - Number(offset[2] || 0)
    },
    orientation
  );
  return [local.x, local.y, local.z].map((value) => Number(value.toFixed(6)));
}

// gripTargetPosition is a generated CONSTANT, and it is the only thing about
// the cell that --sites did not move. Everything else followed the declaration
// -- the scene, the obstacle list, the descriptor every arm reads its sites out
// of -- while !graspCube kept sending the arm to wherever the support used to
// stand. Live cell: the support was moved with the cube on it and Robot1 went
// on descending onto the empty floor the old site had occupied.
//
// Written for every robot, not just the one that can reach it. Which arm serves
// the loaded site is discovered at runtime and must not be decided here; what is
// decided here is only that if an arm is asked to pick, it is asked to pick
// where the cube actually is.
function recordCellPickPoint(contexts, offsets, orientations, sites) {
  const loaded = loadedSite(sites);
  if (!loaded || !loaded.payload) {
    if ((sites || []).length > 0) {
      console.warn(
        "No declared site carries a payload, so every robot keeps the pickup point it was generated with. "
        + "Give a site a payload alias (site@x,y,z,cubeName) to move the pick with the support."
      );
    }
    return;
  }
  const recorded = contexts.map((ctx, index) => {
    const grip = cellSiteGraspInRobotFrame(loaded, offsets[index] || [0, 0, 0], orientations[index] || [0, 0, 0]);
    const manifest = readJson(ctx.manifestPath);
    manifest.agentDefaults = { ...(manifest.agentDefaults || {}), gripPosition: grip };
    writeJson(ctx.manifestPath, manifest);
    writeGripTargetFact(ctx, grip);
    return `${ctx.manifest.name} [${grip.map((value) => value.toFixed(3)).join(", ")}]`;
  });
  console.log(`Pick point follows ${loaded.alias}: ${recorded.join("; ")}`);
}

function configureInteractionOwnership(contexts, pairs, options = {}) {
  const staticChain = options.staticChain === true;
  const ownerHandoverAction = contexts.map((ctx) => {
    const relative = ctx.manifest.generatedFiles?.agentTd;
    const tdPath = relative ? path.resolve(ctx.repoRoot, relative) : null;
    return tdPath && fs.existsSync(tdPath) ? readJson(tdPath).actions?.handoverCube : null;
  }).find(Boolean);

  for (const ctx of contexts) {
    const ownedPairs = pairs.filter((pair) => pair.ownerRobot === ctx.manifest.name);
    const neighborNames = pairs
      .filter((pair) => pair.robots.includes(ctx.manifest.name))
      .map((pair) => pair.robots.find((robot) => robot !== ctx.manifest.name));
    const tdRelative = ctx.manifest.generatedFiles?.agentTd;
    if (tdRelative) {
      const tdPath = path.resolve(ctx.repoRoot, tdRelative);
      if (fs.existsSync(tdPath)) {
        const td = readJson(tdPath);
        if (ownedPairs.length > 0 && ownerHandoverAction) td.actions.handoverCube = ownerHandoverAction;
        if (ownedPairs.length === 0) delete td.actions.handoverCube;
        writeJson(tdPath, td);
      }
    }

    const aslRelative = ctx.manifest.generatedFiles?.agentSpeak;
    if (!aslRelative) continue;
    const aslPath = path.resolve(ctx.repoRoot, aslRelative);
    if (!fs.existsSync(aslPath)) continue;
    let source = fs.readFileSync(aslPath, "utf8");
    const robotAtom = ctx.manifest.name.substring(0, 1).toLowerCase() + ctx.manifest.name.substring(1);
    source = source
      .replace(/^interactionSpaceOwner\([^\n]*\)\.\n?/gm, "")
      .replace(/^interactionSpacePartner\([^\n]*\)\.\n?/gm, "")
      .replace(/^interactionSpaceCoordinator\([^\n]*\)\.\n?/gm, "")
      .replace(/^handoverPartner\([^\n]*\)\.\n?/gm, "")
      .replace(/^handoverSuccessor\([^\n]*\)\.\n?/gm, "")
      .replace(/^!buildInteractionSpace\([^\n]*\)\.\n?/gm, "")
      .replace(/\n?\/\/ BEGIN GENERATED PAIR TOPOLOGY[\s\S]*?\/\/ END GENERATED PAIR TOPOLOGY\n?/g, "\n");
    const atom = (name) => name.substring(0, 1).toLowerCase() + name.substring(1);
    source += [
      "",
      "// BEGIN GENERATED PAIR TOPOLOGY",
      ...neighborNames.map((neighbor) => `interactionSpaceNeighbor(${atom(neighbor)}_agent, ${atom(neighbor)}).`),
      ...neighborNames.map((neighbor) => `handoverPartner(${atom(neighbor)}_agent, ${atom(neighbor)}).`),
      ...ownedPairs.map((pair) => {
        const partner = pair.robots.find((robot) => robot !== ctx.manifest.name);
        return `interactionSpaceOwner(${robotAtom}, ${atom(partner)}).`;
      }),
      // The configured chain, emitted only when asked for. It is the baseline a
      // discovered route is compared against, not the thing that drives
      // execution -- an agent that has worked a route out for itself follows
      // its own nextHop, and these facts would silently take over the moment
      // the search came back empty. Keeping them opt-in means a run that looks
      // like routing IS routing.
      ...(staticChain
        ? ownedPairs.map((pair) => {
          const partner = pair.robots.find((robot) => robot !== ctx.manifest.name);
          return `handoverSuccessor(${atom(partner)}_agent, ${atom(partner)}).`;
        })
        : []),
      // No !buildInteractionSpace goals are emitted here any more. Which spaces
      // this agent builds follows from the neighbours it discovers at startup
      // and from who owns each pair, so writing the goals into the topology
      // block would compute them a second time -- once from the configured
      // topology and once from the discovered one.
      "// END GENERATED PAIR TOPOLOGY",
      ""
    ].join("\n");
    fs.writeFileSync(aslPath, source, "utf8");
  }
}

function writeCombinedJcm(contexts, cellName) {
  const jcmPath = path.join(contexts[0].repoRoot, "generated", "robots", `${cellName}.jcm`);
  const atom = (name) => String(name).replace(/[^A-Za-z0-9_]/g, "_").toLowerCase().replace(/^([^a-z])/, "r_$1");
  const agents = contexts.map((ctx) => {
    const robotAtom = atom(ctx.manifest.name);
    const asl = path.basename(ctx.manifest.generatedFiles.agentSpeak);
    return `    agent ${robotAtom}_agent: generated/${asl} {\n        focus: ${robotAtom}_workspace.wotComm\n    }`;
  });
  const workspaces = contexts.map((ctx) => {
    const robotAtom = atom(ctx.manifest.name);
    return `    workspace ${robotAtom}_workspace {\n        artifact wotComm: wotCommunication.wotCommunicator("${ctx.manifest.configKey}")\n    }`;
  });
  ensureDir(path.dirname(jcmPath));
  fs.writeFileSync(jcmPath, `/*\n    Generated multi-robot JaCaMo application for ${cellName}.\n    Start the shared lower runtime and every AgentThing before this application.\n*/\n\nmas ${atom(cellName)}_mas {\n${agents.join("\n\n")}\n\n${workspaces.join("\n\n")}\n}\n`, "utf8");
  return path.relative(contexts[0].repoRoot, jcmPath);
}

async function generateSharedCell(manifestFiles, offsetsArg, rotationsArg, orientationsArg, interactionPairsArg, wallOffsetsArg, resampleArg, options = {}) {
  const contexts = manifestFiles.map(createContext);
  contexts.forEach((ctx) => {
    if (!validateContext(ctx)) {
      throw new Error(`Invalid pipeline context for ${ctx.manifest.name}.`);
    }
    if (!fs.existsSync(ctx.paths.generatedModel)) {
      throw new Error(`Generated model missing for ${ctx.manifest.name}: ${ctx.paths.generatedModel}`);
    }
  });
  const offsets = parseOffsets(offsetsArg, contexts.length);
  const rotations = parseRotations(rotationsArg, contexts.length);
  const orientations = parseOrientations(orientationsArg, rotations, contexts.length);
  const wallOffsets = parseWallOffsets(wallOffsetsArg, contexts.length);
  const wallYawOverrides = parseWallYaws(options.wallYaws, contexts.length);
  const resample = parseResampleSelection(resampleArg, contexts);
  // Where the task stands, in the cell's frame. Given on the command line it is
  // the declaration; otherwise the cell keeps whatever it was last built with,
  // and a cell that never had one falls back to reading each robot's own.
  const declaredSites = parseCellSites(options.sites);
  offsets[0] = [0, 0, 0];
  if (!orientationsArg) orientations[0] = [0, 0, 0];
  const cellName = contexts.map((ctx) => ctx.manifest.name).join("_");
  const environmentScene = path.join(contexts[0].repoRoot, "generated", "robots", `${cellName}_environment.ttt`);
  const sharedScene = path.join(contexts[0].repoRoot, "generated", "robots", `${cellName}_shared.ttt`);
  const descriptorPath = path.join(contexts[0].repoRoot, "generated", "robots", `${cellName}.shared.json`);
  const previousDescriptor = fs.existsSync(descriptorPath) ? readJson(descriptorPath) : {};
  const cellSites = declaredSites.length > 0
    ? declaredSites
    : resolveCellSites(
      previousDescriptor.taskSites
        ? [{ manifest: { sharedEnvironment: { taskSites: previousDescriptor.taskSites } } }]
        : contexts,
      offsets
    );
  if (cellSites.length > 0) {
    console.log(`Cell sites: ${cellSites.map((site) => `${site.alias} at [${site.position.map((value) => value.toFixed(3)).join(", ")}] yaw ${site.yawDeg} deg${site.payload ? ` holding ${site.payload.alias}` : " empty"}`).join("; ")}`);
  }

  const { sim } = await connect(contexts[0]);
  const wallYaws = orientations.map(floorPlaneYaw);
  const sharedConfig = await createSharedEnvironmentScene(
    sim,
    contexts,
    offsets,
    wallYaws,
    environmentScene,
    wallOffsets,
    wallYawOverrides,
    cellSites
  );
  for (let index = 0; index < contexts.length; index += 1) {
    if (!resample[index]) {
      if (!fs.existsSync(contexts[index].paths.robotDataPoints)) {
        throw new Error(`${contexts[index].manifest.name} is excluded from --resample but has no workspace to reuse: ${contexts[index].paths.robotDataPoints}`);
      }
      console.log(`Reusing existing workspace for ${contexts[index].manifest.name} (excluded from --resample).`);
      continue;
    }
    await calculateRobotInSharedEnvironment(sim, contexts[index], offsets[index], orientations[index], environmentScene, sharedConfig);
  }
  await assembleSharedRobotScene(
    sim,
    contexts,
    offsets,
    orientations,
    environmentScene,
    sharedScene,
    sharedConfig
  );
  const pairs = discoverInteractionPairs(contexts, offsets, orientations, interactionPairsArg).map((pair) => ({
    ...pair,
    output: {
      csv: path.posix.join("generated", "robots", "interactions", pair.key, "space.csv"),
      metadata: path.posix.join("generated", "robots", "interactions", pair.key, "space.json")
    }
  }));
  // One arm has nobody to overlap with, and that is a cell -- a rig for driving
  // a single robot over its own Thing Description, with the same scene, walls
  // and sites the four-arm cell is built from. Only a cell that HAS neighbours
  // and found none of them is misconfigured.
  if (pairs.length === 0 && contexts.length > 1) {
    throw new Error("No neighboring workspace bounding boxes overlap; adjust --offsets or provide --interaction-pairs after checking reachability.");
  }
  if (pairs.length === 0) {
    console.log(`Single-robot cell: ${contexts[0].manifest.name} has no neighbours, so no interaction pairs and no handovers.`);
  }

  const descriptor = {
    name: cellName,
    scene: path.relative(contexts[0].repoRoot, sharedScene),
    environmentScene: path.relative(contexts[0].repoRoot, environmentScene),
    collisionObstacles: sharedConfig.collisionObstacles,
    support: sharedConfig.sharedGrip.supportName,
    cube: sharedConfig.sharedGrip.objectName,
    interactionSpace: {
      ownership: "canonical_pair_first_robot",
      pairs,
      targetVoxelCount: Number(previousDescriptor.interactionSpace?.targetVoxelCount || 1000),
      proximityFactor: Number(previousDescriptor.interactionSpace?.proximityFactor || 1.25),
      sampleSize: Number(previousDescriptor.interactionSpace?.sampleSize || 20000),
      spacingPercentile: Number(previousDescriptor.interactionSpace?.spacingPercentile || 0.75),
      minVoxelSize: Number(previousDescriptor.interactionSpace?.minVoxelSize || 0),
      maxVoxelSize: Number(previousDescriptor.interactionSpace?.maxVoxelSize || 0.2),
      // Below this a pair's overlap is two reach spheres grazing each other, not
      // a region either arm can meet in. See rank() for what it costs to price
      // one anyway.
      minimumSharedVoxels: Number(previousDescriptor.interactionSpace?.minimumSharedVoxels ?? 20),
      bidirectional: previousDescriptor.interactionSpace?.bidirectional !== false,
      seed: Number(previousDescriptor.interactionSpace?.seed || 42),
      centralCore: {
        lowerQuantile: Number(previousDescriptor.interactionSpace?.centralCore?.lowerQuantile ?? 0.2),
        upperQuantile: Number(previousDescriptor.interactionSpace?.centralCore?.upperQuantile ?? 0.8),
        minimumCandidates: Number(previousDescriptor.interactionSpace?.centralCore?.minimumCandidates || 10),
        maximumCandidates: Number(previousDescriptor.interactionSpace?.centralCore?.maximumCandidates || 60)
      },
      visual: {
        object: "workspace_shared_reachable_points",
        color: [0, 255, 0]
      }
    },
    ...(previousDescriptor.handoverCalibration
      ? { handoverCalibration: previousDescriptor.handoverCalibration }
      : {}),
    taskSites: sharedConfig.cellSites,
    robots: contexts.map((ctx, index) => ({
      name: ctx.manifest.name,
      manifest: path.relative(ctx.repoRoot, ctx.manifestPath),
      workspaceCsv: path.relative(ctx.repoRoot, ctx.paths.robotDataPoints),
      offset: offsets[index],
      rotationDeg: orientations[index][2] * 180 / Math.PI,
      orientationDeg: orientations[index].map((value) => value * 180 / Math.PI),
      wall: sharedConfig.wallAliases[index],
      wallOffset: wallOffsets[index],
      // The heading the cell ASKED for, null when the wall kept its stance's.
      // Read back from the scene it would be a number either way, and the
      // difference matters: this is the input a rebuild has to repeat.
      wallYawDeg: wallYawOverrides[index] === null ? null : wallYawOverrides[index] * 180 / Math.PI
    }))
  };
  writeJson(descriptorPath, descriptor);
  for (let index = 0; index < contexts.length; index += 1) {
    const manifest = readJson(contexts[index].manifestPath);
    manifest.sharedEnvironment = {
      descriptor: path.relative(contexts[index].repoRoot, descriptorPath),
      scene: descriptor.scene,
      environmentScene: descriptor.environmentScene,
      offset: offsets[index],
      rotationDeg: orientations[index][2] * 180 / Math.PI,
      orientationDeg: orientations[index].map((value) => value * 180 / Math.PI),
      wallAliases: sharedConfig.wallAliases,
      wall: sharedConfig.wallAliases[index],
      wallOffset: wallOffsets[index],
      wallYawDeg: wallYawOverrides[index] === null ? null : wallYawOverrides[index] * 180 / Math.PI,
      collisionObstacles: sharedConfig.collisionObstacles,
      // The same list in every manifest, in the cell's frame. A robot that only
      // knew its own sites could never truthfully say it does not have one.
      taskSites: sharedConfig.cellSites,
      supportName: sharedConfig.sharedGrip.supportName,
      objectName: sharedConfig.sharedGrip.objectName,
      // A pair's approachMode is carried forward rather than regenerated. It is
      // the one field on a pair entry that nothing here can derive -- it says
      // how one arm has to come at another given both their mountings and both
      // their grippers, which is found by trying it in the cell -- and the two
      // readers of it (the runtime's handoverApproachMode and the ranking's
      // handoverApproachModeFor) both look it up on the manifest. Rebuilding
      // the cell silently dropped it, so the tuning survived exactly until the
      // next regeneration and the pair went back to receiving head-on.
      interactionPairs: pairs.map(({ key, robots, ownerRobot }) => {
        const previous = (manifest.sharedEnvironment?.interactionPairs || [])
          .find((entry) => entry.key === key);
        const approachMode = previous?.approachMode;
        return approachMode === undefined
          ? { key, robots, ownerRobot }
          : { key, robots, ownerRobot, approachMode };
      })
    };
    writeJson(contexts[index].manifestPath, manifest);
  }
  recordCellPickPoint(contexts, offsets, orientations, sharedConfig.cellSites);
  configureInteractionOwnership(contexts, pairs, { staticChain: options.staticChain === true });
  const combinedJcm = writeCombinedJcm(contexts, cellName);
  console.log(`Interaction pairs: ${pairs.map((pair) => `${pair.key} (owner ${pair.ownerRobot})`).join(", ")}`);
  console.log(`Combined JaCaMo application: ${combinedJcm}`);
  console.log(`Saved shared-cell descriptor: ${descriptorPath}`);
  return descriptor;
}

function csvExtremes(csvFile) {
  const rows = fs.readFileSync(csvFile, "utf8").trim().split(/\r?\n/).map((line) => line.split(","));
  const numericRows = rows.slice(1).map((row) => row.map(Number)).filter((row) => row.length >= 4 && row.every((value) => !Number.isNaN(value)));
  const validRows = numericRows.filter((row) => row[3] === 1);
  const sourceRows = validRows.length > 0 ? validRows : numericRows;
  const fallback = [-100, 100, -100, 100, -100, 100];

  if (sourceRows.length === 0) {
    return fallback;
  }

  return [0, 1, 2].flatMap((index) => {
    const values = sourceRows.map((row) => row[index]);
    return [Math.min(...values), Math.max(...values)];
  });
}

function addJointProperties(td, robotInfo) {
  const amount = Number(robotInfo.jointAmount || 0);
  for (let index = 0; index < amount; index += 1) {
    const jointName = `joint${index + 1}`;
    const jointType = robotInfo.jointTypes?.[index];
    const low = Number(robotInfo.jointLimitLows?.[index] ?? -Math.PI);
    const high = Number(robotInfo.jointLimitHighs?.[index] ?? Math.PI);
    const isRevolute = jointType === "Revolute_joint";
    const limits = isRevolute
      ? { unit: "deg", minimum: low * 180 / Math.PI - 1, maximum: high * 180 / Math.PI + 1 }
      : { unit: "meter", minimum: low, maximum: high };

    td.actions.moveTojointPosition.input.properties[jointName] = { type: "number", ...limits };
    td.properties.getJointposition.properties[jointName] = { type: "number", ...limits };
  }
}

function generateLowerTd(ctx) {
  if (!fs.existsSync(ctx.paths.robotInfo)) {
    throw new Error(`Robot info not found. Run --generate-td first: ${ctx.paths.robotInfo}`);
  }
  if (!fs.existsSync(ctx.paths.robotDataPoints)) {
    throw new Error(`Workspace CSV not found: ${ctx.paths.robotDataPoints}`);
  }

  ensureDir(ctx.tdDir);
  const td = readJson(ctx.paths.robotTemplate);
  const robotInfo = readJson(ctx.paths.robotInfo);
  const robotType = ctx.manifest.name;
  const title = `coppeliasim_virtualrobot_${robotType}`;
  // node-wot normalizes the HTTP route derived from the Thing title to lower
  // case. Keep the human-readable TD title, but advertise the actual route.
  const hrefBase = `http://localhost:${ctx.manifest.ports.lowerThing}/${title.toLowerCase()}`;
  const [minX, maxX, minY, maxY, minZ, maxZ] = csvExtremes(ctx.paths.robotDataPoints);

  td.title = title;
  td.id = `urn:dev:ops:womat-generated-virtual-${robotType}`;
  td.description = `Generated lower-level CoppeliaSim Thing for ${robotType}.`;
  td.links[0].href = ctx.paths.robotShape;
  td.links[1].href = ctx.paths.robotDataPoints;
  td.links[2] = {
    href: ctx.paths.generatedWorkspaceScene,
    type: "application/octet-stream",
    rel: "coppeliasim scene"
  };

  addJointProperties(td, robotInfo);

  const motionOutput = {
    type: "object",
    properties: {
      success: { type: "boolean" },
      state: { type: "string" },
      reason: { type: "string" },
      finalError: { type: "number" },
      tolerance: { type: "number" },
      planner: { type: "string" }
    },
    additionalProperties: true
  };
  td.actions.moveTocartesianPosition.output = motionOutput;
  td.actions.moveTojointPosition.output = motionOutput;

  td.properties.getGripperState = {
    type: "object",
    description: "Current generated virtual gripper state.",
    properties: {
      closed: { type: "boolean" },
      holding: { type: "boolean" },
      object: { type: "string" }
    },
    forms: [
      {
        href: `${hrefBase}/properties/getGripperState`,
        "htv:methodName": "GET"
      }
    ]
  };

  td.properties.getCollisionObstacles = {
    type: "array",
    description: "Scene objects currently treated as collision obstacles. The configured pick object is excluded.",
    items: {
      type: "string"
    },
    forms: [
      {
        href: `${hrefBase}/properties/getCollisionObstacles`,
        "htv:methodName": "GET"
      }
    ]
  };

  td.properties.getCubeOwnership = {
    type: "object",
    description: "Shared runtime ownership of the configured cube.",
    properties: {
      object: { type: "string" },
      owner: { type: "string" },
      transferActive: { type: "boolean" }
    },
    forms: [
      {
        href: `${hrefBase}/properties/getCubeOwnership`,
        "htv:methodName": "GET"
      }
    ]
  };

  td.properties.getBasePose = {
    type: "object",
    description: "Live world pose of this robot's base in the loaded scene, with the placement the runtime was configured with.",
    properties: {
      robot: { type: "string" },
      position: { type: "array", items: { type: "number" } },
      orientation: { type: "array", items: { type: "number" } },
      configuredOffset: { type: "array", items: { type: "number" } },
      configuredOrientation: { type: "array", items: { type: "number" } }
    },
    forms: [
      {
        href: `${hrefBase}/properties/getBasePose`,
        "htv:methodName": "GET"
      }
    ]
  };

  td.actions.refreshInteractionVisual = {
    description: "Redraw a pairwise interaction-space point cloud in the loaded scene from its exported CSV.",
    input: {
      type: "object",
      properties: {
        pairKey: { type: "string" },
        csv: { type: "string" },
        voxelSize: { type: "number" },
        color: { type: "array", items: { type: "number" } }
      },
      required: ["pairKey", "csv", "voxelSize"]
    },
    output: {
      type: "object",
      properties: {
        success: { type: "boolean" },
        reason: { type: "string" },
        object: { type: "string" },
        pointCount: { type: "number" }
      }
    },
    forms: [
      {
        href: `${hrefBase}/actions/refreshInteractionVisual`,
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.closeGripper = {
    description: "Close the generated virtual gripper and attach the configured cube when it is near the robot tip.",
    output: {
      type: "object",
      properties: {
        success: { type: "boolean" },
        state: { type: "string" },
        reason: { type: "string" },
        distance: { type: "number" }
      }
    },
    forms: [
      {
        href: `${hrefBase}/actions/closeGripper`,
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.openGripper = {
    description: "Open the generated virtual gripper and release the currently attached object.",
    output: {
      type: "object",
      properties: {
        success: { type: "boolean" },
        state: { type: "string" },
        reason: { type: "string" }
      }
    },
    forms: [
      {
        href: `${hrefBase}/actions/openGripper`,
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.transferCube = {
    description: "Transfer the held shared cube to another robot without pose validation.",
    input: {
      type: "object",
      properties: {
        session: { type: "string" },
        targetRobot: { type: "string" }
      },
      required: ["session", "targetRobot"]
    },
    output: {
      type: "object",
      properties: {
        success: { type: "boolean" },
        state: { type: "string" },
        reason: { type: "string" },
        sourceRobot: { type: "string" },
        targetRobot: { type: "string" },
        validation: { type: "string" },
        placement: { type: "string" },
        retreatCollisionMode: { type: "string" }
      }
    },
    forms: [
      {
        href: `${hrefBase}/actions/transferCube`,
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.beginHandoverSession = {
    description: "Register a physical handover session selected by the interaction-space owner agent.",
    input: {
      type: "object",
      properties: {
        session: { type: "string" },
        targetRobot: { type: "string" }
      },
      required: ["session", "targetRobot"]
    },
    output: {
      type: "object",
      properties: {
        success: { type: "boolean" },
        reason: { type: "string" },
        session: { type: "string" },
        state: { type: "string" },
        sourceRobot: { type: "string" },
        targetRobot: { type: "string" }
      }
    },
    forms: [
      {
        href: `${hrefBase}/actions/beginHandoverSession`,
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.setHandoverApproach = {
    description: "Enable or disable receiver approach mode for an agent-created handover session.",
    input: {
      type: "object",
      properties: {
        session: { type: "string" },
        enabled: { type: "boolean" }
      },
      required: ["session", "enabled"]
    },
    output: {
      type: "object",
      properties: {
        success: { type: "boolean" },
        state: { type: "string" },
        reason: { type: "string" },
        ignoredGripper: { type: "string" },
        planId: { type: "string" }
      }
    },
    forms: [
      {
        href: `${hrefBase}/actions/setHandoverApproach`,
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.setPickupApproach = {
    description: "Enable or disable BDI-controlled pickup approach mode. Only the shared cube support is ignored; walls and robots remain collision obstacles.",
    input: {
      type: "object",
      properties: { enabled: { type: "boolean" } },
      required: ["enabled"]
    },
    output: {
      type: "object",
      properties: {
        success: { type: "boolean" },
        state: { type: "string" },
        reason: { type: "string" },
        enabled: { type: "boolean" },
        ignoredSupport: { type: "string" }
      }
    },
    forms: [
      {
        href: `${hrefBase}/actions/setPickupApproach`,
        "htv:methodName": "POST"
      }
    ]
  };

  // The sampled bounding box constrains a COMMAND and must not constrain a
  // REPORT. A commanded target outside the sampled workspace is worth refusing;
  // the arm's own measured pose is ground truth, and the sample understates the
  // true envelope for the same reason reachRadius understates the true reach --
  // full extension is a pose random joint sampling rarely lands on. Declaring
  // the sample as the report's bounds makes a consumer reject the arm's actual
  // position whenever it stands near its limits, which is not a validation
  // failure but a valid reading being thrown away.
  const commandTarget = td.actions.moveTocartesianPosition.input.properties;
  commandTarget.x.minimum = minX;
  commandTarget.x.maximum = maxX;
  commandTarget.y.minimum = minY;
  commandTarget.y.maximum = maxY;
  commandTarget.z.minimum = minZ;
  commandTarget.z.maximum = maxZ;
  for (const target of [commandTarget, td.properties.getCartesianposition.properties]) {
    for (const axis of ["x", "y", "z"]) {
      target[axis].unit = "meter";
    }
  }
  for (const axis of ["x", "y", "z"]) {
    delete td.properties.getCartesianposition.properties[axis].minimum;
    delete td.properties.getCartesianposition.properties[axis].maximum;
  }

  td.properties.getJointposition.forms[0].href = `${hrefBase}/properties/getJointposition`;
  td.properties.getCartesianposition.forms[0].href = `${hrefBase}/properties/getCartesianposition`;
  td.properties.getGripperState.forms[0].href = `${hrefBase}/properties/getGripperState`;
  td.properties.getCollisionObstacles.forms[0].href = `${hrefBase}/properties/getCollisionObstacles`;
  td.properties.getBasePose.forms[0].href = `${hrefBase}/properties/getBasePose`;
  td.actions.refreshInteractionVisual.forms[0].href = `${hrefBase}/actions/refreshInteractionVisual`;
  td.actions.moveTojointPosition.forms[0].href = `${hrefBase}/actions/moveTojointPosition`;
  td.actions.moveTocartesianPosition.forms[0].href = `${hrefBase}/actions/moveTocartesianPosition`;
  td.actions.closeGripper.forms[0].href = `${hrefBase}/actions/closeGripper`;
  td.actions.openGripper.forms[0].href = `${hrefBase}/actions/openGripper`;

  // The joint count in the Thing Description comes from the SAMPLE -- robotInfo
  // reports #jointHandles as CoppeliaSim saw it -- and the manifest states how
  // many axes the arm has. They can only disagree if the TD was written against
  // a sampling run older than the manifest, which is exactly what a stale TD
  // looks like: a manifest saying four axes and a description advertising five.
  // Consumers believe the description, so say it here, where both are known.
  const sampledAxes = Number(robotInfo.jointAmount || 0);
  const declaredAxes = (ctx.manifest.mobility?.armJoints || []).length;
  if (declaredAxes > 0 && sampledAxes !== declaredAxes) {
    const locked = ctx.manifest.mobility?.lockedJoints || [];
    console.warn(
      `${ctx.manifest.name}: the sample found ${sampledAxes} joint(s) but the manifest declares `
      + `${declaredAxes} arm joint(s)${locked.length ? ` with ${locked.join(", ")} locked` : ""}. `
      + "The Thing Description advertises what was sampled; re-run --generate-td to bring them back in step."
    );
  }
  writeJson(ctx.paths.generatedLowerTd, td);
  console.log(`Saved lower-level TD: ${ctx.paths.generatedLowerTd} (${sampledAxes} joint(s))`);
  return td;
}

// What a solo run built is what the manifest has to say it has.
//
// A robot served on its own loads its OWN workspace scene and rebuilds the pick
// objects from runtime.gripper, so a sampling run that installed declared sites
// and then left the gripper config pointing at this robot's default support and
// cube produced a scene with both -- the site that was asked for, and the pair
// it was meant to replace, standing next to each other. The declaration is the
// scene, so it is written back as the scene.
function recordSoloSceneSites(ctx, sites) {
  if (!sites || sites.length === 0) return;
  const manifest = readJson(ctx.manifestPath);
  const loaded = loadedSite(sites);
  manifest.runtime = manifest.runtime || {};
  manifest.runtime.taskSites = sites.map((site) => ({
    alias: site.alias,
    size: site.size,
    position: site.position,
    ...(site.yawDeg ? { yawDeg: site.yawDeg } : {})
  }));
  if (loaded) {
    const grip = gripFromSite(loaded);
    // A robot served on its own stands at the origin of its own scene, so the
    // cube's declared position IS the point to command it to. Same rule as the
    // cell: the pick follows the support rather than being stated twice.
    const gripPosition = grip.objectPosition.map((value) => Number(Number(value).toFixed(6)));
    manifest.agentDefaults = { ...(manifest.agentDefaults || {}), gripPosition };
    writeGripTargetFact(ctx, gripPosition);
    manifest.runtime.gripper = {
      ...(manifest.runtime.gripper || {}),
      objectName: grip.objectName,
      supportName: grip.supportName,
      objectSize: grip.objectSize,
      objectPosition: grip.objectPosition,
      supportSize: grip.supportSize,
      supportPosition: grip.supportPosition,
      supportYawDeg: grip.yawDeg
    };
  } else {
    console.warn(
      `${ctx.manifest.name}: no declared site carries a payload, so ${manifest.runtime.gripper?.supportName} `
      + "and its cube are still what this robot builds when served on its own. "
      + "Give a site a payload alias (site@x,y,z,cubeName) to replace them."
    );
  }
  // The sites this robot has to reach around, and nothing that is no longer in
  // the scene: a stale alias is a silent no-op that reads like a guard.
  const walls = (manifest.runtime.collisionObstacles || []).filter((alias) => /wall/i.test(alias));
  manifest.runtime.collisionObstacles = uniqueItems([...walls, ...sites.map((site) => site.alias)]);
  manifest.pipeline = manifest.pipeline || {};
  manifest.pipeline.collisionObstacles = uniqueItems([
    ...((manifest.pipeline.collisionObstacles || []).filter((alias) => /wall/i.test(alias))),
    ...sites.map((site) => site.alias)
  ]);
  writeJson(ctx.manifestPath, manifest);
  console.log(`Recorded ${sites.length} scene site(s) in ${ctx.manifest.name}'s manifest: ${sites.map((site) => site.alias).join(", ")}.`);
}

function updateManifest(ctx, updates) {
  const current = readJson(ctx.manifestPath);
  const supportObstacle = current.runtime?.gripper?.supportName || `${current.name}_grip_support`;
  const pipelineObstacles = uniqueItems([...(current.pipeline?.collisionObstacles || ["80cmHighWall50cm"]), supportObstacle]);
  current.phase = Math.max(Number(current.phase || 1), 2);
  current.pipeline = {
    ...(current.pipeline || {}),
    robwotRoot: ctx.robwotRoot,
    urdfLoadScene: ctx.paths.urdfLoadScene,
    workspaceSceneTemplate: ctx.paths.workspaceSceneTemplate,
    workspaceDriver: ctx.paths.workspaceDriver,
    robotTemplate: ctx.paths.robotTemplate,
    collisionObstacles: pipelineObstacles,
    workspaceRandomSteps: Number(current.pipeline?.workspaceRandomSteps || 8000),
    ...updates.pipeline
  };
  delete current.pipeline.generateWorkspacePointClouds;
  current.runtime = {
    jointSteps: 60,
    cartesianSteps: 60,
    stepDelayMs: 25,
    cartesianMaxVelocity: 0.45,
    cartesianFinalTolerance: 0.025,
    enableHandoverGripperCalibration: false,
    jointMaxVelocityDeg: 60,
    workspaceTolerance: 0.08,
    enableCollisionCheck: true,
    enableJointCollisionPreflight: true,
    collisionClearance: 0.02,
    gripperCollisionClearance: 0.005,
    pickupGripperClearance: 0.001,
    collisionCheckStepDeg: 0.5,
    enablePathPlanning: true,
    planningMaxTimeSeconds: 4,
    planningStateCount: 120,
    plannedJointMaxVelocityDeg: 45,
    pathSmoothingPasses: 2,
    pathTrackingToleranceDeg: 0.5,
    pathSettleMaxSteps: 100,
    workspaceVisuals: true,
    ...(current.runtime || {}),
    collisionObstacles: uniqueItems([...(current.runtime?.collisionObstacles || current.pipeline.collisionObstacles || ["80cmHighWall50cm"]), supportObstacle])
  };
  current.generatedFiles = {
    ...(current.generatedFiles || {}),
    lowerTd: path.relative(ctx.repoRoot, ctx.paths.generatedLowerTd),
    model: path.relative(ctx.repoRoot, ctx.paths.generatedModel),
    workspaceScene: path.relative(ctx.repoRoot, ctx.paths.generatedWorkspaceScene),
    robotInfo: path.relative(ctx.repoRoot, ctx.paths.robotInfo),
    robotShape: path.relative(ctx.repoRoot, ctx.paths.robotShape),
    robotDataPoints: path.relative(ctx.repoRoot, ctx.paths.robotDataPoints)
  };
  current.nextPhase = "Generate a lower-level WoT runtime server from the TD with configurable ports and then connect the AgentThing end to end.";
  writeJson(ctx.manifestPath, current);
}

async function main() {
  const args = parseArgs(process.argv.slice(2));
  if (args.help) {
    console.log(usage());
    return;
  }
  if (args["update-shared-space-visual"]) {
    if (!args.manifests) {
      fail("Shared-space visualization requires --manifests with at least two comma-separated manifests.");
    }
    const manifestFiles = String(args.manifests).split(",").map((part) => part.trim()).filter(Boolean);
    if (manifestFiles.length < 2) {
      fail("Shared-space visualization requires at least two comma-separated manifests.");
    }
    if (args["update-shared-space-visual"]) {
      await updateSharedSpaceVisualization(manifestFiles);
    }
    process.exit(0);
  }
  if (args.manifests && args["generate-shared-cell"]) {
    const manifestFiles = String(args.manifests).split(",").map((part) => part.trim()).filter(Boolean);
    if (manifestFiles.length < 2) {
      fail("--generate-shared-cell requires at least two comma-separated manifests.");
    }
    await generateSharedCell(
      manifestFiles,
      args.offsets,
      args.rotations,
      args.orientations,
      args["interaction-pairs"],
      args["wall-offsets"],
      args.resample,
      { staticChain: args["static-chain"] === true, sites: args.sites, wallYaws: args["wall-yaws"] }
    );
    console.log("Shared-cell pipeline complete.");
    process.exit(0);
  }
  if (!args.manifest) {
    fail("Missing --manifest.");
  }

  const ctx = createContext(args.manifest);
  if (!validateContext(ctx)) {
    process.exit(1);
  }

  if (args["dry-run"]) {
    console.log("RobWoT bridge dry-run ok.");
    console.log(`Robot: ${ctx.manifest.name}`);
    console.log(`URDF: ${ctx.paths.urdf}`);
    console.log(`Generated model target: ${ctx.paths.generatedModel}`);
    console.log(`Generated lower TD target: ${ctx.paths.generatedLowerTd}`);
    return;
  }

  // A robot whose manifest names a source scene has its model taken from there
  // rather than imported from a URDF. Both produce the same artifact -- the
  // .ttm every later step loads -- so everything downstream is unchanged.
  const adopts = Boolean(ctx.manifest.pipeline?.sourceScene && ctx.manifest.pipeline?.sourceModel);
  const shouldAdoptModel = args["adopt-model"] || (adopts && args["full-pipeline"]);
  const shouldGenerateModel = !shouldAdoptModel && (args["generate-model"] || args["full-pipeline"]);
  const shouldGenerateTd = args["generate-td"] || args["full-pipeline"];

  if (!shouldAdoptModel && !shouldGenerateModel && !shouldGenerateTd) {
    fail("Choose --dry-run, --generate-model, --adopt-model, --generate-td, or --full-pipeline.");
  }

  if (shouldAdoptModel) {
    await adoptSceneModel(ctx);
  }

  if (shouldGenerateModel) {
    await generateModel(ctx);
  }

  if (shouldGenerateTd) {
    const declaredSites = parseCellSites(args.sites);
    await generateWorkspaceData(ctx, declaredSites);
    generateLowerTd(ctx);
    recordSoloSceneSites(ctx, declaredSites);
  }

  updateManifest(ctx, { pipeline: { lastRunAt: new Date().toISOString() } });
  console.log("RobWoT bridge pipeline complete.");
  process.exit(0);
}

if (require.main === module) {
  main().catch((error) => {
    console.error(error);
    process.exit(1);
  });
}

module.exports = {
  parseArgs,
  adoptSceneModel,
  cellSiteGraspInRobotFrame,
  recordCellPickPoint,
  createContext,
  validateContext,
  parseWallOffsets,
  parseResampleSelection,
  luaNumber,
  floorPlaneYaw,
  rotatePoint,
  generateModel,
  generateWorkspaceData,
  generateSharedCell,
  calculateRobotInSharedEnvironment,
  connect,
  gripSceneConfig,
  instrumentSamplingLoop,
  sharedSamplingObstacles,
  takeObstacleCullReport,
  updateSharedSpaceVisualization,
  generateLowerTd,
  prepareWorkspaceDriverScript,
  buildWorkspaceGripperPreamble,
  buildPipelineGripperLua
};
