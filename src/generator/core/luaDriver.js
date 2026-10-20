"use strict";

// The runtime driver, assembled from the Lua chunks in src/generator/lua.
//
// This was a ~2,900 line Lua program living inside one JavaScript template
// literal. Nothing could lint it, no editor could highlight it, and a mistyped
// substitution rendered the string "undefined" into the middle of the scene
// script, where it surfaced -- if at all -- as an unrelated runtime error much
// later. The chunks are now ordinary .lua files with `${name}` placeholders, and
// render() refuses to produce a script when a placeholder has no value or a
// supplied value is never used.

const fs = require("fs");
const path = require("path");

const CHUNK_DIRECTORY = path.resolve(__dirname, "..", "lua");
const PLACEHOLDER = /\$\{([A-Za-z_][A-Za-z0-9_]*)\}/g;

function luaString(value) {
  return `[[${String(value).replace(/\]\]/g, "]]..']]'..[[")}]]`;
}

function luaArray(values) {
  return `{${values.join(",")}}`;
}

function luaBoolean(value) {
  return value ? "true" : "false";
}

// Which joints the vendor driver treats as the arm.
//
// RobWoT's template takes the first six joints in tree order, on the stated
// assumption that every robot is a six-axis arm ("for common robot, assume all
// joints is controllable"). A five-axis arm with a gripper bolted to it breaks
// that assumption in the worst possible way: the sixth handle it picks up is
// the gripper's own finger joint, and from then on the IK solver is steering a
// jaw as if it were a wrist. It does not fail loudly -- it just never converges
// on any pose that constrains the tool's orientation.
//
// The arm is the joints BETWEEN the base and the tool flange, which the model
// already states: every one of them is an ancestor of the './tip' dummy, and
// the gripper's joints are descendants of it. Selecting on that is exact for
// any number of axes. The original cutoff stays as a fallback for a model with
// no tip dummy, which is the only case it was ever right about.
const ARM_CHAIN_JOINT_SELECTION = `
    local womatTipForChain = sim.getObject('./tip', {noError=true})
    newjointHandles = {}
    if womatTipForChain ~= -1 then
        local womatOnChain = {}
        local womatWalk = sim.getObjectParent(womatTipForChain)
        while womatWalk ~= -1 do
            womatOnChain[womatWalk] = true
            womatWalk = sim.getObjectParent(womatWalk)
        end
        for i=1,#jointHandles,1 do
            if womatOnChain[jointHandles[i]] then
                newjointHandles[#newjointHandles+1] = jointHandles[i]
            end
        end
    end
    if #newjointHandles == 0 then
        for i=1,#jointHandles,1 do
            newjointHandles[i] = jointHandles[i]
            if i==6 then
                break
            end
        end
    end
    jointHandles = newjointHandles`;

// Joints the model carries but the arm does not have.
//
// A URDF describes every joint in the mesh tree, including ones a real machine
// holds fixed through linkage the description cannot express. The Dobot's
// joint_end is one: on the machine a parallel linkage keeps the tool level, and
// nothing drives that axis independently. Drive it in simulation and the end
// link swings away from the wrist it is bolted to, leaving a visible gap -- an
// arm whose fifth axis is a modelling artefact.
//
// Identified by POSITION in the arm chain, not by name, because by the time the
// runtime sees a joint it has no name to match: configureImportedRobot aliases
// every joint in the model "joint" as it imports the URDF, so the scene has five
// objects called "joint" and nothing to tell them apart. The chain order is
// stable -- the vendor driver reads joints in tree order, base to tip, and the
// chain selection above preserves that order -- so the manifest's jointChain
// index is what survives.
//
// Locking one here, in the joint-selection block, is what makes it locked
// EVERYWHERE. Both the runtime driver and the workspace sampler are rendered
// through this function, so the sampled reachable cloud, the IK chain, the
// joint count reported back into the Thing Description and the home pose all
// come out of one statement about how many axes the arm has.
function lockedJointBlock(lockedJoints) {
  const locked = (lockedJoints || [])
    .map((entry) => (typeof entry === "number" ? { index: entry } : entry || {}))
    .map((entry) => ({ index: Number(entry.index), position: Number(entry.position || 0) }))
    .filter((entry) => Number.isInteger(entry.index) && entry.index >= 1);
  if (locked.length === 0) return "";
  return `
    local womatLockedJoints = {${locked.map((entry) => `[${entry.index}]=${entry.position}`).join(",")}}
    local womatMovableJoints = {}
    -- Kept, not just dropped. Removing a joint from jointHandles stops anything
    -- COMMANDING it; it does nothing about the solver, which builds its chain
    -- by walking the scene from base to tip and therefore includes every locked
    -- axis in between. womatFreezeLockedIkJoints needs the handles to tell it
    -- otherwise, and this is the last place they are all still in one list.
    womatLockedJointHandles = {}
    for i=1,#jointHandles,1 do
        local womatHold = womatLockedJoints[i]
        if womatHold ~= nil then
            -- Held by its own motor, and left dynamically enabled.
            --
            -- A locked joint is not a disconnected one. Taking it out of the
            -- dynamic chain -- passive mode, or a disabled motor -- detaches
            -- everything below it: the end link and the gripper bolted to it
            -- drop to the floor the moment the simulation starts. What a fixed
            -- axis actually is, physically, is a motor holding a position, so
            -- that is what it is set to. Only jointHandles loses it, which is
            -- what stops anything commanding or counting it.
            sim.setJointPosition(jointHandles[i], womatHold)
            pcall(sim.setJointTargetPosition, jointHandles[i], womatHold)
            pcall(sim.setObjectInt32Param, jointHandles[i], sim.jointintparam_motor_enabled, 1)
            womatLockedJointHandles[#womatLockedJointHandles+1] = jointHandles[i]
        else
            womatMovableJoints[#womatMovableJoints+1] = jointHandles[i]
        end
    end
    jointHandles = womatMovableJoints`;
}

// Which chain positions a manifest locks. The chain is every joint between base
// and flange in order; armJoints is that list minus the locked ones, so the
// index has to come from the chain and not from either subset.
function lockedChainPositions(mobility) {
  const chain = mobility?.jointChain || [];
  const held = mobility?.lockedJointPositions || {};
  return (mobility?.lockedJoints || [])
    .map((name) => ({ index: chain.indexOf(name) + 1, position: Number(held[name] || 0) }))
    .filter((entry) => entry.index >= 1);
}

// How far each chain joint is allowed to travel, addressed by chain position
// for the same reason lockedChainPositions is: configureImportedRobot aliases
// every arm joint "joint", so by the time the model is saved there is no name
// left to match on.
//
// This matters for a model the pipeline ADOPTS rather than imports. A URDF
// import carries its limits across itself, but an adopted model keeps whatever
// the source scene was built with -- and the Dobot's base joint arrives as
// [-240, +120] degrees. That is a full turn, so nothing is unreachable, but it
// is off-centre: an azimuth of +124 degrees has no representation below the
// +120 ceiling and can only be held at -236. womatUnwrapConfigToward then finds
// no shorter equivalent, correctly, and the arm swings three quarters of a
// circle to a point 80 degrees away. The URDF's own +/-129 degrees admits the
// short branch and excludes the long one.
function chainJointIntervals(mobility) {
  const chain = mobility?.jointChain || mobility?.armJoints || [];
  const limits = mobility?.armJointLimits || {};
  return chain
    .map((name, position) => {
      const entry = limits[name];
      if (!entry) return null;
      const low = Number(entry.lower);
      const high = Number(entry.upper);
      // A continuous joint is described with no limits or with a degenerate
      // pair; either way there is nothing to impose and the scene's own
      // interval is left alone.
      if (!Number.isFinite(low) || !Number.isFinite(high) || high - low <= 1e-9) {
        return null;
      }
      return { index: position + 1, name, low, high };
    })
    .filter((entry) => entry !== null);
}

// The solver's own copy of the arm, built by the vendor driver at init and used
// by the swept Cartesian preflight. It is the fifth IK environment in the
// runtime and the busiest -- it runs before every Cartesian move -- and like the
// four in src/generator/lua it was handed the whole chain including any locked
// axis. The preflight then restores jointHandles, so whatever it did to the
// locked joint stayed. Same line in both vendor files, so both are covered.
const VENDOR_IK_ELEMENT =
  "    local ikElement=simIK.addIkElementFromScene(ikEnv,ikGroup,simBase,simTip,simTarget,simIK.constraint_pose)";

function freezeVendorIkElement(baseLua, lockedJoints) {
  if ((lockedJoints || []).length === 0) return baseLua;
  if (!baseLua.includes(VENDOR_IK_ELEMENT)) {
    throw new Error(
      "RobWoT driver template no longer builds its IK element the expected way; "
      + "re-check src/generator/core/luaDriver.js freezeVendorIkElement against the vendor file."
    );
  }
  return baseLua.replace(
    VENDOR_IK_ELEMENT,
    "    local ikElement, ikSimToIk=simIK.addIkElementFromScene("
    + "ikEnv,ikGroup,simBase,simTip,simTarget,simIK.constraint_pose)\n"
    + "    womatFreezeLockedIkJoints(ikEnv, ikSimToIk)"
  );
}

const VENDOR_JOINT_CUTOFF = `    newjointHandles = {}
    for i=1,#jointHandles,1 do
        newjointHandles[i] = jointHandles[i]
        if i==6 then
            break
        end
    end
    jointHandles = newjointHandles`;

// Both the runtime driver and the workspace sampler are rendered from the same
// vendor file, so both inherited the cutoff -- which means a mis-selected arm
// corrupts the sampled reachable cloud as well as the live IK.
function selectArmChainJoints(baseLua, lockedJoints) {
  if (!baseLua.includes(VENDOR_JOINT_CUTOFF)) {
    throw new Error(
      "RobWoT driver template no longer contains the expected joint-selection block; "
      + "re-check src/generator/core/luaDriver.js selectArmChainJoints against the vendor file."
    );
  }
  return freezeVendorIkElement(
    baseLua.replace(
      VENDOR_JOINT_CUTOFF,
      ARM_CHAIN_JOINT_SELECTION + lockedJointBlock(lockedJoints)
    ),
    lockedJoints
  );
}

// Ordered by filename, which is why they are numbered: the driver is one script
// and later chunks call into earlier ones.
function chunkFiles() {
  return fs.readdirSync(CHUNK_DIRECTORY)
    .filter((name) => name.endsWith(".lua"))
    .sort();
}

function readChunks() {
  return chunkFiles().map((name) => ({
    name,
    source: fs.readFileSync(path.join(CHUNK_DIRECTORY, name), "utf8")
  }));
}

// The whole driver body, placeholders unresolved. Useful to tests that assert
// on the Lua itself without wanting a manifest to render it against.
function driverTemplate() {
  return readChunks().map((chunk) => chunk.source.replace(/\n$/, "")).join("\n");
}

function placeholderNames() {
  return new Set([...driverTemplate().matchAll(PLACEHOLDER)].map((match) => match[1]));
}

// A missing value used to render as "undefined" into the scene script; an
// unused one meant a rename had silently orphaned half a substitution. Both are
// reported here, by name, before the script can reach CoppeliaSim.
function render(values) {
  const chunks = readChunks();
  const missing = new Map();
  const used = new Set();
  const rendered = chunks.map((chunk) => chunk.source.replace(/\n$/, "").replace(PLACEHOLDER, (match, name) => {
    if (!(name in values) || values[name] === undefined) {
      if (!missing.has(name)) missing.set(name, chunk.name);
      return match;
    }
    used.add(name);
    return String(values[name]);
  })).join("\n");

  if (missing.size > 0) {
    const detail = [...missing].map(([name, file]) => `${name} (${file})`).join(", ");
    throw new Error(`Runtime driver is missing substitutions: ${detail}`);
  }
  const unused = Object.keys(values).filter((name) => !used.has(name));
  if (unused.length > 0) {
    throw new Error(`Runtime driver was given substitutions nothing uses: ${unused.join(", ")}`);
  }
  return rendered;
}

// Gripper mount angles, checked for the units they are actually written in.
//
// Every angle in a manifest is RADIANS: sim.setObjectOrientation and
// sim.buildMatrix take radians, and the generator's --gripper-*-deg flags do the
// conversion on the way in. Hand-editing the manifest bypasses that conversion,
// and the failure is silent rather than loud -- 110 written for "110 degrees" is
// 110 radians, which is 17 full turns plus 182.5 degrees, so the gripper lands
// almost-but-not-quite flipped and every downstream measurement is taken from
// that pose. Anything past a full turn is therefore rejected as degrees: no
// mount needs one, and reading it as radians can only be wrong.
function radiansTriple(values, label) {
  const angles = (values || [0, 0, 0]).map(Number);
  if (angles.length !== 3 || angles.some((angle) => !Number.isFinite(angle))) {
    throw new Error(`${label} must be three numbers, got ${JSON.stringify(values)}.`);
  }
  const degreesLike = angles.find((angle) => Math.abs(angle) > 2 * Math.PI);
  if (degreesLike !== undefined) {
    throw new Error(
      `${label} is in radians, but ${degreesLike} is more than a full turn -- `
      + `it looks like degrees. ${degreesLike} degrees is `
      + `${(degreesLike * Math.PI / 180).toFixed(6)} radians. Either write radians here or `
      + "regenerate with the --gripper-orientation-deg / --gripper-local-rotation-deg flags, "
      + "which convert for you."
    );
  }
  return angles;
}

// Where this robot's tool comes from, resolved in one place because the
// pipeline and the runtime both mount it and a disagreement between them puts
// the sampled workspace and the steered tool control point in different spots.
//
//   "model"   - a .ttm loaded from modelPath and bolted to the flange. The RG2
//               and the Baxter gripper: an arm that was imported from a URDF
//               with nothing on the end.
//   "builtin" - the arm's own model already carries the tool, as a subtree named
//               by sourceAlias. Nothing is loaded and NOTHING IS REPARENTED:
//               the tool is a link of the arm's kinematic chain, and moving it
//               onto the flange dummy takes it out of that chain.
//   "none"    - no tool at all. The tool control point is the flange.
//
// shipsWithModel is the older spelling of "builtin" and still resolves to it;
// the adopted Dobot was written that way.
const GRIPPER_SOURCES = ["model", "builtin", "none"];

function gripperSource(grip) {
  const declared = grip?.source;
  if (declared === undefined || declared === null || declared === "") {
    return grip?.shipsWithModel === true ? "builtin" : "model";
  }
  const source = String(declared);
  if (!GRIPPER_SOURCES.includes(source)) {
    throw new Error(
      `runtime.gripper.source must be one of ${GRIPPER_SOURCES.join(", ")}, got ${JSON.stringify(declared)}.`
    );
  }
  return source;
}

// The tool control point in the TOOL's own frame, for a tool that exposes no
// contact pads to measure one from. Measuring beats declaring wherever there is
// something to measure -- pads move when the jaws open, and a declared offset
// does not -- so this is consulted only when the pad scan finds nothing.
function toolControlPointOffset(grip) {
  const offset = grip?.toolControlPointOffset;
  if (offset === undefined || offset === null) return null;
  const values = offset.map(Number);
  if (values.length !== 3 || values.some((value) => !Number.isFinite(value))) {
    throw new Error(
      `runtime.gripper.toolControlPointOffset must be three numbers, got ${JSON.stringify(offset)}.`
    );
  }
  return values;
}

module.exports = {
  CHUNK_DIRECTORY,
  chainJointIntervals,
  lockedChainPositions,
  lockedJointBlock,
  selectArmChainJoints,
  chunkFiles,
  driverTemplate,
  luaArray,
  luaBoolean,
  luaString,
  GRIPPER_SOURCES,
  gripperSource,
  placeholderNames,
  radiansTriple,
  toolControlPointOffset,
  render
};
