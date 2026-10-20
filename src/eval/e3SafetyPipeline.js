#!/usr/bin/env node
// E3 -- what each stage of the safety pipeline contributes.
//
// Random Cartesian targets are commanded to one robot of the LIVE cell, each in
// five configurations of its core: the complete pipeline, and the pipeline with
// workspace admission, the preflight, the fallback planner or the in-motion
// watchdog switched off, one at a time.
//
// Targets are drawn in three kinds, so that every stage has something to do:
//
//   near       a reachable workspace sample, displaced by up to --jitter
//   obstacle   the same, from samples within --near-obstacle of an obstacle
//              box other than the floor (walls, neighbouring arms)
//   box        uniform in the box of the samples enlarged by --margin, above
//              the floor -- mostly out of reach, which is admission's case
//
// Drawn uniformly from the box alone, three quarters of Robot2's targets were
// unreachable or inside the floor or wall, and the fallback never had a
// reachable goal to plan to.
//
// The command runs through the same function the robot Thing's
// moveTocartesianPosition handler calls outside a handover session
// (simpleSmoothCartesianMove), in this process, against the served driver. No
// arbiter is involved -- the cell must be idle, so there is nobody to arbitrate
// with -- and nothing of the served core is changed: each switch is either an
// option of this process or a driver global that is restored after the command.
//
//   admission   the workspace grid is not passed          (as enableWorkspaceAdmission: false)
//   preflight   options.enableCollisionCheck=false for the swept line check,
//               and the dense joint-route sweep of the fallback answers ok
//   fallback    options.enablePathPlanning=false          (no goal search, no joint route, no OMPL)
//   watchdog    womatInMotionSafetyEnabled=false in the driver
//
// Whether the arm really touched something is decided by an observer that is
// independent of all four stages: a hook after the driver's sysCall_actuation
// runs the watchdog's own contact test (robot collection against the obstacle
// collection, plus self-collision) in every simulation step while a commanded
// motion is running -- in the posture the arm stands in and in the one the
// step commands, since a dynamic arm driven into an obstacle can be held off
// it by the contact response and press without overlapping (pressSteps, also
// counted in contactSteps). The watchdog tests the posture a step commands before the
// physics carries it out (womatCommandedPostureUnsafe), so a motion it stops
// never reaches the contact; a contact the observer sees is one the arm really
// made.
//
// Between commands the arm is put back into the same home posture, so every
// command starts from the same place and the five configurations of a target
// are paired. The order of the configurations rotates from target to target.
//
// Output: e3_commands.jsonl (one line per command, appended, so an interrupted
// run resumes), e3_commands.csv, e3_summary.csv (outcome counts per robot and
// configuration), e3_kind_summary.csv (the same per kind of target),
// e3_figure.tex (stacked bars for fig:exp_e3), e3_setup.json, and the drawn
// targets and home posture per robot (e3_targets_*.json, e3_home_*.json).

const fs = require("node:fs");
const path = require("node:path");

const { toCsv } = require("./e2CollisionCost");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const DEFAULT_OUT = "performance_log/e3";
const DEFAULT_ROBOTS = ["Robot2", "Robot4"];

const CONFIGS = ["full", "no_admission", "no_preflight", "no_fallback", "no_watchdog"];
const CONFIG_LABELS = {
  full: "Complete",
  no_admission: "No admission",
  no_preflight: "No preflight",
  no_fallback: "No fallback",
  no_watchdog: "No watchdog"
};
// Order of the stacked bars, best to worst.
const OUTCOMES = ["reached_direct", "reached_fallback", "rejected_admission", "rejected_preflight",
  "rejected_other", "aborted_watchdog", "not_reached", "collision"];
const OUTCOME_LABELS = {
  reached_direct: "Reached directly",
  reached_fallback: "Reached via fallback",
  rejected_admission: "Rejected by admission",
  rejected_preflight: "Rejected by preflight",
  rejected_other: "Rejected otherwise",
  aborted_watchdog: "Aborted by watchdog",
  not_reached: "Moved, target not reached",
  collision: "Collision"
};

// Installed into the robot's driver. Wraps a handful of driver globals so the
// command can be traced; every wrapper passes straight through unless a
// command of this experiment is armed, so the served core is unaffected.
const E3_LUA = String.raw`
function womatE3Brief(r)
    if type(r) ~= 'table' then return {ok=(r == true)} end
    return {ok=r.ok ~= false, reason=r.reason, obstacle=r.obstacle, obstacleOwner=r.obstacleOwner,
        mode=r.mode, blockedAtStep=r.blockedAtStep}
end

function womatE3Started(kind)
    if womatE3Armed and womatE3Trace.startKind == nil then
        womatE3Trace.startKind=kind
        womatE3Trace.startedAt=sim.getSystemTime()
    end
end

function womatE3Touching()
    local obstacles=womatObstacleCollection()
    if obstacles ~= -1 and womatCollectionsTouch(womatRobotCollisionCollection(), obstacles) then
        return 'obstacle'
    end
    if womatRobotSelfCollision() then return 'self' end
    return nil
end

-- The first contact of a command, named in the posture it happens in.
function womatE3Name(kind)
    local trace=womatE3Trace
    trace.contactKind=kind
    if kind == 'obstacle' then
        pcall(function() trace.contactObstacle, trace.contactOwner=womatBlockingObstacleAlias() end)
    else
        pcall(function() trace.contactObstacle, trace.contactOwner=womatSelfCollisionPair() end)
    end
end

-- Runs after the actuation step, before the physics carries it out: the
-- joints stand where the last step left them, their targets where this one
-- sends them. A dynamic arm driven into an obstacle can be held off it by the
-- contact response, so it presses without overlapping; testing only where it
-- stands would miss that. Pressing counts as contact.
function womatE3Observe()
    if not womatE3Armed or womatE3Trace.startKind == nil then return end
    local ok, touching, pressing=pcall(function()
        local touching=womatE3Touching()
        local pressing=nil
        if touching == nil then
            local commanded=womatCommandedConfig(jointHandles)
            local saved={}
            for i=1,#jointHandles,1 do saved[i]=sim.getJointPosition(jointHandles[i]) end
            for i=1,#jointHandles,1 do sim.setJointPosition(jointHandles[i], commanded[i]) end
            local tested, result=pcall(function()
                local found=womatE3Touching()
                if found ~= nil and womatE3Trace.contactKind == nil then womatE3Name(found) end
                return found
            end)
            for i=1,#jointHandles,1 do sim.setJointPosition(jointHandles[i], saved[i]) end
            if not tested then error(result) end
            pressing=result
        end
        return touching, pressing
    end)
    local trace=womatE3Trace
    trace.stepsObserved=trace.stepsObserved+1
    if not ok then
        trace.observerErrors=trace.observerErrors+1
        return
    end
    if pressing ~= nil then
        trace.pressSteps=trace.pressSteps+1
        touching=pressing
    end
    if touching ~= nil then
        trace.contactSteps=trace.contactSteps+1
        if trace.contactKind == nil then womatE3Name(touching) end
    end
end

function womatE3Install()
    if womatE3Orig == nil then
        womatE3Orig={start=womatStartSimpleCartesian, follow=womatFollowConfigs,
            sweep=womatSweepCartesianPath, jointPreflight=womatPreflightJointPath,
            plan=womatPlanJointPath, goal=womatFindGoalConfig}
        womatStartSimpleCartesian=function(...)
            womatE3Started('cartesian')
            return womatE3Orig.start(...)
        end
        womatFollowConfigs=function(...)
            womatE3Started('path')
            return womatE3Orig.follow(...)
        end
        womatSweepCartesianPath=function(...)
            local r=womatE3Orig.sweep(...)
            if womatE3Armed then womatE3Trace.linePreflight=womatE3Brief(r) end
            return r
        end
        womatPreflightJointPath=function(...)
            if womatE3Armed and womatE3PreflightOff then
                womatE3Trace.jointPreflightSkipped=womatE3Trace.jointPreflightSkipped+1
                return {ok=true}
            end
            local r=womatE3Orig.jointPreflight(...)
            if womatE3Armed then
                womatE3Trace.jointPreflightCalls=womatE3Trace.jointPreflightCalls+1
                if womatE3Trace.jointPreflight == nil then womatE3Trace.jointPreflight=womatE3Brief(r) end
            end
            return r
        end
        womatPlanJointPath=function(...)
            local r=womatE3Orig.plan(...)
            if womatE3Armed then womatE3Trace.planner=womatE3Brief(r) end
            return r
        end
        womatFindGoalConfig=function(...)
            local r=womatE3Orig.goal(...)
            if womatE3Armed then
                womatE3Trace.goalSearched=true
                womatE3Trace.goalFound=type(r) == 'table' and r.config ~= nil
            end
            return r
        end
    end
    if womatE3HookMode == nil then
        local register=registerScriptFuncHook
        if register == nil then pcall(function() register=sim.registerScriptFuncHook end) end
        if register ~= nil and pcall(register, 'sysCall_actuation', 'womatE3Observe', false) then
            womatE3HookMode='hook'
        else
            local original=sysCall_actuation
            sysCall_actuation=function(...)
                local result=original(...)
                womatE3Observe()
                return result
            end
            womatE3HookMode='wrapped'
        end
    end
    return {hook=womatE3HookMode, pickup=womatPickupApproachEnabled == true,
        handoverPose=womatHandoverPoseEnabled == true, config=womatCurrentConfig(),
        watchdog=womatInMotionSafetyEnabled == true}
end

function womatE3Arm(watchdog, preflight)
    womatE3WatchdogWas=womatInMotionSafetyEnabled
    womatInMotionSafetyEnabled=watchdog
    womatE3PreflightOff=not preflight
    womatE3Trace={armedAt=sim.getSystemTime(), stepsObserved=0, contactSteps=0, pressSteps=0, observerErrors=0,
        jointPreflightCalls=0, jointPreflightSkipped=0}
    womatE3Armed=true
    return true
end

function womatE3Disarm()
    womatE3Armed=false
    womatE3PreflightOff=false
    if womatE3WatchdogWas ~= nil then womatInMotionSafetyEnabled=womatE3WatchdogWas end
    womatE3WatchdogWas=nil
    return womatE3Trace
end

-- Stops whatever is still running and puts the arm back into the home posture.
function womatE3Reset(home, hard)
    corout_move=nil
    corout_moveJoint=nil
    corout_womatPath=nil
    rState=3
    womatMotionError=nil
    womatCartesianRouteJob=nil
    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], home[i])
        pcall(sim.setJointTargetPosition, jointHandles[i], home[i])
    end
    -- A dynamic arm jammed against an obstacle can be held there by the
    -- physics engine whatever the joints are set to; rebuilding the model in
    -- the engine releases it.
    if hard then
        pcall(sim.resetDynamicObject, womatRootOf(simBase) + sim.handleflag_model)
    end
    sim.setObjectPose(simTarget, sim.handle_world, sim.getObjectPose(simTip, sim.handle_world))
    return womatCurrentConfig()
end

-- Whether the arm is really back: the largest joint error to the home posture,
-- and any contact it stands in.
function womatE3HomeState(home)
    local config=womatCurrentConfig()
    local worst=0
    for i=1,#home,1 do
        worst=math.max(worst, math.abs(config[i]-home[i]))
    end
    local ok, touching=pcall(womatE3Touching)
    return {errorDeg=worst*180/math.pi, touching=ok and touching or nil}
end
`;

// ---------------------------------------------------------------- pure parts

// The same linear congruential generator as seededRandom in interactionSpace.js
// and the one E2 installs into the driver.
function seededRandom(seed) {
  let state = Math.floor(seed) % 4294967296;
  if (state <= 0) state = 1;
  return () => {
    state = (1664525 * state + 1013904223) % 4294967296;
    return state / 4294967296;
  };
}

const TARGET_KINDS = ["near", "obstacle", "box"];

function boxDistance(point, box) {
  let squared = 0;
  for (const axis of ["x", "y", "z"]) {
    const gap = Math.max(box.min[axis] - point[axis], 0, point[axis] - box.max[axis]);
    squared += gap * gap;
  }
  return Math.sqrt(squared);
}

const dot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;

function toVector(up) {
  return Array.isArray(up) ? { x: Number(up[0]), y: Number(up[1]), z: Number(up[2]) } : up;
}

// The heights of a box's lowest and highest corner along `up`.
function heightRange(box, up) {
  let low = Infinity;
  let high = -Infinity;
  for (const x of [box.min.x, box.max.x]) {
    for (const y of [box.min.y, box.max.y]) {
      for (const z of [box.min.z, box.max.z]) {
        const height = dot({ x, y, z }, up);
        low = Math.min(low, height);
        high = Math.max(high, height);
      }
    }
  }
  return [low, high];
}

// The floor by name, or anything shaped like one: thin along `up` and
// metres wide across it. The cell has a second, unnamed ground slab ("box").
function isFloorBox(box, up = { x: 0, y: 0, z: 1 }) {
  if (/floor/i.test(String(box.alias || ""))) return true;
  const [low, high] = heightRange(box, up);
  const extents = ["x", "y", "z"].map((axis) => box.max[axis] - box.min[axis]);
  const across = ["x", "y", "z"].filter((axis) => Math.abs(up[axis]) < 0.5)
    .map((axis) => box.max[axis] - box.min[axis]);
  return high - low <= 0.3 && across.length === 2 && across.every((extent) => extent >= 3)
    && Math.max(...extents) >= 3;
}

// The floor's top as a height along `up` in the robot-local frame: the floor
// obstacle boxes if the scene has them, else the base's own height above the
// world floor.
function floorHeight(obstacleBoxes, baseHeight = 0, up = { x: 0, y: 0, z: 1 }) {
  const floors = obstacleBoxes.filter((box) => isFloorBox(box, up));
  return floors.length ? Math.max(...floors.map((box) => heightRange(box, up)[1])) : -baseHeight;
}

// How many of each kind: half near the workspace, a quarter near obstacles, a
// quarter anywhere. Without samples near an obstacle, their share goes to near.
function kindCounts(count, haveObstacleSamples) {
  const obstacle = haveObstacleSamples ? Math.round(count / 4) : 0;
  const box = Math.round(count / 4);
  return { near: count - obstacle - box, obstacle, box };
}

// Targets in the robot-local frame the command takes. `points` are the
// reachable workspace samples, `obstacleBoxes` the robot-local obstacle boxes
// admission uses. The kinds are interleaved in a seeded order, so a run cut
// short still covers all three.
// Targets in the robot-local frame the command takes. `points` are the
// reachable workspace samples, `obstacleBoxes` the robot-local obstacle boxes
// admission uses, `up` the world vertical in that frame (a robot can be
// mounted on its side). The kinds are interleaved in a seeded order, so a run
// cut short still covers all three.
function sampleTargets({ points, bounds, obstacleBoxes = [], baseHeight = 0, up = [0, 0, 1] }, count, options, seed) {
  const { margin = 0.2, jitter = 0.05, nearObstacle = 0.15 } = options;
  const random = seededRandom(seed);
  const vertical = toVector(up);
  const floor = floorHeight(obstacleBoxes, baseHeight, vertical);
  const aboveFloor = points.filter((point) => dot(point, vertical) >= floor + jitter);
  if (aboveFloor.length === 0) throw new Error("No reachable sample lies above the floor.");
  const walls = obstacleBoxes.filter((box) => !isFloorBox(box, vertical));
  const nearWalls = aboveFloor.filter((point) => walls.some((box) => boxDistance(point, box) <= nearObstacle));
  const counts = kindCounts(count, nearWalls.length > 0);

  const kinds = TARGET_KINDS.flatMap((kind) => Array(counts[kind]).fill(kind));
  for (let index = kinds.length - 1; index > 0; index -= 1) {
    const other = Math.floor(random() * (index + 1));
    [kinds[index], kinds[other]] = [kinds[other], kinds[index]];
  }

  // Lifted back onto the floor if the displacement took it below.
  const onOrAbove = (point) => {
    const below = floor - dot(point, vertical);
    return below > 0 ? { x: point.x + below * vertical.x, y: point.y + below * vertical.y, z: point.z + below * vertical.z } : point;
  };
  const displaced = (pool) => {
    const sample = pool[Math.floor(random() * pool.length)];
    let offset;
    do {
      offset = [random() * 2 - 1, random() * 2 - 1, random() * 2 - 1];
    } while (Math.hypot(...offset) > 1);
    return onOrAbove({ x: sample.x + offset[0] * jitter, y: sample.y + offset[1] * jitter, z: sample.z + offset[2] * jitter });
  };
  // Uniform in the enlarged box, redrawn while below the floor. The box
  // reaches above the floor (the samples do), so this ends.
  const inBox = () => {
    for (;;) {
      const point = Object.fromEntries(["x", "y", "z"].map((axis) => {
        const low = bounds.min[axis] - margin;
        return [axis, low + random() * (bounds.max[axis] + margin - low)];
      }));
      if (dot(point, vertical) >= floor) return point;
    }
  };

  const targets = kinds.map((kind, index) => ({
    index: index + 1,
    kind,
    ...(kind === "near" ? displaced(aboveFloor) : kind === "obstacle" ? displaced(nearWalls) : inBox())
  }));
  return targets;
}

// The configurations of one target, rotated so that no configuration always
// runs first on a fresh home posture.
function configOrder(targetIndex) {
  const shift = (targetIndex - 1) % CONFIGS.length;
  return [...CONFIGS.slice(shift), ...CONFIGS.slice(0, shift)];
}

function stageSwitches(config) {
  return {
    admission: config !== "no_admission",
    preflight: config !== "no_preflight",
    fallback: config !== "no_fallback",
    watchdog: config !== "no_watchdog"
  };
}

function watchdogAborted(record) {
  return record.success !== true && record.reason === "motion_execution_failed"
    && /collision_detected/.test(String(record.plannerMessage || ""));
}

// One outcome per command. A contact the observer saw is a collision whatever
// the core reported: that is the outcome the pipeline exists to prevent.
function classifyOutcome(record) {
  if (record.contactSteps > 0) return "collision";
  if (watchdogAborted(record)) return "aborted_watchdog";
  if (record.success === true) return record.startKind === "path" ? "reached_fallback" : "reached_direct";
  if (record.startKind) return "not_reached";
  // Nothing moved. The straight line is always tried first, so the stage that
  // refused it is the stage that decided -- whatever the fallback then tried.
  if (record.admissionOk === false) return "rejected_admission";
  if (record.linePreflightOk === false) return "rejected_preflight";
  return "rejected_other";
}

function quantile(values, q) {
  if (values.length === 0) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const position = (sorted.length - 1) * q;
  const low = Math.floor(position);
  const high = Math.ceil(position);
  return sorted[low] + (sorted[high] - sorted[low]) * (position - low);
}

function summarize(records) {
  const row = { commands: records.length };
  for (const outcome of OUTCOMES) row[outcome] = records.filter((record) => record.outcome === outcome).length;
  const moved = records.filter((record) => record.startKind);
  const rejected = records.filter((record) => !record.startKind);
  row.moved = moved.length;
  row.contactSteps = records.reduce((sum, record) => sum + (record.contactSteps || 0), 0);
  row.decisionMsMedianMoved = quantile(moved.map((record) => record.decisionMs), 0.5);
  row.decisionMsMedianRejected = quantile(rejected.map((record) => record.decisionMs), 0.5);
  row.decisionMsP95 = quantile(records.map((record) => record.decisionMs), 0.95);
  row.totalSecondsMean = records.length
    ? records.reduce((sum, record) => sum + record.totalMs, 0) / records.length / 1000 : null;
  return row;
}

const SUMMARY_COLUMNS = ["commands", ...OUTCOMES, "moved", "contactSteps",
  "decisionMsMedianMoved", "decisionMsMedianRejected", "decisionMsP95", "totalSecondsMean"];

const RECORD_COLUMNS = ["robot", "target", "kind", "config", "x", "y", "z", "outcome", "success", "reason",
  "plannerMessage", "planner", "startKind", "decisionMs", "totalMs", "contactSteps", "pressSteps", "stepsObserved",
  "contactKind", "contactObstacle", "contactOwner", "admissionOk", "admissionReason",
  "linePreflightOk", "linePreflightReason", "linePreflightObstacle", "goalSearched", "goalFound",
  "jointPreflightOk", "jointPreflightReason", "jointPreflightSkipped", "plannerOk", "plannerReason",
  "finalError", "observerErrors"];

function summaryRows(records) {
  const rows = [];
  const robots = [...new Set(records.map((record) => record.robot))];
  for (const robot of [...robots, "all"]) {
    for (const config of CONFIGS) {
      const group = records.filter((record) => record.config === config && (robot === "all" || record.robot === robot));
      if (group.length) rows.push({ robot, config, ...summarize(group) });
    }
  }
  return rows;
}

// The same, per kind of target and pooled over the robots: which stage decides
// depends mostly on where the target lies.
function kindRows(records) {
  const rows = [];
  for (const kind of TARGET_KINDS) {
    for (const config of CONFIGS) {
      const group = records.filter((record) => record.kind === kind && record.config === config);
      if (group.length) rows.push({ kind, config, ...summarize(group) });
    }
  }
  return rows;
}

// pgfplots stacked horizontal bars, one bar per configuration, pooled over the
// robots, in percent of the commands.
function thesisFigure(rows) {
  const pooled = CONFIGS.map((config) => rows.find((row) => row.robot === "all" && row.config === config))
    .filter(Boolean);
  const labels = pooled.map((row) => CONFIG_LABELS[row.config]);
  const lines = [
    "% Generated by src/eval/e3SafetyPipeline.js -- body of fig:exp_e3.",
    "% Share of the commands per outcome, pooled over the robots; counts in e3_summary.csv.",
    "\\begin{tikzpicture}",
    "\\begin{axis}[",
    "    xbar stacked, bar width=10pt, width=\\linewidth, height=5.5cm,",
    "    xmin=0, xmax=100, xlabel={Share of commands [\\%]},",
    `    symbolic y coords={${[...labels].reverse().join(",")}}, ytick=data, y dir=normal,`,
    "    legend style={at={(0.5,-0.25)}, anchor=north, legend columns=2, font=\\footnotesize},",
    "    reverse legend=false]"
  ];
  for (const outcome of OUTCOMES) {
    const coordinates = pooled.map((row) => `(${(100 * row[outcome] / row.commands).toFixed(1)},${CONFIG_LABELS[row.config]})`);
    lines.push(`\\addplot coordinates {${coordinates.join(" ")}};`);
  }
  lines.push(`\\legend{${OUTCOMES.map((outcome) => OUTCOME_LABELS[outcome]).join(",")}}`);
  lines.push("\\end{axis}", "\\end{tikzpicture}");
  return lines.join("\n") + "\n";
}

function readRecords(file) {
  if (!fs.existsSync(file)) return [];
  return fs.readFileSync(file, "utf8").split("\n").filter(Boolean).map((line) => JSON.parse(line));
}

function recordKey(record) {
  return `${record.robot}|${record.target}|${record.config}`;
}

// ---------------------------------------------------------------- live run

function first(value) {
  return Array.isArray(value) ? value[0] : value;
}

async function scriptOf(sim, name) {
  const objectPath = `/${name}`;
  const handle = Number(first(await sim.getObject(objectPath)));
  const script = Number(first(await sim.getScript(await sim.scripttype_customizationscript, handle, objectPath)));
  if (!(script >= 0)) throw new Error(`${name} has no driver script -- is the cell served (npm run robot:serve)?`);
  return script;
}

function traceRecord(trace, result, admission) {
  const line = trace.linePreflight || null;
  const joint = trace.jointPreflight || null;
  const planner = trace.planner || null;
  const started = trace.startKind ? Number(trace.startedAt) : null;
  return {
    success: result?.success === true,
    reason: result?.reason || null,
    plannerMessage: result?.plannerMessage || null,
    planner: result?.planner || null,
    finalError: Number.isFinite(Number(result?.finalError)) ? Number(result.finalError) : null,
    startKind: trace.startKind || null,
    decisionFromArmMs: started !== null ? (started - Number(trace.armedAt)) * 1000 : null,
    contactSteps: Number(trace.contactSteps || 0),
    pressSteps: Number(trace.pressSteps || 0),
    stepsObserved: Number(trace.stepsObserved || 0),
    observerErrors: Number(trace.observerErrors || 0),
    contactKind: trace.contactKind || null,
    contactObstacle: trace.contactObstacle || null,
    contactOwner: trace.contactOwner || null,
    admissionOk: admission ? admission.ok !== false : null,
    admissionReason: admission && admission.ok === false ? admission.reason || null : null,
    linePreflightOk: line ? line.ok !== false : null,
    linePreflightReason: line && line.ok === false ? line.reason || null : null,
    linePreflightObstacle: line && line.ok === false ? line.obstacle || null : null,
    goalSearched: trace.goalSearched === true,
    goalFound: trace.goalSearched === true ? trace.goalFound === true : null,
    jointPreflightOk: joint ? joint.ok !== false : null,
    jointPreflightReason: joint && joint.ok === false ? joint.reason || null : null,
    jointPreflightSkipped: Number(trace.jointPreflightSkipped || 0),
    plannerOk: planner ? planner.ok !== false : null,
    plannerReason: planner && planner.ok === false ? planner.reason || null : null
  };
}

async function runRobot(sim, name, options, done, append) {
  const { delay } = require("../generator/core/context");
  const { createContext } = require("../generator/core/context");
  const { executeInScript } = require("../generator/core/simConnection");
  const { createRuntimeOptions, loadWorkspaceGrid } = require("../generator/core/runtimeOptions");
  const { luaArray } = require("../generator/core/luaDriver");
  const { simpleSmoothCartesianMove } = require("../generator/core/motion");
  const { WorkspaceGrid } = require("../generator/workspaceGrid");
  const { refreshObstacleBoxes } = require("../generator/core/safety");

  const ctx = createContext(path.join(REPO_ROOT, "generated", "robots", name, "manifest.json"));
  const base = createRuntimeOptions(ctx);
  if (!base.simpleSmoothMotion) throw new Error(`${name} does not use simpleSmoothMotion; E3 drives that path only.`);
  const workspace = loadWorkspaceGrid(ctx, { ...base, enableWorkspaceAdmission: true });
  if (!workspace) throw new Error(`${name}: could not load the workspace grid from ${ctx.paths.robotDataPoints}.`);
  let admission = null;
  workspace.admitsPath = function admitsPath(...args) {
    admission = WorkspaceGrid.prototype.admitsPath.apply(this, args);
    return admission;
  };

  const script = await scriptOf(sim, name);
  const installed = await executeInScript(sim, script, E3_LUA + "\nreturn womatE3Install()");
  if (installed.pickup || installed.handoverPose) {
    throw new Error(`${name} is in pickup or handover-pose mode; restart the cell so it is idle.`);
  }
  if (!installed.watchdog) throw new Error(`${name}: the in-motion watchdog is off in the served core.`);

  // The home posture is fixed on the first run and reused on resume, so a
  // resumed run starts every command from the same posture as the first.
  const homeFile = path.join(options.outDir, `e3_home_${name}.json`);
  const home = fs.existsSync(homeFile)
    ? JSON.parse(fs.readFileSync(homeFile, "utf8")).config
    : installed.config.map(Number);
  if (!fs.existsSync(homeFile)) fs.writeFileSync(homeFile, JSON.stringify({ robot: name, config: home }, null, 2) + "\n");
  // A command that ends with the arm jammed against an obstacle can leave it
  // there however the joints are reset, and every later command would then
  // start from that posture. So the reset is checked; a jam that a rebuild of
  // the arm in the physics engine does not release ends the run.
  const reset = async (after) => {
    let state;
    for (const hard of [false, true]) {
      await executeInScript(sim, script, `return womatE3Reset(${luaArray(home)}, ${hard})`);
      await delay(hard ? options.settleMs * 4 : options.settleMs);
      state = await executeInScript(sim, script, `return womatE3HomeState(${luaArray(home)})`);
      if (Number(state?.errorDeg) <= 1 && !state?.touching) return;
    }
    throw new Error(`${name} could not be returned to its home posture${after ? ` after ${after}` : ""} `
      + `(joint error ${Number(state?.errorDeg).toFixed(1)} deg${state?.touching ? `, touching (${state.touching})` : ""}). `
      + "Restart the cell (npm run robot:serve ...) and run E3 again: it resumes after the last recorded command.");
  };

  // Drawn once and kept, so a resumed run commands exactly the same targets.
  const targetsFile = path.join(options.outDir, `e3_targets_${name}.json`);
  let targets;
  if (fs.existsSync(targetsFile)) {
    targets = JSON.parse(fs.readFileSync(targetsFile, "utf8")).targets;
    if (targets.length !== options.targets) {
      throw new Error(`${targetsFile} holds ${targets.length} targets, not ${options.targets}; delete it to draw new ones.`);
    }
  } else {
    await refreshObstacleBoxes(sim, script, workspace);
    const points = [];
    for (const cell of workspace.cells.values()) {
      for (let offset = 0; offset < cell.length; offset += 3) {
        points.push({ x: cell[offset], y: cell[offset + 1], z: cell[offset + 2] });
      }
    }
    const index = Number(String(name).match(/(\d+)$/)?.[1] || 0);
    targets = sampleTargets({
      points,
      bounds: workspace.bounds,
      obstacleBoxes: workspace.obstacleBoxes,
      baseHeight: Number(ctx.manifest.sharedEnvironment?.offset?.[2] || 0),
      up: base.worldUpLocal
    }, options.targets, options, options.seed * 100 + index);
    fs.writeFileSync(targetsFile, JSON.stringify({ robot: name, seed: options.seed, margin: options.margin,
      jitter: options.jitter, nearObstacle: options.nearObstacle, targets }, null, 2) + "\n");
  }
  const tally = TARGET_KINDS.map((kind) => `${targets.filter((target) => target.kind === kind).length} ${kind}`).join(", ");
  console.log(`${name}: ${targets.length} targets (${tally}), observer via ${installed.hook}; `
    + `admission ${JSON.stringify(workspace.describe())}`);

  await reset();
  let executed = 0;
  for (const target of targets) {
    for (const config of configOrder(target.index)) {
      if (done.has(recordKey({ robot: name, target: target.index, config }))) continue;
      const on = stageSwitches(config);
      const motionOptions = { ...base, enableCollisionCheck: base.enableCollisionCheck && on.preflight,
        enablePathPlanning: base.enablePathPlanning && on.fallback };
      admission = null;
      const point = { x: target.x, y: target.y, z: target.z };
      let result;
      let trace;
      const started = Date.now();
      await executeInScript(sim, script, `return womatE3Arm(${on.watchdog}, ${on.preflight})`);
      try {
        result = await simpleSmoothCartesianMove(sim, script, point, on.admission ? workspace : null, motionOptions);
      } catch (error) {
        result = { success: false, reason: "e3_exception", plannerMessage: error?.message || String(error) };
      } finally {
        trace = await executeInScript(sim, script, "return womatE3Disarm()");
      }
      const totalMs = Date.now() - started;
      const traced = traceRecord(trace || {}, result, on.admission ? admission : null);
      const record = {
        robot: name, target: target.index, kind: target.kind, config, x: target.x, y: target.y, z: target.z,
        ...traced,
        decisionMs: traced.decisionFromArmMs ?? totalMs,
        totalMs
      };
      delete record.decisionFromArmMs;
      record.outcome = classifyOutcome(record);
      append(record);
      executed += 1;
      console.log(`  ${name} #${String(target.index).padStart(3)} ${target.kind.padEnd(8)} ${config.padEnd(12)} ${record.outcome.padEnd(18)} `
        + `${record.reason || ""}${record.contactObstacle ? ` [contact ${record.contactObstacle}/${record.contactOwner}]` : ""} `
        + `(${(totalMs / 1000).toFixed(1)} s)`);
      if (record.startKind && record.stepsObserved === 0) {
        throw new Error(`${name}: a motion ran but the contact observer saw no simulation step -- the hook is not active.`);
      }
      await reset(`target ${target.index} (${config})`);
    }
  }
  return executed;
}

async function run(options) {
  const { connect } = require("../generator/core/simConnection");
  const { sim } = await connect({ robwotRoot: path.join(REPO_ROOT, "external", "RobWoT") });
  if (Number(await sim.getSimulationState()) === Number(await sim.simulation_stopped)) {
    throw new Error("The simulation is stopped -- serve the cell first (npm run robot:serve ...).");
  }
  fs.mkdirSync(options.outDir, { recursive: true });
  const logFile = path.join(options.outDir, "e3_commands.jsonl");
  const done = new Set(readRecords(logFile).map(recordKey));
  if (done.size) console.log(`Resuming: ${done.size} command(s) already in ${path.relative(REPO_ROOT, logFile)}.`);
  const append = (record) => fs.appendFileSync(logFile, JSON.stringify(record) + "\n");
  for (const name of options.robots) {
    await runRobot(sim, name, options, done, append);
  }
}

function analyze(outDir, setup = null) {
  const records = readRecords(path.join(outDir, "e3_commands.jsonl"));
  if (!records.length) throw new Error(`No commands recorded in ${outDir}.`);
  // Re-classified here, so a change of the rule applies to recorded runs too.
  for (const record of records) record.outcome = classifyOutcome(record);
  fs.writeFileSync(path.join(outDir, "e3_commands.csv"), toCsv(records, RECORD_COLUMNS));
  const rows = summaryRows(records);
  fs.writeFileSync(path.join(outDir, "e3_kind_summary.csv"), toCsv(kindRows(records), ["kind", "config", ...SUMMARY_COLUMNS]));
  fs.writeFileSync(path.join(outDir, "e3_summary.csv"), toCsv(rows, ["robot", "config", ...SUMMARY_COLUMNS]));
  fs.writeFileSync(path.join(outDir, "e3_figure.tex"), thesisFigure(rows));
  if (setup) {
    fs.writeFileSync(path.join(outDir, "e3_setup.json"), JSON.stringify({ ...setup, finishedAt: new Date().toISOString() }, null, 2) + "\n");
  }
  return rows;
}

function printSummary(rows) {
  const short = { reached_direct: "direct", reached_fallback: "fallbk", rejected_admission: "rej-adm",
    rejected_preflight: "rej-pre", rejected_other: "rej-oth", aborted_watchdog: "watchdg",
    not_reached: "notrch", collision: "COLLIS" };
  console.log(`\n${"robot".padEnd(8)}${"config".padEnd(14)}${"n".padStart(5)}`
    + OUTCOMES.map((outcome) => short[outcome].padStart(9)).join("") + "  decision[ms] moved/rejected");
  for (const row of rows) {
    const ms = (value) => (value === null ? "-" : value.toFixed(0));
    console.log(`${row.robot.padEnd(8)}${row.config.padEnd(14)}${String(row.commands).padStart(5)}`
      + OUTCOMES.map((outcome) => String(row[outcome]).padStart(9)).join("")
      + `  ${ms(row.decisionMsMedianMoved)}/${ms(row.decisionMsMedianRejected)}`);
  }
}

// ---------------------------------------------------------------- CLI

function cliArguments(argv) {
  const args = { _: [] };
  for (let index = 0; index < argv.length; index += 1) {
    if (!argv[index].startsWith("--")) {
      args._.push(argv[index]);
      continue;
    }
    const next = argv[index + 1];
    if (next === undefined || next.startsWith("--")) {
      args[argv[index].slice(2)] = true;
    } else {
      args[argv[index].slice(2)] = next;
      index += 1;
    }
  }
  return args;
}

function positiveNumber(value, name, integer = true) {
  const number = Number(value);
  if (!Number.isFinite(number) || number <= 0 || (integer && !Number.isInteger(number))) {
    throw new Error(`--${name} must be a positive ${integer ? "integer" : "number"}, got ${value}.`);
  }
  return number;
}

async function main(argv) {
  const args = cliArguments(argv);
  const mode = args._[0] || "run";
  if (args.help || !["run", "analyze"].includes(mode)) {
    console.log([
      "Usage: node src/eval/e3SafetyPipeline.js [run] [--targets 60] [--seed 1] [--jitter 0.05]",
      "                                         [--near-obstacle 0.15] [--margin 0.2]",
      "                                         [--robots Robot2,Robot4] [--settle-ms 300] [--out performance_log/e3]",
      "       node src/eval/e3SafetyPipeline.js analyze [--out performance_log/e3]",
      "run needs the five-robot cell served and idle (no agents). It resumes from e3_commands.jsonl;",
      "delete that file (and e3_home_*.json, e3_targets_*.json) to start over."
    ].join("\n"));
    return mode === "run" || mode === "analyze" ? 0 : 1;
  }
  const outDir = path.resolve(REPO_ROOT, args.out || DEFAULT_OUT);
  if (mode === "analyze") {
    printSummary(analyze(outDir));
    return 0;
  }
  const options = {
    outDir,
    targets: positiveNumber(args.targets || 60, "targets"),
    margin: positiveNumber(args.margin || 0.2, "margin", false),
    jitter: positiveNumber(args.jitter || 0.05, "jitter", false),
    nearObstacle: positiveNumber(args["near-obstacle"] || 0.15, "near-obstacle", false),
    seed: positiveNumber(args.seed || 1, "seed"),
    settleMs: positiveNumber(args["settle-ms"] || 300, "settle-ms"),
    robots: args.robots ? String(args.robots).split(",").map((name) => name.trim()) : DEFAULT_ROBOTS
  };
  await run(options);
  printSummary(analyze(outDir, { ...options, outDir: path.relative(REPO_ROOT, outDir), configs: CONFIGS }));
  console.log(`Results: ${outDir}`);
  return 0;
}

if (require.main === module) {
  main(process.argv.slice(2)).then((code) => process.exit(code), (error) => {
    console.error(error);
    process.exit(1);
  });
}

module.exports = {
  CONFIGS,
  E3_LUA,
  OUTCOMES,
  TARGET_KINDS,
  classifyOutcome,
  configOrder,
  floorHeight,
  kindRows,
  sampleTargets,
  stageSwitches,
  summarize,
  summaryRows,
  thesisFigure
};
