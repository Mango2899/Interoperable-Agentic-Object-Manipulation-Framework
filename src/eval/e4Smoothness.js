#!/usr/bin/env node
// E4 -- how smooth are the executed motions?
//
// One robot of the LIVE, idle cell (Robot2 by default) runs two legs in three
// timing variants each, from the same home posture every time:
//
//   approach   a straight Cartesian line of the tool from the home posture to a
//              handover point of --pair (the shared voxel nearest the centroid
//              of the interaction space that the arm can reach along a line)
//   home       the joint route from where the approach ended back to home
//
//   double_s   the core's own seven-phase double-S (core/timeLaw.js), played
//              one sample per simulation step through womatFollowConfigs --
//              exactly how the reserved routes into and out of a handover run
//   linear     the same geometry and executor, at constant velocity v_max from
//              the first step to the last (the "plain linear interpolation")
//   ruckig     the ordinary motion functions of CoppeliaSim (sim.moveToPose /
//              sim.moveToConfig, i.e. Ruckig) under the same three limits
//
// A hook after the driver's sysCall_actuation records, in every simulation
// step, the joint positions the arm stands in (measured) and the ones the step
// commands (commanded), and the tool position for both -- the commanded tool
// by forward kinematics, placing the joints and restoring them within the call
// as the watchdog does. Velocity, acceleration and jerk are backward
// differences over the simulation time step. For the reserved variants the
// time law itself (nominal) is kept too, so three levels can be told apart:
// the profile, what the IK makes of it, and what the dynamic arm does.
//
// Limits compared against: the joint limits for the home leg, the Cartesian
// limits (tool speed along the path) for the approach leg -- a Cartesian move
// does not bound joint rates (thesis, Trajectory Timing).
//
// Output (performance_log/e4): e4_runs.jsonl (one line per leg, variant and
// repetition), e4_summary.csv, e4_table.tex, traces/<leg>_<variant>_r<k>.csv
// (every joint, every step), lead/<leg>_<variant>.csv (the plotted signal of
// the first repetition, in degrees or metres), e4_figure_home.tex,
// e4_figure_approach.tex, e4_setup.json, e4_target.json, e4_home.json.

const fs = require("node:fs");
const path = require("node:path");

const { toCsv } = require("./e2CollisionCost");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const DEFAULT_OUT = "performance_log/e4";
const LEGS = ["approach", "home"];
const VARIANTS = ["double_s", "linear", "ruckig"];
const VARIANT_LABELS = { double_s: "Double-S", linear: "Linear", ruckig: "Ruckig" };
const DEG = 180 / Math.PI;

const E4_LUA = String.raw`
function womatE4ToolAt(config)
    local saved={}
    for i=1,#jointHandles,1 do saved[i]=sim.getJointPosition(jointHandles[i]) end
    for i=1,#jointHandles,1 do sim.setJointPosition(jointHandles[i], config[i]) end
    local p=getCartesianposition()
    for i=1,#jointHandles,1 do sim.setJointPosition(jointHandles[i], saved[i]) end
    return p
end

-- After the actuation step, before the physics carries it out: the joints
-- stand where the last step left them, their targets where this one sends them.
function womatE4Record()
    if not womatE4Armed then return end
    local ok, err=pcall(function()
        local tr=womatE4Trace
        local measured=womatCurrentConfig()
        local commanded=womatCommandedConfig(jointHandles)
        tr.t[#tr.t+1]=sim.getSimulationTime()
        for i=1,#measured,1 do tr.qm[#tr.qm+1]=measured[i] end
        for i=1,#commanded,1 do tr.qc[#tr.qc+1]=commanded[i] end
        local pm=getCartesianposition()
        local pc=womatE4ToolAt(commanded)
        tr.pm[#tr.pm+1]=pm.x; tr.pm[#tr.pm+1]=pm.y; tr.pm[#tr.pm+1]=pm.z
        tr.pc[#tr.pc+1]=pc.x; tr.pc[#tr.pc+1]=pc.y; tr.pc[#tr.pc+1]=pc.z
    end)
    if not ok then womatE4Trace.errors=womatE4Trace.errors+1; womatE4Trace.lastError=tostring(err) end
end

function womatE4Install()
    if womatE4HookMode == nil then
        local register=registerScriptFuncHook
        if register == nil then pcall(function() register=sim.registerScriptFuncHook end) end
        if register ~= nil and pcall(register, 'sysCall_actuation', 'womatE4Record', false) then
            womatE4HookMode='hook'
        else
            local original=sysCall_actuation
            sysCall_actuation=function(...)
                local result=original(...)
                womatE4Record()
                return result
            end
            womatE4HookMode='wrapped'
        end
    end
    return {hook=womatE4HookMode, config=womatCurrentConfig(), dof=#jointHandles,
        dt=sim.getSimulationTimeStep(), tool=getCartesianposition(),
        pickup=womatPickupApproachEnabled == true, handoverPose=womatHandoverPoseEnabled == true,
        watchdog=womatInMotionSafetyEnabled == true}
end

function womatE4Arm()
    womatE4Trace={t={}, qm={}, qc={}, pm={}, pc={}, errors=0}
    womatE4Armed=true
    return true
end

function womatE4Disarm()
    womatE4Armed=false
    return womatE4Trace
end

function womatE4ToLocal(points)
    local out={}
    for i=1,#points,3 do
        local p=womatWorldToLocal(points[i], points[i+1], points[i+2])
        out[#out+1]=p[1]; out[#out+1]=p[2]; out[#out+1]=p[3]
    end
    return out
end

function womatE4Reset(home, hard)
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
    if hard then
        pcall(sim.resetDynamicObject, womatRootOf(simBase) + sim.handleflag_model)
    end
    sim.setObjectPose(simTarget, sim.handle_world, sim.getObjectPose(simTip, sim.handle_world))
    local config=womatCurrentConfig()
    local worst=0
    for i=1,#home,1 do worst=math.max(worst, math.abs(config[i]-home[i])) end
    return {errorDeg=worst*180/math.pi}
end
`;

// ---------------------------------------------------------------- pure parts

// Constant velocity from the first step to the last: the fraction grows by
// v*dt/D per step and the last sample lands on the target.
function linearSchedule(distance, velocity, dt) {
  const span = Math.abs(Number(distance) || 0);
  if (!(span > 0)) return [0, 1];
  const steps = Math.max(1, Math.ceil(span / (velocity * dt)));
  const fractions = [];
  for (let index = 0; index <= steps; index += 1) fractions.push(Math.min(1, index * velocity * dt / span));
  fractions[fractions.length - 1] = 1;
  return fractions;
}

function linearJointPath(start, target, velocity, dt) {
  const distance = Math.max(...target.map((value, i) => Math.abs(value - start[i])));
  return linearSchedule(distance, velocity, dt).map((fraction) => start.map((value, j) => value + (target[j] - value) * fraction));
}

// Backward differences of a uniformly sampled signal: rows of equal length,
// one per step. Entry k of the derivative belongs to sample k; the first
// entries, which have no predecessor, are zero.
function differentiate(rows, dt) {
  return rows.map((row, k) => (k === 0 ? row.map(() => 0) : row.map((value, i) => (value - rows[k - 1][i]) / dt)));
}

function derivatives(rows, dt) {
  const velocity = differentiate(rows, dt);
  const acceleration = differentiate(velocity, dt);
  const jerk = differentiate(acceleration, dt);
  return { velocity, acceleration, jerk };
}

const norm = (row) => Math.hypot(...row);

// Scalar path signals of the tool: distance travelled along the path and the
// magnitude of its derivatives.
function toolSignals(points, dt) {
  const { velocity, acceleration, jerk } = derivatives(points, dt);
  let travelled = 0;
  const s = points.map((point, k) => {
    if (k > 0) travelled += norm(point.map((value, i) => value - points[k - 1][i]));
    return travelled;
  });
  return { s, velocity: velocity.map(norm), acceleration: acceleration.map(norm), jerk: jerk.map(norm) };
}

function peakAbs(rows) {
  let peak = 0;
  for (const row of rows) for (const value of (Array.isArray(row) ? row : [row])) peak = Math.max(peak, Math.abs(value));
  return peak;
}

// The joint that travels furthest in the reference run: the one the time law
// is sized by, and the one plotted.
function leadingJoint(rows) {
  const first = rows[0];
  const last = rows[rows.length - 1];
  let lead = 0;
  for (let i = 1; i < first.length; i += 1) {
    if (Math.abs(last[i] - first[i]) > Math.abs(last[lead] - first[lead])) lead = i;
  }
  return lead;
}

// First and last step in which the commanded signal changes: the motion as
// commanded, padding before and after excluded from the duration.
function activeWindow(rows, epsilon = 1e-7) {
  let first = -1;
  let last = -1;
  for (let k = 1; k < rows.length; k += 1) {
    if (rows[k].some((value, i) => Math.abs(value - rows[k - 1][i]) > epsilon)) {
      if (first < 0) first = k;
      last = k;
    }
  }
  return first < 0 ? null : { first, last };
}

// When the measured posture has come within `tolerance` of its final commanded
// one and stays there.
function settledIndex(measured, finalCommanded, tolerance) {
  for (let k = measured.length - 1; k >= 0; k -= 1) {
    if (measured[k].some((value, i) => Math.abs(value - finalCommanded[i]) > tolerance)) {
      return k + 1 < measured.length ? k + 1 : null;
    }
  }
  return 0;
}

function reshape(flat, width) {
  const rows = [];
  for (let offset = 0; offset + width <= flat.length; offset += width) rows.push(flat.slice(offset, offset + width).map(Number));
  return rows;
}

// Voxel centres of a pair's interaction space, nearest the centroid first.
function handoverCandidates(csvText) {
  const lines = csvText.split("\n").filter(Boolean);
  const header = lines[0].split(",");
  const [ix, iy, iz] = ["X", "Y", "Z"].map((name) => header.indexOf(name));
  const points = lines.slice(1).map((line) => {
    const cells = line.split(",").map(Number);
    return { x: cells[ix], y: cells[iy], z: cells[iz] };
  }).filter((point) => [point.x, point.y, point.z].every(Number.isFinite));
  const centroid = Object.fromEntries(["x", "y", "z"].map((axis) =>
    [axis, points.reduce((sum, point) => sum + point[axis], 0) / points.length]));
  const distance = (point) => Math.hypot(point.x - centroid.x, point.y - centroid.y, point.z - centroid.z);
  return points.map((point) => ({ ...point, centroidDistance: distance(point) }))
    .sort((a, b) => a.centroidDistance - b.centroidDistance);
}

// Metrics of one recorded run. `limits` are {velocity, acceleration, jerk} in
// the leg's own units (rad or m); peaks are reported in degrees for joints.
function analyzeTrace(trace, { dof, dt, leg, limits, lead, nominal = null, tolerance, startupS = 0.5 }) {
  const commanded = reshape(trace.qc, dof);
  const measured = reshape(trace.qm, dof);
  const toolCommanded = reshape(trace.pc, 3);
  const toolMeasured = reshape(trace.pm, 3);
  const window = activeWindow(commanded);
  const record = { steps: commanded.length, recorderErrors: Number(trace.errors || 0) };
  if (!window) return { ...record, moved: false };
  const settled = settledIndex(measured, commanded[commanded.length - 1], tolerance);
  record.moved = true;
  record.durationS = (window.last - window.first + 1) * dt;
  record.settledS = settled === null ? null : Math.max(0, settled - window.first + 1) * dt;
  const joints = { commanded: derivatives(commanded, dt), measured: derivatives(measured, dt) };
  for (const level of ["commanded", "measured"]) {
    record[`${level}JointVelDeg`] = peakAbs(joints[level].velocity) * DEG;
    record[`${level}JointAccDeg`] = peakAbs(joints[level].acceleration) * DEG;
    record[`${level}JointJerkDeg`] = peakAbs(joints[level].jerk) * DEG;
  }
  const tools = { commanded: toolSignals(toolCommanded, dt), measured: toolSignals(toolMeasured, dt) };
  for (const level of ["commanded", "measured"]) {
    record[`${level}ToolVel`] = peakAbs(tools[level].velocity);
    record[`${level}ToolAcc`] = peakAbs(tools[level].acceleration);
    record[`${level}ToolJerk`] = peakAbs(tools[level].jerk);
  }
  if (nominal) {
    const nominalRows = nominal.map((value) => [value]);
    const law = derivatives(nominalRows, dt);
    const unit = leg === "home" ? DEG : 1;
    record.nominalVel = peakAbs(law.velocity) * unit;
    record.nominalAcc = peakAbs(law.acceleration) * unit;
    record.nominalJerk = peakAbs(law.jerk) * unit;
  }
  // The quantity the leg's limits bound, as a multiple of the limit.
  const [vel, acc, jerk] = leg === "home"
    ? ["JointVelDeg", "JointAccDeg", "JointJerkDeg"].map((key) => (level) => record[`${level}${key}`] / DEG)
    : ["ToolVel", "ToolAcc", "ToolJerk"].map((key) => (level) => record[`${level}${key}`]);
  for (const level of ["commanded", "measured"]) {
    record[`${level}VelRatio`] = vel(level) / limits.velocity;
    record[`${level}AccRatio`] = acc(level) / limits.acceleration;
    record[`${level}JerkRatio`] = jerk(level) / limits.jerk;
  }
  // The same peaks once the motion is under way. Both start-up effects of the
  // cell fall into the first steps: a route begins at the posture the arm
  // stands in, not at the target it was last sent to, so its first sample
  // closes the tracking offset in one step; and from the stretched home
  // posture the IK only follows the line after a few steps.
  const late = window.first + Math.round(startupS / dt);
  const after = (rows) => rows.slice(late);
  for (const level of ["commanded", "measured"]) {
    const signal = leg === "home"
      ? { velocity: joints[level].velocity, acceleration: joints[level].acceleration, jerk: joints[level].jerk, unit: DEG }
      : { ...tools[level], unit: 1 };
    record[`${level}LateVel`] = peakAbs(after(signal.velocity)) * signal.unit;
    record[`${level}LateAcc`] = peakAbs(after(signal.acceleration)) * signal.unit;
    record[`${level}LateJerk`] = peakAbs(after(signal.jerk)) * signal.unit;
  }
  record.finalToolError = norm(toolMeasured[toolMeasured.length - 1]
    .map((value, i) => value - toolCommanded[toolCommanded.length - 1][i]));

  // The plotted signal, relative to the start of the commanded motion.
  const from = Math.max(0, window.first - 3);
  const series = [];
  const jointLevel = (level, k) => ({
    pos: (level === "commanded" ? commanded : measured)[k][lead] * DEG,
    vel: joints[level].velocity[k][lead] * DEG,
    acc: joints[level].acceleration[k][lead] * DEG,
    jerk: joints[level].jerk[k][lead] * DEG
  });
  const toolLevel = (level, k) => ({
    pos: tools[level].s[k],
    vel: tools[level].velocity[k],
    acc: tools[level].acceleration[k],
    jerk: tools[level].jerk[k]
  });
  const level = leg === "home" ? jointLevel : toolLevel;
  for (let k = from; k < commanded.length; k += 1) {
    const c = level("commanded", k);
    const m = level("measured", k);
    series.push({ t: (k - window.first) * dt, pos: c.pos, vel: c.vel, acc: c.acc, jerk: c.jerk,
      posMeas: m.pos, velMeas: m.vel, accMeas: m.acc, jerkMeas: m.jerk });
  }
  return { ...record, series };
}

const SUMMARY_METRICS = ["durationS", "settledS",
  "nominalVel", "nominalAcc", "nominalJerk",
  "commandedJointVelDeg", "commandedJointAccDeg", "commandedJointJerkDeg",
  "measuredJointVelDeg", "measuredJointAccDeg", "measuredJointJerkDeg",
  "commandedToolVel", "commandedToolAcc", "commandedToolJerk",
  "measuredToolVel", "measuredToolAcc", "measuredToolJerk",
  "commandedVelRatio", "commandedAccRatio", "commandedJerkRatio",
  "measuredVelRatio", "measuredAccRatio", "measuredJerkRatio",
  "commandedLateVel", "commandedLateAcc", "commandedLateJerk",
  "measuredLateVel", "measuredLateAcc", "measuredLateJerk", "finalToolError"];

function mean(values) {
  const finite = values.filter(Number.isFinite);
  return finite.length ? finite.reduce((sum, value) => sum + value, 0) / finite.length : null;
}

function summaryRows(records) {
  const rows = [];
  for (const leg of LEGS) {
    for (const variant of VARIANTS) {
      const group = records.filter((record) => record.leg === leg && record.variant === variant);
      if (!group.length) continue;
      const row = { leg, variant, runs: group.length, reached: group.filter((record) => record.success).length };
      for (const metric of SUMMARY_METRICS) row[metric] = mean(group.map((record) => record[metric]));
      rows.push(row);
    }
  }
  return rows;
}

function fixed(value, digits) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

// Duration and commanded peaks per leg and variant against the limits of the
// leg: over the whole motion, and once it is under way (after --startup).
function thesisTable(rows, limits, startupS = 0.5) {
  const lines = [
    "% Generated by src/eval/e4Smoothness.js -- body of tab:exp_e4. Commanded signal, means over the repetitions.",
    "% Acceleration and jerk: whole motion / after the first " + startupS + " s. Approach: tool along the path",
    "% [m/s, m/s^2, m/s^3]; return home: largest joint rate [deg/s, deg/s^2, deg/s^3]. Measured values in e4_summary.csv.",
    "\\begin{tabular}{@{}llrrrr@{}}",
    "\\toprule",
    "\\textbf{Leg} & \\textbf{Time law} & \\textbf{Duration [s]} & \\textbf{Velocity} & \\textbf{Acceleration} & \\textbf{Jerk} \\\\",
    "\\midrule"
  ];
  for (const leg of LEGS) {
    const legRows = rows.filter((row) => row.leg === leg);
    if (!legRows.length) continue;
    const joint = leg === "home";
    const whole = joint ? ["JointVelDeg", "JointAccDeg", "JointJerkDeg"] : ["ToolVel", "ToolAcc", "ToolJerk"];
    const digits = joint ? [1, 1, 0] : [3, 2, 1];
    const limit = joint ? limits.joint : limits.cartesian;
    for (const row of legRows) {
      const pair = (key, lateKey, d) => `${fixed(row[`commanded${key}`], d)} / ${fixed(row[`commanded${lateKey}`], d)}`;
      lines.push(`${joint ? "Return home" : "Approach"} & ${VARIANT_LABELS[row.variant]} & ${fixed(row.durationS, 2)} & `
        + `${fixed(row[`commanded${whole[0]}`], digits[0])} & ${pair(whole[1], "LateAcc", digits[1])} & ${pair(whole[2], "LateJerk", digits[2])} \\\\`);
    }
    lines.push(`& \\emph{Limit} & & ${fixed(limit.velocity, digits[0])} & ${fixed(limit.acceleration, digits[1])} & ${fixed(limit.jerk, digits[2])} \\\\`);
    if (leg !== LEGS[LEGS.length - 1]) lines.push("\\midrule");
  }
  lines.push("\\bottomrule", "\\end{tabular}");
  return lines.join("\n") + "\n";
}

// Four stacked plots over time of one leg, one line per variant (commanded),
// the limits dashed. Reads lead/<leg>_<variant>.csv relative to \EFourData.
function thesisFigure(leg, limits, variants, xmax = 5) {
  const joint = leg === "home";
  const limit = joint ? limits.joint : limits.cartesian;
  const labels = joint
    ? ["$q$ [$^\\circ$]", "$\\dot q$ [$^\\circ$/s]", "$\\ddot q$ [$^\\circ$/s$^2$]", "$\\dddot q$ [$^\\circ$/s$^3$]"]
    : ["$s$ [m]", "$\\dot s$ [m/s]", "$\\ddot s$ [m/s$^2$]", "$\\dddot s$ [m/s$^3$]"];
  const columns = ["pos", "vel", "acc", "jerk"];
  // Rounded, so the limits read 30 and 0.25 rather than 29.999999999999996.
  const bounds = [null, limit.velocity, limit.acceleration, limit.jerk].map((value) => (value === null ? null : Number(value.toPrecision(6))));
  const styles = { double_s: "blue!70!black, thick", linear: "red!70!black", ruckig: "green!50!black, densely dotted, thick" };
  const lines = [
    `% Generated by src/eval/e4Smoothness.js -- body of fig:exp_e4 (${leg} leg, commanded signal).`,
    "% Needs \\usepackage{pgfplots}. Set \\EFourData to the directory holding lead/ before \\input-ing this file.",
    "% The linear law's acceleration and jerk are impulses of height v/dt and v/dt^2; the y axes are clipped",
    "% at four times the limit so the jerk-limited laws stay readable -- the peaks are in e4_table.tex.",
    "\\providecommand{\\EFourData}{e4}",
    "\\pgfplotstableset{col sep=comma}",
    "\\begin{tikzpicture}"
  ];
  columns.forEach((column, index) => {
    const name = `e4${leg}${column}`;
    const placement = index === 0 ? `name=${name}` : `name=${name}, at={(e4${leg}${columns[index - 1]}.below south west)}, anchor=north west, yshift=-2mm`;
    // Clipped at the axis, not dropped: the linear law's spikes leave the plot.
    const clip = bounds[index] ? `, ymin=${-4 * bounds[index]}, ymax=${4 * bounds[index]}, clip=true` : "";
    lines.push("\\begin{axis}[",
      `    ${placement}, width=0.92\\textwidth, height=3.6cm,`,
      `    ylabel={${labels[index]}}, ${index === columns.length - 1 ? "xlabel={$t$ [s]}" : "xticklabels={}"},`,
      `    grid=major, grid style={black!12}, tick label style={font=\\scriptsize}, label style={font=\\small},`,
      `    ${index === 0 ? "legend columns=-1, legend style={at={(0.5,1.05)}, anchor=south, draw=none, font=\\footnotesize}," : ""}`,
      `    xmin=-0.15, xmax=${xmax.toFixed(2)}, unbounded coords=jump${clip}]`);
    for (const variant of variants) {
      lines.push(`  \\addplot[${styles[variant]}, const plot mark right] table[x=t, y=${column}] {\\EFourData/lead/${leg}_${variant}.csv};`);
      if (index === 0) lines.push(`  \\addlegendentry{${VARIANT_LABELS[variant]}}`);
    }
    if (bounds[index]) {
      lines.push(`  \\addplot[black!60, dashed, forget plot, domain=-0.15:${xmax.toFixed(2)}, samples=2] {${bounds[index]}};`,
        `  \\addplot[black!60, dashed, forget plot, domain=-0.15:${xmax.toFixed(2)}, samples=2] {${-bounds[index]}};`);
    }
    lines.push("\\end{axis}");
  });
  lines.push("\\end{tikzpicture}");
  return lines.join("\n") + "\n";
}

// ---------------------------------------------------------------- live run

function readRecords(file) {
  if (!fs.existsSync(file)) return [];
  return fs.readFileSync(file, "utf8").split("\n").filter(Boolean).map((line) => JSON.parse(line));
}

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

async function runLive(options) {
  const { connect } = require("../generator/core/simConnection");
  const { executeInScript } = require("../generator/core/simConnection");
  const { createContext, delay } = require("../generator/core/context");
  const { createRuntimeOptions } = require("../generator/core/runtimeOptions");
  const { luaArray } = require("../generator/core/luaDriver");
  const { asArray } = require("../generator/workspaceGrid");
  const { motionSchedule } = require("../generator/core/timeLaw");
  const { homeTrajectory } = require("../generator/core/continuousHome");
  const { waitForMotionComplete, motionExecutionError } = require("../generator/core/safety");

  const { sim } = await connect({ robwotRoot: path.join(REPO_ROOT, "external", "RobWoT") });
  if (Number(await sim.getSimulationState()) === Number(await sim.simulation_stopped)) {
    throw new Error("The simulation is stopped -- serve the cell first (npm run robot:serve ...).");
  }
  const name = options.robot;
  const ctx = createContext(path.join(REPO_ROOT, "generated", "robots", name, "manifest.json"));
  const base = createRuntimeOptions(ctx);
  const rad = Math.PI / 180;
  const limits = {
    joint: { velocity: base.simpleJointVelocityDeg * rad, acceleration: base.simpleJointAccelerationDeg * rad, jerk: base.simpleJointJerkDeg * rad },
    cartesian: { velocity: base.simpleCartesianVelocity, acceleration: base.simpleCartesianAcceleration, jerk: base.simpleCartesianJerk }
  };

  const script = await scriptOf(sim, name);
  const run = (code) => executeInScript(sim, script, code);
  const installed = await run(E4_LUA + "\nreturn womatE4Install()");
  if (installed.pickup || installed.handoverPose) {
    throw new Error(`${name} is in pickup or handover-pose mode; restart the cell so it is idle (no agents).`);
  }
  const dof = Number(installed.dof);
  const dt = Number(installed.dt);
  fs.mkdirSync(path.join(options.outDir, "traces"), { recursive: true });

  const homeFile = path.join(options.outDir, "e4_home.json");
  const home = fs.existsSync(homeFile) ? JSON.parse(fs.readFileSync(homeFile, "utf8")).config : asArray(installed.config).map(Number);
  if (!fs.existsSync(homeFile)) fs.writeFileSync(homeFile, JSON.stringify({ robot: name, config: home }, null, 2) + "\n");

  const reset = async () => {
    for (const hard of [false, true]) {
      const state = await run(`return womatE4Reset(${luaArray(home)}, ${hard})`);
      await delay(hard ? options.settleMs * 4 : options.settleMs);
      const config = asArray(await run("return womatCurrentConfig()")).map(Number);
      const worst = Math.max(...config.map((value, i) => Math.abs(value - home[i]))) * DEG;
      if (worst <= 0.5) return;
      if (hard) throw new Error(`${name} could not be returned home (joint error ${worst.toFixed(2)} deg, reset ${state?.errorDeg}).`);
    }
  };

  // Reserved execution: one sample per step through the driver's own path
  // executor, watchdog on, as the handover routes run.
  const follow = async (flatPath) => {
    await run(`return womatFollowConfigs(${luaArray(flatPath)}, true)`);
    const done = await waitForMotionComplete(sim, script);
    const error = await motionExecutionError(sim, script);
    const reached = await run("return womatPathDidReachGoal()");
    return { success: done && !error && reached === true, reason: !done ? "timeout" : error || (reached ? null : "goal_not_settled") };
  };
  const ruckig = async (code) => {
    await run(code);
    const done = await waitForMotionComplete(sim, script);
    const error = await motionExecutionError(sim, script);
    return { success: done && !error, reason: !done ? "timeout" : error };
  };
  const cartesianRoute = async (target, schedule) => {
    let result = await run(`return womatPrepareSimpleCartesianRoute(${target.x}, ${target.y}, ${target.z}, `
      + `${base.cartesianFinalTolerance}, ${luaArray(schedule)})`);
    const flat = [];
    while (result?.ok) {
      flat.push(...asArray(result.path).map(Number));
      if (!result.pending) return { ok: true, path: flat };
      await delay(0);
      result = await run("return womatContinueSimpleCartesianRoute()");
    }
    return { ok: false, reason: result?.reason || "cartesian_route_failed" };
  };
  const cartesianSchedule = (variant, distance) => (variant === "linear"
    ? linearSchedule(distance, limits.cartesian.velocity, dt)
    : motionSchedule(distance, limits.cartesian, dt));

  const approach = async (variant, target) => {
    const setup = await run("return womatSimpleCartesianRouteSetup()");
    const from = setup.from;
    const distance = Math.hypot(target.x - from.x, target.y - from.y, target.z - from.z);
    if (variant === "ruckig") {
      return { ...(await ruckig(`return womatStartSimpleCartesian(${target.x}, ${target.y}, ${target.z})`)), distance };
    }
    const schedule = cartesianSchedule(variant, distance);
    const route = await cartesianRoute(target, schedule.slice(1));
    if (!route.ok) return { success: false, reason: route.reason, distance };
    return { ...(await follow(route.path)), distance, nominal: schedule.map((fraction) => fraction * distance) };
  };
  const goHome = async (variant) => {
    const start = asArray(await run("return womatCurrentConfig()")).map(Number);
    const distance = Math.max(...home.map((value, i) => Math.abs(value - start[i])));
    if (variant === "ruckig") return { ...(await ruckig(`return womatStartSimpleJoint(${luaArray(home)})`)), distance };
    const route = variant === "linear"
      ? linearJointPath(start, home, limits.joint.velocity, dt)
      : homeTrajectory(start, home, base, dt);
    const schedule = variant === "linear" ? linearSchedule(distance, limits.joint.velocity, dt) : motionSchedule(distance, limits.joint, dt);
    return { ...(await follow(route.flat())), distance, nominal: schedule.map((fraction) => fraction * distance) };
  };
  const recorded = async (motion) => {
    await run("return womatE4Arm()");
    await delay(options.padMs);
    let result;
    try {
      result = await motion();
    } finally {
      await delay(options.padMs);
    }
    const trace = await run("return womatE4Disarm()");
    return { result, trace };
  };

  // The handover point: nearest the centroid of the pair's shared space, far
  // enough from the home tool position for a cruise, and reachable along a
  // straight line with the watchdog quiet.
  const targetFile = path.join(options.outDir, "e4_target.json");
  let target = fs.existsSync(targetFile) ? JSON.parse(fs.readFileSync(targetFile, "utf8")).local : null;
  if (options.target) target = options.target;
  if (!target) {
    const csv = path.join(REPO_ROOT, "generated", "robots", "interactions", options.pair, "space.csv");
    if (!fs.existsSync(csv)) throw new Error(`No interaction space at ${path.relative(REPO_ROOT, csv)}; run the cell's agents once or pass --target x,y,z.`);
    const candidates = handoverCandidates(fs.readFileSync(csv, "utf8"));
    const locals = reshape(asArray(await run(`return womatE4ToLocal(${luaArray(candidates.flatMap((p) => [p.x, p.y, p.z]))})`)).map(Number), 3);
    const tool = installed.tool;
    let tried = 0;
    for (let index = 0; index < candidates.length && tried < options.maxCandidates; index += 1) {
      const [x, y, z] = locals[index];
      const distance = Math.hypot(x - tool.x, y - tool.y, z - tool.z);
      if (distance < options.minDistance) continue;
      tried += 1;
      await reset();
      const attempt = await approach("double_s", { x, y, z });
      console.log(`  candidate ${index + 1} (${distance.toFixed(3)} m from home tool): ${attempt.success ? "reached" : attempt.reason}`);
      if (attempt.success) {
        target = { x, y, z };
        fs.writeFileSync(targetFile, JSON.stringify({ robot: name, pair: options.pair, world: candidates[index],
          local: target, distance }, null, 2) + "\n");
        break;
      }
    }
    if (!target) throw new Error(`None of ${tried} candidates of ${options.pair} could be approached along a line; pass --target x,y,z.`);
  }
  console.log(`${name}: target ${JSON.stringify(target)}, dt ${dt} s, recorder via ${installed.hook}`);

  const logFile = path.join(options.outDir, "e4_runs.jsonl");
  const done = new Set(readRecords(logFile).map((record) => `${record.repetition}|${record.variant}`));
  for (let repetition = 1; repetition <= options.repeats; repetition += 1) {
    for (const variant of VARIANTS) {
      if (done.has(`${repetition}|${variant}`)) continue;
      await reset();
      const legs = {
        approach: await recorded(() => approach(variant, target)),
        home: await recorded(() => goHome(variant))
      };
      for (const leg of LEGS) {
        const { result, trace } = legs[leg];
        const file = path.join("traces", `${leg}_${variant}_r${repetition}.json`);
        fs.writeFileSync(path.join(options.outDir, file), JSON.stringify({ dof, dt, nominal: result.nominal || null, trace }) + "\n");
        const record = { robot: name, repetition, leg, variant, success: result.success === true,
          reason: result.reason || null, distance: result.distance, dof, dt, trace: file };
        fs.appendFileSync(logFile, JSON.stringify(record) + "\n");
        console.log(`  r${repetition} ${variant.padEnd(9)} ${leg.padEnd(9)} ${record.success ? "ok" : `FAILED ${record.reason}`}`
          + ` (${Number(result.distance).toFixed(3)} ${leg === "home" ? "rad" : "m"}, ${asArray(trace?.t).length} steps)`);
      }
    }
  }
  await reset();
  return { limits, target, dt, home };
}

// ---------------------------------------------------------------- analysis

function analyze(outDir, setup) {
  const logFile = path.join(outDir, "e4_runs.jsonl");
  const runs = readRecords(logFile);
  if (!runs.length) throw new Error(`No runs recorded in ${logFile}.`);
  const setupFile = path.join(outDir, "e4_setup.json");
  const saved = setup || JSON.parse(fs.readFileSync(setupFile, "utf8"));
  const limits = saved.limits;
  // Plotted joint: the one leading the reference (double-S) return home.
  const reference = runs.find((run) => run.leg === "home" && run.variant === "double_s");
  const load = (run) => JSON.parse(fs.readFileSync(path.join(outDir, run.trace), "utf8"));
  const referenceTrace = reference ? load(reference) : null;
  const lead = referenceTrace ? leadingJoint(reshape(referenceTrace.trace.qc, referenceTrace.dof)) : 0;

  fs.mkdirSync(path.join(outDir, "lead"), { recursive: true });
  const records = [];
  const span = Object.fromEntries(LEGS.map((leg) => [leg, 0]));
  for (const run of runs) {
    const { dof, dt, nominal, trace } = load(run);
    const legLimits = run.leg === "home" ? limits.joint : limits.cartesian;
    const metrics = analyzeTrace(trace, { dof, dt, leg: run.leg, limits: legLimits, lead, nominal,
      tolerance: Number(saved.settleToleranceDeg || 1) / DEG });
    const { series, ...rest } = metrics;
    records.push({ ...run, ...rest });
    if (run.repetition === 1 && series) {
      span[run.leg] = Math.max(span[run.leg], series[series.length - 1].t);
      fs.writeFileSync(path.join(outDir, "lead", `${run.leg}_${run.variant}.csv`),
        toCsv(series, ["t", "pos", "vel", "acc", "jerk", "posMeas", "velMeas", "accMeas", "jerkMeas"]));
    }
  }
  const unit = { joint: DEG, cartesian: 1 };
  const shown = Object.fromEntries(Object.entries(limits).map(([kind, bounds]) =>
    [kind, Object.fromEntries(Object.entries(bounds).map(([key, value]) => [key, value * unit[kind]]))]));
  const rows = summaryRows(records);
  const recordColumns = ["robot", "repetition", "leg", "variant", "success", "reason", "distance", "steps", "moved", ...SUMMARY_METRICS, "recorderErrors"];
  fs.writeFileSync(path.join(outDir, "e4_runs.csv"), toCsv(records, recordColumns));
  fs.writeFileSync(path.join(outDir, "e4_summary.csv"), toCsv(rows, ["leg", "variant", "runs", "reached", ...SUMMARY_METRICS]));
  fs.writeFileSync(path.join(outDir, "e4_table.tex"), thesisTable(rows, shown));
  const present = (leg) => VARIANTS.filter((variant) => runs.some((run) => run.leg === leg && run.variant === variant && run.repetition === 1));
  for (const leg of LEGS) fs.writeFileSync(path.join(outDir, `e4_figure_${leg}.tex`), thesisFigure(leg, shown, present(leg), span[leg] || 5));
  if (setup) fs.writeFileSync(setupFile, JSON.stringify({ ...setup, leadJoint: lead + 1, finishedAt: new Date().toISOString() }, null, 2) + "\n");
  return { rows, lead, limits: shown };
}

function printSummary({ rows, lead, limits }) {
  const f = (value, digits) => (Number.isFinite(value) ? value.toFixed(digits) : "-").padStart(9);
  console.log(`\nPeaks commanded / measured; home leg: largest joint rate [deg], approach: tool [m]. Plotted joint: ${lead + 1}.`);
  console.log(`${"leg".padEnd(9)}${"variant".padEnd(10)}${"ok".padStart(5)}${"dur[s]".padStart(9)}`
    + ["vel", "acc", "jerk"].map((key) => `${key} cmd`.padStart(10) + `${key} meas`.padStart(10)).join("")
    + "   x limit (cmd v/a/j)");
  for (const row of rows) {
    const keys = row.leg === "home" ? ["JointVelDeg", "JointAccDeg", "JointJerkDeg"] : ["ToolVel", "ToolAcc", "ToolJerk"];
    console.log(`${row.leg.padEnd(9)}${row.variant.padEnd(10)}${`${row.reached}/${row.runs}`.padStart(5)}${f(row.durationS, 2)}`
      + keys.map((key) => f(row[`commanded${key}`], 3) + " " + f(row[`measured${key}`], 3)).join("")
      + `   ${[row.commandedVelRatio, row.commandedAccRatio, row.commandedJerkRatio].map((v) => (Number.isFinite(v) ? v.toFixed(2) : "-")).join(" / ")}`);
  }
  console.log(`Limits: joint ${limits.joint.velocity}/${limits.joint.acceleration}/${limits.joint.jerk} deg, `
    + `Cartesian ${limits.cartesian.velocity}/${limits.cartesian.acceleration}/${limits.cartesian.jerk} m`);
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
      "Usage: node src/eval/e4Smoothness.js [run] [--robot Robot2] [--pair Robot2__Robot4] [--target x,y,z]",
      "                                     [--repeats 3] [--min-distance 0.3] [--max-candidates 15]",
      "                                     [--pad-ms 400] [--settle-ms 300] [--out performance_log/e4]",
      "       node src/eval/e4Smoothness.js analyze [--out performance_log/e4]",
      "run needs the five-robot cell served and idle (no agents). It resumes from e4_runs.jsonl;",
      "delete the output directory to start over (e4_target.json fixes the approach target)."
    ].join("\n"));
    return mode === "run" || mode === "analyze" ? 0 : 1;
  }
  const outDir = path.resolve(REPO_ROOT, args.out || DEFAULT_OUT);
  if (mode === "analyze") {
    printSummary(analyze(outDir));
    return 0;
  }
  let target = null;
  if (args.target) {
    const [x, y, z] = String(args.target).split(",").map(Number);
    if (![x, y, z].every(Number.isFinite)) throw new Error(`--target must be x,y,z in the robot frame, got ${args.target}.`);
    target = { x, y, z };
  }
  const options = {
    outDir,
    robot: args.robot || "Robot2",
    pair: args.pair || "Robot2__Robot4",
    target,
    repeats: positiveNumber(args.repeats || 3, "repeats"),
    minDistance: positiveNumber(args["min-distance"] || 0.3, "min-distance", false),
    maxCandidates: positiveNumber(args["max-candidates"] || 15, "max-candidates"),
    padMs: positiveNumber(args["pad-ms"] || 400, "pad-ms"),
    settleMs: positiveNumber(args["settle-ms"] || 300, "settle-ms")
  };
  fs.mkdirSync(outDir, { recursive: true });
  const live = await runLive(options);
  const setup = { robot: options.robot, pair: options.pair, repeats: options.repeats, dt: live.dt,
    target: live.target, limits: live.limits, settleToleranceDeg: 1, variants: VARIANTS };
  printSummary(analyze(outDir, setup));
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
  E4_LUA,
  LEGS,
  VARIANTS,
  activeWindow,
  analyzeTrace,
  derivatives,
  handoverCandidates,
  leadingJoint,
  linearJointPath,
  linearSchedule,
  settledIndex,
  summaryRows,
  thesisFigure,
  thesisTable
};
