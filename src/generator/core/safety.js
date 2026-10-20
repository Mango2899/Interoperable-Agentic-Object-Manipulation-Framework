"use strict";

// The safety core: workspace admission, swept preflight, and the queries that
// decide whether a commanded motion is allowed to start.
//
// Each robot's core owns one of these. Nothing in here is shared between
// robots -- the obstacle boxes, the grid and the sweeps all describe one arm.

const { asArray, segmentSamples } = require("../workspaceGrid");
const { luaArray } = require("./luaDriver");
const { delay } = require("./context");
const { executeInScript } = require("./simConnection");

// Obstacle geometry only changes between commands: the shared runtime lets one
// arm move at a time, and this robot's own held cube is never an obstacle to
// itself. One round trip per command therefore keeps the boxes exact.
async function refreshObstacleBoxes(sim, scriptHandle, workspace) {
  if (!workspace) return 0;
  try {
    const boxes = await executeInScript(sim, scriptHandle, "return womatObstacleLocalBoxes()");
    return workspace.setObstacleBoxes(asArray(boxes));
  } catch (error) {
    workspace.setObstacleBoxes([]);
    console.warn("Could not read obstacle bounds for workspace admission:", error?.message || error);
    return 0;
  }
}

async function getJointRadians(sim, scriptHandle, td) {
  const joints = await executeInScript(sim, scriptHandle, "return getJointposition()");
  return Object.keys(td.actions.moveTojointPosition.input.properties)
    .map((jointName) => Number(joints[jointName] || 0) * Math.PI / 180);
}

async function getCartesian(sim, scriptHandle) {
  const pos = await executeInScript(sim, scriptHandle, "return getCartesianposition()");
  return {
    x: Number(pos.x),
    y: Number(pos.y),
    z: Number(pos.z)
  };
}

function interpolate(start, target, steps, step) {
  const alpha = step / steps;
  return target.map((value, index) => start[index] + (value - start[index]) * alpha);
}

function cartesianDistance(start, target) {
  const dx = target.x - start.x;
  const dy = target.y - start.y;
  const dz = target.z - start.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function cartesianStepCount(start, target, options) {
  const distance = cartesianDistance(start, target);
  const maxStep = options.cartesianMaxVelocity * (options.stepDelayMs / 1000);
  const velocitySteps = maxStep > 0 ? Math.ceil(distance / maxStep) : 1;
  return Math.max(1, options.cartesianSteps, velocitySteps);
}

function jointStepCount(start, target, options) {
  const maxDelta = target.reduce((best, value, index) => Math.max(best, Math.abs(value - start[index])), 0);
  const maxStep = options.jointMaxVelocityDeg * Math.PI / 180 * (options.stepDelayMs / 1000);
  const velocitySteps = maxStep > 0 ? Math.ceil(maxDelta / maxStep) : 1;
  return Math.max(1, options.jointSteps, velocitySteps);
}

async function checkBodyCollisionIfEnabled(sim, scriptHandle, options) {
  // Full robot body (all links, gripper, held object) against the obstacle
  // collection — not just the tip point, so elbow sweeps are caught too.
  if (!options.enableCollisionCheck) {
    return false;
  }
  return executeInScript(sim, scriptHandle, "return womatRobotObstacleCollision()");
}

async function checkPointCollisionIfEnabled(sim, scriptHandle, options, point) {
  if (!options.enableCollisionCheck) {
    return false;
  }
  return executeInScript(sim, scriptHandle, `return womatPointObstacleCollision(${point.x}, ${point.y}, ${point.z})`);
}

async function preflightCartesianPath(sim, scriptHandle, start, target, options, steps) {
  for (let step = 1; step <= steps; step += 1) {
    const alpha = step / steps;
    const point = {
      x: start.x + (target.x - start.x) * alpha,
      y: start.y + (target.y - start.y) * alpha,
      z: start.z + (target.z - start.z) * alpha
    };
    if (await checkPointCollisionIfEnabled(sim, scriptHandle, options, point)) {
      return {
        ok: false,
        reason: "path_collision_detected",
        obstacleMode: "tip_vs_obstacle_aabb",
        blockedAtStep: step,
        blockedPoint: point
      };
    }
  }

  return { ok: true };
}

async function preflightCartesianBodyPath(sim, scriptHandle, start, target, options, steps) {
  if (!options.enableCollisionCheck) {
    return { ok: true };
  }

  const points = [];
  for (let step = 1; step <= steps; step += 1) {
    const alpha = step / steps;
    points.push([
      start.x + (target.x - start.x) * alpha,
      start.y + (target.y - start.y) * alpha,
      start.z + (target.z - start.z) * alpha
    ]);
  }

  const result = await executeInScript(
    sim,
    scriptHandle,
    `return womatPreflightCartesianBodyPath(${luaArray(points.map((point) => luaArray(point)))})`
  );
  if (!result || result.ok === false) {
    const blockedAt = Number(result?.blockedAtStep || 1);
    const blockedPoint = points[blockedAt - 1] || points[points.length - 1];
    return {
      ok: false,
      reason: result?.reason || "cartesian_body_path_collision_detected",
      obstacleMode: "robot_vs_obstacle_collection",
      blockedAtStep: blockedAt,
      blockedPoint: {
        x: blockedPoint[0],
        y: blockedPoint[1],
        z: blockedPoint[2]
      }
    };
  }

  return { ok: true };
}

async function preflightJointPath(sim, scriptHandle, start, targetRadians, options, steps) {
  if (!options.enableCollisionCheck || !options.enableJointCollisionPreflight) {
    return { ok: true };
  }

  const configs = [];
  for (let step = 1; step <= steps; step += 1) {
    configs.push(interpolate(start, targetRadians, steps, step));
  }
  // steps also sizes the executed trajectory, which wants a much finer
  // resolution than a preflight does.
  const bounded = boundSamples(configs, options.preflightMaxSamples);

  const swept = await executeInScript(
    sim,
    scriptHandle,
    `return womatPreflightJointPath(${luaArray(bounded.map(luaArray))})`
  );
  if (!swept || swept.ok === false) {
    const blockedAt = Number(swept?.blockedAtStep || bounded.length);
    const blockedConfig = bounded[blockedAt - 1] || bounded[bounded.length - 1];
    return {
      ok: false,
      reason: swept?.reason || "joint_path_collision_detected",
      obstacleMode: swept?.mode || "robot_vs_obstacle_collection",
      obstacle: swept?.obstacle,
      obstacleOwner: swept?.obstacleOwner,
      selfCollisionA: swept?.selfCollisionA,
      selfCollisionB: swept?.selfCollisionB,
      blockedAtStep: blockedAt,
      blockedOfSamples: bounded.length,
      blockedJoints: blockedConfig.map((value) => value * 180 / Math.PI)
    };
  }

  return { ok: true, samples: bounded.length };
}

// A sweep must run inside one script call to stay atomic against simulation
// stepping, so its cost is bounded purely by how many samples it takes.
function boundSamples(samples, maxSamples) {
  if (samples.length <= maxSamples) return samples;
  const kept = [];
  for (let index = 0; index < maxSamples - 1; index += 1) {
    kept.push(samples[Math.round((index * (samples.length - 1)) / (maxSamples - 1))]);
  }
  kept.push(samples[samples.length - 1]);
  return kept;
}

// Swept validation for one simple-mode Cartesian command, run before anything
// moves. This is what replaced the per-simulation-step collision check: the
// same geometry, the same IK, evaluated once instead of at 20 Hz.
async function preflightSimpleCartesianPath(sim, scriptHandle, samples, options) {
  if (!options.enableCollisionCheck || samples.length === 0) {
    return { ok: true };
  }
  const bounded = boundSamples(samples, options.preflightMaxSamples);
  const swept = await executeInScript(
    sim,
    scriptHandle,
    `return womatSweepCartesianPath(${luaArray(bounded.map((s) => luaArray([s.x, s.y, s.z])))})`
  );
  if (!swept || swept.ok === false) {
    const blockedAt = Number(swept?.blockedAtStep || bounded.length);
    return {
      ok: false,
      reason: swept?.reason || "simple_cartesian_path_collision_detected",
      obstacleMode: swept?.mode || "robot_vs_obstacle_collection",
      obstacle: swept?.obstacle,
      obstacleOwner: swept?.obstacleOwner,
      selfCollisionA: swept?.selfCollisionA,
      selfCollisionB: swept?.selfCollisionB,
      blockedAtStep: blockedAt,
      blockedPoint: bounded[blockedAt - 1] || bounded[bounded.length - 1],
      samples: bounded.length
    };
  }
  return { ok: true, samples: bounded.length };
}

// Everything a simple-mode Cartesian pose is checked against, in the frame the
// workspace CSV was recorded in and against the tip target that will actually
// be driven.
async function guardSimpleCartesianPose(sim, scriptHandle, target, workspace, options) {
  const start = await getCartesian(sim, scriptHandle);
  const samples = segmentSamples(start, target, options.preflightStepSize);
  if (workspace) {
    const admission = workspace.admitsPath(samples, options.workspaceTolerance);
    if (!admission.ok) {
      return { ok: false, ...admission, obstacleMode: "workspace_csv_admission", target };
    }
  }
  const swept = await preflightSimpleCartesianPath(sim, scriptHandle, samples, options);
  if (!swept.ok) return { ...swept, target };
  return { ok: true, samples: samples.length };
}

async function waitForMotionComplete(sim, scriptHandle, maxAttempts = 900) {
  for (let attempt = 0; attempt < maxAttempts; attempt += 1) {
    await delay(50);
    const state = Number(await executeInScript(sim, scriptHandle, "return check()"));
    if (state === 3) {
      return true;
    }
  }
  return false;
}

async function motionExecutionError(sim, scriptHandle) {
  const error = await executeInScript(sim, scriptHandle, "return womatGetMotionError()");
  return error === null || error === undefined || error === false || error === "" ? null : String(error);
}

module.exports = {
  boundSamples,
  cartesianDistance,
  cartesianStepCount,
  checkBodyCollisionIfEnabled,
  checkPointCollisionIfEnabled,
  getCartesian,
  getJointRadians,
  guardSimpleCartesianPose,
  interpolate,
  jointStepCount,
  motionExecutionError,
  preflightCartesianBodyPath,
  preflightCartesianPath,
  preflightJointPath,
  preflightSimpleCartesianPath,
  refreshObstacleBoxes,
  waitForMotionComplete
};
