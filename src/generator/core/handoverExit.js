"use strict";

const { asArray } = require("../workspaceGrid");
const { luaArray } = require("./luaDriver");
const { executeInScript, simulationTimeStep } = require("./simConnection");
const { waitForMotionComplete, motionExecutionError } = require("./safety");
const { delay } = require("./context");
const { homeTrajectory } = require("./continuousHome");
const { motionSchedule } = require("./timeLaw");

async function prepareSimpleCartesianRoute(sim, handle, target, options) {
  // The straight line is the simulator's to walk, but its TIMING is the core's,
  // so the reserved route is paced by the same jerk-limited law as every joint
  // leg rather than by a second profile that happened to live in Lua.
  const setup = await executeInScript(sim, handle, "return womatSimpleCartesianRouteSetup()");
  const from = setup?.from;
  if (!setup?.ok || ![from?.x, from?.y, from?.z].every(Number.isFinite)) {
    return { ok: false, reason: "cartesian_route_start_unavailable" };
  }
  const dt = Number(setup.dt) || 0.05;
  const distance = Math.hypot(target.x - from.x, target.y - from.y, target.z - from.z);
  const schedule = motionSchedule(distance, {
    velocity: options.simpleCartesianVelocity,
    acceleration: options.simpleCartesianAcceleration,
    jerk: options.simpleCartesianJerk
  }, dt).slice(1);
  let result = await executeInScript(sim, handle,
    `return womatPrepareSimpleCartesianRoute(${target.x}, ${target.y}, ${target.z}, `
    + `${options.cartesianFinalTolerance}, ${luaArray(schedule)})`);
  const path = [];
  const deadline = Date.now() + 30000;
  while (result?.ok) {
    path.push(...asArray(result.path).map(Number));
    if (!result.pending) return { ...result, path };
    if (Date.now() >= deadline) {
      await executeInScript(sim, handle, "womatCartesianRouteJob=nil; return true");
      return { ok: false, reason: "cartesian_route_timeout" };
    }
    // Each call restores the scene. Other arms and pending lock releases can
    // run between the short proposal chunks.
    await delay(0);
    result = await executeInScript(sim, handle, "return womatContinueSimpleCartesianRoute()");
  }
  return result;
}

async function prepareJointRoute(sim, handle, target, options) {
  if (options.executeReservedPath) {
    // The reserved executor checks this exact path after acquiring its swept
    // claim. Do not perform the same dense sweep inside one blocking Lua call
    // before it. A blocked proposal still falls back with full-room locking.
    const start = asArray(await executeInScript(sim, handle, "return womatCurrentConfig()")).map(Number);
    if (start.length !== target.length || !start.every(Number.isFinite)) return { success: false, reason: "invalid_joint_configuration" };
    const dt = await simulationTimeStep(sim, handle);
    return { success: true, path: homeTrajectory(start, target, options, dt).flat(), dof: target.length };
  }
  const result = await executeInScript(sim, handle,
    `return womatPrepareJointRoute(${luaArray(target)}, ${options.planningMaxTimeSeconds}, `
    + `${options.planningStateCount}, ${options.enablePathPlanning ? "true" : "false"})`);
  if (!result?.ok) return { success: false, reason: result?.reason || "handover_exit_unplannable" };
  const path = asArray(result.path).map(Number);
  const dof = target.length;
  if (!dof || !path.length || path.length % dof || !path.every(Number.isFinite)) {
    return { success: false, reason: "handover_exit_path_invalid" };
  }
  return { success: true, path, dof };
}

// `watched` puts the route under the in-motion watchdog. A reserved exit route
// is replayed exactly as it was reserved and checked, so it does not need one;
// a route the planner produced for a blocked line is checked against the same
// geometry but reaches postures no reservation covered, and that is the route
// E3 saw collide.
async function followHandoverExit(sim, handle, path, target, options, requireJointTarget = true, watched = false) {
  const started = await executeInScript(sim, handle,
    `return womatFollowConfigs(${luaArray(path)}${watched ? ", true" : ""})`);
  if (started?.ok === false) return { success: false, reason: started.reason || "handover_exit_start_failed" };
  if (!await waitForMotionComplete(sim, handle,
    Math.max(900, path.length / target.length + options.pathSettleMaxSteps + 100))) {
    return { success: false, reason: "handover_exit_timeout" };
  }
  const error = await motionExecutionError(sim, handle);
  if (error) return { success: false, reason: "motion_execution_failed", plannerMessage: error };
  // Cartesian callers validate the actual TCP after execution. Their chosen
  // IK solution is not a separate joint-space goal.
  if (!requireJointTarget) return { success: true, planner: "handover_exit_joint_path" };
  const actual = asArray(await executeInScript(sim, handle, "return womatCurrentConfig()")).map(Number);
  const tolerance = options.pathTrackingToleranceDeg * Math.PI / 180;
  if (actual.length !== target.length || actual.some((value, i) =>
    !Number.isFinite(value) || Math.abs(value - target[i]) > tolerance)) {
    return { success: false, reason: "handover_exit_target_not_reached", actual, target,
      maxErrorDeg: Math.max(...actual.map((value, i) => Math.abs(value - target[i]) * 180 / Math.PI)),
      toleranceDeg: options.pathTrackingToleranceDeg };
  }
  return { success: true, planner: "handover_exit_joint_path" };
}

module.exports = { prepareJointRoute, prepareSimpleCartesianRoute, followHandoverExit };
