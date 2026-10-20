"use strict";
const assert = require("node:assert/strict");
const test = require("node:test");
const { SharedRegion } = require("./claims");
const { followHandoverExit, prepareSimpleCartesianRoute, prepareJointRoute } = require("./handoverExit");
const { simpleSmoothCartesianMove, handoverRetreat } = require("./motion");
const { createRuntimeOptions } = require("./runtimeOptions");
const region = new SharedRegion({ pairKey: "A__B", robots: ["A", "B"],
  voxelSize: 0.1, origin: { x: 0, y: 0, z: 0 }, voxelKeys: ["5,5,5"] });
const inside = [0, 0, 0, 0.55, 0.55, 0.55];
const outside = [0, 0, 0, 0.1, 0, 0.1];
const options = createRuntimeOptions({ manifest: { runtime: {
  enableCollisionCheck: true, simpleSmoothMotion: true, preferJointTrajectoryForCartesian: true
} } });

const routeSetup = { ok: true, from: { x: 0, y: 0, z: 0.4 }, dt: 0.05 };

test("Cartesian route preparation combines short simulator calls without starting a partial path", async () => {
  const calls = [];
  const chunks = [
    { ok: true, pending: true, path: [0, 0, 0.1, 0.2], dof: 2 },
    { ok: true, pending: true, path: [0.2, 0.4], dof: 2 },
    { ok: true, pending: false, path: [0.3, 0.6], dof: 2 }
  ];
  const sim = { executeScriptString: async (code) => {
    calls.push(code);
    return code.includes("womatSimpleCartesianRouteSetup") ? routeSetup : chunks.shift();
  } };
  const result = await prepareSimpleCartesianRoute(sim, 1, { x: 1, y: 0, z: 0.4 }, options);
  assert.deepEqual(result.path, [0, 0, 0.1, 0.2, 0.2, 0.4, 0.3, 0.6]);
  assert.match(calls[0], /womatSimpleCartesianRouteSetup/);
  assert.match(calls[1], /womatPrepareSimpleCartesianRoute/);
  assert.deepEqual(calls.slice(2), Array(2).fill("return womatContinueSimpleCartesianRoute()"));
  // The core paces the route: the schedule it hands over must start away from
  // rest, end exactly on the target, and never run backwards.
  const schedule = calls[1].match(/\{([^}]*)\}/)[1].split(",").map(Number);
  assert.ok(schedule.length > 1);
  assert.equal(schedule.at(-1), 1);
  assert.ok(schedule.every((value, index) => index === 0 || value >= schedule[index - 1]));
});

test("a failed Cartesian chunk discards the partial route", async () => {
  const chunks = [{ ok: true, pending: true, path: [0, 0.1], dof: 1 },
    { ok: false, reason: "cartesian_route_ik_failed" }];
  const sim = { executeScriptString: async (code) =>
    (code.includes("womatSimpleCartesianRouteSetup") ? routeSetup : chunks.shift()) };
  assert.deepEqual(await prepareSimpleCartesianRoute(sim, 1, { x: 1, y: 0, z: 0.4 }, options),
    { ok: false, reason: "cartesian_route_ik_failed" });
});

test("a route whose start the simulator cannot report is not attempted", async () => {
  const sim = { executeScriptString: async () => ({ ok: true, from: { x: 0, y: NaN, z: 0 }, dt: 0.05 }) };
  assert.deepEqual(await prepareSimpleCartesianRoute(sim, 1, { x: 1, y: 0, z: 0.4 }, options),
    { ok: false, reason: "cartesian_route_start_unavailable" });
});

test("reserved joint proposals defer collision checking to the reserved executor", async () => {
  const sim = { executeScriptString: async (code) => {
    if (code === "return womatCurrentConfig()") return [0, [0.3, -0.1]];
    if (code === "return sim.getSimulationTimeStep()") return 0.05;
    throw new Error(`Unexpected blocking preflight: ${code}`);
  } };
  const result = await prepareJointRoute(sim, 1, [0, 0], { ...options, executeReservedPath() {} });
  assert.equal(result.success, true);
  assert.deepEqual(result.path.slice(0, 2), [0.3, -0.1]);
  assert.deepEqual(result.path.slice(-2), [0, 0]);
});

test("a failed exit trajectory never reports a reached clearance point", async () => {
  const sim = { executeScriptString: async (code) => {
    if (code.includes("womatFollowConfigs")) return "started";
    if (code === "return check()") return 3;
    if (code.includes("womatGetMotionError")) return "wall collision";
    throw new Error(code);
  } };
  assert.equal((await followHandoverExit(sim, 1, [1, 1, 0, 0], [0, 0], options)).success, false);
});

// The straight-line carry, with a switch for whether the cell allows it.
function carrySimulator({ straightLineBlocked = false } = {}) {
  const calls = [];
  const target = { x: 1, y: 0, z: 0.4 };
  const state = { calls, target, position: { x: 0, y: 0, z: 0.4 } };
  state.sim = { executeScriptString: async (code) => {
    calls.push(code);
    if (code.includes("womatPickupApproachEnabled") || code.includes("womatHandoverPoseEnabled")) return false;
    if (code.includes("getCartesianposition")) return state.position;
    if (code.includes("womatFindGoalConfig")) return { config: [0.5, 0.6] };
    if (code.includes("womatPrepareJointRoute")) return { ok: true, path: [0, 0, 0.2, 0.5, 0.5, 0.6] };
    if (code.includes("womatFollowConfigs")) { state.position = state.target; return "started"; }
    if (code.includes("womatCurrentConfig")) return [0, [0.5 + Math.PI / 180, 0.6]];
    if (code === "return check()") return 3;
    if (code.includes("womatGetMotionError")) return null;
    if (code.includes("womatSweepCartesianPath")) {
      return straightLineBlocked ? { ok: false, reason: "cartesian_body_path_collision_detected" } : { ok: true };
    }
    if (code.includes("womatStartSimpleCartesian")) {
      const numbers = code.match(/\(([^)]+)\)/)[1].split(",").map(Number);
      state.position = { x: numbers[0], y: numbers[1], z: numbers[2] };
      return "started";
    }
    throw new Error(code);
  } };
  return state;
}

test("a carry takes the straight Cartesian line, not a joint route to a solved elbow", async () => {
  // preferJointTrajectoryForCartesian is set on these options. It selects what
  // the FALLBACK does; it must not pre-empt the straight line and swing the
  // tool out and back on the way to the rendezvous.
  const state = carrySimulator();
  const { sim, calls, target } = state;
  const result = await simpleSmoothCartesianMove(sim, 4321, target, null, options);
  assert.equal(result.success, true, JSON.stringify(result));
  assert.equal(result.planner, "coppeliasim_smooth_pose_ik");
  assert.ok(calls.some((code) => code.includes("womatStartSimpleCartesian")), "the tool is streamed along the line");
  assert.ok(calls.every((code) => !code.includes("womatFindGoalConfig") && !code.includes("womatPrepareJointRoute")),
    "no goal configuration is solved for a line the cell allows");
  const retreat = await handoverRetreat(sim, 4321, null, options);
  assert.equal(retreat.success, true);
  assert.ok(Math.abs(state.position.x - (1 - options.handoverStandoffDistance)) < 1e-6);
});

test("a straight line the cell refuses still falls back to the collision-checked joint route", async () => {
  const { sim, calls, target } = carrySimulator({ straightLineBlocked: true });
  const result = await simpleSmoothCartesianMove(sim, 4322, target, null, options);
  assert.equal(result.success, true, JSON.stringify(result));
  assert.equal(result.planner, "collision_checked_joint_route");
  assert.ok(calls.findIndex((code) => code.includes("womatPrepareJointRoute"))
    < calls.findIndex((code) => code.includes("womatFollowConfigs")));
});

test("returning home between handovers keeps the fallback: the route is the agent's, the safety the core's", async () => {
  // returnHomeBetweenHandovers only makes the agent go home with the cube
  // before the next handover. It once also switched this robot's fallback off,
  // so a blocked straight line simply failed.
  const returning = createRuntimeOptions({ manifest: { runtime: {
    enableCollisionCheck: true, simpleSmoothMotion: true, preferJointTrajectoryForCartesian: true,
    returnHomeBetweenHandovers: true
  } } });
  const state = carrySimulator({ straightLineBlocked: true });
  const { sim, calls, target } = state;
  const result = await simpleSmoothCartesianMove(sim, 9876, target, null, returning);
  assert.equal(result.success, true, JSON.stringify(result));
  assert.equal(result.planner, "collision_checked_joint_route");
  assert.ok(calls.some((code) => code.includes("womatPrepareJointRoute")));
  // The retreat out of the rendezvous falls back the same way.
  calls.length = 0;
  // The retreat backs off by the standoff distance along the way it came in.
  state.target = { x: target.x - returning.handoverStandoffDistance, y: target.y, z: target.z };
  const retreat = await handoverRetreat(sim, 9876, null, returning);
  assert.equal(retreat.success, true, JSON.stringify(retreat));
  assert.ok(calls.some((code) => code.includes("womatPrepareJointRoute")), "the blocked retreat line is replanned");
});
