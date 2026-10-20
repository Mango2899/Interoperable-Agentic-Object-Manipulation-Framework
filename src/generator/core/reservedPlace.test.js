"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const { topDownPick } = require("./motion");
const { createRuntimeOptions } = require("./runtimeOptions");

const options = { ...createRuntimeOptions({ manifest: { runtime: {} } }), enablePlaceFaceSearch: false };
const point = { x: 0.3, y: 0.1, z: 0.2 };

// The driver as a top-down release sees it: holding the cube, a hover solved,
// no descent below it.
function releaseSimulator({ holding = true } = {}) {
  const calls = [];
  return { calls, executeScriptString: async (code) => {
    calls.push(code);
    if (code.includes("heldGripObject")) return holding;
    if (code.includes("womatPlanTopDownRelease") || code.includes("womatPlanTopDownPick")) {
      return { ok: true, hover: [0.5, 0.5], descent: holding ? [] : [0.5, 0.6] };
    }
    if (code.includes("womatCurrentConfig")) return [0, [0, 0]];
    if (code.includes("sim.getSimulationTimeStep")) return 0.05;
    if (code.includes("womatPlanJointPath")) return { ok: true, path: [0, 0, 0.3, 0.1, 0.5, 0.5] };
    if (code.includes("getCartesianposition")) return { x: 0.3, y: 0.1, z: 0.2 + options.placeReleaseHeight };
    if (code.includes("womatToolControlPointWorld")) return { x: 1, y: 2, z: 3 };
    throw new Error(`Unexpected call: ${code}`);
  } };
}

test("a place is reserved as the one path it is, not claimed as the regions it might cross", async () => {
  const sim = releaseSimulator();
  const reserved = [];
  const result = await topDownPick(sim, 1, point, null, { ...options, executeReservedPath: async (flat, dof, requireJoint) => {
    reserved.push({ flat, dof, requireJoint });
    return { success: true, planner: "handover_zone_path" };
  } });
  assert.equal(result.success, true);
  assert.equal(result.planner, "top_down_release");
  assert.equal(reserved.length, 1);
  assert.equal(reserved[0].dof, 2);
  assert.equal(reserved[0].requireJoint, true, "the hover is a joint goal");
  assert.deepEqual(reserved[0].flat.slice(-2), [0.5, 0.5]);
  // Nothing moved outside the reserved executor.
  assert.equal(sim.calls.some((code) => /womatStartSimpleJoint|womatPlanAndFollowJointPath|womatFollowDescent/.test(code)), false);
  assert.ok(result.dropPoint && result.retreatPoint, "the agent still learns where it let go");
});

test("a straight line the cell refuses is routed around and THAT path reserved", async () => {
  const sim = releaseSimulator();
  const reserved = [];
  const result = await topDownPick(sim, 1, point, null, { ...options, executeReservedPath: async (flat) => {
    reserved.push(flat);
    return reserved.length === 1 ? { success: false, reason: "home_path_blocked", safeToReplan: true } : { success: true };
  } });
  assert.equal(result.success, true);
  assert.equal(result.approachRoute, "planned_path");
  assert.deepEqual(reserved[1], [0, 0, 0.3, 0.1, 0.5, 0.5]);
});

test("a pick is refused the reserved route rather than run unreserved", async () => {
  const sim = releaseSimulator({ holding: false });
  let executed = false;
  const result = await topDownPick(sim, 1, point, null, { ...options, executeReservedPath: async () => {
    executed = true;
    return { success: true };
  } });
  assert.equal(result.reason, "pick_legs_not_reservable");
  assert.equal(result.safeToReplan, true);
  assert.equal(executed, false);
});
