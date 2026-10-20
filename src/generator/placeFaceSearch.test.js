const assert = require('node:assert/strict');
const test = require('node:test');
const { topDownPick } = require('./core/motion');
const { createRuntimeOptions } = require('./core/runtimeOptions');

const point = { x: 0.4, y: 0.2, z: 0.18 };

async function place({ enabled = false, centerReachable = true, holding = true, target = point, placement, graspHeightOffset = 0 } = {}) {
  const options = createRuntimeOptions({ manifest: { sharedEnvironment: placement, runtime: { gripper: {
    enablePlaceFaceSearch: enabled,
    placeReleaseHeight: 0.2,
    graspHeightOffset
  } } } });
  const calls = [];
  let actual;
  const sim = { executeScriptString: async code => {
    calls.push(code);
    let result;
    if (code.includes('heldGripObject ~= nil')) result = holding;
    else if (code.includes('womatTopDownDropCandidates')) result = [{ x: 0.35, y: 0.2, z: 0.38 }];
    else if (code.includes('womatPlanTopDownRelease') || code.includes('womatPlanTopDownPick')) {
      const [x, y, z] = code.match(/\(([^)]+)\)/)[1].split(',').map(Number);
      actual = { x, y, z };
      result = !centerReachable && x === target.x
        ? { ok: false, reason: 'no_collision_free_hover_config' }
        : { ok: true, hover: [0,0,0,0,0,0], descent: holding ? [] : [0,0,0,0,0,0] };
    }
    else if (code.includes('womatCurrentConfig')) result = [0,0,0,0,0,0];
    else if (code === 'return check()') result = 3;
    else if (code.includes('womatGetMotionError')) result = null;
    else if (code.includes('getCartesianposition')) result = actual;
    else if (code.includes('womatToolControlPointWorld')) result = actual;
    else if (code.includes('womatStartSimpleJoint') || code.includes('womatFollowDescent')) result = true;
    else throw new Error(`Unexpected simulator call: ${code}`);
    return [1, result];
  } };
  return { result: await topDownPick(sim, 1, target, null, options), calls };
}

test('center-only drop hovers above the requested center with no face search or descent', async () => {
  const { result, calls } = await place();
  assert.equal(result.success, true);
  assert.deepEqual(result.dropPoint, { x: point.x, y: point.y, z: point.z + 0.2 });
  assert.equal(result.releaseHeight, 0.2);
  assert.equal(result.dropRelocatedBy, undefined);
  assert.equal(calls.filter(c => c.includes('womatPlanTopDownRelease')).length, 1);
  assert.ok(!calls.some(c => /womatTopDownDropCandidates|womatFollowDescent/.test(c)));
});

test('unreachable center fails without relocating or starting a motion', async () => {
  const { result, calls } = await place({ centerReachable: false });
  assert.equal(result.success, false);
  assert.equal(result.reason, 'no_collision_free_hover_config');
  assert.equal(calls.filter(c => c.includes('womatPlanTopDownRelease')).length, 1);
  assert.ok(!calls.some(c => /womatTopDownDropCandidates|womatStartSimpleJoint/.test(c)));
});

test('enabling face search retains fallback to a supported point off center', async () => {
  const { result, calls } = await place({ enabled: true, centerReachable: false });
  assert.equal(result.success, true);
  assert.equal(result.dropPoint.x, 0.35);
  assert.ok(result.dropRelocatedBy > 0);
  assert.ok(calls.some(c => c.includes('womatTopDownDropCandidates')));
});

test('face search remains enabled by default for existing robots', () => {
  assert.equal(createRuntimeOptions({ manifest: {} }).enablePlaceFaceSearch, true);
});

test('disabling drop face search preserves the pick descent', async () => {
  const { result, calls } = await place({ holding: false });
  assert.equal(result.success, true);
  assert.equal(result.planner, 'top_down_pick');
  assert.ok(calls.some(c => c.includes('womatFollowDescent')));
});


test('Robot5 support, hover, release, and retreat share the support center in world XY', async () => {
  const { classifyTaskSites } = require('./taskSites');
  const { orientationRadians, rotateLocal, rotateWorldToLocal } = require('./handoverGeometry');
  const placement = { offset: [3.48, 0.28, 0.03], orientationDeg: [-90, 20, 90] };
  const orientation = orientationRadians(placement);
  const supportWorld = { x: 4.33, y: 0.28, z: 0.08 };
  const local = rotateWorldToLocal({
    x: supportWorld.x - placement.offset[0],
    y: supportWorld.y - placement.offset[1],
    z: supportWorld.z - placement.offset[2]
  }, orientation);
  const options = createRuntimeOptions({ manifest: { sharedEnvironment: placement } });
  const { sites: [site] } = classifyTaskSites([{
    alias: 'Robot5_grip_support_b', kind: 'support', world: supportWorld, local,
    size: { x: 0.14, y: 0.14, z: 0.16 }
  }], { worldUpLocal: options.worldUpLocal, admits: () => ({ ok: true }) });
  const { result } = await place({ target: site.grasp, placement, graspHeightOffset: 0.005 });
  assert.equal(result.success, true);
  for (const [p, expectedZ] of [
    [site.grasp, 0.1825], [site.hover, 0.3825],
    [result.dropPoint, 0.3875], [result.retreatPoint, 0.5875]
  ]) {
    const delta = rotateLocal(p, orientation);
    const world = { x: delta.x + placement.offset[0], y: delta.y + placement.offset[1], z: delta.z + placement.offset[2] };
    assert.ok(Math.abs(world.x - supportWorld.x) < 1e-9, `world X drifted: ${world.x}`);
    assert.ok(Math.abs(world.y - supportWorld.y) < 1e-9, `world Y drifted: ${world.y}`);
    assert.ok(Math.abs(world.z - expectedZ) < 1e-9, `world Z: ${world.z}, expected ${expectedZ}`);
  }
});
