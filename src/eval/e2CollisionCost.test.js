const assert = require("node:assert/strict");
const { execFileSync } = require("node:child_process");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const { BENCHMARK_LUA, robotIndex, summarizeChecks, thesisTable } = require("./e2CollisionCost");

function luaAvailable() {
  try {
    execFileSync("lua5.4", ["-v"], { stdio: "ignore" });
    return true;
  } catch {
    return false;
  }
}

test("robots are numbered by their scene name", () => {
  assert.equal(robotIndex("Robot3"), 3);
  assert.equal(robotIndex("Robot12_wall"), 12);
  assert.equal(robotIndex("Floor"), null);
});

test("timing is summarized over collision-free checks, and mismatches are counted", () => {
  const checks = [
    { hitWithout: false, hitWith: false, msWithout: 20, msWith: 2, overlaps: 3, robotShapes: 10, obstacleShapes: 50 },
    { hitWithout: false, hitWith: false, msWithout: 30, msWith: 4, overlaps: 5, robotShapes: 10, obstacleShapes: 50 },
    { hitWithout: true, hitWith: true, msWithout: 1, msWith: 1, overlaps: 9, robotShapes: 10, obstacleShapes: 50 },
    { hitWithout: true, hitWith: false, msWithout: 1, msWith: 1, overlaps: 9, robotShapes: -1, obstacleShapes: -1 }
  ];
  const summary = summarizeChecks(checks);
  assert.equal(summary.checks, 4);
  assert.equal(summary.free, 2);
  assert.equal(summary.collidingShare, 0.5);
  assert.equal(summary.mismatches, 1);
  assert.equal(summary.shapePairs, 500); // the unbounded check is not a pair count
  assert.equal(summary.msWithoutMean, 25);
  assert.equal(summary.msWithMean, 3);
  assert.equal(summary.pairsTestedMean, 4);
  assert.ok(Math.abs(summary.speedup - 25 / 3) < 1e-12);
});

test("the thesis rows follow the columns of tab:exp_e2", () => {
  const rows = thesisTable([{ cell: 2, shapePairs: 1520, pairsTestedMean: 2.5, msWithoutMean: 8.123, msWithMean: 1.02, speedup: 7.96 }]);
  assert.match(rows, /Two-robot & 1\\,520 & 2\.5 & 8\.12 & 1\.02 & 8\.0 \\\\/);
});

// The benchmark Lua against a stub of the driver it is installed into: two
// robot shapes against obstacles of three roots, where one pose touches.
test("the benchmark Lua builds per-cell collections, restores the arm, and times both modes", { skip: !luaAvailable() }, () => {
  const stub = String.raw`
local handles={}
local nextHandle=100
local collections={}
local joints={}
sim={object_shape_type=0, handle_single=0}
function sim.getSystemTime() return os.clock() end
function sim.getJointInterval(h) return false, {-1, 2} end
function sim.setJointPosition(h, v) joints[h]=v end
function sim.getJointPosition(h) return joints[h] or 0 end
function sim.createCollection() nextHandle=nextHandle+1; collections[nextHandle]={}; return nextHandle end
function sim.addItemToCollection(c, _, shape) table.insert(collections[c], shape) end
function sim.getCollectionObjects(c) return collections[c] end
function sim.destroyCollection(c) collections[c]=nil end
function sim.getObjectType() return 0 end
jointHandles={1, 2}
local rootsOf={[11]="Robot1", [12]="Robot2_wall", [13]="Robot3", [14]="Floor", [15]="Robot2"}
function womatRootOf(shape) return shape end
function womatPlainAlias(shape) return rootsOf[shape] end
function womatSceneObstacleShapes() return {12, 13, 14, 15} end
function womatIsRenderOnlyShape(shape) return false end
local robot=sim.createCollection(); sim.addItemToCollection(robot, 0, 1); sim.addItemToCollection(robot, 0, 2)
function womatRobotCollisionCollection() return robot end
womatBroadphaseEnabled=true
womatBroadphaseShapes={}
womatBroadphaseMargin=0.001
function womatBroadphaseEntry(c) return {shapes=collections[c]} end
function womatWorldBoxes(entry)
    local boxes={}
    for i, _ in ipairs(entry.shapes) do boxes[i]={0,0,0,1,1,1} end
    return boxes
end
modesSeen={}
function womatCollectionsTouch(a, b)
    modesSeen[womatBroadphaseEnabled]=true
    return (joints[1] or 0) > 0.5
end
function womatCaptureSweepPosture() return {joints={joints[1] or 0, joints[2] or 0}} end
function womatRestoreSweepPosture(state) joints[1]=state.joints[1]; joints[2]=state.joints[2] end
sim.setJointPosition(1, 0.25); sim.setJointPosition(2, -0.25)
`;
  const driver = `${stub}\n${BENCHMARK_LUA}\n` + String.raw`
womatE2Setup(7, 6, {2, 3})
local firstPose={womatE2Configs[4][1], womatE2Configs[4][2]}
-- Whatever else draws from math.random, the poses depend on the seed alone.
math.random(); math.random(); math.randomseed(12345)
local info=womatE2Setup(7, 6, {2, 3})
assert(womatE2Configs[4][1] == firstPose[1] and womatE2Configs[4][2] == firstPose[2], "poses reproducible from the seed")
assert(womatE2Configs[4][1] >= -1 and womatE2Configs[4][1] <= 1, "poses inside the joint limits")
assert(info.robotShapes == 2)
assert(info.cells[1].cell == 2 and info.cells[1].obstacleShapes == 3, "cell 2: Robot2 wall, Robot2, Floor")
assert(info.cells[2].cell == 3 and info.cells[2].obstacleShapes == 4, "cell 3 adds Robot3")
local out=womatE2Run(2, 1, 6, 2)
assert(#out == 6)
for _, check in ipairs(out) do
    assert(check.hitWithout == check.hitWith)
    assert(check.overlaps == 2*3 and check.robotShapes == 2 and check.obstacleShapes == 3)
    assert(check.msWithout >= 0 and check.msWith >= 0)
end
assert(modesSeen[true] and modesSeen[false], "both modes timed")
assert(womatBroadphaseEnabled == true, "the driver's own setting is restored")
assert(sim.getJointPosition(1) == 0.25 and sim.getJointPosition(2) == -0.25, "the arm is back where it was")
womatE2Finish()
assert(womatE2Cells == nil)
print("ok")
`;
  const file = path.join(fs.mkdtempSync(path.join(os.tmpdir(), "e2-lua-")), "driver.lua");
  fs.writeFileSync(file, driver);
  assert.equal(execFileSync("lua5.4", [file], { encoding: "utf8" }).trim(), "ok");
});
