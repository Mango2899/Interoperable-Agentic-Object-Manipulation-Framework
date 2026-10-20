"use strict";
const test = require("node:test");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { execFileSync } = require("node:child_process");

// The in-motion watchdog on dynamic joints. Such a joint is only given a
// target; the physics engine moves it during the step that follows. A watchdog
// that tests the scene as it stands therefore tests the posture the arm is
// leaving, and finds a wall only once the arm is in it -- E3 measured 1 to 4
// steps of contact before every stop. Here a joint reaches its target one step
// late, contact is anything at or beyond 0.5, and the arm must stop short.
const lua = (file) => fs.readFileSync(path.join(__dirname, "../lua", file), "utf8");
const between = (source, from, to) => source.slice(source.indexOf(from), source.indexOf(to));
const safety = between(lua("04-safety.lua"), "function womatCommandedConfig(", "function womatCollectionsUnsafe(");
const simple = lua("02-simple-motion.lua");
const callbacks = between(simple, "function womatSimplePoseCallback(", "function womatSimpleMoveToPose(")
  + between(simple, "function womatSimpleJointCallback(", "function womatSimpleMoveToConfig(");
const playback = lua("06-path-execution.lua")
  .replace(/\$\{pathTrackingToleranceDeg\}/g, "0.5")
  .replace(/\$\{pathSettleMaxSteps\}/g, "20")
  .replace(/\$\{collisionCheckStepDeg\}/g, "2");

const scene = `
jointHandles={1}
local position=0.1
local target=position
reached=position
sim={handle_world=-1}
function sim.isDynamicallyEnabled() return true end
function sim.getJointPosition() return position end
function sim.setJointPosition(_, value) position=value end
function sim.getJointTargetPosition() return target end
function sim.setJointTargetPosition(_, value) target=value end
function sim.switchThread() coroutine.yield() end
function sim.setObjectPose(_, _, pose) ikTarget=pose end
function sim.getObjectPose() return {0} end
simIK={}
function simIK.applyIkEnvironmentToScene() target=ikTarget[1] end
function movCallback(config) target=config[1] end
function womatResetInMotionSafety() end
function womatInMotionUnsafe() return position >= 0.5 end
function womatCurrentFramePositions() return {points={0,0,0}} end
-- One simulation step: the scripts command, then physics carries it out.
function physics()
    position=target
    reached=math.max(reached, position)
end
womatSimpleSafetyEnabled=true
`;

function runLua(t, body) {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-watchdog-lua-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  const script = path.join(dir, "watchdog.lua");
  fs.writeFileSync(script, scene + safety + callbacks + playback + body);
  execFileSync("lua", [script], { encoding: "utf8", timeout: 5000 });
}

test("path playback stops a dynamic arm before the sample that would touch", (t) => {
  runLua(t, `
womatFollowConfigs({0.2,0.3,0.4,0.6,0.8}, true)
for step=1,20 do
    sysCall_actuation()
    physics()
end
assert(rState == 3)
assert(womatMotionError == 'home_path_collision_detected', tostring(womatMotionError))
assert(reached < 0.5, 'the arm reached '..reached)
assert(sim.getJointPosition() == 0.4)
`);
});

test("the Cartesian pose callback tests the posture the IK sends the joints to", (t) => {
  runLua(t, `
local data={target=1, tip=2}
for _, next in ipairs({0.2,0.3,0.4}) do
    womatSimplePoseCallback({next}, nil, nil, data)
    physics()
end
local ok, err=pcall(womatSimplePoseCallback, {0.6}, nil, nil, data)
physics()
assert(not ok and string.find(err, 'simple_cartesian_path_collision_detected'), tostring(err))
assert(reached < 0.5, 'the arm reached '..reached)
assert(sim.getJointTargetPosition() == 0.4)
`);
});

test("the joint callback refuses a configuration before sending it", (t) => {
  runLua(t, `
womatSimpleJointCallback({0.3}, nil, nil, jointHandles)
physics()
local ok, err=pcall(womatSimpleJointCallback, {0.6}, nil, nil, jointHandles)
physics()
assert(not ok and string.find(err, 'simple_joint_path_collision_detected'), tostring(err))
assert(reached < 0.5, 'the arm reached '..reached)
assert(sim.getJointPosition() == 0.3 and sim.getJointTargetPosition() == 0.3)
`);
});
