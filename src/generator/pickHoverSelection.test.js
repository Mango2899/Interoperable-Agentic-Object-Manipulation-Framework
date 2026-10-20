const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");
const { execFileSync, spawnSync } = require("node:child_process");

const root = path.resolve(__dirname, "../..");
// The driver is assembled from src/generator/lua; read it the way the runtime
// does rather than grepping the JavaScript that used to embed it.
const source = require("./core/runtimeSource").runtimeSource();
const luaAvailable = spawnSync("lua", ["-v"], { stdio: "ignore" }).status === 0;

// Pulled out of the driver source rather than restated, so the test exercises
// the Lua that actually ships.
function luaFunctions(names) {
  return names.map((name) => {
    const start = source.indexOf(`function ${name}(`);
    assert.notEqual(start, -1, `missing Lua function ${name}`);
    const end = source.indexOf("\nfunction ", start + 1);
    return source.slice(start, end === -1 ? undefined : end);
  }).join("\n");
}

const STUB = [
  "jointHandles = {1,2,3,4,5,6}",
  "local LIMIT = 361.00001 * math.pi / 180",
  "sim = {}",
  "function sim.getJointInterval(h) return false, {-LIMIT, 2*LIMIT} end",
  // Every candidate below is a real solution; validity is not what is under test.
  "function womatPickConfigValid(c) return true end",
  "local function rad(t) local o={} for i,v in ipairs(t) do o[i]=v*math.pi/180 end return o end",
  "local function degsum(v) return v*180/math.pi end"
].join("\n");

function runLua(body) {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-hover-"));
  const file = path.join(dir, "run.lua");
  fs.writeFileSync(file, [
    STUB,
    luaFunctions(["womatUnwrapConfigToward", "womatConfigTravel", "womatClosestValidConfig"]),
    body
  ].join("\n"));
  return execFileSync("lua", [file], { cwd: root, encoding: "utf8" }).trim();
}

test("a hover pose is re-expressed in the angles nearest the current posture", { skip: !luaAvailable && "lua interpreter not installed" }, () => {
  // The configuration the arm actually chose on a live run: the same tool pose
  // as its unwrapped twin, reached by unwinding a whole turn on two joints.
  const out = runLua([
    "local current = rad{0,0,0,0,0,0}",
    "local wrapped = rad{-186.5, -97.4, 70.3, 187.2, 179.8, -38.6}",
    "local fixed = womatUnwrapConfigToward(wrapped, current)",
    "local turns = 0",
    "for i=1,6 do",
    "  local t = (fixed[i]-wrapped[i])/(2*math.pi)",
    "  if math.abs(t-math.floor(t+0.5)) > 1e-9 then turns = turns + 1 end",
    "end",
    "print(string.format('%.1f %.1f %d', fixed[1]*180/math.pi, fixed[4]*180/math.pi, turns))"
  ].join("\n"));
  const [joint1, joint4, poseChanges] = out.split(/\s+/);

  assert.equal(Number(joint1), 173.5, "joint1 must unwrap to its equivalent inside one turn");
  assert.equal(Number(joint4), -172.8, "joint4 must unwrap to its equivalent inside one turn");
  // The whole point: the POSE is untouched, only the route to it.
  assert.equal(Number(poseChanges), 0, "unwrapping must move every joint by a whole number of turns");
});

test("the hover branch chosen is the one closest to where the arm is standing", { skip: !luaAvailable && "lua interpreter not installed" }, () => {
  // findConfigs returns a set of valid branches. Picking any of them satisfies
  // the pose; picking the far one swings the base most of a turn and wraps the
  // arm back over itself on the way to a cube that is straight ahead.
  const out = runLua([
    "local current = rad{0,0,0,0,0,0}",
    "local candidates = {",
    "  rad{-186.5, -97.4,  70.3, 187.2, 179.8, -38.6},",
    "  rad{ 173.5, -97.4,  70.3,-172.8, 179.8, -38.6},",
    "  rad{  -6.5, -82.6, -70.3, -27.1,  -0.2, 141.4},",
    "  rad{ 353.5, -82.6, -70.3, 332.9, 359.8, 141.4},",
    "}",
    "local best, travel = womatClosestValidConfig(candidates, current)",
    "print(string.format('%.1f %.1f', best[1]*180/math.pi, degsum(travel)))"
  ].join("\n"));
  const [joint1, travel] = out.split(/\s+/).map(Number);

  assert.ok(Math.abs(joint1) < 30, `the base must face the cube, got joint1=${joint1} deg`);
  assert.ok(travel < 400, `travel must be the short branch, got ${travel} deg`);
});

test("the pick searches for branches to choose from, not one at random", () => {
  // findConfig returns a single random valid branch and is deprecated upstream.
  assert.match(source, /simIK\.findConfigs,\n\s*env, group, ikJoints/);
  assert.match(source, /findMultiple=true/);
  assert.match(source, /findAlt=true/);
  assert.match(source, /hover=womatClosestValidConfig\(found, saved\)/);
});

test("self-collision checks the tool against the arm, not a collection against itself", () => {
  // Measured against CoppeliaSim 4.10: for a collection checked against itself,
  // objects with the SAME self-collision indicator ARE checked and objects with
  // different ones are skipped -- the opposite of what this driver assumed.
  // Giving every link its own indicator therefore excluded every pair, so the
  // test returned false for every posture the robot ever held.
  assert.match(source, /function womatPrepareSelfCollisionCollections\(\)/);
  assert.match(source, /return sim\.checkCollision\(tool, links\)~=0/);
  assert.doesNotMatch(source, /sim\.checkCollision\(collection, collection\)/);
  assert.doesNotMatch(source, /objintparam_collection_self_collision_indicator,\n/);

  // Two collections also express what one integer per object cannot: check the
  // tool against the arm without checking the tool against its own fingers.
  assert.match(source, /local tool=sim\.createCollection\(0\)/);
  assert.match(source, /local links=sim\.createCollection\(0\)/);

  // The mount link and the one before it are in permanent contact with the
  // gripper body, so they are excluded by joint identity rather than by name.
  assert.match(source, /skip\[jointHandles\[#jointHandles\]\]=true/);
  assert.match(source, /skip\[jointHandles\[#jointHandles-1\]\]=true/);
  assert.match(source, /not skip\[womatSelfCollisionLinkOf\(shape\)\]/);
});
test("the tool frame is defined once, with Y running down the tool", { skip: !luaAvailable && "lua interpreter not installed" }, () => {
  // The marker visibly carries the gripper's convention -- Y along the tool --
  // so a top-down pick is "the TCP's Y points along world -Z". The axes are a
  // relabelling of the frame womatApproachFrame already computes, not a second
  // Gram-Schmidt: X stays X, Y becomes its Z, Z becomes -Y to stay right-handed.
  const frame = (() => {
    const a = source.indexOf("function womatApproachFrame(axis, rollRad)");
    return source.slice(a, source.indexOf("\nfunction ", a + 1));
  })();
  const helper = source.slice(
    source.indexOf("sim.setObjectMatrix(helper, marker, {"),
    source.indexOf("})", source.indexOf("sim.setObjectMatrix(helper, marker, {"))
  );

  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-frame-"));
  const file = path.join(dir, "run.lua");
  fs.writeFileSync(file, [
    frame,
    "local function n(v) return math.sqrt(v[1]^2+v[2]^2+v[3]^2) end",
    "local function d(a,b) return a[1]*b[1]+a[2]*b[2]+a[3]*b[3] end",
    "local function x(a,b) return {a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3], a[1]*b[2]-a[2]*b[1]} end",
    "local function eq(a,b) return math.abs(a-b)<1e-9 end",
    "local bad = 0",
    "for _, tcp in ipairs({ {0,0.1871,0}, {0.0869,-0.1657,0.0035}, {0.1871,0,0} }) do",
    "  local fx, fy, fz = womatApproachFrame(tcp)",
    "  local X, Y, Z = fx, fz, {-fy[1],-fy[2],-fy[3]}",
    "  local tool = {tcp[1]/n(tcp), tcp[2]/n(tcp), tcp[3]/n(tcp)}",
    "  if not (eq(n(X),1) and eq(n(Y),1) and eq(n(Z),1)) then bad = bad + 1 end",
    "  if not (eq(d(X,Y),0) and eq(d(Y,Z),0) and eq(d(X,Z),0)) then bad = bad + 1 end",
    "  for i=1,3 do if not eq(Y[i],tool[i]) then bad = bad + 1 end end",
    "  local c = x(X,Y)",
    "  for i=1,3 do if not eq(c[i],Z[i]) then bad = bad + 1 end end",
    "end",
    "print(bad)"
  ].join("\n"));
  assert.equal(execFileSync("lua", [file], { encoding: "utf8" }).trim(), "0",
    "marker Y must be the tool direction, orthonormal and right-handed");

  // alpha_beta pins a frame's Z and cannot pin Y, so the IK is driven through a
  // child whose Z IS the marker's Y -- same origin, so position IK is unchanged
  // and the roll about the tool stays free.
  const numbers = helper.match(/-?\d+/g).map(Number);
  const [hX, hY, hZ] = [[0, 4, 8], [1, 5, 9], [2, 6, 10]].map((c) => c.map((i) => numbers[i]));
  assert.deepEqual(hZ, [0, 1, 0], "the IK child's Z must be the marker's Y");
  assert.deepEqual(hX, [1, 0, 0]);
  assert.deepEqual(hY, [0, 0, -1]);
});

test("pose verification and the IK agree on which axis is the tool", () => {
  // Second column of the world matrix is Y. Checking the third would verify an
  // axis perpendicular to the tool and pass poses that point it anywhere.
  assert.match(source, /local axisDot=\(matrix\[2\]\*wanted\[1\]\+matrix\[6\]\*wanted\[2\]\+matrix\[10\]\*wanted\[3\]\)\/length/);
  // Both IK elements steer the tool frame, not the marker directly.
  assert.match(source, /env, group, simBase, womatIkTip\(pickTip\), simTarget/);
  assert.match(source, /env, group, simBase, womatIkTip\(handoverTip\), simTarget/);
  // One definition of the frame, reached from both the build and adopt paths.
  assert.equal((source.match(/womatOrientToolFrame\(/g) || []).length, 3);
});
