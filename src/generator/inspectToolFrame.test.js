const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");
const { execFileSync, spawnSync } = require("node:child_process");

const { buildInspectionLua } = require("./inspectToolFrame");

const root = path.resolve(__dirname, "../..");
const luaAvailable = spawnSync("lua", ["-v"], { stdio: "ignore" }).status === 0;

const grip = {
  modelPosition: [0, 0, 0],
  modelOrientation: [-1.57079632679, 0, 0],
  modelLocalRotation: [0, 0, 0]
};

function lua({ repair }) {
  return buildInspectionLua({
    name: "Robot1",
    gripperAlias: "Robot1_tcp_gripper",
    padAlias: "touch",
    grip,
    repair
  });
}

test("a Lua fault comes back as a message, not as 'operation failed'", () => {
  // The remote API reports a failed executeScriptString with a line number into
  // a string the caller never sees, which is the least useful form a diagnostic
  // can take. Wrapping the body means the error arrives as data.
  for (const repair of [false, true]) {
    const source = lua({ repair });
    assert.match(source, /^local ok, result=pcall\(function\(\)/);
    assert.match(source, /if not ok then return \{error=tostring\(result\)\} end/);
  }
});

test("the inspection script runs, and its repair restores the tool frame", { skip: !luaAvailable && "lua interpreter not installed" }, () => {
  // Executed against a scene-graph stub rather than CoppeliaSim: the arithmetic
  // that decides where the flange, gripper and tool control point end up does
  // not need a simulator, and checking it here is what turns a round trip
  // through the running scene into a deterministic test.
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-toolframe-"));
  const script = path.join(dir, "run.lua");
  fs.writeFileSync(script, [
    `local env=dofile(${JSON.stringify(path.join(root, "src/generator/testdata/simStub.lua"))})`,
    "local report=(function()",
    lua({ repair: true }),
    "end)()",
    "if report.error then print('ERROR '..report.error) os.exit(1) end",
    "local function d(a,b) return math.sqrt((a[1]-b[1])^2+(a[2]-b[2])^2+(a[3]-b[3])^2) end",
    "print(string.format('%.4f %.4f %.4f %d',",
    "    d(report.tip.position, report.gripper.position),",
    "    d(report.tip.position, report.pads),",
    "    d(report.marker.position, report.pads),",
    "    #report.repaired))"
  ].join("\n"));

  const out = execFileSync("lua", [script], { cwd: root, encoding: "utf8" }).trim();
  assert.doesNotMatch(out, /^ERROR/, out);
  const [flangeToGripper, flangeToPads, markerToPads, repairs] = out.split(/\s+/).map(Number);

  // The stub starts from the scene the inspector actually reported: gripper
  // hanging off the tool control point, so the pads sit at twice the tool
  // length. After repair:
  assert.ok(flangeToGripper < 1e-6, `gripper must end up on the flange, got ${flangeToGripper}`);
  assert.ok(Math.abs(flangeToPads - 0.1871) < 1e-6, `pads must be one tool length out, got ${flangeToPads}`);
  assert.ok(markerToPads < 1e-6, `the marker must BE the tool control point, got ${markerToPads}`);
  assert.ok(repairs >= 3, "removing the stale marker, remounting and rebuilding are all needed");
});

test("without --repair the scene is only measured", () => {
  const readOnly = lua({ repair: false });
  for (const mutation of ["sim.setObjectParent(gripper", "sim.removeObject", "sim.removeModel", "sim.createDummy"]) {
    assert.ok(!readOnly.includes(mutation), `inspection must not ${mutation}`);
  }
  assert.ok(lua({ repair: true }).includes("sim.setObjectParent(gripper, flange, false)"));
});

test("workspace sampling attaches the gripper and samples the jaws", { skip: !luaAvailable && "lua interpreter not installed" }, () => {
  // Same offline execution as above, applied to the generation side. A cloud
  // sampled from the flange looks exactly like a good one and is wrong by a
  // whole tool length, so this cannot be left to a round trip through a
  // 40,000-step CoppeliaSim run to discover.
  const { buildWorkspaceGripperPreamble } = require("./robwotBridge");
  const preamble = buildWorkspaceGripperPreamble({
    tcpAlias: "Robot1_tcp",
    gripperModelName: "Robot1_tcp_gripper",
    gripperModelPath: "/models/RG2.ttm",
    gripperModelPosition: [0, 0, 0],
    gripperModelOrientation: [-1.57079632679, 0, 0],
    gripperModelLocalRotation: [0, 0, 0],
    graspPadAlias: "touch"
  });

  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-workspace-"));
  const script = path.join(dir, "run.lua");
  fs.writeFileSync(script, [
    `local env=dofile(${JSON.stringify(path.join(root, "src/generator/testdata/simStub.lua"))})`,
    preamble,
    "womatWorkspaceAttachGripperAndRetargetTip()",
    "local tcp=womatWorkspaceToolControlPoint()",
    "local function d(a,b) return math.sqrt((a[1]-b[1])^2+(a[2]-b[2])^2+(a[3]-b[3])^2) end",
    "local pads=womatWorkspacePadMidpoint(env.gripper)",
    "print(string.format('%.4f %.4f %.4f %s %s',",
    "    d(sim.getObjectPosition(env.tip), sim.getObjectPosition(env.gripper)),",
    "    d(sim.getObjectPosition(env.tip), sim.getObjectPosition(tcp)),",
    "    d(sim.getObjectPosition(tcp), pads),",
    "    tostring(sim.getObjectParent(tcp) == env.tip),",
    "    tostring(sim.getObjectParent(env.gripper) == env.tip)))"
  ].join("\n"));

  const out = execFileSync("lua", [script], { cwd: root, encoding: "utf8" }).trim();
  const [flangeToGripper, flangeToTcp, tcpToPads, tcpOnFlange, gripperOnFlange] = out.split(/\s+/);

  // The stub starts from the broken scene, so this also proves generation
  // repairs a mis-mounted gripper rather than sampling through it.
  assert.ok(Number(flangeToGripper) < 1e-6, `gripper must be bolted to the flange, got ${flangeToGripper}`);
  assert.ok(Math.abs(Number(flangeToTcp) - 0.1871) < 1e-6, `sampled point must be one tool length out, got ${flangeToTcp}`);
  assert.ok(Number(tcpToPads) < 1e-6, `the sampled point must BE the jaw midpoint, got ${tcpToPads}`);
  assert.equal(tcpOnFlange, "true");
  assert.equal(gripperOnFlange, "true");
});

test("the pipeline gripper install leaves the gripper on the flange", { skip: !luaAvailable && "lua interpreter not installed" }, () => {
  // The install runs AFTER the driver's init has rebound simTip to the tool
  // control point. Mounting against simTip there is what put the gripper a
  // whole tool length out, sitting exactly on the TCP marker -- so this
  // reproduces that ordering rather than calling the install in isolation.
  const { buildWorkspaceGripperPreamble, buildPipelineGripperLua } = require("./robwotBridge");
  const preamble = buildWorkspaceGripperPreamble({
    tcpAlias: "Robot1_tcp",
    gripperModelName: "Robot1_tcp_gripper",
    gripperModelPath: "/models/RG2.ttm",
    gripperModelPosition: [0, 0, 0],
    gripperModelOrientation: [-1.57079632679, 0, 0],
    gripperModelLocalRotation: [0, 0, 0],
    graspPadAlias: "touch"
  });

  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-install-"));
  const install = path.join(dir, "install.lua");
  fs.writeFileSync(install, buildPipelineGripperLua({
    modelName: "Robot1_tcp_gripper",
    openJointPositions: [0.06],
    baseOrientation: [0, 0, 0]
  }));

  const script = path.join(dir, "run.lua");
  fs.writeFileSync(script, [
    `local env=dofile(${JSON.stringify(path.join(root, "src/generator/testdata/simStub.lua"))})`,
    preamble,
    "-- What the driver's init does, in order.",
    "womatWorkspaceAttachGripperAndRetargetTip()",
    "simTip=womatWorkspaceToolControlPoint()",
    `local ok, handle=dofile(${JSON.stringify(install)})`,
    "local function d(a,b) return math.sqrt((a[1]-b[1])^2+(a[2]-b[2])^2+(a[3]-b[3])^2) end",
    "local tcp=womatWorkspaceToolControlPoint()",
    "local grippers=0",
    "for h, o in pairs(env.scene) do",
    "    if not o.removed and o.alias == 'Robot1_tcp_gripper' then grippers=grippers+1 end",
    "end",
    "print(string.format('%s %.4f %.4f %s %d',",
    "    tostring(ok),",
    "    d(sim.getObjectPosition(env.tip), sim.getObjectPosition(handle)),",
    "    d(sim.getObjectPosition(env.tip), sim.getObjectPosition(tcp)),",
    "    tostring(sim.getObjectParent(handle) == env.tip),",
    "    grippers))"
  ].join("\n"));

  const out = execFileSync("lua", [script], { cwd: root, encoding: "utf8" }).trim();
  const [ok, flangeToGripper, flangeToTcp, onFlange, grippers] = out.split(/\s+/);

  assert.equal(ok, "true", "the install must report success");
  assert.ok(Number(flangeToGripper) < 1e-6, `the gripper bolts to the flange, got ${flangeToGripper} m out`);
  assert.equal(onFlange, "true", "the gripper's parent must be the flange, not the TCP marker");
  assert.ok(Math.abs(Number(flangeToTcp) - 0.1871) < 1e-6, `the marker stays one tool length out, got ${flangeToTcp}`);
  // Reusing the mounted gripper rather than loading a second one: two objects
  // sharing an alias means the wrong one can be the one that gets measured.
  assert.equal(Number(grippers), 1, "exactly one gripper may carry the alias");
});

test("the workspace treats the arm and its gripper as one body", () => {
  const bridgeSrc = fs.readFileSync(require.resolve("./robwotBridge"), "utf8");

  // Collision: the robot collection is the robot tree unioned with the
  // gripper's own shapes, and a gripper that is missing or shapeless stops
  // generation rather than producing a cloud that ignores it.
  assert.match(bridgeSrc, /local robotShapes=sim\.getObjectsInTree\(base, sim\.object_shape_type, 0\)/);
  assert.match(bridgeSrc, /error\("workspace_gripper_missing/);
  assert.match(bridgeSrc, /error\("workspace_gripper_has_no_collision_shapes/);

  // Sampling: both places the vendor driver resolves its tip are redirected to
  // the tool control point.
  assert.match(bridgeSrc, /simTip=womatWorkspaceToolControlPoint\(\)/);
  assert.match(bridgeSrc, /local tip=womatWorkspaceToolControlPoint\(\)/);

  // And no silent fallbacks: sampling the wrong point is worse than not
  // sampling, so each failure mode raises.
  for (const fault of [
    "workspace_flange_dummy_not_found",
    "workspace_gripper_model_load_failed",
    "workspace_gripper_contact_pads_not_found",
    "workspace_tool_control_point_missing"
  ]) {
    assert.match(bridgeSrc, new RegExp(`error\\('${fault}`), `${fault} must stop generation`);
  }
});
