const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const root = path.resolve(__dirname, "../..");

test("Robot3 bolts its tool straight down its wrist axis", () => {
  // Was "applies its 180-degree tool turn in TCP-local coordinates", against a
  // mount of Rx(-90) plus a local Rz(180). Those numbers were fitted to a
  // flange dummy that pointed at the WORLD rather than down the wrist -- see
  // womatAlignFlangeToWrist -- so they described a frame that no longer exists.
  // With squareToWristJoint the tool is bolted against the axis the wrist rolls
  // about, and the RG2 approaches along its own +Z, so the mount is the identity
  // and the control point lands one tool length straight out. The quarter turn
  // and the local half turn this used to carry were fitted to the orientation
  // the CR5's 'tip' dummy happened to inherit, not to anything in the geometry.
  const manifest = JSON.parse(fs.readFileSync(path.join(root, "generated/robots/Robot3/manifest.json"), "utf8"));
  const gripper = manifest.runtime.gripper;
  assert.deepEqual(gripper.modelOrientation, [0, 0, 0]);
  assert.deepEqual(gripper.modelLocalRotation, [0, 0, 0]);
  assert.equal(gripper.applyBaseOrientationCorrection, false);
  // No offset length: the robot's tool control point is the midpoint of its
  // gripper jaws, so a commanded position is where the object goes.
  assert.equal(gripper.jawCenterOffset, undefined);
  // toolAxis is MEASURED off the pads by the pipeline, so its direction is only
  // settled by the next run against the corrected flange. Pinning a pre-run
  // direction here is what made this test outlive the frame it was fitted to.
  assert.ok(Math.abs(Math.hypot(...gripper.toolAxis) - 1) < 1e-3, "tool axis must be a unit direction");
});

test("pipeline and runtime post-multiply the TCP-local gripper rotation", () => {
  for (const relative of ["src/generator/robwotBridge.js", "src/generator/lowerThingServer.js"]) {
    const source = relative.endsWith("lowerThingServer.js")
      // The runtime half of this rule now lives in the assembled Lua driver.
      ? require("./core/runtimeSource").runtimeSource()
      : fs.readFileSync(path.join(root, relative), "utf8");
    assert.match(source, /sim\.multiplyMatrices\(currentLocal,extraLocal\)/);
  }
});

test("a mount angle past a full turn is rejected as degrees", () => {
  const { radiansTriple } = require("./core/luaDriver");
  // The units these fields are written in are not recoverable from the value
  // alone, with one exception: no mount needs more than a full turn, so a
  // number past 2*pi can only be degrees that were never converted.
  assert.deepEqual(radiansTriple([-Math.PI / 2, 0, 0], "x"), [-Math.PI / 2, 0, 0]);
  assert.deepEqual(radiansTriple([0, 0, Math.PI], "x"), [0, 0, Math.PI]);
  assert.throws(
    () => radiansTriple([110, 0, 0], "runtime.gripper.modelLocalRotation"),
    /modelLocalRotation is in radians.*looks like degrees/s,
    "110 meant as degrees is 17 turns plus 182.5 degrees, and lands almost-but-not-quite flipped"
  );
});

test("Robot4 is fitted with the Baxter gripper's own measurements, not the RG2's", () => {
  const manifest = JSON.parse(fs.readFileSync(path.join(root, "generated/robots/Robot4/manifest.json"), "utf8"));
  const gripper = manifest.runtime.gripper;
  assert.match(gripper.modelPath, /Baxter gripper\.ttm$/);
  // The Baxter gripper's pads are leftPad and righPad -- the typo is the
  // shipped model's. Nothing in its tree contains "touch", so the RG2 default
  // fails the pipeline at workspace estimation and names only the alias.
  assert.equal(gripper.graspPadAlias, "pad");
  // Measured on Robot4's own flange: the pad midpoint is 0.0774 m out, less
  // than half the RG2's 0.19.
  assert.ok(
    Math.abs(gripper.toolLengthEstimate - 0.077) < 0.005,
    `Baxter pads sit ~77 mm from the flange, manifest says ${gripper.toolLengthEstimate}`
  );
  // Its pads are on the model's +Z exactly as the RG2's are, and the CR3 model
  // ships a flange square with its wrist joint, so the mount is the identity.
  assert.deepEqual(gripper.modelOrientation, [0, 0, 0]);
  assert.deepEqual(gripper.modelLocalRotation, [0, 0, 0]);
  // toolAxis is measured off the pads by the pipeline against whatever flange
  // the model carries, so only its unit length is a property of the manifest.
  assert.ok(Math.abs(Math.hypot(...gripper.toolAxis) - 1) < 1e-3, "tool axis must be a unit direction");
});

test("the pipeline and the runtime pick the same shapes out of a gripper", () => {
  // graspPadAlias is a substring match and their midpoint is the tool control
  // point: the pipeline samples the whole workspace at it and the runtime
  // steers it. A render mesh folded into one and not the other moves that
  // point, and the two halves then disagree about where the robot holds things.
  // 'pad' matches the Baxter gripper's leftPad_visible as readily as its leftPad.
  const bridge = fs.readFileSync(path.join(root, "src/generator/robwotBridge.js"), "utf8");
  assert.match(bridge, /womatWorkspaceIsDecorative/);
  assert.match(bridge, /not womatWorkspaceIsDecorative\(alias\)/);
  const runtime = require("./core/runtimeSource").runtimeSource();
  assert.match(runtime, /not womatIsDecorativeShape\(alias\)/);
});

test("a gripper fitted by hand survives regeneration", () => {
  // The manifest is rewritten wholesale, so a value the generator does not read
  // back is silently reset to the RG2's. A Baxter gripper regenerated back to
  // 'touch' pads fails the pipeline at a step that never mentions the gripper.
  const source = fs.readFileSync(path.join(root, "src/generator/generateRobot.js"), "utf8");
  for (const field of ["modelPath", "graspPadAlias", "toolLengthEstimate"]) {
    assert.match(
      source,
      new RegExp(`existingManifest\\?\\.runtime\\?\\.gripper\\?\\.${field}`),
      `${field} must be carried over from an existing manifest`
    );
  }
});

test("every arm mounts its tool against the axis its wrist rolls about", () => {
  // The 'tip' dummy is a mount POINT, not a frame. robot_driver_workspace.lua
  // gives it whatever orientation the last link had, and a URDF visual shape
  // carries its mesh's own origin -- the three CR models land square with their
  // wrist joint by luck, both UR models land twisted. Mounts calibrated against
  // that twist put the jaws out sideways: Robot1's tool control point measured
  // [-0.005, 0.181, 0.048] in its own flange frame instead of straight out, and
  // the workspace was sampled through that same crooked mount.
  //
  // Squaring to the wrist roll axis is what makes the mount a property of the
  // ARM rather than of whatever its exporter left in a dummy, so the same
  // identity mount is right for every arm in the cell. A per-robot angle
  // reappearing here means something is measuring a mesh origin again.
  for (const name of ["Robot1", "Robot2", "Robot3", "Robot4", "Robot5"]) {
    const file = path.join(root, "generated/robots", name, "manifest.json");
    if (!fs.existsSync(file)) continue;
    const gripper = JSON.parse(fs.readFileSync(file, "utf8")).runtime.gripper;
    assert.equal(gripper.squareToWristJoint, true, `${name} must mount off the wrist joint`);
    assert.deepEqual(gripper.modelOrientation, [0, 0, 0], `${name} mount`);
    assert.deepEqual(gripper.modelLocalRotation, [0, 0, 0], `${name} local turn`);
    assert.notEqual(gripper.applyBaseOrientationCorrection, true, `${name} would double-rotate`);
  }
});

test("the sampler and the runtime bolt the tool on the same way", () => {
  // Sampling measures the whole workspace THROUGH the mount, so the two halves
  // disagreeing means an arm is sampled at one tool control point and driven to
  // another -- a workspace that is wrong everywhere and looks fine.
  const { buildWorkspaceGripperPreamble } = require("./robwotBridge");
  const preamble = buildWorkspaceGripperPreamble({
    tcpAlias: "Robot9_tcp",
    gripperModelName: "Robot9_tcp_gripper",
    gripperModelPath: "/tmp/gripper.ttm",
    gripperModelPosition: [0, 0, 0],
    gripperModelOrientation: [0, 0, 0],
    gripperModelLocalRotation: [0, 0, 0],
    applyBaseOrientationCorrection: false,
    squareToWristJoint: true,
    graspPadAlias: "touch",
    source: "model"
  });
  assert.match(preamble, /local squareTo=true and womatWorkspaceWristJoint\(flange\) or -1/);
  assert.match(preamble, /sim\.setObjectOrientation\(gripper, squareTo,/);
  // Position still comes off the flange: that part of the dummy sits on the
  // wrist origin and is correct.
  assert.match(preamble, /sim\.setObjectPosition\(gripper, flange,/);
  const runtime = require("./core/runtimeSource").runtimeSource();
  assert.match(runtime, /womatWristJoint\(womatFlangeTip\)/);
  assert.match(runtime, /sim\.setObjectOrientation\(tcpGripper, squareTo,/);
});
