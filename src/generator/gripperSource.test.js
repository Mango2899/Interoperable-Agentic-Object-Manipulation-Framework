"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const { GRIPPER_SOURCES, gripperSource, toolControlPointOffset } = require("./core/luaDriver");
const { createContext } = require("./core/context");
const { prepareRuntimeDriverScript } = require("./core/driverScript");
const { buildWorkspaceGripperPreamble } = require("./robwotBridge");

const root = path.resolve(__dirname, "../..");

test("where the tool comes from is resolved one way for both halves", () => {
  assert.deepEqual(GRIPPER_SOURCES, ["model", "builtin", "none"]);
  // Unstated means a loaded model, which is every robot the cell had before
  // an arm arrived with its own end effector.
  assert.equal(gripperSource({}), "model");
  assert.equal(gripperSource({ source: "builtin" }), "builtin");
  assert.equal(gripperSource({ source: "none" }), "none");
  // shipsWithModel is the older spelling; the adopted Dobot is written that way.
  assert.equal(gripperSource({ shipsWithModel: true }), "builtin");
  assert.throws(() => gripperSource({ source: "builtIn" }), /must be one of model, builtin, none/);
});

test("a declared tool control point is three numbers or nothing", () => {
  assert.equal(toolControlPointOffset({}), null);
  assert.deepEqual(toolControlPointOffset({ toolControlPointOffset: [0.063, 0, -0.049] }), [0.063, 0, -0.049]);
  assert.throws(
    () => toolControlPointOffset({ toolControlPointOffset: [0.1, 0] }),
    /must be three numbers/
  );
});

test("a built-in tool is never loaded and never reparented", () => {
  // Both halves matter and for different reasons. Reparenting is the dangerous
  // one: the tool is a LINK of the arm's kinematic chain, so bolting it onto
  // the flange dummy the way a loaded gripper is bolted takes it out of that
  // chain -- the arm then drives one link short and the tool is towed by a
  // dummy nothing drives.
  const preamble = buildWorkspaceGripperPreamble({
    tcpAlias: "R_tcp",
    gripperModelName: "R_tcp_gripper",
    gripperModelPath: "",
    gripperModelPosition: [0, 0, 0],
    gripperModelOrientation: [0, 0, 0],
    gripperModelLocalRotation: [0, 0, 0],
    graspPadAlias: "pad",
    source: "builtin",
    sourceAlias: "Link8_respondable",
    tcpOffset: [0.063, 0, -0.049]
  });
  assert.doesNotMatch(preamble, /sim\.loadModel/, "nothing is loaded for a built-in tool");
  assert.doesNotMatch(preamble, /sim\.setObjectParent\(gripper/, "the tool stays in the arm's chain");
  assert.match(preamble, /workspace_builtin_gripper_not_found: Link8_respondable/);
  // With no pads to measure, the declared point is used rather than erroring.
  assert.match(preamble, /pads=sim\.multiplyVector/);
  assert.doesNotMatch(preamble, /workspace_gripper_contact_pads_not_found/);
});

test("a loaded tool still takes exactly the path it took before", () => {
  const preamble = buildWorkspaceGripperPreamble({
    tcpAlias: "R_tcp",
    gripperModelName: "R_tcp_gripper",
    gripperModelPath: "/models/RG2.ttm",
    gripperModelPosition: [0, 0, 0],
    gripperModelOrientation: [-Math.PI / 2, 0, 0],
    gripperModelLocalRotation: [0, 0, 0],
    graspPadAlias: "touch"
  });
  assert.match(preamble, /sim\.loadModel/);
  assert.match(preamble, /sim\.setObjectParent\(gripper, flange, false\)/);
  // No declared offset, so an empty pad scan is still a hard failure: for a
  // jawed gripper it means the alias is wrong, and sampling the flange instead
  // would look like a good workspace and be wrong by a whole tool length.
  assert.match(preamble, /workspace_gripper_contact_pads_not_found: touch/);
});

test("every robot in the cell renders a driver, and only Robot5 adopts its own tool", () => {
  const expected = {
    Robot1: "model", Robot2: "model", Robot3: "model", Robot4: "model", Robot5: "builtin"
  };
  for (const [name, source] of Object.entries(expected)) {
    const manifestPath = path.join("generated", "robots", name, "manifest.json");
    if (!fs.existsSync(path.join(root, manifestPath))) continue;
    const ctx = createContext(manifestPath);
    assert.equal(gripperSource(ctx.manifest.runtime.gripper), source, `${name} gripper source`);
    const driver = prepareRuntimeDriverScript(ctx, {});
    assert.match(driver, new RegExp(`womatGripperSource=\\[\\[${source}\\]\\]`), `${name} driver`);
  }
});

test("Robot5 adopts the swiftpro's own end effector instead of a bolted-on RG2", () => {
  const gripper = JSON.parse(
    fs.readFileSync(path.join(root, "generated/robots/Robot5/manifest.json"), "utf8")
  ).runtime.gripper;
  assert.equal(gripper.source, "builtin");
  assert.equal(gripper.sourceAlias, "Link8_respondable");
  // No file to load, so no path to name. A leftover RG2 path here is the bug
  // this replaced: the arm was sampled with a gripper it does not have.
  assert.equal(gripper.modelPath, undefined);
  // Gripper.STL is a housing with no jaws and no joint, so there is nothing to
  // measure a tool control point from and it is declared instead: the centroid
  // of its downward-facing business end, in Link8's own frame.
  assert.deepEqual(gripper.toolControlPointOffset, [0.063, 0, -0.049]);
  assert.deepEqual(gripper.toolAxis, [0, 0, -1], "a uArm points its tool straight down");
});
