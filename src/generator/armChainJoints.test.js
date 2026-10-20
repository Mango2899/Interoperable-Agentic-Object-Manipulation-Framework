const assert = require("node:assert/strict");
const test = require("node:test");
const fs = require("node:fs");
const path = require("node:path");

const { selectArmChainJoints } = require("./core/luaDriver");

const VENDOR_DRIVER = path.resolve(
  __dirname, "..", "..", "external", "RobWoT",
  "Library", "GenerateRobotDescription", "robot_driver_workspace.lua"
);

test("the vendor six-joint cutoff is replaced, not appended to", () => {
  // A five-axis arm carrying a gripper has the gripper's finger as its sixth
  // joint in tree order. Left in place, the cutoff hands that finger to the IK
  // solver as an arm axis and no orientation-constrained pose ever converges.
  const raw = fs.readFileSync(VENDOR_DRIVER, "utf8");
  assert.ok(raw.includes("if i==6 then"), "precondition: the vendor file still has the cutoff");

  const patched = selectArmChainJoints(raw);
  assert.ok(patched.includes("womatOnChain"), "the chain-walking selection is present");
  // The cutoff survives only as the no-tip-dummy fallback, inside the guard.
  const fallbackIndex = patched.indexOf("if #newjointHandles == 0 then");
  const cutoffIndex = patched.indexOf("if i==6 then");
  assert.ok(fallbackIndex > 0 && cutoffIndex > fallbackIndex,
    "the six-joint cutoff remains only as the fallback for a model with no tip");
});

test("a driver template without the expected block fails loudly", () => {
  // Silently returning the vendor text would put the finger back in the arm
  // and cost another full debugging pass to rediscover.
  assert.throws(
    () => selectArmChainJoints("-- some other driver\n"),
    /no longer contains the expected joint-selection block/
  );
});

test("selection keeps tree order, which is base-to-tip order", () => {
  const patched = selectArmChainJoints(fs.readFileSync(VENDOR_DRIVER, "utf8"));
  // Appending in the order getObjectsInTree returned them is what keeps
  // jointHandles[1] the base joint; the IK element and every joint-space
  // command downstream depend on that ordering.
  assert.ok(patched.includes("newjointHandles[#newjointHandles+1] = jointHandles[i]"));
});
