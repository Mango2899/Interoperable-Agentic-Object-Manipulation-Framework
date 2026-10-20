"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const { MotionLog } = require("./motionLog");

function logTo(file) {
  return new MotionLog({ robot: "Robot5", enabled: true, file, run: "test" });
}

function lines(file) {
  return fs.readFileSync(file, "utf8").trim().split("\n").filter(Boolean).map(JSON.parse);
}

test("a finished motion records which route it actually took", () => {
  // Without this, a placement that interpolated straight to the support and one
  // that needed a four-second OMPL search are the same line in the log -- so
  // "the terminal arm almost always falls back to the planner" stays an
  // impression instead of becoming a number, and a fix for it cannot be shown
  // to have worked.
  const file = path.join(fs.mkdtempSync(path.join(os.tmpdir(), "motionlog-")), "motion.jsonl");
  const log = logTo(file);

  const straight = log.begin("placeCubeAt", { goal: "v_Robot5_grip_support_b" });
  straight.granted();
  straight.finished({ success: true, planner: "top_down_release", approachRoute: "joint_interpolation" });

  const searched = log.begin("placeCubeAt", { goal: "v_Robot5_grip_support_b" });
  searched.granted();
  searched.finished({ success: true, planner: "top_down_release", approachRoute: "planned_path" });

  const finished = lines(file).filter((row) => row.event === "motion_finished");
  assert.deepEqual(finished.map((row) => row.approachRoute), ["joint_interpolation", "planned_path"]);
  assert.deepEqual(finished.map((row) => row.planner), ["top_down_release", "top_down_release"]);
  assert.deepEqual(finished.map((row) => row.success), [true, true]);
});

test("a motion that took neither route says so as null, not as a missing field", () => {
  // A refused or unplannable motion has no route, and a consumer counting
  // fallbacks must be able to tell "no route" from "field not written yet".
  const file = path.join(fs.mkdtempSync(path.join(os.tmpdir(), "motionlog-")), "motion.jsonl");
  const log = logTo(file);
  const attempt = log.begin("placeCubeAt");
  attempt.granted();
  attempt.finished({ success: false, reason: "cartesian_goal_config_unavailable" });
  const [row] = lines(file).filter((entry) => entry.event === "motion_finished");
  assert.equal(row.success, false);
  assert.equal(row.reason, "cartesian_goal_config_unavailable");
  assert.ok("planner" in row && row.planner === null);
  assert.ok("approachRoute" in row && row.approachRoute === null);
});

test("a write that fails mid-run disables the log instead of stopping the arm", () => {
  // The guarantee is about writes during motion, not about configuration: a bad
  // path fails loudly at startup, which is the right time for it. What must never
  // throw is the append itself, because it happens between two commanded moves.
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), "motionlog-"));
  const log = logTo(path.join(directory, "motion.jsonl"));
  const first = log.begin("placeCubeAt");
  first.granted();
  first.finished({ success: true, approachRoute: "joint_interpolation" });
  assert.equal(lines(path.join(directory, "motion.jsonl")).length, 3);

  // Now the destination becomes unwritable under it -- a directory where a file
  // was -- which is what an append failure looks like from inside record().
  log.file = directory;
  const second = log.begin("placeCubeAt");
  assert.doesNotThrow(() => {
    second.granted();
    second.finished({ success: true, approachRoute: "planned_path" });
  });
  assert.equal(log.enabled, false, "the log takes itself out rather than retry every motion");
});
