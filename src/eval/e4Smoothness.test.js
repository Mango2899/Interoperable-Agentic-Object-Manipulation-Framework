const test = require("node:test");
const assert = require("node:assert/strict");

const { motionSchedule } = require("../generator/core/timeLaw");
const {
  activeWindow,
  analyzeTrace,
  handoverCandidates,
  leadingJoint,
  linearJointPath,
  linearSchedule,
  settledIndex,
  thesisFigure,
  thesisTable
} = require("./e4Smoothness");

const DEG = 180 / Math.PI;
const JOINT = { velocity: 30 / DEG, acceleration: 60 / DEG, jerk: 120 / DEG };

// A trace as the recorder writes it: padding at rest, the path, padding at the goal.
function traceOf(path, dof, pad = 5) {
  const rows = [...Array(pad).fill(path[0]), ...path, ...Array(pad).fill(path[path.length - 1])];
  const flat = rows.flat();
  const tool = rows.flatMap((row) => [row[0], 0, 0]);
  return { t: rows.map((_, k) => k * 0.05), qc: flat, qm: flat, pc: tool, pm: tool, errors: 0 };
}

test("linearSchedule moves at constant velocity and ends on the target", () => {
  const fractions = linearSchedule(1, 0.25, 0.05);
  assert.equal(fractions[0], 0);
  assert.equal(fractions[fractions.length - 1], 1);
  assert.equal(fractions.length, 81);
  assert.ok(Math.abs(fractions[1] - 0.0125) < 1e-12);
  assert.deepEqual(linearSchedule(0, 0.25, 0.05), [0, 1]);
});

test("linearJointPath is sized by the joint that travels furthest", () => {
  const path = linearJointPath([0, 0], [0.5, -1], 0.5, 0.1);
  assert.equal(path.length, 21);
  assert.deepEqual(path[path.length - 1], [0.5, -1]);
  assert.ok(Math.abs(path[1][1] + 0.05) < 1e-12);
});

test("activeWindow and settledIndex bracket the motion", () => {
  const rows = [[0], [0], [0.1], [0.3], [0.3], [0.3]];
  assert.deepEqual(activeWindow(rows), { first: 2, last: 3 });
  assert.equal(activeWindow([[1], [1]]), null);
  assert.equal(settledIndex([[0], [0.2], [0.29], [0.3]], [0.3], 0.02), 2);
  assert.equal(settledIndex([[0], [0.1]], [0.3], 0.02), null);
});

test("leadingJoint picks the joint with the largest change", () => {
  assert.equal(leadingJoint([[0, 0, 0], [0.1, -0.5, 0.2]]), 1);
});

test("the double-S law stays within its limits; the linear law does not", () => {
  const dof = 2;
  const start = [0, 0];
  const target = [1.2, -0.6];
  const smooth = motionSchedule(1.2, JOINT, 0.05).map((f) => start.map((v, j) => v + (target[j] - v) * f));
  const linear = linearJointPath(start, target, JOINT.velocity, 0.05);
  const options = { dof, dt: 0.05, leg: "home", limits: JOINT, lead: 0, tolerance: 1 / DEG };
  const s = analyzeTrace(traceOf(smooth, dof), options);
  const l = analyzeTrace(traceOf(linear, dof), options);
  assert.ok(s.commandedVelRatio <= 1.001, `velocity ${s.commandedVelRatio}`);
  assert.ok(s.commandedAccRatio <= 1.001, `acceleration ${s.commandedAccRatio}`);
  assert.ok(s.commandedJerkRatio <= 1.001, `jerk ${s.commandedJerkRatio}`);
  assert.ok(Math.abs(l.commandedVelRatio - 1) < 1e-6);
  assert.ok(l.commandedAccRatio > 9.9);
  assert.ok(l.commandedJerkRatio > 99);
  assert.ok(s.durationS > l.durationS);
  assert.equal(s.series[0].vel, 0);
});

test("handoverCandidates orders voxels by distance to the centroid", () => {
  const csv = "X,Y,Z,RobotCount\n0,0,0,2\n1,0,0,2\n0.5,0,0,2\n";
  const candidates = handoverCandidates(csv);
  assert.deepEqual(candidates.map((c) => c.x), [0.5, 0, 1]);
});

test("table and figure render for the legs present", () => {
  const limits = { joint: { velocity: 30, acceleration: 60, jerk: 120 }, cartesian: { velocity: 0.25, acceleration: 0.4, jerk: 1.2 } };
  const table = thesisTable([{ leg: "home", variant: "double_s", durationS: 3, commandedJointVelDeg: 30,
    measuredJointVelDeg: 29, commandedJointAccDeg: 60, measuredJointAccDeg: 61, commandedJointJerkDeg: 120, measuredJointJerkDeg: 300 }], limits);
  assert.match(table, /Return home & Double-S & 3\.00 & 30\.0 & 60\.0 \/ -- & 120 \/ --/);
  const figure = thesisFigure("home", limits, ["double_s", "linear"], 4);
  assert.match(figure, /lead\/home_linear\.csv/);
  assert.match(figure, /xmax=4\.00/);
});
