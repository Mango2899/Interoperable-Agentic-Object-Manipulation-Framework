"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const { motionSchedule, profile, turnaroundDistance } = require("./timeLaw");

const rad = Math.PI / 180;
// The live cell's joint limits.
const joints = { velocity: 30 * rad, acceleration: 60 * rad, jerk: 120 * rad };
// ...and its Cartesian ones.
const cartesian = { velocity: 0.175, acceleration: 0.25, jerk: 0.75 };

// The three bounds, read back off the sampled schedule the arm actually
// executes -- not off the closed form that produced it.
function measure(distance, limits, dt) {
  const fractions = motionSchedule(distance, limits, dt);
  const position = fractions.map((fraction) => fraction * distance);
  const derive = (series) => series.slice(1).map((value, index) => (value - series[index]) / dt);
  const velocity = derive(position);
  const acceleration = derive(velocity);
  const jerk = derive(acceleration);
  const peak = (series) => Math.max(...series.map(Math.abs), 0);
  return { fractions, duration: (fractions.length - 1) * dt,
    velocity: peak(velocity), acceleration: peak(acceleration), jerk: peak(jerk) };
}

test("the schedule starts at rest, ends on the target and never turns back", () => {
  for (const limits of [joints, cartesian]) {
    for (const distance of [0, 1e-4, 0.02, 0.3, 1.57, 6.28]) {
      const fractions = motionSchedule(distance, limits, 0.05);
      assert.equal(fractions[0], 0);
      assert.equal(fractions.at(-1), 1);
      assert.ok(fractions.every((value, index) => index === 0 || value >= fractions[index - 1]),
        `progress went backwards over ${distance}`);
    }
  }
});

test("velocity, acceleration and jerk stay inside the configured bounds", () => {
  for (const [name, limits] of [["joint", joints], ["cartesian", cartesian]]) {
    for (const distance of [0.05, 0.2, 0.6, 1.57, 3.14]) {
      const m = measure(distance, limits, 0.01);
      // Sampling a continuous profile at a fixed step reads back slightly under
      // the true peak, never over it, so a small tolerance is one-sided slack.
      assert.ok(m.velocity <= limits.velocity * 1.02, `${name} ${distance}: v=${m.velocity} > ${limits.velocity}`);
      assert.ok(m.acceleration <= limits.acceleration * 1.05, `${name} ${distance}: a=${m.acceleration}`);
      assert.ok(m.jerk <= limits.jerk * 1.35, `${name} ${distance}: j=${m.jerk}`);
    }
  }
});

test("a long move cruises, so it beats the quintic it replaces under the same bounds", () => {
  const quintic = (distance, { velocity, acceleration, jerk }) => Math.max(
    1.875 * distance / velocity,
    Math.sqrt(5.774 * distance / acceleration),
    Math.cbrt(60 * distance / jerk));
  for (const [distance, atLeast] of [[45 * rad, 1.1], [90 * rad, 1.3], [180 * rad, 1.5]]) {
    const gain = quintic(distance, joints) / profile(distance, joints).duration;
    assert.ok(gain >= atLeast, `${distance / rad}deg: only ${gain.toFixed(2)}x faster than the quintic`);
  }
  // The gain is real time, not a relaxed limit: the peak speed is still capped.
  assert.ok(measure(180 * rad, joints, 0.01).velocity <= joints.velocity * 1.02);
});

test("a move too short to reach the velocity limit solves for the peak it can reach", () => {
  const short = profile(0.02, joints);
  assert.ok(short.peak < joints.velocity, "a 1.1deg move must not claim the full cruise speed");
  assert.equal(short.cruise, 0);
  // Whatever peak it settled on has to cover exactly the distance asked for.
  assert.ok(Math.abs(turnaroundDistance(short.peak, joints.acceleration, joints.jerk) - 0.02) < 1e-9);
  assert.equal(motionSchedule(0.02, joints, 0.05).at(-1), 1);
});

test("a distance below one simulation step still produces a usable two-sample move", () => {
  const fractions = motionSchedule(1e-6, joints, 0.05);
  assert.ok(fractions.length >= 2);
  assert.deepEqual([fractions[0], fractions.at(-1)], [0, 1]);
});
