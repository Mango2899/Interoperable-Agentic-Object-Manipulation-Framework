"use strict";

// How long a reserved path takes, and where along it the arm is at each step.
//
// A reserved path is executed one sample per simulation step, so the sample
// spacing IS the velocity profile: whatever shape is baked in here is what the
// arm does. That is the whole reason this exists separately from the geometry.
//
// It used to be a quintic, `s = t^3(10 - 15t + 6t^2)`, chosen because it is two
// lines and has zero velocity and acceleration at both ends. What it does not
// have is a cruise: its peak speed is 1.875x its average, so respecting a
// velocity limit as a PEAK costs 1.875*d/v for the move, against roughly
// d/v + v/a + a/j for a jerk-limited trapezoid under exactly the same three
// bounds. Measured over the live cell's joint limits (30 deg/s, 60 deg/s^2,
// 120 deg/s^3) that was 1.2x to 1.6x longer for every leg -- and since only the
// reserved-path route (in and out of a handover) is timed here, while every
// other motion goes through sim.moveToConfig's Ruckig profile, that difference
// is exactly the "slow near the handover" the cell shows.
//
// So: the classic seven-phase double-S. Same three bounds, same zero velocity
// and acceleration at both ends, with the cruise the quintic never had.

// Accel-phase duration for a given peak velocity, and whether amax is reached.
// The mean speed over a symmetric accel phase is exactly half its peak, so the
// distance it covers is peak*duration/2 and no separate integration is needed
// to size the profile.
function accelPhase(peak, acceleration, jerk) {
  if (peak * jerk >= acceleration * acceleration) {
    // Long enough to reach amax: ramp up, hold, ramp down.
    return { duration: peak / acceleration + acceleration / jerk, ramp: acceleration / jerk, peakAccel: acceleration };
  }
  // Too short: the two jerk ramps meet before amax.
  const ramp = Math.sqrt(peak / jerk);
  return { duration: 2 * ramp, ramp, peakAccel: jerk * ramp };
}

// Distance covered accelerating from rest to `peak` and back down to rest.
function turnaroundDistance(peak, acceleration, jerk) {
  return peak * accelPhase(peak, acceleration, jerk).duration;
}

// The profile for a rest-to-rest move of `distance`, respecting all three
// bounds. Short moves never reach the velocity limit, so the peak is solved for
// rather than assumed: turnaroundDistance rises monotonically with the peak,
// which is what makes the bisection sound.
function profile(distance, { velocity, acceleration, jerk }) {
  if (!(distance > 0)) return { duration: 0, peak: 0, accel: accelPhase(0, acceleration, jerk), cruise: 0 };
  if (turnaroundDistance(velocity, acceleration, jerk) <= distance) {
    const accel = accelPhase(velocity, acceleration, jerk);
    const cruise = (distance - velocity * accel.duration) / velocity;
    return { duration: 2 * accel.duration + cruise, peak: velocity, accel, cruise };
  }
  let low = 0;
  let high = velocity;
  for (let iteration = 0; iteration < 60; iteration += 1) {
    const middle = (low + high) / 2;
    if (turnaroundDistance(middle, acceleration, jerk) < distance) low = middle; else high = middle;
  }
  const accel = accelPhase(high, acceleration, jerk);
  return { duration: 2 * accel.duration, peak: high, accel, cruise: 0 };
}

// Distance covered by the accel phase at time `t` into it.
function accelDistance(t, peak, { ramp, duration, peakAccel }, jerk) {
  if (t <= 0) return 0;
  if (t >= duration) return peak * duration / 2;
  if (t <= ramp) return jerk * t ** 3 / 6;
  const rampVelocity = jerk * ramp * ramp / 2;
  const rampDistance = jerk * ramp ** 3 / 6;
  const holdEnd = duration - ramp;
  if (t <= holdEnd) {
    const held = t - ramp;
    return rampDistance + rampVelocity * held + peakAccel * held * held / 2;
  }
  const heldFor = holdEnd - ramp;
  const atHoldEnd = rampDistance + rampVelocity * heldFor + peakAccel * heldFor * heldFor / 2;
  const entryVelocity = rampVelocity + peakAccel * heldFor;
  const down = t - holdEnd;
  return atHoldEnd + entryVelocity * down + peakAccel * down * down / 2 - jerk * down ** 3 / 6;
}

function travelled(t, distance, shape, jerk) {
  const { duration, peak, accel, cruise } = shape;
  if (t <= 0) return 0;
  if (t >= duration) return distance;
  if (t <= accel.duration) return accelDistance(t, peak, accel, jerk);
  if (t <= accel.duration + cruise) return peak * accel.duration / 2 + peak * (t - accel.duration);
  return distance - accelDistance(duration - t, peak, accel, jerk);
}

// The move as fractions of the way along it, one per simulation step, starting
// at 0 and ending exactly at 1. Callers interpolate their own endpoints through
// these, so the same schedule serves a joint move and a Cartesian one.
function motionSchedule(distance, limits, dt) {
  const step = Math.max(Number(dt) || 0.05, 1e-6);
  const span = Math.abs(Number(distance) || 0);
  const bounds = {
    velocity: Math.max(Number(limits.velocity) || 0, 1e-9),
    acceleration: Math.max(Number(limits.acceleration) || 0, 1e-9),
    jerk: Math.max(Number(limits.jerk) || 0, 1e-9)
  };
  const shape = profile(span, bounds);
  const steps = Math.max(1, Math.ceil(shape.duration / step));
  const fractions = [];
  for (let index = 0; index <= steps; index += 1) {
    // Sampled on the profile's own duration rather than on the rounded-up step
    // count, so the last sample lands on the target instead of overshooting it.
    const progress = span > 0
      ? travelled(index / steps * shape.duration, span, shape, bounds.jerk) / span
      : Math.min(index, 1);
    fractions.push(Math.min(1, Math.max(0, progress)));
  }
  fractions[fractions.length - 1] = 1;
  return fractions;
}

module.exports = { motionSchedule, profile, turnaroundDistance };
