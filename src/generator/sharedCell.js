"use strict";

const { orientationRadians, rotateLocal } = require("./handoverGeometry");

// Where a shared-cell robot actually stands, and how that relates to the frame
// its workspace CSV is written in.
//
// The pipeline drops every robot into the cell at z = offset[2] + PEDESTAL, but
// the workspace cloud it samples there is stored relative to offset itself. So
// the base object in the scene is never at the CSV frame's origin: it is one
// pedestal above it. Reading a live base pose back out of a running scene and
// feeding it to the interaction-space estimator without undoing that lift puts
// the whole shared space 3 cm too high.
const BASE_PEDESTAL_Z = 0.03;

function numberAt(values, index) {
  const value = Number(Array.isArray(values) ? values[index] : NaN);
  return Number.isFinite(value) ? value : 0;
}

// The pose a lower Thing reports for its own base, expressed the way the
// shared-cell descriptor expresses placements, so the two are interchangeable.
function placementFromBasePose(pose) {
  const position = pose?.position || pose?.[0];
  const orientation = pose?.orientation || pose?.[1];
  if (!Array.isArray(position) || !Array.isArray(orientation)) return null;
  const orientationDeg = [0, 1, 2].map((axis) => numberAt(orientation, axis) * 180 / Math.PI);
  return {
    offset: [
      numberAt(position, 0),
      numberAt(position, 1),
      numberAt(position, 2) - BASE_PEDESTAL_Z
    ],
    orientationDeg,
    rotationDeg: orientationDeg[2],
    source: "scene"
  };
}

// Whether two placements describe the same layout. Poses come back from the
// simulator as floats, so an exact comparison would report a move on every
// read and rebuild a space that did not change.
function samePlacement(left, right, tolerance = 1e-6) {
  if (!left || !right) return false;
  const close = (a, b, scale) => [0, 1, 2].every(
    (axis) => Math.abs(numberAt(a, axis) - numberAt(b, axis)) <= tolerance * scale
  );
  const orientationOf = (placement) => (Array.isArray(placement.orientationDeg)
    ? placement.orientationDeg
    : [0, 0, Number(placement.rotationDeg || 0)]);
  return close(left.offset, right.offset, 1)
    && close(orientationOf(left), orientationOf(right), 180 / Math.PI);
}

// How finely a placement is pinned down before it is hashed. 0.1 mm and ~0.006
// degrees are far below anything the shared space can resolve -- its voxels are
// centimetres -- and far above the float noise a simulator returns.
const PLACEMENT_PRECISION = 1e-4;

function quantize(value) {
  // -0 and 0 must hash alike; JSON.stringify prints them differently.
  return Math.round(Number(value || 0) / PLACEMENT_PRECISION) * PLACEMENT_PRECISION + 0;
}

// A placement reduced to what it geometrically IS, for cache keying.
//
// Euler angles are a spelling, not an identity: at pitch = +/-90 degrees the
// alpha-beta-gamma decomposition is degenerate, so [0, -90, 180], [180, -90, 0]
// and [90, -90, -90] are three names for one rotation. Robot3 is mounted at
// exactly that pitch, so the triple CoppeliaSim reports for it will generally
// not be the triple the descriptor was written with -- and a version hashed
// over the triple then misses its own cache on every single read, recomputing
// a space that never changed. Hashing the rotation itself is immune to that.
function canonicalPlacement(placement) {
  const orientation = orientationRadians(placement);
  const rotation = [
    { x: 1, y: 0, z: 0 },
    { x: 0, y: 1, z: 0 },
    { x: 0, y: 0, z: 1 }
  ].flatMap((basis) => {
    const rotated = rotateLocal(basis, orientation);
    return [quantize(rotated.x), quantize(rotated.y), quantize(rotated.z)];
  });
  const offset = placement?.offset || [0, 0, 0];
  return {
    offset: [0, 1, 2].map((axis) => quantize(offset[axis])),
    rotation
  };
}

module.exports = {
  BASE_PEDESTAL_Z,
  PLACEMENT_PRECISION,
  canonicalPlacement,
  placementFromBasePose,
  samePlacement
};
