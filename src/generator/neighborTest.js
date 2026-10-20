"use strict";

// Could these two arms ever reach the same place?
//
// The cheap end of neighbour discovery. Two workspaces can only intersect if
// the spheres that bound them do, so the whole test is one distance and two
// radii -- no clouds loaded, no voxels, nothing to transform.
//
// It is a NECESSARY condition and nothing more. Saying "possible" for a pair
// that turns out to share nothing costs one interaction-space computation.
// Saying "impossible" for a pair that shares even one voxel leaves two arms
// believing they can never collide, so every term here errs outward:
//
//   - the radius is a bounding SPHERE around the base, which is invariant under
//     however the robot is mounted. Axis-aligned bounds are not: Robot3 is
//     pitched 90 degrees, and its local z-extent lands somewhere else entirely
//     in world coordinates.
//   - the radius is widened by the sample spacing, because it is the extreme of
//     a finite Monte-Carlo sample and the farthest reachable point is almost
//     never one that was drawn.
//
// Only the interaction-space calculator decides. This decides what is worth
// asking it about.

function basePosition(signature) {
  const position = signature?.basePose?.position || signature?.basePose?.[0];
  if (!Array.isArray(position) || position.length < 3) return null;
  return position.map(Number);
}

function radiusOf(signature) {
  const safe = Number(signature?.safeReachRadius);
  if (Number.isFinite(safe) && safe > 0) return safe;
  const raw = Number(signature?.reachRadius);
  const spacing = Number(signature?.spacing) || 0;
  return Number.isFinite(raw) && raw > 0 ? raw + spacing : null;
}

function couldOverlap(left, right) {
  const here = basePosition(left);
  const there = basePosition(right);
  const reachHere = radiusOf(left);
  const reachThere = radiusOf(right);
  if (!here || !there || reachHere === null || reachThere === null) {
    // A robot that will not say where it is or how far it reaches cannot be
    // ruled out, so it is treated as a neighbour until it can be.
    return {
      neighbor: true,
      reason: "signature_incomplete",
      distance: null,
      combinedReach: null,
      margin: null
    };
  }
  const distance = Math.hypot(here[0] - there[0], here[1] - there[1], here[2] - there[2]);
  const combinedReach = reachHere + reachThere;
  return {
    neighbor: distance <= combinedReach,
    reason: distance <= combinedReach ? "reach_spheres_overlap" : "reach_spheres_disjoint",
    distance,
    combinedReach,
    // How much room the test had. A small positive margin means the pair only
    // just qualifies, which is exactly the case worth computing exactly.
    margin: combinedReach - distance,
    reachHere,
    reachThere
  };
}

module.exports = { basePosition, couldOverlap, radiusOf };
