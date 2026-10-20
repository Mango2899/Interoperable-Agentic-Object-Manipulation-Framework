// Geometry shared by interaction-space ranking and the runtime handover
// planner. Both have to agree on which way each arm approaches the rendezvous:
// the ranking decides WHERE the cube is exchanged, and it can only judge that
// if it computes the approach the runtime will actually attempt.

const AXIS_EPSILON = 1e-9;

function orientationRadians(placement) {
  const degrees = Array.isArray(placement?.orientationDeg)
    ? placement.orientationDeg
    : [0, 0, Number(placement?.rotationDeg || 0)];
  return degrees.map((value) => Number(value || 0) * Math.PI / 180);
}

function rotateLocal(point, orientation) {
  const [roll, pitch, yaw] = orientation;
  const cr = Math.cos(roll); const sr = Math.sin(roll);
  const cp = Math.cos(pitch); const sp = Math.sin(pitch);
  const cy = Math.cos(yaw); const sy = Math.sin(yaw);
  // CoppeliaSim's alpha-beta-gamma Euler matrix is Rx(roll) * Ry(pitch) *
  // Rz(yaw). This differs from the common Rz * Ry * Rx convention whenever
  // more than one angle is non-zero. The lower runtime uses sim.buildMatrix,
  // so interaction-space generation and world/local TCP conversion must use
  // the same composition order.
  return {
    x: (cp * cy) * point.x + (-cp * sy) * point.y + sp * point.z,
    y: (cr * sy + sr * sp * cy) * point.x + (cr * cy - sr * sp * sy) * point.y + (-sr * cp) * point.z,
    z: (sr * sy - cr * sp * cy) * point.x + (sr * cy + cr * sp * sy) * point.y + (cr * cp) * point.z
  };
}

function rotateWorldToLocal(point, orientation) {
  const xAxis = rotateLocal({ x: 1, y: 0, z: 0 }, orientation);
  const yAxis = rotateLocal({ x: 0, y: 1, z: 0 }, orientation);
  const zAxis = rotateLocal({ x: 0, y: 0, z: 1 }, orientation);
  return {
    x: xAxis.x * point.x + xAxis.y * point.y + xAxis.z * point.z,
    y: yAxis.x * point.x + yAxis.y * point.y + yAxis.z * point.z,
    z: zAxis.x * point.x + zAxis.y * point.y + zAxis.z * point.z
  };
}

// A height is measured against gravity, even when the robot's base is tilted.
// Points stay robot-local; worldUpLocal is R^T * (0, 0, 1).
function offsetWorldUp(point, height, worldUpLocal = [0, 0, 1]) {
  return {
    x: point.x + height * worldUpLocal[0],
    y: point.y + height * worldUpLocal[1],
    z: point.z + height * worldUpLocal[2]
  };
}

function normalizedVector(vector) {
  if (!Array.isArray(vector) || vector.length !== 3) return null;
  const length = Math.hypot(vector[0], vector[1], vector[2]);
  if (!Number.isFinite(length) || length < AXIS_EPSILON) return null;
  return [vector[0] / length, vector[1] / length, vector[2] / length];
}

function crossProduct(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
}

function dotProduct(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function worldVectorToLocal(vector, orientation) {
  const rotated = rotateWorldToLocal({ x: vector[0], y: vector[1], z: vector[2] }, orientation);
  return [rotated.x, rotated.y, rotated.z];
}

// The direction the tool points, in world coordinates, when the arm holds the
// pose the tool axis was measured in. A direction only: the robot's tool
// control point is its gripper's jaw midpoint, so there is no offset length.
function toolAxisWorld(placement, toolAxis) {
  const axis = Array.isArray(toolAxis) ? toolAxis.map(Number) : null;
  if (!axis) return null;
  const rotated = rotateLocal({ x: axis[0], y: axis[1], z: axis[2] }, orientationRadians(placement));
  return normalizedVector([rotated.x, rotated.y, rotated.z]);
}

// The direction the receiver faces the giver across the cell. Used as the
// default preference for a perpendicular approach, so it follows the layout
// instead of hardcoding a world axis.
function cellBaselineAxis(receiverPlacement, giverPlacement) {
  const receiver = receiverPlacement?.offset;
  const giver = giverPlacement?.offset;
  if (!Array.isArray(receiver) || !Array.isArray(giver)) return null;
  return normalizedVector([
    Number(giver[0] || 0) - Number(receiver[0] || 0),
    Number(giver[1] || 0) - Number(receiver[1] || 0),
    Number(giver[2] || 0) - Number(receiver[2] || 0)
  ]);
}

function perpendicularAxis(axis, preferred) {
  // Gram-Schmidt: the part of the preferred direction orthogonal to the
  // giver's tool. Perpendicularity is exact by construction; the preference
  // only chooses which perpendicular.
  const dot = dotProduct(axis, preferred);
  const projected = normalizedVector([
    preferred[0] - dot * axis[0],
    preferred[1] - dot * axis[1],
    preferred[2] - dot * axis[2]
  ]);
  if (projected) return projected;
  // The preference was parallel to the giver's tool. Any orthogonal direction
  // will do; take a stable one.
  const seed = Math.abs(axis[0]) > 0.9 ? [0, 1, 0] : [1, 0, 0];
  const seedDot = dotProduct(axis, seed);
  return normalizedVector([
    seed[0] - seedDot * axis[0],
    seed[1] - seedDot * axis[1],
    seed[2] - seedDot * axis[2]
  ]);
}

// Approach modes that name a direction in the CELL's frame instead of deriving
// one from the giver. "Opposed" and "perpendicular" describe a relationship
// between two tools and therefore change whenever either arm moves; these
// describe where the receiver's own tool axis -- runtime.gripper.toolAxis, the
// gripper's +Y, which is also its jaw midpoint's Y -- is pinned to point, and
// so hold whatever the giver is doing. "vertical" is a tool coming straight
// down onto the cube.
const FIXED_APPROACH_AXES = {
  horizontalX: [1, 0, 0],
  horizontalY: [0, 1, 0],
  vertical: [0, 0, -1]
};

const RELATIVE_APPROACH_MODES = ["opposed", "perpendicular"];

// Spelling a mode is not the same as choosing one, so case is forgiven and
// "vertically" reads as "vertical". Anything else is a typo, and the callers
// say so rather than quietly receiving head-on.
const APPROACH_MODE_NAMES = new Map([
  ...RELATIVE_APPROACH_MODES.map((name) => [name, name]),
  ...Object.keys(FIXED_APPROACH_AXES).map((name) => [name.toLowerCase(), name]),
  ["vertically", "vertical"]
]);

function normalizeApproachMode(value, fallback = "opposed") {
  return APPROACH_MODE_NAMES.get(String(value ?? "").toLowerCase()) ?? fallback;
}

function fixedApproachAxis(mode, preferred) {
  const axis = FIXED_APPROACH_AXES[normalizeApproachMode(mode, "")];
  if (!axis) return null;
  // Down is unambiguous. The horizontal ones name a line, not a sense, and a
  // tool has to point AT the cube rather than away from it -- so the sense is
  // taken from the preference, which at both call sites is the line from this
  // robot's base across the cell toward the arm it is exchanging with.
  if (axis[2] !== 0) return axis;
  const toward = normalizedVector(preferred);
  if (toward && dotProduct(axis, toward) < 0) {
    return [-axis[0], -axis[1], -axis[2]];
  }
  return axis;
}

// A named direction names one direction -- but a mounting that cannot solve
// that direction exactly can very often solve one a few degrees off it, and
// failing the whole rendezvous over the difference throws away a handover that
// was almost there. So the named axis gets the same bounded search the
// perpendicular circle already gets: primary first, then a cone around it. The
// axis actually used is reported back, so a tilt is visible rather than silent.
function fixedApproachAxisCandidates(axis, options = {}) {
  const primary = normalizedVector(axis);
  if (!primary) return [];
  const count = Math.max(1, Math.round(Number(options.count ?? 8)));
  // Tighter than the perpendicular default on purpose: 60 degrees off
  // "vertical" is not vertical any more, and the point of naming a direction
  // is that the arm comes in along it.
  const maxTurn = Math.max(0, Number(options.fixedMaxTurnDeg ?? 25)) * Math.PI / 180;
  if (count === 1 || maxTurn < AXIS_EPSILON) return [primary];

  const seed = Math.abs(primary[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1];
  const u = perpendicularAxis(primary, seed);
  const v = u && normalizedVector(crossProduct(primary, u));
  if (!u || !v) return [primary];
  const sides = [u, [-u[0], -u[1], -u[2]], v, [-v[0], -v[1], -v[2]]];

  const candidates = [primary];
  const rings = Math.max(1, Math.round((count - 1) / sides.length));
  for (let ring = 1; ring <= rings && candidates.length < count; ring += 1) {
    const theta = (maxTurn * ring) / rings;
    const cos = Math.cos(theta);
    const sin = Math.sin(theta);
    for (const side of sides) {
      if (candidates.length >= count) break;
      candidates.push([
        primary[0] * cos + side[0] * sin,
        primary[1] * cos + side[1] * sin,
        primary[2] * cos + side[2] * sin
      ]);
    }
  }
  return candidates;
}

// The direction the GIVER presents the cube along at the rendezvous. Its own
// mounting direction unless the manifest names a cell direction -- which is
// what lets an arm hold the cube out horizontally however it is bolted down,
// instead of the exchange geometry being dictated by the mounting.
function giverPresentAxis({ mode, mountingAxis, preferred }) {
  return fixedApproachAxis(mode, preferred) || normalizedVector(mountingAxis) || null;
}

function receiverApproachAxis(giverAxis, mode, preferred) {
  // A named cell direction needs no giver: that is the point of it.
  const fixed = fixedApproachAxis(mode, preferred);
  if (fixed) return fixed;
  const axis = normalizedVector(giverAxis);
  if (!axis) return null;
  if (mode === "perpendicular") {
    return perpendicularAxis(axis, normalizedVector(preferred) || [0, 0, -1]);
  }
  // Nose to nose: the receiver's tool points back down the giver's.
  return [-axis[0], -axis[1], -axis[2]];
}

function receiverApproachAxisCandidates(giverAxis, mode, preferred, options = {}) {
  const count = Math.max(2, Math.round(Number(options.count ?? 8)));
  // How far the search may turn away from the requested direction. Without a
  // bound it will happily answer a blocked sideways approach with one coming
  // up from underneath -- still perpendicular, still collision free, and not
  // remotely what was asked for.
  const maxTurn = Math.max(0, Number(options.maxTurnDeg ?? 60)) * Math.PI / 180;
  const fixed = fixedApproachAxis(mode, preferred);
  if (fixed) return fixedApproachAxisCandidates(fixed, options);
  const first = receiverApproachAxis(giverAxis, mode, preferred);
  if (!first) return [];
  // "Opposed" names exactly one direction, so there is nothing to search.
  if (mode !== "perpendicular") return [first];

  // "Perpendicular" names a circle of directions. Rotating the preferred one
  // about the giver's tool stays exactly on that circle, so every candidate
  // satisfies the constraint just as fully. They differ only in whether the
  // straight line in happens to run through one of the receiver's own singular
  // regions, which no amount of per-point reachability testing can see.
  const axis = normalizedVector(giverAxis);
  const bitangent = normalizedVector(crossProduct(axis, first));
  if (!bitangent) return [first];

  const candidates = [first];
  for (let step = 1; step <= Math.floor(count / 2); step += 1) {
    const theta = (Math.PI * 2 * step) / count;
    if (theta > maxTurn + AXIS_EPSILON) break;
    for (const signed of step < count / 2 ? [theta, -theta] : [theta]) {
      const cos = Math.cos(signed);
      const sin = Math.sin(signed);
      candidates.push([
        first[0] * cos + bitangent[0] * sin,
        first[1] * cos + bitangent[1] * sin,
        first[2] * cos + bitangent[2] * sin
      ]);
    }
  }
  return candidates;
}

// Roll angles about the pinned tool axis for the receiver to try, in degrees.
//
// The roll is the one degree of freedom the approach constraint leaves free,
// and at a handover it is the one that decides whether the two grippers meeting
// at the same cube interleave their fingers or drive through each other. There
// is nothing to compute it from -- it depends on the shape of both grippers and
// on how the giver happens to be holding the cube -- so it is searched, against
// the collision check that can actually see the giver's gripper.
//
// A full turn, because the answer is not near any particular angle: the window
// that works is wherever the two finger assemblies happen to miss, and for the
// live Robot2/Robot4 pair it was roughly ten degrees wide out of three hundred
// and sixty. Ascending from zero rather than spiralling out from a guess, since
// there is no guess -- the driver remembers the winner instead, so the cost is
// paid once per pair rather than once per handover.
function approachRollCandidates(options = {}) {
  const step = Math.max(1, Number(options.rollStepDeg ?? 15));
  const count = Math.min(360, Math.max(1, Math.floor(360 / step)));
  return Array.from({ length: count }, (_, index) => index * step);
}

// How close the wrist comes to the robot's own joint-1 axis while travelling
// the straight approach leg. A revolute base joint is undetermined when the
// wrist sits on its axis, so a line that crosses it cannot be followed
// continuously however reachable each of its points is on its own -- which is
// exactly what per-point workspace admission fails to notice.
function shoulderMargin({ graspLocal, axisLocal, toolLength, standoff }) {
  const axis = normalizedVector(axisLocal);
  if (!axis || !graspLocal) return null;
  const at = (back) => [
    graspLocal.x - axis[0] * (toolLength + back),
    graspLocal.y - axis[1] * (toolLength + back)
  ];
  // Both ends of the leg, projected onto the plane the base joint rotates in.
  const start = at(0);
  const end = at(Math.max(0, Number(standoff) || 0));
  const dx = end[0] - start[0];
  const dy = end[1] - start[1];
  const lengthSquared = dx * dx + dy * dy;
  let t = 0;
  if (lengthSquared > AXIS_EPSILON) {
    t = Math.min(1, Math.max(0, -(start[0] * dx + start[1] * dy) / lengthSquared));
  }
  return Math.hypot(start[0] + t * dx, start[1] + t * dy);
}

module.exports = {
  offsetWorldUp,
  approachRollCandidates,
  FIXED_APPROACH_AXES,
  fixedApproachAxis,
  fixedApproachAxisCandidates,
  giverPresentAxis,
  normalizeApproachMode,
  orientationRadians,
  rotateLocal,
  rotateWorldToLocal,
  worldVectorToLocal,
  normalizedVector,
  crossProduct,
  dotProduct,
  toolAxisWorld,
  cellBaselineAxis,
  perpendicularAxis,
  receiverApproachAxis,
  receiverApproachAxisCandidates,
  shoulderMargin
};
