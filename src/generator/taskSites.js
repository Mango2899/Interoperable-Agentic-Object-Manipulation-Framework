"use strict";

// What an arm concludes about the task furniture standing in its cell.
//
// The scene inventory says only what each object IS -- a support, a payload --
// and where it stands. The two things an agent actually needs to reason about a
// transfer are not in there: whether the site is one THIS arm could serve, and
// whether it is currently holding the payload. Both are derived here, from
// geometry and from the arm's own sampled workspace, so no robot has to be told
// which support is the source and which is the goal. That is the whole point:
// the same scene produces different beliefs in different arms, and which arm
// holds A and which holds B falls out of where they stand.

const { offsetWorldUp } = require("./handoverGeometry");

const DEFAULT_PAYLOAD_HALF_HEIGHT = 0.0225;
// Used only when the simulator could not measure an object's bounding box. A
// site with an assumed footprint is still a usable site; a site with a zero
// footprint would put its grasp point inside the solid block and be rejected as
// unreachable, which is a worse answer than an approximate one.
const DEFAULT_SITE_SIZE = { x: 0.14, y: 0.14, z: 0.16 };
// How far above a support's top face a payload may sit and still count as
// resting on it. Generous enough for a cube dropped from a few millimetres up,
// far too small for one held in a gripper at handover height.
const DEFAULT_RESTING_CLEARANCE = 0.05;
// How far above the grasp point a top-down approach starts. Only a fallback:
// the caller passes the arm's own configured pickup approach height.
const DEFAULT_HOVER_HEIGHT = 0.2;

function point(value) {
  if (!value) return null;
  const x = Number(Array.isArray(value) ? value[0] : value.x);
  const y = Number(Array.isArray(value) ? value[1] : value.y);
  const z = Number(Array.isArray(value) ? value[2] : value.z);
  if (![x, y, z].every(Number.isFinite)) return null;
  return { x, y, z };
}

function halfExtents(size, fallback = { x: 0, y: 0, z: 0 }) {
  const extents = point(size) || point(fallback) || { x: 0, y: 0, z: 0 };
  return { x: Math.abs(extents.x) / 2, y: Math.abs(extents.y) / 2, z: Math.abs(extents.z) / 2 };
}

// Where a payload would be gripped if it were standing on this support: the
// centre of the top face, plus half a payload. This -- not the support's own
// centre, which is buried inside a solid block -- is the point that decides
// whether the arm can serve the site at all.
function graspPointOf(site, payloadHalfHeight, fallbackSize = DEFAULT_SITE_SIZE, worldUpLocal) {
  const local = point(site.local);
  if (!local) return null;
  const half = halfExtents(site.size, fallbackSize);
  return offsetWorldUp(local, half.z + payloadHalfHeight, worldUpLocal);
}

function restsOn(payload, support, restingClearance, fallbackSize = DEFAULT_SITE_SIZE) {
  const payloadWorld = point(payload.world);
  const supportWorld = point(support.world);
  if (!payloadWorld || !supportWorld) return false;
  const supportHalf = halfExtents(support.size, fallbackSize);
  const payloadHalf = halfExtents(payload.size, { x: 0.045, y: 0.045, z: 0.045 });
  // Footprint overlap, not centre proximity: a wide payload on a narrow
  // support is still on it, and a cube 20 cm to the side of a wide table is not.
  if (Math.abs(payloadWorld.x - supportWorld.x) > supportHalf.x + payloadHalf.x) return false;
  if (Math.abs(payloadWorld.y - supportWorld.y) > supportHalf.y + payloadHalf.y) return false;
  const topFace = supportWorld.z + supportHalf.z;
  const payloadBottom = payloadWorld.z - payloadHalf.z;
  if (payloadBottom < topFace - payloadHalf.z) return false;
  return payloadBottom - topFace <= restingClearance;
}

// The cube a robot is carrying is nowhere near a support, so it drops out of
// the occupancy test on geometry alone -- but only if the inventory reports it
// where it actually is. A payload with no position is reported as held rather
// than silently treated as absent.
function classifyTaskSites(objects, options = {}) {
  const admits = typeof options.admits === "function" ? options.admits : null;
  const payloadHalfHeight = Number.isFinite(options.payloadHalfHeight)
    ? options.payloadHalfHeight
    : DEFAULT_PAYLOAD_HALF_HEIGHT;
  const restingClearance = Number.isFinite(options.restingClearance)
    ? options.restingClearance
    : DEFAULT_RESTING_CLEARANCE;
  const fallbackSize = point(options.fallbackSize) || DEFAULT_SITE_SIZE;
  const hoverHeight = Number.isFinite(options.hoverHeight) ? options.hoverHeight : DEFAULT_HOVER_HEIGHT;

  const entries = Array.isArray(objects) ? objects : [];
  const supports = entries.filter((entry) => entry && entry.kind === "support");
  const payloads = entries.filter((entry) => entry && entry.kind === "payload");

  const sites = supports.map((support) => {
    const grasp = graspPointOf(support, payloadHalfHeight, fallbackSize, options.worldUpLocal);
    const hover = grasp ? offsetWorldUp(grasp, hoverHeight, options.worldUpLocal) : null;
    const resting = payloads.find((payload) => restsOn(payload, support, restingClearance, fallbackSize)) || null;
    let reachable = false;
    let admission = "unknown";
    if (!grasp) {
      admission = "no_position";
    } else if (!admits) {
      // No workspace grid means no evidence either way. Claiming the site is
      // reachable here is how an arm ends up believing it can serve a support
      // on the far side of the cell.
      admission = "unavailable";
    } else {
      const atGrasp = admits(grasp) || {};
      if (atGrasp.ok === true) {
        reachable = true;
        admission = atGrasp.admission || "inside";
      } else {
        // The grasp point sits against a solid block that WAS an obstacle while
        // this arm's workspace was sampled, so samples are sparse there by
        // construction -- and a pick never solves for that point directly
        // anyway. It arms the top-down planner, drops the support from the
        // obstacle set, solves the hover pose and descends vertically. Asking
        // whether the hover point is reachable is asking the question the arm
        // actually asks; asking about the grasp point alone rejects a support
        // the arm has been picking from all along.
        const atHover = admits(hover) || {};
        reachable = atHover.ok === true;
        admission = reachable ? "hover_only" : (atGrasp.admission || "outside");
      }
    }
    return {
      alias: support.alias,
      world: point(support.world),
      local: point(support.local),
      grasp,
      hover,
      size: point(support.size) || { ...fallbackSize, assumed: true },
      reachable,
      admission,
      occupiedBy: resting ? resting.alias : null
    };
  });

  return {
    sites,
    payloads: payloads.map((payload) => ({
      alias: payload.alias,
      world: point(payload.world),
      local: point(payload.local),
      restingOn: (supports.find((support) => restsOn(payload, support, restingClearance, fallbackSize)) || {}).alias || null
    }))
  };
}

// The classifier answers with null where it has nothing to say -- no payload on
// this site, no position for that one. A WoT property cannot: its DataSchema
// declares occupiedBy a string and world an object, and a consumer validates
// the payload against that schema before the value ever reaches the caller. A
// null there is not a lenient "absent", it is a type error, and it fails the
// whole read.
//
// So absence is expressed the two ways the schema allows: the key is omitted
// when there is no value, and occupancy uses the same "none" sentinel the cube
// ownership and gripper properties already use.
function withoutNulls(record) {
  const out = {};
  for (const [key, value] of Object.entries(record)) {
    if (value !== null && value !== undefined) out[key] = value;
  }
  return out;
}

function taskSitesPayload(robot, classified) {
  return {
    robot,
    sites: (classified.sites || []).map((site) => withoutNulls({
      ...site,
      occupiedBy: site.occupiedBy || "none"
    })),
    payloads: (classified.payloads || []).map((payload) => withoutNulls({
      ...payload,
      restingOn: payload.restingOn || "none"
    }))
  };
}

module.exports = {
  DEFAULT_HOVER_HEIGHT,
  DEFAULT_PAYLOAD_HALF_HEIGHT,
  DEFAULT_RESTING_CLEARANCE,
  DEFAULT_SITE_SIZE,
  classifyTaskSites,
  graspPointOf,
  restsOn,
  taskSitesPayload
};
