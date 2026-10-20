"use strict";

const test = require("node:test");
const assert = require("node:assert");

const { classifyTaskSites, restsOn } = require("./taskSites");

// The cell as it actually stands: Robot1's loaded support at the origin end,
// Robot4's empty one 2.65 m away, and the cube resting on the first.
const LOADED_SUPPORT = {
  alias: "Robot1_grip_support",
  kind: "support",
  world: { x: 0.9, y: 0, z: 0.08 },
  local: { x: 0.9, y: 0, z: 0.08 },
  size: { x: 0.14, y: 0.14, z: 0.16 }
};

const CUBE = {
  alias: "Robot1_grip_cube",
  kind: "payload",
  world: { x: 0.9, y: 0, z: 0.185 },
  local: { x: 0.9, y: 0, z: 0.185 },
  size: { x: 0.045, y: 0.045, z: 0.045 }
};

const EMPTY_SUPPORT = {
  alias: "Robot4_grip_support_b",
  kind: "support",
  world: { x: 2.65, y: -1.6, z: 0.08 },
  local: { x: 0.25, y: 0, z: 0.08 },
  size: { x: 0.14, y: 0.14, z: 0.16 }
};

const reachEverything = () => ({ ok: true, admission: "inside" });
const reachNothing = () => ({ ok: false, admission: "outside" });

function siteOf(result, alias) {
  return result.sites.find((site) => site.alias === alias);
}

test("a payload standing on a support makes that site occupied", () => {
  const result = classifyTaskSites([LOADED_SUPPORT, CUBE, EMPTY_SUPPORT], { admits: reachEverything });
  assert.equal(siteOf(result, "Robot1_grip_support").occupiedBy, "Robot1_grip_cube");
  assert.equal(siteOf(result, "Robot4_grip_support_b").occupiedBy, null);
});

test("the grasp point is the top face plus half a payload, not the support centre", () => {
  const result = classifyTaskSites([EMPTY_SUPPORT], { admits: reachEverything, payloadHalfHeight: 0.0225 });
  // 0.08 centre + 0.08 half-height + 0.0225 half-cube.
  assert.ok(Math.abs(siteOf(result, "Robot4_grip_support_b").grasp.z - 0.1825) < 1e-9);
  assert.equal(siteOf(result, "Robot4_grip_support_b").grasp.x, 0.25);
});

test("reachability is asked of the grasp point in the robot's own frame", () => {
  const asked = [];
  classifyTaskSites([EMPTY_SUPPORT], {
    admits: (point) => {
      asked.push(point);
      return { ok: true, admission: "inside" };
    }
  });
  assert.equal(asked.length, 1);
  // The local frame, never the world one: 0.25 rather than 2.65.
  assert.equal(asked[0].x, 0.25);
  assert.equal(asked[0].y, 0);
});

test("a site out of reach is still reported, so the arm can name what it cannot serve", () => {
  const result = classifyTaskSites([LOADED_SUPPORT, CUBE], { admits: reachNothing });
  const site = siteOf(result, "Robot1_grip_support");
  assert.equal(site.reachable, false);
  assert.equal(site.admission, "outside");
  // Seeing it and holding the cube are independent: this is exactly the state
  // of a robot that can watch the cube but not pick it up.
  assert.equal(site.occupiedBy, "Robot1_grip_cube");
});

test("without a workspace grid nothing is claimed to be reachable", () => {
  const result = classifyTaskSites([EMPTY_SUPPORT], {});
  assert.equal(siteOf(result, "Robot4_grip_support_b").reachable, false);
  assert.equal(siteOf(result, "Robot4_grip_support_b").admission, "unavailable");
});

test("a cube held at handover height is on no support at all", () => {
  const carried = { ...CUBE, world: { x: 0.9, y: 0, z: 0.5 }, local: { x: 0.9, y: 0, z: 0.5 } };
  const result = classifyTaskSites([LOADED_SUPPORT, carried], { admits: reachEverything });
  assert.equal(siteOf(result, "Robot1_grip_support").occupiedBy, null);
  assert.equal(result.payloads[0].restingOn, null);
});

test("occupancy is footprint overlap, not centre proximity", () => {
  const beside = { ...CUBE, world: { x: 0.9 + 0.14, y: 0, z: 0.185 } };
  assert.equal(restsOn(beside, LOADED_SUPPORT, 0.05), false);
  const onEdge = { ...CUBE, world: { x: 0.9 + 0.08, y: 0, z: 0.185 } };
  assert.equal(restsOn(onEdge, LOADED_SUPPORT, 0.05), true);
});

test("a support whose size could not be measured falls back to a nominal footprint", () => {
  const unmeasured = { ...EMPTY_SUPPORT, size: null };
  const result = classifyTaskSites([unmeasured], {
    admits: reachEverything,
    fallbackSize: { x: 0.14, y: 0.14, z: 0.16 }
  });
  const site = siteOf(result, "Robot4_grip_support_b");
  assert.equal(site.reachable, true);
  // The assumed block, not a zero-height one that would put the grasp point
  // inside the support.
  assert.ok(Math.abs(site.grasp.z - 0.1825) < 1e-9);
  assert.equal(site.size.assumed, true);
});

test("an unmeasured support still detects the cube standing on it", () => {
  const unmeasured = { ...LOADED_SUPPORT, size: null };
  const result = classifyTaskSites([unmeasured, CUBE], {
    admits: reachEverything,
    fallbackSize: { x: 0.14, y: 0.14, z: 0.16 }
  });
  assert.equal(siteOf(result, "Robot1_grip_support").occupiedBy, "Robot1_grip_cube");
});

test("only supports become sites; peers, fixtures and the arm itself do not", () => {
  const result = classifyTaskSites([
    LOADED_SUPPORT,
    { alias: "Robot2", kind: "peer_robot", world: { x: 1.5, y: 0.2, z: 0 }, local: { x: 1.5, y: 0.2, z: 0 } },
    { alias: "Robot1_wall", kind: "fixture", world: { x: 0, y: 1, z: 0.4 }, local: { x: 0, y: 1, z: 0.4 } },
    { alias: "Robot1", kind: "self", world: { x: 0, y: 0, z: 0 }, local: { x: 0, y: 0, z: 0 } }
  ], { admits: reachEverything });
  assert.deepEqual(result.sites.map((site) => site.alias), ["Robot1_grip_support"]);
});

test("a grasp point at the tolerance edge is still reachable if the hover point is", () => {
  // Exactly the live case: samples are sparse right above a support because the
  // support was a collision obstacle while the arm's workspace was sampled. A
  // pick solves the hover pose and descends, so the hover point is the honest
  // question.
  const asked = [];
  const result = classifyTaskSites([LOADED_SUPPORT, CUBE], {
    hoverHeight: 0.2,
    admits: (point) => {
      asked.push(point.z);
      return point.z > 0.3 ? { ok: true, admission: "inside" } : { ok: false, admission: "outside" };
    }
  });
  const site = siteOf(result, "Robot1_grip_support");
  assert.equal(site.reachable, true);
  assert.equal(site.admission, "hover_only", "the reason must say it was the hover point that answered");
  assert.deepEqual(asked, [0.1825, 0.3825]);
});

test("the grasp point is tried first, and settles it when it answers", () => {
  const asked = [];
  const result = classifyTaskSites([EMPTY_SUPPORT], {
    hoverHeight: 0.2,
    admits: (point) => {
      asked.push(point.z);
      return { ok: true, admission: "inside" };
    }
  });
  assert.equal(siteOf(result, "Robot4_grip_support_b").admission, "inside");
  assert.deepEqual(asked, [0.1825], "no need to ask about the hover point once the grasp point is admitted");
});

test("a site out of reach at both heights stays out of reach", () => {
  const result = classifyTaskSites([EMPTY_SUPPORT], {
    hoverHeight: 0.2,
    admits: () => ({ ok: false, admission: "outside" })
  });
  const site = siteOf(result, "Robot4_grip_support_b");
  assert.equal(site.reachable, false);
  assert.equal(site.admission, "outside");
});
