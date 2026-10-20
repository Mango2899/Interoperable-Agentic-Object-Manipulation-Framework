const assert = require("node:assert/strict");
const test = require("node:test");

const {
  BASE_PEDESTAL_Z, canonicalPlacement, placementFromBasePose, samePlacement
} = require("./sharedCell");

test("a live base pose is read back into the frame the workspace cloud uses", () => {
  const placement = placementFromBasePose({
    position: [1.5, 0.2, BASE_PEDESTAL_Z],
    orientation: [0, 0, Math.PI]
  });
  // The base object stands one pedestal above the origin its own reachable
  // points are written relative to. Keeping that lift would raise every
  // recomputed shared space by the same 3 cm.
  assert.deepEqual(placement.offset, [1.5, 0.2, 0]);
  assert.ok(Math.abs(placement.orientationDeg[2] - 180) < 1e-9);
  assert.equal(placement.rotationDeg, placement.orientationDeg[2]);
  assert.equal(placement.source, "scene");
});

test("a pose that cannot be read is reported as absent rather than as the origin", () => {
  assert.equal(placementFromBasePose(null), null);
  assert.equal(placementFromBasePose({ position: [0, 0, 0] }), null);
  assert.equal(placementFromBasePose({ reason: "no_such_property" }), null);
});

test("float noise in a reported pose does not count as a move", () => {
  const descriptor = { offset: [1.5, 0.2, 0], orientationDeg: [0, 0, 180] };
  const reported = placementFromBasePose({
    position: [1.5 + 1e-9, 0.2, BASE_PEDESTAL_Z],
    orientation: [0, 0, Math.PI]
  });
  assert.equal(samePlacement(descriptor, reported), true);
  assert.equal(samePlacement(descriptor, { ...reported, offset: [1.25, 0.2, 0] }), false);
  assert.equal(samePlacement(descriptor, { ...descriptor, orientationDeg: [0, 0, 90] }), false);
});

test("a placement described only by yaw compares against a full orientation", () => {
  assert.equal(
    samePlacement({ offset: [0, 0, 0], rotationDeg: 180 }, { offset: [0, 0, 0], orientationDeg: [0, 0, 180] }),
    true
  );
});

test("a placement is keyed on the rotation it means, not on the angles naming it", () => {
  const key = (orientationDeg) => JSON.stringify(canonicalPlacement({ offset: [2.1, -0.5, 0], orientationDeg }));

  // Pitch -90 is a singularity of the alpha-beta-gamma decomposition, so these
  // are three spellings of one rotation. Robot3 is mounted at exactly that
  // pitch, and CoppeliaSim picks whichever spelling its own matrix-to-Euler
  // conversion produces.
  assert.equal(key([0, -90, 180]), key([180, -90, 0]));
  assert.equal(key([0, -90, 180]), key([90, -90, -90]));
  // Not everything at that pitch is the same rotation, though.
  assert.notEqual(key([0, -90, 180]), key([0, -90, 0]));

  // A bare yaw and the full triple naming it agree.
  assert.equal(
    JSON.stringify(canonicalPlacement({ offset: [1, 2, 3], rotationDeg: 180 })),
    JSON.stringify(canonicalPlacement({ offset: [1, 2, 3], orientationDeg: [0, 0, 180] }))
  );

  // Simulator float noise must not read as a move; a real move must.
  assert.equal(key([0, -90, 180]), JSON.stringify(canonicalPlacement({
    offset: [2.1 + 1e-9, -0.5 - 1e-9, 0], orientationDeg: [0, -90 + 1e-8, 180]
  })));
  assert.notEqual(key([0, -90, 180]), JSON.stringify(canonicalPlacement({
    offset: [2.1, -0.4, 0], orientationDeg: [0, -90, 180]
  })));

  // Signed zero is still zero.
  assert.equal(
    JSON.stringify(canonicalPlacement({ offset: [-0, 0, 0], orientationDeg: [0, 0, 0] })),
    JSON.stringify(canonicalPlacement({ offset: [0, -0, 0], orientationDeg: [0, 0, 0] }))
  );
});
