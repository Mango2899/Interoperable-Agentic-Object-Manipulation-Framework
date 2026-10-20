"use strict";

// A sampled bounding box is a fact about the sample, not about the arm.
//
// The generated TD applied the same box to the moveTo command and to the pose
// report. Constraining the command is reasonable -- a target outside the sampled
// workspace is worth refusing. Constraining the report is not: the arm's own
// measured position is ground truth, and random joint sampling rarely lands on
// full extension, so the box understates the true envelope exactly the way
// reachRadius understates the true reach. Robot3 standing at x = -1.076 against
// a declared minimum of -0.917 made every read of its pose fail validation.

const assert = require("node:assert/strict");
const test = require("node:test");
const Ajv = require("ajv");

const { ensureSharedCellAffordances } = require("./core/thing");

function loadedTd() {
  return ensureSharedCellAffordances({
    title: "Robot3",
    properties: {
      getCartesianposition: {
        type: "object",
        properties: {
          x: { type: "number", minimum: -0.91733200924607, maximum: 1.2675625578243, unit: "meter" },
          y: { type: "number", minimum: -1.0844989826313, maximum: 1.0689657700413, unit: "meter" },
          z: { type: "number", minimum: -1.0884325688441, maximum: 1.0948041993744, unit: "meter" }
        }
      }
    },
    actions: {}
  }, { manifest: { ports: { lowerThing: 8130 } } });
}

test("a pose outside the sampled box is still a readable pose", () => {
  const schema = loadedTd().properties.getCartesianposition;
  const validate = new Ajv({ strict: false }).compile(schema);
  // The reading Robot3 actually returned while parked.
  const parked = { x: -1.0756816280392, z: 0.0000027848340221174, y: -0.43586793986019 };
  assert.ok(validate(parked), `the arm's own pose was rejected: ${JSON.stringify(validate.errors)}`);
});

test("the axes keep their type and unit; only the sample-derived limits go", () => {
  const reported = loadedTd().properties.getCartesianposition.properties;
  for (const axis of ["x", "y", "z"]) {
    assert.equal(reported[axis].type, "number");
    assert.equal(reported[axis].unit, "meter");
    assert.equal("minimum" in reported[axis], false, `${axis} still carries a sampled minimum`);
    assert.equal("maximum" in reported[axis], false, `${axis} still carries a sampled maximum`);
  }
});

test("a TD without the property at all is left alone", () => {
  const td = ensureSharedCellAffordances(
    { title: "Robot9", properties: {}, actions: {} },
    { manifest: { ports: { lowerThing: 8190 } } }
  );
  assert.equal(td.properties.getCartesianposition, undefined);
});

test("the generator constrains the command but not the report", () => {
  const fs = require("node:fs");
  const path = require("node:path");
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  assert.match(bridge, /commandTarget\.x\.minimum = minX/);
  assert.match(bridge, /delete td\.properties\.getCartesianposition\.properties\[axis\]\.minimum/);
  // The old shared loop must be gone, or new robots inherit the same defect.
  assert.doesNotMatch(
    bridge,
    /for \(const target of \[td\.actions\.moveTocartesianPosition\.input\.properties, td\.properties\.getCartesianposition\.properties\]\)/
  );
});
