"use strict";

// The property has to survive its own DataSchema.
//
// A consumed WoT property is validated with ajv against the schema in the TD
// before the value reaches the caller, so a field the classifier leaves null is
// not a lenient "absent" -- it is a type error that fails the entire read, and
// the only symptom upstream is an agent that believes the cell has no supports.
// These tests validate real classifier output against the real served schema.

const assert = require("node:assert/strict");
const test = require("node:test");
const Ajv = require("ajv");

const { classifyTaskSites, taskSitesPayload } = require("./taskSites");
const { ensureSharedCellAffordances } = require("./core/thing");

function servedSchema() {
  const td = ensureSharedCellAffordances(
    { title: "Robot4", properties: {}, actions: {} },
    { manifest: { ports: { lowerThing: 8140 } } }
  );
  const schema = { ...td.properties.getTaskSites };
  // forms are TD plumbing, not part of the value's schema.
  delete schema.forms;
  delete schema.description;
  return schema;
}

function validate(payload) {
  const ajv = new Ajv({ strict: false });
  const check = ajv.compile(servedSchema());
  const ok = check(JSON.parse(JSON.stringify(payload)));
  return { ok, errors: check.errors };
}

const LOADED = {
  alias: "Robot1_grip_support",
  kind: "support",
  world: { x: 0.9, y: 0, z: 0.08 },
  local: { x: 2.0, y: -1.6, z: 0.08 },
  size: { x: 0.14, y: 0.14, z: 0.16 }
};
const CUBE = {
  alias: "Robot1_grip_cube",
  kind: "payload",
  world: { x: 0.9, y: 0, z: 0.185 },
  local: { x: 2.0, y: -1.6, z: 0.185 },
  size: { x: 0.045, y: 0.045, z: 0.045 }
};
const EMPTY = {
  alias: "Robot4_grip_support_b",
  kind: "support",
  world: { x: 2.65, y: -1.6, z: 0.08 },
  local: { x: 0.25, y: 0, z: 0.08 },
  size: { x: 0.14, y: 0.14, z: 0.16 }
};

test("the property declares a schema at all, or a consumer refuses to read it", () => {
  assert.ok(servedSchema().type, "getTaskSites must declare a type");
});

test("a cell with one loaded and one empty support serialises validly", () => {
  const classified = classifyTaskSites([LOADED, CUBE, EMPTY], {
    admits: () => ({ ok: true, admission: "inside" })
  });
  // The empty support is the case that used to fail: occupiedBy was null.
  assert.equal(classified.sites.find((site) => site.alias === EMPTY.alias).occupiedBy, null);
  const { ok, errors } = validate(taskSitesPayload("Robot4", classified));
  assert.ok(ok, `payload rejected by its own schema: ${JSON.stringify(errors)}`);
});

test("an empty site reports the 'none' sentinel rather than a null", () => {
  const payload = taskSitesPayload("Robot4", classifyTaskSites([EMPTY], {
    admits: () => ({ ok: true, admission: "inside" })
  }));
  assert.equal(payload.sites[0].occupiedBy, "none");
  assert.equal(payload.payloads.length, 0);
});

test("a payload resting on nothing serialises validly too", () => {
  const carried = { ...CUBE, world: { x: 0.9, y: 0, z: 0.6 }, local: { x: 2.0, y: -1.6, z: 0.6 } };
  const classified = classifyTaskSites([LOADED, carried], {
    admits: () => ({ ok: true, admission: "inside" })
  });
  assert.equal(classified.payloads[0].restingOn, null);
  const payload = taskSitesPayload("Robot1", classified);
  assert.equal(payload.payloads[0].restingOn, "none");
  const { ok, errors } = validate(payload);
  assert.ok(ok, `payload rejected by its own schema: ${JSON.stringify(errors)}`);
});

test("a site whose position could not be read omits the key instead of nulling it", () => {
  const positionless = { ...EMPTY, local: null, world: null };
  const classified = classifyTaskSites([positionless], { admits: () => ({ ok: true }) });
  const payload = taskSitesPayload("Robot4", classified);
  assert.equal(payload.sites[0].admission, "no_position");
  assert.ok(!("grasp" in payload.sites[0]), "an unknown position is absent, not null");
  const { ok, errors } = validate(payload);
  assert.ok(ok, `payload rejected by its own schema: ${JSON.stringify(errors)}`);
});

test("a cell with no supports at all still serialises validly", () => {
  const { ok, errors } = validate(taskSitesPayload("Robot2", classifyTaskSites([], { admits: () => ({ ok: true }) })));
  assert.ok(ok, `empty payload rejected: ${JSON.stringify(errors)}`);
});
