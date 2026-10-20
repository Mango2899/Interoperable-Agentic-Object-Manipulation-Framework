"use strict";

const assert = require("node:assert/strict");
const path = require("node:path");
const test = require("node:test");

const {
  gripFromSite,
  loadedSite,
  normaliseSite,
  parseCellSites,
  resolveCellSites
} = require("./cellSites");
const { createContext } = require("./core/context");

function contextsFor(names) {
  return names.map((name) => createContext(path.join("generated", "robots", name, "manifest.json")));
}

function cellOf(sites) {
  return [{ manifest: { name: "Cell", sharedEnvironment: { taskSites: sites } } }];
}

test("a site is declared in the cell's frame, not in a robot's", () => {
  const [a, b] = parseCellSites("site_a@0.9,0,0.08,0,cube_a; site_b@1.25,-2,0.08,90");
  assert.deepEqual(a.position, [0.9, 0, 0.08]);
  assert.equal(a.yawDeg, 0);
  assert.equal(a.payload.alias, "cube_a");
  assert.deepEqual(b.position, [1.25, -2, 0.08]);
  assert.equal(b.yawDeg, 90);
  assert.equal(b.payload, null, "a site with no payload is a destination, not a source");
});

test("the yaw and the payload alias are told apart by what they are", () => {
  // Both are optional and both come after the coordinates, so a site can carry a
  // payload without stating a yaw it does not have -- the common case for B.
  const [withPayload] = parseCellSites("s@1,2,0.08,cube");
  assert.equal(withPayload.yawDeg, 0);
  assert.equal(withPayload.payload.alias, "cube");

  const [withYaw] = parseCellSites("s@1,2,0.08,45");
  assert.equal(withYaw.yawDeg, 45);
  assert.equal(withYaw.payload, null);

  const [withBoth] = parseCellSites("s@1,2,0.08,45,cube");
  assert.equal(withBoth.yawDeg, 45);
  assert.equal(withBoth.payload.alias, "cube");
});

test("a malformed declaration is refused where it is written, not in the scene", () => {
  assert.throws(() => parseCellSites("no_at_sign"), /alias@x,y,z/);
  assert.throws(() => parseCellSites("s@1,2"), /world position x,y,z/);
  assert.throws(() => parseCellSites("s@one,two,three"), /world position x,y,z/);
  assert.throws(() => parseCellSites("s@1,2,3; s@4,5,6"), /twice/);
  assert.throws(() => normaliseSite({ position: [1, 2, 3] }), /needs an alias/);
  assert.equal(parseCellSites(undefined).length, 0, "no declaration is not an error");
});

test("the payload rests on the top face of the support it is declared with", () => {
  // Not a second coordinate to keep in step with the first: moving the support
  // moves the cube, because a cube standing on a support is what A means.
  const [site] = parseCellSites("s@1.5,-0.5,0.08,cube");
  const [, , supportHeight] = site.size;
  const [, , payloadHeight] = site.payload.size;
  const topFace = site.position[2] + supportHeight / 2;
  assert.ok(site.payload.position[2] > topFace, "the cube starts above the face, and settles onto it");
  assert.ok(site.payload.position[2] - (topFace + payloadHeight / 2) < 0.005);
  assert.equal(site.payload.position[0], site.position[0]);
  assert.equal(site.payload.position[1], site.position[1]);
});

test("the cell as it stands today is expressible as one declaration", () => {
  // The strongest evidence this is a move of the declaration and not a move of
  // the furniture: the numbers the running cell already uses, written in the new
  // form, reproduce the running cell exactly.
  const contexts = contextsFor(["Robot1", "Robot2", "Robot3", "Robot4"]);
  const offsets = contexts.map((ctx) => ctx.manifest.sharedEnvironment.offset);
  const today = resolveCellSites(contexts, offsets);
  const goal = today.find((site) => site.alias === "Robot4_grip_support_b");
  assert.ok(goal, "Robot4 declares the goal support");

  const grip = contexts[0].manifest.runtime.gripper;
  const declared = parseCellSites(
    `${grip.supportName}@${grip.supportPosition.join(",")},${grip.objectName}`
    + `; ${goal.alias}@${goal.position.join(",")}`
  );
  const a = gripFromSite(loadedSite(declared));
  assert.deepEqual(a.supportName, grip.supportName);
  assert.deepEqual(a.supportPosition, grip.supportPosition);
  assert.deepEqual(a.objectName, grip.objectName);
  // The cube's spawn height is derived, and it lands on the number the manifest
  // has been carrying by hand.
  assert.deepEqual(a.objectPosition.map((value) => Number(value.toFixed(6))), grip.objectPosition);
  assert.deepEqual(a.objectSize, grip.objectSize);
  assert.deepEqual(a.supportSize, grip.supportSize);
});

test("a cell-level list wins over what a robot says about its own frame", () => {
  const contexts = contextsFor(["Robot4"]);
  const offsets = [contexts[0].manifest.sharedEnvironment.offset];
  // Robot-local declarations are only consulted when the cell has none of its
  // own, so reaching that path means taking the cell's list away first.
  const [bare] = contexts.map((ctx) => ({
    manifest: {
      ...ctx.manifest,
      sharedEnvironment: { ...ctx.manifest.sharedEnvironment, taskSites: undefined }
    }
  }));
  const local = resolveCellSites([bare], offsets);
  assert.equal(local[0].robot, "Robot4", "the legacy form records whose frame it was read from");

  const cell = resolveCellSites(
    [{ manifest: { ...contexts[0].manifest, sharedEnvironment: { ...contexts[0].manifest.sharedEnvironment, taskSites: parseCellSites("moved@3,3,0.08,15") } } }],
    offsets
  );
  assert.equal(cell.length, 1);
  assert.equal(cell[0].alias, "moved");
  assert.deepEqual(cell[0].position, [3, 3, 0.08]);
  assert.equal(cell[0].robot, undefined, "a cell site has no declaring robot; that is the point");
});

test("exactly one site starts loaded", () => {
  assert.equal(loadedSite(parseCellSites("a@1,0,0.08; b@2,0,0.08")), null);
  assert.equal(loadedSite(parseCellSites("a@1,0,0.08,cube; b@2,0,0.08")).alias, "a");
  assert.throws(
    () => loadedSite(parseCellSites("a@1,0,0.08,cube1; b@2,0,0.08,cube2")),
    /exactly one site starts loaded/,
    "two payloads would make the origin of the transfer a matter of declaration"
  );
});

test("resolving a cell-level list needs no robot to interpret it", () => {
  // Robot bases move; the sites do not follow them. That is the difference
  // between a site placed in the cell and a site placed on an arm.
  const sites = parseCellSites("b@1.25,-2,0.08,90");
  const near = resolveCellSites(cellOf(sites), [[0, 0, 0]]);
  const far = resolveCellSites(cellOf(sites), [[10, -7, 0]]);
  assert.deepEqual(near[0].position, far[0].position);
  assert.equal(near[0].yawDeg, 90);
});

test("the driver turns a site as well as placing it", () => {
  const { prepareRuntimeDriverScript } = require("./core/driverScript");
  const [ctx] = contextsFor(["Robot4"]);
  const [site] = parseCellSites("turned@1.25,-2,0.08,90");
  const script = prepareRuntimeDriverScript(ctx, { taskSites: [site] });
  // Degrees in the declaration, radians in the scene -- converted once, here.
  assert.match(script, /alias=\[\[turned\]\],size=\{0\.14,0\.14,0\.16\},position=\{1\.25,-2,0\.08\},yaw=1\.5707/);
  assert.match(
    script,
    /function womatCreateGripCuboid\(alias, size, position, color, isStatic, respondable, yaw\)/
  );
  // Both the create path and the reposition path, or a site turns only the
  // first time the scene is built.
  const turns = script.match(/sim\.setObjectOrientation\((?:handle|object), sim\.handle_world, \{0, 0, turn\}\)/g);
  assert.equal(turns?.length, 2, "an existing site must be re-oriented too, not only a new one");
});

test("a loaded site is turned with everything standing on it", () => {
  const { prepareRuntimeDriverScript } = require("./core/driverScript");
  const [ctx] = contextsFor(["Robot4"]);
  const [site] = parseCellSites("a@0.9,0,0.08,30,cube_a");
  const script = prepareRuntimeDriverScript(ctx, { sharedGrip: gripFromSite(site) });
  const yaw = 30 * Math.PI / 180;
  assert.ok(
    script.includes(`support=womatCreateGripCuboid([[a]], {0.14,0.14,0.16}, {0.9,0,0.08}, {0.35,0.35,0.35}, true, true, ${yaw})`),
    "the support carries the declared yaw"
  );
  assert.ok(
    script.includes(`cube=womatCreateGripCuboid([[cube_a]], {0.045,0.045,0.045}, {0.9,0,0.185}, {0.9,0.25,0.1}, false, true, ${yaw})`),
    "and so does the cube standing on it, at the height the support puts it"
  );
});
