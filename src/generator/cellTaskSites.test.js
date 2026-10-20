"use strict";

const test = require("node:test");
const assert = require("node:assert");
const path = require("path");

const { collectCellTaskSites } = require("./core/cellHost");
const { createContext } = require("./core/context");
const { prepareRuntimeDriverScript } = require("./core/driverScript");

function contextsFor(names) {
  return names.map((name) => createContext(path.join("generated", "robots", name, "manifest.json")));
}

// The legacy form: a site declared on a robot, in that robot's frame. Stripping
// the cell's own list is what makes these tests about that path rather than
// about however the cell currently happens to be configured -- once a cell
// declares its sites, the robot-local ones are not consulted at all.
function robotLocalContextsFor(names) {
  return contextsFor(names).map((ctx) => ({
    ...ctx,
    manifest: {
      ...ctx.manifest,
      sharedEnvironment: { ...ctx.manifest.sharedEnvironment, taskSites: undefined }
    }
  }));
}

test("a robot's declared site is placed in the cell using its own base pose", () => {
  const contexts = robotLocalContextsFor(["Robot1", "Robot2", "Robot3", "Robot4"]);
  const offsets = contexts.map((ctx) => ctx.manifest.sharedEnvironment.offset);
  const sites = collectCellTaskSites(contexts, offsets);
  const siteB = sites.find((site) => site.alias === "Robot4_grip_support_b");
  assert.ok(siteB, "Robot4 declares the goal support");
  assert.equal(siteB.robot, "Robot4");
  // A site declared in the robot's frame is turned by the robot's yaw and then
  // offset by its base. Derived from the manifest rather than hardcoded: the
  // arm's yaw is a configuration choice that changes, and a test that assumes
  // one fails for the wrong reason when it does.
  const shared = contexts[3].manifest.sharedEnvironment;
  const [ox, oy, oz] = shared.offset.map(Number);
  const yaw = Number((shared.orientationDeg || [0, 0, shared.rotationDeg || 0])[2] || 0) * Math.PI / 180;
  const [lx, ly, lz] = contexts[3].manifest.runtime.taskSites[0].position.map(Number);
  const expected = [
    ox + Math.cos(yaw) * lx - Math.sin(yaw) * ly,
    oy + Math.sin(yaw) * lx + Math.cos(yaw) * ly,
    oz + lz
  ];
  siteB.position.forEach((value, axis) => {
    assert.ok(Math.abs(value - expected[axis]) < 1e-9, `axis ${axis} was ${value}, expected ${expected[axis]}`);
  });
  // And the site carries the arm's yaw, so a turned base turns the support with
  // it rather than leaving it square to the cell.
  assert.ok(Math.abs(siteB.yawDeg - Number((shared.orientationDeg || [0, 0, 0])[2] || 0)) < 1e-9);
});

test("every robot in the cell is given every site, not only its own", () => {
  const contexts = robotLocalContextsFor(["Robot1", "Robot4"]);
  const offsets = contexts.map((ctx) => ctx.manifest.sharedEnvironment.offset);
  const sites = collectCellTaskSites(contexts, offsets);
  // Robot1 declares none, yet the list handed to its driver still names
  // Robot4's -- otherwise Robot1 could not recognise the goal support at all.
  const script = prepareRuntimeDriverScript(contexts[0], { taskSites: sites, baseOffset: offsets[0] });
  assert.match(script, /womatTaskSiteDefinitions=\{\{alias=\[\[Robot4_grip_support_b\]\]/);
});

test("a site is declared once even if two robots name it", () => {
  const contexts = robotLocalContextsFor(["Robot4", "Robot4"]);
  const offsets = contexts.map((ctx) => ctx.manifest.sharedEnvironment.offset);
  assert.equal(collectCellTaskSites(contexts, offsets).length, 1);
});

test("a robot with no declared sites renders an empty list rather than a broken script", () => {
  const [ctx] = contextsFor(["Robot1"]);
  const script = prepareRuntimeDriverScript(ctx, { taskSites: [] });
  assert.match(script, /womatTaskSiteDefinitions=\{\}/);
});

test("the driver builds and recognises the declared sites", () => {
  const contexts = contextsFor(["Robot4"]);
  const script = prepareRuntimeDriverScript(contexts[0], {
    taskSites: [{ alias: "Robot4_grip_support_b", size: [0.14, 0.14, 0.16], position: [2.65, -1.6, 0.08] }]
  });
  // Created like the pick support, and classified as a support in the
  // inventory the agent reads -- both halves are needed for detection to work.
  assert.match(script, /function womatEnsureTaskSites\(\)/);
  assert.match(script, /womatEnsureTaskSites\(\)\s*\n\s*local gripper=womatEnsureTcpGripperModel\(\)/);
  assert.match(script, /elseif womatIsDeclaredTaskSite\(shape, context\) then\s*\n\s*kind="support"/);
  // The inventory has to carry geometry, or nothing downstream can decide
  // reachability or occupancy.
  assert.match(script, /world=placement\.world/);
  assert.match(script, /localPosition=placement\.localPosition/);
});

test("a standalone robot shifts its own declared sites into world coordinates", () => {
  const [ctx] = robotLocalContextsFor(["Robot4"]);
  // No cell host: the manifest's local (0.25, 0, 0.08) is placed by the
  // robot's own base offset, exactly as the pick support is.
  const script = prepareRuntimeDriverScript(ctx, { baseOffset: [1, 2, 0], baseYaw: 0, baseOrientation: [0, 0, 0] });
  // Derived from what the manifest says, so moving the support does not break
  // the test -- the arithmetic a placement error would get wrong is still checked.
  const [site] = ctx.manifest.runtime.taskSites;
  const [x, y, z] = site.position.map(Number);
  assert.ok(
    script.includes(`alias=[[${site.alias}]],size={${site.size.join(",")}},position={${x + 1},${y + 2},${z}}`),
    `site was not placed by the robot's own base offset: ${script.match(/alias=\[\[Robot4[^}]*\}/)}`
  );
});

// The pick point is a generated CONSTANT, and it was the one thing --sites did
// not move. Moving a support with the cube on it moved the scene, the obstacle
// list and every arm's site beliefs, while !graspCube kept sending the arm to
// where the support used to stand. Live cell: site A moved from x=0.9 to
// x=0.75 and Robot1 went on descending 15 cm past the cube.
test("the pick point follows the support the cube stands on, in each robot's own frame", () => {
  const { cellSiteGraspInRobotFrame } = require("./robwotBridge");
  const { normaliseSite } = require("./cellSites");

  const support = { position: [0.75, 0, 0.08], size: [0.14, 0.14, 0.16] };
  const site = normaliseSite({ alias: "site_a", ...support, payload: "cube_a" });

  // At the cell origin the two frames are the same, so the arithmetic is
  // visible: the declared z is the support's CENTRE, and the cube sits half a
  // support and half a cube above it. Reading it off the site is the whole
  // point -- a hardcoded 0.185 here would be the stale constant again.
  const half = support.size[2] / 2 + site.payload.size[2] / 2;
  const atOrigin = cellSiteGraspInRobotFrame(site, [0, 0, 0], [0, 0, 0]);
  assert.deepEqual(atOrigin.slice(0, 2), [0.75, 0]);
  assert.ok(
    Math.abs(atOrigin[2] - (support.position[2] + half)) < 0.005,
    `${atOrigin[2]} is not the top of a ${support.size[2]} m support plus half a cube`
  );

  // Displaced and turned: the same place, written in the arm's own coordinates.
  // Turning the robot must not move the cube, so the distance from its base is
  // the invariant, not any one coordinate.
  const turned = cellSiteGraspInRobotFrame(site, [1.5, 0.2, 0], [0, 0, Math.PI]);
  const distance = (point, base) => Math.hypot(
    site.payload.position[0] - base[0],
    site.payload.position[1] - base[1],
    site.payload.position[2] - base[2]
  ) - Math.hypot(point[0], point[1], point[2]);
  assert.ok(Math.abs(distance(turned, [1.5, 0.2, 0])) < 1e-6, "a yaw moved the cube relative to the base");
  assert.deepEqual(turned, [0.75, 0.2, 0.185]);

  // Every robot gets one, including the arms that cannot reach it. Which arm
  // serves the loaded site is discovered at runtime and must not be decided
  // here; all that is decided here is that an arm asked to pick is asked to
  // pick where the cube actually is.
  const contexts = contextsFor(["Robot1", "Robot2", "Robot3", "Robot4"]);
  for (const ctx of contexts) {
    assert.ok(
      Array.isArray(ctx.manifest.agentDefaults?.gripPosition),
      `${ctx.manifest.name} has no pick point`
    );
  }
});

test("every arm's recorded pick point is the cell's loaded site seen from where it stands", () => {
  const { cellSiteGraspInRobotFrame } = require("./robwotBridge");
  const { loadedSite, normaliseSite } = require("./cellSites");

  const contexts = contextsFor(["Robot1", "Robot2", "Robot3", "Robot4"]);
  const declared = contexts[0].manifest.sharedEnvironment?.taskSites;
  if (!Array.isArray(declared) || declared.length === 0) return;
  const loaded = loadedSite(declared.map(normaliseSite));
  if (!loaded) return;

  for (const ctx of contexts) {
    const shared = ctx.manifest.sharedEnvironment;
    const orientation = (shared.orientationDeg || [0, 0, shared.rotationDeg || 0])
      .map((degrees) => Number(degrees) * Math.PI / 180);
    assert.deepEqual(
      ctx.manifest.agentDefaults.gripPosition,
      cellSiteGraspInRobotFrame(loaded, shared.offset, orientation),
      `${ctx.manifest.name}'s pick point does not match ${loaded.alias}`
    );
  }
});
