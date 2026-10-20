"use strict";

// Putting the cube down where the arm found the goal, rather than where somebody
// wrote the goal down.
//
// The old terminal behaviour returned the cube to constants -- and those
// constants existed in exactly one generated agent, because the two-robot demo
// only ever needed one. An arm that discovers the goal has the position already.

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const plans = fs.readFileSync(path.resolve(__dirname, "../agt/handoverPlans.asl"), "utf8");
const routes = fs.readFileSync(path.resolve(__dirname, "../agt/routePlans.asl"), "utf8");
const bridge = fs.readFileSync(path.resolve(__dirname, "agentThingServer.js"), "utf8");
const runtime = require("./core/runtimeSource").runtimeSource();

test("the placement reads the site the arm detected, not a written-down constant", () => {
  assert.match(plans, /\+!placeCubeAt\(Goal\)\s*\n\s*: site\(Goal, X, Y, Z\)/);
  // Move the support in the scene and the placement follows it.
  assert.match(plans, /!setPickupApproachTo\(true, Goal\);\s*\n\s*!moveTo\(X, Y, Z\);\s*\n\s*!openGripper/);
});

test("the placement is a pick run backwards, through the same planner", () => {
  // A hover pose and a vertical descent, arming and disarming around it, the
  // way !graspCube does -- not a general Cartesian move into a solid block.
  const place = plans.slice(plans.indexOf("+!placeCubeAt(Goal)"), plans.indexOf("-!placeCubeAt(Goal)"));
  assert.ok(place.indexOf("!setPickupApproachTo(true, Goal)") < place.indexOf("!moveTo(X, Y, Z)"));
  assert.ok(place.indexOf("!openGripper") < place.indexOf("!setPickupApproachTo(false, Goal)"));
  // Straight back up before any lateral move, once the block is an obstacle again.
  assert.match(place, /RetreatZ = Z \+ H;\s*\n\s*!moveTo\(X, Y, RetreatZ\)/);
  assert.ok(place.indexOf("!setPickupApproachTo(false, Goal)") < place.indexOf("RetreatZ = Z + H"));
});

test("a failed placement disarms the approach before the failure propagates", () => {
  // Leaving it armed silently reroutes every later Cartesian move through the
  // pick planner -- the same reason -!graspCube disarms.
  //
  // Through a goal of its own, and on BOTH failure clauses. Called directly, the
  // disarm's own five-second wait could fail -- which is ordinary while the lower
  // layer is still planning -- and a subgoal failing inside a -! handler
  // regenerates the event that ran the handler. See planFailureLoops.test.js.
  assert.match(plans, /\+!disarmPlacementApproach\(Goal\): true <-\s*\n\s*!setPickupApproachTo\(false, Goal\)\./);
  assert.match(plans, /-!disarmPlacementApproach\(Goal\): true <-/);
  // The two -!placeCubeAt clauses alone: the disarm's own definitions sit just
  // below them and would be counted as calls.
  const handlers = plans.slice(plans.indexOf("-!placeCubeAt(Goal)"), plans.indexOf("+!disarmPlacementApproach"));
  assert.equal((handlers.match(/!disarmPlacementApproach\(Goal\)/g) || []).length, 2,
    "the retry and the terminal clause both disarm");
  assert.match(handlers, /!disarmPlacementApproach\(Goal\);\s*\n\s*\.fail\./,
    "and the terminal clause disarms before it propagates");
});

test("the release is confirmed against authoritative ownership, not assumed", () => {
  const place = plans.slice(plans.indexOf("+!placeCubeAt(Goal)"), plans.indexOf("-!placeCubeAt(Goal)"));
  assert.match(place, /!refreshCubeOwner\(after_place\);\s*\n\s*\?cubeOwner\(none\)/);
});

test("only the block the approach is aimed at stops being an obstacle", () => {
  // Every other support in the cell, including this arm's own when it is
  // placing elsewhere, stays hard. And the exemption is gated on actually
  // holding the cube, so arming the flag alone permits nothing.
  assert.match(runtime, /function womatApproachSupportAlias\(\)/);
  assert.match(runtime, /approachSupport=womatObjectByAlias\(womatApproachSupportAlias\(\)\)/);
  const held = runtime.slice(runtime.indexOf("if heldGripObject ~= nil and heldGripObject ~= -1 then"));
  assert.match(held.slice(0, 600), /context\.approachSupport ~= -1 and womatIsDescendantOf\(shape, context\.approachSupport\)/);
});

test("an unset approach target still means this robot's own pick support", () => {
  // So a caller that only passes `enabled` still means exactly what a pick has
  // always meant.
  assert.match(runtime, /if womatPickupIgnoredSupportAlias ~= nil and womatPickupIgnoredSupportAlias ~= "" then\s*\n\s*return womatPickupIgnoredSupportAlias/);
  assert.match(bridge, /site \? \{ enabled, site \} : \{ enabled \}/);
});

test("the agent's atom is resolved back to the scene's alias", () => {
  // The agent reasons with v_Robot4_grip_support_b; the scene knows
  // Robot4_grip_support_b. The mapping was recorded during detection.
  assert.match(bridge, /this\.taskSiteAliases\?\.get\(String\(value\[1\]\)\)/);
});

test("an arm hands on only where it could actually hand over", () => {
  assert.match(routes, /canGiveTo\(Robot\) :- neighbor\(_, Robot\)/);
});
