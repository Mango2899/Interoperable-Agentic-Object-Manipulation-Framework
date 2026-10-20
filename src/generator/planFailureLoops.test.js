"use strict";

// Failure handlers that call subgoals, across every plan file at once.
//
// Jason looks for a failure plan by walking UP the intention, and it does NOT
// skip an intended means that is already a failure plan. So a subgoal that fails
// inside a `-!g` handler regenerates `-!g`, the handler runs again, its subgoal
// fails again, and the agent spins for as long as the run lasts. Measured
// against Jason 3.2.1 with a four-line agent: 47303 repetitions in 20 seconds.
//
// Live cell: -!placeCubeAt called !setPickupApproachTo directly. Robot5 could
// not place the cube, the disarm's five-second wait timed out because the lower
// layer was still planning a path it would not find, and the handler re-entered
// itself until the run was killed -- printing "Could not place the cube" forever
// while the arm waiting on it heard nothing at all.
//
// There are exactly two ways to be safe, and this file insists on one of them:
//
//   1. every subgoal the handler calls cannot fail -- it has a failure handler
//      of its own whose body does not propagate; or
//   2. the handler cannot be re-entered -- its guard requires a marker to be
//      absent and its body asserts that marker before anything else can fail,
//      so the second pass falls to a later clause.
//
// Both are already used in the codebase: -!approachRendezvous has always had (2)
// and -!handoverCubeTo's reroute clause uses it too. This test is what stops the
// next handler from being written without either.

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const AGT = path.resolve(__dirname, "../agt");
const files = [
  ...["handoverPlans.asl", "routePlans.asl", "neighborPlans.asl", "wotPlans.asl"].map((name) => path.join(AGT, name)),
  ...fs.readdirSync(path.join(AGT, "generated"))
    .filter((name) => name.endsWith("_agent.asl"))
    .map((name) => path.join(AGT, "generated", name))
];

// Comments and string literals are not code. A `!` inside either is not a call,
// and a `.fail` mentioned in a comment is not a propagation.
//
// A whole-line comment is removed WITH its newline. Blanking it in place left an
// empty line behind, and clauses here are delimited by a blank line -- so a plan
// with a comment in the middle of its body was analysed as far as that comment
// and no further. Every clause in this codebase is commented, so that silently
// hid most of what this file exists to check.
function strip(source) {
  return source
    .replace(/"(?:[^"\\]|\\.)*"/g, '""')
    .replace(/^[ \t]*\/\/.*\r?\n/gm, "")
    .replace(/\/\/.*$/gm, "");
}

function clauses(source) {
  const found = [];
  const pattern = /^([+-])!([a-zA-Z_][A-Za-z0-9_]*)\(/gm;
  let match = pattern.exec(source);
  while (match) {
    const start = match.index;
    const end = source.indexOf("\n\n", start);
    const text = source.slice(start, end < 0 ? source.length : end);
    const arrow = text.indexOf("<-");
    const head = arrow < 0 ? text : text.slice(0, arrow);
    const body = arrow < 0 ? "" : text.slice(arrow + 2);
    const colon = head.indexOf(":", head.indexOf(")"));
    found.push({
      kind: match[1], name: match[2], text, body,
      guard: colon < 0 ? "" : head.slice(colon + 1).trim()
    });
    match = pattern.exec(source);
  }
  return found;
}

const parsed = files.map((file) => ({ file, clauses: clauses(strip(fs.readFileSync(file, "utf8"))) }));
const every = parsed.flatMap((entry) => entry.clauses.map((clause) => ({ ...clause, file: entry.file })));

// A goal whose failure is handled without propagating: at least one `-!` clause
// whose body neither fails nor is empty.
const infallible = new Set(
  every.filter((clause) => clause.kind === "-" && !/\.fail\b/.test(clause.body)).map((clause) => clause.name)
);

// The marker pattern: the guard requires some belief to be absent and the body
// asserts it. Checked by functor, because the arguments differ (Session, Goal,
// ReceiverAgent) and it is the belief being claimed that makes it a one-shot.
function nonReentrant(clause) {
  for (const [, functor] of clause.guard.matchAll(/not\s+([a-z][A-Za-z0-9_]*)\s*\(/g)) {
    if (new RegExp(`\\+${functor}\\s*\\(`).test(clause.body)) return functor;
  }
  return null;
}

function subgoals(clause) {
  return [...new Set([...clause.body.matchAll(/!([a-zA-Z_][A-Za-z0-9_]*)\(/g)].map((m) => m[1]))]
    // Re-adopting the goal being handled is the ordinary retry, and it is bounded
    // by whatever guard let the clause run in the first place.
    .filter((name) => name !== clause.name);
}

test("no failure handler can be re-entered by a subgoal of its own failing", () => {
  const offenders = [];
  for (const clause of every) {
    if (clause.kind !== "-") continue;
    const called = subgoals(clause);
    if (!called.length) continue;
    if (nonReentrant(clause)) continue;
    const risky = called.filter((name) => !infallible.has(name));
    if (risky.length) {
      offenders.push(`${path.relative(AGT, clause.file)}: -!${clause.name} calls ${risky.join(", ")}`
        + " -- give those a failure handler that does not propagate, or guard this clause with a marker");
    }
  }
  assert.deepEqual(offenders, [], "\n" + offenders.join("\n"));
});

test("the placement failure that spun is guarded, and its tidy-up cannot fail", () => {
  const handlers = every.filter((clause) => clause.kind === "-" && clause.name === "placeCubeAt");
  assert.equal(handlers.length, 2, "one retry from upright, and a terminal clause behind it");

  // The retry re-adopts !placeCubeAt, so the marker is what bounds it. It also
  // happens to be what stops the loop, which is why the marker must go down
  // before anything in the body can fail.
  assert.equal(nonReentrant(handlers[0]), "placementResetTried");
  const retry = handlers[0].body;
  assert.ok(retry.indexOf("+placementResetTried") < retry.indexOf("!disarmPlacementApproach"));
  assert.ok(retry.indexOf("!returnToUprightWithCube") < retry.indexOf("!placeCubeAt"),
    "the point is asked again from upright, not from wherever the failure left the arm");

  // The terminal clause reports, tidies up with something that cannot fail, and
  // propagates.
  assert.deepEqual(subgoals(handlers[1]), ["disarmPlacementApproach"]);
  assert.match(handlers[1].body, /\.abolish\(placementResetTried\(Goal\)\)/);
  assert.match(handlers[1].body, /\.fail/);

  const plans = fs.readFileSync(path.join(AGT, "handoverPlans.asl"), "utf8");
  // Cleared when the episode ends, never at the top of +!placeCubeAt: the retry
  // goes back through that plan, so clearing it there would licence a second
  // retry and a third.
  assert.match(plans, /\.abolish\(placementResetTried\(Goal\)\);\s*\n\s*\.print\("Cube placed on "/);
  const attempt = plans.slice(plans.indexOf("+!placeCubeAt(Goal)"), plans.indexOf("// Lift clear of the cube just dropped"));
  assert.doesNotMatch(attempt.slice(0, attempt.indexOf("!setPickupApproachTo(true, Goal)")),
    /\.abolish\(placementResetTried/, "clearing the marker on entry would licence an unbounded retry");

  // The tidy-up is a goal with a handler that swallows, not a direct call to the
  // fallible one -- which must still fail loudly on the happy path.
  assert.ok(infallible.has("disarmPlacementApproach"));
  assert.ok(!infallible.has("setPickupApproachTo"));
  assert.match(plans, /\+!disarmPlacementApproach\(Goal\): true <-\s*\n\s*!setPickupApproachTo\(false, Goal\)\./);
});

test("the arm that ends the route places from where it stands, and resets only if that fails", () => {
  // The asymmetry with the forwarding clauses is deliberate. An arm that forwards
  // has to reach a rendezvous it has not solved yet and comes upright first; an
  // arm that places is going to a support it detected and can serve, and can
  // usually go there from where the handover left it. A reset on the way would be
  // a full traverse of the workspace paid for on every delivery.
  const plans = fs.readFileSync(path.join(AGT, "handoverPlans.asl"), "utf8");
  const chain = plans.split(/^\+!continueHandoverChain/m).slice(1);
  const sink = chain.find((clause) => /^\(Session, _\)\s*\n\s*: hasSink\(Goal\)/.test(clause));
  assert.ok(sink, "the sink clause must still be there");
  assert.doesNotMatch(sink, /!prepareToForwardCube/,
    "the placement goes straight from the rendezvous; the reset is the recovery, not the route");
  assert.match(sink, /!placeCubeAt\(Goal\)/);

  // Every clause that FORWARDS still comes upright, for the reason above.
  for (const clause of chain) {
    if (!/!handoverCubeTo\(/.test(clause)) continue;
    assert.match(clause, /!prepareToForwardCube/,
      "an arm handing the cube on must come upright with it before solving a rendezvous");
  }

  // And the reset survives as the recovery, which is what makes placing directly
  // safe to attempt: the first failure is answered by a different starting pose,
  // not by giving up on the support.
  const retry = every.find((clause) => clause.kind === "-" && clause.name === "placeCubeAt"
    && nonReentrant(clause) === "placementResetTried");
  assert.ok(retry, "the guarded recovery must still be there");
  assert.ok(retry.body.indexOf("!returnToUprightWithCube") < retry.body.indexOf("!placeCubeAt"),
    "come upright, then ask the same point again");

  // The measurement boundaries E12 and E13 cut here read the print and the
  // placement as adjacent lines.
  assert.match(plans, /\.print\("The goal ", Goal, " is mine; placing the cube on it"\);\s*\n\s*!placeCubeAt\(Goal\);/);
});

test("a failure handler tells its partner before it touches its own layer", () => {
  // The second half of the same lesson. A failure handler's job is to stop the
  // arm on the other side waiting; tidying up the layer is best-effort. With the
  // layer call first, the commonest failure -- the layer having just gone --
  // silenced the notification, and the partner waited out its whole deadline for
  // an arm that had already worked out it could not come.
  //
  // This became load-bearing when wotSend started failing an operation it could
  // not send: before that it returned normally and the .send always followed.
  const offenders = [];
  const notifying = every.filter((clause) =>
    /\.send\(/.test(clause.body) && /wotAchieve\(/.test(clause.body)
    && (clause.kind === "-" || ["abandonHandover"].includes(clause.name)));
  // Named rather than counted, so the check cannot quietly stop covering the one
  // that matters: a receiver whose layer has just been stopped is exactly the
  // E13 kill arm, and its giver waits 120 seconds if this message is lost.
  for (const name of ["receiveCube", "handoverCubeTo", "abandonHandover"]) {
    assert.ok(notifying.some((clause) => clause.name === name),
      `${name} both notifies and touches the layer; it must still be covered here`);
  }
  for (const clause of notifying) {
    // Per if-branch as well as overall: a send and a layer call inside the same
    // braces are ordered by that block, not by the plan.
    const blocks = [clause.body, ...[...clause.body.matchAll(/\{([^{}]*)\}/g)].map((m) => m[1])];
    for (const block of blocks) {
      const layer = block.indexOf("wotAchieve(");
      const send = block.lastIndexOf(".send(");
      if (layer === -1 || send === -1) continue;
      if (layer < send) {
        offenders.push(`${path.relative(AGT, clause.file)}: ${clause.kind}!${clause.name}`
          + " calls its own layer before telling its partner -- move the .send first");
      }
    }
  }
  assert.deepEqual([...new Set(offenders)], [], "\n" + [...new Set(offenders)].join("\n"));
});

test("the handlers that were already guarded still are", () => {
  // -!approachRendezvous is where the pattern comes from: one reset of the
  // posture, then the point is the point. If its marker ever goes, a rendezvous
  // the arm cannot reach becomes a loop instead of a refusal.
  const approach = every.filter((clause) => clause.kind === "-" && clause.name === "approachRendezvous");
  assert.ok(approach.length >= 2);
  assert.equal(nonReentrant(approach[0]), "rendezvousResetTried");
  assert.deepEqual(subgoals(approach.at(-1)), []);
  // The reroute clause re-adopts !handoverCubeTo, so its marker is what stops a
  // partner being struck twice as well as what stops the loop.
  const reroute = every.find((clause) => clause.kind === "-" && clause.name === "handoverCubeTo"
    && /partnerUnfit/.test(clause.guard));
  assert.ok(reroute, "the reroute recovery must still be there");
  assert.equal(nonReentrant(reroute), "rerouteAttempted");
});
