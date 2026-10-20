const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const plansPath = path.resolve(__dirname, "../agt/handoverPlans.asl");
const plans = fs.readFileSync(plansPath, "utf8");
const generatorPath = path.resolve(__dirname, "generateRobot.js");
const generator = fs.readFileSync(generatorPath, "utf8");
const lowerRuntimePath = path.resolve(__dirname, "lowerThingServer.js");
// The runtime is the JavaScript plus the Lua driver it assembles from
// src/generator/lua, so assertions about either half read one surface.
const lowerRuntime = require("./core/runtimeSource").runtimeSource();
const bridgePath = path.resolve(__dirname, "robwotBridge.js");
const bridge = fs.readFileSync(bridgePath, "utf8");

function position(fragment) {
  const index = plans.indexOf(fragment);
  assert.notEqual(index, -1, `Expected handover plan fragment: ${fragment}`);
  return index;
}

test("receiver clears the interaction point before giver returns to joint home", () => {
  const receiverRetreat = position("!leaveRendezvous(Session, ReceiverX, ReceiverY, ReceiverZ)");
  const receiverSignal = position(".send(GiverAgent, tell, receiverRetreated(Session))");
  const receiverWait = position("\n    !awaitGiverClear(Session, GiverAgent);");
  const supportReturn = position("!returnCubeToSupport(Session)");

  assert.ok(receiverRetreat < receiverSignal);
  assert.ok(receiverSignal < receiverWait);
  assert.ok(receiverWait < supportReturn);
});

test("giver waits for receiver retreat, then releases the receiver once its own home motion is dispatched", () => {
  const giverWait = position(".wait(receiverRetreated(Session)[source(ReceiverAgent)], 90000)");
  const clearGoal = position("\n    !clearRendezvous(Session);");
  const giverSignal = position(".send(ReceiverAgent, tell, giverHome(Session))");

  assert.ok(giverWait < clearGoal);
  assert.ok(clearGoal < giverSignal);
  assert.match(
    plans,
    /\+!returnToDefaultJointPose: homeJoints\(HomeJoints\)[\s\S]*?wotAchieve\("ownLayer1", "goHomeJoints", HomeJoints\)/
  );
  // The giver backs out before it says so, and its home request is already with
  // the lower layer before the receiver hears anything -- wotAchieve dispatches
  // and returns, so releasing first handed the receiver the whole of the
  // giver's preparation as a head start to the arbiter.
  const clear = plans.slice(plans.indexOf("+!clearRendezvous(Session): true <-"),
    plans.indexOf("+!returnGiverHome(Session): homeJoints(HomeJoints) <-"));
  assert.ok(clear.indexOf("!backOutOfRendezvous(Session)") < clear.indexOf("!returnGiverHome(Session)"));
  assert.ok(!clear.includes("giverRetreated(Session)"), "the release is not made before the home motion is dispatched");
  const home = plans.slice(plans.indexOf("+!returnGiverHome(Session): homeJoints(HomeJoints) <-"),
    plans.indexOf("+!releaseReceiver(Session)"));
  assert.ok(home.indexOf('wotAchieve("ownLayer1", "goHomeAfterHandover", [Session, HomeJoints])')
    < home.indexOf("!releaseReceiver(Session)"), "dispatch, then release");
  assert.ok(home.indexOf("!releaseReceiver(Session)") < home.indexOf(".wait(movedTo(home)"), "release before blocking on arrival");
  // A giver that could not back out never claims it did, and a giver with no
  // home pose still lets the receiver go.
  const release = plans.slice(plans.indexOf("+!releaseReceiver(Session)"));
  assert.match(release, /not giverRetreated\(Session\)\s*\n\s*& not giverRetreatFailed\(Session\)/);
  assert.match(release, /\.send\(ReceiverAgent, tell, giverRetreated\(Session\)\)/);
  assert.match(plans, /\+!returnGiverHome\(Session\): true <-\s*\n\s*!releaseReceiver\(Session\);/);
});

test("retreat failures and timeouts never issue a Cartesian move back toward the rendezvous", () => {
  for (const call of ["!leaveRendezvous(Session, GiverX, GiverY, GiverZ)",
    "!leaveRendezvous(Session, ReceiverX, ReceiverY, ReceiverZ)"]) {
    assert.ok(plans.includes(call), "both arms use the recorded retreat");
  }
  const leave = plans.slice(plans.indexOf("+!leaveRendezvous(Session, X, Y, Z): true <-"),
    plans.indexOf("// Going to a rendezvous"));
  assert.match(leave, /wotAchieve\("ownLayer1", "retreatFromHandover", \[Session\]\)/);
  assert.match(leave, /if \(handoverRetreatFailed\(Session, Reason\)\) \{[^}]*\.fail/);
  assert.match(leave, /-!leaveRendezvous\(_, _, _, _\): true <-[\s\S]*?\.fail\./);
  assert.doesNotMatch(leave, /!moveTo|!retreatFromRendezvous/);
  assert.doesNotMatch(plans, /handoverBackoff|!retreatFromRendezvous/);
  // Failure bubbles to the giver's recovery, which permits checked joint home
  // but prevents the early clearance notification to the receiver.
  assert.match(plans, /\+!clearRendezvous\(Session\): true <-[\s\S]*?!backOutOfRendezvous\(Session\);[\s\S]*?!returnGiverHome\(Session\)\./);
  const recovery = plans.slice(plans.indexOf("-!backOutOfRendezvous(Session): true <-"));
  const body = recovery.slice(0, recovery.indexOf(".\n"));
  assert.match(body, /\+giverRetreatFailed\(Session\)/);
  assert.doesNotMatch(body, /\.fail|!moveTo/);
  assert.match(plans, /not giverRetreated\(Session\)\s*\n\s*& not giverRetreatFailed\(Session\)/);
});

test("a failure after the cube crossed is not a rendezvous to retry", () => {
  // Retrying reaches for a cube that is now in somebody else's gripper, fails
  // to hold it, and reports "failed before session creation" over a handover
  // that physically succeeded -- taking the receiver's chain down with it.
  const transferred = plans.indexOf("-!handoverCubeTo(ReceiverAgent, _)\n    : handoverSession(Session, _, _)\n      & cubeTransferred(Session)");
  const retry = plans.indexOf("-!handoverCubeTo(ReceiverAgent, ReceiverRobot)\n    : selectedHandoverCandidate(");
  assert.ok(transferred >= 0, "there must be a recovery for a failure after the transfer");
  assert.ok(retry >= 0, "the rendezvous retry must still exist");
  assert.ok(transferred < retry, "the post-transfer case has to be checked before the retry");
  const guard = plans.slice(retry, plans.indexOf("<-", retry));
  assert.match(guard, /not \(handoverSession\(Held, _, _\) & cubeTransferred\(Held\)\)/);
  // And the receiver is told what actually happened, not that the giver parked.
  // Scoped to that one clause: a later recovery in the same file is entitled to
  // report handoverFailed, and reading up to the retry would catch it.
  const afterTransfer = plans.slice(transferred, plans.indexOf("\n\n", transferred));
  assert.match(afterTransfer, /\.send\(ReceiverAgent, tell, giverStuck\(Session\)\)/);
  assert.doesNotMatch(afterTransfer, /handoverFailed/);
});

test("receiver waits for confirmed clearance and stops if the giver cannot clear", () => {
  // Jason's .wait(Cond, Timeout) FAILS on timeout -- it does not resume and
  // carry on. Both places that treat the timeout as the interesting case have
  // to sit their wait in a goal of its own, or the case they were written for
  // is the one case they cannot handle.
  assert.match(
    plans,
    /\+!awaitGiverClear\(Session, GiverAgent\): true <-\s*\n\s*\.wait\(giverRetreated\(Session\)\[source\(GiverAgent\)\] \|\s*\n\s*giverHome\(Session\)\[source\(GiverAgent\)\] \|\s*\n\s*giverStuck\(Session\)\[source\(GiverAgent\)\], \d+\)/
  );
  assert.match(plans, /-!awaitGiverClear\(Session, _\): true <-/);
  const recovery = plans.slice(plans.indexOf("-!awaitGiverClear(Session, _): true <-"));
  assert.match(recovery.slice(0, recovery.indexOf(".\n")), /\.fail/);
});

test("who may give is a property of the pair, not of who computed its space", () => {
  // Ownership decides which arm BUILDS and caches a shared space -- once,
  // deterministically, without negotiation. It was never meant to decide which
  // arm may be the giver, and a discovered route that needs the cube passed
  // toward the pair's non-owner needs exactly that. The space is refreshed
  // inside the plan before any rendezvous is chosen, so requiring it to exist
  // in the guard only kept non-owners out.
  assert.match(plans, /\+!handoverCubeTo\(ReceiverAgent, ReceiverRobot\)\s*\n\s*: agentIdentity\(Me, MyRobot\)\s*\n\s*& canGiveTo\(ReceiverRobot\)/);
  const guard = plans.slice(plans.indexOf("+!handoverCubeTo(ReceiverAgent, ReceiverRobot)"));
  assert.doesNotMatch(guard.slice(0, guard.indexOf("<-")), /interactionSpaceOwner/);
  // Ownership still decides who computes, and the arity-1 form stays gone.
  assert.match(plans, /interactionSpaceOwner\(MyRobot, NextRobot\)/);
  assert.doesNotMatch(plans, /interactionSpaceOwner\(MyRobot\)\s*&/);
});

test("a pair that shares nothing is answered, not waited out", () => {
  // The estimator reaches "these two share no voxel" in about a second and
  // says so as interactionSpaceFailed. Waiting only for interactionSpaceReady
  // meant that verdict sat unread in the belief base until .wait hit its
  // 60-second timeout, and every pair queued behind it waited too. Live cell:
  // Robot1 was told voxel_intersection_empty for Robot3 in 1.3s and did not
  // start on Robot2 for another 59.
  for (const goal of ["buildInteractionSpace", "refreshInteractionSpace"]) {
    const body = plans.slice(plans.indexOf(`+!${goal}(ReceiverRobot)`));
    const wait = body.slice(0, body.indexOf("\n\n"));
    assert.match(
      wait,
      /\.wait\(interactionSpaceReady\(Space, ReceiverRobot\) \| interactionSpaceFailed\(ReceiverRobot, _\), 60000\);/,
      `${goal} must wake on either answer`
    );
    // And a refusal must not read as a success: the plan fails so the pair's
    // handler reports it, rather than falling through to tell the neighbour a
    // space is ready when none was built.
    assert.match(
      wait,
      /if \(interactionSpaceFailed\(ReceiverRobot, _\)\) \{ \.fail \};/,
      `${goal} must fail on a refusal instead of announcing a space`
    );
  }
});

test("the space a neighbour is told about is described in the neighbour's frame", () => {
  // "The space with ReceiverRobot is ready" is true for the sender and useless
  // to the receiver, who IS ReceiverRobot. It needs to know the space with the
  // SENDER is ready before it can ever hand back the other way.
  assert.doesNotMatch(plans, /\.send\(ReceiverAgent, tell, interactionSpaceReady\(Space, ReceiverRobot\)\)/);
  assert.match(plans, /\.send\(ReceiverAgent, tell, interactionSpaceReady\(Space, MyRobot\)\)/);
});

test("the pair owner evaluates central-core candidates in BDI reasoning", () => {
  const candidateRequest = position('wotAchieve("ownLayer1", "rankHandoverCandidates", [ReceiverRobot, MinZ, MaxZ])');
  const agentScoring = position("Score = 0.32 * Centrality + 0.2 * Density");
  const agentMaximum = position(".max(Rankings, rank(BestScore, BestCandidate))");
  const commit = position('wotAchieve("ownLayer1", "commitHandoverCandidate", [ReceiverRobot, BestCandidate])');

  assert.ok(candidateRequest < agentScoring);
  assert.ok(agentScoring < agentMaximum);
  assert.ok(agentMaximum < commit);
  assert.doesNotMatch(plans, /math\.random/);

  // Reachability of the rendezvous point is not sufficient: the approach LINE
  // has to stay clear of each arm's own base-joint axis, and only the geometry
  // capability can measure that. The agent still owns the weighting.
  assert.match(plans, /ReceiverSupport, Posture\)/);
  assert.match(plans, /\+ 0\.14 \* GiverSupport \+ 0\.14 \* ReceiverSupport \+ 0\.2 \* Posture/);
  // Every scored term must be abolished between runs or a stale candidate set
  // outranks the fresh one.
  assert.match(plans, /\.abolish\(handoverCandidate\(_, ReceiverRobot(?:, _){14}\)\)/);
});

test("an intermediate receiver adopts a downstream BDI handover intention", () => {
  assert.match(
    plans,
    /\+!continueHandoverChain\(Session, _\)\s*\n\s*: nextHop\(_, NextAgent, NextRobot, _\)[\s\S]*?canGiveTo\(NextRobot\)/
  );
  assert.ok(position("!continueHandoverChain(Session, GiverAgent)")
    < position(".send(GiverAgent, tell, handoverComplete(Session))"));

  // Checked inside each forwarding clause rather than by position in the file:
  // there are two of them now -- the discovered route and the configured
  // baseline -- and a global ordering check silently compares one clause's
  // opening against the other's body.
  const clauses = plans
    .split("+!continueHandoverChain(Session, _)")
    .slice(1)
    // Each clause ends where the next plan begins; without that bound the last
    // slice runs to the end of the file and swallows the failure handlers.
    .map((clause) => {
      const next = clause.search(/\n[+-]!/);
      return next === -1 ? clause : clause.slice(0, next);
    })
    .filter((clause) => clause.includes("!handoverCubeTo("));
  assert.equal(clauses.length, 2, "a discovered-route clause and a configured baseline clause");

  for (const clause of clauses) {
    const body = clause.slice(0, clause.indexOf("+handoverStage(Session, complete)"));
    const upright = body.indexOf("!prepareToForwardCube;");
    const cleared = body.indexOf(".abolish(handoverSession(Session, _, _));");
    const adopted = body.indexOf("!handoverCubeTo(");
    assert.ok(upright !== -1 && cleared !== -1 && adopted !== -1, `incomplete forwarding clause: ${body}`);
    // The arm prepares its posture according to support reachability, and the
    // incoming session must be gone before the downstream intention is adopted
    // -- !handoverCubeTo is guarded on `not handoverSession(_, _, _)`, so any
    // session left standing makes the retry inapplicable and the forward fails
    // through without ever entering the plan body.
    assert.ok(upright < cleared, "the arm must prepare to forward first");
    assert.ok(cleared < adopted, "the incoming session must be cleared before forwarding");
  }
});

test("intermediate robot verifies upright pose and cube ownership before forwarding", () => {
  assert.match(
    plans,
    /\+!returnToUprightWithCube[\s\S]*?homeJoints\(HomeJoints\)[\s\S]*?cubeOwner\(MyRobot\)[\s\S]*?wotAchieve\("ownLayer1", "goHomeJoints", HomeJoints\)[\s\S]*?\?cubeOwner\(MyRobot\)[\s\S]*?!verifyGripperHolding\(after_intermediate_upright\)/
  );
});

test("only a terminal receiver returns the cube to the support", () => {
  // The arm holding the goal is the terminal one, and it is the goal it
  // detected that says so -- not the absence of a configured successor.
  assert.match(plans, /\+!continueHandoverChain\(Session, _\)\s*\n\s*: hasSink\(Goal\)[\s\S]*?!placeCubeAt\(Goal\)/);
  // The configured chain survives below it as the comparison baseline.
  assert.match(plans, /\+!continueHandoverChain\(Session, _\)\s*\n\s*: not handoverSuccessor\(_, _\)[\s\S]*?!returnCubeToSupport\(Session\)/);
});

test("BDI handover waits accommodate planned motion and react to peer failure", () => {
  assert.match(
    plans,
    /\.wait\(receiverReady\(Session\)\[source\(ReceiverAgent\)\] \|[\s\S]*?handoverFailed\(Session, _\)\[source\(ReceiverAgent\)\], 120000\)/
  );
  assert.match(
    plans,
    /\.wait\(cubeTransferred\(Session\)\[source\(GiverAgent\)\] \|[\s\S]*?handoverFailed\(Session, _\)\[source\(GiverAgent\)\], 30000\)/
  );
});

test("giver failures before session creation have a BDI contingency plan", () => {
  assert.match(
    plans,
    /-!handoverCubeTo\(ReceiverAgent, _\): not handoverSession\(_, _, _\)[\s\S]*?handoverFailed\(before_session, giver_failed_before_session\)/
  );
});

test("generated gripper plans react to failure beliefs without waiting for success timeout", () => {
  assert.match(generator, /\.wait\(grippedObject\(_\) \| gripFailed\(_\), 15000\)/);
  assert.match(generator, /\.wait\(releasedObject\(_\) \| gripFailed\(_\), 15000\)/);
});

test("generated motion plans react to failure beliefs without waiting for success timeout", () => {
  assert.match(generator, /\.wait\(movedTo\(X, Y, Z\) \| motionFailed\(_\), 60000\)/);
  assert.match(plans, /\.wait\(movedTo\(home\) \| motionFailed\(_\), 60000\)/);
});

test("planned motion drives kinematic joints and verifies the physical goal", () => {
  assert.match(lowerRuntime, /sim\.isDynamicallyEnabled\(jointHandles\[i\]\)[\s\S]*?sim\.setJointPosition/);
  assert.match(lowerRuntime, /planned_cartesian_target_not_reached/);
  assert.match(bridge, /td\.actions\.moveTocartesianPosition\.output = motionOutput/);
  assert.match(lowerRuntime, /cartesianFinalTolerance: Number\(runtime\.cartesianFinalTolerance \|\| 0\.025\)/);
  // Clearance is measured against the obstacles this arm is not resting on --
  // a floor-mounted arm is permanently inside any margin of its own floor --
  // while contact is still tested against every obstacle.
  assert.match(lowerRuntime, /pcall\(sim\.checkDistance, robotCollection, clearanceCollection, clearance\)/);
  assert.match(lowerRuntime, /if womatCollectionsTouch\(robotCollection, obstacleCollection\) then/);
  assert.match(lowerRuntime, /local maxJointStep=math\.min\(velocityStep, collisionStep\)/);
});

test("pickup approach is controlled by BDI while support contact remains forbidden", () => {
  // The arm/move/close/disarm/lift sequence lives in exactly one plan,
  // !graspCube, so a standalone pick and the first leg of a handover cannot
  // drift apart -- which they did while the lift was the caller's job.
  assert.match(generator, /\+!graspCube: gripTargetPosition\(X, Y, Z\) & pickLiftHeight\(L\) <-\n    !setPickupApproach\(true\);\n    !moveTo\(X, Y, Z\);\n    !closeGripper;\n    !setPickupApproach\(false\);\n(?:.*\n)*?    LiftZ = Z \+ L;\n    !moveTo\(X, Y, LiftZ\)\./);
  assert.match(plans, /!graspCube;/);
  assert.doesNotMatch(plans, /!setPickupApproach\(true\)/);
  // A failed grasp must not leave the flag armed: it silently reroutes every
  // later Cartesian move through the pick planner.
  assert.match(generator, /-!graspCube: true <-[\s\S]*?!setPickupApproach\(false\);\n    \.fail\./);
  assert.match(lowerRuntime, /if clearance > 0 and not womatPickupApproachEnabled then/);
  assert.doesNotMatch(lowerRuntime, /if womatPickupApproachEnabled then[\s\S]*?womatIsDescendantOf\(shape, gripSupport\)/);
  assert.match(lowerRuntime, /womatSetPickupApproach\(enabled, siteAlias\)/);
  // The approach may be aimed at another support -- that is what placing onto
  // the goal is -- but the exemption stays gated on actually holding the cube,
  // so arming the flag alone never permits contact with anything.
  assert.match(lowerRuntime, /if heldGripObject ~= nil and heldGripObject ~= -1 then[\s\S]*?context\.approachSupport/);
});

test("pickCube is wired end to end, and the agent lifts clear by its own lift height", () => {
  const agentThing = fs.readFileSync(path.resolve(__dirname, "agentThingServer.js"), "utf8");

  // HTTP action -> Jason goal -> plan. A break anywhere in this chain shows up
  // as an action that returns 200 and does nothing at all.
  assert.match(generator, /pickCube: \{[\s\S]*?href: "\/actions\/pickCube"/);
  assert.match(agentThing, /setActionHandler\("pickCube"/);
  assert.match(agentThing, /this\.forwardAchieveToAgent\("pickCube"\)/);
  assert.match(generator, /\+!pickCube: carryPosition\(CX, CY, CZ\)/);

  // The lift belongs to the GRASP, not to what follows it. Every caller of
  // !graspCube leaves the support the same way -- a standalone pick and a
  // handover giver alike -- because the pick is what put the arm in the hazard.
  // Owned by the caller instead, the giver went straight from the grasp to the
  // rendezvous, dragging cube and fingers across the support.
  assert.match(generator, /\+!graspCube: gripTargetPosition\(X, Y, Z\) & pickLiftHeight\(L\)/);
  assert.match(generator, /LiftZ = Z \+ L;\n    !moveTo\(X, Y, LiftZ\)\./);
  assert.doesNotMatch(generator, /\+!pickCube:[\s\S]*?RetreatZ/,
    "the lift must not be repeated by the caller");

  // Its own height, not the approach height: the approach height is a property
  // of the plan -- where the hover sits and how far the descent travels --
  // while the lift only has to clear the support.
  assert.match(generator, /const pickLiftHeight = 0\.10;/);
  assert.match(generator, /pickLiftHeight\(\$\{pickLiftHeight\}\)\./);
  assert.match(generator, /^    pickLiftHeight,$/m);

  // The approach height is still emitted -- the lower Thing's hover depends on
  // it -- it is simply no longer what the retreat is measured by.
  //
  // Read back from an existing manifest rather than reset to the default: the
  // default is a six-axis arm's, and an arm tuned to a shorter hover had the
  // agent's copy regenerated at 0.20 while the lower Thing kept reading its own.
  assert.match(
    generator,
    /existingManifest\?\.runtime\?\.gripper\?\.pickupApproachHeight \?\? 0\.20/,
    "the hover height must survive regeneration"
  );
  assert.match(generator, /pickupHoverHeight\(\$\{pickupHoverHeight\}\)\./);
  assert.match(generator, /^        pickupApproachHeight,$/m);

  // And every generated agent actually carries the beliefs the plan needs.
  for (const robot of ["robot1", "robot2", "robot3"]) {
    const asl = fs.readFileSync(path.resolve(__dirname, `../agt/generated/${robot}_agent.asl`), "utf8");
    assert.match(asl, /^pickupHoverHeight\(0\.2\)\.$/m, `${robot} must know the hover height`);
    assert.match(asl, /^pickLiftHeight\(0\.1\)\.$/m, `${robot} must know the lift height`);
    assert.match(asl, /\+!pickCube:/, `${robot} must define !pickCube`);
    assert.match(asl, /\+!graspCube:/, `${robot} must define !graspCube`);
    assert.match(asl, /LiftZ = Z \+ L;\n    !moveTo\(X, Y, LiftZ\)\./,
      `${robot} must lift clear of the support as part of the grasp`);
  }
});

// The invariant the two halves of the bridge have to hold, checked against the
// robots that actually exist rather than against the generator's default. The
// lower Thing places its hover at runtime.gripper.pickupApproachHeight and the
// agent retreats to pickupHoverHeight; a disagreement puts the retreat
// somewhere the descent never cleared, and on a small arm it puts it outside
// the envelope entirely -- which fails a handover whose cube is already placed.
test("every generated agent retreats to the hover height its lower Thing uses", () => {
  const robotsDir = path.resolve(__dirname, "../../generated/robots");
  if (!fs.existsSync(robotsDir)) return;
  const manifests = fs.readdirSync(robotsDir)
    .map((name) => ({ name, file: path.join(robotsDir, name, "manifest.json") }))
    .filter((robot) => fs.existsSync(robot.file));
  assert.ok(manifests.length > 0, "expected at least one generated robot");

  for (const robot of manifests) {
    const manifest = JSON.parse(fs.readFileSync(robot.file, "utf8"));
    const aslFile = path.resolve(__dirname, `../agt/generated/${robot.name.toLowerCase()}_agent.asl`);
    if (!fs.existsSync(aslFile)) continue;
    const declared = /^pickupHoverHeight\(([-\d.]+)\)\.$/m.exec(fs.readFileSync(aslFile, "utf8"));
    assert.ok(declared, `${robot.name} must declare pickupHoverHeight`);
    assert.equal(
      Number(declared[1]),
      Number(manifest.runtime?.gripper?.pickupApproachHeight),
      `${robot.name}: agent hover height must match runtime.gripper.pickupApproachHeight`
    );
  }
});

test("forwarding a goal fails loudly when no Jason agent is attached", () => {
  const agentThing = fs.readFileSync(path.resolve(__dirname, "agentThingServer.js"), "utf8");

  // The connector broadcasts over a list of websocket clients. Empty list, no
  // error -- the action returns 204 and the goal is dropped, which is
  // indistinguishable from a robot that simply refused to move.
  assert.match(agentThing, /forwardAchieveToAgent\(keyword, values = null\)/);
  assert.match(agentThing, /readyState === 1/);
  assert.match(agentThing, /if \(attached === 0\) \{[\s\S]*?throw new Error\(message\);/);

  // And every HTTP action that hands a goal to the agent goes through it, so
  // none of them can regress to a silent send.
  for (const action of ["horizontalDemo", "moveTo", "goHome", "pickCube", "closeGripper", "openGripper"]) {
    assert.match(
      agentThing,
      new RegExp(`setActionHandler\\("${action}"[\\s\\S]{0,200}?this\\.forwardAchieveToAgent\\("${action}"`),
      `${action} must forward through the guarded helper`
    );
  }
  assert.doesNotMatch(agentThing, /setCustomInteractionAffordanceHandlers[\s\S]*?agentConnector\.send2Agent/);
});

test("a refused rendezvous is struck and the next-best one tried", () => {
  // Jason takes the first applicable plan, so the retry has to sit ahead of
  // both terminal failure plans. Behind them it is dead code and a blocked
  // handover still gives up on its first refused point.
  const retry = position("-!handoverCubeTo(ReceiverAgent, ReceiverRobot)");
  const terminalWithSession = position("-!handoverCubeTo(ReceiverAgent, _): handoverSession(Session, _, _)");
  const terminalWithout = position("-!handoverCubeTo(ReceiverAgent, _): not handoverSession(_, _, _)");
  assert.ok(retry < terminalWithSession, "the retry plan must precede the terminal session failure plan");
  assert.ok(terminalWithSession < terminalWithout);

  // The refusal has to change the next choice, or the retry re-picks the point
  // that was just refused and loops until the budget runs out.
  const rejected = position("+handoverCandidateRejected(Candidate, ReceiverRobot)");
  const filtered = position("& not handoverCandidateRejected(Candidate, ReceiverRobot)");
  assert.ok(filtered < rejected, "the ranking must filter on the same belief the failure records");

  // Bounded, and the bound must be decremented on the retry path.
  assert.ok(plans.includes("& handoverRetriesLeft(Left)\n      & Left > 0"));
  assert.ok(plans.includes("Next = Left - 1"));
  // And released once a handover actually completes, so the next one starts
  // with a full budget and no inherited rejections.
  const cleared = position(".abolish(handoverRetriesLeft(_))");
  assert.ok(cleared < retry, "the budget is released on completion, before the failure plans");

  // Running out of candidates must be a distinct, explained failure rather
  // than .max failing on an empty list.
  assert.ok(plans.includes(".length(Rankings, Remaining)"));
  assert.ok(position(".length(Rankings, Remaining)") < position(".max(Rankings, rank(BestScore, BestCandidate))"));
});

test("an arm that cannot reach a rendezvous resets its posture and asks again", () => {
  // The ranking scores the last stretch into a rendezvous -- the standoff leg
  // along the pinned tool axis. Nothing scores the long leg from wherever the
  // arm is parked, and that is the one that fails: a straight line out of a
  // pose folded in near the base passes back over the base. Live cell: Robot4
  // refused at sample 1 of 14, 3.6 cm into a 51 cm journey, baseVisible against
  // its own link 3 -- a fact about where it stood, not about the point.
  assert.match(plans, /\+!approachRendezvous\(_, X, Y, Z\): true <-\s*\n\s*!moveTo\(X, Y, Z\)\./);
  assert.match(
    plans,
    /-!approachRendezvous\(Session, X, Y, Z\)\s*\n\s*: not rendezvousResetTried\(Session\)\s*<-\s*\n\s*\+rendezvousResetTried\(Session\);[\s\S]*?!resetPostureForApproach;\s*\n\s*!moveTo\(X, Y, Z\)\./,
    "the retry must mark itself, reset, and ask the SAME point again"
  );
  // Exactly one second chance, then the refusal is about the point.
  const giveUp = plans.slice(plans.indexOf("-!approachRendezvous(_, _, _, _): true <-"));
  assert.ok(giveUp.length > 0, "a second failure must end the approach, not loop");
  assert.match(giveUp.slice(0, giveUp.indexOf(".\n")), /\.fail/);

  // Both roles go through it, so neither burns a candidate on its own posture.
  assert.match(plans, /!approachRendezvous\(Session, GiverX, GiverY, GiverZ\)/);
  assert.match(plans, /!approachRendezvous\(Session, ReceiverX, ReceiverY, ReceiverZ\)/);
  assert.doesNotMatch(plans, /!moveTo\(GiverX, GiverY, GiverZ\)/);
  assert.doesNotMatch(plans, /!moveTo\(ReceiverX, ReceiverY, ReceiverZ\)/);

  // Resetting mid-carry and resetting empty-handed are different manoeuvres.
  assert.match(
    plans,
    /\+!resetPostureForApproach\s*\n\s*: agentIdentity\(_, MyRobot\) & cubeOwner\(MyRobot\)\s*<-\s*\n\s*!returnToUprightWithCube\./
  );
  assert.match(plans, /\+!resetPostureForApproach: true <-\s*\n\s*!returnToDefaultJointPose\./);

  // The marker is per approach, not per lifetime: both roles clear it on entry.
  const giver = plans.indexOf("+!handoverCubeTo(ReceiverAgent, ReceiverRobot)");
  assert.match(plans.slice(giver, plans.indexOf("!refreshCubeOwner(before_handover)", giver)), /\.abolish\(rendezvousResetTried\(_\)\)/);
});

test("a receiver that cannot take a request says so instead of going quiet", () => {
  // Silence is indistinguishable from a slow approach: the giver waits out its
  // whole deadline, and the log shows an arm that never tried with no reason
  // recorded anywhere.
  const busy = plans.indexOf("+!receiveCube(Session, GiverAgent, _, _, _, _, _, _, _)\n    : handoverSession(Other, _, _)");
  assert.ok(busy >= 0, "there must be an answer for a request that cannot be taken up");
  const main = plans.indexOf("+!receiveCube(Session, GiverAgent, GiverRobot,");
  assert.ok(main < busy, "the busy refusal must sit AFTER the plan that does the work");
  assert.match(plans.slice(busy), /\.send\(GiverAgent, tell, handoverFailed\(Session, receiver_busy\)\)/);
});

test("the chosen rendezvous is reported with how many it was chosen from", () => {
  // Two retries exist to strike a refused rendezvous off and take the next one.
  // On a pair with a single candidate there is no next one, and the score alone
  // reads the same either way.
  assert.match(
    plans,
    /\.print\("BDI selected central handover candidate ", BestCandidate, " score=", BestScore,\s*\n\s*" \(of ", Remaining, " still usable\)"\)/
  );
});


test("forwarding policy restores home when requested and otherwise preserves direct carriers", () => {
  const body = plans.slice(plans.indexOf("+!prepareToForwardCube: true <-"), plans.indexOf("// Reset with the grip"));
  assert.match(body, /"getHandoverPolicy"/);
  assert.match(body, /handoverReturnHome\(true\) \| siteReachable\(_\)/);
  assert.match(body, /!returnToUprightWithCube/);
  assert.match(body, /Continuing directly to the next handover/);
});

// ---------------------------------------------------------------------------
// The partner, not the point.
// ---------------------------------------------------------------------------

test("a receiver that never answers is a fact about the arm, not about the rendezvous", () => {
  // Jason's .wait FAILS on timeout and does not resume, so a bare wait in the
  // caller's body cannot tell a refusal from silence -- it takes the handover
  // down identically either way, which is the one case it was written for.
  assert.match(plans, /\+!awaitReceiverReady\(Session, ReceiverAgent, _\): true <-\s*\n\s*\.wait\(receiverReady\(Session\)\[source\(ReceiverAgent\)\] \|\s*\n\s*handoverFailed\(Session, _\)\[source\(ReceiverAgent\)\], \d+\)/);
  const refused = plans.slice(plans.indexOf("+!awaitReceiverReady(Session, ReceiverAgent, _): true <-"));
  assert.match(refused.slice(0, refused.indexOf(".\n\n")), /\+partnerUnfit\(ReceiverAgent, ReceiverReason\)[\s\S]*?\.fail/);
  // Silence is the timeout, and it says the same thing: an arm whose agent has
  // stopped reasoning and one that was never started are indistinguishable from
  // here, and neither will take the cube at any point in the shared space.
  const silent = plans.slice(plans.indexOf("-!awaitReceiverReady(_, ReceiverAgent, ReceiverRobot): true <-"));
  const body = silent.slice(0, silent.indexOf(".\n\n"));
  assert.match(body, /if \(not partnerUnfit\(ReceiverAgent, _\)\)/, "the refusal already recorded is not overwritten");
  assert.match(body, /\+partnerUnfit\(ReceiverAgent, silent\)/);
  assert.match(body, /\.fail/, "and the handover still fails");
  // The caller waits through the goal, never inline.
  assert.match(plans, /!awaitReceiverReady\(Session, ReceiverAgent, ReceiverRobot\);/);
});

test("an unfit partner reroutes; only a refused rendezvous is retried", () => {
  const reroute = plans.indexOf("-!handoverCubeTo(ReceiverAgent, ReceiverRobot)\n    : partnerUnfit(");
  const retry = plans.indexOf("-!handoverCubeTo(ReceiverAgent, ReceiverRobot)\n    : selectedHandoverCandidate(");
  assert.ok(reroute >= 0, "there must be a recovery for an arm that cannot take the cube");
  assert.ok(reroute < retry, "the partner is judged before the point");
  // And not only by clause order: spending the rendezvous retries on an arm that
  // is not there costs two more deadlines and ends where it started.
  const retryGuard = plans.slice(retry, plans.indexOf("<-", retry));
  assert.match(retryGuard, /not partnerUnfit\(ReceiverAgent, _\)/);

  const body = plans.slice(reroute, plans.indexOf("\n\n// Everything this arm set up", reroute));
  // One reroute per partner, marked before anything else happens: this clause
  // re-adopts !handoverCubeTo, so its own failure comes back through here.
  assert.match(plans.slice(reroute, plans.indexOf("<-", reroute)), /not rerouteAttempted\(ReceiverAgent\)/);
  assert.ok(body.indexOf("+rerouteAttempted(ReceiverAgent)") < body.indexOf("!abandonHandover"),
    "the mark goes down before anything that can fail");
  // The cube never left the gripper, so the recovery is a change of partner and
  // not an undo of any motion.
  assert.match(body, /!rerouteAround\(ReceiverAgent, ReceiverRobot\)/);
  assert.match(body, /if \(nextHop\(_, NewAgent, NewRobot, NewTotal\)\)/);
  assert.match(body, /!handoverCubeTo\(NewAgent, NewRobot\)/);
  // Upright with the cube first, exactly as the chain does between two hops: the
  // arm is stretched out at the rendezvous it went to, and the approach to the
  // next one is a straight leg in that plans badly from there.
  assert.ok(body.indexOf("!prepareToForwardCube") < body.indexOf("!handoverCubeTo(NewAgent, NewRobot)"));
  assert.match(body, /\+deliveryAbandoned\(ReceiverRobot\)/, "and says so when there is nowhere left to go");
  assert.doesNotMatch(body, /!openGripper|!moveTo/, "nothing is released at a rendezvous nobody came to");
});

test("the arm that refused travels with the cube", () => {
  // Every arm ahead settled its own next hop before anybody knew, so a hop
  // struck here has to be struck by each of them too -- otherwise the cube is
  // carried past the failure and handed straight back towards the arm that
  // refused it. Free, and it rides on the handover that was happening anyway.
  const reroute = plans.slice(plans.indexOf("-!handoverCubeTo(ReceiverAgent, ReceiverRobot)\n    : partnerUnfit("));
  assert.match(reroute.slice(0, reroute.indexOf("\n\n")), /\.send\(NewAgent, tell, armUnfit\(ReceiverRobot\)\)/);
  // And the arm that is told acts on it before following a route that names it.
  const chain = plans.slice(plans.indexOf("+!continueHandoverChain(Session, Ignored)\n    : nextHop("));
  const clause = chain.slice(0, chain.indexOf("\n\n"));
  // Either reason the cell ever has for believing no route runs through an arm:
  // it refused a handover upstream, or it announced its own layer had gone. The
  // second can arrive at any moment, so reading only the first would leave an arm
  // about to hand the cube to a partner that had already said it could not move.
  assert.match(clause, /outOfService\(NextRobot\)/);
  // Bounded by the reroute: rerouteAround always removes the hop it is given,
  // and the replacement is chosen from openAnswer, which excludes out-of-service
  // arms -- so the re-entry below cannot come round on another one.
  assert.match(plans, /outOfService\(Robot\) :- armUnfit\(Robot\)|outOfService/);
  assert.match(clause, /!rerouteAround\(NextAgent, NextRobot\)/);
  assert.match(clause, /!continueHandoverChain\(Session, Ignored\)/);
  // Re-entrant, and bounded by it: rerouteAround always removes the hop it is
  // given, so this cannot come round more times than there are hops to strike.
  assert.ok(
    plans.indexOf("+!continueHandoverChain(Session, Ignored)\n    : nextHop(")
      < plans.indexOf("+!continueHandoverChain(Session, _)\n    : nextHop("),
    "the strike is applied before the route is followed"
  );
});

test("an abandoned handover leaves no session belief standing", () => {
  // Adopting the next handover needs `not handoverSession(_, _, _)`, so ANY
  // session left behind makes the reroute inapplicable -- and it then fails
  // through to "failed before session creation" having tried nothing.
  const teardown = plans.slice(plans.indexOf("+!abandonHandover(ReceiverAgent, ReceiverRobot): true <-"));
  const body = teardown.slice(0, teardown.indexOf(".\n\n"));
  assert.match(body, /\.abolish\(handoverSession\(_, _, _\)\)/);
  for (const belief of ["handoverStage\\(_, _\\)", "handoverFailed\\(_, _\\)", "handoverRetriesLeft\\(_\\)",
    "selectedHandoverCandidate\\(_, ReceiverRobot, _\\)"]) {
    assert.match(body, new RegExp(`\\.abolish\\(${belief}\\)`));
  }
  assert.match(body, /\.send\(ReceiverAgent, tell, handoverFailed\(Session, giver_rerouting\)\)/,
    "the arm being left behind is told why, even if it is not listening");
  assert.match(plans, /-!abandonHandover\(_, _\): true <-/);
});

test("a retried rendezvous starts without the last attempt's verdict on the partner", () => {
  // Otherwise the retry that was meant to try a different POINT is read as a
  // second confirmation that the ARM is gone, and the route is struck over a
  // rendezvous the pair had simply not got right yet.
  const attempt = plans.slice(plans.indexOf("+!handoverCubeTo(ReceiverAgent, ReceiverRobot)\n    : agentIdentity(Me, MyRobot)"));
  assert.match(attempt.slice(0, attempt.indexOf("!refreshCubeOwner")), /\.abolish\(partnerUnfit\(ReceiverAgent, _\)\)/);
  // And a handover that completed clears both marks, so the same partner can be
  // handed to again later in the cell's life.
  const completed = attempt.slice(attempt.indexOf("+handoverFinished(Session)"));
  assert.match(completed, /\.abolish\(partnerUnfit\(ReceiverAgent, _\)\)/);
  assert.match(completed, /\.abolish\(rerouteAttempted\(ReceiverAgent\)\)/);
});
