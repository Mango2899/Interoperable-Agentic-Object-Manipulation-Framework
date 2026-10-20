// Reusable pair-owned cube handover protocol. Each neighboring pair has one
// deterministic interaction-space owner; no global spatial coordinator exists.

+!buildInteractionSpace(ReceiverRobot): true <-
    .abolish(interactionRobot(_, ReceiverRobot));
    .abolish(interactionSpace(_, ReceiverRobot, _, _, _, _, _));
    .abolish(interactionSpaceReady(_, ReceiverRobot));
    .abolish(interactionSpaceFailed(ReceiverRobot, _));
    wotAchieve("ownLayer1", "computeInteractionSpace", [ReceiverRobot]);
    // Both answers end the wait. "These two share nothing" is a verdict the
    // estimator reaches in about a second -- the reach spheres are a necessary
    // condition, so a pair worth asking about can still voxelise to an empty
    // intersection -- and waiting the full minute for a reply already sitting
    // in the belief base held every remaining pair behind it. Live cell:
    // Robot1/Robot3 answered voxel_intersection_empty in 1.3s and Robot1 did
    // not start on Robot2 for another 59.
    .wait(interactionSpaceReady(Space, ReceiverRobot) | interactionSpaceFailed(ReceiverRobot, _), 60000);
    if (interactionSpaceFailed(ReceiverRobot, _)) { .fail };
    // Told in the neighbour's own frame, not in mine. "The space with
    // ReceiverRobot is ready" is a true thing for ME to believe and a useless
    // one for the receiver, who is ReceiverRobot -- it needs to know the space
    // with ME is ready before it can ever hand back this way. Harmless while
    // the pair owner is always the giver; wrong the moment a route runs the
    // other way.
    if (interactionRobot(ReceiverAgent, ReceiverRobot) & agentIdentity(_, MyRobot)) {
        .send(ReceiverAgent, tell, interactionSpaceReady(Space, MyRobot))
    }.

// A shared space describes one arrangement of the cell. Once a base has been
// moved -- dragged in the scene, or served at a different offset -- the space
// this pair agreed on describes a cell that no longer exists. Re-asking for it
// costs a version check when nothing moved and a recomputation exactly when
// something did, so the owner asks before every handover rather than trusting
// what it computed at startup.
+!refreshInteractionSpace(ReceiverRobot): true <-
    .abolish(interactionSpace(_, ReceiverRobot, _, _, _, _, _));
    .abolish(interactionSpaceReady(_, ReceiverRobot));
    .abolish(interactionSpaceFailed(ReceiverRobot, _));
    wotAchieve("ownLayer1", "refreshInteractionSpace", [ReceiverRobot]);
    .wait(interactionSpaceReady(Space, ReceiverRobot) | interactionSpaceFailed(ReceiverRobot, _), 60000);
    if (interactionSpaceFailed(ReceiverRobot, _)) { .fail };
    if (interactionRobot(ReceiverAgent, ReceiverRobot) & agentIdentity(_, MyRobot)) {
        .send(ReceiverAgent, tell, interactionSpaceReady(Space, MyRobot))
    }.

+!startHandover(ReceiverAgent, ReceiverRobot): true <-
    !handoverCubeTo(ReceiverAgent, ReceiverRobot).

+!refreshCubeOwner(QueryId): true <-
    .abolish(cubeOwnership(QueryId, _, _));
    wotAchieve("ownLayer1", "queryCubeOwnership", [QueryId]);
    .wait(cubeOwnership(QueryId, Owner, _), 5000);
    .abolish(cubeOwner(_));
    +cubeOwner(Owner).

+!verifyGripperHolding(QueryId): true <-
    .abolish(gripperState(QueryId, _, _, _));
    wotAchieve("ownLayer1", "queryGripperState", [QueryId]);
    .wait(gripperState(QueryId, _, true, _), 5000).

+!setPickupApproach(Enabled): true <-
    .abolish(pickupApproachSet(_));
    .abolish(pickupApproachFailed(_, _));
    wotAchieve("ownLayer1", "setPickupApproach", [Enabled]);
    .wait(pickupApproachSet(Enabled) | pickupApproachFailed(Enabled, _), 5000);
    if (pickupApproachFailed(Enabled, Reason)) {
        .print("Pickup approach mode failed: ", Reason);
        .fail
    }.

// The same approach, aimed somewhere other than this arm's own pick support.
// Placing onto a support is picking from one run backwards: a hover pose, a
// vertical descent, and one block that must not count as an obstacle at the
// moment of contact.
+!setPickupApproachTo(Enabled, Site): true <-
    .abolish(pickupApproachSet(_));
    .abolish(pickupApproachFailed(_, _));
    wotAchieve("ownLayer1", "setPickupApproach", [Enabled, Site]);
    .wait(pickupApproachSet(Enabled) | pickupApproachFailed(Enabled, _), 5000);
    if (pickupApproachFailed(Enabled, Reason)) {
        .print("Approach mode for ", Site, " failed: ", Reason);
        .fail
    }.

// Put the cube down on a support this arm detected for itself. The position is
// read from the site belief, so moving the support in the scene moves the
// placement with it -- nothing here was written down in advance.
//
// The arm does not carry the cube down onto the face. It stops with the tool
// vertical a release height above where the cube will stand, opens, and lets
// the cube fall the rest of the way: the lower Thing plans the hover and no
// descent at all. So the point named here is where the cube ENDS UP, not a
// pose the arm ever adopts -- the arm stays a release height above it.
+!placeCubeAt(Goal)
    : site(Goal, X, Y, Z)
      & pickupHoverHeight(H)
      & agentIdentity(_, MyRobot)
<-
    .abolish(placedAt(_, _, _));
    .abolish(placementRetreat(_, _, _));
    .abolish(placementFailing(Goal));
    !setPickupApproachTo(true, Goal);
    !moveTo(X, Y, Z);
    !openGripper;
    !setPickupApproachTo(false, Goal);
    !refreshCubeOwner(after_place);
    ?cubeOwner(none);
    !retreatFromPlacement(X, Y, Z, H);
    .abolish(placementResetTried(Goal));
    .print("Cube placed on ", Goal, " by ", MyRobot).

// Lift clear of the cube just dropped. The support is a hard obstacle again now
// the approach is disarmed, and so is the cube standing on it.
//
// Up from where the cube was actually LET GO of, which is not always above the
// point that was asked for: an arm that cannot reach the centre of a face
// top-down releases over another part of it and says so through placedAt.
// Lifting from the requested centre instead is a diagonal move with the fingers
// open at the height the cube was just dropped from -- across the cube, not
// away from it.
// The lower layer rotates world vertical into this robot's local frame.
+!retreatFromPlacement(_, _, _, _): placementRetreat(X, Y, Z) <-
    !moveTo(X, Y, Z).

+!retreatFromPlacement(_, _, _, H): placedAt(PlacedX, PlacedY, PlacedZ) & not placementRetreat(_, _, _) <-
    RetreatZ = PlacedZ + H;
    !moveTo(PlacedX, PlacedY, RetreatZ).

+!retreatFromPlacement(X, Y, Z, H): not placedAt(_, _, _) & not placementRetreat(_, _, _) <-
    RetreatZ = Z + H;
    !moveTo(X, Y, RetreatZ).

// Leaving the approach armed after a failure silently changes what the next
// move means, so it is disarmed before the failure propagates -- the same
// reason -!graspCube does it.
//
// The guard, the marker and the terminal clause below are all load-bearing, and
// so is the disarm going through a goal that cannot fail.
//
// Jason looks for a failure plan by walking UP the intention, and it does not
// skip an intended means that is ALREADY a failure plan. So a SUBGOAL failing
// inside a -! handler regenerates the very event that ran the handler, and the
// handler runs again. With !setPickupApproachTo called straight from here, its
// five-second wait timing out -- which is what happens while the lower layer is
// still planning a path it will not find -- turned one unreachable placement
// into a handler that re-entered itself for as long as the run lasted. Live
// cell: robot5 could not place the cube and said so tens of thousands of times,
// and the arm that was waiting for it never heard anything at all.
//
// The same shape as -!approachRendezvous, which has always been guarded this
// way: a clause that marks itself before doing anything, and a terminal clause
// behind it that only reports and propagates.
// The fallback the direct placement above relies on, and a retry of the POSTURE
// rather than of the point.
//
// The arm went for the support from wherever the handover left it standing. That
// is the cheap route and usually works; when the lower layer cannot plan it --
// the straight joint sweep obstructed and the OMPL search out of budget -- the
// answer is not a different point but a different starting pose. Coming upright
// puts the arm where every other move of its is planned from, and the same
// request is made once more. If it fails from there too, it is the point.
//
// The same bargain -!approachRendezvous strikes for a rendezvous it cannot
// reach: reset the posture, ask once, then believe the geometry.
//
// The marker is what makes the re-adoption below safe, and it must NOT be
// cleared at the top of +!placeCubeAt: the retry goes back through that plan, so
// clearing it there would licence a second retry, and a third. It is cleared
// when the episode ends instead -- on the placement succeeding, and on this
// handler giving up.
-!placeCubeAt(Goal)
    : not placementResetTried(Goal)
<-
    +placementResetTried(Goal);
    .print("Could not place the cube on ", Goal,
           " from here; coming upright with it and asking the same point once more");
    !disarmPlacementApproach(Goal);
    !returnToUprightWithCube;
    !placeCubeAt(Goal).

-!placeCubeAt(Goal): true <-
    .abolish(placementResetTried(Goal));
    .print("Could not place the cube on ", Goal, ", from upright either; it is the point, not the posture");
    !disarmPlacementApproach(Goal);
    .fail.

// Best-effort tidy-up on a failure path. The placement has already failed and is
// already being reported; a tidy-up that fails here must not become the failure
// being handled.
+!disarmPlacementApproach(Goal): true <-
    !setPickupApproachTo(false, Goal).

-!disarmPlacementApproach(Goal): true <-
    .print("Could not disarm the placement approach for ", Goal,
           "; the next move this arm plans will treat the support as a hard obstacle").

+!ensureCubeHeld(Robot): true <-
    .abolish(gripperState(ensure_cube, _, _, _));
    wotAchieve("ownLayer1", "queryGripperState", [ensure_cube]);
    .wait(gripperState(ensure_cube, _, Holding, _), 5000);
    if (Holding == true) {
        .print("Cube already held by ", Robot)
    } else {
        ?cubeOwner(none);
        .print("Picking cube for ", Robot);
        // The same grasp !pickCube uses. Deliberately not !pickCube itself: the
        // handover rendezvous is solved from wherever the giver is standing, so
        // moving to the CARRY pose first would only add a move to undo.
        //
        // The short vertical lift off the support is a different matter and does
        // happen here, because it belongs to the grasp: !graspCube ends with it,
        // so the giver leaves the support the same way a standalone pick does
        // and the rendezvous is approached from clear air.
        !graspCube;
        !refreshCubeOwner(after_pick);
        ?cubeOwner(Robot)
    }.

+!requestHandoverPlan(ReceiverRobot) : handoverHeightRange(MinZ, MaxZ) <-
    .abolish(handoverPlan(_, _, _, _, _, _, _, _, _, _));
    .abolish(handoverPlanFailed(_));
    .abolish(handoverCandidate(_, ReceiverRobot, _, _, _, _, _, _, _, _, _, _, _, _, _, _));
    .abolish(handoverCore(ReceiverRobot, _, _, _, _, _, _, _, _, _, _, _, _));
    .abolish(handoverCandidatesReady(ReceiverRobot, _));
    .abolish(handoverCandidatesFailed(ReceiverRobot, _));
    // Only ever one live selection, so a failure after a completed handover
    // cannot be mistaken for a refused rendezvous and retried against a point
    // that already worked.
    .abolish(selectedHandoverCandidate(_, ReceiverRobot, _));
    wotAchieve("ownLayer1", "rankHandoverCandidates", [ReceiverRobot, MinZ, MaxZ]);
    .wait(handoverCandidatesReady(ReceiverRobot, _) | handoverCandidatesFailed(ReceiverRobot, _), 30000);
    if (handoverCandidatesFailed(ReceiverRobot, Reason)) {
        .print("Central interaction-core construction failed: ", Reason);
        .fail
    };
    // Posture is how far each arm's straight approach leg stays clear of its own
    // base-joint axis. Without it the ranking happily picks a geometrically
    // central point whose approach line crosses a shoulder, which no amount of
    // per-point reachability can rule out and which the runtime can only answer
    // by approaching from somewhere else entirely.
    // A rendezvous the runtime has already refused is struck from the ranking
    // rather than re-picked. The scoring cannot see why it was refused -- the
    // workspace cloud says a point is reachable in SOME configuration, while
    // the handover needs it reachable with the tool pinned along the approach
    // axis and a collision-free straight leg into it -- so the only evidence
    // that a point is unusable is the cell having refused it.
    .findall(rank(Score, Candidate),
        handoverCandidate(Candidate, ReceiverRobot,
            _, _, _, _, _, _, _, _, _, Centrality, Density, GiverSupport, ReceiverSupport, Posture)
        & not handoverCandidateRejected(Candidate, ReceiverRobot)
        & Score = 0.32 * Centrality + 0.2 * Density
            + 0.14 * GiverSupport + 0.14 * ReceiverSupport + 0.2 * Posture,
        Rankings);
    .length(Rankings, Remaining);
    if (Remaining == 0) {
        .print("Every ranked rendezvous for ", ReceiverRobot,
               " has been refused by the cell; the shared space offers nothing this pair can use");
        .fail
    };
    .max(Rankings, rank(BestScore, BestCandidate));
    +selectedHandoverCandidate(BestCandidate, ReceiverRobot, BestScore);
    // The number chosen from is as much a part of the decision as the score. A
    // single-candidate pair is not a choice, and the two retries below have
    // nothing to strike off and try instead.
    .print("BDI selected central handover candidate ", BestCandidate, " score=", BestScore,
           " (of ", Remaining, " still usable)");
    wotAchieve("ownLayer1", "commitHandoverCandidate", [ReceiverRobot, BestCandidate]);
    .wait(handoverPlan(_, _, _, _, _, _, _, _, _, _) | handoverPlanFailed(_), 10000);
    if (handoverPlanFailed(PlanReason)) {
        .print("Selected handover candidate could not be committed: ", PlanReason);
        .fail
    }.

// Reverse the recorded arrival once. A failed endpoint check can arrive AFTER
// the arm has backed out. A Cartesian retry relative to the old rendezvous
// would then drive it forward again; with the pose lock armed it can even be
// dispatched as a fresh approach. Propagate failure instead: the giver's
// backOutOfRendezvous handler proceeds to checked joint home, while the
// receiver must not announce clearance for an unconfirmed retreat.
+!leaveRendezvous(Session, X, Y, Z): true <-
    .abolish(handoverRetreated(Session));
    .abolish(handoverRetreatFailed(Session, _));
    wotAchieve("ownLayer1", "retreatFromHandover", [Session]);
    .wait(handoverRetreated(Session) | handoverRetreatFailed(Session, _), 60000);
    if (handoverRetreatFailed(Session, Reason)) {
        .print("Rendezvous retreat was not confirmed: ", Reason);
        .fail
    }.

// A timeout also leaves the arm's position uncertain. Do not enqueue a
// Cartesian retry while its original retreat may still be running.
-!leaveRendezvous(_, _, _, _): true <-
    .print("Rendezvous departure failed; returning control to the handover recovery");
    .fail.

// Going to a rendezvous, with one second chance from a different starting
// posture.
//
// A rendezvous is ranked on the last stretch into it -- the standoff leg along
// the pinned tool axis, which is what !posturePenalty scores and rejects on.
// Nothing ranks the LONG leg from wherever the arm happens to be parked, and it
// is the one that fails: a straight line from a pose folded in near the base
// passes back over the base on its way out, and the elbow meets it no matter
// how reachable both ends are. Live cell: Robot4 refused its rendezvous at the
// first sample of fourteen, 3.6 cm into a 51 cm journey, on baseVisible against
// its own link 3.
//
// That is a fact about where the arm was standing, not about the point. From
// the all-zero pose the same rendezvous is a different journey, so the arm
// resets and asks the point again before it hands a refusal back and costs the
// giver one of its two candidates.
+!approachRendezvous(_, X, Y, Z): true <-
    !moveTo(X, Y, Z).

-!approachRendezvous(Session, X, Y, Z)
    : not rendezvousResetTried(Session)
<-
    +rendezvousResetTried(Session);
    .print("Could not reach the rendezvous from where I stand; resetting posture and asking the same point once more");
    !resetPostureForApproach;
    !moveTo(X, Y, Z).

-!approachRendezvous(_, _, _, _): true <-
    .print("The rendezvous is out of reach from the all-zero pose too; it is the point, not the posture");
    .fail.

// The cube comes with the arm if the arm is holding it. Resetting with an empty
// gripper and resetting mid-carry are different manoeuvres and the cell already
// has both; picking between them is all this does.
+!resetPostureForApproach
    : agentIdentity(_, MyRobot) & cubeOwner(MyRobot)
<-
    !returnToUprightWithCube.

+!resetPostureForApproach: true <-
    !returnToDefaultJointPose.

+!setHandoverApproach(Session, Enabled): true <-
    .abolish(handoverApproachSet(Session, _));
    .abolish(handoverApproachFailed(Session, _, _));
    wotAchieve("ownLayer1", "setHandoverApproach", [Session, Enabled]);
    .wait(handoverApproachSet(Session, Enabled), 5000).

+!transferCubeTo(Session, ReceiverRobot): true <-
    .abolish(cubeTransferSucceeded(Session, _, _));
    .abolish(cubeTransferFailed(Session, _));
    wotAchieve("ownLayer1", "transferCube", [Session, ReceiverRobot]);
    .wait(cubeTransferSucceeded(Session, _, ReceiverRobot), 5000).

// Use the configured joint home explicitly. Calling !goHome would also match
// the cartesian-home plan on generated fixed robots and would not guarantee the
// requested upright all-zero joint pose.
+!returnToDefaultJointPose: homeJoints(HomeJoints) <-
    .abolish(movedTo(_));
    .abolish(motionFailed(_));
    -myStatus(holdingObject);
    -myStatus(idle);
    +myStatus(moving);
    wotAchieve("ownLayer1", "goHomeJoints", HomeJoints);
    .wait(movedTo(home) | motionFailed(_), 60000);
    if (motionFailed(Reason)) {
        .print("Default joint-pose motion failed: ", Reason);
        -myStatus(moving);
        +myStatus(broken);
        .fail
    };
    -myStatus(moving);
    +myStatus(idle).

// The successful retreat releases the receiver's agent. The lower core still
// arbitrates overlapping motion, releasing each room once the actual arm AND
// the remaining continuous home path have cleared it.
+!clearRendezvous(Session): true <-
    .abolish(giverRetreatFailed(Session));
    !backOutOfRendezvous(Session);
    !returnGiverHome(Session).

// The home motion is DISPATCHED before the receiver is told it may move, and
// that ordering is the whole point of this plan.
//
// wotAchieve hands the request to the lower layer and returns; the .wait below
// is what blocks. Releasing the receiver first therefore gave it a head start
// of the giver's entire preparation -- reading the shared regions, sweeping the
// path's geometry, and only then asking the arbiter for it. The receiver got to
// the arbiter first often enough to matter, and because a reservation covers a
// whole path, the giver then waited out the receiver's ENTIRE next leg: Robot4
// stood at the rendezvous until Robot5 had carried the cube to the support and
// let go of it. Both orders are safe -- the arbiter decides, not this message --
// but only one of them is the order the cell wants, because it is the giver's
// exit that frees the rendezvous for everyone behind it.
+!returnGiverHome(Session): homeJoints(HomeJoints) <-
    .abolish(movedTo(_));
    .abolish(motionFailed(_));
    -myStatus(idle);
    +myStatus(moving);
    wotAchieve("ownLayer1", "goHomeAfterHandover", [Session, HomeJoints]);
    !releaseReceiver(Session);
    .wait(movedTo(home) | motionFailed(_), 90000);
    if (motionFailed(Reason)) {
        .print("Giver home motion failed: ", Reason);
        -myStatus(moving);
        +myStatus(broken);
        .fail
    };
    -myStatus(moving);
    +myStatus(idle).

// Having no home pose to go to is not a reason to strand the receiver.
+!returnGiverHome(Session): true <-
    !releaseReceiver(Session);
    .print("No home joints configured; the giver stays where it is after: ", Session).

// Sent once, and never for a giver still standing in the rendezvous.
+!releaseReceiver(Session)
    : not giverRetreated(Session)
      & not giverRetreatFailed(Session)
      & handoverSession(Session, _, ReceiverAgent)
<-
    +giverRetreated(Session);
    .send(ReceiverAgent, tell, giverRetreated(Session));
    .print("Giver retreated and its home motion is already with the arm; ",
           "receiver may request its next motion, subject to room clearance: ", Session).

+!releaseReceiver(_): true <- true.

+!backOutOfRendezvous(Session)
    : handoverPlan(Session, _, _, _, GiverX, GiverY, GiverZ, _, _, _)
<-
    !leaveRendezvous(Session, GiverX, GiverY, GiverZ).

+!backOutOfRendezvous(Session): true <- +giverRetreatFailed(Session).

-!backOutOfRendezvous(Session): true <-
    +giverRetreatFailed(Session);
    .print("Could not back out of the rendezvous; going for the joint pose from where I stand").

// The manifest can restore the receive -> home -> give route per robot.
+!prepareToForwardCube: true <-
    .abolish(handoverReturnHome(_));
    wotAchieve("ownLayer1", "getHandoverPolicy", []);
    .wait(handoverReturnHome(_), 5000);
    if (handoverReturnHome(true) | siteReachable(_)) {
        .print("Returning home with the cube before the next move");
        !returnToUprightWithCube
    } else {
        .print("Continuing directly to the next handover with the cube")
    }.

// Reset with the grip and authoritative cube ownership preserved.
+!returnToUprightWithCube
    : homeJoints(HomeJoints)
      & agentIdentity(_, MyRobot)
      & cubeOwner(MyRobot)
<-
    .abolish(movedTo(_));
    .abolish(motionFailed(_));
    -myStatus(holdingObject);
    -myStatus(idle);
    +myStatus(moving);
    wotAchieve("ownLayer1", "goHomeJoints", HomeJoints);
    .wait(movedTo(home) | motionFailed(_), 90000);
    if (motionFailed(Reason)) {
        .print("Upright motion with cube failed: ", Reason);
        -myStatus(moving);
        +myStatus(broken);
        .fail
    };
    -myStatus(moving);
    +myStatus(holdingObject);
    !refreshCubeOwner(after_intermediate_upright);
    ?cubeOwner(MyRobot);
    !verifyGripperHolding(after_intermediate_upright);
    .print("Intermediate giver upright with cube: ", MyRobot).

// Who may hand to whom is a property of the PAIR, not of who computed its
// shared space. Ownership decides which arm builds and caches the space, once,
// deterministically -- it was never meant to decide which arm is allowed to be
// the giver, and a route that needs the cube passed the other way needs exactly
// that. The space itself is refreshed below before any rendezvous is chosen, so
// requiring it to already exist here only kept non-owners out.
+!handoverCubeTo(ReceiverAgent, ReceiverRobot)
    : agentIdentity(Me, MyRobot)
      & canGiveTo(ReceiverRobot)
      & not handoverSession(_, _, _)
<-
    if (not handoverRetriesLeft(_)) { +handoverRetriesLeft(2) };
    .abolish(rendezvousResetTried(_));
    // A fresh attempt at this partner starts without last attempt's verdict on
    // it. Otherwise the retry that was meant to try a different POINT is read
    // as a second confirmation that the ARM is gone, and the route is struck
    // over a rendezvous the pair had simply not got right yet.
    .abolish(partnerUnfit(ReceiverAgent, _));
    !refreshCubeOwner(before_handover);
    ?cubeOwner(CurrentOwner);
    .print("Authoritative owner before handover: ", CurrentOwner);
    !ensureCubeHeld(MyRobot);
    !refreshInteractionSpace(ReceiverRobot);
    .print("Requesting handover plan for ", ReceiverRobot);
    !requestHandoverPlan(ReceiverRobot);
    ?handoverPlan(Session, CubeX, CubeY, CubeZ,
        GiverX, GiverY, GiverZ, ReceiverX, ReceiverY, ReceiverZ);
    +handoverSession(Session, Me, ReceiverAgent);
    +handoverStage(Session, moving_giver);

    !approachRendezvous(Session, GiverX, GiverY, GiverZ);
    +giverAtHandover(Session);
    +giverReady(Session);
    .send(ReceiverAgent, achieve,
        receiveCube(Session, Me, MyRobot,
            CubeX, CubeY, CubeZ, ReceiverX, ReceiverY, ReceiverZ));
    .send(ReceiverAgent, tell, giverReady(Session));

    -handoverStage(Session, moving_giver);
    +handoverStage(Session, waiting_receiver);
    // Receiver readiness includes collision preflight, IK and the complete
    // physical approach. Keep this above the lower motion timeout instead of
    // racing a valid but non-trivial path with the old 15-second deadline.
    !awaitReceiverReady(Session, ReceiverAgent, ReceiverRobot);

    !transferCubeTo(Session, ReceiverRobot);
    !refreshCubeOwner(Session);
    ?cubeOwner(ReceiverRobot);
    +cubeTransferred(Session);
    .send(ReceiverAgent, tell, cubeTransferred(Session));

    -handoverStage(Session, waiting_receiver);
    +handoverStage(Session, waiting_receiver_retreat);
    .print("Waiting for receiver to clear the interaction point: ", Session);
    .wait(receiverRetreated(Session)[source(ReceiverAgent)], 90000);

    -handoverStage(Session, waiting_receiver_retreat);
    +handoverStage(Session, giver_returning_home);
    .print("Receiver clear; backing the giver out of the rendezvous: ", Session);
    !clearRendezvous(Session);
    !setHandoverApproach(Session, false);
    +giverHome(Session);
    .send(ReceiverAgent, tell, giverHome(Session));
    .print("Giver reached all-zero joint pose: ", Session);

    -handoverStage(Session, giver_returning_home);
    +handoverStage(Session, waiting_completion);
    // This wait also covers any downstream BDI handovers adopted by an
    // intermediate receiver, including the terminal support return.
    .wait(handoverComplete(Session)[source(ReceiverAgent)], 600000);
    -handoverStage(Session, waiting_completion);
    +handoverStage(Session, complete);
    +handoverFinished(Session);
    .abolish(handoverSession(Session, _, _));
    .abolish(handoverRetriesLeft(_));
    .abolish(partnerUnfit(ReceiverAgent, _));
    .abolish(rerouteAttempted(ReceiverAgent));
    .abolish(handoverCandidateRejected(_, ReceiverRobot));
    // Both copies, or the next handover to this partner is ranked against
    // refusals that belong to one that has already succeeded.
    wotAchieve("ownLayer1", "clearHandoverRejections", [ReceiverRobot]);
    .print("Handover completed: ", Session).

// Whether the arm at the other end is coming at all.
//
// Receiver readiness includes collision preflight, IK and the complete physical
// approach, so the deadline sits above the lower motion timeout rather than
// racing a valid but non-trivial path.
//
// The wait sits in a goal of its own so that its two ways of ending can be told
// apart. `.wait` FAILS when its deadline passes -- it does not resume -- so a
// bare wait in the caller's body took the handover down at exactly the moment
// it was written for, and the caller could not see whether the partner had
// refused, had gone quiet, or had never existed. All three are facts about the
// PARTNER and not about the rendezvous, which is the distinction the recovery
// turns on, so they are written down here as one belief.
+!awaitReceiverReady(Session, ReceiverAgent, _): true <-
    .wait(receiverReady(Session)[source(ReceiverAgent)] |
          handoverFailed(Session, _)[source(ReceiverAgent)], 120000);
    if (handoverFailed(Session, ReceiverReason)[source(ReceiverAgent)]) {
        .print("Receiver failed before readiness: ", ReceiverReason);
        +partnerUnfit(ReceiverAgent, ReceiverReason);
        .fail
    }.

// Nothing came back at all. An arm whose agent has stopped reasoning, or that
// was never started, is indistinguishable from here and does not need to be
// told apart: neither of them will take the cube, and no other rendezvous in
// the space the two of them share will change that.
-!awaitReceiverReady(_, ReceiverAgent, ReceiverRobot): true <-
    if (not partnerUnfit(ReceiverAgent, _)) {
        .print(ReceiverRobot, " neither reported ready nor reported a failure inside the deadline; ",
               "treating the arm itself as unfit rather than the rendezvous");
        +partnerUnfit(ReceiverAgent, silent)
    };
    .fail.

// Clearance wakes the receiver while the giver is still returning home.
// A failed exit or missing acknowledgement is not evidence of clearance.
+!awaitGiverClear(Session, GiverAgent): true <-
    .wait(giverRetreated(Session)[source(GiverAgent)] |
          giverHome(Session)[source(GiverAgent)] |
          giverStuck(Session)[source(GiverAgent)], 90000);
    if (giverStuck(Session)[source(GiverAgent)]
        & not giverRetreated(Session)[source(GiverAgent)]
        & not giverHome(Session)[source(GiverAgent)]) {
        .print("The giver could not clear after handing over; stopping this handover");
        .fail
    }.

-!awaitGiverClear(Session, _): true <-
    .print("No confirmed giver clearance for ", Session);
    .fail.

+!receiveCube(Session, GiverAgent, GiverRobot,
              CubeX, CubeY, CubeZ, ReceiverX, ReceiverY, ReceiverZ)[source(GiverAgent)]
    : handoverPartner(GiverAgent, GiverRobot)
      & agentIdentity(Me, MyRobot)
      & not handoverSession(_, _, _)
<-
    .abolish(rendezvousResetTried(_));
    +handoverSession(Session, GiverAgent, Me);
    +handoverPlan(Session, CubeX, CubeY, CubeZ,
        0, 0, 0, ReceiverX, ReceiverY, ReceiverZ);
    +handoverStage(Session, receiver_approaching);
    .wait(giverReady(Session)[source(GiverAgent)] |
          handoverFailed(Session, _)[source(GiverAgent)], 30000);
    if (handoverFailed(Session, GiverReason)[source(GiverAgent)]) {
        .print("Giver failed before receiver approach: ", GiverReason);
        .fail
    };

    !setHandoverApproach(Session, true);
    !approachRendezvous(Session, ReceiverX, ReceiverY, ReceiverZ);
    +receiverAtHandover(Session);
    +receiverReady(Session);
    .send(GiverAgent, tell, receiverReady(Session));

    -handoverStage(Session, receiver_approaching);
    +handoverStage(Session, waiting_transfer);
    .wait(cubeTransferred(Session)[source(GiverAgent)] |
          handoverFailed(Session, _)[source(GiverAgent)], 30000);
    if (handoverFailed(Session, TransferReason)[source(GiverAgent)]) {
        .print("Giver failed during cube transfer: ", TransferReason);
        .fail
    };
    !refreshCubeOwner(Session);
    ?cubeOwner(MyRobot);
    !verifyGripperHolding(Session);

    -handoverStage(Session, waiting_transfer);
    +handoverStage(Session, clearing_interaction_point);
    .print("Receiver clearing interaction point with cube: ", Session);
    !leaveRendezvous(Session, ReceiverX, ReceiverY, ReceiverZ);
    +receiverRetreated(Session);
    .send(GiverAgent, tell, receiverRetreated(Session));

    -handoverStage(Session, clearing_interaction_point);
    +handoverStage(Session, waiting_giver_clear);
    !awaitGiverClear(Session, GiverAgent);
    !setHandoverApproach(Session, false);

    -handoverStage(Session, waiting_giver_clear);
    !continueHandoverChain(Session, GiverAgent);
    +handoverComplete(Session);
    +handoverFinished(Session);
    .send(GiverAgent, tell, handoverComplete(Session));
    .abolish(handoverSession(Session, _, _)).

// The route the cell worked out for itself, one hop at a time. Each arm reads
// only its OWN next hop -- nobody was ever sent the whole path, and nobody needs
// it. The old session is removed before adopting the next intention, so an
// intermediate arm executes its receive and its give as two atomic handovers.
// An arm struck upstream is struck here too, before this arm acts on a route it
// chose before anybody knew. Without this the cube is carried past the failure
// and handed straight back towards the arm that refused it, because every arm's
// nextHop was settled minutes earlier from the same stale ranking.
//
// Re-entrant on purpose, and bounded: !rerouteAround always removes the hop it
// is given, so this cannot come round more times than this arm has hops to
// strike. When the last one goes there is no nextHop left and the clauses below
// take over.
+!continueHandoverChain(Session, Ignored)
    : nextHop(_, NextAgent, NextRobot, _)
      & outOfService(NextRobot)
<-
    .print("The route names ", NextRobot, ", which cannot take the cube: either it refused ",
           "further up, or it has told the cell its own layer has gone");
    !rerouteAround(NextAgent, NextRobot);
    !continueHandoverChain(Session, Ignored).

+!continueHandoverChain(Session, _)
    : nextHop(_, NextAgent, NextRobot, _)
      & agentIdentity(_, MyRobot)
      & canGiveTo(NextRobot)
<-
    !prepareToForwardCube;
    +handoverStage(Session, forwarding_cube);
    .abolish(handoverSession(Session, _, _));
    .print("Route says the cube goes to ", NextRobot, " next");
    !handoverCubeTo(NextAgent, NextRobot);
    -handoverStage(Session, forwarding_cube);
    +handoverStage(Session, complete).

// I hold the goal, so the cube stops here. Where it goes is read from the site
// this arm detected, not from a constant somebody wrote down: move the support
// and the placement follows it.
+!continueHandoverChain(Session, _)
    : hasSink(Goal)
<-
    +handoverStage(Session, returning_cube);
    // Straight from the rendezvous to the support, without coming upright first.
    //
    // Deliberately unlike every clause that FORWARDS the cube, and the asymmetry
    // is the point: an arm that forwards has to reach a rendezvous it has not
    // solved yet, while an arm that places is going to a support it detected and
    // can serve. Often it can go there from where it stands, and a reset to the
    // all-zero pose on the way would be a whole traverse of the workspace paid
    // for on every delivery to buy nothing.
    //
    // When it cannot, -!placeCubeAt comes upright and asks the same point once
    // more. So the reset is still there, as the recovery rather than the route:
    // paid for only by the deliveries that need it.
    .print("The goal ", Goal, " is mine; placing the cube on it");
    !placeCubeAt(Goal);
    -handoverStage(Session, returning_cube);
    +handoverStage(Session, complete).

// Configured chain, kept as the comparison baseline. Only reachable when the
// cell never worked a route out for itself.
+!continueHandoverChain(Session, _)
    : handoverSuccessor(NextAgent, NextRobot)
      & interactionSpaceOwner(MyRobot, NextRobot)
      & agentIdentity(_, MyRobot)
      & interactionSpaceReady(_, NextRobot)
<-
    !prepareToForwardCube;
    +handoverStage(Session, forwarding_cube);
    .abolish(handoverSession(Session, _, _));
    .print("Adopting downstream handover intention to ", NextRobot);
    !handoverCubeTo(NextAgent, NextRobot);
    -handoverStage(Session, forwarding_cube);
    +handoverStage(Session, complete).

// A robot without a successor is the terminal receiver and restores the cube
// to the shared support, preserving the repeatable behavior of the pair demo.
+!continueHandoverChain(Session, _)
    : not handoverSuccessor(_, _)
<-
    +handoverStage(Session, returning_cube);
    .print("Terminal receiver returning cube to support: ", Session);
    !returnCubeToSupport(Session);
    -handoverStage(Session, returning_cube);
    +handoverStage(Session, complete).

+!returnCubeToSupport(Session)
    : cubeOwner(Robot)
      & agentIdentity(_, Robot)
      & supportApproachPosition(AX, AY, AZ)
      & supportReleasePosition(RX, RY, RZ)
      & supportRetreatPosition(TX, TY, TZ)
<-
    !moveTo(AX, AY, AZ);
    !moveTo(RX, RY, RZ);
    !openGripper;
    !refreshCubeOwner(after_release);
    ?cubeOwner(none);
    !moveTo(TX, TY, TZ).

// The cell refusing a rendezvous is evidence about that POINT, not about the
// handover. The ranking had several candidates and committed the best-scoring
// one; if the arms cannot actually make it, strike it and take the next. This
// is the only way the approach constraint -- tool axis pinned, straight leg in,
// nothing in the way -- gets fed back into a choice made from a cloud that
// knows none of it.
// The cube is across. Whatever failed after that -- this arm's own exit, a
// receiver that never reported parked -- is not evidence about the rendezvous
// and there is nothing to retry: a second attempt reaches for a cube that is in
// somebody else's gripper, fails to hold it, and reports "failed before session
// creation" over a handover that physically succeeded. Live cell: Robot1 could
// not sweep home past Robot2, and a delivered cube was abandoned mid-chain.
-!handoverCubeTo(ReceiverAgent, _)
    : handoverSession(Session, _, _)
      & cubeTransferred(Session)
<-
    .print("The cube crossed to ", ReceiverAgent, "; only this arm's own exit failed");
    if (motionBlockedBy(Obstacle, Owner)) {
        .print("  still standing at the rendezvous, blocked by ", Obstacle, " (", Owner, ")")
    };
    .abolish(motionBlockedBy(_, _));
    // The arm on the other side is told BEFORE the layer is touched. Disarming
    // the approach is best-effort tidying; the message is the only reason that
    // arm ever stops waiting. With the order the other way round, an arm whose
    // own layer had just gone silenced its own notification and left its partner
    // to time out -- which is precisely the case this recovery exists for.
    .send(ReceiverAgent, tell, giverStuck(Session));
    wotAchieve("ownLayer1", "setHandoverApproach", [Session, false]);
    .abolish(handoverSession(Session, _, _));
    .abolish(handoverRetriesLeft(_)).

// The partner, not the point.
//
// A refused rendezvous is evidence about a POINT and is retried below with the
// next-best one. An arm that answered it could not take the cube, or that never
// answered at all, is evidence about the ARM -- and no amount of choosing a
// different point in the space the two of them share will fix it. Spending the
// rendezvous retries on it costs two more deadlines and ends where it started,
// while the route this arm collected holds a second way to the goal that has not
// been tried at all.
//
// The cube never left this gripper, so nothing physical has to be undone: the
// arm is standing at a rendezvous nobody came to, still holding what it carried
// there. What changes is only which neighbour it is offered to next.
//
// One reroute per partner, marked before anything else happens. This clause
// re-adopts !handoverCubeTo, so its own failure comes back through here, and
// without the mark it would strike the same arm a second time and then a third.
-!handoverCubeTo(ReceiverAgent, ReceiverRobot)
    : partnerUnfit(ReceiverAgent, Reason)
      & not rerouteAttempted(ReceiverAgent)
      & not (handoverSession(Held, _, _) & cubeTransferred(Held))
<-
    +rerouteAttempted(ReceiverAgent);
    +armUnfit(ReceiverRobot);
    .print(ReceiverRobot, " cannot take the cube (", Reason,
           "), so the rendezvous is not what failed; looking for another way to the goal");
    !abandonHandover(ReceiverAgent, ReceiverRobot);
    !rerouteAround(ReceiverAgent, ReceiverRobot);
    if (nextHop(_, NewAgent, NewRobot, NewTotal)) {
        .print("Forwarding the cube to ", NewRobot, " instead: ", NewTotal,
               " m to the goal from here");
        // Upright with the cube first, exactly as the chain does between two
        // hops. The arm is stretched out at the rendezvous it went to, and the
        // approach to the next one is a straight leg in with the tool axis
        // pinned -- planned from this pose it is a different, worse problem.
        !prepareToForwardCube;
        // The arm that is out travels with the cube. The route every arm ahead
        // chose was chosen before anybody knew, so a hop struck here has to be
        // struck by each of them too -- otherwise the next arm cheerfully hands
        // the cube back towards the arm that just refused it.
        .send(NewAgent, tell, armUnfit(ReceiverRobot));
        !handoverCubeTo(NewAgent, NewRobot)
    } else {
        +deliveryAbandoned(ReceiverRobot);
        .print("No route to the goal is left from this arm; the cube stays in the gripper ",
               "and the delivery stops here")
    }.

// Everything this arm set up for a handover that is not going to happen. The
// gripper is left alone on purpose: the cube is in it and never left it, and a
// release here would drop it at a rendezvous with nobody on the other side.
+!abandonHandover(ReceiverAgent, ReceiverRobot): true <-
    // Told first, tidied second: the arm being left behind stops waiting on the
    // message, not on the layer call.
    if (handoverSession(Session, _, _)) {
        .send(ReceiverAgent, tell, handoverFailed(Session, giver_rerouting));
        wotAchieve("ownLayer1", "setHandoverApproach", [Session, false])
    };
    // Unconditionally, and not only the session the test above bound: adopting
    // the next handover needs `not handoverSession(_, _, _)`, so ANY session
    // belief left standing makes it inapplicable and the reroute spends itself
    // having tried nothing.
    .abolish(handoverSession(_, _, _));
    .abolish(handoverStage(_, _));
    .abolish(handoverFailed(_, _));
    .abolish(handoverRetriesLeft(_));
    .abolish(selectedHandoverCandidate(_, ReceiverRobot, _));
    .abolish(rendezvousResetTried(_));
    if (motionBlockedBy(Obstacle, Owner)) {
        .print("  last seen blocked by ", Obstacle, " (", Owner, ")")
    };
    .abolish(motionBlockedBy(_, _)).

-!abandonHandover(_, _): true <-
    .print("Could not fully tear down the abandoned handover; continuing with the reroute").

-!handoverCubeTo(ReceiverAgent, ReceiverRobot)
    : selectedHandoverCandidate(Candidate, ReceiverRobot, _)
      & handoverRetriesLeft(Left)
      & Left > 0
      & not partnerUnfit(ReceiverAgent, _)
      & not (handoverSession(Held, _, _) & cubeTransferred(Held))
<-
    // Told before anything that can fail, and that includes the geometry call
    // below: an arm whose own layer has just gone cannot do any of the tidying
    // that follows, and the receiver's only way out of its wait is this message.
    if (handoverSession(Told, _, _)) {
        .send(ReceiverAgent, tell, handoverFailed(Told, giver_retrying))
    };
    +handoverCandidateRejected(Candidate, ReceiverRobot);
    // Told to the geometry capability too. The belief above is what takes the
    // candidate out of the running; this is what stops it counting towards the
    // DENSITY of the ones still in it. Without it a refused rendezvous keeps
    // vouching for its neighbours, and the next pick can win on the strength of
    // sitting beside two poses the cell has just refused -- which is how a
    // point twice as far from the centre of the shared space came to outscore
    // one that was not.
    wotAchieve("ownLayer1", "rejectHandoverCandidate", [ReceiverRobot, Candidate]);
    .abolish(selectedHandoverCandidate(_, ReceiverRobot, _));
    -handoverRetriesLeft(Left);
    Next = Left - 1;
    +handoverRetriesLeft(Next);
    if (handoverSession(Session, _, _)) {
        wotAchieve("ownLayer1", "setHandoverApproach", [Session, false]);
        .abolish(handoverSession(Session, _, _))
    };
    .abolish(handoverStage(_, _));
    .abolish(handoverFailed(_, _));
    // Unconditionally, not just the session the guard above bound. Re-adopting
    // the goal needs `not handoverSession(_, _, _)`, so ANY session belief left
    // standing -- one under a different id, or one the receiver told us about
    // while we were failing -- makes the retry inapplicable. It then fails
    // straight through to "failed before session creation" without ever
    // entering the plan body, and the attempt is spent having tried nothing.
    .abolish(handoverSession(_, _, _));
    if (motionBlockedBy(Obstacle, Owner)) {
        .print("Rendezvous ", Candidate, " refused: blocked by ", Obstacle, " (", Owner, ")")
    } else {
        .print("Rendezvous ", Candidate, " refused by the cell")
    };
    .abolish(motionBlockedBy(_, _));
    .print("Retrying with the next-best rendezvous, ", Next, " attempt(s) left");
    !handoverCubeTo(ReceiverAgent, ReceiverRobot).

-!handoverCubeTo(ReceiverAgent, _): handoverSession(Session, _, _) <-
    +handoverFailed(Session, giver_failed);
    .send(ReceiverAgent, tell, handoverFailed(Session, giver_failed));
    wotAchieve("ownLayer1", "setHandoverApproach", [Session, false]);
    .abolish(handoverSession(Session, _, _));
    .print("Giver handover failed: ", Session).

// Pickup and planning can fail before a physical session exists. Keep a
// separate contingency plan so those BDI failures are still handled and sent
// to the intended receiver instead of becoming an unhandled -! goal event.
-!handoverCubeTo(ReceiverAgent, _): not handoverSession(_, _, _) <-
    +handoverFailed(before_session, giver_failed_before_session);
    .send(ReceiverAgent, tell, handoverFailed(before_session, giver_failed_before_session));
    wotAchieve("ownLayer1", "setPickupApproach", [false]);
    .print("Giver handover failed before session creation").

// A request this arm cannot take up must still be answered. Silence is
// indistinguishable from a slow approach, so the giver waits out its whole
// deadline on a receiver that already knows it is not coming -- and the log
// shows an arm that simply never tried, with no reason given anywhere.
+!receiveCube(Session, GiverAgent, _, _, _, _, _, _, _)
    : handoverSession(Other, _, _)
<-
    .print("Already in handover ", Other, "; refusing ", Session);
    .send(GiverAgent, tell, handoverFailed(Session, receiver_busy)).

// Told first, and deliberately so. The commonest reason a receiver fails is that
// its own layer stopped answering, and tidying up through that same layer BEFORE
// telling the giver meant the tidy-up failed and took the notification with it.
// The giver then waited out its whole 120-second readiness deadline for an arm
// that had already worked out it could not come.
-!receiveCube(Session, GiverAgent, _, _, _, _, _, _, _): true <-
    +handoverFailed(Session, receiver_failed);
    .send(GiverAgent, tell, handoverFailed(Session, receiver_failed));
    wotAchieve("ownLayer1", "setHandoverApproach", [Session, false]);
    .abolish(handoverSession(Session, _, _));
    .print("Receiver handover failed: ", Session).
