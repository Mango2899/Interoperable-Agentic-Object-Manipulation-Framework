// Which arms this one could ever collide with, worked out rather than configured.
//
// The test is a necessary condition and nothing more: two workspaces can only
// intersect if the spheres bounding them do. A pair that passes might still
// share nothing -- only the interaction-space calculator decides that -- but a
// pair that fails cannot share anything, so it never has to be asked about.
//
// Both the evidence and the verdict become beliefs. An agent that believes
// "robot3 is not my neighbour" and cannot say why has been handed an answer
// instead of a reason.

+!discoverNeighbors: true <-
    .abolish(neighbor(_, _));
    .abolish(notNeighbor(_, _, _));
    .abolish(reachTest(_, _, _, _, _));
    .abolish(workspaceReach(_, _, _));
    .abolish(neighborScanComplete(_, _));
    .abolish(neighborScanFailed(_));
    wotAchieve("ownLayer1", "discoverNeighbors", []);
    .wait(neighborScanComplete(_, _) | neighborScanFailed(_), 30000);
    if (neighborScanFailed(Reason)) {
        .print("Neighbour scan failed: ", Reason);
        .fail
    };
    !reportNeighbors.

// A robot whose reach spheres cannot meet mine is not a neighbour and never
// needs an interaction space computed with it. Losing that scan silently is
// how a cell ends up assuming a pair can never meet, so it is said out loud.
-!discoverNeighbors: true <-
    .print("Neighbour discovery did not complete; treating every arm in the cell as a possible neighbour");
    // The layer that measures this arm did not answer, which is not a fact about
    // the cell's geometry but about this arm's ability to take part in it at all.
    // Written down and announced, because every other arm is about to ask this
    // one questions it cannot answer, and each of them would wait out its own
    // deadline to find that out.
    !announceUnavailable(neighbour_scan).

+!reportNeighbors
    : agentIdentity(_, MyRobot)
      & workspaceReach(_, Reach, SafeReach)
      & neighborScanComplete(Tested, Found)
<-
    .print("Neighbour scan for ", MyRobot, ": reach ", Reach, "m, tested against ", SafeReach,
           "m (widened by the sample spacing, because the farthest reachable point is rarely one that was sampled)");
    .findall(line(Other, Distance, Combined, Margin, Verdict),
        reachTest(Other, Distance, Combined, Margin, Verdict), Lines);
    !printReachTests(Lines);
    .findall(N, neighbor(_, N), Neighbours);
    .print("  => neighbours: ", Neighbours, "  (", Found, " of ", Tested, " arms in the cell)").

+!printReachTests([]): true <- true.

+!printReachTests([line(Other, Distance, Combined, Margin, Verdict) | Rest]): true <-
    if (neighbor(_, Other)) {
        .print("  ", Other, ": base distance ", Distance, "m <= combined reach ", Combined,
               "m, margin ", Margin, "m -> NEIGHBOUR (", Verdict, ")")
    } else {
        .print("  ", Other, ": base distance ", Distance, "m >  combined reach ", Combined,
               "m, short by ", Margin, "m -> not a neighbour (", Verdict, ")")
    };
    !printReachTests(Rest).

// A pair worth asking the interaction-space calculator about: one this robot
// could reach, that nothing has yet said is impossible.
worthComputing(Other) :- neighbor(_, Other) & not notNeighbor(Other, _, _).

// ---------------------------------------------------------------------------
// Interaction spaces, built between neighbours only.
// ---------------------------------------------------------------------------

// Who computes a pair, decided without asking anyone.
//
// The canonically first robot of the pair owns the computation. Both agents
// derive the same answer from the pair itself, so no negotiation, no election
// and no coordinator is needed -- and because the estimator is order
// independent, the one that computes produces exactly what the other would
// have. Every pair is therefore computed once, by a robot both of them can name.
ownsPair(Me, Other) :- .sort([Me, Other], [Me|_]).

// A neighbour whose shared space this robot is responsible for.
computesWith(Other) :- neighbor(_, Other) & agentIdentity(_, Me) & ownsPair(Me, Other).

// A neighbour that will compute our shared space for us.
computedBy(Other) :- neighbor(_, Other) & agentIdentity(_, Me) & not ownsPair(Me, Other).

+!shareInteractionSpaces: agentIdentity(_, MyRobot) <-
    .findall(N, neighbor(_, N), Neighbours);
    .print("Neighbours to share space with: ", Neighbours);
    !offerCloudToOwners(Neighbours);
    !buildOwnedSpaces(Neighbours).

// A pair I do not own still needs my cloud described to whoever does. The
// points never travel -- the path and digest do, and the owner reads the file
// it already has access to. On a cell whose robots do not share a filesystem
// the digest is what makes fetching it verifiable.
+!offerCloudToOwners([]): true <- true.

+!offerCloudToOwners([Other | Rest])
    : agentIdentity(_, MyRobot)
      & neighbor(OtherAgent, Other)
      & workspaceCloud(MyRobot, Path, Count, Spacing, Digest)
<-
    if (not ownsPair(MyRobot, Other)) {
        .print("Sending my workspace cloud identity to ", OtherAgent, " (owner of our pair)");
        .send(OtherAgent, tell, workspaceCloud(MyRobot, Path, Count, Spacing, Digest))
    };
    !offerCloudToOwners(Rest).

// Keep going even if one neighbour cannot be described: the others are
// independent of it.
-!offerCloudToOwners([_ | Rest]): true <-
    !offerCloudToOwners(Rest).

+!buildOwnedSpaces([]): true <- true.

+!buildOwnedSpaces([Other | Rest]): agentIdentity(_, MyRobot) <-
    if (ownsPair(MyRobot, Other)) {
        !buildSharedSpaceWith(Other)
    } else {
        .print("Not computing ", Other, ": that pair belongs to ", Other)
    };
    !buildOwnedSpaces(Rest).

-!buildOwnedSpaces([_ | Rest]): true <-
    !buildOwnedSpaces(Rest).

// Wait briefly for the neighbour's cloud to be described before computing.
// It is not required -- the cloud is on disk either way -- but having it means
// the pair was computed from a cloud the neighbour agrees is current, and the
// digest is what says so.
+!buildSharedSpaceWith(Other): agentIdentity(_, MyRobot) <-
    .wait(workspaceCloud(Other, _, _, _, _), 5000);
    if (workspaceCloud(Other, _, Count, _, Digest)) {
        .print("Computing ", MyRobot, "__", Other, " from their cloud ", Digest, " (", Count, " samples)")
    } else {
        .print("Computing ", MyRobot, "__", Other, " without their cloud description; using the exported workspace")
    };
    !buildInteractionSpace(Other);
    !renderInteractionSpace(Other).

// A pair the reach test passed and the geometry refused.
//
// The sphere test is a necessary condition and nothing else, so this outcome
// was always possible: two clouds whose bounding spheres overlap by half a
// metre can still come no closer than a few centimetres once the arms' actual
// shapes and mountings are taken into account, and at that separation they
// occupy no voxel in common. Reported as the answer it is rather than as a
// breakdown -- the cell has learnt something true about itself, and carrying on
// without this pair is the correct thing to do next.
sharesNoSpace(voxel_intersection_empty).
sharesNoSpace(no_geometric_overlap).
sharesNoSpace(insufficient_reachable_points).

-!buildSharedSpaceWith(Other): interactionSpaceFailed(Other, Reason) & sharesNoSpace(Reason) <-
    .print("No shared space with ", Other, " (", Reason, "): their reach spheres overlap, ",
           "so the pair was worth asking about, but the sampled workspaces never meet").

-!buildSharedSpaceWith(Other): interactionSpaceFailed(Other, Reason) <-
    .print("Could not build the shared space with ", Other, ": ", Reason).

-!buildSharedSpaceWith(Other): true <-
    .print("Could not build the shared space with ", Other, ": the estimator never answered").

// Draw the pair's cloud in the scene, from the agent that computed it.
+!renderInteractionSpace(Other): interactionSpaceReady(_, Other) <-
    .abolish(interactionVisual(Other, _));
    wotAchieve("ownLayer1", "renderInteractionSpace", [Other]);
    .wait(interactionVisual(Other, _), 15000);
    if (interactionVisual(Other, Points)) {
        .print("Rendered the ", Other, " interaction space: ", Points, " points")
    }.

-!renderInteractionSpace(Other): true <-
    .print("Interaction space with ", Other, " was computed but not drawn").
