// What this arm believes about the task itself: which cube supports stand in
// the cell, which of them it could actually serve, and which one currently
// holds the cube.
//
// None of this is configured. Two robots running the same plan file in the same
// scene arrive at different beliefs, because the only thing that differs is
// where they stand and how far they reach. That is what makes the search that
// follows a search at all: if the answer to "do you have B?" came from a
// generated fact, there would be nothing to ask anybody about.

+!detectTaskSites: true <-
    .abolish(site(_, _, _, _));
    .abolish(siteReachable(_));
    .abolish(siteOccupied(_, _));
    .abolish(siteEmpty(_));
    .abolish(taskSitesReady(_, _));
    .abolish(taskSitesFailed(_));
    wotAchieve("ownLayer1", "detectTaskSites", []);
    .wait(taskSitesReady(_, _) | taskSitesFailed(_), 20000);
    if (taskSitesFailed(Reason)) {
        .print("Task-site detection failed: ", Reason);
        .fail
    };
    !reportTaskSites.

// Losing the scan silently is how an arm ends up believing the cell has no
// goal site at all, and answering "no route" to a neighbour that was right to
// ask. Say it out loud instead.
-!detectTaskSites: true <-
    .print("Task-site detection did not complete; this arm knows of no cube supports and will answer every goal query with 'not mine'");
    !announceUnavailable(task_sites).

// ---------------------------------------------------------------------------
// An arm that cannot take part at all.
// ---------------------------------------------------------------------------
//
// Distinct from every other failure in this file, and the distinction is the
// whole point: a step that fails WITH a reason is the layer telling this arm
// something, and the arm can still do everything that step was not about. A step
// whose layer never answered is the layer not being there, and there is nothing
// left for the arm to do -- it cannot measure itself, cannot compute a shared
// space, and cannot move.
cannotServe :- ownLayerUnresponsive(_).

// An arm no route may run through, for either of the two reasons the cell ever
// learns one. armUnfit is a refusal of a handover the arm was asked to perform;
// armUnavailable is an arm saying it cannot perform anything. Same consequence,
// so the searches read one rule and not two.
outOfService(Robot) :- armUnfit(Robot).
outOfService(Robot) :- armUnavailable(Robot).

// The layer went away while the arm was working, not while it was starting.
//
// Until this plan existed, `cannotServe` was reachable only from the two init
// handlers, so an arm whose layer died AFTER the route was settled had no way to
// arrive at it at all. It kept calling a socket that was not there, kept failing,
// and never told anybody -- while the rest of the cell went on holding a route
// through an arm that could no longer move. The arm that fails at minute ten is
// in exactly the state of the arm that failed at second one, and this is what
// makes it say so.
//
// One signal per outage, not one per dropped request: the artifact counts the
// rest. Nothing here names a robot or a step -- it is whichever arm's own layer
// stops answering, whenever that happens.
// Told a second time, after the arm has already declared itself out of service.
//
// That is not news about the layer -- the arm knows -- it is news about the ARM:
// it is still issuing commands to a socket it has been told is shut, which is a
// loop it cannot see from the inside. Every plan in the cell fails cleanly on its
// own, and the ones that retry are individually bounded, yet the arm as a whole
// still spun 95000 dropped requests deep and slowed down the arms that were
// working. So the backstop does not try to name the loop: it ends it.
//
// Dropping everything costs nothing here. An arm whose layer is gone has no
// intention in flight that can still succeed, and it has already told the cell to
// route around it. It stays out of service until the MAS is restarted, which is
// the honest state to be in -- and it is one arm being idle instead of one arm
// taxing the whole cell.
+layerUnreachable(Robot, Request)
    : cannotServe
<-
    .print("My layer is still not answering and I am still asking it (", Request,
           "); I have already told the cell I cannot serve, so this is a loop -- dropping everything");
    .drop_all_intentions.

// The first time: the layer went away while the arm was working, not while it was
// starting. Until this plan existed, `cannotServe` was reachable only from the two
// init handlers, so an arm whose layer died AFTER the route was settled had no way
// to arrive at it at all. It kept calling a socket that was not there and never
// told anybody, while the rest of the cell went on holding a route through an arm
// that could no longer move.
+layerUnreachable(Robot, Request): true <-
    .print("My own layer stopped answering (", Request, " to ", Robot,
           "); I cannot act in the cell any more");
    !announceUnavailable(runtime).

// Tell the cell once, unprompted.
//
// A dead arm is a fact every other arm needs and none of them can discover: they
// ask it in turn, and each waits out a deadline for the answer it already knows.
// One broadcast is the difference between the cell routing around it in
// milliseconds and doing so three timeouts later.
//
// Broadcast rather than sent to the neighbours, because the neighbour scan is
// one of the things that failed: an arm in this state does not know who its
// neighbours are. Nothing about which arm is failing is written down anywhere --
// it is whichever arm finds its own layer silent.
+!announceUnavailable(Step)
    : agentIdentity(_, MyRobot) & not unavailabilityAnnounced
<-
    +ownLayerUnresponsive(Step);
    +unavailabilityAnnounced;
    .print("My own layer did not answer during ", Step,
           "; telling the cell I cannot serve, so nobody waits on me to say so");
    .broadcast(tell, armUnavailable(MyRobot)).

// Already announced, or no identity to announce with. The belief is still
// recorded: it is what stops this arm waiting on itself.
+!announceUnavailable(Step): true <-
    +ownLayerUnresponsive(Step).

-!announceUnavailable(_): true <-
    .print("Could not announce that this arm cannot serve").

// A site this arm can serve and that currently holds a payload: this arm is
// standing at A. A site it can serve that is empty: a candidate B.
//
// Reachability is deliberately part of both. A support across the cell is a
// support this robot can SEE and NAME -- which is why site/4 is asserted for
// it -- but not one it can pick from or place onto, and the whole point of the
// distributed search is that the difference is decided locally.
hasSource(Site, Payload) :- site(Site, _, _, _) & siteReachable(Site) & siteOccupied(Site, Payload).
hasSink(Site) :- site(Site, _, _, _) & siteReachable(Site) & siteEmpty(Site).

// An arm this one could hand the cube to. Being neighbours is the whole test:
// whether the pair actually has a rendezvous is settled later by refreshing
// their shared space and ranking it, which fails cleanly and is already
// retried. Ownership of that space says who COMPUTES it, never who may give.
canGiveTo(Robot) :- neighbor(_, Robot).

// A downstream answer this arm may still act on.
//
// The search did not hand anybody a route; it handed every arm on the way a
// RANKING, and each of them kept its own part of it. What changes once the cube
// is physically moving is not the ranking but which of its entries the cell will
// still stand behind, and that is what this rule holds apart: an arm that could
// not be handed to is struck, not forgotten, so the reason stays in the
// transcript and the same hop is not chosen again a minute later.
//
// Two kinds of striking, because they are learnt in two different places. This
// arm's own refusal is per search (`hopExcluded`); an arm struck further up the
// route arrives with the cube and applies to every search this arm is part of
// (`armUnfit`).
openAnswer(Task, Value, Agent, Path) :-
    routeAnswer(Task, Agent, Value, Path)
    & Value \== none
    & not hopExcluded(Task, Agent)
    & neighbor(Agent, Robot)
    & not outOfService(Robot).

// A site this arm knows about but cannot serve. Not an error and not a
// failure: it is the evidence behind a truthful "not mine".
knownButUnreachable(Site) :- site(Site, _, _, _) & not siteReachable(Site).

+!reportTaskSites
    : agentIdentity(_, MyRobot)
      & taskSitesReady(Seen, Reachable)
<-
    .print("Task sites for ", MyRobot, ": ", Seen, " seen in the cell, ", Reachable, " within reach");
    .findall(line(S, X, Y, Z), site(S, X, Y, Z), Sites);
    !printTaskSites(Sites);
    .findall(A, hasSource(A, _), Sources);
    .findall(B, hasSink(B), Sinks);
    if (Sources \== []) {
        .print("  => holds the loaded support (A): ", Sources)
    };
    if (Sinks \== []) {
        .print("  => holds an empty support (candidate B): ", Sinks)
    };
    if (Sources == [] & Sinks == []) {
        .print("  => serves neither end of the transfer; a carrier at most")
    }.

+!printTaskSites([]): true <- true.

+!printTaskSites([line(Site, X, Y, Z) | Rest]): true <-
    if (siteReachable(Site)) {
        if (siteOccupied(Site, Payload)) {
            .print("  ", Site, " at (", X, ", ", Y, ", ", Z, ") -> reachable, holding ", Payload)
        } else {
            .print("  ", Site, " at (", X, ", ", Y, ", ", Z, ") -> reachable, empty")
        }
    } else {
        .print("  ", Site, " at (", X, ", ", Y, ", ", Z, ") -> seen but out of reach")
    };
    !printTaskSites(Rest).

// ---------------------------------------------------------------------------
// What a handover costs the arm that performs it.
// ---------------------------------------------------------------------------

// One edge, priced from this arm's own journey.
//
// The lower layer prices every viable rendezvous and reports none of them as
// chosen. The choice is made here, with the SAME weighting !requestHandoverPlan
// commits a real handover with, because a cost for a rendezvous the handover
// would never select is a cost for something that never happens. Geometry
// below, judgement here -- the same split the candidate ranking already uses.
+!estimateEdgeCost(Neighbour): true <-
    !estimateEdgeCost(Neighbour, pickup).

// One arm answers several searches at once, and they ask for the same edge.
// Two runs of this plan over one neighbour and mode wipe each other's working
// beliefs on the way in, so the first run's .wait comes back to an empty
// ranking and an edge that is perfectly usable is reported as unusable. The
// second asker waits for the first answer instead of starting a second run.
+!estimateEdgeCost(Neighbour, Mode) : edgePricing(Neighbour, Mode) <-
    .wait(not edgePricing(Neighbour, Mode), 70000);
    ?edgeCost(Neighbour, Mode, _, _).

+!estimateEdgeCost(Neighbour, Mode) : handoverHeightRange(MinZ, MaxZ) <-
    +edgePricing(Neighbour, Mode);
    // Only this question's answers. Wiping the neighbour's other mode would
    // erase a price another intention is in the middle of reading.
    .abolish(edgeCandidate(Neighbour, Mode, _, _, _, _, _, _, _));
    .abolish(edgeCostOrigin(Neighbour, Mode, _, _, _, _));
    .abolish(edgeCostReady(Neighbour, Mode, _));
    .abolish(edgeCostFailed(Neighbour, Mode, _));
    .abolish(edgeCost(Neighbour, Mode, _, _));
    wotAchieve("ownLayer1", "estimateEdgeCost", [Neighbour, MinZ, MaxZ, Mode]);
    .wait(edgeCostReady(Neighbour, Mode, _) | edgeCostFailed(Neighbour, Mode, _), 60000);
    if (edgeCostFailed(Neighbour, Mode, Reason)) {
        e11.event(edge_refused, edge_cost, Neighbour, Mode, Reason);
        .print("Cannot price the handover to ", Neighbour, ": ", Reason);
        .fail
    };
    .findall(rank(Score, Candidate, Distance),
        edgeCandidate(Neighbour, Mode, Candidate, Distance,
            Centrality, Density, GiverSupport, ReceiverSupport, Posture)
        & Score = 0.32 * Centrality + 0.2 * Density
            + 0.14 * GiverSupport + 0.14 * ReceiverSupport + 0.2 * Posture,
        Rankings);
    if (Rankings == []) {
        .print("No rendezvous with ", Neighbour, " survived ranking; this arm cannot hand over there");
        .fail
    };
    .max(Rankings, rank(_, BestCandidate, BestDistance));
    +edgeCost(Neighbour, Mode, BestDistance, BestCandidate);
    e11.event(edge_priced, edge_cost, Neighbour, Mode, BestDistance, BestCandidate);
    .abolish(edgePricing(Neighbour, Mode));
    !reportEdgeCost(Neighbour, Mode, BestDistance, BestCandidate);
    // How much choice there was is part of what the price means. A pair whose
    // shared space is one voxel prices exactly as confidently as one with three
    // hundred, and the route reads the two numbers the same way -- but the first
    // has no second candidate to fall back on when the rendezvous is refused,
    // and the arms meet it at the far edge of both their reaches.
    .length(Rankings, Choices);
    if (Choices <= 2) {
        .print("  that edge has ", Choices, " usable rendezvous in total; a route through ",
               Neighbour, " has nothing to fall back on if it is refused")
    }.

// A pricing run that fails still has to release the edge, or every arm waiting
// on it waits out the whole timeout for an answer that is never coming.
-!estimateEdgeCost(Neighbour, Mode) : true <-
    .abolish(edgePricing(Neighbour, Mode));
    .fail.

+!reportEdgeCost(Neighbour, Mode, Distance, Candidate)
    : edgeCostOrigin(Neighbour, Mode, Kind, OX, OY, OZ)
<-
    if (Kind == pickup) {
        .print("Handover to ", Neighbour, " costs ", Distance,
               " m from the cube pickup at (", OX, ", ", OY, ", ", OZ, ") via ", Candidate)
    } else {
        .print("Handover to ", Neighbour, " costs ", Distance,
               " m from the current pose at (", OX, ", ", OY, ", ", OZ, ") via ", Candidate)
    }.

+!reportEdgeCost(Neighbour, _, Distance, Candidate): true <-
    .print("Handover to ", Neighbour, " costs ", Distance, " m via ", Candidate).

// Price every neighbour, then take the cheapest.
//
// One neighbour that cannot be priced must not cost the arm the others: an edge
// this arm cannot use is a real answer about that edge and nothing more.
+!rankHandoverCosts: agentIdentity(_, MyRobot) <-
    .abolish(cheapestHandover(_, _));
    .findall(N, neighbor(_, N), Neighbours);
    .print("Pricing handovers from ", MyRobot, " to ", Neighbours);
    !priceEdges(Neighbours);
    .findall(cost(C, N), edgeCost(N, pickup, C, _), Costs);
    if (Costs == []) {
        .print("No neighbour of ", MyRobot, " can be handed to; this arm is on its own");
        .fail
    };
    .min(Costs, cost(BestCost, BestNeighbour));
    +cheapestHandover(BestNeighbour, BestCost);
    .print("Cheapest handover from ", MyRobot, ": ", BestNeighbour, " at ", BestCost, " m  (of ", Costs, ")").

+!priceEdges([]): true <- true.

+!priceEdges([Neighbour | Rest]): true <-
    !estimateEdgeCost(Neighbour);
    !priceEdges(Rest).

-!priceEdges([Neighbour | Rest]): true <-
    .print("Leaving ", Neighbour, " unpriced and continuing with the rest");
    !priceEdges(Rest).

// The arm standing at A prices its options as soon as it knows it is there.
// Every other arm has nothing to price yet -- what a carry costs depends on
// where the cube is coming from, which nobody has told it.
+!reportHandoverCosts: hasSource(Site, Payload) <-
    .print("Standing at the loaded support ", Site, " with ", Payload, "; pricing the ways out");
    !rankHandoverCosts.

+!reportHandoverCosts: not hasSource(_, _) <- true.

-!reportHandoverCosts: true <-
    .print("Could not price this arm's handovers").

// ---------------------------------------------------------------------------
// Finding the way to the goal, by asking.
// ---------------------------------------------------------------------------
//
// Nobody holds a map. An arm knows only which supports it can serve and which
// arms it could collide with, and from those two local facts the cell works out
// a route by flooding a question outward and accumulating a cost on the way
// back. Each arm adds exactly one term -- what the handover it is answering
// across costs it -- to the total it received from downstream, so the number
// arriving at a requester is already the whole cost of the remaining route and
// the requester decides with a bare minimum.

routeSerial(0).

+!nextRouteSerial(Serial): routeSerial(Serial) <-
    -routeSerial(Serial);
    Next = Serial + 1;
    +routeSerial(Next).

// The arm that holds the cube starts the search and is the only one that also
// pays for its own first leg: it has nobody upstream to report that leg to, so
// it is the one term the accumulated totals cannot contain.
+!findRouteTo(Goal)
    : agentIdentity(Me, MyRobot)
      & hasSource(Source, Cube)
<-
    e11.event(phase_start, route_discovery, Goal);
    !forgetRoutes;
    if (hasSink(Goal)) {
        .print("The goal ", Goal, " is in my own workspace; no handover is needed");
        +routeIsLocal(Goal);
        e11.event(route_end, route_discovery, local, Goal)
    } else {
        !nextRouteSerial(Serial);
        Task = route(MyRobot, Goal, Serial);
        +routeTask(Task);
        +routeOrigin(Task);
        +routeSeen(Task);
        .findall(peer(A, R), neighbor(A, R), Peers);
        if (Peers == []) {
            .print("Holding ", Cube, " at ", Source, " with nobody to pass it to");
            +noRouteTo(Task);
            e11.event(route_end, route_discovery, no_route, Task)
        } else {
            .length(Peers, Asked);
            +routeAsked(Task, Asked);
            .print("Asking ", Asked, " neighbour(s) who can reach ", Goal, "  [", Task, "]");
            // The budget travels with the question and shrinks at every hop, so
            // a neighbour always answers before the arm that asked it gives up.
            // Flat deadlines let a child reply a moment after its parent has
            // already settled on the answers that happened to arrive -- and the
            // parent then reports no route while holding, unread, the reply
            // that proved there was one.
            !askNeighbours(Peers, Task, Cube, Goal, 8, 240000);
            !routeDeadline(Task, 240000);
            .wait(routeSettled(Task), 250000)
        }
    }.

+!forgetRoutes: true <-
    .abolish(routeTask(_));
    .abolish(routeOrigin(_));
    .abolish(routeSeen(_));
    .abolish(routePricing(_));
    .abolish(routeDeferred(_, _));
    .abolish(routeRequestFrom(_, _));
    .abolish(routeAsked(_, _));
    .abolish(routeAskedPeer(_, _));
    .abolish(routeUnreachable(_, _));
    .abolish(routeTakenUp(_, _));
    .abolish(routeAnswer(_, _, _, _));
    .abolish(routeOption(_, _, _, _));
    .abolish(routeSettled(_));
    .abolish(routeIsLocal(_));
    .abolish(nextHop(_, _, _, _));
    .abolish(routeTotal(_, _));
    .abolish(hopExcluded(_, _));
    .abolish(armUnfit(_));
    .abolish(rerouteAttempted(_));
    .abolish(noRouteTo(_)).

+!askNeighbours([], _, _, _, _, _): true <- true.

// Each neighbour is written down by name before the question goes out, so a
// reply can never arrive before the arm that sent it is on the list.
//
// The send is a goal of its own so that one arm being unaskable cannot cost the
// arm the rest of the list. Inline, a failure here stopped the recursion, and
// the neighbours further down were never asked at all.
+!askNeighbours([peer(Agent, _) | Rest], Task, Cube, Goal, TTL, Deadline): true <-
    +routeAskedPeer(Task, Agent);
    !askNeighbour(Agent, Task, Cube, Goal, TTL, Deadline);
    !askNeighbours(Rest, Task, Cube, Goal, TTL, Deadline).

// An arm that has told the cell it cannot serve is not asked at all. Jason would
// deliver the question -- that agent is in the MAS and reasoning -- and the arm
// would then spend a deadline arriving at the answer it has already broadcast.
+!askNeighbour(Agent, Task, _, _, _, _)
    : neighbor(Agent, Robot) & armUnavailable(Robot)
<-
    .print("Not asking ", Robot, " about the route: it has told the cell it cannot serve");
    !refuseOnBehalfOf(Agent, Task).

+!askNeighbour(Agent, Task, Cube, Goal, TTL, Deadline): true <-
    .send(Agent, achieve, findRoute(Task, Cube, Goal, TTL, Deadline)).

// An arm that is not there to be asked is not a way to the goal, and that is
// all it is.
//
// Jason FAILS .send when the receiver is not in the MAS, so an arm whose agent
// never started -- or has since gone -- used to take the whole search down at
// whoever tried to ask it. The arm holding the cube reported that route
// discovery did not complete, while two perfectly good branches stood unasked
// beside the one that could not be reached. Live cell: robot1's neighbours are
// [robot4,robot3,robot2] and robot4 was not in the MAS, so robot4 being first in
// the list was the whole of the failure -- robot3 and robot2 were never asked.
//
// Answering on the absent arm's behalf is what keeps the tally honest. It was
// counted into routeAsked before the send went out, so leaving it without a
// verdict does not merely lose that branch: it makes the count unreachable, and
// the search can then only ever end by running out of time.
-!askNeighbour(Agent, Task, _, _, _, _): true <-
    .print("Cannot put the question to ", Agent,
           "; no route to the goal runs through an arm that is not there to answer");
    !refuseOnBehalfOf(Agent, Task).

// The verdict for an arm that cannot answer, however that was found out: not in
// the MAS at all, or in it and already telling everyone it cannot serve.
//
// Recording it is not optional. The peer was counted into routeAsked before
// anyone tried to reach it, so a branch left without a verdict does not merely go
// unheard -- it puts the tally out of reach, and the search can then only end by
// running out of time.
+!refuseOnBehalfOf(Agent, Task): true <-
    e11.event(peer_unreachable, route_discovery, Agent, Task);
    +routeUnreachable(Task, Agent);
    +routeAnswer(Task, Agent, none, []);
    !settleIfComplete(Task).

-!refuseOnBehalfOf(Agent, _): true <-
    .print("Could not record the refusal on behalf of ", Agent).

// ---- receiving the question ------------------------------------------------
//
// A question can arrive before this arm has finished looking at the cell -- the
// arms start together and take different times to compute their shared spaces.
// An arm that answered from an empty belief base would truthfully report having
// no goal it never looked for, and the search would settle on that. So the
// question waits for the answer to mean something.

// Asked by an arm that has not heard the announcement yet -- or that asked before
// it went out.
//
// Waiting is right when detection is still running and wrong when the layer is
// gone: taskSitesReady can no longer be asserted, so the wait can only end in the
// deadline, and it spends the REQUESTER's budget to reach an answer this arm
// already has. An arm that cannot look at the cell knows of no goal, and nothing
// can be routed through an arm that cannot move.
+!findRoute(Task, _, Goal, _, _)[source(Upstream)]
    : not routeSeen(Task) & cannotServe
<-
    +routeSeen(Task);
    .print("Asked about ", Goal, " but my own layer never answered; saying no route at once ",
           "rather than making the arm that asked wait for me to find out");
    .send(Upstream, tell, noRoute(Task)).

+!findRoute(Task, Cube, Goal, TTL, Deadline)[source(Upstream)]
    : not routeSeen(Task) & not taskSitesReady(_, _)
<-
    .print("Asked about ", Goal, " before I finished looking at the cell; waiting to answer honestly");
    .wait(taskSitesReady(_, _), 30000);
    !answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream).

+!findRoute(Task, Cube, Goal, TTL, Deadline)[source(Upstream)]: true <-
    !answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream).

// Clause order is the protocol.
//
// Holding the goal is checked FIRST, before the already-seen rule. An arm that
// holds the goal never forwards, so answering every arm that asks cannot send
// the flood round a cycle -- and if it answered only the first, a question that
// happened to arrive across an edge it cannot hand over on would spend its one
// answer saying "no route", which was never a statement about reachability at
// all. Every other arm that could have reached the goal through it would then
// be told there was no way there.
+!answerFindRoute(Task, _, Goal, _, _, Upstream)
    : hasSink(Goal)
      & neighbor(Upstream, UpstreamRobot)
      & agentIdentity(_, MyRobot)
<-
    +routeSeen(Task);
    +routeRequestFrom(Task, Upstream);
    +routeSettled(Task);
    !ensureEdgePriced(UpstreamRobot, pose);
    ?edgeCost(UpstreamRobot, pose, Leg, _);
    .print("I hold the goal ", Goal, "; answering ", UpstreamRobot, " with my own leg ", Leg);
    .send(Upstream, tell, routeCost(Task, [], Leg, [MyRobot])).

// I cannot hand over to the arm that asked me, so there is no route to the goal
// through this pair -- and that is all it means.
-!answerFindRoute(Task, _, Goal, _, _, Upstream)
    : hasSink(Goal)
<-
    .print("I hold ", Goal, " but cannot hand over to my requester");
    .send(Upstream, tell, noRoute(Task)).

// I hold the claim on this question, so this is the intention that carries it.
// It sits above the already-seen and still-pricing rules on purpose: the claim
// sets routePricing, and without this clause the claimant's own re-entry would
// be caught by the deferral rule below and wait for itself.
//
// not routeSeen is what stops it matching once the work is done: the claim
// token outlives the take-up, and a later copy from the same upstream must get
// the ordinary "already seen" answer, not a second run of the whole search.
+!answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream)
    : routeTakenUp(Task, Upstream)
      & not routeSeen(Task)
      & agentIdentity(_, MyRobot)
      & neighbor(Upstream, UpstreamRobot)
<-
    !takeUpRoute(Task, Cube, Goal, TTL, Deadline, Upstream, MyRobot, UpstreamRobot).

// A question already seen is answered "no" at once, which is what stops the
// flood going round the cell's cycles forever.
+!answerFindRoute(Task, _, _, _, _, Upstream)
    : routeSeen(Task)
<-
    .send(Upstream, tell, noRoute(Task)).

// The edge a question came in on is priced BEFORE the question is taken up,
// and pricing takes seconds of shared-space work. A second copy of the same
// question arriving inside that window finds no routeSeen yet, so without this
// it would start a second search of its own: two intentions forwarding the same
// question, counting each other's answers against their own tally, and racing
// to settle. Waiting for the first attempt to resolve costs nothing, because
// whatever it resolves to is the right answer for this requester too.
+!answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream)
    : routePricing(Task) & not routeDeferred(Task, Upstream)
<-
    +routeDeferred(Task, Upstream);
    .wait(routeSeen(Task) | not routePricing(Task), 70000);
    !answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream).

// Still being weighed after all that. Answering "no" is wrong only if it is
// silent: the requester would otherwise hold its branch open for an arm that
// has already been asked the same thing by somebody else.
+!answerFindRoute(Task, _, _, _, _, Upstream)
    : routePricing(Task)
<-
    .send(Upstream, tell, noRoute(Task)).

+!answerFindRoute(Task, _, _, _, Deadline, Upstream)
    : Deadline <= 30000
<-
    +routeSeen(Task);
    .print("No time left to answer ", Task, " usefully");
    .send(Upstream, tell, noRoute(Task)).

+!answerFindRoute(Task, _, _, TTL, _, Upstream)
    : TTL <= 0
<-
    +routeSeen(Task);
    .print("Hop budget exhausted for ", Task);
    .send(Upstream, tell, noRoute(Task)).

// I do not have it, so I ask everyone except whoever asked me, and remember
// them so the answer can find its way back.
//
// Nobody has claimed this question yet, so try to. The claim is a goal of its
// own and not a line of this plan, because Jason checks a context in one
// reasoning step and runs the first body action in the next, and is free to
// interleave between the two. Two copies of the same question arriving inside
// that gap both passed the context, both priced their edge, and both forwarded:
// two intentions running one search, counting each other's answers against a
// single tally and racing to settle it. Live cell: robot4, asked by robot2 and
// by robot3 within the same instant.
//
// Whoever wins the claim comes back through here and is picked up by the
// take-up clause above; whoever loses finds the claim set and falls to the
// deferral rule, which is what should have happened to it in the first place.
// The claim cannot be re-entered: its context requires that nobody holds it.
+!answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream)
    : not routeTakenUp(Task, _)
      & agentIdentity(_, _)
      & neighbor(Upstream, _)
<-
    !claimRoute(Task, Upstream);
    !answerFindRoute(Task, Cube, Goal, TTL, Deadline, Upstream).

// Belief operations only and no subgoals, so an atomic plan settles it outright:
// exactly one intention finds the question unclaimed and takes it.
@claimRoute[atomic]
+!claimRoute(Task, Upstream)
    : not routeSeen(Task)
      & not routePricing(Task)
      & not routeTakenUp(Task, _)
<-
    +routePricing(Task);
    +routeTakenUp(Task, Upstream).

// Someone got there first, or the question was already answered. Either way
// there is nothing to claim, and the caller's re-entry will find the right rule.
+!claimRoute(_, _): true <- true.

// The claim is held; this is where the question is actually taken up. The edge
// back to whoever asked is priced first: if our shared space carries no
// rendezvous, no route from them runs through me -- and that is a fact about
// our edge, not about the goal. Answering "no" is right; spending my one answer
// to do it is not, because an arm I COULD have served may ask next and be
// refused by the already-seen rule.
+!takeUpRoute(Task, Cube, Goal, TTL, Deadline, Upstream, _, UpstreamRobot)
    : true
<-
    !ensureEdgePriced(UpstreamRobot, pose);
    +routeSeen(Task);
    +routeRequestFrom(Task, Upstream);
    .abolish(routePricing(Task));
    .findall(peer(A, R), neighbor(A, R) & A \== Upstream, Peers);
    // Seeing a site and being able to serve it are different facts, and the
    // difference is the whole reason the search exists. Saying it out loud is
    // what turns "every branch came back empty" -- which is what the cell
    // reports when NOBODY can serve the goal -- into a sentence naming the arm
    // that was closest to it and by how far it missed.
    if (knownButUnreachable(Goal) & site(Goal, GX, GY, GZ)) {
        .print("I can see ", Goal, " at (", GX, ", ", GY, ", ", GZ,
               ") in my own frame but cannot reach it")
    };
    if (Peers == []) {
        .print("Nobody to pass ", Task, " on to");
        +routeSettled(Task);
        .send(Upstream, tell, noRoute(Task))
    } else {
        .length(Peers, Asked);
        +routeAsked(Task, Asked);
        Next = TTL - 1;
        // Enough taken off for this arm to price its own reply and send it
        // before whoever asked stops listening.
        Onward = Deadline - 45000;
        .print("Passing the question on to ", Asked, " neighbour(s), ",
               Onward, "ms of budget left  [", Task, "]");
        !askNeighbours(Peers, Task, Cube, Goal, Next, Onward);
        !routeDeadline(Task, Onward)
    }.

// Refused before the question was ever taken up: no belief is left behind, so
// the next arm to ask still gets a real answer.
//
// Only the claimant clears the claim. A sibling intention failing on its own
// edge used to wipe routePricing out from under the arm that was mid-pricing,
// which reopened exactly the double take-up the claim exists to prevent.
-!takeUpRoute(Task, _, _, _, _, Upstream, _, UpstreamRobot)
    : not routeSeen(Task)
<-
    .abolish(routePricing(Task));
    .abolish(routeTakenUp(Task, Upstream));
    .print("I cannot hand over to ", UpstreamRobot, ", so no route from them runs through me");
    .send(Upstream, tell, noRoute(Task)).

-!takeUpRoute(Task, _, _, _, _, Upstream, _, _): true <-
    .print("Could not take up ", Task, " after claiming it; answering no route");
    .send(Upstream, tell, noRoute(Task)).

// An arm that cannot answer must still answer. Silence upstream is
// indistinguishable from a slow neighbour, and the requester would wait out its
// whole deadline for a branch that was never coming.
// The arm that could not answer is very often NOT the arm that owns the search:
// the same question reaches it twice, once across an edge it can hand over on
// and once across one it cannot, and only the second copy fails. Settling the
// task here would close the search its sibling intention is still running --
// the deadline stops waiting, settleIfComplete stops counting, and the answers
// already on their way back are read by nobody. Answer this requester and leave
// the search alone.
-!answerFindRoute(Task, _, _, _, _, Upstream): true <-
    .abolish(routeDeferred(Task, Upstream));
    .print("Could not answer ", Task, " for this requester; the search it belongs to is untouched");
    .send(Upstream, tell, noRoute(Task)).

-!findRoute(Task, _, _, _, _)[source(Upstream)]: true <-
    .print("Route question ", Task, " could not be taken up");
    .send(Upstream, tell, noRoute(Task)).

// ---- receiving the answers -------------------------------------------------
//
// The value that arrives is already the whole cost of the route beyond that
// neighbour: they added their own leg before sending it. Nothing is added here.

// One neighbour, one verdict. An arm can be asked the same question across two
// edges and replies to each, and until this was written down per neighbour both
// replies counted towards the same tally -- so a handful of instant refusals
// from arms that were never the live branch could complete the count on their
// own. Live cell: robot4 asked robot5, robot3, robot2 and robot1, and settled
// on the three instant "already seen" refusals while robot5 -- the arm actually
// holding the goal -- was still pricing the leg that proved the route existed.
//
// A real cost may still replace that neighbour's earlier "no". The refusal can
// come from an intention that had already spent its one answer elsewhere, while
// the leg it was refusing was genuinely there; a cheaper cost may likewise
// arrive second. Never the other way round: a "no" never overwrites a leg.
// A reply carries that neighbour's verdict AND the other ways to the goal that
// run through it. Every hop takes a minimum, so without the second half the
// only route the arm holding the cube ever hears about is the one that happened
// to win at every hop in between -- the alternatives are real, priced, and
// discarded one hop from the arm that would have liked to see them. They travel
// inside the reply rather than as messages of their own, so they cannot arrive
// after the answer that completes the tally and settles the search.
+routeCost(Task, Options, Value, Path)[source(Downstream)]
    : routeAnswer(Task, Downstream, none, _)
<-
    -routeAnswer(Task, Downstream, none, _);
    !acceptAnswer(Task, Downstream, Options, Value, Path).

+routeCost(Task, Options, Value, Path)[source(Downstream)]
    : routeAnswer(Task, Downstream, Old, _) & Old \== none & Value < Old
<-
    -routeAnswer(Task, Downstream, Old, _);
    !acceptAnswer(Task, Downstream, Options, Value, Path).

+routeCost(Task, _, _, _)[source(Downstream)]
    : routeAnswer(Task, Downstream, _, _)
<-
    true.

+routeCost(Task, Options, Value, Path)[source(Downstream)]: true <-
    !acceptAnswer(Task, Downstream, Options, Value, Path).

// One neighbour's answer replaces that neighbour's answer whole. Keeping the
// alternatives of a verdict that has been superseded would report routes the
// arm no longer believes in.
+!acceptAnswer(Task, Downstream, Options, Value, Path): true <-
    .abolish(routeOption(Task, Downstream, _, _));
    +routeAnswer(Task, Downstream, Value, Path);
    !recordOptions(Task, Downstream, Options);
    !settleIfComplete(Task).

+!recordOptions(_, _, []): true <- true.

+!recordOptions(Task, Downstream, [option(Value, Path) | Rest]): true <-
    +routeOption(Task, Downstream, Value, Path);
    !recordOptions(Task, Downstream, Rest).

+noRoute(Task)[source(Downstream)]
    : routeAnswer(Task, Downstream, _, _)
<-
    true.

+noRoute(Task)[source(Downstream)]: true <-
    +routeAnswer(Task, Downstream, none, []);
    !settleIfComplete(Task).

// The search is complete when every neighbour that was asked has answered --
// counted by name, not in bulk. A neighbour that answered twice used to stand
// in for one that had not answered at all, and the branch still in flight was
// the slow one: an "already seen" refusal is instant, while a real leg costs
// the arm a rendezvous ranking first.
//
// How many were asked still comes from routeAsked, fixed before the first
// question went out. Deriving it from the peers written down so far would move
// the target: an instant reply from the first neighbour would find a list of
// one and settle the whole search on it.
+!settleIfComplete(Task)
    : routeAsked(Task, Asked)
      & not routeSettled(Task)
<-
    .setof(A, routeAskedPeer(Task, A) & routeAnswer(Task, A, _, _), Answered);
    .length(Answered, Got);
    if (Got >= Asked) {
        !settleRoute(Task)
    }.

+!settleIfComplete(_): true <- true.

// Settling is the only subgoal a failure handler calls in this file, and a
// subgoal that fails inside a -! handler regenerates the event that ran it --
// Jason does not skip an intended means that is already a failure plan. The
// deadline is the designed backstop for a search that does not settle, so
// swallowing the failure here loses nothing that was not already covered.
-!settleIfComplete(Task): true <-
    .print("Could not settle ", Task, " on this answer; the deadline still will").

// A branch that never answers must not hold the whole search open. Whatever
// arrived by the deadline is what the choice is made from.
//
// .wait FAILS when its deadline passes without the condition -- it does not
// resume and carry on -- so a bare .wait here took the whole plan down at the
// exact moment it was written for. The branch below never ran: a quiet
// neighbour failed !answerFindRoute instead of settling the search, and the
// answers that HAD arrived were read by nobody. Sitting the wait in its own
// goal turns that failure back into what the deadline always meant.
+!routeDeadline(Task, Deadline): true <-
    !settleWhenTimeIsUp(Task, Deadline);
    if (not routeSettled(Task)) {
        .setof(A, routeAskedPeer(Task, A) & not routeAnswer(Task, A, _, _), Silent);
        e11.event(route_deadline, route_discovery, Task, Silent);
        .print("Deadline reached for ", Task, "; deciding on the answers that arrived");
        !settleRoute(Task)
    }.

+!settleWhenTimeIsUp(Task, Deadline): true <-
    .wait(routeSettled(Task), Deadline).

-!settleWhenTimeIsUp(_, _): true <- true.

// ---- deciding --------------------------------------------------------------

// The arm that started the search. Its own first leg is the term no accumulated
// total contains, so it is added here -- measured from the cube it is about to
// pick up, not from wherever the arm happens to be standing.
+!settleRoute(Task)
    : routeOrigin(Task)
      & agentIdentity(_, MyRobot)
      & not routeSettled(Task)
<-
    +routeSettled(Task);
    .findall(reached(A, R), openAnswer(Task, _, A, _) & neighbor(A, R), Reached);
    !priceReachedEdges(Reached);
    .findall(total(Total, Agent, Robot, Path),
        openAnswer(Task, Value, Agent, Path)
        & neighbor(Agent, Robot) & edgeCost(Robot, pickup, Leg, _)
        & Total = Leg + Value,
        Totals);
    if (Totals == []) {
        +noRouteTo(Task);
        .print("No route from ", MyRobot, " to the goal: every branch came back empty");
        e11.event(route_end, route_discovery, no_route, Task)
    } else {
        .min(Totals, total(Best, BestAgent, BestRobot, BestPath));
        +nextHop(Task, BestAgent, BestRobot, Best);
        +routeTotal(Task, Best);
        e11.event(route_end, route_discovery, success, Task, Best, [MyRobot|BestPath]);
        .print("Route settled: hand to ", BestRobot, " for a total of ", Best,
               " m, reaching the goal via ", BestPath);
        // The decision is still made on Totals alone, and it is the same
        // decision: every alternative reported through a neighbour is at least
        // as dear as that neighbour's own verdict, which is already in Totals.
        // The rest is what the cell found and did not choose -- worth seeing,
        // because it is the difference between "this is the way" and "this is
        // the cheapest of the ways", and because a route that is refused on the
        // day has to be replaced by something.
        .findall(total(Total, Agent, Robot, Path),
            routeOption(Task, Agent, Value, Path)
            & not hopExcluded(Task, Agent)
            & neighbor(Agent, Robot) & not outOfService(Robot)
            & edgeCost(Robot, pickup, Leg, _)
            & Total = Leg + Value,
            Alternatives);
        .concat(Totals, Alternatives, Every);
        .sort(Every, Ranked);
        .length(Ranked, HowMany);
        e11.event(route_options, route_discovery, Task, HowMany, Ranked);
        .print("  ", HowMany, " route(s) to the goal, cheapest first:");
        !printRoutes(Ranked)
    }.

+!printRoutes([]): true <- true.

+!printRoutes([total(Total, _, _, Path) | Rest]): true <-
    .print("    ", Total, " m  via ", Path);
    !printRoutes(Rest).

// Every other arm reports upstream instead of deciding for itself. It still
// records its own next hop, because that is what it will act on when the cube
// actually arrives.
+!settleRoute(Task)
    : not routeOrigin(Task)
      & not routeSettled(Task)
<-
    +routeSettled(Task);
    .findall(cost(Value, Agent, Path), openAnswer(Task, Value, Agent, Path), Costs);
    if (Costs == []) {
        .print("Nothing downstream of me can reach the goal  [", Task, "]");
        !answerUpstream(Task, none, [], [])
    } else {
        .min(Costs, cost(BestValue, BestAgent, BestPath));
        if (neighbor(BestAgent, BestRobot)) {
            +nextHop(Task, BestAgent, BestRobot, BestValue);
            .print("Best way on from me is ", BestRobot, " at ", BestValue, " m  [", Task, "]")
        };
        // What I act on is the minimum; what I report is everything, so the
        // choice is not quietly made for the arm upstream. Every neighbour's
        // own verdict, plus every alternative they reported from further out,
        // minus the one I am sending as my answer.
        .findall(option(V, P), openAnswer(Task, V, _, P), Direct);
        .findall(option(V, P),
            routeOption(Task, A, V, P) & not hopExcluded(Task, A)
            & neighbor(A, R) & not outOfService(R), Further);
        .concat(Direct, Further, Every);
        .delete(option(BestValue, BestPath), Every, Others);
        !answerUpstream(Task, BestValue, BestPath, Others)
    }.

+!settleRoute(_): true <- true.

// ---- rerouting -------------------------------------------------------------
//
// A hop can be settled on and still turn out to be unusable. The ranking was
// made from what the arms could compute about each other minutes before the
// cube started moving, and the one thing it cannot contain is whether the arm
// at the other end will still be there to take it.
//
// Recovering from that costs no messages at all, which is the whole pay-off of
// having accumulated the costs in the first place. The alternatives this arm
// collected are already priced, already complete all the way to the goal, and
// still sitting in its belief base next to the one it chose. So the arm holding
// the cube strikes the hop that failed and takes the next minimum over exactly
// the same answers -- one arm, deciding locally, with nothing it did not already
// have. Nobody re-floods, nobody is asked again, and no arm that is not on the
// remaining route hears about it.
//
// What this is NOT is a new global optimum. The cube is in this arm's gripper,
// so the choice is over routes that start here, and a cheaper route avoiding the
// failed arm that ran through some other part of the cell is out of reach by the
// time the cube has been carried this far. Optimal among the routes still open
// to the arm that holds the cube is the honest claim.

// Strike the hop and re-decide. Never fails: a reroute that found nothing is an
// answer, and leaving no nextHop behind is how it says so -- execution reads
// nextHop and nothing else, so the arm falls through to its terminal behaviour
// on its own.
+!rerouteAround(FailedAgent, FailedRobot)
    : nextHop(Task, FailedAgent, FailedRobot, Total)
      & agentIdentity(_, MyRobot)
<-
    +hopExcluded(Task, FailedAgent);
    -nextHop(Task, FailedAgent, FailedRobot, Total);
    .print(MyRobot, " strikes ", FailedRobot, " from the route: the ", Total,
           " m way to the goal through it cannot be executed");
    e11.event(hop_struck, reroute, FailedRobot, Total);
    !chooseAnotherHop(Task).

// Asked to reroute around an arm this arm was never going to hand to. Saying so
// is better than inventing a hop: whoever asked has the wrong idea of the route.
+!rerouteAround(_, FailedRobot): true <-
    .print("No hop of mine goes through ", FailedRobot, "; there is nothing to reroute around").

-!rerouteAround(_, FailedRobot): true <-
    .print("Could not reroute around ", FailedRobot).

// The origin re-decides with its own first leg priced in, exactly as it did at
// settling time. The leg is measured from the cube, and if the origin is still
// the arm holding it the cube has not moved, so the price still stands.
+!chooseAnotherHop(Task)
    : routeOrigin(Task) & agentIdentity(_, MyRobot)
<-
    .findall(reached(A, R), openAnswer(Task, _, A, _) & neighbor(A, R), Reached);
    !priceReachedEdges(Reached);
    .findall(total(Total, Agent, Robot, Path),
        openAnswer(Task, Value, Agent, Path)
        & neighbor(Agent, Robot) & edgeCost(Robot, pickup, Leg, _)
        & Total = Leg + Value,
        Totals);
    !adoptCheapest(Task, MyRobot, Totals).

// Every other arm decides on the totals as they arrived: each already contains
// the whole cost of the route beyond that neighbour, so this is the same bare
// minimum the settling did, over one fewer candidate.
+!chooseAnotherHop(Task): agentIdentity(_, MyRobot) <-
    .findall(total(Value, Agent, Robot, Path),
        openAnswer(Task, Value, Agent, Path) & neighbor(Agent, Robot),
        Totals);
    !adoptCheapest(Task, MyRobot, Totals).

+!adoptCheapest(_, MyRobot, []): true <-
    e11.event(reroute_exhausted, reroute, MyRobot);
    .print("No way on from ", MyRobot, " is left: every route the search found runs through an arm that cannot take the cube").

+!adoptCheapest(Task, MyRobot, Totals): true <-
    .min(Totals, total(Best, BestAgent, BestRobot, BestPath));
    +nextHop(Task, BestAgent, BestRobot, Best);
    .print("Rerouting from ", MyRobot, ": hand to ", BestRobot, " instead, ", Best,
           " m to the goal via ", BestPath);
    .length(Totals, HowMany);
    e11.event(hop_replaced, reroute, BestRobot, Best, HowMany, BestPath);
    .print("  chosen over ", HowMany, " remaining option(s): ", Totals).

-!chooseAnotherHop(_): true <-
    .print("Could not re-decide the route").

// Price an edge unless it is already priced. A node answers several arms across
// the same edges, and re-ranking a shared space for each of them costs seconds
// of the deadline for a number that cannot have changed.
// A price is only reusable for the question it answered. A leg measured from
// the cube and a leg measured from where the arm stands are different numbers,
// and handing one back for the other is how an arm that will never pick the
// cube up ends up quoting the cost of picking it up.
+!ensureEdgePriced(Robot, Mode): edgeCost(Robot, Mode, _, _) <- true.

+!ensureEdgePriced(Robot, Mode): true <-
    !estimateEdgeCost(Robot, Mode).

+!priceReachedEdges([]): true <- true.

+!priceReachedEdges([reached(_, Robot) | Rest]): true <-
    !ensureEdgePriced(Robot, pickup);
    !priceReachedEdges(Rest).

-!priceReachedEdges([_ | Rest]): true <-
    !priceReachedEdges(Rest).

// The reply is priced per upstream edge: each requester is a different
// rendezvous, so each gets a different number. Only the downstream part is
// shared between them.
+!answerUpstream(Task, Value, Path, Options)
    : agentIdentity(_, MyRobot)
<-
    .findall(U, routeRequestFrom(Task, U), Upstreams);
    .concat([MyRobot], Path, MyPath);
    // This arm joins every path it reports, not just the one it chose.
    !prefixOptions(Options, MyRobot, [], MyOptions);
    !answerEach(Upstreams, Task, Value, MyPath, MyOptions).

+!prefixOptions([], _, Done, Done): true <- true.

+!prefixOptions([option(Value, Path) | Rest], MyRobot, Acc, Out): true <-
    .concat([MyRobot], Path, MyPath);
    .concat(Acc, [option(Value, MyPath)], Next);
    !prefixOptions(Rest, MyRobot, Next, Out).

// The leg to this requester is the same for every route through me, so adding
// it to all of them leaves their order alone: the answer stays the cheapest.
+!priceOptions([], _, Done, Done): true <- true.

+!priceOptions([option(Value, Path) | Rest], Leg, Acc, Out): true <-
    Total = Leg + Value;
    .concat(Acc, [option(Total, Path)], Next);
    !priceOptions(Rest, Leg, Next, Out).

+!answerEach([], _, _, _, _): true <- true.

+!answerEach([Upstream | Rest], Task, none, Path, Options): true <-
    .send(Upstream, tell, noRoute(Task));
    !answerEach(Rest, Task, none, Path, Options).

+!answerEach([Upstream | Rest], Task, Value, Path, Options)
    : neighbor(Upstream, UpstreamRobot)
<-
    !ensureEdgePriced(UpstreamRobot, pose);
    ?edgeCost(UpstreamRobot, pose, Leg, _);
    Total = Leg + Value;
    !priceOptions(Options, Leg, [], Priced);
    .length(Priced, Spare);
    if (Spare > 0) {
        .print("Answering ", UpstreamRobot, ": ", Total, " m to the goal through me, and ",
               Spare, " dearer way(s) round  ", Priced)
    } else {
        .print("Answering ", UpstreamRobot, ": ", Total, " m to the goal through me")
    };
    .send(Upstream, tell, routeCost(Task, Priced, Total, Path));
    !answerEach(Rest, Task, Value, Path, Options).

// An edge I cannot price is an edge I cannot hand over across, so a route
// through me to that neighbour does not exist. Saying so is the truthful answer.
-!answerEach([Upstream | Rest], Task, Value, Path, Options): true <-
    .print("Cannot price the handover back to my requester; answering no route");
    .send(Upstream, tell, noRoute(Task));
    !answerEach(Rest, Task, Value, Path, Options).

// ---- starting the search at all --------------------------------------------

+!discoverRoute: hasSource(_, _) & goalSite(Goal) <-
    !findRouteTo(Goal).

// With no goal configured, one empty support in the cell is unambiguous and any
// other number is not. Guessing between two candidate destinations would make
// the "optimal route" claim meaningless, so it says what it sees and waits.
+!discoverRoute: hasSource(Source, Cube) & not goalSite(_) <-
    .findall(S, site(S, _, _, _) & siteEmpty(S), Empty);
    if (.length(Empty, 1)) {
        .nth(0, Empty, Goal);
        .print("Holding ", Cube, " at ", Source, "; the cell has exactly one empty support, ", Goal);
        !findRouteTo(Goal)
    } else {
        .print("Holding ", Cube, " at ", Source, " but ", Empty,
               " are empty; name the goal with goalSite/1 before a route can mean anything")
    }.

+!discoverRoute: not hasSource(_, _) <- true.

-!discoverRoute: true <-
    .print("Route discovery did not complete").

// ---------------------------------------------------------------------------
// Acting on the route.
// ---------------------------------------------------------------------------
//
// Discovering a route and travelling it are separate: the search settles in
// seconds and the transfer takes minutes of real motion, so the cell works the
// route out at startup and waits to be told to go. One call to whichever arm
// holds the cube is enough -- it knows its own next hop, and every arm the cube
// reaches knows its own, so nobody has to be handed an itinerary.

+!deliverCube
    : nextHop(_, NextAgent, NextRobot, Total)
      & agentIdentity(_, MyRobot)
<-
    e11.event(delivery_start, transport, MyRobot, NextRobot, Total);
    .print("Delivering the cube from ", MyRobot, " via ", NextRobot, "; the whole route costs ", Total);
    !handoverCubeTo(NextAgent, NextRobot).

// Already at the goal: nothing to hand over, just put it down.
+!deliverCube
    : hasSink(Goal) & agentIdentity(_, MyRobot) & cubeOwner(MyRobot)
<-
    .print("The goal ", Goal, " is already mine; placing the cube");
    !placeCubeAt(Goal).

+!deliverCube: noRouteTo(_) <-
    .print("No route to the goal was found, so there is nothing to deliver").

+!deliverCube: true <-
    .print("No route has been worked out yet; run !discoverRoute first").

-!deliverCube: true <-
    .print("The delivery did not complete").
