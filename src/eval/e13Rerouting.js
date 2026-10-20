"use strict";

// E13: the alternative route, measured two ways.
//
// E12 times a delivery that goes to plan. E13 times the one that does not: the
// cell settles on the cheap route through Robot4, Robot4's agent stops being
// there, and Robot2 -- which is holding the cube -- has to replace the hop it
// had already settled on with the second route the same search priced.
//
// Two arms, because "the route through Robot3 was taken" is the same sentence
// for two quite different runs and the thesis needs them apart:
//
//   cold   Robot4's agent is absent when Gradle starts. Its AgentThing is up,
//          so every arm still finds it by reach and still prices the edge to it,
//          but it cannot be asked: Jason fails `.send` to an agent that is not in
//          the MAS, and routePlans.asl records a "no" on its behalf. So the cheap
//          route is never discovered and the search still ends on a COMPLETE
//          TALLY, in about the time an undisturbed one takes -- the recorder names
//          the unaskable peer rather than counting a deadline. The delivery that
//          follows is an ordinary E12 delivery down the dearer route. This is what
//          knowing in advance costs. A `deadline` in this arm means something else
//          went wrong, which is why the two are still told apart.
//   kill   All five agents start, the search settles on the cheap route, and
//          Robot4's agent is killed after that. Discovery is fast, and the price
//          is paid later: one wasted approach to a rendezvous nobody comes to,
//          the detection deadline, a posture reset, and the dearer route from
//          Robot2 onwards. This is what not knowing in advance costs.
//
// The two arms share every other condition, so the pair of numbers is a straight
// comparison. Neither arm uses an obstacle: the failure is the agent's absence,
// which is the only part of the cell the experiment perturbs.
//
// Like E11 and E12 this observes. `watch` starts nothing, `launch` starts only
// Gradle, `deliver` sends only the !deliverCube the user would otherwise type.
// Killing Robot4's AgentThing is the user's own action and is recorded, when the
// user marks it, as an event in the trace -- this tool never kills anything.
const fs = require("fs");
const path = require("path");
const {
  ROOT, read, write, append, relative, quote, replaceOnce, readLines,
  csv, meanAndSampleSd, texNumber, runGradle, LOGGING
} = require("./evalTrace");
const {
  instrumentHandovers, instrumentRoutesWithLegs, instrumentAgent, verifyPlans,
  pairPhases, handovers, decompose, parseOptions, HANDOVER_PHASES, STARTUP_PHASES,
  CELL, agentName, names, manifests
} = require("./e12Principal");
const { parseArgs } = require("../generator/core/context");

const OUT = path.join(ROOT, "performance_log/e13");
// The arm whose agent fails. Robot4 is not a free choice: it is the arm on the
// cheap route that has an alternative at the same hop, which is what makes a
// reroute possible at all rather than a dead end.
const FAILED_ROBOT = "Robot4";
const FAILED_AGENT = agentName(FAILED_ROBOT);
const ARMS = ["cold", "kill"];
const PER_ARM = 5;
const RUNS = ARMS.length * PER_ARM;
// The shape of result.json. Every field `analyze` reads has to exist in every
// result it reads, and a trial recorded before a field was added does not have
// it -- so the version is stamped into the file and `results` rebuilds anything
// older from its own trace instead of handing `analyze` a result it cannot read.
const SCHEMA = 2;

const num = value => { const n = Number(String(value)); return Number.isFinite(n) ? n : null; };
const sameRobot = (a, b) => String(a).toLowerCase() === String(b).toLowerCase();
const ms = value => value === null || value === undefined ? "--" : (value / 1000).toFixed(1) + " s";

// ------------------------------------------------------------------ recovery
//
// Two recoveries decide the execution time of this cell and neither is a route
// decision, so neither reaches the event trace: they are postures, not hops.
// Both announce themselves with a fixed line in the MAS console, which the run
// directory keeps as gradle.log, so they are recorded data rather than something
// re-derived by hand -- and `recoveryMarkers.test.js` pins every marker below
// against the plan that prints it, so a reworded plan fails a test instead of
// silently counting zero.
//
//   rendezvous  A giver holding the cube cannot reach the handover point its own
//               ranking chose, because IK diverges from the posture it is
//               carrying in. It resets to the all-zero pose and asks the same
//               point again; if that fails too the point itself is refused and
//               the next-best candidate is taken.
//   placement   The sink cannot put the cube down from where it stands. Same
//               remedy, same two outcomes.
// [key, the console text, the file that produces it]. The third entry is what
// the marker test reads: a marker is only evidence while the thing that prints
// it still prints it.
const RECOVERY_MARKERS = [
  ["rendezvous_reset", "Could not reach the rendezvous from where I stand", "src/agt/handoverPlans.asl"],
  ["rendezvous_reset_failed", "The rendezvous is out of reach from the all-zero pose too", "src/agt/handoverPlans.asl"],
  ["rendezvous_refused", "refused by the cell", "src/agt/handoverPlans.asl"],
  ["placement_reset", "coming upright with it and asking the same point once more", "src/agt/handoverPlans.asl"],
  ["placement_reset_failed", "from upright either; it is the point, not the posture", "src/agt/handoverPlans.asl"],
  ["upright_with_cube", "Intermediate giver upright with cube", "src/agt/handoverPlans.asl"],
  // Not a plan at all: the arbitration refusing a motion because a neighbour
  // still holds the region. It reaches the console through the agent's own
  // "Move failed for ..." report, which carries the reason verbatim.
  ["region_claimed", "region_claimed", "src/generator/core/motionArbiter.js"]
];

// Counts, per agent and in total, of each recovery in one run's console log.
// A missing or unreadable log gives every count 0 rather than null, because the
// absence of a recovery and the absence of a log are told apart by `logged`.
function recoveries(log) {
  const totals = Object.fromEntries(RECOVERY_MARKERS.map(([key]) => [key, 0]));
  const byAgent = {};
  for (const line of String(log || "").split("\n")) {
    const who = line.match(/\[(\w+_agent)\]/);
    if (!who) continue;
    for (const [key, marker] of RECOVERY_MARKERS) {
      // "refused by the cell" is printed for a rendezvous and, in a different
      // plan, for a whole pair; only the rendezvous one is a recovery.
      if (key === "rendezvous_refused" && !/Rendezvous \S+ refused by the cell/.test(line)) continue;
      if (!line.includes(marker)) continue;
      totals[key]++;
      byAgent[who[1]] = byAgent[who[1]] || Object.fromEntries(RECOVERY_MARKERS.map(([k]) => [k, 0]));
      byAgent[who[1]][key]++;
    }
  }
  return { logged: Boolean(String(log || "").trim()), ...totals, by_agent: byAgent };
}

// ---------------------------------------------------------------- instrument

// E12's route instrumentation plus the two things E13 turns on: how the search
// ENDED, and what a refused hop is replaced with.
function instrumentRouteRecovery(source) {
  source = instrumentRoutesWithLegs(source);

  // Whether the search ended because every branch answered or because the clock
  // ran out, and which branch was still silent when it did. In the cold arm this
  // is the whole measurement: the absent agent is named here, by the arm that
  // waited for it, and the route-discovery duration is a deadline rather than a
  // convergence. Reporting one as the other is the mistake this event prevents.
  source = replaceOnce(source,
    '        .print("Deadline reached for ", Task, "; deciding on the answers that arrived");',
    "        .setof(A, routeAskedPeer(Task, A) & not routeAnswer(Task, A, _, _), Silent);\n"
    + "        e11.event(route_deadline, route_discovery, Task, Silent);\n"
    + '        .print("Deadline reached for ", Task, "; deciding on the answers that arrived");');

  // The hop goes. `Total` is what the cell believed that route was worth, so the
  // difference between it and the replacement is the price of the failure in
  // metres -- which is the quantity the thesis reports beside the seconds.
  // An arm nobody could put the question to. This is the cold arm's direct
  // observable -- the absent agent named by whoever tried to ask it -- and it is
  // the boundary the search now ends on instead of waiting out its deadline.
  source = replaceOnce(source, "    +routeUnreachable(Task, Agent);",
    "    e11.event(peer_unreachable, route_discovery, Agent, Task);\n"
    + "    +routeUnreachable(Task, Agent);");

  source = replaceOnce(source, "    !chooseAnotherHop(Task).",
    "    e11.event(hop_struck, reroute, FailedRobot, Total);\n"
    + "    !chooseAnotherHop(Task).");

  source = replaceOnce(source, "    .length(Totals, HowMany);",
    "    .length(Totals, HowMany);\n"
    + "    e11.event(hop_replaced, reroute, BestRobot, Best, HowMany, BestPath);");

  // Nothing left to fall back on. Recorded so that a run which stopped holding
  // the cube is distinguishable in the data from one that was never attempted.
  return replaceOnce(source,
    '    .print("No way on from ", MyRobot, " is left: every route the search found runs through an arm that cannot take the cube").',
    "    e11.event(reroute_exhausted, reroute, MyRobot);\n"
    + '    .print("No way on from ", MyRobot, " is left: every route the search found runs through an arm that cannot take the cube").');
}

// E12's handover instrumentation plus the failure and the recovery around it.
function instrumentPartnerFailure(source) {
  source = instrumentHandovers(source);

  // Detection, and what it was based on. An explicit refusal arrives in
  // milliseconds; silence costs the whole 120 s readiness deadline. Averaging
  // the two would describe neither, so the reason travels with the event and the
  // analysis reports them apart.
  source = replaceOnce(source, "        +partnerUnfit(ReceiverAgent, ReceiverReason);",
    "        e11.event(partner_unfit, reroute, ReceiverAgent, ReceiverReason, refused);\n"
    + "        +partnerUnfit(ReceiverAgent, ReceiverReason);");
  source = replaceOnce(source, "        +partnerUnfit(ReceiverAgent, silent)",
    "        e11.event(partner_unfit, reroute, ReceiverAgent, silent, timeout);\n"
    + "        +partnerUnfit(ReceiverAgent, silent)");

  // The local decision, on its own. Everything else in the recovery is motion or
  // teardown; this pair of boundaries is the claim -- the arm re-decides from
  // beliefs it already holds, so the interval should be milliseconds and the
  // message count across it should be nil.
  source = replaceOnce(source, "    !rerouteAround(ReceiverAgent, ReceiverRobot);",
    "    e11.event(phase_start, reroute_decide, ReceiverRobot);\n"
    + "    !rerouteAround(ReceiverAgent, ReceiverRobot);\n"
    + "    e11.event(phase_end, reroute_decide, ReceiverRobot);");

  // The physical price of the reroute: the arm is stretched out at a rendezvous
  // nobody came to and has to come back upright with the cube before it can plan
  // an approach to the next one. Eight spaces, so this is the reroute's own call
  // and not the one the ordinary chain makes between two hops.
  return replaceOnce(source, "        !prepareToForwardCube;",
    "        e11.event(phase_start, reroute_reset, ReceiverRobot);\n"
    + "        !prepareToForwardCube;\n"
    + "        e11.event(phase_end, reroute_reset, ReceiverRobot);");
}

// Two MAS declarations over one set of instrumented plans. The cold arm's is the
// same file list with `robot4_agent` left out -- which is the whole of the
// intervention, and why it can be read off the .jcm rather than trusted.
function preparePlans(out) {
  const directory = path.join(out, "plans");
  fs.mkdirSync(directory, { recursive: true });
  const route = path.join(directory, "routePlans.asl");
  const handover = path.join(directory, "handoverPlans.asl");
  fs.writeFileSync(route, instrumentRouteRecovery(fs.readFileSync(path.join(ROOT, "src/agt/routePlans.asl"), "utf8")));
  fs.writeFileSync(handover, instrumentPartnerFailure(fs.readFileSync(path.join(ROOT, "src/agt/handoverPlans.asl"), "utf8")));

  const blocks = [];
  for (const [index, file] of manifests().entries()) {
    const manifest = read(file), agent = `robot${index + 1}_agent`, workspace = `robot${index + 1}_workspace`;
    const dest = path.join(directory, agent + ".asl");
    fs.writeFileSync(dest, instrumentAgent(fs.readFileSync(path.join(ROOT, manifest.generatedFiles.agentSpeak), "utf8"), route, handover));
    if (/\s/.test(relative(dest))) throw Error("E13 paths must not contain whitespace (JaCaMo source syntax).");
    blocks.push({
      agent,
      agentBlock: `    agent ${agent}: ${relative(dest)} {\n        ag-arch: e11.MessageTrace\n        focus: ${workspace}.wotComm\n    }`,
      workspaceBlock: `    workspace ${workspace} {\n        artifact wotComm: wotCommunication.wotCommunicator("${manifest.configKey}")\n    }`
    });
  }
  const jcm = {};
  for (const arm of ARMS) {
    // The workspaces stay whole in both arms. Robot4's AgentThing is running in
    // the cold arm too, so every other arm still discovers it by reach and still
    // prices the edge to it -- only the reasoning agent is missing. Dropping the
    // workspace as well would make the cold arm a four-robot cell, which is a
    // different experiment.
    const kept = arm === "cold" ? blocks.filter(b => b.agent !== FAILED_AGENT) : blocks;
    const file = path.join(directory, `e13_${arm}.jcm`);
    fs.writeFileSync(file, `mas e13_${arm} {\n${kept.map(b => b.agentBlock).join("\n")}\n`
      + `${blocks.map(b => b.workspaceBlock).join("\n")}\n}\n`);
    jcm[arm] = file;
  }
  fs.writeFileSync(path.join(directory, "logging.properties"), LOGGING);
  return jcm;
}

// ------------------------------------------------------------------ analysis

// The route the cube actually travelled, read from the handovers that carried
// it rather than from the route the cell announced. In the kill arm those are
// two different paths, and the difference is the result.
function executedPath(hops) {
  const carried = hops.filter(h => !h.abandoned && h.duration_ms !== null);
  if (!carried.length) return [];
  return [carried[0].giver, ...carried.map(h => h.receiver)];
}

// A handover interval that opened, moved the giver, and was then abandoned
// because the arm at the other end could not take the cube. E12 marks abandoned
// intervals from `candidate_refused`, which is a refused POINT; this is a
// refused PARTNER, and it has to be marked too or the interval that never closed
// is read as a handover that failed and the run is scored incomplete.
function markPartnerFailures(hops, events) {
  for (const event of events) {
    if (event.event !== "partner_unfit") continue;
    const receiver = String(event.data?.[0] ?? "").replace(/_agent$/, "");
    const attempt = hops.filter(h => h.giver_agent === event.agent && sameRobot(h.receiver, receiver)
      && h.end_mono === null && h.start_mono <= event.mono_ms).at(-1);
    if (!attempt) continue;
    attempt.abandoned = true;
    attempt.abandoned_reason = "partner_unfit";
    attempt.detected_mono = event.mono_ms;
  }
  // E12 numbers the carried hops as it builds them, so anything marked here has
  // to be taken back out of that numbering.
  let carried = 0;
  for (const hop of hops) hop.hop = hop.abandoned ? null : ++carried;
  return hops;
}

// Everything the failure cost, as one row per reroute. A run has at most one in
// this cell, but the shape does not assume it.
function reroutes(events, hops, meta) {
  const decide = pairPhases(events, "reroute_decide");
  const reset = pairPhases(events, "reroute_reset");
  const struck = events.filter(e => e.event === "hop_struck");
  const replaced = events.filter(e => e.event === "hop_replaced");
  const exhausted = events.filter(e => e.event === "reroute_exhausted");

  return events.filter(e => e.event === "partner_unfit").map((detection, index) => {
    const at = detection.agent;
    const failed = String(detection.data?.[0] ?? "").replace(/_agent$/, "");
    const after = list => list.filter(e => e.agent === at && e.mono_ms >= detection.mono_ms)[0] ?? null;
    const afterPhase = list => list.filter(p => p.agent === at && p.start_mono >= detection.mono_ms)[0] ?? null;
    const strike = after(struck), swap = after(replaced), give_up = after(exhausted);
    const decision = afterPhase(decide), posture = afterPhase(reset);
    // The attempt this failure ended: the abandoned interval whose giver is the
    // arm that detected it and whose receiver is the arm that failed.
    const attempt = hops.filter(h => h.giver_agent === at && sameRobot(h.receiver, failed)
      && h.start_mono <= detection.mono_ms).at(-1) ?? null;
    // The replacement: the first hop this arm carried to anybody else afterwards.
    const replacement = hops.find(h => !h.abandoned && h.giver_agent === at
      && h.start_mono >= detection.mono_ms && !sameRobot(h.receiver, failed)) ?? null;

    const struck_m = strike ? num(strike.data?.[1]) : null;
    const replaced_m = swap ? num(swap.data?.[1]) : null;
    return {
      run: meta.run, arm: meta.arm, index: index + 1,
      at_agent: at, at_robot: String(at).replace(/_agent$/, ""), failed_robot: failed,
      // How the failure announced itself: an explicit refusal, or silence that
      // cost the whole readiness deadline. These are different experiments
      // sharing a row, so the reason is never averaged away.
      reason: detection.data?.[1] ?? null, detected_by: detection.data?.[2] ?? null,
      // Giver's first commanded motion towards the doomed rendezvous, to the
      // moment the arm concluded the partner was not coming.
      detect_ms: attempt && attempt.start_mono !== null ? detection.mono_ms - attempt.start_mono : null,
      wasted_approach_ms: attempt?.giver_approach_ms ?? null,
      // The claim: a local re-decision over answers already held.
      decide_ms: decision?.duration_ms ?? null,
      reset_ms: posture?.duration_ms ?? null,
      // Detection to the giver's first motion of the replacement handover: the
      // whole gap the failure opened in the delivery.
      recover_ms: replacement ? replacement.start_mono - detection.mono_ms : null,
      struck_robot: strike ? strike.data?.[0] ?? null : null,
      struck_total_m: struck_m,
      replacement_robot: swap ? swap.data?.[0] ?? null : null,
      replacement_total_m: replaced_m,
      // What the failure cost in metres of Eq. (route_cost), from the arm that
      // re-decided: the difference between the two totals it had collected.
      penalty_m: struck_m !== null && replaced_m !== null ? replaced_m - struck_m : null,
      options_left: swap ? num(swap.data?.[2]) : null,
      replacement_path: swap ? String(swap.data?.[3] ?? "") : null,
      exhausted: Boolean(give_up),
      detected_mono: detection.mono_ms,
      decided_mono: decision?.end_mono ?? null,
      // Messages sent while the arm was re-deciding, which is the falsifiable
      // form of "a reroute costs no messages": the alternatives were priced by
      // the search that chose the route and came home inside the same replies,
      // so nothing should cross this boundary at all.
      messages_during: decision && decision.end_mono !== null
        ? events.filter(e => e.event === "message"
            && e.mono_ms >= decision.start_mono && e.mono_ms <= decision.end_mono).length
        : null,
      // Counted separately, because it is not part of the decision and is not
      // nil: the arm being left behind is told the handover is off, and the arm
      // being handed to instead is told which arm is out. Two tells that ride on
      // a handover that was happening anyway, and no question to anybody.
      messages_teardown: decision
        ? events.filter(e => e.event === "message"
            && e.mono_ms >= detection.mono_ms && e.mono_ms < decision.start_mono).length
        : null
    };
  });
}

function summarize(meta, events, reason = null, log = "") {
  const running = names(CELL).map(n => agentName(n)).filter(a => meta.arm !== "cold" || a !== FAILED_AGENT);
  const issues = [];

  const durations = {};
  for (const phase of STARTUP_PHASES) {
    const rows = pairPhases(events, phase).filter(p => running.includes(p.agent));
    for (const agent of running) if (!rows.some(r => r.agent === agent && r.duration_ms !== null)) issues.push(`Missing ${phase} boundary for ${agent}`);
    durations[phase] = rows.length && rows.every(p => p.duration_ms !== null)
      ? Math.max(...rows.map(p => p.end_mono)) - Math.min(...rows.map(p => p.start_mono)) : null;
  }
  const routeStart = events.find(e => e.event === "phase_start" && e.phase === "route_discovery");
  const routeEnd = events.find(e => e.event === "route_end");
  durations.route_discovery = routeStart && routeEnd ? routeEnd.mono_ms - routeStart.mono_ms : null;
  if (!routeEnd) issues.push("No route decision was recorded");

  // How the search ended, which is the cold arm's result. A deadline at the
  // origin bounds the whole discovery; a deadline at a carrier bounds its branch.
  const deadlines = events.filter(e => e.event === "route_deadline").map(e => ({
    agent: e.agent, task: e.data?.[0] ?? null,
    silent: String(e.data?.[1] ?? "").replace(/^\[|\]$/g, "").split(",").map(s => s.trim()).filter(Boolean),
    mono_ms: e.mono_ms
  }));
  // Arms that could not be asked at all, named by whoever tried. An agent that
  // is not in the MAS fails the sender's .send, and the sender answers on its
  // behalf so the arms further down its own list still get asked -- so this is
  // the cold arm's intervention, observed rather than assumed.
  const unaskable = [...new Set(events.filter(e => e.event === "peer_unreachable").map(e => String(e.data?.[0] ?? "")))];
  // Which arms still priced an edge to the failed robot. It says whether the
  // robot was present in the cell as a physical partner while its agent was
  // missing: with its AgentThing up the edge is priced and the cost table is
  // complete, with it down nobody can build the shared space and the run pays
  // each arm's estimator timeout in startup instead.
  const pricedFailed = [...new Set(events.filter(e =>
    (e.event === "edge_priced" || e.event === "edge_refused") && sameRobot(e.data?.[0], FAILED_ROBOT)).map(e => e.agent))];
  const convergence = {
    ended_on: deadlines.length ? "deadline" : routeEnd ? "complete_tally" : "missing",
    deadlines: deadlines.length,
    unaskable_peers: unaskable,
    failed_robot_priced_by: pricedFailed,
    // Named branches that had not answered when the clock ran out, across every
    // arm that gave up waiting. In the cold arm this is the absent agent.
    silent_branches: [...new Set(deadlines.flatMap(d => d.silent))],
    // Time from the first arm starting to reason to the route being settled,
    // which is what the two arms are compared on.
    settled_after_gradle_ms: routeEnd ? routeEnd.t - meta.t : null,
    messages_to_settle: routeEnd
      ? events.filter(e => e.event === "message" && e.mono_ms <= routeEnd.mono_ms).length : null
  };

  const legs = events.filter(e => e.event === "edge_priced").map(e => ({
    agent: e.agent, neighbour: e.data?.[0], mode: e.data?.[1], distance_m: num(e.data?.[2]), candidate: e.data?.[3] }));
  const refusedEdges = events.filter(e => e.event === "edge_refused").map(e => ({
    agent: e.agent, neighbour: e.data?.[0], mode: e.data?.[1], reason: e.data?.[2] }));
  const refusedCandidates = events.filter(e => e.event === "candidate_refused").map(e => ({
    agent: e.agent, candidate: e.data?.[0], receiver: e.data?.[1] }));

  const optionEvent = events.find(e => e.event === "route_options");
  const planned = {
    outcome: routeEnd?.data?.[0] || "missing", origin: routeEnd?.agent ?? null,
    cost_m: num(routeEnd?.data?.[2]),
    path: String(routeEnd?.data?.[3] ?? "").replace(/^\[|\]$/g, "").split(",").map(s => s.trim()).filter(Boolean),
    options_reported: optionEvent ? num(optionEvent.data?.[1]) : null,
    options: String(optionEvent?.data?.[2] ?? "")
  };

  const hops = markPartnerFailures(handovers(events, meta), events);
  const carried = hops.filter(h => !h.abandoned);
  const detours = reroutes(events, hops, meta);
  const executed = executedPath(hops);

  const placed = events.find(e => e.event === "cube_placed");
  // First attempt of the last carrier to the last completion of it. `null` when
  // the cube never got that far, so a failed run cannot pretend to a hop time.
  const lastHopSpan = (sessions, sink) => {
    if (!sink || executed.length < 2) return null;
    const mine = sessions.filter(h => sameRobot(h.giver, executed.at(-2)) && sameRobot(h.receiver, sink));
    const first = mine.map(h => h.start_mono).filter(v => v != null);
    const last = mine.map(h => h.end_mono).filter(v => v != null);
    return first.length && last.length ? Math.max(...last) - Math.min(...first) : null;
  };
  const deliveryStart = events.find(e => e.event === "delivery_start");
  const sourcePickup = pairPhases(events, "pickup").find(p => p.agent === agentName(planned.path[0] || "Robot1"));
  const placements = pairPhases(events, "placement");
  const transport = {
    delivery_requested_ms: deliveryStart ? deliveryStart.t - meta.t : null,
    delivery_ms: deliveryStart && placed ? placed.mono_ms - deliveryStart.mono_ms : null,
    source_pickup_ms: sourcePickup?.duration_ms ?? null,
    placement_ms: placements[0]?.duration_ms ?? null,
    // The wall clock of the hop into the sink, counted from the first attempt
    // rather than the successful one. A giver that cannot reach the point its own
    // ranking chose retries -- from the all-zero pose, and then at the next-best
    // point -- and each retry opens a new session, so the successful session's
    // duration alone understates the hop exactly when it went worst.
    last_hop_ms: lastHopSpan(hops, executed.at(-1)),
    last_hop_sessions: executed.length ? hops.filter(h => sameRobot(h.giver, executed.at(-2))).length : null,
    handover_total_ms: carried.length && carried.every(h => h.duration_ms !== null)
      ? carried.reduce((sum, h) => sum + h.duration_ms, 0) : null,
    // Motion the cell performed and then threw away: the approach to a
    // rendezvous nobody came to, plus the reset that undid it. Nil in the cold
    // arm by construction, which is the point of having the cold arm.
    wasted_ms: detours.reduce((sum, r) => sum + (r.wasted_approach_ms ?? 0) + (r.reset_ms ?? 0), 0) || null,
    sink: placed?.data?.[1] ?? null, goal: placed?.data?.[0] ?? null,
    start_mono: deliveryStart?.mono_ms ?? null, placed_mono: placed?.mono_ms ?? null,
    // Gradle start to the cube on support B. Kept because it is the raw fact,
    // but it contains the operator's own pause between the route settling and
    // !deliverCube -- and in the kill arm that pause is where the AgentThing is
    // stopped, so it is longer on purpose. Never compare the arms on it.
    end_to_end_ms: placed ? placed.t - meta.t : null,
    // What the two arms ARE compared on: the machine's own time, discovery plus
    // delivery, with no human in it. Both halves of the trade are inside it --
    // the cold arm pays in discovery, the kill arm pays inside the delivery --
    // and it is a sum of two monotonic-clock intervals from the one JVM.
    machine_total_ms: null
  };
  if (durations.route_discovery !== null && transport.delivery_ms !== null) {
    transport.machine_total_ms = durations.route_discovery + transport.delivery_ms;
  }

  const executedCost = decompose(executed, legs);
  const route = {
    ...planned, executed_path: executed, executed_cost_m: executedCost.total_m,
    rerouted: detours.length > 0,
    // What the failure cost in metres, measured on the route actually walked
    // against the route the cell had announced.
    penalty_m: planned.cost_m !== null && executedCost.total_m !== null ? executedCost.total_m - planned.cost_m : null
  };

  const ready = new Set(events.filter(e => e.event === "agent_ready").map(e => e.agent));
  if (ready.size !== running.length) issues.push(`Only ${ready.size}/${running.length} expected agents initialized`);
  if (!placed) issues.push("The cube was never reported placed on the target support");
  for (const hop of carried) if (hop.duration_ms === null) issues.push(`Handover ${hop.giver}->${hop.receiver} never completed`);
  if (placed && executed.length && carried.length !== executed.length - 1) {
    issues.push(`${carried.length} handover(s) carried the cube but the executed route has ${executed.length - 1} hop(s)`);
  }
  // Arm-specific expectations. A run that did not test what its arm exists to
  // test is worse than a missing run, because it averages in as if it had.
  if (executed.length && executed.some(r => sameRobot(r, FAILED_ROBOT))) {
    issues.push(`The cube travelled through ${FAILED_ROBOT}, whose agent was supposed to be unavailable`);
  }
  if (meta.arm === "cold") {
    if (ready.has(FAILED_AGENT)) issues.push(`${FAILED_AGENT} reported ready in a cold run`);
    // An absent agent has to show up as an arm somebody could not ask. If it
    // does not, the MAS that ran still had it, whatever the label on the run.
    if (!unaskable.includes(FAILED_AGENT)) {
      issues.push(`No arm reported being unable to ask ${FAILED_AGENT}; this MAS may still have had it`);
    }
    if (detours.length) issues.push("A cold run rerouted; the cheap route should never have been settled on");
    if (planned.path.some(r => sameRobot(r, FAILED_ROBOT))) issues.push(`A cold run settled on a route through ${FAILED_ROBOT}`);
  }
  if (meta.arm === "kill") {
    if (!planned.path.some(r => sameRobot(r, FAILED_ROBOT))) {
      issues.push(`A kill run settled on a route avoiding ${FAILED_ROBOT}; the agent was killed too early`);
    }
    if (!detours.length) issues.push("A kill run recorded no reroute");
    if (detours.some(r => r.exhausted)) issues.push("The reroute ran out of alternatives");
  }

  return {
    schema: SCHEMA,
    run: meta.run, arm: meta.arm,
    status: issues.length || reason ? "incomplete" : planned.outcome === "no_route" ? "no_route" : "ok",
    reason, issues, started_at: new Date(meta.t).toISOString(), elapsed_ms: (events.at(-1)?.t ?? meta.t) - meta.t,
    durations_ms: durations, convergence, route, reroutes: detours, recovery: recoveries(log),
    legs, refused_edges: refusedEdges, refused_candidates: refusedCandidates,
    handovers: hops, transport,
    agent_killed_at_ms: (events.find(e => e.event === "agent_killed")?.t ?? null) === null
      ? null : events.find(e => e.event === "agent_killed").t - meta.t,
    messages_total: events.filter(e => e.event === "message" && running.includes(e.agent)).length
  };
}

// ---------------------------------------------------------------- recording

// The MAS console of one run, as the launch left it. Absent while the run is
// still going, which is why every recovery count is 0 rather than null then.
function readLog(directory) {
  const file = path.join(directory, "gradle.log");
  return fs.existsSync(file) ? fs.readFileSync(file, "utf8") : "";
}

function finishTrial(directory, reason, out) {
  if (fs.existsSync(path.join(directory, "result.json"))) return;
  const meta = read(path.join(directory, "run.json"));
  const events = readLines(path.join(directory, "events.jsonl"));
  const result = summarize(meta, events, reason, readLog(directory));
  fs.writeFileSync(path.join(directory, "measurement-events.jsonl"), events.map(e => JSON.stringify(e) + "\n").join(""));
  fs.writeFileSync(path.join(directory, "messages.csv"), csv(events.filter(e => e.event === "message").map(e => ({
    since_gradle_ms: e.t - meta.t, sender: e.agent, receiver: e.receiver, performative: e.performative,
    content: e.content, message_id: e.message_id }))));
  write(path.join(directory, "result.json"), result);
  const detour = result.reroutes[0];
  console.log(`Run ${meta.run} [${meta.arm}]: ${result.status}; discovery ${result.durations_ms.route_discovery ?? "--"} ms `
    + `(${result.convergence.ended_on}${result.convergence.silent_branches.length ? ", silent " + result.convergence.silent_branches.join("/") : ""}); `
    + `planned ${result.route.path.join("-") || "--"} at ${result.route.cost_m?.toFixed(3) ?? "--"} m; `
    + `executed ${result.route.executed_path.join("-") || "--"} at ${result.route.executed_cost_m?.toFixed(3) ?? "--"} m; `
    + (detour ? `reroute at ${detour.at_robot}: detect ${ms(detour.detect_ms)} (${detour.detected_by}), decide ${detour.decide_ms ?? "--"} ms, `
      + `reset ${ms(detour.reset_ms)}, ${detour.messages_during ?? "--"} message(s); ` : "no reroute; ")
    + `delivery ${ms(result.transport.delivery_ms)}`);
  if (result.issues.length) console.log("  issues: " + result.issues.join("; "));
  // The trial is recorded and reported by this point, and the run is what the
  // user paid for in simulation time. Rebuilding the experiment-wide tables is
  // a convenience on top of it, so a failure there costs the tables and says so
  // -- it does not cost the run, and it does not stop the recorder waiting for
  // the next one.
  try {
    analyze(out);
  } catch (error) {
    console.error(`Run ${meta.run} is recorded, but the combined tables could not be rebuilt: ${error.message}`);
    console.error("  Fix the cause and run: node src/eval/e13Rerouting.js analyze "
      + `--out ${quote(relative(out))}`);
  }
}

function resummarize(out, runs) {
  let rebuilt = 0;
  for (let run = 1; run <= runs; run++) {
    const directory = path.join(out, `run${run}`);
    const meta = path.join(directory, "run.json"), previous = path.join(directory, "result.json");
    if (!fs.existsSync(meta) || !fs.existsSync(previous)) continue;
    const reason = read(previous).reason ?? null;
    write(previous, summarize(read(meta), readLines(path.join(directory, "events.jsonl")), reason, readLog(directory)));
    rebuilt++;
  }
  console.log(`Rebuilt ${rebuilt} result file(s) from their traces.`);
  return rebuilt;
}

// Every result `analyze` will read, brought up to the current schema first. A
// trial keeps its whole trace, so a result written by an older version of this
// recorder is not lost data -- it is a summary that can be computed again. One
// that cannot be rebuilt is reported and left out, because an experiment that
// still has runs to record must not be stopped by a summary from a finished one.
function results(out, runs) {
  const rows = [];
  for (let run = 1; run <= runs; run++) {
    const directory = path.join(out, `run${run}`);
    const file = path.join(directory, "result.json");
    if (!fs.existsSync(file)) continue;
    const previous = read(file);
    if (previous.schema === SCHEMA) { rows.push(previous); continue; }
    const meta = path.join(directory, "run.json"), trace = path.join(directory, "events.jsonl");
    if (!fs.existsSync(meta) || !fs.existsSync(trace)) {
      console.warn(`Run ${run} was recorded by an older version of E13 and its trace is gone; `
        + "leaving it out of the analysis.");
      continue;
    }
    const rebuilt = summarize(read(meta), readLines(trace), previous.reason ?? null, readLog(directory));
    write(file, rebuilt);
    console.warn(`Run ${run} was recorded by an older version of E13; rebuilt its summary from its trace.`);
    rows.push(rebuilt);
  }
  return rows;
}

// -------------------------------------------------------- thesis artifacts

const ROW = String.raw`\\`;
const texRobot = text => String(text).replace(/robot\s*(\d+)/gi, "Robot~$1");
const pm = (stat, digits = 2) => stat.mean === null ? "--"
  : stat.sd === null ? texNumber(stat.mean, digits) : `${texNumber(stat.mean, digits)} \\pm ${texNumber(stat.sd, digits)}`;
const stat = values => meanAndSampleSd(values.filter(v => v != null));

// The comparison the experiment exists to make, one column per arm. Every row is
// a quantity that differs between them for a reason, and the two halves of the
// trade are the route-discovery row and the wasted-motion row.
function armTable(valid) {
  const arms = ARMS.filter(arm => valid.some(r => r.arm === arm));
  const of = arm => valid.filter(r => r.arm === arm);
  const rows = [
    ["Runs", arm => String(of(arm).length)],
    ["Route discovery [s]", arm => `$${pm(scaled(of(arm).map(r => r.durations_ms.route_discovery)))}$`],
    ["Ended on deadline", arm => `${of(arm).filter(r => r.convergence.ended_on === "deadline").length}/${of(arm).length}`],
    ["Arms that could not be asked", arm => {
      const counts = of(arm).map(r => r.convergence.unaskable_peers.length);
      return counts.length && counts.every(c => c === counts[0]) ? String(counts[0]) : `$${pm(stat(counts), 1)}$`;
    }],
    ["Messages to settle", arm => `$${pm(stat(of(arm).map(r => r.convergence.messages_to_settle)), 0)}$`],
    ["Planned route cost [m]", arm => `$${pm(stat(of(arm).map(r => r.route.cost_m)), 3)}$`],
    ["Executed route cost [m]", arm => `$${pm(stat(of(arm).map(r => r.route.executed_cost_m)), 3)}$`],
    ["Reroutes", arm => `${of(arm).filter(r => r.route.rerouted).length}/${of(arm).length}`],
    ["Detection, silence [s]", arm => detect(of(arm), "timeout")],
    ["Detection, refusal [s]", arm => detect(of(arm), "refused")],
    ["Reroute decision [ms]", arm => `$${pm(stat(of(arm).flatMap(r => r.reroutes.map(d => d.decide_ms))), 0)}$`],
    ["Messages to re-decide", arm => `$${pm(stat(of(arm).flatMap(r => r.reroutes.map(d => d.messages_during))), 1)}$`],
    ["Discarded motion [s]", arm => `$${pm(scaled(of(arm).map(r => r.transport.wasted_ms)))}$`],
    ["Delivery [s]", arm => `$${pm(scaled(of(arm).map(r => r.transport.delivery_ms)))}$`],
    ["Discovery + delivery [s]", arm => `$${pm(scaled(of(arm).map(r => r.transport.machine_total_ms)))}$`]
  ];
  return [
    `% Generated from the E13 result.json files by src/eval/e13Rerouting.js (${valid.length} successful runs).`,
    `\\begin{tabular}{@{}l${"r".repeat(arms.length)}@{}}`, "\\toprule",
    `        \\textbf{Quantity} & ${arms.map(a => `\\textbf{${a === "cold" ? "Agent absent" : "Agent killed"}}`).join(" & ")} ${ROW}`,
    "\\midrule",
    ...rows.map(([label, cell]) => `        ${label} & ${arms.map(cell).join(" & ")} ${ROW}`),
    "\\bottomrule", "\\end{tabular}"
  ].join("\n") + "\n";
}

// Detection times for one way of finding out, with how many runs found out that
// way. Reported apart because they are two different quantities: an arm that
// answers "I cannot" is a message hop, and an arm that says nothing is the whole
// 120-second readiness deadline. A single mean over both is a number that
// describes no run in the table.
function detect(rows, by) {
  const values = rows.flatMap(r => r.reroutes.filter(d => d.detected_by === by).map(d => d.detect_ms));
  if (!values.filter(v => v != null).length) return "$--$";
  return `$${pm(scaled(values))}$ (${values.filter(v => v != null).length})`;
}

function scaled(values) {
  const s = stat(values);
  return { mean: s.mean === null ? null : s.mean / 1000, sd: s.sd === null ? null : s.sd / 1000 };
}

// The reroute itself, broken into the parts that cost seconds and the part that
// is the contribution. Reading this table is how anybody sees that the decision
// is free and everything expensive about a failure is physical.
function rerouteTable(valid) {
  const detours = valid.flatMap(r => r.reroutes.map(d => ({ ...d, run: r.run })));
  if (!detours.length) return "% No E13 run recorded a reroute.\n";
  const byReason = [...new Set(detours.map(d => d.detected_by))];
  const lines = byReason.map(reason => {
    const rows = detours.filter(d => d.detected_by === reason);
    const cells = [
      pm(scaled(rows.map(d => d.detect_ms))),
      pm(scaled(rows.map(d => d.wasted_approach_ms))),
      pm(stat(rows.map(d => d.decide_ms)), 0),
      pm(scaled(rows.map(d => d.reset_ms))),
      pm(scaled(rows.map(d => d.recover_ms)))
    ];
    return `        ${reason === "timeout" ? "Silence (readiness deadline)" : "Explicit refusal"} & `
      + cells.map(c => `$${c}$`).join(" & ") + ` & ${rows.length} ${ROW}`;
  });
  const penalty = stat(detours.map(d => d.penalty_m));
  return [
    "% Generated from the E13 result.json files by src/eval/e13Rerouting.js.",
    "\\begin{tabular}{@{}lrrrrrr@{}}", "\\toprule",
    `        \\textbf{Detected by} & \\textbf{Detect [s]} & \\textbf{Wasted approach [s]} `
      + `& \\textbf{Decide [ms]} & \\textbf{Reset [s]} & \\textbf{Recover [s]} & \\textbf{n} ${ROW}`,
    "\\midrule", ...lines, "\\midrule",
    `        \\multicolumn{6}{@{}l}{Route cost penalty of the replacement: $${pm(penalty, 3)}$ m} & ${detours.length} ${ROW}`,
    "\\bottomrule", "\\end{tabular}"
  ].join("\n") + "\n";
}

// Both routes of the cell, priced from a run that measured them, with the one
// each arm executed marked. The same decomposition E12 uses, so the two
// experiments' cost tables are the same table with a different row marked.
function costTable(valid) {
  const reference = valid.find(r => r.route.options && parseOptions(r.route.options).length > 1) || valid[0];
  if (!reference) return "% No E13 run to decompose.\n";
  const options = parseOptions(reference.route.options);
  const paths = options.length ? options.map(o => [reference.route.path[0], ...o.path]) : [reference.route.path];
  const columns = paths.map(p => ({ path: p, ...decompose(p, reference.legs) }));
  const common = paths.reduce((keep, p) => keep.filter(r => p.some(x => sameRobot(x, r))), [...paths[0]]);
  const labels = columns.map(c => {
    const distinct = c.path.slice(1, -1).filter(r => !common.some(x => sameRobot(x, r)));
    return "Via " + (distinct.length ? distinct : c.path.slice(1, -1)).map(texRobot).join(", ") + " [m]";
  });
  const executedByArm = Object.fromEntries(ARMS.map(arm => {
    const run = valid.find(r => r.arm === arm && r.route.executed_path.length);
    return [arm, run ? run.route.executed_path.join(">") : null];
  }));
  const seen = [];
  for (const column of columns) for (const row of column.rows) if (!seen.some(s => s.leg === row.leg)) seen.push(row);
  const cell = (column, leg) => {
    const hit = column.rows.find(r => r.leg === leg);
    return hit && hit.distance_m != null ? texNumber(hit.distance_m, 3) : "";
  };
  const executedMark = column => ARMS
    .filter(arm => executedByArm[arm] === column.path.join(">"))
    .map(arm => arm === "cold" ? "absent" : "killed");
  return [
    "% Generated from the E13 result.json files by src/eval/e13Rerouting.js.",
    `\\begin{tabular}{@{}l${"r".repeat(columns.length)}@{}}`, "\\toprule",
    `        \\textbf{Leg} & ${labels.map(l => "\\textbf{" + l + "}").join(" & ")} ${ROW}`,
    "\\midrule",
    ...seen.map(row => "        " + texRobot(row.leg) + " & " + columns.map(c => cell(c, row.leg)).join(" & ") + ` ${ROW}`),
    "\\midrule",
    `        Cumulative cost & ${columns.map(c => c.total_m == null ? "--" : texNumber(c.total_m, 3)).join(" & ")} ${ROW}`,
    `        Executed in & ${columns.map(c => { const m = executedMark(c); return m.length ? m.join(", ") : "--"; }).join(" & ")} ${ROW}`,
    "\\bottomrule", "\\end{tabular}"
  ].join("\n") + "\n";
}

// Where each arm's time goes, as two stacked bars. The shapes are the argument:
// the cold arm's time is nearly all discovery, the kill arm's is nearly all
// motion it had to redo, and the totals are close enough that neither strategy
// is obviously the cheaper one.
// The two postural recoveries of the execution, each as its outcome against the
// time it cost. Neither is a routing decision and neither depends on the arm --
// which is the point of the table: the tail of this cell's delivery is not where
// the experiment perturbs it.
// Legend entries laid out left to right, each advancing by its own width rather
// than by a fixed step -- a fixed step collides as soon as one label is longer
// than the gap, which "Discarded motion" is -- and wrapping to a second row when
// the next entry would run past the plot. The width per character is measured
// against \scriptsize at the scale these pictures are set at; it only has to be
// close enough to keep the labels apart.
function legendRow(entries, y, width = 12) {
  const widthOf = label => 0.45 + label.length * 0.17 + 0.5;
  const lines = [];
  let x = 0, row = 0;
  for (const { style, label } of entries) {
    if (x > 0 && x + widthOf(label) > width) { x = 0; row++; }
    lines.push(`\\fill[${style}] (${texNumber(x, 3)},${texNumber(y - row * 0.45, 3)}) rectangle ++(0.28,0.28) `
      + `node[right,font=\\scriptsize,anchor=west,text=black,xshift=0.32cm,yshift=-0.14cm] {${label}};`);
    x += widthOf(label);
  }
  return lines;
}

function recoveryTable(valid) {
  const done = valid.filter(r => r.recovery?.logged);
  if (!done.length) return "% No E13 run kept its MAS console, so no recovery can be counted.\n";

  // Grouped by what actually happened, not by a count, because "reset once and
  // then reached it" and "reset once, gave the point up" cost different things.
  const rendezvous = [
    ["Reached the first point directly", r => !r.recovery.rendezvous_reset],
    ["Reset to the all-zero pose, then reached it",
      r => r.recovery.rendezvous_reset > 0 && !r.recovery.rendezvous_refused],
    ["Gave the point up, took the next candidate", r => r.recovery.rendezvous_refused > 0]
  ];
  const placement = [
    ["Placed directly", r => !r.recovery.placement_reset],
    ["One recovery", r => r.recovery.placement_reset === 1],
    ["Two recoveries", r => r.recovery.placement_reset >= 2]
  ];
  const band = (label, rows, of) => {
    const s = stat(rows.map(of).map(v => v == null ? null : v / 1000));
    const arms = ARMS.filter(a => rows.some(r => r.arm === a));
    return `        ${label} & ${rows.length} & $${pm(s, 1)}$ & `
      + `${arms.length === ARMS.length ? "both" : arms.length ? (arms[0] === "cold" ? "absent" : "killed") : "--"} ${ROW}`;
  };
  const section = (title, groups, of, unit) => [
    `        \\multicolumn{4}{@{}l}{\\emph{${title}}} ${ROW}`,
    ...groups.map(([label, test]) => {
      const rows = done.filter(test);
      return rows.length ? band(label, rows, of) : `        ${label} & 0 & $--$ & -- ${ROW}`;
    })
  ];
  return [
    "% Generated from the E13 result.json files by src/eval/e13Rerouting.js.",
    "\\begin{tabular}{@{}lrrl@{}}",
    "\\toprule",
    `        \\textbf{Outcome} & \\textbf{n} & \\textbf{Duration [s]} & \\textbf{Arm} ${ROW}`,
    "\\midrule",
    ...section(`Hop into the sink, ${texRobot("robot3")} $\\rightarrow$ ${texRobot("robot5")}`,
      rendezvous, r => r.transport.last_hop_ms),
    "\\midrule",
    ...section("Placement on support B", placement, r => r.transport.placement_ms),
    "\\bottomrule",
    "\\end{tabular}"
  ].join("\n") + "\n";
}

// Where the delivery of each run actually went. One bar per run in the order
// they were flown, so the two arms are side by side and the placement tail is
// visible as what it is: three runs, in both arms, unrelated to the reroute.
function deliveryFigure(valid) {
  const rows = valid.filter(r => r.transport.delivery_ms != null)
    .sort((a, b) => a.run - b.run);
  if (!rows.length) return "% No E13 run with a complete delivery.\n";
  const segments = [
    { key: "Pickup", fill: "gray!25", draw: "gray!60", of: r => r.transport.source_pickup_ms },
    { key: "Discarded motion", fill: "orange!40", draw: "orange!85!black", of: r => r.transport.wasted_ms },
    { key: "Handovers", fill: "green!30", draw: "green!50!black", of: r => r.transport.handover_total_ms },
    { key: "Placement", fill: "red!30", draw: "red!65", of: r => r.transport.placement_ms }
  ];
  const span = Math.max(1, ...rows.map(r => r.transport.delivery_ms / 1000));
  const scale = 11 / span;
  const x = seconds => texNumber(seconds * scale, 3);
  const lines = [];
  rows.forEach((r, i) => {
    const y = -i * 0.58;
    lines.push(`\\node[anchor=east,font=\\scriptsize] at (-0.12,${texNumber(y, 3)}) `
      + `{${r.run} (${r.arm === "cold" ? "absent" : "killed"})};`);
    let cursor = 0;
    for (const segment of segments) {
      const seconds = (segment.of(r) ?? 0) / 1000;
      if (seconds <= 0) continue;
      lines.push(`\\fill[${segment.fill},draw=${segment.draw}] (${x(cursor)},${texNumber(y - 0.2, 3)}) `
        + `rectangle (${x(cursor + seconds)},${texNumber(y + 0.2, 3)});`);
      cursor += seconds;
    }
    // The rest of the delivery is carrying legs and retreats, which no phase of
    // the trace owns; drawn as an outline so the bar still adds up to the total.
    const total = r.transport.delivery_ms / 1000;
    if (total - cursor > 0.05) {
      lines.push(`\\draw[gray!55,densely dotted] (${x(cursor)},${texNumber(y - 0.2, 3)}) `
        + `rectangle (${x(total)},${texNumber(y + 0.2, 3)});`);
    }
  });
  const axis = -rows.length * 0.58 - 0.1;
  const legend = legendRow([
    ...segments.map(s => ({ style: `${s.fill},draw=${s.draw}`, label: s.key })),
    { style: "white,draw=gray!55,densely dotted", label: "Unattributed" }
  ], axis - 0.8, 11);
  return [
    `% Generated from the E13 result.json files by src/eval/e13Rerouting.js (${rows.length} runs).`,
    "\\begin{tikzpicture}[font=\\small]",
    ...lines,
    `\\draw[->] (0,${texNumber(axis, 3)}) -- (${x(span * 1.03)},${texNumber(axis, 3)}) node[right] {$t$ [s]};`,
    ...Array.from({ length: 5 }, (_, i) => {
      const seconds = (span * i) / 4;
      return `\\draw (${x(seconds)},${texNumber(axis, 3)}) -- ++(0,-0.08) `
        + `node[below,font=\\scriptsize] {${texNumber(seconds, 0)}};`;
    }),
    ...legend,
    "\\end{tikzpicture}"
  ].join("\n") + "\n";
}

function budgetFigure(valid) {
  const arms = ARMS.filter(arm => valid.some(r => r.arm === arm && r.transport.machine_total_ms != null));
  if (!arms.length) return "% No E13 run with a complete end-to-end window.\n";
  const segments = [
    { key: "Discovery", fill: "blue!30", draw: "blue!65", of: r => r.durations_ms.route_discovery },
    { key: "Detection", fill: "red!30", draw: "red!65", of: r => r.reroutes.reduce((s, d) => s + (d.detect_ms ?? 0), 0) || null },
    { key: "Discarded motion", fill: "orange!40", draw: "orange!85!black", of: r => r.transport.wasted_ms },
    { key: "Handovers", fill: "green!30", draw: "green!50!black", of: r => r.transport.handover_total_ms }
  ];
  const value = (arm, segment) => stat(valid.filter(r => r.arm === arm).map(segment.of)).mean;
  const spans = arms.map(arm => segments.reduce((sum, s) => sum + (value(arm, s) ?? 0), 0) / 1000);
  const span = Math.max(1, ...spans);
  const scale = 12 / span;
  const x = seconds => texNumber(seconds * scale, 3);
  const lines = [];
  arms.forEach((arm, lane) => {
    const y = -lane * 1.1;
    let cursor = 0;
    lines.push(`\\node[anchor=east] at (-0.15,${texNumber(y, 3)}) {${arm === "cold" ? "Agent absent" : "Agent killed"}};`);
    for (const segment of segments) {
      const seconds = (value(arm, segment) ?? 0) / 1000;
      if (seconds <= 0) continue;
      lines.push(`\\fill[${segment.fill},draw=${segment.draw}] (${x(cursor)},${texNumber(y - 0.3, 3)}) `
        + `rectangle (${x(cursor + seconds)},${texNumber(y + 0.3, 3)});`);
      cursor += seconds;
    }
  });
  const legend = legendRow(segments.map(s => ({ style: `${s.fill},draw=${s.draw}`, label: s.key })),
    -arms.length * 1.1 - 0.45);
  return [
    `% Generated from the E13 result.json files by src/eval/e13Rerouting.js (means over ${valid.length} runs).`,
    "\\begin{tikzpicture}[font=\\small]",
    ...lines,
    `\\draw[->] (0,${texNumber(-(arms.length - 1) * 1.1 - 0.75, 3)}) -- (${x(span * 1.02)},`
      + `${texNumber(-(arms.length - 1) * 1.1 - 0.75, 3)}) node[right] {$t$ [s]};`,
    ...Array.from({ length: 6 }, (_, i) => {
      const seconds = (span * i) / 5;
      return `\\draw (${x(seconds)},${texNumber(-(arms.length - 1) * 1.1 - 0.75, 3)}) -- ++(0,-0.09) `
        + `node[below,font=\\scriptsize] {${texNumber(seconds, 0)}};`;
    }),
    ...legend,
    "\\end{tikzpicture}"
  ].join("\n") + "\n";
}

function analyze(out, runs = RUNS) {
  const all = results(out, runs);
  const valid = all.filter(r => r.status === "ok");
  fs.mkdirSync(out, { recursive: true });

  fs.writeFileSync(path.join(out, "e13_runs.csv"), csv(all.map(r => ({
    run: r.run, arm: r.arm, status: r.status, reason: r.reason,
    ...Object.fromEntries(Object.entries(r.durations_ms).map(([k, v]) => [k + "_ms", v])),
    ended_on: r.convergence.ended_on, deadlines: r.convergence.deadlines,
    silent_branches: r.convergence.silent_branches.join("/"),
    unaskable_peers: r.convergence.unaskable_peers.join("/"),
    failed_robot_priced_by: r.convergence.failed_robot_priced_by.join("/"),
    settled_after_gradle_ms: r.convergence.settled_after_gradle_ms,
    messages_to_settle: r.convergence.messages_to_settle,
    planned_route: r.route.path.join("-"), planned_cost_m: r.route.cost_m,
    executed_route: r.route.executed_path.join("-"), executed_cost_m: r.route.executed_cost_m,
    penalty_m: r.route.penalty_m, rerouted: r.route.rerouted, reroutes: r.reroutes.length,
    options_reported: r.route.options_reported,
    handovers: r.handovers.filter(h => !h.abandoned).length,
    abandoned_handovers: r.handovers.filter(h => h.abandoned).length,
    handover_total_ms: r.transport.handover_total_ms, wasted_ms: r.transport.wasted_ms,
    source_pickup_ms: r.transport.source_pickup_ms, placement_ms: r.transport.placement_ms,
    last_hop_ms: r.transport.last_hop_ms, last_hop_sessions: r.transport.last_hop_sessions,
    delivery_ms: r.transport.delivery_ms, machine_total_ms: r.transport.machine_total_ms,
    end_to_end_ms: r.transport.end_to_end_ms,
    ...Object.fromEntries(RECOVERY_MARKERS.map(([k]) => ["recovery_" + k, r.recovery?.[k] ?? null])),
    agent_killed_at_ms: r.agent_killed_at_ms,
    messages_total: r.messages_total, issues: r.issues.join("; ")
  }))));

  fs.writeFileSync(path.join(out, "e13_reroutes.csv"), csv(all.flatMap(r => r.reroutes)));

  fs.writeFileSync(path.join(out, "e13_handovers.csv"), csv(all.flatMap(r => r.handovers.map(h => ({
    run: r.run, arm: r.arm, hop: h.hop, abandoned: h.abandoned,
    abandoned_reason: h.abandoned_reason ?? (h.refused_candidate ? "candidate_refused" : ""),
    giver: h.giver, receiver: h.receiver, duration_ms: h.duration_ms, giver_observed_ms: h.giver_observed_ms,
    ...Object.fromEntries(HANDOVER_PHASES.map(p => [p + "_ms", h[p + "_ms"]]))
  })))));

  fs.writeFileSync(path.join(out, "e13_routes.csv"), csv(all.flatMap(r =>
    parseOptions(r.route.options).map((o, i) => {
      const full = [r.route.path[0], ...o.path];
      return { run: r.run, arm: r.arm, rank: i + 1, cost_m: o.cost_m, path: full.join("-"),
        planned: full.join(",") === r.route.path.join(","),
        executed: full.join(",") === r.route.executed_path.join(",") };
    }))));

  fs.writeFileSync(path.join(out, "e13_legs.csv"), csv(all.flatMap(r => [
    ...r.legs.map(l => ({ run: r.run, arm: r.arm, kind: "priced", agent: l.agent, neighbour: l.neighbour,
      mode: l.mode, distance_m: l.distance_m, candidate: l.candidate, reason: "" })),
    ...r.refused_edges.map(l => ({ run: r.run, arm: r.arm, kind: "refused", agent: l.agent, neighbour: l.neighbour,
      mode: l.mode, distance_m: null, candidate: "", reason: l.reason }))
  ])));

  // One row per arm per metric, because every headline number in E13 is a
  // comparison between the arms and a pooled mean would hide the whole result.
  const summary = [];
  const metric = (arm, name, values) => {
    const rows = values.filter(v => v != null);
    const s = meanAndSampleSd(rows);
    summary.push({ arm, metric: name, n: rows.length, mean: s.mean, sample_sd: s.sd });
  };
  for (const arm of ARMS) {
    const rows = valid.filter(r => r.arm === arm);
    for (const phase of [...STARTUP_PHASES, "route_discovery"]) metric(arm, phase + "_ms", rows.map(r => r.durations_ms[phase]));
    metric(arm, "settled_after_gradle_ms", rows.map(r => r.convergence.settled_after_gradle_ms));
    metric(arm, "messages_to_settle", rows.map(r => r.convergence.messages_to_settle));
    metric(arm, "planned_cost_m", rows.map(r => r.route.cost_m));
    metric(arm, "executed_cost_m", rows.map(r => r.route.executed_cost_m));
    metric(arm, "penalty_m", rows.map(r => r.route.penalty_m));
    metric(arm, "handover_total_ms", rows.map(r => r.transport.handover_total_ms));
    metric(arm, "wasted_ms", rows.map(r => r.transport.wasted_ms));
    metric(arm, "source_pickup_ms", rows.map(r => r.transport.source_pickup_ms));
    metric(arm, "last_hop_ms", rows.map(r => r.transport.last_hop_ms));
    metric(arm, "placement_ms", rows.map(r => r.transport.placement_ms));
    metric(arm, "delivery_ms", rows.map(r => r.transport.delivery_ms));
    metric(arm, "machine_total_ms", rows.map(r => r.transport.machine_total_ms));
    metric(arm, "end_to_end_ms", rows.map(r => r.transport.end_to_end_ms));
    for (const by of ["timeout", "refused"]) {
      metric(arm, `reroute_detect_ms_${by}`, rows.flatMap(r => r.reroutes.filter(d => d.detected_by === by).map(d => d.detect_ms)));
    }
    metric(arm, "messages_total", rows.map(r => r.messages_total));
    for (const [key] of RECOVERY_MARKERS) metric(arm, "recovery_" + key, rows.map(r => r.recovery?.[key]));
    for (const field of ["detect_ms", "wasted_approach_ms", "decide_ms", "reset_ms", "recover_ms",
      "messages_during", "messages_teardown", "penalty_m"]) {
      metric(arm, "reroute_" + field, rows.flatMap(r => r.reroutes.map(d => d[field])));
    }
    summary.push({ arm, metric: "runs_successful", n: all.filter(r => r.arm === arm).length, mean: rows.length, sample_sd: null });
    summary.push({ arm, metric: "ended_on_deadline", n: rows.length,
      mean: rows.filter(r => r.convergence.ended_on === "deadline").length, sample_sd: null });
    summary.push({ arm, metric: "distinct_executed_routes", n: rows.length,
      mean: new Set(rows.map(r => r.route.executed_path.join("-"))).size, sample_sd: null });
  }
  fs.writeFileSync(path.join(out, "e13_summary.csv"), csv(summary));

  fs.writeFileSync(path.join(out, "e13_arm_table.tex"), armTable(valid));
  fs.writeFileSync(path.join(out, "e13_reroute_table.tex"), rerouteTable(valid));
  fs.writeFileSync(path.join(out, "e13_cost_table.tex"), costTable(valid));
  fs.writeFileSync(path.join(out, "e13_figure_budget.tex"), budgetFigure(valid));
  fs.writeFileSync(path.join(out, "e13_recovery_table.tex"), recoveryTable(valid));
  fs.writeFileSync(path.join(out, "e13_figure_delivery.tex"), deliveryFigure(valid));

  return all.map(r => ({ run: r.run, arm: r.arm, status: r.status,
    discovery_ms: r.durations_ms.route_discovery, ended_on: r.convergence.ended_on,
    planned: r.route.path.join("-"), executed: r.route.executed_path.join("-"),
    penalty_m: r.route.penalty_m === null ? null : Number(r.route.penalty_m.toFixed(3)),
    detect_ms: r.reroutes[0]?.detect_ms ?? null, decide_ms: r.reroutes[0]?.decide_ms ?? null,
    delivery_ms: r.transport.delivery_ms, issues: r.issues.length }));
}

// ------------------------------------------------------------------ driving

// Which arm the next slot belongs to. Fixed by slot number rather than chosen
// per launch, so the two arms cannot drift apart in count and a half-finished
// experiment always says what is still owed.
const armOfRun = run => ARMS[Math.floor((run - 1) / PER_ARM)] ?? ARMS.at(-1);

function plannedSchedule(runs) {
  return Array.from({ length: runs }, (_, i) => ({ run: i + 1, arm: armOfRun(i + 1) }));
}

async function watch(out, runs, timeoutSeconds) {
  const armFile = path.join(out, "watch.json");
  if (fs.existsSync(armFile)) {
    try { process.kill(read(armFile).pid, 0); throw Error("A recorder is already running for this experiment."); }
    catch (e) { if (e.code !== "ESRCH") throw e; }
  }
  const jcm = preparePlans(out);
  const verified = verifyPlans(out);
  console.log(verified.skipped ? `Plan parse check skipped: ${verified.skipped}.` : `Parse-checked ${verified.checked} instrumented agent files.`);
  write(armFile, { pid: process.pid, jcm, runs, per_arm: PER_ARM, arms: ARMS, failed_robot: FAILED_ROBOT, armed_at: Date.now() });
  console.log(`E13 armed for ${runs} manually started runs: ${PER_ARM} with ${FAILED_AGENT} absent, then ${PER_ARM} with it killed after the route settles.`);
  console.log(`Keep every AgentThing running in BOTH arms, including ${FAILED_ROBOT}'s: the cold arm removes the reasoning agent, not the robot.`);
  console.log(`For each run:\n  node src/eval/e13Rerouting.js launch --out ${quote(relative(out))}`);
  console.log(`  cold runs: wait for "Route settled", then deliver.`);
  console.log(`  kill runs: wait for "Route settled", stop ${FAILED_ROBOT}'s AgentThing, run`
    + `\n    node src/eval/e13Rerouting.js killed --out ${quote(relative(out))}\n  and then deliver:`);
  console.log(`  node src/eval/e13Rerouting.js deliver --out ${quote(relative(out))}`);
  console.log("Waiting for Gradle; arming time is excluded.");
  let stopping = false;
  const stop = () => { stopping = true; };
  process.on("SIGINT", stop); process.on("SIGTERM", stop);
  try {
    while (!stopping) {
      let completed = 0;
      for (let run = 1; run <= runs; run++) {
        const directory = path.join(out, `run${run}`);
        if (fs.existsSync(path.join(directory, "result.json"))) { completed++; continue; }
        if (!fs.existsSync(path.join(directory, "run.json"))) continue;
        const meta = read(path.join(directory, "run.json"));
        const events = readLines(path.join(directory, "events.jsonl"));
        const expected = names(CELL).map(agentName).filter(a => meta.arm !== "cold" || a !== FAILED_AGENT);
        const ready = new Set(events.filter(e => e.event === "agent_ready").map(e => e.agent));
        const placed = events.some(e => e.event === "cube_placed");
        const exit = events.find(e => e.event === "gradle_exit");
        // As in E12, the transfer closes the trial -- not the route decision,
        // and not the reroute: a reroute that decided and then could not carry
        // the cube is a failed run, and has to be recorded as one.
        if (placed) finishTrial(directory, null, out);
        else if (exit) finishTrial(directory, "Gradle exited before the cube was placed", out);
        else if (Date.now() - meta.t > timeoutSeconds * 1000) finishTrial(directory, "Measurement timeout", out);
        else if (events.some(e => e.event === "reroute_exhausted")) {
          finishTrial(directory, "The reroute ran out of alternatives and the cube was never placed", out);
        } else if (ready.size === expected.length && events.some(e => e.event === "route_end" && e.data?.[0] === "no_route")) {
          finishTrial(directory, "The cell found no route to the goal", out);
        }
      }
      if (completed === runs) {
        console.log(`All ${runs} runs recorded. See e13_runs.csv, e13_reroutes.csv, e13_routes.csv and e13_summary.csv.`);
        break;
      }
      await new Promise(resolve => setTimeout(resolve, 250));
    }
  } finally {
    fs.rmSync(armFile, { force: true });
    process.off("SIGINT", stop); process.off("SIGTERM", stop);
  }
}

// A slot holding an aborted trial is not a measurement, and the experiment needs
// five usable runs of each arm. Discarding moves the whole trial -- trace, log
// and summary -- into discarded/ and frees its slot to be flown again. The arm
// follows the slot number rather than the order of launches, so a repeat lands
// in the same arm as the run it replaces and the two arms cannot drift apart.
function discard(out, runs, which, force) {
  const wanted = String(which ?? "").split(",").map(part => Number(part.trim())).filter(n => n);
  if (!wanted.length) throw Error("discard needs --run N (or --run N,M); see status for which slots are used");
  const moved = [];
  for (const run of wanted) {
    if (!Number.isInteger(run) || run < 1 || run > runs) throw Error(`--run ${run} is not a slot of this experiment`);
    const directory = path.join(out, `run${run}`);
    if (!fs.existsSync(directory)) throw Error(`Run ${run} has no recording to discard`);
    const file = path.join(directory, "result.json");
    const status = fs.existsSync(file) ? read(file).status : "unfinished";
    if (status === "ok" && !force) {
      throw Error(`Run ${run} is a usable measurement; pass --force if you really mean to discard it`);
    }
    const attic = path.join(out, "discarded", `run${run}-${new Date().toISOString().replace(/[:.]/g, "-")}`);
    fs.mkdirSync(path.dirname(attic), { recursive: true });
    fs.renameSync(directory, attic);
    moved.push({ run, arm: armOfRun(run), status, kept_in: relative(attic) });
  }
  for (const m of moved) {
    console.log(`Discarded run ${m.run} [${m.arm}] (${m.status}); its trace is kept in ${m.kept_in}.`);
  }
  console.log(`Slot(s) ${moved.map(m => m.run).join(", ")} are free again. `
    + "The next launch takes the lowest free slot, so the repeat stays in the same arm.");
  try {
    analyze(out, runs);
  } catch (error) {
    console.error("The combined tables could not be rebuilt: " + error.message);
  }
  return moved;
}

function nextSlot(out, runs) {
  let run = 1;
  while (run <= runs && fs.existsSync(path.join(out, `run${run}`))) run++;
  if (run > runs) throw Error(`All ${runs} run slots are used. Use a new --out for a new experiment.`);
  if (run > 1 && !fs.existsSync(path.join(out, `run${run - 1}`, "result.json"))) {
    throw Error("Wait for the previous result and stop its Gradle process before starting the next run.");
  }
  return run;
}

function activeSlot(out, runs) {
  for (let run = runs; run >= 1; run--) {
    const directory = path.join(out, `run${run}`);
    if (fs.existsSync(path.join(directory, "run.json")) && !fs.existsSync(path.join(directory, "result.json"))) return directory;
  }
  throw Error("No run is currently recording; start one with `launch` first.");
}

async function launch(out, override) {
  const armed = read(path.join(out, "watch.json"));
  process.kill(armed.pid, 0); // Existence check only; signal 0 sends no signal.
  const runs = armed.runs || RUNS;
  const run = nextSlot(out, runs);
  const arm = override || armOfRun(run);
  if (!ARMS.includes(arm)) throw Error(`--arm must be one of ${ARMS.join(", ")}`);
  const jcm = armed.jcm[arm];
  if (!jcm) throw Error(`No MAS declaration was prepared for the ${arm} arm; re-run watch.`);

  const snapshots = manifests().map(file => ({ file: relative(file), manifest: read(file) }));
  const descriptor = read(path.join(ROOT, snapshots[0].manifest.sharedEnvironment.descriptor));
  if (descriptor.robots.map(r => r.name).join(",") !== names(CELL).join(","))
    throw Error("Generate the five-robot cell first: its descriptor has a different robot list.");
  const directory = path.join(out, `run${run}`);
  fs.mkdirSync(directory); // Exclusive slot; never overwrite a trial.
  const trace = path.join(directory, "events.jsonl");
  fs.writeFileSync(trace, "");
  // The .jcm is copied into the slot, so which agents this run actually started
  // is part of the record rather than a claim about it.
  fs.copyFileSync(jcm, path.join(directory, path.basename(jcm)));
  write(path.join(directory, "inputs.json"), { manifests: snapshots, descriptor, arm, jcm: relative(jcm) });
  const meta = { cell: CELL, run, runs, arm, failed_robot: FAILED_ROBOT, t: Date.now(), jcm };
  write(path.join(directory, "run.json"), meta);
  append(trace, { event: "gradle_start", t: meta.t, arm });
  console.log(`Starting Gradle for run ${run}/${runs}, arm "${arm}" `
    + (arm === "cold" ? `(${FAILED_AGENT} is not in this MAS).` : `(all five agents; kill ${FAILED_ROBOT}'s AgentThing after the route settles).`));
  const { exit } = await runGradle({ jcm, directory, trace, env: { WOMAT_E11_CELL: String(CELL) } });
  write(path.join(directory, "exit.json"), exit);
}

// The user's own kill, written into the trace so the run can be checked rather
// than assumed: a kill recorded before the route settled means the run tested
// the cold arm's question under the kill arm's label, and the analysis says so.
// This only appends a timestamp. It never signals any process.
function killed(out, runs) {
  const directory = activeSlot(out, runs);
  const meta = read(path.join(directory, "run.json"));
  const events = readLines(path.join(directory, "events.jsonl"));
  const settled = events.find(e => e.event === "route_end");
  const t = Date.now();
  append(path.join(directory, "events.jsonl"),
    { event: "agent_killed", t, agent: FAILED_AGENT, robot: FAILED_ROBOT, after_route_end: Boolean(settled) });
  console.log(`Recorded the loss of ${FAILED_ROBOT} at +${((t - meta.t) / 1000).toFixed(1)} s for ${relative(directory)}.`);
  if (!settled) {
    console.log("  WARNING: no route decision has been recorded yet, so this run may not have settled on the cheap route.");
  } else {
    console.log(`  The route was settled ${((t - settled.t) / 1000).toFixed(1)} s ago; the cheap hop is the one that will fail.`);
  }
}

// The same !deliverCube the user would type into the Jason console, sent over the
// AgentThing so the request has a recorded timestamp.
async function deliver(out, runs, agent) {
  const directory = activeSlot(out, runs);
  const file = manifests().find(m => agentName(path.basename(path.dirname(m))) === agent);
  if (!file) throw Error(`${agent} is not one of ${names(CELL).map(agentName).join(", ")}`);
  const manifest = read(file);
  const url = `http://localhost:${manifest.ports.agentThing}/${agent}/actions/deliverCube`;
  const response = await fetch(url, { method: "POST", headers: { "content-type": "application/json" }, body: "{}" });
  const text = await response.text();
  append(path.join(directory, "events.jsonl"),
    { event: "delivery_requested", t: Date.now(), agent, url, status: response.status, body: text.slice(0, 500) });
  if (!response.ok) throw Error(`${agent} refused !deliverCube (HTTP ${response.status}): ${text.slice(0, 200)}`);
  console.log(`Requested !deliverCube at ${agent} for ${relative(directory)}.`);
}

function status(out, runs) {
  const done = results(out, runs);
  const rows = plannedSchedule(runs).map(({ run, arm }) => {
    const result = done.find(r => r.run === run);
    return { run, arm, recorded: Boolean(result), status: result?.status ?? "pending",
      executed: result?.route.executed_path.join("-") ?? "", issues: result?.issues.length ?? "" };
  });
  console.table(rows);
  for (const arm of ARMS) {
    const owed = rows.filter(r => r.arm === arm && r.status !== "ok").length;
    console.log(`${arm}: ${rows.filter(r => r.arm === arm && r.status === "ok").length}/${PER_ARM} usable`
      + (owed ? `, ${owed} still owed` : ""));
  }
  const attic = path.join(out, "discarded");
  if (fs.existsSync(attic)) {
    const kept = fs.readdirSync(attic).filter(name => name.startsWith("run"));
    if (kept.length) console.log(`${kept.length} discarded trial(s) kept in ${relative(attic)}: ${kept.join(", ")}`);
  }
  const free = rows.filter(r => !r.recorded).map(r => `${r.run} [${r.arm}]`);
  if (free.length) console.log("Free slot(s): " + free.join(", ") + "; the next launch takes the first of them.");
}

async function main() {
  const command = process.argv[2], args = parseArgs(process.argv.slice(3));
  const out = path.resolve(ROOT, args.out || OUT);
  const runs = Number(args.runs || RUNS);
  if (!Number.isInteger(runs) || runs < 1) throw Error("--runs must be a positive integer");
  if (!command || command === "help" || command === "--help") {
    console.log("E13: watch [--runs 10] [--timeout 2400] | launch [--arm cold|kill] | killed | deliver [--agent robot1_agent]\n"
      + "     | status | verify | analyze | resummarize | discard --run N[,M] [--force]\n"
      + `Optional --out DIR (default performance_log/e13). Runs 1-${PER_ARM} are the "cold" arm `
      + `(${FAILED_AGENT} absent), runs ${PER_ARM + 1}-${RUNS} the "kill" arm.\n`
      + "watch never starts or contacts robots; launch starts only Gradle; deliver sends only !deliverCube;\n"
      + "killed only writes a timestamp -- stopping the AgentThing is the user's own action.\n"
      + "Route discovery is measured exactly as in E11 and E12; the reroute is added.\n"
      + "discard frees the slot of an aborted trial, keeping its trace, so the repeat stays in its arm.\n"
      + "See docs/e13_rerouting.md.");
    return;
  }
  if (command === "analyze") { console.table(analyze(out, runs)); return; }
  if (command === "resummarize") { resummarize(out, runs); console.table(analyze(out, runs)); return; }
  if (command === "status") { status(out, runs); return; }
  if (command === "discard") { discard(out, runs, args.run, Boolean(args.force)); return; }
  if (command === "verify") {
    preparePlans(out);
    const v = verifyPlans(out);
    console.log(v.skipped ? `Skipped: ${v.skipped}` : `Parse-checked ${v.checked} instrumented agent files.`);
    return;
  }
  if (command === "watch") {
    const timeout = Number(args.timeout || 2400);
    if (!Number.isFinite(timeout) || timeout <= 0) throw Error("--timeout must be positive seconds");
    return watch(out, runs, timeout);
  }
  if (command === "launch") return launch(out, args.arm);
  if (command === "killed") { killed(out, runs); return; }
  if (command === "deliver") return deliver(out, runs, args.agent || "robot1_agent");
  throw Error("Unknown E13 command: " + command);
}

if (require.main === module) main().catch(error => { console.error(error.message); process.exitCode = 1; });

module.exports = {
  instrumentRouteRecovery, instrumentPartnerFailure, preparePlans,
  executedPath, markPartnerFailures, reroutes, summarize, analyze, results, resummarize,
  armTable, rerouteTable, costTable, budgetFigure, recoveryTable, deliveryFigure,
  recoveries, RECOVERY_MARKERS, armOfRun, plannedSchedule,
  discard, nextSlot, finishTrial,
  ARMS, PER_ARM, RUNS, SCHEMA, FAILED_ROBOT, FAILED_AGENT, OUT
};
