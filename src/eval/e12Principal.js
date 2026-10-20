"use strict";

// E12: the principal five-robot experiment, repeated ten times.
//
// E11 measures how long the cell takes to AGREE on a route. E12 keeps that
// measurement verbatim -- the same instrumented routePlans.asl, the same
// e11.Trace JSONL, the same monotonic clock -- and adds the part E11 never
// timed: the transfer itself. Three quantities per run:
//
//   route discovery   reused from E11 (evalTrace.instrumentRoutes)
//   handovers         per hop, giver's first motion -> receiver retreated
//   cube transport    pickup from support A -> cube released on support B
//
// The handover interval reproduces the stopwatch definition used for the first
// E12 reading: it opens when the giver starts moving towards the handover point
// and closes when the receiver has retreated holding the cube. It is taken from
// the RECEIVER's own retreat event rather than from the giver's belief of it,
// so the message hop back to the giver is not counted as motion; the
// giver-observed value is kept alongside as `giver_observed_ms` and the
// difference between the two is the messaging overhead.
//
// Like E11, this observes: only `launch` starts anything, and it starts only
// Gradle. Scene generation, the lower servers and the AgentThings stay under
// the user's control, and `deliver` sends exactly the !deliverCube the user
// would otherwise type into the Jason console.
const fs = require("fs");
const path = require("path");
const {
  ROOT, read, write, append, relative, quote, replaceOnce, readLines,
  csv, meanAndSampleSd, texNumber, instrumentRoutes, runGradle, LOGGING
} = require("./evalTrace");
const { parseArgs } = require("../generator/core/context");

const OUT = path.join(ROOT, "performance_log/e12");
const CELL = 5;
const RUNS = 10;
const STARTUP_PHASES = ["neighbour_discovery", "interaction_space", "task_sites", "initial_edge_pricing"];
// Per-handover sub-phases, in the order the giver and receiver execute them.
const HANDOVER_PHASES = ["handover_plan", "giver_approach", "receiver_approach", "wait_receiver",
  "transfer", "receiver_retreat_motion", "receiver_retreat", "giver_home"];
const TRANSPORT_PHASES = ["pickup", "placement"];
const names = n => Array.from({ length: n }, (_, i) => `Robot${i + 1}`);
const agentName = robot => String(robot).toLowerCase().replace(/^robot/, "robot") + "_agent";
const manifests = () => names(CELL).map(name => path.join(ROOT, "generated/robots", name, "manifest.json"));

// ---------------------------------------------------------------- instrument

function instrumentHandovers(source) {
  // Pickup from the loaded support. Only the robot at the source executes the
  // grasp branch; for every later giver the cube is already held and this
  // measures the confirmation query alone, which is why it is reported per
  // agent rather than summed.
  source = replaceOnce(source, "    !ensureCubeHeld(MyRobot);",
    "    e11.event(phase_start, pickup, MyRobot);\n"
    + "    !ensureCubeHeld(MyRobot);\n"
    + "    e11.event(phase_end, pickup, MyRobot);");

  // Choosing the handover point: ranking, selection and commit. No arm moves
  // here, so this is the planning cost of a hop as distinct from its motion.
  source = replaceOnce(source, "    !requestHandoverPlan(ReceiverRobot);",
    "    e11.event(phase_start, handover_plan, ReceiverRobot);\n"
    + "    !requestHandoverPlan(ReceiverRobot);\n"
    + "    e11.event(phase_end, handover_plan, ReceiverRobot);");

  source = replaceOnce(source, "    +selectedHandoverCandidate(BestCandidate, ReceiverRobot, BestScore);",
    "    +selectedHandoverCandidate(BestCandidate, ReceiverRobot, BestScore);\n"
    + "    e11.event(candidate_selected, handover, ReceiverRobot, BestCandidate, BestScore, Remaining);");

  // The handover interval opens here: the giver's first commanded motion
  // towards the rendezvous.
  source = replaceOnce(source,
    "    !approachRendezvous(Session, GiverX, GiverY, GiverZ);\n    +giverAtHandover(Session);",
    "    e11.event(handover_start, handover, Session, MyRobot, ReceiverRobot);\n"
    + "    e11.event(phase_start, giver_approach, Session);\n"
    + "    !approachRendezvous(Session, GiverX, GiverY, GiverZ);\n"
    + "    e11.event(phase_end, giver_approach, Session);\n"
    + "    +giverAtHandover(Session);");

  source = replaceOnce(source,
    "    .wait(receiverReady(Session)[source(ReceiverAgent)] |\n          handoverFailed(Session, _)[source(ReceiverAgent)], 120000);",
    "    e11.event(phase_start, wait_receiver, Session);\n"
    + "    .wait(receiverReady(Session)[source(ReceiverAgent)] |\n          handoverFailed(Session, _)[source(ReceiverAgent)], 120000);\n"
    + "    e11.event(phase_end, wait_receiver, Session);");

  // The exchange proper: the giver opens, the receiver closes, ownership moves.
  source = replaceOnce(source, "    !transferCubeTo(Session, ReceiverRobot);",
    "    e11.event(phase_start, transfer, Session);\n"
    + "    !transferCubeTo(Session, ReceiverRobot);\n"
    + "    e11.event(phase_end, transfer, Session);");

  // Giver-observed close of the handover interval.
  source = replaceOnce(source, "    .wait(receiverRetreated(Session)[source(ReceiverAgent)], 90000);",
    "    e11.event(phase_start, receiver_retreat, Session);\n"
    + "    .wait(receiverRetreated(Session)[source(ReceiverAgent)], 90000);\n"
    + "    e11.event(phase_end, receiver_retreat, Session);\n"
    + "    e11.event(handover_end, handover, Session, MyRobot, ReceiverRobot);");

  source = replaceOnce(source, "    !clearRendezvous(Session);",
    "    e11.event(phase_start, giver_home, Session);\n"
    + "    !clearRendezvous(Session);\n"
    + "    e11.event(phase_end, giver_home, Session);");

  // Receiver side.
  source = replaceOnce(source,
    "    !approachRendezvous(Session, ReceiverX, ReceiverY, ReceiverZ);\n    +receiverAtHandover(Session);",
    "    e11.event(phase_start, receiver_approach, Session);\n"
    + "    !approachRendezvous(Session, ReceiverX, ReceiverY, ReceiverZ);\n"
    + "    e11.event(phase_end, receiver_approach, Session);\n"
    + "    +receiverAtHandover(Session);");

  // The authoritative close of the handover: the receiver is clear of the
  // rendezvous and holds the cube. Emitted before the belief so the giver's
  // copy can never precede it.
  source = replaceOnce(source,
    "    !leaveRendezvous(Session, ReceiverX, ReceiverY, ReceiverZ);\n    +receiverRetreated(Session);",
    "    e11.event(phase_start, receiver_retreat_motion, Session);\n"
    + "    !leaveRendezvous(Session, ReceiverX, ReceiverY, ReceiverZ);\n"
    + "    e11.event(phase_end, receiver_retreat_motion, Session);\n"
    + "    e11.event(receiver_retreated, handover, Session, MyRobot, GiverRobot);\n"
    + "    +receiverRetreated(Session);");

  // Placement on the target support, and the end of the transport.
  source = replaceOnce(source,
    '    .print("The goal ", Goal, " is mine; placing the cube on it");\n    !placeCubeAt(Goal);',
    '    .print("The goal ", Goal, " is mine; placing the cube on it");\n'
    + "    e11.event(phase_start, placement, Goal);\n"
    + "    !placeCubeAt(Goal);\n"
    + "    e11.event(phase_end, placement, Goal);");

  source = replaceOnce(source, '    .print("Cube placed on ", Goal, " by ", MyRobot).',
    '    .print("Cube placed on ", Goal, " by ", MyRobot);\n'
    + "    e11.event(cube_placed, transport, Goal, MyRobot).");

  // A rendezvous the cell refused at execution time. Distinct from an edge the
  // pricing could not value at all; the thesis counts both.
  return replaceOnce(source, "    +handoverCandidateRejected(Candidate, ReceiverRobot);",
    "    e11.event(candidate_refused, handover, Candidate, ReceiverRobot);\n"
    + "    +handoverCandidateRejected(Candidate, ReceiverRobot);");
}

// E11's route instrumentation plus the leg decomposition: every priced edge,
// every refused edge, and the full ranked option list the source agent settles
// on. Table 5.x of the thesis is generated from these, not transcribed by hand.
function instrumentRoutesWithLegs(source) {
  source = instrumentRoutes(source);
  source = replaceOnce(source, "    +edgeCost(Neighbour, Mode, BestDistance, BestCandidate);",
    "    +edgeCost(Neighbour, Mode, BestDistance, BestCandidate);\n"
    + "    e11.event(edge_priced, edge_cost, Neighbour, Mode, BestDistance, BestCandidate);");
  source = replaceOnce(source, '        .print("Cannot price the handover to ", Neighbour, ": ", Reason);',
    "        e11.event(edge_refused, edge_cost, Neighbour, Mode, Reason);\n"
    + '        .print("Cannot price the handover to ", Neighbour, ": ", Reason);');
  source = replaceOnce(source, "        .length(Ranked, HowMany);",
    "        .length(Ranked, HowMany);\n"
    + "        e11.event(route_options, route_discovery, Task, HowMany, Ranked);");
  return replaceOnce(source,
    '    .print("Delivering the cube from ", MyRobot, " via ", NextRobot, "; the whole route costs ", Total);',
    "    e11.event(delivery_start, transport, MyRobot, NextRobot, Total);\n"
    + '    .print("Delivering the cube from ", MyRobot, " via ", NextRobot, "; the whole route costs ", Total);');
}

function instrumentAgent(source, routeFile, handoverFile) {
  for (const [goal, phase] of [["discoverNeighbors", STARTUP_PHASES[0]], ["shareInteractionSpaces", STARTUP_PHASES[1]],
    ["detectTaskSites", STARTUP_PHASES[2]], ["reportHandoverCosts", STARTUP_PHASES[3]]]) {
    source = replaceOnce(source, `    !${goal};`, `    e11.event(phase_start, ${phase});\n    !${goal};\n    e11.event(phase_end, ${phase});`);
  }
  source = replaceOnce(source, "    !discoverRoute;", "    !discoverRoute;\n    e11.event(agent_ready, startup);");
  source = replaceOnce(source, 'include("src/agt/routePlans.asl")', `include(${JSON.stringify(routeFile)})`);
  return replaceOnce(source, 'include("src/agt/handoverPlans.asl")', `include(${JSON.stringify(handoverFile)})`);
}

function preparePlans(out) {
  const directory = path.join(out, "plans");
  fs.mkdirSync(directory, { recursive: true });
  const route = path.join(directory, "routePlans.asl");
  const handover = path.join(directory, "handoverPlans.asl");
  fs.writeFileSync(route, instrumentRoutesWithLegs(fs.readFileSync(path.join(ROOT, "src/agt/routePlans.asl"), "utf8")));
  fs.writeFileSync(handover, instrumentHandovers(fs.readFileSync(path.join(ROOT, "src/agt/handoverPlans.asl"), "utf8")));
  const agents = [], workspaces = [];
  for (const [i, file] of manifests().entries()) {
    const manifest = read(file), agent = `robot${i + 1}_agent`, workspace = `robot${i + 1}_workspace`;
    const dest = path.join(directory, agent + ".asl");
    fs.writeFileSync(dest, instrumentAgent(fs.readFileSync(path.join(ROOT, manifest.generatedFiles.agentSpeak), "utf8"), route, handover));
    if (/\s/.test(relative(dest))) throw Error("E12 paths must not contain whitespace (JaCaMo source syntax).");
    agents.push(`    agent ${agent}: ${relative(dest)} {\n        ag-arch: e11.MessageTrace\n        focus: ${workspace}.wotComm\n    }`);
    workspaces.push(`    workspace ${workspace} {\n        artifact wotComm: wotCommunication.wotCommunicator("${manifest.configKey}")\n    }`);
  }
  const jcm = path.join(directory, "e12.jcm");
  fs.writeFileSync(jcm, `mas e12_principal {\n${agents.join("\n")}\n${workspaces.join("\n")}\n}\n`);
  fs.writeFileSync(path.join(directory, "logging.properties"), LOGGING);
  return jcm;
}

// ------------------------------------------------------------------ analysis

const num = value => { const n = Number(String(value)); return Number.isFinite(n) ? n : null; };
const sameRobot = (a, b) => String(a).toLowerCase() === String(b).toLowerCase();

// Pair phase_start with the next phase_end of the same agent, phase and key.
// Keyed pairing rather than positional: an intermediate robot runs its receive
// and its give as two sessions, and a retried handover reopens a phase the
// previous attempt never closed.
function pairPhases(events, phase) {
  const open = new Map(), pairs = [];
  for (const event of events) {
    if (event.phase !== phase) continue;
    const key = `${event.agent}|${event.data?.[0] ?? ""}`;
    if (event.event === "phase_start") open.set(key, event);
    else if (event.event === "phase_end" && open.has(key)) {
      const start = open.get(key);
      open.delete(key);
      pairs.push({ phase, agent: start.agent, key: start.data?.[0] ?? null,
        start_mono: start.mono_ms, end_mono: event.mono_ms, duration_ms: event.mono_ms - start.mono_ms, t: start.t });
    }
  }
  for (const [, start] of open) pairs.push({ phase, agent: start.agent, key: start.data?.[0] ?? null,
    start_mono: start.mono_ms, end_mono: null, duration_ms: null, t: start.t });
  return pairs;
}

function handovers(events, meta) {
  const phases = Object.fromEntries(HANDOVER_PHASES.map(p => [p, pairPhases(events, p)]));
  const opened = [];
  for (const event of events) {
    if (event.event === "handover_start") {
      opened.push({ session: event.data?.[0] ?? null, giver: event.data?.[1] ?? null, receiver: event.data?.[2] ?? null,
        giver_agent: event.agent, start_mono: event.mono_ms, start_t: event.t,
        end_mono: null, giver_end_mono: null, retreat_agent: null });
      continue;
    }
    if (event.event !== "receiver_retreated" && event.event !== "handover_end") continue;
    const session = event.data?.[0] ?? null;
    // The open interval for this session, oldest first: a retry reuses nothing,
    // but an intermediate robot is giver in one session and receiver in another.
    const open = opened.find(h => h.session === session
      && (event.event === "receiver_retreated" ? h.end_mono === null : h.giver_end_mono === null));
    if (!open) continue;
    if (event.event === "receiver_retreated") { open.end_mono = event.mono_ms; open.retreat_agent = event.agent; }
    else open.giver_end_mono = event.mono_ms;
  }
  // Phases inside a session are keyed on it. handover_plan runs before the
  // session exists -- it is what produces it -- so it is keyed on the receiver
  // robot and matched to the latest such phase opened before the hop began.
  // A rendezvous the cell refuses ends the attempt, not the hop: the giver
  // strikes the candidate, picks the next-best and opens a SECOND interval to
  // the same receiver. The abandoned interval never closes, and counting it as
  // a failed handover would report a run that succeeded on the retry as broken.
  for (const event of events) {
    if (event.event !== "candidate_refused") continue;
    const [candidate, receiver] = [event.data?.[0], event.data?.[1]];
    const attempt = opened.filter(h => h.giver_agent === event.agent && sameRobot(h.receiver, receiver)
      && h.end_mono === null && h.start_mono <= event.mono_ms).at(-1);
    if (attempt) { attempt.abandoned = true; attempt.refused_candidate = candidate; }
  }
  const rowsFor = (hop, phase) => {
    if (phase !== "handover_plan") return phases[phase].filter(p => String(p.key) === String(hop.session));
    const candidates = phases[phase]
      .filter(p => sameRobot(p.key, hop.receiver) && p.agent === hop.giver_agent && p.start_mono <= hop.start_mono)
      .sort((a, b) => a.start_mono - b.start_mono);
    return candidates.length ? [candidates.at(-1)] : [];
  };
  const subPhase = (hop, phase) => {
    const rows = rowsFor(hop, phase);
    return rows.length ? rows.reduce((sum, p) => sum + (p.duration_ms ?? 0), 0) : null;
  };
  const windows = hop => Object.fromEntries(HANDOVER_PHASES.map(phase => {
    const rows = rowsFor(hop, phase).filter(r => r.end_mono !== null);
    return [phase, rows.length ? [Math.min(...rows.map(r => r.start_mono)), Math.max(...rows.map(r => r.end_mono))] : null];
  }));
  let carried = 0;
  return opened.map(hop => ({
    run: meta.run, hop: hop.abandoned ? null : ++carried, abandoned: Boolean(hop.abandoned),
    refused_candidate: hop.refused_candidate ?? null,
    session: hop.session, giver: hop.giver, receiver: hop.receiver,
    giver_agent: hop.giver_agent, start_ms: hop.start_t - meta.t, start_mono: hop.start_mono, end_mono: hop.end_mono, windows: windows(hop),
    // The stopwatch-equivalent quantity: giver's first motion to the receiver's
    // completed retreat, both read from the receiver's and giver's own events
    // on the one JVM clock.
    duration_ms: hop.end_mono !== null ? hop.end_mono - hop.start_mono : null,
    giver_observed_ms: hop.giver_end_mono !== null ? hop.giver_end_mono - hop.start_mono : null,
    ...Object.fromEntries(HANDOVER_PHASES.map(p => [p + "_ms", subPhase(hop, p)]))
  }));
}

function summarize(meta, events, reason = null) {
  const expected = names(CELL).map(n => n.toLowerCase() + "_agent");
  const ready = new Set(events.filter(e => e.event === "agent_ready").map(e => e.agent));
  const issues = [];
  const durations = {};
  for (const phase of STARTUP_PHASES) {
    const rows = pairPhases(events, phase).filter(p => expected.includes(p.agent));
    for (const agent of expected) if (!rows.some(r => r.agent === agent && r.duration_ms !== null)) issues.push(`Missing ${phase} boundary for ${agent}`);
    durations[phase] = rows.length && rows.every(p => p.duration_ms !== null)
      ? Math.max(...rows.map(p => p.end_mono)) - Math.min(...rows.map(p => p.start_mono)) : null;
  }
  const routeStart = events.find(e => e.event === "phase_start" && e.phase === "route_discovery");
  const routeEnd = events.find(e => e.event === "route_end");
  durations.route_discovery = routeStart && routeEnd ? routeEnd.mono_ms - routeStart.mono_ms : null;
  if (!routeEnd) issues.push("No route decision was recorded");

  const legs = events.filter(e => e.event === "edge_priced").map(e => ({
    agent: e.agent, neighbour: e.data?.[0], mode: e.data?.[1], distance_m: num(e.data?.[2]), candidate: e.data?.[3] }));
  const refusedEdges = events.filter(e => e.event === "edge_refused").map(e => ({
    agent: e.agent, neighbour: e.data?.[0], mode: e.data?.[1], reason: e.data?.[2] }));
  const refusedCandidates = events.filter(e => e.event === "candidate_refused").map(e => ({
    agent: e.agent, candidate: e.data?.[0], receiver: e.data?.[1] }));
  const selected = events.filter(e => e.event === "candidate_selected").map(e => ({
    agent: e.agent, receiver: e.data?.[0], candidate: e.data?.[1], score: num(e.data?.[2]),
    usable: num(e.data?.[3]), mono_ms: e.mono_ms }));

  const optionEvent = events.find(e => e.event === "route_options");
  const outcome = routeEnd?.data?.[0] || "missing";
  const route = {
    outcome, origin: routeEnd?.agent ?? null,
    cost_m: num(routeEnd?.data?.[2]),
    path: String(routeEnd?.data?.[3] ?? "").replace(/^\[|\]$/g, "").split(",").map(s => s.trim()).filter(Boolean),
    options_reported: optionEvent ? num(optionEvent.data?.[1]) : null,
    options: String(optionEvent?.data?.[2] ?? "")
  };

  const hops = handovers(events, meta);
  const carriedHops = hops.filter(h => !h.abandoned);
  const placed = events.find(e => e.event === "cube_placed");
  const deliveryStart = events.find(e => e.event === "delivery_start");
  const pickups = pairPhases(events, "pickup");
  const placements = pairPhases(events, "placement");
  const sourcePickup = pickups.find(p => p.agent === agentName(route.path[0] || "Robot1"));
  const transport = {
    delivery_requested_ms: deliveryStart ? deliveryStart.t - meta.t : null,
    // Delivery: from the !deliverCube that the source agent adopts to the cube
    // being released on the target support. It contains the pickup from support
    // A, every handover, the carrying legs between them and the placement.
    delivery_ms: deliveryStart && placed ? placed.mono_ms - deliveryStart.mono_ms : null,
    source_pickup_ms: sourcePickup?.duration_ms ?? null,
    placement_ms: placements[0]?.duration_ms ?? null,
    handover_total_ms: carriedHops.length && carriedHops.every(h => h.duration_ms !== null)
      ? carriedHops.reduce((sum, h) => sum + h.duration_ms, 0) : null,
    // What the cost function never prices: carrying legs, retreats, placement.
    // Delivery minus the handover intervals, in the same run.
    unpriced_ms: null,
    sink: placed?.data?.[1] ?? null, goal: placed?.data?.[0] ?? null,
    start_mono: deliveryStart?.mono_ms ?? null, placed_mono: placed?.mono_ms ?? null,
    pickup_window: sourcePickup ? [sourcePickup.start_mono, sourcePickup.end_mono] : null,
    placement_window: placements[0] && placements[0].end_mono !== null ? [placements[0].start_mono, placements[0].end_mono] : null
  };
  if (transport.delivery_ms !== null && transport.handover_total_ms !== null) {
    transport.unpriced_ms = transport.delivery_ms - transport.handover_total_ms;
  }

  if (ready.size !== CELL) issues.push(`Only ${ready.size}/${CELL} agents initialized`);
  if (!placed) issues.push("The cube was never reported placed on the target support");
  const carried = hops.filter(h => !h.abandoned);
  for (const hop of carried) if (hop.duration_ms === null) issues.push(`Handover ${hop.giver}->${hop.receiver} never completed`);
  if (placed && carried.length && route.path.length && carried.length !== route.path.length - 1) {
    issues.push(`${carried.length} handover(s) carried the cube but the selected route has ${route.path.length - 1} hop(s)`);
  }

  return {
    run: meta.run, status: issues.length || reason ? "incomplete" : outcome === "no_route" ? "no_route" : "ok",
    reason, issues, started_at: new Date(meta.t).toISOString(), elapsed_ms: (events.at(-1)?.t ?? meta.t) - meta.t,
    durations_ms: durations, route, legs, refused_edges: refusedEdges, refused_candidates: refusedCandidates,
    selected_candidates: selected, handovers: hops, transport,
    messages_total: events.filter(e => e.event === "message" && expected.includes(e.agent) && expected.includes(e.receiver)).length
  };
}

// ------------------------------------------------------- cost decomposition

// Eq. (route_cost): the source pays its way from the cube to the first
// rendezvous, every other robot pays its approach to the rendezvous at which it
// receives. Rebuilt here from the priced edges so the thesis table is generated
// from the run rather than transcribed from the console.
function decompose(path, legs) {
  const rows = [];
  for (let i = 1; i < path.length; i++) {
    const giver = path[i - 1], receiver = path[i];
    if (i === 1) {
      const pickup = legs.find(l => l.mode === "pickup" && sameRobot(l.agent, agentName(giver)) && sameRobot(l.neighbour, receiver));
      rows.push({ leg: `Pickup leg of ${giver} (to ${receiver})`, robot: giver, kind: "pickup",
        distance_m: pickup?.distance_m ?? null, candidate: pickup?.candidate ?? null });
    }
    const approach = legs.find(l => l.mode === "pose" && sameRobot(l.agent, agentName(receiver)) && sameRobot(l.neighbour, giver));
    rows.push({ leg: `Approach of ${receiver} (from ${giver})`, robot: receiver, kind: "approach",
      distance_m: approach?.distance_m ?? null, candidate: approach?.candidate ?? null });
  }
  const total = rows.every(r => r.distance_m !== null) ? rows.reduce((sum, r) => sum + r.distance_m, 0) : null;
  return { rows, total_m: total };
}

// `Ranked` as Jason prints it: [total(Cost,Agent,Robot,[path]),...].
function parseOptions(text) {
  const options = [];
  for (const match of String(text).matchAll(/total\(\s*([-\d.eE+]+)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*\[([^\]]*)\]\s*\)/g)) {
    options.push({ cost_m: Number(match[1]), agent: match[2], robot: match[3],
      path: match[4].split(",").map(s => s.trim()).filter(Boolean) });
  }
  return options.sort((a, b) => a.cost_m - b.cost_m);
}

// Metres of Eq. (route_cost) attributable to each executed hop: the first hop
// carries the source's pickup leg as well as the receiver's approach, every
// later hop only its receiver's approach. This is the quantity the measured
// handover duration is plotted against.
function pricedPerHop(result) {
  const path = ["", ...result.route.path].slice(1);
  const decomposition = decompose(path, result.legs);
  return result.handovers.map(hop => {
    const approach = decomposition.rows.find(r => r.kind === "approach" && sameRobot(r.robot, hop.receiver));
    const pickup = hop.hop === 1 ? decomposition.rows.find(r => r.kind === "pickup") : null;
    const parts = [approach?.distance_m, pickup?.distance_m].filter(v => v != null);
    // The selection this attempt acted on: the last one made before it opened,
    // which after a retry is the replacement rather than the refused original.
    const executed = result.selected_candidates.filter(c => sameRobot(c.receiver, hop.receiver)
      && c.agent === hop.giver_agent && c.mono_ms <= hop.start_mono).at(-1);
    return { ...hop, priced_m: parts.length ? parts.reduce((a, b) => a + b, 0) : null,
      candidate: approach?.candidate ?? null, executed_candidate: executed?.candidate ?? null,
      candidate_matches_price: executed && approach ? executed.candidate === approach.candidate : null,
      usable_candidates: executed?.usable ?? null };
  });
}

// ---------------------------------------------------------------- recording

function finishTrial(directory, reason, out) {
  if (fs.existsSync(path.join(directory, "result.json"))) return;
  const meta = read(path.join(directory, "run.json"));
  const events = readLines(path.join(directory, "events.jsonl"));
  const result = summarize(meta, events, reason);
  fs.writeFileSync(path.join(directory, "measurement-events.jsonl"), events.map(e => JSON.stringify(e) + "\n").join(""));
  fs.writeFileSync(path.join(directory, "messages.csv"), csv(events.filter(e => e.event === "message").map(e => ({
    since_gradle_ms: e.t - meta.t, sender: e.agent, receiver: e.receiver, performative: e.performative, content: e.content, message_id: e.message_id }))));
  write(path.join(directory, "result.json"), result);
  analyze(out);
  const hops = result.handovers.map(h => `${h.giver}->${h.receiver} ${h.duration_ms === null ? "--" : (h.duration_ms / 1000).toFixed(1) + "s"}`).join(", ");
  console.log(`Run ${meta.run}: ${result.status}; route discovery ${result.durations_ms.route_discovery ?? "--"} ms; `
    + `route ${result.route.path.join("-") || "--"} at ${result.route.cost_m ?? "--"} m; handovers ${hops || "none"}; `
    + `delivery ${result.transport.delivery_ms === null ? "--" : (result.transport.delivery_ms / 1000).toFixed(1) + " s"}`);
  if (result.issues.length) console.log("  issues: " + result.issues.join("; "));
}

// Rebuild every result.json from the stored traces. The raw events are the
// record; result.json is a derived view of them, so a correction to the
// analysis is applied to runs already recorded instead of requiring the cell to
// be run again. The recorded `reason` is preserved.
function resummarize(out, runs) {
  let rebuilt = 0;
  for (let run = 1; run <= runs; run++) {
    const directory = path.join(out, `run${run}`);
    const meta = path.join(directory, "run.json"), previous = path.join(directory, "result.json");
    if (!fs.existsSync(meta) || !fs.existsSync(previous)) continue;
    const reason = read(previous).reason ?? null;
    write(previous, summarize(read(meta), readLines(path.join(directory, "events.jsonl")), reason));
    rebuilt++;
  }
  console.log(`Rebuilt ${rebuilt} result file(s) from their traces.`);
  return rebuilt;
}

function results(out, runs) {
  const rows = [];
  for (let run = 1; run <= runs; run++) {
    const file = path.join(out, `run${run}`, "result.json");
    if (fs.existsSync(file)) rows.push(read(file));
  }
  return rows;
}

// ------------------------------------------------------- thesis artifacts

const ROW = String.raw`\\`;
// Robot names arrive from the agents as `robot4`; the thesis writes `Robot~4`.
const texRobot = text => String(text).replace(/robot\s*(\d+)/gi, "Robot~$1");
const texRobots = list => list.map(r => texRobot(String(r))).join(", ");
const pm = (stat, digits = 2) => stat.mean === null ? "--"
  : stat.sd === null ? texNumber(stat.mean, digits) : `${texNumber(stat.mean, digits)} \\pm ${texNumber(stat.sd, digits)}`;

function handoverTable(valid) {
  const order = [];
  for (const result of valid) for (const hop of result.handovers.filter(h => !h.abandoned)) {
    const key = `${hop.giver}|${hop.receiver}`;
    if (!order.some(o => o.key === key)) order.push({ key, giver: hop.giver, receiver: hop.receiver, hop: hop.hop });
  }
  order.sort((a, b) => a.hop - b.hop);
  const priced = valid.map(pricedPerHop);
  const lines = order.map(({ key, giver, receiver }) => {
    const hops = priced.flat().filter(h => !h.abandoned && `${h.giver}|${h.receiver}` === key);
    const seconds = meanAndSampleSd(hops.map(h => h.duration_ms).filter(v => v != null).map(v => v / 1000));
    const metres = meanAndSampleSd(hops.map(h => h.priced_m).filter(v => v != null));
    const label = `${texRobot(giver)} $\\rightarrow$ ${texRobot(receiver)}`;
    return `        ${label} & ${metres.mean === null ? "--" : texNumber(metres.mean, 3)} & $${pm(seconds)}$ & ${hops.filter(h => h.duration_ms != null).length} ${ROW}`;
  });
  const totals = meanAndSampleSd(valid.map(r => r.transport.handover_total_ms).filter(v => v != null).map(v => v / 1000));
  const delivery = meanAndSampleSd(valid.map(r => r.transport.delivery_ms).filter(v => v != null).map(v => v / 1000));
  return [
    `% Generated from the E12 result.json files by src/eval/e12Principal.js (${valid.length} successful runs).`,
    "\\begin{tabular}{@{}lrrr@{}}", "\\toprule",
    `        \\textbf{Handover} & \\textbf{Priced [m]} & \\textbf{Duration [s]} & \\textbf{n} ${ROW}`,
    "\\midrule", ...lines, "\\midrule",
    `        All handovers & -- & $${pm(totals)}$ & ${valid.length} ${ROW}`,
    `        Delivery (pickup to placement) & -- & $${pm(delivery)}$ & ${valid.length} ${ROW}`,
    "\\bottomrule", "\\end{tabular}"
  ].join("\n") + "\n";
}

function costTable(valid) {
  const reference = valid.find(r => r.route.path.length) || valid[0];
  if (!reference) return "% No successful E12 run to decompose.\n";
  const options = parseOptions(reference.route.options);
  const paths = options.length ? options.map(o => [reference.route.path[0], ...o.path]) : [reference.route.path];
  const columns = paths.map(p => decompose(p, reference.legs));
  // Name each column by what distinguishes it. Routes that share a prefix --
  // which is the interesting case, and the one the five-robot cell presents --
  // would otherwise all be headed by the robots they have in common.
  const common = paths.reduce((keep, p) => keep.filter(r => p.some(x => sameRobot(x, r))), [...paths[0]]);
  const labels = paths.map(p => {
    const distinct = p.slice(1, -1).filter(r => !common.some(c => sameRobot(c, r)));
    return "Via " + texRobots(distinct.length ? distinct : p.slice(1, -1)) + " [m]";
  });
  // Legs are listed in the order of the cheapest route; a leg absent from a
  // column is left blank rather than zero, because zero is a priced value.
  const seen = [];
  for (const column of columns) for (const row of column.rows) if (!seen.some(s => s.leg === row.leg)) seen.push(row);
  const lines = seen.map(row => "        " + texRobot(row.leg) + " & "
    + columns.map(c => { const hit = c.rows.find(r => r.leg === row.leg); return hit && hit.distance_m != null ? texNumber(hit.distance_m, 3) : ""; }).join(" & ") + ` ${ROW}`);
  return [
    "% Generated from the E12 result.json files by src/eval/e12Principal.js.",
    `\\begin{tabular}{@{}l${"r".repeat(columns.length)}@{}}`, "\\toprule",
    `        \\textbf{Leg} & ${labels.map(l => "\\textbf{" + l + "}").join(" & ")} ${ROW}`,
    "\\midrule", ...lines, "\\midrule",
    `        Cumulative cost & ${columns.map(c => c.total_m == null ? "--" : texNumber(c.total_m, 3)).join(" & ")} ${ROW}`,
    "\\bottomrule", "\\end{tabular}"
  ].join("\n") + "\n";
}

// One lane per robot, drawn against seconds since !deliverCube. Bars cover the
// intervals the arm is engaged in the transfer, including waiting; they are not
// claims of continuous motion, which is the same caveat E10's figure carries.
function timelineFigure(valid) {
  const usable = valid.filter(r => r.transport.delivery_ms != null && r.transport.start_mono != null);
  if (!usable.length) return "% No successful E12 run with a complete transport window.\n";
  const sorted = [...usable].sort((a, b) => a.transport.delivery_ms - b.transport.delivery_ms);
  const run = sorted[Math.floor((sorted.length - 1) / 2)]; // lower median
  const t0 = run.transport.start_mono;
  const seconds = mono => (mono - t0) / 1000;
  const lanes = [...new Set(run.route.path.map(r => String(r)))];
  const bars = [];
  for (const hop of run.handovers.filter(h => !h.abandoned)) {
    const giverLane = lanes.findIndex(l => sameRobot(l, hop.giver));
    const receiverLane = lanes.findIndex(l => sameRobot(l, hop.receiver));
    const giverEnd = hop.windows.giver_home?.[1] ?? hop.end_mono;
    if (giverLane >= 0 && hop.end_mono != null) bars.push({ lane: giverLane, from: seconds(hop.start_mono), to: seconds(giverEnd), fill: "blue!30", draw: "blue!65" });
    const receiverStart = hop.windows.receiver_approach?.[0];
    const receiverEnd = hop.windows.receiver_retreat_motion?.[1] ?? hop.end_mono;
    if (receiverLane >= 0 && receiverStart != null && receiverEnd != null) bars.push({ lane: receiverLane, from: seconds(receiverStart), to: seconds(receiverEnd), fill: "green!35", draw: "green!50!black" });
  }
  if (run.transport.pickup_window) bars.push({ lane: 0, from: seconds(run.transport.pickup_window[0]), to: seconds(run.transport.pickup_window[1]), fill: "orange!45", draw: "orange!85!black" });
  const sinkLane = lanes.findIndex(l => sameRobot(l, run.transport.sink));
  if (run.transport.placement_window && sinkLane >= 0) bars.push({ lane: sinkLane, from: seconds(run.transport.placement_window[0]), to: seconds(run.transport.placement_window[1]), fill: "orange!45", draw: "orange!85!black" });
  const span = Math.max(1, ...bars.map(b => b.to));
  const scale = 13.5 / span; // cm per second across a \textwidth figure
  const x = value => texNumber(value * scale, 3);
  const y = lane => texNumber(-lane * 0.85, 3);
  return [
    `% Generated from E12 run ${run.run} (lower median delivery of ${usable.length}) by src/eval/e12Principal.js.`,
    "\\begin{tikzpicture}[font=\\small]",
    ...lanes.map((lane, i) => `\\node[anchor=east] at (-0.15,${y(i)}) {${texRobot(lane)}};`),
    ...bars.map(b => `\\fill[${b.fill},draw=${b.draw}] (${x(b.from)},${texNumber(Number(y(b.lane)) - 0.22, 3)}) rectangle (${x(b.to)},${texNumber(Number(y(b.lane)) + 0.22, 3)});`),
    `\\draw[->] (0,${y(lanes.length - 1 + 0.75)}) -- (${x(span * 1.02)},${y(lanes.length - 1 + 0.75)}) node[right] {$t$ [s]};`,
    ...Array.from({ length: 6 }, (_, i) => {
      const value = (span * i) / 5;
      return `\\draw (${x(value)},${y(lanes.length - 1 + 0.75)}) -- ++(0,-0.09) node[below,font=\\scriptsize] {${texNumber(value, 0)}};`;
    }),
    `\\draw[dashed] (${x(seconds(run.transport.placed_mono))},0.4) -- (${x(seconds(run.transport.placed_mono))},${y(lanes.length - 1 + 0.6)}) node[above right,font=\\scriptsize,anchor=south west,rotate=0] at (${x(seconds(run.transport.placed_mono))},0.4) {cube placed};`,
    "\\end{tikzpicture}"
  ].join("\n") + "\n";
}

function analyze(out, runs = RUNS) {
  const all = results(out, runs);
  const valid = all.filter(r => r.status === "ok");
  fs.mkdirSync(out, { recursive: true });

  fs.writeFileSync(path.join(out, "e12_runs.csv"), csv(all.map(r => ({
    run: r.run, status: r.status, reason: r.reason,
    ...Object.fromEntries(Object.entries(r.durations_ms).map(([k, v]) => [k + "_ms", v])),
    route: r.route.path.join("-"), route_cost_m: r.route.cost_m, options_reported: r.route.options_reported,
    handovers: r.handovers.filter(h => !h.abandoned).length,
    retried_handovers: r.handovers.filter(h => h.abandoned).length,
    handover_total_ms: r.transport.handover_total_ms, delivery_ms: r.transport.delivery_ms,
    source_pickup_ms: r.transport.source_pickup_ms, placement_ms: r.transport.placement_ms,
    unpriced_ms: r.transport.unpriced_ms,
    refused_edges: r.refused_edges.length, refused_candidates: r.refused_candidates.length,
    messages_total: r.messages_total, issues: r.issues.join("; ")
  }))));

  fs.writeFileSync(path.join(out, "e12_handovers.csv"), csv(all.flatMap(r => pricedPerHop(r).map(h => ({
    run: r.run, hop: h.hop, abandoned: h.abandoned, refused_candidate: h.refused_candidate,
    giver: h.giver, receiver: h.receiver,
    priced_candidate: h.candidate, executed_candidate: h.executed_candidate,
    candidate_matches_price: h.candidate_matches_price, usable_candidates: h.usable_candidates,
    priced_m: h.priced_m, duration_ms: h.duration_ms, giver_observed_ms: h.giver_observed_ms,
    messaging_overhead_ms: h.giver_observed_ms != null && h.duration_ms != null ? h.giver_observed_ms - h.duration_ms : null,
    ...Object.fromEntries(HANDOVER_PHASES.map(p => [p + "_ms", h[p + "_ms"]]))
  })))));

  fs.writeFileSync(path.join(out, "e12_legs.csv"), csv(all.flatMap(r => [
    ...r.legs.map(l => ({ run: r.run, kind: "priced", agent: l.agent, neighbour: l.neighbour, mode: l.mode, distance_m: l.distance_m, candidate: l.candidate, reason: "" })),
    ...r.refused_edges.map(l => ({ run: r.run, kind: "refused", agent: l.agent, neighbour: l.neighbour, mode: l.mode, distance_m: null, candidate: "", reason: l.reason }))
  ])));

  fs.writeFileSync(path.join(out, "e12_routes.csv"), csv(all.flatMap(r =>
    parseOptions(r.route.options).map((o, i) => ({ run: r.run, rank: i + 1, cost_m: o.cost_m,
      path: [r.route.path[0], ...o.path].join("-"), selected: i === 0 && o.path.join(",") === r.route.path.slice(1).join(",") })))));

  const summary = [];
  const metric = (name, values) => { const s = meanAndSampleSd(values.filter(v => v != null)); summary.push({ metric: name, n: values.filter(v => v != null).length, mean: s.mean, sample_sd: s.sd }); };
  for (const phase of [...STARTUP_PHASES, "route_discovery"]) metric(phase + "_ms", valid.map(r => r.durations_ms[phase]));
  metric("handover_total_ms", valid.map(r => r.transport.handover_total_ms));
  metric("delivery_ms", valid.map(r => r.transport.delivery_ms));
  metric("unpriced_ms", valid.map(r => r.transport.unpriced_ms));
  metric("source_pickup_ms", valid.map(r => r.transport.source_pickup_ms));
  metric("placement_ms", valid.map(r => r.transport.placement_ms));
  metric("route_cost_m", valid.map(r => r.route.cost_m));
  for (const phase of HANDOVER_PHASES) metric(phase + "_ms", valid.flatMap(r => r.handovers.filter(h => !h.abandoned).map(h => h[phase + "_ms"])));
  summary.push({ metric: "runs_successful", n: all.length, mean: valid.length, sample_sd: null });
  const distinct = new Set(valid.map(r => r.route.path.join("-")));
  summary.push({ metric: "distinct_selected_routes", n: valid.length, mean: distinct.size, sample_sd: null });
  fs.writeFileSync(path.join(out, "e12_summary.csv"), csv(summary));

  fs.writeFileSync(path.join(out, "e12_handover_table.tex"), handoverTable(valid));
  fs.writeFileSync(path.join(out, "e12_cost_table.tex"), costTable(valid));
  fs.writeFileSync(path.join(out, "e12_figure_timeline.tex"), timelineFigure(valid));
  return all.map(r => ({ run: r.run, status: r.status, route: r.route.path.join("-"), cost_m: r.route.cost_m,
    route_discovery_ms: r.durations_ms.route_discovery, handover_total_ms: r.transport.handover_total_ms,
    delivery_ms: r.transport.delivery_ms, issues: r.issues.length }));
}

// ------------------------------------------------------------------ driving

// Instrumented plans are parsed with Jason's own parser before the cell is
// armed, so a stale anchor or a malformed probe is caught here and not halfway
// through a run the user has set the scene up for.
function verifyPlans(out) {
  const { execFileSync } = require("child_process");
  const directory = path.join(out, "plans");
  const check = path.join(directory, "ParseCheck.java");
  const jars = [path.join(process.env.HOME || "", ".gradle/caches/modules-2"), path.join(ROOT, ".gradle/caches/modules-2")]
    .filter(d => fs.existsSync(d));
  if (!jars.length) return { skipped: "no Gradle module cache found" };
  let classpath;
  try {
    classpath = jars.flatMap(d => execFileSync("find", [d, "-name", "*.jar"], { encoding: "utf8" }).split("\n")).filter(Boolean).join(":");
  } catch { return { skipped: "could not enumerate the Gradle module cache" }; }
  if (!/jason-interpreter/.test(classpath)) return { skipped: "the Jason interpreter jar is not in the Gradle cache" };
  fs.writeFileSync(check, [
    "import jason.asSyntax.parser.as2j;", "import jason.asSemantics.Agent;", "import java.io.FileReader;",
    "public class ParseCheck {", "  public static void main(String[] a) throws Exception {", "    int bad = 0;",
    "    for (String f : a) {", "      try { Agent ag = new Agent(); ag.initAg(); new as2j(new FileReader(f)).agent(ag); }",
    "      catch (Throwable t) { bad++; System.out.println(\"FAIL \" + f + \": \" + t.getMessage()); }", "    }",
    "    System.exit(bad == 0 ? 0 : 1);", "  }", "}"
  ].join("\n") + "\n");
  const agents = names(CELL).map(n => path.join(directory, agentName(n) + ".asl"));
  try {
    execFileSync("javac", ["-cp", classpath, "-d", directory, check], { stdio: "pipe" });
    execFileSync("java", ["-cp", classpath + ":" + directory, "ParseCheck", ...agents], { stdio: "pipe" });
  } catch (error) {
    if (error.code === "ENOENT") return { skipped: "no JDK on PATH (javac/java)" };
    throw Error("Instrumented plans do not parse:\n" + String(error.stdout || error.message));
  }
  return { checked: agents.length };
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
  write(armFile, { pid: process.pid, jcm, runs, armed_at: Date.now() });
  console.log(`E12 armed for ${runs} manually started runs of the five-robot cell.`);
  console.log(`In another terminal, for each run:\n  node src/eval/e12Principal.js launch --out ${quote(relative(out))}`);
  console.log(`  then, once the route is settled:\n  node src/eval/e12Principal.js deliver --out ${quote(relative(out))}`);
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
        const ready = new Set(events.filter(e => e.event === "agent_ready").map(e => e.agent));
        const placed = events.some(e => e.event === "cube_placed");
        const exit = events.find(e => e.event === "gradle_exit");
        // The transfer, not the route decision, closes an E12 trial.
        if (placed) finishTrial(directory, null, out);
        else if (exit) finishTrial(directory, "Gradle exited before the cube was placed", out);
        else if (Date.now() - meta.t > timeoutSeconds * 1000) finishTrial(directory, "Measurement timeout", out);
        else if (ready.size === CELL && events.some(e => e.event === "route_end" && e.data?.[0] === "no_route")) {
          finishTrial(directory, "The cell found no route to the goal", out);
        }
      }
      if (completed === runs) { console.log(`All ${runs} runs recorded. See e12_runs.csv, e12_handovers.csv, e12_legs.csv and e12_summary.csv.`); break; }
      await new Promise(resolve => setTimeout(resolve, 250));
    }
  } finally {
    fs.rmSync(armFile, { force: true });
    process.off("SIGINT", stop); process.off("SIGTERM", stop);
  }
}

function nextSlot(out, runs) {
  let run = 1;
  while (run <= runs && fs.existsSync(path.join(out, `run${run}`))) run++;
  if (run > runs) throw Error(`All ${runs} run slots are used. Use a new --out for a new experiment.`);
  if (run > 1) {
    const previous = path.join(out, `run${run - 1}`);
    if (!fs.existsSync(path.join(previous, "result.json")))
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

async function launch(out) {
  const armed = read(path.join(out, "watch.json"));
  process.kill(armed.pid, 0); // Existence check only; signal 0 sends no signal.
  const runs = armed.runs || RUNS;
  const run = nextSlot(out, runs);
  const snapshots = manifests().map(file => ({ file: relative(file), manifest: read(file) }));
  const descriptor = read(path.join(ROOT, snapshots[0].manifest.sharedEnvironment.descriptor));
  if (descriptor.robots.map(r => r.name).join(",") !== names(CELL).join(","))
    throw Error("Generate the five-robot cell first: its descriptor has a different robot list.");
  const directory = path.join(out, `run${run}`);
  fs.mkdirSync(directory); // Exclusive slot; never overwrite a trial.
  const trace = path.join(directory, "events.jsonl");
  fs.writeFileSync(trace, "");
  write(path.join(directory, "inputs.json"), { manifests: snapshots, descriptor });
  const meta = { cell: CELL, run, runs, t: Date.now(), jcm: armed.jcm };
  write(path.join(directory, "run.json"), meta);
  append(trace, { event: "gradle_start", t: meta.t });
  console.log(`Starting Gradle for run ${run}/${runs}. Send !deliverCube once the route is settled; stop Gradle after the recorder prints the result.`);
  const { exit } = await runGradle({ jcm: armed.jcm, directory, trace, env: { WOMAT_E11_CELL: String(CELL) } });
  write(path.join(directory, "exit.json"), exit);
}

// The same !deliverCube the user would type into the Jason console, sent over
// the AgentThing so that the request has a recorded timestamp. The transfer is
// timed from the agent's own delivery_start event, not from this call.
async function deliver(out, runs, agent) {
  const directory = activeSlot(out, runs);
  // Each robot's AgentThing listens on its own port and serves only its own
  // agent, so --agent has to pick the matching manifest, not the first one.
  const file = manifests().find(m => agentName(path.basename(path.dirname(m))) === agent);
  if (!file) throw Error(`${agent} is not one of ${names(CELL).map(agentName).join(", ")}`);
  const manifest = read(file);
  const url = `http://localhost:${manifest.ports.agentThing}/${agent}/actions/deliverCube`;
  const response = await fetch(url, { method: "POST", headers: { "content-type": "application/json" }, body: "{}" });
  const text = await response.text();
  append(path.join(directory, "events.jsonl"), { event: "delivery_requested", t: Date.now(), agent, url, status: response.status, body: text.slice(0, 500) });
  if (!response.ok) throw Error(`${agent} refused !deliverCube (HTTP ${response.status}): ${text.slice(0, 200)}`);
  console.log(`Requested !deliverCube at ${agent} for ${relative(directory)}.`);
}

async function main() {
  const command = process.argv[2], args = parseArgs(process.argv.slice(3));
  const out = path.resolve(ROOT, args.out || OUT);
  const runs = Number(args.runs || RUNS);
  if (!Number.isInteger(runs) || runs < 1) throw Error("--runs must be a positive integer");
  if (!command || command === "help" || command === "--help") {
    console.log("E12: watch [--runs 10] [--timeout 2400] | launch | deliver [--agent robot1_agent] | verify | analyze | resummarize\n"
      + "Optional --out DIR (default performance_log/e12).\n"
      + "watch never starts or contacts robots; launch starts only Gradle; deliver sends only !deliverCube.\n"
      + "Route discovery is measured exactly as in E11; handovers and the transport are added.\n"
      + "See docs/e12_principal.md.");
    return;
  }
  if (command === "analyze") { console.table(analyze(out, runs)); return; }
  if (command === "resummarize") { resummarize(out, runs); console.table(analyze(out, runs)); return; }
  if (command === "verify") { preparePlans(out); const v = verifyPlans(out); console.log(v.skipped ? `Skipped: ${v.skipped}` : `Parse-checked ${v.checked} instrumented agent files.`); return; }
  if (command === "watch") {
    const timeout = Number(args.timeout || 2400);
    if (!Number.isFinite(timeout) || timeout <= 0) throw Error("--timeout must be positive seconds");
    return watch(out, runs, timeout);
  }
  if (command === "launch") return launch(out);
  if (command === "deliver") return deliver(out, runs, args.agent || "robot1_agent");
  throw Error("Unknown E12 command: " + command);
}

if (require.main === module) main().catch(error => { console.error(error.message); process.exitCode = 1; });

module.exports = { instrumentHandovers, instrumentRoutesWithLegs, instrumentAgent, preparePlans, verifyPlans,
  pairPhases, handovers, summarize, decompose, parseOptions, pricedPerHop, analyze, results, resummarize,
  handoverTable, costTable, timelineFigure, HANDOVER_PHASES, STARTUP_PHASES, CELL, RUNS, agentName, names, manifests, OUT };
