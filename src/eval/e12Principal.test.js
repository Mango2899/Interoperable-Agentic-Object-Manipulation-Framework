"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("fs");
const os = require("os");
const path = require("path");
const {
  summarize, decompose, parseOptions, pricedPerHop, instrumentHandovers, instrumentRoutesWithLegs,
  handoverTable, costTable, timelineFigure, analyze, HANDOVER_PHASES
} = require("./e12Principal");

// The legs of the recorded five-robot run, to full precision. Every cumulative
// value the agents announced is a sum of these six numbers, which is what makes
// them usable as a fixture: if the analysis reproduces the announcements it is
// decomposing the route the same way the cell composed it.
const LEGS = [
  { agent: "robot1_agent", neighbour: "robot2", mode: "pickup", d: 0.6767385133318435, candidate: "candidate_26" },
  { agent: "robot2_agent", neighbour: "robot1", mode: "pose", d: 0.5179844344010257, candidate: "candidate_26" },
  { agent: "robot4_agent", neighbour: "robot2", mode: "pose", d: 0.9036097523538572, candidate: "candidate_59" },
  { agent: "robot3_agent", neighbour: "robot2", mode: "pose", d: 1.1298527271823458, candidate: "candidate_24" },
  { agent: "robot5_agent", neighbour: "robot4", mode: "pose", d: 1.9514085713745684, candidate: "candidate_31" },
  { agent: "robot5_agent", neighbour: "robot3", mode: "pose", d: 1.9119780994975764, candidate: "candidate_55" }
];
const REFUSED = [
  { agent: "robot1_agent", neighbour: "robot4", reason: "voxel_intersection_empty" },
  { agent: "robot1_agent", neighbour: "robot3", reason: "interaction_space_below_minimum" },
  { agent: "robot5_agent", neighbour: "robot2", reason: "interaction_space_below_minimum" }
];
const RANKED = "[total(4.049741271461295,robot2_agent,robot2,[robot2,robot4,robot5]),"
  + "total(4.236553774412791,robot2_agent,robot2,[robot2,robot3,robot5])]";
// The three stopwatch readings, as milliseconds of handover interval.
const HOPS = [
  { giver: "robot1", receiver: "robot2", ms: 11300 },
  { giver: "robot2", receiver: "robot4", ms: 10600 },
  { giver: "robot4", receiver: "robot5", ms: 11400 }
];

function fixture({ dropRetreat = false } = {}) {
  const meta = { cell: 5, run: 1, runs: 10, t: 1_000_000 };
  const events = [{ event: "gradle_start", t: meta.t }];
  let clock = 0;
  const add = (event, agent, phase, data = [], step = 1) => {
    clock += step;
    events.push({ t: meta.t + clock, mono_ms: 500_000 + clock, agent, event, phase, data });
  };
  for (let i = 1; i <= 5; i++) {
    const agent = `robot${i}_agent`;
    for (const phase of ["neighbour_discovery", "interaction_space", "task_sites", "initial_edge_pricing"]) {
      add("phase_start", agent, phase);
      add("phase_end", agent, phase, [], 40);
    }
    add("agent_ready", agent, "startup");
  }
  add("phase_start", "robot1_agent", "route_discovery", ["v_Robot5_grip_support_b"]);
  for (const leg of LEGS) add("edge_priced", leg.agent, "edge_cost", [leg.neighbour, leg.mode, String(leg.d), leg.candidate]);
  for (const edge of REFUSED) add("edge_refused", edge.agent, "edge_cost", [edge.neighbour, "pose", edge.reason]);
  add("route_options", "robot1_agent", "route_discovery", ["route(robot1,b,0)", "2", RANKED]);
  add("route_end", "robot1_agent", "route_discovery",
    ["success", "route(robot1,b,0)", "4.049741271461295", "[robot1,robot2,robot4,robot5]"], 3000);

  add("delivery_start", "robot1_agent", "transport", ["robot1", "robot2", "4.049741271461295"], 1000);
  add("phase_start", "robot1_agent", "pickup", ["robot1"]);
  add("phase_end", "robot1_agent", "pickup", ["robot1"], 4000);

  for (const [index, hop] of HOPS.entries()) {
    const session = `session_${index + 1}`;
    const giver = hop.giver + "_agent", receiver = hop.receiver + "_agent";
    add("phase_start", giver, "handover_plan", [hop.receiver]);
    add("phase_end", giver, "handover_plan", [hop.receiver], 900);
    add("candidate_selected", giver, "handover", [hop.receiver, "candidate_26", "0.71", "12"]);
    add("handover_start", giver, "handover", [session, hop.giver, hop.receiver]);
    const opened = clock;
    add("phase_start", giver, "giver_approach", [session]);
    add("phase_end", giver, "giver_approach", [session], 2500);
    add("phase_start", receiver, "receiver_approach", [session]);
    add("phase_end", receiver, "receiver_approach", [session], 3000);
    add("phase_start", giver, "transfer", [session]);
    add("phase_end", giver, "transfer", [session], 2000);
    add("phase_start", receiver, "receiver_retreat_motion", [session]);
    // Close the interval exactly on the stopwatch definition.
    clock = opened + hop.ms - 1;
    add("phase_end", receiver, "receiver_retreat_motion", [session]);
    if (!dropRetreat) add("receiver_retreated", receiver, "handover", [session, hop.receiver, hop.giver], 0);
    add("handover_end", giver, "handover", [session, hop.giver, hop.receiver], 60);
    add("phase_start", giver, "giver_home", [session]);
    add("phase_end", giver, "giver_home", [session], 3000);
  }
  add("phase_start", "robot5_agent", "placement", ["v_Robot5_grip_support_b"], 500);
  add("phase_end", "robot5_agent", "placement", ["v_Robot5_grip_support_b"], 5000);
  add("cube_placed", "robot5_agent", "transport", ["v_Robot5_grip_support_b", "robot5"]);
  return { meta, events };
}

test("the six priced legs reproduce every cumulative value the agents announced", () => {
  const { meta, events } = fixture();
  const result = summarize(meta, events);
  assert.equal(result.status, "ok");
  assert.deepEqual(result.issues, []);
  const selected = decompose(["robot1", "robot2", "robot4", "robot5"], result.legs);
  const alternative = decompose(["robot1", "robot2", "robot3", "robot5"], result.legs);
  assert.equal(selected.total_m, 4.049741271461295);
  assert.equal(alternative.total_m, 4.236553774412791);
  assert.deepEqual(selected.rows.map(r => r.distance_m),
    [0.6767385133318435, 0.5179844344010257, 0.9036097523538572, 1.9514085713745684]);
  // The pickup leg is priced once and shared by both routes; the alternative
  // must not silently re-price it from the wrong agent.
  assert.equal(alternative.rows[0].distance_m, selected.rows[0].distance_m);
  assert.equal(selected.rows[2].candidate, "candidate_59");
});

test("route options are parsed and ranked cheapest first", () => {
  const options = parseOptions(RANKED);
  assert.equal(options.length, 2);
  assert.deepEqual(options[0].path, ["robot2", "robot4", "robot5"]);
  assert.equal(options[0].cost_m, 4.049741271461295);
  assert.ok(options[0].cost_m < options[1].cost_m);
});

test("handover duration is the receiver's retreat minus the giver's first motion", () => {
  const { meta, events } = fixture();
  const result = summarize(meta, events);
  const hops = pricedPerHop(result);
  assert.deepEqual(hops.map(h => h.duration_ms), [11300, 10600, 11400]);
  // The giver only learns of the retreat by message; that hop is excluded from
  // the interval and reported separately.
  assert.deepEqual(hops.map(h => h.giver_observed_ms), [11360, 10660, 11460]);
  assert.deepEqual(hops.map(h => [h.giver, h.receiver]),
    [["robot1", "robot2"], ["robot2", "robot4"], ["robot4", "robot5"]]);
  // Metres of Eq. (route_cost) attributable to each hop: the first carries the
  // pickup leg as well as the receiver's approach.
  assert.deepEqual(hops.map(h => h.priced_m),
    [0.6767385133318435 + 0.5179844344010257, 0.9036097523538572, 1.9514085713745684]);
  assert.equal(result.transport.handover_total_ms, 33300);
  // Sub-phases are per session and must not leak across hops.
  assert.deepEqual(hops.map(h => h.giver_approach_ms), [2500, 2500, 2500]);
  assert.deepEqual(hops.map(h => h.handover_plan_ms), [900, 900, 900]);
});

test("delivery covers more than the handovers, and the remainder is the unpriced motion", () => {
  const { meta, events } = fixture();
  const result = summarize(meta, events);
  assert.equal(result.transport.source_pickup_ms, 4000);
  assert.equal(result.transport.placement_ms, 5000);
  assert.ok(result.transport.delivery_ms > result.transport.handover_total_ms);
  assert.equal(result.transport.unpriced_ms, result.transport.delivery_ms - 33300);
  assert.equal(result.transport.sink, "robot5");
});

test("refused edges are kept apart from priced ones and counted", () => {
  const { meta, events } = fixture();
  const result = summarize(meta, events);
  assert.equal(result.legs.length, 6);
  assert.equal(result.refused_edges.length, 3);
  assert.deepEqual(result.refused_edges.map(e => e.reason).sort(),
    ["interaction_space_below_minimum", "interaction_space_below_minimum", "voxel_intersection_empty"]);
  assert.equal(result.route.options_reported, 2);
});

test("an unfinished handover is reported as an issue rather than a duration of zero", () => {
  const { meta, events } = fixture({ dropRetreat: true });
  const result = summarize(meta, events);
  assert.equal(result.status, "incomplete");
  assert.equal(result.handovers.length, 3);
  assert.ok(result.handovers.every(h => h.duration_ms === null));
  assert.ok(result.handovers.every(h => h.giver_observed_ms !== null));
  assert.ok(result.issues.some(i => /never completed/.test(i)));
  assert.equal(result.transport.handover_total_ms, null);
});

test("instrumentation applies to the production plans and only adds probes", () => {
  const handover = fs.readFileSync(path.join(__dirname, "../agt/handoverPlans.asl"), "utf8");
  const route = fs.readFileSync(path.join(__dirname, "../agt/routePlans.asl"), "utf8");
  const instrumentedHandover = instrumentHandovers(handover);
  const instrumentedRoute = instrumentRoutesWithLegs(route);
  for (const [before, after] of [[handover, instrumentedHandover], [route, instrumentedRoute]]) {
    assert.ok(after.length > before.length);
    // Every original line survives; instrumentation adds probes and never
    // rewrites behaviour. The one exception is the plan terminator of
    // placeCubeAt, which has to become `;` so the cube_placed probe can be the
    // plan's last formula.
    const removed = before.split("\n").filter(line => line.trim() && !after.includes(line));
    assert.deepEqual(removed, before === handover ? ['    .print("Cube placed on ", Goal, " by ", MyRobot).'] : []);
    for (const line of removed) assert.ok(after.includes(line.replace(/\.$/, ";")));
  }
  assert.ok(instrumentedHandover.includes("e11.event(handover_start, handover, Session, MyRobot, ReceiverRobot)"));
  assert.ok(instrumentedRoute.includes("e11.event(edge_priced, edge_cost, Neighbour, Mode, BestDistance, BestCandidate)"));
  // A reworded anchor must fail loudly, not silently skip a boundary.
  assert.throws(() => instrumentHandovers(handover.replace("    !transferCubeTo(Session, ReceiverRobot);", "    !transferCubeTo(Session, R);")),
    /Instrumentation anchor changed/);
});

test("analyze writes the thesis tables from the runs and keeps failed runs out of them", () => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), "e12-"));
  const first = fixture();
  const good = summarize(first.meta, first.events);
  fs.mkdirSync(path.join(out, "run1"), { recursive: true });
  fs.writeFileSync(path.join(out, "run1", "result.json"), JSON.stringify(good));
  const second = fixture({ dropRetreat: true });
  const bad = summarize({ ...second.meta, run: 2 }, second.events);
  fs.mkdirSync(path.join(out, "run2"), { recursive: true });
  fs.writeFileSync(path.join(out, "run2", "result.json"), JSON.stringify({ ...bad, run: 2 }));
  const rows = analyze(out, 10);
  assert.equal(rows.length, 2);
  const handovers = fs.readFileSync(path.join(out, "e12_handover_table.tex"), "utf8");
  assert.match(handovers, /1 successful runs/);
  assert.match(handovers, /Robot~1 \$\\rightarrow\$ Robot~2 & 1\.195 &/);
  assert.match(handovers, /11\.30/);
  const costs = fs.readFileSync(path.join(out, "e12_cost_table.tex"), "utf8");
  assert.match(costs, /Cumulative cost & 4\.050 & 4\.237/);
  assert.match(costs, /Pickup leg of Robot~1 \(to Robot~2\) & 0\.677 & 0\.677/);
  const timeline = fs.readFileSync(path.join(out, "e12_figure_timeline.tex"), "utf8");
  assert.match(timeline, /tikzpicture/);
  assert.match(timeline, /Robot~5/);
  // The per-hop file keeps the failed run's hops too -- that is what a failure
  // has to be diagnosed from -- while the aggregated tables use only valid runs.
  const csvText = fs.readFileSync(path.join(out, "e12_handovers.csv"), "utf8");
  assert.equal(csvText.trim().split("\n").length, 7); // header plus three hops from each of the two runs
  assert.match(fs.readFileSync(path.join(out, "e12_runs.csv"), "utf8"), /"incomplete"/);
  fs.rmSync(out, { recursive: true, force: true });
});

test("a refused rendezvous abandons its attempt instead of failing the run", () => {
  const { meta, events } = fixture();
  // Reproduce run 3: the giver's rendezvous with robot5 is refused mid-approach,
  // it strikes the candidate and opens a second interval to the same receiver.
  const third = events.findIndex(e => e.event === "handover_start" && e.data[2] === "robot5");
  const opened = events[third];
  const refusal = { t: opened.t + 400, mono_ms: opened.mono_ms + 400, agent: "robot4_agent",
    event: "candidate_refused", phase: "handover", data: ["candidate_31", "robot5"] };
  const reselect = { t: opened.t + 500, mono_ms: opened.mono_ms + 500, agent: "robot4_agent",
    event: "candidate_selected", phase: "handover", data: ["robot5", "candidate_38", "0.82", "59"] };
  const retry = { ...opened, t: opened.t + 600, mono_ms: opened.mono_ms + 600,
    data: ["session_3b", "robot4", "robot5"] };
  // The abandoned attempt never closes; everything after it belongs to the retry.
  const rest = events.slice(third + 1).map(e => Array.isArray(e.data) && e.data[0] === "session_3"
    ? { ...e, data: ["session_3b", ...e.data.slice(1)] } : e);
  const patched = [...events.slice(0, third + 1), refusal, reselect, retry, ...rest];

  const result = summarize(meta, patched);
  assert.deepEqual(result.issues, []);
  assert.equal(result.status, "ok");
  assert.equal(result.handovers.length, 4);
  const abandoned = result.handovers.filter(h => h.abandoned);
  assert.equal(abandoned.length, 1);
  assert.equal(abandoned[0].refused_candidate, "candidate_31");
  assert.equal(abandoned[0].hop, null);
  // The three attempts that carried the cube are still numbered 1..3, and the
  // abandoned one contributes nothing to the total.
  assert.deepEqual(result.handovers.filter(h => !h.abandoned).map(h => h.hop), [1, 2, 3]);
  assert.equal(result.transport.handover_total_ms,
    result.handovers.filter(h => !h.abandoned).reduce((sum, h) => sum + h.duration_ms, 0));
  // The retry acted on the replacement candidate, not the refused original.
  assert.equal(pricedPerHop(result).find(h => h.hop === 3).executed_candidate, "candidate_38");
});
