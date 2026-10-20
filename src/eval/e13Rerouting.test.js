"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("fs");
const os = require("os");
const path = require("path");
const {
  summarize, analyze, reroutes, executedPath, markPartnerFailures, armOfRun, plannedSchedule,
  instrumentRouteRecovery, instrumentPartnerFailure, armTable, rerouteTable, costTable, budgetFigure,
  results, discard, nextSlot, recoveries, recoveryTable, deliveryFigure, RECOVERY_MARKERS,
  ARMS, PER_ARM, RUNS, SCHEMA, FAILED_AGENT
} = require("./e13Rerouting");
const { handovers } = require("./e12Principal");

// The legs of the recorded five-robot cell, to full precision -- the same
// fixture E12 uses, because E13 has to decompose the same two routes the same
// way or the two experiments' cost tables cannot be put side by side.
const LEGS = [
  { agent: "robot1_agent", neighbour: "robot2", mode: "pickup", d: 0.6767385133318435, candidate: "candidate_26" },
  { agent: "robot2_agent", neighbour: "robot1", mode: "pose", d: 0.5179844344010257, candidate: "candidate_26" },
  { agent: "robot4_agent", neighbour: "robot2", mode: "pose", d: 0.9036097523538572, candidate: "candidate_59" },
  { agent: "robot3_agent", neighbour: "robot2", mode: "pose", d: 1.1298527271823458, candidate: "candidate_24" },
  { agent: "robot5_agent", neighbour: "robot4", mode: "pose", d: 1.9514085713745684, candidate: "candidate_31" },
  { agent: "robot5_agent", neighbour: "robot3", mode: "pose", d: 1.9119780994975764, candidate: "candidate_55" }
];
const VIA_4 = 4.049741271461295, VIA_3 = 4.236553774412791;
const RANKED = `[total(${VIA_4},robot2_agent,robot2,[robot2,robot4,robot5]),`
  + `total(${VIA_3},robot2_agent,robot2,[robot2,robot3,robot5])]`;

// A trace of one run, built the way the JaCaMo JVM writes them: one clock, one
// event per line, monotonic milliseconds. `arm` decides which story it tells.
function fixture({ arm, discoveryMs = 3000, deadline = null, detectedBy = "timeout",
  detectMs = 120000, decideMs = 4, resetMs = 9000, killAfterRoute = true } = {}) {
  const running = ["robot1_agent", "robot2_agent", "robot3_agent", "robot4_agent", "robot5_agent"]
    .filter(a => arm !== "cold" || a !== FAILED_AGENT);
  const meta = { cell: 5, run: arm === "cold" ? 1 : 6, runs: RUNS, arm, t: 1_000_000 };
  const events = [{ event: "gradle_start", t: meta.t, arm }];
  let clock = 0;
  const add = (event, agent, phase, data = [], step = 1) => {
    clock += step;
    const row = { t: meta.t + clock, mono_ms: 500_000 + clock, agent, event, phase, data };
    events.push(row);
    return row;
  };
  const message = (from, to, step = 1) => {
    clock += step;
    events.push({ t: meta.t + clock, mono_ms: 500_000 + clock, agent: from, event: "message",
      receiver: to, performative: "tell", content: "routeCost", message_id: "m" + clock });
  };

  for (const agent of running) {
    for (const phase of ["neighbour_discovery", "interaction_space", "task_sites", "initial_edge_pricing"]) {
      add("phase_start", agent, phase);
      add("phase_end", agent, phase, [], 40);
    }
    add("agent_ready", agent, "startup");
  }

  add("phase_start", "robot1_agent", "route_discovery", ["v_Robot5_grip_support_b"]);
  for (const leg of LEGS) add("edge_priced", leg.agent, "edge_cost", [leg.neighbour, leg.mode, String(leg.d), leg.candidate]);
  for (const agent of running) message(agent, "robot2_agent");
  // In the cold arm every arm that has Robot4 as a neighbour fails its send to
  // it and answers on its behalf, which is what keeps the rest of its own list
  // getting asked. Robot1 asks [robot4,robot3,robot2] in that order, so without
  // this the search would die before Robot3 and Robot2 were ever reached.
  if (arm === "cold") for (const asker of ["robot1_agent", "robot2_agent", "robot3_agent"]) {
    add("peer_unreachable", asker, "route_discovery", [FAILED_AGENT, "route(robot1,b,0)"]);
  }
  // The cold arm ends on its deadline, with the absent agent named as the branch
  // that never answered. The kill arm ends on a complete tally.
  if (deadline) add("route_deadline", deadline.agent, "route_discovery", [deadline.task, `[${deadline.silent.join(",")}]`], discoveryMs - 10);
  const chosen = arm === "cold" ? [VIA_3, "[robot1,robot2,robot3,robot5]"] : [VIA_4, "[robot1,robot2,robot4,robot5]"];
  add("route_options", "robot1_agent", "route_discovery", ["route(robot1,b,0)", "2", RANKED]);
  add("route_end", "robot1_agent", "route_discovery",
    ["success", "route(robot1,b,0)", String(chosen[0]), chosen[1]], deadline ? 10 : discoveryMs);

  if (arm === "kill") add("agent_killed", null, null, [], killAfterRoute ? 5000 : -1);
  add("delivery_start", "robot1_agent", "transport", ["robot1", "robot2", String(chosen[0])], 1000);
  add("phase_start", "robot1_agent", "pickup", ["robot1"]);
  add("phase_end", "robot1_agent", "pickup", ["robot1"], 4000);

  // One completed hop, giver -> receiver, as the instrumented plans emit it.
  const hop = (giver, receiver, session, durationMs) => {
    const giverAgent = giver + "_agent", receiverAgent = receiver + "_agent";
    add("phase_start", giverAgent, "handover_plan", [receiver]);
    add("phase_end", giverAgent, "handover_plan", [receiver], 900);
    add("candidate_selected", giverAgent, "handover", [receiver, "candidate_26", "0.71", "12"]);
    add("handover_start", giverAgent, "handover", [session, giver, receiver]);
    add("phase_start", giverAgent, "giver_approach", [session]);
    add("phase_end", giverAgent, "giver_approach", [session], Math.round(durationMs * 0.35));
    add("phase_start", receiverAgent, "receiver_approach", [session]);
    add("phase_end", receiverAgent, "receiver_approach", [session], Math.round(durationMs * 0.25));
    add("phase_start", giverAgent, "transfer", [session]);
    add("phase_end", giverAgent, "transfer", [session], Math.round(durationMs * 0.2));
    add("phase_start", receiverAgent, "receiver_retreat_motion", [session]);
    add("phase_end", receiverAgent, "receiver_retreat_motion", [session], Math.round(durationMs * 0.2));
    add("receiver_retreated", receiverAgent, "handover", [session, receiver, giver]);
    add("handover_end", giverAgent, "handover", [session, giver, receiver]);
  };

  hop("robot1", "robot2", "session_1", 11300);

  if (arm === "kill") {
    // The attempt that was chosen and then failed: the giver plans it, moves to
    // the rendezvous, and nobody arrives.
    add("phase_start", "robot2_agent", "handover_plan", ["robot4"]);
    add("phase_end", "robot2_agent", "handover_plan", ["robot4"], 900);
    add("candidate_selected", "robot2_agent", "handover", ["robot4", "candidate_59", "0.68", "17"]);
    add("handover_start", "robot2_agent", "handover", ["session_2", "robot2", "robot4"]);
    add("phase_start", "robot2_agent", "giver_approach", ["session_2"], 0);
    add("phase_end", "robot2_agent", "giver_approach", ["session_2"], 4200);
    add("phase_start", "robot2_agent", "wait_receiver", ["session_2"], 0);
    // Detection: measured from the giver's first commanded motion, so the wait
    // deadline is most of it and the approach it wasted is the rest.
    add("phase_end", "robot2_agent", "wait_receiver", ["session_2"], detectMs - 4200);
    add("partner_unfit", "robot2_agent", "reroute",
      ["robot4_agent", detectedBy === "timeout" ? "silent" : "receiver_failed", detectedBy], 0);
    // The teardown tell to the arm being left behind. It happens BEFORE the
    // decision, which is why the two message counts are kept apart.
    message("robot2_agent", "robot4_agent");
    // One tick after the teardown tell, because that is the order they happen in:
    // a tie here is the one case where the two message counts could disagree
    // about which side of the boundary a message fell on.
    add("phase_start", "robot2_agent", "reroute_decide", ["robot4"], 1);
    add("hop_struck", "robot2_agent", "reroute", ["robot4", "2.855142558141388"], 0);
    add("hop_replaced", "robot2_agent", "reroute", ["robot3", "3.0418353001721057", "1", "[robot3,robot5]"], 0);
    add("phase_end", "robot2_agent", "reroute_decide", ["robot4"], decideMs);
    // And the struck arm travelling forward with the cube, after the decision.
    message("robot2_agent", "robot3_agent");
    add("phase_start", "robot2_agent", "reroute_reset", ["robot4"], 0);
    add("phase_end", "robot2_agent", "reroute_reset", ["robot4"], resetMs);
  }

  hop("robot2", "robot3", "session_3", 12100);
  hop("robot3", "robot5", "session_4", 11900);

  add("phase_start", "robot5_agent", "placement", ["v_Robot5_grip_support_b"]);
  add("phase_end", "robot5_agent", "placement", ["v_Robot5_grip_support_b"], 6000);
  add("cube_placed", "robot5_agent", "transport", ["v_Robot5_grip_support_b", "robot5"]);
  return { meta, events };
}

const cold = () => fixture({ arm: "cold", discoveryMs: 246000,
  deadline: { agent: "robot2_agent", task: "route(robot1,b,0)", silent: ["robot4_agent"] } });
const kill = (options = {}) => fixture({ arm: "kill", ...options });

// ------------------------------------------------------------- instrumentation

test("every E13 anchor is still in the production plans, and the cold MAS drops one agent", () => {
  const route = fs.readFileSync(path.resolve(__dirname, "../agt/routePlans.asl"), "utf8");
  const handover = fs.readFileSync(path.resolve(__dirname, "../agt/handoverPlans.asl"), "utf8");
  // replaceOnce throws on a moved or duplicated anchor, which is the point: a
  // reworded plan must stop the experiment here and not halfway through a run
  // the user has set the cell up for.
  const instrumentedRoute = instrumentRouteRecovery(route);
  const instrumentedHandover = instrumentPartnerFailure(handover);
  for (const probe of ["route_deadline", "hop_struck", "hop_replaced", "reroute_exhausted"]) {
    assert.match(instrumentedRoute, new RegExp(`e11\\.event\\(${probe},`), `missing ${probe}`);
  }
  for (const probe of ["partner_unfit", "reroute_decide", "reroute_reset"]) {
    assert.match(instrumentedHandover, new RegExp(probe), `missing ${probe}`);
  }
  // The silent branches are named by the arm that gave up waiting, not inferred.
  assert.match(instrumentedRoute, /\.setof\(A, routeAskedPeer\(Task, A\) & not routeAnswer\(Task, A, _, _\), Silent\)/);
  // E12's boundaries survive: the two experiments must time discovery and the
  // handovers identically or their numbers cannot be compared.
  assert.match(instrumentedRoute, /e11\.event\(route_end, route_discovery, success, Task, Best/);
  assert.match(instrumentedHandover, /e11\.event\(handover_start, handover, Session, MyRobot, ReceiverRobot\)/);
  // The reset boundary belongs to the reroute's own call, not the chain's.
  const chainCall = instrumentedHandover.indexOf("    !prepareToForwardCube;\n    +handoverStage(Session, forwarding_cube)");
  assert.ok(chainCall > 0, "the ordinary chain still forwards without a reroute_reset boundary");
});

test("the two arms differ only in which agents reason", () => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), "e13-plans-"));
  const { preparePlans } = require("./e13Rerouting");
  const jcm = preparePlans(out);
  const cold = fs.readFileSync(jcm.cold, "utf8"), full = fs.readFileSync(jcm.kill, "utf8");
  assert.ok(!cold.includes(`agent ${FAILED_AGENT}:`), "the cold arm has no Robot4 agent");
  assert.ok(full.includes(`agent ${FAILED_AGENT}:`), "the kill arm has all five");
  // The robot is still in the cell in both arms. Dropping its workspace too
  // would make the cold arm a four-robot cell, which is a different experiment:
  // every other arm would stop finding Robot4 by reach and stop pricing the edge
  // to it, and the route the cold arm failed to find would not exist to fail.
  for (const declaration of [cold, full]) assert.match(declaration, /workspace robot4_workspace/);
  assert.equal((cold.match(/^    agent /gm) || []).length, 4);
  assert.equal((full.match(/^    agent /gm) || []).length, 5);
  fs.rmSync(out, { recursive: true, force: true });
});

// -------------------------------------------------------------------- analysis

test("a kill run plans one route and executes another", () => {
  const { meta, events } = kill();
  const result = summarize(meta, events);
  assert.deepEqual(result.issues, []);
  assert.equal(result.status, "ok");
  assert.deepEqual(result.route.path, ["robot1", "robot2", "robot4", "robot5"]);
  assert.deepEqual(result.route.executed_path, ["robot1", "robot2", "robot3", "robot5"]);
  assert.equal(result.route.rerouted, true);
  // Both costs are decomposed from the legs the run itself priced, so the
  // penalty is the difference between two measured routes and not an arithmetic
  // claim about them.
  assert.ok(Math.abs(result.route.cost_m - VIA_4) < 1e-9);
  assert.ok(Math.abs(result.route.executed_cost_m - VIA_3) < 1e-9);
  assert.ok(Math.abs(result.route.penalty_m - (VIA_3 - VIA_4)) < 1e-9);
  // The cube never went near the arm that failed.
  assert.ok(!result.route.executed_path.some(r => r === "robot4"));
});

test("the failure is broken into the part that is free and the parts that are not", () => {
  const { meta, events } = kill({ detectMs: 120000, decideMs: 6, resetMs: 9000 });
  const [detour] = summarize(meta, events).reroutes;
  assert.equal(detour.at_robot, "robot2");
  assert.equal(detour.failed_robot, "robot4");
  assert.equal(detour.detected_by, "timeout");
  // Detection runs from the giver's first commanded motion towards a rendezvous
  // nobody came to, so it contains the approach as well as the deadline.
  assert.equal(detour.detect_ms, 120000);
  assert.equal(detour.wasted_approach_ms, 4200);
  // The contribution: re-deciding is arithmetic over answers already held.
  assert.equal(detour.decide_ms, 6);
  assert.ok(detour.decide_ms < 100, "a local re-decision cannot take a tenth of a second");
  assert.equal(detour.reset_ms, 9000);
  assert.equal(detour.struck_robot, "robot4");
  assert.equal(detour.replacement_robot, "robot3");
  assert.equal(detour.options_left, 1);
  assert.equal(detour.exhausted, false);
  // The metres the re-deciding arm itself gave up, which is the local view of
  // the same penalty the route-level figure reports end to end.
  assert.ok(Math.abs(detour.penalty_m - (3.0418353001721057 - 2.855142558141388)) < 1e-9);
  // Recovery ends when the giver starts moving on the replacement hop, so it
  // contains the reset but not the new handover.
  assert.ok(detour.recover_ms > detour.reset_ms);
  assert.ok(detour.recover_ms < detour.detect_ms);
});

test("no messages are needed to re-decide, and the count says so rather than the comment", () => {
  const { meta, events } = kill();
  const [detour] = summarize(meta, events).reroutes;
  // The alternatives were priced by the search that chose the route and came
  // home inside the same replies, so nothing at all crosses the decision.
  assert.equal(detour.messages_during, 0);
  // The teardown is counted separately and is NOT nil: the arm being left behind
  // is told the handover is off. Folding it into the decision would make the
  // headline claim look false; leaving it out of the data would make it look
  // truer than it is.
  assert.equal(detour.messages_teardown, 1);
});

test("the abandoned approach is not counted as a handover that failed", () => {
  const { meta, events } = kill();
  const result = summarize(meta, events);
  const abandoned = result.handovers.filter(h => h.abandoned);
  assert.equal(abandoned.length, 1);
  assert.equal(abandoned[0].receiver, "robot4");
  assert.equal(abandoned[0].abandoned_reason, "partner_unfit");
  assert.equal(abandoned[0].hop, null, "an abandoned attempt is not a hop");
  // And the carried hops are numbered as if it had never happened.
  assert.deepEqual(result.handovers.filter(h => !h.abandoned).map(h => h.hop), [1, 2, 3]);
  // Its motion is still counted, because the cell really performed it.
  assert.equal(result.transport.wasted_ms, 4200 + 9000);
});

test("a cold run ends on the deadline, and says which branch it waited for", () => {
  const { meta, events } = cold();
  const result = summarize(meta, events);
  assert.deepEqual(result.issues, []);
  assert.equal(result.status, "ok");
  assert.equal(result.convergence.ended_on, "deadline");
  assert.deepEqual(result.convergence.silent_branches, ["robot4_agent"]);
  // The intervention is observed, not assumed: the absent agent is named by each
  // arm that tried to ask it. A cold run in which nobody reports that is a run
  // whose MAS still had the agent, whatever the label said.
  assert.deepEqual(result.convergence.unaskable_peers, [FAILED_AGENT]);
  // Robot4's AgentThing was up, so the edge to it was still priced and the cost
  // table is complete even though its agent never reasoned.
  assert.ok(result.convergence.failed_robot_priced_by.length > 0);
  // The cheap route was never found, so there is nothing to reroute around and
  // no motion is discarded. That is the whole difference between the arms.
  assert.deepEqual(result.reroutes, []);
  assert.equal(result.route.rerouted, false);
  assert.equal(result.transport.wasted_ms, null);
  assert.deepEqual(result.route.path, result.route.executed_path);
  assert.ok(Math.abs(result.route.penalty_m) < 1e-9, "the route it planned is the route it walked");
  // Discovery is a deadline, not a convergence, and is much longer for it.
  assert.ok(result.durations_ms.route_discovery > 200000);
  // Four agents reasoning is not an incomplete cell in this arm.
  assert.ok(!result.issues.some(i => /initialized/.test(i)));
});

test("a run that did not test its arm's question is reported, not averaged in", () => {
  // A kill run whose route avoided Robot4 means the agent was killed before the
  // search settled: it measured the cold arm's question under the kill arm's
  // label, which is worse than a missing run.
  const early = fixture({ arm: "kill" });
  early.events = early.events.map(e => e.event === "route_end"
    ? { ...e, data: ["success", "route(robot1,b,0)", String(VIA_3), "[robot1,robot2,robot3,robot5]"] } : e);
  const result = summarize(early.meta, early.events);
  assert.equal(result.status, "incomplete");
  assert.ok(result.issues.some(i => /killed too early/.test(i)), result.issues.join("; "));

  // A cold run that rerouted means Robot4 answered the flood, so its agent was
  // running after all.
  const warm = cold();
  warm.events.push({ t: warm.meta.t + 1, mono_ms: 600_000, agent: "robot2_agent",
    event: "partner_unfit", phase: "reroute", data: ["robot4_agent", "silent", "timeout"] });
  const rerouted = summarize(warm.meta, warm.events);
  assert.ok(rerouted.issues.some(i => /cold run rerouted/.test(i)), rerouted.issues.join("; "));

  // And a cube that travelled through the arm that was supposed to be gone
  // invalidates either arm, whatever else the run looks like.
  const through = kill();
  through.events = through.events.map(e =>
    e.data?.[2] === "robot3" && e.event === "handover_start" ? { ...e, data: [e.data[0], e.data[1], "robot4"] } : e);
  assert.ok(summarize(through.meta, through.events).issues.some(i => /travelled through Robot4/.test(i)));
});

test("a cold run whose MAS still had the agent is caught", () => {
  // The label on a run is a claim; the trace is the evidence. Without the
  // absent agent showing up as an arm somebody could not ask, a cold run may
  // simply have been an ordinary five-agent run that happened to be slow.
  const { meta, events } = cold();
  const relabelled = events.filter(e => e.event !== "peer_unreachable");
  const result = summarize(meta, relabelled);
  assert.equal(result.status, "incomplete");
  assert.ok(result.issues.some(i => /may still have had it/.test(i)), result.issues.join("; "));
});

test("a reroute with nowhere left to go is a failed run", () => {
  const { meta, events } = kill();
  const stuck = events.filter(e => e.event !== "cube_placed");
  stuck.push({ t: meta.t + 999, mono_ms: 900_000, agent: "robot2_agent",
    event: "reroute_exhausted", phase: "reroute", data: ["robot2"] });
  const result = summarize(meta, stuck, "The reroute ran out of alternatives");
  assert.equal(result.status, "incomplete");
  assert.ok(result.reroutes[0].exhausted);
});

test("the kill is timestamped against the route decision, not trusted", () => {
  const good = summarize(...Object.values(kill({ killAfterRoute: true })).slice(0, 2));
  assert.ok(good.agent_killed_at_ms > 0);
  const { meta, events } = kill();
  assert.ok(events.find(e => e.event === "agent_killed"));
});

// ------------------------------------------------------------------- artifacts

test("the schedule fixes which arm each slot belongs to", () => {
  assert.equal(armOfRun(1), "cold");
  assert.equal(armOfRun(PER_ARM), "cold");
  assert.equal(armOfRun(PER_ARM + 1), "kill");
  assert.equal(armOfRun(RUNS), "kill");
  const schedule = plannedSchedule(RUNS);
  assert.equal(schedule.length, RUNS);
  for (const arm of ARMS) assert.equal(schedule.filter(s => s.arm === arm).length, PER_ARM);
});

test("the artifacts are generated from the runs and name both arms", () => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), "e13-"));
  const runs = [];
  for (let i = 0; i < PER_ARM; i++) runs.push({ ...cold(), run: i + 1 });
  for (let i = 0; i < PER_ARM; i++) runs.push({ ...kill(), run: PER_ARM + i + 1 });
  for (const entry of runs) {
    const directory = path.join(out, `run${entry.run}`);
    fs.mkdirSync(directory, { recursive: true });
    const meta = { ...entry.meta, run: entry.run };
    fs.writeFileSync(path.join(directory, "result.json"), JSON.stringify(summarize(meta, entry.events)));
  }
  const table = analyze(out, RUNS);
  assert.equal(table.length, RUNS);
  assert.ok(table.every(r => r.status === "ok"), JSON.stringify(table.filter(r => r.status !== "ok")));

  const runsCsv = fs.readFileSync(path.join(out, "e13_runs.csv"), "utf8");
  assert.match(runsCsv, /"arm"/);
  assert.match(runsCsv, /robot1-robot2-robot4-robot5/, "the planned route of the kill arm");
  assert.match(runsCsv, /robot1-robot2-robot3-robot5/, "and the route both arms executed");
  // One row per reroute, and only the kill arm has any.
  const reroutesCsv = fs.readFileSync(path.join(out, "e13_reroutes.csv"), "utf8").trim().split("\n");
  assert.equal(reroutesCsv.length - 1, PER_ARM);
  // The summary is per arm, because every headline number here is a comparison
  // and a pooled mean would hide the result.
  const summaryCsv = fs.readFileSync(path.join(out, "e13_summary.csv"), "utf8");
  for (const arm of ARMS) assert.ok(summaryCsv.includes(`"${arm}"`), `no ${arm} rows in the summary`);
  assert.match(summaryCsv, /route_discovery_ms/);
  assert.match(summaryCsv, /reroute_decide_ms/);

  const arms = fs.readFileSync(path.join(out, "e13_arm_table.tex"), "utf8");
  assert.match(arms, /Agent absent/);
  assert.match(arms, /Agent killed/);
  assert.match(arms, /Route discovery/);
  assert.match(arms, /Reroute decision/);
  assert.doesNotMatch(arms, /NaN|undefined/);

  const reroute = fs.readFileSync(path.join(out, "e13_reroute_table.tex"), "utf8");
  assert.match(reroute, /Silence \(readiness deadline\)/);
  assert.doesNotMatch(reroute, /NaN|undefined/);

  const cost = fs.readFileSync(path.join(out, "e13_cost_table.tex"), "utf8");
  assert.match(cost, /Robot~4/);
  assert.match(cost, /Executed in/);
  // Both arms walked the Robot3 route, so it is the marked column in both.
  assert.match(cost, /absent, killed|killed, absent/);
  assert.doesNotMatch(cost, /NaN|undefined/);

  const figure = fs.readFileSync(path.join(out, "e13_figure_budget.tex"), "utf8");
  assert.match(figure, /tikzpicture/);
  assert.match(figure, /Discarded motion/);
  assert.doesNotMatch(figure, /NaN|undefined/);
  fs.rmSync(out, { recursive: true, force: true });
});

test("an explicit refusal and a silent partner are reported apart", () => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), "e13-reason-"));
  const entries = [
    { ...kill({ detectedBy: "timeout", detectMs: 120000 }), run: 6 },
    { ...kill({ detectedBy: "refused", detectMs: 5200 }), run: 7 }
  ];
  for (const entry of entries) {
    const directory = path.join(out, `run${entry.run}`);
    fs.mkdirSync(directory, { recursive: true });
    fs.writeFileSync(path.join(directory, "result.json"),
      JSON.stringify(summarize({ ...entry.meta, run: entry.run }, entry.events)));
  }
  analyze(out, RUNS);
  const table = fs.readFileSync(path.join(out, "e13_reroute_table.tex"), "utf8");
  // Averaging a 120-second deadline with a 5-second refusal would describe
  // neither, so they are separate rows with their own n.
  assert.match(table, /Silence \(readiness deadline\).*& 1 /);
  assert.match(table, /Explicit refusal.*& 1 /);
  fs.rmSync(out, { recursive: true, force: true });
});

// --------------------------------------------------------------- bookkeeping

// Writes one finished trial into a slot, trace and all, the way a real run
// leaves it behind. `mangle` lets a test age the summary on disk.
function trial(out, run, entry, mangle = r => r) {
  const directory = path.join(out, `run${run}`);
  fs.mkdirSync(directory, { recursive: true });
  const meta = { ...entry.meta, run };
  fs.writeFileSync(path.join(directory, "run.json"), JSON.stringify(meta));
  fs.writeFileSync(path.join(directory, "events.jsonl"),
    entry.events.map(e => JSON.stringify(e) + "\n").join(""));
  fs.writeFileSync(path.join(directory, "result.json"), JSON.stringify(mangle(summarize(meta, entry.events))));
  return directory;
}

// The failure that lost a recorded run: `analyze` reads every result.json in the
// experiment, and a result written before a field was added does not have it. An
// experiment in progress must survive its own history, because the runs that are
// still owed cost simulation time and an operator's attention to fly.
test("a summary from an older recorder is rebuilt, not a crash that stops the recorder", () => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), "e13-old-"));
  const entry = cold();
  // Exactly what was on disk: an older schema, missing the fields added since.
  trial(out, 1, entry, r => {
    const aged = { ...r, convergence: { ...r.convergence } };
    delete aged.schema;
    delete aged.convergence.unaskable_peers;
    delete aged.convergence.failed_robot_priced_by;
    return aged;
  });
  trial(out, 2, entry);

  const rebuilt = results(out, RUNS);
  assert.equal(rebuilt.length, 2);
  assert.ok(rebuilt.every(r => r.schema === SCHEMA), "every result handed to analyze is current");
  // Rebuilt on disk too, so the next command does not pay for it again.
  const onDisk = JSON.parse(fs.readFileSync(path.join(out, "run1", "result.json"), "utf8"));
  assert.equal(onDisk.schema, SCHEMA);
  assert.ok(Array.isArray(onDisk.convergence.unaskable_peers));

  const table = analyze(out, RUNS);
  assert.equal(table.length, 2);
  assert.match(fs.readFileSync(path.join(out, "e13_runs.csv"), "utf8"), /unaskable_peers/);
  fs.rmSync(out, { recursive: true, force: true });
});

test("a result whose trace is gone is left out rather than stopping the analysis", () => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), "e13-gone-"));
  trial(out, 1, cold(), r => { const aged = { ...r }; delete aged.schema; return aged; });
  fs.rmSync(path.join(out, "run1", "events.jsonl"));
  fs.rmSync(path.join(out, "run1", "run.json"));
  trial(out, 2, cold());
  const table = analyze(out, RUNS);
  assert.equal(table.length, 1);
  assert.equal(table[0].run, 2);
  fs.rmSync(out, { recursive: true, force: true });
});

// The arm follows the slot number, so an aborted trial holding a cold slot costs
// a cold measurement. Freeing the slot has to keep the trace: the run happened,
// it just did not answer the arm's question.
test("discarding an aborted trial frees its slot and keeps its trace", () => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), "e13-discard-"));
  trial(out, 1, cold(), r => ({ ...r, status: "incomplete", reason: "Gradle exited before the cube was placed" }));
  trial(out, 2, cold());
  assert.equal(nextSlot(out, RUNS), 3);

  const moved = discard(out, RUNS, "1", false);
  assert.equal(moved.length, 1);
  assert.equal(moved[0].arm, armOfRun(1));
  assert.ok(!fs.existsSync(path.join(out, "run1")), "the slot is free");
  assert.equal(nextSlot(out, RUNS), 1, "and the repeat lands back in the same arm");
  const kept = fs.readdirSync(path.join(out, "discarded"));
  assert.equal(kept.length, 1);
  assert.ok(fs.existsSync(path.join(out, "discarded", kept[0], "events.jsonl")), "the trace is kept");

  // A usable measurement is not thrown away by a typo.
  assert.throws(() => discard(out, RUNS, "2", false), /usable measurement/);
  assert.ok(fs.existsSync(path.join(out, "run2")));
  discard(out, RUNS, "2", true);
  assert.ok(!fs.existsSync(path.join(out, "run2")));
  assert.throws(() => discard(out, RUNS, "", false), /--run/);
  assert.throws(() => discard(out, RUNS, "9", false), /no recording/);
  fs.rmSync(out, { recursive: true, force: true });
});

// ----------------------------------------------------------------- recovery

// Every marker has to be a line the plans really print. If one is reworded the
// count silently becomes zero, and a zero is indistinguishable from "this never
// happened" -- which is exactly the reading the thesis takes from the table.
test("every recovery marker is still produced by the file it is attributed to", () => {
  for (const [key, marker, source] of RECOVERY_MARKERS) {
    const text = fs.readFileSync(path.join(__dirname, "..", "..", source), "utf8");
    assert.ok(text.includes(marker), `${key}: ${source} no longer produces "${marker}"`);
  }
});

test("the recoveries are counted per agent, and a refused pair is not a refused point", () => {
  const log = [
    "[robot3_agent] Could not reach the rendezvous from where I stand; resetting posture and asking the same point once more",
    "[robot3_agent] Intermediate giver upright with cube: robot3",
    "[robot3_agent] The rendezvous is out of reach from the all-zero pose too; it is the point, not the posture",
    "[robot3_agent] Rendezvous candidate_55 refused by the cell",
    // The other "refused by the cell": a whole pair, during pricing. Not a recovery.
    "[robot2_agent] The pair robot2 and robot4 has been refused by the cell; the shared space offers nothing this pair can use",
    "[robot5_agent] Move failed for Robot5: region_claimed",
    "[robot5_agent] Could not place the cube on v_Robot5_grip_support_b from here; coming upright with it and asking the same point once more",
    "a line with no agent at all, which is not evidence of anything"
  ].join("\n");
  const found = recoveries(log);
  assert.equal(found.logged, true);
  assert.equal(found.rendezvous_reset, 1);
  assert.equal(found.rendezvous_reset_failed, 1);
  assert.equal(found.rendezvous_refused, 1, "the pair refusal must not be counted as a rendezvous refusal");
  assert.equal(found.placement_reset, 1);
  assert.equal(found.region_claimed, 1);
  assert.equal(found.by_agent.robot3_agent.rendezvous_refused, 1);
  assert.equal(found.by_agent.robot5_agent.placement_reset, 1);
  assert.ok(!found.by_agent.robot2_agent, "the pair refusal belongs to no recovery");

  const none = recoveries("");
  assert.equal(none.logged, false, "no log and no recovery are different facts");
  for (const [key] of RECOVERY_MARKERS) assert.equal(none[key], 0);
});

// The hop is timed from the first attempt, not the successful one: a giver that
// cannot reach its own chosen point opens a new session per attempt, so timing
// only the last session hides the cost exactly where it is largest.
test("a retried hop into the sink is timed from its first attempt", () => {
  const { meta, events } = kill();
  const plain = summarize(meta, events, null);
  const hop = plain.transport.last_hop_ms;
  assert.ok(hop > 0);
  assert.equal(plain.transport.last_hop_sessions, 1);
  const done = plain.handovers.filter(h => !h.abandoned).at(-1);
  assert.ok(Math.abs(hop - done.duration_ms) < 1, "one session, so the hop is that session");
});

test("the recovery table and the delivery figure are built from the runs", () => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), "e13-rec-"));
  const rows = [];
  for (let i = 0; i < PER_ARM; i++) rows.push({ ...cold(), run: i + 1 });
  for (let i = 0; i < PER_ARM; i++) rows.push({ ...kill(), run: PER_ARM + i + 1 });
  const logs = [
    // run 1 resets and then gives the point up; run 2 resets and reaches it;
    // run 3 places only after one recovery. The rest are clean.
    "[robot3_agent] Could not reach the rendezvous from where I stand\n"
      + "[robot3_agent] Rendezvous candidate_55 refused by the cell",
    "[robot3_agent] Could not reach the rendezvous from where I stand",
    "[robot5_agent] Could not place the cube on b from here; coming upright with it and asking the same point once more"
  ];
  const summaries = rows.map((entry, i) => {
    const meta = { ...entry.meta, run: entry.run };
    return summarize(meta, entry.events, null, logs[i] ?? "[robot1_agent] nothing went wrong");
  });
  assert.equal(summaries[0].recovery.rendezvous_refused, 1);
  assert.equal(summaries[2].recovery.placement_reset, 1);

  const table = recoveryTable(summaries);
  assert.match(table, /Reached the first point directly/);
  assert.match(table, /Gave the point up, took the next candidate/);
  assert.match(table, /One recovery/);
  assert.doesNotMatch(table, /NaN|undefined/);

  const figure = deliveryFigure(summaries);
  assert.match(figure, /tikzpicture/);
  assert.match(figure, /Placement/);
  assert.match(figure, /Unattributed/);
  assert.doesNotMatch(figure, /NaN|undefined/);

  // And with no console kept at all, the table says so rather than inventing bands.
  assert.match(recoveryTable(rows.map((e, i) => summarize({ ...e.meta, run: e.run }, e.events))),
    /No E13 run kept its MAS console/);
  fs.rmSync(out, { recursive: true, force: true });
});
