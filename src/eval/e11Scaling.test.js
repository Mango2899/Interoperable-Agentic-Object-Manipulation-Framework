"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("fs");
const os = require("os");
const path = require("path");
const { summarize, instrumentAgent, instrumentRoutes, readLines, analyze, cellCommands, cellMismatch } = require("./e11Scaling");
const { parseArgs, parseResampleSelection } = require("../generator/robwotBridge");
function fixture() {
  const events = [{ event: "gradle_start", t: 1000 }];
  const add = (event, agent, phase, t, data = [], beliefs = []) => events.push({ event, agent, phase, t: 1000 + t, mono_ms: 9000 + t, data, beliefs });
  for (let i = 1; i <= 3; i++) {
    const agent = `robot${i}_agent`;
    for (const [j, phase] of ["neighbour_discovery", "interaction_space", "task_sites", "initial_edge_pricing"].entries()) {
      add("phase_start", agent, phase, 100 + j * 100 + i);
      add("phase_end", agent, phase, 150 + j * 100 + i, [], ["neighborScanComplete(2,1)", "taskSitesReady(2,1)"]);
    }
    add("agent_ready", agent, "startup", 900 + i);
  }
  add("phase_start", "robot1_agent", "route_discovery", 460);
  add("route_end", "robot1_agent", "route_discovery", 899, ["success", "route(robot1,b,0)", "2.5", "[robot1,robot2,robot3]"]);
  for (const [content, receiver] of [["workspaceCloud(robot3,x,1,1,y)", "robot2_agent"], ["findRoute(task,c,b,8,200)", "robot2_agent"], ["routeCost(task,[],1,[robot3])", "robot1_agent"], ["noRoute(task)", "robot1_agent"], ["diagnostic(x)", "outsider"]]) {
    events.push({ event: "message", agent: "robot3_agent", receiver, content, t: 1500, mono_ms: 9500 });
  }
  events.sort((a, b) => a.t - b.t);
  return { meta: { cell: 3, run: 1, t: 1000 }, events };
}
test("phase makespan is not the sum of overlapping agents; route ends after actual pricing/decision", () => {
  const { meta, events } = fixture();
  const r = summarize(meta, events, [{ cached: false, durationMs: 12, t: 1220, agent: "Robot1" }, { cached: true, durationMs: 3, t: 1221, agent: "Robot2" }]);
  assert.equal(r.status, "ok");
  assert.equal(r.gradle_to_first_phase_ms, 101);
  assert.equal(r.durations_ms.neighbour_discovery, 52);
  assert.equal(r.durations_ms.route_discovery, 439);
  assert.equal(r.messages_total, 4);
  assert.deepEqual(r.messages_by_type, { workspaceCloud: 1, findRoute: 1, routeCost: 1, noRoute: 1 });
  assert.equal(r.cache_observation, "mixed");
  assert.equal(r.interaction_compute_work_ms, 15);
});
test("missing boundaries, failed scans, no route and local route are retained distinctly", () => {
  const { meta, events } = fixture();
  assert.equal(summarize(meta, events.filter(e => !(e.phase === "interaction_space" && e.event === "phase_end")), []).durations_ms.interaction_space, null);
  assert.equal(summarize(meta, events.filter(e => e.event !== "route_end"), []).status, "incomplete");
  events.find(e => e.event === "route_end").data[0] = "no_route";
  assert.equal(summarize(meta, events, []).status, "no_route");
  events.find(e => e.event === "route_end").data[0] = "local";
  assert.equal(summarize(meta, events, []).status, "incomplete");
  events.find(e => e.event === "phase_end").beliefs = ["neighborScanFailed(error)"];
  assert.match(summarize(meta, events, []).issues.join(";"), /Neighbour scan did not succeed/);
});
test("duration uses monotonic timestamps even if wall clock shifts", () => {
  const { meta, events } = fixture();
  events.find(e => e.event === "route_end").t += 20000;
  assert.equal(summarize(meta, events, []).durations_ms.route_discovery, 439);
});
test("a five-robot AgentThing catalogue cannot produce a valid three-robot trial", () => {
  const { meta, events } = fixture();
  assert.equal(cellMismatch(3, events), null);
  events.find(e => e.event === "phase_end" && e.phase === "neighbour_discovery").beliefs = ["neighborScanComplete(4,4)[source(ownLayer1)]"];
  assert.match(cellMismatch(3, events), /scanned 4 other robots; cell 3 requires 2/);
  assert.equal(summarize(meta, events, []).status, "incomplete");
  assert.match(cellMismatch(3, [{ event: "cell_mismatch", detail: "wrong robot names" }]), /wrong robot names/);
});
test("instrumentation applies uniquely to actual production plans without modifying them", () => {
  const route = fs.readFileSync(path.join(__dirname, "../agt/routePlans.asl"), "utf8");
  const changed = instrumentRoutes(route);
  assert.match(changed, /\+routeTotal\(Task, Best\);\n\s*e11.event\(route_end/);
  assert.equal((changed.match(/e11.event\(route_end/g) || []).length, 4);
  for (let n = 1; n <= 5; n++) {
    const source = fs.readFileSync(path.join(__dirname, `../agt/generated/robot${n}_agent.asl`), "utf8");
    const instrumented = instrumentAgent(source, "/tmp/e11/routePlans.asl");
    assert.equal((instrumented.match(/e11.event\(phase_start/g) || []).length, 4);
    assert.match(instrumented, /e11.event\(agent_ready/);
  }
  assert.throws(() => instrumentRoutes("changed upstream plans"), /anchor changed/);
});
test("JSONL reader ignores a partial append and obeys byte offsets", () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "e11-lines-"));
  try {
    const p = path.join(dir, "events.jsonl");
    fs.writeFileSync(p, '{"old":1}\n{"new":2}\n{"partial":');
    assert.deepEqual(readLines(p, Buffer.byteLength('{"old":1}\n')), [{ new: 2 }]);
  } finally { fs.rmSync(dir, { recursive: true }); }
});
test("analysis retains failed runs and reports missing repetitions", () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "e11-analysis-"));
  try {
    const { meta, events } = fixture();
    const dest = path.join(dir, "cell3/run1"); fs.mkdirSync(dest, { recursive: true });
    fs.writeFileSync(path.join(dest, "result.json"), JSON.stringify(summarize(meta, events, [], "timeout")));
    assert.equal(analyze(dir)[0].status, "incomplete");
    assert.match(fs.readFileSync(path.join(dir, "e11_summary.csv"), "utf8"), /"3","neighbour_discovery_ms","1","0","5"/);
  } finally { fs.rmSync(dir, { recursive: true }); }
});
test("generation keeps placement and explicit empty resample selection really reuses clouds", () => {
  const descriptor = require("../../generated/robots/Robot1_Robot2_Robot3_Robot4_Robot5.shared.json");
  const command = cellCommands(3, descriptor, [2.74, .84]);
  assert.match(command.generate, /0,0,-15;0,0,50;-90,-155,90/);
  assert.match(command.generate, /Robot3_grip_support_b@2.74,0.84,0.08/);
  const args = parseArgs(["--generate-shared-cell", "--resample", ""]);
  assert.equal(args.resample, "");
  assert.deepEqual(parseResampleSelection(args.resample, descriptor.robots.map(r => ({ manifest: r }))), [false, false, false, false, false]);
});

test("passive watcher records five synthetic runs and exits without launching a service", async () => {
  const { spawn } = require("child_process");
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "e11-watch-"));
  const child = spawn(process.execPath, [path.join(__dirname, "e11Scaling.js"), "watch", "--cell", "5", "--out", dir]);
  let output = "";
  child.stdout.on("data", b => { output += b; }); child.stderr.on("data", b => { output += b; });
  const exited = new Promise(resolve => child.on("exit", code => resolve(code)));
  const sleep = () => new Promise(resolve => setTimeout(resolve, 30));
  try {
    const deadline = Date.now() + 10000;
    while (!fs.existsSync(path.join(dir, "cell5/watch.json"))) {
      if (Date.now() > deadline || child.exitCode !== null) throw Error(output || "Recorder did not arm");
      await sleep();
    }
    const computes = path.join(dir, "computes.jsonl"); fs.writeFileSync(computes, "");
    for (let run = 1; run <= 5; run++) {
      const { meta, events } = fixture();
      const sourceEvents = events.filter(e => e.agent === "robot3_agent" && e.event !== "message");
      for (let n = 4; n <= 5; n++) events.push(...sourceEvents.map(e => ({ ...e, agent: `robot${n}_agent` })));
      for (const event of events) if (event.beliefs) event.beliefs = event.beliefs.map(b => b.replace("neighborScanComplete(2,1)", "neighborScanComplete(4,1)"));
      const dest = path.join(dir, `cell5/run${run}`); fs.mkdirSync(dest);
      fs.writeFileSync(path.join(dest, "events.jsonl"), events.map(e => JSON.stringify(e) + "\n").join(""));
      fs.writeFileSync(path.join(dest, "run.json"), JSON.stringify({ ...meta, run, cell: 5, compute_file: computes, compute_offset: 0 }));
    }
    while (child.exitCode === null && Date.now() < deadline) await sleep();
    if (child.exitCode === null) throw Error("Recorder did not complete: " + output);
    assert.equal(await exited, 0, output);
    for (let run = 1; run <= 5; run++) {
      const result = JSON.parse(fs.readFileSync(path.join(dir, `cell5/run${run}/result.json`)));
      assert.equal(result.status, "ok", JSON.stringify(result));
      assert.equal(result.messages_total, 4);
    }
    assert.equal(fs.existsSync(path.join(dir, "cell5/watch.json")), false);
  } finally {
    if (child.exitCode === null) { child.kill("SIGTERM"); await exited; }
    fs.rmSync(dir, { recursive: true, force: true });
  }
});
