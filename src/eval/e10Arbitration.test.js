const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { execFileSync } = require("node:child_process");
const { parseMarker, terminalFailure, scheduledTrials, updateCompletion, taskComplete, inSimulationTime, stats, summarize, analyze, prepareAgents } = require("./e10Arbitration");

test("completion requires final-role idle marker, six finite zero joints and sustained settling", () => {
  const robot = { beginAt: 0, homeRequestedAt: null, endAt: null };
  const state = { robots: { Robot1: robot } };
  const options = { toleranceDeg: 1, settleMs: 500 };
  const sample = (t, joints = Array(6).fill(0)) => updateCompletion(state, { t, joints: { Robot1: joints } }, options);
  sample(0); sample(1000);
  assert.equal(robot.endAt, null, "startup or intermediate home must not complete the robot");
  robot.homeRequestedAt = 1000;
  sample(1100); sample(1400, [0, 0, 0, 0, 0, 2]); sample(1600);
  sample(2000); assert.equal(robot.endAt, null);
  sample(2100); assert.equal(robot.endAt, 2100);
  robot.endAt = null; robot.stableSince = null;
  sample(3000, []); sample(4000, [0, 0, 0, 0, 0, NaN]);
  assert.equal(robot.endAt, null);
});

test("marker parsing ignores ordinary logs and preserves robot identity", () => {
  assert.deepEqual(parseMarker("[robot2_agent] E10|begin|robot2"), { event: "begin", robot: "Robot2" });
  assert.deepEqual(parseMarker("[robot5_agent] E10|home|robot5"), { event: "home", robot: "Robot5" });
  assert.equal(parseMarker("Giver reached all-zero joint pose"), null);
});

test("a missing robot marker cannot silently complete the selected route", () => {
  const state = { placedAt: 100, selectedRoute: ["Robot1", "Robot2", "Robot5"],
    robots: { Robot1: { endAt: 90 }, Robot2: { endAt: 95 } } };
  assert.equal(taskComplete(state), false);
  state.robots.Robot5 = { endAt: 110 };
  assert.equal(taskComplete(state), true);
  state.placedAt = null;
  assert.equal(taskComplete(state), false);
});

test("terminal failures stop observation but recoverable rendezvous refusals do not", () => {
  assert.match(terminalFailure("[robot5_agent] Could not place the cube on support"), /Could not place/);
  assert.match(terminalFailure("[robot1_agent] Giver home motion failed: blocked"), /home motion failed/);
  assert.equal(terminalFailure("Rendezvous candidate_31 refused by the cell"), null);
  assert.equal(terminalFailure("Retrying with the next-best rendezvous"), null);
});

test("resume preserves failed trials and the counterbalanced schedule", () => {
  const options = { modes: ["exclusive", "region"], repetitions: 2 };
  assert.deepEqual(scheduledTrials(options), [
    { mode: "exclusive", repetition: 1 }, { mode: "region", repetition: 1 },
    { mode: "region", repetition: 2 }, { mode: "exclusive", repetition: 2 }
  ]);
  assert.deepEqual(scheduledTrials(options, [
    { mode: "exclusive", repetition: 1, startedAt: 100, success: false },
    { mode: "region", repetition: 1, startedAt: 200, success: true }
  ]), [{ mode: "region", repetition: 2 }, { mode: "exclusive", repetition: 2 }]);
});

test("simulation time uses exact endpoint samples and brackets events across variable playback rates", () => {
  const r = { mode: "region", repetition: 1, success: true, startedAt: 1000, startedSimTime: 20,
    placedAt: 2200, endedAt: 5000, endedSimTime: 27,
    robots: { Robot1: { beginAt: 1100, endAt: 5000 } } };
  const samples = [{ t: 1200, simTime: 20.5 }, { t: 2000, simTime: 22 }, { t: 3000, simTime: 22.1 }, { t: 5000, simTime: 27 }];
  const converted = inSimulationTime(r, samples);
  assert.equal(converted.endedAt, 7000);
  assert.equal(converted.robots.Robot1.endAt, 7000);
  assert.equal(converted.deliveryBounds.lower, 2000);
  assert.ok(Math.abs(converted.deliveryBounds.upper - 2100) < 1e-8);
  assert.deepEqual(converted.robots.Robot1.beginBounds, { lower: 0, upper: 500 });
  assert.throws(() => inSimulationTime(r, samples.slice(0, -1)), /Missing exact joint sample/);
  const failed = inSimulationTime({ ...r, success: false, placedAt: null,
    robots: { Robot1: { beginAt: 1100, endAt: null } } }, []);
  assert.equal(failed.robots.Robot1.beginAt, null);
  assert.equal(failed.endedAt, null);
});

const record = (mode, seconds) => ({ mode, repetition: 1, success: true, startedAt: 1000,
  placedAt: 1000 + seconds * 800, endedAt: 1000 + seconds * 1000,
  robots: { Robot1: { beginAt: 2000, endAt: 1000 + seconds * 1000 } } });

test("statistics exclude failed runs and unused robots without treating them as zero", () => {
  const rows = summarize([record("exclusive", 10), record("exclusive", 14),
    { ...record("exclusive", 1), success: false }, record("region", 8)]);
  const task = rows.find(r => r.mode === "exclusive" && r.metric === "task");
  assert.equal(task.attempted, 3);
  assert.equal(task.n, 2);
  assert.equal(task.mean, 12);
  assert.equal(task.sd, Math.sqrt(8));
  assert.equal(rows.find(r => r.mode === "exclusive" && r.metric === "Robot1").mean, 12);
  assert.equal(summarize([record("exclusive", 10), record("exclusive", 14)], "participation")
    .find(r => r.mode === "exclusive" && r.metric === "Robot1").mean, 11);
  assert.equal(rows.find(r => r.metric === "Robot3").mean, null);
  assert.equal(stats([1]).sd, null);
});

test("analysis regenerates raw-derived tables and figures without a simulator", () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "e10-analysis-"));
  try {
    fs.writeFileSync(path.join(dir, "e10_runs.jsonl"), [record("exclusive", 10), record("region", 8)].map(JSON.stringify).join("\n"));
    const result = analyze(dir);
    assert.equal(result.comparison.task.speedup, 1.25);
    assert.equal(result.comparison.task.reductionPercent, 20);
    assert.match(fs.readFileSync(path.join(dir, "e10_figure_timeline.tex"), "utf8"), /Bars are participation, NOT motion/);
    assert.match(fs.readFileSync(path.join(dir, "e10_table.tex"), "utf8"), /Robot3 \[s\] & -- & --/);
    assert.match(fs.readFileSync(path.join(dir, "e10_robots.csv"), "utf8"), /Robot3,0,nan,nan,nan/);
  } finally { fs.rmSync(dir, { recursive: true, force: true }); }
});

test("evaluation copies instrument both roles and add the terminal home without editing production plans", () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "e10-agents-"));
  try {
    const source = path.resolve(__dirname, "../agt/handoverPlans.asl");
    const original = fs.readFileSync(source, "utf8");
    const jcm = prepareAgents(dir);
    const plans = fs.readFileSync(path.join(dir, "handoverPlans.asl"), "utf8");
    assert.equal(plans.split('E10|begin|').length - 1, 2);
    assert.match(plans, /E10\|placed\|[\s\S]*?!goHome;[\s\S]*?E10\|home\|/);
    assert.match(fs.readFileSync(jcm, "utf8"), /agent robot5_agent:/);
    assert.equal(fs.readFileSync(source, "utf8"), original);
  } finally { fs.rmSync(dir, { recursive: true, force: true }); }
});

test("the generated evaluation project parses with JaCaMo's real parser", () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "e10-jcm-x"));
  try {
    const cache = path.join(os.homedir(), ".gradle/caches/modules-2/files-2.1");
    const jars = [];
    const walk = directory => {
      for (const entry of fs.readdirSync(directory, { withFileTypes: true })) {
        const file = path.join(directory, entry.name);
        if (entry.isDirectory()) walk(file);
        else if (entry.name.endsWith(".jar") && !/-sources|-javadoc/.test(entry.name)) jars.push(file);
      }
    };
    walk(cache);
    assert.ok(jars.some(file => file.includes("/jacamo/")));
    const jcm = prepareAgents(path.join(dir, "agent_sources"));
    const checker = path.join(dir, "E10JcmCheck.java");
    fs.writeFileSync(checker, "import java.io.*; import jacamo.project.parser.JaCaMoProjectParser;\n"
      + "class E10JcmCheck { public static void main(String[] a) throws Exception {\n"
      + "var p = new JaCaMoProjectParser(new FileInputStream(a[0])).parse(new File(a[0]).getParent());\n"
      + "if(p.getAgents().size()!=5) throw new Exception(\"Expected five agents\"); System.out.println(\"ok\"); }}\n");
    const result = execFileSync("java", ["-cp", jars.join(path.delimiter), checker, jcm], { encoding: "utf8" });
    assert.match(result, /ok/);
  } finally { fs.rmSync(dir, { recursive: true, force: true }); }
});
