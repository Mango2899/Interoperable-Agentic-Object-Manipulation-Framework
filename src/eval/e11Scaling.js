"use strict";

// E11 observes files only. Only the explicit `launch` command starts Gradle;
// scene generation, lower servers and AgentThings remain under user control.
const fs = require("fs");
const path = require("path");
const { spawn } = require("child_process");
// Shared with E12, which measures route discovery the same way and adds the
// transfer. Changing a helper here changes both experiments on purpose.
const {
  ROOT, read, write, append, relative, quote, replaceOnce, readLines,
  csv, meanAndSampleSd, texNumber, instrumentRoutes
} = require("./evalTrace");
const { parseArgs } = require("../generator/core/context");
const OUT = path.join(ROOT, "performance_log/e11");
const CELL_ROOT = path.join(ROOT, "generated/experiments/e11");
const PHASES = ["neighbour_discovery", "interaction_space", "task_sites", "initial_edge_pricing"];
const names = n => Array.from({ length: n }, (_, i) => `Robot${i + 1}`);
function cellNumber(n) { n = Number(n); if (![3, 4, 5].includes(n)) throw Error("--cell must be 3, 4 or 5"); return n; }
function manifests(n) {
  return names(n).map(name => path.join(n === 5 ? path.join(ROOT, "generated/robots") : path.join(CELL_ROOT, `cell${n}`), name, "manifest.json"));
}
function cellCommands(n, descriptor, support) {
  const robots = descriptor.robots.slice(0, n);
  const list = manifests(n).map(relative).join(",");
  const sites = `Robot1_grip_support@0.55,0.55,0.08,0,Robot1_grip_cube;Robot${n}_grip_support_b@${support[0]},${support[1]},0.08`;
  return {
    generate: "npm run robot:pipeline -- --manifests " + list + " --generate-shared-cell"
      + " --offsets " + quote(robots.map(r => r.offset.join(",")).join(";"))
      + " --orientations " + quote(robots.map(r => r.orientationDeg.map(v => +v.toFixed(6)).join(",")).join(";"))
      + " --wall-offsets " + quote(robots.map(r => r.wallOffset.join(",")).join(";"))
      + " --sites " + quote(sites) + " --resample ''",
    lower: "npm run robot:serve -- --manifests " + list,
    agents: manifests(n).map(p => "npm run robot:agent -- --manifest " + relative(p))
  };
}
function prepareCells() {
  const descriptor = read(path.join(ROOT, "generated/robots/Robot1_Robot2_Robot3_Robot4_Robot5.shared.json"));
  // Chosen offline from collision-free samples at grasp z=.1825 and hover z=.3825.
  // These are candidates, not a simulator IK/collision certificate.
  for (const [n, support] of [[3, [2.74, 0.84]], [4, [2.42, -0.14]]]) {
    const directory = path.join(CELL_ROOT, `cell${n}`);
    if (fs.existsSync(directory)) throw Error(`${relative(directory)} already exists; refusing to overwrite a prepared cell.`);
    for (const name of names(n)) {
      const source = path.join(ROOT, "generated/robots", name);
      const target = path.join(directory, name);
      const manifest = read(path.join(source, "manifest.json"));
      for (const folder of ["model", "info", "td", "scene"]) fs.cpSync(path.join(source, folder), path.join(target, folder), { recursive: true });
      for (const [key, file] of Object.entries(manifest.generatedFiles)) {
        const src = path.join(ROOT, file);
        if (!fs.existsSync(src) || !fs.statSync(src).isFile()) continue;
        const dest = file.startsWith(`generated/robots/${name}/`)
          ? path.join(target, file.slice(`generated/robots/${name}/`.length))
          : path.join(target, path.basename(file));
        fs.mkdirSync(path.dirname(dest), { recursive: true });
        if (!fs.existsSync(dest)) fs.copyFileSync(src, dest);
        manifest.generatedFiles[key] = relative(dest);
      }
      write(path.join(target, "manifest.json"), manifest);
    }
    const commands = cellCommands(n, descriptor, support);
    write(path.join(directory, "setup.json"), { robots: n, supportB: [...support, 0.08], baseline: descriptor, commands });
    for (const key of ["generate", "lower"]) fs.writeFileSync(path.join(directory, `${key}.sh`),
      `#!/usr/bin/env bash\nset -euo pipefail\ncd ${quote(ROOT)}\n${commands[key]}\n`);
    console.log(`Prepared ${relative(directory)} (offline).\n${commands.generate}\n${commands.lower}`);
  }
}
function instrumentAgent(source, routeFile) {
  for (const [goal, phase] of [["discoverNeighbors", PHASES[0]], ["shareInteractionSpaces", PHASES[1]],
    ["detectTaskSites", PHASES[2]], ["reportHandoverCosts", PHASES[3]]]) {
    source = replaceOnce(source, `    !${goal};`, `    e11.event(phase_start, ${phase});\n    !${goal};\n    e11.event(phase_end, ${phase});`);
  }
  source = replaceOnce(source, "    !discoverRoute;", "    !discoverRoute;\n    e11.event(agent_ready, startup);");
  return replaceOnce(source, 'include("src/agt/routePlans.asl")', `include(${JSON.stringify(routeFile)})`);
}
function preparePlans(n, out) {
  const directory = path.join(out, "plans", `cell${n}`);
  fs.mkdirSync(directory, { recursive: true });
  const route = path.join(directory, "routePlans.asl");
  fs.writeFileSync(route, instrumentRoutes(fs.readFileSync(path.join(ROOT, "src/agt/routePlans.asl"), "utf8")));
  const agents = [], workspaces = [];
  for (const [i, file] of manifests(n).entries()) {
    const manifest = read(file), agent = `robot${i + 1}_agent`, workspace = `robot${i + 1}_workspace`;
    const dest = path.join(directory, agent + ".asl");
    fs.writeFileSync(dest, instrumentAgent(fs.readFileSync(path.join(ROOT, manifest.generatedFiles.agentSpeak), "utf8"), route));
    if (/\s/.test(relative(dest))) throw Error("E11 paths must not contain whitespace (JaCaMo source syntax).");
    agents.push(`    agent ${agent}: ${relative(dest)} {\n        ag-arch: e11.MessageTrace\n        focus: ${workspace}.wotComm\n    }`);
    workspaces.push(`    workspace ${workspace} {\n        artifact wotComm: wotCommunication.wotCommunicator("${manifest.configKey}")\n    }`);
  }
  const jcm = path.join(directory, "e11.jcm");
  fs.writeFileSync(jcm, `mas e11_cell${n} {\n${agents.join("\n")}\n${workspaces.join("\n")}\n}\n`);
  fs.writeFileSync(path.join(directory, "logging.properties"), "handlers=java.util.logging.ConsoleHandler\n.level=INFO\njava.util.logging.ConsoleHandler.level=ALL\njava.util.logging.ConsoleHandler.formatter=jason.runtime.MASConsoleLogFormatter\n");
  return jcm;
}
function cellMismatch(n, events) {
  const explicit = events.find(e => e.event === "cell_mismatch");
  if (explicit) return explicit.detail;
  for (const event of events.filter(e => e.event === "phase_end" && e.phase === "neighbour_discovery")) {
    const scan = event.beliefs?.map(b => /^neighborScanComplete\((\d+),/.exec(b)).find(Boolean);
    if (scan && Number(scan[1]) !== n - 1) return `E11 cell mismatch: ${event.agent} scanned ${scan[1]} other robots; cell ${n} requires ${n - 1}. Restart every AgentThing with the selected cell's manifest copies.`;
  }
  return null;
}
function summarize(meta, events, computes, reason = null) {
  const expected = names(meta.cell).map(n => n.toLowerCase() + "_agent");
  const ready = new Set(events.filter(e => e.event === "agent_ready").map(e => e.agent));
  const ends = events.filter(e => e.event === "route_end");
  const start = events.filter(e => e.event === "phase_start");
  const phases = [], issues = [];
  const mismatch = cellMismatch(meta.cell, events);
  if (mismatch) issues.push(mismatch);
  for (const phase of PHASES) {
    for (const agent of expected) {
      const a = start.find(e => e.phase === phase && e.agent === agent);
      const b = events.find(e => e.event === "phase_end" && e.phase === phase && e.agent === agent);
      if (!a || !b) issues.push(`Missing ${phase} boundary for ${agent}`);
      if (b && phase === "neighbour_discovery" && !b.beliefs?.some(x => x.startsWith("neighborScanComplete("))) issues.push(`Neighbour scan did not succeed for ${agent}`);
      if (b && phase === "task_sites" && !b.beliefs?.some(x => x.startsWith("taskSitesReady("))) issues.push(`Task-site scan did not succeed for ${agent}`);
      phases.push({ phase, agent, start_ms: a ? a.t - meta.t : null, end_ms: b ? b.t - meta.t : null,
        duration_ms: a && b ? b.mono_ms - a.mono_ms : null, start_mono: a?.mono_ms, end_mono: b?.mono_ms });
    }
  }
  for (const a of start.filter(e => e.phase === "route_discovery")) {
    const b = ends.find(e => e.agent === a.agent);
    phases.push({ phase: "route_discovery", agent: a.agent, start_ms: a.t - meta.t, end_ms: b ? b.t - meta.t : null,
      duration_ms: b ? b.mono_ms - a.mono_ms : null, start_mono: a.mono_ms, end_mono: b?.mono_ms });
    if (!b) issues.push(`Missing route decision for ${a.agent}`);
  }
  if (ends.length !== 1) issues.push(`Expected one route origin/decision, observed ${ends.length}`);
  if (ready.size !== meta.cell) issues.push(`Only ${ready.size}/${meta.cell} agents initialized`);
  const durations = {};
  for (const phase of [...PHASES, "route_discovery"]) {
    const rows = phases.filter(p => p.phase === phase);
    durations[phase] = rows.length && rows.every(p => p.duration_ms !== null)
      ? Math.max(...rows.map(p => p.end_mono)) - Math.min(...rows.map(p => p.start_mono)) : null;
  }
  const messages = events.filter(e => e.event === "message" && expected.includes(e.agent) && expected.includes(e.receiver));
  const counts = {};
  for (const m of messages) { const type = /^([\w]+)/.exec(m.content)?.[1] || "other"; counts[type] = (counts[type] || 0) + 1; }
  const fresh = computes.filter(c => c.cached === false).length, cached = computes.filter(c => c.cached === true).length;
  const failed = computes.filter(c => c.reason).length;
  const startupComputes = computes.filter(c => phases.some(p => p.phase === "interaction_space"
    && p.agent === c.agent.toLowerCase() + "_agent" && p.start_ms !== null && p.end_ms !== null
    && c.t - meta.t >= p.start_ms && c.t - meta.t <= p.end_ms));
  const startupFresh = startupComputes.filter(c => c.cached === false).length;
  const startupCached = startupComputes.filter(c => c.cached === true).length;
  const outcome = ends[0]?.data?.[0] || "missing";
  if (outcome === "local") issues.push("Support B is local to the source; no multi-robot route was measured");
  return { cell: meta.cell, run: meta.run, status: issues.length || reason ? "incomplete" : outcome === "no_route" ? "no_route" : "ok",
    reason, issues, started_at: new Date(meta.t).toISOString(), elapsed_ms: events.at(-1)?.t - meta.t || 0,
    gradle_to_first_phase_ms: start.length ? Math.min(...start.map(e => e.t)) - meta.t : null,
    durations_ms: durations, phases, messages_total: messages.length, messages_by_type: counts,
    route: ends.map(e => ({ agent: e.agent, data: e.data })),
    interaction_attempts: computes.length, interaction_computed: fresh, interaction_cached: cached, interaction_refused: failed,
    startup_interaction_computed: startupFresh, startup_interaction_cached: startupCached,
    cache_observation: startupFresh && startupCached ? "mixed" : startupFresh ? "computed" : startupCached ? "cached" : "unobserved_or_refused",
    interaction_compute_work_ms: computes.reduce((sum, c) => sum + (c.durationMs || 0), 0) };
}
function writeThesisArtifacts(out, rows, results) {
  const valid = rows.filter(row => row.status === "ok");
  const phasePlot = [
    "% Generated from the 15 E11 result.json files by src/eval/e11Scaling.js.",
    "\\begin{tikzpicture}",
    "\\begin{axis}[ybar,bar width=14pt,width=0.31\\linewidth,height=5.3cm,symbolic x coords={3,4,5},xtick=data,ymin=0,ymax=0.40,ylabel={Time [s]},xlabel={Robots},title={Neighbour discovery},ymajorgrids=true]",
    "\\addplot+[draw=blue!65,fill=blue!30,error bars/.cd,y dir=both,y explicit] coordinates {"
      + [3, 4, 5].map(cell => { const s = meanAndSampleSd(valid.filter(r => r.cell === cell).map(r => r.neighbour_discovery_ms / 1000)); return `(${cell},${texNumber(s.mean, 4)}) +- (0,${texNumber(s.sd, 4)})`; }).join(" ") + "};",
    "\\end{axis}",
    "\\end{tikzpicture}%",
    "\\begin{tikzpicture}",
    "\\begin{axis}[ybar,bar width=14pt,width=0.31\\linewidth,height=5.3cm,symbolic x coords={3,4,5},xtick=data,ymin=0,ymax=5.2,xlabel={Robots},title={Interaction-space phase},ymajorgrids=true]",
    "\\addplot+[draw=orange!85!black,fill=orange!45,error bars/.cd,y dir=both,y explicit] coordinates {"
      + [3, 4, 5].map(cell => { const s = meanAndSampleSd(valid.filter(r => r.cell === cell).map(r => r.interaction_space_ms / 1000)); return `(${cell},${texNumber(s.mean, 4)}) +- (0,${texNumber(s.sd, 4)})`; }).join(" ") + "};",
    "\\end{axis}",
    "\\end{tikzpicture}%",
    "\\begin{tikzpicture}",
    "\\begin{axis}[ybar,bar width=14pt,width=0.31\\linewidth,height=5.3cm,symbolic x coords={3,4,5},xtick=data,ymin=0,ymax=5.0,xlabel={Robots},title={Route discovery},ymajorgrids=true]",
    "\\addplot+[draw=green!50!black,fill=green!35,error bars/.cd,y dir=both,y explicit] coordinates {"
      + [3, 4, 5].map(cell => { const s = meanAndSampleSd(valid.filter(r => r.cell === cell).map(r => r.route_discovery_ms / 1000)); return `(${cell},${texNumber(s.mean, 4)}) +- (0,${texNumber(s.sd, 4)})`; }).join(" ") + "};",
    "\\end{axis}",
    "\\end{tikzpicture}"
  ].join("\n") + "\n";
  fs.writeFileSync(path.join(out, "e11_figure_phases.tex"), phasePlot);

  const messageTypes = [
    ["workspaceCloud", "Workspace cloud", "blue!35"],
    ["interactionSpaceReady", "Space ready", "cyan!55"],
    ["findRoute", "findRoute", "orange!55"],
    ["routeCost", "routeCost", "green!45"],
    ["noRoute", "noRoute", "red!40"]
  ];
  const byCell = new Map([3, 4, 5].map(cell => [cell, results.find(r => r.cell === cell && r.status === "ok")]));
  const messagePlot = [
    "% Generated from the 15 E11 result.json files by src/eval/e11Scaling.js.",
    "\\begin{tikzpicture}",
    "\\begin{axis}[ybar stacked,bar width=24pt,width=0.78\\linewidth,height=6.2cm,symbolic x coords={3,4,5},xtick=data,ymin=0,ymax=45,ylabel={Inter-agent messages},xlabel={Robots},legend style={at={(0.5,1.03)},anchor=south,legend columns=3},ymajorgrids=true]",
    ...messageTypes.flatMap(([key, label, color]) => [
      `\\addplot+[draw=${color},fill=${color}] coordinates {${[3, 4, 5].map(cell => `(${cell},${byCell.get(cell)?.messages_by_type?.[key] || 0})`).join(" ")}};`,
      `\\addlegendentry{${label}}`
    ]),
    "\\end{axis}",
    "\\end{tikzpicture}"
  ].join("\n") + "\n";
  fs.writeFileSync(path.join(out, "e11_figure_messages.tex"), messagePlot);

  const rowEnd = String.raw`\\`;
  const tableRows = [3, 4, 5].map(cell => {
    const group = valid.filter(row => row.cell === cell);
    const result = byCell.get(cell);
    const route = result?.route?.[0]?.data || [];
    const routeText = String(route[3] || "").replace(/^\[|\]$/g, "").split(",")
      .map(robot => robot.trim().replace(/^robot/, "" )).filter(Boolean).join("--");
    return `${cell} & ${group[0]?.messages_total ?? "--"} & ${group[0]?.findRoute ?? "--"} & ${group[0]?.routeCost ?? "--"} & ${group[0]?.noRoute ?? "--"} & ${routeText} (${Number(route[2]).toFixed(2)}\\,m) & ${group.length}/5 ${rowEnd}`;
  });
  fs.writeFileSync(path.join(out, "e11_message_table.tex"), [
    "% Generated from the 15 E11 result.json files by src/eval/e11Scaling.js.",
    "\\begin{tabular}{@{}lrrrrlr@{}}", "\\toprule",
    `Robots & All & \\texttt{findRoute} & \\texttt{routeCost} & \\texttt{noRoute} & Selected route (cost) & Success ${rowEnd}`,
    "\\midrule", ...tableRows, "\\bottomrule", "\\end{tabular}"
  ].join("\n") + "\n");
}
function analyze(out) {
  const results = [];
  for (const n of [3, 4, 5]) for (let run = 1; run <= 5; run++) {
    const file = path.join(out, `cell${n}`, `run${run}`, "result.json");
    if (fs.existsSync(file)) results.push(read(file));
  }
  const rows = results.map(r => ({ cell: r.cell, run: r.run, status: r.status, cache: r.cache_observation,
    gradle_to_first_phase_ms: r.gradle_to_first_phase_ms, ...Object.fromEntries(Object.entries(r.durations_ms).map(([k, v]) => [k + "_ms", v])),
    messages_total: r.messages_total, findRoute: r.messages_by_type.findRoute || 0, routeCost: r.messages_by_type.routeCost || 0,
    noRoute: r.messages_by_type.noRoute || 0, workspaceCloud: r.messages_by_type.workspaceCloud || 0,
    computed: r.interaction_computed, cached: r.interaction_cached, refused: r.interaction_refused,
    startup_computed: r.startup_interaction_computed, startup_cached: r.startup_interaction_cached,
    interaction_compute_work_ms: r.interaction_compute_work_ms, route: r.route, issues: r.issues, reason: r.reason }));
  fs.mkdirSync(out, { recursive: true });
  fs.writeFileSync(path.join(out, "e11_runs.csv"), csv(rows));
  fs.writeFileSync(path.join(out, "e11_phases.csv"), csv(results.flatMap(r => r.phases.map(p => ({ cell: r.cell, run: r.run,
    phase: p.phase, agent: p.agent, start_ms: p.start_ms, end_ms: p.end_ms, duration_ms: p.duration_ms })))));
  const summary = [];
  for (const n of [3, 4, 5]) {
    const group = rows.filter(r => r.cell === n), valid = group.filter(r => r.status === "ok");
    for (const metric of ["neighbour_discovery_ms", "interaction_space_ms", "route_discovery_ms", "messages_total"]) {
      const values = valid.map(r => r[metric]).filter(v => v !== null), mean = values.length ? values.reduce((a, b) => a + b, 0) / values.length : null;
      summary.push({ cell: n, metric, runs_recorded: group.length, successful: valid.length, expected_runs: 5,
        cache_modes: [...new Set(group.map(r => r.cache))].join("/"), mean,
        sample_sd: values.length > 1 ? Math.sqrt(values.reduce((s, v) => s + (v - mean) ** 2, 0) / (values.length - 1)) : null });
    }
  }
  fs.writeFileSync(path.join(out, "e11_summary.csv"), csv(summary));
  writeThesisArtifacts(out, rows, results);
  return rows;
}
function finishTrial(directory, reason, out) {
  if (fs.existsSync(path.join(directory, "result.json"))) return;
  const meta = read(path.join(directory, "run.json"));
  const events = readLines(path.join(directory, "events.jsonl"));
  const end = events.at(-1)?.t || Date.now();
  const computes = readLines(meta.compute_file, meta.compute_offset).filter(c => c.t >= meta.t && c.t <= end && names(meta.cell).includes(c.agent));
  const result = summarize(meta, events, computes, reason);
  fs.writeFileSync(path.join(directory, "measurement-events.jsonl"), events.map(e => JSON.stringify(e) + "\n").join(""));
  fs.writeFileSync(path.join(directory, "interaction-compute.jsonl"), computes.map(c => JSON.stringify(c) + "\n").join(""));
  fs.writeFileSync(path.join(directory, "messages.csv"), csv(events.filter(e => e.event === "message").map(e => ({
    since_gradle_ms: e.t - meta.t, sender: e.agent, receiver: e.receiver, performative: e.performative, content: e.content, message_id: e.message_id }))));
  write(path.join(directory, "result.json"), result);
  analyze(out);
  console.log(`Cell ${meta.cell}, run ${meta.run}: ${result.status}; ${result.messages_total} messages; phases (ms) ${JSON.stringify(result.durations_ms)}; cache=${result.cache_observation}`);
}
async function watch(n, out, timeoutSeconds = 600) {
  const cell = path.join(out, `cell${n}`), armFile = path.join(cell, "watch.json");
  if (fs.existsSync(armFile)) { try { process.kill(read(armFile).pid, 0); throw Error("A recorder is already running for this cell."); } catch (e) { if (e.code !== "ESRCH") throw e; } }
  const jcm = preparePlans(n, out);
  write(armFile, { pid: process.pid, jcm, armed_at: Date.now() });
  console.log(`E11 armed for five manually started runs of cell ${n}.\nIn another terminal, for each run:\nnode src/eval/e11Scaling.js launch --cell ${n} --out ${quote(relative(out))}\nWaiting for Gradle; arming time is excluded.`);
  let stopping = false;
  const stop = () => { stopping = true; };
  process.on("SIGINT", stop); process.on("SIGTERM", stop);
  try {
    while (!stopping) {
      let completed = 0;
      for (let run = 1; run <= 5; run++) {
        const directory = path.join(cell, `run${run}`);
        if (fs.existsSync(path.join(directory, "result.json"))) { completed++; continue; }
        if (!fs.existsSync(path.join(directory, "run.json"))) continue;
        const meta = read(path.join(directory, "run.json")), events = readLines(path.join(directory, "events.jsonl"));
        const ready = new Set(events.filter(e => e.event === "agent_ready").map(e => e.agent));
        const ended = events.some(e => e.event === "route_end");
        const searching = events.some(e => e.event === "phase_start" && e.phase === "route_discovery");
        const exit = events.find(e => e.event === "gradle_exit");
        const mismatch = cellMismatch(n, events);
        if (mismatch) finishTrial(directory, mismatch, out);
        else if (ready.size === n && ended) finishTrial(directory, null, out);
        else if (exit || Date.now() - meta.t > timeoutSeconds * 1000 || (ready.size === n && !searching)) {
          finishTrial(directory, exit ? "Gradle exited before measurement completed" : !searching && ready.size === n ? "No route search initiated" : "Measurement timeout", out);
        }
      }
      if (completed === 5) { console.log("Five runs recorded. See e11_runs.csv, e11_phases.csv and e11_summary.csv."); break; }
      await new Promise(resolve => setTimeout(resolve, 250));
    }
  } finally {
    fs.rmSync(armFile, { force: true });
    process.off("SIGINT", stop); process.off("SIGTERM", stop);
  }
}
async function launch(n, out, computeFile) {
  const cell = path.join(out, `cell${n}`), armed = read(path.join(cell, "watch.json"));
  process.kill(armed.pid, 0); // Existence check only; signal 0 sends no signal.
  let run = 1;
  while (run <= 5 && fs.existsSync(path.join(cell, `run${run}`))) run++;
  if (run > 5) throw Error("All five run slots are used. Use a new --out for a new experiment.");
  if (run > 1) {
    const prev = path.join(cell, `run${run - 1}`);
    if (!fs.existsSync(path.join(prev, "result.json")) || !fs.existsSync(path.join(prev, "exit.json"))) throw Error("Wait for the previous result and stop its Gradle process before starting the next run.");
  }
  const snapshots = manifests(n).map(file => ({ file: relative(file), manifest: read(file) }));
  const descriptorFile = path.join(ROOT, snapshots[0].manifest.sharedEnvironment.descriptor);
  const descriptor = read(descriptorFile);
  if (descriptor.robots.map(r => r.name).join(",") !== names(n).join(",")) throw Error("Generate the requested cell first: its descriptor has a different robot list.");
  const directory = path.join(cell, `run${run}`);
  fs.mkdirSync(directory); // Exclusive slot; never overwrite a trial.
  const trace = path.join(directory, "events.jsonl");
  fs.writeFileSync(trace, "");
  write(path.join(directory, "inputs.json"), { manifests: snapshots, descriptor });
  const args = ["--console=plain", "-Pheadless", "run", "--args=" + relative(armed.jcm) + " --log-conf " + relative(path.join(path.dirname(armed.jcm), "logging.properties"))];
  const meta = { cell: n, run, t: Date.now(), compute_file: computeFile,
    compute_offset: fs.existsSync(computeFile) ? fs.statSync(computeFile).size : 0, command: ["./gradlew", ...args] };
  write(path.join(directory, "run.json"), meta);
  append(trace, { event: "gradle_start", t: meta.t });
  console.log(`Starting Gradle for cell ${n}, run ${run}/5. Stop it with Ctrl+C after the recorder prints the result.`);
  const child = spawn(path.join(ROOT, "gradlew"), args, { cwd: ROOT, env: { ...process.env, WOMAT_E11_TRACE: trace, WOMAT_E11_CELL: String(n) }, stdio: ["inherit", "pipe", "pipe"] });
  const log = fs.createWriteStream(path.join(directory, "gradle.log"));
  for (const stream of [child.stdout, child.stderr]) stream.on("data", chunk => { log.write(chunk); process.stdout.write(chunk); });
  const forward = signal => child.kill(signal);
  const interrupt = () => forward("SIGINT"), terminate = () => forward("SIGTERM");
  process.on("SIGINT", interrupt); process.on("SIGTERM", terminate);
  child.on("error", error => append(trace, { event: "launch_error", t: Date.now(), error: error.message }));
  await new Promise(resolve => child.on("close", (code, signal) => {
    append(trace, { event: "gradle_exit", t: Date.now(), code, signal });
    write(path.join(directory, "exit.json"), { code, signal }); log.end(); resolve();
  }));
  process.off("SIGINT", interrupt); process.off("SIGTERM", terminate);
}
function resetCache(n, out) {
  const backup = path.join(out, "cache-backups", `${Date.now()}-cell${n}`);
  let count = 0;
  for (const a of names(n)) for (const b of names(n)) {
    if (a >= b) continue;
    const key = `${a}__${b}`, file = path.join(ROOT, "generated/robots/interactions", key, "space.json");
    if (!fs.existsSync(file)) continue;
    fs.mkdirSync(backup, { recursive: true }); fs.renameSync(file, path.join(backup, key + ".json")); count++;
  }
  console.log(`Archived ${count} interaction cache metadata files in ${relative(backup)}. Start fresh AgentThing processes before the next Gradle run.`);
}
async function main() {
  const command = process.argv[2], args = parseArgs(process.argv.slice(3)), out = path.resolve(ROOT, args.out || OUT);
  if (!command || command === "help" || command === "--help") {
    console.log("E11: prepare-cells | watch --cell 3|4|5 | launch --cell 3|4|5 | reset-cache --cell 3|4|5 | analyze\nOptional --out DIR; watch --timeout 600; launch --compute-log FILE.\nwatch never starts or contacts robots. launch starts only Gradle, explicitly requested by the user.\nreset-cache is for stopped AgentThings between runs; it archives cached metadata.\nSee docs/e11_scaling.md."); return;
  }
  if (command === "prepare-cells") return prepareCells();
  if (command === "analyze") { console.table(analyze(out)); return; }
  const n = cellNumber(args.cell);
  if (command === "watch") {
    const timeout = Number(args.timeout || 600);
    if (!Number.isFinite(timeout) || timeout <= 0) throw Error("--timeout must be positive seconds");
    return watch(n, out, timeout);
  }
  if (command === "launch") return launch(n, out, path.resolve(ROOT, args["compute-log"] || "performance_log/interaction-compute.jsonl"));
  if (command === "reset-cache") return resetCache(n, out);
  throw Error("Unknown E11 command: " + command);
}
if (require.main === module) main().catch(error => { console.error(error.message); process.exitCode = 1; });
module.exports = { instrumentAgent, instrumentRoutes, preparePlans, summarize, analyze, readLines, cellCommands, cellMismatch, csv };
