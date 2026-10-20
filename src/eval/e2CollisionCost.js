#!/usr/bin/env node
// E2 -- cost of collision checking, with and without the bounding-box broad
// phase.
//
// Runs inside the LIVE cell: every robot's runtime driver already holds the
// contact test the preflight and the watchdog use (womatCollectionsTouch in
// 04-safety.lua), so the benchmark calls that function and nothing else. The
// only thing switched is womatBroadphaseEnabled, which is exactly what the
// manifest flag runtime.collisionBroadphase switches.
//
// Cell sizes are subsets of the five-robot cell. For a cell of k robots, each
// of Robot1..Robotk is checked against the obstacle shapes whose scene root
// belongs to Robot1..Robotk -- their bodies, grippers, walls and supports --
// plus every shape that belongs to no robot. Robots k+1..5 stay in the scene
// but are not in the collection, which for a contact test is the same as not
// being there. The robots keep the positions they have in the five-robot cell.
//
// Per robot, a fixed set of seeded random joint configurations is drawn inside
// the joint limits. Every configuration is checked with and without the broad
// phase, in alternating order, each timed over a few repetitions, and the two
// verdicts are compared: the broad phase must never change one.
//
// Nothing is moved for longer than one call: each batch captures the arm's
// posture, sets the configurations, and restores the posture before returning.
// Run it with the cell idle (lower Things served, no agents moving arms).

const fs = require("node:fs");
const path = require("node:path");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const DEFAULT_DESCRIPTOR = "generated/robots/Robot1_Robot2_Robot3_Robot4_Robot5.shared.json";
const DEFAULT_OUT = "performance_log/e2";

// Installed into each robot's driver. Uses only the driver's own globals:
// jointHandles, womatRobotCollisionCollection, womatSceneObstacleShapes,
// womatIsRenderOnlyShape, womatRootOf, womatPlainAlias, womatBroadphaseEntry,
// womatWorldBoxes, womatBroadphaseMargin, womatCollectionsTouch,
// womatCaptureSweepPosture, womatRestoreSweepPosture.
const BENCHMARK_LUA = String.raw`
function womatE2Now()
    return sim.getSystemTime()
end

function womatE2RootIndex(shape)
    local alias=womatPlainAlias(womatRootOf(shape))
    local digits=string.match(alias, "^Robot(%d+)")
    return digits and tonumber(digits) or nil, alias
end

function womatE2Finish()
    if womatE2Cells ~= nil then
        for _, cell in pairs(womatE2Cells) do
            if cell.collection ~= -1 then
                womatBroadphaseShapes[cell.collection]=nil
                pcall(sim.destroyCollection, cell.collection)
            end
        end
    end
    womatE2Cells=nil
    if womatE2BroadphaseWas ~= nil then
        womatBroadphaseEnabled=womatE2BroadphaseWas
    end
    return true
end

function womatE2Setup(seed, count, cells)
    womatE2Finish()
    womatE2BroadphaseWas=womatBroadphaseEnabled
    local lows={}
    local ranges={}
    for i=1,#jointHandles,1 do
        local cyclic, interval=sim.getJointInterval(jointHandles[i])
        if cyclic then
            lows[i]=-math.pi
            ranges[i]=2*math.pi
        else
            lows[i]=interval[1]
            ranges[i]=interval[2]
        end
    end
    -- Its own generator, not math.random: the driver's Lua state is shared
    -- with the running cell, which draws from math.random between and even
    -- during these calls, so the same seed gave different poses (measured for
    -- Robot1 and Robot5). The same linear congruential generator as
    -- seededRandom in interactionSpace.js.
    local state=math.floor(seed) % 4294967296
    if state == 0 then state=1 end
    local function random()
        state=(1664525*state+1013904223) % 4294967296
        return state/4294967296
    end
    womatE2Configs={}
    for c=1,count,1 do
        local q={}
        for i=1,#jointHandles,1 do
            q[i]=lows[i]+random()*ranges[i]
        end
        womatE2Configs[c]=q
    end
    womatE2Robot=womatRobotCollisionCollection()
    local robotShapes=0
    for _, handle in ipairs(sim.getCollectionObjects(womatE2Robot)) do
        if sim.getObjectType(handle) == sim.object_shape_type then
            robotShapes=robotShapes+1
        end
    end
    local shapes=womatSceneObstacleShapes()
    local roots={}
    womatE2Cells={}
    local info={robotShapes=robotShapes, joints=#jointHandles, cells={}, roots={}}
    for _, k in ipairs(cells) do
        local collection=-1
        local shapeCount=0
        for _, shape in ipairs(shapes) do
            if not womatIsRenderOnlyShape(shape) then
                local index, alias=womatE2RootIndex(shape)
                if index == nil or index <= k then
                    if collection == -1 then
                        collection=sim.createCollection(0)
                    end
                    sim.addItemToCollection(collection, sim.handle_single, shape, 0)
                    shapeCount=shapeCount+1
                    roots[alias]=index or 0
                end
            end
        end
        womatE2Cells[k]={collection=collection, shapes=shapeCount}
        info.cells[#info.cells+1]={cell=k, obstacleShapes=shapeCount}
    end
    for alias, index in pairs(roots) do
        info.roots[#info.roots+1]={alias=alias, robot=index}
    end
    return info
end

function womatE2CountOverlaps(entryA, entryB)
    local a=womatWorldBoxes(entryA)
    local b=womatWorldBoxes(entryB)
    local m=womatBroadphaseMargin
    local n=0
    for i=1,#a,1 do
        local p=a[i]
        for j=1,#b,1 do
            local q=b[j]
            if p[1] <= q[4]+m and q[1] <= p[4]+m
                and p[2] <= q[5]+m and q[2] <= p[5]+m
                and p[3] <= q[6]+m and q[3] <= p[6]+m then
                n=n+1
            end
        end
    end
    return n, #a, #b
end

function womatE2Time(cell, reps, broadphase)
    womatBroadphaseEnabled=broadphase
    local hit=false
    local started=womatE2Now()
    for r=1,reps,1 do
        hit=womatCollectionsTouch(womatE2Robot, cell) and true or false
    end
    return hit, (womatE2Now()-started)*1000/reps
end

function womatE2Run(k, first, last, reps)
    local cell=womatE2Cells and womatE2Cells[k]
    local out={}
    if cell == nil or cell.collection == -1 then
        return out
    end
    local posture=womatCaptureSweepPosture()
    local ok, err=pcall(function()
        if first == 1 then
            -- The first query of a new collection builds CoppeliaSim's
            -- collision caches; it is not a check the running cell ever pays.
            local q=womatE2Configs[1]
            for i=1,#jointHandles,1 do
                sim.setJointPosition(jointHandles[i], q[i])
            end
            womatE2Time(cell.collection, 1, false)
            womatE2Time(cell.collection, 1, true)
        end
        for c=first,last,1 do
            local q=womatE2Configs[c]
            for i=1,#jointHandles,1 do
                sim.setJointPosition(jointHandles[i], q[i])
            end
            local entryA=womatBroadphaseEntry(womatE2Robot)
            local entryB=womatBroadphaseEntry(cell.collection)
            local overlaps, boxesA, boxesB=-1, -1, -1
            if entryA and entryB then
                overlaps, boxesA, boxesB=womatE2CountOverlaps(entryA, entryB)
            end
            local hitWithout, msWithout, hitWith, msWith
            -- Alternating order, so neither mode always runs on warm caches.
            if c % 2 == 1 then
                hitWithout, msWithout=womatE2Time(cell.collection, reps, false)
                hitWith, msWith=womatE2Time(cell.collection, reps, true)
            else
                hitWith, msWith=womatE2Time(cell.collection, reps, true)
                hitWithout, msWithout=womatE2Time(cell.collection, reps, false)
            end
            out[#out+1]={config=c, hitWithout=hitWithout, hitWith=hitWith,
                msWithout=msWithout, msWith=msWith,
                overlaps=overlaps, robotShapes=boxesA, obstacleShapes=boxesB}
        end
    end)
    womatBroadphaseEnabled=womatE2BroadphaseWas
    womatRestoreSweepPosture(posture)
    if not ok then
        error(err)
    end
    return out
end
`;

// ---------------------------------------------------------------- pure parts

function robotIndex(name) {
  const match = String(name).match(/^Robot(\d+)/);
  return match ? Number(match[1]) : null;
}

function quantile(values, q) {
  if (values.length === 0) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const position = (sorted.length - 1) * q;
  const low = Math.floor(position);
  const high = Math.ceil(position);
  return sorted[low] + (sorted[high] - sorted[low]) * (position - low);
}

function mean(values) {
  return values.length === 0 ? null : values.reduce((sum, value) => sum + value, 0) / values.length;
}

// Standard error of the mean, relative to the mean: how far the reported mean
// time can be trusted with this many configurations.
function relativeStandardError(values) {
  if (values.length < 2) return null;
  const average = mean(values);
  const variance = values.reduce((sum, value) => sum + (value - average) ** 2, 0) / (values.length - 1);
  return average > 0 ? Math.sqrt(variance / values.length) / average : null;
}

// One summary row for a group of checks. Timing is reported over the
// collision-free configurations: a check that finds a contact stops at the
// first touching pair, so its time depends on where the contact is, and a
// preflight spends nearly all of its checks on free samples.
function summarizeChecks(checks) {
  const free = checks.filter((check) => !check.hitWithout);
  const timed = free.length > 0 ? free : checks;
  const without = timed.map((check) => check.msWithout);
  const withBroadphase = timed.map((check) => check.msWith);
  const pairs = checks.filter((check) => check.robotShapes > 0 && check.obstacleShapes > 0)
    .map((check) => check.robotShapes * check.obstacleShapes);
  const overlaps = free.map((check) => check.overlaps).filter((value) => value >= 0);
  const meanWithout = mean(without);
  const meanWith = mean(withBroadphase);
  return {
    checks: checks.length,
    free: free.length,
    collidingShare: checks.length === 0 ? null : 1 - free.length / checks.length,
    mismatches: checks.filter((check) => check.hitWithout !== check.hitWith).length,
    shapePairs: mean(pairs),
    pairsTestedMean: mean(overlaps),
    pairsTestedMedian: quantile(overlaps, 0.5),
    pairsTestedMax: overlaps.length ? Math.max(...overlaps) : null,
    msWithoutMean: meanWithout,
    msWithoutMedian: quantile(without, 0.5),
    msWithoutP95: quantile(without, 0.95),
    msWithoutRelSem: relativeStandardError(without),
    msWithMean: meanWith,
    msWithMedian: quantile(withBroadphase, 0.5),
    msWithP95: quantile(withBroadphase, 0.95),
    msWithRelSem: relativeStandardError(withBroadphase),
    speedup: meanWithout !== null && meanWith > 0 ? meanWithout / meanWith : null,
    msPerPairWithout: meanWithout !== null && mean(pairs) > 0 ? (meanWithout / mean(pairs)) * 1000 : null
  };
}

function csvCell(value) {
  if (value === null || value === undefined || (typeof value === "number" && !Number.isFinite(value))) return "nan";
  if (typeof value === "boolean") return value ? "1" : "0";
  if (typeof value === "number") return Number.isInteger(value) ? String(value) : value.toPrecision(6);
  return String(value);
}

function toCsv(rows, columns) {
  return [columns.join(","), ...rows.map((row) => columns.map((column) => csvCell(row[column])).join(","))].join("\n") + "\n";
}

const SUMMARY_COLUMNS = ["checks", "free", "collidingShare", "mismatches", "shapePairs",
  "pairsTestedMean", "pairsTestedMedian", "pairsTestedMax",
  "msWithoutMean", "msWithoutMedian", "msWithoutP95", "msWithoutRelSem", "msWithMean", "msWithMedian", "msWithP95", "msWithRelSem",
  "speedup", "msPerPairWithout"];

function cellName(k) {
  return ({ 2: "Two-robot", 3: "Three-robot", 4: "Four-robot", 5: "Five-robot" })[k] || `${k}-robot`;
}

function thesisTable(cellRows) {
  const number = (value, digits) => (value === null ? "\\tbd" : value.toFixed(digits));
  const thousands = (value) => (value === null ? "\\tbd" : Math.round(value).toLocaleString("en-US").replace(/,/g, "\\,"));
  return [
    "% Generated by src/eval/e2CollisionCost.js -- rows for tab:exp_e2.",
    "% Shape pairs and pairs tested exactly: mean per check; times: mean per collision-free check.",
    ...cellRows.map((row) => `        ${cellName(row.cell)} & ${thousands(row.shapePairs)} & ${number(row.pairsTestedMean, 1)} & `
      + `${number(row.msWithoutMean, 2)} & ${number(row.msWithMean, 2)} & ${number(row.speedup, 1)} \\\\`)
  ].join("\n") + "\n";
}

// ---------------------------------------------------------------- live run

function first(value) {
  return Array.isArray(value) ? value[0] : value;
}

async function scriptOf(sim, name) {
  const objectPath = `/${name}`;
  const handle = Number(first(await sim.getObject(objectPath)));
  const script = Number(first(await sim.getScript(await sim.scripttype_customizationscript, handle, objectPath)));
  if (!(script >= 0)) throw new Error(`${name} has no driver script -- is the cell served (npm run robot:serve)?`);
  return script;
}

async function run(options) {
  const { connect, executeInScript } = require("../generator/core/simConnection");
  const descriptor = JSON.parse(fs.readFileSync(options.descriptorPath, "utf8"));
  const names = descriptor.robots.map((robot) => robot.name)
    .filter((name) => !options.robots || options.robots.includes(name));
  const cells = options.cells;
  const { sim } = await connect({ robwotRoot: path.join(REPO_ROOT, "external", "RobWoT") });
  const checks = [];
  const setups = [];

  for (const name of names) {
    const index = robotIndex(name);
    const robotCells = cells.filter((k) => index !== null && k >= index);
    if (robotCells.length === 0) continue;
    const script = await scriptOf(sim, name);
    await executeInScript(sim, script, BENCHMARK_LUA + "\nreturn true");
    const setup = await executeInScript(sim, script,
      `return womatE2Setup(${options.seed * 100 + index}, ${options.configs}, {${robotCells.join(",")}})`);
    setups.push({ robot: name, ...setup });
    console.log(`${name}: ${setup.robotShapes} robot shapes; obstacle shapes per cell `
      + setup.cells.map((cell) => `${cell.cell}: ${cell.obstacleShapes}`).join(", "));
    try {
      for (const k of robotCells) {
        const started = Date.now();
        for (let from = 1; from <= options.configs; from += options.batch) {
          const to = Math.min(options.configs, from + options.batch - 1);
          const batch = await executeInScript(sim, script, `return womatE2Run(${k}, ${from}, ${to}, ${options.reps})`);
          for (const check of Array.isArray(batch) ? batch : []) {
            checks.push({ robot: name, cell: k, ...check });
          }
        }
        const done = summarizeChecks(checks.filter((check) => check.robot === name && check.cell === k));
        console.log(`  cell ${k}: ${done.free}/${done.checks} free, without ${done.msWithoutMean?.toFixed(2)} ms, `
          + `with ${done.msWithMean?.toFixed(2)} ms, x${done.speedup?.toFixed(1)}, `
          + `${done.mismatches} verdict mismatch(es) (${((Date.now() - started) / 1000).toFixed(0)} s)`);
      }
    } finally {
      await executeInScript(sim, script, "return womatE2Finish()");
    }
  }
  return { checks, setups };
}

function writeResults(outDir, { checks, setups }, options) {
  fs.mkdirSync(outDir, { recursive: true });
  fs.writeFileSync(path.join(outDir, "e2_checks.csv"), toCsv(checks,
    ["robot", "cell", "config", "hitWithout", "hitWith", "msWithout", "msWith", "overlaps", "robotShapes", "obstacleShapes"]));

  const robotRows = [];
  for (const robot of [...new Set(checks.map((check) => check.robot))]) {
    for (const cell of options.cells) {
      const group = checks.filter((check) => check.robot === robot && check.cell === cell);
      if (group.length) robotRows.push({ robot, cell, ...summarizeChecks(group) });
    }
  }
  fs.writeFileSync(path.join(outDir, "e2_robot_summary.csv"), toCsv(robotRows, ["robot", "cell", ...SUMMARY_COLUMNS]));

  const cellRows = options.cells
    .map((cell) => ({ cell, group: checks.filter((check) => check.cell === cell) }))
    .filter(({ group }) => group.length > 0)
    .map(({ cell, group }) => ({ cell, robots: new Set(group.map((check) => check.robot)).size, ...summarizeChecks(group) }));
  fs.writeFileSync(path.join(outDir, "e2_cell_summary.csv"), toCsv(cellRows, ["cell", "robots", ...SUMMARY_COLUMNS]));
  fs.writeFileSync(path.join(outDir, "e2_table.tex"), thesisTable(cellRows));
  fs.writeFileSync(path.join(outDir, "e2_setup.json"), JSON.stringify({
    ...options, finishedAt: new Date().toISOString(), robots: setups
  }, null, 2) + "\n");
  return cellRows;
}

// ---------------------------------------------------------------- CLI

function cliArguments(argv) {
  const args = {};
  for (let index = 0; index < argv.length; index += 1) {
    if (!argv[index].startsWith("--")) continue;
    const next = argv[index + 1];
    if (next === undefined || next.startsWith("--")) {
      args[argv[index].slice(2)] = true;
    } else {
      args[argv[index].slice(2)] = next;
      index += 1;
    }
  }
  return args;
}

function positiveInteger(value, name) {
  const number = Number(value);
  if (!Number.isInteger(number) || number <= 0) throw new Error(`--${name} must be a positive integer, got ${value}.`);
  return number;
}

async function main(argv) {
  const args = cliArguments(argv);
  if (args.help) {
    console.log([
      "Usage: node src/eval/e2CollisionCost.js [--configs 200] [--reps 3] [--seed 1] [--batch 10]",
      "                                        [--cells 2,3,4,5] [--robots Robot1,Robot2] [--out performance_log/e2]",
      "Needs the five-robot cell served (npm run robot:serve ...) and idle."
    ].join("\n"));
    return 0;
  }
  const options = {
    descriptorPath: path.resolve(REPO_ROOT, args.descriptor || DEFAULT_DESCRIPTOR),
    configs: positiveInteger(args.configs || 200, "configs"),
    reps: positiveInteger(args.reps || 3, "reps"),
    seed: positiveInteger(args.seed || 1, "seed"),
    batch: positiveInteger(args.batch || 10, "batch"),
    cells: String(args.cells || "2,3,4,5").split(",").map((value) => positiveInteger(value.trim(), "cells")),
    robots: args.robots ? String(args.robots).split(",").map((name) => name.trim()) : null
  };
  const outDir = path.resolve(REPO_ROOT, args.out || DEFAULT_OUT);
  const result = await run(options);
  const cellRows = writeResults(outDir, result, { ...options, descriptorPath: path.relative(REPO_ROOT, options.descriptorPath) });
  console.log("\nCell         pairs   tested   without[ms]  with[ms]  speed-up  mismatches");
  for (const row of cellRows) {
    console.log(`${cellName(row.cell).padEnd(12)} ${String(Math.round(row.shapePairs)).padStart(6)} `
      + `${row.pairsTestedMean.toFixed(1).padStart(8)} ${row.msWithoutMean.toFixed(2).padStart(12)} `
      + `${row.msWithMean.toFixed(2).padStart(9)} ${row.speedup.toFixed(1).padStart(9)} ${String(row.mismatches).padStart(11)}`);
  }
  console.log(`Results: ${outDir}`);
  return 0;
}

if (require.main === module) {
  main(process.argv.slice(2)).then((code) => process.exit(code), (error) => {
    console.error(error);
    process.exit(1);
  });
}

module.exports = { BENCHMARK_LUA, robotIndex, summarizeChecks, thesisTable, toCsv };
