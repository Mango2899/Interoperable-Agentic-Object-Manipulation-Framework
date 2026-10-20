const assert = require("node:assert/strict");
const { execFileSync } = require("node:child_process");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const {
  analyze,
  checkpointAt,
  nestedOccupiedVoxels,
  occupiedVoxels,
  overlapMetrics,
  parseProgress,
  parseSeeds,
  parseWorkspaceRows,
  reachablePrefix,
  summarize
} = require("./e1Convergence");
const {
  createContext,
  instrumentSamplingLoop,
  prepareWorkspaceDriverScript,
  sharedSamplingObstacles
} = require("../generator/robwotBridge");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const VENDOR_DRIVER = path.join(REPO_ROOT, "external", "RobWoT", "Library", "GenerateRobotDescription", "robot_driver_workspace.lua");

function luacAvailable() {
  try {
    execFileSync("luac", ["-v"], { stdio: "ignore" });
    return true;
  } catch {
    return false;
  }
}

function assertLuaParses(source) {
  const file = path.join(fs.mkdtempSync(path.join(os.tmpdir(), "e1-lua-")), "driver.lua");
  fs.writeFileSync(file, source);
  execFileSync("luac", ["-p", file]);
}

test("seeds are read as lists and ranges", () => {
  assert.deepEqual(parseSeeds("1-3,7"), [1, 2, 3, 7]);
  assert.deepEqual(parseSeeds("5"), [5]);
  assert.throws(() => parseSeeds("3-1"));
  assert.throws(() => parseSeeds("a"));
});

test("a prefix is taken over written rows, and only then filtered to reachable points", () => {
  const rows = parseWorkspaceRows("X,Y,Z,Collision_state\n0,0,0,1\n1,0,0,0\n2,0,0,1\n3,0,0,1\n");
  const progress = parseProgress("Samples,Rows,Seconds\n0,0,0\n2,2,0.5\n4,3,1.0\n");
  // Four samples wrote three rows: one colliding sample went below the floor.
  assert.equal(checkpointAt(progress, 4).rows, 3);
  assert.deepEqual(reachablePrefix(rows, checkpointAt(progress, 2).rows).map((p) => p.x), [0]);
  assert.deepEqual(reachablePrefix(rows, 3).map((p) => p.x), [0, 2]);
  assert.throws(() => checkpointAt(progress, 3), /No progress checkpoint at 3/);
  assert.throws(() => reachablePrefix(rows, 9), /CSV holds 4/);
});

test("the grid is anchored at the world origin, not at the cloud", () => {
  const near = [{ x: 0.12, y: 0.31, z: 0.05 }];
  const withFarPoint = [...near, { x: -4, y: -4, z: -4 }];
  const alone = occupiedVoxels(near, 0.05);
  const joined = occupiedVoxels(withFarPoint, 0.05);
  // Adding a far point must not move the voxel the near one falls in.
  assert.ok([...alone].every((key) => joined.has(key)));
  assert.deepEqual([...alone], ["2,6,1"]);
});

test("nested prefix voxel sets equal the sets of each prefix built on its own", () => {
  const points = Array.from({ length: 200 }, (_, i) => ({ x: Math.sin(i) * 0.4, y: Math.cos(i * 1.3) * 0.4, z: (i % 17) * 0.02 }));
  const snapshots = nestedOccupiedVoxels(points, [0, 10, 75, 200], 0.05);
  for (const boundary of [0, 10, 75, 200]) {
    assert.deepEqual([...snapshots.get(boundary)].sort(), [...occupiedVoxels(points.slice(0, boundary), 0.05)].sort());
  }
  assert.throws(() => nestedOccupiedVoxels(points, [201], 0.05));
});

test("a subset of the reference has Jaccard equal to its coverage and full precision", () => {
  const reference = new Set(["0,0,0", "1,0,0", "2,0,0", "3,0,0"]);
  const subset = new Set(["0,0,0", "1,0,0", "2,0,0"]);
  const metrics = overlapMetrics(subset, reference);
  assert.equal(metrics.jaccard, 0.75);
  assert.equal(metrics.coverage, 0.75);
  assert.equal(metrics.precision, 1);
  const stray = overlapMetrics(new Set(["0,0,0", "9,9,9"]), reference);
  assert.equal(stray.jaccard, 1 / 5);
  assert.equal(stray.precision, 0.5);
});

test("summaries ignore missing values and report the band columns", () => {
  const stats = summarize([1, 2, 3, 4, null, Number.NaN]);
  assert.equal(stats.n, 4);
  assert.equal(stats.median, 2.5);
  assert.equal(stats.q25, 1.75);
  assert.equal(stats.q75, 3.25);
  assert.equal(stats.min, 1);
  assert.equal(stats.max, 4);
  assert.equal(summarize([]).median, null);
});

// A two-robot cell with seeded synthetic clouds, run through the whole
// analysis: tables, per-pair files and the figure file must all come out, and
// Jaccard must rise to the reference as the sample count grows.
test("analyze writes the E1 tables and figures for a synthetic cell", () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "e1-analyze-"));
  const samplesDir = path.join(dir, "samples");
  const outDir = path.join(dir, "out");
  const descriptorPath = path.join(dir, "cell.shared.json");
  fs.writeFileSync(descriptorPath, JSON.stringify({
    environmentScene: "unused.ttt",
    interactionSpace: { targetVoxelCount: 200, proximityFactor: 1.25, sampleSize: 2000, spacingPercentile: 0.75, minVoxelSize: 0, maxVoxelSize: 0.2, minimumSharedVoxels: 5, bidirectional: true, seed: 42 },
    robots: [
      { name: "RobotA", offset: [0, 0, 0], orientationDeg: [0, 0, 0] },
      { name: "RobotB", offset: [0.6, 0, 0], orientationDeg: [0, 0, 90] }
    ]
  }));
  const samples = 4000;
  for (const seed of [1, 2, 3]) {
    const seedDir = path.join(samplesDir, `seed_0${seed}`);
    fs.mkdirSync(seedDir, { recursive: true });
    for (const name of ["RobotA", "RobotB"]) {
      let state = seed * 7919 + name.length;
      const random = () => {
        state = (1664525 * state + 1013904223) >>> 0;
        return state / 0x100000000;
      };
      const lines = ["X,Y,Z,Collision_state"];
      const progress = ["Samples,Rows,Seconds"];
      for (let count = 0; count < samples; count += 1) {
        if (count % 1000 === 0) progress.push(`${count},${lines.length - 1},${(count / 1000).toFixed(4)}`);
        const radius = 0.5 * Math.cbrt(random());
        const theta = random() * 2 * Math.PI;
        const phi = Math.acos(2 * random() - 1);
        const z = radius * Math.cos(phi);
        if (z < -0.2) continue; // "below the floor": sampled but never written
        lines.push(`${radius * Math.sin(phi) * Math.cos(theta)},${radius * Math.sin(phi) * Math.sin(theta)},${z},${random() < 0.9 ? 1 : 0}`);
      }
      progress.push(`${samples},${lines.length - 1},${(samples / 1000).toFixed(4)}`);
      fs.writeFileSync(path.join(seedDir, `${name}_data_point.csv`), lines.join("\n") + "\n");
      fs.writeFileSync(path.join(seedDir, `${name}_sampling_progress.csv`), progress.join("\n") + "\n");
    }
  }

  const result = analyze({ descriptorPath, samplesDir, outDir, counts: [1000, 2000, 4000], band: "iqr" });
  assert.deepEqual(result.seeds, [1, 2, 3]);
  assert.equal(result.maxCount, 4000);
  for (const file of ["e1_runs.csv", "e1_summary.csv", "e1_reference.csv", "e1_sampling_runs.csv",
    "e1_sampling_summary.csv", "e1_figures.tex", "pairs/RobotA__RobotB.csv", "robots/RobotA.csv"]) {
    assert.ok(fs.existsSync(path.join(outDir, file)), `${file} missing`);
  }
  const summary = fs.readFileSync(path.join(outDir, "pairs", "RobotA__RobotB.csv"), "utf8").trim().split("\n");
  const header = summary[0].split(",");
  const column = (name) => summary.slice(1).map((line) => Number(line.split(",")[header.indexOf(name)]));
  const jaccard = column("jaccard_median");
  assert.equal(jaccard.length, 3);
  assert.ok(jaccard.every((value) => value > 0 && value <= 1));
  assert.ok(jaccard[0] < jaccard[2], `Jaccard should grow with N: ${jaccard}`);
  assert.ok(column("jaccard_q25").every((low, index) => low <= jaccard[index] + 1e-12));
  assert.ok(column("volumeAuto_median").every((value) => value > 0));
  const figures = fs.readFileSync(path.join(outDir, "e1_figures.tex"), "utf8");
  assert.match(figures, /xmode=log/);
  assert.match(figures, /fill between\[of=/);
  assert.match(figures, /jaccard_q25/);
});

test("the sampler is seeded and records progress only when asked", () => {
  const vendor = fs.readFileSync(VENDOR_DRIVER, "utf8");
  assert.equal(instrumentSamplingLoop(vendor, {}), vendor);
  const seeded = instrumentSamplingLoop(vendor, { seed: 7, progressEvery: 1000 });
  assert.match(seeded, /if generateRandom then\n\s+math\.randomseed\(7\)/);
  assert.match(seeded, /for cnt=1,randomSteps,1 do\n\s+womatSamplingCheckpoint\(cnt-1,/);
  assert.match(seeded, /womatSamplingCheckpoint\(randomSteps, \(#points\+#collisionpoints\)\/\/3, true\)\n\s+file:close\(\)/);
  assert.throws(() => instrumentSamplingLoop(vendor, { seed: 1.5 }), /integer/);
  assert.throws(() => instrumentSamplingLoop("no loop here", { seed: 1 }), /exactly once/);
  if (luacAvailable()) assertLuaParses(seeded);
});

test("the generated workspace driver culls unreachable obstacles unless told not to", () => {
  const ctx = createContext(path.join(REPO_ROOT, "generated", "robots", "Robot5", "manifest.json"));
  const withSeed = {
    ...ctx,
    manifest: { ...ctx.manifest, pipeline: { ...ctx.manifest.pipeline, workspaceSeed: 3, workspaceProgressLog: true, samplingCullMargin: 0.3 } }
  };
  const script = prepareWorkspaceDriverScript(withSeed);
  assert.match(script, /womatObstacleCulling=true/);
  assert.match(script, /womatObstacleCullMargin=0\.3\b/);
  assert.match(script, /math\.randomseed\(3\)/);
  assert.match(script, /distance > cullRadius/);
  if (luacAvailable()) assertLuaParses(script);

  const off = prepareWorkspaceDriverScript({
    ...ctx,
    manifest: { ...ctx.manifest, pipeline: { ...ctx.manifest.pipeline, samplingObstacleCulling: false } }
  });
  assert.match(off, /womatObstacleCulling=false/);
  assert.doesNotMatch(off, /math\.randomseed/);
  assert.throws(() => prepareWorkspaceDriverScript({
    ...ctx,
    manifest: { ...ctx.manifest, pipeline: { ...ctx.manifest.pipeline, samplingCullMargin: -1 } }
  }), /samplingCullMargin/);
});

test("a manual sampling obstacle list replaces the cell's and names what it leaves out", (t) => {
  const cell = ["Robot1_wall", "Robot5_wall", "Robot1_grip_support"];
  const plain = { manifest: { name: "Robot5", pipeline: {} } };
  assert.deepEqual(sharedSamplingObstacles(plain, cell), cell);

  const warnings = [];
  t.mock.method(console, "warn", (message) => warnings.push(message));
  const override = { manifest: { name: "Robot5", pipeline: { sharedSamplingObstacles: ["Robot5_wall"] } } };
  assert.deepEqual(sharedSamplingObstacles(override, cell), ["Robot5_wall"]);
  assert.equal(warnings.length, 1);
  assert.match(warnings[0], /Robot1_wall, Robot1_grip_support/);
  assert.throws(() => sharedSamplingObstacles({ manifest: { name: "R", pipeline: { sharedSamplingObstacles: "Robot5_wall" } } }, cell));
});
