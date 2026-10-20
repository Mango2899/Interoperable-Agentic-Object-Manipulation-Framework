const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const {
  estimateInteractionSpace,
  loadReachableCsvShared,
  reachableTreeShared,
  writeFileAtomic,
  estimateSpacing,
  InteractionSpaceController,
  canonicalPairKey,
  pointToVoxel,
  targetForRobot,
  toWorld
} = require("./interactionSpace");

function lattice(x0, x1, y0 = 0, y1 = 2, z0 = 0, z1 = 2) {
  const points = [];
  for (let x = x0; x <= x1; x += 1) {
    for (let y = y0; y <= y1; y += 1) {
      for (let z = z0; z <= z1; z += 1) points.push({ x, y, z });
    }
  }
  return points;
}

test("estimates native spacing robustly and ignores duplicate samples", () => {
  const points = [...lattice(0, 2), { x: 0, y: 0, z: 0 }];
  assert.equal(estimateSpacing(points, 100, 0.75, 42), 1);
});

test("estimates once and intersects both grids using an explicit shared origin", () => {
  const result = estimateInteractionSpace(lattice(-2, 0), lattice(-1, 1), {
    targetVoxelCount: 27,
    proximityFactor: 1.25,
    sampleSize: 100,
    spacingPercentile: 0.5,
    seed: 42
  });
  assert.equal(result.success, true);
  assert.deepEqual(result.origin, { x: -2, y: 0, z: 0 });
  assert.equal(result.voxelSize, 1);
  assert.equal(result.voxels.length, 18);
  assert.deepEqual(pointToVoxel({ x: -1, y: 0, z: 0 }, result.origin, 1), [1, 0, 0]);
});

test("returns a typed failure when workspace bounds do not overlap", () => {
  const result = estimateInteractionSpace(lattice(0, 1), lattice(10, 11));
  assert.deepEqual(result, { success: false, reason: "no_geometric_overlap" });
});

test("an empty intersection says how far apart the clouds actually are", () => {
  // "voxel_intersection_empty" on its own reads like a defect in the estimator,
  // and it is normally a fact about the cell: two arms whose reach spheres
  // overlap by half a metre can have sampled workspaces that never meet,
  // because a sphere is not the shape of a workspace. Without a distance the
  // refusal is unanswerable -- there is no space file to inspect afterwards,
  // since none is written for a pair that shares nothing. Live cell: Robot1 and
  // Robot3 pass the sphere test with 0.63m to spare and their clouds' nearest
  // approach is 0.029m.
  //
  // Two clouds either side of a diagonal gap: their bounding boxes overlap
  // almost entirely, so this is not the no_geometric_overlap case, and no point
  // of one is near any point of the other.
  const cloud = (seed, count, keep) => {
    let state = seed;
    const next = () => {
      state = (1103515245 * state + 12345) % 2147483648;
      return state / 2147483648;
    };
    const points = [];
    while (points.length < count) {
      const point = { x: next(), y: next(), z: next() };
      if (keep(point)) points.push(point);
    }
    return points;
  };
  const below = cloud(7, 600, (point) => point.x + point.y < 1);
  const above = cloud(99, 600, (point) => point.x + point.y > 1.25);

  const result = estimateInteractionSpace(below, above, {
    targetVoxelCount: 1000, sampleSize: 300, spacingPercentile: 0.5, seed: 42
  });
  assert.equal(result.success, false);
  assert.equal(result.reason, "voxel_intersection_empty");

  // The reported distance is the real one, not an estimate off a subsample:
  // it is the number that says whether a pair is marginal or nowhere near.
  let brute = Number.POSITIVE_INFINITY;
  for (const left of below) {
    for (const right of above) {
      brute = Math.min(brute, Math.hypot(left.x - right.x, left.y - right.y, left.z - right.z));
    }
  }
  assert.equal(result.nearestApproach, brute);
  // Symmetric, like every other answer this estimator gives.
  assert.equal(
    estimateInteractionSpace(above, below, {
      targetVoxelCount: 1000, sampleSize: 300, spacingPercentile: 0.5, seed: 42
    }).nearestApproach,
    brute
  );
});

test("rejects a maximum voxel size below native sampling resolution", () => {
  assert.throws(
    () => estimateInteractionSpace(lattice(0, 2), lattice(1, 3), { maxVoxelSize: 0.5 }),
    /native-spacing/
  );
});

test("transforms local workspace coordinates with placement and yaw", () => {
  const point = toWorld({ x: 1, y: 0, z: 0.5 }, { offset: [2, 3, 0.25], rotationDeg: 180 });
  assert.ok(Math.abs(point.x - 1) < 1e-12);
  assert.ok(Math.abs(point.y - 3) < 1e-12);
  assert.equal(point.z, 0.75);
});

test("transforms a model-frame pitch correction in three dimensions", () => {
  const point = toWorld(
    { x: 1, y: 0, z: 0 },
    { offset: [2, 3, 0.25], orientationDeg: [0, -90, 0] }
  );
  assert.ok(Math.abs(point.x - 2) < 1e-12);
  assert.equal(point.y, 3);
  assert.ok(Math.abs(point.z - 1.25) < 1e-12);
});

test("composes Robot3 pitch and yaw in CoppeliaSim Euler order", () => {
  const point = toWorld(
    { x: 1, y: 0, z: 0 },
    { offset: [2, 3, 0.25], orientationDeg: [0, -90, 180] }
  );
  assert.ok(Math.abs(point.x - 2) < 1e-12);
  assert.ok(Math.abs(point.y - 3) < 1e-12);
  assert.ok(Math.abs(point.z + 0.75) < 1e-12);
});

test("Robot3 world-to-local TCP target round-trips through its CoppeliaSim placement", () => {
  const robot = { name: "Robot3", offset: [2.1, -0.2, 0], orientationDeg: [0, -90, 180] };
  const manifest = { runtime: { gripper: { jawCenterOffset: [0, -0.18, -0.005] } } };
  const cubeWorld = { x: 1.4949175655148954, y: -0.02930865557405804, z: 0.392258918630207 };
  const target = targetForRobot(cubeWorld, robot, manifest);
  const tcpWorld = toWorld(target.localTcp, robot);

  for (const axis of ["x", "y", "z"]) {
    assert.ok(Math.abs(tcpWorld[axis] - target.worldTcp[axis]) < 1e-12);
  }
  assert.ok(target.localTcp.x < 0, "Robot3 local x must keep this world target above the floor");
  assert.ok(target.localTcp.z > 0, "Robot3 local z must point toward the interaction region");
});

test("uses stable pair keys independent of caller order", () => {
  assert.equal(canonicalPairKey("Robot2", "Robot1"), "Robot1__Robot2");
  assert.equal(canonicalPairKey("Robot10", "Robot2"), "Robot2__Robot10");
});

test("only the pair owner computes and a restarted owner loads the versioned cache", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-pair-cache-"));
  const writeCsv = (name, points) => {
    const relative = `${name}.csv`;
    fs.writeFileSync(path.join(root, relative), ["X,Y,Z,Reachable", ...points.map((p) => `${p.x},${p.y},${p.z},1`)].join("\n"));
    fs.writeFileSync(path.join(root, `${name}.manifest.json`), JSON.stringify({
      name,
      generatedFiles: { robotDataPoints: relative },
      runtime: { workspaceTolerance: 2, gripper: { jawCenterOffset: [0, 0, 0] } }
    }));
    return relative;
  };
  const robot1Csv = writeCsv("Robot1", lattice(0, 2));
  const robot2Csv = writeCsv("Robot2", lattice(1, 3));
  const descriptor = {
    interactionSpace: {
      pairs: [{
        key: "Robot1__Robot2",
        robots: ["Robot1", "Robot2"],
        ownerRobot: "Robot1",
        output: { csv: "cache/space.csv", metadata: "cache/space.json" }
      }],
      targetVoxelCount: 27,
      proximityFactor: 1.25,
      sampleSize: 100,
      spacingPercentile: 0.5,
      seed: 42
    },
    robots: [
      { name: "Robot1", manifest: "Robot1.manifest.json", workspaceCsv: robot1Csv, offset: [0, 0, 0], rotationDeg: 0 },
      { name: "Robot2", manifest: "Robot2.manifest.json", workspaceCsv: robot2Csv, offset: [0, 0, 0], rotationDeg: 0 }
    ]
  };
  fs.writeFileSync(path.join(root, "cell.json"), JSON.stringify(descriptor));
  const manifest = (name) => ({ name, sharedEnvironment: { descriptor: "cell.json" } });

  const owner = new InteractionSpaceController({ repoRoot: root, manifest: manifest("Robot1") });
  const first = owner.build("Robot2");
  assert.equal(first.success, true);
  assert.equal(first.cached, false);
  // A synthetic cell of 100 samples and 27 target voxels is deliberately smaller
  // than any real rendezvous, so the viability floor is lifted here the same way
  // the candidate minimum already is. This test is about caching and ranking
  // mechanics; whether the overlap is big enough to hand a cube across is
  // asserted against the real cell instead.
  const ranking = owner.rank("Robot2", {
    minZ: 0, maxZ: 2, minimumCandidates: 1, minimumSharedVoxels: 0
  });
  assert.equal(ranking.success, true);
  assert.ok(ranking.candidates.length > 0);
  assert.ok(ranking.candidates.every((candidate) =>
    Number.isFinite(candidate.centrality) && Number.isFinite(candidate.density) &&
    Number.isFinite(candidate.giverSupport) && Number.isFinite(candidate.receiverSupport)
  ));
  const committed = owner.commit("Robot2", ranking.candidates[0].id);
  assert.equal(committed.success, true);
  assert.match(committed.id, /^handover_/);
  const second = new InteractionSpaceController({ repoRoot: root, manifest: manifest("Robot1") }).build("Robot2");
  assert.equal(second.success, true);
  assert.equal(second.cached, true);
  assert.equal(second.version, first.version);

  // Ownership decides who may OPEN a handover, not who may know the geometry.
  // Robot2 asking about its own shared space with Robot1 must get the same
  // answer Robot1 gets -- same version, same voxels -- and must get it from the
  // cache Robot1 already wrote rather than recomputing a near-copy of it.
  const partner = new InteractionSpaceController({ repoRoot: root, manifest: manifest("Robot2") }).build("Robot1");
  assert.equal(partner.success, true);
  assert.equal(partner.cached, true);
  assert.equal(partner.version, first.version);
  assert.equal(partner.voxelCount, first.voxelCount);
  assert.equal(partner.voxelSize, first.voxelSize);
});

test("the shared space of two robots does not depend on which of them asked", () => {
  // The estimator used derived seeds per argument, so Robot1/Robot2 came out as
  // 358 voxels one way round and 360 the other. Two robots cannot coordinate in
  // a space they do not agree on, so this is a property of the estimator itself
  // and not something the callers are trusted to preserve.
  const cloud = (seed, count, spread) => {
    let state = seed;
    const next = () => {
      state = (1103515245 * state + 12345) % 2147483648;
      return state / 2147483648;
    };
    return Array.from({ length: count }, () => ({
      x: next() * spread, y: next() * spread, z: next() * spread
    }));
  };
  const first = cloud(7, 400, 1);
  const second = cloud(99, 350, 1).map((point) => ({ ...point, x: point.x + 0.35 }));
  const options = { targetVoxelCount: 40, sampleSize: 200, spacingPercentile: 0.5, seed: 42, bidirectional: true };

  const forward = estimateInteractionSpace(first, second, options);
  const reverse = estimateInteractionSpace(second, first, options);
  assert.equal(forward.success, true);
  assert.equal(reverse.success, true);
  assert.equal(forward.voxelSize, reverse.voxelSize);
  assert.deepEqual(forward.origin, reverse.origin);
  assert.deepEqual(
    forward.voxels.map((voxel) => voxel.index),
    reverse.voxels.map((voxel) => voxel.index)
  );
  assert.equal(forward.diagnostics.orderIndependent, true);
});

test("a base that has moved invalidates the pair's cached space instead of reusing it", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-moved-base-"));
  const writeCsv = (name, points) => {
    const relative = `${name}.csv`;
    fs.writeFileSync(path.join(root, relative), ["X,Y,Z,Reachable", ...points.map((p) => `${p.x},${p.y},${p.z},1`)].join("\n"));
    fs.writeFileSync(path.join(root, `${name}.manifest.json`), JSON.stringify({
      name,
      generatedFiles: { robotDataPoints: relative },
      runtime: { workspaceTolerance: 2, gripper: { jawCenterOffset: [0, 0, 0] } }
    }));
    return relative;
  };
  const descriptor = {
    interactionSpace: {
      pairs: [{
        key: "Robot1__Robot2",
        robots: ["Robot1", "Robot2"],
        ownerRobot: "Robot1",
        output: { csv: "cache/space.csv", metadata: "cache/space.json" }
      }],
      targetVoxelCount: 27,
      proximityFactor: 1.25,
      sampleSize: 100,
      spacingPercentile: 0.5,
      seed: 42
    },
    robots: [
      { name: "Robot1", manifest: "Robot1.manifest.json", workspaceCsv: writeCsv("Robot1", lattice(0, 2)), offset: [0, 0, 0], rotationDeg: 0 },
      { name: "Robot2", manifest: "Robot2.manifest.json", workspaceCsv: writeCsv("Robot2", lattice(0, 2)), offset: [1, 0, 0], rotationDeg: 0 }
    ]
  };
  fs.writeFileSync(path.join(root, "cell.json"), JSON.stringify(descriptor));
  const controller = () => new InteractionSpaceController({
    repoRoot: root,
    manifest: { name: "Robot1", sharedEnvironment: { descriptor: "cell.json" } }
  });

  const asGenerated = controller().build("Robot2");
  assert.equal(asGenerated.success, true);
  assert.equal(asGenerated.placementSource, "descriptor");

  // Restarting on the same layout is the case the cache exists for.
  const restarted = controller().build("Robot2", {
    placements: { Robot2: { offset: [1, 0, 0], orientationDeg: [0, 0, 0] } }
  });
  assert.equal(restarted.cached, true);
  assert.equal(restarted.version, asGenerated.version);

  // Reporting a base half a metre from where the cell was generated must not
  // hand back the space that was computed for the old arrangement.
  const moved = controller().build("Robot2", {
    placements: { Robot2: { offset: [1.5, 0, 0], orientationDeg: [0, 0, 0] } }
  });
  assert.equal(moved.success, true);
  assert.equal(moved.cached, false);
  assert.notEqual(moved.version, asGenerated.version);
  assert.equal(moved.placementSource, "scene");
  assert.deepEqual(moved.placements.Robot2.offset, [1.5, 0, 0]);

  // And the exported artifact says which arrangement it belongs to, so a stale
  // one is recognisable without re-deriving it.
  const exported = JSON.parse(fs.readFileSync(path.join(root, "cache/space.json"), "utf8"));
  assert.equal(exported.version, moved.version);
  assert.deepEqual(exported.placements.Robot2.offset, [1.5, 0, 0]);
});

test("an unpinned wrist is not scored against an approach leg it never travels", () => {
  const placement = { offset: [0, 0, 0], orientationDeg: [0, 0, 0] };
  const gripper = { toolAxis: [0, 1, 0], handoverStandoffDistance: 0.15 };
  const controllerFor = (receiverManifest) => {
    const controller = Object.create(InteractionSpaceController.prototype);
    controller.ctx = { manifest: { name: "Giver" } };
    controller.descriptor = { interactionSpace: {} };
    controller.spaces = new Map();
    const space = {
      owner: placement,
      partner: { offset: [1, 0, 0], orientationDeg: [0, 0, 180] },
      ownerManifest: { runtime: { gripper } },
      partnerManifest: receiverManifest,
      targetRobot: "Receiver"
    };
    controller.spaces.set("Receiver", space);
    return { controller, space };
  };
  const cubeWorld = { x: 0.5, y: 0, z: 0.5 };

  // Pinned, both arms contribute a margin: each travels a commanded straight
  // leg, and either one crossing its own base axis is disqualifying.
  const pinned = controllerFor({ runtime: { gripper } });
  const withLock = pinned.controller.posturePenalty(cubeWorld, pinned.space, "Receiver");
  assert.ok(Number.isFinite(withLock.giverMargin));
  assert.ok(Number.isFinite(withLock.receiverMargin));

  // Unpinned, the receiver has no commanded leg at all, so its term drops out
  // rather than rejecting rendezvous points its free wrist can reach.
  const free = controllerFor({ runtime: { enableHandoverPoseLock: false, gripper } });
  const withoutLock = free.controller.posturePenalty(cubeWorld, free.space, "Receiver");
  assert.equal(withoutLock.receiverMargin, null);
  assert.equal(withoutLock.giverMargin, withLock.giverMargin, "the giver is unaffected");
  assert.equal(withoutLock.margin, withLock.giverMargin, "only the pinned arm is judged");
});

test("a base at gimbal lock is recognised however the simulator spells its rotation", () => {
  const controller = () => new InteractionSpaceController({
    repoRoot: process.cwd(),
    manifest: JSON.parse(fs.readFileSync("generated/robots/Robot2/manifest.json", "utf8"))
  });
  const placement = JSON.parse(fs.readFileSync("generated/robots/Robot3/manifest.json", "utf8"))
    .sharedEnvironment;
  assert.deepEqual(placement.orientationDeg, [0, -90, 180]);
  assert.equal(controller().build("Robot3").success, true);

  // Robot3 is mounted at pitch -90, which is exactly where the alpha-beta-gamma
  // decomposition goes degenerate: these three triples are ONE rotation, and
  // which of them getObjectPose hands back is the solver's business, not a
  // property of the cell. Keying the cache on the triple made every live-pose
  // read a cache miss, so the pair recomputed on every handover forever.
  for (const orientationDeg of [[0, -90, 180], [180, -90, 0], [90, -90, -90]]) {
    const rebuilt = controller().build("Robot3", {
      placements: { Robot3: { offset: placement.offset, orientationDeg } }
    });
    assert.equal(rebuilt.success, true);
    assert.equal(rebuilt.cached, true, `spelling ${JSON.stringify(orientationDeg)} must hit the cache`);
  }

  // That a real move still invalidates is checked against a temporary cell
  // above -- deliberately not here, because a cache miss exports, and a test
  // must not overwrite the shipped cell's space with a layout it invented.
});

test("the ranking approaches along the mode the receiver's own manifest declares", () => {
  const controller = new InteractionSpaceController({
    repoRoot: process.cwd(),
    manifest: JSON.parse(fs.readFileSync("generated/robots/Robot1/manifest.json", "utf8"))
  });
  assert.equal(controller.build("Robot2").success, true);

  // The mode is the RECEIVER's, read from Robot2's manifest rather than from
  // the robot doing the ranking, and it is resolved exactly the way the runtime
  // resolves it -- otherwise the ranking scores a manoeuvre nobody performs.
  // Which mode the cell is tuned to is deliberately not asserted: that is a
  // setting, and pinning it makes retuning the cell break the test suite.
  const { handoverApproachMode } = require("./lowerThingServer");
  const receiver = JSON.parse(fs.readFileSync("generated/robots/Robot2/manifest.json", "utf8"));
  const mode = controller.handoverApproachModeFor("Robot2");
  assert.equal(mode, handoverApproachMode({ manifest: receiver }, "Robot1"));

  const ranked = controller.rank("Robot2", { minZ: 0.3, maxZ: 0.65 });
  assert.equal(ranked.success, true);
  assert.ok(ranked.candidates.length > 0);
  const { receiverApproachAxis: axisFor } = require("./handoverGeometry");
  for (const candidate of ranked.candidates) {
    // Every candidate is judged along the axis that mode actually produces,
    // recomputed here from the same inputs rather than restated.
    const expected = axisFor(candidate.giverApproachAxis, mode, candidate.receiverApproachAxis);
    assert.deepEqual(
      candidate.receiverApproachAxis.map((value) => Number(value.toFixed(6))),
      expected.map((value) => Number(value.toFixed(6)))
    );
    assert.ok(candidate.postureMargin >= 0.05);
  }

  // A named direction does not follow the giver, so it survives the giver
  // presenting the cube any which way -- which is the whole reason Robot3,
  // mounted pitched 90 degrees, can be told to come down instead of sideways.
  const { receiverApproachAxis } = require("./handoverGeometry");
  assert.deepEqual(receiverApproachAxis([1, 0, 0], "vertical", null), [0, 0, -1]);
});

test("candidate ranking rejects rendezvous points whose approach crosses a shoulder", () => {
  const {
    shoulderMargin, toolAxisWorld, cellBaselineAxis, receiverApproachAxis,
    worldVectorToLocal, orientationRadians
  } = require("./handoverGeometry");

  // A tool 0.19 m long, approaching along +X, from a grasp point 0.24 m out in
  // +X: the wrist ends up over the base however reachable every point is.
  const acrossTheAxis = shoulderMargin({
    graspLocal: { x: 0.2418, y: 0.0013, z: 0.4937 },
    axisLocal: [1, 0, 0], toolLength: 0.19, standoff: 0.15
  });
  assert.ok(acrossTheAxis < 0.01, `expected the wrist to cross the shoulder, got ${acrossTheAxis} m`);

  // The same grasp point approached from the side keeps the wrist well clear.
  const alongside = shoulderMargin({
    graspLocal: { x: 0.2418, y: 0.0013, z: 0.4937 },
    axisLocal: [0, 1, 0], toolLength: 0.19, standoff: 0.15
  });
  assert.ok(alongside > 0.2, `expected clearance, got ${alongside} m`);

  // The margin is taken over the whole leg, not just its ends: a line that only
  // grazes the axis in the middle must still be caught.
  const grazing = shoulderMargin({
    graspLocal: { x: 0.19, y: 0.0, z: 0.4 },
    axisLocal: [1, 0, 0], toolLength: 0.12, standoff: 0.2
  });
  assert.ok(grazing < 0.001, `a mid-leg crossing must be caught, got ${grazing} m`);

  // And against the shipped cell: the point the ranking used to pick is exactly
  // the one this rejects, while most of the shared space survives.
  const manifest = (name) =>
    JSON.parse(fs.readFileSync(`generated/robots/${name}/manifest.json`, "utf8"));
  const owner = manifest("Robot1");
  const partner = manifest("Robot2");
  const giverAxis = toolAxisWorld(owner.sharedEnvironment, owner.runtime.gripper.toolAxis);
  const preferred = cellBaselineAxis(partner.sharedEnvironment, owner.sharedEnvironment);
  const receiverAxis = receiverApproachAxis(giverAxis, "perpendicular", preferred);

  // The commanded point IS the grasp point, and the tool control point is the
  // wrist's own tip, so the approach leg starts there rather than one tool
  // length behind it.
  const marginFor = (cubeWorld, placement, man, axisWorld) => shoulderMargin({
    graspLocal: targetForRobot(cubeWorld, placement, man).localTcp,
    axisLocal: worldVectorToLocal(axisWorld, orientationRadians(placement)),
    toolLength: 0,
    standoff: man.runtime.gripper.handoverStandoffDistance ?? 0.15
  });

  // With the tool control point at the jaws the wrist no longer hangs a tool
  // length behind the grasp point, so this particular rendezvous is no longer
  // singular -- but the criterion must still separate good geometry from bad.
  const [receiverX, receiverY] = partner.sharedEnvironment.offset;
  const overShoulder = { x: receiverX, y: receiverY, z: 0.55 };
  assert.ok(
    marginFor(overShoulder, partner.sharedEnvironment, partner, receiverAxis) < 0.05,
    "a rendezvous directly over the receiver's base must not pass the posture filter"
  );

  const rows = fs.readFileSync("generated/robots/interactions/Robot1__Robot2/space.csv", "utf8")
    .trim().split("\n").slice(1).map((line) => line.split(",").map(Number));
  let inBand = 0;
  let survivors = 0;
  for (const [x, y, z] of rows) {
    if (z < 0.3 || z > 0.65) continue;
    inBand += 1;
    const cubeWorld = { x, y, z };
    const margin = Math.min(
      marginFor(cubeWorld, owner.sharedEnvironment, owner, giverAxis),
      marginFor(cubeWorld, partner.sharedEnvironment, partner, receiverAxis)
    );
    if (margin >= 0.05) survivors += 1;
  }
  // A filter that rejected everything would be worse than no filter at all.
  // How much it rejects here is a property of the layout the cell currently
  // stands in, not of the criterion -- move the arms apart and every remaining
  // rendezvous clears both shoulders -- so what is asserted against the shipped
  // space is only that the ranking is left real choice. The criterion's teeth
  // are shown above, against constructed geometry that does not drift when
  // somebody re-lays out the cell.
  assert.ok(inBand > 0, "the shipped space has candidates in the handover height band");
  assert.ok(
    survivors / inBand > 0.5,
    `the filter must leave the ranking real choice, kept ${survivors}/${inBand}`
  );
});

test("two robots exporting the same pair at once cannot interleave their files", () => {
  // Concurrent agents computing the same pair write the same two files. A plain
  // write lets the second land inside the first, and the reader that follows
  // sees a truncated cloud with no way to tell it is damaged.
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-atomic-"));
  const target = path.join(root, "space.csv");
  const long = `${"a".repeat(200000)}\n`;
  const short = "b\n";
  for (let round = 0; round < 20; round += 1) {
    writeFileAtomic(target, round % 2 === 0 ? long : short);
    const seen = fs.readFileSync(target, "utf8");
    assert.ok(seen === long || seen === short, "a reader never sees a partial file");
  }
  // Nothing is left behind for the next reader to trip over.
  assert.deepEqual(fs.readdirSync(root).filter((name) => name.includes(".tmp")), []);
  fs.rmSync(root, { recursive: true, force: true });
});

test("a cached space whose CSV disagrees with its own metadata is recomputed", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-torn-cache-"));
  const points = [];
  for (let x = 0; x < 6; x += 1) {
    for (let y = 0; y < 6; y += 1) {
      for (let z = 0; z < 6; z += 1) points.push({ x: x * 0.1, y: y * 0.1, z: z * 0.1 });
    }
  }
  const writeRobot = (name, shift) => {
    fs.writeFileSync(path.join(root, `${name}.csv`), ["X,Y,Z,Reachable",
      ...points.map((p) => `${p.x + shift},${p.y},${p.z},1`)].join("\n"));
    fs.writeFileSync(path.join(root, `${name}.manifest.json`), JSON.stringify({
      name,
      generatedFiles: { robotDataPoints: `${name}.csv` },
      runtime: { workspaceTolerance: 2, gripper: {} }
    }));
  };
  writeRobot("Robot1", 0);
  writeRobot("Robot2", 0.2);
  fs.writeFileSync(path.join(root, "cell.json"), JSON.stringify({
    interactionSpace: {
      pairs: [{
        key: "Robot1__Robot2", robots: ["Robot1", "Robot2"], ownerRobot: "Robot1",
        output: { csv: "space.csv", metadata: "space.json" }
      }],
      targetVoxelCount: 20, sampleSize: 50, spacingPercentile: 0.5, seed: 42
    },
    robots: [
      { name: "Robot1", manifest: "Robot1.manifest.json", workspaceCsv: "Robot1.csv", offset: [0, 0, 0], rotationDeg: 0 },
      { name: "Robot2", manifest: "Robot2.manifest.json", workspaceCsv: "Robot2.csv", offset: [0, 0, 0], rotationDeg: 0 }
    ]
  }));
  const manifest = (name) => ({ name, sharedEnvironment: { descriptor: "cell.json" } });
  const first = new InteractionSpaceController({ repoRoot: root, manifest: manifest("Robot1") }).build("Robot2");
  assert.equal(first.success, true);

  // Truncate the CSV the way a torn write would, leaving the metadata claiming
  // the full count.
  const csv = fs.readFileSync(path.join(root, "space.csv"), "utf8").split("\n");
  fs.writeFileSync(path.join(root, "space.csv"), csv.slice(0, Math.max(2, csv.length - 3)).join("\n"));
  const reread = new InteractionSpaceController({ repoRoot: root, manifest: manifest("Robot2") }).build("Robot1");
  assert.equal(reread.success, true);
  assert.equal(reread.cached, false, "a short CSV is a torn write, not a cache hit");
  assert.equal(reread.voxelCount, first.voxelCount);
  fs.rmSync(root, { recursive: true, force: true });
});

test("a workspace cloud is loaded and indexed once per process, however many agents ask", () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-shared-cloud-"));
  const csv = path.join(root, "cloud.csv");
  fs.writeFileSync(csv, ["X,Y,Z,Reachable",
    ...Array.from({ length: 200 }, (_, i) => `${i * 0.01},0,0,1`)].join("\n"));
  const points = loadReachableCsvShared(csv);
  assert.equal(loadReachableCsvShared(csv), points, "the same cloud object is handed to every caller");
  assert.equal(reachableTreeShared(csv), reachableTreeShared(csv), "and the same KD-tree");

  // A regenerated workspace must not be served from the cache.
  fs.writeFileSync(csv, ["X,Y,Z,Reachable", "9,9,9,1", "8,8,8,1"].join("\n"));
  const now = new Date();
  fs.utimesSync(csv, now, new Date(now.getTime() + 5000));
  assert.notEqual(loadReachableCsvShared(csv), points);
  assert.equal(loadReachableCsvShared(csv).length, 2);
  fs.rmSync(root, { recursive: true, force: true });
});

test("a pair has to share enough space to be a rendezvous at all", () => {
  // Overlap is not a yes/no fact. Two arms whose reach spheres barely touch
  // produce a handful of voxels at the far edge of both workspaces, and one
  // voxel prices exactly as confidently as three hundred -- so the route reads
  // the two numbers identically and commits to a pair with nothing for the
  // ranking to choose between and nothing for the retry to fall back on. Live
  // cell: Robot1 and Robot3 shared ONE voxel and the cube was routed through it.
  //
  // The thin pair is BUILT here rather than borrowed from the cell. It used to
  // be Robot1/Robot3, which stopped being thin and became empty the moment a
  // base moved: their clouds now come no closer than 0.029m, the estimator
  // answers voxel_intersection_empty, and this test failed for a reason that
  // had nothing to do with the floor it exists to guard. A property of the
  // ranking should not be pinned to whatever shape the cell happens to be in.
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-thin-pair-"));
  const writeCsv = (name, points) => {
    const relative = `${name}.csv`;
    fs.writeFileSync(path.join(root, relative), ["X,Y,Z,Reachable", ...points.map((p) => `${p.x},${p.y},${p.z},1`)].join("\n"));
    fs.writeFileSync(path.join(root, `${name}.manifest.json`), JSON.stringify({
      name,
      generatedFiles: { robotDataPoints: relative },
      runtime: { workspaceTolerance: 2, gripper: { jawCenterOffset: [0, 0, 0] } }
    }));
    return relative;
  };
  // Robot2 shares a three-slab-thick slice of Robot1's block; Robot3 clips one
  // corner of it. Both overlaps are real -- this is not the empty case.
  const wideCsv = writeCsv("Wide", lattice(0, 5, 0, 5, 0, 5));
  const healthyCsv = writeCsv("Healthy", lattice(3, 8, 0, 5, 0, 5));
  const thinCsv = writeCsv("Thin", lattice(4, 9, 4, 9, 0, 2));
  const pair = (partner) => ({
    key: `Wide__${partner}`,
    robots: ["Wide", partner],
    ownerRobot: "Wide",
    output: { csv: `cache/Wide__${partner}.csv`, metadata: `cache/Wide__${partner}.json` }
  });
  fs.writeFileSync(path.join(root, "cell.json"), JSON.stringify({
    interactionSpace: {
      pairs: [pair("Healthy"), pair("Thin")],
      // High enough that the estimator settles on the lattice's own spacing, so
      // a voxel is one lattice cell and the counts below are readable by hand.
      targetVoxelCount: 1000,
      proximityFactor: 1.25,
      sampleSize: 500,
      spacingPercentile: 0.5,
      seed: 42
    },
    robots: [
      { name: "Wide", manifest: "Wide.manifest.json", workspaceCsv: wideCsv, offset: [0, 0, 0], rotationDeg: 0 },
      { name: "Healthy", manifest: "Healthy.manifest.json", workspaceCsv: healthyCsv, offset: [0, 0, 0], rotationDeg: 0 },
      { name: "Thin", manifest: "Thin.manifest.json", workspaceCsv: thinCsv, offset: [0, 0, 0], rotationDeg: 0 }
    ]
  }));
  const synthetic = new InteractionSpaceController({
    repoRoot: root,
    manifest: { name: "Wide", sharedEnvironment: { descriptor: "cell.json" } }
  });
  const band = { minZ: 0, maxZ: 6, minimumCandidates: 1 };

  const thin = synthetic.build("Thin");
  assert.equal(thin.success, true, "the space still computes; it is the rendezvous that is refused");
  const refused = synthetic.rank("Thin", band);
  assert.equal(refused.success, false);
  assert.equal(refused.reason, "interaction_space_below_minimum");
  // The numbers travel with the refusal, or "too small" is unanswerable.
  assert.equal(refused.sharedVoxelCount, thin.voxelCount);
  assert.ok(refused.sharedVoxelCount < refused.minimumSharedVoxels);

  // And a real overlap is untouched by the floor.
  const healthy = synthetic.build("Healthy");
  assert.equal(healthy.success, true);
  const usable = synthetic.rank("Healthy", band);
  assert.equal(usable.success, true, `the healthy pair was refused: ${usable.reason}`);
  assert.ok(usable.candidates.length > 0);

  // The floor is configuration, not a constant welded into the ranking. Lifting
  // it shows exactly what it is protecting against: the thin pair ranks
  // perfectly happily, and hands back a couple of candidates with nothing
  // behind them. Nothing about that answer says it is a worse rendezvous than
  // the dozens the healthy pair offers, which is the whole problem.
  const unguarded = synthetic.rank("Thin", { ...band, minimumSharedVoxels: 0 });
  assert.equal(unguarded.success, true);
  assert.ok(
    unguarded.candidates.length < 3,
    `the pair this floor exists for should offer almost nothing, got ${unguarded.candidates.length}`
  );
  assert.ok(unguarded.candidates.length < usable.candidates.length / 10);
  fs.rmSync(root, { recursive: true, force: true });
});

test("the cell's own healthy pair clears the viability floor", () => {
  // The floor is a number in the descriptor, and a number chosen badly is the
  // kind of thing that only shows up against real clouds: it has to refuse a
  // sliver and pass a pair the cell actually hands cubes across.
  const robot1 = new InteractionSpaceController({
    repoRoot: process.cwd(),
    manifest: JSON.parse(fs.readFileSync("generated/robots/Robot1/manifest.json", "utf8"))
  });
  const built = robot1.build("Robot2");
  if (built && built.success === false) return; // pair artifacts absent in this checkout
  const usable = robot1.rank("Robot2", { minZ: 0.3, maxZ: 0.65 });
  assert.equal(usable.success, true, `Robot1-Robot2 was refused: ${usable.reason}`);
  assert.ok(usable.candidates.length > 0);
});

test("a refused rendezvous stops vouching for the density of its neighbours", () => {
  const controller = new InteractionSpaceController({
    repoRoot: process.cwd(),
    manifest: JSON.parse(fs.readFileSync("generated/robots/Robot3/manifest.json", "utf8"))
  });
  const built = controller.build("Robot4");
  if (built && built.success === false) return; // pair artifacts absent in this checkout

  const band = { minZ: 0.15, maxZ: 0.45 };
  const weights = { centrality: 0.32, density: 0.2, giverSupport: 0.14, receiverSupport: 0.14, posture: 0.2 };
  const score = (c) => weights.centrality * c.centrality + weights.density * c.density
    + weights.giverSupport * c.giverSupport + weights.receiverSupport * c.receiverSupport
    + weights.posture * c.posture;
  const bestOf = (result, struck) => result.candidates
    .filter((c) => !struck.has(c.id))
    .map((c) => ({ c, s: score(c) }))
    .sort((left, right) => right.s - left.s)[0].c;

  const before = controller.rank("Robot4", band);
  const byId = new Map(before.candidates.map((c) => [c.id, c]));

  // Robot3 and Robot4 share a thin space, so the shape of this is stable: the
  // two best candidates stand in one column and the cell refuses both.
  const struck = new Set();
  const first = bestOf(before, struck);
  struck.add(first.id);
  controller.rejectCandidate("Robot4", first.id);
  const second = bestOf(controller.rank("Robot4", band), struck);
  struck.add(second.id);
  controller.rejectCandidate("Robot4", second.id);
  const third = bestOf(controller.rank("Robot4", band), struck);

  // The third pick must not have been carried by neighbours that are gone. Its
  // density is now measured over what is still usable, so a point standing
  // beside the two refusals no longer outranks a more central one on that
  // strength alone.
  const centrality = (id) => byId.get(id).centrality;
  assert.ok(
    centrality(third.id) > 0.5,
    `third pick ${third.id} has centrality ${centrality(third.id)}, which is the stale-density regression`
  );

  // Striking changes density and nothing else: the ids name the same voxels and
  // centrality still describes the region the refusals are still part of.
  const after = controller.rank("Robot4", band);
  assert.deepEqual(after.candidates.map((c) => c.id), before.candidates.map((c) => c.id),
    "a strike must not renumber the candidates it did not touch");
  for (const candidate of after.candidates) {
    assert.equal(candidate.centrality, byId.get(candidate.id).centrality,
      "centrality measures the region, which a refusal does not change");
  }
  assert.ok(after.candidates.some((c) => c.density !== byId.get(c.id).density),
    "density must react to the refusals");

  // And clearing them puts it back exactly as it was.
  controller.clearRejectedCandidates("Robot4");
  const restored = controller.rank("Robot4", band);
  for (const candidate of restored.candidates) {
    assert.equal(candidate.density, byId.get(candidate.id).density);
  }
});
