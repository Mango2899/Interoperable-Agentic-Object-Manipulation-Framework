#!/usr/bin/env node
// E9 -- selection of the handover point.
//
// The question of the thesis is whether the weighted score of Eq.
// (handover_score),
//
//     score = 0.32 C + 0.2 D + 0.14 S_g + 0.14 S_r + 0.2 P,
//
// puts the rendezvous on a point BOTH arms can actually serve, compared with
// the simpler rules one would otherwise reach for. Five strategies choose from
// the same pool, for every usable pair of the five-robot cell and both
// directions of the handover:
//
//   weighted     all five terms of Eq. (handover_score)
//   centrality   C alone -- nearest to the centroid of the shared region
//   density      D alone -- the thickest part of the shared region
//   random       a uniformly drawn voxel, averaged over --draws repetitions
//   bbox         nearest to the centre of the interaction space's bounding box
//
// THE POOL, and why it is not the deployed one. rank() in interactionSpace.js
// screens the height band before it scores anything: a voxel whose TCP target
// falls outside either arm's sampled workspace, or farther than that arm's
// admission tolerance from the nearest sample, or whose straight approach leg
// would pass over the arm's own base-joint axis, never reaches the agent as a
// candidate. Run on that pool, every strategy scores 100 per cent and the
// experiment measures nothing but the screen. So all five strategies here rank
// the UNSCREENED height band -- every voxel of the interaction space between
// z_min and z_max -- and feasibility is the outcome, not a precondition. What
// is compared is therefore the CRITERION: left to itself, does it land on a
// servable point?
//
// The deployed pipeline is reported as a sixth row, `deployed`, which is the
// real rank() with its central-core trim, its screen and its cap of 60
// candidates, scored by Eq. (handover_score). Its feasibility is 1.0 by
// construction and it is there to be read that way: the row separates what the
// weighting contributes from what the screen contributes.
//
// FEASIBILITY is the screen rank() applies, evaluated per voxel rather than
// used to filter: both TCP targets inside the respective workspace bounds, both
// within the admission tolerance tau of a reachable sample, and a posture
// margin at or above m_min. It is a planning-time predicate -- the same one the
// protocol applies before it plans the approach legs -- and it is computed by
// the deployed code (posturePenalty, the KD-trees of the two clouds), not
// reimplemented here.
//
// REFUSALS follow the protocol: a strategy that proposes an infeasible point
// has it struck and offers its next candidate, and the number of strikes before
// the first feasible point is the second metric. Density is recomputed after
// every strike, since a refused voxel is not usable room and must stop counting
// towards its neighbours' scores; centrality is not, since it describes the
// geometry of the region and a refused point is still part of it. That is the
// rule of Section (voxel_centrality) and this walk obeys it, which is why
// `weighted` and `density` are re-ranked between strikes and the other three
// are not.
//
// BOUNDARY DISTANCE of the selected point is the distance from its voxel centre
// to the nearest centre of a cell the interaction space does not occupy, on the
// pair's own grid. A point one voxel in from the edge scores s; the deepest
// point of a thin spur cannot score more than s whatever its centrality says.
//
// EDGE COST is Section (edge_costs) evaluated offline: the distance from the
// giver's TCP origin to the giver's TCP target at the selected point, in the
// giver's local frame, exactly as estimateEdgeCost measures it. The origin is
// the cell's loaded pickup (mode `pickup`), the one origin that is the same
// physical place for every pair and direction, so the comparison between
// strategies within a pair is the quantity of interest and the constant offset
// between pairs is not read as a result.
//
// Offline, like E5, E6 and E7: the clouds are the deployed ones the live cell
// serves (generated/robots/RobotN/info), placed by the five-robot descriptor.
// Nothing here talks to CoppeliaSim, and nothing is written into
// generated/ -- the exporter of the controller is disabled for the run.
//
// The last metric of the thesis, the success rate of the executed handovers,
// needs the live cell and is the `run` command below, which drives the two
// lower Things through a complete handover of each strategy's choice. `analyze`
// folds its result in whenever e9_handovers.jsonl is present, so the offline
// half stays runnable on its own.
//
// Output in performance_log/e9: e9_runs.csv (one row per pair, direction,
// strategy and repetition), e9_strategies.csv (the summary per strategy),
// e9_pairs.csv (per pair and direction), e9_refused.csv (the histogram),
// e9_slice.csv and e9_selected.csv (the scores of one interaction space and the
// point each strategy picked in it), e9_table.tex, e9_figure.tex, e9_setup.json.

const fs = require("node:fs");
const path = require("node:path");

const {
  InteractionSpaceController,
  canonicalPairRobots,
  pointToVoxel,
  seededRandom,
  targetForRobot
} = require("../generator/interactionSpace");
const { summarize, toCsv } = require("./e1Convergence");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const DEFAULT_OUT_DIR = "performance_log/e9";
const DEFAULT_CELL = 5;
const DEFAULT_MIN_Z = 0.15;
const DEFAULT_MAX_Z = 0.45;
const DEFAULT_DRAWS = 200;
const DEFAULT_SEED = 42;
const DEFAULT_SLICE_PAIR = "Robot2__Robot4";
// The agent's own handoverRetriesLeft(2): two replacements after the first
// choice, so three candidates per strategy at most.
const DEFAULT_RETRIES = 2;
// Rounds of the whole live half. One round is one handover per direction per
// strategy and carries no error bar at all; several give the executed success
// rates an interval.
const DEFAULT_REPEATS = 1;
const DEFAULT_SETTLE_MS = 500;
// Generous: a receiver's approach includes its preflight, its planner and the
// arbiter, and the agent's own wait for it is 120 s.
const DEFAULT_TIMEOUT_MS = 180000;

const WEIGHTS = { centrality: 0.32, density: 0.2, giverSupport: 0.14, receiverSupport: 0.14, posture: 0.2 };
const DENSITY_RADIUS_FACTOR = 2.5;
const AXES = ["x", "y", "z"];
// The bounding-box centre is not one of the pool's voxels and carries this index.
const CENTRE_INDEX = -1;

const STRATEGIES = ["weighted", "centrality", "density", "random", "bbox"];
const STRATEGY_LABELS = {
  weighted: "weighted score",
  centrality: "centrality only",
  density: "density only",
  random: "random voxel",
  bbox: "bounding-box centre",
  deployed: "deployed (screened)"
};
// Whether a strike changes the strategy's remaining order. Density does, because
// a refused voxel stops counting as usable room; the purely geometric criteria
// do not.
const RERANKS_ON_REFUSAL = { weighted: true, density: true, centrality: false, random: false, bbox: false };
// The live half also executes the deployed pipeline itself, which the offline
// half can only report as feasible-by-construction. It is the row the thesis
// most needs a number for: what the cell actually does.
const LIVE_STRATEGIES = [...STRATEGIES, "deployed"];

function weightedScore(scores) {
  return WEIGHTS.centrality * scores.centrality
    + WEIGHTS.density * scores.density
    + WEIGHTS.giverSupport * scores.giverSupport
    + WEIGHTS.receiverSupport * scores.receiverSupport
    + WEIGHTS.posture * scores.posture;
}

function descriptorForCell(size) {
  const names = Array.from({ length: size }, (_, index) => `Robot${index + 1}`);
  return `generated/robots/${names.join("_")}.shared.json`;
}

function manifestPath(robot) {
  return path.join(REPO_ROOT, "generated", "robots", robot, "manifest.json");
}

// CoppeliaSim's remote API hands single values back inside an array.
function first(value) {
  return Array.isArray(value) ? value[0] : value;
}

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, "utf8"));
}

// A controller that computes and never writes. The deployed interaction spaces
// under generated/robots/interactions are the live cell's, and an experiment
// has no business rewriting them.
function readOnlyController(robot) {
  const manifest = readJson(manifestPath(robot));
  const controller = new InteractionSpaceController({ repoRoot: REPO_ROOT, manifest });
  controller.export = () => {};
  return controller;
}

// The reverse direction of a pair is the same geometry with the roles swapped:
// the same voxels, the same grid, the other arm giving. Building it a second
// time from the other manifest would recompute two KD-trees over 100k points to
// arrive at exactly this.
function reverseView(space) {
  return {
    ...space,
    targetRobot: space.owner.name,
    owner: space.partner,
    partner: space.owner,
    ownerManifest: space.partnerManifest,
    partnerManifest: space.ownerManifest,
    ownerTree: space.partnerTree,
    partnerTree: space.ownerTree,
    ownerBounds: space.partnerBounds,
    partnerBounds: space.ownerBounds
  };
}

function withinBounds(point, box) {
  return AXES.every((axis) => point[axis] >= box.min[axis] && point[axis] <= box.max[axis]);
}

function toleranceOf(manifest) {
  return Number(manifest.runtime?.workspaceTolerance || 0.08);
}

// Every score term of Eq. (handover_score) for every voxel of the height band,
// feasible or not, plus the predicate itself. Density is left out here because
// it depends on which voxels are still standing; it is added per re-rank.
function poolOf(controller, space, giver, receiver, { minZ, maxZ }) {
  const calibration = controller.descriptor?.handoverCalibration;
  const offset = calibration?.type === "shared_point_world_offset"
    && calibration.sourceRobot === giver && calibration.targetRobot === receiver
    ? calibration.sharedPointWorldOffset.map(Number)
    : [0, 0, 0];
  const band = space.voxels.filter(({ center }) => center.z >= minZ && center.z <= maxZ);
  if (band.length === 0) return null;
  const centroid = Object.fromEntries(AXES.map((axis) => [
    axis, band.reduce((sum, { center }) => sum + center[axis], 0) / band.length
  ]));
  const distanceTo = (center) => Math.hypot(
    center.x - centroid.x, center.y - centroid.y, center.z - centroid.z
  );
  const maxDistance = Math.max(...band.map(({ center }) => distanceTo(center)), 1e-12);
  const giverTolerance = toleranceOf(space.ownerManifest);
  const receiverTolerance = toleranceOf(space.partnerManifest);

  const evaluate = (center, index) => {
    const cubeWorld = {
      x: center.x + offset[0],
      y: center.y + offset[1],
      z: center.z + offset[2]
    };
    const giverTarget = targetForRobot(cubeWorld, space.owner, space.ownerManifest);
    const receiverTarget = targetForRobot(cubeWorld, space.partner, space.partnerManifest);
    const giverDistance = space.ownerTree.nearestDistance(giverTarget.localTcp);
    const receiverDistance = space.partnerTree.nearestDistance(receiverTarget.localTcp);
    const posture = controller.posturePenalty(cubeWorld, space, receiver);
    const reachable = withinBounds(giverTarget.localTcp, space.ownerBounds)
      && withinBounds(receiverTarget.localTcp, space.partnerBounds)
      && giverDistance <= giverTolerance
      && receiverDistance <= receiverTolerance;
    return {
      index,
      voxel: { center, index: pointToVoxel(center, space.origin, space.voxelSize) },
      cubeWorld,
      giverTarget,
      receiverTarget,
      centrality: Math.max(0, 1 - distanceTo(center) / maxDistance),
      giverSupport: Math.max(0, 1 - giverDistance / giverTolerance),
      receiverSupport: Math.max(0, 1 - receiverDistance / receiverTolerance),
      posture: posture.score,
      postureMargin: posture.margin,
      reachable,
      postureOk: posture.margin >= posture.minimumMargin,
      feasible: reachable && posture.margin >= posture.minimumMargin
    };
  };

  const entries = band.map((voxel, index) => evaluate(voxel.center, index));

  // Neighbourhoods for the density term, computed once. Density after a strike
  // is then a count over the survivors rather than another O(n^2) sweep.
  const radius = space.voxelSize * DENSITY_RADIUS_FACTOR;
  const radiusSquared = radius * radius;
  const neighbours = entries.map(() => []);
  for (let i = 0; i < entries.length; i += 1) {
    neighbours[i].push(i);
    for (let j = i + 1; j < entries.length; j += 1) {
      const a = entries[i].voxel.center;
      const b = entries[j].voxel.center;
      const dx = a.x - b.x;
      const dy = a.y - b.y;
      const dz = a.z - b.z;
      if (dx * dx + dy * dy + dz * dz <= radiusSquared) {
        neighbours[i].push(j);
        neighbours[j].push(i);
      }
    }
  }
  return { entries, neighbours, centroid, offset, band, evaluate };
}

function densities(pool, refused) {
  const raw = pool.neighbours.map(
    (list) => list.reduce((count, index) => count + (refused.has(index) ? 0 : 1), 0)
  );
  const maximum = Math.max(...raw, 1);
  return raw.map((value) => value / maximum);
}

// The OUTSIDE of an interaction space: the cells of its bounding box, padded by
// one, that a 6-connected flood fill reaches from the shell without passing
// through an occupied cell.
//
// The distinction matters because these voxel sets are POROUS. They are the
// intersection of two independently sampled clouds, so an occupied cell whose
// face neighbour happens to hold no sample of one of the two robots is the
// normal case, not an edge. Measured against "the nearest cell that is not
// occupied", every candidate of every pair of the cell scored exactly one voxel
// and the metric said nothing. Measured against the cells that are genuinely
// outside the region, it says how deep in the shared space a point sits.
function outsideCells(space) {
  const occupied = new Set(space.voxels.map(({ index }) => index.join(",")));
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (const { index } of space.voxels) {
    for (let axis = 0; axis < 3; axis += 1) {
      if (index[axis] < min[axis]) min[axis] = index[axis];
      if (index[axis] > max[axis]) max[axis] = index[axis];
    }
  }
  const low = min.map((value) => value - 1);
  const high = max.map((value) => value + 1);
  const inside = (cell) => cell.every((value, axis) => value >= low[axis] && value <= high[axis]);
  const outside = new Set();
  const queue = [];
  for (let i = low[0]; i <= high[0]; i += 1) {
    for (let j = low[1]; j <= high[1]; j += 1) {
      for (let k = low[2]; k <= high[2]; k += 1) {
        const onShell = i === low[0] || i === high[0] || j === low[1] || j === high[1]
          || k === low[2] || k === high[2];
        if (!onShell) continue;
        const key = `${i},${j},${k}`;
        if (occupied.has(key) || outside.has(key)) continue;
        outside.add(key);
        queue.push([i, j, k]);
      }
    }
  }
  const steps = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
  while (queue.length > 0) {
    const cell = queue.pop();
    for (const step of steps) {
      const next = [cell[0] + step[0], cell[1] + step[1], cell[2] + step[2]];
      if (!inside(next)) continue;
      const key = next.join(",");
      if (occupied.has(key) || outside.has(key)) continue;
      outside.add(key);
      queue.push(next);
    }
  }
  return { occupied, outside, low, high };
}

// Distance from a voxel centre to the nearest centre of a cell outside the
// interaction space, on the pair's own grid. Expanding shells: the first
// Chebyshev radius that holds an outside cell bounds the answer, and everything
// within sqrt(3) of that radius is then checked so the Euclidean minimum is not
// missed on a diagonal.
function boundaryDistance(space, grid, voxel) {
  const [i, j, k] = voxel.index;
  const size = space.voxelSize;
  const limit = Math.max(
    grid.high[0] - grid.low[0], grid.high[1] - grid.low[1], grid.high[2] - grid.low[2]
  ) + 1;
  for (let radius = 1; radius <= limit; radius += 1) {
    let found = false;
    for (let di = -radius; di <= radius && !found; di += 1) {
      for (let dj = -radius; dj <= radius && !found; dj += 1) {
        for (let dk = -radius; dk <= radius; dk += 1) {
          if (Math.max(Math.abs(di), Math.abs(dj), Math.abs(dk)) !== radius) continue;
          if (grid.outside.has(`${i + di},${j + dj},${k + dk}`)) { found = true; break; }
        }
      }
    }
    if (!found) continue;
    const outer = Math.ceil(radius * Math.sqrt(3));
    let best = Infinity;
    for (let di = -outer; di <= outer; di += 1) {
      for (let dj = -outer; dj <= outer; dj += 1) {
        for (let dk = -outer; dk <= outer; dk += 1) {
          if (di === 0 && dj === 0 && dk === 0) continue;
          if (!grid.outside.has(`${i + di},${j + dj},${k + dk}`)) continue;
          const distance = Math.hypot(di, dj, dk) * size;
          if (distance < best) best = distance;
        }
      }
    }
    return best;
  }
  return null;
}

function edgeCost(entry, origin) {
  if (!origin) return null;
  const tcp = entry.giverTarget.localTcp;
  return Math.hypot(tcp.x - origin.x, tcp.y - origin.y, tcp.z - origin.z);
}

// The origin of Section (edge_costs), mode `pickup`: the loaded task site of the
// cell, in the giver's local frame.
function pickupOrigin(descriptor, placement, manifest) {
  const site = (descriptor.taskSites || []).find((entry) => entry.payload?.position);
  if (!site) return null;
  const world = {
    x: Number(site.payload.position[0]),
    y: Number(site.payload.position[1]),
    z: Number(site.payload.position[2])
  };
  return { world, site: site.alias, local: targetForRobot(world, placement, manifest).localTcp };
}

// One strategy's ordering, as a function of which voxels are still standing.
// `weighted` and `density` are asked again after every strike; the others
// return the same order and are asked once.
function ordering(strategy, pool, refused, context) {
  const { entries } = pool;
  const alive = entries.map((entry) => entry.index).filter((index) => !refused.has(index));
  if (strategy === "random") {
    return context.shuffled.filter((index) => !refused.has(index));
  }
  if (strategy === "bbox") {
    // The centre of the bounding box is a POINT, not a voxel, and the whole
    // point of the baseline is that it need not lie in the interaction space at
    // all: a shared region shaped like an L or a shell has its box centre in
    // thin air. It is offered first and, if the cell refuses it, the strategy
    // falls back on the voxels nearest to it, which is the protocol's "next
    // candidate of the same strategy".
    const voxels = alive.slice().sort((left, right) =>
      context.bboxDistance[left] - context.bboxDistance[right] || left - right);
    return refused.has(CENTRE_INDEX) ? voxels : [CENTRE_INDEX, ...voxels];
  }
  if (strategy === "centrality") {
    return alive.slice().sort((left, right) =>
      entries[right].centrality - entries[left].centrality || left - right);
  }
  const density = densities(pool, refused);
  if (strategy === "density") {
    return alive.slice().sort((left, right) =>
      density[right] - density[left] || entries[right].centrality - entries[left].centrality || left - right);
  }
  const score = entries.map((entry, index) => weightedScore({ ...entry, density: density[index] }));
  return alive.slice().sort((left, right) => score[right] - score[left] || left - right);
}

// Walk the strategy's order, striking what the cell would refuse, until a
// feasible point stands. Returns the point, how many strikes it took, and the
// scores it was chosen on.
function walk(strategy, pool, context) {
  const entryOf = (index) => (index === CENTRE_INDEX ? context.centreEntry : pool.entries[index]);
  const refused = new Set();
  let order = ordering(strategy, pool, refused, context);
  const firstChoice = order.length > 0 ? order[0] : null;
  let strikes = 0;
  while (order.length > 0) {
    const index = order[0];
    const entry = entryOf(index);
    if (entry.feasible) {
      const density = densities(pool, refused);
      const own = index === CENTRE_INDEX ? context.centreDensity : density[index];
      return {
        selected: entry,
        firstChoice: firstChoice === null ? null : entryOf(firstChoice),
        refused: strikes,
        density: own,
        score: weightedScore({ ...entry, density: own })
      };
    }
    refused.add(index);
    strikes += 1;
    order = RERANKS_ON_REFUSAL[strategy]
      ? ordering(strategy, pool, refused, context)
      : order.slice(1);
  }
  return {
    selected: null,
    firstChoice: firstChoice === null ? null : entryOf(firstChoice),
    refused: strikes,
    density: null,
    score: null
  };
}

// The box is taken over the HEIGHT BAND, not over the whole interaction space.
// The band is a hard constraint of the protocol -- the agent holds it as a
// belief and the cell will not present the cube outside it -- and every other
// strategy chooses inside it. Over the whole space the centre of the box for
// Robot2/Robot4 lands at z = 0.104 m, below the band, and the baseline would
// then be failing on a constraint rather than on its own idea of a centre. The
// unrestricted centre is recorded in the setup for the record.
function bboxCentre(voxels) {
  const box = { min: {}, max: {} };
  for (const axis of AXES) {
    const values = voxels.map(({ center }) => center[axis]);
    box.min[axis] = Math.min(...values);
    box.max[axis] = Math.max(...values);
  }
  return Object.fromEntries(AXES.map((axis) => [axis, (box.min[axis] + box.max[axis]) / 2]));
}

function directionRows({
  controller, space, giver, receiver, pairKey, minZ, maxZ, draws, seed, descriptor
}) {
  const pool = poolOf(controller, space, giver, receiver, { minZ, maxZ });
  if (!pool) {
    return { rows: [], skipped: { pairKey, giver, receiver, reason: "no_height_filtered_interaction_voxel" } };
  }
  const grid = outsideCells(space);
  const centre = bboxCentre(pool.band);
  const unrestrictedCentre = bboxCentre(space.voxels);
  const bboxDistance = pool.entries.map(({ voxel }) => Math.hypot(
    voxel.center.x - centre.x, voxel.center.y - centre.y, voxel.center.z - centre.z
  ));
  const origin = pickupOrigin(descriptor, space.owner, space.ownerManifest);
  const feasibleCount = pool.entries.filter((entry) => entry.feasible).length;

  // The bounding-box centre, scored like any other point so that its refusal is
  // a measurement and not an assumption.
  const centreEntry = pool.evaluate(centre, CENTRE_INDEX);
  const radius = space.voxelSize * DENSITY_RADIUS_FACTOR;
  const rawCentreDensity = pool.entries.reduce((count, entry) => count + (
    Math.hypot(
      entry.voxel.center.x - centre.x,
      entry.voxel.center.y - centre.y,
      entry.voxel.center.z - centre.z
    ) <= radius ? 1 : 0
  ), 0);
  const maxRawDensity = Math.max(...pool.neighbours.map((list) => list.length), 1);
  const context = {
    shuffled: [],
    bboxDistance,
    centreEntry,
    centreDensity: rawCentreDensity / maxRawDensity
  };
  const centreInside = grid.occupied.has(centreEntry.voxel.index.join(","));

  const rows = [];
  const record = (strategy, repeat, result) => {
    const entry = result.selected;
    rows.push({
      pair: pairKey,
      giver,
      receiver,
      strategy,
      repeat,
      poolSize: pool.entries.length,
      feasiblePool: feasibleCount,
      feasibleFraction: feasibleCount / pool.entries.length,
      firstChoiceFeasible: result.firstChoice ? (result.firstChoice.feasible ? 1 : 0) : null,
      found: entry ? 1 : 0,
      refused: result.refused,
      boundaryDistance: entry
        ? (grid.occupied.has(entry.voxel.index.join(",")) ? boundaryDistance(space, grid, entry.voxel) : 0)
        : null,
      edgeCost: entry ? edgeCost(entry, origin?.local) : null,
      centrality: entry ? entry.centrality : null,
      density: entry ? result.density : null,
      giverSupport: entry ? entry.giverSupport : null,
      receiverSupport: entry ? entry.receiverSupport : null,
      posture: entry ? entry.posture : null,
      score: entry ? result.score : null,
      x: entry ? entry.voxel.center.x : null,
      y: entry ? entry.voxel.center.y : null,
      z: entry ? entry.voxel.center.z : null,
      voxelSize: space.voxelSize,
      sharedVoxels: space.voxels.length,
      boxCentreInSpace: centreInside ? 1 : 0,
      boxCentreFeasible: centreEntry.feasible ? 1 : 0
    });
  };

  for (const strategy of STRATEGIES) {
    if (strategy === "random") {
      const random = seededRandom(seed);
      for (let repeat = 0; repeat < draws; repeat += 1) {
        const shuffled = pool.entries.map((entry) => entry.index);
        for (let i = shuffled.length - 1; i > 0; i -= 1) {
          const j = Math.floor(random() * (i + 1));
          [shuffled[i], shuffled[j]] = [shuffled[j], shuffled[i]];
        }
        record(strategy, repeat, walk(strategy, pool, { ...context, shuffled }));
      }
      continue;
    }
    record(strategy, 0, walk(strategy, pool, context));
  }

  // The deployed pipeline, for the row that separates the weighting from the
  // screen: the real rank(), its central core, its cap, scored by Eq.
  // (handover_score) as the agent scores it.
  const ranked = controller.rank(receiver, { minZ, maxZ });
  if (ranked.success && ranked.candidates.length > 0) {
    const best = ranked.candidates.slice().sort((left, right) => weightedScore(right) - weightedScore(left))[0];
    const key = `${best.sharedPointWorld.x},${best.sharedPointWorld.y},${best.sharedPointWorld.z}`;
    const entry = pool.entries.find(({ voxel }) =>
      `${voxel.center.x},${voxel.center.y},${voxel.center.z}` === key) || null;
    rows.push({
      pair: pairKey,
      giver,
      receiver,
      strategy: "deployed",
      repeat: 0,
      poolSize: ranked.coreCandidateCount,
      feasiblePool: ranked.candidateCount,
      feasibleFraction: 1,
      firstChoiceFeasible: 1,
      found: 1,
      refused: 0,
      boundaryDistance: entry ? boundaryDistance(space, grid, entry.voxel) : null,
      edgeCost: origin ? Math.hypot(
        best.giver.localTcp.x - origin.local.x,
        best.giver.localTcp.y - origin.local.y,
        best.giver.localTcp.z - origin.local.z
      ) : null,
      centrality: best.centrality,
      density: best.density,
      giverSupport: best.giverSupport,
      receiverSupport: best.receiverSupport,
      posture: best.posture,
      score: weightedScore(best),
      x: best.sharedPointWorld.x,
      y: best.sharedPointWorld.y,
      z: best.sharedPointWorld.z,
      voxelSize: space.voxelSize,
      sharedVoxels: space.voxels.length,
      boxCentreInSpace: centreInside ? 1 : 0,
      boxCentreFeasible: centreEntry.feasible ? 1 : 0
    });
  }
  return { rows, pool, space, grid, bboxDistance, origin, centreEntry, centreInside, unrestrictedCentre };
}

// The three files the thesis plots from. They are deliberately flat and
// pre-joined: a pgfplots axis reads one column pair per \addplot and cannot
// filter rows, so the completed and the refused attempts are written apart
// rather than sorted out in LaTeX.
function thesisFigureData(rows, executed, outDir) {
  const offline = new Map(rows.length > 0 ? strategySummary(rows).map((row) => [row.strategy, row]) : []);
  const live = new Map(executionSummary(executed).map((row) => [row.strategy, row]));
  const order = [...LIVE_STRATEGIES];
  const strategies = order.map((strategy, index) => {
    const own = executed.filter((row) => row.strategy === strategy && !isSetupFailure(row));
    const stageCount = (stage) => own.filter((row) => !row.completed && row.stage === stage).length;
    return {
      index,
      strategy,
      label: STRATEGY_LABELS[strategy] || strategy,
      offlineFeasible: offline.get(strategy)?.feasibilityRate ?? null,
      inSharedSet: offline.get(strategy)?.inSpaceRate ?? null,
      edgeCost: offline.get(strategy)?.edgeCost_median ?? null,
      liveFirst: live.get(strategy)?.firstAttemptSuccess ?? null,
      // pgfplots wants the whisker LENGTHS, not the endpoints.
      liveFirstMinus: live.get(strategy)?.firstAttemptSuccess == null ? null
        : live.get(strategy).firstAttemptSuccess - live.get(strategy).firstAttemptLow,
      liveFirstPlus: live.get(strategy)?.firstAttemptSuccess == null ? null
        : live.get(strategy).firstAttemptHigh - live.get(strategy).firstAttemptSuccess,
      liveRetried: live.get(strategy)?.successRate ?? null,
      liveRetriedMinus: live.get(strategy)?.successRate == null ? null
        : live.get(strategy).successRate - live.get(strategy).successLow,
      liveRetriedPlus: live.get(strategy)?.successRate == null ? null
        : live.get(strategy).successHigh - live.get(strategy).successRate,
      rounds: live.get(strategy)?.rounds ?? 0,
      trials: live.get(strategy)?.firstAttemptTrials ?? 0,
      attempts: live.get(strategy)?.attempts ?? 0,
      refusedPerDirection: live.get(strategy)?.liveRefused_mean ?? null,
      completed: own.filter((row) => row.completed).length,
      giverApproach: stageCount("giver_approach"),
      receiverApproach: stageCount("receiver_approach"),
      transferRefused: stageCount("transfer")
    };
  });
  fs.writeFileSync(path.join(outDir, "e9_figure_strategies.csv"), toCsv(strategies, [
    "index", "strategy", "label", "offlineFeasible", "inSharedSet", "edgeCost",
    "liveFirst", "liveFirstMinus", "liveFirstPlus",
    "liveRetried", "liveRetriedMinus", "liveRetriedPlus",
    "rounds", "trials", "attempts", "refusedPerDirection",
    "completed", "giverApproach", "receiverApproach", "transferRefused"
  ]));

  // One row per executed attempt, for the panel that asks what actually
  // separates a rendezvous the cell can serve from one it refuses. The x
  // coordinate is the strategy index, spread a little so that attempts of one
  // strategy do not land on top of each other.
  const indexOf = new Map(order.map((strategy, index) => [strategy, index]));
  const point = (row, position, total) => ({
    x: indexOf.get(row.strategy) + (total > 1 ? (position / (total - 1) - 0.5) * 0.5 : 0),
    strategy: row.strategy,
    centrality: row.centrality,
    density: row.density,
    posture: row.posture,
    postureMargin: row.postureMargin,
    score: row.score,
    stage: row.stage
  });
  const spread = (subset) => subset.map((row, position) => point(row, position, subset.length));
  const tried = executed.filter((row) => !isSetupFailure(row));
  const columns = ["x", "strategy", "centrality", "density", "posture", "postureMargin", "score", "stage"];
  // Not e9_refused.csv: that name already belongs to the histogram of refused
  // candidates, and writing both would leave whichever ran last on disk.
  for (const [file, subset] of [
    ["e9_points_completed.csv", tried.filter((row) => row.completed)],
    ["e9_points_refused.csv", tried.filter((row) => !row.completed)]
  ]) {
    const grouped = order.flatMap((strategy) => spread(subset.filter((row) => row.strategy === strategy)));
    fs.writeFileSync(path.join(outDir, file), toCsv(grouped, columns));
  }
  return strategies;
}

function strategySummary(rows) {
  const order = [...STRATEGIES, "deployed"];
  return order.map((strategy) => {
    const own = rows.filter((row) => row.strategy === strategy);
    if (own.length === 0) return null;
    const refused = summarize(own.map((row) => (row.found ? row.refused : null)));
    const boundary = summarize(own.map((row) => row.boundaryDistance));
    const cost = summarize(own.map((row) => row.edgeCost));
    const firstFeasible = own.filter((row) => row.firstChoiceFeasible !== null);
    return {
      strategy,
      label: STRATEGY_LABELS[strategy] || strategy,
      evaluations: own.length,
      feasibilityRate: firstFeasible.length === 0
        ? null
        : firstFeasible.reduce((sum, row) => sum + row.firstChoiceFeasible, 0) / firstFeasible.length,
      foundRate: own.reduce((sum, row) => sum + row.found, 0) / own.length,
      refused_mean: refused.mean,
      refused_median: refused.median,
      refused_q75: refused.q75,
      refused_max: refused.max,
      inSpaceRate: own.filter((row) => row.found).length === 0
        ? null
        : own.filter((row) => row.found && row.boundaryDistance > 0).length
          / own.filter((row) => row.found).length,
      boundary_median: boundary.median,
      boundary_q25: boundary.q25,
      boundary_q75: boundary.q75,
      centrality_median: summarize(own.map((row) => row.centrality)).median,
      density_median: summarize(own.map((row) => row.density)).median,
      giverSupport_median: summarize(own.map((row) => row.giverSupport)).median,
      receiverSupport_median: summarize(own.map((row) => row.receiverSupport)).median,
      posture_median: summarize(own.map((row) => row.posture)).median,
      edgeCost_median: cost.median,
      edgeCost_q25: cost.q25,
      edgeCost_q75: cost.q75,
      score_median: summarize(own.map((row) => row.score)).median
    };
  }).filter(Boolean);
}

function pairSummary(rows) {
  const keys = [...new Set(rows.map((row) => `${row.pair}|${row.giver}|${row.receiver}|${row.strategy}`))];
  return keys.map((key) => {
    const [pair, giver, receiver, strategy] = key.split("|");
    const own = rows.filter((row) =>
      row.pair === pair && row.giver === giver && row.receiver === receiver && row.strategy === strategy);
    const firstFeasible = own.filter((row) => row.firstChoiceFeasible !== null);
    return {
      pair,
      giver,
      receiver,
      strategy,
      repeats: own.length,
      sharedVoxels: own[0].sharedVoxels,
      voxelSize: own[0].voxelSize,
      poolSize: own[0].poolSize,
      feasiblePool: own[0].feasiblePool,
      feasibleFraction: own[0].feasibleFraction,
      feasibilityRate: firstFeasible.length === 0
        ? null
        : firstFeasible.reduce((sum, row) => sum + row.firstChoiceFeasible, 0) / firstFeasible.length,
      refused_mean: summarize(own.map((row) => (row.found ? row.refused : null))).mean,
      boundary_median: summarize(own.map((row) => row.boundaryDistance)).median,
      edgeCost_median: summarize(own.map((row) => row.edgeCost)).median
    };
  });
}

function refusedHistogram(rows, bins) {
  const order = [...STRATEGIES, "deployed"];
  const edges = bins || [0, 1, 2, 3, 5, 10, 25, 50, 100];
  return edges.map((edge, position) => {
    const upper = position + 1 < edges.length ? edges[position + 1] : Infinity;
    const row = { from: edge, to: Number.isFinite(upper) ? upper - 1 : null };
    for (const strategy of order) {
      const own = rows.filter((entry) => entry.strategy === strategy && entry.found);
      const inside = own.filter((entry) => entry.refused >= edge && entry.refused < upper).length;
      row[strategy] = own.length === 0 ? null : inside / own.length;
    }
    return row;
  });
}

// The optional third panel of the figure: the scores of one interaction space
// with the point every strategy picked in it.
function sliceRows(context) {
  const { pool, space } = context;
  const density = densities(pool, new Set());
  return pool.entries.map((entry, index) => ({
    x: entry.voxel.center.x,
    y: entry.voxel.center.y,
    z: entry.voxel.center.z,
    centrality: entry.centrality,
    density: density[index],
    giverSupport: entry.giverSupport,
    receiverSupport: entry.receiverSupport,
    posture: entry.posture,
    score: weightedScore({ ...entry, density: density[index] }),
    reachable: entry.reachable ? 1 : 0,
    postureOk: entry.postureOk ? 1 : 0,
    feasible: entry.feasible ? 1 : 0,
    voxelSize: space.voxelSize
  }));
}

function texNumber(value, digits = 3) {
  if (value === null || value === undefined || !Number.isFinite(value)) return "--";
  return value.toFixed(digits);
}

function strategyTable(summary, { pairs, directions, draws, executed = 0 }) {
  const lines = [
    "% Generated by src/eval/e9HandoverPoint.js -- do not edit by hand.",
    "\\begin{table}[htbp]",
    "    \\centering",
    `    \\caption{E9: selection of the handover point over ${pairs} usable pairs of the five-robot cell `
      + `in ${directions} directions, the random strategy averaged over ${draws} draws per direction. `
      + "All strategies rank the unscreened height band; \\emph{deployed} is the pipeline of "
      + "Section~\\ref{subsec:voxel_centrality} with its reachability screen, whose feasibility is one by construction. "
      + "$C$ and $D$ are normalized within the pool they are computed over, so the score of the "
      + "\\emph{deployed} row, whose pool is the trimmed central core, is not directly comparable with the others. "
      + (executed > 0
        ? "The two right-hand columns are the executed handovers of the five owner directions: the share that "
          + "completed on the strategy's first choice, and the share that completed once refused candidates were "
          + "replaced as the protocol replaces them."
        : "The executed-handover columns are empty because the live half has not been run.")
      + "}",
    "    \\label{tab:exp_e9}",
    "    \\small",
    "    \\begin{tabular}{@{}lrrrrrr@{}}",
    "        \\toprule",
    "        \\textbf{Strategy} & \\textbf{Feasible} & \\textbf{In the} & \\textbf{Boundary} "
      + "& \\textbf{Edge cost} & \\textbf{Executed} & \\textbf{Executed} \\\\",
    "         & first choice & shared set & distance [m] & [m] & first attempt & with retries \\\\",
    "        \\midrule"
  ];
  for (const row of summary) {
    lines.push(`        ${row.label} & ${texNumber(row.feasibilityRate, 3)} `
      + `& ${texNumber(row.inSpaceRate, 3)} & ${texNumber(row.boundary_median, 4)} `
      + `& ${texNumber(row.edgeCost_median, 3)} & ${texNumber(row.executedFirstAttempt, 3)} `
      + `& ${texNumber(row.executedSuccessRate, 3)} \\\\`);
  }
  lines.push("        \\bottomrule", "    \\end{tabular}", "\\end{table}", "");
  return lines.join("\n");
}

function figure(summary, histogram) {
  // pgfplots symbolic coordinates are names, not typeset text: a "$\geq$ 100"
  // among them is a parse error, and so is anything else that wants math mode.
  // The bins are therefore numbered and the labels attached separately.
  const binLabel = (row) => {
    if (row.to === null) return `$\\geq$ ${row.from}`;
    return row.from === row.to ? `${row.from}` : `${row.from}--${row.to}`;
  };
  const bins = histogram.map((row, index) => `b${index}`);
  const strategies = [...STRATEGIES, "deployed"];
  return [
    "% Generated by src/eval/e9HandoverPoint.js -- do not edit by hand.",
    "\\begin{figure}[htbp]",
    "    \\centering",
    "    \\begin{tikzpicture}",
    "        \\begin{axis}[",
    "            name=feasibility, width=0.46\\textwidth, height=5.5cm,",
    "            ybar, bar width=10pt, ymin=0, ymax=1.08,",
    "            ylabel={feasible first choice}, xlabel={strategy},",
    `            symbolic x coords={${summary.map((row, index) => `s${index}`).join(", ")}}, xtick=data,`,
    `            xticklabels={${summary.map((row) => row.label).join(", ")}},`,
    "            xticklabel style={rotate=35, anchor=east, font=\\footnotesize},",
    "            nodes near coords, every node near coord/.append style={font=\\tiny},",
    "            ymajorgrids, tick align=outside",
    "        ]",
    `            \\addplot+[fill=blue!35] coordinates {${summary
      .map((row, index) => `(s${index}, ${texNumber(row.feasibilityRate, 4)})`).join(" ")}};`,
    "        \\end{axis}",
    "        \\begin{axis}[",
    "            at={(feasibility.south east)}, xshift=1.6cm, anchor=south west,",
    "            width=0.46\\textwidth, height=5.5cm,",
    "            ybar, bar width=3pt, ymin=0, ymax=1.08,",
    "            ylabel={share of evaluations}, xlabel={refused candidates until success},",
    `            symbolic x coords={${bins.join(", ")}}, xtick=data,`,
    `            xticklabels={${histogram.map(binLabel).join(", ")}},`,
    "            xticklabel style={font=\\footnotesize},",
    "            legend style={font=\\tiny, at={(0.98,0.98)}, anchor=north east},",
    "            ymajorgrids, tick align=outside",
    "        ]",
    ...strategies.map((strategy) => [
      `            \\addplot coordinates {${histogram
        .map((row, index) => `(b${index}, ${texNumber(row[strategy], 4)})`).join(" ")}};`,
      `            \\addlegendentry{${STRATEGY_LABELS[strategy]}}`
    ].join("\n")),
    "        \\end{axis}",
    "    \\end{tikzpicture}",
    "    \\caption{E9: left, the rate at which the first point a strategy proposes can be served by both arms; "
      + "right, how many candidates the cell has to refuse before a feasible point stands. "
      + "All strategies rank the unscreened height band; the deployed pipeline screens it first, "
      + "so its feasibility is one by construction and its refusal count zero.}",
    "    \\label{fig:exp_e9}",
    "\\end{figure}",
    ""
  ].join("\n");
}

function usablePairs(descriptor, minimumSharedVoxels) {
  return (descriptor.interactionSpace?.pairs || []).map((pair) => ({
    key: pair.key,
    robots: canonicalPairRobots(pair.robots[0], pair.robots[1]),
    ownerRobot: pair.ownerRobot,
    minimumSharedVoxels
  }));
}

function analyze(options = {}) {
  const cell = options.cell || DEFAULT_CELL;
  const outDir = options.outDir || path.resolve(REPO_ROOT, DEFAULT_OUT_DIR);
  const minZ = options.minZ ?? DEFAULT_MIN_Z;
  const maxZ = options.maxZ ?? DEFAULT_MAX_Z;
  const draws = options.draws ?? DEFAULT_DRAWS;
  const seed = options.seed ?? DEFAULT_SEED;
  const descriptorPath = path.resolve(REPO_ROOT, descriptorForCell(cell));
  const descriptor = readJson(descriptorPath);
  const minimumSharedVoxels = Number(descriptor.interactionSpace?.minimumSharedVoxels ?? 20);
  const pairs = usablePairs(descriptor, minimumSharedVoxels);

  const rows = [];
  const skipped = [];
  let sliceContext = null;
  const slicePair = options.slicePair || DEFAULT_SLICE_PAIR;

  for (const pair of pairs) {
    const [first, second] = pair.robots;
    const controller = readOnlyController(pair.ownerRobot || first);
    const other = (pair.ownerRobot || first) === first ? second : first;
    const built = controller.build(other, { synthesizePair: true });
    if (!built.success) {
      skipped.push({ pair: pair.key, reason: built.reason });
      continue;
    }
    const space = controller.spaces.get(other);
    if (space.voxels.length < minimumSharedVoxels) {
      skipped.push({ pair: pair.key, reason: "interaction_space_below_minimum", sharedVoxels: space.voxels.length });
      continue;
    }
    const forward = directionRows({
      controller, space, giver: space.owner.name, receiver: space.partner.name,
      pairKey: pair.key, minZ, maxZ, draws, seed, descriptor
    });
    rows.push(...forward.rows);
    if (forward.skipped) skipped.push(forward.skipped);
    if (pair.key === slicePair && forward.pool) sliceContext = forward;

    // The other direction. rank() reads the owner off the space it is given,
    // so the reversed view is passed in and the controller's own manifest name
    // is only used for the deployed row, which is therefore taken from the
    // forward direction alone.
    const reversed = reverseView(space);
    const backward = directionRows({
      controller: Object.assign(Object.create(Object.getPrototypeOf(controller)), controller, {
        ctx: { ...controller.ctx, manifest: reversed.ownerManifest }
      }),
      space: reversed,
      giver: reversed.owner.name,
      receiver: reversed.partner.name,
      pairKey: pair.key,
      minZ, maxZ, draws, seed, descriptor
    });
    rows.push(...backward.rows.filter((row) => row.strategy !== "deployed"));
    if (backward.skipped) skipped.push(backward.skipped);
  }

  const summary = strategySummary(rows);
  const histogram = refusedHistogram(rows);
  const perPair = pairSummary(rows);
  // The live half, if it has been run. Its rows are joined onto the offline
  // summary by strategy, so one table can carry both halves and neither command
  // depends on the other having been run.
  const executionFile = path.join(outDir, "e9_handovers.jsonl");
  const executed = fs.existsSync(executionFile) ? readRecords(executionFile) : [];
  const execution = executed.length > 0 ? writeExecution(executed, outDir) : [];
  if (executed.length > 0) thesisFigureData(rows, executed, outDir);
  for (const row of summary) {
    const live = execution.find((entry) => entry.strategy === row.strategy);
    row.executedHandovers = live ? live.handovers : null;
    row.executedSuccessRate = live ? live.successRate : null;
    row.executedFirstAttempt = live ? live.firstAttemptSuccess : null;
    row.executedTransferRate = live ? live.transferRate : null;
    row.executedCommonFailure = live ? live.commonFailure : null;
  }
  const directions = new Set(rows.map((row) => `${row.pair}|${row.giver}|${row.receiver}`)).size;

  fs.mkdirSync(outDir, { recursive: true });
  fs.writeFileSync(path.join(outDir, "e9_runs.csv"), toCsv(rows, [
    "pair", "giver", "receiver", "strategy", "repeat", "poolSize", "feasiblePool", "feasibleFraction",
    "firstChoiceFeasible", "found", "refused", "boundaryDistance", "edgeCost",
    "centrality", "density", "giverSupport", "receiverSupport", "posture", "score",
    "x", "y", "z", "voxelSize", "sharedVoxels", "boxCentreInSpace", "boxCentreFeasible"
  ]));
  fs.writeFileSync(path.join(outDir, "e9_strategies.csv"), toCsv(summary, [
    "strategy", "label", "evaluations", "feasibilityRate", "foundRate",
    "refused_mean", "refused_median", "refused_q75", "refused_max",
    "inSpaceRate", "boundary_median", "boundary_q25", "boundary_q75",
    "centrality_median", "density_median", "giverSupport_median", "receiverSupport_median",
    "posture_median", "edgeCost_median", "edgeCost_q25", "edgeCost_q75", "score_median",
    "executedHandovers", "executedSuccessRate", "executedFirstAttempt", "executedTransferRate",
    "executedCommonFailure"
  ]));
  fs.writeFileSync(path.join(outDir, "e9_pairs.csv"), toCsv(perPair, [
    "pair", "giver", "receiver", "strategy", "repeats", "sharedVoxels", "voxelSize",
    "poolSize", "feasiblePool", "feasibleFraction", "feasibilityRate",
    "refused_mean", "boundary_median", "edgeCost_median"
  ]));
  fs.writeFileSync(path.join(outDir, "e9_refused.csv"), toCsv(histogram, [
    "from", "to", ...STRATEGIES, "deployed"
  ]));
  if (sliceContext) {
    fs.writeFileSync(path.join(outDir, "e9_slice.csv"), toCsv(sliceRows(sliceContext), [
      "x", "y", "z", "centrality", "density", "giverSupport", "receiverSupport",
      "posture", "score", "reachable", "postureOk", "feasible", "voxelSize"
    ]));
    const selected = rows.filter((row) => row.pair === slicePair && row.repeat === 0 && row.found);
    fs.writeFileSync(path.join(outDir, "e9_selected.csv"), toCsv(selected, [
      "pair", "giver", "receiver", "strategy", "x", "y", "z", "score", "refused", "boundaryDistance", "edgeCost"
    ]));
  }
  fs.writeFileSync(path.join(outDir, "e9_table.tex"),
    strategyTable(summary, { pairs: pairs.length - skipped.length, directions, draws, executed: executed.length }));
  fs.writeFileSync(path.join(outDir, "e9_figure.tex"), figure(summary, histogram));
  fs.writeFileSync(path.join(outDir, "e9_setup.json"), JSON.stringify({
    cell,
    descriptor: path.relative(REPO_ROOT, descriptorPath),
    clouds: "deployed (generated/robots/RobotN/info), placed by the five-robot descriptor",
    heightBand: [minZ, maxZ],
    weights: WEIGHTS,
    densityRadiusFactor: DENSITY_RADIUS_FACTOR,
    strategies: STRATEGIES,
    pool: "the unscreened height band of the interaction space; the deployed row uses rank()'s screened central core",
    feasibility: "both TCP targets within the workspace bounds and the admission tolerance of a reachable sample, "
      + "and posture margin at or above m_min -- the screen of rank(), evaluated rather than applied",
    edgeCostOrigin: "the loaded task site of the cell, in the giver's local frame (mode pickup of Section edge_costs)",
    draws,
    seed,
    minimumSharedVoxels,
    evaluatedPairs: pairs.length - skipped.length,
    directions,
    skipped,
    executedHandovers: executed.length > 0
      ? path.relative(REPO_ROOT, executionFile)
      : "not run -- see the run command for the executed-handover success rate",
    finishedAt: new Date().toISOString()
  }, null, 2) + "\n");

  return { rows, summary, histogram, perPair, execution, skipped, outDir, directions, draws };
}

// ---------------------------------------------------------------------------
// E9, live half -- the executed handover.
//
// The offline half answers what can be decided before the arms move, and on
// this cell it answers "everything is feasible": the pool IS the intersection
// of two reachable clouds, so both arms reach every point in it by
// construction, and the admission tolerance is wide enough that even the centre
// of the bounding box, which falls outside the shared voxel set in every
// direction, passes the screen. The predicate that separates a good rendezvous
// from a bad one is the one the cell applies with its arms: the approach legs,
// the preflight, the arbiter, and the receiver's own jaw gate.
//
// So each strategy's choice is handed to the LIVE cell and executed as a
// complete handover, and what comes back is the stage it failed at. The gate
// that matters is the receiver's: acceptTransferredGripObject measures from its
// contact pads to the cube and refuses with `receiver_not_at_cube` when the
// rendezvous did not actually put the cube between its jaws. Nothing here snaps
// the cube anywhere -- a success is a cube the receiver really closed on.
//
// DIRECTION. Only the owner of a pair can register a session
// (beginHandoverSession refuses `not_interaction_space_owner`), which is the
// rule of Section~(neighbours) and not a limitation of this script: along the
// routes of the scenario the giver is always the owner. The live half therefore
// runs the five owner directions, while the offline half scores both
// directions of every pair.
//
// SEEDING. Every trial starts with the cube in the giver's jaws, put there by
// this harness: static, unparented, placed at the giver's own tool control
// point, then closeGripper -- the driver's own grasp, which still measures the
// distance from the contact pads and can still refuse. E9 measures the
// rendezvous, not the route to it; driving the cube up the chain so that Robot4
// can give would make every Robot4 trial depend on three earlier handovers,
// which is E12's experiment.
//
// REFUSALS follow the protocol: a candidate the cell refuses is struck and the
// strategy's next one is offered, up to --retries, as the agent's
// handoverRetriesLeft(2) does. Density is re-ranked between strikes and
// centrality is not, exactly as offline. Every attempt is logged, so the
// refusal histogram has a live counterpart.
//
// The order of the strategies rotates from pair to pair, so no strategy is
// systematically the one that inherits a cell left untidy by its predecessor.
//
// Output: e9_handovers.jsonl (one line per ATTEMPT, appended, so an interrupted
// run resumes), e9_handovers.csv, e9_execution.csv (success rate and failure
// stages per strategy), e9_run_setup.json. `analyze` folds the jsonl into
// e9_strategies.csv and e9_table.tex whenever it is there.

const E9_LUA = String.raw`
function womatE9Cube(name)
    return sim.getObject('/'..name, {noError=true})
end

function womatE9State(name)
    local object=womatE9Cube(name)
    if object == -1 then return {success=false, reason="grip_object_missing", object=name} end
    local grasp, mode=womatToolControlPointWorld()
    if grasp == nil then grasp=womatAttachMarkerPosition() ; mode="attach_point_fallback" end
    local cube=sim.getObjectPosition(object, sim.handle_world)
    local state=getGripperState()
    return {success=true, holding=state.holding == true, object=name,
        cube={cube[1], cube[2], cube[3]},
        grasp={grasp[1], grasp[2], grasp[3]}, reference=mode,
        reach=womatDistance(grasp, cube)}
end

-- Put the cube in THIS arm's jaws and close on it. It is made static before it
-- is unparented so it cannot fall between the two calls, and the grasp itself
-- is the driver's own closeGripper, which still measures and can still refuse.
--
-- The two lines that look redundant are not. closeGripper begins with
-- womatEnsureGripObjects, and that function puts a cube NOBODY IS HOLDING back
-- on its support:
--
--     if heldGripObject ~= cube then
--         sim.setObjectPosition(cube, sim.handle_world, <configured position>)
--
-- so a seed that teleports the cube and then calls closeGripper has its
-- placement undone inside the very call it is preparing, and the grasp is then
-- measured against the support more than a metre away. That is exactly what the
-- first version of this did: every trial of every strategy refused with
-- object_not_in_grip_range, no arm moved, and the cube never left support A.
-- Hence: run the ensure step FIRST, while the cube is still where the driver
-- expects it, and claim the cube before closing so the ensure step inside
-- closeGripper leaves it where it has just been put.
function womatE9SeedCube(name)
    local object=womatE9Cube(name)
    if object == -1 then return {success=false, reason="grip_object_missing", object=name} end
    local ready=womatEnsureGripObjects()
    if ready.success == false then return ready end
    local grasp, mode=womatToolControlPointWorld()
    if grasp == nil then grasp=womatAttachMarkerPosition() ; mode="attach_point_fallback" end
    if grasp == nil then return {success=false, reason="no_tool_control_point", object=name} end
    womatSetShapePhysics(object, true, false)
    sim.setObjectParent(object, -1, true)
    sim.setObjectPosition(object, sim.handle_world, {grasp[1], grasp[2], grasp[3]})
    local claimed=heldGripObject
    heldGripObject=object
    local closed=closeGripper()
    if closed.success ~= true then heldGripObject=claimed end
    womatInvalidateSafetyCollections()
    local placed=sim.getObjectPosition(object, sim.handle_world)
    return {success=closed.success == true, reason=closed.reason, reference=mode,
        grasp={grasp[1], grasp[2], grasp[3]},
        cube={placed[1], placed[2], placed[3]},
        distance=closed.distance, threshold=closed.threshold, close=closed}
end

-- Let go and park the cube, static, so a trial that ended with it in the wrong
-- arm does not leave it lying in somebody's approach.
function womatE9ParkCube(name, x, y, z)
    local object=womatE9Cube(name)
    if object == -1 then return {success=false, reason="grip_object_missing", object=name} end
    if heldGripObject == object then openGripper() end
    womatSetShapePhysics(object, true, false)
    sim.setObjectParent(object, -1, true)
    sim.setObjectPosition(object, sim.handle_world, {x, y, z})
    womatInvalidateSafetyCollections()
    return {success=true, object=name}
end
`;

const HOME_JOINTS_DEG = [0, 0, 0, 0, 0, 0];
// In the order they are attempted. A failure is attributed to the first stage
// that refused, which is what makes "where does this rendezvous break" readable.
const STAGES = [
  "seed", "session", "giver_approach", "receiver_arm", "receiver_approach",
  "transfer", "receiver_retreat", "giver_retreat", "giver_home"
];
// The stages whose failure is a verdict on the POINT rather than on the cell.
// Only these strike the candidate and let the strategy offer its next one; a
// retreat that fails is the cell's problem and retrying the same rendezvous
// would not address it.
const POINT_STAGES = new Set(["giver_approach", "receiver_approach", "transfer"]);

function thingClient(manifest, timeoutMs) {
  const base = String(manifest.lowerLayerUrl).replace(/\/$/, "");
  const call = async (suffix, options = {}) => {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const response = await fetch(`${base}${suffix}`, { ...options, signal: controller.signal });
      if (!response.ok) return { success: false, reason: "thing_request_failed", status: response.status };
      const text = await response.text();
      return text ? JSON.parse(text) : { success: true };
    } catch (error) {
      return {
        success: false,
        reason: error?.name === "AbortError" ? "thing_timeout" : "thing_unreachable",
        message: error?.message || String(error)
      };
    } finally {
      clearTimeout(timer);
    }
  };
  return {
    name: manifest.name,
    manifest,
    get: (property) => call(`/properties/${property}`),
    post: (action, body) => call(`/actions/${action}`, {
      method: "POST",
      headers: { "content-type": "application/json" },
      body: JSON.stringify(body ?? {})
    })
  };
}

function jointPayload(td, degrees) {
  const names = Object.keys(td.actions?.moveTojointPosition?.input?.properties || {});
  return Object.fromEntries(names.map((name, index) => [name, degrees[index] ?? 0]));
}

function reasonOf(result) {
  if (!result) return "no_response";
  if (result.success === true) return null;
  return result.reason || result.state || result.message || "refused";
}

function localTcpPayload(target) {
  return { x: target.localTcp.x, y: target.localTcp.y, z: target.localTcp.z };
}

// One complete handover of one candidate. Returns the stage it stopped at and
// the reason the cell gave, with the milliseconds each stage took.
async function executeHandover({ giver, receiver, candidate, sessionId, log }) {
  const timing = {};
  const responses = {};
  let failed = null;
  const step = async (stage, action) => {
    if (failed) return null;
    const started = Date.now();
    const result = await action();
    timing[stage] = Date.now() - started;
    responses[stage] = result;
    const reason = reasonOf(result);
    if (reason) failed = { stage, reason };
    log(`      ${stage}: ${reason ? `refused (${reason})` : "ok"} [${timing[stage]} ms]`);
    return result;
  };

  await step("session", () => giver.post("beginHandoverSession", {
    session: sessionId,
    targetRobot: receiver.name,
    cubeWorld: candidate.cubeWorld
  }));
  await step("giver_approach", () => giver.post("moveTocartesianPosition", localTcpPayload(candidate.giverTarget)));
  await step("receiver_arm", () => receiver.post("setHandoverApproach", { session: sessionId, enabled: true }));
  await step("receiver_approach", () => receiver.post("moveTocartesianPosition", localTcpPayload(candidate.receiverTarget)));
  await step("transfer", () => giver.post("transferCube", { session: sessionId, targetRobot: receiver.name }));
  const transferred = !failed;
  await step("receiver_retreat", () => receiver.post("retreatFromHandover"));
  await step("giver_retreat", () => giver.post("retreatFromHandover"));
  await step("giver_home", () => giver.post("returnHomeAfterHandover", {
    session: sessionId,
    homeJoints: HOME_JOINTS_DEG
  }));

  // Teardown is unconditional: a session left open blocks the next trial's
  // beginHandoverSession with `handover_session_active`, and an approach left
  // armed leaves the other arm's gripper off the obstacle list.
  for (const client of [receiver, giver]) {
    await client.post("setHandoverApproach", { session: sessionId, enabled: false });
    await client.post("closeHandoverSession", { session: sessionId });
  }

  return {
    transferred,
    completed: !failed,
    stage: failed ? failed.stage : "complete",
    reason: failed ? failed.reason : null,
    timing,
    graspDistance: responses.transfer?.graspDistance ?? responses.transfer?.distance ?? null,
    graspTolerance: responses.transfer?.graspTolerance ?? responses.transfer?.tolerance ?? null,
    durationMs: Object.values(timing).reduce((sum, value) => sum + value, 0)
  };
}

// The cell as every trial must find it: nobody holding the cube, the cube
// parked on its support, all arms at the all-zero pose. `clients` must include
// the whole cell, even when --pairs selects only part of it: the last receiver
// can be outside the next pair and still claim the cube after it is reparented.
async function resetCell({ clients, homeFirst = [], tds, run, parkAt, cubeName, settleMs }) {
  for (const client of clients) {
    // Release unconditionally: an old run can leave a stale claim on an
    // unparented cube, for which getGripperState itself fails when it tries to
    // read the alias of parent -1. openGripper clears that claim normally.
    const released = await client.post("openGripper");
    if (released?.success !== true) {
      return { success: false, reason: `${client.name}_release_failed: ${reasonOf(released)}` };
    }
    const after = await client.get("getGripperState");
    if (after?.holding !== false || after.success === false) {
      return { success: false, reason: `${client.name}_release_not_confirmed` };
    }
  }
  const parked = await run(clients[0].name, `return womatE9ParkCube(${JSON.stringify(cubeName)}, `
    + `${parkAt.x}, ${parkAt.y}, ${parkAt.z})`);
  if (parked?.success !== true) return { success: false, reason: `park_failed: ${reasonOf(parked)}` };
  // As in the protocol's retreat, clear the receiver before the giver. A
  // refused receiver approach can leave it blocking the giver's home route.
  for (const client of new Set([...homeFirst, ...clients])) {
    const home = await client.post("moveTojointPosition", jointPayload(tds[client.name], HOME_JOINTS_DEG));
    if (home?.success !== true) {
      return { success: false, reason: `${client.name}_home_failed: ${reasonOf(home)}` };
    }
  }
  await new Promise((resolve) => setTimeout(resolve, settleMs));
  return { success: true };
}

// Records written before --repeats existed have no repetition index and are
// the zeroth round.
function repeatOf(record) {
  return Number.isFinite(Number(record.repeat)) ? Number(record.repeat) : 0;
}

function recordKey(record) {
  return `${record.pair}|${record.giver}|${record.strategy}|${repeatOf(record)}|${record.attempt}`;
}

function directionKey(record) {
  return `${record.pair}|${record.giver}|${record.strategy}|${repeatOf(record)}`;
}

// Wilson score interval. The normal approximation is useless at these counts --
// five successes out of five give it the width zero -- and a success rate
// measured on a handful of handovers has to carry an interval that admits how
// few they were.
function wilson(successes, total, z = 1.96) {
  if (!(total > 0)) return { low: null, high: null };
  const p = successes / total;
  const denominator = 1 + (z * z) / total;
  const centre = (p + (z * z) / (2 * total)) / denominator;
  const half = (z / denominator) * Math.sqrt(p * (1 - p) / total + (z * z) / (4 * total * total));
  return { low: Math.max(0, centre - half), high: Math.min(1, centre + half) };
}

// Older runs logged stale grip claims as session failures. Keep those raw
// records for diagnosis, but allow a rerun and exclude them from point metrics.
function isSetupFailure(record) {
  return record.stage === "seed"
    || (record.stage === "session" && record.reason === "cube_held_by_multiple_robots");
}

function resumeState(records) {
  const executed = records.filter((row) => !isSetupFailure(row));
  return {
    done: new Set(executed.map(recordKey)),
    finished: new Set(executed.filter((row) => row.final).map(directionKey))
  };
}

function readRecords(file) {
  if (!fs.existsSync(file)) return [];
  return fs.readFileSync(file, "utf8").split(/\r?\n/)
    .filter((line) => line.trim().length > 0)
    .map((line) => JSON.parse(line));
}

async function runLive(options) {
  const { connect, executeInScript } = require("../generator/core/simConnection");

  const outDir = options.outDir;
  fs.mkdirSync(outDir, { recursive: true });
  const jsonlFile = path.join(outDir, "e9_handovers.jsonl");
  const { done, finished } = resumeState(readRecords(jsonlFile));
  const append = (record) => fs.appendFileSync(jsonlFile, JSON.stringify(record) + "\n");
  const log = options.quiet ? () => {} : (line) => console.log(line);

  const descriptorPath = path.resolve(REPO_ROOT, descriptorForCell(options.cell));
  const descriptor = readJson(descriptorPath);
  const cubeName = descriptor.cube || descriptor.interactionSpace?.cube;
  const site = (descriptor.taskSites || []).find((entry) => entry.payload?.position);
  if (!cubeName || !site) throw new Error("The cell descriptor names no cube on a loaded task site.");
  const parkAt = {
    x: Number(site.payload.position[0]),
    y: Number(site.payload.position[1]),
    z: Number(site.payload.position[2])
  };

  const { sim } = await connect({ robwotRoot: path.join(REPO_ROOT, "external", "RobWoT") });
  if (Number(await sim.getSimulationState()) === Number(await sim.simulation_stopped)) {
    throw new Error("The simulation is stopped -- serve the cell first (npm run robot:serve -- --manifests ...).");
  }
  const scripts = new Map();
  const scriptOf = async (name) => {
    if (scripts.has(name)) return scripts.get(name);
    const handle = Number(first(await sim.getObject(`/${name}`)));
    const script = Number(first(await sim.getScript(await sim.scripttype_customizationscript, handle, `/${name}`)));
    if (!(script >= 0)) throw new Error(`${name} has no driver script -- is the cell served?`);
    await executeInScript(sim, script, E9_LUA + "\nreturn true");
    scripts.set(name, script);
    return script;
  };
  const run = async (name, code) => executeInScript(sim, await scriptOf(name), code);

  const minimumSharedVoxels = Number(descriptor.interactionSpace?.minimumSharedVoxels ?? 20);
  const pairs = usablePairs(descriptor, minimumSharedVoxels)
    .filter((pair) => !options.pairs || options.pairs.includes(pair.key));
  const strategies = options.strategies || LIVE_STRATEGIES;

  const clients = new Map();
  const tds = {};
  const clientFor = (name) => {
    if (!clients.has(name)) {
      const manifest = readJson(manifestPath(name));
      clients.set(name, thingClient(manifest, options.timeoutMs));
      tds[name] = readJson(path.resolve(REPO_ROOT, manifest.generatedFiles.lowerTd));
    }
    return clients.get(name);
  };
  const cellClients = descriptor.robots.map((robot) => clientFor(robot.name));

  const records = [];
  // Repetitions are the OUTER loop, so an interrupted run leaves whole rounds
  // behind rather than a deep sample of the first pair and nothing of the last.
  const repeats = Math.max(1, Number(options.repeats) || 1);
  let round = 0;
  let pairIndex = 0;
  // Two setups in a row that the cell refuses mean the harness cannot put the
  // cube in an arm at all, and every remaining trial would fail identically and
  // report a success rate of zero that says nothing about any rendezvous.
  let seedFailures = 0;
  for (round = 0; round < repeats; round += 1) {
    if (repeats > 1) log(`\nRound ${round + 1} of ${repeats}`);
    pairIndex = 0;
    for (const pair of pairs) {
      const [firstRobot, secondRobot] = pair.robots;
      const giverName = pair.ownerRobot || firstRobot;
      const receiverName = giverName === firstRobot ? secondRobot : firstRobot;
      const controller = readOnlyController(giverName);
      const built = controller.build(receiverName, { synthesizePair: true });
      if (!built.success) {
        log(`  ${pair.key}: no interaction space (${built.reason})`);
        pairIndex += 1;
        continue;
      }
      const space = controller.spaces.get(receiverName);
      const pool = poolOf(controller, space, giverName, receiverName, { minZ: options.minZ, maxZ: options.maxZ });
      if (!pool) {
        log(`  ${pair.key}: no voxel in the height band`);
        pairIndex += 1;
        continue;
      }
      const centre = bboxCentre(pool.band);
      const bboxDistance = pool.entries.map(({ voxel }) => Math.hypot(
        voxel.center.x - centre.x, voxel.center.y - centre.y, voxel.center.z - centre.z
      ));
      const grid = outsideCells(space);
      const centreEntry = pool.evaluate(centre, CENTRE_INDEX);
      const maxRawDensity = Math.max(...pool.neighbours.map((list) => list.length), 1);
      const centreDensity = pool.entries.reduce((count, entry) => count + (
        Math.hypot(
          entry.voxel.center.x - centre.x, entry.voxel.center.y - centre.y, entry.voxel.center.z - centre.z
        ) <= space.voxelSize * DENSITY_RADIUS_FACTOR ? 1 : 0), 0) / maxRawDensity;
      const context = { shuffled: [], bboxDistance, centreEntry, centreDensity };

      const giver = clientFor(giverName);
      const receiver = clientFor(receiverName);

      // `deployed` does not draw from the unscreened pool: it is rank() with its
      // central-core trim, its reachability screen and its cap, ranked by Eq.
      // (handover_score) exactly as the agent ranks it, and its refusals go
      // through the controller so that density is recomputed the way the live
      // cell recomputes it.
      const deployedCandidate = () => {
        const ranked = controller.rank(receiverName, { minZ: options.minZ, maxZ: options.maxZ });
        if (!ranked.success || ranked.candidates.length === 0) return null;
        // rank() discounts a refused candidate's DENSITY but still returns it:
        // striking it from the running is the AGENT's job, "among all candidates
        // that have not been refused before", and the geometry capability never
        // sees the belief that does it. Doing it here is therefore not a
        // workaround but the missing half of the deployed pipeline. Without it
        // every retry re-offers the point that was just struck -- on the first
        // live run Robot3/Robot5 was handed the same voxel at (2.613, 0.764,
        // 0.292) three times and refused it three times, and the strategy's
        // retried success rate measured nothing.
        const struck = controller.rejectedCandidates.get(receiverName) || new Set();
        const open = ranked.candidates.filter((candidate) => !struck.has(candidate.id));
        if (open.length === 0) return null;
        const best = open.slice()
          .sort((left, right) => weightedScore(right) - weightedScore(left))[0];
        return {
          id: best.id,
          entry: {
            index: best.id,
            voxel: {
              center: best.sharedPointWorld,
              index: pointToVoxel(best.sharedPointWorld, space.origin, space.voxelSize)
            },
            cubeWorld: best.cubeWorld,
            giverTarget: best.giver,
            receiverTarget: best.receiver,
            centrality: best.centrality,
            density: best.density,
            giverSupport: best.giverSupport,
            receiverSupport: best.receiverSupport,
            posture: best.posture,
            postureMargin: best.postureMargin,
            feasible: true
          }
        };
      };

      // Rotate, so that a strategy is not always the one that runs first (or
      // last) on a given pair.
      const order = strategies.map(
        (_, index) => strategies[(index + pairIndex + round) % strategies.length]
      );
      for (const strategy of order) {
        const key = `${pair.key}|${giverName}|${strategy}|${round}`;
        if (finished.has(key)) { log(`  ${pair.key} ${strategy}: already recorded, skipping`); continue; }
        log(`  ${pair.key} ${giverName} -> ${receiverName}, ${STRATEGY_LABELS[strategy]}`);
        const refused = new Set();
        if (strategy === "deployed") controller.clearRejectedCandidates(receiverName);
        if (strategy === "random") {
          // A new draw every round: repeating one draw would measure the point,
          // not the strategy, and the spread of the random strategy is exactly
          // what the repetitions are for.
          // Round 0 reduces to the pre-repetition formula, so the draws of the
          // round already on disk stay reproducible.
          const random = seededRandom(options.seed + pairIndex + round * 7919);
          context.shuffled = pool.entries.map((entry) => entry.index);
          for (let i = context.shuffled.length - 1; i > 0; i -= 1) {
            const j = Math.floor(random() * (i + 1));
            [context.shuffled[i], context.shuffled[j]] = [context.shuffled[j], context.shuffled[i]];
          }
        }
        for (let attempt = 0; attempt <= options.retries; attempt += 1) {
          let index = null;
          let entry = null;
          let own = null;
          if (strategy === "deployed") {
            const picked = deployedCandidate();
            if (!picked) break;
            index = picked.id;
            entry = picked.entry;
            own = entry.density;
          } else {
            const order_ = ordering(strategy, pool, refused, context);
            if (order_.length === 0) break;
            index = order_[0];
            entry = index === CENTRE_INDEX ? centreEntry : pool.entries[index];
            own = index === CENTRE_INDEX ? centreDensity : densities(pool, refused)[index];
          }

          // Rebuild the refusal history of an interrupted strategy so its next
          // attempt does not accidentally choose the point it already refused.
          if (done.has(`${key}|${attempt}`)) {
            if (strategy === "deployed") controller.rejectCandidate(receiverName, index);
            else refused.add(index);
            continue;
          }

          const ready = await resetCell({
            clients: cellClients, homeFirst: [receiver, giver],
            tds, run, parkAt, cubeName, settleMs: options.settleMs
          });
          if (!ready.success) throw new Error(`The cell could not be reset before ${key}: ${ready.reason}`);
          const seeded = await run(giverName, `return womatE9SeedCube(${JSON.stringify(cubeName)})`);
          const sessionId = `e9_${pair.key}_${strategy}_r${round}_${attempt}`.replace(/[^A-Za-z0-9_]/g, "_").toLowerCase();

          let outcome;
          if (seeded?.success !== true) {
            seedFailures += 1;
            outcome = {
              transferred: false, completed: false, stage: "seed",
              reason: seeded?.reason || "seed_failed", timing: {}, durationMs: 0,
              graspDistance: Number.isFinite(seeded?.distance) ? seeded.distance : null,
              graspTolerance: Number.isFinite(seeded?.threshold) ? seeded.threshold : null
            };
            log(`      seed: refused (${outcome.reason}`
              + `${Number.isFinite(seeded?.distance) ? `, ${seeded.distance.toFixed(3)} m from the jaws` : ""})`);
            if (seedFailures >= 2) {
              throw new Error(
                `The cube could not be put into ${giverName}'s gripper (${outcome.reason}). `
                + "Nothing was executed. This is the harness, not the cell: check that the cell is served with "
                + "the shared grip (npm run robot:serve -- --manifests ...) and that the arms are idle, "
                + `then delete ${path.join(outDir, "e9_handovers.jsonl")} and run again.`
              );
            }
          } else {
            seedFailures = 0;
            outcome = await executeHandover({ giver, receiver, candidate: entry, sessionId, log });
          }

          const strike = !outcome.completed && POINT_STAGES.has(outcome.stage);
          const record = {
            pair: pair.key,
            giver: giverName,
            receiver: receiverName,
            strategy,
            repeat: round,
            attempt,
            session: sessionId,
            x: entry.voxel.center.x,
            y: entry.voxel.center.y,
            z: entry.voxel.center.z,
            fromBoxCentre: index === CENTRE_INDEX ? 1 : 0,
            inSharedSet: grid.occupied.has(entry.voxel.index.join(",")) ? 1 : 0,
            offlineFeasible: entry.feasible ? 1 : 0,
            centrality: entry.centrality,
            density: own,
            giverSupport: entry.giverSupport,
            receiverSupport: entry.receiverSupport,
            posture: entry.posture,
            postureMargin: entry.postureMargin,
            score: weightedScore({ ...entry, density: own }),
            transferred: outcome.transferred ? 1 : 0,
            completed: outcome.completed ? 1 : 0,
            stage: outcome.stage,
            reason: outcome.reason,
            graspDistance: outcome.graspDistance,
            graspTolerance: outcome.graspTolerance,
            durationMs: outcome.durationMs,
            timing: outcome.timing,
            struck: strike ? 1 : 0,
            // The last attempt for this strategy, whether it succeeded or ran out
            // of candidates. A resumed run skips only strategies that reached one.
            final: outcome.completed || !strike || attempt === options.retries,
            at: new Date().toISOString()
          };
          append(record);
          records.push(record);
          if (record.stage === "session" && isSetupFailure(record)) {
            throw new Error(`The cell still reports multiple cube holders after reset before ${key}. `
              + "Stop other controllers and check the gripper states before rerunning E9; this setup failure is retryable.");
          }
          done.add(`${key}|${attempt}`);
          log(`      -> ${record.completed ? "complete" : `failed at ${record.stage} (${record.reason})`}`);
          if (record.final) { finished.add(key); break; }
          if (strategy === "deployed") controller.rejectCandidate(receiverName, index);
          else refused.add(index);
        }
      }
      pairIndex += 1;
    }
  }

  // Leave the cell as it was found rather than mid-rendezvous.
  const cleaned = await resetCell({
    clients: cellClients, tds, run, parkAt, cubeName, settleMs: options.settleMs
  });
  if (!cleaned.success) throw new Error(`The cell could not be reset after E9: ${cleaned.reason}`);

  const all = readRecords(jsonlFile);
  writeExecution(all, outDir);
  fs.writeFileSync(path.join(outDir, "e9_run_setup.json"), JSON.stringify({
    cell: options.cell,
    descriptor: path.relative(REPO_ROOT, descriptorPath),
    cube: cubeName,
    parkedAt: parkAt,
    directions: "owner directions only -- beginHandoverSession refuses a non-owner giver",
    heightBand: [options.minZ, options.maxZ],
    strategies,
    retries: options.retries,
    repeats,
    interval: "95 % Wilson over directions x rounds. It treats those trials as independent, which they are "
      + "not: rounds of one direction share a geometry, so the interval is optimistic where a direction "
      + "succeeds or fails consistently. e9_directions.csv carries the per-direction counts, which is where "
      + "that clustering is visible.",
    seed: options.seed,
    settleMs: options.settleMs,
    timeoutMs: options.timeoutMs,
    seeding: "the cube is placed at the giver's tool control point and grasped with the driver's own closeGripper",
    successCriterion: "transferCube accepted by the receiver's own jaw gate (acceptTransferredGripObject), "
      + "and both arms clear of the rendezvous afterwards",
    attempts: all.length,
    finishedAt: new Date().toISOString()
  }, null, 2) + "\n");
  return { records: all, outDir };
}

function executionSummary(records) {
  const order = [...LIVE_STRATEGIES];
  return order.map((strategy) => {
    const own = records.filter((row) => row.strategy === strategy);
    if (own.length === 0) return null;
    // Setup failures are counted and reported, never averaged into a rate:
    // without exclusive giver ownership, the trial tested no rendezvous.
    const setupFailures = own.filter(isSetupFailure).length;
    const tried = own.filter((row) => !isSetupFailure(row));
    const firsts = tried.filter((row) => row.attempt === 0);
    const finals = tried.filter((row) => row.final);
    const stages = {};
    for (const row of tried) {
      if (row.completed) continue;
      stages[row.stage] = (stages[row.stage] || 0) + 1;
    }
    const worst = Object.entries(stages).sort((left, right) => right[1] - left[1])[0] || [null, 0];
    // One trial per direction per round, so the denominators here are
    // directions x rounds and the interval is over that many Bernoulli trials.
    const firstWon = firsts.filter((row) => row.completed).length;
    const finalWon = finals.filter((row) => row.completed).length;
    const firstInterval = wilson(firstWon, firsts.length);
    const finalInterval = wilson(finalWon, finals.length);
    const rounds = new Set(tried.map(repeatOf)).size;
    return {
      strategy,
      label: STRATEGY_LABELS[strategy] || strategy,
      handovers: finals.length,
      attempts: tried.length,
      setupFailures,
      rounds,
      directions: new Set(tried.map((row) => `${row.pair}|${row.giver}`)).size,
      firstAttemptWon: firstWon,
      firstAttemptTrials: firsts.length,
      firstAttemptLow: firstInterval.low,
      firstAttemptHigh: firstInterval.high,
      successWon: finalWon,
      successTrials: finals.length,
      successLow: finalInterval.low,
      successHigh: finalInterval.high,
      firstAttemptSuccess: firsts.length === 0
        ? null
        : firsts.filter((row) => row.completed).length / firsts.length,
      transferRate: firsts.length === 0
        ? null
        : firsts.filter((row) => row.transferred).length / firsts.length,
      successRate: finals.length === 0
        ? null
        : finals.filter((row) => row.completed).length / finals.length,
      liveRefused_mean: finals.length === 0
        ? null
        : tried.filter((row) => row.struck).length / finals.length,
      commonFailure: worst[0],
      commonFailureCount: worst[1],
      duration_median: summarize(tried.map((row) => row.durationMs)).median,
      graspDistance_median: summarize(tried.map((row) => row.graspDistance)).median
    };
  }).filter(Boolean);
}

// Which direction was flaky, rather than only how often a strategy worked.
// A strategy that fails the same direction every round and one that fails a
// different direction each round have the same success rate and are not the
// same finding.
function directionSummary(records) {
  const tried = records.filter((row) => !isSetupFailure(row));
  const keys = [...new Set(tried.map((row) => `${row.pair}|${row.giver}|${row.receiver}|${row.strategy}`))];
  return keys.map((key) => {
    const [pair, giver, receiver, strategy] = key.split("|");
    const own = tried.filter((row) =>
      row.pair === pair && row.giver === giver && row.strategy === strategy);
    const firsts = own.filter((row) => row.attempt === 0);
    const finals = own.filter((row) => row.final);
    return {
      pair,
      giver,
      receiver,
      strategy,
      rounds: new Set(own.map(repeatOf)).size,
      firstAttemptWon: firsts.filter((row) => row.completed).length,
      firstAttemptTrials: firsts.length,
      completed: finals.filter((row) => row.completed).length,
      trials: finals.length,
      attempts: own.length,
      duration_median: summarize(own.map((row) => row.durationMs)).median
    };
  }).sort((left, right) => left.strategy.localeCompare(right.strategy)
    || left.pair.localeCompare(right.pair));
}

function writeExecution(records, outDir) {
  const summary = executionSummary(records);
  fs.writeFileSync(path.join(outDir, "e9_directions.csv"), toCsv(directionSummary(records), [
    "pair", "giver", "receiver", "strategy", "rounds",
    "firstAttemptWon", "firstAttemptTrials", "completed", "trials", "attempts", "duration_median"
  ]));
  fs.writeFileSync(path.join(outDir, "e9_execution.csv"), toCsv(summary, [
    "strategy", "label", "handovers", "attempts", "setupFailures", "rounds", "directions",
    "firstAttemptSuccess", "firstAttemptWon", "firstAttemptTrials", "firstAttemptLow", "firstAttemptHigh",
    "transferRate", "successRate", "successWon", "successTrials", "successLow", "successHigh",
    "liveRefused_mean", "commonFailure", "commonFailureCount",
    "duration_median", "graspDistance_median"
  ]));
  fs.writeFileSync(path.join(outDir, "e9_handovers.csv"), toCsv(records, [
    "pair", "giver", "receiver", "strategy", "attempt", "session",
    "x", "y", "z", "fromBoxCentre", "inSharedSet", "offlineFeasible",
    "centrality", "density", "giverSupport", "receiverSupport", "posture", "postureMargin", "score",
    "transferred", "completed", "stage", "reason", "graspDistance", "graspTolerance",
    "durationMs", "struck", "final", "at"
  ]));
  return summary;
}

function cliArguments(argv) {
  const args = {};
  for (let index = 0; index < argv.length; index += 1) {
    const token = argv[index];
    if (!token.startsWith("--")) continue;
    const [name, inline] = token.slice(2).split("=");
    if (inline !== undefined) { args[name] = inline; continue; }
    const next = argv[index + 1];
    if (next && !next.startsWith("--")) { args[name] = next; index += 1; } else { args[name] = true; }
  }
  return args;
}

function printExecution(summary, outDir) {
  if (summary.length === 0) {
    console.log("\nE9 run: no handovers were executed.");
    return;
  }
  console.log("\nE9, executed handovers:");
  for (const row of summary) {
    const interval = (value, low, high) => (row.rounds > 1 && low !== null
      ? `${texNumber(value, 3)} [${texNumber(low, 3)}, ${texNumber(high, 3)}]`
      : texNumber(value, 3));
    console.log(`  ${row.label.padEnd(22)} first attempt `
      + `${row.firstAttemptWon}/${row.firstAttemptTrials} = `
      + `${interval(row.firstAttemptSuccess, row.firstAttemptLow, row.firstAttemptHigh)}, `
      + `with retries ${interval(row.successRate, row.successLow, row.successHigh)}, `
      + `${row.handovers} handover(s) in ${row.attempts} attempt(s)`
      + `${row.setupFailures > 0 ? `, ${row.setupFailures} setup failure(s)` : ""}, `
      + `usual failure ${row.commonFailure || "none"}, `
      + `${texNumber(row.duration_median, 0)} ms`);
  }
  const rounds = Math.max(...summary.map((row) => row.rounds), 0);
  console.log(rounds > 1
    ? `${rounds} rounds; the brackets are 95\u00a0% Wilson intervals over directions x rounds.`
    : "One round only, so the rates carry no interval -- pass --repeats to get one.");
  console.log(`Records: ${outDir}`);
  console.log("Run \"npm run eval:e9\" to fold these into the tables.");
}

function printSummary(result) {
  console.log(`\nE9: ${result.directions} pair/direction evaluation(s) of the five-robot cell, `
    + `random averaged over ${result.draws} draws.`);
  for (const row of result.summary) {
    console.log(`  ${row.label.padEnd(22)} feasible ${texNumber(row.feasibilityRate, 3)}, `
      + `refused ${texNumber(row.refused_mean, 2)} (max ${texNumber(row.refused_max, 0)}), `
      + `in the shared set ${texNumber(row.inSpaceRate, 3)}, `
      + `boundary ${texNumber(row.boundary_median, 4)} m, `
      + `C ${texNumber(row.centrality_median, 3)}, D ${texNumber(row.density_median, 3)}, `
      + `P ${texNumber(row.posture_median, 3)}, score ${texNumber(row.score_median, 3)}, `
      + `edge cost ${texNumber(row.edgeCost_median, 3)} m`);
  }
  for (const entry of result.skipped) {
    console.log(`  skipped ${entry.pair || entry.pairKey}: ${entry.reason}`);
  }
  console.log(`Tables and figures: ${result.outDir}`);
}

function parseList(value, allowed, what) {
  const parts = String(value).split(",").map((part) => part.trim()).filter((part) => part.length > 0);
  const unknown = allowed ? parts.filter((part) => !allowed.includes(part)) : [];
  if (unknown.length > 0) throw new Error(`Unknown ${what}: ${unknown.join(", ")}`);
  return parts;
}

async function main(argv) {
  const args = cliArguments(argv);
  const command = argv.find((token) => !token.startsWith("--")) || "analyze";
  if (command !== "analyze" && command !== "run") {
    console.error(`Unknown command "${command}". E9 has two: "analyze" (offline) and "run" (the live cell).`);
    return 1;
  }
  const outDir = args.out ? path.resolve(REPO_ROOT, args.out) : path.resolve(REPO_ROOT, DEFAULT_OUT_DIR);
  if (command === "run") {
    const live = await runLive({
      cell: args.cell ? Number(args.cell) : DEFAULT_CELL,
      outDir,
      minZ: args["min-z"] ? Number(args["min-z"]) : DEFAULT_MIN_Z,
      maxZ: args["max-z"] ? Number(args["max-z"]) : DEFAULT_MAX_Z,
      strategies: args.strategies ? parseList(args.strategies, LIVE_STRATEGIES, "strategy") : null,
      pairs: args.pairs ? parseList(args.pairs, null, "pair") : null,
      retries: args.retries === undefined ? DEFAULT_RETRIES : Number(args.retries),
      repeats: args.repeats === undefined ? DEFAULT_REPEATS : Number(args.repeats),
      seed: args.seed ? Number(args.seed) : DEFAULT_SEED,
      settleMs: args.settle ? Number(args.settle) : DEFAULT_SETTLE_MS,
      timeoutMs: args.timeout ? Number(args.timeout) : DEFAULT_TIMEOUT_MS,
      quiet: args.quiet === true
    });
    printExecution(executionSummary(live.records), live.outDir);
    return 0;
  }
  const result = analyze({
    cell: args.cell ? Number(args.cell) : DEFAULT_CELL,
    outDir,
    minZ: args["min-z"] ? Number(args["min-z"]) : DEFAULT_MIN_Z,
    maxZ: args["max-z"] ? Number(args["max-z"]) : DEFAULT_MAX_Z,
    draws: args.draws ? Number(args.draws) : DEFAULT_DRAWS,
    seed: args.seed ? Number(args.seed) : DEFAULT_SEED,
    slicePair: args["slice-pair"] || DEFAULT_SLICE_PAIR
  });
  printSummary(result);
  return 0;
}

if (require.main === module) {
  main(process.argv.slice(2))
    .then((code) => process.exit(code))
    .catch((error) => {
      console.error(error?.message || error);
      process.exit(1);
    });
}

module.exports = {
  LIVE_STRATEGIES,
  STRATEGIES,
  STRATEGY_LABELS,
  WEIGHTS,
  analyze,
  boundaryDistance,
  outsideCells,
  bboxCentre,
  densities,
  descriptorForCell,
  edgeCost,
  figure,
  ordering,
  pairSummary,
  pickupOrigin,
  poolOf,
  readOnlyController,
  refusedHistogram,
  reverseView,
  sliceRows,
  strategySummary,
  E9_LUA,
  STAGES,
  POINT_STAGES,
  directionSummary,
  executeHandover,
  executionSummary,
  isSetupFailure,
  jointPayload,
  reasonOf,
  readRecords,
  resetCell,
  resumeState,
  runLive,
  strategyTable,
  thesisFigureData,
  thingClient,
  walk,
  weightedScore,
  wilson
};
