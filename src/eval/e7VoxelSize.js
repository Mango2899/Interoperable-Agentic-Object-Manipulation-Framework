#!/usr/bin/env node
// E7 and E8 -- the automatic voxel size.
//
// The objective is one sentence of the thesis: does Eq. (voxel_size) yield
// interaction spaces of the intended SIZE and RESOLUTION at a low COST? Each
// word is measured on its own.
//
//   size        the voxel count against N_target = 1000, which needs nothing
//               but the count itself.
//   resolution  whether the grid is one the samples support: an independent run
//               of the sampler must produce the same interaction space, and the
//               size must stay above the native spacing that the estimator's
//               own lower bound protects.
//   cost        the time to choose the size and the time to voxelize, separated.
//
// E7 compares the rule with three baselines on those three terms; E8 sweeps the
// size from 0.01 to 0.2 m and shows what each of them buys and costs, with the
// size the estimator chose marked on the curve it is judged against.
//
// Entirely offline, like E5 and E6: the clouds are the ones E1 sampled
// (generated/experiments/e1/samples, ten seeds of 100k joint configurations per
// robot), placed at the base positions of the two-, three-, four- and
// five-robot descriptors. Nothing here talks to CoppeliaSim.
//
// Four methods choose a voxel size for the same pair of clouds:
//
//   fixed        s = 0.05 m, the same edge length for every pair, which is the
//                baseline any hand-tuned system starts from.
//   bbox         s = cbrt(V_overlap / N_target) from the overlap volume of the
//                two axis-aligned bounding boxes ALONE -- the information the
//                estimator has before it looks at a single sample. This is the
//                estimator's own fallback path (interactionSpace.js), used here
//                as a standalone method and without the native-spacing floor,
//                since the point of the baseline is what the boxes know.
//   proposed     the deployed estimateInteractionSpace(), i.e. native spacing,
//                the bidirectional proximity estimate N_close, and Eq.
//                (voxel_size) with its lower bound at the native spacing.
//   sweep        two offline oracles over the E8 sweep, which serve only as a
//                reference and are not implementable online: the size whose
//                shared-voxel count comes closest to N_target, and the finest
//                size whose interaction space an independent run reproduces.
//
// The reference. The thesis plans "the intersection on a fine grid of 0.01 m"
// as ground truth, and these clouds cannot carry it. On a 1 cm grid two clouds
// almost never put a sample in the same cell -- Robot2/Robot4 shares 3143
// voxels at the size the estimator picks and 456 cells at 0.01 m, Robot3/Robot5
// 3536 and 109 -- so the criterion measures how often two independent samplers
// hit the same cubic centimetre. Pooling all ten runs does not repair it
// (23 967 cells, 0.024 m^3, for Robot2/Robot4), and giving every sample a
// volume, by dilating the pooled clouds before intersecting them, only moves
// the problem into the dilation radius: "calibrate" measures that region at
// radii from 0 to 0.03 m and it grows from 0.008 to 0.40 m^3, while doubling
// the sample budget still adds 20 per cent at the widest radius. A region built
// this way is a free parameter, not a ground truth, and a Jaccard against it
// would report the parameter.
//
// What can be measured without inventing a region is whether the interaction
// space is a property of the cell or of the draw. Two things are compared, both
// on the pair's fixed grid, so that "on a common grid" is true by construction:
//
//   J_repro    the same pair computed from an INDEPENDENT run of the sampler at
//              the same budget the deployed system uses (seeds are paired off
//              1-2, 3-4, ...). If the grid is finer than the samples support,
//              the two runs disagree, because a voxel counts as shared only when
//              both clouds happen to place a sample in it; if it is coarse
//              enough, they agree. This is what the resolution claim is about,
//              and it has no free parameter.
//   coverage   the same pair computed from the nine other runs pooled, i.e. at
//              ten times the samples: how much of what more sampling finds one
//              run recovers. This is the conservativeness of E1, priced at the
//              size each method chose.
//
// Neither needs a ground-truth region, and the estimator's own claims are
// checked directly besides: the voxel count against N_target, the size against
// the native spacing it may not fall below (s/d_eff), and the samples per
// occupied voxel, which is why that floor exists.
//
// E8 sweeps the size from 0.01 to 0.2 m. The upper end is wider than the thesis
// plans because the agreement of two runs is still below 0.85 at 0.1 m for
// every pair of the cell, so a threshold crossing has to be looked for further
// out; and the cost side of "accuracy against computation time" turns out to be
// flat, since voxelizing is linear in the samples and barely depends on s.
//
// Timing. The deployed estimator chooses the size and voxelizes in one call, so
// the split the thesis asks for is obtained by timing the whole call and
// subtracting a separate voxelization at the size it returned: estimateMs is
// the KD-trees, the spacing and the proximity estimate, voxelizeMs the two hash
// sets and their intersection. For the oracles, estimateMs is the whole sweep,
// which is what makes them offline.
//
// Output: performance_log/e7 (per-run and per-method tables, the E7 table and
// the scatter for the left panel) and performance_log/e8 (the sweep, the
// accuracy/cost curve and the figure).

const fs = require("node:fs");
const path = require("node:path");

const {
  KdTree3D,
  bboxOverlapVolume,
  canonicalPairKey,
  canonicalPairRobots,
  estimateInteractionSpace,
  estimateSpacing,
  loadReachableCsv,
  pointToVoxel,
  toWorld,
  voxelKey
} = require("../generator/interactionSpace");
const { parseSeeds, summarize, toCsv } = require("./e1Convergence");
const { descriptorForCell, parseIntegerList, placeCloud } = require("./e5Neighbours");

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const DEFAULT_SAMPLES_DIR = "generated/experiments/e1/samples";
const DEFAULT_E7_DIR = "performance_log/e7";
const DEFAULT_E8_DIR = "performance_log/e8";
const DEFAULT_CELLS = [2, 3, 4, 5];
// Wide enough that the reproducibility threshold is actually crossed: at 0.1 m
// the agreement of two runs is still below 0.85 for every pair of the cell.
const DEFAULT_SWEEP = { from: 0.01, to: 0.2, step: 0.005 };
const DEFAULT_REPRO_THRESHOLD = 0.75;
const DEFAULT_REFERENCE_SIZE = 0.01;
// Measured, not assumed: "calibrate" reports the smallest radius at which the
// reference region stops growing with the sample budget, and this is what it
// found for the cells of this thesis (see performance_log/e7/e7_reference_choice.csv).
const DEFAULT_REFERENCE_RADIUS = 0.02;
const DEFAULT_CALIBRATION_RADII = [0, 0.01, 0.015, 0.02, 0.025, 0.03];
const DEFAULT_CALIBRATION_TOLERANCE = 0.05;
const DEFAULT_FIXED_SIZE = 0.05;
const DEFAULT_MINIMUM_SHARED_VOXELS = 20;
const WORLD_ORIGIN = { x: 0, y: 0, z: 0 };
const AXES = ["x", "y", "z"];

// ---------------------------------------------------------------- pure parts

function sweepSizes({ from, to, step }) {
  if (!(from > 0) || !(to >= from) || !(step > 0)) {
    throw new Error(`Sweep must be 0 < from <= to with a positive step: ${from},${to},${step}`);
  }
  const sizes = [];
  const count = Math.round((to - from) / step);
  for (let index = 0; index <= count; index += 1) {
    // Accumulating the step would drift; 0.01 + 6*0.005 must be exactly 0.04.
    sizes.push(Number((from + index * step).toFixed(6)));
  }
  return sizes;
}

function parseSweep(value) {
  const parts = String(value).split(",").map((token) => Number(token.trim()));
  if (parts.length !== 3 || parts.some((part) => !Number.isFinite(part))) {
    throw new Error(`--sweep must be from,to,step, e.g. 0.01,0.1,0.005: ${value}`);
  }
  return { from: parts[0], to: parts[1], step: parts[2] };
}

function bounds(points) {
  const min = { x: Infinity, y: Infinity, z: Infinity };
  const max = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (const point of points) {
    for (const axis of AXES) {
      if (point[axis] < min[axis]) min[axis] = point[axis];
      if (point[axis] > max[axis]) max[axis] = point[axis];
    }
  }
  return { min, max };
}

function boxIntersection(left, right, margin = 0) {
  const min = {};
  const max = {};
  for (const axis of AXES) {
    min[axis] = Math.max(left.min[axis], right.min[axis]) - margin;
    max[axis] = Math.min(left.max[axis], right.max[axis]) + margin;
    if (max[axis] <= min[axis]) return null;
  }
  return { min, max };
}

function pointsInBox(points, box) {
  if (!box) return [];
  return points.filter((point) => AXES.every((axis) => point[axis] >= box.min[axis] && point[axis] <= box.max[axis]));
}

// The grid the deployed estimator would use for these two clouds: the
// component-wise minimum over both. Every method shares it, so that two methods
// differing only in s differ only in s.
function gridOrigin(firstBounds, secondBounds) {
  return {
    x: Math.min(firstBounds.min.x, secondBounds.min.x),
    y: Math.min(firstBounds.min.y, secondBounds.min.y),
    z: Math.min(firstBounds.min.z, secondBounds.min.z)
  };
}

function occupiedKeys(points, size, origin) {
  const keys = new Set();
  for (const point of points) keys.add(voxelKey(pointToVoxel(point, origin, size)));
  return keys;
}

// The interaction space at a GIVEN size: the same two hash sets and the same
// intersection as estimateInteractionSpace(), without its choice of size, which
// is exactly what a baseline or a sweep step needs.
function sharedKeysAt(first, second, size, origin) {
  const firstOccupied = occupiedKeys(first, size, origin);
  const secondOccupied = occupiedKeys(second, size, origin);
  const [small, large] = firstOccupied.size <= secondOccupied.size
    ? [firstOccupied, secondOccupied]
    : [secondOccupied, firstOccupied];
  const shared = new Set();
  for (const key of small) if (large.has(key)) shared.add(key);
  return shared;
}

function timedShared(first, second, size, origin) {
  const started = process.hrtime.bigint();
  const firstOccupied = occupiedKeys(first, size, origin);
  const secondOccupied = occupiedKeys(second, size, origin);
  const [small, large] = firstOccupied.size <= secondOccupied.size
    ? [firstOccupied, secondOccupied]
    : [secondOccupied, firstOccupied];
  const shared = new Set();
  for (const key of small) if (large.has(key)) shared.add(key);
  const voxelizeMs = Number(process.hrtime.bigint() - started) / 1e6;
  return { shared, voxelizeMs, occupiedFirst: firstOccupied.size, occupiedSecond: secondOccupied.size };
}

// What a voxel size does to the clouds themselves, without any reference.
//
// The estimator's lower bound is the native spacing, and the reason is here: as
// the voxels shrink towards it, each occupied voxel holds about one sample, and
// a voxel counts as shared only if BOTH clouds placed a sample in it, so the
// intersection collapses into dust however much space the arms really share.
// samplesPerVoxel says how far from that limit a size is, sizeOverSpacing says
// whether the estimator sat on its own floor, and fill says whether what came
// out is a body or a scatter: the share of the cells of the interaction space's
// own bounding box that the interaction space occupies.
function resolutionDiagnostics({ shared, first, second, occupiedFirst, occupiedSecond, size, spacing }) {
  const box = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] };
  for (const key of shared) {
    const index = key.split(",");
    for (let axis = 0; axis < 3; axis += 1) {
      const value = Number(index[axis]);
      if (value < box.min[axis]) box.min[axis] = value;
      if (value > box.max[axis]) box.max[axis] = value;
    }
  }
  const boxCells = shared.size === 0
    ? 0
    : [0, 1, 2].reduce((product, axis) => product * (box.max[axis] - box.min[axis] + 1), 1);
  return {
    samplesPerVoxelFirst: occupiedFirst === 0 ? null : first.length / occupiedFirst,
    samplesPerVoxelSecond: occupiedSecond === 0 ? null : second.length / occupiedSecond,
    sizeOverSpacing: spacing > 0 ? size / spacing : null,
    fill: boxCells === 0 ? null : shared.size / boxCells
  };
}

// Reference cells live as integer codes, not as "i,j,k" strings and not as
// {x,y,z} objects: a pooled occupancy runs to a million cells per robot, and a
// million strings or objects cost hundreds of megabytes where a million numbers
// in a Set cost a few. The code packs the three indices of the 0.01 m lattice,
// which reaches +-10 m around the world origin.
const REFERENCE_OFFSET = 1024;
const REFERENCE_SPAN = 2048;

function referenceCode(i, j, k) {
  const x = i + REFERENCE_OFFSET;
  const y = j + REFERENCE_OFFSET;
  const z = k + REFERENCE_OFFSET;
  if (x < 0 || y < 0 || z < 0 || x >= REFERENCE_SPAN || y >= REFERENCE_SPAN || z >= REFERENCE_SPAN) {
    throw new Error("A reference cell lies outside +-10 m of the world origin; the lattice cannot hold it.");
  }
  return (x * REFERENCE_SPAN + y) * REFERENCE_SPAN + z;
}

function decodeReference(code) {
  const z = code % REFERENCE_SPAN;
  const rest = (code - z) / REFERENCE_SPAN;
  const y = rest % REFERENCE_SPAN;
  const x = (rest - y) / REFERENCE_SPAN;
  return [x - REFERENCE_OFFSET, y - REFERENCE_OFFSET, z - REFERENCE_OFFSET];
}

// The occupied cells of one robot on the reference lattice, over every seed of
// the run. The points are consumed seed by seed and dropped again: only the
// cell codes survive, which is what makes a million samples per robot
// affordable.
function poolOccupancy(placedCloudsBySeed, size) {
  const codes = new Set();
  for (const points of placedCloudsBySeed) {
    for (const point of points) {
      codes.add(referenceCode(
        Math.floor(point.x / size),
        Math.floor(point.y / size),
        Math.floor(point.z / size)
      ));
    }
  }
  return codes;
}

// Every offset within a ball of `radius` cells, the radius included.
function dilationOffsets(radius) {
  const offsets = [];
  const limit = Math.floor(radius);
  for (let i = -limit; i <= limit; i += 1) {
    for (let j = -limit; j <= limit; j += 1) {
      for (let k = -limit; k <= limit; k += 1) {
        if (i * i + j * j + k * k <= radius * radius) offsets.push([i, j, k]);
      }
    }
  }
  return offsets;
}

// The reference region of one pair: cells that both robots reach to within
// `radius` cells, i.e. the intersection of the two dilated pooled occupancies.
//
// Both dilations are built and intersected, which is linear in the occupancies
// times the size of the ball; testing one side through the offsets instead
// would be quadratic in the ball and unusable at the radii the calibration has
// to reach. Everything is cut to the region that can hold a reference cell
// first -- the overlap of the two bounding boxes grown by the radius, since a
// cell within the radius of both clouds lies within the radius of both boxes.
function pooledReference(firstCodes, secondCodes, radius) {
  const offsets = dilationOffsets(radius);
  const limit = Math.ceil(radius);
  const boxOf = (codes) => {
    const min = [Infinity, Infinity, Infinity];
    const max = [-Infinity, -Infinity, -Infinity];
    for (const code of codes) {
      const cell = decodeReference(code);
      for (let axis = 0; axis < 3; axis += 1) {
        if (cell[axis] < min[axis]) min[axis] = cell[axis];
        if (cell[axis] > max[axis]) max[axis] = cell[axis];
      }
    }
    return { min, max };
  };
  const firstBox = boxOf(firstCodes);
  const secondBox = boxOf(secondCodes);
  // Where a reference cell can lie, and where a sample that produces one can.
  const candidateMin = [0, 1, 2].map((axis) => Math.max(firstBox.min[axis], secondBox.min[axis]) - limit);
  const candidateMax = [0, 1, 2].map((axis) => Math.min(firstBox.max[axis], secondBox.max[axis]) + limit);
  if ([0, 1, 2].some((axis) => candidateMax[axis] < candidateMin[axis])) return new Set();

  const within = (cell, min, max) => [0, 1, 2].every((axis) => cell[axis] >= min[axis] && cell[axis] <= max[axis]);
  const sourceMin = candidateMin.map((value) => value - limit);
  const sourceMax = candidateMax.map((value) => value + limit);
  const dilate = (codes) => {
    const grown = new Set();
    for (const code of codes) {
      const cell = decodeReference(code);
      if (!within(cell, sourceMin, sourceMax)) continue;
      for (const offset of offsets) {
        const candidate = [cell[0] + offset[0], cell[1] + offset[1], cell[2] + offset[2]];
        if (!within(candidate, candidateMin, candidateMax)) continue;
        grown.add(referenceCode(candidate[0], candidate[1], candidate[2]));
      }
    }
    return grown;
  };
  const grownFirst = dilate(firstCodes);
  if (grownFirst.size === 0) return new Set();
  const grownSecond = dilate(secondCodes);
  const [small, large] = grownFirst.size <= grownSecond.size ? [grownFirst, grownSecond] : [grownSecond, grownFirst];
  const reference = new Set();
  for (const code of small) if (large.has(code)) reference.add(code);
  return reference;
}

function relativeDeviation(shared, target) {
  return (shared - target) / target;
}

// ---------------------------------------------------------------- methods

function fixedSize(size) {
  return { name: `fixed_${size}`, estimateMs: 0, voxelSize: size, mode: "fixed" };
}

// cbrt(V_overlap / N_target) -- the boxes and nothing else. No native-spacing
// floor: that floor is knowledge from the samples, which this baseline does not
// have.
function boundingBoxSize(first, second, target) {
  const started = process.hrtime.bigint();
  const overlap = bboxOverlapVolume(bounds(first), bounds(second));
  const size = Math.cbrt(overlap / target);
  return {
    name: "bbox",
    estimateMs: Number(process.hrtime.bigint() - started) / 1e6,
    voxelSize: size,
    mode: "bounding_box",
    overlapVolume: overlap
  };
}

function proposedSize(first, second, options, origin) {
  const started = process.hrtime.bigint();
  const result = estimateInteractionSpace(first, second, options);
  const totalMs = Number(process.hrtime.bigint() - started) / 1e6;
  if (!result.voxelSize) {
    return { name: "proposed", estimateMs: totalMs, voxelSize: null, mode: result.reason || "failed" };
  }
  // The call did both jobs; the voxelization is priced separately and taken off
  // so that estimateMs is the size choice alone.
  const { voxelizeMs } = timedShared(first, second, result.voxelSize, origin);
  return {
    name: "proposed",
    estimateMs: Math.max(totalMs - voxelizeMs, 0),
    voxelSize: result.voxelSize,
    mode: result.success ? (result.diagnostics?.estimatorMode ?? "kd_tree_proximity") : result.estimatorMode || result.reason,
    deployedVoxels: result.success ? result.voxels.length : 0,
    effectiveSpacing: result.diagnostics?.effectiveSpacing ?? null,
    estimatedClosePoints: result.diagnostics?.estimatedClosePoints ?? result.estimatedClosePoints ?? null,
    deployedOriginMatches: result.origin
      ? AXES.every((axis) => Math.abs(result.origin[axis] - origin[axis]) < 1e-12)
      : false
  };
}

// ---------------------------------------------------------------- one pair

function jaccardOf(left, right) {
  const [small, large] = left.size <= right.size ? [left, right] : [right, left];
  let common = 0;
  for (const key of small) if (large.has(key)) common += 1;
  const union = left.size + right.size - common;
  return {
    common,
    jaccard: union === 0 ? null : common / union,
    coverage: right.size === 0 ? null : common / right.size,
    precision: left.size === 0 ? null : common / left.size
  };
}

// Seeds are paired off 1-2, 3-4, ... so that a run is compared with exactly one
// other run and no run is compared with itself. An odd count leaves the last
// seed paired with the one before it, which repeats a comparison rather than
// inventing one.
function partnerSeeds(seeds) {
  const partner = new Map();
  for (let index = 0; index < seeds.length; index += 2) {
    const left = seeds[index];
    const right = seeds[index + 1] ?? seeds[index - 1];
    if (right === undefined) continue;
    partner.set(left, right);
    partner.set(right, left);
  }
  return partner;
}

// The set of shared voxels of one run at one size, on the pair's fixed grid.
//
// Every run of a pair uses the SAME origin, or two runs could not be compared
// voxel by voxel at all; the deployed estimator re-derives the origin from the
// two clouds it was handed, which moves the lattice by a few millimetres
// between runs. Its own count is recorded separately as deployedVoxels, so what
// the fixed lattice costs stays visible.
//
// The sets are cached per pair. One pair asks for the same (size, run) over and
// over -- the sweep walks every size for every run, the comparison against the
// nine other runs asks for all of them at the size one method chose, and a
// fixed baseline asks for the same size ten times over -- while a shared set is
// a few thousand keys, so keeping them is cheap and recomputing them is not.
function sharedAt(clouds, size, origin, cache) {
  if (!cache) return sharedKeysAt(clouds.first, clouds.second, size, origin);
  const key = size.toFixed(6);
  let bySeed = cache.get(key);
  if (!bySeed) {
    bySeed = new Map();
    cache.set(key, bySeed);
  }
  let shared = bySeed.get(clouds.seed);
  if (!shared) {
    shared = sharedKeysAt(clouds.first, clouds.second, size, origin);
    bySeed.set(clouds.seed, shared);
  }
  return shared;
}

// What one voxel size does, measured against the two things that need no
// ground truth:
//
//   reproducibility  the same computation on an INDEPENDENT run of the sampler,
//                    at the same budget the deployed system uses. If the grid is
//                    finer than the samples support, two runs of the same cell
//                    disagree, because a voxel counts as shared only when both
//                    clouds happen to place a sample in it; if it is coarse
//                    enough, they agree. This is the reference the resolution
//                    claim is actually about, and it has no free parameter.
//
//   completeness     the same computation on the nine other runs pooled, i.e.
//                    at ten times the sample budget. Coverage says how much of
//                    what more sampling finds one run recovers, precision how
//                    much of one run's space the denser sampling confirms. It
//                    is the conservativeness of E1, priced at the size the
//                    method chose.
function agreementAt({ perSeed, seed, partner, size, origin, withPooled, cache }) {
  const own = sharedAt(perSeed.get(seed), size, origin, cache);
  const other = sharedAt(perSeed.get(partner), size, origin, cache);
  const repro = jaccardOf(own, other);
  const result = {
    sharedVoxels: own.size,
    partnerVoxels: other.size,
    reproJaccard: repro.jaccard,
    reproCommon: repro.common
  };
  if (!withPooled) return { ...result, own };
  const pooled = new Set();
  for (const [otherSeed, clouds] of perSeed) {
    if (otherSeed === seed) continue;
    for (const key of sharedAt(clouds, size, origin, cache)) pooled.add(key);
  }
  const against = jaccardOf(own, pooled);
  return {
    ...result,
    own,
    pooledVoxels: pooled.size,
    pooledCoverage: against.coverage,
    pooledPrecision: against.precision,
    pooledJaccard: against.jaccard
  };
}

// One pair in one run: every method's size and what that size yields. The sweep
// is measured once per pair for all runs together (sweepPair), because every
// run of the pair walks the same sizes.
function evaluateRun({
  clouds, full, perSeed, seed, partner, origin, sizes, sweep, options, target, fixed,
  minimumSharedVoxels, cache, effectiveSpacing
}) {
  // Measured once per run by the caller, from the whole clouds and with the
  // deployed parameters. One call costs a KD-tree over ninety thousand points
  // and twenty thousand queries, so it is not measured twice.

  const methods = [];
  const measure = (chosen) => {
    if (!(chosen.voxelSize > 0)) {
      methods.push({
        ...chosen,
        sharedVoxels: 0,
        reproJaccard: null,
        pooledCoverage: null,
        pooledPrecision: null,
        voxelizeMs: null
      });
      return;
    }
    // The cost is what the deployed code pays: both clouds whole, not the
    // trimmed ones the set comparisons use.
    const timed = timedShared(full.first, full.second, chosen.voxelSize, origin);
    const agreement = agreementAt({
      perSeed, seed, partner, size: chosen.voxelSize, origin, withPooled: true, cache
    });
    methods.push({
      ...chosen,
      sharedVoxels: agreement.sharedVoxels,
      relDeviation: relativeDeviation(agreement.sharedVoxels, target),
      sharedVolume: agreement.sharedVoxels * chosen.voxelSize ** 3,
      reproJaccard: agreement.reproJaccard,
      partnerVoxels: agreement.partnerVoxels,
      pooledVoxels: agreement.pooledVoxels,
      pooledCoverage: agreement.pooledCoverage,
      pooledPrecision: agreement.pooledPrecision,
      pooledJaccard: agreement.pooledJaccard,
      voxelizeMs: timed.voxelizeMs,
      ...resolutionDiagnostics({
        shared: agreement.own,
        first: full.first,
        second: full.second,
        occupiedFirst: timed.occupiedFirst,
        occupiedSecond: timed.occupiedSecond,
        size: chosen.voxelSize,
        spacing: effectiveSpacing
      })
    });
  };

  measure(fixedSize(fixed));
  measure(boundingBoxSize(full.first, full.second, target));
  const proposed = proposedSize(full.first, full.second, options, origin);
  measure(proposed);

  // The offline references, both read from the sweep of this pair and this run,
  // so both pay for the whole sweep.
  const own = sweep.filter((step) => step.seed === seed);
  const sweepMs = own.reduce((total, step) => total + step.voxelizeMs, 0);
  const byTarget = [...own].sort((left, right) =>
    Math.abs(left.sharedVoxels - target) - Math.abs(right.sharedVoxels - target) || left.voxelSize - right.voxelSize)[0];
  // The finest size whose interaction space an independent run reproduces:
  // what the estimator would have to find to be right about the resolution.
  const reproducible = own
    .filter((step) => step.reproJaccard !== null && step.reproJaccard >= options.reproThreshold)
    .sort((left, right) => left.voxelSize - right.voxelSize)[0];
  if (byTarget) measure({ name: "sweep_target", estimateMs: sweepMs, voxelSize: byTarget.voxelSize, mode: "offline_sweep" });
  if (reproducible) {
    measure({ name: "sweep_repro", estimateMs: sweepMs, voxelSize: reproducible.voxelSize, mode: "offline_sweep" });
  } else {
    methods.push({
      name: "sweep_repro",
      estimateMs: sweepMs,
      voxelSize: null,
      mode: "no_size_reproducible",
      sharedVoxels: 0,
      reproJaccard: null,
      pooledCoverage: null,
      pooledPrecision: null,
      voxelizeMs: null
    });
  }

  return {
    effectiveSpacing,
    deployedVoxels: proposed.deployedVoxels ?? 0,
    deployedOrigin: proposed.deployedOrigin ?? null,
    usable: (proposed.deployedVoxels ?? 0) >= minimumSharedVoxels,
    methods
  };
}

// The sweep of one pair: every size, every run, the agreement of each run with
// its partner. The pooled comparison is left out here -- it is ten times the
// work and the methods are where it is read.
function sweepPair({ perSeed, partner, origin, sizes, target, spacingBySeed, cache }) {
  const rows = [];
  for (const size of sizes) {
    const shared = new Map();
    for (const [seed, clouds] of perSeed) {
      const started = process.hrtime.bigint();
      shared.set(seed, sharedAt(clouds, size, origin, cache));
      rows.push({
        seed,
        voxelSize: size,
        voxelizeMs: Number(process.hrtime.bigint() - started) / 1e6
      });
    }
    for (const row of rows.filter((entry) => entry.voxelSize === size)) {
      const own = shared.get(row.seed);
      const other = shared.get(partner.get(row.seed));
      const repro = other ? jaccardOf(own, other) : { jaccard: null, common: null };
      row.sharedVoxels = own.size;
      row.relDeviation = relativeDeviation(own.size, target);
      row.sharedVolume = own.size * size ** 3;
      row.reproJaccard = repro.jaccard;
      row.partnerVoxels = other ? other.size : null;
      row.sizeOverSpacing = spacingBySeed.get(row.seed) > 0 ? size / spacingBySeed.get(row.seed) : null;
    }
  }
  return rows;
}

// ---------------------------------------------------------------- experiment

function seedDirectories(samplesDir, wanted) {
  if (!fs.existsSync(samplesDir)) {
    throw new Error(`No samples directory: ${samplesDir}. Run "npm run eval:e1:sample" first, or point --samples-dir at the clouds.`);
  }
  const found = fs.readdirSync(samplesDir)
    .map((name) => ({ name, match: name.match(/^seed_(-?\d+)$/) }))
    .filter((entry) => entry.match)
    .map((entry) => ({ seed: Number(entry.match[1]), dir: path.join(samplesDir, entry.name) }))
    .sort((left, right) => left.seed - right.seed);
  return wanted ? found.filter((entry) => wanted.includes(entry.seed)) : found;
}

function loadDescriptor(descriptorPath) {
  const descriptor = JSON.parse(fs.readFileSync(descriptorPath, "utf8"));
  if (!Array.isArray(descriptor.robots) || descriptor.robots.length < 2) {
    throw new Error(`Descriptor lists fewer than two robots: ${descriptorPath}`);
  }
  return descriptor;
}

function placementOf(robot) {
  return {
    offset: (robot.offset || [0, 0, 0]).map(Number),
    orientationDeg: Array.isArray(robot.orientationDeg)
      ? robot.orientationDeg.map(Number)
      : [0, 0, Number(robot.rotationDeg || 0)]
  };
}

function cellPairs(robotNames) {
  const pairs = [];
  for (let first = 0; first < robotNames.length; first += 1) {
    for (let second = first + 1; second < robotNames.length; second += 1) {
      const robots = canonicalPairRobots(robotNames[first], robotNames[second]);
      pairs.push({ key: canonicalPairKey(robots[0], robots[1]), robots });
    }
  }
  return pairs.sort((left, right) => left.key.localeCompare(right.key));
}

// Every seed's clouds, in the robot-local frame, as flat coordinate triples.
// Ten seeds of five robots are four and a half million points; as objects they
// would be a gigabyte, as Float64Arrays they are a hundred megabytes, and the
// placement into a cell rebuilds the objects for one robot at a time.
function loadLocalClouds(seedDirs, robotNames) {
  const clouds = new Map();
  for (const { seed, dir } of seedDirs) {
    const perRobot = new Map();
    let complete = true;
    for (const name of robotNames) {
      const csv = path.join(dir, `${name}_data_point.csv`);
      if (!fs.existsSync(csv)) { complete = false; break; }
      const points = loadReachableCsv(csv);
      if (points.length === 0) { complete = false; break; }
      const flat = new Float64Array(points.length * 3);
      for (let index = 0; index < points.length; index += 1) {
        flat[index * 3] = points[index].x;
        flat[index * 3 + 1] = points[index].y;
        flat[index * 3 + 2] = points[index].z;
      }
      perRobot.set(name, flat);
    }
    if (!complete) {
      console.warn(`${path.basename(dir)}: incomplete (a robot's CSV is missing), skipped.`);
      continue;
    }
    clouds.set(seed, perRobot);
  }
  if (clouds.size === 0) throw new Error("No complete seed directory to analyse.");
  return clouds;
}

function placeFlat(flat, placement) {
  const points = new Array(flat.length / 3);
  for (let index = 0; index < points.length; index += 1) {
    points[index] = { x: flat[index * 3], y: flat[index * 3 + 1], z: flat[index * 3 + 2] };
  }
  return placeCloud(points, placement);
}

// Outside the overlap of the two bounding boxes, grown by one voxel, no voxel
// can hold a sample of both clouds: a voxel of edge s containing one point of
// each has both within s of the box. The sets are therefore identical to the
// ones the whole clouds produce, and small enough to build ten of them per
// size. Timings never use these -- they use the whole clouds.
function trimToOverlap(first, second, firstBounds, secondBounds, margin) {
  const box = boxIntersection(firstBounds, secondBounds, margin);
  if (!box) return { first: [], second: [], box: null };
  return { first: pointsInBox(first, box), second: pointsInBox(second, box), box };
}

function runCells({
  cells, seedDirs, options, sizes, target, fixed, minimumSharedVoxels, reproThreshold
}) {
  const descriptors = new Map();
  const robotNames = new Set();
  for (const size of cells) {
    const descriptorPath = path.resolve(REPO_ROOT, descriptorForCell(size));
    if (!fs.existsSync(descriptorPath)) throw new Error(`No descriptor for the ${size}-robot cell: ${descriptorPath}`);
    const descriptor = loadDescriptor(descriptorPath);
    descriptors.set(size, {
      placements: new Map(descriptor.robots.map((robot) => [robot.name, placementOf(robot)])),
      pairs: cellPairs(descriptor.robots.map((robot) => robot.name))
    });
    for (const robot of descriptor.robots) robotNames.add(robot.name);
  }

  const localClouds = loadLocalClouds(seedDirs, [...robotNames]);
  const seeds = [...localClouds.keys()].sort((left, right) => left - right);
  const partner = partnerSeeds(seeds);
  if (partner.size === 0) throw new Error("The reference compares a run with another run; at least two seeds are needed.");
  const widest = Math.max(...sizes);

  // JIT warm-up, so the first timed run is not an outlier.
  estimateInteractionSpace(
    [{ x: 0, y: 0, z: 0 }, { x: 0.1, y: 0, z: 0 }],
    [{ x: 0, y: 0, z: 0 }, { x: 0.1, y: 0, z: 0 }],
    options
  );

  const runs = [];
  const sweepRows = [];
  const pairMeta = [];
  for (const cell of cells) {
    const { placements, pairs } = descriptors.get(cell);
    for (const pair of pairs) {
      const [nameA, nameB] = pair.robots;
      const started = Date.now();

      // Pass one: where the pair's clouds are, over every run, so that the grid
      // and the trim box are the same for all of them.
      const pooledFirst = { min: { x: Infinity, y: Infinity, z: Infinity }, max: { x: -Infinity, y: -Infinity, z: -Infinity } };
      const pooledSecond = { min: { x: Infinity, y: Infinity, z: Infinity }, max: { x: -Infinity, y: -Infinity, z: -Infinity } };
      const merge = (into, box) => {
        for (const axis of AXES) {
          into.min[axis] = Math.min(into.min[axis], box.min[axis]);
          into.max[axis] = Math.max(into.max[axis], box.max[axis]);
        }
      };
      for (const seed of seeds) {
        merge(pooledFirst, bounds(placeFlat(localClouds.get(seed).get(nameA), placements.get(nameA))));
        merge(pooledSecond, bounds(placeFlat(localClouds.get(seed).get(nameB), placements.get(nameB))));
      }
      const origin = gridOrigin(pooledFirst, pooledSecond);

      // Pass two: the trimmed clouds of every run, which every set comparison
      // of this pair is built from.
      const perSeed = new Map();
      const spacingBySeed = new Map();
      for (const seed of seeds) {
        const first = placeFlat(localClouds.get(seed).get(nameA), placements.get(nameA));
        const second = placeFlat(localClouds.get(seed).get(nameB), placements.get(nameB));
        perSeed.set(seed, { seed, ...trimToOverlap(first, second, pooledFirst, pooledSecond, widest) });
        spacingBySeed.set(seed, Math.max(
          estimateSpacing(first, options.sampleSize ?? 20000, options.spacingPercentile ?? 0.75, options.seed ?? 42),
          estimateSpacing(second, options.sampleSize ?? 20000, options.spacingPercentile ?? 0.75, options.seed ?? 42)
        ));
      }

      // Two arms whose bounding boxes miss each other share no voxel at any
      // size: the deployed estimator refuses the pair at its overlap gate, and
      // there is nothing here to sweep either.
      const overlapping = [...perSeed.values()].some((entry) => entry.first.length > 0 && entry.second.length > 0);
      if (!overlapping) {
        pairMeta.push({
          cell,
          pair: pair.key,
          robotA: nameA,
          robotB: nameB,
          seeds: seeds.length,
          usableSeeds: 0,
          origin: `${origin.x.toFixed(3)} ${origin.y.toFixed(3)} ${origin.z.toFixed(3)}`,
          medianDeployedVoxels: 0,
          medianVoxelSize: null,
          medianReproJaccard: null,
          seconds: (Date.now() - started) / 1000
        });
        console.log(`${cell}-robot cell, ${pair.key}: bounding boxes disjoint, no shared voxel at any size.`);
        continue;
      }

      const cache = new Map();
      const sweep = sweepPair({ perSeed, partner, origin, sizes, target, spacingBySeed, cache });
      for (const row of sweep) {
        sweepRows.push({ cell, pair: pair.key, robotA: nameA, robotB: nameB, ...row });
      }

      for (const seed of seeds) {
        const full = {
          first: placeFlat(localClouds.get(seed).get(nameA), placements.get(nameA)),
          second: placeFlat(localClouds.get(seed).get(nameB), placements.get(nameB))
        };
        const result = evaluateRun({
          clouds: perSeed.get(seed),
          full,
          perSeed,
          seed,
          partner: partner.get(seed),
          origin,
          sizes,
          sweep,
          options: { ...options, reproThreshold },
          target,
          fixed,
          minimumSharedVoxels,
          cache,
          effectiveSpacing: spacingBySeed.get(seed)
        });
        const common = {
          cell,
          seed,
          partnerSeed: partner.get(seed),
          pair: pair.key,
          robotA: nameA,
          robotB: nameB,
          usable: result.usable,
          effectiveSpacing: result.effectiveSpacing,
          deployedVoxels: result.deployedVoxels
        };
        for (const method of result.methods) {
          runs.push({
            ...common,
            method: method.name,
            mode: method.mode,
            voxelSize: method.voxelSize,
            sharedVoxels: method.sharedVoxels,
            absDeviation: Math.abs(method.sharedVoxels - target),
            relDeviation: method.relDeviation ?? null,
            sharedVolume: method.sharedVolume ?? null,
            partnerVoxels: method.partnerVoxels ?? null,
            reproJaccard: method.reproJaccard,
            pooledVoxels: method.pooledVoxels ?? null,
            pooledCoverage: method.pooledCoverage,
            pooledPrecision: method.pooledPrecision,
            samplesPerVoxelA: method.samplesPerVoxelFirst ?? null,
            samplesPerVoxelB: method.samplesPerVoxelSecond ?? null,
            sizeOverSpacing: method.sizeOverSpacing ?? null,
            fill: method.fill ?? null,
            estimateMs: method.estimateMs,
            voxelizeMs: method.voxelizeMs,
            totalMs: method.voxelizeMs === null ? method.estimateMs : method.estimateMs + method.voxelizeMs
          });
        }
      }

      const proposedRuns = runs.filter((row) => row.pair === pair.key && row.cell === cell && row.method === "proposed");
      const usable = proposedRuns.filter((row) => row.usable).length;
      pairMeta.push({
        cell,
        pair: pair.key,
        robotA: nameA,
        robotB: nameB,
        seeds: seeds.length,
        usableSeeds: usable,
        origin: `${origin.x.toFixed(3)} ${origin.y.toFixed(3)} ${origin.z.toFixed(3)}`,
        medianDeployedVoxels: summarize(proposedRuns.map((row) => row.deployedVoxels)).median,
        medianVoxelSize: summarize(proposedRuns.map((row) => row.voxelSize)).median,
        medianReproJaccard: summarize(proposedRuns.map((row) => row.reproJaccard)).median,
        seconds: (Date.now() - started) / 1000
      });
      const last = pairMeta[pairMeta.length - 1];
      console.log(`${cell}-robot cell, ${pair.key}: ${usable}/${seeds.length} usable, `
        + `median ${last.medianDeployedVoxels ?? 0} voxel(s) at ${(last.medianVoxelSize ?? 0).toFixed(4)} m, `
        + `reproducibility ${last.medianReproJaccard === null ? "--" : last.medianReproJaccard.toFixed(3)}, `
        + `${last.seconds.toFixed(1)} s`);
    }
  }
  return { runs, sweepRows, pairMeta, seeds, partner };
}

// ---------------------------------------------------------------- calibration

// How wide the reference has to be dilated before it stops being an artefact of
// the sample budget.
//
// The reference is meant to stand for the region both arms can reach, not for
// the region both samplers happened to hit. A radius that is too small measures
// coincidence: doubling the samples then finds a visibly larger region, because
// every new sample opens cells its neighbours had left empty. A radius that is
// large enough closes those gaps, and from there on more samples change
// nothing -- the region is the geometry, and only the radius still inflates it.
//
// So the radius is chosen, not assumed: the same pairs are measured with half
// the seeds and with all of them, and the smallest radius whose volume ratio
// between the two budgets is within `tolerance` of one is the one the
// experiment uses. Everything below it would judge the estimator against the
// sampler; everything above it would pad the reference with space nobody
// reaches.
function calibrate({ cells, seeds, samplesDir, referenceSize, radii, tolerance, outDir }) {
  const descriptors = new Map();
  const robotNames = new Set();
  for (const size of cells) {
    const descriptor = loadDescriptor(path.resolve(REPO_ROOT, descriptorForCell(size)));
    descriptors.set(size, {
      placements: new Map(descriptor.robots.map((robot) => [robot.name, placementOf(robot)])),
      pairs: cellPairs(descriptor.robots.map((robot) => robot.name))
    });
    for (const robot of descriptor.robots) robotNames.add(robot.name);
  }
  const seedDirs = seedDirectories(samplesDir, seeds);
  if (seedDirs.length === 0) throw new Error(`No seed_* directories in ${samplesDir}.`);
  const localClouds = loadLocalClouds(seedDirs, [...robotNames]);
  const allSeeds = [...localClouds.keys()].sort((left, right) => left - right);
  if (allSeeds.length < 2) throw new Error("Calibration compares two sample budgets and needs at least two seeds.");
  const halfSeeds = allSeeds.slice(0, Math.ceil(allSeeds.length / 2));

  const rows = [];
  for (const cell of cells) {
    const { placements, pairs } = descriptors.get(cell);
    const occupancy = new Map();
    for (const [name, placement] of placements) {
      occupancy.set(name, {
        half: poolOccupancy(halfSeeds.map((seed) => placeFlat(localClouds.get(seed).get(name), placement)), referenceSize),
        all: poolOccupancy(allSeeds.map((seed) => placeFlat(localClouds.get(seed).get(name), placement)), referenceSize)
      });
    }
    for (const pair of pairs) {
      const [nameA, nameB] = pair.robots;
      let previous = null;
      for (const radius of radii) {
        const cellsRadius = radius / referenceSize;
        const started = Date.now();
        const half = pooledReference(occupancy.get(nameA).half, occupancy.get(nameB).half, cellsRadius).size;
        const all = pooledReference(occupancy.get(nameA).all, occupancy.get(nameB).all, cellsRadius).size;
        const row = {
          cell,
          pair: pair.key,
          radius,
          radiusCells: cellsRadius,
          cellsHalf: half,
          cellsAll: all,
          volumeHalf: half * referenceSize ** 3,
          volumeAll: all * referenceSize ** 3,
          // > 1 means the region still grows with the sample budget.
          budgetRatio: half === 0 ? null : all / half,
          // What the radius itself adds, against the previous radius.
          radiusGrowth: previous === null || previous === 0 ? null : all / previous,
          seconds: (Date.now() - started) / 1000
        };
        previous = all;
        rows.push(row);
        console.log(`${cell}-robot cell, ${pair.key}, radius ${radius.toFixed(3)} m: `
          + `${all} cell(s) at ${allSeeds.length} seeds, ${half} at ${halfSeeds.length}, `
          + `budget ratio ${row.budgetRatio === null ? "--" : row.budgetRatio.toFixed(3)}, `
          + `${row.seconds.toFixed(1)} s`);
      }
    }
  }

  // Only pairs with a region worth measuring say anything about the radius; a
  // pair that shares nothing is flat at zero whatever the radius.
  const significant = rows.filter((row) => row.cellsAll >= 1000);
  const perRadius = radii.map((radius) => {
    const group = significant.filter((row) => row.radius === radius);
    const ratios = group.map((row) => row.budgetRatio).filter((value) => value !== null);
    return {
      radius,
      pairs: group.length,
      budgetRatioMedian: ratios.length === 0 ? null : summarize(ratios).median,
      budgetRatioMax: ratios.length === 0 ? null : Math.max(...ratios),
      volumeMedian: group.length === 0 ? null : summarize(group.map((row) => row.volumeAll)).median
    };
  });
  const converged = perRadius.find((entry) =>
    entry.budgetRatioMax !== null && entry.budgetRatioMax <= 1 + tolerance);

  if (outDir) {
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(path.join(outDir, "e7_reference_calibration.csv"), toCsv(rows,
      ["cell", "pair", "radius", "radiusCells", "cellsHalf", "cellsAll", "volumeHalf", "volumeAll",
        "budgetRatio", "radiusGrowth", "seconds"]));
    fs.writeFileSync(path.join(outDir, "e7_reference_choice.csv"), toCsv(perRadius,
      ["radius", "pairs", "budgetRatioMedian", "budgetRatioMax", "volumeMedian"]));
  }
  return { rows, perRadius, converged, seeds: allSeeds, halfSeeds, tolerance };
}

// ---------------------------------------------------------------- analyse

const STAT_NAMES = ["median", "q25", "q75", "mean", "sd", "min", "max"];

function summarizeBy(rows, keyOf, metrics, extra = () => ({})) {
  const groups = new Map();
  for (const row of rows) {
    const key = keyOf(row);
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(row);
  }
  const out = [];
  for (const [key, group] of groups) {
    const row = { key, n: group.length, ...extra(group) };
    for (const metric of metrics) {
      const stats = summarize(group.map((entry) => entry[metric]));
      for (const stat of STAT_NAMES) row[`${metric}_${stat}`] = stats[stat];
    }
    out.push(row);
  }
  return out;
}

function methodSummary(runs, target) {
  const metrics = ["voxelSize", "sharedVoxels", "relDeviation", "absDeviation", "sharedVolume",
    "reproJaccard", "pooledCoverage", "pooledPrecision", "samplesPerVoxelA", "samplesPerVoxelB",
    "sizeOverSpacing", "fill", "estimateMs", "voxelizeMs", "totalMs"];
  return summarizeBy(runs, (row) => row.method, metrics, (group) => ({
    method: group[0].method,
    hitsTarget: group.filter((row) => Math.abs(row.sharedVoxels - target) <= 0.25 * target).length,
    empty: group.filter((row) => row.sharedVoxels === 0).length
  })).sort((left, right) => left.method.localeCompare(right.method));
}

function pairSummary(runs) {
  const metrics = ["voxelSize", "sharedVoxels", "relDeviation", "reproJaccard", "pooledCoverage",
    "pooledPrecision", "sizeOverSpacing", "totalMs"];
  return summarizeBy(runs, (row) => `${row.cell}|${row.pair}|${row.method}`, metrics, (group) => ({
    cell: group[0].cell,
    pair: group[0].pair,
    method: group[0].method,
    usable: group.every((row) => row.usable)
  })).sort((left, right) => left.cell - right.cell || left.pair.localeCompare(right.pair) || left.method.localeCompare(right.method));
}

function sweepSummary(rows) {
  const metrics = ["sharedVoxels", "relDeviation", "sharedVolume", "reproJaccard", "sizeOverSpacing",
    "voxelizeMs"];
  return summarizeBy(rows, (row) => row.voxelSize.toFixed(6), metrics, (group) => ({
    voxelSize: group[0].voxelSize
  })).sort((left, right) => left.voxelSize - right.voxelSize);
}

// The accuracy/cost curve of E8, per pair: one row per voxel size, with the
// size the estimator chose on the same axis, so that the figure can mark where
// the deployed rule lands on the curve it is being judged against.
function sweepCurves(rows, chosenByPair = new Map()) {
  const curves = new Map();
  for (const row of rows) {
    const key = `${row.cell}|${row.pair}`;
    if (!curves.has(key)) curves.set(key, []);
    curves.get(key).push(row);
  }
  const out = [];
  for (const [key, group] of curves) {
    const [cell, pair] = key.split("|");
    const bySize = summarizeBy(group, (row) => row.voxelSize.toFixed(6),
      ["reproJaccard", "sharedVoxels", "sizeOverSpacing", "voxelizeMs"],
      (entries) => ({ voxelSize: entries[0].voxelSize }))
      .sort((left, right) => left.voxelSize - right.voxelSize);
    const chosen = chosenByPair.get(key) ?? null;
    // Half a sweep step is the widest a marker may sit from the chosen size
    // and still be the step that represents it.
    const step = bySize.length > 1 ? bySize[1].voxelSize - bySize[0].voxelSize : Infinity;
    for (const size of bySize) {
      out.push({
        cell: Number(cell),
        pair,
        voxelSize: size.voxelSize,
        reproJaccard: size.reproJaccard_median,
        reproQ25: size.reproJaccard_q25,
        reproQ75: size.reproJaccard_q75,
        sharedVoxels: size.sharedVoxels_median,
        sizeOverSpacing: size.sizeOverSpacing_median,
        voxelizeMs: size.voxelizeMs_median,
        chosenByEstimator: chosen,
        isChosen: chosen !== null && Math.abs(size.voxelSize - chosen) <= step / 2
      });
    }
  }
  return out.sort((left, right) => left.cell - right.cell || left.pair.localeCompare(right.pair) || left.voxelSize - right.voxelSize);
}

// Where each method's size sits against the finest size an independent run
// reproduces: 1 means the method found exactly that size, below 1 means it
// chose a grid finer than the samples support, above 1 that it gave away
// resolution it could have kept.
function resolutionRatios(runs) {
  const oracle = new Map();
  for (const run of runs.filter((row) => row.method === "sweep_repro")) {
    oracle.set(runKey(run), run.voxelSize);
  }
  return runs.map((run) => {
    const target = oracle.get(runKey(run));
    return {
      ...run,
      reproducibleSize: target ?? null,
      sizeOverReproducible: target && run.voxelSize ? run.voxelSize / target : null
    };
  });
}

// Left panel of the E7 figure: what each method achieved against what the
// offline sweep achieved for the same pair, one marker per pair and method.
function scatterRows(runs) {
  const oracle = new Map();
  for (const run of runs.filter((row) => row.method === "sweep_target")) {
    oracle.set(runKey(run), run.sharedVoxels);
  }
  return runs
    .filter((run) => run.method !== "sweep_target")
    .map((run) => ({
      method: run.method,
      cell: run.cell,
      pair: run.pair,
      seed: run.seed,
      reference: oracle.get(runKey(run)) ?? null,
      achieved: run.sharedVoxels,
      reproJaccard: run.reproJaccard,
      totalMs: run.totalMs
    }))
    .filter((row) => row.reference !== null);
}

function runKey(run) {
  return `${run.cell}|${run.seed}|${run.pair}`;
}

function texNumber(value, digits = 3) {
  if (value === null || value === undefined || !Number.isFinite(value)) return "--";
  return value.toFixed(digits);
}

const METHOD_LABELS = {
  "fixed_0.05": "Fixed size 0.05\\,m",
  bbox: "Bounding-box estimate",
  proposed: "Proposed estimator",
  sweep_target: "Offline sweep, target count (reference)",
  sweep_repro: "Offline sweep, finest reproducible (reference)"
};

function methodTable({ rows, target, seeds, pairs, reproThreshold }) {
  const order = ["fixed_0.05", "bbox", "proposed", "sweep_target", "sweep_repro"];
  const body = order
    .map((name) => rows.find((row) => row.method === name))
    .filter(Boolean)
    .map((row) => [
      METHOD_LABELS[row.method] || row.method,
      texNumber(row.voxelSize_median, 4),
      texNumber(row.sharedVoxels_median, 0),
      texNumber(row.relDeviation_median, 2),
      texNumber(row.sizeOverSpacing_median, 2),
      texNumber(row.reproJaccard_median, 3),
      texNumber(row.pooledCoverage_median, 3),
      texNumber(row.totalMs_median, 1)
    ].join(" & ") + " \\\\");
  return [
    "% Generated by src/eval/e7VoxelSize.js analyze -- do not edit, rerun instead.",
    "\\begin{table}[htbp]",
    "    \\centering",
    `    \\caption{E7: What each rule for the voxel size produces, as the median over ${pairs} usable pair evaluations `
      + `(${seeds} seed${seeds === 1 ? "" : "s"}). $N$ is the number of shared voxels and its deviation from `
      + `$N_\\mathrm{target} = ${target}$; $s/d_\\mathrm{eff}$ says how far the size lies above the native spacing, `
      + `which is the estimator's own lower bound. $J_\\mathrm{repro}$ is the agreement of the interaction space with the one `
      + `computed from an independent run of the sampler at the same budget, on the same grid: the reference needs no ground truth `
      + `and has no free parameter. Coverage is the share of the interaction space of the nine other runs pooled, i.e.\\ of ten times `
      + `the samples, that one run recovers. The two sweep rows read every size of E8 and are therefore not implementable online; `
      + `the second is the finest size whose space an independent run reproduces with $J_\\mathrm{repro} \\geq ${reproThreshold}$.}`,
    "    \\label{tab:exp_e7}",
    "    \\small",
    "    \\begin{tabular}{@{}lrrrrrrr@{}}",
    "        \\toprule",
    "        \\textbf{Method} & $s$ [m] & $N$ & \\textbf{Rel.\\ dev.} & $s/d_\\mathrm{eff}$ & $J_\\mathrm{repro}$ & \\textbf{Coverage} & \\textbf{Time [ms]} \\\\",
    "        \\midrule",
    ...body.map((line) => `        ${line}`),
    "        \\bottomrule",
    "    \\end{tabular}",
    "\\end{table}",
    ""
  ].join("\n");
}

// Left panel of the E7 figure: what each method achieved against what the
// offline sweep achieved on the same clouds, log-log, with the diagonal as the
// line a method would sit on if it chose the sweep's size.
function scatterFigure({ target, methods }) {
  const series = methods
    .filter((name) => name !== "sweep_target")
    .map((name) => (
      `        \\addplot+[only marks, mark size=1.3pt] table[x=reference, y=achieved, `
      + `discard if not={method}{${name}}] {\\ESevenData/e7_scatter.csv};\n`
      + `        \\addlegendentry{${(METHOD_LABELS[name] || name).replace(/\\,/g, "\\,")}}`
    ));
  return [
    "% Generated by src/eval/e7VoxelSize.js analyze -- do not edit, rerun instead.",
    "% Needs in the preamble: \\usepackage{pgfplots} \\usepackage{pgfplotstable}",
    "% and the discard-if-not style:",
    "% \\pgfplotsset{discard if not/.style 2 args={x filter/.code={\\edef\\tempa{\\thisrow{#1}}\\edef\\tempb{#2}%",
    "%   \\ifx\\tempa\\tempb\\else\\def\\pgfmathresult{inf}\\fi}}}",
    "\\providecommand{\\ESevenData}{e7}",
    "\\pgfplotstableset{col sep=comma}",
    "",
    "\\begin{figure}[htbp]",
    "    \\centering",
    "    \\begin{tikzpicture}",
    "    \\begin{axis}[",
    "        width=0.72\\linewidth, height=0.6\\linewidth,",
    "        xlabel={Shared voxels of the offline sweep}, ylabel={Shared voxels of the method},",
    "        xmode=log, ymode=log,",
    "        grid=major, grid style={gray!25},",
    "        legend cell align=left,",
    "        legend style={font=\\footnotesize, at={(0.02,0.98)}, anchor=north west, draw=none, fill=white, fill opacity=0.8, text opacity=1},",
    "    ]",
    "        \\addplot[gray, dashed, thick, forget plot, domain=10:10000] {x};",
    `        \\addplot[gray, dotted, thick, forget plot, domain=10:10000] {${target}};`,
    ...series,
    "    \\end{axis}",
    "    \\end{tikzpicture}",
    `    \\caption{E7: Shared voxels each method produces against the shared voxels of the offline sweep on the same clouds, `
      + `one marker per usable pair evaluation. The dashed diagonal is the sweep itself, the dotted line the target `
      + `$N_\\mathrm{target} = ${target}$; a marker above the diagonal means the method chose a size that yields more voxels than the `
      + `sweep's best approach to the target.}`,
    "    \\label{fig:exp_e7}",
    "\\end{figure}",
    ""
  ].join("\n");
}

function sweepFigure({ curves, sizes, reproThreshold }) {
  const largest = Math.max(...curves.map((entry) => entry.cell));
  const pairs = [...new Set(curves.filter((row) => row.cell === largest).map((row) => row.pair))];
  const plots = pairs.flatMap((pair) => [
    `        \\addplot+[thick, mark=*, mark size=1.2pt] table[x=voxelSize, y=reproJaccard, `
    + `discard if not={pair}{${pair}}] {\\ESevenData/e8_curves.csv};\n`
    + `        \\addlegendentry{${pair.replace(/_+/g, "--")}}`,
    // Where the deployed rule lands on the curve it is judged against.
    `        \\addplot[forget plot, only marks, mark=o, mark size=3.5pt, black] table[x=voxelSize, y=reproJaccard, `
    + `discard if not={isChosen}{1}, discard if not={pair}{${pair}}] {\\ESevenData/e8_curves.csv};`
  ]);
  return [
    "% Generated by src/eval/e7VoxelSize.js analyze -- do not edit, rerun instead.",
    "% Needs in the preamble: \\usepackage{pgfplots} \\usepackage{pgfplotstable}",
    "% and the discard-if-not style:",
    "% \\pgfplotsset{discard if not/.style 2 args={x filter/.code={\\edef\\tempa{\\thisrow{#1}}\\edef\\tempb{#2}%",
    "%   \\ifx\\tempa\\tempb\\else\\def\\pgfmathresult{inf}\\fi}}}",
    "\\providecommand{\\ESevenData}{e8}",
    "\\pgfplotstableset{col sep=comma}",
    "",
    "\\begin{figure}[htbp]",
    "    \\centering",
    "    \\begin{tikzpicture}",
    "    \\begin{axis}[",
    "        width=0.82\\linewidth, height=0.55\\linewidth,",
    "        xlabel={Voxel size $s$ [m]}, ylabel={Reproducibility $J_\\mathrm{repro}$},",
    `        xmin=${sizes[0]}, xmax=${sizes[sizes.length - 1]}, ymin=0, ymax=1,`,
    "        grid=major, grid style={gray!25},",
    "        legend cell align=left,",
    "        legend style={font=\\footnotesize, at={(1.02,1)}, anchor=north west, draw=none},",
    "    ]",
    `        \\addplot[gray, dashed, thick, forget plot, domain=${sizes[0]}:${sizes[sizes.length - 1]}] {${reproThreshold}};`,
    ...plots,
    "    \\end{axis}",
    "    \\end{tikzpicture}",
    `    \\caption{E8: How much of an interaction space an independent run of the sampler reproduces, over voxel sizes from `
      + `${sizes[0]} to ${sizes[sizes.length - 1]}\\,m (${sizes.length} sizes, step ${(sizes[1] - sizes[0]).toFixed(4)}\\,m), `
      + `per usable pair of the ${largest}-robot cell, median over the seeds. The circle marks the size the estimator chose, the `
      + `dashed line the threshold ${reproThreshold} of Table~\\ref{tab:exp_e7}. A finer grid is not cheaper -- the voxelization time `
      + `is flat in $s$ and is listed in e8\\_curves.csv -- so the price of resolution is paid in reproducibility, not in seconds.}`,
    "    \\label{fig:exp_e7e8}",
    "\\end{figure}",
    ""
  ].join("\n");
}

function analyze(options) {
  const { cells, seeds, samplesDir, e7Dir, e8Dir, sweep, fixed, reproThreshold } = options;
  const descriptor = loadDescriptor(path.resolve(REPO_ROOT, descriptorForCell(Math.max(...cells))));
  const estimatorOptions = descriptor.interactionSpace || {};
  const target = Number(estimatorOptions.targetVoxelCount ?? 1000);
  const minimumSharedVoxels = Number(estimatorOptions.minimumSharedVoxels ?? DEFAULT_MINIMUM_SHARED_VOXELS);
  const sizes = sweepSizes(sweep);

  const seedDirs = seedDirectories(samplesDir, seeds);
  if (seedDirs.length === 0) throw new Error(`No seed_* directories in ${samplesDir}.`);
  const result = runCells({
    cells,
    seedDirs,
    options: estimatorOptions,
    sizes,
    target,
    fixed,
    minimumSharedVoxels,
    reproThreshold
  });

  const runs = resolutionRatios(result.runs);
  const usableRuns = runs.filter((row) => row.usable);
  if (usableRuns.length === 0) throw new Error("No usable pair in the evaluated cells; nothing to summarize.");
  const usablePairs = new Set(usableRuns.map((row) => `${row.cell}|${row.pair}`));
  const usableSweep = result.sweepRows.filter((row) => usablePairs.has(`${row.cell}|${row.pair}`));

  const methods = methodSummary(usableRuns, target);
  const pairs = pairSummary(usableRuns);
  const perSize = sweepSummary(usableSweep);
  const chosenByPair = new Map();
  for (const key of usablePairs) {
    const [cell, pair] = key.split("|");
    const own = usableRuns.filter((row) => row.method === "proposed" && `${row.cell}|${row.pair}` === key);
    chosenByPair.set(key, summarize(own.map((row) => row.voxelSize)).median);
  }
  const curves = sweepCurves(usableSweep, chosenByPair);
  const scatter = scatterRows(usableRuns);

  fs.mkdirSync(e7Dir, { recursive: true });
  fs.mkdirSync(e8Dir, { recursive: true });

  const runColumns = ["cell", "seed", "partnerSeed", "pair", "robotA", "robotB", "usable", "method", "mode",
    "voxelSize", "sharedVoxels", "absDeviation", "relDeviation", "sharedVolume", "partnerVoxels",
    "reproJaccard", "pooledVoxels", "pooledCoverage", "pooledPrecision", "samplesPerVoxelA",
    "samplesPerVoxelB", "sizeOverSpacing", "fill", "reproducibleSize", "sizeOverReproducible",
    "estimateMs", "voxelizeMs", "totalMs", "effectiveSpacing", "deployedVoxels"];
  fs.writeFileSync(path.join(e7Dir, "e7_runs.csv"), toCsv(runs, runColumns));
  fs.writeFileSync(path.join(e7Dir, "e7_methods.csv"), toCsv(methods, Object.keys(methods[0])));
  fs.writeFileSync(path.join(e7Dir, "e7_pairs.csv"), toCsv(pairs, Object.keys(pairs[0])));
  fs.writeFileSync(path.join(e7Dir, "e7_reference.csv"), toCsv(result.pairMeta,
    ["cell", "pair", "robotA", "robotB", "seeds", "usableSeeds", "origin", "medianDeployedVoxels",
      "medianVoxelSize", "medianReproJaccard", "seconds"]));
  fs.writeFileSync(path.join(e7Dir, "e7_scatter.csv"),
    toCsv(scatter, ["method", "cell", "pair", "seed", "reference", "achieved", "reproJaccard", "totalMs"]));
  fs.writeFileSync(path.join(e7Dir, "e7_table.tex"), methodTable({
    rows: methods,
    target,
    seeds: result.seeds.length,
    pairs: usableRuns.filter((row) => row.method === "proposed").length,
    reproThreshold
  }));

  const sweepColumns = ["cell", "seed", "pair", "robotA", "robotB", "voxelSize", "sharedVoxels",
    "relDeviation", "sharedVolume", "partnerVoxels", "reproJaccard", "sizeOverSpacing", "voxelizeMs"];
  fs.writeFileSync(path.join(e8Dir, "e8_sweep.csv"), toCsv(result.sweepRows, sweepColumns));
  fs.writeFileSync(path.join(e8Dir, "e8_summary.csv"), toCsv(perSize, Object.keys(perSize[0])));
  fs.writeFileSync(path.join(e8Dir, "e8_curves.csv"), toCsv(curves,
    ["cell", "pair", "voxelSize", "reproJaccard", "reproQ25", "reproQ75", "sharedVoxels",
      "sizeOverSpacing", "voxelizeMs", "chosenByEstimator", "isChosen"]));
  fs.writeFileSync(path.join(e8Dir, "e8_figure.tex"), sweepFigure({ curves, sizes, reproThreshold }));
  fs.writeFileSync(path.join(e7Dir, "e7_figure.tex"), scatterFigure({
    target,
    methods: [...new Set(scatter.map((row) => row.method))]
  }));

  const setup = {
    cells,
    seeds: result.seeds,
    seedPartners: Object.fromEntries(result.partner),
    samplesDir: path.relative(REPO_ROOT, samplesDir),
    cloudsFrom: "five-robot scene (E1 sampling), reused in every cell",
    targetVoxelCount: target,
    minimumSharedVoxels,
    fixedBaselineSize: fixed,
    sweep: { ...sweep, sizes },
    reproThreshold,
    reference: "the same pair computed from an independent run of the sampler, on the pair's fixed grid "
      + "(J_repro), and from the other runs pooled (coverage). No ground-truth region, no free parameter; "
      + "see e7_reference_calibration.csv for why a fine-grid reference region was rejected.",
    estimatorOptions: { ...estimatorOptions, pairs: undefined },
    finishedAt: new Date().toISOString()
  };
  fs.writeFileSync(path.join(e7Dir, "e7_setup.json"), JSON.stringify(setup, null, 2) + "\n");
  fs.writeFileSync(path.join(e8Dir, "e8_setup.json"), JSON.stringify(setup, null, 2) + "\n");

  return { e7Dir, e8Dir, seeds: result.seeds, runs, usableRuns, methods, pairs, perSize, curves, target, sizes, reproThreshold };
}

// ---------------------------------------------------------------- CLI

function cliArguments(argv) {
  const args = { _: [] };
  for (let index = 0; index < argv.length; index += 1) {
    const token = argv[index];
    if (!token.startsWith("--")) {
      args._.push(token);
      continue;
    }
    const next = argv[index + 1];
    if (next === undefined || next.startsWith("--")) {
      args[token.slice(2)] = true;
    } else {
      args[token.slice(2)] = next;
      index += 1;
    }
  }
  return args;
}

const USAGE = [
  "Usage:",
  "  node src/eval/e7VoxelSize.js analyze   [--cells 2,3,4,5] [--seeds 1-10] [--samples-dir <dir>]",
  "                                         [--sweep 0.01,0.2,0.005] [--repro-threshold 0.75] [--fixed 0.05]",
  "                                         [--out-e7 <dir>] [--out-e8 <dir>]",
  "  node src/eval/e7VoxelSize.js calibrate [--cells 5] [--seeds 1-10] [--radii 0,0.01,...]",
  "                                         [--tolerance 0.05] [--out-e7 <dir>]",
  "",
  "calibrate is the evidence for the reference: it measures how a ground-truth",
  "region built from these clouds moves with the dilation radius and the sample",
  "budget, which is why the analysis compares runs with each other instead.",
  `Defaults: cells ${DEFAULT_CELLS.join(",")}, samples ${DEFAULT_SAMPLES_DIR}, results ${DEFAULT_E7_DIR} and ${DEFAULT_E8_DIR}.`,
  "Offline: it reads the clouds E1 sampled and never talks to CoppeliaSim.",
  "One run covers both experiments -- E8's sweep is where E7's offline oracles come from.",
  "A full run (four cells, ten seeds) takes roughly half an hour; --cells 5 --seeds 1 is the smoke test."
].join("\n");

function parseNumberList(value, what) {
  const values = String(value).split(",").map((token) => Number(token.trim()));
  if (values.length === 0 || values.some((entry) => !Number.isFinite(entry) || entry < 0)) {
    throw new Error(`${what} must be non-negative numbers: ${value}`);
  }
  return [...new Set(values)].sort((left, right) => left - right);
}

function runCalibration(args) {
  const referenceSize = Number(args["reference-size"] || DEFAULT_REFERENCE_SIZE);
  const result = calibrate({
    cells: args.cells ? parseIntegerList(args.cells, "Cell sizes") : [Math.max(...DEFAULT_CELLS)],
    seeds: args.seeds ? parseSeeds(args.seeds) : null,
    samplesDir: path.resolve(REPO_ROOT, args["samples-dir"] || DEFAULT_SAMPLES_DIR),
    referenceSize,
    radii: args.radii ? parseNumberList(args.radii, "Radii") : DEFAULT_CALIBRATION_RADII,
    tolerance: Number(args.tolerance ?? DEFAULT_CALIBRATION_TOLERANCE),
    outDir: path.resolve(REPO_ROOT, args["out-e7"] || DEFAULT_E7_DIR)
  });
  console.log(`\nReference radius, ${result.seeds.length} seeds against ${result.halfSeeds.length}:`);
  for (const entry of result.perRadius) {
    console.log(`  ${entry.radius.toFixed(3)} m: median volume ${entry.volumeMedian === null ? "--" : entry.volumeMedian.toFixed(4)} m^3, `
      + `budget ratio ${entry.budgetRatioMedian === null ? "--" : entry.budgetRatioMedian.toFixed(3)} median, `
      + `${entry.budgetRatioMax === null ? "--" : entry.budgetRatioMax.toFixed(3)} worst`);
  }
  if (result.converged) {
    console.log(`Use --reference-radius ${result.converged.radius}: the first radius at which no pair's region `
      + `grows by more than ${(result.tolerance * 100).toFixed(0)}% when the sample budget doubles.`);
  } else {
    console.log(`No radius in the list holds every pair within ${(result.tolerance * 100).toFixed(0)}%; `
      + "widen --radii before choosing one.");
  }
  return 0;
}

function main(argv) {
  const args = cliArguments(argv);
  if (args._[0] === "calibrate") return runCalibration(args);
  if (args._[0] !== "analyze") {
    console.error(USAGE);
    return 1;
  }
  const reproThreshold = Number(args["repro-threshold"] ?? DEFAULT_REPRO_THRESHOLD);
  if (!(reproThreshold > 0) || reproThreshold > 1) {
    throw new Error(`--repro-threshold must lie in (0,1]: ${args["repro-threshold"]}`);
  }
  const result = analyze({
    cells: args.cells ? parseIntegerList(args.cells, "Cell sizes") : DEFAULT_CELLS,
    seeds: args.seeds ? parseSeeds(args.seeds) : null,
    samplesDir: path.resolve(REPO_ROOT, args["samples-dir"] || DEFAULT_SAMPLES_DIR),
    e7Dir: path.resolve(REPO_ROOT, args["out-e7"] || DEFAULT_E7_DIR),
    e8Dir: path.resolve(REPO_ROOT, args["out-e8"] || DEFAULT_E8_DIR),
    sweep: args.sweep ? parseSweep(args.sweep) : DEFAULT_SWEEP,
    reproThreshold,
    fixed: Number(args.fixed || DEFAULT_FIXED_SIZE)
  });

  const usablePairs = result.usableRuns.filter((row) => row.method === "proposed").length;
  console.log(`\nE7: ${usablePairs} usable pair evaluation(s) over seeds ${result.seeds.join(", ")}, `
    + `target ${result.target} voxels, reproducibility threshold ${result.reproThreshold}.`);
  for (const row of result.methods) {
    console.log(`  ${(METHOD_LABELS[row.method] || row.method).replace(/\\,/g, " ")}: `
      + `s = ${texNumber(row.voxelSize_median, 4)} m (${texNumber(row.sizeOverSpacing_median, 2)}x the native spacing), `
      + `${texNumber(row.sharedVoxels_median, 0)} voxels, rel. deviation ${texNumber(row.relDeviation_median, 2)}, `
      + `J_repro ${texNumber(row.reproJaccard_median, 3)}, coverage ${texNumber(row.pooledCoverage_median, 3)}, `
      + `${texNumber(row.totalMs_median, 1)} ms`);
  }
  const crossing = result.perSize.find((row) => (row.reproJaccard_median ?? 0) >= result.reproThreshold);
  console.log(`E8: ${result.sizes.length} sizes from ${result.sizes[0]} to ${result.sizes[result.sizes.length - 1]} m. `
    + (crossing
      ? `The median interaction space first reaches J_repro >= ${result.reproThreshold} at s = ${crossing.voxelSize} m `
        + `(${texNumber(crossing.sharedVoxels_median, 0)} voxels, ${texNumber(crossing.voxelizeMs_median, 1)} ms).`
      : `No size in the sweep reaches J_repro >= ${result.reproThreshold}; widen --sweep.`));
  console.log(`Tables and figures: ${result.e7Dir} and ${result.e8Dir}`);
  return 0;
}

if (require.main === module) {
  try {
    process.exit(main(process.argv.slice(2)));
  } catch (error) {
    console.error(error);
    process.exit(1);
  }
}

module.exports = {
  agreementAt,
  analyze,
  bounds,
  boundingBoxSize,
  boxIntersection,
  calibrate,
  cellPairs,
  decodeReference,
  dilationOffsets,
  evaluateRun,
  fixedSize,
  gridOrigin,
  jaccardOf,
  methodSummary,
  methodTable,
  occupiedKeys,
  parseSweep,
  partnerSeeds,
  poolOccupancy,
  pooledReference,
  pointsInBox,
  proposedSize,
  referenceCode,
  relativeDeviation,
  resolutionDiagnostics,
  resolutionRatios,
  scatterFigure,
  scatterRows,
  sharedKeysAt,
  sweepCurves,
  sweepFigure,
  sweepPair,
  sweepSizes,
  sweepSummary,
  trimToOverlap
};
