"use strict";

const fs = require("fs");
const path = require("path");
const crypto = require("crypto");
const {
  orientationRadians,
  rotateLocal,
  rotateWorldToLocal,
  worldVectorToLocal,
  toolAxisWorld,
  cellBaselineAxis,
  giverPresentAxis,
  normalizeApproachMode,
  receiverApproachAxis,
  shoulderMargin
} = require("./handoverGeometry");
const { canonicalPlacement } = require("./sharedCell");

const AXES = ["x", "y", "z"];

// How many shared voxels a pair needs before it counts as a rendezvous. Twenty
// is a shared region, not a coincidence of two clouds grazing each other: the
// cell's usable pairs sit at 47, 128 and 345, and the pair this floor exists to
// exclude sat at 1.
const DEFAULT_MINIMUM_SHARED_VOXELS = 20;
const EPSILON = 1e-12;
const INTERACTION_ALGORITHM_VERSION = "pair-cache-v5-order-independent";
// Kept in step with the pipeline's shared-space palette so a cloud redrawn at
// runtime keeps the colour the scene already used for that pair.
const VISUAL_PALETTE = [[0, 255, 0], [0, 210, 255], [255, 210, 0], [210, 0, 255]];

// Beyond the shipped palette, colours are generated rather than repeated. Four
// entries covered a three-robot cell with two pairs; a four-robot cell has six,
// and two clouds drawn in the same colour in the same scene cannot be told
// apart. Hues are spread by the golden angle so consecutive indices land far
// apart on the wheel instead of drifting.
function generatedColor(index) {
  const hue = (index * 137.508) % 360;
  const sector = hue / 60;
  const x = Math.round(255 * (1 - Math.abs((sector % 2) - 1)));
  const table = [[255, x, 0], [x, 255, 0], [0, 255, x], [0, x, 255], [x, 0, 255], [255, 0, x]];
  return table[Math.floor(sector) % 6];
}

function paletteColor(index) {
  return index < VISUAL_PALETTE.length ? VISUAL_PALETTE[index] : generatedColor(index);
}

function compareRobotIdentity(left, right) {
  return String(left).localeCompare(String(right), undefined, { numeric: true, sensitivity: "base" });
}

function canonicalPairRobots(first, second) {
  return compareRobotIdentity(first, second) <= 0 ? [first, second] : [second, first];
}

function canonicalPairKey(first, second) {
  return canonicalPairRobots(first, second).join("__");
}

// A pair entry for two robots nobody configured as neighbours. Same canonical
// key, same output layout and same deterministic owner as a configured pair, so
// a scanned pair and a configured one are interchangeable everywhere below.
function synthesizePair(first, second) {
  const robots = canonicalPairRobots(first, second);
  const key = robots.join("__");
  return {
    key,
    robots,
    ownerRobot: robots[0],
    discovery: "pair_scan",
    output: {
      csv: `generated/robots/interactions/${key}/space.csv`,
      metadata: `generated/robots/interactions/${key}/space.json`
    }
  };
}

function pairContains(pair, first, second) {
  if (!Array.isArray(pair?.robots) || pair.robots.length !== 2) return false;
  return canonicalPairKey(pair.robots[0], pair.robots[1]) === canonicalPairKey(first, second);
}

function distanceSquared(a, b) {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

class KdTree3D {
  constructor(points) {
    if (!Array.isArray(points) || points.length === 0) {
      throw new Error("KD-tree requires at least one point");
    }
    this.root = KdTree3D.build([...points], 0);
  }

  static build(points, depth) {
    if (points.length === 0) return null;
    const axis = depth % 3;
    const key = AXES[axis];
    points.sort((a, b) => a[key] - b[key]);
    const middle = Math.floor(points.length / 2);
    return {
      point: points[middle],
      axis,
      left: KdTree3D.build(points.slice(0, middle), depth + 1),
      right: KdTree3D.build(points.slice(middle + 1), depth + 1)
    };
  }

  nearestDistance(point, excludeSameReference = false) {
    let bestSquared = Number.POSITIVE_INFINITY;
    const search = (node) => {
      if (!node) return;
      const candidateSquared = distanceSquared(point, node.point);
      if (
        (!excludeSameReference || node.point !== point) &&
        (!excludeSameReference || candidateSquared > EPSILON) &&
        candidateSquared < bestSquared
      ) {
        bestSquared = candidateSquared;
      }
      const key = AXES[node.axis];
      const difference = point[key] - node.point[key];
      const near = difference <= 0 ? node.left : node.right;
      const far = difference <= 0 ? node.right : node.left;
      search(near);
      if (difference * difference < bestSquared) search(far);
    };
    search(this.root);
    return Math.sqrt(bestSquared);
  }

  hasPointWithin(point, radius) {
    const radiusSquared = radius * radius;
    let found = false;
    const search = (node) => {
      if (!node || found) return;
      if (distanceSquared(point, node.point) <= radiusSquared) {
        found = true;
        return;
      }
      const key = AXES[node.axis];
      const difference = point[key] - node.point[key];
      search(difference <= 0 ? node.left : node.right);
      if (difference * difference <= radiusSquared) {
        search(difference <= 0 ? node.right : node.left);
      }
    };
    search(this.root);
    return found;
  }
}

function seededRandom(seedValue) {
  let state = (Number(seedValue) >>> 0) || 1;
  return () => {
    state = (1664525 * state + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

function deterministicSample(values, maxCount, seed = 1) {
  if (values.length <= maxCount) return [...values];
  const indexes = values.map((_, index) => index);
  const random = seededRandom(seed);
  for (let index = indexes.length - 1; index > 0; index -= 1) {
    const other = Math.floor(random() * (index + 1));
    [indexes[index], indexes[other]] = [indexes[other], indexes[index]];
  }
  return indexes.slice(0, maxCount).map((index) => values[index]);
}

function percentile(values, quantile) {
  if (values.length === 0) throw new Error("Cannot calculate a percentile from an empty array");
  if (!Number.isFinite(quantile) || quantile < 0 || quantile > 1) {
    throw new Error("spacingPercentile must be between 0 and 1");
  }
  const sorted = [...values].sort((a, b) => a - b);
  const position = (sorted.length - 1) * quantile;
  const low = Math.floor(position);
  const high = Math.ceil(position);
  if (low === high) return sorted[low];
  const fraction = position - low;
  return sorted[low] * (1 - fraction) + sorted[high] * fraction;
}

function uniquePoints(points) {
  const seen = new Set();
  return points.filter((point) => {
    const key = `${point.x},${point.y},${point.z}`;
    if (seen.has(key)) return false;
    seen.add(key);
    return true;
  });
}

function estimateSpacing(points, sampleSize = 20000, quantile = 0.75, seed = 1) {
  const distinct = uniquePoints(points);
  if (distinct.length < 2) throw new Error("At least two distinct reachable points are required");
  const tree = new KdTree3D(distinct);
  const distances = deterministicSample(distinct, sampleSize, seed)
    .map((point) => tree.nearestDistance(point, true))
    .filter(Number.isFinite);
  if (distances.length === 0) throw new Error("Could not estimate non-zero point spacing");
  return percentile(distances, quantile);
}

function bounds(points) {
  if (points.length === 0) throw new Error("Cannot calculate bounds for an empty workspace");
  const result = {
    min: { x: Number.POSITIVE_INFINITY, y: Number.POSITIVE_INFINITY, z: Number.POSITIVE_INFINITY },
    max: { x: Number.NEGATIVE_INFINITY, y: Number.NEGATIVE_INFINITY, z: Number.NEGATIVE_INFINITY }
  };
  for (const point of points) {
    for (const axis of AXES) {
      result.min[axis] = Math.min(result.min[axis], point[axis]);
      result.max[axis] = Math.max(result.max[axis], point[axis]);
    }
  }
  return result;
}

function bboxOverlapVolume(left, right) {
  return AXES.reduce((volume, axis) => (
    volume * Math.max(0, Math.min(left.max[axis], right.max[axis]) - Math.max(left.min[axis], right.min[axis]))
  ), 1);
}

// The tree over the TARGET is the caller's, not this function's. Both
// directions of a bidirectional estimate index the same two clouds, and the
// empty-intersection path wants one of them again to say how far apart the
// clouds really are -- three builds of the same two trees for one verdict.
function estimateCloseCount(source, targetTree, targetCount, radius, sampleSize, seed) {
  const sample = deterministicSample(source, sampleSize, seed);
  const close = sample.reduce((count, point) => count + (targetTree.hasPointWithin(point, radius) ? 1 : 0), 0);
  return (close / sample.length) * source.length;
}

function pointToVoxel(point, origin, voxelSize) {
  return AXES.map((axis) => Math.floor((point[axis] - origin[axis]) / voxelSize));
}

function voxelKey(index) {
  return index.join(",");
}

function compareVoxelKeys(left, right) {
  const a = left.split(",").map(Number);
  const b = right.split(",").map(Number);
  return a[0] - b[0] || a[1] - b[1] || a[2] - b[2];
}

function estimateInteractionSpace(robot1, robot2, options = {}) {
  const first = uniquePoints(robot1);
  const second = uniquePoints(robot2);
  if (first.length < 2 || second.length < 2) {
    return { success: false, reason: "insufficient_reachable_points" };
  }
  const targetVoxelCount = Number(options.targetVoxelCount ?? 1000);
  const proximityFactor = Number(options.proximityFactor ?? 1.25);
  const sampleSize = Math.max(2, Number(options.sampleSize ?? 20000));
  const spacingPercentile = Number(options.spacingPercentile ?? 0.75);
  const minVoxelSize = Number(options.minVoxelSize ?? 0);
  const maxVoxelSize = Number(options.maxVoxelSize ?? Number.POSITIVE_INFINITY);
  const seed = Number(options.seed ?? 42);
  const bidirectional = options.bidirectional !== false;
  if (!(targetVoxelCount > 0) || !(proximityFactor > 0) || minVoxelSize < 0 || !(maxVoxelSize > 0)) {
    throw new Error("Invalid interaction-space estimator configuration");
  }

  const firstBounds = bounds(first);
  const secondBounds = bounds(second);
  const overlapVolume = bboxOverlapVolume(firstBounds, secondBounds);
  if (overlapVolume <= 0) return { success: false, reason: "no_geometric_overlap" };

  // Every seed here is the SAME seed, and that is the whole point. Derived
  // seeds (seed+1, seed+2, seed+3) made the estimate depend on which cloud was
  // passed first: the pair Robot1/Robot2 came out as 358 voxels at 0.04879 one
  // way round and 360 at 0.04887 the other. A shared space that changes
  // depending on which of the two robots asked for it is not shared, and it is
  // what stopped any agent but the configured owner from computing its own.
  // With one seed the subsample of a cloud depends only on that cloud, so
  // swapping the arguments swaps the outputs and changes nothing else.
  const spacingRobot1 = estimateSpacing(first, sampleSize, spacingPercentile, seed);
  const spacingRobot2 = estimateSpacing(second, sampleSize, spacingPercentile, seed);
  const effectiveSpacing = Math.max(spacingRobot1, spacingRobot2);
  const lowerBound = Math.max(effectiveSpacing, minVoxelSize);
  if (maxVoxelSize < lowerBound) {
    throw new Error("maxVoxelSize is smaller than the native-spacing/minimum lower bound");
  }
  const proximityRadius = proximityFactor * effectiveSpacing;
  const firstTree = new KdTree3D(first);
  const secondTree = new KdTree3D(second);
  const close12 = estimateCloseCount(first, secondTree, second.length, proximityRadius, sampleSize, seed + 1);
  let estimatedClosePoints = close12;
  if (bidirectional) {
    const close21 = estimateCloseCount(second, firstTree, first.length, proximityRadius, sampleSize, seed + 1);
    // Symmetric by construction: min() over both normalised directions.
    estimatedClosePoints = Math.min(close12 / first.length, close21 / second.length) * Math.min(first.length, second.length);
  }

  let estimatorMode = "kd_tree_proximity";
  let estimatedVoxelSize;
  if (estimatedClosePoints > 0) {
    estimatedVoxelSize = effectiveSpacing * Math.cbrt(Math.max(estimatedClosePoints / targetVoxelCount, 1));
  } else {
    estimatorMode = "bounding_box_fallback";
    estimatedVoxelSize = Math.max(effectiveSpacing, Math.cbrt(overlapVolume / targetVoxelCount));
  }
  const voxelSize = Math.min(Math.max(estimatedVoxelSize, lowerBound), maxVoxelSize);
  const origin = {
    x: Math.min(firstBounds.min.x, secondBounds.min.x),
    y: Math.min(firstBounds.min.y, secondBounds.min.y),
    z: Math.min(firstBounds.min.z, secondBounds.min.z)
  };
  const firstOccupied = new Set(first.map((point) => voxelKey(pointToVoxel(point, origin, voxelSize))));
  const secondOccupied = new Set(second.map((point) => voxelKey(pointToVoxel(point, origin, voxelSize))));
  const intersectionKeys = [...(firstOccupied.size <= secondOccupied.size ? firstOccupied : secondOccupied)]
    .filter((key) => (firstOccupied.size <= secondOccupied.size ? secondOccupied : firstOccupied).has(key))
    .sort(compareVoxelKeys);
  const voxels = intersectionKeys.map((key) => {
    const index = key.split(",").map(Number);
    return {
      index,
      center: {
        x: origin.x + (index[0] + 0.5) * voxelSize,
        y: origin.y + (index[1] + 0.5) * voxelSize,
        z: origin.z + (index[2] + 0.5) * voxelSize
      }
    };
  });
  if (voxels.length === 0) {
    // How far apart the two clouds actually are, said in metres.
    //
    // "voxel_intersection_empty" on its own reads like a defect in the
    // estimator, and it is usually a fact about the cell: two arms whose reach
    // spheres overlap by half a metre can still have sampled workspaces that
    // never come closer than a few centimetres, because a sphere is not the
    // shape of a workspace. Live cell: Robot1 and Robot3 pass the sphere test
    // with 0.63m to spare and their clouds' nearest approach is 0.029m -- under
    // one voxel, so no cell of the grid holds a point from both. Measured only
    // on this path, where the pair has already been refused and the cost of one
    // exact nearest-neighbour sweep buys the difference between a verdict and a
    // mystery.
    const [source, targetTree] = first.length <= second.length ? [first, secondTree] : [second, firstTree];
    const nearestApproach = source.reduce(
      (best, point) => Math.min(best, targetTree.nearestDistance(point)),
      Number.POSITIVE_INFINITY
    );
    return {
      success: false,
      reason: "voxel_intersection_empty",
      voxelSize,
      origin,
      estimatorMode,
      estimatedClosePoints,
      nearestApproach
    };
  }
  return {
    success: true,
    voxelSize,
    origin,
    voxels,
    diagnostics: {
      // A one-directional estimate reads only the first cloud, so it is the one
      // configuration in which two robots can still disagree about their shared
      // space. Recorded rather than silently assumed.
      orderIndependent: bidirectional,
      spacingRobot1,
      spacingRobot2,
      effectiveSpacing,
      proximityRadius,
      estimatedClosePoints,
      estimatedVoxelSize,
      targetVoxelCount,
      estimatorMode,
      reachablePointCounts: [first.length, second.length],
      occupiedVoxelCounts: [firstOccupied.size, secondOccupied.size]
    }
  };
}

// One cloud, one KD-tree, one digest per file per process.
//
// Three agents in one process previously loaded and re-sorted the same 35k
// point CSV once per pair they took part in, and hashed the same 1.5 MB file
// again for every version check. Nothing here is mutated after construction --
// KdTree3D copies before it sorts, and every consumer maps rather than writes --
// so the same objects are safely handed to every controller that asks.
//
// Keyed on size and mtime so a regenerated workspace is picked up rather than
// served stale from the cache.
const fileCache = new Map();

function fileCacheEntry(filePath) {
  const resolved = path.resolve(filePath);
  const stats = fs.statSync(resolved);
  const key = `${stats.size}:${stats.mtimeMs}`;
  const existing = fileCache.get(resolved);
  if (existing && existing.key === key) return existing;
  const entry = { key, points: null, tree: null, digest: null };
  fileCache.set(resolved, entry);
  return entry;
}

function loadReachableCsvShared(csvPath) {
  const entry = fileCacheEntry(csvPath);
  if (!entry.points) entry.points = loadReachableCsv(csvPath);
  return entry.points;
}

function reachableTreeShared(csvPath) {
  const entry = fileCacheEntry(csvPath);
  if (!entry.tree) entry.tree = new KdTree3D(loadReachableCsvShared(csvPath));
  return entry.tree;
}

function reachableDigestShared(csvPath) {
  const entry = fileCacheEntry(csvPath);
  if (!entry.digest) {
    entry.digest = crypto.createHash("sha256").update(fs.readFileSync(path.resolve(csvPath))).digest("hex");
  }
  return entry.digest;
}

// Two agents may compute the same pair at the same moment and export it to the
// same two files. A plain writeFileSync lets the second one land inside the
// first one's CSV, and the reader that follows sees a truncated cloud it has no
// way to recognise as damaged. Rename is atomic within a filesystem, so a
// reader sees either the old file or the whole new one, never a mixture.
function writeFileAtomic(filePath, contents) {
  const directory = path.dirname(filePath);
  fs.mkdirSync(directory, { recursive: true });
  const temporary = path.join(
    directory,
    `.${path.basename(filePath)}.${process.pid}.${crypto.randomBytes(4).toString("hex")}.tmp`
  );
  fs.writeFileSync(temporary, contents);
  try {
    fs.renameSync(temporary, filePath);
  } catch (error) {
    fs.rmSync(temporary, { force: true });
    throw error;
  }
}

function loadReachableCsv(csvPath) {
  const text = fs.readFileSync(csvPath, "utf8").trim();
  if (!text) return [];
  return text.split(/\r?\n/).slice(1).map((line) => line.split(",").map(Number))
    .filter((row) => row.length >= 4 && row.slice(0, 4).every(Number.isFinite) && row[3] === 1)
    .map(([x, y, z]) => ({ x, y, z }));
}

function toWorld(point, robot) {
  const offset = robot.offset || [0, 0, 0];
  const rotated = rotateLocal(point, orientationRadians(robot));
  return {
    x: Number(offset[0] || 0) + rotated.x,
    y: Number(offset[1] || 0) + rotated.y,
    z: Number(offset[2] || 0) + rotated.z
  };
}

function targetForRobot(cubeWorld, robot, manifest) {
  // The robot's tool control point IS the midpoint of its gripper jaws, so the
  // shared cube point is the target directly. There is no jaw offset to
  // subtract any more -- that offset, and the assumption about wrist pose it
  // silently carried, was the source of most of the handover's misalignment.
  const orientation = orientationRadians(robot);
  const offset = robot.offset || [0, 0, 0];
  const worldTcp = { x: cubeWorld.x, y: cubeWorld.y, z: cubeWorld.z };
  const localTcp = rotateWorldToLocal({
    x: worldTcp.x - Number(offset[0] || 0),
    y: worldTcp.y - Number(offset[1] || 0),
    z: worldTcp.z - Number(offset[2] || 0)
  }, orientation);
  return {
    robot: robot.name,
    worldTcp,
    localTcp
  };
}

// The layout a space was computed for, recorded so an exported artifact says
// which arrangement of the cell it describes rather than only when it was made.
function placementOf(robot) {
  return {
    offset: (robot.offset || [0, 0, 0]).map(Number),
    orientationDeg: Array.isArray(robot.orientationDeg)
      ? robot.orientationDeg.map(Number)
      : [0, 0, Number(robot.rotationDeg || 0)]
  };
}

function shuffled(values, seed) {
  const result = [...values];
  const random = seededRandom(seed);
  for (let index = result.length - 1; index > 0; index -= 1) {
    const other = Math.floor(random() * (index + 1));
    [result[index], result[other]] = [result[other], result[index]];
  }
  return result;
}

// The candidate's name, derived from its position in the core array and nothing
// else. It has to stay stable across re-ranks: the agent strikes a candidate BY
// NAME, so a name that shifted when the set changed would strike a different
// voxel than the one the cell refused.
function candidateId(index) {
  return `candidate_${index + 1}`;
}

class InteractionSpaceController {
  constructor(ctx) {
    this.ctx = ctx;
    this.spaces = new Map();
    this.rankedCandidates = new Map();
    // Candidates the cell has refused, per partner. Kept here because DENSITY
    // is measured against them: see rank(). The agent keeps its own copy as
    // handoverCandidateRejected beliefs, which is what actually removes a
    // candidate from the running -- this set only changes what the geometry
    // capability reports about the ones still in it.
    this.rejectedCandidates = new Map();
    this.nextPlanId = 1;
    this.descriptor = null;
    const relative = ctx.manifest.sharedEnvironment?.descriptor;
    if (relative) {
      const descriptorPath = path.resolve(ctx.repoRoot, relative);
      if (fs.existsSync(descriptorPath)) {
        this.descriptorPath = descriptorPath;
        this.descriptor = JSON.parse(fs.readFileSync(descriptorPath, "utf8"));
      }
    }
  }

  get configuration() {
    return this.descriptor?.interactionSpace || null;
  }

  get isOwner() {
    return this.ownedPartners.length > 0;
  }

  get pairs() {
    if (Array.isArray(this.configuration?.pairs)) return this.configuration.pairs;
    const ownerRobot = this.configuration?.ownerRobot;
    return Array.isArray(this.configuration?.partners)
      ? this.configuration.partners.map((partner) => ({ robots: [ownerRobot, partner], ownerRobot }))
      : [];
  }

  get ownedPartners() {
    return this.pairs
      .filter((pair) => pair.ownerRobot === this.ctx.manifest.name)
      .map((pair) => pair.robots.find((robot) => robot !== this.ctx.manifest.name))
      .filter(Boolean);
  }

  pairFor(targetRobot, options = {}) {
    const configured = this.pairs.find((pair) => pairContains(pair, this.ctx.manifest.name, targetRobot));
    if (configured || !options.synthesize) return configured || null;
    // A pair nobody configured still has a geometry, and refusing to compute it
    // is how Robot1 and Robot3 came to be treated as if they could never reach
    // the same point. They share exactly one voxel -- small, but the difference
    // between "one voxel" and "no such pair" is the difference between a cell
    // that knows where its arms can meet and one that assumes.
    return synthesizePair(this.ctx.manifest.name, targetRobot);
  }

  robot(name) {
    return this.descriptor?.robots?.find((robot) => robot.name === name);
  }

  // The descriptor records where the pipeline PUT each robot, which stops being
  // true the moment somebody drags a base in the scene or serves the cell with
  // different offsets. A placement read back from the running simulator wins
  // over it, and because the placement is part of the version hash, a robot
  // that has moved invalidates its own cached space instead of quietly reusing
  // the layout it was generated for.
  placedRobot(name, placements) {
    const robot = this.robot(name);
    const live = placements?.[name];
    if (!robot || !live || !Array.isArray(live.offset)) return robot;
    const orientationDeg = Array.isArray(live.orientationDeg)
      ? live.orientationDeg.map(Number)
      : [0, 0, Number(live.rotationDeg || 0)];
    return {
      ...robot,
      offset: live.offset.map(Number),
      orientationDeg,
      rotationDeg: Number(live.rotationDeg ?? orientationDeg[2]),
      placementSource: "scene"
    };
  }

  // What the lower layer needs to redraw this pair's cloud in the live scene.
  visualFor(targetRobot) {
    const space = this.spaces.get(targetRobot);
    if (!space) return null;
    const output = this.outputFor(space.pair);
    if (!output.csv) return null;
    const recorded = this.descriptor?.sharedSpaces?.find((entry) => entry.pairKey === space.pairKey);
    // Ordered over the whole cell, not over this agent's own pairs: two agents
    // indexing their own lists both start at zero and draw two different pairs
    // in the same colour.
    const allPairs = (this.descriptor?.sharedSpaces || []).map((entry) => entry.pairKey).sort();
    const index = Math.max(0, allPairs.indexOf(space.pairKey));
    return {
      pairKey: space.pairKey,
      csv: output.csv,
      voxelSize: space.voxelSize,
      color: recorded?.color || paletteColor(index)
    };
  }

  manifestFor(robot) {
    const relative = robot.manifest || `generated/robots/${robot.name}/manifest.json`;
    return JSON.parse(fs.readFileSync(path.resolve(this.ctx.repoRoot, relative), "utf8"));
  }

  // Any robot in the pair may compute the pair.
  //
  // Ownership is a PROTOCOL rule -- who is allowed to open a handover session --
  // and it stays enforced where handovers are opened. It was never a geometric
  // rule, and using it as one meant Robot2 could not answer a question about its
  // own shared space with Robot1 without asking Robot1 to answer it first. Now
  // the version hash, the estimator and the exported files are all keyed on the
  // canonical pair rather than on the caller, so whoever computes it computes
  // the same space, byte for byte, and the second one to ask hits the cache.
  build(targetRobot, options = {}) {
    const pair = this.pairFor(targetRobot, { synthesize: options.synthesizePair === true });
    if (!pair) return { success: false, reason: "robots_are_not_neighbors", targetRobot };
    if (targetRobot === this.ctx.manifest.name) {
      return { success: false, reason: "source_and_target_are_equal", targetRobot };
    }
    const placements = options.placements || null;
    const owner = this.placedRobot(this.ctx.manifest.name, placements);
    const partner = this.placedRobot(targetRobot, placements);
    if (!owner || !partner) {
      return { success: false, reason: "unknown_interaction_robot", targetRobot };
    }
    const ownerManifest = this.manifestFor(owner);
    const partnerManifest = this.manifestFor(partner);
    const ownerCsv = path.resolve(this.ctx.repoRoot, owner.workspaceCsv || ownerManifest.generatedFiles.robotDataPoints);
    const partnerCsv = path.resolve(this.ctx.repoRoot, partner.workspaceCsv || partnerManifest.generatedFiles.robotDataPoints);
    const localOwner = loadReachableCsvShared(ownerCsv);
    const localPartner = loadReachableCsvShared(partnerCsv);
    // Canonical pair order, not caller order. Robot2 asking about Robot1 must
    // hash what Robot1 asking about Robot2 hashes, or the two of them cache-miss
    // against each other forever and rewrite the same files in a loop.
    const canonicalNames = canonicalPairRobots(owner.name, partner.name);
    const sideOf = (name) => (name === owner.name
      ? { csv: ownerCsv, placement: owner }
      : { csv: partnerCsv, placement: partner });
    const canonicalSides = canonicalNames.map(sideOf);
    const version = crypto.createHash("sha256")
      .update(INTERACTION_ALGORITHM_VERSION)
      .update(reachableDigestShared(canonicalSides[0].csv))
      .update(reachableDigestShared(canonicalSides[1].csv))
      .update(JSON.stringify({
        pair: canonicalNames,
        // Keyed on the rotation itself rather than on a set of Euler angles
        // naming it, and quantized. Two spellings of one pose -- a bare yaw in
        // the descriptor against the full triple the robot reports, or any of
        // the infinitely many triples that name a gimbal-locked mounting like
        // Robot3's -- must hash alike, or the cache misses on every read and
        // recomputes a space that never changed.
        transforms: canonicalSides.map((side) => canonicalPlacement(side.placement)),
        configuration: {
          targetVoxelCount: this.configuration.targetVoxelCount,
          proximityFactor: this.configuration.proximityFactor,
          sampleSize: this.configuration.sampleSize,
          spacingPercentile: this.configuration.spacingPercentile,
          minVoxelSize: this.configuration.minVoxelSize,
          maxVoxelSize: this.configuration.maxVoxelSize,
          bidirectional: this.configuration.bidirectional,
          seed: this.configuration.seed
        }
      }))
      .digest("hex");
    const existing = this.spaces.get(targetRobot);
    if (existing?.version === version) {
      return this.summary(existing, true);
    }

    const cached = this.loadCached(pair, {
      version, targetRobot, owner, partner, ownerManifest, partnerManifest, localOwner, localPartner,
      ownerTree: reachableTreeShared(ownerCsv),
      partnerTree: reachableTreeShared(partnerCsv)
    });
    if (cached) {
      this.spaces.set(targetRobot, cached);
      return this.summary(cached, true);
    }

    // Fed in canonical order too. The estimator is symmetric, so this does not
    // change the voxels -- it makes the diagnostics it reports name the same
    // robot whoever calls, which is what keeps the exported metadata identical
    // rather than merely equivalent.
    const canonicalClouds = canonicalSides.map(
      (side) => loadReachableCsvShared(side.csv).map((point) => toWorld(point, side.placement))
    );
    const result = estimateInteractionSpace(canonicalClouds[0], canonicalClouds[1], this.configuration);
    if (!result.success) return { ...result, targetRobot };
    const id = `${canonicalPairKey(this.ctx.manifest.name, targetRobot)}_${version.slice(0, 12)}`.replace(/[^A-Za-z0-9_]/g, "_");
    const space = {
      id,
      version,
      pairKey: canonicalPairKey(this.ctx.manifest.name, targetRobot),
      pair,
      targetRobot,
      owner,
      partner,
      ownerManifest,
      partnerManifest,
      localOwner,
      localPartner,
      ownerTree: reachableTreeShared(ownerCsv),
      partnerTree: reachableTreeShared(partnerCsv),
      ownerBounds: bounds(localOwner),
      partnerBounds: bounds(localPartner),
      placementSource: owner.placementSource || partner.placementSource ? "scene" : "descriptor",
      ...result
    };
    this.spaces.set(targetRobot, space);
    this.export(space);
    return this.summary(space, false);
  }

  summary(space, cached) {
    return {
      success: true,
      id: space.id,
      version: space.version,
      pairKey: space.pairKey,
      ownerRobot: this.ctx.manifest.name,
      targetRobot: space.targetRobot,
      voxelSize: space.voxelSize,
      origin: space.origin,
      voxelCount: space.voxels.length,
      diagnostics: space.diagnostics,
      placementSource: space.placementSource || "descriptor",
      placements: {
        [this.ctx.manifest.name]: placementOf(space.owner),
        [space.targetRobot]: placementOf(space.partner)
      },
      cached
    };
  }

  outputFor(pair) {
    return pair?.output || this.configuration.output || {};
  }

  loadCached(pair, details) {
    const output = this.outputFor(pair);
    if (!output.csv || !output.metadata) return null;
    const csvPath = path.resolve(this.ctx.repoRoot, output.csv);
    const metadataPath = path.resolve(this.ctx.repoRoot, output.metadata);
    if (!fs.existsSync(csvPath) || !fs.existsSync(metadataPath)) return null;
    let metadata;
    try {
      metadata = JSON.parse(fs.readFileSync(metadataPath, "utf8"));
    } catch (error) {
      return null;
    }
    if (metadata.version !== details.version || metadata.pairKey !== canonicalPairKey(this.ctx.manifest.name, details.targetRobot)) {
      return null;
    }
    const voxels = fs.readFileSync(csvPath, "utf8").trim().split(/\r?\n/).slice(1)
      .map((line) => line.split(",").slice(0, 3).map(Number))
      .filter((row) => row.length === 3 && row.every(Number.isFinite))
      .map(([x, y, z]) => ({
        center: { x, y, z },
        index: pointToVoxel({ x, y, z }, metadata.origin, metadata.voxelSize)
      }));
    if (voxels.length === 0) return null;
    // A CSV that does not hold the number of voxels its metadata claims is a
    // half-written or half-read file, not a cache hit. Recomputing costs a few
    // seconds; trusting it costs a shared space with holes in it.
    if (Number.isFinite(Number(metadata.sharedVoxelCount))
      && Number(metadata.sharedVoxelCount) !== voxels.length) {
      return null;
    }
    return {
      ...details,
      id: metadata.id,
      pairKey: metadata.pairKey,
      pair,
      placementSource: details.owner.placementSource || details.partner.placementSource ? "scene" : "descriptor",
      voxelSize: metadata.voxelSize,
      origin: metadata.origin,
      voxels,
      diagnostics: metadata.diagnostics || {},
      ownerTree: details.ownerTree,
      partnerTree: details.partnerTree,
      ownerBounds: bounds(details.localOwner),
      partnerBounds: bounds(details.localPartner)
    };
  }

  export(space) {
    const output = this.outputFor(space.pair);
    if (!output.csv || !output.metadata) return;
    const csvPath = path.resolve(this.ctx.repoRoot, output.csv);
    const metadataPath = path.resolve(this.ctx.repoRoot, output.metadata);
    writeFileAtomic(
      csvPath,
      ["X,Y,Z,RobotCount", ...space.voxels.map(({ center }) => `${center.x},${center.y},${center.z},2`)].join("\n") + "\n"
    );
    writeFileAtomic(metadataPath, JSON.stringify({
      id: space.id,
      pairKey: space.pairKey,
      version: space.version,
      // The pair's configured owner -- a protocol role, and the same value no
      // matter who ran the computation. computedBy records who actually did.
      ownerRobot: space.pair?.ownerRobot || canonicalPairRobots(this.ctx.manifest.name, space.targetRobot)[0],
      computedBy: this.ctx.manifest.name,
      robots: canonicalPairRobots(this.ctx.manifest.name, space.targetRobot),
      interactionRobot: space.targetRobot,
      coordinateFrame: "world",
      voxelSize: space.voxelSize,
      origin: space.origin,
      sharedVoxelCount: space.voxels.length,
      csv: output.csv,
      placementSource: space.placementSource || "descriptor",
      placements: {
        [this.ctx.manifest.name]: placementOf(space.owner),
        [space.targetRobot]: placementOf(space.partner)
      },
      diagnostics: space.diagnostics
    }, null, 2) + "\n");
  }

  select(targetRobot, options = {}) {
    const ranked = this.rank(targetRobot, options);
    if (!ranked.success) return ranked;
    const seed = Number.isFinite(Number(options.seed)) ? Number(options.seed) : 42;
    const random = seededRandom(seed);
    const candidate = ranked.candidates[Math.floor(random() * ranked.candidates.length)];
    return this.commit(targetRobot, candidate.id);
  }

  // Resolved exactly as the runtime resolves it, because the ranking can only
  // judge a rendezvous if it computes the approach the runtime will attempt.
  handoverApproachModeFor(targetRobot) {
    const pairs = this.ctx.manifest.sharedEnvironment?.interactionPairs;
    if (Array.isArray(pairs)) {
      const pair = pairs.find((candidate) => Array.isArray(candidate.robots)
        && candidate.robots.includes(this.ctx.manifest.name)
        && candidate.robots.includes(targetRobot));
      const fromPair = normalizeApproachMode(pair?.approachMode, null);
      if (fromPair) return fromPair;
    }
    // The receiving robot's own default, since that is whose approach it is.
    const configured = this.spaces.get(targetRobot)?.partnerManifest?.runtime?.handoverApproachMode;
    return normalizeApproachMode(configured, "opposed");
  }

  // How far each arm's straight approach leg stays clear of its own base-joint
  // axis, and that expressed as a 0..1 score. This is the term the workspace
  // cloud cannot supply, because it judges points while the failure is a
  // property of the path between them.
  posturePenalty(cubeWorld, space, targetRobot) {
    const tuning = this.configuration?.singularity || {};
    const minimumMargin = Number(tuning.minimumMargin ?? 0.05);
    const goodMargin = Math.max(minimumMargin + EPSILON, Number(tuning.goodMargin ?? 0.15));

    // Resolved per role, exactly as the runtime resolves it: a robot in the
    // middle of a chain can be pinned as giver and free as receiver, and the
    // ranking has to score the leg each one will actually travel.
    const poseLocked = (manifest, role) => {
      const runtime = manifest.runtime || {};
      const perRole = role === "giver"
        ? runtime.enableHandoverPoseLockAsGiver
        : runtime.enableHandoverPoseLockAsReceiver;
      if (perRole !== undefined && perRole !== null) return perRole !== false;
      return runtime.enableHandoverPoseLock !== false;
    };

    const forRobot = (placement, manifest, axisWorld, role) => {
      // An arm whose wrist is not pinned has no approach leg to judge. It is
      // sent to the rendezvous position and the free wrist picks its own
      // posture, so scoring it against a straight leg it will never travel
      // would reject rendezvous points it can reach perfectly well.
      if (!poseLocked(manifest, role)) return null;
      const grip = manifest.runtime?.gripper || {};
      const axis = axisWorld && worldVectorToLocal(axisWorld, orientationRadians(placement));
      if (!axis) return null;
      // The commanded point is the grasp point, and the tool control point is
      // the wrist's own tip, so the approach leg starts at the grasp point
      // itself rather than one tool length behind it.
      return shoulderMargin({
        graspLocal: targetForRobot(cubeWorld, placement, manifest).localTcp,
        axisLocal: axis,
        toolLength: 0,
        standoff: Number(grip.handoverStandoffDistance ?? 0.15)
      });
    };

    // The same axis beginHandoverSession will lock the giver to, for the same
    // reason the receiver's is resolved here: a ranking that scores a different
    // presentation than the one performed is scoring a manoeuvre nobody makes.
    const giverAxis = giverPresentAxis({
      mode: space.ownerManifest.runtime?.handoverPresentMode,
      mountingAxis: toolAxisWorld(space.owner, space.ownerManifest.runtime?.gripper?.toolAxis),
      preferred: cellBaselineAxis(space.owner, space.partner) || [0, 0, -1]
    });
    const preferred = space.partnerManifest?.runtime?.handoverPerpendicularAxis
      || cellBaselineAxis(space.partner, space.owner)
      || [0, 0, -1];
    const receiverAxis = receiverApproachAxis(
      giverAxis, this.handoverApproachModeFor(targetRobot), preferred
    );

    const giverMargin = forRobot(space.owner, space.ownerManifest, giverAxis, "giver");
    const receiverMargin = forRobot(space.partner, space.partnerManifest, receiverAxis, "receiver");
    const margins = [giverMargin, receiverMargin].filter((value) => Number.isFinite(value));
    // No axis means no approach to judge; do not silently reject everything.
    const margin = margins.length > 0 ? Math.min(...margins) : Number.POSITIVE_INFINITY;
    const score = Number.isFinite(margin)
      ? Math.min(1, Math.max(0, (margin - minimumMargin) / (goodMargin - minimumMargin)))
      : 1;
    return { margin, minimumMargin, score, giverMargin, receiverMargin, giverAxis, receiverAxis };
  }

  rejectCandidate(targetRobot, candidateId) {
    // Told at the moment the agent strikes one, rather than passed back on the
    // next rank: the id is a bare atom, which is the argument shape the achieve
    // bridge already carries reliably for commitHandoverCandidate.
    const id = String(candidateId);
    if (!this.rejectedCandidates.has(targetRobot)) {
      this.rejectedCandidates.set(targetRobot, new Set());
    }
    this.rejectedCandidates.get(targetRobot).add(id);
    return { success: true, targetRobot, candidate: id, rejectedCount: this.rejectedCandidates.get(targetRobot).size };
  }

  clearRejectedCandidates(targetRobot) {
    const had = this.rejectedCandidates.get(targetRobot)?.size || 0;
    this.rejectedCandidates.delete(targetRobot);
    return { success: true, targetRobot, cleared: had };
  }

  rank(targetRobot, options = {}) {
    const space = this.spaces.get(targetRobot);
    if (!space) return { success: false, reason: "interaction_space_not_ready", targetRobot };
    const minZ = Number.isFinite(Number(options.minZ)) ? Number(options.minZ) : 0.3;
    const maxZ = Number.isFinite(Number(options.maxZ)) ? Number(options.maxZ) : 0.7;
    const coreConfiguration = this.configuration?.centralCore || {};
    const lowerQuantile = Number.isFinite(Number(options.lowerQuantile))
      ? Number(options.lowerQuantile) : Number(coreConfiguration.lowerQuantile ?? 0.2);
    const upperQuantile = Number.isFinite(Number(options.upperQuantile))
      ? Number(options.upperQuantile) : Number(coreConfiguration.upperQuantile ?? 0.8);
    const minimumCandidates = Math.max(1, Number(options.minimumCandidates || coreConfiguration.minimumCandidates || 10));
    const maximumCandidates = Math.max(
      minimumCandidates,
      Number(options.maximumCandidates || coreConfiguration.maximumCandidates || 60)
    );
    // A pair has to share enough space to be a rendezvous at all.
    //
    // Overlap is not a yes/no fact: two arms whose reach spheres barely touch
    // produce a handful of voxels at the far edge of both their workspaces, and
    // one voxel prices exactly as confidently as three hundred. The route then
    // reads the two numbers identically and commits to a pair whose entire
    // shared world is a single cell -- with nothing for the ranking to choose
    // between and nothing for the retry to fall back on when the cell refuses
    // it. Live cell: Robot1 and Robot3 shared ONE voxel and the cube was routed
    // through them. Below the floor the pair is not usable, which is a different
    // and more honest answer than an over-confident price.
    const minimumSharedVoxels = Math.max(0, Number(
      options.minimumSharedVoxels ?? this.configuration?.minimumSharedVoxels ?? DEFAULT_MINIMUM_SHARED_VOXELS
    ));
    if (space.voxels.length < minimumSharedVoxels) {
      return {
        success: false,
        reason: "interaction_space_below_minimum",
        targetRobot,
        sharedVoxelCount: space.voxels.length,
        minimumSharedVoxels
      };
    }
    const calibration = this.descriptor.handoverCalibration;
    const offset = calibration?.type === "shared_point_world_offset" &&
      calibration.sourceRobot === this.ctx.manifest.name && calibration.targetRobot === targetRobot
      ? calibration.sharedPointWorldOffset.map(Number) : [0, 0, 0];
    const heightCandidates = space.voxels.filter(({ center }) => center.z >= minZ && center.z <= maxZ);
    if (heightCandidates.length === 0) {
      return { success: false, reason: "no_height_filtered_interaction_voxel", targetRobot, minZ, maxZ };
    }
    const bounds = {};
    for (const axis of AXES) {
      const values = heightCandidates.map(({ center }) => center[axis]);
      bounds[axis] = [percentile(values, lowerQuantile), percentile(values, upperQuantile)];
    }
    let core = heightCandidates.filter(({ center }) => AXES.every(
      (axis) => center[axis] >= bounds[axis][0] && center[axis] <= bounds[axis][1]
    ));
    if (core.length < minimumCandidates) core = heightCandidates;
    const centroid = Object.fromEntries(AXES.map((axis) => [
      axis,
      core.reduce((sum, { center }) => sum + center[axis], 0) / core.length
    ]));
    const centerDistance = ({ center }) => Math.sqrt(distanceSquared(center, centroid));
    const maxCenterDistance = Math.max(...core.map(centerDistance), EPSILON);
    // Density is "how much usable room surrounds this point": the number of
    // core voxels within a couple of voxels of it. It is a proxy for a
    // rendezvous standing in the THICK part of the shared region rather than on
    // a thin spur, where both arms have somewhere to be slightly wrong.
    //
    // A refused voxel is not usable room. Density is recomputed on every rank,
    // and a candidate the cell has already refused stops counting towards its
    // neighbours' scores -- otherwise a point inherits its standing from poses
    // that have just been struck as unusable. Live cell: Robot3's third choice
    // for Robot4 scored density 1.000 on four neighbours, two of which were the
    // candidates refused on the first two attempts. It beat a point half as far
    // from the centre of the shared space, purely on that stale credit.
    //
    // CENTRALITY deliberately keeps counting them. It measures where a point
    // sits in the shared REGION, and a refused rendezvous is still part of that
    // region -- the arm simply cannot serve it. Dropping refusals from the
    // centroid would move the centre away from the geometry it describes and
    // make "central" mean something different on every retry.
    const rejected = this.rejectedCandidates.get(targetRobot) || new Set();
    const usable = core.map((_, index) => !rejected.has(candidateId(index)));
    const densityRadius = space.voxelSize * 2.5;
    const rawDensities = core.map((voxel) => core.reduce(
      (count, other, otherIndex) => count + (
        usable[otherIndex] && distanceSquared(voxel.center, other.center) <= densityRadius * densityRadius
          ? 1
          : 0
      ),
      0
    ));
    const maxDensity = Math.max(...rawDensities, 1);
    const ranked = [];
    let rejectedForPosture = 0;
    for (let index = 0; index < core.length; index += 1) {
      const voxel = core[index];
      const cubeWorld = {
        x: voxel.center.x + offset[0],
        y: voxel.center.y + offset[1],
        z: voxel.center.z + offset[2]
      };
      const giver = targetForRobot(cubeWorld, space.owner, space.ownerManifest);
      const receiver = targetForRobot(cubeWorld, space.partner, space.partnerManifest);
      const within = (point, pointBounds) => AXES.every(
        (axis) => point[axis] >= pointBounds.min[axis] && point[axis] <= pointBounds.max[axis]
      );
      const giverTolerance = Number(space.ownerManifest.runtime?.workspaceTolerance || 0.08);
      const receiverTolerance = Number(space.partnerManifest.runtime?.workspaceTolerance || 0.08);
      if (
        !within(giver.localTcp, space.ownerBounds) ||
        !within(receiver.localTcp, space.partnerBounds) ||
        space.ownerTree.nearestDistance(giver.localTcp) > giverTolerance ||
        space.partnerTree.nearestDistance(receiver.localTcp) > receiverTolerance
      ) {
        continue;
      }
      // Reachability of the individual points is not enough. The runtime
      // travels the last stretch as a straight line with the tool axis pinned,
      // and if that line passes over an arm's own base-joint axis it cannot be
      // followed continuously no matter how reachable every point on it is.
      // The workspace cloud cannot see this: it is sampled from independent
      // configurations, so it happily reports a wrist position the arm can only
      // reach by first rotating its base 180 degrees.
      const posture = this.posturePenalty(cubeWorld, space, targetRobot);
      if (posture.margin < posture.minimumMargin) {
        rejectedForPosture += 1;
        continue;
      }
      const giverDistance = space.ownerTree.nearestDistance(giver.localTcp);
      const receiverDistance = space.partnerTree.nearestDistance(receiver.localTcp);
      const centrality = Math.max(0, 1 - centerDistance(voxel) / maxCenterDistance);
      const density = rawDensities[index] / maxDensity;
      const giverSupport = Math.max(0, 1 - giverDistance / giverTolerance);
      const receiverSupport = Math.max(0, 1 - receiverDistance / receiverTolerance);
      ranked.push({
        id: candidateId(index),
        sourceRobot: this.ctx.manifest.name,
        targetRobot,
        selection: "agent_ranked_central_core",
        sharedPointWorld: voxel.center,
        sharedPointWorldOffset: { x: offset[0], y: offset[1], z: offset[2] },
        cubeWorld,
        giver,
        receiver,
        centrality,
        density,
        giverSupport,
        receiverSupport,
        posture: posture.score,
        postureMargin: posture.margin,
        giverApproachAxis: posture.giverAxis,
        receiverApproachAxis: posture.receiverAxis
      });
    }
    // Bounding and evidence calculation belong to the geometry capability;
    // the weighted ranking itself deliberately remains in the BDI agent.
    // If an unusually dense core exceeds the belief budget, retain its most
    // central voxels without applying the agent's multi-factor score here.
    ranked.sort((left, right) =>
      right.centrality - left.centrality || left.id.localeCompare(right.id, undefined, { numeric: true })
    );
    const keepCount = Math.min(ranked.length, maximumCandidates);
    const candidates = ranked.slice(0, keepCount);
    if (candidates.length === 0) {
      return {
        success: false,
        reason: rejectedForPosture > 0
          ? "no_singularity_free_interaction_voxel"
          : "no_reachable_central_interaction_voxel",
        candidateCount: core.length,
        rejectedForPosture,
        minZ,
        maxZ
      };
    }
    this.rankedCandidates.set(targetRobot, new Map(candidates.map((candidate) => [candidate.id, candidate])));
    return {
      success: true,
      targetRobot,
      center: centroid,
      bounds,
      heightCandidateCount: heightCandidates.length,
      coreCandidateCount: core.length,
      candidateCount: candidates.length,
      candidates
    };
  }

  commit(targetRobot, candidateId) {
    const candidate = this.rankedCandidates.get(targetRobot)?.get(String(candidateId));
    if (!candidate) return { success: false, reason: "unknown_ranked_handover_candidate", targetRobot, candidateId };
    return {
      ...candidate,
      success: true,
      id: `handover_${Date.now()}_${this.nextPlanId++}`
    };
  }
}

function cliArguments(argv) {
  const result = {};
  for (let index = 0; index < argv.length; index += 1) {
    if (!argv[index].startsWith("--")) continue;
    const next = argv[index + 1];
    // A flag with no value of its own is a flag, not a key with an undefined
    // value -- otherwise --all-pairs reads as falsy and silently does nothing.
    if (!next || next.startsWith("--")) {
      result[argv[index].slice(2)] = true;
      continue;
    }
    result[argv[index].slice(2)] = next;
    index += 1;
  }
  return result;
}

// Every configured pair, each computed by the robot that owns it. This is the
// step that moving a robot needs and that generating the shared cell does not
// do: the cell generator resamples each arm's own workspace and rewrites the
// descriptor, but the pairwise spaces are left describing the layout the cell
// used to have, and the visual in the scene is drawn from those.
function buildEveryPair(repoRoot, manifest, options = {}) {
  const relative = manifest.sharedEnvironment?.descriptor;
  if (!relative) throw new Error("Manifest has no sharedEnvironment.descriptor; --all-pairs needs a shared cell.");
  const descriptorPath = path.resolve(repoRoot, relative);
  const descriptor = JSON.parse(fs.readFileSync(descriptorPath, "utf8"));
  const configuredPairs = descriptor.interactionSpace?.pairs || [];
  const robots = (descriptor.robots || []).map((robot) => robot.name);
  if (robots.length < 2) throw new Error(`Descriptor lists fewer than two robots: ${descriptorPath}`);

  // Every combination, not only the configured ones. Which robots hand over to
  // each other is a decision; which robots can physically reach the same voxel
  // is a fact, and the cell was previously blind to any of those facts nobody
  // had written down. Robot1 and Robot3 share exactly one voxel -- too small to
  // hand over in, and exactly the kind of thing that must be known rather than
  // assumed away.
  const scanEveryCombination = options.scope !== "configured";
  const wanted = [];
  for (let first = 0; first < robots.length; first += 1) {
    for (let second = first + 1; second < robots.length; second += 1) {
      const configured = configuredPairs.find((pair) => pairContains(pair, robots[first], robots[second]));
      if (!configured && !scanEveryCombination) continue;
      wanted.push(configured || synthesizePair(robots[first], robots[second]));
    }
  }

  const results = wanted.map((pair) => {
    const [ownerRobot] = canonicalPairRobots(pair.robots[0], pair.robots[1]);
    const targetRobot = pair.robots.find((robot) => robot !== ownerRobot);
    const entry = descriptor.robots?.find((robot) => robot.name === ownerRobot);
    if (!entry) throw new Error(`Descriptor has no placement for ${ownerRobot}`);
    const ownerManifest = JSON.parse(fs.readFileSync(path.resolve(repoRoot, entry.manifest), "utf8"));
    const controller = new InteractionSpaceController({ repoRoot, manifest: ownerManifest });
    const built = controller.build(targetRobot, { synthesizePair: true });
    return {
      pairKey: pair.key,
      ownerRobot: pair.ownerRobot,
      targetRobot,
      discovery: pair.discovery || "configured",
      handoverPair: Boolean(configuredPairs.find((candidate) => candidate.key === pair.key)),
      output: controller.outputFor(pair),
      ...built
    };
  });

  // A pair that turns out to share nothing is a real answer, not a failure: it
  // is the one result that lets a robot stop treating another as a neighbour.
  const overlapping = results.filter((result) => result.success);
  const disjoint = results.filter((result) => !result.success
    && (result.reason === "no_geometric_overlap" || result.reason === "voxel_intersection_empty"));
  const failed = results.filter((result) => !overlapping.includes(result) && !disjoint.includes(result));

  if (failed.length === 0) {
    const existing = new Map((descriptor.sharedSpaces || []).map((entry) => [entry.pairKey, entry]));
    // Colours are assigned in one pass over the pairs in key order. A pair keeps
    // the colour it was already drawn in unless another pair got there first,
    // so two clouds in the same scene never come out the same colour.
    const claimed = new Set();
    const ordered = [...results].sort((left, right) => left.pairKey.localeCompare(right.pairKey));
    const colorOf = new Map();
    // A pair that shares nothing is never drawn, so it must not consume a
    // colour that a drawn pair then has to share.
    let nextIndex = 0;
    const freeColor = () => {
      while (claimed.has(JSON.stringify(paletteColor(nextIndex)))) nextIndex += 1;
      return paletteColor(nextIndex);
    };
    for (const result of ordered.filter((candidate) => candidate.success)) {
      const previous = existing.get(result.pairKey)?.color;
      const color = previous && !claimed.has(JSON.stringify(previous)) ? previous : freeColor();
      claimed.add(JSON.stringify(color));
      colorOf.set(result.pairKey, color);
    }
    for (const result of ordered.filter((candidate) => !candidate.success)) {
      colorOf.set(result.pairKey, existing.get(result.pairKey)?.color || null);
    }
    descriptor.sharedSpaces = ordered.map((result) => {
      const previous = existing.get(result.pairKey) || {};
      return {
        ...previous,
        pairKey: result.pairKey,
        discovery: result.discovery,
        handoverPair: result.handoverPair,
        version: result.success ? result.version : null,
        voxelSize: result.success ? result.voxelSize : null,
        voxelCount: result.success ? result.voxelCount : 0,
        ...(result.success ? {} : { reason: result.reason }),
        csv: result.output?.csv || previous.csv,
        metadata: result.output?.metadata || previous.metadata,
        visualObject: previous.visualObject || `interaction_${result.pairKey}`,
        color: colorOf.get(result.pairKey)
      };
    });
    if (descriptor.sharedSpaces.length === 1) descriptor.sharedSpace = descriptor.sharedSpaces[0];
    writeFileAtomic(descriptorPath, JSON.stringify(descriptor, null, 2) + "\n");
  }
  return { descriptorPath, results, overlapping, disjoint, failed };
}

if (require.main === module) {
  const args = cliArguments(process.argv.slice(2));
  const usage = [
    "Usage:",
    "  node src/generator/interactionSpace.js --manifest <manifest.json> --target <RobotName>",
    "  node src/generator/interactionSpace.js --manifest <any-robot-in-the-cell.json> --all-pairs",
    "",
    "--all-pairs computes EVERY combination of robots in the cell, not only the",
    "configured handover pairs, and resyncs the shared-cell descriptor. Add",
    "--configured to restrict it to the configured pairs. A pair that shares no",
    "voxels is reported and recorded, not treated as an error. Run it after moving",
    "a robot, then redraw the scene with:",
    "  npm run robot:pipeline -- --manifests <a>,<b>,<c> --update-shared-space-visual"
  ].join("\n");
  if (!args.manifest || (!args.target && !args["all-pairs"])) {
    console.error(usage);
    process.exit(1);
  }
  const repoRoot = path.resolve(__dirname, "..", "..");
  const manifest = JSON.parse(fs.readFileSync(path.resolve(process.cwd(), args.manifest), "utf8"));

  if (args["all-pairs"]) {
    const scope = args.configured ? "configured" : "all";
    const { descriptorPath, results, disjoint, failed } = buildEveryPair(repoRoot, manifest, { scope });
    for (const result of results) {
      const role = `${result.handoverPair ? "handover" : "geometry"}/${result.discovery}`;
      if (result.success) {
        console.log(`${result.pairKey} [${role}]: ${result.voxelCount} voxels, voxel size `
          + `${result.voxelSize.toFixed(5)} (${result.cached ? "unchanged" : "recomputed"})`);
      } else if (disjoint.includes(result)) {
        console.log(`${result.pairKey} [${role}]: no shared voxels (${result.reason})`);
      } else {
        console.log(`${result.pairKey} [${role}]: FAILED (${result.reason})`);
      }
    }
    console.log(`Descriptor: ${descriptorPath}`);
    process.exitCode = failed.length === 0 ? 0 : 1;
  } else {
    const controller = new InteractionSpaceController({ repoRoot, manifest });
    const result = controller.build(args.target);
    console.log(JSON.stringify(result, null, 2));
    process.exitCode = result.success ? 0 : 1;
  }
}

module.exports = {
  InteractionSpaceController,
  paletteColor,
  buildEveryPair,
  synthesizePair,
  writeFileAtomic,
  loadReachableCsvShared,
  reachableDigestShared,
  reachableTreeShared,
  KdTree3D,
  bboxOverlapVolume,
  canonicalPairKey,
  canonicalPairRobots,
  deterministicSample,
  estimateInteractionSpace,
  estimateSpacing,
  loadReachableCsv,
  percentile,
  pointToVoxel,
  seededRandom,
  voxelKey,
  targetForRobot,
  toWorld
};
