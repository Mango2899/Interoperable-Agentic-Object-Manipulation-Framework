"use strict";

// What part of the shared space a motion is about to occupy.
//
// The cell moved one arm at a time because every planner has the other robots
// in its obstacle collection, so a preflighted path stops being valid the
// moment another arm moves. That reasoning only applies where two arms can
// actually reach the same place. This module answers, for a specific planned
// motion, WHICH shared voxels it will pass through -- and the common case is
// none at all, because 95-99% of each robot's workspace is reachable by nobody
// else (see docs, workspace overlap).
//
// A claim is deliberately an OVER-approximation. Claiming more space than the
// arm uses costs a little concurrency; claiming less permits a collision.

const AXES = ["x", "y", "z"];

function toPoint(values, offset) {
  return { x: Number(values[offset]), y: Number(values[offset + 1]), z: Number(values[offset + 2]) };
}

// A sample's flat [x,y,z,x,y,z,...] becomes the polyline through this arm's
// joint origins and tool point. The links lie along its segments.
function framePolyline(flat) {
  const points = [];
  for (let offset = 0; offset + 2 < flat.length; offset += 3) {
    points.push(toPoint(flat, offset));
  }
  return points;
}

function segmentPoints(from, to, step) {
  const distance = Math.hypot(to.x - from.x, to.y - from.y, to.z - from.z);
  const steps = Math.max(1, Math.ceil(distance / Math.max(step, 1e-6)));
  const points = [];
  for (let index = 0; index <= steps; index += 1) {
    const alpha = index / steps;
    points.push({
      x: from.x + (to.x - from.x) * alpha,
      y: from.y + (to.y - from.y) * alpha,
      z: from.z + (to.z - from.z) * alpha
    });
  }
  return points;
}

// Squared distance from a complete link segment to a voxel box. Between the
// crossings of the box's six planes, distance squared is a quadratic in t.
// Check its minimum on each interval, including both endpoints.
function segmentBoxDistanceSquared(from, to, min, max) {
  const delta = AXES.map((axis) => to[axis] - from[axis]);
  const cuts = [0, 1];
  AXES.forEach((axis, i) => {
    if (delta[i] === 0) return;
    for (const bound of [min[axis], max[axis]]) {
      const t = (bound - from[axis]) / delta[i];
      if (t > 0 && t < 1) cuts.push(t);
    }
  });
  cuts.sort((a, b) => a - b);
  const distanceAt = (t) => AXES.reduce((sum, axis, i) => {
    const value = from[axis] + t * delta[i];
    const gap = Math.max(min[axis] - value, 0, value - max[axis]);
    return sum + gap * gap;
  }, 0);
  let best = Infinity;
  for (let i = 1; i < cuts.length; i++) {
    const low = cuts[i - 1], high = cuts[i], middle = (low + high) / 2;
    let quadratic = 0, linear = 0;
    AXES.forEach((axis, j) => {
      const value = from[axis] + middle * delta[j];
      const bound = value < min[axis] ? min[axis] : value > max[axis] ? max[axis] : null;
      if (bound === null) return;
      quadratic += delta[j] ** 2;
      linear += delta[j] * (from[axis] - bound);
    });
    const t = quadratic > 0 ? Math.max(low, Math.min(high, -linear / quadratic)) : middle;
    best = Math.min(best, distanceAt(low), distanceAt(high), distanceAt(t));
  }
  return best;
}

// One region of shared space: the pair it belongs to, the grid it is voxelised
// on, and the set of voxel keys that pair actually shares.
class SharedRegion {
  constructor({ pairKey, robots, voxelSize, origin, voxelKeys }) {
    this.pairKey = pairKey;
    this.robots = robots;
    this.voxelSize = Number(voxelSize);
    this.origin = origin;
    this.voxelKeys = voxelKeys instanceof Set ? voxelKeys : new Set(voxelKeys);
  }

  static fromMetadata(metadata, voxelCenters) {
    const keys = new Set(voxelCenters.map((center) => SharedRegion.keyOf(center, metadata.origin, metadata.voxelSize)));
    return new SharedRegion({
      pairKey: metadata.pairKey,
      robots: metadata.robots || String(metadata.pairKey).split("__"),
      voxelSize: metadata.voxelSize,
      origin: metadata.origin,
      voxelKeys: keys
    });
  }

  static keyOf(point, origin, voxelSize) {
    return AXES.map((axis) => Math.floor((point[axis] - origin[axis]) / voxelSize)).join(",");
  }

  includes(robot) {
    return this.robots.includes(robot);
  }

  get bounds() {
    if (this.cachedBounds) return this.cachedBounds;
    const min = { x: Infinity, y: Infinity, z: Infinity };
    const max = { x: -Infinity, y: -Infinity, z: -Infinity };
    for (const key of this.voxelKeys) {
      const index = key.split(",").map(Number);
      AXES.forEach((axis, position) => {
        const low = this.origin[axis] + index[position] * this.voxelSize;
        min[axis] = Math.min(min[axis], low);
        max[axis] = Math.max(max[axis], low + this.voxelSize);
      });
    }
    this.cachedBounds = { min, max };
    return this.cachedBounds;
  }

  // Every voxel, as the claim keys the arbiter compares.
  allKeys() {
    return [...this.voxelKeys].map((key) => `${this.pairKey}:${key}`);
  }

  get voxelBoxes() {
    if (!this.cachedVoxelBoxes) this.cachedVoxelBoxes = [...this.voxelKeys].map((key) => {
      const indices = key.split(",").map(Number);
      const min = {}, max = {};
      AXES.forEach((axis, i) => {
        min[axis] = this.origin[axis] + indices[i] * this.voxelSize;
        max[axis] = min[axis] + this.voxelSize;
      });
      return { key, min, max };
    });
    return this.cachedVoxelBoxes;
  }

  withinBounds(point, margin = 0) {
    const { min, max } = this.bounds;
    return AXES.every((axis) => point[axis] >= min[axis] - margin && point[axis] <= max[axis] + margin);
  }

  // Which of this region's voxels a world point could touch, given a body
  // radius.
  //
  // Distance is measured from the point's OWN voxel, not from the point. That
  // is what dilates the claim by one voxel on every axis -- a link passing near
  // a boundary claims the voxel it is about to enter, not only the one it is in
  // -- and it is also what keeps the answer a function of the containing cell
  // alone, which is what makes the cache below and claimFor's per-cell dedup
  // sound. Taking the whole index cube of that reach instead is a much larger
  // claim than a body radius warrants: at the live cell's 0.048 m grid a 0.15 m
  // radius took an 11-voxel cube, 0.53 m on a side, so a single arm sample near
  // the middle of a shared region claimed roughly half of it and two arms
  // retreating from the same rendezvous could never hold disjoint claims. The
  // ball is the same over-approximation without the corners: 389 offsets rather
  // than 1331, and it is still a strict superset of the voxels within `radius`
  // of any point in the cell, because the cell-to-voxel distance can only be
  // shorter than the point-to-voxel distance.
  touching(point, radius) {
    const reach = Math.floor(radius / this.voxelSize) + 1;
    // The point sits somewhere inside its own cell, and the furthest voxel the
    // ball can reach is `reach` cells beyond it.
    if (!this.withinBounds(point, (reach + 1) * this.voxelSize)) return [];
    const base = AXES.map((axis) => Math.floor((point[axis] - this.origin[axis]) / this.voxelSize));
    // Keyed on the radius itself, not on the reach: two radii can share a reach
    // and still claim different balls inside it.
    const cacheKey = `${radius}:${base.join(",")}`;
    if (!this.touchCache) this.touchCache = new Map();
    if (this.touchCache.has(cacheKey)) return this.touchCache.get(cacheKey);
    // The overlap is sparse. Index occupied columns instead of visiting every
    // empty cell in a radius cube for every link sample, on every motion poll.
    if (!this.columns) {
      this.columns = new Map();
      for (const key of this.voxelKeys) {
        const [x, y, z] = key.split(",").map(Number);
        if (!this.columns.has(x)) this.columns.set(x, new Map());
        const ys = this.columns.get(x);
        if (!ys.has(y)) ys.set(y, []);
        ys.get(y).push({ z, key });
      }
      for (const ys of this.columns.values()) for (const zs of ys.values()) zs.sort((a, b) => a.z - b.z);
    }
    // Gap from this cell to a voxel `offset` cells away, along one axis.
    const gap = (offset) => Math.max(Math.abs(offset) - 1, 0) * this.voxelSize;
    const hits = [];
    for (let dx = -reach; dx <= reach; dx += 1) {
      const ys = this.columns.get(base[0] + dx);
      if (!ys) continue;
      const remainingX = radius * radius - gap(dx) ** 2;
      if (remainingX < 0) continue;
      for (let dy = -reach; dy <= reach; dy += 1) {
        const zs = ys.get(base[1] + dy);
        if (!zs) continue;
        const remaining = remainingX - gap(dy) ** 2;
        if (remaining < 0) continue;
        // The widest dz whose gap still fits inside what the radius has left.
        const spread = Math.floor(Math.sqrt(remaining) / this.voxelSize) + 1;
        const lowZ = base[2] - spread;
        const highZ = base[2] + spread;
        let low = 0, high = zs.length;
        while (low < high) {
          const mid = (low + high) >>> 1;
          if (zs[mid].z < lowZ) low = mid + 1; else high = mid;
        }
        for (let i = low; i < zs.length && zs[i].z <= highZ; i++) {
          hits.push(zs[i].key);
        }
      }
    }
    if (this.touchCache.size >= 4096) this.touchCache.clear();
    this.touchCache.set(cacheKey, hits);
    return hits;
  }
}

// The claim a set of swept frames makes on every region this robot shares.
//
// `frames` is one flat coordinate array per path sample, as the driver reports
// it. `bodyRadius` covers the thickness of the links, which the joint-origin
// polyline does not describe on its own.
function claimFor(robot, regions, frames, options = {}) {
  const bodyRadius = Number(options.bodyRadius ?? 0.12);
  const mine = regions.filter((region) => region.includes(robot));
  const claimed = new Map(mine.map((region) => [region.pairKey, new Set()]));
  const visited = new Map(mine.map((region) => [region.pairKey, new Set()]));
  if (mine.length === 0) return { robot, regions: [], voxels: [], contested: false };

  const step = Math.min(...mine.map((region) => region.voxelSize)) / 2;
  for (const flat of frames) {
    const polyline = framePolyline(flat);
    for (let index = 0; index + 1 < polyline.length; index += 1) {
      for (const point of segmentPoints(polyline[index], polyline[index + 1], step)) {
        for (const region of mine) {
          const cell = SharedRegion.keyOf(point, region.origin, region.voxelSize);
          const seen = visited.get(region.pairKey);
          if (seen.has(cell)) continue;
          seen.add(cell);
          for (const key of region.touching(point, bodyRadius)) {
            claimed.get(region.pairKey).add(key);
          }
        }
      }
    }
  }

  const voxels = [];
  for (const [pairKey, keys] of claimed) {
    for (const key of keys) voxels.push(`${pairKey}:${key}`);
  }
  return {
    robot,
    regions: [...claimed].filter(([, keys]) => keys.size > 0).map(([pairKey]) => pairKey),
    voxels,
    // The whole point: most motions touch nothing anyone else can reach, and
    // those never need to ask permission at all.
    contested: voxels.length > 0
  };
}

// Live occupancy uses link capsules, not the containing-cell dilation used to
// reserve a future path. That dilation jumps a voxel at grid boundaries, so a
// sub-millimetre motion could otherwise appear outside a 3 cm padded path.
// The body radius and every link are retained; only grid artefacts are removed.
function occupancyFor(robot, regions, frames, options = {}) {
  const radius = Number(options.bodyRadius ?? 0.12);
  const segments = frames.flatMap((flat) => {
    const points = framePolyline(flat);
    return points.slice(1).map((to, i) => ({ from: points[i], to }));
  });
  const voxels = [], touched = [];
  for (const region of regions.filter((region) => region.includes(robot))) {
    const before = voxels.length;
    for (const box of region.voxelBoxes) {
      if (segments.some(({ from, to }) => AXES.every((axis) =>
        Math.max(from[axis], to[axis]) + radius >= box.min[axis]
        && Math.min(from[axis], to[axis]) - radius <= box.max[axis])
        && segmentBoxDistanceSquared(from, to, box.min, box.max) <= radius * radius + 1e-12)) {
        voxels.push(`${region.pairKey}:${box.key}`);
      }
    }
    if (voxels.length > before) touched.push(region.pairKey);
  }
  return { robot, regions: touched, voxels, contested: voxels.length > 0 };
}

// The claim for a motion whose swept path is not yet known.
//
// Arbitration happens before planning, so the only postures available are the
// one the arm is standing in and the point it was told to go to. A claim built
// from those alone would describe neither the path between them nor where the
// elbow goes on the way, and a claim smaller than the motion is a collision.
//
// So the granularity is the REGION, not the voxel: if this motion could enter a
// shared region at all, it claims that region whole. Coarse, and deliberately
// so -- it cannot under-claim, and it still answers the question that matters,
// because a motion which cannot enter any shared region claims nothing and
// waits for nobody. That is the overwhelming majority of motions.
//
// Refining this to the actual swept volume means arbitrating after planning
// rather than before, and womatSweepFramePositions already reports what that
// would need.
function conservativeClaim(robot, regions, frames, options = {}) {
  const bodyRadius = Number(options.bodyRadius ?? 0.12);
  const target = options.target || null;
  const reach = Number(options.reachMargin ?? 0.35);
  const mine = regions.filter((region) => region.includes(robot));
  const entered = [];
  for (const region of mine) {
    const touching = claimFor(robot, [region], frames, { bodyRadius }).contested;
    const targetNear = target ? region.withinBounds(target, reach) : false;
    if (touching || targetNear) entered.push(region);
  }
  return {
    robot,
    regions: entered.map((region) => region.pairKey),
    voxels: entered.flatMap((region) => region.allKeys()),
    contested: entered.length > 0,
    granularity: "region"
  };
}

module.exports = { SharedRegion, claimFor, occupancyFor, conservativeClaim, framePolyline, segmentPoints, segmentBoxDistanceSquared };
