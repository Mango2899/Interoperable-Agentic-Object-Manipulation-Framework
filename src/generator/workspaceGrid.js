"use strict";

// Runtime workspace admission built from the RobWoT workspace CSV.
//
// The CSV is a Monte-Carlo sample of tip positions that some collision-free
// posture can reach, recorded once during generation. It is therefore a
// reachability oracle and nothing more: it knows where the tip can be, not
// which path leads there and not where the scene's obstacles are today.
// This module turns the reachable samples into a voxel hash so admission
// costs one lookup instead of a scan over every sample, and carries the
// obstacle boxes that the driver reports for the current command.
//
// Voxel size defaults to the same estimator the interaction-space pipeline
// uses (a robust percentile of nearest-neighbour spacing), so the grid never
// claims resolution the source data does not support.

const { estimateSpacing, loadReachableCsv, pointToVoxel, voxelKey } = require("./interactionSpace");

const AXES = ["x", "y", "z"];

function isFinitePoint(point) {
  return point != null && AXES.every((axis) => Number.isFinite(Number(point[axis])));
}

function toPoint(value) {
  return { x: Number(value.x), y: Number(value.y), z: Number(value.z) };
}

function normalizeBox(box) {
  if (!box || !isFinitePoint(box.min) || !isFinitePoint(box.max)) return null;
  const min = toPoint(box.min);
  const max = toPoint(box.max);
  return {
    alias: box.alias == null ? "obstacle" : String(box.alias),
    min: { x: Math.min(min.x, max.x), y: Math.min(min.y, max.y), z: Math.min(min.z, max.z) },
    max: { x: Math.max(min.x, max.x), y: Math.max(min.y, max.y), z: Math.max(min.z, max.z) }
  };
}

// CoppeliaSim returns an empty Lua table as an object rather than an array.
function asArray(value) {
  if (Array.isArray(value)) return value;
  if (value && typeof value === "object") return Object.values(value);
  return [];
}

function segmentSamples(start, target, stepSize) {
  const from = toPoint(start);
  const to = toPoint(target);
  const distance = Math.hypot(to.x - from.x, to.y - from.y, to.z - from.z);
  const steps = Math.max(1, Math.ceil(distance / Math.max(stepSize, 1e-6)));
  const samples = [];
  for (let step = 1; step <= steps; step += 1) {
    const alpha = step / steps;
    samples.push({
      x: from.x + (to.x - from.x) * alpha,
      y: from.y + (to.y - from.y) * alpha,
      z: from.z + (to.z - from.z) * alpha
    });
  }
  return samples;
}

class WorkspaceGrid {
  static fromCsv(csvPath, options = {}) {
    return new WorkspaceGrid(loadReachableCsv(csvPath), { ...options, source: csvPath });
  }

  constructor(points, options = {}) {
    const reachable = asArray(points).filter(isFinitePoint).map(toPoint);
    if (reachable.length === 0) {
      throw new Error("A workspace grid needs at least one reachable sample");
    }

    this.source = options.source || null;
    this.pointCount = reachable.length;
    // A percentile of nearest-neighbour spacing over a subsample is stable
    // well below the full cloud, and keeps server startup off the KD-tree's
    // full-size build.
    this.voxelSize = Number(options.voxelSize) > 0
      ? Number(options.voxelSize)
      : estimateSpacing(
        reachable,
        Number(options.spacingSampleSize) > 0 ? Number(options.spacingSampleSize) : 4000,
        Number.isFinite(options.spacingPercentile) ? Number(options.spacingPercentile) : 0.75,
        Number(options.seed) || 1
      );

    // Admission tolerance is a property of how densely this robot's workspace
    // was sampled, not a constant. A point inside the workspace can still fall
    // in a sampling gap, and the gap scales with spacing: the same 80 mm is
    // 1.8x spacing for a UR10 cloud and 4.5x for a UR3 one, so a single value
    // is simultaneously too strict for one arm and too loose for another.
    this.spacingFactor = Number(options.spacingFactor) > 0 ? Number(options.spacingFactor) : 3;
    this.defaultTolerance = Number(options.tolerance) > 0
      ? Number(options.tolerance)
      : this.spacingFactor * this.voxelSize;

    // Two different scales, deliberately. voxelSize describes the resolution
    // the source data actually supports and is what the estimator reports.
    // cellSize is the hash bucket, and must be at least the admission radius:
    // sizing buckets to the sample spacing instead would make a single query
    // sweep a thousand buckets, which is slower than scanning the raw cloud.
    this.cellSize = Math.max(this.voxelSize, this.defaultTolerance);

    this.bounds = {
      min: { x: Infinity, y: Infinity, z: Infinity },
      max: { x: -Infinity, y: -Infinity, z: -Infinity }
    };
    for (const point of reachable) {
      for (const axis of AXES) {
        this.bounds.min[axis] = Math.min(this.bounds.min[axis], point[axis]);
        this.bounds.max[axis] = Math.max(this.bounds.max[axis], point[axis]);
      }
    }
    this.origin = { ...this.bounds.min };

    // Cells hold flattened coordinates so a tolerance query can still report
    // the exact nearest-sample distance the scanning implementation reported.
    this.cells = new Map();
    for (const point of reachable) {
      const key = voxelKey(pointToVoxel(point, this.origin, this.cellSize));
      const cell = this.cells.get(key);
      if (cell) {
        cell.push(point.x, point.y, point.z);
      } else {
        this.cells.set(key, [point.x, point.y, point.z]);
      }
    }

    // The radius of the smallest sphere centred on the base that contains every
    // reachable sample. Two arms can only share space if their spheres overlap,
    // and unlike the axis-aligned bounds this number does not change when the
    // robot is mounted at an angle -- a sphere around the base is invariant
    // under any rotation of the arm. That is what makes it usable as a
    // neighbour test between robots mounted differently.
    this.reachRadius = 0;
    for (const point of reachable) {
      this.reachRadius = Math.max(this.reachRadius, Math.hypot(point.x, point.y, point.z));
    }

    this.obstacleBoxes = [];
  }

  // The radius to actually test against.
  //
  // reachRadius is the extreme of a FINITE sample, so it under-states the true
  // reach by up to the gap between samples -- the farthest reachable point is
  // almost never one that was drawn. Understating it is the dangerous
  // direction: a neighbour test is a necessary condition, so a radius that is
  // too small prunes a pair that really can collide. Widening by the measured
  // sample spacing covers exactly that gap.
  get safeReachRadius() {
    return this.reachRadius + this.voxelSize;
  }

  // Everything another robot needs to decide whether this one could possibly
  // reach the same place, and nothing else. Small enough to be a property.
  signature() {
    return {
      sampleCount: this.pointCount,
      spacing: this.voxelSize,
      reachRadius: this.reachRadius,
      safeReachRadius: this.safeReachRadius,
      localBounds: this.bounds,
      source: this.source
    };
  }

  get occupiedVoxelCount() {
    return this.cells.size;
  }

  voxelOf(point) {
    return pointToVoxel(point, this.origin, this.voxelSize);
  }

  // Obstacles stay exact axis-aligned boxes instead of being voxelized: a
  // 4-5 cm grid would inflate every obstacle by up to a voxel, and a handover
  // brings the two tools deliberately close. Fifty boxes cost less than the
  // quantization error would.
  setObstacleBoxes(boxes) {
    this.obstacleBoxes = asArray(boxes).map(normalizeBox).filter(Boolean);
    return this.obstacleBoxes.length;
  }

  // Bounds are the extremes of a finite sample, so they are widened by the
  // admission tolerance: this gate exists to reject far-away targets cheaply,
  // and must never be stricter than the nearest-sample test behind it. A
  // target a millimetre past the outermost sample is inside the workspace.
  withinBounds(point, tolerance = 0) {
    return AXES.every((axis) => (
      point[axis] >= this.bounds.min[axis] - tolerance && point[axis] <= this.bounds.max[axis] + tolerance
    ));
  }

  // Exact distance to the nearest reachable sample, or null when no sample
  // lies within radius. Only the voxels that can hold such a sample are read.
  nearestWithin(point, radius) {
    const span = Math.max(0, Math.ceil(radius / this.cellSize));
    const [cx, cy, cz] = pointToVoxel(point, this.origin, this.cellSize);
    let bestSquared = Infinity;
    for (let dx = -span; dx <= span; dx += 1) {
      for (let dy = -span; dy <= span; dy += 1) {
        for (let dz = -span; dz <= span; dz += 1) {
          const cell = this.cells.get(voxelKey([cx + dx, cy + dy, cz + dz]));
          if (!cell) continue;
          for (let index = 0; index < cell.length; index += 3) {
            const ex = point.x - cell[index];
            const ey = point.y - cell[index + 1];
            const ez = point.z - cell[index + 2];
            const squared = ex * ex + ey * ey + ez * ez;
            if (squared < bestSquared) bestSquared = squared;
          }
        }
      }
    }
    const nearest = Math.sqrt(bestSquared);
    return nearest <= radius ? nearest : null;
  }

  obstacleAt(point) {
    for (const box of this.obstacleBoxes) {
      if (
        point.x >= box.min.x && point.x <= box.max.x &&
        point.y >= box.min.y && point.y <= box.max.y &&
        point.z >= box.min.z && point.z <= box.max.z
      ) {
        return box;
      }
    }
    return null;
  }

  admits(point, tolerance) {
    if (!isFinitePoint(point)) {
      return { ok: false, reason: "target_not_a_point" };
    }
    const radius = Number(tolerance) > 0 ? Number(tolerance) : this.defaultTolerance;
    const candidate = toPoint(point);
    if (!this.withinBounds(candidate, radius)) {
      return { ok: false, reason: "target_outside_workspace_bounds", bounds: this.bounds };
    }
    const nearest = this.nearestWithin(candidate, radius);
    if (nearest === null) {
      return { ok: false, reason: "target_not_near_workspace_samples", tolerance: radius };
    }
    const obstacle = this.obstacleAt(candidate);
    if (obstacle) {
      return { ok: false, reason: "target_inside_obstacle", obstacle: obstacle.alias };
    }
    return { ok: true, nearestReachableDistance: nearest };
  }

  // Straight-line admission for a commanded tip motion. moveToPose interpolates
  // the tip along this line, so the sampled line is the path that will run.
  //
  // The first sample is where the arm ALREADY IS, and that is a measurement,
  // not a request. Sampling is random, so a cloud can fail to cover a pose the
  // arm genuinely holds -- a fully extended rest pose is the usual one, because
  // full extension is a singular configuration at the very edge of the
  // reachable set that uniform joint sampling essentially never lands on. Judge
  // such a start and the arm is stranded: every motion it could make is refused
  // for where it stands, including the one that would bring it back inside.
  //
  // So a leading run of samples outside the cloud is the arm on its way in, and
  // only refused if the line never gets there. Everything from the first
  // admitted sample onward is checked as before: leaving the workspace once
  // inside it is still a refusal. This is not the collision check -- the swept
  // body preflight guards obstacles independently, and is unaffected.
  admitsPath(samples, tolerance) {
    const points = asArray(samples).filter(isFinitePoint).map(toPoint);
    let entered = false;
    let firstFailure = null;
    for (let index = 0; index < points.length; index += 1) {
      const result = this.admits(points[index], tolerance);
      if (result.ok) {
        entered = true;
        continue;
      }
      if (!firstFailure) {
        firstFailure = { ...result, blockedAtSample: index + 1, blockedPoint: points[index], samples: points.length };
      }
      if (entered) {
        return { ...result, blockedAtSample: index + 1, blockedPoint: points[index], samples: points.length };
      }
    }
    // Never inside at any point: the line does not describe a reachable motion.
    if (!entered && firstFailure) {
      return firstFailure;
    }
    return { ok: true, samples: points.length };
  }

  describe() {
    return {
      points: this.pointCount,
      voxelSize: Number(this.voxelSize.toFixed(4)),
      tolerance: Number(this.defaultTolerance.toFixed(4)),
      cellSize: Number(this.cellSize.toFixed(4)),
      cells: this.cells.size,
      obstacleBoxes: this.obstacleBoxes.length
    };
  }
}

module.exports = { WorkspaceGrid, asArray, segmentSamples };
