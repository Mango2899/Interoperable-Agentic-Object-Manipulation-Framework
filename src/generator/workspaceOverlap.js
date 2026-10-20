#!/usr/bin/env node
"use strict";

// How much of each robot's workspace is contested by another arm.
//
// This is the measurement that says what a motion lock actually costs. A global
// mutex serialises the whole cell; this reports the fraction of each robot's
// reachable space that any other arm can also reach, which is the only part a
// spatial claim scheme would ever have to block on.
//
// Deliberately voxelised at ONE resolution against ONE origin for every robot.
// The pairwise interaction spaces each pick their own voxel size from their own
// two clouds -- correct for planning a rendezvous in that pair, useless for
// comparing one robot against another, because a percentage of a differently
// sized voxel is not a percentage of the same thing.

const fs = require("fs");
const path = require("path");

const {
  bboxOverlapVolume,
  loadReachableCsv,
  pointToVoxel,
  toWorld,
  voxelKey
} = require("./interactionSpace");
const { REPO_ROOT } = require("./core/context");

const DEFAULT_VOXEL_SIZE = 0.05;
const AXES = ["x", "y", "z"];

function boundsOf(points) {
  const bounds = { min: {}, max: {} };
  for (const axis of AXES) {
    const values = points.map((point) => point[axis]);
    bounds.min[axis] = Math.min(...values);
    bounds.max[axis] = Math.max(...values);
  }
  return bounds;
}

function overlapReport(descriptorPath, options = {}) {
  const descriptor = JSON.parse(fs.readFileSync(descriptorPath, "utf8"));
  const voxelSize = Number(options.voxelSize) > 0 ? Number(options.voxelSize) : DEFAULT_VOXEL_SIZE;
  const entries = descriptor.robots || [];
  if (entries.length < 2) throw new Error(`Descriptor lists fewer than two robots: ${descriptorPath}`);

  const clouds = new Map(entries.map((robot) => [
    robot.name,
    loadReachableCsv(path.resolve(REPO_ROOT, robot.workspaceCsv)).map((point) => toWorld(point, robot))
  ]));

  // One origin for every robot, placed below the whole cell, so a voxel index
  // means the same cell of space whichever cloud produced it.
  const all = [...clouds.values()].flat();
  const cellBounds = boundsOf(all);
  const origin = Object.fromEntries(AXES.map((axis) => [axis, cellBounds.min[axis] - voxelSize]));

  const occupied = new Map([...clouds].map(([name, points]) => [
    name,
    new Set(points.map((point) => voxelKey(pointToVoxel(point, origin, voxelSize))))
  ]));

  const names = [...clouds.keys()];
  const contested = new Map(names.map((name) => [name, new Set()]));
  const pairs = [];
  for (let first = 0; first < names.length; first += 1) {
    for (let second = first + 1; second < names.length; second += 1) {
      const [left, right] = [names[first], names[second]];
      const shared = [...occupied.get(left)].filter((key) => occupied.get(right).has(key));
      for (const key of shared) {
        contested.get(left).add(key);
        contested.get(right).add(key);
      }
      pairs.push({
        pairKey: `${left}__${right}`,
        robots: [left, right],
        sharedVoxels: shared.length,
        boundingBoxOverlapVolume: bboxOverlapVolume(boundsOf(clouds.get(left)), boundsOf(clouds.get(right))),
        shareOf: {
          [left]: shared.length / occupied.get(left).size,
          [right]: shared.length / occupied.get(right).size
        }
      });
    }
  }

  const robots = names.map((name) => ({
    robot: name,
    reachableSamples: clouds.get(name).length,
    workspaceVoxels: occupied.get(name).size,
    contestedVoxels: contested.get(name).size,
    contestedFraction: contested.get(name).size / occupied.get(name).size
  }));

  return { descriptor: descriptorPath, voxelSize, origin, robots, pairs };
}

function formatReport(report) {
  const percent = (value) => `${(value * 100).toFixed(2)}%`;
  const lines = [
    `Workspace overlap at ${report.voxelSize} m voxels, common origin.`,
    "",
    "robot      samples   voxels   contested   share   uncontested"
  ];
  for (const robot of report.robots) {
    lines.push(
      `${robot.robot.padEnd(10)} ${String(robot.reachableSamples).padStart(7)} `
      + `${String(robot.workspaceVoxels).padStart(8)} ${String(robot.contestedVoxels).padStart(11)} `
      + `${percent(robot.contestedFraction).padStart(7)} ${percent(1 - robot.contestedFraction).padStart(13)}`
    );
  }
  lines.push("", "pair                    shared   of each");
  for (const pair of report.pairs) {
    const shares = pair.robots.map((name) => `${percent(pair.shareOf[name])} of ${name}`).join(", ");
    lines.push(`${pair.pairKey.padEnd(22)} ${String(pair.sharedVoxels).padStart(7)}   ${shares}`);
  }
  return lines.join("\n");
}

if (require.main === module) {
  const args = process.argv.slice(2);
  const valueOf = (flag) => {
    const index = args.indexOf(flag);
    return index === -1 ? null : args[index + 1];
  };
  const descriptor = valueOf("--descriptor")
    || path.join(REPO_ROOT, "generated", "robots", "Robot1_Robot2_Robot3.shared.json");
  const report = overlapReport(path.resolve(process.cwd(), descriptor), {
    voxelSize: valueOf("--voxel-size")
  });
  console.log(args.includes("--json") ? JSON.stringify(report, null, 2) : formatReport(report));
}

module.exports = { DEFAULT_VOXEL_SIZE, formatReport, overlapReport };
