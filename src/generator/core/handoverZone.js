"use strict";

const { claimFor, occupancyFor, conservativeClaim, framePolyline } = require("./claims");

function zoneSettings(manifest) {
  const setting = manifest?.runtime?.handoverZone;
  const radius = Number(setting?.radius ?? 0.12);
  return setting?.enabled === true && Number.isFinite(radius) && radius > 0
    ? { radius } : null;
}

function sessionZone(source, target, center) {
  const a = zoneSettings(source);
  const b = zoneSettings(target);
  if (!a || !b || !center || ![center.x, center.y, center.z].every(Number.isFinite)) return null;
  return { pairKey: [source.name, target.name].sort().join("__"),
    center: { x: center.x, y: center.y, z: center.z }, radius: Math.max(a.radius, b.radius) };
}

function validZone(zone, robot) {
  return zone && typeof zone.pairKey === "string" && zone.pairKey.split("__").includes(robot)
    && Number.isFinite(zone.radius) && zone.radius > 0
    && [zone.center?.x, zone.center?.y, zone.center?.z].every(Number.isFinite);
}

// Use the original grid keys, never a separate lock namespace. A normal motion
// claiming the whole region must conflict with every smaller session claim.
function zoneKeys(region, zone) {
  return region.allKeys().filter((full) => {
    const indices = full.slice(region.pairKey.length + 1).split(",").map(Number);
    let distanceSquared = 0;
    ["x", "y", "z"].forEach((axis, i) => {
      const low = region.origin[axis] + indices[i] * region.voxelSize;
      const high = low + region.voxelSize;
      const distance = Math.max(low - zone.center[axis], 0, zone.center[axis] - high);
      distanceSquared += distance * distance;
    });
    return distanceSquared <= zone.radius * zone.radius;
  });
}

// The rendezvous pocket is claimed whole, so two arms are never inside it at
// once: it is where the grippers interleave around one cube, and a polyline
// through joint origins is exactly the model that describes that badly.
//
// WHAT ARMS IT is the whole question. Arming it from the swept-path claim -- the
// entire body inflated by a body radius -- meant an elbow passing a quarter of a
// metre away held the pocket, and neither arm released it until it had backed
// most of the way out of the shared region. Since both the giver going home and
// the receiver moving on start AT the rendezvous, both wanted the pocket for the
// larger part of their motion, and the two motions serialised completely: the
// contested region had been shrunk to the zone, but nothing could overlap.
//
// It is the tool that meets the other tool, so the tool is what arms the lock.
// "The giver has left the pocket" then means what the plans upstream take it to
// mean, and the arms are free to pass on opposite sides of it.
function toolInZone(frames, zone, bodyRadius) {
  return frames.some((flat) => {
    const polyline = framePolyline(flat);
    const tool = polyline[polyline.length - 1];
    if (!tool || ![tool.x, tool.y, tool.z].every(Number.isFinite)) return false;
    return Math.hypot(tool.x - zone.center.x, tool.y - zone.center.y, tool.z - zone.center.z)
      <= zone.radius + bodyRadius;
  });
}

function handoverPathClaims(robot, regions, zones, bodyRadius, { occupancy = false } = {}) {
  const partitions = regions.filter((region) => region.includes(robot)).map((region) => {
    const named = zones.filter((zone) => validZone(zone, robot) && zone.pairKey === region.pairKey);
    const centers = named.map((zone) => ({ zone, keys: zoneKeys(region, zone) }))
      .filter((center) => center.keys.length);
    // Coarse in exactly one case: this IS a rendezvous region and its pocket
    // geometry did not resolve, so something is wrong with the very thing the
    // motion is being arbitrated around. Falling back to the whole region for
    // every OTHER region as well made a robot leaving one handover reserve all
    // of the shared space it has with every other arm in the chain -- Robot2
    // walking away from Robot1 took the whole of Robot2__Robot3 (357 voxels)
    // and Robot2__Robot4 (1043) with it, and then waited on arms that were
    // nowhere near the handover. Those regions are claimed from the path, which
    // is known here in full; that is what makes it a reserved path.
    return { region, centers, coarse: named.length > 0 && centers.length === 0 };
  });
  return (frames) => {
    const voxels = new Set();
    const touched = [];
    for (const { region, centers, coarse } of partitions) {
      const claim = coarse
        ? conservativeClaim(robot, [region], frames, { bodyRadius })
        : (occupancy ? occupancyFor : claimFor)(robot, [region], frames, { bodyRadius });
      const inPocket = centers.filter((center) => toolInZone(frames, center.zone, bodyRadius));
      if (!claim.voxels.length && !inPocket.length) continue;
      touched.push(region.pairKey);
      claim.voxels.forEach((key) => voxels.add(key));
      for (const center of inPocket) center.keys.forEach((key) => voxels.add(key));
    }
    return { robot, regions: touched, voxels: [...voxels], contested: voxels.size > 0, granularity: "handover_zone_and_path" };
  };
}

module.exports = { zoneSettings, sessionZone, validZone, zoneKeys, handoverPathClaims };
