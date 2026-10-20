"use strict";

// Where the task stands in the cell, in the cell's own coordinates.
//
// A and B are not properties of a robot. Which arm holds the loaded support and
// which holds the empty one is exactly what the route search discovers, and it
// can only discover it if the supports are placed independently of the arms --
// otherwise the answer is written into the configuration that poses the
// question. Before this, A lived in Robot1's gripper config and was treated as
// world coordinates because "the scene owner sits at the origin", and B lived in
// Robot4's manifest in Robot4's frame. Two sites, two frames, neither of them
// the cell's.
//
// So a site is declared once, in world coordinates, and every robot is handed
// the same list. Whether a site is A or B is still never declared: it follows
// from whether a payload stands on it, which is what the arms read back out of
// the scene.
//
// Yaw only, about the site's own centre. These are boxes standing on the floor,
// the grasp point is the middle of the top face, and a level top face is what
// the descent models -- rolling one onto its side needs the pick path revisited,
// not another number here.

const DEFAULT_SUPPORT_SIZE = [0.14, 0.14, 0.16];
const DEFAULT_PAYLOAD_SIZE = [0.045, 0.045, 0.045];
// A payload is spawned this far above the top face so it settles onto it rather
// than starting interpenetrated with it.
const PAYLOAD_SPAWN_CLEARANCE = 0.0025;

function numbers(value, fallback) {
  if (!Array.isArray(value)) return [...fallback];
  const mapped = value.map(Number);
  return mapped.length === fallback.length && mapped.every(Number.isFinite) ? mapped : [...fallback];
}

// Resting on the top face, centred. The same arithmetic graspPointOf() uses to
// read a loaded site back out of the scene, so a site declared here and the
// grasp point detected from it describe one place.
function payloadSpawnPosition(supportPosition, supportSize, payloadSize) {
  return [
    Number(supportPosition[0]),
    Number(supportPosition[1]),
    Number(supportPosition[2]) + Number(supportSize[2]) / 2
      + Number(payloadSize[2]) / 2 + PAYLOAD_SPAWN_CLEARANCE
  ];
}

function normalisePayload(raw, supportPosition, supportSize) {
  if (!raw) return null;
  const payload = typeof raw === "string" ? { alias: raw } : raw;
  const alias = String(payload.alias || "").trim();
  if (!alias) return null;
  const size = numbers(payload.size, DEFAULT_PAYLOAD_SIZE);
  return {
    alias,
    size,
    position: Array.isArray(payload.position)
      ? numbers(payload.position, payloadSpawnPosition(supportPosition, supportSize, size))
      : payloadSpawnPosition(supportPosition, supportSize, size)
  };
}

// One site, in the shape everything downstream reads. Accepts what a hand-edited
// descriptor is likely to contain as well as what the parser emits.
function normaliseSite(raw) {
  const alias = String(raw?.alias || "").trim();
  if (!alias) throw new Error("A cell site needs an alias.");
  const position = numbers(raw.position, [0, 0, 0]);
  if (!Array.isArray(raw.position) || position.some((value) => !Number.isFinite(value))) {
    throw new Error(`Cell site "${alias}" needs a world position x,y,z.`);
  }
  const size = numbers(raw.size, DEFAULT_SUPPORT_SIZE);
  const yawDeg = Number.isFinite(Number(raw.yawDeg)) ? Number(raw.yawDeg) : 0;
  return {
    alias,
    size,
    position,
    yawDeg,
    payload: normalisePayload(raw.payload, position, size)
  };
}

// --sites "site_a@0.9,0,0.08,0,cube_a; site_b@1.25,-2,0.08,90"
//
// Semicolons separate sites and commas separate a site's numbers, the same way
// --offsets and --wall-offsets already read. After the three coordinates come an
// optional yaw and an optional payload alias, told apart by whether the token is
// a number -- so a site can carry a payload without stating a yaw it does not
// have, which is the common case for B.
function parseCellSites(value) {
  if (value === undefined || value === null || value === true || value === "") return [];
  const entries = String(value).split(";").map((part) => part.trim()).filter(Boolean);
  const seen = new Set();
  return entries.map((entry, index) => {
    const at = entry.indexOf("@");
    if (at <= 0) {
      throw new Error(`--sites entry ${index + 1} must be alias@x,y,z[,yawDeg][,payloadAlias] (got "${entry}").`);
    }
    const alias = entry.slice(0, at).trim();
    if (seen.has(alias)) {
      throw new Error(`--sites declares "${alias}" twice; a site alias names one object in the scene.`);
    }
    seen.add(alias);
    const fields = entry.slice(at + 1).split(",").map((token) => token.trim()).filter((token) => token !== "");
    const coordinates = fields.slice(0, 3).map(Number);
    if (coordinates.length < 3 || coordinates.some((number) => !Number.isFinite(number))) {
      throw new Error(`--sites entry ${index + 1} ("${alias}") needs a world position x,y,z.`);
    }
    let yawDeg = 0;
    let payload = null;
    for (const field of fields.slice(3)) {
      if (Number.isFinite(Number(field))) yawDeg = Number(field);
      else payload = field;
    }
    return normaliseSite({ alias, position: coordinates, yawDeg, payload });
  });
}

// The cell's sites, in world coordinates, however they were declared.
//
// A cell-level list wins wherever one exists: it is already the cell's frame and
// needs no robot to interpret it. The robot-local form stays readable so a
// manifest written before this still places its site where it always did -- its
// coordinates are shifted by the declaring robot's own offset and yaw, which is
// what made a site a property of a robot in the first place.
function resolveCellSites(contexts, offsets) {
  const declared = contexts
    .map((ctx) => ctx.manifest.sharedEnvironment?.taskSites)
    .find((sites) => Array.isArray(sites) && sites.length > 0);
  if (declared) return declared.map(normaliseSite);

  const sites = [];
  const seen = new Set();
  contexts.forEach((ctx, index) => {
    const offset = offsets[index] || [0, 0, 0];
    const orientationDeg = ctx.manifest.sharedEnvironment?.orientationDeg
      || [0, 0, Number(ctx.manifest.sharedEnvironment?.rotationDeg || 0)];
    const yaw = Number(orientationDeg[2] || 0) * Math.PI / 180;
    for (const site of ctx.manifest.runtime?.taskSites || []) {
      const alias = String(site.alias || "");
      if (!alias || seen.has(alias)) continue;
      seen.add(alias);
      const [x, y, z] = numbers(site.position, [0, 0, 0]);
      sites.push({
        // Which robot's frame these coordinates were read out of. Legacy only:
        // a cell-level site has no declaring robot, which is the whole point.
        robot: ctx.manifest.name,
        ...normaliseSite({
          alias,
          size: site.size,
          yawDeg: Number(orientationDeg[2] || 0) + Number(site.yawDeg || 0),
          position: [
            Number(offset[0] || 0) + Math.cos(yaw) * x - Math.sin(yaw) * y,
            Number(offset[1] || 0) + Math.sin(yaw) * x + Math.cos(yaw) * y,
            Number(offset[2] || 0) + z
          ]
        })
      });
    }
  });
  return sites;
}

// The site the cube starts on. Declaring two would make "the origin of the
// transfer" ambiguous, and the whole point is that the cell works that out for
// itself rather than being told twice.
function loadedSite(sites) {
  const loaded = (sites || []).filter((site) => site.payload);
  if (loaded.length > 1) {
    throw new Error(
      `${loaded.length} cell sites carry a payload (${loaded.map((site) => site.alias).join(", ")}); `
      + "exactly one site starts loaded."
    );
  }
  return loaded[0] || null;
}

// The pick scene, read off the site rather than off a robot's gripper config.
// Same shape gripSceneConfig has always returned, so everything that installs,
// samples against or drives the pick objects is unchanged.
function gripFromSite(site) {
  return {
    objectName: site.payload.alias,
    supportName: site.alias,
    objectSize: site.payload.size,
    objectPosition: site.payload.position,
    supportSize: site.size,
    supportPosition: site.position,
    yawDeg: site.yawDeg
  };
}

module.exports = {
  DEFAULT_PAYLOAD_SIZE,
  DEFAULT_SUPPORT_SIZE,
  PAYLOAD_SPAWN_CLEARANCE,
  gripFromSite,
  loadedSite,
  normaliseSite,
  parseCellSites,
  payloadSpawnPosition,
  resolveCellSites
};
