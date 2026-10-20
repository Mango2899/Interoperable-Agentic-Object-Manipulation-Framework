"use strict";

// One robot's WoT lower Thing: the TD affordances wired to its movement,
// safety, gripper and handover cores.

const fs = require("fs");
const path = require("path");

const { Servient } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");

const { HandoverSessions } = require("../cellPeers");
const { LocalArbiter } = require("./motionArbiter");
const { motionLogFor } = require("./motionLog");
const { luaArray, luaString } = require("./luaDriver");
const { readJson } = require("./context");
const { executeInScript, sandboxScriptHandle, simulationTimeStep } = require("./simConnection");
const {
  checkWorkspace,
  createRuntimeOptions,
  jointPayloadToRadians,
  loadSharedRegions,
  loadWorkspaceGrid
} = require("./runtimeOptions");
const { classifyTaskSites, taskSitesPayload } = require("../taskSites");
const { claimFor, conservativeClaim } = require("./claims");
const { reachableDigestShared } = require("../interactionSpace");
const { asArray } = require("../workspaceGrid");
const {
  handoverRetreat,
  simpleSmoothCartesianMove,
  simpleSmoothJointMove,
  smoothCartesianMove,
  smoothJointMove
} = require("./motion");
const { beginHandoverSession, setHandoverApproach, transferCubeOwnership } = require("./handover");
const { continuousHome } = require("./continuousHome");
const { zoneSettings, validZone, handoverPathClaims } = require("./handoverZone");

// Two affordances the shared cell needs at runtime rather than at generation
// time. They are added to the loaded TD when it predates them, so an already
// generated robot picks them up without a pipeline re-run.
function ensureSharedCellAffordances(td, ctx) {
  const hrefBase = `http://localhost:${ctx.manifest.ports.lowerThing}/${String(td.title).toLowerCase()}`;
  if (td.actions.beginHandoverSession?.input?.properties) {
    td.actions.beginHandoverSession.input.properties.cubeWorld = {
      type: "object", properties: { x: { type: "number" }, y: { type: "number" }, z: { type: "number" } },
      required: ["x", "y", "z"]
    };
  }
  if (!td.properties.getBasePose) {
    td.properties.getBasePose = {
      type: "object",
      description: "Live world pose of this robot's base in the loaded scene, with the placement the runtime was configured with.",
      properties: {
        robot: { type: "string" },
        position: { type: "array", items: { type: "number" } },
        orientation: { type: "array", items: { type: "number" } },
        configuredOffset: { type: "array", items: { type: "number" } },
        configuredOrientation: { type: "array", items: { type: "number" } }
      },
      forms: [{ href: `${hrefBase}/properties/getBasePose`, "htv:methodName": "GET" }]
    };
  }
  if (!td.actions.refreshInteractionVisual) {
    td.actions.refreshInteractionVisual = {
      description: "Redraw a pairwise interaction-space point cloud in the loaded scene from its exported CSV.",
      input: {
        type: "object",
        properties: {
          pairKey: { type: "string" },
          csv: { type: "string" },
          voxelSize: { type: "number" },
          color: { type: "array", items: { type: "number" } }
        },
        required: ["pairKey", "csv", "voxelSize"]
      },
      output: {
        type: "object",
        properties: {
          success: { type: "boolean" },
          reason: { type: "string" },
          object: { type: "string" },
          pointCount: { type: "number" }
        }
      },
      forms: [{ href: `${hrefBase}/actions/refreshInteractionVisual`, "htv:methodName": "POST" }]
    };
  }
  // Three affordances a sibling core needs once the robots run as separate
  // processes: the physical half of a cube transfer, and the two session
  // messages that used to be an object reference shared inside one heap. They
  // are peer-to-peer plumbing rather than agent affordances -- an agent never
  // invokes them; the giver's core invokes them on the receiver's core.
  if (!td.properties.getWorkspaceSignature) {
    td.properties.getWorkspaceSignature = {
      type: "object",
      description: "What another robot needs to decide whether this one could reach the same place: where its base is, and how far it can reach from there. safeReachRadius is the value to test against -- reachRadius is the extreme of a finite sample and understates the true reach.",
      properties: {
        robot: { type: "string" },
        basePose: { type: "object" },
        reachRadius: { type: "number" },
        safeReachRadius: { type: "number" },
        spacing: { type: "number" },
        sampleCount: { type: "number" },
        localBounds: { type: "object" }
      },
      forms: [{ href: `${hrefBase}/properties/getWorkspaceSignature`, "htv:methodName": "GET" }]
    };
  }
  if (!td.properties.getSceneObjects) {
    td.properties.getSceneObjects = {
      type: "object",
      description: "Every object this robot can see in the cell, with the kind it was discovered to be: self, peer_robot, payload, support or fixture.",
      properties: {
        discovery: { type: "string" },
        objects: {
          type: "array",
          items: {
            type: "object",
            properties: {
              alias: { type: "string" },
              kind: { type: "string" },
              shapes: { type: "number" },
              ignored: { type: "boolean" }
            }
          }
        }
      },
      forms: [{ href: `${hrefBase}/properties/getSceneObjects`, "htv:methodName": "GET" }]
    };
  }
  // Already-generated TDs carry the sampled bounding box on getCartesianposition,
  // which rejects the arm's own pose whenever it stands outside what the sampler
  // happened to reach. Robot3 parked at 1.076 m against a declared minimum of
  // 0.917 m is the case that found this. Relaxed as the TD loads, so a robot
  // generated before the fix does not need a pipeline re-run.
  for (const axis of ["x", "y", "z"]) {
    const reported = td.properties?.getCartesianposition?.properties?.[axis];
    if (reported) {
      delete reported.minimum;
      delete reported.maximum;
    }
  }
  // An approach can be aimed at a support other than this robot's own, which is
  // what placing onto the goal support is. Declared here rather than in the
  // generated TD so an already generated robot accepts it without a re-run.
  if (td.actions?.setPickupApproach?.input?.properties && !td.actions.setPickupApproach.input.properties.site) {
    td.actions.setPickupApproach.input.properties.site = {
      type: "string",
      description: "Alias of the support this approach is aimed at. Omitted means this robot's own pick support."
    };
  }
  if (!td.properties.getTaskSites) {
    td.properties.getTaskSites = {
      type: "object",
      description: "The cube supports standing in the cell, each with the point a payload would be gripped at, whether THIS robot can reach that point, and which payload -- if any -- is currently resting on it.",
      properties: {
        robot: { type: "string" },
        sites: {
          type: "array",
          items: {
            type: "object",
            properties: {
              alias: { type: "string" },
              world: { type: "object" },
              local: { type: "object" },
              grasp: { type: "object" },
              reachable: { type: "boolean" },
              admission: { type: "string" },
              occupiedBy: { type: "string" }
            }
          }
        },
        payloads: { type: "array", items: { type: "object" } }
      },
      forms: [{ href: `${hrefBase}/properties/getTaskSites`, "htv:methodName": "GET" }]
    };
  }
  if (!td.actions.acceptTransfer) {
    td.actions.acceptTransfer = {
      description: "Physically accept the cube into this robot's gripper. Invoked by the giving robot's core, not by an agent.",
      output: {
        type: "object",
        properties: {
          success: { type: "boolean" },
          reason: { type: "string" },
          distance: { type: "number" },
          tolerance: { type: "number" }
        }
      },
      forms: [{ href: `${hrefBase}/actions/acceptTransfer`, "htv:methodName": "POST" }]
    };
  }
  // The way out of a rendezvous, which for a long time was a plain Cartesian
  // move and therefore was dispatched as another approach. Declared here rather
  // than in the generated TD so an already generated robot accepts it without a
  // pipeline re-run.
  td.actions.returnHomeAfterHandover = {
    description: "Return the giver home while releasing vacated regions between short checked segments.",
    input: { type: "object", properties: {
      session: { type: "string" }, homeJoints: { type: "array", items: { type: "number" } }
    }, required: ["session", "homeJoints"] },
    output: { type: "object", properties: { success: { type: "boolean" } } },
    forms: [{ href: `${hrefBase}/actions/returnHomeAfterHandover`, "htv:methodName": "POST" }]
  };
  td.actions.checkHandoverClearance = {
    description: "Check whether this robot has cleared its current handover region.",
    input: { type: "object", properties: { session: { type: "string" } }, required: ["session"] },
    output: { type: "object", properties: { success: { type: "boolean" }, clear: { type: "boolean" } } },
    forms: [{ href: `${hrefBase}/actions/checkHandoverClearance`, "htv:methodName": "POST" }]
  };
  if (!td.actions.retreatFromHandover) {
    td.actions.retreatFromHandover = {
      description: "Leave the rendezvous along the approach the arm came in on, played backwards to the standoff it started from.",
      output: {
        type: "object",
        properties: {
          success: { type: "boolean" },
          state: { type: "string" },
          reason: { type: "string" },
          planner: { type: "string" },
          retreatStates: { type: "number" },
          standoffDistance: { type: "number" }
        }
      },
      forms: [{ href: `${hrefBase}/actions/retreatFromHandover`, "htv:methodName": "POST" }]
    };
  }
  if (!td.actions.registerHandoverSession) {
    td.actions.registerHandoverSession = {
      description: "Store the giver's handover session record in this robot's own registry.",
      input: {
        type: "object",
        properties: { session: { type: "object" } },
        required: ["session"]
      },
      output: { type: "object", properties: { success: { type: "boolean" }, session: { type: "string" } } },
      forms: [{ href: `${hrefBase}/actions/registerHandoverSession`, "htv:methodName": "POST" }]
    };
  }
  if (!td.actions.updateHandoverSession) {
    td.actions.updateHandoverSession = {
      description: "Apply a state change to a handover session this robot already holds.",
      input: {
        type: "object",
        properties: { session: { type: "string" }, changes: { type: "object" } },
        required: ["session"]
      },
      output: { type: "object", properties: { success: { type: "boolean" }, reason: { type: "string" } } },
      forms: [{ href: `${hrefBase}/actions/updateHandoverSession`, "htv:methodName": "POST" }]
    };
  }
  if (!td.actions.closeHandoverSession) {
    td.actions.closeHandoverSession = {
      description: "Drop this robot's replica of a handover session the giver has abandoned.",
      input: {
        type: "object",
        properties: { session: { type: "string" } },
        required: ["session"]
      },
      output: { type: "object", properties: { success: { type: "boolean" }, session: { type: "string" } } },
      forms: [{ href: `${hrefBase}/actions/closeHandoverSession`, "htv:methodName": "POST" }]
    };
  }
  return td;
}

// The interaction cloud is scene furniture, not robot state, so it is drawn
// from the sandbox script rather than from any one arm's driver. Redrawing it
// is what makes a recomputed shared space visible: the version baked into the
// shared .ttt is only ever as fresh as the last pipeline run.
async function refreshInteractionVisual(ctx, sim, payload) {
  const pairKey = String(payload.pairKey || "");
  const alias = `interaction_${pairKey}`.replace(/[^A-Za-z0-9_]/g, "_");
  const voxelSize = Number(payload.voxelSize);
  if (!pairKey || !Number.isFinite(voxelSize) || voxelSize <= 0) {
    return { success: false, reason: "invalid_interaction_visual_request", object: alias };
  }
  const csvPath = path.resolve(ctx.repoRoot, String(payload.csv || ""));
  if (!fs.existsSync(csvPath)) {
    return { success: false, reason: "interaction_csv_missing", object: alias };
  }
  const color = Array.isArray(payload.color) && payload.color.length === 3
    ? payload.color.map((value) => Math.max(0, Math.min(255, Math.round(Number(value) || 0))))
    : [0, 255, 0];
  const sandboxHandle = await sandboxScriptHandle(sim);
  const result = await executeInScript(
    sim,
    sandboxHandle,
    [
      `local csvPath=${luaString(csvPath)}`,
      `local alias=${luaString(alias)}`,
      "local existing=sim.getObject('/'..alias, {noError=true})",
      "if existing ~= -1 then pcall(sim.removeObject, existing) end",
      "local file=io.open(csvPath, 'r')",
      "if file == nil then return false, 'interaction_csv_unreadable' end",
      "local points={}",
      "local first=true",
      "for line in file:lines() do",
      "    if first then first=false else",
      "        local x,y,z=string.match(line, '([^,]+),([^,]+),([^,]+)')",
      "        if z ~= nil then",
      "            points[#points+1]=tonumber(x)",
      "            points[#points+1]=tonumber(y)",
      "            points[#points+1]=tonumber(z)",
      "        end",
      "    end",
      "end",
      "file:close()",
      "if #points == 0 then return true,0 end",
      `local cloud=sim.createPointCloud(${voxelSize},1,0,4)`,
      "sim.setObjectAlias(cloud,alias)",
      `sim.insertPointsIntoPointCloud(cloud,0,points,${luaArray(color)})`,
      "return true,#points/3"
    ].join("\n")
  );
  const ok = Array.isArray(result) ? result[0] : result;
  if (!ok) {
    return {
      success: false,
      reason: Array.isArray(result) ? String(result[1] || "interaction_visual_failed") : "interaction_visual_failed",
      object: alias
    };
  }
  return {
    success: true,
    object: alias,
    pointCount: Array.isArray(result) ? Number(result[1]) : 0
  };
}

async function startThing(
  ctx,
  sim,
  scriptHandle,
  arbiter = new LocalArbiter(),
  peers = null,
  sessions = new HandoverSessions(ctx.manifest.name)
) {
  const td = ensureSharedCellAffordances(readJson(ctx.paths.td), ctx);
  const options = createRuntimeOptions(ctx);
  const workspace = loadWorkspaceGrid(ctx, options);
  const motionLog = motionLogFor(ctx);
  const sharedRegions = () => options.enableSpatialClaims === false ? [] : loadSharedRegions(ctx);
  // Shared regions are in world coordinates; a commanded point is robot-local.
  const placement = ctx.manifest.sharedEnvironment || {};
  const worldTargetOf = (point) => {
    const offset = placement.offset || [0, 0, 0];
    const orientation = placement.orientationDeg || [0, 0, Number(placement.rotationDeg || 0)];
    const yaw = Number(orientation[2] || 0) * Math.PI / 180;
    return {
      x: Number(offset[0]) + Math.cos(yaw) * point.x - Math.sin(yaw) * point.y,
      y: Number(offset[1]) + Math.sin(yaw) * point.x + Math.cos(yaw) * point.y,
      z: Number(offset[2]) + point.z
    };
  };
  const port = Number(ctx.manifest.ports.lowerThing);
  const servient = new Servient();
  servient.addServer(new HttpServer({ port }));
  const WoT = await servient.start();
  const thing = await WoT.produce(td);
  let localMotionActive = false;
  // A mirror of the driver's womatPickupApproachEnabled.
  //
  // Read once per LEG by the motion below, and a place manoeuvre is dozens of
  // legs, so asking the driver was dozens of websocket round trips for a value
  // only the setPickupApproach handler further down can change. It starts
  // false because that is what the driver initialises the flag to, and a
  // driver reload resets both sides together.
  let pickupApproachActive = false;
  const departureZones = new Map();
  // Only a handover that has already happened arms this.
  //
  // The reserved-path route exists to let the two arms LEAVE a rendezvous at
  // the same time. Nothing about arriving at one is concurrent: the giver is
  // parked holding the cube while the receiver comes in, and the receiver is
  // parked while the giver presents it. Arming the route from the moment a
  // session was registered therefore bought no concurrency and charged the
  // approach, the descent and the grasp the whole cost of it -- offline
  // geometry, a dense preflight and a re-timed replay -- on the one leg that
  // most needs to look and feel like a straight reach. State "transferred" is
  // the line: before it the cube has not crossed and the arms take turns
  // anyway; after it they are entangled and both want out.
  const motionZones = () => zoneSettings(ctx.manifest)
    ? [...sessions.records.values()].filter((record) => record.state === "transferred")
      .map((record) => record.handoverZone).concat([...departureZones.values()])
      .filter((zone) => validZone(zone, ctx.manifest.name)) : [];

  // A session may end in BDI before the receiver's first departure. Retain its
  // zone for that one motion, including a direct move into the next handover.
  async function runHandoverMotion(action, operation, target = null) {
    const zones = motionZones();
    const fullClaim = () => {
      const regions = sharedRegions();
      return { regions: regions.map((region) => region.pairKey), voxels: regions.flatMap((region) => region.allKeys()) };
    };
    if (!zones.length || !options.simpleSmoothMotion || !options.enableInMotionCollisionCheck) {
      return runExclusiveMotion(action, () => operation(options), target);
    }
    if (localMotionActive) return { success: false, reason: "motion_busy", activeRobot: ctx.manifest.name };
    let result;
    const departures = [...departureZones.keys()];
    // A pick has its own multi-leg execution, and stays coarse until every leg
    // of it can be reserved before the first moves.
    //
    // Coarse means exactly what a cell with no handover zones at all does:
    // one ordinary claim computed from where this arm actually is and where it
    // is going. It used to mean a refusal dressed up as a failed proposal,
    // which reached the fallback below and locked EVERY shared region this
    // robot takes part in -- for each of the manoeuvre's legs, over a pick
    // happening at a task site none of the neighbours can reach.
    //
    // A PLACE is one leg, and takes the reserved route below like any other
    // known path. Coarse there meant the claim was region-sized because the
    // arm was still standing in the region it had just received in: the whole
    // of it, held until the cube was down, with the giver waiting to go home.
    if (pickupApproachActive && await executeInScript(sim, scriptHandle,
      "return heldGripObject ~= nil and heldGripObject ~= -1") !== true) {
      result = await runExclusiveMotion(action, () => operation(options), target);
      if (result.success) departures.forEach((id) => departureZones.delete(id));
      return result;
    }
    localMotionActive = true;
    try {
      const regions = sharedRegions();
      console.log(`[LowerThing:${ctx.manifest.name}] ${action} using handover zones with swept-path reservations`, zones);
      const pathClaim = handoverPathClaims(ctx.manifest.name, regions, zones, options.claimBodyRadius + 0.03);
      const occupancyClaim = handoverPathClaims(ctx.manifest.name, regions, zones, options.claimBodyRadius, { occupancy: true });
      const executeReservedPath = async (flat, dof, requireJointTarget = false) => {
        const current = asArray(await executeInScript(sim, scriptHandle, "return womatCurrentConfig()")).map(Number);
        if (!Number.isInteger(dof) || dof <= 0 || !flat.length || flat.length % dof
          || current.length !== dof || !current.every(Number.isFinite) || !flat.every(Number.isFinite)) {
          return { success: false, reason: "invalid_reserved_path", safeToReplan: true };
        }
        const dt = await simulationTimeStep(sim, scriptHandle);
        const step = options.simpleJointVelocityDeg * Math.PI / 180 * dt;
        const path = [current];
        for (let offset = 0; offset < flat.length; offset += dof) {
          const next = flat.slice(offset, offset + dof);
          const from = path[path.length - 1];
          // A reserved path is played one sample per simulation step, so any
          // sample further than one step's travel from the last is split. The
          // tolerance is not cosmetic: a trajectory timed to this exact
          // velocity limit sits at a ratio of precisely 1 through its whole
          // cruise, and a bare ceil() rounded every one of those steps up to
          // two -- halving the speed over the fastest part of the move, which
          // is most of it. A 180deg leg came out 35% longer for nothing.
          const travel = Math.max(...next.map((value, i) => Math.abs(value - from[i])));
          const steps = Math.max(1, Math.ceil(travel / step - 1e-9));
          for (let i = 1; i <= steps; i++) path.push(from.map((value, j) => value + (next[j] - value) * i / steps));
        }
        return continuousHome(sim, scriptHandle, ctx.manifest.name, path[path.length - 1],
          { ...options, reservedPath: path, requireJointTarget, pathClaim, occupancyClaim, handoverZones: zones }, arbiter, () => regions, motionLog);
      };
      result = await operation({ ...options, executeReservedPath });
    } finally { localMotionActive = false; }
    if (result.safeToReplan) {
      // A failed proposal has executed no movement. An unknown replacement
      // path must reacquire whole rooms; never reuse a smaller reservation.
      console.log(`[LowerThing:${ctx.manifest.name}] ${action} falling back to full-region locking: ${result.reason || "unsupported_motion"}`);
      result = await runExclusiveMotion(action, () => operation(options), target, fullClaim);
    }
    if (result.success) departures.forEach((id) => departureZones.delete(id));
    return result;
  }

  // What this motion will occupy of the space other arms can also reach.
  //
  // Asked of the driver, because only the scene knows where the links go: the
  // arm's current posture plus, when the motion is a joint path, the postures it
  // will pass through. A failure to compute the claim is answered with "the
  // whole of my shared regions", which refuses concurrency rather than
  // permitting an unchecked motion.
  async function claimForMotion(action, target = null) {
    const regions = sharedRegions();
    if (regions.length === 0) return { voxels: [], contested: false, regions: [] };
    try {
      const current = await executeInScript(sim, scriptHandle, "return womatCurrentFramePositions()");
      const points = asArray(current?.points).map(Number);
      if (points.length === 0) throw new Error("no frame positions reported");
      return conservativeClaim(ctx.manifest.name, regions, [points], {
        bodyRadius: options.claimBodyRadius,
        reachMargin: options.claimReachMargin,
        target
      });
    } catch (error) {
      console.warn(
        `[LowerThing:${ctx.manifest.name}] could not compute a motion claim (${error?.message || error}); `
        + "claiming every shared region this robot takes part in."
      );
      return {
        voxels: regions.flatMap((region) => [...region.voxelKeys].map((key) => `${region.pairKey}:${key}`)),
        contested: true,
        regions: regions.map((region) => region.pairKey),
        degraded: true
      };
    }
  }

  async function runExclusiveMotion(action, operation, target = null, claimedPath = null) {
    const entry = motionLog.begin(action);
    if (localMotionActive) {
      entry.refused({ reason: "motion_busy", activeRobot: ctx.manifest.name });
      entry.finished({ success: false, reason: "motion_busy" });
      return { success: false, reason: "motion_busy", activeRobot: ctx.manifest.name };
    }
    const claim = claimedPath ? await claimedPath() : await claimForMotion(action, target);
    const granted = await arbiter.acquire(ctx.manifest.name, claim.voxels);
    if (!granted.granted) {
      entry.refused({
        reason: granted.reason || "motion_busy",
        activeRobot: granted.activeRobot,
        claimedVoxels: claim.voxels.length,
        conflictVoxel: granted.conflictVoxel || null
      });
      entry.finished({ success: false, reason: granted.reason || "motion_busy" });
      return {
        success: false,
        reason: granted.reason || "motion_busy",
        activeRobot: granted.activeRobot || ctx.manifest.name,
        conflictVoxel: granted.conflictVoxel || null,
        conflictRegion: granted.conflictVoxel?.split(":")[0] || null
      };
    }
    localMotionActive = true;
    entry.granted({
      activeRobot: ctx.manifest.name,
      claimedVoxels: claim.voxels.length,
      contestedRegions: claim.regions,
      claimDegraded: Boolean(claim.degraded),
      concurrentArms: granted.concurrentArms || 1
    });
    try {
      const result = await operation();
      entry.finished(result || {});
      return result;
    } catch (error) {
      const message = error?.message || String(error);
      console.error(`[LowerThing:${ctx.manifest.name}] motion action failed:`, message);
      entry.finished({ success: false, reason: "lower_motion_exception" });
      return {
        success: false,
        reason: "lower_motion_exception",
        message
      };
    } finally {
      localMotionActive = false;
      await arbiter.release(ctx.manifest.name);
    }
  }

  const gripSetup = await executeInScript(sim, scriptHandle, "return womatSafeEnsureGripObjects()");
  if (!gripSetup?.success) {
    console.warn("Generated grip scene setup did not complete:", gripSetup);
  } else if (gripSetup.gripperSuccess === false) {
    console.warn("Generated gripper is not installed in the workspace scene. Re-run robot:pipeline --full-pipeline.", gripSetup);
  }

  thing.setPropertyReadHandler("getJointposition", async () => {
    return executeInScript(sim, scriptHandle, "return getJointposition()");
  });

  thing.setPropertyReadHandler("getCartesianposition", async () => {
    return executeInScript(sim, scriptHandle, "return getCartesianposition()");
  });

  if (td.properties.getGripperState) {
    thing.setPropertyReadHandler("getGripperState", async () => {
      return executeInScript(sim, scriptHandle, "return getGripperState()");
    });
  }

  if (td.properties.getBasePose) {
    thing.setPropertyReadHandler("getBasePose", async () => {
      return executeInScript(sim, scriptHandle, "return womatBasePose()");
    });
  }

  if (td.properties.getCollisionObstacles) {
    thing.setPropertyReadHandler("getCollisionObstacles", async () => {
      return executeInScript(sim, scriptHandle, "return womatSceneObstacleAliases()");
    });
  }

  if (td.properties.getWorkspaceSignature) {
    thing.setPropertyReadHandler("getWorkspaceSignature", async () => {
      const basePose = await executeInScript(sim, scriptHandle, "return womatBasePose()");
      // The cloud is named and fingerprinted, never shipped. 35k points are not
      // a belief and not a message; the path says which cloud, and the digest
      // says which VERSION of it, so two agents can confirm they computed their
      // shared space from the same data without either of them sending it.
      const csv = ctx.paths.robotDataPoints;
      return {
        robot: ctx.manifest.name,
        basePose,
        ...workspace.signature(),
        cloud: {
          path: path.relative(ctx.repoRoot, csv),
          digest: reachableDigestShared(csv).slice(0, 16),
          sampleCount: workspace.pointCount,
          spacing: workspace.voxelSize
        }
      };
    });
  }

  if (td.properties.getSceneObjects) {
    thing.setPropertyReadHandler("getSceneObjects", async () => {
      return executeInScript(sim, scriptHandle, "return womatSceneObjectInventory()");
    });
  }

  if (td.properties.getTaskSites) {
    thing.setPropertyReadHandler("getTaskSites", async () => {
      const inventory = await executeInScript(sim, scriptHandle, "return womatSceneObjectInventory()");
      // Reachability is the one part of this the scene cannot answer: it is a
      // property of THIS arm's sampled workspace, which lives here and not in
      // the simulator. Everything else -- what each object is, where it stands,
      // how big it is -- comes from the scene as it actually is.
      const classified = classifyTaskSites(
        (inventory?.objects || []).map((entry) => ({
          alias: entry.alias,
          kind: entry.kind,
          world: entry.world,
          local: entry.localPosition,
          size: entry.size
        })),
        {
          admits: workspace ? (point) => checkWorkspace(point, workspace, options) : null,
          payloadHalfHeight: Number(ctx.manifest.runtime?.gripper?.objectSize?.[2] || 0.045) / 2,
          fallbackSize: ctx.manifest.runtime?.gripper?.supportSize,
          worldUpLocal: options.worldUpLocal,
          hoverHeight: Number(ctx.manifest.runtime?.gripper?.pickupApproachHeight ?? 0.2)
        }
      );
      return taskSitesPayload(ctx.manifest.name, classified);
    });
  }

  if (td.properties.getCubeOwnership) {
    thing.setPropertyReadHandler("getCubeOwnership", async () => ({
      object: peers?.cube || ctx.manifest.runtime?.gripper?.objectName || `${ctx.manifest.name}_grip_cube`,
      owner: peers ? await peers.resolveCubeOwner() : "none",
      transferActive: sessions.transferInFlight
    }));
  }

  thing.setActionHandler("moveTojointPosition", async (params) => {
    const payload = await params.value();
    const radians = jointPayloadToRadians(payload, td);
    return runHandoverMotion("moveTojointPosition", (motionOptions) => options.simpleSmoothMotion
      ? simpleSmoothJointMove(sim, scriptHandle, td, radians, motionOptions)
      : smoothJointMove(sim, scriptHandle, td, radians, motionOptions));
  });

  thing.setActionHandler("moveTocartesianPosition", async (params) => {
    const payload = await params.value();
    const point = {
      x: Number(payload.x),
      y: Number(payload.y),
      z: Number(payload.z)
    };
    return runHandoverMotion("moveTocartesianPosition", (motionOptions) => options.simpleSmoothMotion
      ? simpleSmoothCartesianMove(sim, scriptHandle, point, workspace, motionOptions)
      : smoothCartesianMove(sim, scriptHandle, point, workspace, motionOptions), worldTargetOf(point));
  });

  if (td.actions.closeGripper) {
    thing.setActionHandler("closeGripper", async () => {
      // Refuse only if another arm is demonstrably holding it right now. The
      // scene answers that; the variable this replaces could still claim an
      // owner after a grip had slipped, and refuse a legitimate re-pick.
      const holder = peers ? await peers.resolveCubeOwner() : "none";
      if (holder !== "none" && holder !== ctx.manifest.name) {
        return { success: false, reason: "cube_owned_by_other_robot", owner: holder };
      }
      // Nothing is recorded on success: closing the gripper on the cube parents
      // it to this robot, and that parenting is the ownership.
      return executeInScript(sim, scriptHandle, "return womatSafeCall(closeGripper)");
    });
  }

  if (td.actions.openGripper) {
    thing.setActionHandler("openGripper", async () => {
      return executeInScript(sim, scriptHandle, "return womatSafeCall(openGripper)");
    });
  }

  if (td.actions.transferCube) {
    thing.setActionHandler("transferCube", async (params) => {
      const payload = await params.value();
      return transferCubeOwnership(
        ctx,
        sim,
        scriptHandle,
        peers,
        sessions,
        String(payload.session || ""),
        String(payload.targetRobot || "")
      );
    });
  }

  if (td.actions.beginHandoverSession) {
    thing.setActionHandler("beginHandoverSession", async (params) => {
      const payload = await params.value();
      return beginHandoverSession(
        ctx,
        sim,
        scriptHandle,
        peers,
        sessions,
        String(payload.session || ""),
        String(payload.targetRobot || ""),
        payload.cubeWorld
      );
    });
  }

  if (td.actions.setHandoverApproach) {
    thing.setActionHandler("setHandoverApproach", async (params) => {
      const payload = await params.value();
      const record = sessions.get(String(payload.session || ""));
      const result = await setHandoverApproach(
        ctx,
        sim,
        scriptHandle,
        peers,
        sessions,
        String(payload.session || ""),
        payload.enabled === true
      );
      if (result.success && payload.enabled !== true && record?.targetRobot === ctx.manifest.name
        && record.state === "transferred" && validZone(record.handoverZone, ctx.manifest.name)) {
        departureZones.set(record.id, record.handoverZone);
      }
      return result;
    });
  }

  thing.setActionHandler("returnHomeAfterHandover", async (params) => {
    const payload = await params.value();
    const session = sessions.get(String(payload.session || ""));
    if (!session || session.sourceRobot !== ctx.manifest.name || session.state !== "transferred") {
      return { success: false, reason: "handover_session_missing_or_mismatched" };
    }
    const home = asArray(payload.homeJoints).map(Number);
    if (!home.length || home.length !== Object.keys(td.actions.moveTojointPosition.input.properties).length
      || !home.every(Number.isFinite)) return { success: false, reason: "invalid_home_joints" };
    if (localMotionActive) return { success: false, reason: "motion_busy" };
    const radians = home.map((value) => value * Math.PI / 180);
    localMotionActive = true;
    let result;
    try {
      const regions = sharedRegions();
      const zones = motionZones();
      const homeOptions = zones.length && options.enableInMotionCollisionCheck
        ? { ...options,
          pathClaim: handoverPathClaims(ctx.manifest.name, regions, zones, options.claimBodyRadius + 0.03),
          occupancyClaim: handoverPathClaims(ctx.manifest.name, regions, zones, options.claimBodyRadius, { occupancy: true }), handoverZones: zones } : options;
      result = await continuousHome(sim, scriptHandle, ctx.manifest.name, radians,
        homeOptions, arbiter, () => regions, motionLog);
    } finally { localMotionActive = false; }
    if (!result.safeToReplan) return result;
    // If the direct home sweep is blocked, retain the established
    // complete-home planner as recovery. Its unknown detour reserves all rooms.
    return runExclusiveMotion("giverHomeRecovery", () => options.simpleSmoothMotion
      ? simpleSmoothJointMove(sim, scriptHandle, td, radians, options)
      : smoothJointMove(sim, scriptHandle, td, radians, options), null, () => {
        const regions = sharedRegions();
        return { regions: regions.map((region) => region.pairKey), voxels: regions.flatMap((region) => region.allKeys()) };
      });
  });

  thing.setActionHandler("checkHandoverClearance", async (params) => {
    const payload = await params.value();
    const session = sessions.get(String(payload.session || ""));
    if (!session || session.sourceRobot !== ctx.manifest.name) {
      return { success: false, clear: false, reason: "handover_session_missing_or_mismatched" };
    }
    try {
      const regions = sharedRegions().filter((region) => region.includes(session.targetRobot));
      const current = await executeInScript(sim, scriptHandle, "return womatCurrentFramePositions()");
      return handoverClearance(ctx.manifest.name, regions, asArray(current?.points).map(Number), options);
    } catch (error) {
      return { success: false, clear: false, reason: "handover_clearance_unavailable" };
    }
  });

  if (td.actions.retreatFromHandover) {
    thing.setActionHandler("retreatFromHandover", async () => {
      return runHandoverMotion(
        "retreatFromHandover",
        (motionOptions) => handoverRetreat(sim, scriptHandle, workspace, motionOptions)
      );
    });
  }

  if (td.actions.acceptTransfer) {
    thing.setActionHandler("acceptTransfer", async () => {
      return executeInScript(sim, scriptHandle, "return womatSafeCall(acceptTransferredGripObject)");
    });
  }

  if (td.actions.registerHandoverSession) {
    thing.setActionHandler("registerHandoverSession", async (params) => {
      const payload = await params.value();
      const record = payload?.session;
      if (!record?.id) return { success: false, reason: "handover_session_record_missing" };
      sessions.open(record);
      return { success: true, session: String(record.id), targetRobot: ctx.manifest.name };
    });
  }

  if (td.actions.updateHandoverSession) {
    thing.setActionHandler("updateHandoverSession", async (params) => {
      const payload = await params.value();
      const updated = sessions.update(String(payload?.session || ""), payload?.changes || {});
      return updated
        ? { success: true, session: String(payload.session), targetRobot: ctx.manifest.name }
        : { success: false, reason: "handover_session_missing_or_mismatched" };
    });
  }

  if (td.actions.closeHandoverSession) {
    thing.setActionHandler("closeHandoverSession", async (params) => {
      const payload = await params.value();
      const session = String(payload?.session || "");
      // Dropping a replica that is already gone is the normal outcome of a
      // receiver that tore its own copy down first, not an error.
      sessions.close(session);
      departureZones.delete(session);
      return { success: true, session, targetRobot: ctx.manifest.name };
    });
  }

  if (td.actions.refreshInteractionVisual) {
    thing.setActionHandler("refreshInteractionVisual", async (params) => {
      const payload = await params.value();
      return refreshInteractionVisual(ctx, sim, payload || {});
    });
  }

  if (td.actions.setPickupApproach) {
    thing.setActionHandler("setPickupApproach", async (params) => {
      const payload = await params.value();
      // The site is optional and defaults to this robot's own pick support, so
      // an existing caller that only passes `enabled` still means "a pick".
      const site = typeof payload.site === "string" && payload.site.length > 0 ? payload.site : null;
      const response = await executeInScript(
        sim,
        scriptHandle,
        `return womatSafeCall(function() return womatSetPickupApproach(`
        + `${payload.enabled === true ? "true" : "false"}, ${site ? luaString(site) : "nil"}) end)`
      );
      // The driver reports the flag it ended up with, which is the only value
      // worth mirroring: a call that failed inside womatSafeCall changed
      // nothing, and answering it by assuming the requested value would leave
      // the mirror describing a driver state that never happened.
      if (typeof response?.enabled === "boolean") pickupApproachActive = response.enabled;
      return response;
    });
  }

  if (options.enableSpatialClaims !== false) {
    console.log("Spatial claims: shared regions are loaded before each motion, including regions created later by agents.");
  } else {
    console.log("Spatial claims: disabled by runtime configuration.");
  }
  if (motionLog.enabled) {
    console.log(`Motion log: ${motionLog.file} (run "${motionLog.run}").`);
  }
  await thing.expose();
  // node-wot lowercases the route it serves the Thing on, so printing the title
  // verbatim hands out a URL that 404s.
  console.log(`Lower-level Thing exposed at http://localhost:${port}/${String(td.title).toLowerCase()}`);
  console.log(`Runtime motion: ${options.cartesianSteps} cartesian steps, ${options.jointSteps} joint steps, ${options.stepDelayMs}ms delay.`);
  console.log(`Runtime motion limits: ${options.cartesianMaxVelocity}m/s cartesian, ${options.jointMaxVelocityDeg}deg/s joint.`);
  console.log(`Runtime safety: workspace tolerance ${options.workspaceTolerance === null ? `${options.workspaceToleranceSpacingFactor}x sampled spacing` : `${options.workspaceTolerance}m`}, collision check ${options.enableCollisionCheck ? "enabled" : "disabled"}, joint preflight ${options.enableJointCollisionPreflight ? "enabled" : "disabled"}.`);
  console.log(`Runtime safety: swept preflight per command at ${options.preflightStepSize}m, in-motion watchdog ${options.enableInMotionCollisionCheck ? `every ${options.inMotionCollisionStride} steps` : "disabled"}.`);
  console.log(`Runtime planning: ${options.enablePathPlanning ? `OMPL RRTConnect fallback enabled (${options.planningMaxTimeSeconds}s budget, ${options.planningStateCount} states)` : "disabled"}.`);
  if (options.simpleSmoothMotion) {
    console.log("Runtime motion strategy: smooth motion with collision-checked joint routing for configured Cartesian moves.");
  }
  if (options.preferJointTrajectoryForCartesian) {
    console.log("Runtime Cartesian strategy: collision-checked uniform joint trajectory; shortened OMPL fallback only when blocked.");
  } else if (options.preferOmplForCartesian) {
    console.log("Runtime Cartesian strategy: collision-checked planned motion before visible direct IK.");
  }
  if (options.preferBaseJointFirst) {
    console.log(`Runtime staged motion: collision-checked joint 1 first (${options.baseJointFirstMinDeltaDeg}deg minimum), then remaining joints; OMPL fallback retained.`);
  }
  console.log(`Runtime path tracking: ${options.plannedJointMaxVelocityDeg}deg/s planned reference, ${options.pathSmoothingPasses} smoothing passes, ${options.pathTrackingToleranceDeg}deg final tolerance.`);
}

function handoverClearance(robot, regions, points, options) {
  if (!regions.length || points.length < 6 || points.length % 3 || !points.every(Number.isFinite)) {
    return { success: false, clear: false, reason: "handover_clearance_unavailable" };
  }
  const occupied = claimFor(robot, regions, [points], { bodyRadius: options.claimBodyRadius });
  return { success: true, clear: !occupied.contested };
}

module.exports = {
  handoverClearance,
  ensureSharedCellAffordances,
  refreshInteractionVisual,
  startThing
};
