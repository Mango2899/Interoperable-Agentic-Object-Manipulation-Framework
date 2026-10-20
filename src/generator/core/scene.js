"use strict";

// Everything that changes the CoppeliaSim scene: loading it, spawning robots
// into it, attaching their driver scripts, and drawing their workspace clouds.

const fs = require("fs");

const { BASE_PEDESTAL_Z } = require("../sharedCell");
const { gripFromSite, loadedSite, resolveCellSites } = require("../cellSites");
const { luaString } = require("./luaDriver");
const { fail, uniqueItems } = require("./context");
const { executeInScript, sandboxScriptHandle, stopSimulation } = require("./simConnection");
const { prepareRuntimeDriverScript } = require("./driverScript");

async function removeWorkspaceVisuals(sim, robotHandle) {
  const script = [
    "local removed={pointClouds=0, hullShapes=0}",
    "local okType, pointCloudType=pcall(function() return sim.object_pointcloud_type end)",
    "if okType and pointCloudType ~= nil then",
    "    local clouds=sim.getObjectsInTree(sim.handle_scene, pointCloudType, 0)",
    "    for i=1,#clouds,1 do",
    "        local alias=sim.getObjectAlias(clouds[i], 4)",
    "        if string.match(alias, '^workspace_') or string.match(alias, '^collision_') then",
    "            if pcall(sim.removeObject, clouds[i]) then removed.pointClouds=removed.pointClouds+1 end",
    "        end",
    "    end",
    "end",
    `local shapes=sim.getObjectsInTree(${robotHandle}, sim.object_shape_type, 1)`,
    "for i=1,#shapes,1 do",
    "    local alias=sim.getObjectAlias(shapes[i], 4)",
    "    if alias == 'workspace_hull' then",
    "        if pcall(sim.removeObject, shapes[i]) then removed.hullShapes=removed.hullShapes+1 end",
    "    end",
    "end",
    "return removed"
  ].join("\n");
  const sandboxHandle = await sim.getScript(await sim.scripttype_sandboxscript);
  const scriptHandle = Array.isArray(sandboxHandle) ? sandboxHandle[0] : sandboxHandle;
  const removed = await executeInScript(sim, scriptHandle, script);
  console.log(`Workspace visuals disabled in manifest; removed: ${JSON.stringify(removed)}.`);
}

async function attachRuntimeDriver(ctx, sim, driverOptions = {}) {
  const objectHandleResult = await sim.getObject(`/${ctx.manifest.name}`);
  const objectHandle = Number(Array.isArray(objectHandleResult) ? objectHandleResult[0] : objectHandleResult);

  if (ctx.manifest.runtime?.workspaceVisuals === false) {
    await removeWorkspaceVisuals(sim, objectHandle);
  }

  const existing = await sim.getScript(await sim.scripttype_customizationscript, objectHandle, `/${ctx.manifest.name}`);
  if (existing[0] !== -1) {
    await sim.removeScript(existing[0]);
  }

  const scriptHandle = Number(await sim.addScript(await sim.scripttype_customizationscript));
  await sim.setScriptStringParam(scriptHandle, Number(await sim.scriptstringparam_text), prepareRuntimeDriverScript(ctx, driverOptions));
  await sim.associateScriptWithObject(scriptHandle, objectHandle);
  await sim.initScript(scriptHandle);
  return scriptHandle;
}

async function loadRuntimeScene(ctx, sim) {
  await stopSimulation(sim);
  await sim.loadScene(ctx.paths.scene);
  // The workspace scene auto-starts the simulation (needed for workspace generation).
  // Stop it so the runtime script's sysCall_init can run cleanly, then re-start it
  // after the script is in place. Simulation must be running for physics-based
  // collision response and for sysCall_actuation to drive motion coroutines.
  await stopSimulation(sim);

  const scriptHandle = await attachRuntimeDriver(ctx, sim);
  // Runtime failures are returned through the lower TD and logged. Do not
  // interrupt a presentation with CoppeliaSim's modal Abort execution dialog.
  try {
    await sim.setBoolProperty(await sim.handle_scene, "pauseSimulationAtError", false);
  } catch (error) {
    console.warn("Could not disable CoppeliaSim pause-on-error for this scene:", error?.message || error);
  }
  await sim.startSimulation();
  console.log(`Loaded runtime scene and driver for ${ctx.manifest.name}.`);
  return scriptHandle;
}


async function spawnAdditionalRobot(ctx, sim, offset, wallAlias, wallOffset) {
  if (!ctx.paths.model || !fs.existsSync(ctx.paths.model)) {
    fail(`Generated model for ${ctx.manifest.name} not found (run robot:pipeline --generate-model first): ${ctx.paths.model}`);
  }

  let robotHandle = await sim.loadModel(ctx.paths.model);
  robotHandle = Array.isArray(robotHandle) ? robotHandle[0] : robotHandle;
  await sim.setObjectPosition(robotHandle, await sim.handle_world, [offset[0], offset[1], BASE_PEDESTAL_Z + (offset[2] || 0)]);

  // Give the spawned robot its own copy of the demo wall obstacle, mirrored to
  // the far side of its base (position offset - original, yaw rotated by pi) so
  // the wall never lands inside the overlap region between the robots. The
  // manifest may displace it further in the robot's own frame -- see
  // --wall-offsets in the shared-cell pipeline, which uses the same convention.
  const [wallDx, wallDy, wallDz] = wallOffset || [0, 0, 0];
  const sandboxHandle = await sandboxScriptHandle(sim);
  const wallResult = await executeInScript(
    sim,
    sandboxHandle,
    [
      "local wall=sim.getObject('/80cmHighWall50cm', {noError=true})",
      "if wall == -1 then return false, 'wall_missing' end",
      `local existing=sim.getObject('/'..${luaString(wallAlias)}, {noError=true})`,
      "if existing ~= -1 then pcall(sim.removeObject, existing) end",
      "local tree=sim.getObjectsInTree(wall, sim.handle_all, 0)",
      "local copies=sim.copyPasteObjects(tree, 0)",
      "local newWall=copies[1]",
      `sim.setObjectAlias(newWall, ${luaString(wallAlias)})`,
      "local p=sim.getObjectPosition(wall, sim.handle_world)",
      `sim.setObjectPosition(newWall, sim.handle_world, {${offset[0]}-(p[1]+${wallDx}), ${offset[1]}-(p[2]+${wallDy}), p[3]+${offset[2] || 0}+${wallDz}})`,
      "local o=sim.getObjectOrientation(newWall, sim.handle_world)",
      "o[3]=o[3]+math.pi",
      "sim.setObjectOrientation(newWall, sim.handle_world, o)",
      "return true, newWall"
    ].join("\n")
  );
  const wallOk = Array.isArray(wallResult) ? wallResult[0] : wallResult;
  if (!wallOk) {
    console.warn(`Could not clone wall obstacle for ${ctx.manifest.name}:`, wallResult);
  }

  console.log(`Spawned ${ctx.manifest.name} model at offset [${offset.join(", ")}].`);
  return robotHandle;
}

async function installWorkspaceVisuals(ctx, sim, offset) {
  // Spawned robots come from their bare .ttm, which has no workspace visuals.
  // Rebuild both CSV classes plus the reachable convex hull, shifted by the
  // base offset, mirroring what robot_driver_workspace.lua does.
  const sandboxHandle = await sandboxScriptHandle(sim);
  const result = await executeInScript(
    sim,
    sandboxHandle,
    [
      `local csvPath=${luaString(ctx.paths.robotDataPoints)}`,
      `local robot=sim.getObject('/'..${luaString(ctx.manifest.name)}, {noError=true})`,
      "if robot == -1 then return false, 'robot_missing' end",
      "local file=io.open(csvPath, 'r')",
      "if file == nil then return false, 'csv_unreadable' end",
      "local points={}",
      "local collisionPoints={}",
      "local first=true",
      "for line in file:lines() do",
      "    if first then",
      "        first=false",
      "    else",
      "        local x,y,z,reachable=string.match(line, '([^,]+),([^,]+),([^,]+),([^,]+)')",
      "        if reachable ~= nil then",
      `            local shifted={tonumber(x)+${offset[0]}, tonumber(y)+${offset[1]}, tonumber(z)+${offset[2] || 0}}`,
      "            local destination=tonumber(reachable) == 1 and points or collisionPoints",
      "            destination[#destination+1]=shifted[1]",
      "            destination[#destination+1]=shifted[2]",
      "            destination[#destination+1]=shifted[3]",
      "        end",
      "    end",
      "end",
      "file:close()",
      "if #points == 0 then return false, 'no_reachable_points' end",
      "local ptcld=sim.createPointCloud(0.05,1,0,2)",
      `pcall(sim.setObjectAlias, ptcld, ${luaString(`workspace_${ctx.manifest.name}_reachable_points`)})`,
      "sim.insertPointsIntoPointCloud(ptcld,0,points)",
      "if #collisionPoints > 0 then",
      "    local collPtcld=sim.createPointCloud(0.05,1,0,2)",
      `    pcall(sim.setObjectAlias, collPtcld, ${luaString(`collision_${ctx.manifest.name}_workspace_points`)})`,
      "    sim.insertPointsIntoPointCloud(collPtcld,0,collisionPoints,{255,0,0})",
      "end",
      "local ok, vertices, indices=pcall(sim.getQHull, points)",
      "if not ok then return true, #points/3, #collisionPoints/3, 'hull_failed' end",
      "local shape=sim.createMeshShape(3,0,vertices,indices)",
      "pcall(sim.reorientShapeBoundingBox, shape, -1)",
      "sim.setShapeColor(shape,nil,0,{1,0,1})",
      "sim.setShapeColor(shape,nil,4,{0.2})",
      "pcall(sim.setObjectAlias, shape, 'workspace_hull')",
      "sim.setObjectParent(shape,robot,true)",
      "return true, #points/3, #collisionPoints/3, shape"
    ].join("\n")
  );

  const ok = Array.isArray(result) ? result[0] : result;
  if (!ok) {
    console.warn(`Could not create workspace visuals for ${ctx.manifest.name}:`, result);
    return;
  }
  const pointCount = Array.isArray(result) ? result[1] : null;
  const collisionPointCount = Array.isArray(result) ? result[2] : null;
  console.log(`Installed workspace visuals for ${ctx.manifest.name} (${pointCount} reachable, ${collisionPointCount} collision points).`);
}

function buildObstacleNames(contexts, wallAliases, index, sharedGrip) {
  const ctx = contexts[index];
  const ownSupport = ctx.manifest.runtime?.gripper?.supportName || `${ctx.manifest.name}_grip_support`;
  const own = ctx.manifest.runtime?.collisionObstacles || ctx.manifest.pipeline?.collisionObstacles || ["80cmHighWall50cm"];
  const mapped = own.map((name) => {
    if (name === "80cmHighWall50cm") {
      return wallAliases[index];
    }
    if (sharedGrip && name === ownSupport) {
      return sharedGrip.supportName;
    }
    return name;
  });
  const others = contexts.flatMap((other, otherIndex) => {
    if (otherIndex === index) {
      return [];
    }
    const otherGrip = other.manifest.runtime?.gripper || {};
    return [
      other.manifest.name,
      wallAliases[otherIndex],
      sharedGrip ? sharedGrip.supportName : (otherGrip.supportName || `${other.manifest.name}_grip_support`)
    ];
  });
  return uniqueItems([...mapped, ...others]);
}

function sharedGripConfig(ownerCtx, cellSites) {
  // A is the site that starts loaded, wherever the cell put it. Reading it from
  // the site list is the whole point: it holds in a cell whose scene owner does
  // not sit at the origin, and it lets A be moved without editing a robot.
  const loaded = loadedSite(cellSites || resolveCellSites([ownerCtx], [[0, 0, 0]]));
  if (loaded) return gripFromSite(loaded);
  // No site carries a payload, so the pick scene is still the owner's own grip
  // config -- whose coordinates are world coordinates only because the scene
  // owner sits at the origin. That assumption is what a cell-level site removes.
  const grip = ownerCtx.manifest.runtime?.gripper || {};
  return {
    objectName: grip.objectName || `${ownerCtx.manifest.name}_grip_cube`,
    supportName: grip.supportName || `${ownerCtx.manifest.name}_grip_support`,
    objectSize: grip.objectSize || [0.045, 0.045, 0.045],
    objectPosition: grip.objectPosition || [0.45, 0.2, 0.325],
    supportSize: grip.supportSize || [0.18, 0.18, 0.24],
    supportPosition: grip.supportPosition || [0.45, 0.2, 0.18]
  };
}

module.exports = {
  attachRuntimeDriver,
  buildObstacleNames,
  installWorkspaceVisuals,
  loadRuntimeScene,
  removeWorkspaceVisuals,
  sharedGripConfig,
  spawnAdditionalRobot
};
