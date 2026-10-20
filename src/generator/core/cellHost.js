"use strict";

// The cell host: scene owner and supervisor.
//
// It loads the shared scene, spawns every robot into it, attaches their driver
// scripts and starts the simulation -- the parts of startup that belong to the
// cell rather than to any one arm. Each robot's own movement and safety core is
// then served from here.

const fs = require("fs");
const path = require("path");
const { fork } = require("child_process");

const { CellPeers, HandoverSessions, localPeer } = require("../cellPeers");
const { ArbiterServer, LocalArbiter, RegionArbiter } = require("./motionArbiter");
const {
  assertRuntimeInputs,
  createContext,
  fail,
  parseOffsets,
  uniqueItems
} = require("./context");
const { connect, executeInScript, stopSimulation } = require("./simConnection");
const { createRuntimeOptions } = require("./runtimeOptions");
const {
  attachRuntimeDriver,
  buildObstacleNames,
  installWorkspaceVisuals,
  sharedGripConfig,
  spawnAdditionalRobot
} = require("./scene");
const { resolveCellSites } = require("../cellSites");
const { startThing } = require("./thing");

// One port for the cell, not one per robot: there is one motion lock.
const DEFAULT_ARBITER_PORT = 8199;

async function serveMultipleRobots(manifestFiles, offsetsArg, options = {}) {
  const contexts = manifestFiles.map(createContext);
  contexts.forEach(assertRuntimeInputs);
  const names = contexts.map((ctx) => ctx.manifest.name);
  if (new Set(names).size !== names.length) {
    fail(`Robot names in --manifests must be unique: ${names.join(", ")}`);
  }

  const sharedScene = contexts[0].paths.sharedScene &&
    contexts.every((ctx) => ctx.paths.sharedScene === contexts[0].paths.sharedScene) &&
    fs.existsSync(contexts[0].paths.sharedScene)
    ? contexts[0].paths.sharedScene
    : null;
  const offsets = offsetsArg
    ? parseOffsets(offsetsArg, contexts.length)
    : contexts.map((ctx, index) => ctx.manifest.sharedEnvironment?.offset || [index * 2, 0, 0]);
  if (!sharedScene && offsets[0].some((value) => value !== 0)) {
    console.warn(`First robot ${names[0]} owns the scene and stays at the origin; ignoring its offset [${offsets[0].join(", ")}].`);
    offsets[0] = [0, 0, 0];
  }
  const wallAliases = sharedScene
    ? contexts[0].manifest.sharedEnvironment.wallAliases
    : contexts.map((ctx, index) => (index === 0 ? "80cmHighWall50cm" : `${ctx.manifest.name}_wall`));
  // Resolved before the pick scene, because the pick scene is one of them: the
  // site that starts loaded IS A, and reading it from the cell's list is what
  // stops A being a property of whichever robot happens to own the scene.
  const cellTaskSites = collectCellTaskSites(contexts, offsets);
  if (cellTaskSites.length > 0) {
    console.log(`Cell task sites: ${cellTaskSites.map((site) => `${site.alias} at [${site.position.map((v) => v.toFixed(3)).join(", ")}]${site.yawDeg ? ` turned ${site.yawDeg} deg` : ""}${site.payload ? ` holding ${site.payload.alias}` : " empty"}`).join("; ")}.`);
  }
  const sharedGrip = options.perRobotGrip ? null : sharedGripConfig(contexts[0], cellTaskSites);
  if (sharedGrip) {
    console.log(`Shared pick scene: single cube ${sharedGrip.objectName} at [${sharedGrip.objectPosition.join(", ")}] for all robots.`);
  }

  const { sim } = await connect(contexts[0]);
  await stopSimulation(sim);
  await sim.loadScene(sharedScene || contexts[0].paths.scene);
  await stopSimulation(sim);
  if (sharedScene) {
    console.log(`Loaded preassembled shared environment: ${sharedScene}`);
  }

  const robots = [];
  for (let index = 0; index < contexts.length; index += 1) {
    const ctx = contexts[index];
    const baseOffset = offsets[index];
    const orientationDeg = ctx.manifest.sharedEnvironment?.orientationDeg || [0, 0, Number(ctx.manifest.sharedEnvironment?.rotationDeg || 0)];
    const baseOrientation = orientationDeg.map((value) => Number(value || 0) * Math.PI / 180);
    const baseYaw = baseOrientation[2];
    if (!sharedScene && index > 0) {
      await spawnAdditionalRobot(ctx, sim, offsets[index], wallAliases[index], ctx.manifest.sharedEnvironment?.wallOffset);
      if (ctx.manifest.runtime?.workspaceVisuals !== false) {
        await installWorkspaceVisuals(ctx, sim, offsets[index]);
      }
    }
    const scriptHandle = await attachRuntimeDriver(ctx, sim, {
      baseOffset,
      baseYaw,
      baseOrientation,
      // Task sites belong in every arm's obstacle list, including the arm that
      // declared one. A support is a solid block whoever is next to it, and the
      // arm that will eventually place onto it is the one most likely to hit
      // it. The shared descriptor cannot carry them -- the pipeline rewrites
      // that list from the wall aliases and the shared support -- so they are
      // added here, where the list is actually assembled.
      obstacleNames: sharedScene
        ? uniqueItems([
          ...(ctx.manifest.sharedEnvironment?.collisionObstacles || []),
          ...cellTaskSites.map((site) => site.alias),
          ...contexts.filter((_, otherIndex) => otherIndex !== index).map((other) => other.manifest.name)
        ])
        : uniqueItems([
          ...buildObstacleNames(contexts, wallAliases, index, sharedGrip),
          ...cellTaskSites.map((site) => site.alias)
        ]),
      sharedGrip,
      taskSites: cellTaskSites
    });
    robots.push({
      ctx,
      scriptHandle,
      baseOffset,
      baseYaw,
      baseOrientation,
      options: createRuntimeOptions(ctx)
    });
  }

  try {
    await sim.setBoolProperty(await sim.handle_scene, "pauseSimulationAtError", false);
  } catch (error) {
    console.warn("Could not disable CoppeliaSim pause-on-error for this scene:", error?.message || error);
  }
  await sim.startSimulation();
  console.log(`Loaded shared runtime scene with ${contexts.length} robots: ${names.join(", ")}.`);

  // Every planner has the other robots in its obstacle collection, so a path
  // accepted by preflight stops being valid the moment another arm moves. That
  // is why the cell moves one arm at a time, and why the rule survives the
  // process split rather than being dropped with the object that held it.
  //
  // The scene is ready. From here the cell either serves every core from this
  // process or hands each one to its own, and the difference is confined to
  // these two calls: what a core does with its robot is identical either way.
  if (options.isolatedCores) {
    return superviseIsolatedCores(robots, sharedGrip, options);
  }
  return serveCoresInProcess(robots, sim, sharedGrip, options);
}

// Every core in the host process, sharing one simulator connection. Still the
// default: it needs no extra ports and is the easier thing to debug.
async function serveCoresInProcess(robots, sim, sharedGrip, options = {}) {
  // A shared object rather than an HTTP service, but the same two policies.
  const arbiter = options.exclusiveMotion ? new LocalArbiter() : new RegionArbiter();
  const peers = sharedGrip ? new CellPeers({ cube: sharedGrip.objectName }) : null;
  const sessionsByRobot = new Map(
    robots.map((robot) => [robot.ctx.manifest.name, new HandoverSessions(robot.ctx.manifest.name)])
  );
  if (peers) {
    for (const robot of robots) {
      const name = robot.ctx.manifest.name;
      peers.register(name, localPeer({
        name,
        manifest: robot.ctx.manifest,
        invoke: (code) => executeInScript(sim, robot.scriptHandle, code),
        sessions: sessionsByRobot.get(name)
      }));
    }
  }
  for (const robot of robots) {
    await startThing(
      robot.ctx,
      sim,
      robot.scriptHandle,
      arbiter,
      peers,
      sessionsByRobot.get(robot.ctx.manifest.name)
    );
  }
  console.log(
    `Serving ${robots.length} robot core(s) in the cell host process, `
    + `motion arbitration: ${options.exclusiveMotion ? "EXCLUSIVE (one arm at a time -- serialised baseline)" : "REGION (one arm per contested region)"}.`
  );
}

// One process per robot.
//
// The host keeps the scene and the motion lock -- the two things that belong to
// the cell rather than to an arm -- and nothing else. It does not proxy for the
// cores: once forked they talk to CoppeliaSim and to each other directly.
//
// A core that dies is not restarted. Restarting a core whose arm is halfway
// through a planned trajectory would resume a cell in a state nobody checked;
// the host releases that robot's motion lock so the others are not left waiting
// on a process that no longer exists, says plainly what happened, and stops.
async function superviseIsolatedCores(robots, sharedGrip, options) {
  const arbiterPort = Number(options.arbiterPort || DEFAULT_ARBITER_PORT);
  const mode = options.exclusiveMotion ? "exclusive" : "region";
  const arbiterServer = await new ArbiterServer(arbiterPort, { mode }).start();
  console.log(
    `Cell motion arbiter listening on 127.0.0.1:${arbiterPort} in ${mode} mode`
    + `${mode === "exclusive" ? " (one arm at a time -- serialised baseline)" : " (one arm per contested region)"}.`
  );

  const directory = robots.map((robot) => ({
    name: robot.ctx.manifest.name,
    url: robot.ctx.manifest.lowerLayerUrl,
    manifest: robot.ctx.manifest
  }));

  const children = new Map();
  let shuttingDown = false;

  const shutdown = (reason, code) => {
    if (shuttingDown) return;
    shuttingDown = true;
    console.log(`Cell host shutting down: ${reason}`);
    for (const [name, child] of children) {
      if (child.exitCode === null && child.signalCode === null) {
        console.log(`  stopping ${name} core (pid ${child.pid})`);
        child.kill("SIGTERM");
      }
    }
    arbiterServer.stop().finally(() => process.exit(code));
  };

  for (const robot of robots) {
    const name = robot.ctx.manifest.name;
    const config = {
      manifest: robot.ctx.manifestPath,
      scriptHandle: robot.scriptHandle,
      arbiterPort,
      cube: sharedGrip ? sharedGrip.objectName : null,
      peers: directory
    };
    const child = fork(
      path.join(__dirname, "robotCore.js"),
      ["--core-config", JSON.stringify(config)],
      { stdio: ["ignore", "pipe", "pipe", "ipc"] }
    );
    const prefix = (stream, label) => {
      let buffered = "";
      stream.on("data", (chunk) => {
        buffered += chunk.toString();
        const lines = buffered.split("\n");
        buffered = lines.pop();
        for (const line of lines) console[label](`[${name}] ${line}`);
      });
    };
    prefix(child.stdout, "log");
    prefix(child.stderr, "error");
    child.on("exit", (code, signal) => {
      // The dead core still holds the motion lock; nothing else can release it.
      arbiterServer.releaseFor(name);
      if (shuttingDown) return;
      shutdown(`${name} core exited (${signal || `code ${code}`}); the cell cannot continue without it`, 1);
    });
    children.set(name, child);
    console.log(`Forked ${name} core (pid ${child.pid}) on script handle ${robot.scriptHandle}.`);
  }

  for (const signal of ["SIGINT", "SIGTERM"]) {
    process.on(signal, () => shutdown(`received ${signal}`, 0));
  }
  console.log(`Cell host supervising ${children.size} isolated robot core(s).`);
  return { arbiterServer, children };
}

// The cell's sites, in the cell's own coordinates.
//
// Kept as a named export because the whole cell reads it: a robot that only knew
// about its own sites could never answer "no, I do not have that one" about a
// neighbour's, and that answer is what a distributed search for a goal site is
// made of. resolveCellSites decides where the declaration lives -- the cell's
// list where there is one, each robot's own frame where there is not.
function collectCellTaskSites(contexts, offsets) {
  return resolveCellSites(contexts, offsets);
}

module.exports = {
  DEFAULT_ARBITER_PORT,
  collectCellTaskSites,
  serveCoresInProcess,
  serveMultipleRobots,
  superviseIsolatedCores
};
