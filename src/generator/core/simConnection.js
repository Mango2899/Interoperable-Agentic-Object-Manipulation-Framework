"use strict";

// The CoppeliaSim websocket connection, and running a call inside a scene
// script. Every module that touches the simulator goes through here.

const { delay, resolveFromRobwot } = require("./context");

async function connect(ctx) {
  const { RemoteAPIClient } = resolveFromRobwot("coppelia-ws-api", ctx.robwotRoot);
  const client = new RemoteAPIClient("localhost", 23050, "json");
  try {
    await client.websocket.open();
    const sim = await client.getObject("sim");
    return { client, sim };
  } catch (error) {
    throw new Error(`Could not connect to CoppeliaSim websocket remote API at localhost:23050: ${error.message || error}`);
  }
}

async function stopSimulation(sim) {
  const state = Number(await sim.getSimulationState());
  if (state !== Number(await sim.simulation_stopped)) {
    await sim.stopSimulation();
  }
  for (let attempt = 0; attempt < 50; attempt += 1) {
    if (Number(await sim.getSimulationState()) === Number(await sim.simulation_stopped)) {
      return;
    }
    await delay(200);
  }
  throw new Error("CoppeliaSim simulation did not stop in time.");
}

async function executeInScript(sim, scriptHandle, code) {
  const result = await sim.executeScriptString(code, scriptHandle);
  if (!Array.isArray(result)) {
    return result;
  }
  return result.length > 2 ? result.slice(1) : result[1];
}

// The scene's step, read once per connection.
//
// It is fixed for a run, and every reserved path asks for it: the route
// proposal, the executor that resamples it and the executor that plays it, each
// paying a websocket round trip the simulator services from its own loop. Three
// round trips for one unchanging number, on the very leg a giver takes while a
// receiver waits for the rendezvous.
const simulationTimeSteps = new WeakMap();

async function simulationTimeStep(sim, scriptHandle) {
  if (!simulationTimeSteps.has(sim)) {
    const value = Number(await executeInScript(sim, scriptHandle, "return sim.getSimulationTimeStep()"));
    simulationTimeSteps.set(sim, Number.isFinite(value) && value > 0 ? value : 0.05);
  }
  return simulationTimeSteps.get(sim);
}

// The scene handle other robots name this one by, read once per driver. Keyed
// by script as well as by connection, because one process can serve several
// robots over the same connection and each has its own root.
const sceneRoots = new WeakMap();

async function sceneRootHandle(sim, scriptHandle) {
  if (!sceneRoots.has(sim)) sceneRoots.set(sim, new Map());
  const roots = sceneRoots.get(sim);
  if (!roots.has(scriptHandle)) {
    const value = Number(await executeInScript(sim, scriptHandle, "return womatSceneRootHandle()"));
    if (!Number.isInteger(value) || value < 0) return null;
    roots.set(scriptHandle, value);
  }
  return roots.get(scriptHandle);
}

async function sandboxScriptHandle(sim) {
  const handle = await sim.getScript(await sim.scripttype_sandboxscript);
  return Array.isArray(handle) ? handle[0] : handle;
}

module.exports = {
  connect,
  executeInScript,
  sandboxScriptHandle,
  sceneRootHandle,
  simulationTimeStep,
  stopSimulation
};
