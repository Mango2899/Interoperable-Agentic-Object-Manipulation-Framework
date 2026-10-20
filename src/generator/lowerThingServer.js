#!/usr/bin/env node
"use strict";

// Entry point for a robot's lower Thing.
//
// The runtime used to live here as one 5,500-line file with a 2,900-line Lua
// program inside a template literal. It is now src/generator/core (movement,
// safety, gripper, handover, scene, WoT wiring) and src/generator/lua (the
// driver itself). What is left is the command line and the two ways to start:
// one robot in its own scene, or a shared cell through the cell host.
//
// The re-exports below keep this module's public surface unchanged for the
// pipeline scripts and tests that import from it.

const { CellPeers, HandoverSessions, localPeer } = require("./cellPeers");
const {
  assertRuntimeInputs,
  createContext,
  fail,
  parseArgs,
  usage
} = require("./core/context");
const { connect, executeInScript } = require("./core/simConnection");
const { LocalArbiter } = require("./core/motionArbiter");
const { prepareRuntimeDriverScript } = require("./core/driverScript");
const { loadRuntimeScene } = require("./core/scene");
const { checkWorkspace, createRuntimeOptions, loadWorkspaceGrid } = require("./core/runtimeOptions");
const { baseJointFirstConfigs, jointConfigStates } = require("./core/motion");
const {
  beginHandoverSession,
  giverPresentAxisFor,
  handoverApproachMode,
  setHandoverApproach,
  transferCubeOwnership
} = require("./core/handover");
const { startThing } = require("./core/thing");
const { serveMultipleRobots } = require("./core/cellHost");
const { cellBaselineAxis, receiverApproachAxis, receiverApproachAxisCandidates } = require("./handoverGeometry");

async function main() {
  const args = parseArgs(process.argv.slice(2));
  if (args.help) {
    console.log(usage());
    return;
  }

  if (args.manifests) {
    const manifestFiles = String(args.manifests).split(",").map((part) => part.trim()).filter((part) => part.length > 0);
    if (manifestFiles.length < 2) {
      fail("--manifests needs at least two comma-separated manifest files (use --manifest for a single robot).");
    }
    await serveMultipleRobots(manifestFiles, args.offsets, {
      perRobotGrip: args["per-robot-grip"] === true,
      isolatedCores: args["isolated-cores"] === true,
      exclusiveMotion: args["exclusive-motion"] === true
    });
    return;
  }

  if (!args.manifest) {
    fail("Missing --manifest.");
  }

  const ctx = createContext(args.manifest);
  assertRuntimeInputs(ctx);
  const { sim } = await connect(ctx);
  const scriptHandle = await loadRuntimeScene(ctx, sim);
  // A cell of one. Its own gripper still answers who holds the cube, so a
  // single-robot run reports ownership the same way a shared cell does instead
  // of always answering "none".
  const peers = new CellPeers({ cube: ctx.manifest.runtime?.gripper?.objectName });
  const sessions = new HandoverSessions(ctx.manifest.name);
  peers.register(ctx.manifest.name, localPeer({
    name: ctx.manifest.name,
    invoke: (code) => executeInScript(sim, scriptHandle, code),
    manifest: ctx.manifest,
    sessions
  }));
  await startThing(ctx, sim, scriptHandle, new LocalArbiter(), peers, sessions);
}

if (require.main === module) {
  main().catch((error) => {
    console.error(error);
    process.exit(1);
  });
}

module.exports = {
  giverPresentAxisFor,
  transferCubeOwnership,
  setHandoverApproach,
  baseJointFirstConfigs,
  jointConfigStates,
  createContext,
  createRuntimeOptions,
  beginHandoverSession,
  cellBaselineAxis,
  handoverApproachMode,
  receiverApproachAxis,
  receiverApproachAxisCandidates,
  checkWorkspace,
  loadWorkspaceGrid,
  prepareRuntimeDriverScript,
  serveMultipleRobots,
  startThing
};
