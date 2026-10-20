#!/usr/bin/env node
"use strict";

// One robot's movement and safety core, as its own process.
//
// The cell host has already loaded the scene, spawned every arm into it,
// attached the driver scripts and started the simulation -- all of that is
// cell-level work and none of it belongs to one robot. What is left, and what
// this process owns end to end, is one arm: its own connection to CoppeliaSim,
// its own driver script, its own workspace grid and preflight, its own gripper
// and its own handover sessions.
//
// Script handles are scene-global integers, so the host passes this process the
// handle it already attached rather than re-attaching it. Sibling robots are
// reached over their lower Things, never by reaching into their scripts through
// this process's connection -- that would put one robot's core back inside
// another's, which is the thing being undone here.

const { CellPeers, HandoverSessions, localPeer, remotePeer } = require("../cellPeers");
const { createContext } = require("./context");
const { connect, executeInScript } = require("./simConnection");
const { RemoteArbiter } = require("./motionArbiter");
const { startThing } = require("./thing");

function parseCoreConfig(value) {
  const config = JSON.parse(value);
  for (const key of ["manifest", "scriptHandle", "arbiterPort", "peers"]) {
    if (config[key] === undefined) throw new Error(`Robot core config is missing ${key}`);
  }
  return config;
}

async function runRobotCore(config) {
  const ctx = createContext(config.manifest);
  const name = ctx.manifest.name;
  const { sim } = await connect(ctx);
  const scriptHandle = Number(config.scriptHandle);

  const sessions = new HandoverSessions(name);
  const peers = new CellPeers({ cube: config.cube || null });
  // This robot is its own peer, served locally: a core reads its own gripper
  // down its own connection rather than over HTTP to itself.
  peers.register(name, localPeer({
    name,
    manifest: ctx.manifest,
    invoke: (code) => executeInScript(sim, scriptHandle, code),
    sessions
  }));
  for (const peer of config.peers) {
    if (peer.name === name) continue;
    peers.register(peer.name, remotePeer({
      name: peer.name,
      manifest: peer.manifest,
      url: peer.url
    }));
  }

  const arbiter = new RemoteArbiter({ port: config.arbiterPort });
  await startThing(ctx, sim, scriptHandle, arbiter, peers, sessions);
  console.log(
    `[RobotCore:${name}] serving its own movement and safety core `
    + `(script ${scriptHandle}, ${config.peers.length - 1} peer core(s)).`
  );
  return { ctx, sim, scriptHandle, peers, sessions, arbiter };
}

async function main() {
  const index = process.argv.indexOf("--core-config");
  if (index === -1 || !process.argv[index + 1]) {
    console.error("robotCore is started by the cell host; it needs --core-config <json>.");
    process.exit(1);
  }
  await runRobotCore(parseCoreConfig(process.argv[index + 1]));
}

if (require.main === module) {
  main().catch((error) => {
    console.error(error);
    process.exit(1);
  });
}

module.exports = { parseCoreConfig, runRobotCore };
