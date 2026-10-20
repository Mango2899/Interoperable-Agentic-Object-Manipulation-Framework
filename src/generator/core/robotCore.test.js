"use strict";

const assert = require("node:assert/strict");
const test = require("node:test");

const { parseCoreConfig } = require("./robotCore");
const { runtimeSource } = require("./runtimeSource");

test("a robot core is told which arm it owns and how to reach the others", () => {
  const config = parseCoreConfig(JSON.stringify({
    manifest: "generated/robots/Robot2/manifest.json",
    scriptHandle: 42,
    arbiterPort: 8199,
    cube: "Robot1_grip_cube",
    peers: [{ name: "Robot1", url: "http://localhost:8110/lower_robot1", manifest: {} }]
  }));
  // Script handles are scene-global, so the host passes the handle it already
  // attached rather than making each core re-attach its own driver.
  assert.equal(config.scriptHandle, 42);
  assert.equal(config.peers.length, 1);
  for (const missing of ["manifest", "scriptHandle", "arbiterPort", "peers"]) {
    const partial = {
      manifest: "m", scriptHandle: 1, arbiterPort: 2, peers: []
    };
    delete partial[missing];
    assert.throws(() => parseCoreConfig(JSON.stringify(partial)), new RegExp(missing));
  }
});

test("a core reaches its siblings over their Things, never through their scripts", () => {
  const source = runtimeSource();
  // The whole point of the split is that one robot's core is not inside
  // another's. A peer resolved with remotePeer goes over HTTP; reaching a
  // sibling's script handle through this process's connection would put it
  // straight back.
  assert.match(source, /peers\.register\(peer\.name, remotePeer\(\{/);
  assert.match(source, /if \(peer\.name === name\) continue;/);
});

test("the cell host keeps the scene and the lock, and forks a core per arm", () => {
  const source = runtimeSource();
  assert.match(source, /fork\([\s\S]{0,40}?robotCore\.js/);
  // A dead core still holds the motion lock and nothing else can release it.
  assert.match(source, /arbiterServer\.releaseFor\(name\)/);
  // Restarting an arm halfway through a planned trajectory would resume a cell
  // in a state nobody checked, so the host stops instead.
  assert.doesNotMatch(source, /restartCore|child\.respawn/);
});
