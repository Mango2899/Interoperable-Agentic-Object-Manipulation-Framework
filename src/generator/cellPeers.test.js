"use strict";

const assert = require("node:assert/strict");
const test = require("node:test");

const { CellPeers, HandoverSessions, localPeer, remotePeer } = require("./cellPeers");
const { transferCubeOwnership } = require("./lowerThingServer");

// A cell whose grippers answer from a mutable scene, so a test can move the
// cube the way a transfer does and watch ownership follow it.
function scene(names, holder = null) {
  const state = { holder, unreachable: new Set(), accepted: [] };
  const peers = new CellPeers({ cube: "grip_cube" });
  const sessions = new Map(names.map((name) => [name, new HandoverSessions(name)]));
  // A robot's own core reads its own gripper down its own connection rather
  // than through the address book, so the simulator stub answers from the same
  // scene the peers do.
  const invokeFor = {};
  for (const name of names) {
    invokeFor[name] = async (code) => {
      if (state.unreachable.has(name)) throw new Error(`${name} is not answering`);
      if (/getGripperState/.test(String(code))) {
        return { holding: state.holder === name, object: "grip_cube" };
      }
      if (/acceptTransferredGripObject/.test(String(code))) {
        state.accepted.push(name);
        state.holder = name;
        return { success: true, distance: 0.01, tolerance: 0.05 };
      }
      if (/relinquishTransferredGripObject/.test(String(code))) return { success: true };
      return { success: true };
    };
    peers.register(name, localPeer({
      name,
      invoke: invokeFor[name],
      manifest: { name, runtime: { gripper: { modelName: `${name}_tcp_gripper` } } },
      sessions: sessions.get(name)
    }));
  }
  const simFor = (name) => ({
    executeScriptString: async (code) => [1, await invokeFor[name](code)]
  });
  return { state, peers, sessions, simFor, sessionsFor: (name) => sessions.get(name) };
}

test("the cube's owner is whichever gripper is holding it, not a recorded value", async () => {
  const { state, peers } = scene(["Robot1", "Robot2", "Robot3"], "Robot2");
  assert.equal(await peers.resolveCubeOwner(), "Robot2");
  // Nothing publishes this change; the scene simply is different now.
  state.holder = "Robot3";
  assert.equal(await peers.resolveCubeOwner(), "Robot3");
  state.holder = null;
  assert.equal(await peers.resolveCubeOwner(), "none");
});

test("an arm that cannot be reached is reported, not counted as empty-handed", async () => {
  const { state, peers } = scene(["Robot1", "Robot2"], "Robot2");
  state.unreachable.add("Robot2");
  const ownership = await peers.resolveCubeOwnership();
  // Robot2 is in fact holding it. Answering "none" here without saying that one
  // arm never replied is how a second robot gets cleared to pick the same cube.
  assert.equal(ownership.owner, "none");
  assert.deepEqual(ownership.unreachable, ["Robot2"]);
});

test("two arms holding the cube is reported as contested rather than resolved", async () => {
  const peers = new CellPeers({ cube: "grip_cube" });
  for (const name of ["Robot1", "Robot2"]) {
    peers.register(name, localPeer({
      name, invoke: async () => ({ holding: true, object: "grip_cube" }), manifest: { name }
    }));
  }
  const ownership = await peers.resolveCubeOwnership();
  assert.equal(ownership.contested, true);
  assert.equal(ownership.owner, "none");
  assert.deepEqual(ownership.holders, ["Robot1", "Robot2"]);
});

test("a transfer moves ownership because it moves the cube, not because it records it", async () => {
  const { state, peers, sessionsFor, simFor } = scene(["Robot1", "Robot2"], "Robot1");
  const sessions = sessionsFor("Robot1");
  sessions.open({ id: "s1", sourceRobot: "Robot1", targetRobot: "Robot2", state: "registered" });
  const ctx = { manifest: { name: "Robot1" } };
  const result = await transferCubeOwnership(ctx, simFor("Robot1"), 1, peers, sessions, "s1", "Robot2");
  assert.equal(result.success, true);
  assert.deepEqual(state.accepted, ["Robot2"]);
  assert.equal(await peers.resolveCubeOwner(), "Robot2");
  // The lock is released whatever happened, so the next handover is not blocked
  // by the last one's bookkeeping.
  assert.equal(sessions.transferInFlight, false);
});

test("the transfer lock is the giver's, so unrelated pairs are not serialized by it", async () => {
  const first = new HandoverSessions("Robot1");
  const second = new HandoverSessions("Robot3");
  assert.equal(first.beginTransfer(), true);
  // The old shared flag refused this for no reason either robot could observe.
  assert.equal(second.beginTransfer(), true);
  assert.equal(first.beginTransfer(), false, "one giver still cannot run two transfers at once");
  first.endTransfer();
  assert.equal(first.beginTransfer(), true);
});

test("a giver cannot transfer a cube another arm is holding", async () => {
  const { peers, sessionsFor, simFor } = scene(["Robot1", "Robot2"], "Robot2");
  const sessions = sessionsFor("Robot1");
  sessions.open({ id: "s1", sourceRobot: "Robot1", targetRobot: "Robot2", state: "registered" });
  const ctx = { manifest: { name: "Robot1" } };
  const result = await transferCubeOwnership(ctx, simFor("Robot1"), 1, peers, sessions, "s1", "Robot2");
  assert.equal(result.success, false);
  assert.equal(result.reason, "source_not_holding_cube");
});

test("each robot's session registry is its own", () => {
  const { peers, sessionsFor } = scene(["Robot1", "Robot2"], "Robot1");
  const record = sessionsFor("Robot1").open({ id: "s1", sourceRobot: "Robot1", targetRobot: "Robot2" });
  assert.equal(sessionsFor("Robot2").get("s1"), null);
  return peers.openSessionOn("Robot2", record).then((replicated) => {
    assert.equal(replicated.success, true);
    assert.deepEqual(sessionsFor("Robot2").get("s1"), sessionsFor("Robot1").get("s1"));
    // Copies, not the same object: one robot mutating its record must not
    // silently rewrite the other's.
    assert.notEqual(sessionsFor("Robot2").get("s1"), sessionsFor("Robot1").get("s1"));
    sessionsFor("Robot1").update("s1", { state: "transferred" });
    assert.equal(sessionsFor("Robot2").get("s1").state, undefined);
  });
});

test("a peer in another process answers the same four operations", async () => {
  // The seam Phase 2 turns: the same operations, over HTTP to that robot's own
  // lower Thing instead of down a shared simulator connection.
  const seen = [];
  const peer = remotePeer({
    name: "Robot2",
    manifest: { name: "Robot2" },
    url: "http://localhost:8120/lower_robot2",
    fetchImpl: async (url, options) => {
      seen.push(`${options?.method || "GET"} ${url}`);
      return { ok: true, text: async () => JSON.stringify({ success: true, holding: false }) };
    }
  });
  await peer.gripperState();
  await peer.acceptTransfer();
  await peer.openSession({ id: "s1" });
  await peer.updateSession("s1", { state: "transferred" });
  assert.deepEqual(seen, [
    "GET http://localhost:8120/lower_robot2/properties/getGripperState",
    "POST http://localhost:8120/lower_robot2/actions/acceptTransfer",
    "POST http://localhost:8120/lower_robot2/actions/registerHandoverSession",
    "POST http://localhost:8120/lower_robot2/actions/updateHandoverSession"
  ]);
});

test("a core in another process that stops answering is unreachable, not empty-handed", async () => {
  const peers = new CellPeers({ cube: "grip_cube" });
  peers.register("Robot1", localPeer({
    name: "Robot1", invoke: async () => ({ holding: false }), manifest: { name: "Robot1" }
  }));
  peers.register("Robot2", remotePeer({
    name: "Robot2",
    manifest: { name: "Robot2" },
    url: "http://localhost:8120/lower_robot2",
    fetchImpl: async () => { throw new Error("ECONNREFUSED"); }
  }));
  const ownership = await peers.resolveCubeOwnership();
  assert.equal(ownership.owner, "none");
  assert.deepEqual(ownership.unreachable, ["Robot2"]);
});

test("a peer must implement every operation, not merely most of them", () => {
  const peers = new CellPeers({});
  assert.throws(
    () => peers.register("Robot2", { gripperState: async () => ({}), acceptTransfer: async () => ({}) }),
    /does not implement openSession/
  );
});
