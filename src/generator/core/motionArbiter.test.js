"use strict";

const assert = require("node:assert/strict");
const test = require("node:test");

const { ArbiterServer, LocalArbiter, RegionArbiter, RemoteArbiter } = require("./motionArbiter");

test("the cell moves one arm at a time, and a refusal names the holder", async () => {
  const arbiter = new LocalArbiter();
  assert.deepEqual(await arbiter.acquire("Robot1"), { granted: true, activeRobot: "Robot1" });
  assert.deepEqual(await arbiter.acquire("Robot2"), { granted: false, activeRobot: "Robot1" });
  // Naming the holder is what lets a caller tell "busy" from "broken".
  assert.deepEqual(await arbiter.acquire("Robot1"), { granted: false, activeRobot: "Robot1" });
  await arbiter.release("Robot1");
  assert.deepEqual(await arbiter.acquire("Robot2"), { granted: true, activeRobot: "Robot2" });
});

test("a robot cannot release a lock it does not hold", async () => {
  const arbiter = new LocalArbiter();
  await arbiter.acquire("Robot1");
  await arbiter.release("Robot2");
  assert.equal(arbiter.activeRobot, "Robot1");
});

test("cores in separate processes contend for the same lock over the host", async () => {
  const server = await new ArbiterServer(0, { mode: "exclusive" }).start();
  const port = server.server.address().port;
  const first = new RemoteArbiter({ port });
  const second = new RemoteArbiter({ port });
  try {
    assert.equal((await first.acquire("Robot1")).granted, true);
    const refused = await second.acquire("Robot3");
    assert.equal(refused.granted, false);
    assert.equal(refused.activeRobot, "Robot1");
    await first.release("Robot1");
    assert.equal((await second.acquire("Robot3")).granted, true);
  } finally {
    await server.stop();
  }
});

test("a core that cannot reach the arbiter is refused, never assumed free", async () => {
  // Assuming the lock is free because the question could not be asked is the
  // one failure mode the whole mechanism exists to prevent: it would put two
  // arms in motion against preflights that each assumed the other was still.
  const unreachable = new RemoteArbiter({
    port: 1,
    fetchImpl: async () => { throw new Error("ECONNREFUSED"); }
  });
  const result = await unreachable.acquire("Robot1");
  assert.equal(result.granted, false);
  assert.equal(result.reason, "arbiter_unreachable");
});

test("the host can release the lock for a core that died holding it", async () => {
  const server = await new ArbiterServer(0).start();
  try {
    await server.arbiter.acquire("Robot2");
    assert.equal(server.arbiter.activeRobot, "Robot2");
    // Nothing else can: the process that would have released it is gone.
    await server.releaseFor("Robot2");
    assert.equal(server.arbiter.activeRobot, null);
  } finally {
    await server.stop();
  }
});

test("two arms whose motions share no space both move", async () => {
  // This is the whole of Phase 4. Most motions touch no voxel any other arm can
  // reach, so most motions never need permission from anyone.
  const arbiter = new RegionArbiter();
  const first = await arbiter.acquire("Robot1", []);
  const second = await arbiter.acquire("Robot3", []);
  assert.equal(first.granted, true);
  assert.equal(second.granted, true);
  assert.deepEqual(arbiter.activeRobots.sort(), ["Robot1", "Robot3"]);
});

test("two arms wanting the same voxel do not", async () => {
  const arbiter = new RegionArbiter();
  assert.equal((await arbiter.acquire("Robot1", ["Robot1__Robot2:3,4,5"])).granted, true);
  const refused = await arbiter.acquire("Robot2", ["Robot1__Robot2:3,4,5", "Robot1__Robot2:3,4,6"]);
  assert.equal(refused.granted, false);
  assert.equal(refused.reason, "region_claimed");
  assert.equal(refused.activeRobot, "Robot1");
  assert.equal(refused.conflictVoxel, "Robot1__Robot2:3,4,5");
  // Disjoint claims inside the SAME shared region still both proceed: the lock
  // is on space, not on the region as a unit.
  assert.equal((await arbiter.acquire("Robot3", ["Robot1__Robot2:9,9,9"])).granted, true);
});

test("a claim released frees exactly the space it held", async () => {
  const arbiter = new RegionArbiter();
  await arbiter.acquire("Robot1", ["p:1,1,1"]);
  assert.equal((await arbiter.acquire("Robot2", ["p:1,1,1"])).granted, false);
  await arbiter.release("Robot1");
  assert.equal((await arbiter.acquire("Robot2", ["p:1,1,1"])).granted, true);
});

test("a claim nobody released expires, so a lost core cannot wedge the cell", async () => {
  let clock = 1000;
  const arbiter = new RegionArbiter({ leaseMs: 500, now: () => clock });
  await arbiter.acquire("Robot1", ["p:1,1,1"]);
  assert.equal((await arbiter.acquire("Robot2", ["p:1,1,1"])).granted, false);
  clock += 501;
  assert.equal((await arbiter.acquire("Robot2", ["p:1,1,1"])).granted, true);
});

test("one arm still cannot start a second motion while its first is running", async () => {
  const arbiter = new RegionArbiter();
  assert.equal((await arbiter.acquire("Robot1", [])).granted, true);
  const second = await arbiter.acquire("Robot1", []);
  assert.equal(second.granted, false);
  assert.equal(second.reason, "motion_busy");
});

test("claims carry across processes, and the state endpoint names every mover", async () => {
  const server = await new ArbiterServer(0).start();
  const port = server.server.address().port;
  const one = new RemoteArbiter({ port });
  const two = new RemoteArbiter({ port });
  try {
    assert.equal((await one.acquire("Robot1", ["p:1,1,1"])).granted, true);
    assert.equal((await two.acquire("Robot2", ["p:1,1,1"])).granted, false);
    assert.equal((await two.acquire("Robot2", ["p:2,2,2"])).granted, true);
    const state = await (await globalThis.fetch(`http://127.0.0.1:${port}/state`)).json();
    assert.equal(state.mode, "region");
    assert.deepEqual(state.activeRobots.sort(), ["Robot1", "Robot2"]);
  } finally {
    await server.stop();
  }
});

test("a running arm atomically sheds space without allowing expansion or a second motion", async () => {
  const arbiter = new RegionArbiter();
  await arbiter.acquire("A", ["pair:1", "pair:2"]);
  assert.equal((await arbiter.narrow("A", ["pair:2"])).updated, true);
  assert.equal((await arbiter.acquire("B", ["pair:1"])).granted, true);
  assert.equal((await arbiter.acquire("C", ["pair:2"])).granted, false);
  assert.equal((await arbiter.acquire("A", [])).granted, false);
  assert.equal((await arbiter.narrow("A", ["pair:1", "pair:2"])).reason, "motion_claim_expansion");
  assert.equal((await arbiter.narrow("missing", [])).reason, "motion_claim_missing");
});

test("claim narrowing works through the host for isolated cores", async () => {
  const server = await new ArbiterServer(0).start();
  try {
    const remote = new RemoteArbiter({ port: server.server.address().port });
    await remote.acquire("A", ["p:1", "p:2"]);
    assert.equal((await remote.narrow("A", ["p:2"])).updated, true);
    assert.equal((await remote.acquire("B", ["p:1"])).granted, true);
    assert.equal((await remote.acquire("C", ["p:2"])).granted, false);
  } finally { await server.stop(); }
});

test("narrowing retains exclusivity in the baseline mode", async () => {
  const arbiter = new LocalArbiter();
  await arbiter.acquire("A");
  assert.equal((await arbiter.narrow("A", [])).updated, true);
  assert.equal((await arbiter.acquire("B")).granted, false);
});

test("an ordered claim is granted behind an overlapping holder and told what it still holds", async () => {
  const arbiter = new RegionArbiter();
  assert.equal((await arbiter.acquire("giver", ["A__B:1", "A__B:2", "A__B:3"], { root: 11 })).granted, true);
  // A plain claim still refuses any overlap: an unknown path cannot follow.
  assert.equal((await arbiter.acquire("picker", ["A__B:3"])).reason, "region_claimed");
  const follower = await arbiter.acquire("receiver", ["A__B:2", "A__B:3", "A__B:9"], { ordered: true, root: 22 });
  assert.equal(follower.granted, true);
  assert.equal(follower.ordered, true);
  assert.deepEqual(follower.blockedBy, { giver: ["A__B:2", "A__B:3"] });
  assert.equal(follower.holders.giver.root, 11);
  // The leader shrinks as it leaves, and the follower is told on its next poll.
  await arbiter.narrow("giver", ["A__B:3"]);
  const polled = await arbiter.narrow("receiver", ["A__B:2", "A__B:3", "A__B:9"]);
  assert.deepEqual(polled.blockedBy, { giver: ["A__B:3"] });
  await arbiter.release("giver");
  assert.deepEqual((await arbiter.narrow("receiver", ["A__B:3", "A__B:9"])).blockedBy, {});
});

test("an ordered claim never waits on a claim granted after it, so no two claims wait on each other", async () => {
  const arbiter = new RegionArbiter();
  await arbiter.acquire("first", ["R:1", "R:2"], { ordered: true });
  const second = await arbiter.acquire("second", ["R:2", "R:3"], { ordered: true });
  assert.deepEqual(second.blockedBy, { first: ["R:2"] });
  // The earlier claim is blocked by nobody, however the later one overlaps it.
  assert.deepEqual((await arbiter.narrow("first", ["R:1", "R:2"])).blockedBy, {});
  // A holder that finished and claimed again is a new motion: a new sequence.
  const before = second.holders.first.sequence;
  await arbiter.release("first");
  const again = await arbiter.acquire("first", ["R:7"], { ordered: true });
  assert.notEqual(again.holders.first.sequence, before);
});

test("the cell host forwards ordering and roots, and an exclusive host grants plainly", async () => {
  const server = await new ArbiterServer(0).start();
  const port = server.server.address().port;
  try {
    const remote = new RemoteArbiter({ port });
    await remote.acquire("giver", ["R:1"], { ordered: true, root: 5 });
    const follower = await remote.acquire("receiver", ["R:1", "R:2"], { ordered: true, root: 6 });
    assert.equal(follower.ordered, true);
    assert.deepEqual(follower.blockedBy, { giver: ["R:1"] });
    assert.equal(follower.holders.giver.root, 5);
  } finally { await server.stop(); }
  const exclusive = await new ArbiterServer(0, { mode: "exclusive" }).start();
  try {
    const remote = new RemoteArbiter({ port: exclusive.server.address().port });
    const grant = await remote.acquire("giver", ["R:1"], { ordered: true, root: 5 });
    assert.equal(grant.granted, true);
    assert.equal(grant.ordered, undefined, "nobody to follow under one-arm-at-a-time");
  } finally { await exclusive.stop(); }
});
