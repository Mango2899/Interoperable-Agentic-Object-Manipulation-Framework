const assert = require("node:assert/strict");
const test = require("node:test");

const { CellPeers, HandoverSessions, localPeer } = require("./cellPeers");

// The chain that broke: Robot1 hands to Robot2, Robot2 hands the same cube on
// to Robot3. Robot2 is a receiver first and a giver second, and its own core is
// the one registry that sees both roles.

test("a rendezvous the giver abandoned does not stop the receiver handing the cube on", () => {
  const robot2 = new HandoverSessions("Robot2");

  // Robot1 announces a rendezvous, so Robot2 is given a replica...
  robot2.open({ id: "abandoned", sourceRobot: "Robot1", targetRobot: "Robot2", state: "registered" });
  // ...then picks a different point and never uses this one. Robot2 ran no
  // intention for it, so nothing on Robot2's side ever cleared it.

  assert.equal(
    robot2.blockingSession("Robot2"),
    null,
    "an announced-but-never-performed rendezvous describes nothing physical and must not block Robot2"
  );
});

test("a handover that physically happened still blocks a second one", () => {
  const robot2 = new HandoverSessions("Robot2");
  robot2.open({ id: "live", sourceRobot: "Robot1", targetRobot: "Robot2", state: "transferred" });
  // The cube has crossed; both arms are still entangled at the interaction
  // point. Robot2 turning giver here would move an arm mid-handover.
  assert.equal(robot2.blockingSession("Robot2")?.id, "live");
});

test("a robot's own handover blocks it from starting a second one", () => {
  const robot2 = new HandoverSessions("Robot2");
  robot2.open({ id: "mine", sourceRobot: "Robot2", targetRobot: "Robot3", state: "registered" });
  assert.equal(robot2.blockingSession("Robot2")?.id, "mine");
});

test("the giver takes the replica back when it walks away from a rendezvous", async () => {
  const robot2 = new HandoverSessions("Robot2");
  const peers = new CellPeers({ cube: "cube" });
  peers.register("Robot2", localPeer({
    name: "Robot2",
    manifest: { name: "Robot2" },
    invoke: async () => ({ success: true }),
    sessions: robot2
  }));

  robot2.open({ id: "abandoned", sourceRobot: "Robot1", targetRobot: "Robot2", state: "registered" });
  assert.ok(robot2.get("abandoned"), "precondition: Robot2 holds the replica");

  const result = await peers.closeSessionOn("Robot2", "abandoned");
  assert.equal(result.success, true);
  assert.equal(robot2.get("abandoned"), null, "the replica is gone from the receiver's registry");
});

test("taking back a replica the receiver already dropped is not an error", async () => {
  const robot2 = new HandoverSessions("Robot2");
  const peers = new CellPeers({ cube: "cube" });
  peers.register("Robot2", localPeer({
    name: "Robot2",
    manifest: { name: "Robot2" },
    invoke: async () => ({ success: true }),
    sessions: robot2
  }));
  // The receiver tears its own copy down at the end of a handover it performed,
  // so the giver's cleanup routinely arrives second.
  const result = await peers.closeSessionOn("Robot2", "never_there");
  assert.equal(result.success, true);
});
