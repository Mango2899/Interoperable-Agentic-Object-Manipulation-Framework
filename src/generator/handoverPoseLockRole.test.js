const assert = require("node:assert/strict");
const test = require("node:test");

const { handoverPoseLockEnabled } = require("./core/handover");

const ctx = (runtime) => ({ manifest: { name: "Robot3", runtime } });

test("the robot-level flag still answers both roles when nothing is split", () => {
  // Every existing robot sets only this flag; splitting the question must not
  // change what any of them do.
  assert.equal(handoverPoseLockEnabled(ctx({ enableHandoverPoseLock: true }), "giver"), true);
  assert.equal(handoverPoseLockEnabled(ctx({ enableHandoverPoseLock: true }), "receiver"), true);
  assert.equal(handoverPoseLockEnabled(ctx({ enableHandoverPoseLock: false }), "giver"), false);
  assert.equal(handoverPoseLockEnabled(ctx({ enableHandoverPoseLock: false }), "receiver"), false);
});

test("unset means pinned, as before", () => {
  assert.equal(handoverPoseLockEnabled(ctx({}), "giver"), true);
  assert.equal(handoverPoseLockEnabled(ctx({}), "receiver"), true);
});

test("a robot in the middle of a chain can be pinned as giver and free as receiver", () => {
  // Robot3 needs a repeatable presentation handing on to Robot4, and reached
  // the rendezvous from Robot2 reliably only while its own wrist was free.
  const middle = ctx({
    enableHandoverPoseLock: true,
    enableHandoverPoseLockAsReceiver: false
  });
  assert.equal(handoverPoseLockEnabled(middle, "giver"), true);
  assert.equal(handoverPoseLockEnabled(middle, "receiver"), false);
});

test("a per-role override wins over the robot default in either direction", () => {
  const pinnedReceiverOnly = ctx({
    enableHandoverPoseLock: false,
    enableHandoverPoseLockAsReceiver: true
  });
  assert.equal(handoverPoseLockEnabled(pinnedReceiverOnly, "giver"), false);
  assert.equal(handoverPoseLockEnabled(pinnedReceiverOnly, "receiver"), true);
});
