"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const source = fs.readFileSync(path.resolve(__dirname, "core", "motion.js"), "utf8");

test("the approach axis that worked is tried first next time", () => {
  // A rejected axis is rejected by an IK solve that runs to its own timeout
  // before giving up. Live cell: Robot2 spent seven rejections to reach the
  // eighth candidate -- six of them a full findConfig budget on a seed that had
  // already diverged -- and then repeated the identical search, in the identical
  // order, for the identical answer on the very next approach.
  assert.match(source, /const rememberedApproachAxis = new Map\(\);/);
  assert.match(source, /const candidates = orderByRememberedAxis\(declared\);/);

  // Remembered against the DECLARED order, or the memo means something
  // different on the run after it is first used.
  assert.match(source, /const declaredIndex = declared\.findIndex/);
  assert.match(source, /rememberedApproachAxis\.set\(memoKey, declaredIndex\)/);

  // Keyed on the candidate list, so a different mode, partner or cone gets no
  // stale advice rather than the wrong advice.
  assert.match(source, /function approachAxisMemoKey\(candidates\)/);
  assert.match(source, /rememberedApproachAxis\.get\(approachAxisMemoKey\(candidates\)\)/);

  // Nothing is skipped: the remembered axis leads and the rest still follow, so
  // a stale memo costs one attempt and then behaves exactly as before.
  const order = source.slice(source.indexOf("function orderByRememberedAxis"), source.indexOf("// The handover rendezvous"));
  assert.match(order, /ordered\.unshift\(winner\)/);
  assert.doesNotMatch(order, /slice\(0,|length = 1/, "the memo reorders the search, it does not shorten it");
  // An out-of-range or absent memo leaves the declared order untouched.
  assert.match(order, /remembered === undefined \|\| remembered <= 0 \|\| remembered >= candidates\.length/);
});

test("a memo that was wrong is dropped rather than paid for again", () => {
  // If every axis failed, whatever was remembered is not the answer; left in
  // place it costs one wasted IK budget on every future approach.
  assert.match(source, /axisMemoCleared: rememberedApproachAxis\.delete\(memoKey\) \|\| undefined/);
  const failure = source.slice(source.indexOf("if (!plan) {"), source.indexOf("const standoff = asArray"));
  assert.match(failure, /rememberedApproachAxis\.delete/, "the clear belongs on the all-failed path");
});

test("the result says which order the winning axis came from", () => {
  // axisIndex is the position in the order actually tried; once the memo has
  // learned something that is no longer the position the mode declares, and
  // comparing two runs needs both.
  assert.match(source, /declaredAxisIndex: rememberedApproachAxis\.get\(memoKey\)/);
  assert.match(source, /axisTriedFirst: axisIndex === 0/);
});
