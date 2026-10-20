"use strict";

// What happens to an arm whose own lower layer stops answering.
//
// There are two moments this can happen and they used to be handled as if only
// one of them existed. At startup, the neighbour scan and the task-site scan
// both fail, their handlers announce the arm out of service, and the cell routes
// around it. After startup -- the route already settled, the arm already part of
// it -- nothing announced anything: the arm kept calling a socket that was not
// there, and the rest of the cell went on holding a route through an arm that
// could no longer move.
//
// The artifact is the only thing that knows the socket is shut, so it is the
// thing that says so: once per outage, to the agent, plus a count of what it
// dropped in between.

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const artifact = fs.readFileSync(
  path.resolve(__dirname, "../env/wotCommunication/wotCommunicator.java"), "utf8");
const routePlans = fs.readFileSync(path.resolve(__dirname, "../agt/routePlans.asl"), "utf8");

// The else-branch of wotSend: everything that happens when the socket is shut.
const offline = artifact.slice(
  artifact.indexOf("droppedSends++;"),
  artifact.indexOf("@OPERATION", artifact.indexOf("droppedSends++;")));

test("a request that could not be sent fails the operation instead of reporting success", () => {
  // Returning normally made every caller wait out its own deadline for an answer
  // that could not come: 30s for the neighbour scan, 20s for task-site
  // detection, then 30s per neighbour that asked about a route.
  assert.match(offline, /failed\(/);
  assert.match(offline, /is not connected, so/);
  // Scope: wotAchieve always targets the arm's own layer, while agent-to-agent
  // traffic goes through Jason's .send, so this fails layer calls and nothing
  // else. If that ever stops being true, this assertion is the warning.
  assert.doesNotMatch(routePlans, /wotAchieve\("(?!ownLayer1)/);
});

test("a shut socket is one log line and a count, not one line per request", () => {
  // The MAS console is synchronous, so a dead layer printing per request slowed
  // down the arms that were still working -- the failure of one arm became a
  // tax on the whole cell.
  assert.match(offline, /if \(!layerDownReported\)/);
  assert.match(offline, /layerDownReported = true/);
  assert.match(offline, /droppedSends\+\+/);
  assert.match(offline, /droppedSends % \d+ == 0/, "and the rest are counted, so the volume stays visible");
  // Named, or the line cannot be attributed to an arm -- which is exactly the
  // position the first report of this left us in.
  assert.match(offline, /\[" \+ this\.Name \+ "\]/);
});

test("a layer that comes back is news, and the next outage is not suppressed as a repeat", () => {
  const onOpen = artifact.slice(artifact.indexOf("public void onOpen"), artifact.indexOf("public void onClose"));
  assert.match(onOpen, /layerDownReported = false/);
  assert.match(onOpen, /droppedSends = 0/);
  assert.match(onOpen, /reconnected after/);
});

test("the agent is told once, as a fact about itself rather than about the request", () => {
  assert.match(offline, /signal\("layerUnreachable", this\.Name, keyword\)/);
  // Inside the first-report branch, so an outage is one event and not thousands.
  assert.ok(offline.indexOf("layerDownReported = true") < offline.indexOf('signal("layerUnreachable"'));
  assert.ok(offline.indexOf('signal("layerUnreachable"') < offline.indexOf("} else if"));
});

test("an arm whose layer dies after startup reaches the same state as one that never had it", () => {
  // The whole point. cannotServe used to be reachable only from the two init
  // handlers, so an arm that failed at minute ten never got there -- and the
  // cell went on routing through it.
  assert.match(routePlans, /\+layerUnreachable\(Robot, Request\): true <-[\s\S]*?!announceUnavailable\(runtime\)/);
  // Which is the same announcement the init handlers make, so the consequences
  // are identical: ownLayerUnresponsive, hence cannotServe, hence an immediate
  // "no route" to anyone who asks and a broadcast so nobody asks again.
  assert.match(routePlans, /cannotServe :- ownLayerUnresponsive\(_\)\./);
  const announce = routePlans.slice(routePlans.indexOf("+!announceUnavailable(Step)"));
  const first = announce.slice(0, announce.indexOf("\n\n"));
  assert.match(first, /\+ownLayerUnresponsive\(Step\)/);
  assert.match(first, /\.broadcast\(tell, armUnavailable\(MyRobot\)\)/);
  // Once per arm, however many ways it finds out.
  assert.match(first, /not unavailabilityAnnounced/);
  assert.match(first, /\+unavailabilityAnnounced/);
});

test("the periodic report names the request being retried, not just the count", () => {
  // A count says an arm is looping. The keyword says what it is looping on, and
  // without it the only way to find out is to guess -- which is how this bug
  // survived three rounds of static reading.
  assert.match(artifact, /private final Map<String, Long> droppedByKeyword/);
  assert.match(artifact, /static String requestKeyword\(Object content\)/);
  assert.match(offline, /droppedByKeyword\.merge\(keyword, 1L, Long::sum\)/);
  assert.match(offline, /by request: " \+ droppedByKeyword/);
  // Cleared on reconnect, so the next outage's breakdown is its own.
  const onOpen = artifact.slice(artifact.indexOf("public void onOpen"), artifact.indexOf("public void onClose"));
  assert.match(onOpen, /droppedByKeyword\.clear\(\)/);
});

test("an arm that keeps calling a layer it has already given up on is stopped", () => {
  // The backstop, and it does not try to name the loop -- it ends it. Every plan
  // in the cell fails cleanly on its own and every retry is individually bounded,
  // and yet one arm still spun 95000 dropped requests deep and slowed down the
  // arms that were working, because nothing was watching the arm as a whole.
  //
  // The second signal is what distinguishes the two cases: the first says the
  // layer has gone, the second says the arm is still asking it anyway.
  assert.ok(offline.indexOf('signal("layerUnreachable"') > 0, "the first signal announces");
  const periodic = artifact.slice(artifact.indexOf("} else if (droppedSends %"));
  assert.match(periodic.slice(0, periodic.indexOf("failed(")), /signal\("layerUnreachable", this\.Name, keyword\)/,
    "and the periodic one repeats it, which is what the backstop reads");

  const backstop = routePlans.slice(routePlans.indexOf("+layerUnreachable(Robot, Request)\n    : cannotServe"));
  const body = backstop.slice(0, backstop.indexOf("\n\n"));
  assert.match(body, /\.drop_all_intentions/);
  // Guarded on the arm having ALREADY declared itself out of service, so an
  // ordinary first failure announces and recovers rather than being killed.
  assert.ok(
    routePlans.indexOf("+layerUnreachable(Robot, Request)\n    : cannotServe")
      < routePlans.indexOf("+layerUnreachable(Robot, Request): true <-"),
    "the loop case must be checked before the announce case"
  );
  const announce = routePlans.slice(routePlans.indexOf("+layerUnreachable(Robot, Request): true <-"));
  assert.match(announce.slice(0, announce.indexOf("\n\n")), /!announceUnavailable\(runtime\)/);
  assert.doesNotMatch(announce.slice(0, announce.indexOf("\n\n")), /drop_all_intentions/,
    "a first failure is announced and recovered from, not fatal");
});

test("every agent carries the runtime plan, not just the ones that include it by luck", () => {
  // It lives in routePlans.asl, which every generated agent includes -- the same
  // file the init handlers that announce are in.
  const directory = path.resolve(__dirname, "../agt/generated");
  const agents = fs.readdirSync(directory).filter((file) => /^robot\d+_agent\.asl$/.test(file));
  assert.ok(agents.length >= 4);
  for (const file of agents) {
    assert.match(fs.readFileSync(path.join(directory, file), "utf8"),
      /include\("src\/agt\/routePlans\.asl"\)/, `${file} would not hear that its own layer had gone`);
  }
});
