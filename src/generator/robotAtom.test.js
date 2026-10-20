"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const { asJasonAtom, robotAtom } = require("./agentThingServer");

// The generator is a CLI that runs on require, so its converter is read as
// source rather than imported. Reading it is the point: this test exists to
// pin the runtime's spelling of a robot's name against the one the generator
// wrote into the plans, and importing a copy would not do that.
function generatorJcmAtom() {
  const source = fs.readFileSync(path.resolve(__dirname, "generateRobot.js"), "utf8");
  const body = source.slice(source.indexOf("function jcmAtom"), source.indexOf("function agentAslFileName"));
  assert.ok(body.includes("return `robot_"), "jcmAtom moved; this test is reading the wrong function");
  // eslint-disable-next-line no-eval
  return eval(`(${body.replace("function jcmAtom", "function")})`);
}

test("a robot's name reaches the agent as the atom its own plans are written in", () => {
  // asJasonAtom guards against an atom Jason would read as a variable by
  // prefixing anything that does not start lower-case, so it turned "Robot3"
  // into v_Robot3 -- a valid atom naming a robot that does not exist. The
  // beliefs sent that way did not fail loudly; they unified with nothing and
  // the plan waiting for them sat until its timeout. Live cell: Robot1 waited
  // the full 60s on every startup for an interactionSpaceFailed(robot3, _) that
  // had been delivered as interactionSpaceFailed(v_Robot3, _) a second in.
  assert.equal(asJasonAtom("Robot3"), "v_Robot3", "the trap this helper walked into");
  assert.equal(robotAtom("Robot3"), "robot3");

  const jcmAtom = generatorJcmAtom();
  for (const name of ["Robot1", "Robot2", "Robot3", "Robot4", "YouBot", "robot3", "3Robot", "my-arm"]) {
    assert.equal(robotAtom(name), jcmAtom(name), `${name} must be spelled the way the generator spells it`);
  }
  // A whole-name toLowerCase() -- the other spelling that was in use here --
  // agrees on Robot3 and quietly disagrees on any name with an inner capital.
  assert.equal(robotAtom("YouBot"), "youBot");
  assert.notEqual(robotAtom("YouBot"), "YouBot".toLowerCase());

  assert.equal(robotAtom(undefined), "unknown_robot");
  assert.equal(robotAtom(""), "unknown_robot");
});

test("no belief carries a robot's name through the general-purpose atom helper", () => {
  // The two spellings coexisted in this file for as long as they did because
  // nothing compared them. Every robot-named belief now goes through robotAtom,
  // and the shapes that produced v_Robot3 must not come back.
  const source = fs.readFileSync(path.resolve(__dirname, "agentThingServer.js"), "utf8");
  for (const pattern of [
    /asJasonAtom\((?:canonical)?[Tt]argetRobot/,
    /asJasonAtom\(canonicalTarget/,
    /asJasonAtom\((?:other|me)\.toLowerCase\(\)/
  ]) {
    assert.doesNotMatch(source, pattern, `a robot name is still going through asJasonAtom: ${pattern}`);
  }
});
