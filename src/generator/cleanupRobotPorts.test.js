const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const { discoverRobotPorts, SIM_PORTS } = require("./cleanupRobotPorts");

const root = path.resolve(__dirname, "../..");

test("cleanup covers every port every generated robot binds", () => {
  const robots = discoverRobotPorts();
  const manifests = fs.readdirSync(path.join(root, "generated", "robots"))
    .map((entry) => path.join(root, "generated", "robots", entry, "manifest.json"))
    .filter((file) => fs.existsSync(file))
    .map((file) => JSON.parse(fs.readFileSync(file, "utf8")));

  assert.equal(robots.length, manifests.length, "every generated robot must be covered");

  // Discovered, never written down: a hardcoded list stops covering the stack
  // the moment a robot is generated or renumbered, and the symptom is a port
  // still held by a stale server long after cleanup claimed success.
  const covered = new Set(robots.flatMap((robot) => robot.ports));
  const config = JSON.parse(fs.readFileSync(path.join(root, "config.json"), "utf8"));
  for (const manifest of manifests) {
    for (const role of ["lowerThing", "agentThing", "websocket"]) {
      const port = Number(manifest.ports?.[role]);
      assert.ok(covered.has(port), `${manifest.name} ${role} port ${port} must be cleaned`);
    }
    // config.json is what the AgentThing actually binds, so drift between the
    // two files must not leave a port uncleaned.
    const thingConfig = config[manifest.configKey || manifest.name] || {};
    for (const key of ["TDPort", "websocketPort"]) {
      if (thingConfig[key]) {
        assert.ok(covered.has(Number(thingConfig[key])), `${manifest.name} ${key} must be cleaned`);
      }
    }
  }
});

test("cleanup covers CoppeliaSim's websocket remote API, including extra instances", () => {
  // The add-on binds 23050 + process index, so a stale second instance sits on
  // 23051 and holds nothing the bridge looks for -- but it does keep the scene.
  assert.ok(SIM_PORTS.includes(23050));
  assert.ok(SIM_PORTS.includes(23051), "a second CoppeliaSim instance must be covered");
});

test("the process patterns cannot match the shell that invoked cleanup", () => {
  const source = fs.readFileSync(require.resolve("./cleanupRobotPorts"), "utf8");

  // pgrep -f matches full command lines, so a bare word like "lowerThingServer"
  // also matches the shell running the cleanup. Patterns are script paths, and
  // self/parent/grandparent are excluded outright.
  assert.match(source, /"src\/generator\/lowerThingServer\.js"/);
  assert.match(source, /"src\/generator\/agentThingServer\.js"/);
  assert.match(source, /function protectedPids/);
  assert.match(source, /String\(process\.pid\), String\(process\.ppid\)/);
  assert.match(source, /\.filter\(\(pid\) => !protect\.has\(pid\)\)/);
});
