"use strict";

const assert = require("node:assert/strict");
const crypto = require("node:crypto");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const { archiveRobot, cellPaths, robotPaths } = require("./archiveRobot");

// A repository just real enough to archive out of: one robot in a two-robot
// cell, with a file named by the manifest that also lives inside the robot's
// own directory, because that overlap is what the copy has to get right.
function buildCell(root) {
  const write = (relative, contents) => {
    const file = path.join(root, relative);
    fs.mkdirSync(path.dirname(file), { recursive: true });
    fs.writeFileSync(file, contents, "utf8");
    return relative;
  };
  const manifest = {
    name: "Robot4",
    configKey: "Robot4",
    urdfPath: "/somewhere/dobot1.urdf",
    ports: { lowerThing: 8140, agentThing: 8141, websocket: 8241 },
    mobility: { type: "fixed", baseLink: "dobot1/base_link", armJoints: ["dobot1/joint_1"] },
    pipeline: { sourceScene: "/somewhere/dobot.ttt", sourceModel: "/dobot_normal" },
    sharedEnvironment: {
      descriptor: "generated/robots/Robot3_Robot4.shared.json",
      scene: "generated/robots/Robot3_Robot4_shared.ttt"
    },
    generatedFiles: {
      agentSpeak: "src/agt/generated/robot4_agent.asl",
      agentTd: "src/td/generated/Robot4.agent.td.json",
      // Named by the manifest AND inside generated/robots/Robot4, which is a
      // source in its own right.
      dataPoints: "generated/robots/Robot4/info/Robot4_data_point.csv"
    }
  };
  write("generated/robots/Robot4/manifest.json", JSON.stringify(manifest, null, 2));
  write("generated/robots/Robot4/info/Robot4_data_point.csv", "X,Y,Z,Reachable\n0,0,0,1\n");
  write("generated/robots/Robot4/model/Robot4.ttm", "model bytes");
  write("src/agt/generated/robot4_agent.asl", "// plans\n");
  write("src/td/generated/Robot4.agent.td.json", "{}\n");
  write("generated/robots/Robot3_Robot4.shared.json", '{"cell":true}\n');
  write("generated/robots/Robot3_Robot4_shared.ttt", "scene bytes");
  write("generated/robots/interactions/Robot3__Robot4/space.csv", "x,y,z\n");
  // Another robot's files, which must not be swept up.
  write("generated/robots/Robot3/manifest.json", '{"name":"Robot3"}');
  write("src/agt/generated/robot3_agent.asl", "// not mine\n");
  write("config.json", JSON.stringify({
    Robot3: { TDPort: 8131, websocketPort: 8231 },
    Robot4: { TDPort: 8141, websocketPort: 8241 }
  }, null, 2));
  return manifest;
}

function withCell(run) {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "womat-archive-"));
  try {
    run(root, buildCell(root));
  } finally {
    fs.rmSync(root, { recursive: true, force: true });
  }
}

test("the archive is a byte-for-byte copy, and every file appears once", () => {
  withCell((root) => {
    const result = archiveRobot({ root, name: "Robot4", folder: "Robot4_Dobot" });

    // The overlap is the point: Robot4_data_point.csv is reached both through
    // generatedFiles and by walking generated/robots/Robot4. Copying it twice
    // is invisible -- the second copy overwrites the first with itself -- but
    // it doubles the file count and puts a duplicate line in SHA256SUMS, which
    // still verifies. Nothing would have reported it.
    const relatives = result.files.map((file) => file.relative);
    assert.deepEqual([...new Set(relatives)], relatives, "a file was archived twice");
    assert.ok(relatives.includes("generated/robots/Robot4/info/Robot4_data_point.csv"));

    const sums = fs.readFileSync(path.join(result.archiveRoot, "SHA256SUMS"), "utf8").trim().split("\n");
    assert.equal(sums.length, result.files.length);
    for (const line of sums) {
      const [expected, relative] = line.split("  ");
      const copied = path.join(result.archiveRoot, "files", relative);
      assert.ok(fs.existsSync(copied), `${relative} is listed but was not copied`);
      assert.equal(crypto.createHash("sha256").update(fs.readFileSync(copied)).digest("hex"), expected);
    }
  });
});

test("the cell the robot was part of travels with it", () => {
  withCell((root, manifest) => {
    const cell = cellPaths(root, "Robot4", manifest);
    // Regenerating the robot rebuilds all of these, so a copy of the robot
    // without them describes an arrangement that no longer exists anywhere.
    assert.ok(cell.includes("generated/robots/Robot3_Robot4.shared.json"));
    assert.ok(cell.includes("generated/robots/Robot3_Robot4_shared.ttt"));
    assert.ok(cell.includes("generated/robots/interactions/Robot3__Robot4"));
  });
});

test("another robot's files are left where they are", () => {
  withCell((root) => {
    const result = archiveRobot({ root, name: "Robot4", folder: "Robot4_Dobot" });
    const relatives = result.files.map((file) => file.relative);
    assert.ok(!relatives.some((relative) => relative.includes("Robot3/")));
    assert.ok(!relatives.includes("src/agt/generated/robot3_agent.asl"));
    // Robot3 is still there afterwards: this copies, it never moves.
    assert.ok(fs.existsSync(path.join(root, "generated/robots/Robot3/manifest.json")));
    assert.ok(fs.existsSync(path.join(root, "generated/robots/Robot4/manifest.json")));
  });
});

test("only this robot's config.json entry is kept", () => {
  withCell((root) => {
    const { archiveRoot } = archiveRobot({ root, name: "Robot4", folder: "Robot4_Dobot" });
    const entry = JSON.parse(fs.readFileSync(path.join(archiveRoot, "config.entry.json"), "utf8"));
    // Restoring a whole config.json would take every other robot's ports back
    // to whatever they happened to be on the day the copy was taken.
    assert.deepEqual(Object.keys(entry), ["Robot4"]);
    assert.deepEqual(entry.Robot4, { TDPort: 8141, websocketPort: 8241 });
  });
});

test("an archive cannot be written where it would be mistaken for a robot", () => {
  withCell((root) => {
    // Anything under generated/robots with a manifest.json in it IS a robot to
    // the rest of the repo: cleanupRobotPorts would read the copy's ports and
    // kill the live robot holding them, and the plan tests would assert against
    // an arm that is not in the cell.
    assert.throws(
      () => archiveRobot({ root, name: "Robot4", folder: "Robot4_Dobot", into: "generated/robots" }),
      /cannot live under generated\/robots/
    );
  });
});

test("an existing archive is not overwritten unless asked", () => {
  withCell((root) => {
    archiveRobot({ root, name: "Robot4", folder: "Robot4_Dobot" });
    assert.throws(
      () => archiveRobot({ root, name: "Robot4", folder: "Robot4_Dobot" }),
      /already exists/
    );
    assert.ok(archiveRobot({ root, name: "Robot4", folder: "Robot4_Dobot", force: true }).files.length > 0);
  });
});

test("a dry run writes nothing", () => {
  withCell((root) => {
    const result = archiveRobot({ root, name: "Robot4", folder: "Robot4_Dobot", dryRun: true });
    assert.ok(result.files.length > 0);
    assert.ok(!fs.existsSync(result.archiveRoot));
  });
});

test("archiving a robot that does not exist says so", () => {
  withCell((root) => {
    assert.throws(() => archiveRobot({ root, name: "Robot9", folder: "Robot9" }), /No generated robot named Robot9/);
  });
});

test("the manifest decides what belongs to a robot, with the same fallbacks delete uses", () => {
  const paths = robotPaths("Robot4", {
    generatedFiles: { agentSpeak: "src/agt/generated/robot4_agent.asl", missingKey: 7 }
  });
  assert.ok(paths.includes("src/agt/generated/robot4_agent.asl"));
  assert.ok(paths.includes(path.join("generated", "robots", "Robot4")));
  assert.ok(paths.includes(path.join("src", "td", "generated", "Robot4.agent.td.json")));
  // A non-string generatedFiles value is not a path.
  assert.ok(!paths.includes(7));
});
