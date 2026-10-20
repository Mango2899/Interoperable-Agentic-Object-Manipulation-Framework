const assert = require("node:assert/strict");
const test = require("node:test");

const {
  parseWallOffsets, parseResampleSelection, luaNumber, floorPlaneYaw, rotatePoint
} = require("./robwotBridge");

const cell = (...names) => names.map((name) => ({ manifest: { name } }));

test("an unlisted robot keeps the wall placement the scene template gives it", () => {
  // Every existing cell was assembled without the option, so no displacement
  // has to mean byte-identical placement, not "close enough".
  assert.deepEqual(parseWallOffsets(undefined, 3), [[0, 0, 0], [0, 0, 0], [0, 0, 0]]);
  assert.deepEqual(parseWallOffsets("0.5,0", 3), [[0.5, 0, 0], [0, 0, 0], [0, 0, 0]]);
  assert.deepEqual(parseWallOffsets(";;-0.43,-0.26", 3), [[0, 0, 0], [0, 0, 0], [-0.43, -0.26, 0]]);
});

test("a wall displacement carries the height it was given", () => {
  assert.deepEqual(parseWallOffsets("0,0,0.2", 1), [[0, 0, 0.2]]);
});

test("a malformed wall displacement names the entry rather than silently placing a wall", () => {
  // A wall put somewhere unintended is only discovered after a full resample,
  // so this has to fail at parse time.
  assert.throws(() => parseWallOffsets("0.5", 1), /--wall-offsets entry 1/);
  assert.throws(() => parseWallOffsets("a,b", 1), /--wall-offsets entry 1/);
});

test("resampling every robot is what a run does unless it is told otherwise", () => {
  assert.deepEqual(parseResampleSelection(undefined, cell("Robot1", "Robot2")), [true, true]);
  // --resample with no value parses as a bare flag, which cannot mean "none".
  assert.deepEqual(parseResampleSelection(true, cell("Robot1", "Robot2")), [true, true]);
});

test("only the named robots are resampled, so the others keep the workspace a calibration was tuned against", () => {
  assert.deepEqual(parseResampleSelection("Robot3", cell("Robot1", "Robot2", "Robot3")), [false, false, true]);
  assert.deepEqual(parseResampleSelection("Robot1,Robot3", cell("Robot1", "Robot2", "Robot3")), [true, false, true]);
});

test("a misspelled robot resamples nothing rather than everything", () => {
  // Silently resampling all four would destroy exactly the workspaces the flag
  // was used to protect.
  assert.throws(() => parseResampleSelection("robot3", cell("Robot1", "Robot3")), /not one of the robots/);
});

test("a positional list that does not line up with the cell is refused", () => {
  assert.throws(() => parseWallOffsets("0,0;0,0;0,0", 2), /entries are positional/);
});

test("a number emitted into Lua survives its own sign", () => {
  // Lua reads "--" as a line comment. The wall placement builds
  // `...-<sin(yaw)>*...`, so a robot yawed anywhere in (-180, 0) used to emit
  // `--0.9063...` and comment out the rest of the line, including the closing
  // brace of the position table -- the chunk then failed to compile and
  // executeScriptString reported only "operation failed". Cells whose robots
  // were all yawed 0 or 180 never saw it, because those sines are 0 and
  // +1.2e-16.
  assert.equal(luaNumber(0.5), "0.5");
  assert.equal(luaNumber(0), "0");
  assert.equal(luaNumber(-0.25881904510252074), "(-0.25881904510252074)");
  assert.ok(!`a-${luaNumber(Math.sin(-15 * Math.PI / 180))}*b`.includes("--"));
  assert.ok(!`a-${luaNumber(Math.sin(-115 * Math.PI / 180))}*b`.includes("--"));
});

test("a number Lua could not use is refused rather than emitted", () => {
  // NaN reaches the scene as the literal "NaN", which Lua reads as a nil
  // global and silently places the wall at the origin.
  assert.throws(() => luaNumber(NaN), /non-finite/);
  assert.throws(() => luaNumber(Infinity), /non-finite/);
});

test("a mount is read the way the simulator reads it, not the textbook way", () => {
  // CoppeliaSim composes Rx(alpha)*Ry(beta)*Rz(gamma). The two CR-series mounts
  // below both stand an arm whose joint 1 runs along model -X upright; under the
  // Rz*Ry*Rx reading this file used to carry, the second one tips over instead.
  const deg = (d) => d * Math.PI / 180;
  const up = (o) => rotatePoint({ x: -1, y: 0, z: 0 }, o.map(deg));
  for (const mount of [[0, -90, 180], [-90, -155, 90], [-90, 35, 90], [-90, 20, 90]]) {
    const axis = up(mount);
    assert.ok(Math.abs(axis.z - 1) < 1e-9, `mount ${mount} does not stand the arm up`);
  }
  // The single mount the old convention happened to agree on, which is why no
  // cell ever caught this: at [0,-90,180] only one angle pair is in play.
  assert.ok(Math.abs(up([0, -90, -115]).z - 1) > 0.5, "[0,-90,-115] is not upright");
});

test("the wall follows the stance's heading, not the third Euler angle", () => {
  const deg = (d) => d * Math.PI / 180;
  const back = (o) => floorPlaneYaw(o.map(deg)) * 180 / Math.PI;
  // A bare yaw has to come back exactly, or every cell built before this moves.
  for (const yaw of [-180, -115, -15, 0, 50, 70, 180]) {
    assert.ok(Math.abs(back([0, 0, yaw]) - yaw) < 1e-9, `bare yaw ${yaw} changed`);
  }
  // A CR arm at [-90, beta, 90] has gamma pinned at 90, so reading orientation[2]
  // would give every wall in the cell the same heading.
  assert.ok(Math.abs(back([-90, 35, 90]) - 55) < 1e-9);
  assert.ok(Math.abs(back([-90, 20, 90]) - 70) < 1e-9);
  assert.ok(Math.abs(back([-90, -155, 90]) - (-115)) < 1e-9);
  assert.ok(Math.abs(back([0, -90, 180]) - 180) < 1e-9);
});

test("the flange a model ships is left alone", () => {
  // A repair here could only recognise a bad flange by its being square with
  // the WORLD, and that is true at exactly one arm pose -- the one the dummy
  // was created at. So it fires at scene load, does nothing once the arm moves,
  // and a simulation restart puts the scene's own poses back: the tool jumps
  // before the run and snaps back at the start of it. Meanwhile the models that
  // would have been "repaired" are the two whose mounts are calibrated around
  // the flange they ship. Fixed at creation instead, in the workspace driver.
  const { buildWorkspaceGripperPreamble } = require("./robwotBridge");
  const preamble = buildWorkspaceGripperPreamble({
    tcpAlias: "Robot9_tcp",
    gripperModelName: "Robot9_tcp_gripper",
    gripperModelPath: "/tmp/gripper.ttm",
    gripperModelPosition: [0, 0, 0],
    gripperModelOrientation: [0, 0, 0],
    gripperModelLocalRotation: [0, 0, 0],
    applyBaseOrientationCorrection: false,
    graspPadAlias: "touch",
    gripperSource: "model"
  });
  assert.doesNotMatch(preamble, /AlignFlangeToWrist/);
  assert.doesNotMatch(preamble, /setObjectOrientation\(flange/);
});

test("the base-orientation correction is off unless a manifest asks for it", () => {
  // It re-rotates an already-mounted gripper in the WORLD by the base
  // orientation. Whether that is a fix or a fault depends on the model: a
  // gripper on a flange square with the wrist already inherits the base
  // rotation through the hierarchy, so applying it again double-rotates the
  // tool by exactly the base heading -- which is why all three CR arms, whose
  // models ship a square flange, keep it off. Both UR models ship a twisted
  // flange, so their tool does NOT inherit that rotation and the correction is
  // what makes it follow the base; they ask for it by name.
  //
  // Defaulting it ON made it silently right for the arms that ship one kind of
  // flange and silently wrong for the arms that ship the other, so a new robot
  // now gets nothing it did not ask for.
  const source = require("node:fs").readFileSync(
    require("node:path").join(__dirname, "robwotBridge.js"), "utf8"
  );
  assert.match(source, /applyBaseOrientationCorrection: grip\.applyBaseOrientationCorrection === true/);
  assert.doesNotMatch(source, /applyBaseOrientationCorrection: grip\.applyBaseOrientationCorrection !== false/);
});
