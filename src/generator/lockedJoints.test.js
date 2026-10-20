"use strict";

const assert = require("node:assert/strict");
const { execFileSync } = require("node:child_process");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

const {
  chainJointIntervals,
  lockedChainPositions,
  lockedJointBlock,
  selectArmChainJoints
} = require("./core/luaDriver");
const { createContext } = require("./core/context");
const { prepareRuntimeDriverScript } = require("./core/driverScript");

// The block is Lua, so it is checked by running it. Enough of the CoppeliaSim
// API to hold a joint list and record what was done to each handle -- which is
// the entire surface the selection touches.
function runBlock(locked, handles) {
  const stub = `
local held = {}
local targets = {}
local motors = {}
local modes = {}
sim = {
  setJointPosition = function(handle, value) held[handle] = value end,
  setJointTargetPosition = function(handle, value) targets[handle] = value end,
  setJointMode = function(handle, mode) modes[handle] = mode end,
  setObjectInt32Param = function(handle, param, value) motors[handle] = value end,
  jointmode_passive = 0,
  jointintparam_motor_enabled = 2001
}
jointHandles = {${handles.join(",")}}
${lockedJointBlock(locked)}
local remaining = {}
for i=1,#jointHandles,1 do remaining[#remaining+1] = tostring(jointHandles[i]) end
local holds = {}
for handle, value in pairs(held) do holds[#holds+1] = handle .. "=" .. tostring(value) end
table.sort(holds)
local extra = {}
for handle, value in pairs(motors) do extra[#extra+1] = "motor" .. handle .. "=" .. tostring(value) end
for handle, value in pairs(targets) do extra[#extra+1] = "target" .. handle .. "=" .. tostring(value) end
for handle in pairs(modes) do extra[#extra+1] = "mode" .. handle end
table.sort(extra)
local lockedHandles = {}
for i=1,#(womatLockedJointHandles or {}),1 do lockedHandles[#lockedHandles+1] = tostring(womatLockedJointHandles[i]) end
print(table.concat(remaining, ",") .. "|" .. table.concat(holds, ",") .. "|" .. table.concat(extra, ",")
  .. "|" .. table.concat(lockedHandles, ","))
`;
  const file = path.join(fs.mkdtempSync(path.join(os.tmpdir(), "womat-lua-")), "block.lua");
  fs.writeFileSync(file, stub);
  try {
    const [remaining, holds, extra, lockedHandles] = execFileSync("lua", [file], { encoding: "utf8" }).trim().split("|");
    return {
      remaining: remaining ? remaining.split(",").map(Number) : [],
      held: holds ? holds.split(",") : [],
      extra: extra ? extra.split(",") : [],
      lockedHandles: lockedHandles ? lockedHandles.split(",").map(Number) : []
    };
  } finally {
    fs.rmSync(path.dirname(file), { recursive: true, force: true });
  }
}

test("a joint is locked by its place in the chain, because it has no name to lock by", () => {
  // configureImportedRobot aliases EVERY joint "joint" as it imports the URDF,
  // so by the time the runtime sees the model there are five objects called
  // "joint" and nothing to tell them apart. Matching on "joint_end" matched
  // nothing at all, which is why the fifth axis still moved. Tree order is what
  // survives: the vendor driver reads joints base to tip and the chain selection
  // preserves that order.
  const result = runBlock(lockedChainPositions({
    jointChain: ["a/joint_1", "a/joint_2", "a/joint_3", "a/joint_4", "a/joint_end"],
    lockedJoints: ["a/joint_end"]
  }), [11, 12, 13, 14, 15]);
  assert.deepEqual(result.remaining, [11, 12, 13, 14]);
  assert.deepEqual(result.held, ["15=0"]);
  // A locked joint is not a disconnected one. Taking it out of the dynamic chain
  // detaches everything below it -- the end link and the gripper bolted to it
  // drop to the floor at simulation start. A fixed axis IS a motor holding a
  // position, so the mode is left alone and the motor is what holds it.
  assert.ok(result.extra.includes("target15=0"), "the target is what the motor holds it to");
  assert.ok(result.extra.includes("motor15=1"), "and the motor stays enabled");
  assert.ok(!result.extra.some((entry) => entry.startsWith("mode")), "the joint mode is never changed");
});

test("the handles of the locked joints survive the selection that drops them", () => {
  // Dropping a joint from jointHandles stops it being commanded and nothing
  // else. The solver builds its chain by walking the scene, so it still has the
  // locked axis -- and this is the last point at which every joint is still in
  // one list, so it is where the handles have to be kept for the freeze below.
  const result = runBlock(lockedChainPositions({
    jointChain: ["a/j1", "a/j2", "a/j3", "a/j4", "a/j_end"],
    lockedJoints: ["a/j_end"]
  }), [11, 12, 13, 14, 15]);
  assert.deepEqual(result.remaining, [11, 12, 13, 14], "the locked joint leaves jointHandles");
  assert.deepEqual(result.lockedHandles, [15], "and is remembered by handle");
});

test("a locked joint is passive in the solver, not merely absent from the command list", () => {
  // A locked axis was locked against joint commands and wide open to every
  // Cartesian move: simIK.addIkElementFromScene walks the scene from base to
  // tip, so the joint is in the chain whatever jointHandles says, and the
  // collision rollback in womatSimplePoseCallback restores jointHandles only --
  // so the drift is never undone and accumulates. Live cell: Robot4's joint_end,
  // locked at zero, was found at 8.97 degrees with a target written to match.
  //
  // Rendered from a manifest that locks something rather than from a shipped
  // robot: which arms happen to lock a joint is a configuration choice that
  // changes, and this is about the mechanism.
  const base = createContext(path.join("generated", "robots", "Robot4", "manifest.json"));
  const locking = {
    ...base,
    manifest: {
      ...base.manifest,
      mobility: {
        ...base.manifest.mobility,
        jointChain: ["a/j1", "a/j2", "a/j3", "a/j4", "a/j_end"],
        lockedJoints: ["a/j_end"]
      }
    }
  };
  const driver = prepareRuntimeDriverScript(locking, {});
  assert.match(driver, /function womatFreezeLockedIkJoints\(env, simToIk\)/);
  assert.match(driver, /simIK\.setJointMode, env, ikJoint, simIK\.jointmode_passive/);
  // Every environment the driver builds, not just the one that happened to be
  // noticed: the vendor driver's own -- which the swept Cartesian preflight
  // uses, so it runs before every move -- plus the general move, the pick, the
  // handover and the planner.
  const built = driver.match(/simIK\.addIkElementFromScene[,(]/g) || [];
  const frozen = driver.match(/womatFreezeLockedIkJoints\(/g) || [];
  assert.equal(built.length, 5, "the runtime builds five IK environments");
  assert.equal(
    frozen.length - 1,
    built.length,
    "every simIK environment must freeze the locked joints (one match is the definition)"
  );
  assert.match(driver, /local ikElement, ikSimToIk=simIK\.addIkElementFromScene\(ikEnv,ikGroup,/);
  // Guarded: a driver that cannot say "passive" still has to move the arm.
  assert.match(driver, /local ok = pcall\(simIK\.setJointMode/);

  // An arm that locks nothing is rendered exactly as it was: the vendor line is
  // not rewritten, and the freeze calls it never reaches are inert.
  const plain = prepareRuntimeDriverScript(base, {});
  assert.match(plain, /local ikElement=simIK\.addIkElementFromScene\(ikEnv,ikGroup,/);
  assert.doesNotMatch(plain, /womatLockedJointHandles = \{\}/);
});

test("a name that is not in the chain locks nothing", () => {
  assert.deepEqual(lockedChainPositions({ jointChain: ["j1", "j2"], lockedJoints: ["j9"] }), []);
  assert.equal(lockedJointBlock(lockedChainPositions({ jointChain: ["j1"], lockedJoints: ["j9"] })), "");
});

test("an arm that locks nothing is rendered exactly as before", () => {
  // Every other robot in the cell has to be untouched by this, or a change meant
  // for one Dobot silently re-samples four arms.
  assert.equal(lockedJointBlock([]), "");
  assert.equal(lockedJointBlock(undefined), "");
  const vendor = fs.readFileSync(
    createContext(path.join("generated", "robots", "Robot1", "manifest.json")).paths.driver,
    "utf8"
  );
  assert.equal(selectArmChainJoints(vendor), selectArmChainJoints(vendor, []));
});

test("Robot4 adopts a model whose end joint is rigid, so it locks nothing", () => {
  // It used to lock dobot1/joint_end -- and lock the wrong axis, because
  // joint_end is the tool yaw and joint_4 is the parallelogram levelling
  // linkage. The model it now adopts settles that at the source: joint_end is a
  // FIXED joint in dobot1.urdf and a force sensor in the scene, so there is no
  // fifth revolute axis to hold still, and joint_4 stays free on purpose --
  // lockToolOrientation makes the solver drive it to keep the tool level.
  const robot4 = createContext(path.join("generated", "robots", "Robot4", "manifest.json"));
  assert.deepEqual(robot4.manifest.mobility.lockedJoints, []);
  assert.deepEqual(lockedChainPositions(robot4.manifest.mobility), []);
  assert.equal(robot4.manifest.mobility.armJoints.length, 5, "four arm axes plus the gripper rotation");
  assert.ok(
    robot4.manifest.mobility.armJoints.includes("dobot1/joint_grip"),
    "the gripper rotation is one of the arm's degrees of freedom, not a locked axis"
  );
  assert.ok(
    !robot4.manifest.mobility.jointChain.includes("dobot1/joint_end"),
    "joint_end is fixed, so it is not in the chain at all"
  );
  assert.equal(
    robot4.manifest.agentDefaults.homeJoints.length,
    robot4.manifest.mobility.armJoints.length,
    "the home pose names one value per axis"
  );
  const driver = prepareRuntimeDriverScript(robot4, {});
  assert.doesNotMatch(driver, /womatLockedJoints/);
  // No free wrist to give away: the tool is held level by a linkage the tree
  // models as three independent parallel axes, so the solver is told to keep
  // the orientation instead.
  assert.equal(robot4.manifest.runtime.lockToolOrientation, true);
  assert.match(
    driver,
    /addIkElementFromScene\(\s*\n\s*womatSimpleIkEnv[\s\S]*?simIK\.constraint_pose/
  );
  const robot1 = createContext(path.join("generated", "robots", "Robot1", "manifest.json"));
  assert.match(
    prepareRuntimeDriverScript(robot1, {}),
    /addIkElementFromScene\(\s*\n\s*womatSimpleIkEnv[\s\S]*?simIK\.constraint_position/,
    "a six-axis arm keeps its free wrist"
  );
});

test("a Thing Description out of step with the manifest is said out loud", () => {
  // The joint count in the TD comes from the SAMPLE -- robotInfo reports
  // #jointHandles as CoppeliaSim saw it -- while the manifest states how many
  // axes the arm has. They can only disagree if the TD was written against an
  // older sampling run, which is what a stale TD looks like: a manifest saying
  // four axes and a description still advertising five. Consumers believe the
  // description, so the mismatch has to surface where both are known.
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  const writer = bridge.slice(bridge.indexOf("function generateLowerTd(ctx)"));
  assert.match(writer, /const sampledAxes = Number\(robotInfo\.jointAmount \|\| 0\)/);
  assert.match(writer, /declaredAxes > 0 && sampledAxes !== declaredAxes/);
  const warning = writer.slice(writer.indexOf("declaredAxes > 0 && sampledAxes"), writer.indexOf("writeJson(ctx.paths.generatedLowerTd"));
  assert.match(warning, /re-run --generate-td/, "the warning has to say what fixes it");
  assert.match(warning, /lockedJoints/, "and name the locked joints, which are the usual cause");
  // The count is in the success line too, so a run that did the right thing
  // says so rather than leaving it to be discovered in the file.
  assert.match(writer, /Saved lower-level TD: \$\{ctx\.paths\.generatedLowerTd\} \(\$\{sampledAxes\} joint\(s\)\)/);
});

test("sampling one arm alone can still place the cell's sites", () => {
  // Otherwise seeing a locked arm over its own TD means building a whole shared
  // cell, which rewrites the robot's sharedEnvironment on the way past.
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  assert.match(bridge, /async function generateWorkspaceData\(ctx, cellSites\)/);
  assert.match(bridge, /const declaredSites = parseCellSites\(args\.sites\);/);
  assert.match(bridge, /await generateWorkspaceData\(ctx, declaredSites\);/);
  const body = bridge.slice(bridge.indexOf("async function generateWorkspaceData(ctx, cellSites)"));
  // Placed AND treated as obstacles: a support the arm reaches around is not
  // scenery, and a cloud sampled without it claims space it cannot use.
  assert.match(body, /installPipelineGripObjects\(sim, robotScriptHandle, ctx, sites\)/);
  assert.match(body, /collisionObstacles: uniqueItems\(\[[\s\S]*?sites\.map\(\(site\) => site\.alias\)/);
  // An arm with no sites is sampled exactly as before.
  assert.match(body, /const samplingCtx = sites\.length === 0 \? ctx : \{/);
});

test("the lock is baked into the generated model, not only into the driver", () => {
  // "Blocking the generated joints for CoppeliaSim" is not the same as the
  // robot having four axes. The GUI will happily drag a dynamic joint the driver
  // filtered out of its own list, so the joint is made passive in the .ttm as it
  // is imported -- everything built from that model inherits four axes.
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  const configure = bridge.slice(
    bridge.indexOf("async function configureImportedRobot"),
    bridge.indexOf("async function loadWorkspaceDriver")
  );
  assert.match(configure, /const locked = new Map\(\(lockedPositions \|\| \[\]\)\.map/);
  // Set up exactly like a driven joint, then parked on a target it is never
  // asked to leave -- NOT taken out of the dynamic chain, which would drop the
  // end link and its gripper on the floor.
  assert.doesNotMatch(configure, /jointmode_passive/, "a locked joint stays dynamically connected");
  assert.match(configure, /await sim\.setJointTargetPosition\(joint, held\)/);
  assert.match(configure, /setObjectAlias\(joint, "joint_locked"\)/, "and visible as such in the scene tree");
  const lockedBranch = configure.slice(configure.indexOf("if (locked.has(chainIndex))"));
  assert.ok(
    configure.indexOf("await sim.setJointMode(joint, await sim.jointmode_dynamic, 0)") < configure.indexOf("if (locked.has(chainIndex))"),
    "the dynamic setup runs for a locked joint too, before it is parked"
  );
  // Chain positions are counted over the ARM. An adopted model's gripper is a
  // mechanism with its own named joints, preserved rather than aliased, and
  // counting those would shift every lock index past them.
  assert.match(configure, /const preserved = new Set\(\(options\.preserve \|\| \[\]\)\.map\(Number\)\);/);
  assert.match(configure, /if \(preserved\.has\(Number\(joint\)\)\) \{\s*\n\s*continue;/);
  assert.match(lockedBranch, /setJointPosition\(joint, held\)/);
  // The rename is the reason names cannot be used to find a joint later.
  assert.match(configure, /setObjectAlias\(joint, "joint"\)/);
  assert.match(bridge, /configureImportedRobot\(\s*\n?\s*sim,\s*\n?\s*ctx\.manifest\.name,\s*\n?\s*lockedChainPositions\(ctx\.manifest\.mobility\)/);
});

test("the manifest's joint limits are addressed by chain position, like the locks", () => {
  const mobility = {
    jointChain: ["a/joint_1", "a/joint_2", "a/joint_3", "a/joint_4"],
    armJointLimits: {
      "a/joint_1": { lower: -2.25, upper: 2.25 },
      // Continuous: nothing to impose, so the scene's own interval stands.
      "a/joint_3": { lower: 0, upper: 0 },
      "a/joint_4": { lower: -3.1416, upper: 3.1416 }
    }
  };
  assert.deepEqual(chainJointIntervals(mobility), [
    { index: 1, name: "a/joint_1", low: -2.25, high: 2.25 },
    { index: 4, name: "a/joint_4", low: -3.1416, high: 3.1416 }
  ]);
  // armJoints is the chain for a robot that locks nothing, and every arm
  // generated before jointChain existed has only that.
  assert.deepEqual(
    chainJointIntervals({ armJoints: ["j"], armJointLimits: { j: { lower: -1, upper: 1 } } }),
    [{ index: 1, name: "j", low: -1, high: 1 }]
  );
  assert.deepEqual(chainJointIntervals({}), []);
});

test("an adopted model gets the URDF's joint limits, not the source scene's", () => {
  // A URDF import carries its own limits across; an adopted model keeps
  // whatever the scene it came from was built with. The Dobot arrives with a
  // base joint of [-240, +120] degrees -- a full turn, so nothing is
  // unreachable, but off-centre, so an azimuth of +124 degrees exists only at
  // -236 and the arm takes three quarters of a circle to a point 80 degrees
  // away. Nothing downstream can recover from that: womatUnwrapConfigToward
  // rejects the short equivalent for being outside the interval, correctly.
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  const configure = bridge.slice(
    bridge.indexOf("async function configureImportedRobot"),
    bridge.indexOf("async function loadWorkspaceDriver")
  );
  assert.match(configure, /const intervals = new Map\(declaredIntervals\.map/);
  assert.match(configure, /await sim\.setJointInterval\(joint, false, \[interval\.low, interval\.high - interval\.low\]\)/);
  // Position-addressed, so it is only as good as the agreement between the
  // manifest's chain and the scene's -- and where they disagree the damage is
  // silent. Robot2's manifest lists five arm joints for a six-joint UR3.
  assert.match(configure, /const applyIntervals = intervals\.size > 0 && intervals\.size === armJointCount;/);
  assert.match(configure, /if \(intervals\.size > 0 && !applyIntervals\) \{/,
    "a chain that does not match must be said out loud, not applied partially");
  // A locked axis is parked on a target it never leaves, so its interval
  // decides nothing and narrowing it around the held position only makes the
  // two disagree.
  assert.ok(
    configure.indexOf("setJointInterval") > configure.indexOf('setObjectAlias(joint, "joint_locked")'),
    "the interval is set after the lock branch has taken its joints out"
  );
  // Both paths into the model, because both save a .ttm the rest of the
  // pipeline consumes.
  assert.equal(
    bridge.match(/jointIntervals: chainJointIntervals\(ctx\.manifest\.mobility\)/g)?.length,
    2,
    "the URDF import and the scene adoption must both pass the limits"
  );
});

test("a declared site is the whole pick scene, and a solo run records it", () => {
  // Two halves of one bug. The pipeline installed the declared site AND this
  // robot's default support and cube, because it fell back to the gripper config
  // whenever nothing declared carried a payload. And a robot served on its own
  // rebuilds its pick objects from runtime.gripper, so even a clean sampling run
  // put the pair back the moment the runtime started.
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  const install = bridge.slice(
    bridge.indexOf("async function installPipelineGripObjects"),
    bridge.indexOf("function radians(")
  );
  assert.match(install, /sites\.length > 0 \? \(loadedSite\(sites\) && gripFromSite\(loadedSite\(sites\)\)\) : gripSceneConfig\(ctx\)/);
  assert.match(install, /if \(!grip && empties\.length === 0\) return;/);

  // The declaration is written back as the scene, so the runtime rebuilds what
  // was sampled instead of what the manifest happened to be carrying.
  const record = bridge.slice(
    bridge.indexOf("function recordSoloSceneSites(ctx, sites)"),
    bridge.indexOf("function updateManifest(ctx, updates)")
  );
  assert.ok(record.length > 0, "a solo run has to record the sites it built");
  assert.match(record, /manifest\.runtime\.taskSites = sites\.map/);
  assert.match(record, /supportName: grip\.supportName/);
  assert.match(record, /objectName: grip\.objectName/);
  // A declaration with nothing loaded cannot silently leave the old pair behind.
  assert.match(record, /no declared site carries a payload/);
  // Stale aliases are dropped: an obstacle that is not in the scene is a no-op
  // that reads like a guard.
  assert.match(record, /filter\(\(alias\) => \/wall\/i\.test\(alias\)\)/);
  assert.match(bridge, /recordSoloSceneSites\(ctx, declaredSites\)/);

  // A robot served alone reads its own manifest, never the cell's list: the cell
  // describes a different scene, and reading it put Robot1's support and cube
  // into Robot4's solo scene.
  const driver = fs.readFileSync(path.resolve(__dirname, "core", "driverScript.js"), "utf8");
  assert.doesNotMatch(driver, /sharedEnvironment\?\.taskSites/);
});

test("the tool's mount link is found from the tool, not from a joint index", () => {
  // Locking an arm's final axis takes it out of jointHandles while the link
  // below it still carries the gripper. The old skip set was
  // jointHandles[#jointHandles] and the one before, so on a locked arm it named
  // the wrong two links and the tool was reported in permanent collision with
  // the very link it is bolted to -- Robot4_tcp_gripper against
  // dobot1_link_end_respondable, at step 1 of every motion.
  const lua = fs.readFileSync(path.resolve(__dirname, "lua", "04-safety.lua"), "utf8");
  const prepare = lua.slice(
    lua.indexOf("function womatPrepareSelfCollisionCollections()"),
    lua.indexOf("function womatRobotSelfCollision()")
  );
  assert.ok(prepare.length > 0);
  assert.match(prepare, /local womatMountLink=womatSelfCollisionLinkOf\(tcpGripper\)/);
  assert.match(prepare, /skip\[womatMountLink\]=true/);
  assert.match(prepare, /skip\[womatSelfCollisionLinkOf\(womatMountLink\)\]=true/);
  // The indexed form survives only as the fallback for a tool with no joint
  // above it at all.
  const fallback = prepare.slice(prepare.indexOf("else"), prepare.indexOf("local links="));
  assert.match(fallback, /skip\[jointHandles\[#jointHandles\]\]=true/);
  assert.ok(
    prepare.indexOf("local womatMountLink") < prepare.indexOf("skip[jointHandles[#jointHandles]]=true"),
    "the tool decides which links are skipped; the index is only the fallback"
  );
});

test("the pass that gets saved builds the same scene the sampling pass did", () => {
  // Grip objects are rebuilt after the sweep because sampling leaves the scene
  // mid-posture -- and that second pass is the one written to disk. Called
  // without the declared sites it fell back to the robot's own grip config, so
  // every run ended by putting the default support and cube into the scene it
  // was about to save, whatever it had been asked to build.
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  const start = bridge.indexOf("async function generateWorkspaceData(ctx, cellSites)");
  // To the closing brace of that function, not to the next thing that happens to
  // mention it: createSharedEnvironmentScene installs sites too.
  const body = bridge.slice(start, bridge.indexOf("\n}\n", start));
  const installs = body.match(/installPipelineGripObjects\(sim, robotScriptHandle, ctx[^)]*\)/g) || [];
  assert.equal(installs.length, 2, "one install before the sweep, one after it");
  for (const call of installs) {
    assert.match(call, /, sites\)$/, `an install that ignores the declared sites: ${call}`);
  }
  assert.ok(
    body.indexOf("saveScene(ctx.paths.generatedWorkspaceScene)") > body.lastIndexOf("installPipelineGripObjects"),
    "the save has to come after the rebuild it is meant to capture"
  );
});

test("a robot whose end effector is a mechanism is adopted, not imported", () => {
  // A URDF describes links and joints. It cannot describe the Dobot's rod
  // gripper: a four-bar linkage closed by dummy link pairs, driven by two
  // prismatic hand joints, with a script reading a custom data block. In
  // dobot1.urdf the whole thing is link_grip.STL, one rigid mesh. The working
  // version exists only as a built scene, so the model is taken from there.
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  const adopt = bridge.slice(
    bridge.indexOf("async function adoptSceneModel"),
    bridge.indexOf("async function loadWorkspaceDriver")
  );
  assert.ok(adopt.length > 0, "there must be an adoption step");

  // The source scene drives the arm itself. Left in place, two controllers
  // command the same joints; the gripper's own script is the mechanism, not a
  // controller, and is deliberately kept.
  assert.match(adopt, /gripperTree\[gripper\]=true/);
  assert.match(adopt, /if not gripperTree\[arm\[i\]\] then/);
  assert.match(adopt, /pcall\(sim\.removeScript, script\)/);

  // The source's tip dummy sits out at the jaws -- it was a full-pose IK tip,
  // not a mount. womatResolveFlange would take it for the flange and mount the
  // gripper on it, putting the jaws at two tool lengths.
  assert.match(adopt, /for _, alias in ipairs\(\{'tip', 'target'\}\) do/);
  assert.match(adopt, /sim\.setObjectAlias\(flange, 'tip'\)/);
  assert.match(adopt, /sim\.setObjectPose\(flange, sim\.handle_world, mount\)/);
  assert.match(adopt, /sim\.setObjectParent\(gripper, flange, true\)/);

  // The gripper's joints keep their names, or the arm chain gains seven axes
  // and the gripper's script can no longer find its own.
  assert.match(adopt, /preserve: Object\.values\(prepared\.preserve \|\| \{\}\)/);
  assert.match(adopt, /saveModel\(robotHandle, ctx\.paths\.generatedModel\)/);

  // Same artifact as the URDF path produces, so everything downstream is
  // unchanged -- and a manifest that names a source scene picks the adoption.
  assert.match(bridge, /const adopts = Boolean\(ctx\.manifest\.pipeline\?\.sourceScene && ctx\.manifest\.pipeline\?\.sourceModel\)/);
  assert.match(bridge, /const shouldGenerateModel = !shouldAdoptModel &&/);

  const robot4 = createContext(path.join("generated", "robots", "Robot4", "manifest.json"));
  assert.ok(robot4.manifest.pipeline.sourceScene.endsWith("dobot_magician_with_gripper.ttt"));
  assert.equal(robot4.manifest.pipeline.sourceModel, "/dobot_normal");
  assert.equal(robot4.manifest.pipeline.sourceGripper, "gripper_rod");
  assert.ok(fs.existsSync(robot4.manifest.pipeline.sourceScene), "the source scene has to exist");
});

test("a gripper that actuates itself is asked, not driven", () => {
  // The rod gripper's script reads a custom data block every step and sets a
  // target VELOCITY from it, so the jaws close until they meet something and
  // hold. Writing those joints from here as well is two controllers on one
  // mechanism: ours sets a kinematic position, its sets a velocity, and each
  // step undoes the other.
  const robot4 = createContext(path.join("generated", "robots", "Robot4", "manifest.json"));
  const driver = prepareRuntimeDriverScript(robot4, {});
  assert.match(driver, /local name=\[\[activity\]\]/);
  const state = driver.slice(driver.indexOf("function womatSetTcpGripperState"));
  const body = state.slice(0, state.indexOf("\nfunction "));
  assert.match(body, /if activity ~= nil then[\s\S]*?driver="activityDataBlock"/);
  assert.ok(
    body.indexOf('driver="activityDataBlock"') < body.indexOf("setJointTargetPosition"),
    "the mechanism is asked before any joint is written, and the write is skipped"
  );

  // Its gripper came with it, so there is no file to load -- but the tool
  // control point still has to be measured onto the jaw pads, or the arm is
  // sampled and driven at its flange.
  assert.equal(robot4.manifest.runtime.gripper.shipsWithModel, true);
  assert.equal(robot4.manifest.runtime.gripper.graspPadAlias, "hand");
  assert.equal(robot4.manifest.runtime.gripper.modelPath, undefined);
  const bridge = fs.readFileSync(path.resolve(__dirname, "robwotBridge.js"), "utf8");
  // The file check guards the LOAD, so only a tool that is loaded is checked
  // for. shipsWithModel is now spelled source: "builtin", and the check is
  // scoped to the one source that has a file at all.
  assert.match(bridge, /grip\.source === "model" && !fs\.existsSync\(grip\.modelPath\)/);

  // 0 leaves the jaws 21.2 mm apart and 0.016 opens them to ~53 mm, so the
  // 45 mm cube fits with about 4 mm a side.
  assert.deepEqual(robot4.manifest.runtime.gripper.closedJointPositions, [0]);
  assert.deepEqual(robot4.manifest.runtime.gripper.openJointPositions, [0.016]);

  // Every other arm keeps the joint drive.
  const robot1 = createContext(path.join("generated", "robots", "Robot1", "manifest.json"));
  assert.match(prepareRuntimeDriverScript(robot1, {}), /local name=\[\[\]\]/);
});
