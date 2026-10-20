"use strict";

// The per-robot runtime driver: RobWoT's base Lua with its blocking entry
// points renamed, plus this repository's driver rendered from src/generator/lua
// against one robot's manifest.

const fs = require("fs");

const {
  gripperSource,
  lockedChainPositions,
  luaArray,
  luaBoolean,
  luaString,
  radiansTriple,
  render: renderDriver,
  selectArmChainJoints,
  toolControlPointOffset
} = require("./luaDriver");

function prepareRuntimeDriverScript(ctx, driverOptions = {}) {
  const grip = ctx.manifest.runtime?.gripper || {};
  const obstacles = driverOptions.obstacleNames || ctx.manifest.runtime?.collisionObstacles || ctx.manifest.pipeline?.collisionObstacles || ["80cmHighWall50cm"];
  const baseOffset = driverOptions.baseOffset || [0, 0, 0];
  const baseOrientation = Array.isArray(driverOptions.baseOrientation)
    ? driverOptions.baseOrientation.map(Number)
    : [0, 0, Number(driverOptions.baseYaw || 0)];
  const baseYaw = baseOrientation[2];
  const shift = (position) => [
    baseOffset[0] + Math.cos(baseYaw) * position[0] - Math.sin(baseYaw) * position[1],
    baseOffset[1] + Math.sin(baseYaw) * position[0] + Math.cos(baseYaw) * position[1],
    baseOffset[2] + position[2]
  ];
  const obstacleLua = `{${obstacles.map(luaString).join(",")}}`;
  // Scene discovery by default: the backstop enumerates the scene and excludes
  // what is provably this robot's own, rather than trusting a list. The list
  // survives as an override for a non-shared scene, where "everything that is
  // not me" would sweep in scenery that is deliberately ignored.
  const obstacleDiscovery = ctx.manifest.runtime?.obstacleDiscovery === "configured" ? "configured" : "scene";
  const obstacleIgnore = ctx.manifest.runtime?.obstacleIgnoreAliases || [];
  // In shared-grip mode all robots target the same pick cube and support,
  // defined by the scene-owning robot in world coordinates (no offset shift).
  // In shared-grip mode all robots target the same pick cube and support, handed
  // down by the cell host in the cell's coordinates.
  //
  // A robot served on its own does NOT read the cell's list. It loads its own
  // workspace scene, and the cell's sites describe a different scene entirely --
  // reading them put Robot1's support and cube into Robot4's solo scene, which
  // is worse than the default pair it replaced. What a solo run builds is what
  // its own manifest says it has, which is what the solo pipeline writes there.
  const sharedGrip = driverOptions.sharedGrip || null;
  const cubeName = sharedGrip ? sharedGrip.objectName : (grip.objectName || `${ctx.manifest.name}_grip_cube`);
  const supportName = sharedGrip ? sharedGrip.supportName : (grip.supportName || `${ctx.manifest.name}_grip_support`);
  const cubeSize = sharedGrip ? sharedGrip.objectSize : (grip.objectSize || [0.045, 0.045, 0.045]);
  const cubePosition = sharedGrip ? sharedGrip.objectPosition : shift(grip.objectPosition || [0.45, 0.2, 0.325]);
  const supportSize = sharedGrip ? sharedGrip.supportSize : (grip.supportSize || [0.18, 0.18, 0.24]);
  const supportPosition = sharedGrip ? sharedGrip.supportPosition : shift(grip.supportPosition || [0.45, 0.2, 0.18]);
  // How the loaded site is turned. Declared in the cell's frame like its
  // position, so a robot-local grip config inherits the robot's own yaw and
  // lands exactly where it always did.
  const gripYawDeg = sharedGrip
    ? Number(sharedGrip.yawDeg || 0)
    : Number(grip.supportYawDeg || 0) + Number(baseOrientation[2] || 0) * 180 / Math.PI;
  // Additional cube supports standing in the cell. In a shared scene the cell
  // host has already resolved every robot's declared sites into world
  // coordinates and hands the whole list to every driver -- an arm has to be
  // able to recognise a site it cannot reach, or it could never tell a
  // neighbour it does not have one. In a single-robot scene there is nobody
  // else to collect them from, so the manifest's own list is shifted the same
  // way the pick support is.
  const taskSites = (
    driverOptions.taskSites
    || (ctx.manifest.runtime?.taskSites || []).map((site) => ({
      ...site,
      position: shift(site.position || [0, 0, 0])
    }))
  ).map((site) => ({
    alias: String(site.alias),
    size: (site.size || supportSize).map(Number),
    position: (site.position || [0, 0, 0]).map(Number),
    yaw: Number(site.yawDeg || 0) * Math.PI / 180
  }));
  const taskSitesLua = `{${taskSites.map((site) =>
    `{alias=${luaString(site.alias)},size=${luaArray(site.size)},position=${luaArray(site.position)},yaw=${site.yaw}}`
  ).join(",")}}`;
  const gripperModelName = grip.modelName || `${ctx.manifest.name}_tcp_gripper`;
  const gripperModelPath = grip.modelPath || "/home/maxchrs/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/models/components/grippers/RG2.ttm";
  const gripperModelPosition = grip.modelPosition || [0, 0, 0];
  const gripperModelOrientation = radiansTriple(
    grip.modelOrientation || [-1.57079632679, 0, 0],
    "runtime.gripper.modelOrientation"
  );
  const gripperModelLocalRotation = radiansTriple(
    grip.modelLocalRotation || [0, 0, 0],
    "runtime.gripper.modelLocalRotation"
  );
  // Same default as the pipeline's two copies: off unless a manifest asks. This
  // one is the RUNTIME's, and it disagreeing with them is how a gripper could be
  // mounted one way during sampling and another way when the cell came up.
  const applyGripperBaseOrientationCorrection = grip.applyBaseOrientationCorrection === true;
  // Mount the tool against the axis the wrist rolls about rather than against
  // the 'tip' dummy's inherited orientation. See womatWristJoint in 07-tool-frame.
  const gripperSquareToWristJoint = grip.squareToWristJoint === true;
  const gripperOpenJointPositions = grip.openJointPositions || [0.06];
  const gripperClosedJointPositions = grip.closedJointPositions || [0];
  // Direction the tool points, in the robot's own frame. There is no offset
  // length any more: simTip is the tool control point, so a commanded position
  // is where the object goes.
  const toolAxis = grip.toolAxis || [0, 1, 0];
  const toolLengthEstimate = Number(grip.toolLengthEstimate ?? 0.19);
  const gripThreshold = Number(grip.gripThreshold || 0.04);
  const pickupApproachHeight = Number(grip.pickupApproachHeight ?? 0.10);
  const graspHeightOffset = Number(grip.graspHeightOffset ?? 0);
  const graspPadAlias = String(grip.graspPadAlias ?? "touch").toLowerCase();
  // Where the tool comes from. "builtin" is the one that changes what the
  // runtime may do to it: the tool is a link of the arm's own chain, so it is
  // never loaded, never reparented and never removed for looking wrong.
  const gripperSourceName = gripperSource(grip);
  const gripperSourceAlias = String(grip.sourceAlias ?? "");
  // Declared tool control point, in the tool's frame, for a tool with no pads
  // to measure one from. Empty means measure -- which is every jawed gripper.
  const gripperTcpOffset = toolControlPointOffset(grip);
  // How far in from the lip of a support the cube's own footprint must stay
  // when a placement is moved off the centre of the face. Small on purpose: the
  // point of moving off centre is that the centre could not be served, so every
  // millimetre of face given away here is reach the arm may need.
  const placeEdgeMargin = Number(grip.placeEdgeMargin ?? 0.005);
  const pickIkDamping = Number(grip.pickIkDamping ?? 0.05);
  // The tool control point marker: a dummy at the contact-pad midpoint, sized to
  // be seen in the scene. It is the same object the pick IK is driven through,
  // so what is on screen is what the solver is steering.
  const tcpMarkerSize = Number(ctx.manifest.runtime?.tcpMarkerSize ?? 0.02);
  // How close the receiver's jaw centre must actually be to the cube before a
  // handover transfer is allowed. This is the check that used to be absent.
  // Turning this off restores the old behaviour: the cube is accepted wherever
  // the receiver happens to be. It is only honest to do that when the receiver
  // is not actually being steered onto the cube.
  const enableHandoverGraspGate = ctx.manifest.runtime?.enableHandoverGraspGate !== false;
  // How close the pads must end up to the commanded grasp point, and how many
  // corrective passes are allowed. Tighter than handoverGraspTolerance on
  // purpose: calibrating only to the transfer gate's own threshold leaves no
  // margin, and each corrective move is itself a free-wrist Cartesian move that
  // can shift the pads again.
  const handoverGraspTolerance = Number(ctx.manifest.runtime?.handoverGraspTolerance ?? 0.04);
  // simIK.findConfig's "thresholdDist" is NOT a solution tolerance: it is the
  // radius within which a randomly drawn configuration is considered worth
  // refining with IK at all. Anything tighter than a few tenths rejects nearly
  // every draw before IK runs, and the search returns nil after burning its
  // whole time budget. CoppeliaSim's own default for the same parameter in
  // simIK.findConfigs is 0.3; the pick element also constrains the tool axis,
  // and the free roll about that axis still counts towards the pose distance
  // (metric[4]*angle), so the gate has to be looser here than for a
  // position-only search.
  const pickSearchThresholdDist = Number(grip.pickSearchThresholdDist ?? 0.6);
  // Accuracy the returned configuration must actually achieve, checked against
  // the scene rather than trusted from the solver.
  const pickPositionTolerance = Number(grip.pickPositionTolerance ?? 0.002);
  const pickAxisToleranceDeg = Number(grip.pickAxisToleranceDeg ?? 2);
  const pathTrackingToleranceDeg = Number(ctx.manifest.runtime?.pathTrackingToleranceDeg || 0.5);
  const plannedJointMaxVelocityDeg = Number(ctx.manifest.runtime?.plannedJointMaxVelocityDeg || 45);
  const pathSmoothingPasses = Number(ctx.manifest.runtime?.pathSmoothingPasses ?? 2);
  const pathSettleMaxSteps = Number(ctx.manifest.runtime?.pathSettleMaxSteps || 100);
  const collisionClearance = Number(ctx.manifest.runtime?.collisionClearance ?? 0.02);
  const gripperCollisionClearance = Number(ctx.manifest.runtime?.gripperCollisionClearance ?? 0.005);
  const pickupGripperClearance = Number(ctx.manifest.runtime?.pickupGripperClearance ?? 0.001);
  const collisionCheckStepDeg = Number(ctx.manifest.runtime?.collisionCheckStepDeg || 0.5);
  const inMotionCollisionStride = Math.max(1, Math.round(Number(ctx.manifest.runtime?.inMotionCollisionStride || 1)));
  const preflightClearanceStride = Math.max(0, Math.round(Number(ctx.manifest.runtime?.preflightClearanceStride ?? 0)));
  const inMotionClearanceStride = Math.max(0, Math.round(Number(ctx.manifest.runtime?.inMotionClearanceStride ?? 0)));
  const simpleJointVelocityDeg = Number(ctx.manifest.runtime?.simpleJointVelocityDeg || 30);
  const simpleJointAccelerationDeg = Number(ctx.manifest.runtime?.simpleJointAccelerationDeg || 60);
  const simpleJointJerkDeg = Number(ctx.manifest.runtime?.simpleJointJerkDeg || 120);
  const simpleCartesianVelocity = Number(ctx.manifest.runtime?.simpleCartesianVelocity || 0.175);
  const simpleCartesianAcceleration = Number(ctx.manifest.runtime?.simpleCartesianAcceleration || 0.25);
  const simpleCartesianJerk = Number(ctx.manifest.runtime?.simpleCartesianJerk || 0.75);
  return selectArmChainJoints(fs.readFileSync(ctx.paths.driver, "utf8"), lockedChainPositions(ctx.manifest.mobility))
    .replace("function sysCall_actuation()", "function sysCall_actuation_disabled()")
    .replace("function moveTojoint(targetConf)", "function moveTojoint_async(targetConf)")
    .replace("function moveToPosition(targetPosition)", "function moveToPosition_async(targetPosition)")
    + "\n" + renderDriver({
    robotNameLua: luaString(ctx.manifest.name),
    baseOffsetLua: luaArray(baseOffset),
    baseOrientationLua: luaArray(baseOrientation),
    baseYaw,
    simpleJointVelocityDeg,
    simpleJointAccelerationDeg,
    simpleJointJerkDeg,
    simpleCartesianVelocity,
    simpleCartesianAcceleration,
    simpleCartesianJerk,
    simpleSafetyEnabledLua: luaBoolean(ctx.manifest.runtime?.enableCollisionCheck === true),
    obstacleLua,
    obstacleDiscoveryLua: luaString(obstacleDiscovery),
    obstacleIgnoreLua: `{${obstacleIgnore.map(luaString).join(",")}}`,
    inMotionSafetyEnabledLua: luaBoolean(ctx.manifest.runtime?.enableInMotionCollisionCheck !== false),
    // Box-culled contact checks; false restores one collection check per test.
    collisionBroadphaseLua: luaBoolean(ctx.manifest.runtime?.collisionBroadphase !== false),
    inMotionCollisionStride,
    inMotionClearanceStride,
    collisionClearance,
    gripperCollisionClearance,
    pickupGripperClearance,
    collisionCheckStepDeg,
    preflightClearanceStride,
    cubeNameLua: luaString(cubeName),
    cubeSizeLua: luaArray(cubeSize),
    cubePositionLua: luaArray(cubePosition),
    supportNameLua: luaString(supportName),
    supportSizeLua: luaArray(supportSize),
    supportPositionLua: luaArray(supportPosition),
    gripYawLua: gripYawDeg * Math.PI / 180,
    taskSitesLua,
    gripperModelNameLua: luaString(gripperModelName),
    gripperModelPathLua: luaString(gripperModelPath),
    gripperModelPositionLua: luaArray(gripperModelPosition),
    gripperModelOrientationLua: luaArray(gripperModelOrientation),
    gripperModelLocalRotationLua: luaArray(gripperModelLocalRotation),
    gripperOpenJointPositionsLua: luaArray(gripperOpenJointPositions),
    gripperClosedJointPositionsLua: luaArray(gripperClosedJointPositions),
    gripperVisualJawLeftLua: luaString(gripperModelName + "_visual_jaw_left"),
    gripperVisualJawRightLua: luaString(gripperModelName + "_visual_jaw_right"),
    applyGripperBaseOrientationCorrectionLua: luaBoolean(applyGripperBaseOrientationCorrection),
    gripperSquareToWristJointLua: luaBoolean(gripperSquareToWristJoint),
    toolAxisLua: luaArray(toolAxis),
    toolLengthEstimate,
    gripThreshold,
    graspPadAliasLua: luaString(graspPadAlias),
    gripperSourceLua: luaString(gripperSourceName),
    gripperSourceAliasLua: luaString(gripperSourceAlias),
    // A table, empty when nothing is declared, so the Lua reads it as "measure".
    gripperTcpOffsetLua: gripperTcpOffset ? luaArray(gripperTcpOffset) : "nil",
    placeEdgeMarginLua: placeEdgeMargin,
    // A gripper whose own script actuates it. The Dobot's rod gripper is a
    // four-bar linkage driven by two prismatic hand joints, and its script
    // reads a custom data block on the gripper root to decide which way to
    // drive them. Commanding those joints from here as well is two controllers
    // on one mechanism, so when a block is named the runtime writes it and
    // leaves the joints alone. Empty means the joints are the interface, which
    // is the RG2 and every other gripper the cell has had.
    gripperActivityBlockLua: luaString(String(ctx.manifest.runtime?.gripper?.activityDataBlock || "")),
    gripperActivityClosedLua: luaString(String(ctx.manifest.runtime?.gripper?.activityClosedValue ?? "on")),
    gripperActivityOpenLua: luaString(String(ctx.manifest.runtime?.gripper?.activityOpenValue ?? "off")),
    // Which rotational freedom the general Cartesian move leaves the solver.
    // A six-axis arm is given all of it: position only, wrist free, which is
    // what lets it reach a point from whatever posture suits. A Dobot has no
    // such freedom to give -- its tool is held level by a parallelogram the
    // kinematic tree does not model, so the levelling only happens if the
    // solver is told to keep the tool where it is. Constraining the pose IS
    // that instruction, and it is how the arm's own scene script does it.
    simpleIkConstraintLua: ctx.manifest.runtime?.lockToolOrientation === true
      ? "simIK.constraint_pose"
      : "simIK.constraint_position",
    pickIkDamping,
    tcpMarkerSize,
    tcpAliasLua: luaString(`${ctx.manifest.name}_tcp`),
    tcpIkAliasLua: luaString(`${ctx.manifest.name}_tcp_ik`),
    enableHandoverGraspGateLua: luaBoolean(enableHandoverGraspGate),
    handoverPlacementLabelLua: enableHandoverGraspGate ? '"grasped_in_place"' : '"accepted_ungated"',
    handoverGraspTolerance,
    pickSearchThresholdDist,
    pickPositionTolerance,
    pickAxisToleranceDeg,
    pathTrackingToleranceDeg,
    pathSettleMaxSteps,
    pathSmoothingPasses,
    plannedJointMaxVelocityDeg
  }) + "\n";
}

module.exports = {
  prepareRuntimeDriverScript
};
