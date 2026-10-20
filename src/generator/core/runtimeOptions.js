"use strict";

// One robot's runtime tuning, resolved from its manifest, plus the workspace
// admission grid built from its reachability cloud.

const fs = require("fs");
const path = require("path");

const { orientationRadians, worldVectorToLocal } = require("../handoverGeometry");
const { WorkspaceGrid } = require("../workspaceGrid");
const { SharedRegion } = require("./claims");

function jointPayloadToRadians(payload, td) {
  const jointNames = Object.keys(td.actions.moveTojointPosition.input.properties);
  return jointNames.map((jointName) => Number(payload[jointName] || 0) * Math.PI / 180);
}

function createRuntimeOptions(ctx) {
  const runtime = ctx.manifest.runtime || {};
  const positive = (value, fallback) => Number.isFinite(Number(value)) && Number(value) > 0
    ? Number(value) : fallback;
  return {
    // How thick the arm is treated as being when it claims shared space. The
    // reported polyline runs through joint origins, which describes where the
    // links are but not how wide they are.
    simpleJointVelocityDeg: Number(runtime.simpleJointVelocityDeg || 30),
    simpleJointAccelerationDeg: Number(runtime.simpleJointAccelerationDeg || 60),
    simpleJointJerkDeg: Number(runtime.simpleJointJerkDeg || 120),
    // The same three bounds the driver is generated with. The core needs them
    // too, because it paces the reserved Cartesian route itself.
    simpleCartesianVelocity: Number(runtime.simpleCartesianVelocity || 0.175),
    simpleCartesianAcceleration: Number(runtime.simpleCartesianAcceleration || 0.25),
    simpleCartesianJerk: Number(runtime.simpleCartesianJerk || 0.75),
    returnHomeBetweenHandovers: runtime.returnHomeBetweenHandovers === true,
    // Geometry is batched by sample count; collision preflight yields after a
    // time budget. A sample already in progress must finish before yielding.
    homeGeometryChunk: Number(runtime.homeGeometryChunk || 128),
    collisionCheckStepDeg: positive(runtime.collisionCheckStepDeg, 0.5),
    preflightBudgetMs: positive(runtime.preflightBudgetMs, 50),
    // A known path queues behind the arms already moving through its voxels
    // and plays up to the first one they still hold, instead of waiting for
    // all of them to be free. Off restores reserving the path whole.
    orderedReservations: runtime.orderedReservations !== false,
    // Samples kept between the hold point and the first held voxel: room for a
    // dynamic arm to come to rest behind a limit it reached at speed.
    followLookaheadSteps: Math.max(0, Math.round(Number(runtime.followLookaheadSteps ?? 2))),
    // How long a follower waits behind one hold before giving the motion up.
    followWaitMs: positive(runtime.followWaitMs, 60000),
    claimBodyRadius: Number(ctx.manifest.runtime?.claimBodyRadius ?? 0.12),
    enableSpatialClaims: ctx.manifest.runtime?.enableSpatialClaims !== false,
    // How far outside a shared region a commanded target still counts as
    // possibly entering it, since the arm reaching that target sweeps well
    // beyond the point itself.
    claimReachMargin: Number(ctx.manifest.runtime?.claimReachMargin ?? 0.35),
    jointSteps: Number(runtime.jointSteps || 60),
    cartesianSteps: Number(runtime.cartesianSteps || 60),
    stepDelayMs: Number(runtime.stepDelayMs || 25),
    cartesianMaxVelocity: Number(runtime.cartesianMaxVelocity || 0.45),
    // Generated URDF IK commonly settles 10-20 mm from a full-pose target.
    // Keep this below the gripper's physical capture threshold while allowing
    // a demonstrably graspable pose to proceed through BDI execution.
    cartesianFinalTolerance: Number(runtime.cartesianFinalTolerance || 0.025),
    jointMaxVelocityDeg: Number(runtime.jointMaxVelocityDeg || 60),
    // null means "derive from this robot's sample spacing"; the interaction
    // pipeline keeps its own explicit default.
    workspaceTolerance: Number(runtime.workspaceTolerance) > 0 ? Number(runtime.workspaceTolerance) : null,
    enableCollisionCheck: runtime.enableCollisionCheck === true,
    enableJointCollisionPreflight: runtime.enableJointCollisionPreflight !== false,
    gripperCollisionClearance: Number(runtime.gripperCollisionClearance ?? 0.005),
    pickupGripperClearance: Number(runtime.pickupGripperClearance ?? 0.001),
    enablePathPlanning: runtime.enablePathPlanning !== false,
    simpleSmoothMotion: runtime.simpleSmoothMotion === true,
    preferJointTrajectoryForCartesian: runtime.preferJointTrajectoryForCartesian !== false,
    preferOmplForCartesian: runtime.preferOmplForCartesian === true,
    preferBaseJointFirst: runtime.preferBaseJointFirst === true,
    baseJointFirstMinDeltaDeg: Number(runtime.baseJointFirstMinDeltaDeg ?? 2),
    planningMaxTimeSeconds: Number(runtime.planningMaxTimeSeconds || 4),
    planningStateCount: Number(runtime.planningStateCount || 120),
    pathTrackingToleranceDeg: Number(runtime.pathTrackingToleranceDeg || 0.5),
    plannedJointMaxVelocityDeg: Number(runtime.plannedJointMaxVelocityDeg || 45),
    pathSmoothingPasses: Number(runtime.pathSmoothingPasses ?? 2),
    pathSettleMaxSteps: Number(runtime.pathSettleMaxSteps || 100),
    // Workspace admission from the generated CSV, and the swept check that
    // replaced per-simulation-step collision testing.
    enableWorkspaceAdmission: runtime.enableWorkspaceAdmission !== false,
    // Left unset, admission tolerance is derived from the robot's own sampled
    // workspace density rather than assumed.
    workspaceToleranceSpacingFactor: Number(runtime.workspaceToleranceSpacingFactor) > 0
      ? Number(runtime.workspaceToleranceSpacingFactor)
      : 3,
    workspaceVoxelSize: Number(runtime.workspaceVoxelSize) > 0 ? Number(runtime.workspaceVoxelSize) : null,
    enableInMotionCollisionCheck: runtime.enableInMotionCollisionCheck !== false,
    // Every step. Anything sparser tunnels through the thin wall and the cube.
    inMotionCollisionStride: Math.max(1, Math.round(Number(runtime.inMotionCollisionStride || 1))),
    inMotionClearanceStride: Math.max(0, Math.round(Number(runtime.inMotionClearanceStride ?? 0))),
    // Sampling is deliberately coarse, and that is safe only because the
    // per-step watchdog is the guarantee. This sweep exists to reject a bad
    // command before anything moves; it is not dense enough to be relied on
    // for thin obstacles, and making it so would reintroduce the long
    // blocking script call that raises CoppeliaSim's modal abort dialog.
    preflightStepSize: Number(runtime.preflightStepSize) > 0 ? Number(runtime.preflightStepSize) : 0.04,
    preflightMaxSamples: Math.max(2, Math.round(Number(runtime.preflightMaxSamples || 16))),
    // 0 means the mesh-distance clearance margin is evaluated only at the goal
    // pose. Contact is still tested at every sample.
    preflightClearanceStride: Math.max(0, Math.round(Number(runtime.preflightClearanceStride ?? 0))),
    // Handover rendezvous. The approach is split so the last stretch into the
    // other robot's reach is a straight line with the tool axis held, not a
    // joint interpolation that sweeps the shared volume.
    toolAxis: (runtime.gripper?.toolAxis || [0, 1, 0]).map(Number),
    handoverStandoffDistance: Number(runtime.gripper?.handoverStandoffDistance ?? 0.15),
    handoverApproachSteps: Math.max(1, Math.round(Number(runtime.gripper?.handoverApproachSteps ?? 8))),
    // Per-direction planning budget. One script call must stay well short of
    // the point where CoppeliaSim offers to abort it.
    handoverPlanningMaxTime: Number(runtime.gripper?.handoverPlanningMaxTime ?? 2.5),
    // Ceiling on the whole (direction x roll) search, so a rendezvous nothing
    // can reach costs a bounded number of refused plans rather than the product
    // of two candidate lists.
    handoverApproachMaxAttempts:
      Math.max(1, Math.round(Number(runtime.handoverApproachMaxAttempts ?? 64))),
    // How many rolls either side of the predicted one each direction is given
    // before any direction is walked out in full. Twenty-four five-degree steps
    // is a sixty-degree arc around the orthogonal-jaw prediction.
    handoverApproachRollBand:
      Math.max(1, Math.round(Number(runtime.handoverApproachRollBand ?? 24))),
    // The largest single-joint excursion a rendezvous posture may cost before
    // the search prefers a different roll. Zero disables the preference; a
    // rendezvous nothing satisfies is still taken on the second pass.
    handoverApproachMaxTravelDeg:
      Math.max(0, Number(runtime.handoverApproachMaxTravelDeg ?? 150)),
    // How far any one joint may have drifted from the pose the recorded
    // approach ended on before its reverse stops being the path the arm
    // actually walked. Loose enough to survive a dynamic joint settling,
    // tight enough that a deliberate move in between is not replayed over.
    handoverRetreatDriftTolerance:
      Number(runtime.gripper?.handoverRetreatDriftToleranceDeg ?? 5) * Math.PI / 180,
    // World vertical expressed in the same local frame as Cartesian targets.
    worldUpLocal: worldVectorToLocal([0, 0, 1], orientationRadians(ctx.manifest.sharedEnvironment)),
    // Top-down pick
    enableTopDownPick: runtime.gripper?.enableTopDownPick !== false,
    graspHeightOffset: Number(runtime.gripper?.graspHeightOffset ?? 0),
    pickupApproachHeight: Number(runtime.gripper?.pickupApproachHeight ?? 0.20),
    pickDescentSteps: Math.max(1, Math.round(Number(runtime.gripper?.pickDescentSteps ?? 8))),
    pickPlanningMaxTime: Number(runtime.gripper?.pickPlanningMaxTime ?? 4),
    // Divisions per half-axis of the grid walked over a support's top face when
    // the drop point at its centre cannot be planned. 2 gives a 5x5 grid: the
    // centre, the edges and the corners of the region the cube may occupy,
    // which is enough to find the near corner of a face at the edge of reach
    // without spending a planning budget on each of forty-nine points.
    enablePlaceFaceSearch: runtime.gripper?.enablePlaceFaceSearch !== false,
    placeFaceSearchSteps: Math.max(1, Math.round(Number(runtime.gripper?.placeFaceSearchSteps ?? 2))),
    // Planning budget per candidate while sweeping a support's face.
    //
    // Deliberately small, because the budget only ever buys FAILURE time here.
    // A drop point the arm can serve is solved by the seeded solve in about
    // 50 ms and never reaches the randomised search; one it cannot serve is
    // out of the top-down envelope, which no amount of searching changes, so it
    // spends the entire budget to arrive at the same "no". Measured on Robot4's
    // support_b: 4435 ms to reject a point at the full budget, 423 ms at this
    // one, same verdict, and 53 ms to accept the one that works.
    //
    // Nothing is given up: a sweep that rejects every candidate is run again at
    // the full budget before the place is called impossible.
    placeFaceSearchMaxTime: Number(runtime.gripper?.placeFaceSearchMaxTime ?? 0.4),
    // How high above the resting position the cube is let go of. The arm stops
    // there with the tool vertical and opens; the cube falls the rest of the
    // way. Defaults to the pick's approach height because that is the same
    // clearance measured the same way -- one knob until someone needs two.
    placeReleaseHeight: Number(
      runtime.gripper?.placeReleaseHeight ?? runtime.gripper?.pickupApproachHeight ?? 0.20
    ),
    // Flange-to-TCP distance, used only to probe the tip-frame workspace cloud.
    toolLengthEstimate: Number(runtime.gripper?.toolLengthEstimate ?? 0.19)
  };
}

function loadWorkspaceGrid(ctx, options) {
  if (!options.enableWorkspaceAdmission) return null;
  try {
    const grid = WorkspaceGrid.fromCsv(ctx.paths.robotDataPoints, {
      voxelSize: options.workspaceVoxelSize,
      tolerance: options.workspaceTolerance,
      spacingFactor: options.workspaceToleranceSpacingFactor
    });
    console.log(`${ctx.manifest.name} workspace admission: ${JSON.stringify(grid.describe())}.`);
    return grid;
  } catch (error) {
    console.warn(`Could not build the workspace grid for ${ctx.manifest.name}:`, error?.message || error);
    return null;
  }
}

function checkWorkspace(point, workspace, options) {
  if (!workspace) return { ok: true, admission: "disabled" };
  return workspace.admits(point, options.workspaceTolerance);
}

module.exports = {
  checkWorkspace,
  loadSharedRegions,
  createRuntimeOptions,
  jointPayloadToRadians,
  loadWorkspaceGrid
};

// The shared regions this robot takes part in, read from the pairwise spaces the
// interaction-space calculator exported.
//
// Reloaded before motion from files, not recomputed: every participant derives
// the same space (the estimator is order-independent), so reading the pair's
// own artifact is both cheaper and exactly as correct as recomputing it.
function loadSharedRegions(ctx) {
  const descriptorPath = ctx.manifest.sharedEnvironment?.descriptor;
  if (!descriptorPath) return [];
  const resolved = path.resolve(ctx.repoRoot, descriptorPath);
  if (!fs.existsSync(resolved)) return [];
  const descriptor = JSON.parse(fs.readFileSync(resolved, "utf8"));
  const regions = [];
  const entries = new Map((descriptor.sharedSpaces || []).map((entry) => [entry.pairKey, entry]));
  // Agents export discovered pairs without rewriting the cell descriptor.
  // Look for their canonical artifacts too, including pairs absent at startup.
  for (const peer of descriptor.robots || []) {
    if (peer.name === ctx.manifest.name) continue;
    const robots = [ctx.manifest.name, peer.name].sort((a, b) =>
      a.localeCompare(b, undefined, { numeric: true, sensitivity: "base" }));
    const pairKey = robots.join("__");
    if (!entries.has(pairKey)) entries.set(pairKey, {
      pairKey,
      metadata: `generated/robots/interactions/${pairKey}/space.json`,
      csv: `generated/robots/interactions/${pairKey}/space.csv`
    });
  }
  for (const entry of entries.values()) {
    const robots = String(entry.pairKey).split("__");
    if (!robots.includes(ctx.manifest.name)) continue;
    if (!entry.metadata || !entry.csv) continue;
    const metadataPath = path.resolve(ctx.repoRoot, entry.metadata);
    const csvPath = path.resolve(ctx.repoRoot, entry.csv);
    if (!fs.existsSync(metadataPath) || !fs.existsSync(csvPath)) continue;
    const metadata = JSON.parse(fs.readFileSync(metadataPath, "utf8"));
    const centers = fs.readFileSync(csvPath, "utf8").trim().split(/\r?\n/).slice(1)
      .map((line) => line.split(",").slice(0, 3).map(Number))
      .filter((row) => row.length === 3 && row.every(Number.isFinite))
      .map(([x, y, z]) => ({ x, y, z }));
    if (centers.length === 0) continue;
    regions.push(SharedRegion.fromMetadata({ ...metadata, robots }, centers));
  }
  return regions;
}
