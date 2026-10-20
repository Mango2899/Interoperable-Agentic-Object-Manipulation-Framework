"use strict";

// The handover protocol as one robot's core sees it: which way it approaches,
// the session it owns, and the physical transfer of the cube.

const { asArray } = require("../workspaceGrid");
const { luaArray, luaString } = require("./luaDriver");
const { executeInScript } = require("./simConnection");
const { sessionZone } = require("./handoverZone");
const {
  approachRollCandidates,
  normalizedVector,
  cellBaselineAxis,
  giverPresentAxis,
  normalizeApproachMode,
  receiverApproachAxisCandidates
} = require("../handoverGeometry");

// Which way the receiver comes in. "opposed" is the two tools nose to nose --
// the shortest rendezvous, but it needs both arms in the same narrow corridor.
// "perpendicular" turns the receiver's tool 90 degrees off the giver's, which
// buys clearance when that corridor is too tight for two arms.
function handoverApproachMode(ctx, sourceRobot) {
  // A pair entry wins over the robot default: Robot2 can receive head-on from
  // Robot1 and hand off perpendicular to Robot3 if that is what fits.
  const pairs = ctx.manifest.sharedEnvironment?.interactionPairs;
  if (Array.isArray(pairs)) {
    const pair = pairs.find((candidate) => Array.isArray(candidate.robots)
      && candidate.robots.includes(ctx.manifest.name)
      && candidate.robots.includes(sourceRobot));
    const fromPair = normalizeApproachMode(pair?.approachMode, null);
    if (fromPair) return fromPair;
  }
  const configured = ctx.manifest.runtime?.handoverApproachMode;
  const mode = normalizeApproachMode(configured, null);
  if (mode) return mode;
  if (configured !== undefined && configured !== null) {
    // Silently receiving head-on because a mode was misspelled is the kind of
    // thing that only shows up as an arm coming in from the wrong side.
    console.warn(
      `[LowerThing:${ctx.manifest.name}] unknown handoverApproachMode `
      + `${JSON.stringify(configured)}; receiving opposed to the giver instead.`
    );
  }
  return "opposed";
}

// The direction this robot holds the cube out along when it is the giver.
// Unset, it presents along its own mounting, which makes the exchange geometry
// a consequence of how each arm happens to be bolted down; naming a cell
// direction pins it instead, so every giver meets the rendezvous the same way.
function giverPresentAxisFor(ctx, peers, targetRobot) {
  const configured = ctx.manifest.runtime?.handoverPresentMode;
  if (configured === undefined || configured === null) return null;
  const receiver = peers?.manifestOf?.(targetRobot)?.sharedEnvironment;
  // Signed toward the arm being handed to, so a horizontal present reaches out
  // across the cell rather than away from it.
  const preferred = cellBaselineAxis(ctx.manifest.sharedEnvironment, receiver) || [0, 0, -1];
  // No mounting axis is passed: the driver already falls back to its own
  // mounting when handed no axis, and null is how that is asked for.
  const axis = giverPresentAxis({ mode: configured, mountingAxis: null, preferred });
  if (!axis) {
    console.warn(
      `[LowerThing:${ctx.manifest.name}] unknown handoverPresentMode `
      + `${JSON.stringify(configured)}; presenting along this robot's mounting instead.`
    );
  }
  return axis;
}

// Whether this robot plans its handover legs at all. Turned off, it falls back
// to the plain position-only Cartesian move with the standard collision guards,
// and none of the pose-locking, direction search or two-leg planning runs.
//
// Asked per ROLE, because a robot in the middle of a chain plays both and does
// not need the same thing from each. Pinning buys a giver a repeatable
// presentation -- without it the free wrist chooses, and the receiver aims
// relative to whatever it happened to choose. A receiver gets less from it and
// pays more: the pinned approach is solved by a randomised, time-bounded
// search, so an arm that reaches the rendezvous reliably when it is free can
// start missing intermittently once pinned, and three misses exhaust the
// rendezvous retries and fail the leg.
//
// The robot-level flag remains the default for both roles, so nothing that did
// not ask for a split behaves differently.
function handoverPoseLockEnabled(ctx, role) {
  const runtime = ctx.manifest.runtime || {};
  const perRole = role === "giver"
    ? runtime.enableHandoverPoseLockAsGiver
    : runtime.enableHandoverPoseLockAsReceiver;
  if (perRole !== undefined && perRole !== null) return perRole !== false;
  return runtime.enableHandoverPoseLock !== false;
}

async function setHandoverPoseLock(sim, scriptHandle, enabled, axis = null, alternates = [], rolls = []) {
  // Both roles approach the rendezvous with the tool axis pinned, so the plan's
  // constant jaw offset describes the arm it is actually steering. Passing no
  // axis leaves the robot on its own mounting direction, which is the giver's
  // case and, for a pair of arms facing each other, already the opposed axis.
  const axisArg = Array.isArray(axis) && axis.length === 3 ? `, ${luaArray(axis)}` : "";
  const alternateArg = axisArg && alternates.length > 0
    ? `, ${luaArray(alternates.map(luaArray))}`
    : "";
  // A roll list needs the alternates slot filled to reach its own, so an empty
  // one is still passed positionally when there are rolls to send.
  const rollArg = axisArg && rolls.length > 0
    ? `${alternateArg || ", {}"}, ${luaArray(rolls)}`
    : alternateArg;
  return executeInScript(
    sim,
    scriptHandle,
    `return womatSafeCall(function() return womatSetHandoverPoseLock(`
      + `${enabled ? "true" : "false"}${axisArg}${rollArg}) end)`
  );
}

async function beginHandoverSession(ctx, sim, scriptHandle, peers, sessions, sessionId, targetRobot, cubeWorld = null) {
  if (!peers) return { success: false, reason: "cell_peers_unavailable" };
  const configuredPairs = ctx.manifest.sharedEnvironment?.interactionPairs;
  if (Array.isArray(configuredPairs)) {
    const pair = configuredPairs.find((candidate) => Array.isArray(candidate.robots) &&
      candidate.robots.includes(ctx.manifest.name) && candidate.robots.includes(targetRobot));
    if (!pair) return { success: false, reason: "robots_are_not_neighbors", targetRobot };
    if (pair.ownerRobot !== ctx.manifest.name) {
      return { success: false, reason: "not_interaction_space_owner", ownerRobot: pair.ownerRobot };
    }
  } else if (ctx.manifest.sharedEnvironment?.interactionSpaceOwner !== ctx.manifest.name) {
    return { success: false, reason: "not_interaction_space_owner" };
  }
  if (!sessionId) return { success: false, reason: "handover_session_id_missing" };
  if (!peers.has(targetRobot)) {
    return { success: false, reason: "unknown_target_robot", targetRobot };
  }
  if (targetRobot === ctx.manifest.name) {
    return { success: false, reason: "source_and_target_are_equal", targetRobot };
  }
  // Asked of the scene rather than of a variable this process happens to hold.
  // The giver is the robot whose gripper has the cube parented to it, and if
  // that is not this robot then no amount of local bookkeeping makes it so.
  const ownership = await peers.resolveCubeOwnership();
  if (ownership.contested) {
    return { success: false, reason: "cube_held_by_multiple_robots", holders: ownership.holders };
  }
  if (ownership.owner !== ctx.manifest.name) {
    return { success: false, reason: "source_not_cube_owner", owner: ownership.owner };
  }
  const existingSession = sessions.blockingSession(ctx.manifest.name);
  if (existingSession) {
    return { success: false, reason: "handover_session_active", session: existingSession.id };
  }
  const record = sessions.open({
    id: String(sessionId),
    sourceRobot: ctx.manifest.name,
    targetRobot,
    state: "registered"
  });
  const zone = sessionZone(ctx.manifest, peers.manifestOf(targetRobot), cubeWorld);
  if (zone) record.handoverZone = zone;
  // The giver's next move is its leg of the rendezvous; the receiver arms its
  // own lock in setHandoverApproach, immediately before its approach. Record
  // the axis the giver will present the cube along, because the receiver's
  // approach is defined relative to it.
  const poseLocked = handoverPoseLockEnabled(ctx, "giver");
  const presentAxis = giverPresentAxisFor(ctx, peers, targetRobot);
  const poseLock = poseLocked
    ? await setHandoverPoseLock(sim, scriptHandle, true, presentAxis)
    : null;
  // The giver stops treating the receiver's gripper as an obstacle for the
  // whole session, not only after the cube has crossed.
  //
  // A rendezvous ENDS with the two grippers interleaved around one cube --
  // that is what a handover is -- so from the moment the receiver arrives the
  // giver is in contact with it and cannot plan its way anywhere, including
  // out. Arming this only on a successful transfer meant the one case where
  // the giver most needs to leave, a handover that did not complete, was the
  // one case where it could not: live cell, Robot2 refused its own way home on
  // Robot4's leftPad and stood in the interaction point. The RG2 pairs never
  // showed it because their transfers succeeded and armed it in passing.
  //
  // The giver moves to the rendezvous BEFORE the receiver approaches, so this
  // costs no real guard: the receiver is parked elsewhere for the giver's own
  // leg, and every other robot, wall and support stays a hard obstacle.
  const receiverGripper = peers.manifestOf(targetRobot)?.runtime?.gripper?.modelName
    || `${targetRobot}_tcp_gripper`;
  const receiverPadAlias = peers.manifestOf(targetRobot)?.runtime?.gripper?.graspPadAlias || "";
  await executeInScript(
    sim,
    scriptHandle,
    `return womatSafeCall(function() return womatSetHandoverApproachGripper(`
      + `${luaString(receiverGripper)}, ${luaString(receiverPadAlias)}) end)`
  );
  // Read the axis even when this robot plans nothing: it is the direction the
  // interaction-space plan assumes it presents along, and the receiver's
  // approach is defined relative to it.
  //
  // An unlocked giver is asked where its tool is actually POINTING, not where
  // its mounting says it would point. A position-only move leaves the wrist
  // free, so the two are unrelated -- live cell: Robot2 stood at the rendezvous
  // holding the cube with its tool straight up while its mounting axis was
  // horizontal and aimed away from Robot4, and Robot4's opposed approach was
  // therefore solved against a direction nobody was holding. The mounting
  // remains the last resort, for a robot with no tool frame to measure yet.
  const reportedAxis = poseLock?.axis
    ?? await executeInScript(sim, scriptHandle, "return womatToolDirectionWorld()")
    ?? await executeInScript(sim, scriptHandle, "return womatHandoverApproachAxisWorld()");
  const giverAxis = normalizedVector(asArray(reportedAxis).map(Number));
  record.giverAxis = giverAxis;
  // The receiver needs its own copy before it can arm its approach. In one
  // process this hands over the record; across processes it is a message to
  // that robot's own core, and the giver does not wait on the receiver's
  // bookkeeping to call its own registration done.
  const replicated = await peers.openSessionOn(targetRobot, record);
  return {
    success: true,
    state: "handover_session_registered",
    session: String(sessionId),
    sourceRobot: ctx.manifest.name,
    targetRobot,
    poseLock: poseLocked ? poseLock?.state : "handover_pose_lock_disabled",
    giverAxis,
    receiverNotified: replicated.success === true
  };
}

async function transferCubeOwnership(ctx, sim, scriptHandle, peers, sessions, sessionId, targetRobot) {
  if (!peers) {
    return { success: false, reason: "cell_peers_unavailable" };
  }
  if (!peers.has(targetRobot)) {
    return {
      success: false,
      reason: "unknown_target_robot",
      targetRobot,
      availableRobots: peers.names()
    };
  }
  if (targetRobot === ctx.manifest.name) {
    return { success: false, reason: "source_and_target_are_equal", targetRobot };
  }
  const session = sessions.get(sessionId);
  if (!session) {
    return { success: false, reason: "handover_session_missing_or_mismatched", session: String(sessionId || "") };
  }
  if (session.sourceRobot !== ctx.manifest.name || session.targetRobot !== targetRobot) {
    return { success: false, reason: "handover_session_robot_mismatch", session: session.id };
  }
  // The transfer lock belongs to the giver running it, not to the cell. Two
  // unrelated pairs handing over at the same time were previously refused by a
  // single shared flag for no reason either of them could see.
  if (!sessions.beginTransfer()) {
    return { success: false, reason: "cube_transfer_busy", sourceRobot: ctx.manifest.name };
  }
  try {
    const sourceState = await executeInScript(sim, scriptHandle, "return getGripperState()");
    if (!sourceState?.holding) {
      return { success: false, reason: "source_not_holding_cube", sourceRobot: ctx.manifest.name };
    }
    const ownership = await peers.resolveCubeOwnership();
    if (ownership.contested) {
      return { success: false, reason: "cube_held_by_multiple_robots", holders: ownership.holders };
    }
    if (ownership.owner !== ctx.manifest.name) {
      return {
        success: false,
        reason: "cube_owned_by_other_robot",
        owner: ownership.owner
      };
    }

    const targetState = await peers.gripperState(targetRobot);
    if (targetState?.holding) {
      return { success: false, reason: "target_already_holding_object", targetRobot };
    }

    const accepted = await peers.acceptTransfer(targetRobot);
    if (!accepted?.success) {
      return {
        success: false,
        reason: "target_accept_failed",
        targetRobot,
        detail: accepted
      };
    }

    // Ownership is not published here, because it is not stored anywhere to
    // publish: the receiver now has the cube parented to its gripper, and that
    // parenting IS the ownership. The next reader asks the scene and gets the
    // new answer without anyone having written it down.
    const relinquished = await executeInScript(
      sim,
      scriptHandle,
      "return womatSafeCall(relinquishTransferredGripObject)"
    );
    if (!relinquished?.success) {
      return {
        success: false,
        reason: "source_relinquish_failed",
        sourceRobot: ctx.manifest.name,
        targetRobot,
        detail: relinquished
      };
    }

    if (session.sourceRobot === ctx.manifest.name && session.targetRobot === targetRobot) {
      // Re-armed rather than armed: beginHandoverSession already did it for the
      // whole session. Kept because it is also the repair for a giver whose
      // driver was re-initialised mid-session, and it costs one call.
      const targetGripper = peers.manifestOf(targetRobot)?.runtime?.gripper?.modelName
        || `${targetRobot}_tcp_gripper`;
      const targetPadAlias = peers.manifestOf(targetRobot)?.runtime?.gripper?.graspPadAlias || "";
      await executeInScript(
        sim,
        scriptHandle,
        `return womatSafeCall(function() return womatSetHandoverApproachGripper(`
          + `${luaString(targetGripper)}, ${luaString(targetPadAlias)}) end)`
      );
      session.state = "transferred";
      await peers.updateSessionOn(targetRobot, session.id, { state: "transferred" });
    }

    return {
      success: true,
      state: "cube_transferred",
      object: peers.cube || sourceState.object,
      sourceRobot: ctx.manifest.name,
      targetRobot,
      validation: "receiver_jaw_distance",
      graspDistance: accepted.distance,
      graspTolerance: accepted.tolerance,
      placement: accepted.placement || "grasped_in_place",
      retreatCollisionMode: "ignore_peer_gripper_until_explicitly_disabled"
    };
  } finally {
    sessions.endTransfer();
  }
}

async function setHandoverApproach(ctx, sim, scriptHandle, peers, sessions, sessionId, enabled) {
  if (!peers) {
    return { success: false, reason: "cell_peers_unavailable" };
  }
  const activeSession = sessions.get(sessionId);
  if (!activeSession) {
    return { success: false, reason: "handover_session_missing_or_mismatched" };
  }
  if (!enabled) {
    await setHandoverPoseLock(sim, scriptHandle, false);
    const result = await executeInScript(
      sim,
      scriptHandle,
      "return womatSafeCall(womatClearHandoverApproachGripper)"
    );
    // Each robot closes its own copy. The reference count this replaces existed
    // only because one object was standing in for two robots' bookkeeping.
    sessions.close(sessionId);
    // Except when this robot is the giver walking away from a rendezvous the
    // cube never crossed. The receiver was handed a replica it will never use,
    // and it has no intention running that would ever clear it -- the receiver
    // only tears its copy down at the end of a handover it actually performed.
    // Whoever abandons the session takes the replica back with it. After a
    // transfer this does not run: both robots are then finishing a handover
    // that happened, and each still needs its own copy to do that.
    if (activeSession.sourceRobot === ctx.manifest.name && activeSession.state === "registered") {
      await peers.closeSessionOn(activeSession.targetRobot, sessionId);
    }
    return result;
  }

  const session = activeSession;
  if (session.targetRobot !== ctx.manifest.name) {
    return {
      success: false,
      reason: "handover_approach_receiver_only",
      receiver: session.targetRobot
    };
  }
  if (session.state !== "registered") {
    return { success: false, reason: "handover_session_not_active", state: session.state };
  }
  const giverManifest = peers.manifestOf(session.sourceRobot);
  const giverGripper = giverManifest?.runtime?.gripper?.modelName || `${session.sourceRobot}_tcp_gripper`;
  // Which of the giver's shapes are its contact pads. Only the giver's own
  // manifest knows -- the cell has RG2s calling them "touch" and a Baxter
  // gripper calling them "pad" -- and without it the receiver cannot see which
  // way the giver's jaws close, which is what its own roll is chosen against.
  const giverPadAlias = giverManifest?.runtime?.gripper?.graspPadAlias || "";
  const result = await executeInScript(
    sim,
    scriptHandle,
    `return womatSafeCall(function() return womatSetHandoverApproachGripper(`
      + `${luaString(giverGripper)}, ${luaString(giverPadAlias)}, ${luaString(`${session.sourceRobot}_tcp`)}) end)`
  );
  if (!result?.success) {
    return { ...result, session: session.id, receiver: session.targetRobot };
  }

  if (!handoverPoseLockEnabled(ctx, "receiver")) {
    // Plain Cartesian approach with the standard collision guards. Nothing is
    // pinned, so where the jaws end up is whatever the free wrist chooses --
    // see handoverGraspTolerance, which is what then decides whether the
    // transfer is allowed.
    return {
      ...result,
      session: session.id,
      receiver: session.targetRobot,
      approachMode: "simple_cartesian",
      poseLock: "handover_pose_lock_disabled"
    };
  }

  const mode = handoverApproachMode(ctx, session.sourceRobot);
  // An explicitly configured axis wins; otherwise come in across the cell,
  // along the line between the two bases. Overhead is only the last resort,
  // for a layout where that line says nothing (bases stacked, or the giver
  // holding the cube along the baseline).
  const configured = ctx.manifest.runtime?.handoverPerpendicularAxis;
  const preferred = Array.isArray(configured)
    ? configured.map(Number)
    : cellBaselineAxis(
        ctx.manifest.sharedEnvironment,
        giverManifest?.sharedEnvironment
      ) || [0, 0, -1];
  // Read after the giver reaches the rendezvous: its wrist may have moved
  // since registration, especially when the giver uses position-only IK.
  const giverAxis = normalizedVector(asArray(result.peerToolAxis).map(Number));
  if (!giverAxis && (mode === "opposed" || mode === "perpendicular")) {
    await setHandoverPoseLock(sim, scriptHandle, false);
    await executeInScript(sim, scriptHandle, "return womatSafeCall(womatClearHandoverApproachGripper)");
    return { success: false, reason: "handover_giver_tool_axis_unavailable", session: session.id };
  }
  const candidates = receiverApproachAxisCandidates(giverAxis, mode, preferred, {
    count: ctx.manifest.runtime?.handoverApproachCandidates ?? 8,
    maxTurnDeg: ctx.manifest.runtime?.handoverApproachMaxTurnDeg ?? 60,
    fixedMaxTurnDeg: ctx.manifest.runtime?.handoverFixedApproachMaxTurnDeg ?? 25
  });
  // Whether this arm also chooses where its fingers land. The pinned axis says
  // which way the tool points and leaves the rotation about it free, and that
  // rotation is what decides whether two grippers closing on the same cube
  // interleave or drive through each other. An arm whose gripper is long enough
  // to keep its wrist out of the giver's does not need to care; a short one
  // reaching into an RG2 does. Off by default, because searching it is only
  // worth its cost where the geometry is that tight.
  const rolls = ctx.manifest.runtime?.enableHandoverRollSearch === true
    ? approachRollCandidates({ rollStepDeg: ctx.manifest.runtime?.handoverApproachRollStepDeg })
    : [];
  const poseLock = await setHandoverPoseLock(
    sim, scriptHandle, true, candidates[0] || null, candidates.slice(1), rolls
  );
  return {
    ...result,
    session: session.id,
    receiver: session.targetRobot,
    approachMode: candidates.length > 0 ? mode : "robot_mounting_default",
    giverAxis,
    approachAxis: poseLock?.axis,
    alternateAxes: poseLock?.alternateAxes,
    approachRolls: poseLock?.rolls,
    giverClosingAxis: result?.peerClosingAxis
  };
}

module.exports = {
  beginHandoverSession,
  giverPresentAxisFor,
  handoverApproachMode,
  handoverPoseLockEnabled,
  setHandoverApproach,
  setHandoverPoseLock,
  transferCubeOwnership
};
