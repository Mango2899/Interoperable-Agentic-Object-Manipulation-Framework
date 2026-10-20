"use strict";

// The movement core: the planners and the moves they drive.
//
// Every function here takes the sim connection and one robot's script handle,
// so a robot's motions are independent of every other robot's by construction.

const path = require("path");

const { offsetWorldUp } = require("../handoverGeometry");
const { asArray } = require("../workspaceGrid");
const { luaArray } = require("./luaDriver");
const { delay } = require("./context");
const { executeInScript, simulationTimeStep } = require("./simConnection");
const { checkWorkspace } = require("./runtimeOptions");
const { prepareJointRoute, prepareSimpleCartesianRoute, followHandoverExit } = require("./handoverExit");
const { homeTrajectory } = require("./continuousHome");
const {
  cartesianDistance,
  cartesianStepCount,
  checkBodyCollisionIfEnabled,
  checkPointCollisionIfEnabled,
  getCartesian,
  getJointRadians,
  guardSimpleCartesianPose,
  interpolate,
  jointStepCount,
  motionExecutionError,
  preflightCartesianBodyPath,
  preflightCartesianPath,
  preflightJointPath,
  refreshObstacleBoxes,
  waitForMotionComplete
} = require("./safety");

// Industrial-style pick: solve the approach globally, get there in joint
// space, then descend vertically along a pre-validated path. No leg of this
// depends on incremental Cartesian IK holding the wrist steady.
async function topDownPick(sim, scriptHandle, point, workspace, options) {
  const graspLocal = offsetWorldUp(point, options.graspHeightOffset, options.worldUpLocal);

  // Which of the two motions this is. Explicitly nil-safe: an unset global
  // would read as "holding" under a bare inequality, and a PICK that took the
  // place branch would stop in mid-air and close on nothing.
  const placing = await executeInScript(
    sim, scriptHandle, "return heldGripObject ~= nil and heldGripObject ~= -1"
  ) === true;

  // A place is a hover and a release; a pick is a hover and a descent.
  //
  // The arm already holds the cube, so the cube is wherever the tool control
  // point is: stopping the tool one release height above the resting position
  // and opening the jaws drops the cube that far onto the face. Nothing has to
  // be threaded between the fingers, the cube and the support top on the way
  // down, because there is no way down -- and the descent was the leg that had
  // to solve a continuous vertical path with the cube in the jaws.
  //
  // A pick cannot do this. It has to ARRIVE at the cube before it can close on
  // it, so its hover is a staging point and the descent is the motion that
  // matters.
  const releasePoint = offsetWorldUp(graspLocal, options.placeReleaseHeight, options.worldUpLocal);
  const requested = placing ? releasePoint : graspLocal;

  // The CSV is a cloud of tip positions; with the tool vertical the flange
  // sits one tool length straight above the commanded point.
  const admits = (candidate) => (workspace
    ? workspace.admits(
      offsetWorldUp(candidate, options.toolLengthEstimate, options.worldUpLocal),
      options.workspaceTolerance
    )
    : { ok: true });
  const planAt = (candidate, maxTime) => executeInScript(
    sim,
    scriptHandle,
    placing
      ? `return womatPlanTopDownRelease(${candidate.x}, ${candidate.y}, ${candidate.z}, ${maxTime})`
      : `return womatPlanTopDownPick(${candidate.x}, ${candidate.y}, ${candidate.z}, `
        + `${options.pickupApproachHeight}, ${options.pickDescentSteps}, ${maxTime})`
  );

  let descentPoint = requested;
  const admission = admits(requested);
  let plan = null;
  let relocatedBy = null;

  // With face search disabled, placement uses exactly the requested center
  // and one planning budget. Failure must not silently move the drop point.
  if (!placing || options.enablePlaceFaceSearch === false) {
    plan = admission.ok ? await planAt(requested, options.pickPlanningMaxTime) : null;
  } else {
    // Putting the cube down is picking it up run backwards in every respect but
    // one: WHERE on the support it lands is free. A pick has to arrive at the
    // cube, wherever the cube happens to be standing; a place only has to leave
    // it somewhere on the face, fully supported.
    //
    // That freedom is worth spending, because a top-down pose costs reach -- the
    // tool must stand vertically above the point -- and a support near the limit
    // of an arm's envelope can have its CENTRE outside the top-down workspace
    // while most of its face is inside. Raising the release makes that strictly
    // worse, so no height rescues it and the failure reads as
    // no_collision_free_hover_config however the height is tuned.
    //
    // Swept cheaply first, and only then thoroughly. Every candidate the arm
    // CAN serve is solved by the planner's seeded solve almost immediately;
    // every one it cannot is outside the envelope, where searching longer finds
    // nothing that searching briefly did not. So a long budget is spent
    // exclusively on candidates that were never going to work -- on Robot4's
    // support_b, five rejections at 4.4 seconds each before the sixth candidate
    // planned in 53 ms. The short pass reaches the same answer in about a
    // tenth of the time, and the full budget still runs over the whole face if
    // the short pass came back empty, so nothing reachable is given up.
    const candidates = asArray(await executeInScript(
      sim,
      scriptHandle,
      `return womatTopDownDropCandidates(${requested.x}, ${requested.y}, ${requested.z}, `
        + `${options.placeFaceSearchSteps})`
    ))
      .map((candidate) => ({
        x: Number(candidate.x),
        y: Number(candidate.y),
        z: Number(candidate.z)
      }))
      .filter((candidate) => Number.isFinite(candidate.x)
        && Number.isFinite(candidate.y)
        && Number.isFinite(candidate.z)
        && cartesianDistance(candidate, requested) > 1e-6);

    // The point that was asked for leads, so a servable centre still wins.
    const targets = [requested, ...candidates];
    const budgets = options.placeFaceSearchMaxTime < options.pickPlanningMaxTime
      ? [options.placeFaceSearchMaxTime, options.pickPlanningMaxTime]
      : [options.pickPlanningMaxTime];

    for (const budget of budgets) {
      for (const target of targets) {
        if (!admits(target).ok) continue;
        const attempt = await planAt(target, budget);
        if (attempt && attempt.ok !== false) {
          descentPoint = target;
          plan = attempt;
          relocatedBy = cartesianDistance(target, requested);
          break;
        }
      }
      if (plan) break;
    }
    if (relocatedBy !== null && relocatedBy < 1e-6) relocatedBy = null;
  }

  // Only when nothing was planned. A place whose requested point sits outside
  // the sampled workspace can still be served elsewhere on the same face, and
  // that plan must not be thrown away for the sake of the point it replaced.
  if (!plan && !admission.ok) {
    return { success: false, ...admission, reason: admission.reason, target: point, stage: "admission" };
  }
  if (!plan || plan.ok === false) {
    // A pick's admission gate only covers the grasp, and the hover it failed to
    // solve is one approach height above that -- so say whether THAT point is
    // in the sampled workspace at all: "no_collision_free_hover_config" reads
    // like an obstacle problem when it is usually a reach problem. A place has
    // no second point to report: the release IS the commanded point, and the
    // gate above already covered it.
    const hoverHeight = placing ? 0 : options.pickupApproachHeight;
    const hoverAdmission = workspace
      ? workspace.admits(
          offsetWorldUp(requested, hoverHeight + options.toolLengthEstimate, options.worldUpLocal),
          options.workspaceTolerance
        )
      : null;
    return {
      success: false,
      reason: plan?.reason || "top_down_pick_unplannable",
      plannerMessage: plan?.message,
      blockedAtStep: plan?.blockedAtStep,
      motion: placing ? "release" : "pick",
      hoverHeight,
      hoverReachable: hoverAdmission ? hoverAdmission.ok : null,
      hoverAdmission: hoverAdmission && !hoverAdmission.ok ? hoverAdmission : undefined,
      target: point,
      stage: "planning"
    };
  }

  const hover = asArray(plan.hover).map(Number);
  // Empty for a release, by construction: there is no leg below the hover.
  const descent = asArray(plan.descent).map(Number);
  if (hover.length === 0 || (!placing && descent.length === 0)) {
    return { success: false, reason: "top_down_pick_empty_plan", target: point, stage: "planning" };
  }

  // A place is ONE leg -- the hover is where the cube is let go -- so its whole
  // path can be known before anything moves, and then it can be reserved as a
  // path rather than claimed as the regions it might cross. That is the
  // difference between the terminal receiver sharing the rendezvous region
  // with the arm that just handed to it, and holding the whole of that region
  // until the cube is on the support: Robot4 stood at the rendezvous for the
  // entire placement, unable to start home. A pick is two legs, the second a
  // descent whose path is only valid once the first has arrived, and cannot be
  // reserved in one piece -- refused rather than run unreserved.
  if (options.executeReservedPath) {
    if (!placing) {
      return { success: false, reason: "pick_legs_not_reservable", safeToReplan: true, target: point, stage: "approach" };
    }
    const from = asArray(await executeInScript(sim, scriptHandle, "return womatCurrentConfig()")).map(Number);
    const dt = await simulationTimeStep(sim, scriptHandle);
    let approachRoute = "joint_interpolation";
    let executed = await options.executeReservedPath(homeTrajectory(from, hover, options, dt).flat(), hover.length, true);
    // The straight line refused before anything moved: route around, and
    // reserve THAT path instead, exactly as the unreserved branch below falls
    // back to the planner.
    if (!executed.success && executed.safeToReplan && options.enablePathPlanning) {
      const planned = await executeInScript(sim, scriptHandle,
        `return womatPlanJointPath(${luaArray(hover)}, ${options.planningMaxTimeSeconds}, ${options.planningStateCount})`);
      if (planned?.ok) {
        approachRoute = "planned_path";
        executed = await options.executeReservedPath(asArray(planned.path).map(Number), hover.length, true);
      }
    }
    if (!executed.success) return { ...executed, target: point, stage: "approach" };
    const actual = await getCartesian(sim, scriptHandle);
    const finalError = cartesianDistance(actual, descentPoint);
    if (finalError > options.cartesianFinalTolerance) {
      return { success: false, reason: "pick_target_not_reached", planner: "top_down_release", approachRoute,
        target: descentPoint, actual, finalError, tolerance: options.cartesianFinalTolerance, stage: "release" };
    }
    return {
      success: true,
      state: "cartesian_motion_complete",
      planner: "top_down_release",
      approachRoute,
      releaseHeight: options.placeReleaseHeight,
      retreatPoint: offsetWorldUp(descentPoint, options.pickupApproachHeight, options.worldUpLocal),
      actual,
      finalError,
      dropPoint: descentPoint,
      ...(relocatedBy === null ? {} : { dropRelocatedBy: relocatedBy }),
      toolControlPoint: await executeInScript(sim, scriptHandle, "return womatToolControlPointWorld()")
    };
  }

  // Leg 1: joint-space move to the hover configuration. A joint move cannot
  // diverge, which is the whole point of solving the pose first. A straight
  // interpolation is tried before the planner because it is smooth and
  // predictable -- but the hover pose is reached from wherever the arm happens
  // to be standing, and from the upright home posture that straight line sweeps
  // through the cell wall. A reachable pick must not fail because of the route
  // taken to it, so the planner routes around instead, exactly as the handover
  // standoff does.
  const start = await executeInScript(sim, scriptHandle, "return womatCurrentConfig()");
  const approachSteps = jointStepCount(asArray(start).map(Number), hover, options);
  const approachSafety = await preflightJointPath(
    sim, scriptHandle, asArray(start).map(Number), hover, options, approachSteps
  );
  let approachRoute = "joint_interpolation";
  if (!approachSafety.ok) {
    if (!options.enablePathPlanning) {
      return { success: false, ...approachSafety, target: point, stage: "approach" };
    }
    const planned = await executeInScript(
      sim,
      scriptHandle,
      `return womatPlanAndFollowJointPath(${luaArray(hover)}, `
        + `${options.planningMaxTimeSeconds}, ${options.planningStateCount})`
    );
    if (!planned || planned.ok === false) {
      return {
        success: false,
        ...approachSafety,
        reason: planned?.reason === "no_safe_path_found" ? "pick_hover_unreachable" : approachSafety.reason,
        plannerMessage: planned?.message,
        plannerReason: planned?.reason,
        target: point,
        stage: "approach"
      };
    }
    approachRoute = "planned_path";
  } else {
    await executeInScript(sim, scriptHandle, `return womatStartSimpleJoint(${luaArray(hover)})`);
  }
  if (!await waitForMotionComplete(sim, scriptHandle)) {
    return { success: false, reason: "pick_approach_timeout", target: point, stage: "approach" };
  }
  let executionError = await motionExecutionError(sim, scriptHandle);
  if (executionError) {
    return { success: false, reason: "motion_execution_failed", plannerMessage: executionError, stage: "approach" };
  }

  // Leg 2: the vertical descent, already collision-checked at plan time. A
  // release has none -- the arm is standing where it lets go, and the cube
  // makes the rest of the trip on its own.
  if (descent.length > 0) {
    await executeInScript(sim, scriptHandle, `return womatFollowDescent(${luaArray(descent)})`);
    const settled = await waitForMotionComplete(
      sim, scriptHandle, Math.max(900, descent.length + options.pathSettleMaxSteps + 100)
    );
    if (!settled) {
      return { success: false, reason: "pick_descent_timeout", target: point, stage: "descent" };
    }
    executionError = await motionExecutionError(sim, scriptHandle);
    if (executionError) {
      return { success: false, reason: "motion_execution_failed", plannerMessage: executionError, stage: "descent" };
    }
  }

  // The plan is verified against the scene before it is accepted, but nothing
  // until now checked that EXECUTING it arrived. A pick that reports success
  // with the jaws a third of a metre from the cube is worse than one that
  // fails: the agent goes on to close the gripper on air and the reason it
  // gets back names the gripper, not the move that missed.
  const actual = await getCartesian(sim, scriptHandle);
  // Measured against the point the plan actually aimed at, which is not the
  // commanded one when a place had to move off the centre of the face. Judging
  // it against the request would call every relocated placement a miss.
  const finalError = cartesianDistance(actual, descentPoint);
  if (finalError > options.cartesianFinalTolerance) {
    return {
      success: false,
      reason: "pick_target_not_reached",
      planner: placing ? "top_down_release" : "top_down_pick",
      approachRoute,
      target: descentPoint,
      actual,
      finalError,
      tolerance: options.cartesianFinalTolerance,
      stage: placing ? "release" : "descent"
    };
  }
  return {
    success: true,
    state: "cartesian_motion_complete",
    planner: placing ? "top_down_release" : "top_down_pick",
    approachRoute,
    descentStates: plan.descentStates,
    // How far the cube falls once the jaws open. Nothing below this point was
    // planned or swept, so it is reported rather than implied.
    ...(placing ? {
      releaseHeight: options.placeReleaseHeight,
      retreatPoint: offsetWorldUp(descentPoint, options.pickupApproachHeight, options.worldUpLocal)
    } : {}),
    actual,
    finalError,
    // Where the tool actually came down. The caller retreats from here rather
    // than from what it asked for: a relocated placement leaves the cube off
    // centre, and a retreat aimed back over the centre would drag the open
    // fingers straight through the cube that was just released.
    dropPoint: descentPoint,
    ...(relocatedBy === null ? {} : { dropRelocatedBy: relocatedBy }),
    toolControlPoint: await executeInScript(sim, scriptHandle, "return womatToolControlPointWorld()")
  };
}

// The approach axis that last worked, per candidate list.
//
// The search is expensive in a very particular way: a rejected axis is rejected
// by an IK solve that runs to its own timeout before giving up. Robot2's
// rendezvous took SEVEN rejections before the eighth candidate planned, six of
// them a full findConfig budget spent on a seed that had already diverged -- and
// then did exactly the same thing again on the next approach, rejecting the same
// seven axes in the same order to arrive at the same answer.
//
// The geometry does not change between two approaches to the same rendezvous, so
// neither does the answer. Keyed on the candidate list itself: a different mode,
// a different partner or a different cone produces a different list and gets no
// stale advice. Nothing is skipped -- the remembered axis is tried FIRST and the
// rest still follow it, so a memo that has gone stale costs one attempt and then
// behaves exactly as before.
const rememberedApproachAxis = new Map();

// The last leg into a rendezvous, kept so the way out can be its reverse.
//
// Leaving is not a move to a point, and treating it as one is what put both
// arms wrong. A pose-locked arm has to back out while the lock is still armed
// -- the lock is also what keeps the other robot's gripper out of its obstacle
// set -- and while it is armed EVERY Cartesian move is dispatched as an
// approach: standoff first, then straight in. So a move to a point ten
// centimetres behind the rendezvous planned an arrival at that point, and the
// arm backed off and then drove forward again, through the rendezvous it was
// leaving, with the other arm still standing in it. An unlocked arm has the
// milder version of the same problem: it backs straight down its own local -X,
// which is a direction chosen for its base and not for the way it came in.
//
// Either way the answer is the same, and it is the leg the arm just walked.
// Pose-locked, that is a joint path checked one waypoint at a time against the
// cell as it stood with BOTH arms in it, and its reverse passes through no pose
// the arm has not just occupied. Unlocked, it is the straight Cartesian line
// from wherever the approach started, and its reverse is that line run the
// other way. Keyed by script handle: one entry per robot, and a robot is in at
// most one rendezvous at a time.
const executedRendezvousLeg = new Map();

function approachAxisMemoKey(candidates) {
  return JSON.stringify(candidates.map((axis) => (axis ? axis.map((value) => Number(value.toFixed(6))) : null)));
}

// The winning roll, remembered per axis set for the same reason the winning
// axis is: it is a property of this pair of grippers meeting this way, so it is
// the same answer next handover, and finding it again from scratch costs a
// couple of dozen refused plans every time.
const rememberedApproachRoll = new Map();

function orderByRememberedAxis(candidates) {
  const remembered = rememberedApproachAxis.get(approachAxisMemoKey(candidates));
  if (remembered === undefined || remembered <= 0 || remembered >= candidates.length) return candidates;
  const ordered = [...candidates];
  const [winner] = ordered.splice(remembered, 1);
  ordered.unshift(winner);
  return ordered;
}

// The handover rendezvous, solved the same way as the pick: one pose, verified,
// then a joint move onto it. The difference is the constraint -- the tool axis
// is pinned to the direction the configured jaw offset was measured along, so
// the interaction-space plan's constant offset describes the arm that actually
// arrives.
async function handoverApproach(sim, scriptHandle, point, workspace, options) {
  // A plan that fails below leaves the arm part-way along a path whose reverse
  // goes nowhere useful, so nothing is left claiming to lead back out until one
  // succeeds. (The caller has already cleared it; this holds for a direct call.)
  executedRendezvousLeg.delete(scriptHandle);
  // No pre-admission on the commanded point. It is a TIP target derived for one
  // particular approach direction, and a perpendicular receiver does not go
  // there -- it comes at the same cube from ninety degrees away, with its tip
  // somewhere else entirely. The planner's IK plus collision validation is the
  // real reachability test; the cloud only serves as a diagnostic below.
  //
  // The direction search runs HERE rather than inside the driver so that every
  // script call stays short. Looping over candidates in Lua multiplies one
  // call's worst case by the number of candidates, and a long-blocking script
  // call is what raises CoppeliaSim's modal abort dialog.
  const axes = await executeInScript(sim, scriptHandle, "return womatHandoverApproachAxisCandidates()");
  const declared = Array.isArray(axes) && axes.length > 0
    ? axes.map((axis) => asArray(axis).map(Number)).filter((axis) => axis.length === 3)
    : [null];
  const memoKey = approachAxisMemoKey(declared);
  const candidates = orderByRememberedAxis(declared);
  // The roll about each axis, when this robot searches one, already ordered by
  // the driver around the angle that crosses this gripper's jaws with the
  // giver's. Asked per axis because the prediction is measured in the frame
  // that axis builds.
  const rollsFor = new Map();
  for (const axis of candidates) {
    const key = approachAxisMemoKey([axis]);
    if (rollsFor.has(key)) continue;
    rollsFor.set(key, asArray(await executeInScript(
      sim, scriptHandle,
      `return womatHandoverApproachRollCandidates(${axis ? luaArray(axis) : "nil"})`
    )).map(Number).filter((roll) => Number.isFinite(roll)));
  }
  const rollCount = Math.max(...[...rollsFor.values()].map((list) => list.length), 0);

  // (direction, roll) pairs in the order to try them.
  //
  // Each direction gets a fair look near ITS predicted roll before any
  // direction is walked out in full. Exhausting the first direction's whole
  // turn before touching the second is what cost the live cell 129 refused
  // plans and 20 seconds: the primary direction had no workable roll at all,
  // and the answer was twelve degrees off it at a roll the search only reached
  // after all seventy-two of the primary's.
  //
  // The band is generous rather than tight because the prediction is a good
  // guess and not a derivation -- on the live pair it landed 16 degrees from
  // the only roll that worked.
  const band = Math.max(1, Math.round(Number(options.handoverApproachRollBand ?? 24)));
  const order = [];
  const rollsAt = (index) => rollsFor.get(approachAxisMemoKey([candidates[index]])) || [];
  for (const [from, to] of [[0, band], [band, rollCount]]) {
    for (let index = 0; index < candidates.length; index += 1) {
      const list = rollsAt(index);
      for (let depth = from; depth < Math.min(to, list.length); depth += 1) {
        order.push({ index, roll: list[depth] });
      }
    }
  }
  if (order.length === 0) {
    for (let index = 0; index < candidates.length; index += 1) order.push({ index, roll: null });
  }
  const remembered = rememberedApproachRoll.get(memoKey);
  if (remembered !== undefined) {
    // The pair already met this way once. That answer first, whole search behind it.
    const at = order.findIndex((entry) => entry.roll === remembered);
    if (at > 0) order.unshift(...order.splice(at, 1));
  }

  // Two passes over that order: once refusing postures that get to the
  // rendezvous the long way round, then, only if nothing at all was found, once
  // taking whatever is reachable.
  //
  // A pinned roll constrains the whole pose, and damped least squares answers
  // that from wherever the arm is standing -- sometimes by winding most of the
  // way round. Every such pose is valid and the journey to it is not: slow to
  // plan, slow to walk, and it leaves the arm somewhere it cannot reach home
  // from in a straight line. A long way in is still better than no way in,
  // which is why the bound is a preference and not a rule.
  const travelBoundDeg = rollCount > 0 ? Number(options.handoverApproachMaxTravelDeg ?? 150) : 0;
  const passes = travelBoundDeg > 0 ? [travelBoundDeg, 0] : [0];

  let plan = null;
  let axisIndex = -1;
  const rejected = [];
  let attempts = 0;
  // Never below one full turn of rolls: a cap that stops a sweep part-way
  // through reports "nothing works" about angles it never tried, and the window
  // that works can be ten degrees wide.
  const maxAttempts = Math.max(
    rollCount,
    Math.max(1, Number(options.handoverApproachMaxAttempts ?? 64))
  );
  search:
  for (const travelDeg of passes) {
    for (const { index, roll } of order) {
      if (attempts >= maxAttempts) break search;
      attempts += 1;
      const axis = candidates[index];
      const axisArg = axis ? `, ${luaArray(axis)}` : "";
      // A roll only means anything alongside an axis: the angle is measured in
      // the frame that axis builds, and the travel bound only alongside a roll.
      const rollArg = axisArg && roll !== null ? `, ${roll}` : "";
      const travelArg = rollArg && travelDeg > 0 ? `, ${travelDeg}` : "";
      const attempt = await executeInScript(
        sim,
        scriptHandle,
        `return womatPlanHandoverApproach(${point.x}, ${point.y}, ${point.z}, `
          + `${options.handoverStandoffDistance}, ${options.handoverApproachSteps}, `
          + `${options.handoverPlanningMaxTime}${axisArg}${rollArg}${travelArg})`
      );
      if (attempt && attempt.ok !== false) {
        plan = attempt;
        axisIndex = index;
        // Recorded against the DECLARED order, so the memo means the same thing
        // whether this attempt was itself reordered or not.
        const declaredIndex = declared.findIndex((declaredAxis) => approachAxisMemoKey([declaredAxis]) === approachAxisMemoKey([axis]));
        if (declaredIndex >= 0) rememberedApproachAxis.set(memoKey, declaredIndex);
        if (roll !== null) rememberedApproachRoll.set(memoKey, roll);
        break search;
      }
      rejected.push({
        axis,
        roll: roll === null ? undefined : roll,
        travelBoundDeg: travelDeg || undefined,
        reason: attempt?.reason,
        blockedAtStep: attempt?.blockedAtStep,
        detail: attempt?.detail,
        seedFailure: attempt?.seedFailure,
        obstacle: attempt?.obstacle,
        obstacleOwner: attempt?.obstacleOwner,
        selfCollisionA: attempt?.selfCollisionA,
        selfCollisionB: attempt?.selfCollisionB
      });
    }
  }

  if (!plan) {
    // The commanded point IS the grasp point now: simTip is the tool control
    // point, so there is nothing to add back.
    const graspLocal = point;
    const graspAdmission = workspace ? workspace.admits(graspLocal, options.workspaceTolerance) : null;
    const last = rejected[rejected.length - 1] || {};
    return {
      success: false,
      reason: last.reason || "handover_approach_unplannable",
      blockedAtStep: last.blockedAtStep,
      seedFailure: last.seedFailure,
      obstacle: last.obstacle,
      obstacleOwner: last.obstacleOwner,
      selfCollisionA: last.selfCollisionA,
      selfCollisionB: last.selfCollisionB,
      standoffDistance: options.handoverStandoffDistance,
      graspPoint: graspLocal,
      graspReachable: graspAdmission ? graspAdmission.ok : null,
      graspAdmission: graspAdmission && !graspAdmission.ok ? graspAdmission : undefined,
      axisCandidates: candidates.length,
      rejectedAxes: rejected,
      // Every axis failed, so whatever was remembered is wrong. Left in place
      // would cost one wasted attempt on every future approach.
      axisMemoCleared: rememberedApproachAxis.delete(memoKey) || undefined,
      rollMemoCleared: rememberedApproachRoll.delete(memoKey) || undefined,
      rollCandidates: rollCount || undefined,
      attempts,
      target: point,
      stage: "planning"
    };
  }

  const standoff = asArray(plan.standoff).map(Number);
  const approach = asArray(plan.approach).map(Number);
  if (standoff.length === 0 || approach.length === 0) {
    return { success: false, reason: "handover_approach_empty_plan", target: point, stage: "planning" };
  }

  let standoffRoute = "joint_interpolation";
  if (options.executeReservedPath) {
    const route = await prepareJointRoute(sim, scriptHandle, standoff, options);
    if (!route.success) return { ...route, safeToReplan: true };
    const executed = await options.executeReservedPath([...route.path, ...approach], standoff.length);
    if (!executed.success) return executed;
    standoffRoute = "handover_zone_path";
  } else {
    // Leg 1: get to the standoff. A straight joint interpolation is tried first
    // because it is smooth and predictable, but the giver is already parked in
    // the shared volume, so when that line sweeps through it the planner has to
    // route around instead of the move simply failing.
    const start = asArray(await executeInScript(sim, scriptHandle, "return womatCurrentConfig()")).map(Number);
    const steps = jointStepCount(start, standoff, options);
    const safety = await preflightJointPath(sim, scriptHandle, start, standoff, options, steps);
    if (!safety.ok) {
      if (!options.enablePathPlanning) {
        return { success: false, ...safety, target: point, stage: "standoff" };
      }
      const planned = await executeInScript(
        sim,
        scriptHandle,
        `return womatPlanAndFollowJointPath(${luaArray(standoff)}, `
          + `${options.planningMaxTimeSeconds}, ${options.planningStateCount})`
      );
      if (!planned || planned.ok === false) {
        return {
          success: false,
          ...safety,
          reason: planned?.reason === "no_safe_path_found" ? "handover_standoff_unreachable" : safety.reason,
          plannerMessage: planned?.message,
          plannerReason: planned?.reason,
          target: point,
          stage: "standoff"
        };
      }
      standoffRoute = "planned_path";
    } else {
      await executeInScript(sim, scriptHandle, `return womatStartSimpleJoint(${luaArray(standoff)})`);
    }
    if (!await waitForMotionComplete(sim, scriptHandle)) {
      return { success: false, reason: "handover_standoff_timeout", target: point, stage: "standoff" };
    }
    let executionError = await motionExecutionError(sim, scriptHandle);
    if (executionError) {
      return { success: false, reason: "motion_execution_failed", plannerMessage: executionError, stage: "standoff" };
    }

    // Leg 2: the straight line in, already collision-checked at plan time.
    await executeInScript(sim, scriptHandle, `return womatFollowDescent(${luaArray(approach)})`);
    const settled = await waitForMotionComplete(
      sim, scriptHandle, Math.max(900, approach.length + options.pathSettleMaxSteps + 100)
    );
    if (!settled) {
      return { success: false, reason: "handover_approach_timeout", target: point, stage: "approach" };
    }
    executionError = await motionExecutionError(sim, scriptHandle);
    if (executionError) {
      return { success: false, reason: "motion_execution_failed", plannerMessage: executionError, stage: "approach" };
    }
  }

  executedRendezvousLeg.set(scriptHandle, {
    kind: "pose_locked_approach",
    standoff,
    approach,
    axis: plan.axis,
    standoffDistance: plan.standoffDistance,
    target: point
  });

  return {
    success: true,
    state: "cartesian_motion_complete",
    planner: "handover_pose_locked",
    standoffRoute,
    standoffSolve: plan.standoffRoute,
    standoffDistance: plan.standoffDistance,
    approachStates: plan.approachStates,
    approachAxis: plan.axis,
    // The angle the wrist was rolled to about that axis, when this robot
    // searched one. Absent means the roll was left free, which is every pair
    // that does not have to place its fingers around another gripper.
    approachRoll: plan.roll,
    // How far the furthest joint had to travel to reach the standoff. The
    // number that separates a rendezvous the arm steps into from one it winds
    // most of the way round to.
    approachTravelDeg: plan.travel,
    rollCandidates: rollCount || undefined,
    planAttempts: attempts,
    // Where it came in the order actually tried, and where it sits in the order
    // the mode declares. They differ once the memo has learned something, and a
    // reader needs the second to compare two runs.
    axisIndex,
    declaredAxisIndex: rememberedApproachAxis.get(memoKey),
    axisTriedFirst: axisIndex === 0,
    axisCandidates: candidates.length,
    rejectedAxes: rejected,
    jawPosition: plan.jawPosition
  };
}

// The unlocked reverse: the straight Cartesian line the arm came in on, run the
// other way, back to the pose the approach departed from.
//
// It is a longer move than the ten-centimetre back-off it replaces, and that is
// the point -- it ends where the arm was standing before it entered the shared
// volume, so the joint sweep that follows starts from outside it rather than
// from inside. Re-admitted and re-swept on the way out, because the cell it
// crosses is not the one the approach crossed: a cube has changed hands.
async function reverseCartesianLeg(sim, scriptHandle, record, workspace, options) {
  const actual = await getCartesian(sim, scriptHandle);
  const drift = cartesianDistance(actual, record.to);
  if (drift > options.cartesianFinalTolerance) {
    executedRendezvousLeg.delete(scriptHandle);
    return {
      success: false,
      reason: "handover_approach_path_stale",
      drift,
      tolerance: options.cartesianFinalTolerance
    };
  }

  let execution = await executeSimpleCartesianPose(sim, scriptHandle, record.from, workspace, options);
  if (!execution.success && execution.reason !== "motion_execution_failed"
    && execution.reason !== "cartesian_motion_timeout" && (!options.executeReservedPath || execution.safeToReplan)) {
    execution = await cartesianPlannerFallback(sim, scriptHandle, record.from, options, execution);
  }
  if (!execution.success) return { ...execution, stage: "retreat" };
  executedRendezvousLeg.delete(scriptHandle);

  const arrived = await getCartesian(sim, scriptHandle);
  const finalError = cartesianDistance(arrived, record.from);
  if (finalError > options.cartesianFinalTolerance) {
    return {
      success: false,
      reason: "handover_retreat_not_reached",
      target: record.from,
      actual: arrived,
      finalError,
      tolerance: options.cartesianFinalTolerance,
      stage: "retreat"
    };
  }
  return {
    success: true,
    state: "handover_retreat_complete",
    planner: "cartesian_leg_reversed",
    rendezvous: record.to,
    actual: arrived,
    finalError
  };
}

// Leaving a rendezvous, as the reverse of arriving at one. Whichever leg the
// arm actually walked in on is the leg it walks back out.
//
// Below is the pose-locked case: not a move to a point but a REPLAY, of leg 2
// of the approach, from the grasp pose back out to the standoff configuration
// leg 1 had left the arm in. The tool axis therefore stays pinned the whole way
// out exactly as it was on the way in, and the arm passes through no pose it
// has not already occupied. What happens after this -- the sweep to the
// all-zero joint pose -- then starts one standoff clear of the other robot
// instead of inside the shared volume.
async function handoverRetreat(sim, scriptHandle, workspace, options) {
  const record = executedRendezvousLeg.get(scriptHandle);
  if (!record) {
    return { success: false, reason: "no_recorded_handover_approach" };
  }
  if (record.kind === "cartesian_leg") {
    return reverseCartesianLeg(sim, scriptHandle, record, workspace, options);
  }
  const dof = record.standoff.length;
  const states = dof > 0 ? Math.floor(record.approach.length / dof) : 0;
  if (states === 0) {
    executedRendezvousLeg.delete(scriptHandle);
    return { success: false, reason: "recorded_handover_approach_empty" };
  }

  // The reverse is the way out only while the arm is still standing at the end
  // of the way in. Anything that moved it since -- a home the agent asked for
  // first, a failed move, a drag in the scene -- turns the first waypoint into
  // a jump, and a jump is precisely the thing nothing has checked.
  const current = asArray(
    await executeInScript(sim, scriptHandle, "return womatCurrentConfig()")
  ).map(Number);
  const arrival = record.approach.slice((states - 1) * dof, states * dof);
  const drift = current.length === dof
    ? Math.max(...arrival.map((value, index) => Math.abs(current[index] - value)))
    : Infinity;
  if (!(drift <= options.handoverRetreatDriftTolerance)) {
    executedRendezvousLeg.delete(scriptHandle);
    return {
      success: false,
      reason: "handover_approach_path_stale",
      drift,
      tolerance: options.handoverRetreatDriftTolerance
    };
  }

  // Every state of the way in, last to first, and then the standoff itself --
  // which is not in the approach array, because leg 2 was solved FROM it and
  // its first sample is already one step in.
  const reversed = [];
  for (let state = states - 1; state >= 0; state -= 1) {
    for (let joint = 0; joint < dof; joint += 1) {
      reversed.push(record.approach[state * dof + joint]);
    }
  }
  for (let joint = 0; joint < dof; joint += 1) {
    reversed.push(record.standoff[joint]);
  }

  if (options.executeReservedPath) {
    const executed = await options.executeReservedPath(reversed, dof);
    if (!executed.success) return executed;
  } else {
    await executeInScript(sim, scriptHandle, `return womatFollowDescent(${luaArray(reversed)})`);
    const settled = await waitForMotionComplete(
      sim, scriptHandle, Math.max(900, reversed.length + options.pathSettleMaxSteps + 100)
    );
    if (!settled) {
      return { success: false, reason: "handover_retreat_timeout", stage: "retreat" };
    }
    const executionError = await motionExecutionError(sim, scriptHandle);
    if (executionError) {
      return {
        success: false,
        reason: "motion_execution_failed",
        plannerMessage: executionError,
        stage: "retreat"
      };
    }
  }

  // Walked. The recorded poses no longer describe where the arm is, so the
  // record is spent rather than left to be replayed a second time.
  executedRendezvousLeg.delete(scriptHandle);
  return {
    success: true,
    state: "handover_retreat_complete",
    planner: "handover_approach_reversed",
    retreatStates: states,
    standoffDistance: record.standoffDistance,
    approachAxis: record.axis,
    rendezvous: record.target,
    actual: await getCartesian(sim, scriptHandle)
  };
}

// One straight Cartesian leg: admitted, swept, executed, waited on. Shared by
// the general move and by the reverse of one, so a retreat is guarded exactly
// as the approach it undoes was.
async function executeSimpleCartesianPose(sim, scriptHandle, target, workspace, options) {
  if (options.executeReservedPath) {
    const admission = checkWorkspace(target, workspace, options);
    if (!admission.ok) return { success: false, ...admission };
    const plan = await prepareSimpleCartesianRoute(sim, scriptHandle, target, options);
    if (!plan?.ok) return { success: false, reason: plan?.reason || "cartesian_route_unavailable", safeToReplan: true };
    return options.executeReservedPath(asArray(plan.path).map(Number), Number(plan.dof));
  }
  // Admission and the swept check run against the corrected tip target, not
  // the raw commanded point: the jaw-offset correction moves the tip by the
  // tool length, so validating the request would validate a pose the arm
  // never assumes.
  const guard = await guardSimpleCartesianPose(sim, scriptHandle, target, workspace, options);
  if (!guard.ok) return { success: false, ...guard };
  await executeInScript(
    sim,
    scriptHandle,
    `return womatStartSimpleCartesian(${target.x}, ${target.y}, ${target.z})`
  );
  const done = await waitForMotionComplete(sim, scriptHandle);
  if (!done) return { success: false, reason: "cartesian_motion_timeout" };
  const executionError = await motionExecutionError(sim, scriptHandle);
  if (executionError) {
    return { success: false, reason: "motion_execution_failed", plannerMessage: executionError };
  }
  return { success: true, safety: { samples: guard.samples } };
}

async function simpleSmoothCartesianMove(sim, scriptHandle, point, workspace, options) {
  await refreshObstacleBoxes(sim, scriptHandle, workspace);
  // Whichever branch below runs records its own leg. Clearing first means a
  // branch that records nothing -- a pick -- and a branch that fails part-way
  // both leave no path claiming to lead out of somewhere the arm no longer is.
  executedRendezvousLeg.delete(scriptHandle);

  // A pickup is a different manoeuvre from a general move, not a special case
  // of one: fixed approach orientation, solved before anything moves.
  if (options.enableTopDownPick) {
    const pickup = await executeInScript(sim, scriptHandle, "return womatPickupApproachEnabled");
    if (pickup === true) {
      return topDownPick(sim, scriptHandle, point, workspace, options);
    }
  }

  // So is a handover rendezvous, for the same reason.
  const handoverLocked = await executeInScript(sim, scriptHandle, "return womatHandoverPoseEnabled");
  if (handoverLocked === true) {
    return handoverApproach(sim, scriptHandle, point, workspace, options);
  }

  // Where the arm stands before this move is the far end of it, and the far end
  // is the only thing a reverse needs. Read here rather than remembered from
  // the last move's target: the arm may have been moved by something that is
  // not a move at all.
  const departure = await getCartesian(sim, scriptHandle);
  // The straight Cartesian line stays the first choice, as it was before the
  // concurrency work: position IK streams the tool along it and the move looks
  // like what it is. Solving a goal CONFIGURATION and interpolating to it in
  // joint space reaches the same point through whatever elbow the solver picked,
  // so the tool swings out and back on the way in -- which is what a rendezvous
  // approach started doing on every robot whose manifest sets
  // preferJointTrajectoryForCartesian. That joint route is the FALLBACK, for a
  // straight line the cell will not allow; it is not the opening move.
  let execution = await executeSimpleCartesianPose(sim, scriptHandle, point, workspace, options);
  // Every robot keeps the whole pipeline. returnHomeBetweenHandovers is a
  // handover route (receive, go home, give), read by the agent; it says nothing
  // about how a single move is made safe.
  if (!execution.success && execution.reason !== "motion_execution_failed"
    && execution.reason !== "cartesian_motion_timeout" && (!options.executeReservedPath || execution.safeToReplan)) {
    execution = await cartesianPlannerFallback(sim, scriptHandle, point, options, execution);
  }
  if (!execution.success) return execution;

  // simTip is the tool control point, so the commanded position IS where the
  // gripper ends up. There is nothing left to correct for: the jaw-offset
  // calibration this used to run existed only because the arm's tip and its
  // grasp point were different places.
  const actual = await getCartesian(sim, scriptHandle);
  const finalError = cartesianDistance(actual, point);
  if (finalError > options.cartesianFinalTolerance) {
    return {
      success: false,
      reason: "cartesian_target_not_reached",
      target: point,
      actual,
      finalError,
      tolerance: options.cartesianFinalTolerance,
      planner: "coppeliasim_smooth_pose_ik"
    };
  }
  // The exit is a short backoff, not a return across the cell to the previous
  // handover. Its own sweep (and planner fallback) validates that new leg.
  const distance = cartesianDistance(departure, point);
  const fraction = distance > 0 ? Math.min(1, options.handoverStandoffDistance / distance) : 0;
  executedRendezvousLeg.set(scriptHandle, {
    kind: "cartesian_leg",
    from: Object.fromEntries(["x", "y", "z"].map((axis) =>
      [axis, point[axis] + (departure[axis] - point[axis]) * fraction])),
    to: point
  });
  return {
    success: true,
    state: "cartesian_motion_complete",
    planner: execution.planner || "coppeliasim_smooth_pose_ik",
    actual,
    finalError
  };
}

async function simpleSmoothJointMove(sim, scriptHandle, td, targetRadians, options) {
  // A joint command is how an arm leaves a rendezvous for good, and it is also
  // the one thing that can move it off the end of a recorded leg without ever
  // touching this file's Cartesian path. Either way the leg no longer leads out
  // of where the arm is.
  executedRendezvousLeg.delete(scriptHandle);
  if (options.executeReservedPath) {
    const start = await getJointRadians(sim, scriptHandle, td);
    const dt = await simulationTimeStep(sim, scriptHandle);
    return options.executeReservedPath(homeTrajectory(start, targetRadians, options, dt).flat(), targetRadians.length, true);
  }
  // Joint commands get the same treatment: one swept preflight before the
  // move instead of a check inside every Ruckig callback. The sweep samples
  // the linear interpolation, while moveToConfig follows per-joint S-curves,
  // so the in-motion watchdog stays on to cover that deviation.
  const start = await getJointRadians(sim, scriptHandle, td);
  const steps = jointStepCount(start, targetRadians, options);
  const pathSafety = await preflightJointPath(sim, scriptHandle, start, targetRadians, options, steps);
  if (!pathSafety.ok) {
    // Ask the planner for a detour, exactly as smoothJointMove does. Straight
    // interpolation being blocked is a fact about the LINE between two
    // configurations, not about the goal, and this path used to report it as
    // though the goal were unreachable -- so an arm that could plan its way
    // into a rendezvous had no way to plan its way back out of one.
    //
    // Live cell: Robot4 finished a handover holding the cube with joint 1
    // nearly a full turn round, and its own gripper swept its link 2 on the
    // straight line home. A route around existed; nothing asked for it, because
    // simple motion is what this robot uses and only the other joint path knew
    // to fall back.
    return planAndFollow(sim, scriptHandle, targetRadians, options, { ...pathSafety, motion: { steps } });
  }

  await executeInScript(sim, scriptHandle, `return womatStartSimpleJoint(${luaArray(targetRadians)})`);
  const done = await waitForMotionComplete(sim, scriptHandle);
  if (!done) return { success: false, reason: "joint_motion_timeout" };
  const executionError = await motionExecutionError(sim, scriptHandle);
  if (executionError) {
    return { success: false, reason: "motion_execution_failed", plannerMessage: executionError };
  }
  const actual = await getJointRadians(sim, scriptHandle, td);
  const maxError = targetRadians.reduce(
    (largest, target, index) => Math.max(largest, Math.abs(target - actual[index])),
    0
  );
  const tolerance = options.pathTrackingToleranceDeg * Math.PI / 180;
  if (maxError > tolerance) {
    return {
      success: false,
      reason: "joint_target_not_reached",
      maxErrorDeg: maxError * 180 / Math.PI,
      toleranceDeg: options.pathTrackingToleranceDeg,
      planner: "coppeliasim_smooth_joint"
    };
  }
  return {
    success: true,
    state: "joint_motion_complete",
    planner: "coppeliasim_smooth_joint",
    maxErrorDeg: maxError * 180 / Math.PI
  };
}

function baseJointFirstConfigs(start, target, options) {
  if (start.length === 0 || start.length !== target.length) return null;
  const minimumDelta = Number(options.baseJointFirstMinDeltaDeg || 0) * Math.PI / 180;
  if (Math.abs(target[0] - start[0]) < minimumDelta) return null;

  const baseCleared = [...start];
  baseCleared[0] = target[0];
  const configs = [];
  const baseSteps = jointStepCount(start, baseCleared, options);
  for (let step = 1; step <= baseSteps; step += 1) {
    configs.push(...interpolate(start, baseCleared, baseSteps, step));
  }
  const remainingSteps = jointStepCount(baseCleared, target, options);
  for (let step = 1; step <= remainingSteps; step += 1) {
    configs.push(...interpolate(baseCleared, target, remainingSteps, step));
  }
  return { configs, baseSteps, remainingSteps };
}

function jointConfigStates(flatConfigs, dof) {
  if (!Number.isInteger(dof) || dof <= 0 || flatConfigs.length % dof !== 0) {
    throw new Error("Flat joint path must contain a whole number of configurations");
  }
  return Array.from(
    { length: flatConfigs.length / dof },
    (_, state) => flatConfigs.slice(state * dof, (state + 1) * dof)
  );
}

async function tryBaseJointFirst(sim, scriptHandle, targetRadians, options, requireJointGoal = true) {
  if (!options.preferBaseJointFirst) return { attempted: false };
  const start = await executeInScript(sim, scriptHandle, "return womatCurrentConfig()");
  const staged = baseJointFirstConfigs(start.map(Number), targetRadians, options);
  if (!staged) return { attempted: false };

  const safety = await executeInScript(
    sim,
    scriptHandle,
    `return womatPreflightJointPath({${jointConfigStates(staged.configs, targetRadians.length).map(luaArray).join(",")}})`
  );
  if (!safety || safety.ok === false) {
    return { attempted: true, success: false, reason: safety?.reason || "base_joint_first_path_unsafe" };
  }

  await executeInScript(sim, scriptHandle, `return womatFollowConfigs(${luaArray(staged.configs)})`);
  const timeoutAttempts = Math.max(900, staged.baseSteps + staged.remainingSteps + options.pathSettleMaxSteps + 100);
  const done = await waitForMotionComplete(sim, scriptHandle, timeoutAttempts);
  if (!done) return { attempted: true, success: false, reason: "base_joint_first_timeout" };
  const executionError = await motionExecutionError(sim, scriptHandle);
  if (executionError) {
    return { attempted: true, success: false, reason: "motion_execution_failed", plannerMessage: executionError };
  }
  const reached = await executeInScript(sim, scriptHandle, "return womatPathDidReachGoal()");
  if (reached !== true && requireJointGoal) {
    return { attempted: true, success: false, reason: "base_joint_first_target_not_reached" };
  }
  return {
    attempted: true,
    success: true,
    state: "planned_motion_complete",
    planner: "collision_checked_base_joint_first",
    jointGoalReached: reached === true,
    states: staged.configs.length / targetRadians.length,
    phases: ["joint1", "remaining_joints"]
  };
}

async function planAndFollow(sim, scriptHandle, targetRadians, options, failure, requireJointGoal = true) {
  const staged = await tryBaseJointFirst(sim, scriptHandle, targetRadians, options, requireJointGoal);
  if (staged.success) return { ...staged, avoided: failure.reason };

  if (!options.enablePathPlanning) {
    return { success: false, ...failure, stagedPlanner: staged.attempted ? staged.reason : undefined };
  }

  const plan = await executeInScript(
    sim,
    scriptHandle,
    `return womatPlanAndFollowJointPath(${luaArray(targetRadians)}, ${options.planningMaxTimeSeconds}, ${options.planningStateCount})`
  );
  if (!plan || plan.ok === false) {
    return {
      success: false,
      ...failure,
      planner: plan?.reason || "planner_failed",
      plannerMessage: plan?.message
    };
  }

  const timeoutAttempts = Math.max(
    900,
    Math.ceil(Number(plan.states || 1) + options.pathSettleMaxSteps + 100)
  );
  const done = await waitForMotionComplete(sim, scriptHandle, timeoutAttempts);
  if (!done) {
    return { success: false, reason: "planned_motion_timeout", planner: "rrtconnect", states: plan.states };
  }
  const executionError = await motionExecutionError(sim, scriptHandle);
  if (executionError) {
    return { success: false, reason: "motion_execution_failed", planner: "rrtconnect", plannerMessage: executionError };
  }
  const reached = await executeInScript(sim, scriptHandle, "return womatPathDidReachGoal()");
  if (reached !== true && requireJointGoal) {
    return { success: false, reason: "planned_joint_target_not_reached", planner: "rrtconnect", states: plan.states };
  }
  return {
    success: true,
    state: "planned_motion_complete",
    planner: "rrtconnect",
    jointGoalReached: reached === true,
    states: plan.states,
    rawStates: plan.rawStates,
    smoothed: plan.smoothed,
    avoided: failure.reason
  };
}

async function tryDirectJointTrajectory(sim, scriptHandle, targetRadians, options) {
  const start = (await executeInScript(sim, scriptHandle, "return womatCurrentConfig()")).map(Number);
  const trajectoryOptions = {
    ...options,
    jointSteps: 1,
    jointMaxVelocityDeg: options.plannedJointMaxVelocityDeg
  };
  const steps = jointStepCount(start, targetRadians, trajectoryOptions);
  const safety = await preflightJointPath(sim, scriptHandle, start, targetRadians, options, steps);
  if (!safety.ok) return { success: false, ...safety };

  // The direct joint sweep is now fully validated. In simple mode, execute
  // that accepted goal through CoppeliaSim/Ruckig so velocity, acceleration
  // and jerk remain smooth instead of replaying discrete samples.
  if (options.simpleSmoothMotion) {
    await executeInScript(sim, scriptHandle, `return womatStartSimpleJoint(${luaArray(targetRadians)})`);
    const done = await waitForMotionComplete(sim, scriptHandle);
    if (!done) return { success: false, reason: "direct_joint_trajectory_timeout" };
    const executionError = await motionExecutionError(sim, scriptHandle);
    if (executionError) {
      return { success: false, reason: "motion_execution_failed", plannerMessage: executionError };
    }
    return {
      success: true,
      state: "planned_motion_complete",
      planner: "collision_checked_coppeliasim_smooth_joint",
      states: steps,
      smoothed: true
    };
  }

  const configs = [];
  for (let step = 1; step <= steps; step += 1) {
    configs.push(...interpolate(start, targetRadians, steps, step));
  }
  await executeInScript(sim, scriptHandle, `return womatFollowConfigs(${luaArray(configs)})`);
  const done = await waitForMotionComplete(
    sim,
    scriptHandle,
    Math.max(900, steps + options.pathSettleMaxSteps + 100)
  );
  if (!done) return { success: false, reason: "direct_joint_trajectory_timeout" };
  const executionError = await motionExecutionError(sim, scriptHandle);
  if (executionError) {
    return { success: false, reason: "motion_execution_failed", plannerMessage: executionError };
  }
  return {
    success: true,
    state: "planned_motion_complete",
    planner: "collision_checked_joint_trajectory",
    states: steps,
    smoothed: true
  };
}

async function smoothJointMove(sim, scriptHandle, td, targetRadians, options) {
  const start = await getJointRadians(sim, scriptHandle, td);
  const steps = jointStepCount(start, targetRadians, options);
  const pathSafety = await preflightJointPath(sim, scriptHandle, start, targetRadians, options, steps);
  if (!pathSafety.ok) {
    // Direct interpolation would collide: ask the planner for a safe detour.
    return planAndFollow(sim, scriptHandle, targetRadians, options, { ...pathSafety, motion: { steps } });
  }

  // Execute exactly the preflighted interpolation, one waypoint per simulation
  // step, so the executed path cannot deviate from the collision-checked one.
  const configs = [];
  for (let step = 1; step <= steps; step += 1) {
    configs.push(...interpolate(start, targetRadians, steps, step));
  }
  await executeInScript(sim, scriptHandle, `return womatFollowConfigs(${luaArray(configs)})`);

  const timeoutAttempts = Math.max(900, steps + options.pathSettleMaxSteps + 100);
  const done = await waitForMotionComplete(sim, scriptHandle, timeoutAttempts);
  if (!done) {
    return { success: false, reason: "joint_motion_timeout", motion: { steps } };
  }
  const executionError = await motionExecutionError(sim, scriptHandle);
  if (executionError) {
    return { success: false, reason: "motion_execution_failed", plannerMessage: executionError, motion: { steps } };
  }
  return { success: true, state: "joint_motion_complete", motion: { steps } };
}

async function cartesianPlannerFallback(sim, scriptHandle, point, options, failure) {
  if (!options.enablePathPlanning) {
    return { success: false, ...failure };
  }

  const goal = await executeInScript(
    sim,
    scriptHandle,
    `return womatFindGoalConfig(${point.x}, ${point.y}, ${point.z}, ${options.planningMaxTimeSeconds})`
  );
  if (!goal || !goal.config) {
    return {
      success: false,
      reason: "cartesian_goal_config_unavailable",
      trigger: failure.reason,
      safety: failure.safety,
      planner: goal?.reason || "goal_config_unavailable"
    };
  }

  // A Cartesian action is complete when its TCP reaches the requested point.
  // Position-only IK is redundant, so requiring one exact joint solution can
  // reject a physically correct TCP pose. Explicit joint actions continue to
  // call planAndFollow with the strict default.
  // Prefer the same clean, uniformly timed interpolation used by explicit
  // joint actions. Invoke OMPL only if that swept joint path is obstructed.
  let planned;
  if (options.simpleSmoothMotion) {
    // Validate densely and execute those exact samples. A coarse sweep followed
    // by independent per-joint Ruckig curves could still hit a thin wall.
    const target = goal.config.map(Number);
    const route = await prepareJointRoute(sim, scriptHandle, target, options);
    if (!route.success) return route;
    planned = options.executeReservedPath
      ? await options.executeReservedPath(route.path, target.length)
      : await followHandoverExit(sim, scriptHandle, route.path, target, options, false, true);
    if (!planned.success) return planned;
    planned = { ...planned, planner: "collision_checked_joint_route", states: route.path.length / target.length };
  } else {
    planned = options.preferOmplForCartesian
      ? { success: false, reason: "ompl_first_preferred" }
      : await tryDirectJointTrajectory(sim, scriptHandle, goal.config.map(Number), options);
  }
  if (!planned.success) {
    planned = await planAndFollow(
      sim,
      scriptHandle,
      goal.config.map(Number),
      options,
      { ...failure, directTrajectory: planned.reason },
      false
    );
  }
  if (!planned.success) return planned;
  const actual = await getCartesian(sim, scriptHandle);
  const finalError = cartesianDistance(actual, point);
  if (finalError > options.cartesianFinalTolerance) {
    return {
      success: false,
      reason: "planned_cartesian_target_not_reached",
      target: point,
      actual,
      finalError,
      tolerance: options.cartesianFinalTolerance,
      planner: planned.planner
    };
  }
  return { ...planned, actual, finalError };
}

async function smoothCartesianMove(sim, scriptHandle, point, workspace, options) {
  await refreshObstacleBoxes(sim, scriptHandle, workspace);
  const safetyResult = checkWorkspace(point, workspace, options);
  if (!safetyResult.ok) {
    return { success: false, ...safetyResult };
  }

  // Resolve Cartesian targets into complete, collision-checked joint
  // trajectories before any visible movement. Robot3 can retain OMPL-first;
  // the other robots prefer a shorter direct joint trajectory and use OMPL
  // only when that swept path is blocked.
  if (options.preferJointTrajectoryForCartesian || options.preferOmplForCartesian) {
    return cartesianPlannerFallback(sim, scriptHandle, point, options, {
      reason: "ompl_first_preferred",
      safety: safetyResult
    });
  }

  // Robots mounted close to a wall can use a deliberate two-phase joint path:
  // rotate joint 1 out of the constrained region, then move the remaining
  // joints. Every sampled configuration is collision checked; if this route is
  // unsafe, planAndFollow automatically falls back to OMPL.
  if (options.preferBaseJointFirst) {
    return cartesianPlannerFallback(sim, scriptHandle, point, options, {
      reason: "base_joint_first_preferred",
      safety: safetyResult
    });
  }

  const start = await getCartesian(sim, scriptHandle);
  const steps = cartesianStepCount(start, point, options);
  const pathSafety = await preflightCartesianPath(sim, scriptHandle, start, point, options, steps);
  if (!pathSafety.ok) {
    // The straight tip line is blocked: plan a joint-space detour to the IK
    // goal configuration instead of failing outright.
    return cartesianPlannerFallback(sim, scriptHandle, point, options, { ...pathSafety, motion: { steps } });
  }

  const bodyPathSafety = await preflightCartesianBodyPath(sim, scriptHandle, start, point, options, steps);
  if (!bodyPathSafety.ok) {
    // Plan before anything moves. Previously this collision was discovered
    // reactively below, which made the arm move forward, roll back, and only
    // then follow the safe detour.
    return cartesianPlannerFallback(
      sim,
      scriptHandle,
      point,
      options,
      { ...bodyPathSafety, motion: { steps } }
    );
  }

  let previous = [start.x, start.y, start.z];
  for (let step = 1; step <= steps; step += 1) {
    const alpha = step / steps;
    const next = [
      start.x + (point.x - start.x) * alpha,
      start.y + (point.y - start.y) * alpha,
      start.z + (point.z - start.z) * alpha,
      0,
      0.707,
      0,
      0.707
    ];
    if (await checkPointCollisionIfEnabled(sim, scriptHandle, options, { x: next[0], y: next[1], z: next[2] })) {
      await executeInScript(sim, scriptHandle, `return moveToPosition(${luaArray([...previous, 0, 0.707, 0, 0.707])})`);
      return cartesianPlannerFallback(sim, scriptHandle, point, options, {
        reason: "collision_detected",
        obstacleMode: "tip_vs_obstacle_aabb",
        motion: { steps }
      });
    }
    await executeInScript(sim, scriptHandle, `return moveToPosition(${luaArray(next)})`);
    const executionError = await motionExecutionError(sim, scriptHandle);
    if (executionError) {
      return {
        success: false,
        reason: "motion_execution_failed",
        plannerMessage: executionError,
        motion: { steps, failedAtStep: step }
      };
    }
    if (await checkBodyCollisionIfEnabled(sim, scriptHandle, options)) {
      // A link (not the tip) touched an obstacle: step back to the last safe
      // pose and let the planner find a detour.
      await executeInScript(sim, scriptHandle, `return moveToPosition(${luaArray([...previous, 0, 0.707, 0, 0.707])})`);
      return cartesianPlannerFallback(sim, scriptHandle, point, options, {
        reason: "collision_detected",
        obstacleMode: "robot_vs_obstacle_collection",
        motion: { steps }
      });
    }
    previous = [next[0], next[1], next[2]];
    await delay(options.stepDelayMs);
  }

  const actual = await getCartesian(sim, scriptHandle);
  const finalError = cartesianDistance(actual, point);
  if (finalError > options.cartesianFinalTolerance) {
    return cartesianPlannerFallback(sim, scriptHandle, point, options, {
      reason: "cartesian_target_not_reached",
      target: point,
      actual,
      finalError,
      tolerance: options.cartesianFinalTolerance,
      motion: { steps }
    });
  }

  return {
    success: true,
    state: "cartesian_motion_complete",
    safety: safetyResult,
    actual,
    finalError,
    motion: { steps }
  };
}

module.exports = {
  baseJointFirstConfigs,
  cartesianPlannerFallback,
  handoverApproach,
  handoverRetreat,
  jointConfigStates,
  planAndFollow,
  simpleSmoothCartesianMove,
  simpleSmoothJointMove,
  smoothCartesianMove,
  smoothJointMove,
  topDownPick,
  tryBaseJointFirst,
  tryDirectJointTrajectory
};
