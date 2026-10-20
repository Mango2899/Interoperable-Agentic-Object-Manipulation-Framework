"use strict";

const { asArray } = require("../workspaceGrid");
const { luaArray } = require("./luaDriver");
const { executeInScript, sceneRootHandle, simulationTimeStep } = require("./simConnection");
const { conservativeClaim, occupancyFor } = require("./claims");
const { delay } = require("./context");
const { zoneKeys } = require("./handoverZone");
const { motionSchedule } = require("./timeLaw");

// A single time-parameterised straight joint path, timed by the shared
// jerk-limited law: zero velocity and acceleration at both ends, the configured
// velocity, acceleration and jerk bounds respected, and a cruise in between.
// Geometry is sampled separately for the swept-room check.
function homeTrajectory(start, target, options, dt) {
  const distance = Math.max(...target.map((value, i) => Math.abs(value - start[i])));
  const rad = Math.PI / 180;
  return motionSchedule(distance, {
    velocity: options.simpleJointVelocityDeg * rad,
    acceleration: options.simpleJointAccelerationDeg * rad,
    jerk: options.simpleJointJerkDeg * rad
  }, dt).map((fraction) => start.map((value, j) => value + (target[j] - value) * fraction));
}

function validFrames(frames, count) {
  return frames.length === count && frames.every((frame) => frame.length >= 6
    && frame.length % 3 === 0 && frame.every(Number.isFinite));
}

// Dynamic arms can sag beyond the endpoint tolerance at intermediate poses.
// Only the final home configuration is a goal; room release does not stop it.
async function continuousHome(sim, handle, robot, target, options, arbiter, readRegions, motionLog) {
  const preparationStartedAt = Date.now();
  if (!target.length) return { success: false, reason: "invalid_home_configuration" };
  if (![options.collisionCheckStepDeg, options.preflightBudgetMs].every((value) => Number.isFinite(value) && value > 0)) {
    return { success: false, reason: "invalid_preflight_options" };
  }
  let path = options.reservedPath;
  // A reserved path already begins at the posture the arm is standing in --
  // whoever reserved it read that and put it there -- so asking the simulator
  // again is a websocket round trip for an answer already in hand. On the
  // giver's exit that round trip is time the receiver spends waiting.
  if (!path) {
    const start = asArray(await executeInScript(sim, handle, "return womatCurrentConfig()")).map(Number);
    if (start.length !== target.length || !start.every(Number.isFinite)) {
      return { success: false, reason: "invalid_home_configuration" };
    }
    path = homeTrajectory(start, target, options, await simulationTimeStep(sim, handle));
  }
  if (!path.length || path.some((row) => row.length !== target.length || !row.every(Number.isFinite))) {
    return { success: false, reason: "invalid_reserved_path" };
  }
  const action = options.reservedPath ? "handoverZoneMotion" : "giverHome";
  const regions = readRegions();
  const pathClaim = options.pathClaim || ((frames) => conservativeClaim(robot, regions, frames, { bodyRadius: options.claimBodyRadius }));
  const lastTouch = new Map();
  // Following another arm needs an arbiter that can queue a claim behind the
  // live ones; without one, the path is reserved whole as before.
  const following = options.orderedReservations !== false && arbiter.supportsOrdered === true;
  // The first sample whose BODY reaches each voxel: how far an arm following
  // another may play before it needs a voxel the other still holds.
  //
  // The body, not the reservation. A sample's reservation is padded, dilated to
  // whole grid cells and -- while the tool is anywhere near a rendezvous --
  // arms the entire handover pocket, because it is what OTHER arms must stay
  // out of. Gating on it made the receiver wait for pocket voxels it was
  // nowhere near: Robot5, leaving the Robot4 rendezvous, claimed the whole
  // pocket at its first sample, the edge of that pocket sits inside the padded
  // claim of Robot4's own home posture (a CR3 parks 0.36 m from it), and
  // Robot5 stood still for the entire 19 s of Robot4's return. What this arm
  // must not do is put its own links into space the arm ahead holds -- and
  // that holding already includes the pocket whenever the arm ahead has a
  // tool in it.
  const gateTouch = new Map();
  const samples = [];
  // Associate each dense geometry sample with the end of its executed segment.
  // Release cannot happen until that command has been passed AND the measured
  // whole-arm footprint is outside the room (dynamic lag is not clearance).
  for (let i = 0; i < path.length; i++) {
    const previous = path[Math.max(0, i - 1)];
    const delta = Math.max(...path[i].map((value, j) => Math.abs(value - previous[j])));
    // The density of the check, and the single biggest lever on what it costs:
    // every sample is a pair of mesh queries. Same knob the planners use.
    const count = Math.max(1, Math.ceil(delta / (options.collisionCheckStepDeg * Math.PI / 180)));
    for (let sample = 1; sample <= count; sample++) {
      samples.push({ index: i + 1, config: previous.map((value, j) => value + (path[i][j] - value) * sample / count) });
    }
  }
  // Forward kinematics is cheap -- about 0.6 ms a sample in the live cell -- so
  // the geometry sweep is sized in samples and takes big bites. The collision
  // preflight is not, and is sized in time instead; see womatBeginPreflight.
  const geometryChunk = Math.max(1, Number(options.homeGeometryChunk) || 128);
  let geometryCalls = 0;
  for (let offset = 0; offset < samples.length; offset += geometryChunk) {
    const chunk = samples.slice(offset, offset + geometryChunk);
    geometryCalls += 1;
    const swept = await executeInScript(sim, handle,
      `return womatSweepFramePositions(${luaArray(chunk.flatMap((s) => s.config))})`);
    const frames = asArray(swept?.frames).map((frame) => asArray(frame).map(Number));
    if (!swept?.ok || !validFrames(frames, chunk.length)) {
      return { success: false, reason: "home_path_frames_unavailable", safeToReplan: true };
    }
    frames.forEach((frame, i) => {
      const claim = pathClaim([frame]);
      for (const key of claim.voxels) lastTouch.set(key, chunk[i].index);
      if (!following) return;
      for (const key of occupancyFor(robot, regions, [frame], { bodyRadius: options.claimBodyRadius }).voxels) {
        if (!gateTouch.has(key)) gateTouch.set(key, chunk[i].index);
      }
    });
  }
  const geometryMs = Date.now() - preparationStartedAt;
  let held = [...lastTouch.keys()];
  const originallyReserved = new Set(held);
  const zoneLocks = (options.handoverZones || []).flatMap((zone) => {
    const region = regions.find((region) => region.pairKey === zone.pairKey);
    if (!region) return [];
    const keys = zoneKeys(region, zone);
    return keys.some((key) => lastTouch.has(key)) ? [{ zone, keys, released: false }] : [];
  });
  // The swept collision preflight, and what one costs.
  //
  // Sized in time on the simulator side (see womatBeginPreflight), so the
  // number of calls follows from what the meshes actually cost rather than
  // being guessed. The round trip is timed here too, because it is not free
  // either: at a 50 ms slice budget the websocket overhead measured about a
  // quarter of the whole sweep.
  let preflightCalls = 0;
  let preflightRpcMs = 0, preflightMaxCallMs = 0, preflightCpuMs = 0;
  let cpuTimingAvailable = true;
  const preflightCall = async (code) => {
    const began = Date.now();
    preflightCalls += 1;
    const verdict = await executeInScript(sim, handle, code);
    const elapsed = Date.now() - began;
    preflightRpcMs += elapsed;
    preflightMaxCallMs = Math.max(preflightMaxCallMs, elapsed);
    if (Number.isFinite(verdict?.sliceCpuMs)) preflightCpuMs += verdict.sliceCpuMs;
    else cpuTimingAvailable = false;
    return verdict;
  };
  // Handed over once, then walked in time-bounded slices. Sizing the slices
  // in samples sized how long the whole simulator stopped instead, because a
  // mesh query costs forty times what a sample of forward kinematics does.
  const sweepReservedPath = async ({ from = 0, exemptRoots = [] } = {}) => {
    const exempt = exemptRoots.length ? `, ${luaArray(exemptRoots)}` : "";
    let verdict = await preflightCall(
      `return womatBeginPreflight(${luaArray(samples.slice(from).map((s) => luaArray(s.config)))}, true, ${from}, `
      + `${options.preflightBudgetMs / 1000}${exempt})`);
    while (verdict?.ok && verdict.pending) verdict = await preflightCall("return womatContinuePreflight()");
    return verdict;
  };

  const log = motionLog.begin(action);
  const root = following ? await sceneRootHandle(sim, handle) : null;
  let checked = null;
  let preflightMs = 0;
  let rejectedBeforeClaim = false;
  if (!following) {

    // Preflight BEFORE the rooms are claimed, not after.
    //
    // A dense sweep is seconds of stopped simulator, and holding a reservation
    // across it charges every one of those seconds to whichever arm needs the
    // same space next. Measured in the five-robot cell: 9.6 s of preflight
    // inside the giver's claim, and 11.7 s of claim wait for the receiver of the
    // very same rendezvous. Nothing in the sweep needs the reservation -- it
    // asks where THIS arm's links go through postures already computed -- so
    // that wait bought nothing.
    //
    // What the reservation does buy is a scene the peers have left, so a
    // rejection found out here is not a verdict on the path: a peer standing in
    // the space this path is about to reserve is gone once the claim is granted,
    // and the sweep is simply run again under it. Only a rejection the scene
    // cannot lift on its own -- a posture colliding with ITSELF, or a sweep that
    // failed outright -- is reported without acquiring anything.
    console.log(`[LowerThing:${robot}] ${action} checking reserved path before claiming its rooms`, {
      checkedSamples: samples.length, budgetMs: options.preflightBudgetMs
    });
    const preflightStartedAt = Date.now();
    checked = await sweepReservedPath();
    preflightMs = Date.now() - preflightStartedAt;
    rejectedBeforeClaim = checked?.ok !== true;
    if (rejectedBeforeClaim && (checked?.mode === "self_collision" || checked?.reason === "path_sweep_failed")) {
      const refusal = { ...checked, success: false, reason: checked?.reason || "home_path_blocked", safeToReplan: true };
      log.finished(refusal);
      return refusal;
    }
  }

  const deadline = Date.now() + 80000;
  const claimStartedAt = Date.now();
  let grant;
  let previousBlocker = null;
  do {
    grant = following ? await arbiter.acquire(robot, held, { ordered: true, root })
      : await arbiter.acquire(robot, held);
    if (grant.granted) break;
    if (!["region_claimed", "motion_busy"].includes(grant.reason || "motion_busy") || Date.now() >= deadline) {
      log.refused(grant);
      const result = { success: false, reason: grant.reason || "motion_busy",
        activeRobot: grant.activeRobot || "unknown", conflictVoxel: grant.conflictVoxel || null,
        conflictRegion: grant.conflictVoxel?.split(":")[0] || null };
      log.finished(result);
      return result;
    }
    const blocker = { activeRobot: grant.activeRobot || "unknown", reason: grant.reason || "motion_busy",
      conflictRegion: grant.conflictVoxel?.split(":")[0] || null };
    const signature = JSON.stringify(blocker);
    if (signature !== previousBlocker) {
      console.log(`[LowerThing:${robot}] Home waiting for motion permission`, blocker);
      motionLog.record?.("home_waiting_for_room", blocker);
      previousBlocker = signature;
    }
    await delay(100);
  } while (true);
  const claimWaitMs = Date.now() - claimStartedAt;
  log.granted({ contestedRegions: [...new Set(held.map((key) => key.split(":")[0]))],
    claimedVoxels: held.length, granularity: options.pathClaim ? "handover_zone_and_path" : "region",
    concurrentArms: grant.concurrentArms || 1, ordered: grant.ordered === true });

  // Who this path is queued behind, and what of it they still hold. Only ever
  // shrinks: the claims are earlier than this one, and earlier claims never
  // grow. An exclusive host grants the ordered request as a plain claim, and
  // then there is nobody to follow.
  const queued = grant.ordered === true;
  const blockers = new Map();
  const readBlockers = (reply) => {
    const holders = reply?.holders || {};
    const blockedBy = reply?.blockedBy || {};
    for (const [robotName, keys] of Object.entries(blockedBy)) {
      const known = blockers.get(robotName);
      const sequence = holders[robotName]?.sequence ?? null;
      if (!known) {
        blockers.set(robotName, { keys: new Set(keys), sequence, root: holders[robotName]?.root ?? null });
      } else if (known.sequence === sequence) {
        known.keys = new Set(keys);
      }
    }
    // A blocker gone from the table has finished, and one holding under a new
    // sequence finished and started again. Either way it is PARKED now, at a
    // pose it held under a claim this path was exempted from checking against,
    // so what it held of this path is checked again before any of it is used.
    const finished = [];
    for (const [robotName, known] of blockers) {
      const now = holders[robotName];
      const stillHolding = now && (now.sequence ?? null) === known.sequence;
      if (!stillHolding) finished.push([robotName, known]);
      else if (!blockedBy[robotName]) known.keys = new Set();
    }
    for (const [robotName, known] of finished) {
      blockers.delete(robotName);
      if (known.keys.size) parked.set(robotName, known.keys);
    }
  };
  // Released by an arm that then stopped somewhere this path goes: held back
  // from playback until the rest of the path has been checked against it.
  const parked = new Map();
  if (queued) readBlockers(grant);
  const exemptRoots = () => [...blockers.values()].map((known) => known.root).filter(Number.isInteger);
  const lookahead = Math.max(0, Math.round(Number(options.followLookaheadSteps ?? 2)));
  // The last path state playback may send. Stops short of the first sample
  // that needs a voxel still held by an arm ahead of this one -- or released
  // by one that has not been checked against since it parked.
  const playbackLimit = () => {
    let first = Infinity;
    const consider = (keys) => {
      for (const key of keys) first = Math.min(first, gateTouch.get(key) ?? Infinity);
    };
    for (const known of blockers.values()) consider(known.keys);
    for (const keys of parked.values()) consider(keys);
    return Number.isFinite(first) ? Math.max(0, first - 1 - lookahead) : path.length;
  };
  const initiallyFollowing = [...blockers.keys()];

  let running = false;
  let playbackStartedAt = null;
  let recheckMs = 0;
  let result;
  try {
    if (queued) {
      // Swept under the claim, because only now is it known whom this path is
      // following. Those arms are exempt -- they will have left every voxel
      // before playback enters it -- and everything else is tested, idle arms
      // included, exactly as a path reserved whole would be.
      const preflightStartedAt = Date.now();
      console.log(`[LowerThing:${robot}] ${action} checking reserved path`, {
        checkedSamples: samples.length, budgetMs: options.preflightBudgetMs,
        following: initiallyFollowing });
      checked = await sweepReservedPath({ exemptRoots: exemptRoots() });
      preflightMs = Date.now() - preflightStartedAt;
      if (checked?.ok !== true) {
        result = { ...checked, success: false, reason: checked?.reason || "home_path_blocked", safeToReplan: true };
        return result;
      }
    } else {
      // Re-tested under the claim, because only now is the space this path needs
      // one no peer may be standing in. A path the peers WERE standing in gets
      // the whole sweep again; an already-clear one gets only the pose the arm
      // comes to REST in -- the single sample the sweep measures clearance at
      // rather than contact alone, and the one pose a reservation cannot vouch
      // for, since the arm stops there and stays.
      const recheckStartedAt = Date.now();
      checked = rejectedBeforeClaim
        ? await sweepReservedPath()
        : await executeInScript(sim, handle,
          `return womatPreflightJointPath(${luaArray([luaArray(path[path.length - 1])])}, true, ${samples.length - 1})`);
      recheckMs = Date.now() - recheckStartedAt;
      if (checked?.ok !== true) {
        result = { ...checked, success: false, reason: checked?.reason || "home_path_blocked", safeToReplan: true };
        return result;
      }
    }
    let limit = playbackLimit();
    const limitArg = (value) => (value >= path.length ? -1 : value);
    const started = await executeInScript(sim, handle,
      `return womatFollowConfigs(${luaArray(path.flat())}, true${queued ? `, ${limitArg(limit)}` : ""})`);
    if (started?.ok === false) return (result = { success: false, reason: started.reason || "home_path_start_failed" });
    running = true;
    playbackStartedAt = Date.now();
    const timing = { action, states: path.length, checkedSamples: samples.length,
      geometryCalls, preflightCalls, geometryMs,
      claimWaitMs, preflightRpcMs, preflightMaxCallMs,
      preflightCpuMs: cpuTimingAvailable ? preflightCpuMs : null,
      preflightMs, recheckMs, resweptUnderClaim: rejectedBeforeClaim,
      ...(queued ? { following: initiallyFollowing, heldAtState: limit < path.length ? limit : null } : {}) };
    console.log(`[LowerThing:${robot}] ${action} trajectory started`, timing);
    motionLog.record?.("trajectory_started", timing);
    let heldSince = null;
    let reportedHold = null;
    const followWaitMs = Math.max(1000, Number(options.followWaitMs ?? 60000));
    const maxPolls = Math.max(900, path.length + options.pathSettleMaxSteps + 100);
    for (let poll = 0; poll < maxPolls; poll++) {
      await delay(50);
      const status = await executeInScript(sim, handle,
        queued ? `return womatHomePathStatus(${limitArg(limit)})` : "return womatHomePathStatus()");
      if (status?.error) return (result = { success: false, reason: "motion_execution_failed", plannerMessage: status.error });
      if (Number(status?.state) === 3) {
        running = false;
        const actual = asArray(status.config).map(Number);
        const errors = actual.map((value, i) => Math.abs(value - target[i]) * 180 / Math.PI);
        const reached = actual.length === target.length && actual.every(Number.isFinite)
          && (options.requireJointTarget === false || errors.every((error) => error <= options.pathTrackingToleranceDeg));
        result = reached ? { success: true, planner: options.reservedPath ? "handover_zone_path" : "continuous_joint_home", states: path.length }
          : { success: false, reason: "home_target_not_reached", actual, target,
            maxErrorDeg: Math.max(...errors), toleranceDeg: options.pathTrackingToleranceDeg };
        return result;
      }
      const frames = [asArray(status?.frames).map(Number)];
      if (!validFrames(frames, 1) || !Number.isInteger(status?.index) || status.index < 0 || status.index > path.length) {
        return (result = { success: false, reason: "home_path_status_invalid" });
      }
      const occupied = new Set((options.occupancyClaim || pathClaim)(frames).voxels);
      const heldSet = new Set(held);
      const outside = options.occupancyClaim ? [...occupied].filter((key) => !heldSet.has(key)) : [];
      if (outside.length) {
        const details = { pathIndex: status.index, outsideVoxelCount: outside.length,
          outsideVoxels: outside.slice(0, 8),
          previouslyReleasedCount: outside.filter((key) => originallyReserved.has(key)).length };
        console.warn(`[LowerThing:${robot}] ${action} left its reserved path`, details);
        motionLog.record?.("reservation_escape", details);
        return (result = { success: false, reason: "handover_path_left_reservation", ...details });
      }
      // Keep the planning margin around the measured arm until it has cleared,
      // too. Releasing down to the unpadded footprint makes a small controller
      // correction enter a voxel we just gave away. Intersect with held keys:
      // this never expands the reservation into another arm's claim.
      const releaseGuard = options.occupancyClaim ? new Set(pathClaim(frames).voxels) : occupied;
      const remaining = held.filter((key) => lastTouch.get(key) >= status.index || releaseGuard.has(key));
      // Also renew the lease while moving. Shrinking never acquires new space.
      const updated = await arbiter.narrow(robot, remaining);
      if (!updated.updated) return (result = { success: false, reason: updated.reason || "home_claim_update_failed" });
      if (queued) {
        readBlockers(updated);
        for (const [robotName, keys] of [...parked]) {
          // Checked from the first sample that needs any of it. Playback cannot
          // be there yet -- those samples were behind the hold -- and it keeps
          // playing up to the hold while this runs.
          // A parked arm whose last holding this body never reaches is no
          // obstacle to it, however close the two reservations came.
          const firstNeeded = Math.min(...[...keys].map((key) => gateTouch.get(key) ?? Infinity));
          const from = samples.findIndex((sample) => sample.index >= firstNeeded);
          if (from < 0) { parked.delete(robotName); continue; }
          const recheckStartedAt = Date.now();
          const verdict = await sweepReservedPath({ from, exemptRoots: exemptRoots() });
          recheckMs += Date.now() - recheckStartedAt;
          if (verdict?.ok !== true) {
            const details = { parkedRobot: robotName, pathIndex: status.index,
              blockedAtSample: from + Number(verdict?.blockedAtStep || 0),
              obstacle: verdict?.obstacle || null, obstacleOwner: verdict?.obstacleOwner || null };
            console.warn(`[LowerThing:${robot}] ${action} stopped: ${robotName} parked in the rest of this path`, details);
            motionLog.record?.("followed_arm_parked_in_path", details);
            return (result = { ...verdict, success: false, reason: "followed_arm_parked_in_path", ...details,
              safeToReplan: status.index === 0 });
          }
          parked.delete(robotName);
        }
        const nextLimit = playbackLimit();
        const holding = nextLimit < path.length && status.index >= nextLimit;
        if (holding) {
          heldSince ??= Date.now();
          const signature = JSON.stringify([...blockers.keys()]);
          if (signature !== reportedHold) {
            const pocket = new Set(zoneLocks.flatMap((lock) => lock.keys));
            const needed = [...blockers.values()].flatMap((known) => [...known.keys])
              .filter((key) => gateTouch.has(key));
            const hold = { behind: [...blockers.keys()], limit: nextLimit, states: path.length,
              blockingVoxels: needed.length, blockingPocketVoxels: needed.filter((key) => pocket.has(key)).length,
              firstBlockedState: needed.length ? Math.min(...needed.map((key) => gateTouch.get(key))) : null };
            console.log(`[LowerThing:${robot}] ${action} holding at state ${nextLimit} of ${path.length} `
              + `behind ${hold.behind.join(", ") || "a parked arm"}`, hold);
            motionLog.record?.("following_hold", { action, pathIndex: status.index, ...hold });
            reportedHold = signature;
          }
          if (Date.now() - heldSince > followWaitMs) {
            return (result = { success: false, reason: "reservation_follow_timeout",
              behind: [...blockers.keys()], pathIndex: status.index, safeToReplan: status.index === 0 });
          }
          // Waiting behind another arm is not progress along this path.
          poll -= 1;
        } else if (heldSince !== null) {
          console.log(`[LowerThing:${robot}] ${action} moving on after ${Date.now() - heldSince} ms behind the arm ahead`);
          heldSince = null;
          reportedHold = null;
        }
        limit = nextLimit;
      }
      if (remaining.length < held.length) {
        const stillHeld = new Set(remaining);
        for (const lock of zoneLocks) {
          if (!lock.released && lock.keys.every((key) => !stillHeld.has(key))) {
            lock.released = true;
            console.log(`[LowerThing:${robot}] Handover zone cleared: ${lock.zone.pairKey}; continuing on reserved path`);
            motionLog.record?.("handover_zone_released", { action, ...lock.zone, pathIndex: status.index });
          }
        }
        const released = held.filter((key) => !stillHeld.has(key));
        const heldRegions = new Set(remaining.map((key) => key.split(":")[0]));
        const clearedRegions = [...new Set(released.map((key) => key.split(":")[0]))].filter((key) => !heldRegions.has(key));
        if (clearedRegions.length) console.log(`[LowerThing:${robot}] ${action} cleared rooms: ${clearedRegions.join(", ")}; continuing motion`);
        motionLog.record?.("home_rooms_released", { action, regions: clearedRegions, releasedVoxels: released.length, pathIndex: status.index });
      }
      held = remaining;
    }
    return (result = { success: false, reason: "home_path_timeout" });
  } finally {
    // Do not free a moving arm's claims on timeout, missing status or an
    // unreachable arbiter until the simulator has acknowledged the stop.
    if (running) {
      const stopped = await executeInScript(sim, handle, 'return womatStopPath("home_motion_stopped")');
      if (stopped?.ok !== true) throw new Error("Home stop was not acknowledged; retaining motion claim");
    }
    if (playbackStartedAt !== null) motionLog.record?.("trajectory_finished", {
      action, durationMs: Date.now() - playbackStartedAt,
      success: result?.success === true, reason: result?.reason || null
    });
    log.finished(result || { success: false, reason: "home_motion_exception" });
    await arbiter.release(robot);
  }
}

module.exports = { continuousHome, homeTrajectory };
