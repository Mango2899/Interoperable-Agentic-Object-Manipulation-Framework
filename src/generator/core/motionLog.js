"use strict";

// What each arm was doing, and when.
//
// Phase 4 replaces "one arm at a time" with "one arm per contested region", and
// the only honest way to say whether that helped is to have measured the cell
// before it. This records one line per motion per robot, with monotonic
// timestamps, so a run in either mode produces the same shape of evidence.
//
// Written as JSON lines in append mode, one write per record. Several robot
// cores in separate processes share one file: a single write() of a short line
// to an O_APPEND descriptor is not interleaved on Linux, which is why records
// are flushed whole rather than buffered and joined.

const fs = require("fs");
const path = require("path");

// A wall-clock anchor plus a monotonic clock. Wall clock alone drifts and jumps;
// the monotonic clock alone cannot be compared across processes. Recording both
// lets records from three cores be placed on one timeline.
const EPOCH_WALL_MS = Date.now();
const EPOCH_MONOTONIC_MS = Number(process.hrtime.bigint()) / 1e6;

function nowMs() {
  return EPOCH_WALL_MS + (Number(process.hrtime.bigint()) / 1e6 - EPOCH_MONOTONIC_MS);
}

class MotionLog {
  constructor({ robot, file = null, enabled = true, run = null } = {}) {
    this.robot = robot;
    this.enabled = Boolean(enabled && file);
    this.file = file;
    this.run = run || "unnamed";
    if (this.enabled) fs.mkdirSync(path.dirname(file), { recursive: true });
  }

  record(event, fields = {}) {
    if (!this.enabled) return null;
    const line = JSON.stringify({ t: nowMs(), run: this.run, robot: this.robot, event, ...fields });
    try {
      fs.appendFileSync(this.file, `${line}\n`);
    } catch (error) {
      // Instrumentation must never be the reason a robot stops moving.
      this.enabled = false;
      console.warn(`[MotionLog:${this.robot}] disabled after write failure: ${error?.message || error}`);
    }
    return null;
  }

  // One motion, from the moment it was asked for. `requested` and `started` are
  // deliberately separate: the gap between them is arbitration wait, which is
  // the cost the space lock is meant to remove.
  begin(action, fields = {}) {
    const requestedAt = nowMs();
    this.record("motion_requested", { action, ...fields });
    let startedAt = null;
    return {
      granted: (grantFields = {}) => {
        startedAt = nowMs();
        this.record("motion_started", {
          action,
          waitMs: startedAt - requestedAt,
          ...grantFields
        });
      },
      refused: (refuseFields = {}) => {
        this.record("motion_refused", {
          action,
          waitMs: nowMs() - requestedAt,
          ...refuseFields
        });
      },
      finished: (result = {}) => {
        const endedAt = nowMs();
        this.record("motion_finished", {
          action,
          startedAt,
          durationMs: startedAt === null ? 0 : endedAt - startedAt,
          totalMs: endedAt - requestedAt,
          success: result.success !== false,
          reason: result.reason || null,
          // Which route the motion actually took. Without these two the OMPL
          // fallback rate is invisible: a placement that interpolated straight
          // there and one that needed a four-second search look identical in
          // this log, and "the terminal arm almost always falls back to the
          // planner" is an impression rather than a number.
          planner: result.planner || null,
          approachRoute: result.approachRoute || null
        });
      }
    };
  }
}

function motionLogFor(ctx, options = {}) {
  const configured = process.env.WOMAT_MOTION_LOG || options.file;
  if (!configured) return new MotionLog({ robot: ctx.manifest.name, enabled: false });
  const file = path.isAbsolute(configured) ? configured : path.resolve(ctx.repoRoot, configured);
  return new MotionLog({
    robot: ctx.manifest.name,
    file,
    run: process.env.WOMAT_RUN_LABEL || options.run || "run",
    enabled: true
  });
}

module.exports = { MotionLog, motionLogFor, nowMs };
