"use strict";

// Who is allowed to move, right now.
//
// The shared cell has always let exactly one arm move at a time, for a reason
// worth restating: every planner has the other robots in its obstacle
// collection, so a path accepted by preflight stops being valid the moment
// another arm moves. That was enforced by a plain object shared between every
// robot's handlers in one process. Once each robot's core runs as its own
// process there is no such object, so the rule needs somewhere to live.
//
// It lives here, behind one interface with two implementations. `LocalArbiter`
// is the object, unchanged in behaviour. `RemoteArbiter` asks the cell host
// over HTTP. Both answer the same question and both are exclusive.
//
// This is the seam Phase 4 replaces: the spatial claim scheme keeps the same
// acquire/release shape and swaps "one arm at a time" for "one arm per
// contested region", which is what lets two robots move at once.

const http = require("http");

// Exclusive: one arm at a time, whatever it intends to do. This is the Phase 2
// behaviour, kept as the fallback for a cell with no shared-space geometry and
// as the mode a run can be forced into for a serialised baseline measurement.
class LocalArbiter {
  constructor(state = { activeRobot: null }) {
    this.state = state;
  }

  get activeRobot() {
    return this.state.activeRobot;
  }

  async acquire(robot) {
    if (this.state.activeRobot !== null && this.state.activeRobot !== robot) {
      return { granted: false, activeRobot: this.state.activeRobot };
    }
    if (this.state.activeRobot === robot) {
      return { granted: false, activeRobot: robot };
    }
    this.state.activeRobot = robot;
    return { granted: true, activeRobot: robot };
  }

  async release(robot) {
    if (this.state.activeRobot === robot) this.state.activeRobot = null;
    return { released: true };
  }

  async narrow(robot) {
    // Exclusive baseline keeps the lock until the complete motion finishes.
    return { updated: this.state.activeRobot === robot };
  }
}

// One arm per contested region.
//
// A motion arrives with the set of shared voxels it will pass through. An empty
// set is granted unconditionally -- the arm is moving where no other arm can
// reach, so there is nothing to coordinate about, and that is the overwhelming
// majority of motions. A non-empty set is granted if it is disjoint from every
// live claim.
//
// Centralising this table is deliberate and consistent with the arity argument
// in the design notes: pairwise safety genuinely favours a single consistent
// view, and this is a pairwise structure. What stays decentralised is the
// GEOMETRY -- each core derives the shared regions itself, and gets the same
// answer as its peers because the estimator is order-independent.
//
// Two ways to ask. A plain claim is granted only if nothing live overlaps it,
// which is what a motion whose path is not known yet must have: it cannot say
// WHEN it will be anywhere, so it needs all of it at once.
//
// An ORDERED claim is a known path that is willing to follow. It is granted at
// once, behind every claim already live, and told which of its voxels those
// earlier claims still hold; the core then plays its path only up to the first
// sample that needs one. Earlier claims only ever shrink, so a follower never
// waits on anything that is not already leaving. And because an ordered claim
// only ever waits on claims granted before it, no two claims can wait on each
// other: the earliest holder is never waiting, so the chain always moves.
//
// That is what lets the receiver leave a rendezvous while the giver is still
// inside the shared region. Reserving the receiver's WHOLE path up front made it
// wait until the giver had left every voxel of it -- near a shared start, most
// of the common volume -- when all it needed was to stay behind the giver.
class RegionArbiter {
  constructor({ leaseMs = 120000, now = () => Date.now() } = {}) {
    this.leaseMs = leaseMs;
    this.now = now;
    this.claims = new Map();
    this.sequence = 0;
  }

  get supportsOrdered() { return true; }

  // What earlier claims still hold of this robot's claim, by holder.
  blockersOf(robot) {
    const mine = this.claims.get(robot);
    if (!mine) return {};
    const blockedBy = {};
    for (const [holder, claim] of this.claims) {
      if (holder === robot || claim.sequence > mine.sequence) continue;
      const shared = [...claim.voxels].filter((key) => mine.voxels.has(key));
      if (shared.length) blockedBy[holder] = shared;
    }
    return blockedBy;
  }

  // Who holds anything right now, and the scene root each named at claim time,
  // so a follower can tell its preflight which arm it is following. The
  // sequence tells a follower that an arm it was behind finished and claimed
  // again -- the same name, but no longer the motion it was following.
  holders() {
    return Object.fromEntries([...this.claims].map(([robot, claim]) =>
      [robot, { root: claim.root ?? null, sequence: claim.sequence }]));
  }

  expire() {
    const cutoff = this.now();
    for (const [robot, claim] of this.claims) {
      if (claim.expiresAt <= cutoff) this.claims.delete(robot);
    }
  }

  get activeRobot() {
    this.expire();
    const first = this.claims.keys().next();
    return first.done ? null : first.value;
  }

  get activeRobots() {
    this.expire();
    return [...this.claims.keys()];
  }

  async acquire(robot, voxels = [], { ordered = false, root = null } = {}) {
    this.expire();
    if (this.claims.has(robot)) {
      return { granted: false, activeRobot: robot, reason: "motion_busy" };
    }
    const wanted = new Set(voxels);
    if (ordered) {
      this.sequence += 1;
      this.claims.set(robot, { voxels: wanted, expiresAt: this.now() + this.leaseMs,
        sequence: this.sequence, ordered: true, root });
      return {
        granted: true,
        ordered: true,
        activeRobot: robot,
        claimedVoxels: wanted.size,
        concurrentArms: this.claims.size,
        blockedBy: this.blockersOf(robot),
        holders: this.holders()
      };
    }
    if (wanted.size > 0) {
      for (const [holder, claim] of this.claims) {
        for (const key of claim.voxels) {
          if (wanted.has(key)) {
            return {
              granted: false,
              activeRobot: holder,
              reason: "region_claimed",
              conflictVoxel: key
            };
          }
        }
      }
    }
    this.sequence += 1;
    this.claims.set(robot, { voxels: wanted, expiresAt: this.now() + this.leaseMs,
      sequence: this.sequence, ordered: false, root });
    return {
      granted: true,
      activeRobot: robot,
      claimedVoxels: wanted.size,
      concurrentArms: this.claims.size
    };
  }

  async release(robot) {
    this.claims.delete(robot);
    return { released: true };
  }

  async narrow(robot, voxels = []) {
    this.expire();
    const claim = this.claims.get(robot);
    if (!claim) return { updated: false, reason: "motion_claim_missing" };
    const remaining = new Set(voxels);
    if ([...remaining].some((key) => !claim.voxels.has(key))) {
      return { updated: false, reason: "motion_claim_expansion" };
    }
    // Atomic shrink: never drop ownership between release and reacquire.
    claim.voxels = remaining;
    claim.expiresAt = this.now() + this.leaseMs;
    return { updated: true, claimedVoxels: remaining.size, concurrentArms: this.claims.size,
      ...(claim.ordered ? { ordered: true, blockedBy: this.blockersOf(robot), holders: this.holders() } : {}) };
  }
}

// The host side of the same lock. Deliberately tiny and deliberately not a WoT
// Thing: it is cell infrastructure, not an affordance any agent should see.
class ArbiterServer {
  constructor(port, options = {}) {
    this.port = Number(port);
    this.arbiter = options.mode === "exclusive"
      ? new LocalArbiter()
      : new RegionArbiter({ leaseMs: options.leaseMs });
    this.mode = options.mode === "exclusive" ? "exclusive" : "region";
    this.server = null;
  }

  async start() {
    this.server = http.createServer((request, response) => {
      const url = new URL(request.url, "http://localhost");
      const robot = url.searchParams.get("robot") || "";
      const reply = (body) => {
        response.writeHead(200, { "content-type": "application/json" });
        response.end(JSON.stringify(body));
      };
      if (url.pathname === "/acquire" || url.pathname === "/narrow") {
        // A claim can hold thousands of voxel keys, which is a request body
        // rather than a query string.
        const chunks = [];
        request.on("data", (chunk) => chunks.push(chunk));
        request.on("end", () => {
          let voxels = [];
          let claimOptions = {};
          if (chunks.length > 0) {
            try {
              const body = JSON.parse(Buffer.concat(chunks).toString());
              voxels = body.voxels || [];
              claimOptions = { ordered: body.ordered === true,
                root: Number.isInteger(body.root) ? body.root : null };
            } catch (error) {
              reply({ granted: false, updated: false, reason: "invalid_claim" });
              return;
            }
          }
          if (!Array.isArray(voxels)) {
            reply({ granted: false, updated: false, reason: "invalid_claim" });
            return;
          }
          (url.pathname === "/narrow"
            ? this.arbiter.narrow(robot, voxels)
            : this.arbiter.acquire(robot, voxels, claimOptions)).then(reply);
        });
        return;
      }
      if (url.pathname === "/release") {
        this.arbiter.release(robot).then(reply);
        return;
      }
      if (url.pathname === "/state") {
        reply({
          mode: this.mode,
          activeRobot: this.arbiter.activeRobot,
          activeRobots: this.arbiter.activeRobots || [this.arbiter.activeRobot].filter(Boolean)
        });
        return;
      }
      response.writeHead(404);
      response.end();
    });
    await new Promise((resolve, reject) => {
      this.server.once("error", reject);
      this.server.listen(this.port, "127.0.0.1", resolve);
    });
    return this;
  }

  async stop() {
    if (!this.server) return;
    await new Promise((resolve) => this.server.close(resolve));
    this.server = null;
  }

  // Whichever robot's core died still holds the lock. Clearing it is the host's
  // job, because the process that would have released it is gone.
  releaseFor(robot) {
    return this.arbiter.release(robot);
  }
}

class RemoteArbiter {
  constructor({ port, timeoutMs = 5000, fetchImpl = globalThis.fetch }) {
    this.base = `http://127.0.0.1:${Number(port)}`;
    this.timeoutMs = timeoutMs;
    this.fetchImpl = fetchImpl;
  }

  async call(pathname, robot, body = null) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.timeoutMs);
    try {
      const response = await this.fetchImpl(
        `${this.base}${pathname}?robot=${encodeURIComponent(robot)}`,
        body === null
          ? { signal: controller.signal }
          : {
            method: "POST",
            headers: { "content-type": "application/json" },
            body: JSON.stringify(body),
            signal: controller.signal
          }
      );
      return await response.json();
    } finally {
      clearTimeout(timer);
    }
  }

  // Whether an ordered claim was honoured is the server's to say: an exclusive
  // host grants it as a plain claim, and the reply then carries no `ordered`.
  get supportsOrdered() { return true; }

  // A core that cannot reach the arbiter must not move. Assuming the lock is
  // free because the question could not be asked is the one failure mode this
  // whole mechanism exists to prevent.
  async acquire(robot, voxels = [], { ordered = false, root = null } = {}) {
    try {
      return await this.call("/acquire", robot, ordered ? { voxels, ordered: true, root } : { voxels });
    } catch (error) {
      return { granted: false, activeRobot: "unknown", reason: "arbiter_unreachable" };
    }
  }

  async release(robot) {
    try {
      return await this.call("/release", robot);
    } catch (error) {
      return { released: false, reason: "arbiter_unreachable" };
    }
  }

  async narrow(robot, voxels = []) {
    try {
      return await this.call("/narrow", robot, { voxels });
    } catch (error) {
      return { updated: false, reason: "arbiter_unreachable" };
    }
  }
}

module.exports = { ArbiterServer, LocalArbiter, RegionArbiter, RemoteArbiter };
