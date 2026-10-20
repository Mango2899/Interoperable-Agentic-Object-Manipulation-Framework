"use strict";

// The cell's address book, and the scene queries that answer questions about it.
//
// This replaces the shared `gripCoordinator` object, which held four different
// kinds of thing at once: an address book, a cached cube owner, a transfer lock
// and the live handover session. Only the first of those is configuration; the
// other three were mutable coordination state living in one process's heap,
// which is precisely what stops each robot's movement and safety core from
// running as its own process.
//
// What is left here is deliberately state-free. A peer entry says how to reach
// a robot and what its manifest configures -- nothing that changes as the cell
// runs. Every question that DOES change ("who is holding the cube?") is answered
// by asking the scene, because the scene is where the answer actually lives: a
// robot holds the cube when its gripper has the cube parented to it, and a
// cached boolean that says otherwise is simply wrong.
//
// A peer is four named operations, not a channel for arbitrary Lua. That is the
// seam: `localPeer` builds them from a driver call through the shared simulator
// connection, `remotePeer` builds the same four from HTTP calls to that robot's
// own lower Thing. Everything below is written against the operations, so which
// one a cell is using changes nothing here.

const NO_OWNER = "none";

class CellPeers {
  constructor(options = {}) {
    this.cube = options.cube || null;
    this.entries = new Map();
  }

  // `peer` implements gripperState, acceptTransfer, openSession, updateSession
  // and closeSession. `manifest` is the robot's manifest, read for
  // configuration only. Build one with localPeer() or remotePeer() rather than
  // by hand.
  register(name, peer) {
    if (!name) throw new Error("A cell peer needs a robot name");
    for (const operation of ["gripperState", "acceptTransfer", "openSession", "updateSession", "closeSession"]) {
      if (typeof peer?.[operation] !== "function") {
        throw new Error(`Cell peer ${name} does not implement ${operation}`);
      }
    }
    this.entries.set(name, { ...peer, name });
    return this;
  }

  has(name) {
    return this.entries.has(name);
  }

  get(name) {
    return this.entries.get(name) || null;
  }

  names() {
    return [...this.entries.keys()];
  }

  manifestOf(name) {
    return this.entries.get(name)?.manifest || null;
  }

  sessionsOf(name) {
    return this.entries.get(name)?.sessions || null;
  }

  async gripperState(name) {
    const peer = this.entries.get(name);
    if (!peer) return null;
    return peer.gripperState();
  }

  // Who physically holds the cube, asked of every arm at once.
  //
  // Two robots reporting a hold is not something to average over or resolve by
  // precedence: it means the scene has the cube parented to two grippers, or a
  // transfer tore. Saying so is the only useful response.
  async resolveCubeOwnership() {
    const names = this.names();
    const states = await Promise.all(names.map(async (name) => {
      try {
        return { name, state: await this.gripperState(name) };
      } catch (error) {
        return { name, state: null, error: error?.message || String(error) };
      }
    }));
    const holders = states.filter(({ state }) => state?.holding === true).map(({ name }) => name);
    const unreachable = states.filter(({ state, error }) => !state && error).map(({ name }) => name);
    const object = states.find(({ state }) => state?.object)?.state?.object || this.cube || null;
    return {
      owner: holders.length === 1 ? holders[0] : NO_OWNER,
      object,
      holders,
      contested: holders.length > 1,
      // An arm that cannot be reached cannot be shown NOT to hold the cube, so a
      // "nobody holds it" answer with an unreachable arm in the cell is a guess.
      unreachable
    };
  }

  async resolveCubeOwner() {
    return (await this.resolveCubeOwnership()).owner;
  }

  async acceptTransfer(name) {
    const peer = this.entries.get(name);
    if (!peer) return { success: false, reason: "unknown_target_robot", targetRobot: name };
    return peer.acceptTransfer();
  }

  // Give the receiving core its own copy of the session. The giver registers a
  // session and the receiver has to know about it before it can arm its
  // approach; with one shared object that was implicit, and across processes it
  // is a message. Failing to deliver it is not fatal to the giver's own leg --
  // the receiver reports the missing session when it tries to use it, which is
  // a clearer failure than the giver aborting on the peer's behalf.
  async openSessionOn(name, record) {
    const peer = this.entries.get(name);
    if (!peer) return { success: false, reason: "unknown_target_robot", targetRobot: name };
    return peer.openSession(record);
  }

  async updateSessionOn(name, sessionId, changes) {
    const peer = this.entries.get(name);
    if (!peer) return { success: false, reason: "unknown_target_robot", targetRobot: name };
    return peer.updateSession(sessionId, changes);
  }

  // The other half of openSessionOn. A giver that abandons a rendezvous -- it
  // retried with a different point, or failed outright -- has to take the
  // replica back, because the receiver holds a record for a handover that is
  // never going to happen and has no intention running that would ever notice.
  // Left behind, that record is what the receiver's own core later reads as
  // "a handover is already in progress" when the chain reaches it and it tries
  // to become a giver in its turn.
  async closeSessionOn(name, sessionId) {
    const peer = this.entries.get(name);
    if (!peer) return { success: false, reason: "unknown_target_robot", targetRobot: name };
    return peer.closeSession(sessionId);
  }
}

// A peer served from this process: the driver call goes straight down the
// shared simulator connection, and the session registry is an object reference.
function localPeer({ name, manifest, invoke, sessions = null }) {
  if (typeof invoke !== "function") throw new Error(`Local cell peer ${name} needs an invoke function`);
  return {
    name,
    manifest,
    sessions,
    gripperState: () => invoke("return getGripperState()"),
    acceptTransfer: () => invoke("return womatSafeCall(acceptTransferredGripObject)"),
    openSession: async (record) => {
      if (!sessions) return { success: false, reason: "peer_has_no_session_registry", targetRobot: name };
      sessions.open(record);
      return { success: true, session: record.id, targetRobot: name };
    },
    updateSession: async (sessionId, changes) => {
      if (!sessions) return { success: false, reason: "peer_has_no_session_registry", targetRobot: name };
      return sessions.update(sessionId, changes)
        ? { success: true, session: sessionId, targetRobot: name }
        : { success: false, reason: "handover_session_missing_or_mismatched", targetRobot: name };
    },
    closeSession: async (sessionId) => {
      if (!sessions) return { success: false, reason: "peer_has_no_session_registry", targetRobot: name };
      sessions.close(sessionId);
      return { success: true, session: sessionId, targetRobot: name };
    }
  };
}

// A peer running in its own process, reached over its lower Thing.
//
// The same four operations, over HTTP. A peer that does not answer must surface
// as a failed operation rather than as a hang, because the caller's next move
// is a physical one: `resolveCubeOwnership` reports an unreachable arm, and
// deciding nobody holds the cube while an arm never replied is how two robots
// get cleared to pick the same one.
function remotePeer({ name, manifest, url, timeoutMs = 5000, fetchImpl = globalThis.fetch }) {
  if (!url) throw new Error(`Remote cell peer ${name} needs a lower Thing URL`);
  const base = String(url).replace(/\/$/, "");
  const call = async (suffix, options = {}) => {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const response = await fetchImpl(`${base}${suffix}`, { ...options, signal: controller.signal });
      if (!response.ok) {
        return { success: false, reason: "peer_request_failed", status: response.status, targetRobot: name };
      }
      const text = await response.text();
      return text ? JSON.parse(text) : { success: true };
    } finally {
      clearTimeout(timer);
    }
  };
  const post = (action, body) => call(`/actions/${action}`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body ?? {})
  });
  return {
    name,
    manifest,
    sessions: null,
    gripperState: () => call("/properties/getGripperState"),
    acceptTransfer: () => post("acceptTransfer"),
    openSession: (record) => post("registerHandoverSession", { session: record }),
    updateSession: (sessionId, changes) => post("updateHandoverSession", { session: sessionId, changes }),
    closeSession: (sessionId) => post("closeHandoverSession", { session: sessionId })
  };
}

// One robot's own view of the handovers it is part of.
//
// Each core keeps its own registry. The giver creates the record and replicates
// it; the receiver holds its own copy and closes it on its own schedule. The old
// shared object needed an `approachEnabledRobots` list to decide when the
// session was over, because one object was serving two robots -- a reference
// count standing in for the fact that each robot has its own answer.
class HandoverSessions {
  constructor(robotName) {
    this.robotName = robotName;
    this.records = new Map();
    this.transferInFlight = false;
  }

  get active() {
    return this.records.size > 0;
  }

  open(record) {
    const stored = { ...record, id: String(record.id) };
    this.records.set(stored.id, stored);
    return stored;
  }

  get(sessionId) {
    return this.records.get(String(sessionId)) || null;
  }

  update(sessionId, changes) {
    const record = this.get(sessionId);
    if (!record) return null;
    Object.assign(record, changes);
    return record;
  }

  close(sessionId) {
    return this.records.delete(String(sessionId));
  }

  // The giver may only run one handover at a time, and a second registration
  // has to say which one is already running rather than quietly replacing it.
  activeSession() {
    return this.records.values().next().value || null;
  }

  // What actually stops this robot from starting a handover of its own.
  //
  // Not simply "any record": this registry holds two different things. A
  // session this robot SOURCED is its own handover and does block a second
  // one. A session it is the TARGET of is a replica the giver put here, and
  // once the cube has physically moved -- state "transferred" -- the arms are
  // still entangled and it blocks too. A replica still sitting at "registered"
  // is a rendezvous that was announced and never happened; it describes
  // nothing physical, and treating it as an active handover is what strands an
  // intermediate robot in a chain, unable to pass the cube on because of a
  // record its own giver abandoned.
  blockingSession(robotName) {
    for (const record of this.records.values()) {
      if (record.sourceRobot === robotName) return record;
      if (record.state !== "registered") return record;
    }
    return null;
  }

  beginTransfer() {
    if (this.transferInFlight) return false;
    this.transferInFlight = true;
    return true;
  }

  endTransfer() {
    this.transferInFlight = false;
  }
}

module.exports = { CellPeers, HandoverSessions, NO_OWNER, localPeer, remotePeer };
