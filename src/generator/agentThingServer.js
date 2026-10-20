#!/usr/bin/env node

const fs = require("fs");
const path = require("path");
const net = require("net");

const { AgentThing } = require("../../dist/WoT/agentThing");
const { AgentMessage } = require("../../dist/WoT/agentConnector");
const { InteractionSpaceController } = require("./interactionSpace");
const { couldOverlap } = require("./neighborTest");
const { computeLogFor } = require("./computeLog");
const { placementFromBasePose } = require("./sharedCell");

function parseArgs(argv) {
  const args = {};
  for (let i = 0; i < argv.length; i += 1) {
    const token = argv[i];
    if (!token.startsWith("--")) {
      continue;
    }
    const key = token.slice(2);
    const next = argv[i + 1];
    if (!next || next.startsWith("--")) {
      args[key] = true;
    } else {
      args[key] = next;
      i += 1;
    }
  }
  return args;
}

function usage() {
  return [
    "Usage:",
    "  npm run robot:agent -- --manifest generated/robots/<name>/manifest.json",
    "",
    "Run the generated robot AgentThing bridge before starting the generated .jcm.",
    "",
    "Testing the route search without a simulator:",
    "  --mock-sites \"Robot1_grip_support=holding:Robot1_grip_cube,Robot4_grip_support_b=out\"",
    "  --mock-costs \"Robot2=0.4,Robot3=0.9\"",
    "  --mock-neighbors \"Robot2,Robot3\"",
    "A site is out | empty | holding:<payload>. An unlisted neighbour's edge is unpriceable."
  ].join("\n");
}

function fail(message) {
  console.error(message);
  console.error("");
  console.error(usage());
  process.exit(1);
}

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, "utf8"));
}

function resolveRepoPath(repoRoot, relativePath) {
  return path.join(repoRoot, relativePath);
}

function canListen(port) {
  return new Promise((resolve) => {
    const server = net.createServer();
    server.once("error", (error) => {
      resolve({ ok: false, error });
    });
    server.once("listening", () => {
      server.close(() => resolve({ ok: true }));
    });
    server.listen(port);
  });
}

async function assertPortsAvailable(ctx) {
  const checks = [
    ["Agent TD HTTP", Number(ctx.thingConfig.TDPort)],
    ["Agent websocket", Number(ctx.thingConfig.websocketPort)]
  ];

  for (const [label, port] of checks) {
    const result = await canListen(port);
    if (!result.ok) {
      if (result.error && result.error.code === "EADDRINUSE") {
        fail(`${label} port ${port} is already in use. The generated AgentThing may already be running; if so, keep it running and start JaCaMo with ./gradlew -Pjcm=${ctx.manifest.generatedFiles.jcm}.`);
      }
      throw result.error;
    }
  }
}

function createContext(manifestFile, options = {}) {
  const repoRoot = path.resolve(__dirname, "..", "..");
  const manifestPath = path.resolve(process.cwd(), manifestFile);
  if (!fs.existsSync(manifestPath)) {
    fail(`Manifest file does not exist: ${manifestPath}`);
  }

  const manifest = readJson(manifestPath);
  const configPath = path.join(repoRoot, "config.json");
  const config = readJson(configPath);
  const configKey = manifest.configKey || manifest.name;
  const thingConfig = config[configKey];
  if (!thingConfig) {
    fail(`Missing config.json entry for generated robot key: ${configKey}`);
  }

  return {
    repoRoot,
    manifest,
    configKey,
    thingConfig,
    mockSites: options.mockSites || null,
    mockCosts: options.mockCosts || null,
    mockNeighbors: options.mockNeighbors || null,
    agentTdPath: resolveRepoPath(repoRoot, manifest.generatedFiles.agentTd),
    lowerLayerUrl: manifest.lowerLayerUrl
  };
}

function unwrapArrayValue(value) {
  // JaCaMo turns a structure nested inside a list into its textual form.
  // [Session, array(...)] therefore arrives differently from a top-level
  // array(...) parsed by JasonTermParser. Accept numeric arrays in both forms.
  if (typeof value === "string") {
    const match = /^array\((.*)\)$/s.exec(value.trim());
    if (match) {
      try {
        const numbers = JSON.parse(`[${match[1]}]`);
        if (numbers.every((number) => typeof number === "number" && Number.isFinite(number))) return numbers;
      } catch (_) { /* Leave malformed terms for the caller to reject. */ }
    }
  }
  if (Array.isArray(value)) {
    return value;
  }
  if (value && Array.isArray(value.value)) {
    return value.value;
  }
  if (value && Array.isArray(value.args)) {
    return value.args;
  }
  if (value && Array.isArray(value.terms)) {
    return value.terms;
  }
  if (value && value.functor === "array") {
    return unwrapArrayValue(value.values || value.value || value.args || value.terms);
  }
  return value;
}

function asBoolean(value) {
  if (typeof value === "boolean") {
    return value;
  }
  return String(value).toLowerCase() === "true";
}

// Number(null) is 0 and Number("") is 0, so coercing a missing coordinate
// yields a perfectly finite point at the robot's own base. A cost measured from
// there is not obviously wrong -- it is a plausible number for a rendezvous
// nobody will ever travel to, which is worse than no answer at all. Absence has
// to survive as absence all the way to the check that rejects it.
// Standing in for the simulator.
//
// The consensus protocol is agents talking to agents: a flood of questions, a
// wave of accumulated costs, a minimum at every node. None of that needs a
// robot -- but reaching it through CoppeliaSim means a scene load and a full
// cell startup for every protocol change, and a routing bug found there costs
// forty minutes to reproduce. These two flags let the whole search run on four
// bridges and a MAS with no simulator at all, against costs and sites the test
// chose, so the topology under test is the one the test meant.
//
//   --mock-sites  "Robot1_grip_support=holding:Robot1_grip_cube,Robot4_grip_support_b=out"
//   --mock-costs  "Robot2=0.4,Robot3=0.9"
//
// A site is "out" (seen, not reachable), "empty" (reachable, no payload), or
// "holding:<payload>". An unlisted neighbour has no cost and its edge is
// reported unpriceable, which is a state the real cell produces too.
function parseMockSites(spec) {
  if (!spec || spec === true) return null;
  const sites = [];
  for (const entry of String(spec).split(",").map((part) => part.trim()).filter(Boolean)) {
    const [alias, rawState = "empty"] = entry.split("=").map((part) => part.trim());
    if (!alias) continue;
    const [state, payload] = rawState.split(":").map((part) => part.trim());
    const reachable = state !== "out";
    sites.push({
      alias,
      reachable,
      admission: reachable ? "mock" : "outside",
      occupiedBy: state === "holding" ? (payload || `${alias}_payload`) : "none",
      grasp: { x: 0, y: 0, z: 0 },
      world: { x: 0, y: 0, z: 0 },
      local: { x: 0, y: 0, z: 0 }
    });
  }
  return sites;
}

// The cell's shape, when there is no cell. Without this the mock mode is not
// simulator-free after all: neighbour discovery reads workspace signatures off
// the lower Things, and an arm with no neighbours has nobody to ask, so the
// search would have nothing to do.
//
//   --mock-neighbors "Robot2,Robot3"
function parseMockNeighbors(spec) {
  if (!spec || spec === true) return null;
  return String(spec)
    .split(",")
    .map((part) => canonicalRobotName(part.trim()))
    .filter(Boolean);
}

function parseMockCosts(spec) {
  if (!spec || spec === true) return null;
  const costs = new Map();
  for (const entry of String(spec).split(",").map((part) => part.trim()).filter(Boolean)) {
    const [robot, value] = entry.split("=").map((part) => part.trim());
    const cost = Number(value);
    if (robot && Number.isFinite(cost)) costs.set(canonicalRobotName(robot), cost);
  }
  return costs;
}

function strictNumber(value) {
  if (value === null || value === undefined || value === "") return NaN;
  return Number(value);
}

function coordinate(position, axis) {
  const value = Number(position?.[axis]);
  return Number.isFinite(value) ? value : 0;
}

function asJasonAtom(value, fallback = "unknown") {
  const sanitized = String(value ?? fallback)
    .replace(/[^A-Za-z0-9_]/g, "_")
    .replace(/^([^a-z])/, "v_$1");
  return sanitized.length > 0
    ? sanitized.substring(0, 1).toLowerCase() + sanitized.substring(1)
    : fallback;
}

// A robot's name as the atom the cell already calls it by.
//
// asJasonAtom cannot do this job. Its guard against an atom that Jason would
// read as a variable prefixes anything not starting with a lower-case letter,
// so "Robot3" becomes v_Robot3 -- a perfectly valid atom naming a robot that
// does not exist. Every plan matches on robot3, so a belief carrying v_Robot3
// unifies with nothing and simply never arrives: interactionSpaceFailed and
// handoverCandidatesFailed were both sent that way, and both waits sat until
// their timeout with the answer already in the belief base under a name nobody
// was listening for. Live cell: Robot1 spent 60s per startup not hearing that
// Robot3 shares no space with it.
//
// The rule is the generator's own jcmAtom -- lower-case the leading run, keep
// the rest -- so Robot3 is robot3 and YouBot stays youBot rather than
// collapsing to youbot, which is what a whole-name toLowerCase() produced and
// what the generated pair topology never called it.
function robotAtom(value, fallback = "unknown_robot") {
  const atom = String(value ?? "")
    .replace(/[^A-Za-z0-9_]/g, "_")
    .replace(/^[^a-z]+/, (prefix) => prefix.toLowerCase());
  if (/^[a-z][A-Za-z0-9_]*$/.test(atom)) return atom;
  // A name no lower-casing can rescue -- one starting with a digit, say. The
  // generator prefixes those rather than inventing a name, and this has to
  // agree with it character for character or the belief misses again for a
  // subtler reason than the one it was written to fix.
  return atom.length > 0 ? `robot_${atom.toLowerCase()}` : fallback;
}

// The numbers behind a refusal, kept small enough to sit on one log line.
function failureDiagnostics(result) {
  const round = (value) => (Number.isFinite(value) ? Number(value.toFixed(4)) : undefined);
  const entries = {
    nearestApproach: round(result.nearestApproach),
    voxelSize: round(result.voxelSize),
    estimatedClosePoints: round(result.estimatedClosePoints),
    estimatorMode: result.estimatorMode,
    detail: result.detail
  };
  const kept = Object.entries(entries).filter(([, value]) => value !== undefined);
  return kept.length > 0 ? Object.fromEntries(kept) : undefined;
}

function canonicalRobotName(value) {
  const name = String(value || "");
  return name.length > 0 ? name.substring(0, 1).toUpperCase() + name.substring(1) : name;
}

class GeneratedRobotAgentThing extends AgentThing {
  constructor(ctx) {
    super(ctx.thingConfig, ctx.agentTdPath, {}, {});
    this.ctx = ctx;
    this.lowerTD = null;
    this.lowerThing = null;
    this.motionQueue = Promise.resolve();
    this.interactionSpace = new InteractionSpaceController(ctx);
    this.mockSites = ctx.mockSites || null;
    this.mockCosts = ctx.mockCosts || null;
    this.mockNeighbors = ctx.mockNeighbors || null;
    // Any mock flag means there is no cell behind this bridge to reach for.
    this.mocked = Boolean(this.mockSites || this.mockCosts || this.mockNeighbors);
    this.partnerLowerThings = new Map();
    const defaults = ctx.manifest.agentDefaults || {};
    const homePosition = defaults.homePosition || [0, 0, 0.3];
    this.homePosition = {
      x: Number(homePosition[0]),
      y: Number(homePosition[1]),
      z: Number(homePosition[2])
    };
    this.homeJoints = defaults.homeJoints || null;
  }

  // Created on first use rather than in the constructor: an agent assembled
  // from its prototype (as the tests do) still logs what it computed.
  get computeLog() {
    if (!this.cachedComputeLog) this.cachedComputeLog = computeLogFor(this.ctx);
    return this.cachedComputeLog;
  }

  async ensureLowerThing() {
    if (this.lowerThing) {
      return;
    }
    if (!this.ctx.lowerLayerUrl) {
      throw new Error("Manifest does not define lowerLayerUrl.");
    }

    [this.lowerTD, this.lowerThing] = await this.fetchAgentTD(this.ctx.lowerLayerUrl);
  }

  async handleCustomAgentRequests(data) {
    const targetAgentId = data.targetAgentId;
    const content = data.content || {};
    const keyword = content.keyword;
    const value = content.value ?? [];

    console.log(`[AgentThing:${this.ctx.manifest.name}] agent request ${data.performative} target=${targetAgentId} keyword=${keyword}`);

    if (targetAgentId !== "ownLayer1") {
      return false;
    }

    // In mock mode the two affordances the search needs answer from the flags,
    // so the bridge must not insist on a lower Thing it will never call.
    // With no cell behind it, reaching for a lower Thing throws and the agent
    // waits out a 60-second deadline for a belief that was never coming. The
    // affordances the search uses answer from the flags instead.
    const mockedHere = this.mocked && [
      "detectTaskSites", "estimateEdgeCost", "discoverNeighbors",
      "computeInteractionSpace", "refreshInteractionSpace", "renderInteractionSpace"
    ].includes(keyword);
    if (!mockedHere) {
      await this.ensureLowerThing();
    }

    if (mockedHere && (keyword === "computeInteractionSpace" || keyword === "refreshInteractionSpace")) {
      // The shared space is what a real handover is planned in. The search does
      // not read it -- it reads the costs, which are mocked -- so it is
      // declared ready rather than computed, and the arms get on with talking.
      const targetRobot = canonicalRobotName(value[0]);
      const atom = robotAtom(targetRobot);
      await this.addBelief2Agent("interactionRobot", [`${atom}_agent`, atom], false, "ownLayer1");
      await this.addBelief2Agent("interactionSpace", ["mock_space", atom, 0, 0, 0, 0, 0], false, "ownLayer1");
      await this.addBelief2Agent("interactionSpaceReady", ["mock_space", atom], false, "ownLayer1");
      return true;
    }

    if (mockedHere && keyword === "renderInteractionSpace") {
      const targetRobot = canonicalRobotName(value[0]);
      await this.addBelief2Agent(
        "interactionVisual",
        [robotAtom(targetRobot), 0],
        false,
        "ownLayer1"
      );
      return true;
    }

    if (keyword === "moveTo" || keyword === "moveAt") {
      const payload = { x: Number(value[0]), y: Number(value[1]), z: Number(value[2]) };
      const result = await this.enqueueMotion(() => this.invokeLowerMove(payload), {
        action: "moveTocartesianPosition", payload
      });
      console.log(`[AgentThing:${this.ctx.manifest.name}] lower move result`, result);
      if (!result.success) {
        await this.publishMotionFailure(result, "move_failed");
        return true;
      }
      // Where the arm came to rest to let the cube go. Published for every
      // release, not only a relocated one, so the retreat has one rule: lift
      // from where the cube actually was let go of. Told before movedTo,
      // because movedTo is what the waiting plan wakes on -- publish it second
      // and the plan can read the belief base before this one has arrived, and
      // retreat over the centre of a support the cube is not standing on.
      if (result.planner === "top_down_release" && result.dropPoint) {
        if (result.retreatPoint) {
          await this.addBelief2Agent(
            "placementRetreat",
            [result.retreatPoint.x, result.retreatPoint.y, result.retreatPoint.z],
            false,
            "ownLayer1"
          );
        }
        await this.addBelief2Agent(
          "placedAt",
          [result.dropPoint.x, result.dropPoint.y, result.dropPoint.z],
          false,
          "ownLayer1"
        );
      }
      await this.addBelief2Agent("movedTo", value, false, "ownLayer1");
      return true;
    }

    if (keyword === "goHome" || keyword === "homeing" || keyword === "goHomeJoints" || keyword === "goHomeAfterHandover") {
      const requestedHomeJoints = keyword === "goHomeJoints" ? value : null;
      const result = await this.enqueueMotion(async () => {
        if (keyword === "goHomeAfterHandover") {
          const homeJoints = unwrapArrayValue(value[1]);
          if (!Array.isArray(homeJoints) || !homeJoints.length
            || !homeJoints.every((joint) => typeof joint === "number" && Number.isFinite(joint))) {
            return { success: false, reason: "invalid_home_joints" };
          }
          return this.invokeLowerAction("returnHomeAfterHandover", {
            session: value[0], homeJoints
          });
        }
        return this.invokeHomeMotion(requestedHomeJoints);
      }, { action: keyword === "goHomeAfterHandover" ? "returnHomeAfterHandover" : "home motion" });
      console.log(`[AgentThing:${this.ctx.manifest.name}] lower home result`, result);
      if (!result.success) {
        await this.publishMotionFailure(result, "home_failed");
        return true;
      }
      await this.addBelief2Agent("movedTo", ["home"], false, "ownLayer1");
      return true;
    }

    if (keyword === "closeGripper" || keyword === "openGripper") {
      const result = await this.enqueueMotion(() => this.invokeLowerGripper(keyword), { action: keyword });
      console.log(`[AgentThing:${this.ctx.manifest.name}] lower gripper result`, result);
      if (!result.success) {
        await this.addBelief2Agent("gripFailed", [result.reason || `${keyword}_failed`], false, "ownLayer1");
        return true;
      }
      if (keyword === "closeGripper") {
        await this.addBelief2Agent("grippedObject", [result.object || "object"], false, "ownLayer1");
      } else {
        await this.addBelief2Agent("releasedObject", [result.object || "object"], false, "ownLayer1");
      }
      return true;
    }

    if (keyword === "computeInteractionSpace" || keyword === "refreshInteractionSpace") {
      const targetRobot = canonicalRobotName(value[0]);
      let result;
      const startedAt = Date.now();
      try {
        const placements = await this.readCellPlacements(targetRobot);
        // synthesizePair: with neighbours discovered rather than configured, a
        // pair worth computing may not appear in the descriptor at all.
        result = this.interactionSpace.build(targetRobot, { placements, synthesizePair: true });
      } catch (error) {
        result = { success: false, reason: "interaction_space_estimation_failed", detail: error.message };
      }
      this.computeLog.record({
        pairKey: result.pairKey || `${this.ctx.manifest.name}__${targetRobot}`,
        targetRobot,
        cached: result.success ? Boolean(result.cached) : undefined,
        voxelCount: result.voxelCount,
        version: result.version ? String(result.version).slice(0, 12) : undefined,
        durationMs: Date.now() - startedAt,
        reason: result.success ? undefined : result.reason,
        // What the estimator saw when it refused. A bare reason atom cannot
        // tell "these two arms genuinely never meet" apart from "something
        // went wrong in here", and the pairs that fail are exactly the ones
        // nobody can inspect afterwards -- no space file is written for them.
        diagnostics: result.success ? undefined : failureDiagnostics(result)
      });
      if (!result.success) {
        await this.addBelief2Agent(
          "interactionSpaceFailed",
          [robotAtom(targetRobot), asJasonAtom(result.reason, "estimation_failed")],
          false,
          "ownLayer1"
        );
        return true;
      }
      const targetAtom = robotAtom(targetRobot);
      const spaceId = asJasonAtom(result.id, "interaction_space");
      // A recomputed space and the cloud drawn in the scene are two different
      // artifacts. The scene's copy is baked by the pipeline, so without this
      // the arms would rendezvous correctly while the visible green points
      // still described the layout the cell used to have.
      if (true) {
        const redrawn = await this.redrawInteractionVisual(targetRobot);
        if (redrawn && !redrawn.success) {
          console.warn(
            `[AgentThing:${this.ctx.manifest.name}] could not redraw ${result.pairKey} visual:`,
            redrawn.reason || redrawn.message
          );
        }
      }
      await this.addBelief2Agent("interactionRobot", [`${targetAtom}_agent`, targetAtom], false, "ownLayer1");
      await this.addBelief2Agent(
        "interactionSpace",
        [spaceId, targetAtom, result.voxelSize, result.origin.x, result.origin.y, result.origin.z, result.voxelCount],
        false,
        "ownLayer1"
      );
      await this.addBelief2Agent("interactionSpaceReady", [spaceId, targetAtom], false, "ownLayer1");
      await this.addBelief2Agent(
        "interactionSpaceCache",
        [spaceId, asJasonAtom(String(result.pairKey || "").toLowerCase(), "interaction_pair"), asJasonAtom(result.version, "version"), Boolean(result.cached)],
        false,
        "ownLayer1"
      );
      return true;
    }

    if (keyword === "renderInteractionSpace") {
      const targetRobot = canonicalRobotName(value[0]);
      const drawn = await this.redrawInteractionVisual(targetRobot);
      const atom = robotAtom(targetRobot);
      if (!drawn?.success) {
        await this.addBelief2Agent(
          "interactionVisualFailed",
          [atom, asJasonAtom(drawn?.reason || "render_failed", "render_failed")],
          false,
          "ownLayer1"
        );
        return true;
      }
      await this.addBelief2Agent("interactionVisual", [atom, Number(drawn.pointCount || 0)], false, "ownLayer1");
      return true;
    }

    if (keyword === "discoverNeighbors") {
      await this.discoverNeighbors();
      return true;
    }

    if (keyword === "detectTaskSites") {
      await this.detectTaskSites();
      return true;
    }

    if (keyword === "estimateEdgeCost") {
      const [targetRobot, minZ, maxZ, mode] = value;
      await this.estimateEdgeCost(canonicalRobotName(targetRobot), Number(minZ), Number(maxZ), String(mode || "pickup"));
      return true;
    }

    if (keyword === "planHandover") {
      const [targetRobot, seed, minZ, maxZ] = value;
      const canonicalTarget = canonicalRobotName(targetRobot);
      const result = this.interactionSpace.select(canonicalTarget, {
        seed: Number(seed),
        minZ: Number(minZ),
        maxZ: Number(maxZ)
      });
      if (!result.success) {
        await this.addBelief2Agent(
          "handoverPlanFailed",
          [result.reason || "handover_plan_failed"],
          false,
          "ownLayer1"
        );
        return true;
      }
      const session = await this.invokeLowerAction("beginHandoverSession", {
        session: result.id,
        targetRobot: canonicalTarget,
        cubeWorld: result.cubeWorld
      });
      if (!session.success) {
        await this.addBelief2Agent(
          "handoverPlanFailed",
          [session.reason || "handover_session_rejected"],
          false,
          "ownLayer1"
        );
        return true;
      }
      await this.addBelief2Agent(
        "handoverPlan",
        [
          asJasonAtom(result.id, "handover_session"),
          coordinate(result.cubeWorld, "x"), coordinate(result.cubeWorld, "y"), coordinate(result.cubeWorld, "z"),
          coordinate(result.giver?.localTcp, "x"), coordinate(result.giver?.localTcp, "y"), coordinate(result.giver?.localTcp, "z"),
          coordinate(result.receiver?.localTcp, "x"), coordinate(result.receiver?.localTcp, "y"), coordinate(result.receiver?.localTcp, "z")
        ],
        false,
        "ownLayer1"
      );
      return true;
    }

    if (keyword === "rankHandoverCandidates") {
      const [targetRobot, minZ, maxZ] = value;
      const canonicalTarget = canonicalRobotName(targetRobot);
      const result = this.interactionSpace.rank(canonicalTarget, {
        minZ: Number(minZ),
        maxZ: Number(maxZ)
      });
      if (!result.success) {
        await this.addBelief2Agent(
          "handoverCandidatesFailed",
          [robotAtom(canonicalTarget), result.reason || "candidate_ranking_failed"],
          false,
          "ownLayer1"
        );
        return true;
      }
      const targetAtom = robotAtom(canonicalTarget);
      await this.addBelief2Agent(
        "handoverCore",
        [
          targetAtom,
          result.center.x, result.center.y, result.center.z,
          result.bounds.x[0], result.bounds.x[1],
          result.bounds.y[0], result.bounds.y[1],
          result.bounds.z[0], result.bounds.z[1],
          result.heightCandidateCount, result.coreCandidateCount, result.candidateCount
        ],
        false,
        "ownLayer1"
      );
      for (const candidate of result.candidates) {
        await this.addBelief2Agent(
          "handoverCandidate",
          [
            asJasonAtom(candidate.id, "candidate"), targetAtom,
            coordinate(candidate.cubeWorld, "x"), coordinate(candidate.cubeWorld, "y"), coordinate(candidate.cubeWorld, "z"),
            coordinate(candidate.giver?.localTcp, "x"), coordinate(candidate.giver?.localTcp, "y"), coordinate(candidate.giver?.localTcp, "z"),
            coordinate(candidate.receiver?.localTcp, "x"), coordinate(candidate.receiver?.localTcp, "y"), coordinate(candidate.receiver?.localTcp, "z"),
            candidate.centrality, candidate.density, candidate.giverSupport, candidate.receiverSupport,
            candidate.posture
          ],
          false,
          "ownLayer1"
        );
      }
      await this.addBelief2Agent("handoverCandidatesReady", [targetAtom, result.candidateCount], false, "ownLayer1");
      return true;
    }

    // The strike, told to the geometry capability as well as remembered as a
    // belief. The belief is what removes the candidate from the running; this
    // is what stops it propping up its neighbours' density scores.
    if (keyword === "rejectHandoverCandidate") {
      const [targetRobot, candidateId] = value;
      this.interactionSpace.rejectCandidate(canonicalRobotName(targetRobot), String(candidateId));
      return true;
    }

    if (keyword === "clearHandoverRejections") {
      const [targetRobot] = value;
      this.interactionSpace.clearRejectedCandidates(canonicalRobotName(targetRobot));
      return true;
    }

    if (keyword === "commitHandoverCandidate") {
      const [targetRobot, candidateId] = value;
      const canonicalTarget = canonicalRobotName(targetRobot);
      const result = this.interactionSpace.commit(canonicalTarget, String(candidateId));
      if (!result.success) {
        await this.addBelief2Agent("handoverPlanFailed", [result.reason || "candidate_commit_failed"], false, "ownLayer1");
        return true;
      }
      const session = await this.invokeLowerAction("beginHandoverSession", {
        session: result.id,
        targetRobot: canonicalTarget,
        cubeWorld: result.cubeWorld
      });
      if (!session.success) {
        await this.addBelief2Agent("handoverPlanFailed", [session.reason || "handover_session_rejected"], false, "ownLayer1");
        return true;
      }
      await this.addBelief2Agent(
        "handoverPlan",
        [
          asJasonAtom(result.id, "handover_session"),
          coordinate(result.cubeWorld, "x"), coordinate(result.cubeWorld, "y"), coordinate(result.cubeWorld, "z"),
          coordinate(result.giver?.localTcp, "x"), coordinate(result.giver?.localTcp, "y"), coordinate(result.giver?.localTcp, "z"),
          coordinate(result.receiver?.localTcp, "x"), coordinate(result.receiver?.localTcp, "y"), coordinate(result.receiver?.localTcp, "z")
        ],
        false,
        "ownLayer1"
      );
      return true;
    }

    if (keyword === "setHandoverApproach") {
      const [session, requestedEnabled] = value;
      const enabled = asBoolean(requestedEnabled);
      const result = await this.invokeLowerAction("setHandoverApproach", { session, enabled });
      if (!result.success) {
        await this.addBelief2Agent(
          "handoverApproachFailed",
          [session, enabled, result.reason || "handover_approach_failed"],
          false,
          "ownLayer1"
        );
        return true;
      }
      await this.addBelief2Agent("handoverApproachSet", [session, enabled], false, "ownLayer1");
      return true;
    }

    if (keyword === "getHandoverPolicy") {
      await this.addBelief2Agent("handoverReturnHome", [this.ctx.manifest.runtime?.returnHomeBetweenHandovers === true], false, "ownLayer1");
      return true;
    }

    if (keyword === "checkHandoverClearance") {
      const [session] = value;
      const result = await this.invokeLowerAction("checkHandoverClearance", { session });
      await this.addBelief2Agent("handoverClearance", [session, result.success === true && result.clear === true], false, "ownLayer1");
      return true;
    }

    if (keyword === "retreatFromHandover") {
      const [session] = value;
      const result = await this.enqueueMotion(() => this.invokeLowerAction("retreatFromHandover"), {
        action: "retreatFromHandover"
      });
      console.log(`[AgentThing:${this.ctx.manifest.name}] lower retreat result`, result);
      if (!result.success) {
        // Retreat may have moved before failing its endpoint check. Let the
        // handover plan recover without claiming clearance or retrying an
        // approach to a point based on the old rendezvous.
        await this.addBelief2Agent(
          "handoverRetreatFailed",
          [session, asJasonAtom(result.reason || "handover_retreat_failed", "handover_retreat_failed")],
          false,
          "ownLayer1"
        );
        return true;
      }
      await this.addBelief2Agent("handoverRetreated", [session], false, "ownLayer1");
      return true;
    }

    if (keyword === "setPickupApproach") {
      const enabled = asBoolean(value[0]);
      // The agent reasons with an atom; the scene knows an alias. The mapping
      // was recorded when the sites were detected, so a goal carrying the atom
      // can still name the object it stands for.
      const site = value.length > 1 ? this.taskSiteAliases?.get(String(value[1])) : null;
      const result = await this.invokeLowerAction(
        "setPickupApproach",
        site ? { enabled, site } : { enabled }
      );
      if (!result.success) {
        await this.addBelief2Agent(
          "pickupApproachFailed",
          [enabled, result.reason || "pickup_approach_failed"],
          false,
          "ownLayer1"
        );
        return true;
      }
      await this.addBelief2Agent("pickupApproachSet", [enabled], false, "ownLayer1");
      return true;
    }

    if (keyword === "transferCube") {
      const [session, targetRobot] = value;
      const canonicalTarget = canonicalRobotName(targetRobot);
      const result = await this.invokeLowerAction("transferCube", { session, targetRobot: canonicalTarget });
      if (!result.success) {
        await this.addBelief2Agent(
          "cubeTransferFailed",
          [session, result.reason || "cube_transfer_failed"],
          false,
          "ownLayer1"
        );
        return true;
      }
      await this.addBelief2Agent(
        "cubeTransferSucceeded",
        [session, result.sourceRobot || this.ctx.manifest.name, result.targetRobot || canonicalTarget],
        false,
        "ownLayer1"
      );
      return true;
    }

    if (keyword === "queryCubeOwnership") {
      const [queryId] = value;
      const result = await this.readLowerProperty("getCubeOwnership");
      await this.addBelief2Agent(
        "cubeOwnership",
        [queryId, result.owner || "none", Boolean(result.transferActive)],
        false,
        "ownLayer1"
      );
      return true;
    }

    if (keyword === "queryGripperState") {
      const [queryId] = value;
      const result = await this.readLowerProperty("getGripperState");
      await this.addBelief2Agent(
        "gripperState",
        [queryId, Boolean(result.closed), Boolean(result.holding), result.object || "none"],
        false,
        "ownLayer1"
      );
      return true;
    }

    return false;
  }

  // The lower layer names the object it hit and which robot owns it. Dropping
  // that on the floor is why a blocked handover could only be diagnosed by
  // reading CoppeliaSim's popup: the agent knew only "motion_failed". The
  // reason keeps its arity so existing .wait(motionFailed(_)) still matches,
  // and the detail arrives as its own belief.
  async publishMotionFailure(result, fallbackReason) {
    await this.addBelief2Agent(
      "motionFailed",
      [asJasonAtom(result.reason || fallbackReason, fallbackReason)],
      false,
      "ownLayer1"
    );
    if (result.obstacle) {
      console.warn(
        `[AgentThing:${this.ctx.manifest.name}] motion blocked by ${result.obstacle}`
        + `${result.obstacleOwner ? ` (owned by ${result.obstacleOwner})` : ""}`
      );
      await this.addBelief2Agent(
        "motionBlockedBy",
        [asJasonAtom(result.obstacle, "obstacle"), robotAtom(result.obstacleOwner, "unknown")],
        false,
        "ownLayer1"
      );
    }
  }

  async enqueueMotion(task, { action = "motion", payload } = {}) {
    // Competing for a room is normal with concurrent cores. Retry a refused
    // command before publishing failure; refusals have executed no motion.
    const attempt = async () => {
      const deadline = Date.now() + 45000;
      const prefix = `[AgentThing:${this.ctx.manifest.name}]`;
      console.log(`${prefix} invoking lower ${action}`, ...(payload === undefined ? [] : [payload]));
      let refusals = 0;
      let previousBlocker = null;
      while (true) {
        const result = await task();
        const busy = result?.success === false && ["region_claimed", "motion_busy"].includes(result.reason);
        if (busy) refusals++;
        if (!busy || Date.now() >= deadline) {
          if (refusals) console.log(`${prefix} ${action} ${busy ? "contention wait expired" : "request finished"}`
            + ` after ${refusals} contention refusal(s); no motion was executed by those refused attempts`);
          return result;
        }
        const blocker = {
          reason: result.reason,
          activeRobot: result.activeRobot || "unknown",
          conflictRegion: result.conflictRegion || result.conflictVoxel?.split(":")[0] || null
        };
        const signature = JSON.stringify(blocker);
        if (signature !== previousBlocker) {
          console.log(`${prefix} ${action} waiting for motion permission; retrying the same queued request`, blocker);
          previousBlocker = signature;
        }
        await new Promise((resolve) => setTimeout(resolve, 250));
      }
    };
    const run = this.motionQueue.then(attempt, attempt);
    this.motionQueue = run.catch(() => {});
    return run;
  }

  // Every HTTP action here is a request for the Jason agent to adopt a goal, and
  // the connector broadcasts to whatever websocket clients are attached. With no
  // agent attached that broadcast is a no-op over an empty list, so the action
  // returns 204 and absolutely nothing happens -- the single most confusing way
  // this bridge can fail, because the HTTP side looks perfect. It happens
  // whenever the MAS has not started yet, or the bridge was restarted after the
  // MAS connected and the dropped websocket was never re-established.
  forwardAchieveToAgent(keyword, values = null) {
    const attached = (this.agentConnector.clients || []).filter(
      (client) => client && client.readyState === 1
    ).length;
    if (attached === 0) {
      const message =
        `No Jason agent is attached to the websocket on port ${this.ctx.thingConfig.websocketPort}, `
        + `so !${keyword} cannot be delivered. Start the MAS with `
        + `./gradlew -Pjcm=${this.ctx.manifest.generatedFiles.jcm} and, if it was already running when `
        + `this bridge restarted, restart it so it reconnects.`;
      console.error(`[AgentThing:${this.ctx.manifest.name}] ${message}`);
      throw new Error(message);
    }
    console.log(
      `[AgentThing:${this.ctx.manifest.name}] HTTP ${keyword} requested; forwarding !${keyword} `
        + `to Jason agent (${attached} attached)`
        + (values ? ` ${JSON.stringify(values)}` : "")
    );
    this.agentConnector.send2Agent(new AgentMessage("achieve", "unknown", "", { keyword, values }));
  }

  async setCustomInteractionAffordanceHandlers(producer) {
    producer.setActionHandler("horizontalDemo", async () => {
      this.forwardAchieveToAgent("horizontalDemo");
    });

    producer.setActionHandler("moveTo", async (params) => {
      const payload = await params.value();
      this.forwardAchieveToAgent("moveTo", [Number(payload.x), Number(payload.y), Number(payload.z)]);
    });

    producer.setActionHandler("goHome", async () => {
      this.forwardAchieveToAgent("goHome");
    });

    producer.setActionHandler("pickCube", async () => {
      this.forwardAchieveToAgent("pickCube");
    });

    if (producer.getThingDescription().actions?.deliverCube) {
      producer.setActionHandler("deliverCube", async () => {
        this.forwardAchieveToAgent("deliverCube");
      });
    }

    producer.setActionHandler("closeGripper", async () => {
      this.forwardAchieveToAgent("closeGripper");
    });

    producer.setActionHandler("openGripper", async () => {
      this.forwardAchieveToAgent("openGripper");
    });

    if (producer.getThingDescription().actions?.handoverCube && this.interactionSpace.isOwner) {
      producer.setActionHandler("handoverCube", async (params) => {
        const payload = await params.value();
        this.forwardAchieveToAgent("startHandover", [String(payload.targetAgent), String(payload.targetRobot)]);
      });
    }

  }

  // Where both arms of a pair actually stand, asked of the robots themselves
  // rather than read out of the descriptor the cell was generated from. This is
  // what lets an agent notice that its neighbour has moved: the placements go
  // into the space's version hash, so a moved base invalidates the cached
  // shared space instead of being silently computed around.
  async readCellPlacements(targetRobot) {
    const placements = {};
    const own = await this.readBasePlacement(this.ctx.manifest.name, () => this.lowerThing);
    if (own) placements[this.ctx.manifest.name] = own;
    const partner = await this.readBasePlacement(
      targetRobot,
      () => this.partnerLowerThing(targetRobot)
    );
    if (partner) placements[targetRobot] = partner;
    return placements;
  }

  async readBasePlacement(robotName, resolveThing) {
    try {
      const thing = await resolveThing();
      if (!thing) return null;
      const pose = await this.unwrapOutput(await thing.readProperty("getBasePose"));
      const placement = placementFromBasePose(pose);
      if (!placement) return null;
      return placement;
    } catch (error) {
      // An older lower Thing has no getBasePose, and a neighbour may simply not
      // be served yet. Either way the descriptor placement still describes the
      // cell as generated, so fall back to it rather than refusing to compute.
      console.warn(
        `[AgentThing:${this.ctx.manifest.name}] no live base pose for ${robotName}, `
        + `using the descriptor placement:`,
        error?.message || error
      );
      return null;
    }
  }

  async partnerLowerThing(robotName) {
    if (this.partnerLowerThings.has(robotName)) {
      return this.partnerLowerThings.get(robotName);
    }
    const robot = this.interactionSpace.robot(robotName);
    if (!robot) return null;
    const url = this.interactionSpace.manifestFor(robot).lowerLayerUrl;
    if (!url) return null;
    const [, thing] = await this.fetchAgentTD(url);
    this.partnerLowerThings.set(robotName, thing);
    return thing;
  }

  // Every other robot the shared cell knows about. Neighbours are a conclusion,
  // not a configuration, so this deliberately starts from "all of them" rather
  // than from the configured handover pairs.
  cellRobots() {
    const descriptor = this.interactionSpace.descriptor;
    return (descriptor?.robots || [])
      .map((robot) => robot.name)
      .filter((name) => name && name !== this.ctx.manifest.name);
  }

  async readWorkspaceSignature(robotName) {
    try {
      if (robotName === this.ctx.manifest.name) {
        await this.ensureLowerThing();
        return unwrapArrayValue(await (await this.lowerThing.readProperty("getWorkspaceSignature")).value());
      }
      const partner = await this.partnerLowerThing(robotName);
      if (!partner) return null;
      return unwrapArrayValue(await (await partner.readProperty("getWorkspaceSignature")).value());
    } catch (error) {
      console.warn(
        `[AgentThing:${this.ctx.manifest.name}] could not read ${robotName}'s workspace signature: `
        + (error?.message || error)
      );
      return null;
    }
  }

  // Ask every arm in the cell where it stands and how far it reaches, then let
  // the agent believe which of them it could ever collide with.
  //
  // Both the inputs and the verdict are published. An agent that believes
  // "Robot3 is not my neighbour" and cannot say why has been told an answer
  // rather than given a reason, and the margin is the term that says how close
  // the call was.
  // What this arm concludes about the cell's cube supports by looking at the
  // scene, rather than by being told. Every site is reported, reachable or not:
  // an agent that cannot name a support it cannot serve has no way to answer a
  // neighbour asking whether it holds the goal.
  async detectTaskSites() {
    let result;
    if (this.mocked) {
      console.log(`[AgentThing:${this.ctx.manifest.name}] task sites from --mock-sites, no simulator consulted`);
      result = { sites: this.mockSites || [] };
    } else {
    try {
      result = await this.readLowerProperty("getTaskSites");
    } catch (error) {
      await this.addBelief2Agent(
        "taskSitesFailed",
        [asJasonAtom(error?.message || "task_site_read_failed", "task_site_read_failed")],
        false,
        "ownLayer1"
      );
      return;
    }
    }

    const sites = Array.isArray(result?.sites) ? result.sites : [];
    this.taskSites = sites;
    // The atom an agent reasons with is not the alias the scene knows. Keeping
    // the mapping here means a later goal carrying the atom can still be
    // resolved to the object it names.
    this.taskSiteAliases = new Map();
    let reachableCount = 0;
    for (const site of sites) {
      const atom = asJasonAtom(site.alias, "task_site");
      this.taskSiteAliases.set(atom, site.alias);
      const grasp = site.grasp || site.local;
      await this.addBelief2Agent(
        "site",
        [atom, coordinate(grasp, "x"), coordinate(grasp, "y"), coordinate(grasp, "z")],
        false,
        "ownLayer1"
      );
      if (site.reachable) {
        reachableCount += 1;
        await this.addBelief2Agent("siteReachable", [atom], false, "ownLayer1");
      }
      // "none" is how the property says "empty" -- a WoT DataSchema cannot
      // carry a null string, so absence arrives as a sentinel, not as nothing.
      const occupant = site.occupiedBy && site.occupiedBy !== "none" ? site.occupiedBy : null;
      if (occupant) {
        await this.addBelief2Agent(
          "siteOccupied",
          [atom, asJasonAtom(occupant, "payload")],
          false,
          "ownLayer1"
        );
      } else {
        await this.addBelief2Agent("siteEmpty", [atom], false, "ownLayer1");
      }
    }

    console.log(
      `[AgentThing:${this.ctx.manifest.name}] task sites: ${sites.length} seen, ${reachableCount} reachable`
      + `${sites.map((site) => ` | ${site.alias} ${site.reachable ? "reachable" : `out of reach (${site.admission})`}`
        + `${site.occupiedBy && site.occupiedBy !== "none" ? ` holding ${site.occupiedBy}` : " empty"}`).join("")}`
    );
    await this.addBelief2Agent("taskSitesReady", [sites.length, reachableCount], false, "ownLayer1");
  }

  // Where this arm's journey to a rendezvous actually starts.
  //
  // For an arm standing at A the answer is not its current pose: it is going to
  // pick the cube up first, so the leg it pays for is the one from the cube to
  // the handover point. Every other arm starts from where it is. Both are read,
  // never assumed, and which one applies is decided by whether this arm can
  // serve a support that currently holds the payload.
  async edgeCostOrigin(mode = "pickup") {
    // Only the arm that will actually pick the cube up pays from the cube.
    // Every other arm on the route RECEIVES it, and its leg starts wherever it
    // is standing when the question reaches it. Both arms that can reach the
    // loaded support have a pickup to offer; only one of them is going to use
    // it, and it is not the caller's reachability that decides which.
    if (mode === "pose") {
      const tcp = await this.readLowerProperty("getCartesianposition");
      return {
        kind: "tcp",
        point: { x: strictNumber(tcp?.x), y: strictNumber(tcp?.y), z: strictNumber(tcp?.z) },
        detail: "current_pose"
      };
    }
    const sites = this.taskSites || (await this.readTaskSites());
    const loaded = (sites || []).find((site) => site.reachable
      && site.occupiedBy && site.occupiedBy !== "none" && site.grasp);
    if (loaded) {
      return { kind: "pickup", point: loaded.grasp, detail: loaded.alias };
    }
    const tcp = await this.readLowerProperty("getCartesianposition");
    return {
      kind: "tcp",
      point: { x: strictNumber(tcp?.x), y: strictNumber(tcp?.y), z: strictNumber(tcp?.z) },
      detail: "current_pose"
    };
  }

  async readTaskSites() {
    const result = await this.readLowerProperty("getTaskSites");
    this.taskSites = Array.isArray(result?.sites) ? result.sites : [];
    return this.taskSites;
  }

  // What one handover edge costs this arm: the distance from where its journey
  // starts to where it must stand to hand over. The rendezvous is not chosen
  // here -- every viable candidate is priced and reported, and the agent picks
  // with the same weighting it commits a real handover with. Pricing a
  // candidate the handover would never select would be a cost for a rendezvous
  // that never happens.
  async estimateEdgeCost(targetRobot, minZ, maxZ, mode = "pickup") {
    const neighbourAtom = robotAtom(targetRobot);
    // The mode is part of every answer, not just of the question. An arm prices
    // the same edge twice -- once from the cube for its own search, once from
    // its pose to answer somebody else's -- and those runs overlap. Beliefs
    // keyed only by the neighbour let one run's abolish erase the other's
    // result mid-flight, and the arm then reports no route across an edge it
    // had just successfully priced.
    const modeAtom = asJasonAtom(mode, "pickup");
    // A priced edge logs a line and an unpriceable one used to log nothing, so
    // the only trace of a neighbour this arm cannot reach was a belief nobody
    // watching the bridge could see. An edge that cannot be priced is a result,
    // not an absence of one.
    const failed = async (reason) => {
      console.warn(
        `[AgentThing:${this.ctx.manifest.name}] edge to ${targetRobot} cannot be priced: ${reason}`
      );
      await this.addBelief2Agent(
        "edgeCostFailed",
        [neighbourAtom, modeAtom, asJasonAtom(reason, "edge_cost_failed")],
        false,
        "ownLayer1"
      );
    };

    if (this.mocked) {
      if (!this.mockCosts || !this.mockCosts.has(targetRobot)) {
        await failed("no_mock_cost_for_neighbour");
        return;
      }
      const cost = this.mockCosts.get(targetRobot);
      // One candidate with neutral scores: the agent's weighting still runs and
      // still chooses, so the protocol exercises the same path it will use for
      // real -- only the number it weighs is the test's rather than the scene's.
      await this.addBelief2Agent("edgeCostOrigin", [neighbourAtom, modeAtom, "mock", 0, 0, 0], false, "ownLayer1");
      await this.addBelief2Agent(
        "edgeCandidate",
        [neighbourAtom, modeAtom, "mock_candidate", cost, 1, 1, 1, 1, 1],
        false,
        "ownLayer1"
      );
      console.log(`[AgentThing:${this.ctx.manifest.name}] edge to ${targetRobot}: ${cost} m from --mock-costs`);
      await this.addBelief2Agent("edgeCostReady", [neighbourAtom, modeAtom, 1], false, "ownLayer1");
      return;
    }

    let origin;
    try {
      origin = await this.edgeCostOrigin(mode);
    } catch (error) {
      await failed(error?.message || "cost_origin_unreadable");
      return;
    }
    if (![origin.point.x, origin.point.y, origin.point.z].every(Number.isFinite)) {
      await failed("cost_origin_unreadable");
      return;
    }

    // Any arm can price any edge it touches. Which robot canonically OWNS a
    // pair decides who builds and caches its space, not who is allowed to ask
    // what it would cost them -- and rank() always treats the computing robot
    // as the giver, so the answer is this arm's own approach either way.
    let ranked = this.interactionSpace.rank(targetRobot, { minZ, maxZ });
    if (!ranked.success && ranked.reason === "interaction_space_not_ready") {
      try {
        const placements = await this.readCellPlacements(targetRobot);
        const built = this.interactionSpace.build(targetRobot, { placements, synthesizePair: true });
        if (!built.success) {
          await failed(built.reason || "interaction_space_estimation_failed");
          return;
        }
      } catch (error) {
        await failed("interaction_space_estimation_failed");
        return;
      }
      ranked = this.interactionSpace.rank(targetRobot, { minZ, maxZ });
    }
    if (!ranked.success) {
      // The reason atom is what the agent reasons with; the numbers behind it
      // are what a reader needs. A pair refused for being too small is the one
      // case where "how small, against what floor" is the whole story.
      if (ranked.reason === "interaction_space_below_minimum") {
        console.log(
          `[AgentThing:${this.ctx.manifest.name}] ${targetRobot} shares only `
          + `${ranked.sharedVoxelCount} voxel(s) with this arm, below the floor of `
          + `${ranked.minimumSharedVoxels}; that pair is not a rendezvous.`
        );
      }
      await failed(ranked.reason || "candidate_ranking_failed");
      return;
    }

    await this.addBelief2Agent(
      "edgeCostOrigin",
      [
        neighbourAtom,
        modeAtom,
        asJasonAtom(origin.kind, "tcp"),
        coordinate(origin.point, "x"), coordinate(origin.point, "y"), coordinate(origin.point, "z")
      ],
      false,
      "ownLayer1"
    );

    let cheapest = null;
    for (const candidate of ranked.candidates) {
      const distance = Math.hypot(
        coordinate(candidate.giver?.localTcp, "x") - origin.point.x,
        coordinate(candidate.giver?.localTcp, "y") - origin.point.y,
        coordinate(candidate.giver?.localTcp, "z") - origin.point.z
      );
      if (cheapest === null || distance < cheapest) cheapest = distance;
      await this.addBelief2Agent(
        "edgeCandidate",
        [
          neighbourAtom, modeAtom, asJasonAtom(candidate.id, "candidate"), distance,
          candidate.centrality, candidate.density,
          candidate.giverSupport, candidate.receiverSupport, candidate.posture
        ],
        false,
        "ownLayer1"
      );
    }

    console.log(
      `[AgentThing:${this.ctx.manifest.name}] edge to ${targetRobot}: ${ranked.candidates.length} rendezvous priced`
      + ` from ${origin.kind === "pickup" ? `the pickup at ${origin.detail}` : "the current pose"}`
      + ` (${origin.point.x.toFixed(3)}, ${origin.point.y.toFixed(3)}, ${origin.point.z.toFixed(3)})`
      + `${cheapest === null ? "" : `, nearest ${cheapest.toFixed(3)} m`}`
    );
    await this.addBelief2Agent(
      "edgeCostReady",
      [neighbourAtom, modeAtom, ranked.candidates.length],
      false,
      "ownLayer1"
    );
  }

  async discoverNeighbors() {
    const me = this.ctx.manifest.name;

    if (this.mocked) {
      this.mockNeighbors = this.mockNeighbors || [];
      console.log(`[AgentThing:${me}] neighbours from --mock-neighbors: ${this.mockNeighbors.join(", ") || "none"}`);
      for (const other of this.mockNeighbors) {
        const atom = robotAtom(other);
        await this.addBelief2Agent(
          "reachTest",
          [atom, 0, 0, 0, asJasonAtom("declared_for_test", "unknown")],
          false,
          "ownLayer1"
        );
        await this.addBelief2Agent("neighbor", [`${atom}_agent`, atom], false, "ownLayer1");
      }
      await this.addBelief2Agent(
        "workspaceReach",
        [robotAtom(me), 0, 0],
        false,
        "ownLayer1"
      );
      await this.addBelief2Agent(
        "neighborScanComplete",
        [this.mockNeighbors.length, this.mockNeighbors.length],
        false,
        "ownLayer1"
      );
      return { success: true, robot: me, mocked: true };
    }

    const mine = await this.readWorkspaceSignature(me);
    if (!mine) {
      await this.addBelief2Agent("neighborScanFailed", ["own_signature_unavailable"], false, "ownLayer1");
      return { success: false, reason: "own_signature_unavailable" };
    }

    await this.addBelief2Agent(
      "workspaceReach",
      [robotAtom(me), Number(mine.reachRadius.toFixed(4)), Number(mine.safeReachRadius.toFixed(4))],
      false,
      "ownLayer1"
    );

    // The agent's own cloud, as identity rather than payload: which file, which
    // version of it, how densely sampled. This is what it hands to whichever
    // robot owns a shared pair -- 35k points are not a belief and not a message,
    // and the digest is what lets both sides confirm they computed from the
    // same data without either of them sending it.
    if (mine.cloud) {
      await this.addBelief2Agent(
        "workspaceCloud",
        [
          robotAtom(me),
          `"${mine.cloud.path}"`,
          Number(mine.cloud.sampleCount),
          Number(Number(mine.cloud.spacing).toFixed(6)),
          asJasonAtom(mine.cloud.digest, "unknown")
        ],
        false,
        "ownLayer1"
      );
    }

    const results = [];
    for (const other of this.cellRobots()) {
      const theirs = await this.readWorkspaceSignature(other);
      const verdict = couldOverlap(mine, theirs);
      const atom = robotAtom(other);
      const round = (value) => (Number.isFinite(value) ? Number(value.toFixed(4)) : 0);

      // The evidence, then the conclusion drawn from it.
      await this.addBelief2Agent(
        "reachTest",
        [atom, round(verdict.distance), round(verdict.combinedReach), round(verdict.margin),
          asJasonAtom(verdict.reason, "unknown")],
        false,
        "ownLayer1"
      );
      if (verdict.neighbor) {
        await this.addBelief2Agent("neighbor", [`${atom}_agent`, atom], false, "ownLayer1");
      } else {
        await this.addBelief2Agent("notNeighbor", [atom, round(verdict.distance), round(verdict.combinedReach)], false, "ownLayer1");
      }
      results.push({ robot: other, ...verdict });
      console.log(
        `[AgentThing:${me}] ${other}: distance ${verdict.distance === null ? "?" : verdict.distance.toFixed(3)}m `
        + `vs combined reach ${verdict.combinedReach === null ? "?" : verdict.combinedReach.toFixed(3)}m `
        + `-> ${verdict.neighbor ? "NEIGHBOUR" : "not a neighbour"} (${verdict.reason})`
      );
    }

    await this.addBelief2Agent(
      "neighborScanComplete",
      [results.length, results.filter((result) => result.neighbor).length],
      false,
      "ownLayer1"
    );
    return { success: true, robot: me, results };
  }

  async redrawInteractionVisual(targetRobot) {
    const visual = this.interactionSpace.visualFor(targetRobot);
    if (!visual) return null;
    return this.invokeLowerAction("refreshInteractionVisual", visual);
  }

  async invokeLowerMove(payload) {
    try {
      const output = await this.lowerThing.invokeAction("moveTocartesianPosition", payload);
      return this.unwrapOutput(output);
    } catch (error) {
      const message = error?.message || String(error);
      console.error(`[AgentThing:${this.ctx.manifest.name}] lower Cartesian invocation failed:`, message);
      return { success: false, reason: "lower_http_error", message };
    }
  }

  async invokeHomeMotion(homeJoints = null) {
    try {
      const configuredHomeJoints = unwrapArrayValue(homeJoints !== null ? homeJoints : this.homeJoints);
      if (configuredHomeJoints !== null && configuredHomeJoints !== undefined) {
        const payload = {};
        const jointProperties = this.lowerTD?.actions?.moveTojointPosition?.input?.properties || {};
        const jointNames = Object.keys(jointProperties);
        jointNames.forEach((jointName, index) => {
          payload[jointName] = Number(configuredHomeJoints[index] || 0);
        });
        console.log(`[AgentThing:${this.ctx.manifest.name}] home joint payload`, payload);
        const output = await this.lowerThing.invokeAction("moveTojointPosition", payload);
        return this.unwrapOutput(output);
      }

      return this.invokeLowerMove(this.homePosition);
    } catch (error) {
      const message = error?.message || String(error);
      console.error(`[AgentThing:${this.ctx.manifest.name}] lower home invocation failed:`, message);
      return { success: false, reason: "lower_http_error", message };
    }
  }

  async invokeLowerGripper(actionName) {
    try {
      const output = await this.lowerThing.invokeAction(actionName);
      return this.unwrapOutput(output);
    } catch (error) {
      const message = error?.message || String(error);
      console.error(`[AgentThing:${this.ctx.manifest.name}] lower ${actionName} invocation failed:`, message);
      return { success: false, reason: "lower_http_error", message };
    }
  }

  async invokeLowerAction(actionName, payload) {
    try {
      const output = await this.lowerThing.invokeAction(actionName, payload);
      return this.unwrapOutput(output);
    } catch (error) {
      const message = error?.message || String(error);
      console.error(`[AgentThing:${this.ctx.manifest.name}] lower ${actionName} invocation failed:`, message);
      return { success: false, reason: "lower_http_error", message };
    }
  }

  async readLowerProperty(propertyName) {
    const output = await this.lowerThing.readProperty(propertyName);
    return this.unwrapOutput(output);
  }

  async unwrapOutput(output) {
    if (output && typeof output.value === "function") {
      const value = await output.value();
      return value || { success: true };
    }
    return output || { success: true };
  }
}

async function main() {
  const args = parseArgs(process.argv.slice(2));
  if (args.help) {
    console.log(usage());
    return;
  }
  if (!args.manifest) {
    fail("Missing --manifest.");
  }

  const ctx = createContext(args.manifest, {
    mockSites: parseMockSites(args["mock-sites"]),
    mockCosts: parseMockCosts(args["mock-costs"]),
    mockNeighbors: parseMockNeighbors(args["mock-neighbors"])
  });
  if (ctx.mockSites || ctx.mockCosts || ctx.mockNeighbors) {
    console.log(
      `[AgentThing:${ctx.manifest.name}] MOCK MODE -- `
      + `${ctx.mockSites ? `${ctx.mockSites.length} site(s) ` : ""}`
      + `${ctx.mockCosts ? `${ctx.mockCosts.size} edge cost(s) ` : ""}`
      + `${ctx.mockNeighbors ? `${ctx.mockNeighbors.length} neighbour(s) ` : ""}`
      + "from the command line. No simulator is consulted for these."
    );
  }
  await assertPortsAvailable(ctx);
  const robot = new GeneratedRobotAgentThing(ctx);
  await robot.init();
  console.log(`Generated AgentThing bridge running for ${ctx.manifest.name}.`);
  console.log(`Agent websocket port: ${ctx.thingConfig.websocketPort}`);
  console.log(`Agent TD port: ${ctx.thingConfig.TDPort}`);
  console.log(`Lower robot TD: ${ctx.lowerLayerUrl}`);
}

if (require.main === module) {
  main().catch((error) => {
    console.error(error);
    process.exit(1);
  });
}

module.exports = {
  GeneratedRobotAgentThing,
  parseMockCosts,
  parseMockNeighbors,
  parseMockSites,
  asJasonAtom,
  robotAtom,
  asBoolean,
  canonicalRobotName,
  coordinate,
  createContext,
  unwrapArrayValue
};
