#!/usr/bin/env node

const fs = require("fs");
const path = require("path");
const { analyzeUrdfFile } = require("./urdfStructure");

const LEGACY_ROBWOT_ROOT = "/home/maxchrs/Desktop/MA/RobWoT";

function defaultRobwotRoot() {
  const localSubmodule = path.resolve(__dirname, "..", "..", "external", "RobWoT");
  return fs.existsSync(localSubmodule) ? localSubmodule : LEGACY_ROBWOT_ROOT;
}

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
    "  npm run generate:robot -- --urdf <file.urdf> --name <robotName> [--thing-port 8090] [--agent-port 8091] [--websocket-port 8191] [--home-position x,y,z] [--horizontal-position x,y,z] [--grip-position x,y,z] [--carry-position x,y,z] [--home-joints j1,j2,...] [--locked-joints name1,name2] [--gripper-orientation-deg roll,pitch,yaw] [--gripper-local-rotation-deg roll,pitch,yaw] [--no-gripper-base-orientation-correction] [--gripper-tool-axis x,y,z] [--gripper-model-path <file.ttm>] [--gripper-pad-alias touch] [--gripper-tool-length 0.19] [--gripper-source model|builtin|none] [--gripper-source-alias <alias>] [--gripper-tcp-offset x,y,z] [--robwot-root external/RobWoT] [--skip-config-update]",
    "",
    "Mobile platform options:",
    "  [--platform auto|fixed|omnidirectional]  Platform type; auto classifies the URDF (default).",
    "  [--wheel-joints j1,j2,...]               Override wheel/caster joint names.",
    "  [--arm-joints j1,j2,...]                 Override arm chain joint names.",
    "  [--gripper-joints j1,j2,...]             Override gripper joint names.",
    "  [--base-home-pose x,y,theta]             Platform home pose in meters/degrees (default 0,0,0).",
    "  [--base-pick-pose x,y,theta]             Platform pose for the pick demo (default 0.9,0,0).",
    "  [--base-drop-pose x,y,theta]             Platform pose for the drop demo (default 0,0.9,90).",
    "",
    "Phase one creates WoMAT scaffolding only. CoppeliaSim model generation is reserved for phase two.",
    "The URDF is parsed to classify wheel, arm, and gripper joints; the result is stored as manifest.mobility."
  ].join("\n");
}

function fail(message) {
  console.error(message);
  console.error("");
  console.error(usage());
  process.exit(1);
}

function validateRobotName(name) {
  if (!name || !/^[A-Za-z][A-Za-z0-9_]*$/.test(name)) {
    fail("Robot name must start with a letter and contain only letters, numbers, and underscores.");
  }
}

function ensureDir(dir) {
  fs.mkdirSync(dir, { recursive: true });
}

function writeJson(file, value) {
  writeFile(file, JSON.stringify(value, null, 2) + "\n");
}

function writeFile(file, content) {
  ensureDir(path.dirname(file));
  fs.writeFileSync(file, content, "utf8");
}

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, "utf8"));
}

// A first generation has no manifest to read, and a malformed one is not a
// reason to refuse to regenerate the robot it belongs to.
function readJsonIfPresent(file) {
  if (!fs.existsSync(file)) {
    return null;
  }
  try {
    return readJson(file);
  } catch (error) {
    console.warn(`Ignoring unreadable manifest ${file}: ${error.message || error}`);
    return null;
  }
}

function posixPath(file) {
  return file.split(path.sep).join(path.posix.sep);
}

function agentThingTitle(name) {
  return `${name}_agent`;
}

function lowerThingTitle(name) {
  return `coppeliasim_virtualrobot_${name}`;
}

function lowerThingSlug(name) {
  return lowerThingTitle(name).toLowerCase();
}

function jcmAtom(name) {
  const atom = String(name)
    .replace(/[^A-Za-z0-9_]/g, "_")
    .replace(/^[^a-z]+/, (prefix) => prefix.toLowerCase());

  if (/^[a-z][A-Za-z0-9_]*$/.test(atom)) {
    return atom;
  }

  return `robot_${atom.toLowerCase()}`;
}

function agentAslFileName(name) {
  return `${jcmAtom(name)}_agent.asl`;
}

const { GRIPPER_SOURCES } = require("./core/luaDriver");

function parseNumberList(value, expectedLength, label) {
  if (!value) {
    return null;
  }

  const parsed = String(value)
    .split(",")
    .map((part) => Number(part.trim()));

  const hasExpectedLength = expectedLength === null || parsed.length === expectedLength;
  if (!hasExpectedLength || parsed.some((part) => Number.isNaN(part))) {
    const count = expectedLength === null ? "one or more" : expectedLength;
    fail(`${label} must contain ${count} comma-separated numbers.`);
  }

  return parsed;
}

function parseNameList(value) {
  if (!value) {
    return null;
  }
  return String(value)
    .split(",")
    .map((part) => part.trim())
    .filter((part) => part.length > 0);
}

function analyzeUrdfStructure(urdfPath, args) {
  const platform = args.platform || "auto";
  if (!["auto", "fixed", "omnidirectional"].includes(platform)) {
    fail("--platform must be one of: auto, fixed, omnidirectional.");
  }

  const overrides = {
    platform,
    wheelJoints: parseNameList(args["wheel-joints"]),
    armJoints: parseNameList(args["arm-joints"]),
    gripperJoints: parseNameList(args["gripper-joints"]),
    lockedJoints: parseNameList(args["locked-joints"])
  };

  try {
    const structure = analyzeUrdfFile(urdfPath, overrides);
    return { ...structure.mobility, warnings: structure.warnings };
  } catch (error) {
    const fallbackType = platform === "auto" ? "fixed" : platform;
    console.warn(`URDF structure analysis failed (${error.message}); assuming ${fallbackType} platform.`);
    return {
      type: fallbackType,
      baseLink: null,
      wheelJoints: overrides.wheelJoints || [],
      armJoints: overrides.armJoints || [],
      jointChain: overrides.armJoints || [],
      lockedJoints: overrides.lockedJoints || [],
      lockedJointLimits: {},
      armRootLink: null,
      tipLink: null,
      gripperJoints: overrides.gripperJoints || [],
      armJointLimits: {},
      warnings: [`URDF structure analysis failed: ${error.message}`]
    };
  }
}

function createAgentTd(name) {
  return {
    title: agentThingTitle(name),
    description: `Generated WoMAT AgentThing for ${name}.`,
    "@context": [
      "https://www.w3.org/2022/wot/td/v1.1",
      { saref: "https://w3id.org/saref#" }
    ],
    securityDefinitions: {
      no_sec: {
        scheme: "nosec"
      }
    },
    security: "no_sec",
    properties: {
      state: {
        type: "string",
        "@type": "testOntology:State",
        description: "The status of the generated robot agent.",
        enum: ["idle", "moving", "holdingObject", "broken"],
        forms: [
          {
            href: "/properties/state",
            "htv:methodName": "GET"
          }
        ]
      }
    },
    actions: {
      goHome: {
        description: "Moves the robot to its configured home position.",
        "@type": "testOntology:GoHome",
        forms: [
          {
            href: "/actions/goHome",
            "htv:methodName": "POST"
          }
        ]
      },
      moveTo: {
        description: "Moves the robot to a cartesian position.",
        "@type": "testOntology:MoveToPosition",
        input: {
          type: "object",
          properties: {
            x: { type: "number", "@type": "testOntology:PositionX" },
            y: { type: "number", "@type": "testOntology:PositionY" },
            z: { type: "number", "@type": "testOntology:PositionZ" }
          },
          required: ["x", "y", "z"]
        },
        forms: [
          {
            href: "/actions/moveTo",
            contentType: "application/json",
            "htv:methodName": "POST"
          }
        ]
      },
      horizontalDemo: {
        description: "Moves the robot to its generated horizontal demo target and then back home.",
        "@type": "testOntology:HorizontalDemo",
        forms: [
          {
            href: "/actions/horizontalDemo",
            "htv:methodName": "POST"
          }
        ]
      },
      pickCube: {
        description: "Picks the generated cube: hovers above it with the tool pointing down, descends, grips, and lifts to the configured carry pose still holding it.",
        "@type": "testOntology:PickCube",
        forms: [
          {
            href: "/actions/pickCube",
            "htv:methodName": "POST"
          }
        ]
      },
      deliverCube: {
        description: "Send the cube to the goal support along the route this cell discovered for itself. Each arm follows its own next hop, so one call to whichever arm holds the cube moves it the whole way.",
        forms: [
          {
            href: "/actions/deliverCube",
            "htv:methodName": "POST"
          }
        ]
      },
      closeGripper: {
        description: "Close the generated virtual gripper.",
        "@type": "testOntology:CloseGripper",
        forms: [
          {
            href: "/actions/closeGripper",
            "htv:methodName": "POST"
          }
        ]
      },
      openGripper: {
        description: "Open the generated virtual gripper.",
        "@type": "testOntology:OpenGripper",
        forms: [
          {
            href: "/actions/openGripper",
            "htv:methodName": "POST"
          }
        ]
      },
      handoverCube: {
        description: "Start a coordinated BDI cube handover to another robot agent.",
        input: {
          type: "object",
          properties: {
            targetAgent: { type: "string" },
            targetRobot: { type: "string" }
          },
          required: ["targetAgent", "targetRobot"]
        },
        forms: [
          {
            href: "/actions/handoverCube",
            contentType: "application/json",
            "htv:methodName": "POST"
          }
        ]
      },
      askHow: {
        description: "Ask the agent for exchangeable plans.",
        input: {
          type: "object",
          properties: {
            planLabel: { type: "string" },
            targetTDAdress: { type: "string" }
          },
          required: ["planLabel", "targetTDAdress"]
        },
        output: {
          type: "array",
          items: { type: "object" }
        },
        forms: [
          {
            href: "/actions/askPlan",
            contentType: "application/json"
          }
        ]
      },
      tell: {
        description: "Tell the agent a belief.",
        input: {
          type: "object",
          properties: {
            belief: { type: "string" },
            values: {
              type: "array",
              items: {
                oneOf: [
                  { type: "string" },
                  { type: "number" },
                  { type: "boolean" }
                ]
              }
            }
          },
          required: ["belief"],
          additionalProperties: true
        },
        forms: [
          {
            href: "/actions/tell",
            contentType: "application/json"
          }
        ]
      },
      send: {
        description: "Send a message to the agent.",
        input: { type: "object" },
        forms: [
          {
            href: "/actions/send",
            contentType: "application/json"
          }
        ]
      }
    }
  };
}

function createMobileAgentTd(name) {
  const td = createAgentTd(name);
  td.description = `Generated WoMAT AgentThing for the mobile platform ${name}.`;
  td.properties.state.enum = ["idle", "navigating", "moving", "holdingObject", "broken"];

  // The platform navigates in the plane; the arm works in cartesian space
  // relative to the platform. Demos are replaced by a transport demo.
  delete td.actions.moveTo;
  delete td.actions.horizontalDemo;
  delete td.actions.pickCube;
  delete td.actions.handoverCube;

  td.actions.goTo = {
    description: "Drives the platform to a pose in the plane. theta is the heading in degrees.",
    "@type": "testOntology:GoToPose",
    input: {
      type: "object",
      properties: {
        x: { type: "number", unit: "meter", "@type": "testOntology:PositionX" },
        y: { type: "number", unit: "meter", "@type": "testOntology:PositionY" },
        theta: { type: "number", unit: "deg", "@type": "testOntology:Heading" }
      },
      required: ["x", "y"]
    },
    forms: [
      {
        href: "/actions/goTo",
        contentType: "application/json",
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.moveArmTo = {
    description: "Moves the arm end-effector to a cartesian position relative to the platform base.",
    "@type": "testOntology:MoveArmToPosition",
    input: {
      type: "object",
      properties: {
        x: { type: "number", unit: "meter", "@type": "testOntology:PositionX" },
        y: { type: "number", unit: "meter", "@type": "testOntology:PositionY" },
        z: { type: "number", unit: "meter", "@type": "testOntology:PositionZ" }
      },
      required: ["x", "y", "z"]
    },
    forms: [
      {
        href: "/actions/moveArmTo",
        contentType: "application/json",
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.pick = {
    description: "Drives to the configured pick pose, moves the arm to the grip position, closes the gripper, and lifts to the carry position.",
    "@type": "testOntology:Pick",
    forms: [
      {
        href: "/actions/pick",
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.drop = {
    description: "Drives to the configured drop pose, lowers the arm, opens the gripper, and retracts to the carry position.",
    "@type": "testOntology:Drop",
    forms: [
      {
        href: "/actions/drop",
        "htv:methodName": "POST"
      }
    ]
  };

  td.actions.transportDemo = {
    description: "Full mobile manipulation demo: pick at the pick pose, drop at the drop pose, return to the home pose.",
    "@type": "testOntology:TransportDemo",
    forms: [
      {
        href: "/actions/transportDemo",
        "htv:methodName": "POST"
      }
    ]
  };

  return td;
}

function createAgentAsl(name, defaults) {
  const homePosition = defaults.homePosition;
  const horizontalPosition = defaults.horizontalPosition;
  const gripPosition = defaults.gripPosition;
  const carryPosition = defaults.carryPosition;
  const homeJoints = defaults.homeJoints;
  // The same height the lower Thing hovers at. Taken from the manifest rather
  // than restated, because the agent retreats to this point after the grasp and
  // a disagreement would put the retreat somewhere the descent never cleared.
  const pickupHoverHeight = defaults.pickupApproachHeight;
  // How far the arm lifts straight up once it has hold of the cube. Separate
  // from the approach height on purpose: the approach height is a property of
  // the PLAN (how far above the cube the hover pose sits, and how far the
  // descent has to travel), while this is just "get clear before moving
  // laterally" and wants to be as short as does that job.
  const pickLiftHeight = defaults.pickLiftHeight ?? 0.10;
  const agentName = `${jcmAtom(name)}_agent`;
  const homeJointBelief = homeJoints ? `\nhomeJoints(array(${homeJoints.join(", ")})).\n` : "";

  return `// Generated by src/generator/generateRobot.js
// Phase one scaffold for ${name}.

homePosition(${homePosition.join(", ")}).
horizontalDemoPosition(${horizontalPosition.join(", ")}).${homeJointBelief}
gripTargetPosition(${gripPosition.join(", ")}).
pickupHoverHeight(${pickupHoverHeight}).
pickLiftHeight(${pickLiftHeight}).
carryPosition(${carryPosition.join(", ")}).
agentIdentity(${agentName}, ${jcmAtom(name)}).
// The band of world heights a rendezvous may sit in, applied to every
// neighbour this arm prices. It is a preference, not a reach limit, so it has
// to be low enough for the SHORTEST arm in the cell: a band chosen around a
// UR10's comfortable working height leaves a small arm with no shared voxel in
// it at all, and the pricing then reports no_height_filtered_interaction_voxel
// for a pair that shares plenty of space -- just none of it that high. Live
// cell: Robot3 and Robot4 share 17 voxels spanning -0.04..0.28, so the old
// [0.30, 0.65] band emptied the pair and broke the chain at its last leg. The
// floor clears the 0.16 m task-site benches; the ceiling stays under the point
// where the taller arms meet at the far edge of their reach.
handoverHeightRange(0.15, 0.45).

+addAchieveGoal2Agent(TermString): true <-
    .print("Received external achieve request: ", TermString);
    .term2string(X, TermString);
    !X.

!init.

+!init: true <-
    .print("Initializing ${name} agent");
    +myStatus(idle);
    // Who this arm could ever collide with is a conclusion it draws for itself
    // from where every arm stands and how far each can reach, before it is
    // asked to do anything. Configured pair topology, if any, is separate.
    !discoverNeighbors;
    // Neighbours first, then the shared spaces with them -- and only with them.
    // Which pairs this agent computes follows from that scan, not from a list.
    !shareInteractionSpaces;
    // What the cell is actually FOR: which supports stand in it, which of them
    // this arm can serve, and which one holds the cube. Read from the scene at
    // startup, so moving the cube to another arm's bench changes who believes
    // it is at the start of the transfer without regenerating anything.
    !detectTaskSites;
    // An arm that finds itself at the loaded support prices the ways out of it
    // straight away. Every other arm has nothing to price yet: what a carry
    // costs depends on where the cube is coming from, and nobody has said.
    !reportHandoverCosts;
    // Nobody holds a map. The arm with the cube asks its neighbours who can
    // reach the goal, they ask theirs, and the answer comes back with a cost
    // accumulated one leg per arm along the way.
    !discoverRoute;
    .print("${name} agent ready").

+!horizontalDemo: horizontalDemoPosition(X, Y, Z) <-
    .print("Starting horizontal demo motion for ${name}");
    !moveTo(X, Y, Z);
    .wait(500);
    !goHome;
    .print("Horizontal demo motion finished for ${name}").

// The approach, in one place. setPickupApproach arms the lower Thing's
// top-down planner, which is what turns the following !moveTo from a general
// Cartesian move into a pick: it solves a hover pose pickupHoverHeight above
// the cube with the tool axis pinned straight down, then descends along a
// pre-validated vertical path. Every caller goes through this, so a handover
// picks the cube exactly the way a standalone !pickCube does.
+!graspCube: gripTargetPosition(X, Y, Z) & pickLiftHeight(L) <-
    !setPickupApproach(true);
    !moveTo(X, Y, Z);
    !closeGripper;
    !setPickupApproach(false);
    // Straight up before anything lateral, and part of the GRASP rather than of
    // whatever follows it. The pick is what puts the arm in the hazard -- jaws
    // closed on a cube still sitting in its support, the support a hard obstacle
    // again the moment the flag is cleared -- so the pick is what gets it out.
    // Leaving the lift to the caller meant a standalone pick cleared the support
    // and a handover giver did not: it went straight from the grasp to the
    // rendezvous, dragging cube and fingers across the block it had just lifted
    // from.
    LiftZ = Z + L;
    !moveTo(X, Y, LiftZ).

// While the flag is armed every Cartesian move is routed through the pick
// planner, and the grip support is dropped from the obstacle set. Leaving it
// set after a failure silently changes what the next move means, so it is
// disarmed before the failure propagates.
-!graspCube: true <-
    .print("Grasp failed for ${name}; disarming pickup approach");
    !setPickupApproach(false);
    .fail.

+!pickCube: carryPosition(CX, CY, CZ) <-
    .print("Picking cube with ${name}");
    // !graspCube has already lifted clear of the support, so the carry move is
    // a lateral move from a safe height rather than a diagonal out of the jaws
    // of one.
    !graspCube;
    !moveTo(CX, CY, CZ);
    .print("${name} holding cube at carry position").

+!moveTo(X, Y, Z): true <-
    .abolish(movedTo(_, _, _));
    .abolish(movedTo(_));
    .abolish(motionFailed(_));
    -myStatus(idle);
    +myStatus(moving);
    wotAchieve("ownLayer1", "moveTo", [X, Y, Z]);
    .wait(movedTo(X, Y, Z) | motionFailed(_), 60000);
    if (motionFailed(Reason)) {
        .print("Move failed for ${name}: ", Reason);
        -myStatus(moving);
        +myStatus(broken);
        .fail
    };
    -myStatus(moving);
    +myStatus(idle).

+!closeGripper: true <-
    .abolish(gripFailed(_));
    .abolish(grippedObject(_));
    wotAchieve("ownLayer1", "closeGripper", []);
    .wait(grippedObject(_) | gripFailed(_), 15000);
    if (gripFailed(Reason)) {
        .print("Close gripper failed for ${name}: ", Reason);
        .fail
    };
    -myStatus(idle);
    +myStatus(holdingObject).

+!openGripper: true <-
    .abolish(gripFailed(_));
    .abolish(releasedObject(_));
    wotAchieve("ownLayer1", "openGripper", []);
    .wait(releasedObject(_) | gripFailed(_), 15000);
    if (gripFailed(Reason)) {
        .print("Open gripper failed for ${name}: ", Reason);
        .fail
    };
    -myStatus(holdingObject);
    +myStatus(idle).

+!goHome: homeJoints(HomeJoints) <-
    .abolish(movedTo(_));
    .abolish(motionFailed(_));
    -myStatus(idle);
    +myStatus(moving);
    wotAchieve("ownLayer1", "goHomeJoints", HomeJoints);
    .wait(movedTo(home) | motionFailed(_), 60000);
    if (motionFailed(Reason)) {
        .print("Home motion failed for ${name}: ", Reason);
        -myStatus(moving);
        +myStatus(broken);
        .fail
    };
    -myStatus(moving);
    +myStatus(idle).

+!goHome: homePosition(X, Y, Z) <-
    .abolish(movedTo(_));
    .abolish(motionFailed(_));
    -myStatus(idle);
    +myStatus(moving);
    wotAchieve("ownLayer1", "goHome", []);
    .wait(movedTo(home) | motionFailed(_), 60000);
    if (motionFailed(Reason)) {
        .print("Home motion failed for ${name}: ", Reason);
        -myStatus(moving);
        +myStatus(broken);
        .fail
    };
    -myStatus(moving);
    +myStatus(idle).

+?state(MessageID): myStatus(Status) <-
    wotSendTestResponse(Status, MessageID).

@[wotExchangable]
+!goTo(X, Y, Z): true <-
    !moveTo(X, Y, Z).

@[wotExchangable]
+!returnHome: true <-
    !goHome.

@[wotExchangable]
+!gripObject: true <-
    !closeGripper.

@[wotExchangable]
+!releaseObject: true <-
    !openGripper.

{ include("src/agt/wotPlans.asl") }
{ include("src/agt/neighborPlans.asl") }
{ include("src/agt/routePlans.asl") }
{ include("src/agt/handoverPlans.asl") }
{ include("$jacamo/templates/common-cartago.asl") }
`;
}

function createMobileAgentAsl(name, defaults) {
  const baseHomePose = defaults.baseHomePose;
  const basePickPose = defaults.basePickPose;
  const baseDropPose = defaults.baseDropPose;
  const gripPosition = defaults.gripPosition;
  const carryPosition = defaults.carryPosition;
  const homeJoints = defaults.homeJoints;
  const homeJointBelief = homeJoints ? `\nhomeJoints(array(${homeJoints.join(", ")})).\n` : "";

  return `// Generated by src/generator/generateRobot.js
// Phase one scaffold for the mobile platform ${name}.
// Base poses are (x meter, y meter, theta degree); arm positions are cartesian
// (x, y, z meter) relative to the platform base.

baseHomePose(${baseHomePose.join(", ")}).
basePickPose(${basePickPose.join(", ")}).
baseDropPose(${baseDropPose.join(", ")}).${homeJointBelief}
armGripPosition(${gripPosition.join(", ")}).
armCarryPosition(${carryPosition.join(", ")}).

+addAchieveGoal2Agent(TermString): true <-
    .print("Received external achieve request: ", TermString);
    .term2string(X, TermString);
    !X.

!init.

+!init: true <-
    .print("Initializing ${name} mobile platform agent");
    +myStatus(idle);
    .wait(1500);
    !goHome;
    !navDemo.

+!navDemo: basePickPose(X, Y, Theta) <-
    .print("Starting navigation demo for ${name}");
    !goTo(X, Y, Theta);
    .wait(500);
    !goHomeBase;
    .print("Navigation demo finished for ${name}").

+!transportDemo: true <-
    .print("Starting transport demo for ${name}");
    !pick;
    !drop;
    !goHomeBase;
    !goHome;
    .print("Transport demo finished for ${name}").

+!pick: basePickPose(BX, BY, BTheta) & armGripPosition(X, Y, Z) & armCarryPosition(CX, CY, CZ) <-
    !goTo(BX, BY, BTheta);
    !moveArmTo(X, Y, Z);
    !closeGripper;
    .wait(500);
    !moveArmTo(CX, CY, CZ).

+!drop: baseDropPose(BX, BY, BTheta) & armGripPosition(X, Y, Z) & armCarryPosition(CX, CY, CZ) <-
    !goTo(BX, BY, BTheta);
    !moveArmTo(X, Y, Z);
    !openGripper;
    .wait(500);
    !moveArmTo(CX, CY, CZ).

+!goTo(X, Y, Theta): true <-
    .abolish(arrivedAt(_, _, _));
    .abolish(motionFailed(_));
    -myStatus(idle);
    +myStatus(navigating);
    wotAchieve("ownLayer1", "goTo", [X, Y, Theta]);
    .wait(6000);
    if (motionFailed(Reason)) {
        .print("Navigation failed for ${name}: ", Reason);
        -myStatus(navigating);
        +myStatus(broken);
        .fail
    };
    -myStatus(navigating);
    +myStatus(idle).

+!goHomeBase: baseHomePose(X, Y, Theta) <-
    !goTo(X, Y, Theta).

+!moveArmTo(X, Y, Z): true <-
    .abolish(movedTo(_, _, _));
    .abolish(movedTo(_));
    .abolish(motionFailed(_));
    -myStatus(idle);
    +myStatus(moving);
    wotAchieve("ownLayer1", "moveArmTo", [X, Y, Z]);
    .wait(3500);
    if (motionFailed(Reason)) {
        .print("Arm motion failed for ${name}: ", Reason);
        -myStatus(moving);
        +myStatus(broken);
        .fail
    };
    -myStatus(moving);
    +myStatus(idle).

+!closeGripper: true <-
    .abolish(gripFailed(_));
    .abolish(grippedObject(_));
    wotAchieve("ownLayer1", "closeGripper", []);
    .wait(1000);
    if (gripFailed(Reason)) {
        .print("Close gripper failed for ${name}: ", Reason);
        .fail
    };
    -myStatus(idle);
    +myStatus(holdingObject).

+!openGripper: true <-
    .abolish(gripFailed(_));
    wotAchieve("ownLayer1", "openGripper", []);
    .wait(1000);
    if (gripFailed(Reason)) {
        .print("Open gripper failed for ${name}: ", Reason);
        .fail
    };
    -myStatus(holdingObject);
    +myStatus(idle).

+!goHome: homeJoints(HomeJoints) <-
    .abolish(movedTo(_));
    .abolish(motionFailed(_));
    -myStatus(idle);
    +myStatus(moving);
    wotAchieve("ownLayer1", "goHomeJoints", HomeJoints);
    .wait(3500);
    if (motionFailed(Reason)) {
        .print("Arm home motion failed for ${name}: ", Reason);
        -myStatus(moving);
        +myStatus(broken);
        .fail
    };
    -myStatus(moving);
    +myStatus(idle).

+!goHome: armCarryPosition(X, Y, Z) <-
    !moveArmTo(X, Y, Z).

-!goTo(X, Y, Theta): motionFailed(Reason) <-
    .print("Navigation failed for ${name}: ", Reason);
    -myStatus(navigating);
    +myStatus(broken).

-!moveArmTo(X, Y, Z): motionFailed(Reason) <-
    .print("Arm motion failed for ${name}: ", Reason);
    -myStatus(moving);
    +myStatus(broken).

+?state(MessageID): myStatus(Status) <-
    wotSendTestResponse(Status, MessageID).

@[wotExchangable]
+!driveTo(X, Y, Theta): true <-
    !goTo(X, Y, Theta).

@[wotExchangable]
+!returnHome: true <-
    !goHomeBase.

@[wotExchangable]
+!transport: true <-
    !transportDemo.

@[wotExchangable]
+!gripObject: true <-
    !closeGripper.

@[wotExchangable]
+!releaseObject: true <-
    !openGripper.

{ include("../wotPlans.asl") }
{ include("$jacamo/templates/common-cartago.asl") }
`;
}

function createAgentThingTs(name, configKey, tdPath, lowerLayerUrl, defaults) {
  const homePosition = defaults.homePosition;
  const homeJoints = defaults.homeJoints;

  return `import { AgentThing } from "../agentThing";
import { AgentMessage } from "../agentConnector";

class GeneratedRobotAgentThing extends AgentThing {
    private lowerThing: any = null;
    private lowerTD: any = null;
    private lowerLayerAddress: string;
    public homePosition = { x: ${homePosition[0]}, y: ${homePosition[1]}, z: ${homePosition[2]} };
    public homeJoints: any = ${homeJoints ? JSON.stringify(homeJoints) : "null"};

    constructor(config: any, path2TD: string, initialKnownAgents: { [key: string]: string }, lowerLayerAddress: string, additionalOntology: any = {}) {
        super(config, path2TD, initialKnownAgents, additionalOntology);
        this.lowerLayerAddress = lowerLayerAddress;
    }

    protected async customInit(): Promise<boolean> {
        try {
            [this.lowerTD, this.lowerThing] = await this.fetchAgentTD(this.lowerLayerAddress);
        } catch (error) {
            console.error("Failed to connect to generated lower-level Thing. Start it before this AgentThing.", error);
            return false;
        }

        await this.invokeHomeMotion();
        return true;
    }

    protected async handleCustomAgentRequests(data: AgentMessage): Promise<boolean> {
        const targetAgentId = data.targetAgentId;
        const content = data.content as any;
        const keyword = content.keyword;
        const value = content?.value ?? [];

        if (targetAgentId !== "ownLayer1") {
            return false;
        }

        if (keyword === "moveTo" || keyword === "moveAt") {
            const payload = { x: value[0], y: value[1], z: value[2] };
            const result = await this.invokeLowerMove(payload);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "move_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("movedTo", value, false, "ownLayer1");
            return true;
        }

        if (keyword === "goHome" || keyword === "homeing" || keyword === "goHomeJoints") {
            const requestedHomeJoints = keyword === "goHomeJoints" ? value : null;
            const result = await this.invokeHomeMotion(requestedHomeJoints);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "home_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("movedTo", ["home"], false, "ownLayer1");
            return true;
        }

        if (keyword === "horizontalDemo") {
            await this.addAchieveGoal2Agent("horizontalDemo", null, false, "unknown");
            return true;
        }

        return false;
    }

    private async invokeLowerMove(payload: { x: number; y: number; z: number }) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const output = await this.lowerThing.invokeAction("moveTocartesianPosition", payload);
        return this.unwrapOutput(output);
    }

    private async invokeHomeMotion(homeJoints: any = null) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const configuredHomeJoints = homeJoints !== null ? homeJoints : this.homeJoints;
        if (configuredHomeJoints !== null) {
            const joints = Array.isArray(configuredHomeJoints) ? configuredHomeJoints : configuredHomeJoints?.value ?? configuredHomeJoints;
            const payload: any = {};
            const jointNames = Object.keys(this.lowerTD?.actions?.moveTojointPosition?.input?.properties || {});
            jointNames.forEach((jointName, index) => {
                payload[jointName] = Number(joints[index] || 0);
            });
            const output = await this.lowerThing.invokeAction("moveTojointPosition", payload);
            return this.unwrapOutput(output);
        }

        return this.invokeLowerMove(this.homePosition);
    }

    private async unwrapOutput(output: any) {
        if (output && typeof output.value === "function") {
            const value = await output.value();
            return value || { success: true };
        }
        return output || { success: true };
    }
}

async function main() {
    const config = require("./../../../config.json");
    const ownTD = "./../../src/td/generated/${path.basename(tdPath)}";
    const lowerLayerAddress = "${lowerLayerUrl}";
    const initialKnownAgents = {};
    const additionalOntology = {};

    const robot = new GeneratedRobotAgentThing(config[${JSON.stringify(configKey)}], ownTD, initialKnownAgents, lowerLayerAddress, additionalOntology);
    await robot.init();
}

main();

export {};
`;
}

function createMobileAgentThingTs(name, configKey, tdPath, lowerLayerUrl, defaults) {
  const baseHomePose = defaults.baseHomePose;
  const carryPosition = defaults.carryPosition;
  const homeJoints = defaults.homeJoints;

  return `import { AgentThing } from "../agentThing";
import { AgentMessage } from "../agentConnector";

class GeneratedMobileRobotAgentThing extends AgentThing {
    private lowerThing: any = null;
    private lowerTD: any = null;
    private lowerLayerAddress: string;
    public baseHomePose = { x: ${baseHomePose[0]}, y: ${baseHomePose[1]}, theta: ${baseHomePose[2]} };
    public armHomePosition = { x: ${carryPosition[0]}, y: ${carryPosition[1]}, z: ${carryPosition[2]} };
    public homeJoints: any = ${homeJoints ? JSON.stringify(homeJoints) : "null"};

    constructor(config: any, path2TD: string, initialKnownAgents: { [key: string]: string }, lowerLayerAddress: string, additionalOntology: any = {}) {
        super(config, path2TD, initialKnownAgents, additionalOntology);
        this.lowerLayerAddress = lowerLayerAddress;
    }

    protected async customInit(): Promise<boolean> {
        try {
            [this.lowerTD, this.lowerThing] = await this.fetchAgentTD(this.lowerLayerAddress);
        } catch (error) {
            console.error("Failed to connect to generated lower-level Thing. Start it before this AgentThing.", error);
            return false;
        }

        await this.invokeArmHomeMotion();
        return true;
    }

    protected async handleCustomAgentRequests(data: AgentMessage): Promise<boolean> {
        const targetAgentId = data.targetAgentId;
        const content = data.content as any;
        const keyword = content.keyword;
        const value = content?.value ?? [];

        if (targetAgentId !== "ownLayer1") {
            return false;
        }

        if (keyword === "goTo") {
            const payload = { x: value[0], y: value[1], theta: value[2] ?? 0 };
            const result = await this.invokeBaseMove(payload);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "navigation_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("arrivedAt", value, false, "ownLayer1");
            return true;
        }

        if (keyword === "moveArmTo" || keyword === "moveTo" || keyword === "moveAt") {
            const payload = { x: value[0], y: value[1], z: value[2] };
            const result = await this.invokeArmMove(payload);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "arm_move_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("movedTo", value, false, "ownLayer1");
            return true;
        }

        if (keyword === "goHome" || keyword === "homeing" || keyword === "goHomeJoints") {
            const requestedHomeJoints = keyword === "goHomeJoints" ? value : null;
            const result = await this.invokeArmHomeMotion(requestedHomeJoints);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "home_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("movedTo", ["home"], false, "ownLayer1");
            return true;
        }

        if (keyword === "goHomeBase") {
            const result = await this.invokeBaseMove(this.baseHomePose);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "navigation_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("arrivedAt", [this.baseHomePose.x, this.baseHomePose.y, this.baseHomePose.theta], false, "ownLayer1");
            return true;
        }

        return false;
    }

    private async invokeBaseMove(payload: { x: number; y: number; theta: number }) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const output = await this.lowerThing.invokeAction("moveTo", payload);
        return this.unwrapOutput(output);
    }

    private async invokeArmMove(payload: { x: number; y: number; z: number }) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const output = await this.lowerThing.invokeAction("moveArmTo", payload);
        return this.unwrapOutput(output);
    }

    private async invokeArmHomeMotion(homeJoints: any = null) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const configuredHomeJoints = homeJoints !== null ? homeJoints : this.homeJoints;
        if (configuredHomeJoints !== null) {
            const joints = Array.isArray(configuredHomeJoints) ? configuredHomeJoints : configuredHomeJoints?.value ?? configuredHomeJoints;
            const payload: any = {};
            const jointNames = Object.keys(this.lowerTD?.actions?.moveTojointPosition?.input?.properties || {});
            jointNames.forEach((jointName, index) => {
                payload[jointName] = Number(joints[index] || 0);
            });
            const output = await this.lowerThing.invokeAction("moveTojointPosition", payload);
            return this.unwrapOutput(output);
        }

        return this.invokeArmMove(this.armHomePosition);
    }

    private async unwrapOutput(output: any) {
        if (output && typeof output.value === "function") {
            const value = await output.value();
            return value || { success: true };
        }
        return output || { success: true };
    }
}

async function main() {
    const config = require("./../../../config.json");
    const ownTD = "./../../src/td/generated/${path.basename(tdPath)}";
    const lowerLayerAddress = "${lowerLayerUrl}";
    const initialKnownAgents = {};
    const additionalOntology = {};

    const robot = new GeneratedMobileRobotAgentThing(config[${JSON.stringify(configKey)}], ownTD, initialKnownAgents, lowerLayerAddress, additionalOntology);
    await robot.init();
}

main();

export {};
`;
}

function createLowerThingTs(name, thingPort) {
  return `// Generated lower-level bridge entry for ${name}.
// Phase two: delegates URDF/model/TD generation to src/generator/robwotBridge.js.

async function main() {
    const bridge = require("../../generator/robwotBridge");
    const manifest = "./generated/robots/${name}/manifest.json";
    const ctx = bridge.createContext(manifest);

    if (!bridge.validateContext(ctx)) {
        process.exit(1);
    }

    console.log("${name} lower-level bridge scaffold");
    console.log("Expected TD port: ${thingPort}");
    console.log("Manifest: " + manifest);
    console.log("Run model generation with:");
    console.log("  npm run robot:pipeline -- --manifest " + manifest + " --generate-model");
    console.log("Run TD generation after the model exists with:");
    console.log("  npm run robot:pipeline -- --manifest " + manifest + " --generate-td");
}

main();
`;
}

function createJcm(name, configKey) {
  const robotAtom = jcmAtom(name);
  const masAtom = `${robotAtom}_mas`;
  const agentAtom = `${robotAtom}_agent`;
  const workspaceAtom = `${robotAtom}_workspace`;

  return `/*
    Generated JaCaMo application for ${name}.
    Run with: ./gradlew -Pjcm=generated/robots/${name}/${name}.jcm
*/

mas ${masAtom} {
    agent ${agentAtom}: generated/${agentAslFileName(name)} {
        focus: ${workspaceAtom}.wotComm
    }

    workspace ${workspaceAtom} {
        artifact wotComm: wotCommunication.wotCommunicator("${configKey}")
    }
}
`;
}

function createReadme(name, urdfPath, thingPort, agentPort, websocketPort, mobility) {
  const isMobile = mobility && mobility.type !== "fixed";
  const mobileSection = !isMobile
    ? ""
    : `
## Mobile Platform

The URDF was classified as a mobile platform (\`${mobility.type}\`):

- Wheel joints: ${mobility.wheelJoints.join(", ") || "-"}
- Arm joints: ${mobility.armJoints.join(", ") || "-"}
- Gripper joints: ${mobility.gripperJoints.join(", ") || "-"}
- Base link: ${mobility.baseLink || "-"}, arm root: ${mobility.armRootLink || "-"}, tip: ${mobility.tipLink || "-"}

The classification is stored in \`manifest.json\` under \`mobility\` and can be
overridden with \`--wheel-joints\`, \`--arm-joints\`, \`--gripper-joints\`, and
\`--platform\`.

Planned lower-level Thing affordances for mobile platforms (later phases):
\`moveTo {x, y, theta(deg)}\` for the base, \`moveArmTo {x, y, z}\` in the base
frame, \`pick\`/\`drop\` composites, and a \`getBasePose\` property. The agent
scaffold (TD, .asl, AgentThing) already targets these names. Note that
\`robot:pipeline\` and \`robot:serve\` do not handle mobile platforms yet; the
CoppeliaSim steps below will treat the robot as a fixed manipulator until the
bridge is extended.
`;

  return `# Generated Robot: ${name}

This folder was created by the phase-one WoMAT robot generator.

## Source Input

- URDF: ${urdfPath}
${mobileSection}
## Generated Runtime Shape

- Lower-level robot Thing port: ${thingPort}
- AgentThing TD port: ${agentPort}
- Agent connector websocket port: ${websocketPort}
- AgentSpeak file: src/agt/generated/${agentAslFileName(name)}
- AgentThing file: src/WoT/generated/${name}_agent_thing.ts
- Agent TD: src/td/generated/${name}.agent.td.json
- JaCaMo file: generated/robots/${name}/${name}.jcm
- Config fragment: generated/robots/${name}/config.fragment.json

## Phase-Two Pipeline

Check the generated pipeline paths without CoppeliaSim:

\`\`\`bash
npm run robot:pipeline -- --manifest generated/robots/${name}/manifest.json --dry-run
\`\`\`

With CoppeliaSim running and the websocket remote API available on localhost:23050, generate the CoppeliaSim model:

\`\`\`bash
npm run robot:pipeline -- --manifest generated/robots/${name}/manifest.json --generate-model
\`\`\`

Then generate workspace data and the lower-level robot TD:

\`\`\`bash
npm run robot:pipeline -- --manifest generated/robots/${name}/manifest.json --generate-td
\`\`\`

Or run both CoppeliaSim-dependent steps together:

\`\`\`bash
npm run robot:pipeline -- --manifest generated/robots/${name}/manifest.json --full-pipeline
\`\`\`

If the command says it cannot connect to \`localhost:23050\`, start CoppeliaSim and make sure its websocket remote API is available. If it says \`simURDF.import\` is missing, enable/install the CoppeliaSim URDF plugin and restart CoppeliaSim.

Expected generated artifacts:

- generated/robots/${name}/model/${name}.ttm
- generated/robots/${name}/scene/${name}_workspace.ttt
- generated/robots/${name}/info/${name}_info.json
- generated/robots/${name}/info/${name}_shape.stl
- generated/robots/${name}/info/${name}_data_point.csv
- generated/robots/${name}/td/${name}.lower.td.json

## Lower Thing Runtime

After phase two artifacts exist, expose the generated lower-level robot Thing:

\`\`\`bash
npm run robot:serve -- --manifest generated/robots/${name}/manifest.json
\`\`\`

The workspace scene includes visuals for the reachable workspace (white point cloud plus a transparent convex hull) and collision regions (red point clouds). To spare rendering resources when many robots are spawned, set \`runtime.workspaceVisuals\` to \`false\` in the manifest and restart the server; the visuals are then removed from the scene at load time. No pipeline re-run is needed to toggle this.

In another terminal, expose the generated robot AgentThing bridge. This is what the JaCaMo \`wotCommunication\` artifact connects to over websocket:

\`\`\`bash
npm run robot:agent -- --manifest generated/robots/${name}/manifest.json
\`\`\`

In another terminal, check the TD:

\`\`\`bash
curl http://localhost:${thingPort}/coppeliasim_virtualrobot_${name}
\`\`\`

Phase 5 motion test:

\`\`\`bash
curl -X POST http://localhost:${thingPort}/coppeliasim_virtualrobot_${name.toLowerCase()}/actions/moveTocartesianPosition \\
  -H 'Content-Type: application/json' \\
  -d '{"x":0.1,"y":0.1,"z":0.4}'
\`\`\`

Phase 5 joint motion test:

\`\`\`bash
curl -X POST http://localhost:${thingPort}/coppeliasim_virtualrobot_${name.toLowerCase()}/actions/moveTojointPosition \\
  -H 'Content-Type: application/json' \\
  -d '{"joint1":10,"joint2":-20,"joint3":15,"joint4":0,"joint5":0,"joint6":0}'
\`\`\`

The runtime applies smooth interpolation and workspace prechecks. Motion settings are configurable in \`manifest.json\` via \`runtime.jointSteps\`, \`runtime.cartesianSteps\`, \`runtime.stepDelayMs\`, \`runtime.cartesianMaxVelocity\`, and \`runtime.jointMaxVelocityDeg\`.

Low-level collision checking is configurable in \`manifest.json\` via \`runtime.enableCollisionCheck\`. Joint-space preflight is configurable via \`runtime.enableJointCollisionPreflight\`. OMPL detours and their tracking are configurable via \`runtime.enablePathPlanning\`, \`runtime.planningMaxTimeSeconds\`, \`runtime.planningStateCount\`, \`runtime.plannedJointMaxVelocityDeg\`, \`runtime.pathSmoothingPasses\`, \`runtime.pathTrackingToleranceDeg\`, and \`runtime.pathSettleMaxSteps\`. By default generated robots check the configured wall obstacle \`80cmHighWall50cm\`.

Phase 6 wall collision test:

\`\`\`bash
curl -X POST http://localhost:${thingPort}/coppeliasim_virtualrobot_${name.toLowerCase()}/actions/moveTocartesianPosition \\
  -H 'Content-Type: application/json' \\
  -d '{"x":-0.25,"y":-0.2,"z":0.3}'
\`\`\`

With \`runtime.enableCollisionCheck\` set to \`true\`, a wall-crossing path should return \`path_collision_detected\` before movement starts.

## Agent Check

The JaCaMo side is scaffolded with a generated horizontal demo plan:

- \`!horizontalDemo\` moves to \`horizontalDemoPosition(X,Y,Z)\`.
- It then calls \`!goHome\`.
- If \`homeJoints(array(...))\` is present in the generated \`.asl\`, \`!goHome\` uses joint-space home motion for an upright pose.
- Otherwise \`!goHome\` uses the generated cartesian \`homePosition(X,Y,Z)\`.

Run it after both \`robot:serve\` and \`robot:agent\` are running:

\`\`\`bash
./gradlew -Pjcm=generated/robots/${name}/${name}.jcm
\`\`\`
`;
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  if (args.help) {
    console.log(usage());
    return;
  }

  const repoRoot = path.resolve(__dirname, "..", "..");
  const urdf = args.urdf;
  const name = args.name;
  validateRobotName(name);

  if (!urdf) {
    fail("Missing --urdf.");
  }

  const urdfPath = path.resolve(process.cwd(), urdf);
  if (!fs.existsSync(urdfPath)) {
    fail(`URDF file does not exist: ${urdfPath}`);
  }

  const thingPort = Number(args["thing-port"] || 8090);
  const agentPort = Number(args["agent-port"] || 8091);
  const websocketPort = Number(args["websocket-port"] || agentPort + 100);
  const configKey = args["config-key"] || name;
  const lowerLayerUrl = args["lower-url"] || `http://localhost:${thingPort}/${lowerThingSlug(name)}`;
  const robwotRoot = args["robwot-root"] ? path.resolve(process.cwd(), args["robwot-root"]) : defaultRobwotRoot();
  const homePosition = parseNumberList(args["home-position"], 3, "--home-position") || [0, 0, 0.3];
  const horizontalPosition = parseNumberList(args["horizontal-position"], 3, "--horizontal-position") || [0.2, 0, 0.3];
  const gripPosition = parseNumberList(args["grip-position"], 3, "--grip-position") || [0.45, 0.02, 0.33];
  const carryPosition = parseNumberList(args["carry-position"], 3, "--carry-position") || [0.4, 0.02, 0.53];
  const explicitHomeJoints = parseNumberList(args["home-joints"], null, "--home-joints");
  // Everything this robot's manifest already says, when it has one. The
  // manifest is rewritten wholesale below, so anything read back here is a
  // value that survives regeneration and everything else is not.
  const existingManifest = readJsonIfPresent(
    path.join(repoRoot, "generated", "robots", name, "manifest.json")
  );
  // One value, used by the lower Thing to place the hover pose and by the agent
  // to retreat to it after the grasp. See runtime.gripper.pickupApproachHeight.
  //
  // Carried over from an existing manifest rather than reset to the default,
  // because it is a property of the ARM and the default is a six-axis arm's.
  // A Dobot tuned to 0.03 kept it in its manifest -- which the lower Thing
  // reads -- while the agent's copy was regenerated at 0.20, and the two then
  // disagreed by a factor of seven: the arm placed the cube correctly and then
  // failed reaching 20 cm straight up for a retreat that is outside its
  // envelope, which failed the whole handover after it had physically
  // succeeded. One value means one value on both sides of the bridge.
  const pickupApproachHeight = Number(
    existingManifest?.runtime?.gripper?.pickupApproachHeight ?? 0.20
  );
  // The lift straight up after the grasp, before any lateral move. See
  // pickLiftHeight in createAgentAsl.
  const pickLiftHeight = 0.10;
  const gripperOrientationDeg = parseNumberList(args["gripper-orientation-deg"], 3, "--gripper-orientation-deg") || [-90, 0, 0];
  const gripperOrientation = gripperOrientationDeg.map((degrees) => degrees * Math.PI / 180);
  const gripperLocalRotationDeg = parseNumberList(args["gripper-local-rotation-deg"], 3, "--gripper-local-rotation-deg") || [0, 0, 0];
  const gripperLocalRotation = gripperLocalRotationDeg.map((degrees) => degrees * Math.PI / 180);
  const gripperToolAxisRaw = parseNumberList(args["gripper-tool-axis"], 3, "--gripper-tool-axis") || [0, 1, 0];
  // Which end effector, and the two measurements that are properties OF that
  // end effector rather than of the arm. All three are carried over from an
  // existing manifest for the same reason pickupApproachHeight is: the manifest
  // is rewritten wholesale, so a value that is not read back here is silently
  // reset to the RG2's on the next regeneration -- and a Baxter gripper
  // regenerated back to "touch" pads fails the pipeline at the workspace step,
  // where nothing names the gripper as the cause.
  const gripperModelPath = String(
    args["gripper-model-path"]
      ?? existingManifest?.runtime?.gripper?.modelPath
      ?? "/home/maxchrs/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/models/components/grippers/RG2.ttm"
  );
  // Substring that picks the contact pads out of the gripper's shapes. The RG2
  // names them leftTouch/rightTouch; the Baxter gripper names them
  // leftPad/righPad -- note the model's own typo, which is why the match is a
  // substring and not a pair of names. Their midpoint is the tool control
  // point, so this is what the workspace is sampled at and what the runtime
  // steers; a substring that matches nothing is a hard pipeline failure.
  const graspPadAlias = String(
    args["gripper-pad-alias"] ?? existingManifest?.runtime?.gripper?.graspPadAlias ?? "touch"
  ).toLowerCase();
  // Distance from the flange to those pads. Only a fallback -- the real figure
  // is measured from the scene -- but it is what the hover and admission
  // arithmetic uses, and the two grippers are not close: 0.187 m for the RG2,
  // 0.077 m for the Baxter gripper.
  const toolLengthEstimate = Number(
    args["gripper-tool-length"] ?? existingManifest?.runtime?.gripper?.toolLengthEstimate ?? 0.19
  );
  if (!Number.isFinite(toolLengthEstimate) || toolLengthEstimate <= 0) {
    fail("--gripper-tool-length must be a positive number of metres.");
  }
  // Where the tool comes from. "model" loads modelPath and bolts it to the
  // flange -- an arm imported from a URDF with nothing on the end. "builtin"
  // means the arm's own model already carries one, as the subtree named by
  // --gripper-source-alias; nothing is loaded and nothing is reparented,
  // because that subtree is a link of the kinematic chain. "none" is no tool.
  const gripperSourceName = String(
    args["gripper-source"] ?? existingManifest?.runtime?.gripper?.source
    ?? (existingManifest?.runtime?.gripper?.shipsWithModel === true ? "builtin" : "model")
  );
  if (!GRIPPER_SOURCES.includes(gripperSourceName)) {
    fail(`--gripper-source must be one of ${GRIPPER_SOURCES.join(", ")}.`);
  }
  const gripperSourceAlias = String(
    args["gripper-source-alias"] ?? existingManifest?.runtime?.gripper?.sourceAlias ?? ""
  );
  if (gripperSourceName === "builtin" && gripperSourceAlias === "") {
    fail("--gripper-source builtin also needs --gripper-source-alias, the end effector's alias in the robot's own model.");
  }
  // Only for a tool with no contact pads to measure a tool control point from.
  // Measuring wins wherever there is anything to measure, so this stays unset
  // for every jawed gripper.
  const gripperTcpOffset = parseNumberList(args["gripper-tcp-offset"], 3, "--gripper-tcp-offset")
    ?? existingManifest?.runtime?.gripper?.toolControlPointOffset
    ?? null;
  const gripperToolAxisLength = Math.hypot(...gripperToolAxisRaw);
  if (!(gripperToolAxisLength > 1e-9)) {
    fail("--gripper-tool-axis must be a non-zero direction.");
  }
  const gripperToolAxis = gripperToolAxisRaw.map((value) => value / gripperToolAxisLength);

  const mobility = analyzeUrdfStructure(urdfPath, args);
  // One zero per degree of freedom unless the caller says otherwise. Sized from
  // the arm rather than left to a constant, so locking a joint cannot leave a
  // home pose that names an axis the arm does not have.
  const homeJoints = explicitHomeJoints
    || (mobility.armJoints.length > 0 ? mobility.armJoints.map(() => 0) : null);
  if (explicitHomeJoints && mobility.armJoints.length > 0
      && explicitHomeJoints.length !== mobility.armJoints.length) {
    console.warn(
      `--home-joints lists ${explicitHomeJoints.length} values for ${mobility.armJoints.length} `
      + `arm joint(s)${mobility.lockedJoints?.length ? ` (${mobility.lockedJoints.length} locked)` : ""}.`
    );
  }
  const isMobile = mobility.type !== "fixed";
  const baseHomePose = parseNumberList(args["base-home-pose"], 3, "--base-home-pose") || [0, 0, 0];
  const basePickPose = parseNumberList(args["base-pick-pose"], 3, "--base-pick-pose") || [0.9, 0, 0];
  const baseDropPose = parseNumberList(args["base-drop-pose"], 3, "--base-drop-pose") || [0, 0.9, 90];
  const motionDefaults = {
    homePosition,
    horizontalPosition,
    gripPosition,
    carryPosition,
    homeJoints,
    pickupApproachHeight,
    pickLiftHeight,
    ...(isMobile ? { baseHomePose, basePickPose, baseDropPose } : {})
  };

  const robotRoot = path.join(repoRoot, "generated", "robots", name);
  const tdFile = path.join(repoRoot, "src", "td", "generated", `${name}.agent.td.json`);
  const agentFile = path.join(repoRoot, "src", "agt", "generated", agentAslFileName(name));
  const agentThingFile = path.join(repoRoot, "src", "WoT", "generated", `${name}_agent_thing.ts`);
  const lowerThingFile = path.join(repoRoot, "src", "WoT", "generated", `${name}_lower_thing.ts`);
  const jcmFile = path.join(robotRoot, `${name}.jcm`);
  const manifestFile = path.join(robotRoot, "manifest.json");
  const configFragmentFile = path.join(robotRoot, "config.fragment.json");
  const readmeFile = path.join(robotRoot, "README.md");
  const configFile = path.join(repoRoot, "config.json");

  const configFragment = {
    [configKey]: {
      websocketPort,
      TDPort: agentPort,
      staticAddress: "localhost"
    }
  };

  const shouldUpdateConfig = !args["skip-config-update"];
  let configUpdated = false;
  if (shouldUpdateConfig) {
    const rootConfig = readJson(configFile);
    if (!rootConfig[configKey]) {
      rootConfig[configKey] = configFragment[configKey];
      writeJson(configFile, rootConfig);
      configUpdated = true;
    }
  }

  const manifest = {
    name,
    phase: 1,
    urdfPath,
    configKey,
    mobility,
    lowerLayerUrl,
    ports: {
      lowerThing: thingPort,
      agentThing: agentPort,
      websocket: websocketPort
    },
    generatedFiles: {
      agentTd: posixPath(path.relative(repoRoot, tdFile)),
      agentSpeak: posixPath(path.relative(repoRoot, agentFile)),
      agentThing: posixPath(path.relative(repoRoot, agentThingFile)),
      lowerThing: posixPath(path.relative(repoRoot, lowerThingFile)),
      jcm: posixPath(path.relative(repoRoot, jcmFile)),
      configFragment: posixPath(path.relative(repoRoot, configFragmentFile))
    },
    pipeline: {
      robwotRoot,
      urdfLoadScene: path.join(robwotRoot, "Coppeliasim scene", "robot_urdf_load_scene.ttt"),
      workspaceSceneTemplate: path.join(robwotRoot, "Coppeliasim scene", "robot_virtual_workspace.ttt"),
      workspaceDriver: path.join(robwotRoot, "Library", "GenerateRobotDescription", "robot_driver_workspace.lua"),
      robotTemplate: path.join(robwotRoot, "Library", "GenerateRobotDescription", "robot_template.json"),
      workspaceDivisions: 8,
      collisionObstacles: ["80cmHighWall50cm", `${name}_grip_support`],
      gripperCollisionClearance: 0.005,
      workspaceRandomSteps: 8000
    },
    runtime: {
      simpleSmoothMotion: true,
      simpleJointVelocityDeg: 30,
      simpleJointAccelerationDeg: 60,
      simpleJointJerkDeg: 120,
      simpleCartesianVelocity: 0.175,
      simpleCartesianAcceleration: 0.25,
      simpleCartesianJerk: 0.75,
      jointSteps: 60,
      cartesianSteps: 60,
      stepDelayMs: 25,
      cartesianMaxVelocity: 0.45,
      cartesianFinalTolerance: 0.025,
      // Size of the visible tool-control-point dummy, at the contact-pad
      // midpoint. This is where the robot actually holds things and what every
      // offset in the runtime is measured to.
      tcpMarkerSize: 0.02,
      // Whether this robot plans its handover legs with the wrist pinned at
      // all. On, the rendezvous is a standoff plus a straight line in with the
      // tool axis held, and handoverApproachMode below chooses that axis. Off,
      // none of that runs: the arm is simply sent to the rendezvous position
      // and the free wrist picks its own posture, exactly like any other
      // Cartesian move, with the same collision guards. That is the setting to
      // reach for when an arm cannot converge on the constrained pose -- a
      // mounting can make a commanded tool direction unreachable at the
      // standoff however reachable the point itself is. The cost is that where
      // the jaws end up is no longer commanded, which is what
      // handoverGraspTolerance then has to catch.
      enableHandoverPoseLock: existingManifest?.runtime?.enableHandoverPoseLock ?? true,
      ...(existingManifest?.runtime?.enableHandoverPoseLockAsGiver !== undefined
        ? { enableHandoverPoseLockAsGiver: existingManifest.runtime.enableHandoverPoseLockAsGiver } : {}),
      ...(existingManifest?.runtime?.enableHandoverPoseLockAsReceiver !== undefined
        ? { enableHandoverPoseLockAsReceiver: existingManifest.runtime.enableHandoverPoseLockAsReceiver } : {}),
      // How the receiver meets the giver at the rendezvous. The first two are
      // relative to the giver's tool, so they follow it as the cell is
      // re-laid-out; the last three pin this robot's own tool axis (the
      // gripper's +Y, which runs through its jaw midpoint) to a direction in
      // world coordinates and hold it whatever the giver does.
      //   "opposed"       - the two tools nose to nose. Shortest path, but both
      //                     arms have to share one narrow corridor.
      //   "perpendicular" - the receiver's tool turned 90 degrees off the
      //                     giver's (by default coming down from above), which
      //                     buys clearance when that corridor is too tight.
      //   "horizontalX"   - tool axis along world X, signed so it points across
      //                     the cell at the arm being exchanged with.
      //   "horizontalY"   - the same along world Y.
      //   "vertical"      - tool axis straight down (world -Z): the receiver
      //                     comes down onto the cube from above. The one to
      //                     reach for when a robot's mounting leaves its tool
      //                     pointing the wrong way for a head-on exchange.
      // Overridable per pair via sharedEnvironment.interactionPairs[].approachMode.
      handoverApproachMode: "opposed",
      // Which way this robot holds the cube out when it is the GIVER, using the
      // same named cell directions. Unset, it presents along its own mounting,
      // so the exchange geometry follows how the arm happens to be bolted down.
      // Setting it -- "horizontalY" holds the gripper level, reaching across
      // the cell at the receiver -- pins the presentation instead, so every
      // giver meets the rendezvous the same way whatever its mounting.
      handoverPresentMode: "horizontalY",
      // How many directions around that circle the planner may try before
      // giving up, and how far it may turn away from the requested one. The
      // bound matters: unbounded, "sideways is blocked" gets answered with
      // "then come up from underneath", which is still perpendicular and still
      // collision free but is not the manoeuvre that was asked for.
      handoverApproachCandidates: 8,
      handoverApproachMaxTurnDeg: 60,
      // The same bound for the named cell directions, kept tighter: a named
      // direction is chosen because the arm should come in ALONG it, and 60
      // degrees off "vertical" is not vertical any more. The primary is always
      // tried first; the tilts only exist so a rendezvous that is almost
      // solvable is not thrown away over a few degrees.
      handoverFixedApproachMaxTurnDeg: 25,
      // Which perpendicular, when the mode is "perpendicular". Omitted on
      // purpose: with no explicit axis the receiver comes in across the cell,
      // along the line between the two bases, which follows the layout instead
      // of hardcoding a world direction. Set it to a world vector (e.g.
      // [-1, 0, 0]) only to override that.
      jointMaxVelocityDeg: 60,
      workspaceTolerance: 0.08,
      enableCollisionCheck: true,
      enableJointCollisionPreflight: true,
      // Safety runs once per command (CSV admission plus a swept body check)
      // with only a cheap strided watchdog on the executed trajectory.
      enableWorkspaceAdmission: true,
      enableInMotionCollisionCheck: true,
      // Contact every step: at 0.175 m/s a stride of 4 leaves 35 mm between
      // tests, and the wall and cube are thinner than that.
      inMotionCollisionStride: 1,
      inMotionClearanceStride: 0,
      // The sweep runs in one script call so it stays atomic against
      // simulation stepping; sample budget is what keeps that call short.
      preflightStepSize: 0.04,
      preflightMaxSamples: 16,
      preflightClearanceStride: 0,
      collisionClearance: 0.02,
      gripperCollisionClearance: 0.005,
      pickupGripperClearance: 0.001,
      collisionCheckStepDeg: 0.5,
      enablePathPlanning: true,
      planningMaxTimeSeconds: 4,
      planningStateCount: 120,
      plannedJointMaxVelocityDeg: 45,
      pathSmoothingPasses: 2,
      pathTrackingToleranceDeg: 0.5,
      pathSettleMaxSteps: 100,
      workspaceVisuals: true,
      collisionObstacles: ["80cmHighWall50cm", `${name}_grip_support`],
      gripper: {
        modelName: `${name}_tcp_gripper`,
        modelPath: gripperModelPath,
        modelPosition: [0, 0, 0],
        modelOrientation: gripperOrientation,
        modelLocalRotation: gripperLocalRotation,
        applyBaseOrientationCorrection: args["no-gripper-base-orientation-correction"] !== true,
        openJointPositions: [0.06],
        closedJointPositions: [0],
        // Direction the tool points, in the robot's own frame. A direction
        // only: the robot's tool control point is the midpoint of its gripper
        // jaws, so a commanded Cartesian position is where the object goes and
        // there is no offset length to carry around.
        toolAxis: gripperToolAxis,
        objectName: `${name}_grip_cube`,
        supportName: `${name}_grip_support`,
        objectSize: [0.045, 0.045, 0.045],
        objectPosition: [0.45, 0.2, 0.325],
        supportSize: [0.18, 0.18, 0.24],
        supportPosition: [0.45, 0.2, 0.18],
        gripThreshold: 0.04,
        // Pick in two legs: hover this far above the grasp, then descend.
        // The first leg is a straight line from wherever the arm is, so the
        // hover wants enough height that the line clears whatever the object
        // is standing on.
        pickupApproachHeight,
        // Bias the grasp along the object's height. With a top-down approach
        // the tool control point sits at the object centre, so this only buys
        // fingertip clearance over whatever the object stands on.
        graspHeightOffset: 0.005,
        // Shapes whose alias contains this are the contact pads; their midpoint
        // is the tool control point.
        graspPadAlias,
        source: gripperSourceName,
        ...(gripperSourceAlias ? { sourceAlias: gripperSourceAlias } : {}),
        ...(gripperTcpOffset ? { toolControlPointOffset: gripperTcpOffset } : {}),
        enableTopDownPick: true,
        // Disable to release above the support center without searching its face.
        enablePlaceFaceSearch: existingManifest?.runtime?.gripper?.enablePlaceFaceSearch ?? true,
        placeReleaseHeight: existingManifest?.runtime?.gripper?.placeReleaseHeight ?? pickupApproachHeight,
        // Handover: back off this far along the approach axis, then travel
        // straight in. A joint-space line to the rendezvous sweeps a much
        // larger volume than it looks like and rotates the tool on the way,
        // which fouls the other arm even when the rendezvous pose is clear.
        handoverStandoffDistance: 0.15,
        handoverApproachSteps: 8,
        // Planning budget for ONE approach direction. The direction search runs
        // a separate script call per candidate, so this bounds every individual
        // blocking call rather than their total.
        handoverPlanningMaxTime: 2.5,
        pickDescentSteps: 8,
        pickPlanningMaxTime: 4,
        pickIkDamping: 0.05,
        // Acceptance radius for simIK.findConfig's random restarts, not the
        // accuracy of the pose it returns. CoppeliaSim's own default for the
        // equivalent findConfigs parameter is 0.3; the pick also constrains the
        // tool axis, and the unconstrained roll about that axis still counts
        // towards the pose distance, so the gate is wider here.
        pickSearchThresholdDist: 0.6,
        // What the returned configuration must actually achieve, verified
        // against the scene before the plan is accepted.
        pickPositionTolerance: 0.002,
        pickAxisToleranceDeg: 2,
        toolLengthEstimate
      },
      ...(isMobile
        ? {
            base: {
              maxLinearVelocity: 0.4,
              maxAngularVelocityDeg: 90,
              stepDelayMs: 25,
              positionTolerance: 0.03,
              thetaToleranceDeg: 2
            }
          }
        : {})
    },
    agentDefaults: motionDefaults,
    config: {
      rootConfigUpdated: configUpdated,
      rootConfigKeyExists: shouldUpdateConfig
    },
    nextPhase: isMobile
      ? "Extend the RobWoT bridge for mobile platforms: kinematic base import, arm-only workspace sampling, and a lower TD with moveTo/moveArmTo/pick/drop."
      : "Replace the lower Thing placeholder with RobWoT URDF loading, CoppeliaSim model generation, and TD generation."
  };

  writeJson(tdFile, isMobile ? createMobileAgentTd(name) : createAgentTd(name));
  writeFile(agentFile, isMobile ? createMobileAgentAsl(name, motionDefaults) : createAgentAsl(name, motionDefaults));
  writeFile(
    agentThingFile,
    isMobile
      ? createMobileAgentThingTs(name, configKey, tdFile, lowerLayerUrl, motionDefaults)
      : createAgentThingTs(name, configKey, tdFile, lowerLayerUrl, motionDefaults)
  );
  writeFile(lowerThingFile, createLowerThingTs(name, thingPort));
  writeFile(jcmFile, createJcm(name, configKey));
  writeJson(configFragmentFile, configFragment);
  writeJson(manifestFile, manifest);
  writeFile(readmeFile, createReadme(name, urdfPath, thingPort, agentPort, websocketPort, mobility));

  console.log(`Generated phase-one robot scaffold for ${name}.`);
  console.log(`Platform type: ${mobility.type}`);
  if (isMobile) {
    console.log(`Wheel joints: ${mobility.wheelJoints.join(", ") || "-"}`);
    console.log(`Arm joints: ${mobility.armJoints.join(", ") || "-"}`);
    console.log(`Gripper joints: ${mobility.gripperJoints.join(", ") || "-"}`);
  }
  for (const warning of mobility.warnings || []) {
    console.warn(`Warning: ${warning}`);
  }
  console.log(`Manifest: ${posixPath(path.relative(repoRoot, manifestFile))}`);
  if (configUpdated) {
    console.log(`Config: added ${configKey} to config.json`);
  } else if (shouldUpdateConfig) {
    console.log(`Config: ${configKey} already exists in config.json`);
  } else {
    console.log("Config: skipped config.json update");
  }
  console.log(`JaCaMo: ./gradlew -Pjcm=${posixPath(path.relative(repoRoot, jcmFile))}`);
}

main();
