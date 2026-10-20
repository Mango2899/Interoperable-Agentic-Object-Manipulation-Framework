#!/usr/bin/env node

// Kinematic-structure analysis for URDF files.
// Splits actuated joints into base (wheels), arm chain, and gripper groups so
// that the generator and the RobWoT bridge can treat mobile platforms with
// arms differently from fixed manipulators. No external dependencies.

const fs = require("fs");

const ACTUATED_TYPES = new Set(["revolute", "continuous", "prismatic"]);
const WHEEL_NAME_PATTERN = /wheel|caster|roller/i;
const GRIPPER_NAME_PATTERN = /gripper|finger|hand|jaw|claw/i;

// --- Minimal XML parsing (elements + attributes, enough for URDF) ---

function parseXml(xml) {
  const root = { tag: "#document", attributes: {}, children: [] };
  const stack = [root];
  const tagPattern = /<!--[\s\S]*?-->|<\?[\s\S]*?\?>|<!\[CDATA\[[\s\S]*?\]\]>|<!DOCTYPE[^>]*>|<\/([^\s>]+)\s*>|<([^\s/>]+)((?:[^>"']|"[^"]*"|'[^']*')*?)(\/?)>/g;

  let match;
  while ((match = tagPattern.exec(xml)) !== null) {
    const [token, closeTag, openTag, attrText, selfClose] = match;
    if (token.startsWith("<!--") || token.startsWith("<?") || token.startsWith("<!")) {
      continue;
    }
    if (closeTag) {
      if (stack.length > 1) {
        stack.pop();
      }
      continue;
    }
    const node = { tag: openTag, attributes: parseAttributes(attrText || ""), children: [] };
    stack[stack.length - 1].children.push(node);
    if (!selfClose) {
      stack.push(node);
    }
  }

  return root;
}

function parseAttributes(text) {
  const attributes = {};
  const attrPattern = /([^\s=]+)\s*=\s*("([^"]*)"|'([^']*)')/g;
  let match;
  while ((match = attrPattern.exec(text)) !== null) {
    attributes[match[1]] = match[3] !== undefined ? match[3] : match[4];
  }
  return attributes;
}

function childByTag(node, tag) {
  return node.children.find((child) => child.tag === tag) || null;
}

// --- URDF model extraction ---

function parseUrdf(xml) {
  const document = parseXml(xml);
  const robot = childByTag(document, "robot");
  if (!robot) {
    throw new Error("No <robot> element found in URDF.");
  }

  const links = robot.children
    .filter((node) => node.tag === "link" && node.attributes.name)
    .map((node) => node.attributes.name);

  // Only direct children of <robot> are kinematic joints; joints nested in
  // <transmission>/<gazebo> blocks are references without parent/child.
  const joints = robot.children
    .filter((node) => node.tag === "joint" && node.attributes.name && childByTag(node, "parent"))
    .map((node) => {
      const limit = childByTag(node, "limit");
      const mimic = childByTag(node, "mimic");
      const axis = childByTag(node, "axis");
      return {
        name: node.attributes.name,
        type: node.attributes.type || "fixed",
        parent: childByTag(node, "parent").attributes.link,
        child: childByTag(node, "child") ? childByTag(node, "child").attributes.link : null,
        axis: axis ? axis.attributes.xyz : null,
        mimic: mimic ? mimic.attributes.joint : null,
        limit: limit
          ? {
              lower: limit.attributes.lower !== undefined ? Number(limit.attributes.lower) : null,
              upper: limit.attributes.upper !== undefined ? Number(limit.attributes.upper) : null,
              effort: limit.attributes.effort !== undefined ? Number(limit.attributes.effort) : null,
              velocity: limit.attributes.velocity !== undefined ? Number(limit.attributes.velocity) : null
            }
          : null
      };
    });

  return { name: robot.attributes.name || "robot", links, joints };
}

// --- Kinematic tree helpers ---

function buildTree(model) {
  const childJointsByLink = new Map();
  const parentJointByLink = new Map();
  for (const joint of model.joints) {
    if (!childJointsByLink.has(joint.parent)) {
      childJointsByLink.set(joint.parent, []);
    }
    childJointsByLink.get(joint.parent).push(joint);
    parentJointByLink.set(joint.child, joint);
  }

  const rootLinks = model.links.filter((link) => !parentJointByLink.has(link));
  return { childJointsByLink, parentJointByLink, rootLinks };
}

function subtreeActuatedCount(link, tree, jointFilter) {
  const queue = [link];
  let count = 0;
  while (queue.length > 0) {
    const current = queue.shift();
    for (const joint of tree.childJointsByLink.get(current) || []) {
      if (ACTUATED_TYPES.has(joint.type) && jointFilter(joint)) {
        count += 1;
      }
      queue.push(joint.child);
    }
  }
  return count;
}

// Walks from rootLink towards the leaves and returns the arm joints of the
// path that carries the most arm joints, in root-to-tip order.
function longestArmChain(rootLink, tree, isArmJoint) {
  function walk(link) {
    let best = [];
    for (const joint of tree.childJointsByLink.get(link) || []) {
      const below = walk(joint.child);
      const candidate = isArmJoint(joint) ? [joint, ...below] : below;
      if (candidate.length > best.length) {
        best = candidate;
      }
    }
    return best;
  }
  return walk(rootLink);
}

// --- Classification ---

function classifyRobot(model, overrides = {}) {
  const warnings = [];
  const tree = buildTree(model);

  if (tree.rootLinks.length === 0) {
    throw new Error("URDF has no root link (kinematic loop?).");
  }
  if (tree.rootLinks.length > 1) {
    warnings.push(`Multiple root links found (${tree.rootLinks.join(", ")}); using ${tree.rootLinks[0]}.`);
  }
  const rootLink = tree.rootLinks[0];
  const actuated = model.joints.filter((joint) => ACTUATED_TYPES.has(joint.type));

  const forced = (list) => new Set(Array.isArray(list) ? list : []);
  const forcedWheels = forced(overrides.wheelJoints);
  const forcedArm = forced(overrides.armJoints);
  const forcedGripper = forced(overrides.gripperJoints);
  const isForced = (joint) =>
    forcedWheels.has(joint.name) || forcedArm.has(joint.name) || forcedGripper.has(joint.name);

  const wheelJoints = [];
  const gripperJoints = [];

  for (const joint of actuated) {
    if (forcedWheels.has(joint.name)) {
      wheelJoints.push(joint);
      continue;
    }
    if (forcedGripper.has(joint.name)) {
      gripperJoints.push(joint);
      continue;
    }
    if (forcedArm.has(joint.name)) {
      continue;
    }
    if (joint.mimic || GRIPPER_NAME_PATTERN.test(joint.name)) {
      gripperJoints.push(joint);
      continue;
    }
    if (WHEEL_NAME_PATTERN.test(joint.name) && (joint.type === "continuous" || joint.type === "revolute")) {
      wheelJoints.push(joint);
      continue;
    }
    // Unnamed-wheel fallback: an unlimited joint with no actuated joints below
    // it, hanging directly off the base structure, behaves like a wheel.
    if (
      joint.type === "continuous" &&
      subtreeActuatedCount(joint.child, tree, (candidate) => !isForced(candidate)) === 0
    ) {
      wheelJoints.push(joint);
      warnings.push(`Joint ${joint.name} classified as wheel by structure (continuous leaf joint).`);
    }
  }

  const wheelSet = new Set(wheelJoints.map((joint) => joint.name));
  const gripperSet = new Set(gripperJoints.map((joint) => joint.name));
  const isArmJoint = (joint) =>
    ACTUATED_TYPES.has(joint.type) && !wheelSet.has(joint.name) && !gripperSet.has(joint.name);

  const fullArmChain = longestArmChain(rootLink, tree, isArmJoint);
  // Joints on the arm's own chain that the machine does not drive: a URDF has to
  // give every mesh a joint, and a linkage the description cannot express comes
  // out as an axis nobody actuates. Left in the chain -- the tip is still on the
  // far side of it -- and taken out of the degrees of freedom, which is what
  // "arm joints" has always meant to everything downstream.
  const lockedNames = forced(overrides.lockedJoints);
  const armChain = fullArmChain.filter((joint) => !lockedNames.has(joint.name));
  const lockedChain = fullArmChain.filter((joint) => lockedNames.has(joint.name));
  for (const name of lockedNames) {
    if (!fullArmChain.some((joint) => joint.name === name)) {
      warnings.push(`Locked joint ${name} is not on the arm chain; nothing was locked.`);
    }
  }
  const armSet = new Set(fullArmChain.map((joint) => joint.name));
  const unassigned = actuated.filter(
    (joint) => !wheelSet.has(joint.name) && !gripperSet.has(joint.name) && !armSet.has(joint.name)
  );
  for (const joint of unassigned) {
    warnings.push(`Actuated joint ${joint.name} is on a side branch and was not assigned to any group.`);
  }

  let type = overrides.platform && overrides.platform !== "auto" ? overrides.platform : null;
  if (!type) {
    type = wheelJoints.length >= 3 ? "omnidirectional" : "fixed";
  }
  if (type !== "fixed" && wheelJoints.length === 0) {
    warnings.push("Platform declared mobile but no wheel joints were found; base motion will be purely kinematic.");
  }

  const armJointLimits = {};
  for (const joint of armChain) {
    armJointLimits[joint.name] = joint.limit;
  }
  const lockedJointLimits = {};
  for (const joint of lockedChain) {
    lockedJointLimits[joint.name] = joint.limit;
  }

  return {
    robotName: model.name,
    rootLink,
    counts: {
      links: model.links.length,
      joints: model.joints.length,
      actuated: actuated.length
    },
    mobility: {
      type,
      baseLink: rootLink,
      wheelJoints: wheelJoints.map((joint) => joint.name),
      armJoints: armChain.map((joint) => joint.name),
      // Every joint between the base and the tool flange, in order, locked ones
      // included. The runtime cannot identify a joint by name -- the model
      // generator aliases them all "joint" on import -- so a joint's position in
      // this chain is the only durable way to point at one.
      jointChain: fullArmChain.map((joint) => joint.name),
      // The chain still runs through them, so the tip is where it always was.
      lockedJoints: lockedChain.map((joint) => joint.name),
      lockedJointLimits,
      armRootLink: fullArmChain.length > 0 ? fullArmChain[0].parent : null,
      tipLink: fullArmChain.length > 0 ? fullArmChain[fullArmChain.length - 1].child : null,
      gripperJoints: gripperJoints.map((joint) => joint.name),
      armJointLimits
    },
    warnings
  };
}

function analyzeUrdfFile(urdfPath, overrides = {}) {
  const xml = fs.readFileSync(urdfPath, "utf8");
  return classifyRobot(parseUrdf(xml), overrides);
}

// --- CLI ---

function formatSummary(result) {
  const mobility = result.mobility;
  const lines = [
    `Robot: ${result.robotName}`,
    `Root link: ${result.rootLink}`,
    `Links: ${result.counts.links}, joints: ${result.counts.joints}, actuated: ${result.counts.actuated}`,
    `Platform type: ${mobility.type}`,
    `Wheel joints (${mobility.wheelJoints.length}): ${mobility.wheelJoints.join(", ") || "-"}`,
    `Arm joints (${mobility.armJoints.length}): ${mobility.armJoints.join(", ") || "-"}`,
    `Locked joints (${(mobility.lockedJoints || []).length}): ${(mobility.lockedJoints || []).join(", ") || "-"}`,
    `Arm root link: ${mobility.armRootLink || "-"}`,
    `Tip link: ${mobility.tipLink || "-"}`,
    `Gripper joints (${mobility.gripperJoints.length}): ${mobility.gripperJoints.join(", ") || "-"}`
  ];
  for (const warning of result.warnings) {
    lines.push(`Warning: ${warning}`);
  }
  return lines.join("\n");
}

function main() {
  const args = process.argv.slice(2);
  const urdfPath = args.find((arg) => !arg.startsWith("--"));
  if (!urdfPath) {
    console.error("Usage: node src/generator/urdfStructure.js <file.urdf> [--json]");
    process.exit(1);
  }

  const result = analyzeUrdfFile(urdfPath);
  if (args.includes("--json")) {
    console.log(JSON.stringify(result, null, 2));
  } else {
    console.log(formatSummary(result));
  }
}

if (require.main === module) {
  main();
}

module.exports = { parseUrdf, classifyRobot, analyzeUrdfFile };
