#!/usr/bin/env node

// Reports where the tool frame actually is, measured from the live scene, and
// optionally repairs it.
//
// The flange, the gripper and the tool control point are three objects whose
// relationship is easy to get wrong and impossible to read off a screenshot: a
// gripper mounted one tool length out looks much like a tool control point
// placed one tool length in, and an orphan left by an earlier run looks like
// both. This prints the parentage and the distances so the question is settled
// by measurement.

const fs = require("fs");
const path = require("path");

function parseArgs(argv) {
  const args = {};
  for (let i = 0; i < argv.length; i += 1) {
    const token = argv[i];
    if (!token.startsWith("--")) continue;
    const next = argv[i + 1];
    if (!next || next.startsWith("--")) {
      args[token.slice(2)] = true;
    } else {
      args[token.slice(2)] = next;
      i += 1;
    }
  }
  return args;
}

function q(value) {
  return JSON.stringify(value);
}

function luaNumbers(values) {
  return `{${values.map((v) => Number(v)).join(",")}}`;
}

function padMidpointLua(indent) {
  // Shared by the report and the repair, so the number that is checked is the
  // number that was acted on.
  const pad = " ".repeat(indent);
  return [
    `${pad}local low={math.huge, math.huge, math.huge}`,
    `${pad}local high={-math.huge, -math.huge, -math.huge}`,
    `${pad}local found=false`,
    `${pad}for _, shape in ipairs(sim.getObjectsInTree(gripper, sim.object_shape_type, 0)) do`,
    `${pad}    local alias=string.lower(sim.getObjectAlias(shape, 4) or '')`,
    `${pad}    if string.find(alias, PAD_ALIAS, 1, true) ~= nil then`,
    `${pad}        found=true`,
    `${pad}        local m=sim.getObjectMatrix(shape, sim.handle_world)`,
    `${pad}        local e={`,
    `${pad}            {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_x), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_x)},`,
    `${pad}            {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_y), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_y)},`,
    `${pad}            {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_z), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_z)}`,
    `${pad}        }`,
    `${pad}        for a=1,2,1 do`,
    `${pad}            for b=1,2,1 do`,
    `${pad}                for c=1,2,1 do`,
    `${pad}                    local w=sim.multiplyVector(m, {e[1][a], e[2][b], e[3][c]})`,
    `${pad}                    for k=1,3,1 do`,
    `${pad}                        if w[k] < low[k] then low[k]=w[k] end`,
    `${pad}                        if w[k] > high[k] then high[k]=w[k] end`,
    `${pad}                    end`,
    `${pad}                end`,
    `${pad}            end`,
    `${pad}        end`,
    `${pad}    end`,
    `${pad}end`,
    `${pad}if not found then return nil end`,
    `${pad}return {(low[1]+high[1])/2, (low[2]+high[2])/2, (low[3]+high[3])/2}`
  ].join("\n");
}

function buildInspectionLua({ name, gripperAlias, padAlias, grip, repair, orientTcp = false }) {
  const markerAlias = `${name}_tcp`;
  // Wrapped in pcall so a Lua error comes back as data. Unwrapped, the remote
  // API reports only "operation failed" with a line number into a string the
  // caller never sees, which is the least useful form a diagnostic can take.
  return ["local ok, result=pcall(function()"].concat([
    fs.readFileSync(path.join(__dirname, "lua/00-tool-frame-math.lua"), "utf8"),
    `local PAD_ALIAS=${q(padAlias)}`,
    "local report={duplicates={}}",
    `local robot=sim.getObject('/'..${q(name)}, {noError=true})`,
    "if robot == -1 then return {error='robot_not_in_scene'} end",
    "",
    "local function plain(handle)",
    "    local alias=sim.getObjectAlias(handle, 4)",
    "    if alias == nil then return '' end",
    "    return (string.gsub(alias, '__%d+__$', ''))",
    "end",
    "",
    "local function chain(handle)",
    "    local names={}",
    "    local current=handle",
    "    while current ~= nil and current ~= -1 and #names < 12 do",
    "        names[#names+1]=sim.getObjectAlias(current, 4)",
    "        current=sim.getObjectParent(current)",
    "    end",
    "    return table.concat(names, ' <- ')",
    "end",
    "",
    "local function describe(handle)",
    "    if handle == nil or handle == -1 then return nil end",
    "    local parent=sim.getObjectParent(handle)",
    "    return {",
    "        alias=sim.getObjectAlias(handle, 4),",
    "        parent=(parent ~= -1) and sim.getObjectAlias(parent, 4) or 'scene root',",
    "        position=sim.getObjectPosition(handle, sim.handle_world),",
    "        matrix=sim.getObjectMatrix(handle, sim.handle_world),",
    "        chain=chain(handle)",
    "    }",
    "end",
    "",
    "local function padMidpoint(gripper)",
    "    if gripper == nil or gripper == -1 then return nil end",
    padMidpointLua(4),
    "end",
    "",
    "-- Scene-wide, not just the robot tree: an orphan left by an earlier run",
    "-- sits outside it and is exactly what a bad mount looks like on screen.",
    "local everything=sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)",
    "local flange=-1",
    "local markers={}",
    "local grippers={}",
    "for i=1,#everything,1 do",
    "    local handle=everything[i]",
    "    local alias=plain(handle)",
    "    if alias == 'tip' then",
    "        local root=handle",
    "        while sim.getObjectParent(root) ~= -1 do root=sim.getObjectParent(root) end",
    `        if root == robot or plain(root) == ${q(name)} then flange=handle end`,
    "    end",
    `    if alias == ${q(markerAlias)} then markers[#markers+1]=handle end`,
    `    if alias == ${q(gripperAlias)} then grippers[#grippers+1]=handle end`,
    "end",
    "",
    "report.tip=describe(flange)",
    "report.marker=describe(markers[1])",
    "report.gripper=describe(grippers[1])",
    "report.markerCount=#markers",
    "report.gripperCount=#grippers",
    "for i=2,#markers,1 do",
    "    report.duplicates[#report.duplicates+1]={kind='marker', info=describe(markers[i])}",
    "end",
    "for i=2,#grippers,1 do",
    "    report.duplicates[#report.duplicates+1]={kind='gripper', info=describe(grippers[i])}",
    "end",
    "report.pads=padMidpoint(grippers[1])",
    ""
  ]).concat(repair ? [
    "if flange ~= -1 and grippers[1] ~= nil then",
    "    report.repaired={}",
    "    -- Every marker was measured THROUGH whatever the gripper was mounted",
    "    -- on, so all of them are suspect. Remove first, rebuild after.",
    "    for i=1,#markers,1 do",
    "        pcall(sim.removeObject, markers[i])",
    "        report.repaired[#report.repaired+1]='removed marker'",
    "    end",
    "    for i=2,#grippers,1 do",
    "        pcall(sim.removeModel, grippers[i])",
    "        report.repaired[#report.repaired+1]='removed duplicate gripper'",
    "    end",
    "    local gripper=grippers[1]",
    "    sim.setObjectParent(gripper, flange, false)",
    `    sim.setObjectPosition(gripper, flange, ${luaNumbers(grip.modelPosition || [0, 0, 0])})`,
    `    sim.setObjectOrientation(gripper, flange, ${luaNumbers(grip.modelOrientation || [-1.57079632679, 0, 0])})`,
    `    local extra=${luaNumbers(grip.modelLocalRotation || [0, 0, 0])}`,
    "    if math.abs(extra[1])+math.abs(extra[2])+math.abs(extra[3]) > 0.0000001 then",
    "        local currentLocal=sim.getObjectMatrix(gripper, flange)",
    "        sim.setObjectMatrix(gripper, flange, sim.multiplyMatrices(currentLocal, sim.buildMatrix({0,0,0}, extra)))",
    "    end",
    "    report.repaired[#report.repaired+1]='remounted gripper on flange'",
    "    local pads=padMidpoint(gripper)",
    "    if pads ~= nil then",
    "        local dummy=sim.createDummy(0.02)",
    `        sim.setObjectAlias(dummy, ${q(markerAlias)})`,
    "        pcall(sim.setObjectColor, dummy, 0, sim.colorcomponent_ambient_diffuse, {1, 0.35, 0})",
    "        sim.setObjectParent(dummy, flange, true)",
    "        sim.setObjectPosition(dummy, sim.handle_world, pads)",
    "        sim.setObjectMatrix(dummy, flange, womatToolFrameMatrix(sim.getObjectPosition(dummy, flange)))",
    "        report.repaired[#report.repaired+1]='rebuilt tool control point'",
    "        report.marker=describe(dummy)",
    "        report.pads=pads",
    "    end",
    "    report.gripper=describe(gripper)",
    "    report.markerCount=1",
    "    report.gripperCount=1",
    "    report.duplicates={}",
    "end"
  ] : []).concat(orientTcp ? [
    "if #markers ~= 1 or flange == -1 then error('unique_tcp_and_flange_required') end",
    "local marker=markers[1]",
    "if sim.getObjectParent(marker) ~= flange then error('tcp_not_parented_to_flange') end",
    "local frame=womatToolFrameMatrix(sim.getObjectPosition(marker, flange))",
    "if frame == nil then error('tool_frame_degenerate') end",
    "sim.setObjectMatrix(marker, flange, frame)",
    "report.marker=describe(marker)",
    "report.repaired={'oriented TCP with Y toward jaws'}"
  ] : []).concat([
    "",
    "return report",
    "end)",
    "if not ok then return {error=tostring(result)} end",
    "return result"
  ]).join("\n");
}

function distance(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function show(v) {
  return v ? `(${v.map((c) => Number(c).toFixed(4).padStart(8)).join(", ")})` : "(none)";
}

async function main() {
  const args = parseArgs(process.argv.slice(2));
  if (!args.manifest) {
    console.error("Usage: npm run robot:inspect-tool -- --manifest generated/robots/<name>/manifest.json [--repair | --orient-tcp]");
    process.exit(1);
  }
  const manifest = JSON.parse(fs.readFileSync(path.resolve(process.cwd(), args.manifest), "utf8"));
  const robwotRoot = manifest.pipeline?.robwotRoot
    || path.resolve(__dirname, "..", "..", "external", "RobWoT");
  const { RemoteAPIClient } = require(require.resolve("coppelia-ws-api", { paths: [robwotRoot] }));
  const client = new RemoteAPIClient("localhost", 23050, "json");
  await client.websocket.open();
  const sim = await client.getObject("sim");

  const name = manifest.name;
  const grip = manifest.runtime?.gripper || {};
  const lua = buildInspectionLua({
    name,
    gripperAlias: grip.modelName || `${name}_tcp_gripper`,
    padAlias: String(grip.graspPadAlias ?? "touch").toLowerCase(),
    grip,
    orientTcp: args["orient-tcp"] === true,
    repair: args.repair === true
  });

  const sandbox = await sim.getScript(await sim.scripttype_sandboxscript);
  const handle = Array.isArray(sandbox) ? sandbox[0] : sandbox;
  const raw = await sim.executeScriptString(lua, handle);
  const report = Array.isArray(raw) ? (raw.length > 2 ? raw.slice(1) : raw[1]) : raw;

  if (!report || report.error) {
    console.error(`Could not inspect ${name}: ${report?.error || "no report returned"}`);
    process.exit(1);
  }

  console.log(`\n${name}\n`);
  for (const [label, entry] of [
    ["tip (flange)", report.tip],
    ["gripper", report.gripper],
    [`${name}_tcp marker`, report.marker]
  ]) {
    console.log(`  ${label.padEnd(18)} ${entry ? show(entry.position) : "(absent)"}   parent: ${entry?.parent ?? "-"}`);
    if (entry?.matrix) console.log(`  ${"".padEnd(18)} Y axis: ${show([entry.matrix[1], entry.matrix[5], entry.matrix[9]])}`);
    if (entry?.chain) console.log(`  ${"".padEnd(18)} chain: ${entry.chain}`);
  }
  console.log(`  ${"contact pads".padEnd(18)} ${show(report.pads)}`);

  if (Number(report.gripperCount) > 1 || Number(report.markerCount) > 1) {
    console.log(`\n  DUPLICATES: ${report.gripperCount} gripper(s), ${report.markerCount} marker(s)`);
    for (const entry of report.duplicates || []) {
      console.log(`    extra ${String(entry.kind).padEnd(8)} ${show(entry.info?.position)}   parent: ${entry.info?.parent}`);
    }
    console.log("    An orphan from an earlier run looks exactly like a bad mount on screen.");
  }
  if (report.repaired) console.log(`\n  REPAIRED: ${report.repaired.join(", ")}`);

  const tip = report.tip?.position;
  const gripper = report.gripper?.position;
  const marker = report.marker?.position;
  console.log("");
  if (tip && gripper) console.log(`  flange -> gripper   ${distance(tip, gripper).toFixed(4)} m   (expect ~0: the gripper bolts to the flange)`);
  if (tip && report.pads) console.log(`  flange -> pads      ${distance(tip, report.pads).toFixed(4)} m   (expect ~one tool length)`);
  if (marker && report.pads) console.log(`  marker -> pads      ${distance(marker, report.pads).toFixed(4)} m   (expect ~0: the marker IS the tool control point)`);
  if (tip && marker) console.log(`  flange -> marker    ${distance(tip, marker).toFixed(4)} m   (expect ~one tool length)`);
  console.log("");

  await client.websocket.close();
}

if (require.main === module) {
  main().catch((error) => {
    console.error(error.message || error);
    process.exit(1);
  });
}

module.exports = { buildInspectionLua };
