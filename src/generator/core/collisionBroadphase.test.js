"use strict";
const test = require("node:test");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { execFileSync } = require("node:child_process");

// The driver's broadphase, run as Lua against a scene of spheres. A sphere's
// contact is exactly known -- centre distance against the sum of radii -- and
// its box in its own frame, rotated with it, must still contain it, so the
// culled answer can be held to the brute-force one on every random scene.
const source = fs.readFileSync(path.join(__dirname, "../lua/04-safety.lua"), "utf8");
const driver = source.slice(source.indexOf("function womatSweepSampleUnsafe("),
  source.indexOf("function womatPointObstacleCollision("));

const scene = `
sim={object_shape_type=0, handle_world=-1,
  objfloatparam_objbbox_min_x=1, objfloatparam_objbbox_min_y=2, objfloatparam_objbbox_min_z=3,
  objfloatparam_objbbox_max_x=4, objfloatparam_objbbox_max_y=5, objfloatparam_objbbox_max_z=6}
-- Each shape: a sphere of radius r at local centre c, placed by matrix m.
shapes={}
collections={}
pairChecks=0
collectionChecks=0
local function rotation(ax, ay, az)
  local cx, sx, cy, sy, cz, sz=math.cos(ax), math.sin(ax), math.cos(ay), math.sin(ay), math.cos(az), math.sin(az)
  return {cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx,
          sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx,
          -sy,   cy*sx,          cy*cx}
end
function place(handle, r, c, t, angles)
  local R=rotation(angles[1], angles[2], angles[3])
  shapes[handle]={r=r, c=c, m={R[1],R[2],R[3],t[1], R[4],R[5],R[6],t[2], R[7],R[8],R[9],t[3]}}
end
local function world(handle)
  local s=shapes[handle]
  local m=s.m
  return {m[1]*s.c[1]+m[2]*s.c[2]+m[3]*s.c[3]+m[4],
          m[5]*s.c[1]+m[6]*s.c[2]+m[7]*s.c[3]+m[8],
          m[9]*s.c[1]+m[10]*s.c[2]+m[11]*s.c[3]+m[12]}
end
local function touching(a, b)
  local p, q=world(a), world(b)
  local d=math.sqrt((p[1]-q[1])^2+(p[2]-q[2])^2+(p[3]-q[3])^2)
  return d <= shapes[a].r+shapes[b].r
end
function sim.getCollectionObjects(collection) return collections[collection] end
function sim.getObjectType() return 0 end
function sim.getObjectMatrix(handle) return shapes[handle].m end
-- The mesh, in the shape's own frame: the corners of the sphere's box.
function sim.getShapeMesh(handle)
  local s=shapes[handle]
  local vertices={}
  for i=0,7 do
    vertices[#vertices+1]=s.c[1]+((i%2==0) and -s.r or s.r)
    vertices[#vertices+1]=s.c[2]+((i%4<2) and -s.r or s.r)
    vertices[#vertices+1]=s.c[3]+((i<4) and -s.r or s.r)
  end
  return vertices
end
-- The objbbox parameters as CoppeliaSim reports them for an imported mesh: in
-- a bounding-box frame turned against the shape frame (here x and z swapped).
-- A box built from them does not contain the shape, so any code that still
-- reads them fails the equivalence test below.
function sim.getObjectFloatParam(handle, param)
  local s=shapes[handle]
  local axis=({3, 2, 1})[(param-1)%3+1]
  return param <= 3 and s.c[axis]-s.r or s.c[axis]+s.r
end
function sim.checkCollision(a, b)
  if collections[a] then
    collectionChecks=collectionChecks+1
    for _, x in ipairs(collections[a]) do
      for _, y in ipairs(collections[b]) do
        if touching(x, y) then return 1 end
      end
    end
    return 0
  end
  pairChecks=pairChecks+1
  return touching(a, b) and 1 or 0
end
${driver}
function bruteForce(a, b)
  for _, x in ipairs(collections[a]) do
    for _, y in ipairs(collections[b]) do
      if touching(x, y) then return true end
    end
  end
  return false
end
math.randomseed(7)
local function rand(lo, hi) return lo+(hi-lo)*math.random() end
`;

function runLua(t, name, body) {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-broadphase-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  fs.writeFileSync(path.join(dir, `${name}.lua`), scene + body);
  execFileSync("lua", [path.join(dir, `${name}.lua`)], { encoding: "utf8", timeout: 20000 });
}

test("a rotated shape's world box contains every corner of its own box, and no more", (t) => runLua(t, "boxes", `
for trial=1,500 do
  place(1, rand(0.01, 0.3), {rand(-0.2,0.2), rand(-0.2,0.2), rand(-0.2,0.2)},
    {rand(-3,3), rand(-3,3), rand(-1,2)}, {rand(-3.2,3.2), rand(-3.2,3.2), rand(-3.2,3.2)})
  collections={[1000]={1}}
  womatBroadphaseShapes={}
  local box=womatWorldBoxes(womatBroadphaseEntry(1000))[1]
  local s=shapes[1]
  local m=s.m
  local touchedLow, touchedHigh={false,false,false}, {false,false,false}
  for i=0,7 do
    local corner={s.c[1]+((i%2==0) and -s.r or s.r), s.c[2]+((i%4<2) and -s.r or s.r), s.c[3]+((i<4) and -s.r or s.r)}
    for row=0,2 do
      local v=m[4*row+1]*corner[1]+m[4*row+2]*corner[2]+m[4*row+3]*corner[3]+m[4*row+4]
      assert(v >= box[row+1]-1e-9 and v <= box[row+4]+1e-9, 'corner outside the world box')
      if math.abs(v-box[row+1]) < 1e-9 then touchedLow[row+1]=true end
      if math.abs(v-box[row+4]) < 1e-9 then touchedHigh[row+1]=true end
    end
  end
  for axis=1,3 do assert(touchedLow[axis] and touchedHigh[axis], 'the box is tight on every face') end
end
`));

test("culled contact agrees with brute force on every random scene, at a fraction of the pairs", (t) => runLua(t, "equivalence", `
womatBroadphaseEnabled=true
local asked, total, hits=0, 0, 0
for trial=1,400 do
  shapes={}
  womatBroadphaseShapes={}
  -- An arm of 19 shapes against a cell of 81, crowded enough that some touch.
  local arm, cell={}, {}
  for i=1,19 do
    place(i, rand(0.02, 0.08), {rand(-0.05,0.05), 0, 0}, {rand(0,1), rand(0,1), rand(0,1)},
      {rand(-3,3), rand(-3,3), rand(-3,3)})
    arm[#arm+1]=i
  end
  for i=100,180 do
    place(i, rand(0.02, 0.1), {0, rand(-0.05,0.05), 0}, {rand(-1.5,2.5), rand(-1.5,2.5), rand(-0.5,1.5)},
      {rand(-3,3), rand(-3,3), rand(-3,3)})
    cell[#cell+1]=i
  end
  collections={[1000]=arm, [2000]=cell}
  pairChecks=0
  collectionChecks=0
  local culled=womatCollectionsTouch(1000, 2000)
  assert(culled == bruteForce(1000, 2000), 'trial '..trial..' disagreed with brute force')
  assert(collectionChecks == 0, 'answered pair by pair, not by the collection')
  asked=asked+pairChecks
  total=total+#arm*#cell
  if culled then hits=hits+1 end
end
assert(hits > 20 and hits < 380, 'the scenes must exercise both verdicts, got '..hits)
assert(asked < total/10, 'pairs actually asked: '..asked..' of '..total)
`));

test("anything the broadphase cannot bound or ask falls back to the collection check", (t) => runLua(t, "fallback", `
place(1, 0.1, {0,0,0}, {0,0,0}, {0,0,0})
place(2, 0.1, {0,0,0}, {0.15,0,0}, {0,0,0})
collections={[1000]={1}, [2000]={2}}
-- Switched off: one collection check, as before.
womatBroadphaseEnabled=false
assert(womatCollectionsTouch(1000, 2000) == true and collectionChecks == 1 and pairChecks == 0)
womatBroadphaseEnabled=true
-- A simulator without collection listing.
collectionChecks=0
local list=sim.getCollectionObjects
sim.getCollectionObjects=nil
womatBroadphaseShapes={}
assert(womatCollectionsTouch(1000, 2000) == true and collectionChecks == 1)
sim.getCollectionObjects=list
-- A shape whose mesh cannot be read, and with no bounding box to fall back
-- to, is never culled.
local mesh=sim.getShapeMesh
sim.getShapeMesh=function() error('no mesh') end
womatBroadphaseShapes={}
collectionChecks=0
assert(womatCollectionsTouch(1000, 2000) == true and collectionChecks == 1)
sim.getShapeMesh=mesh
-- A pair the simulator refuses to answer: fall back now, and from then on.
local check=sim.checkCollision
sim.checkCollision=function(a, b)
  if not collections[a] then error('pair not collidable') end
  return check(a, b)
end
womatBroadphaseShapes={}
collectionChecks=0
assert(womatCollectionsTouch(1000, 2000) == true and collectionChecks == 1)
assert(womatBroadphaseShapes[1000] == false and womatBroadphaseShapes[2000] == false)
assert(womatCollectionsTouch(1000, 2000) == true and collectionChecks == 2)
sim.checkCollision=check
-- Apart, and culled without a single exact check.
place(2, 0.1, {0,0,0}, {0.5,0,0}, {0,0,0})
womatBroadphaseShapes={}
pairChecks=0
collectionChecks=0
assert(womatCollectionsTouch(1000, 2000) == false and pairChecks == 0 and collectionChecks == 0)
`));
