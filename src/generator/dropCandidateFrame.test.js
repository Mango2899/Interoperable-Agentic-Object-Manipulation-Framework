const assert = require('node:assert/strict');
const test = require('node:test');
const fs = require('node:fs');
const path = require('node:path');
const { execFileSync } = require('node:child_process');

test('face candidates preserve world release height for a tilted robot', () => {
  const source = fs.readFileSync(path.join(__dirname, 'lua/08-pick.lua'), 'utf8');
  const fn = source.slice(source.indexOf('function womatTopDownDropCandidates('), source.indexOf('function womatPlanTopDownPick('))
    .replace('${cubeSizeLua}', '{0.045,0.045,0.045}').replace('${placeEdgeMarginLua}', '0.005');
  const dir = fs.mkdtempSync('/tmp/womat-drop-frame-');
  const file = path.join(dir, 'test.lua');
  fs.writeFileSync(file, `
sim={handle_world=-1}
for _,axis in ipairs({'x','y','z'}) do
 sim['objfloatparam_objbbox_min_'..axis]='min_'..axis
 sim['objfloatparam_objbbox_max_'..axis]='max_'..axis
end
function sim.getObjectMatrix() return {} end
function sim.getObjectFloatParam(h,key)
 return string.sub(key,1,3)=='min' and -0.07 or 0.07
end
function sim.multiplyVector(m,p) return {4.33+p[1],0.28+p[2],0.08+p[3]} end
function womatObjectByAlias() return 1 end
function womatApproachSupportAlias() return 'support_b' end
-- A right-handed rotated frame: local Y is world vertical.
function womatLocalToWorld(x,y,z) return {z,x,y} end
function womatWorldToLocal(x,y,z) return {y,z,x} end
${fn}
local candidates=womatTopDownDropCandidates(0.28,0.4,4.33,2)
assert(#candidates==25)
for _,p in ipairs(candidates) do
 local w=womatLocalToWorld(p.x,p.y,p.z)
 assert(math.abs(w[3]-0.4)<1e-9, 'release height changed')
 assert(math.abs(w[1]-4.33)<0.05 and math.abs(w[2]-0.28)<0.05, 'outside support face')
end
assert(candidates[1].distance<1e-9, 'center must be first')
`);
  try { execFileSync('lua', [file], { encoding: 'utf8', timeout: 5000 }); }
  finally { fs.rmSync(dir, { recursive: true, force: true }); }
});
