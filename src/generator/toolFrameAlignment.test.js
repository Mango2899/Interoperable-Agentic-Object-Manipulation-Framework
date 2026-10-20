const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { execFileSync } = require('node:child_process');
const test = require('node:test');
const { setHandoverApproach } = require('./core/handover');

test('TCP frame is right handed with Y along the tool for arbitrary flange offsets', () => {
  const math = fs.readFileSync(path.join(__dirname, 'lua/00-tool-frame-math.lua'), 'utf8');
  const dir = fs.mkdtempSync('/tmp/womat-tool-frame-');
  const file = path.join(dir, 'test.lua');
  fs.writeFileSync(file, math + `
for _, tcp in ipairs({{0,0,0.19}, {0,0.19,0}, {-0.07,0.02,0.01}}) do
    local m=womatToolFrameMatrix(tcp)
    local length=math.sqrt(tcp[1]^2+tcp[2]^2+tcp[3]^2)
    for row=0,2 do
        assert(math.abs(m[row*4+2]-tcp[row+1]/length)<1e-9)
        assert(m[row*4+4]==tcp[row+1])
    end
    for a=1,3 do
        for b=1,3 do
            local dot=m[a]*m[b]+m[a+4]*m[b+4]+m[a+8]*m[b+8]
            assert(math.abs(dot-(a==b and 1 or 0))<1e-9)
        end
    end
    local cross={m[5]*m[10]-m[9]*m[6],m[9]*m[2]-m[1]*m[10],m[1]*m[6]-m[5]*m[2]}
    for row=0,2 do assert(math.abs(cross[row+1]-m[row*4+3])<1e-9) end
end
assert(womatToolFrameMatrix({0,0,0})==nil)
`);
  try {
    execFileSync('lua', [file], { encoding: 'utf8', timeout: 5000 });
  } finally {
    fs.rmSync(dir, { recursive: true, force: true });
  }
});

async function approach(peerToolAxis, mode = 'opposed') {
  const calls = [];
  const result = await setHandoverApproach(
    { manifest: { name: 'Robot4', runtime: { handoverApproachMode: mode, enableHandoverRollSearch: true } } },
    { executeScriptString: async (code) => {
      calls.push(code);
      return [1, { success: true, peerToolAxis }];
    } }, 1,
    { manifestOf: () => ({ name: 'Robot2', runtime: { gripper: { graspPadAlias: 'touch' } } }) },
    { get: () => ({ id: 'test', sourceRobot: 'Robot2', targetRobot: 'Robot4', state: 'registered', giverAxis: [1,0,0] }) },
    'test', true
  );
  return { calls, result };
}

test('receiver opposes live giver TCP after it turns since session registration', async () => {
  const { calls, result } = await approach([0, 0, 1]);
  assert.deepEqual(result.giverAxis, [0,0,1]);
  assert.ok(calls.some(code => code.includes('womatSetHandoverPoseLock(true, {0,0,-1}')));
  assert.ok(calls.some(code => code.includes('[[Robot2_tcp]]')));
});

test('missing live TCP fails a relative approach instead of using the stale axis', async () => {
  const { calls, result } = await approach(undefined);
  assert.equal(result.reason, 'handover_giver_tool_axis_unavailable');
  assert.ok(calls.some(code => code.includes('womatClearHandoverApproachGripper')));
  assert.ok(!calls.some(code => code.includes('womatSetHandoverPoseLock(true')));
});

test('explicit fixed-axis modes remain available without a peer TCP', async () => {
  const { calls, result } = await approach(undefined, 'horizontalX');
  assert.equal(result.approachMode, 'horizontalX');
  assert.ok(calls.some(code => code.includes('womatSetHandoverPoseLock(true, {1,0,0}')));
});


test('live peer direction reads TCP Y in world coordinates, excluding other flange children', () => {
  const source = fs.readFileSync(path.join(__dirname, 'lua/05-gripper.lua'), 'utf8');
  const body = source.slice(source.indexOf('function womatPeerToolAxisWorld('), source.indexOf('function womatClearHandoverApproachGripper('));
  const dir = fs.mkdtempSync('/tmp/womat-peer-frame-');
  const file = path.join(dir, 'test.lua');
  fs.writeFileSync(file, body + `
function womatPlainAlias(h) return h==3 and 'Robot2_tcp' or 'other' end
sim={handle_all=-2, handle_world=-1,
    getObjectParent=function(h) return 1 end,
    getObjectsInTree=function(root) assert(root==1) return {2,3} end,
    getObjectMatrix=function(h, relative)
        assert(h==3 and relative==-1)
        return {0,-1,0,10, 1,0,0,20, 0,0,1,30}
    end
}
local axis=womatPeerToolAxisWorld(4, 'Robot2_tcp')
assert(axis[1]==-1 and axis[2]==0 and axis[3]==0)
assert(womatPeerToolAxisWorld(4, 'missing')==nil)
assert(womatPeerToolAxisWorld(4, nil)==nil)
`);
  try {
    execFileSync('lua', [file], { encoding: 'utf8', timeout: 5000 });
  } finally {
    fs.rmSync(dir, { recursive: true, force: true });
  }
});
