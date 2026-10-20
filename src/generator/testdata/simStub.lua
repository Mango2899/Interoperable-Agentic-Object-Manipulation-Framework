-- Minimal CoppeliaSim stand-in: enough scene graph to run the tool-frame
-- inspection offline. Positions only, identity rotations -- which is all the
-- flange/gripper/tool-control-point arithmetic depends on.
local scene = {}
local nextHandle = 0

local function add(alias, parent, localPos, kind)
  nextHandle = nextHandle + 1
  scene[nextHandle] = {
    alias=alias, parent=parent or -1,
    pos={localPos[1], localPos[2], localPos[3]}, removed=false,
    isDummy=(kind == 'dummy'), isShape=(kind == 'shape')
  }
  return nextHandle
end

local function worldPos(h)
  local x, y, z = 0, 0, 0
  local current = h
  while current ~= nil and current ~= -1 do
    local o = scene[current]
    x = x + o.pos[1]; y = y + o.pos[2]; z = z + o.pos[3]
    current = o.parent
  end
  return {x, y, z}
end

sim = {
  handle_world = -1, handle_scene = -2, handle_all = -3,
  object_shape_type = 1, object_dummy_type = 2, object_joint_type = 3,
  shapeintparam_static = 20, shapeintparam_respondable = 21,
  objectspecialproperty_collidable = 1, jointmode_kinematic = 0,
  objfloatparam_objbbox_min_x = 10, objfloatparam_objbbox_max_x = 11,
  objfloatparam_objbbox_min_y = 12, objfloatparam_objbbox_max_y = 13,
  objfloatparam_objbbox_min_z = 14, objfloatparam_objbbox_max_z = 15,
  colorcomponent_ambient_diffuse = 0
}

function sim.getObject(path, opts)
  local wanted = string.gsub(path, '^/', '')
  for h, o in pairs(scene) do
    if not o.removed and o.alias == wanted then return h end
  end
  if opts and opts.noError then return -1 end
  error('object not found: ' .. path)
end
function sim.getObjectAlias(h) local o = scene[h]; return o and o.alias or nil end
function sim.getObjectParent(h) local o = scene[h]; return o and o.parent or -1 end
-- Used by the flange alignment to walk up to the wrist joint. The stub's scene
-- carries no joints, so the walk falls through to the parent link exactly as it
-- does on a model whose chain ends in one.
function sim.getObjectType(h)
  local o = scene[h]
  if o == nil then return -1 end
  if o.kind == 'joint' then return sim.object_joint_type end
  if o.kind == 'dummy' then return sim.object_dummy_type end
  return sim.object_shape_type
end
function sim.getObjectPosition(h) return worldPos(h) end
local function isDescendant(handle, root)
  -- handle_world/handle_scene both mean "the whole scene" as a tree root.
  if root == nil or root == -1 or root == -2 then return true end
  local current = handle
  while current ~= nil and current ~= -1 do
    if current == root then return true end
    current = scene[current].parent
  end
  return false
end

function sim.getObjectsInTree(base, objectType)
  local out = {}
  for h, o in pairs(scene) do
    -- Scoped to the subtree, as the real call is: an orphan left at the scene
    -- root by an earlier run must not answer a search of the robot's own tree.
    if not o.removed and isDescendant(h, base) then
      -- Only dummies have no pad alias and no children in this stub, which is
      -- enough for the flange search; shape queries filter on the alias.
      if objectType == sim.object_dummy_type then
        if o.isDummy then out[#out+1] = h end
      elseif objectType == sim.object_shape_type then
        if o.isShape then out[#out+1] = h end
      elseif objectType == sim.object_joint_type then
        if o.isJoint then out[#out+1] = h end
      else
        out[#out+1] = h
      end
    end
  end
  table.sort(out)
  return out
end
function sim.getObjectMatrix(h)
  local p = worldPos(h)
  return {1,0,0,p[1], 0,1,0,p[2], 0,0,1,p[3]}
end
function sim.getObjectFloatParam(h, param)
  -- A 1 cm pad, so its bounding-box midpoint is the shape origin.
  if param % 2 == 0 then return -0.005 end
  return 0.005
end
function sim.multiplyVector(m, v)
  return {m[1]*v[1]+m[2]*v[2]+m[3]*v[3]+m[4],
          m[5]*v[1]+m[6]*v[2]+m[7]*v[3]+m[8],
          m[9]*v[1]+m[10]*v[2]+m[11]*v[3]+m[12]}
end
function sim.setObjectParent(h, parent, keepInPlace)
  local before = worldPos(h)
  scene[h].parent = parent
  if keepInPlace then
    local parentWorld = (parent ~= -1) and worldPos(parent) or {0,0,0}
    scene[h].pos = {before[1]-parentWorld[1], before[2]-parentWorld[2], before[3]-parentWorld[3]}
  end
end
function sim.setObjectPosition(h, ref, pos)
  if pos == nil then pos = ref; ref = sim.handle_world end
  if ref == sim.handle_world then
    local parent = scene[h].parent
    local parentWorld = (parent ~= -1) and worldPos(parent) or {0,0,0}
    scene[h].pos = {pos[1]-parentWorld[1], pos[2]-parentWorld[2], pos[3]-parentWorld[3]}
  else
    scene[h].pos = {pos[1], pos[2], pos[3]}
  end
end
-- Orientation is tracked only well enough for the flange-alignment repair to
-- run against this stub. The stub's geometry is positional, so every object
-- starts square with the world; womatAlignFlangeToWrist then finds a wrist that
-- is square too and correctly leaves the frame alone.
function sim.getObjectOrientation(h, ref)
  local o = scene[h] and scene[h].orientation
  return {o and o[1] or 0, o and o[2] or 0, o and o[3] or 0}
end
function sim.setObjectOrientation(h, ref, o)
  if o == nil then o = ref end
  if scene[h] ~= nil and type(o) == 'table' then
    scene[h].orientation = {o[1] or 0, o[2] or 0, o[3] or 0}
  end
end
function sim.setObjectMatrix() end
function sim.multiplyMatrices(a) return a end
function sim.buildMatrix() return {1,0,0,0, 0,1,0,0, 0,0,1,0} end
function sim.createDummy(size) return add('dummy', -1, {0,0,0}, 'dummy') end
function sim.setObjectAlias(h, alias) scene[h].alias = alias end
function sim.setObjectColor() end
function sim.removeObject(h) scene[h].removed = true end
function sim.removeModel(h) scene[h].removed = true end
function sim.isHandle(h) return scene[h] ~= nil and not scene[h].removed end
function sim.loadModel(p) return add('loaded_model', -1, {0, 0, 0}) end
function sim.addLog() end
-- The shape/joint conditioning the pipeline does after mounting. None of it
-- moves anything, so no-ops are faithful enough for the frame arithmetic.
function sim.setObjectInt32Param() end
function sim.getObjectSpecialProperty() return 0 end
function sim.setObjectSpecialProperty() end
function sim.resetDynamicObject() end
function sim.getJointInterval() return false, {0, 0} end
function sim.setJointMode() end
function sim.setJointTargetForce() end
function sim.setJointTargetPosition() end
function sim.setJointPosition() end
sim.verbosity_scriptinfos = 1
sim.verbosity_errors = 2

-- The workspace driver reads simBase, which the vendor script sets before the
-- injection point.
function sim.getObjectsInTreeTyped() end

-- The broken scene the inspector actually reported: the gripper hangs off the
-- tool control point instead of the flange, so the pads land at twice the
-- tool length.
local TOOL = 0.1871
local robot = add('Robot1', -1, {0, 0, 0})
local link = add('link6_visual', robot, {-0.0025, 0.2434, 1.4283}, 'shape')
local tip = add('tip', link, {0, 0, 0}, 'dummy')
local marker = add('Robot1_tcp', tip, {0, TOOL, 0}, 'dummy')
local gripper = add('Robot1_tcp_gripper', marker, {0, 0, 0})
add('RG2_leftTouch', gripper, {0, TOOL, 0}, 'shape')
add('RG2_rightTouch', gripper, {0, TOOL, 0}, 'shape')

simBase = robot

return {scene=scene, worldPos=worldPos, tip=tip, gripper=gripper, TOOL=TOOL, robot=robot}
