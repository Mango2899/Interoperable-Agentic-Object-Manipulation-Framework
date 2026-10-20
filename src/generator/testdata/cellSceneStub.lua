-- A cell scene stand-in for the task-site inventory: two arms standing apart,
-- the loaded support in front of the first, an empty one in front of the
-- second, and the cube resting on the loaded one. Enough scene graph and
-- geometry to run womatSceneObjectInventory offline; no kinematics.
local scene = {}
local nextHandle = 0

local function add(alias, parent, localPos, kind, size)
  nextHandle = nextHandle + 1
  scene[nextHandle] = {
    alias=alias, parent=parent or -1,
    pos={localPos[1], localPos[2], localPos[3]}, removed=false,
    isDummy=(kind == 'dummy'), isShape=(kind == 'shape'), isJoint=(kind == 'joint'),
    size=size or {0.1, 0.1, 0.1}
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
  objectspecialproperty_collidable = 1,
  objfloatparam_objbbox_min_x = 10, objfloatparam_objbbox_max_x = 11,
  objfloatparam_objbbox_min_y = 12, objfloatparam_objbbox_max_y = 13,
  objfloatparam_objbbox_min_z = 14, objfloatparam_objbbox_max_z = 15,
  colorcomponent_ambient_diffuse = 0,
  verbosity_scriptinfos = 1, verbosity_errors = 2
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
function sim.getObjectPosition(h) return worldPos(h) end

local function isDescendant(handle, root)
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
    if not o.removed and isDescendant(h, base) then
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

-- Half-extents about the object origin, so the bounding box reports the size
-- the object was created with.
function sim.getObjectFloatParam(h, param)
  local size = scene[h] and scene[h].size or {0, 0, 0}
  local axis = math.floor((param - 10) / 2) + 1
  if param % 2 == 0 then return -size[axis] / 2 end
  return size[axis] / 2
end

function sim.getObjectMatrix(h)
  local p = worldPos(h)
  return {1,0,0,p[1], 0,1,0,p[2], 0,0,1,p[3]}
end
-- The real thing, not an identity: the world-to-local conversion under test is
-- pure rotation arithmetic, and a stub that rotates by nothing would report a
-- site on the wrong side of a yawed base and call it a pass. Row-major 3x4,
-- Rx*Ry*Rz, as CoppeliaSim builds it.
function sim.buildMatrix(position, euler)
  local px, py, pz = 0, 0, 0
  if position then px, py, pz = position[1], position[2], position[3] end
  local a, b, g = 0, 0, 0
  if euler then a, b, g = euler[1] or 0, euler[2] or 0, euler[3] or 0 end
  local ca, sa = math.cos(a), math.sin(a)
  local cb, sb = math.cos(b), math.sin(b)
  local cg, sg = math.cos(g), math.sin(g)
  return {
    cb*cg,              -cb*sg,              sb,     px,
    sa*sb*cg + ca*sg,   -sa*sb*sg + ca*cg,   -sa*cb, py,
    -ca*sb*cg + sa*sg,  ca*sb*sg + sa*cg,    ca*cb,  pz
  }
end
function sim.multiplyVector(m, v)
  return {m[1]*v[1]+m[2]*v[2]+m[3]*v[3]+m[4],
          m[5]*v[1]+m[6]*v[2]+m[7]*v[3]+m[8],
          m[9]*v[1]+m[10]*v[2]+m[11]*v[3]+m[12]}
end
function sim.setObjectAlias(h, alias) scene[h].alias = alias end
function sim.setObjectPosition(h, ref, pos)
  if pos == nil then pos = ref end
  local parent = scene[h].parent
  local parentWorld = (parent ~= -1) and worldPos(parent) or {0,0,0}
  scene[h].pos = {pos[1]-parentWorld[1], pos[2]-parentWorld[2], pos[3]-parentWorld[3]}
end
function sim.createPrimitiveShape(kind, size) return add('primitive', -1, {0,0,0}, 'shape', size) end
function sim.createPureShape(a, b, size) return add('primitive', -1, {0,0,0}, 'shape', size) end
function sim.setObjectOrientation() end
function sim.setObjectMatrix() end
function sim.setObjectParent(h, parent) scene[h].parent = parent end
function sim.setShapeColor() end
function sim.setObjectColor() end
function sim.removeObject(h) scene[h].removed = true end
function sim.isHandle(h) return scene[h] ~= nil and not scene[h].removed end
function sim.addLog() end
function sim.setObjectInt32Param() end
function sim.getObjectSpecialProperty() return 0 end
function sim.setObjectSpecialProperty() end
function sim.resetDynamicObject() end
function sim.getShapeBB(h) return scene[h].size end

-- Robot1 at the origin holding the loaded support and the cube; Robot4 2.9 m
-- away with an empty support of its own. Joints make each arm an articulated
-- body, which is how the inventory tells a peer robot from a fixture.
local robot1 = add('Robot1', -1, {0, 0, 0})
add('Robot1_link', robot1, {0, 0, 0.5}, 'shape', {0.2, 0.2, 1.0})
add('Robot1_joint', robot1, {0, 0, 0.2}, 'joint')

local robot4 = add('Robot4', -1, {2.9, -1.6, 0})
add('Robot4_link', robot4, {0, 0, 0.2}, 'shape', {0.15, 0.15, 0.4})
add('Robot4_joint', robot4, {0, 0, 0.1}, 'joint')

add('Robot1_grip_support', -1, {0.9, 0, 0.08}, 'shape', {0.14, 0.14, 0.16})
add('Robot1_grip_cube', -1, {0.9, 0, 0.185}, 'shape', {0.045, 0.045, 0.045})
add('Robot4_grip_support_b', -1, {2.65, -1.6, 0.08}, 'shape', {0.14, 0.14, 0.16})
add('Robot1_wall', -1, {0, 1.0, 0.4}, 'shape', {0.5, 0.05, 0.8})

simBase = robot1

return {scene=scene, worldPos=worldPos, robot1=robot1, robot4=robot4}
