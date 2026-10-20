function womatObjectByAlias(alias)
    if alias == nil or alias == "" then
        return -1
    end
    return sim.getObject('/'..alias, {noError=true})
end

function womatObjectByAliasInRobot(alias)
    -- Scoped to this robot's tree and tolerant of nesting. '/alias' is a
    -- root-level lookup, so it misses the gripper and the tool-control-point
    -- marker once they hang off the flange -- and a miss here is not an error,
    -- it loads or creates a SECOND object with the same alias, which
    -- CoppeliaSim then suffixes, leaving two candidates of which the wrong one
    -- is the one that gets measured.
    if alias == nil or alias == "" or simBase == nil or simBase == -1 then
        return womatObjectByAlias(alias)
    end
    local candidates=sim.getObjectsInTree(simBase, sim.handle_all, 0)
    for i=1,#candidates,1 do
        if womatPlainAlias(candidates[i]) == alias then
            return candidates[i]
        end
    end
    return womatObjectByAlias(alias)
end

function womatIsDescendantOf(object, root)
    if object == -1 or root == -1 then
        return false
    end
    local current=object
    while current ~= -1 do
        if current == root then
            return true
        end
        current=sim.getObjectParent(current)
    end
    return false
end

function womatPlainAlias(object)
    -- Alias option 4 appends "__<handle>__" when names repeat in the scene
    -- (unavoidable once several robots are spawned); strip it for comparisons.
    local alias=sim.getObjectAlias(object, 4)
    if alias == nil then
        return ""
    end
    return (string.gsub(alias, "__%d+__$", ""))
end

-- How the obstacle set is found.
--
-- "scene" enumerates every shape in the scene and excludes what is provably
-- mine; "configured" walks only the aliases the manifest names. Scene discovery
-- is the default because the backstop must not depend on anyone having listed
-- the right things: a neighbour discovered at runtime, or an object dropped
-- into the scene by hand, is an obstacle whether or not it was configured. A
-- watchdog that can only see what it was told about stops being a backstop
-- exactly when it matters.
womatObstacleDiscovery=${obstacleDiscoveryLua}
womatObstacleAliases=${obstacleLua}
womatObstacleIgnoreAliases=${obstacleIgnoreLua}

-- Task furniture beyond this robot's own pick support: the cell's other cube
-- supports. Every arm is given the same list because every arm has to be able
-- to RECOGNISE one -- which of them it can actually serve is decided later, by
-- its own workspace, not by this list.
womatTaskSiteDefinitions=${taskSitesLua}

function womatSceneObstacleContext()
    -- Resolved once per enumeration instead of once per shape: these alias
    -- lookups walk the scene graph, and the obstacle set can hold every shape
    -- of every other robot in a shared cell.
    local ignored={}
    for i=1,#womatObstacleIgnoreAliases,1 do
        local handle=womatObjectByAlias(womatObstacleIgnoreAliases[i])
        if handle ~= -1 then
            ignored[#ignored+1]=handle
        end
    end
    local taskSites={}
    for i=1,#womatTaskSiteDefinitions,1 do
        local handle=womatObjectByAlias(womatTaskSiteDefinitions[i].alias)
        if handle ~= -1 then
            taskSites[#taskSites+1]=handle
        end
    end
    return {
        pickObject=womatObjectByAlias(${cubeNameLua}),
        tcpTool=womatObjectByAlias(${gripperModelNameLua}),
        gripSupport=womatObjectByAlias(${supportNameLua}),
        approachSupport=womatObjectByAlias(womatApproachSupportAlias()),
        taskSites=taskSites,
        ignoredRoots=ignored
    }
end

function womatIsDeclaredTaskSite(shape, context)
    local sites=context.taskSites or {}
    for i=1,#sites,1 do
        if womatIsDescendantOf(shape, sites[i]) then
            return true
        end
    end
    return false
end

-- Where an object stands and how big it is, in both frames. World coordinates
-- so two robots can talk about the same object, local ones so this robot can
-- test the point against its own sampled workspace without converting twice.
function womatObjectPlacement(object)
    local world=sim.getObjectPosition(object, sim.handle_world)
    local localPosition=womatWorldToLocal(world[1], world[2], world[3])
    -- Two ways to ask, because neither works for everything: the bounding-box
    -- parameters answer for any object including a model root, and getShapeBB
    -- answers for a plain shape on installations where those parameters are
    -- not exposed. A site whose size cannot be measured is still reported --
    -- the reader falls back to a nominal footprint rather than losing the site.
    local size=nil
    local okParams, sx, sy, sz=pcall(function()
        local minX=sim.getObjectFloatParam(object, sim.objfloatparam_objbbox_min_x)
        local minY=sim.getObjectFloatParam(object, sim.objfloatparam_objbbox_min_y)
        local minZ=sim.getObjectFloatParam(object, sim.objfloatparam_objbbox_min_z)
        local maxX=sim.getObjectFloatParam(object, sim.objfloatparam_objbbox_max_x)
        local maxY=sim.getObjectFloatParam(object, sim.objfloatparam_objbbox_max_y)
        local maxZ=sim.getObjectFloatParam(object, sim.objfloatparam_objbbox_max_z)
        return maxX-minX, maxY-minY, maxZ-minZ
    end)
    if okParams and sx ~= nil and sy ~= nil and sz ~= nil then
        size={x=sx, y=sy, z=sz}
    else
        local okBB, bb=pcall(sim.getShapeBB, object)
        if okBB and bb ~= nil and bb[1] ~= nil then
            size={x=bb[1], y=bb[2], z=bb[3]}
        end
    end
    return {
        world={x=world[1], y=world[2], z=world[3]},
        localPosition={x=localPosition[1], y=localPosition[2], z=localPosition[3]},
        size=size
    }
end

function womatObstacleIgnored(shape, context)
    local roots=context.ignoredRoots or {}
    for i=1,#roots,1 do
        if womatIsDescendantOf(shape, roots[i]) then
            return true
        end
    end
    return false
end

function womatSceneObstacleCandidates()
    if womatObstacleDiscovery == "scene" then
        return sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    end
    local candidates={}
    for i=1,#womatObstacleAliases,1 do
        local root=womatObjectByAlias(womatObstacleAliases[i])
        if root ~= -1 then
            local shapes=sim.getObjectsInTree(root, sim.object_shape_type, 0)
            for j=1,#shapes,1 do
                candidates[#candidates+1]=shapes[j]
            end
        end
    end
    return candidates
end

function womatIsSceneObstacleShape(shape, obstacleContext)
    local context=obstacleContext or womatSceneObstacleContext()
    local pickObject=context.pickObject
    local tcpTool=context.tcpTool
    -- Workspace hull visuals (own or of another spawned robot) span the whole
    -- reachable volume and must never count as obstacles.
    local shapeAlias=womatPlainAlias(shape)
    if shapeAlias == "workspace_hull" or string.match(shapeAlias, "^shape") then
        return false
    end
    if womatIsDescendantOf(shape, simBase) then
        return false
    end
    if womatIsDescendantOf(shape, pickObject) then
        return false
    end
    if womatIsDescendantOf(shape, tcpTool) then
        return false
    end

    -- The pickup support remains a hard collision obstacle. Clearance is
    -- relaxed separately during the final approach, but physical contact is
    -- never ignored.

    -- During the receiver's final handover approach, ignore only the giver's
    -- gripper subtree. The giver arm, walls, support and every other obstacle
    -- remain active.
    if womatHandoverIgnoredGripper ~= nil and womatHandoverIgnoredGripper ~= -1 then
        if womatIsDescendantOf(shape, womatHandoverIgnoredGripper) then
            return false
        end
    end

    -- A freshly grasped cube is initially touching (and can be numerically
    -- intersecting) its support. Since the held cube becomes part of this
    -- robot's collision body, retaining that support as an obstacle makes an
    -- upward escape path fail at its first sample. Ignore only the configured
    -- support while this robot holds the cube; walls and other robots remain
    -- in the obstacle collection and still account for the carried cube.
    -- The same argument now covers placing. A cube being lowered onto a support
    -- ends in contact with it exactly as a freshly grasped one starts in
    -- contact with the one it came off, and the held cube is part of this
    -- arm's collision body either way. Only the ONE block the approach is
    -- aimed at is dropped -- every other support in the cell, including this
    -- arm's own when it is placing somewhere else, stays a hard obstacle.
    if heldGripObject ~= nil and heldGripObject ~= -1 then
        if context.gripSupport ~= -1 and womatIsDescendantOf(shape, context.gripSupport) then
            return false
        end
        if context.approachSupport ~= -1 and womatIsDescendantOf(shape, context.approachSupport) then
            return false
        end
    end
    return true
end

function womatSceneObstacleShapes()
    local context=womatSceneObstacleContext()
    local candidates=womatSceneObstacleCandidates()
    local obstacles={}
    for i=1,#candidates,1 do
        if womatIsSceneObstacleShape(candidates[i], context)
            and not womatObstacleIgnored(candidates[i], context) then
            obstacles[#obstacles+1]=candidates[i]
        end
    end
    return obstacles
end

-- What this robot believes is standing in the cell, and what kind of thing each
-- one is. The kinds are DERIVED, not configured: an articulated body is
-- something with joints in its tree, the payload is the object this robot picks
-- and the support is the thing it stands on. That matters because the planner
-- already treats them differently -- the payload is not an obstacle to whoever
-- is approaching or holding it, the support is not one during a pick -- and an
-- agent that is told "obstacle" for all of them cannot reason about any of it.
function womatSceneObjectInventory()
    local context=womatSceneObstacleContext()
    local shapes=sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    local seen={}
    local inventory={}
    for i=1,#shapes,1 do
        local shape=shapes[i]
        local root=shape
        local current=sim.getObjectParent(shape)
        while current ~= -1 do
            root=current
            current=sim.getObjectParent(current)
        end
        local alias=womatPlainAlias(root)
        if not seen[alias] then
            seen[alias]=true
            local kind="fixture"
            if root == simBase or womatIsDescendantOf(shape, simBase) then
                kind="self"
            elseif context.pickObject ~= -1 and womatIsDescendantOf(shape, context.pickObject) then
                kind="payload"
            elseif context.gripSupport ~= -1 and womatIsDescendantOf(shape, context.gripSupport) then
                kind="support"
            elseif womatIsDeclaredTaskSite(shape, context) then
                kind="support"
            elseif #sim.getObjectsInTree(root, sim.object_joint_type, 0) > 0 then
                kind="peer_robot"
            end
            local placement=womatObjectPlacement(root)
            inventory[#inventory+1]={
                alias=alias,
                kind=kind,
                shapes=#sim.getObjectsInTree(root, sim.object_shape_type, 0),
                ignored=womatObstacleIgnored(shape, context),
                world=placement.world,
                localPosition=placement.localPosition,
                size=placement.size
            }
        end
    end
    return {discovery=womatObstacleDiscovery, objects=inventory}
end

function womatSceneObstacleAliases()
    local shapes=womatSceneObstacleShapes()
    local aliases={}
    for i=1,#shapes,1 do
        aliases[i]=sim.getObjectAlias(shapes[i], 4)
    end
    return aliases
end

