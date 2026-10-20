-- Safety collections are built once and reused. Rebuilding them per call did
-- not only repeat the scene-graph enumeration above: destroying a collection
-- also discards CoppeliaSim's collision and distance caches for it, so every
-- check paid for a cold mesh query. The workspace-generation driver already
-- caches its obstacle collections the same way.
womatCachedObstacleCollection=nil
womatCachedRobotCollection=nil
womatCachedToolCollection=nil
womatCachedDistantLinkCollection=nil
-- Pair-level culling for the arm-against-cell contact test; see
-- womatCollectionsTouch. Off falls back to the collection check everywhere.
womatBroadphaseEnabled=${collisionBroadphaseLua}
womatBroadphaseShapes={}

function womatInvalidateSafetyCollections()
    -- Called whenever collection membership can actually change: grip
    -- ownership (the cube is parented into the robot tree), the handover
    -- gripper exemption, and tool or scene setup.
    if womatCachedClearanceCollection ~= nil then
        pcall(sim.destroyCollection, womatCachedClearanceCollection)
        womatCachedClearanceCollection=nil
    end
    if womatCachedObstacleCollection ~= nil then
        pcall(sim.destroyCollection, womatCachedObstacleCollection)
    end
    if womatCachedRobotCollection ~= nil then
        pcall(sim.destroyCollection, womatCachedRobotCollection)
    end
    if womatCachedToolCollection ~= nil then
        pcall(sim.destroyCollection, womatCachedToolCollection)
    end
    if womatCachedDistantLinkCollection ~= nil then
        pcall(sim.destroyCollection, womatCachedDistantLinkCollection)
    end
    if womatRuntimeGripperCollisionCollection ~= nil then
        pcall(sim.destroyCollection, womatRuntimeGripperCollisionCollection)
    end
    womatDropExemptCollections()
    -- Keyed by collection handle, and a destroyed handle can be handed out again.
    womatBroadphaseShapes={}
    womatCachedObstacleCollection=nil
    womatCachedRobotCollection=nil
    womatCachedToolCollection=nil
    womatCachedDistantLinkCollection=nil
    womatRuntimeGripperCollisionCollection=nil
    return true
end

-- Render meshes, and how to tell one from the body it depicts.
--
-- A URDF import brings in two shapes per link: the collision body the physics
-- and the collision tests are meant to use, and the render mesh drawn beside
-- it. Both land in the scene, both are shapes, and both were going into every
-- collision collection this driver builds -- so every check paid for the same
-- link twice, against geometry that is typically far denser than the collision
-- hull it duplicates.
--
-- Live cell: 109 obstacle shapes, 46 of them render meshes, and the robot's own
-- 32 down to 19. One sim.checkCollision of the arm against the cell went from
-- 70 ms to 23 ms with the duplicates dropped and the verdict unchanged. That
-- check is the inner loop of the pick search, which called it thousands of
-- times per plan -- see womatPickConfigFailure.
--
-- The name is the only reliable marker here: this scene imports every shape
-- non-respondable, so respondability cannot separate them, and the visibility
-- layers are model-specific.
function womatIsDecorativeShape(alias)
    local low=string.lower(alias)
    return string.find(low, "visual", 1, true) ~= nil
        or string.find(low, "visible", 1, true) ~= nil
end

function womatIsRenderOnlyShape(shape)
    -- Named like a render mesh AND demonstrably standing in for a body that is
    -- still in the collection. The second half is what makes dropping it safe:
    -- a link represented ONLY by a mesh with "visual" in its name is not a
    -- duplicate of anything, and removing it would remove a real obstacle. So
    -- it is kept, and only genuine duplicates are dropped.
    if not womatIsDecorativeShape(womatPlainAlias(shape)) then
        return false
    end
    local parent=sim.getObjectParent(shape)
    if parent == -1 then
        return false
    end
    if sim.getObjectType(parent) == sim.object_shape_type
        and not womatIsDecorativeShape(womatPlainAlias(parent)) then
        return true
    end
    for _, sibling in ipairs(sim.getObjectsInTree(parent, sim.object_shape_type, 0)) do
        if sibling ~= shape and not womatIsDecorativeShape(womatPlainAlias(sibling)) then
            return true
        end
    end
    return false
end

function womatObstacleCollection()
    if womatCachedObstacleCollection ~= nil then
        return womatCachedObstacleCollection
    end
    local obstacleShapes=womatSceneObstacleShapes()
    if #obstacleShapes == 0 then
        return -1
    end
    local obstacleCollection=sim.createCollection(0)
    for i=1,#obstacleShapes,1 do
        if not womatIsRenderOnlyShape(obstacleShapes[i]) then
            sim.addItemToCollection(obstacleCollection, sim.handle_single, obstacleShapes[i], 0)
        end
    end
    womatCachedObstacleCollection=obstacleCollection
    return obstacleCollection
end

-- Contact and clearance are different questions, and the floor is why.
--
-- A floor-mounted arm stands ON the floor: its base is permanently inside any
-- sensible clearance margin of it. Ask "is anything within 2 cm" against the
-- full obstacle set and the answer is yes at every pose the robot can adopt,
-- so every motion is refused. Ask "is anything touching" and the answer is
-- correctly no -- and stays correctly YES if the arm ever actually strikes the
-- floor.
--
-- So the surface a robot rests on is exempt from the CLEARANCE test and never
-- from the CONTACT test. It is detected rather than configured: at rest, an
-- obstacle inside the margin but not touching is something this arm is mounted
-- on or parked beside. Detection runs once, on the pose the scene loads in,
-- before anything has moved.
womatCachedClearanceCollection=nil
womatClearanceExemptAliases=nil

function womatDetectClearanceExempt()
    if womatClearanceExemptAliases ~= nil then
        return womatClearanceExemptAliases
    end
    local exempt={}
    local clearance=${collisionClearance}
    local robotCollection=womatRobotCollisionCollection()
    if clearance > 0 and robotCollection ~= -1 then
        local shapes=womatSceneObstacleShapes()
        for i=1,#shapes,1 do
            local single=sim.createCollection(0)
            sim.addItemToCollection(single, sim.handle_single, shapes[i], 0)
            local touching=sim.checkCollision(robotCollection, single) ~= 0
            local ok, near=pcall(sim.checkDistance, robotCollection, single, clearance)
            if not touching and ok and near ~= nil and near > 0 then
                exempt[#exempt+1]=womatPlainAlias(shapes[i])
            end
            pcall(sim.destroyCollection, single)
        end
    end
    womatClearanceExemptAliases=exempt
    return exempt
end

function womatIsClearanceExempt(shape)
    local exempt=womatDetectClearanceExempt()
    local alias=womatPlainAlias(shape)
    for i=1,#exempt,1 do
        if exempt[i] == alias then
            return true
        end
    end
    return false
end

function womatClearanceObstacleCollection()
    if womatCachedClearanceCollection ~= nil then
        return womatCachedClearanceCollection
    end
    local shapes=womatSceneObstacleShapes()
    local kept={}
    for i=1,#shapes,1 do
        if not womatIsClearanceExempt(shapes[i]) and not womatIsRenderOnlyShape(shapes[i]) then
            kept[#kept+1]=shapes[i]
        end
    end
    if #kept == 0 then
        return -1
    end
    local collection=sim.createCollection(0)
    for i=1,#kept,1 do
        sim.addItemToCollection(collection, sim.handle_single, kept[i], 0)
    end
    womatCachedClearanceCollection=collection
    return collection
end

function womatRobotObstacleCollision()
    local obstacleCollection=womatObstacleCollection()
    if obstacleCollection == -1 then
        return false
    end
    local robotCollection=womatRobotCollisionCollection()
    local colliding=womatCollectionsUnsafe(robotCollection, obstacleCollection)
    if not colliding then
        colliding=womatGripperClearanceUnsafe(obstacleCollection)
    end
    return colliding
end

womatRuntimeGripperCollisionCollection=nil

function womatGripperCollisionCollection()
    if womatRuntimeGripperCollisionCollection ~= nil then
        return womatRuntimeGripperCollisionCollection
    end
    womatRuntimeGripperCollisionCollection=sim.createCollection(0)
    local shapes=sim.getObjectsInTree(tcpGripper, sim.object_shape_type, 0)
    for i=1,#shapes,1 do
        if not womatIsRenderOnlyShape(shapes[i]) then
            sim.addItemToCollection(
                womatRuntimeGripperCollisionCollection,
                sim.handle_single,
                shapes[i],
                0
            )
        end
    end
    return womatRuntimeGripperCollisionCollection
end

function womatGripperClearanceUnsafe(obstacleCollection)
    local clearance=${gripperCollisionClearance}
    if womatPickupApproachEnabled then
        clearance=${pickupGripperClearance}
    end
    if clearance <= 0 or tcpGripper == nil or tcpGripper == -1 then
        return false
    end
    local gripperCollection=womatGripperCollisionCollection()
    local ok, result=pcall(sim.checkDistance, gripperCollection, obstacleCollection, clearance)
    return ok and result ~= nil and result > 0
end

function womatSelfCollisionLinkOf(object)
    -- The moving link a shape belongs to: the first joint above it. Shapes
    -- under the same joint move together and can never collide with each other.
    local current=sim.getObjectParent(object)
    while current ~= -1 and current ~= simBase do
        if sim.getObjectType(current) == sim.object_joint_type then
            return current
        end
        current=sim.getObjectParent(current)
    end
    return simBase
end

function womatPrepareSelfCollisionCollections()
    -- TWO collections, checked against each other -- not one collection checked
    -- against itself.
    --
    -- The self-against-itself form is governed by each object's
    -- collection_self_collision_indicator, and measured against CoppeliaSim
    -- 4.10 the rule is "same indicator => the pair IS checked", the opposite of
    -- what this driver assumed. Giving every link a distinct indicator
    -- therefore excluded every pair, and the self-collision test returned false
    -- for every posture the robot has ever held.
    --
    -- Two collections also express what a single integer per object cannot:
    -- check the tool against the arm WITHOUT checking the tool against itself.
    -- A gripper's own fingers and body touch each other in every posture.
    if womatCachedToolCollection ~= nil then
        return womatCachedToolCollection, womatCachedDistantLinkCollection
    end
    if tcpGripper == nil or tcpGripper == -1 or #jointHandles < 3 then
        return -1, -1
    end

    local tool=sim.createCollection(0)
    local toolShapes=0
    for _, shape in ipairs(sim.getObjectsInTree(tcpGripper, sim.object_shape_type, 0)) do
        sim.addItemToCollection(tool, sim.handle_single, shape, 0)
        toolShapes=toolShapes+1
    end

    -- The mount link and the one before it are skipped: the gripper body abuts
    -- the wrist by construction, so those two report contact in every posture.
    -- Measured, not assumed -- gripper against links 1..4 is clear at a valid
    -- pose, against 5 and 6 it is not.
    --
    -- Found by walking up from the gripper, not by indexing jointHandles. The
    -- mount link is wherever the tool is bolted on, which is not the same thing
    -- as the last CONTROLLABLE joint: lock an arm's final axis and it leaves
    -- jointHandles while the link below it still carries the gripper. The
    -- indexed form then skipped the wrong two links and reported the tool in
    -- permanent collision with the very link it is mounted to.
    local skip={}
    local womatMountLink=womatSelfCollisionLinkOf(tcpGripper)
    if womatMountLink ~= simBase then
        skip[womatMountLink]=true
        skip[womatSelfCollisionLinkOf(womatMountLink)]=true
    else
        -- No joint above the tool at all: fall back to the end of the chain.
        skip[jointHandles[#jointHandles]]=true
        skip[jointHandles[#jointHandles-1]]=true
    end

    local links=sim.createCollection(0)
    local linkShapes=0
    for _, shape in ipairs(sim.getObjectsInTree(simBase, sim.object_shape_type, 0)) do
        if womatIsRobotCollisionShape(shape)
            and not womatIsDescendantOf(shape, tcpGripper)
            and not womatIsDescendantOf(shape, heldGripObject)
            and not skip[womatSelfCollisionLinkOf(shape)] then
            sim.addItemToCollection(links, sim.handle_single, shape, 0)
            linkShapes=linkShapes+1
        end
    end

    if toolShapes == 0 or linkShapes == 0 then
        pcall(sim.destroyCollection, tool)
        pcall(sim.destroyCollection, links)
        return -1, -1
    end
    womatCachedToolCollection=tool
    womatCachedDistantLinkCollection=links
    return tool, links
end

function womatRobotSelfCollision()
    local tool, links=womatPrepareSelfCollisionCollections()
    if tool == -1 or links == -1 then
        return false
    end
    return sim.checkCollision(tool, links)~=0
end

function womatSelfCollisionPair()
    -- Only ever called on a rejection: names the two shapes actually touching,
    -- so a real fold of the arm can be told from a pair that is in permanent
    -- contact by construction and should never have been checked.
    local tool, links=womatPrepareSelfCollisionCollections()
    if tool == -1 or links == -1 then
        return nil, nil
    end
    local result, handles=sim.checkCollision(tool, links)
    if result == 0 or handles == nil or #handles < 2 then
        return nil, nil
    end
    return womatPlainAlias(handles[1]), womatPlainAlias(handles[2])
end

function womatRobotMotionUnsafe()
    return womatRobotObstacleCollision() or womatRobotSelfCollision()
end

-- In-motion watchdog: the actual guarantee for the executed trajectory.
--
-- Contact is tested on EVERY simulation step. Striding it is not a safe
-- economy: at 0.175 m/s and a 50 ms step the tip advances 8.75 mm per step, so
-- a stride of 4 leaves 35 mm between tests and the arm passes straight through
-- a 20 mm wall or a 45 mm cube without a single test landing inside it. What
-- made the original per-step check expensive was never the contact test -- it
-- was rebuilding three collections and running two full-mesh distance queries
-- every step. With the collections cached and the distance query off this
-- path, testing contact every step costs a couple of API calls.
--
-- The clearance margin is the one thing worth striding, since it is the dear
-- query; at stride 0 it is left to the pre-motion sweep entirely.
womatInMotionSafetyEnabled=${inMotionSafetyEnabledLua}
womatInMotionSafetyStride=${inMotionCollisionStride}
womatInMotionClearanceStride=${inMotionClearanceStride}
womatInMotionSafetyTick=0

function womatResetInMotionSafety()
    womatInMotionSafetyTick=0
    return true
end

function womatInMotionUnsafe()
    if not womatInMotionSafetyEnabled then
        return false
    end
    womatInMotionSafetyTick=womatInMotionSafetyTick+1
    if womatInMotionSafetyStride > 1 and (womatInMotionSafetyTick % womatInMotionSafetyStride) ~= 0 then
        return false
    end
    local obstacleCollection=womatObstacleCollection()
    if obstacleCollection == -1 then
        return false
    end
    local robotCollection=womatRobotCollisionCollection()
    if womatCollectionsTouch(robotCollection, obstacleCollection) then
        return true
    end
    if womatRobotSelfCollision() then
        return true
    end
    if womatInMotionClearanceStride > 0
        and (womatInMotionSafetyTick % womatInMotionClearanceStride) == 0 then
        return womatClearanceUnsafe(robotCollection, obstacleCollection)
    end
    return false
end

-- The posture a step COMMANDS, not the one the arm is in.
--
-- A dynamic joint is only given a target; the physics engine moves it during
-- the step that follows. Reading the joints right after commanding them
-- therefore reads the previous step's posture, so a watchdog that tests the
-- scene as it stands finds a contact only once the arm is already in it -- and
-- its rollback then holds the arm there. E3 measured 1 to 4 steps of contact
-- before every such stop. Testing the commanded posture instead stops the arm
-- one step before it would touch.
function womatCommandedConfig(handles)
    local config={}
    for i=1,#handles,1 do
        local ok, target=pcall(sim.getJointTargetPosition, handles[i])
        if sim.isDynamicallyEnabled(handles[i]) and ok and target ~= nil then
            config[i]=target
        else
            config[i]=sim.getJointPosition(handles[i])
        end
    end
    return config
end

-- The in-motion test, run on `config` as the preflight runs it on a sample:
-- joints placed, tested, and put back within the one call, so nothing is
-- simulated in the commanded posture.
function womatCommandedPostureUnsafe(config, handles)
    handles=handles or jointHandles
    local saved={}
    for i=1,#handles,1 do
        saved[i]=sim.getJointPosition(handles[i])
    end
    for i=1,#handles,1 do
        sim.setJointPosition(handles[i], config[i])
    end
    local ok, unsafe=pcall(womatInMotionUnsafe)
    for i=1,#handles,1 do
        sim.setJointPosition(handles[i], saved[i])
    end
    if not ok then
        error(unsafe)
    end
    return unsafe
end

function womatCollectionsUnsafe(robotCollection, obstacleCollection)
    -- Contact against everything.
    if womatCollectionsTouch(robotCollection, obstacleCollection) then
        return true
    end
    -- And against itself. The obstacle collection holds what the arm may not
    -- touch in the cell, never its own links, so a posture that folds the
    -- gripper into a link passes the test above. Every route the planner
    -- validates with this function is executed, and E3 (Section 4.3.2) showed
    -- that this is where the arm collided: the swept check of the preflight
    -- tests the same pair separately, so a planned route was held to a weaker
    -- standard than a straight line.
    if womatRobotSelfCollision() then
        return true
    end
    -- Clearance against everything this arm is not resting on.
    local clearance=${collisionClearance}
    if clearance > 0 and not womatPickupApproachEnabled then
        local clearanceCollection=womatClearanceObstacleCollection()
        if clearanceCollection ~= -1 then
            local ok, result=pcall(sim.checkDistance, robotCollection, clearanceCollection, clearance)
            if ok and result ~= nil and result > 0 then
                return true
            end
        end
    end
    return false
end

function womatRobotCollisionCollection()
    if womatCachedRobotCollection ~= nil then
        return womatCachedRobotCollection
    end
    local robotCollection=sim.createCollection(0)
    local shapes=sim.getObjectsInTree(simBase, sim.object_shape_type, 0)
    for i=1,#shapes,1 do
        if womatIsRobotCollisionShape(shapes[i]) then
            sim.addItemToCollection(robotCollection, sim.handle_single, shapes[i], 0)
        end
    end
    womatCachedRobotCollection=robotCollection
    return robotCollection
end

-- Whether any joint moves this shape.
--
-- Walk from the shape up to the model root: cross a joint and the shape is
-- driven by it, cross none and the shape is rigidly fixed to the world. The
-- base and its pedestal are the latter.
function womatIsJointDrivenShape(shape)
    local current=shape
    while current ~= -1 and current ~= simBase do
        if sim.getObjectType(current) == sim.object_joint_type then
            return true
        end
        current=sim.getObjectParent(current)
    end
    return false
end

function womatIsRobotCollisionShape(shape)
    local alias=womatPlainAlias(shape)
    local parent=sim.getObjectParent(shape)

    -- RobWoT workspace scenes contain a generated convex-hull visual (aliased
    -- workspace_hull by the WoMAT pipeline, "shape*" in older scenes) directly
    -- below the robot model. It is not a robot body link, but it can overlap
    -- walls and would otherwise make every joint motion look unsafe.
    if parent == simBase and (alias == "workspace_hull" or string.match(alias, "^shape")) then
        return false
    end

    -- A link no joint moves has a collision state that never changes, so
    -- testing it every step tests a constant. It matters because a
    -- floor-mounted arm's base RESTS ON the floor: once the scene is
    -- enumerated rather than configured, the floor is an obstacle, the base is
    -- permanently touching it, and every motion this robot could ever make is
    -- refused for a contact that was there before it started.
    --
    -- Nothing is lost by dropping it. The base cannot move into anything, and
    -- anything that moves into the base is caught by that mover's own check,
    -- which has this base in its obstacle set.
    if not womatIsJointDrivenShape(shape) then
        return false
    end

    -- The render mesh of a link whose collision body is already in the
    -- collection. Testing both is testing the same link twice.
    if womatIsRenderOnlyShape(shape) then
        return false
    end

    return true
end

function jointConfigCollisionCheck(targetConf)
    local currentConf={}
    for i=1,#jointHandles,1 do
        currentConf[i]=sim.getJointPosition(jointHandles[i])
        sim.setJointPosition(jointHandles[i], targetConf[i])
    end

    local colliding=womatRobotObstacleCollision()

    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], currentConf[i])
    end

    return colliding
end

function womatObstacleLocalBoxes()
    -- Axis-aligned obstacle bounds in robot-local coordinates, so the Node
    -- runtime can reject a commanded tip point or line without a round trip
    -- per sample. The grip support and the pick cube are deliberately left
    -- out: the demo requires the tool to descend onto them, and clearance
    -- around them is covered by the swept body preflight instead.
    local context=womatSceneObstacleContext()
    local shapes=womatSceneObstacleShapes()
    local boxes={}
    for i=1,#shapes,1 do
        local shape=shapes[i]
        if not womatIsDescendantOf(shape, context.gripSupport)
            and not womatIsDescendantOf(shape, context.pickObject) then
            local extents={
                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_x), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_x)},
                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_y), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_y)},
                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_z), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_z)}
            }
            local matrix=sim.getObjectMatrix(shape, sim.handle_world)
            local low={math.huge, math.huge, math.huge}
            local high={-math.huge, -math.huge, -math.huge}
            for cx=1,2,1 do
                for cy=1,2,1 do
                    for cz=1,2,1 do
                        local world=sim.multiplyVector(matrix, {extents[1][cx], extents[2][cy], extents[3][cz]})
                        local localCorner=womatWorldToLocal(world[1], world[2], world[3])
                        for axis=1,3,1 do
                            low[axis]=math.min(low[axis], localCorner[axis])
                            high[axis]=math.max(high[axis], localCorner[axis])
                        end
                    end
                end
            end
            boxes[#boxes+1]={
                alias=womatPlainAlias(shape),
                min={x=low[1], y=low[2], z=low[3]},
                max={x=high[1], y=high[2], z=high[3]}
            }
        end
    end
    return boxes
end

-- Swept validation of one command.
--
-- The sweep drives the scene's joints to each candidate posture and restores
-- them afterwards, so it MUST complete inside a single script call. Split
-- across several calls the simulator steps and renders in between, leaving the
-- arm standing in a sampled posture: it visibly swings, the controller fights
-- the displacement, and every later sample is measured from a corrupted state.
--
-- Staying atomic means staying cheap, because a long-blocking call raises
-- CoppeliaSim's modal abort dialog. Contact is therefore tested at every
-- sample, while the clearance margin -- a distance query over full meshes
-- against every other robot in the cell, an order of magnitude dearer -- runs
-- only at the pose the arm comes to rest in, or on a stride if one is set.
womatPreflightClearanceStride=${preflightClearanceStride}

function womatClearanceUnsafe(robotCollection, obstacleCollection)
    local clearance=${collisionClearance}
    if clearance > 0 and not womatPickupApproachEnabled then
        -- Measured against the obstacles this arm is not resting on. Contact is
        -- still tested against every obstacle, by the caller.
        local clearanceCollection=womatSweepClearanceCollection()
        if clearanceCollection ~= -1 then
            local ok, result=pcall(sim.checkDistance, robotCollection, clearanceCollection, clearance)
            if ok and result ~= nil and result > 0 then
                return true
            end
        end
    end
    return womatGripperClearanceUnsafe(obstacleCollection)
end

function womatSweepSampleUnsafe(index, isFinal)
    local obstacleCollection=womatSweepObstacleCollection()
    if obstacleCollection == -1 then
        return nil
    end
    local robotCollection=womatRobotCollisionCollection()
    if womatCollectionsTouch(robotCollection, obstacleCollection) then
        return "obstacle_contact"
    end
    if womatRobotSelfCollision() then
        return "self_collision"
    end
    local strided=womatPreflightClearanceStride > 0 and (index % womatPreflightClearanceStride) == 0
    if (isFinal or strided) and womatClearanceUnsafe(robotCollection, obstacleCollection) then
        return "clearance_margin"
    end
    return nil
end

-- Peers a sweep is ordered BEHIND, by scene root handle.
--
-- An ordered reservation lets an arm start while an earlier arm still holds
-- part of its path: playback never enters a voxel the earlier arm holds, and
-- the earlier arm never takes a new one. Where that arm is standing NOW is
-- therefore not an obstacle to the path -- it will have left every place this
-- path goes before this arm gets there -- and testing against it refuses
-- exactly the overlap the ordering exists to allow. Every other shape, idle
-- arms included, is still tested: an arm that holds no reservation has
-- promised nothing about where it will be.
--
-- Set for the duration of one script call and cleared before it returns.
-- Script calls and actuation never interleave, so the in-motion watchdog,
-- which runs between calls, always sees the whole cell.
womatSweepExemptRoots=nil
womatSweepExemptKey=nil
womatCachedExemptCollections={}

function womatRootOf(object)
    local root=object
    local current=sim.getObjectParent(object)
    while current ~= -1 do
        root=current
        current=sim.getObjectParent(current)
    end
    return root
end

-- The handle other arms name this one by. Every robot runs in the same scene,
-- so a handle means the same object to all of them.
function womatSceneRootHandle()
    return womatRootOf(simBase)
end

function womatSetSweepExemption(roots)
    womatSweepExemptRoots=nil
    womatSweepExemptKey=nil
    if roots == nil or #roots == 0 then
        return
    end
    local set={}
    local keys={}
    for i=1,#roots,1 do
        local handle=math.floor(roots[i])
        if not set[handle] then
            set[handle]=true
            keys[#keys+1]=handle
        end
    end
    table.sort(keys)
    womatSweepExemptRoots=set
    womatSweepExemptKey=table.concat(keys, ",")
end

function womatIsSweepExempt(shape)
    return womatSweepExemptRoots ~= nil and womatSweepExemptRoots[womatRootOf(shape)] == true
end

function womatDropExemptCollections()
    for _, collection in pairs(womatCachedExemptCollections or {}) do
        if collection ~= -1 then
            pcall(sim.destroyCollection, collection)
        end
    end
    womatCachedExemptCollections={}
    womatBroadphaseShapes={}
end

-- Contact between two collections, answered pair by pair.
--
-- Measured in the live five-robot cell, sim stopped: one checkCollision of the
-- arm against the cell costs about 26 ms -- 19 arm shapes against 81 obstacle
-- shapes, 17 us a pair, and FLAT: a shape four metres away costs the same as
-- one touching the arm. Removing whole objects that are out of reach removed
-- four or five of the 81 and saved nothing, because with a body margin every
-- arm is every other arm's neighbour. Only removing PAIRS helps.
--
-- So each shape's bounding box is carried into the world and only pairs whose
-- boxes overlap are asked, one shape against one shape. A shape lies inside
-- its box, so two shapes in contact always have overlapping boxes: no pair
-- that could touch is skipped, and the verdict is the collection check's.
-- Measured: 1.9 ms a pass, 0 to 6 surviving pairs, identical contact sets.
--
-- It matters twice over. The preflight runs this per sample, and the in-motion
-- watchdog runs it every 50 ms step for every moving arm, inside the one
-- simulation thread: two arms moving at once was 52 ms of collision work per
-- 50 ms step, which is why simultaneous motion ran slower than real time.
-- A shape's box in its OWN frame, the frame getObjectMatrix places.
--
-- Not objfloatparam_objbbox_*: those are expressed in the shape's bounding-box
-- frame, which for an imported mesh is centred on the mesh and turned against
-- the shape frame. Read as if they were shape-frame values, 62 of the 180
-- shapes of the five-robot cell got a box that did not contain their mesh --
-- the URDF links among them -- and the broad phase dropped real contacts
-- (E2: six missed contacts in 1000 configurations, three with no overlapping
-- box at all). The vertices are in the shape frame, so their extremes are the
-- tightest box there is. getShapeBB, which carries the box frame with it, is
-- the fallback.
function womatShapeLocalBox(handle)
    local ok, vertices=pcall(sim.getShapeMesh, handle)
    if ok and type(vertices) == "table" and #vertices >= 3 then
        local low={math.huge, math.huge, math.huge}
        local high={-math.huge, -math.huge, -math.huge}
        for i=1,#vertices,3 do
            for axis=1,3,1 do
                local value=vertices[i+axis-1]
                if value < low[axis] then low[axis]=value end
                if value > high[axis] then high[axis]=value end
            end
        end
        return low, high
    end
    local okBox, size, pose=pcall(sim.getShapeBB, handle)
    if okBox and type(size) == "table" and type(pose) == "table" then
        local m=sim.poseToMatrix(pose)
        local low={math.huge, math.huge, math.huge}
        local high={-math.huge, -math.huge, -math.huge}
        for _, sx in ipairs({-0.5, 0.5}) do
            for _, sy in ipairs({-0.5, 0.5}) do
                for _, sz in ipairs({-0.5, 0.5}) do
                    local p=sim.multiplyVector(m, {sx*size[1], sy*size[2], sz*size[3]})
                    for axis=1,3,1 do
                        if p[axis] < low[axis] then low[axis]=p[axis] end
                        if p[axis] > high[axis] then high[axis]=p[axis] end
                    end
                end
            end
        end
        return low, high
    end
    return nil, nil
end

function womatBroadphaseEntry(collection)
    womatBroadphaseShapes=womatBroadphaseShapes or {}
    local entry=womatBroadphaseShapes[collection]
    if entry ~= nil then
        return entry
    end
    -- Anything that cannot be bounded is answered by the collection check, so
    -- nothing unbounded is ever culled.
    entry=false
    local listed, handles=pcall(sim.getCollectionObjects, collection)
    if listed and type(handles) == "table" then
        local shapes={}
        local bounded=true
        for i=1,#handles,1 do
            local handle=handles[i]
            if sim.getObjectType(handle) == sim.object_shape_type then
                local ok, low, high=pcall(womatShapeLocalBox, handle)
                if not ok or low == nil or high == nil then
                    bounded=false
                    break
                end
                -- The box in the shape's own frame, as a centre and half sizes,
                -- which is all a rotation needs to be applied to.
                shapes[#shapes+1]={handle=handle,
                    center={(low[1]+high[1])/2, (low[2]+high[2])/2, (low[3]+high[3])/2},
                    half={(high[1]-low[1])/2, (high[2]-low[2])/2, (high[3]-low[3])/2}}
            end
        end
        if bounded then
            entry={shapes=shapes}
        end
    end
    if entry == false and not womatBroadphaseWarned then
        -- Correct either way, but a silent fallback would be a silent loss of
        -- the whole speed-up, so it is said once where the scene log shows it.
        womatBroadphaseWarned=true
        pcall(sim.addLog, sim.verbosity_warnings,
            "WoMAT: contact broadphase unavailable for a collection; using the collection check")
    end
    womatBroadphaseShapes[collection]=entry
    return entry
end

-- Each shape's box, rotated into the world and boxed again: the centre is
-- transformed, and each world half size is the sum of the local ones weighted
-- by the absolute rotation. That is the tightest axis-aligned box around the
-- rotated one, without transforming eight corners.
function womatWorldBoxes(entry)
    local boxes={}
    for i=1,#entry.shapes,1 do
        local shape=entry.shapes[i]
        local m=sim.getObjectMatrix(shape.handle, sim.handle_world)
        local c=shape.center
        local h=shape.half
        local box={}
        for row=0,2,1 do
            local r1, r2, r3=m[4*row+1], m[4*row+2], m[4*row+3]
            local mid=r1*c[1]+r2*c[2]+r3*c[3]+m[4*row+4]
            local reach=math.abs(r1)*h[1]+math.abs(r2)*h[2]+math.abs(r3)*h[3]
            box[row+1]=mid-reach
            box[row+4]=mid+reach
        end
        boxes[i]=box
    end
    return boxes
end

-- Slack on the box test, so rounding can only ever add a pair, never lose one.
womatBroadphaseMargin=0.001

function womatCollectionsTouch(collectionA, collectionB)
    local a=womatBroadphaseEnabled and womatBroadphaseEntry(collectionA) or false
    local b=a and womatBroadphaseEntry(collectionB) or false
    if a and b then
        local ran, hit, handleA, handleB=pcall(function()
            local boxesA=womatWorldBoxes(a)
            local boxesB=womatWorldBoxes(b)
            local margin=womatBroadphaseMargin
            for i=1,#boxesA,1 do
                local p=boxesA[i]
                for j=1,#boxesB,1 do
                    local q=boxesB[j]
                    if p[1] <= q[4]+margin and q[1] <= p[4]+margin
                        and p[2] <= q[5]+margin and q[2] <= p[5]+margin
                        and p[3] <= q[6]+margin and q[3] <= p[6]+margin
                        and sim.checkCollision(a.shapes[i].handle, b.shapes[j].handle) ~= 0 then
                        return true, a.shapes[i].handle, b.shapes[j].handle
                    end
                end
            end
            return false
        end)
        if ran then
            return hit, handleA, handleB
        end
        -- A pair the simulator will not answer on its own: never guess, and
        -- never ask this collection pair-wise again.
        womatBroadphaseShapes[collectionA]=false
        womatBroadphaseShapes[collectionB]=false
    end
    return sim.checkCollision(collectionA, collectionB) ~= 0
end

-- Cached per exemption set, for the same reason the full collections are:
-- destroying a collection discards CoppeliaSim's collision caches for it.
-- There are only ever a handful of peers, so a handful of sets.
function womatExemptCollection(kind, include)
    local key=kind..":"..womatSweepExemptKey
    local cached=womatCachedExemptCollections[key]
    if cached ~= nil then
        return cached
    end
    local count=0
    for _ in pairs(womatCachedExemptCollections) do count=count+1 end
    if count >= 16 then
        womatDropExemptCollections()
    end
    local collection=-1
    local shapes=womatSceneObstacleShapes()
    for i=1,#shapes,1 do
        if include(shapes[i]) and not womatIsSweepExempt(shapes[i]) then
            if collection == -1 then
                collection=sim.createCollection(0)
            end
            sim.addItemToCollection(collection, sim.handle_single, shapes[i], 0)
        end
    end
    womatCachedExemptCollections[key]=collection
    return collection
end

function womatSweepObstacleCollection()
    if womatSweepExemptKey == nil then
        return womatObstacleCollection()
    end
    return womatExemptCollection("contact", function(shape)
        return not womatIsRenderOnlyShape(shape)
    end)
end

function womatSweepClearanceCollection()
    if womatSweepExemptKey == nil then
        return womatClearanceObstacleCollection()
    end
    return womatExemptCollection("clearance", function(shape)
        return not womatIsClearanceExempt(shape) and not womatIsRenderOnlyShape(shape)
    end)
end

function womatObstacleOwnerAlias(shape)
    -- Link aliases repeat across robots ("link4_respondable" belongs to every
    -- arm in the cell), so a bare alias does not say which one was hit. Walk to
    -- the tree root and name that instead.
    local root=shape
    local current=sim.getObjectParent(shape)
    while current ~= -1 do
        root=current
        current=sim.getObjectParent(current)
    end
    return womatPlainAlias(root)
end

function womatBlockingObstacleAlias()
    -- Only ever called once, on a rejection: names the obstacle the arm is
    -- actually touching so a real block can be told from a false positive.
    local robotCollection=womatRobotCollisionCollection()
    local shapes=womatSceneObstacleShapes()
    for i=1,#shapes,1 do
        if womatIsSweepExempt(shapes[i]) then goto continue end
        local single=sim.createCollection(0)
        sim.addItemToCollection(single, sim.handle_single, shapes[i], 0)
        local hit=sim.checkCollision(robotCollection, single)~=0
        sim.destroyCollection(single)
        if hit then
            return womatPlainAlias(shapes[i]), womatObstacleOwnerAlias(shapes[i])
        end
        ::continue::
    end
    return nil, nil
end

function womatRestoreSweepPosture(state)
    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], state.joints[i])
        pcall(sim.setJointTargetPosition, jointHandles[i], state.targets[i])
    end
    sim.setObjectPose(simTarget, sim.handle_world, state.targetPose)
end

function womatCaptureSweepPosture()
    local saved={}
    local savedTargets={}
    for i=1,#jointHandles,1 do
        saved[i]=sim.getJointPosition(jointHandles[i])
        local ok, target=pcall(sim.getJointTargetPosition, jointHandles[i])
        savedTargets[i]=(ok and target ~= nil) and target or saved[i]
    end
    return {joints=saved, targets=savedTargets, targetPose=sim.getObjectPose(simTarget, sim.handle_world)}
end

function womatRunSweep(count, apply, includeGoalClearance, sampleOffset)
    -- Restoration is protected: an error mid-sweep must not leave the arm
    -- parked in a candidate posture.
    local state=womatCaptureSweepPosture()
    local blockedAt=nil
    local mode=nil
    local ok, err=pcall(function()
        for step=1,count,1 do
            apply(step)
            -- Internal batches are not new goals. Keep clearance strides on
            -- the complete path's sample indices across remote calls.
            mode=womatSweepSampleUnsafe((sampleOffset or 0)+step, includeGoalClearance ~= false and step >= count)
            if mode ~= nil then
                blockedAt=step
                return
            end
        end
    end)
    local obstacle=nil
    local obstacleOwner=nil
    local selfA=nil
    local selfB=nil
    if blockedAt ~= nil and mode == "obstacle_contact" then
        obstacle, obstacleOwner=womatBlockingObstacleAlias()
    elseif blockedAt ~= nil and mode == "self_collision" then
        selfA, selfB=womatSelfCollisionPair()
    end
    womatRestoreSweepPosture(state)
    if not ok then
        return {ok=false, reason="path_sweep_failed", message=tostring(err)}
    end
    if blockedAt ~= nil then
        return {
            ok=false, blockedAtStep=blockedAt, mode=mode,
            obstacle=obstacle, obstacleOwner=obstacleOwner,
            selfCollisionA=selfA, selfCollisionB=selfB
        }
    end
    return {ok=true, checked=count}
end

-- The same sweep, resumable, bounded by TIME rather than by sample count.
--
-- What a preflight costs is the mesh queries, and in the live cell those are
-- about 25 ms per sample -- against 0.6 ms for a sample of forward kinematics,
-- roughly forty times more. Sizing a call in SAMPLES therefore sizes how long
-- the entire simulator stops, because a script string runs in the simulation
-- thread: 48 samples is 1.3 seconds in which no arm moves, no physics steps
-- and no other core is answered, which is visible as every robot in the cell
-- stuttering whenever one of them prepares a path.
--
-- Sized in time, a call costs what it says whatever the meshes cost, and the
-- number of calls follows from that instead of being guessed. The path is
-- handed over once and walked in slices; the posture is captured and restored
-- around each slice, exactly as one whole sweep did.
womatPreflightJob=nil

function womatContinuePreflight()
    local job=womatPreflightJob
    if job == nil then return {ok=false, reason="preflight_job_missing"} end
    local dof=#jointHandles
    local count=#job.configs
    local state=womatCaptureSweepPosture()
    local began=os.clock()
    local mode=nil
    womatSetSweepExemption(job.exemptRoots)
    local ok, err=pcall(function()
        repeat
            job.index=job.index+1
            local config=job.configs[job.index]
            for i=1,dof,1 do
                sim.setJointPosition(jointHandles[i], config[i])
            end
            mode=womatSweepSampleUnsafe(job.offset+job.index,
                job.includeGoalClearance and job.index >= count)
        until mode ~= nil or job.index >= count or os.clock()-began >= job.budget
    end)
    local obstacle=nil
    local obstacleOwner=nil
    local selfA=nil
    local selfB=nil
    if ok and mode == "obstacle_contact" then
        local named, a, b=pcall(womatBlockingObstacleAlias)
        if named then obstacle, obstacleOwner=a, b end
    elseif ok and mode == "self_collision" then
        local named, a, b=pcall(womatSelfCollisionPair)
        if named then selfA, selfB=a, b end
    end
    womatSetSweepExemption(nil)
    womatRestoreSweepPosture(state)
    if not ok then
        womatPreflightJob=nil
        return {ok=false, reason="path_sweep_failed", message=tostring(err)}
    end
    if mode ~= nil then
        womatPreflightJob=nil
        return {
            ok=false, blockedAtStep=job.index, mode=mode,
            obstacle=obstacle, obstacleOwner=obstacleOwner,
            selfCollisionA=selfA, selfCollisionB=selfB,
            sliceCpuMs=(os.clock()-began)*1000
        }
    end
    local pending=job.index < count
    if not pending then womatPreflightJob=nil end
    return {ok=true, pending=pending, checked=job.index, total=count,
        sliceCpuMs=(os.clock()-began)*1000}
end

function womatBeginPreflight(configs, includeGoalClearance, sampleOffset, budgetSeconds, exemptRoots)
    womatPreflightJob=nil
    if configs == nil or #configs == 0 then
        return {ok=false, reason="no_configurations"}
    end
    womatPreflightJob={configs=configs, index=0, offset=sampleOffset or 0,
        includeGoalClearance=includeGoalClearance ~= false,
        budget=(budgetSeconds ~= nil and budgetSeconds > 0) and budgetSeconds or 0.1,
        exemptRoots=exemptRoots}
    return womatContinuePreflight()
end

-- Where this arm's links actually go, along a path it is about to travel.
--
-- The claim a robot makes on the shared space is computed from this. A tip
-- polyline is not enough: the elbow bows out well clear of the line the tool
-- follows, and a claim that misses it lets another arm reserve space this one
-- is about to sweep through. Reporting every joint origin plus the tool point
-- gives a polyline per sample whose segments follow the links themselves.
--
-- Runs inside one script call, restoring the posture it borrowed, for the same
-- reason the collision sweeps do: the robot must not be observed mid-sample.
function womatSweepFramePositions(configs)
    local dof=#jointHandles
    if dof == 0 or configs == nil or #configs == 0 then
        return {ok=false, reason="no_configurations"}
    end
    local count=#configs/dof
    local state=womatCaptureSweepPosture()
    local frames={}
    for sample=1,count,1 do
        for joint=1,dof,1 do
            sim.setJointPosition(jointHandles[joint], configs[(sample-1)*dof+joint])
        end
        local points={}
        for joint=1,dof,1 do
            local position=sim.getObjectPosition(jointHandles[joint], sim.handle_world)
            points[#points+1]=position[1]
            points[#points+1]=position[2]
            points[#points+1]=position[3]
        end
        local tip=womatToolControlPointWorld()
        if tip ~= nil then
            points[#points+1]=tip[1]
            points[#points+1]=tip[2]
            points[#points+1]=tip[3]
        end
        frames[#frames+1]=points
    end
    womatRestoreSweepPosture(state)
    return {ok=true, samples=count, pointsPerSample=(#frames[1])/3, frames=frames}
end

-- The same question for the pose the arm is standing in right now, which is
-- what a Cartesian command needs before anything has been planned in joint
-- space.
function womatCurrentFramePositions()
    local dof=#jointHandles
    local points={}
    for joint=1,dof,1 do
        local position=sim.getObjectPosition(jointHandles[joint], sim.handle_world)
        points[#points+1]=position[1]
        points[#points+1]=position[2]
        points[#points+1]=position[3]
    end
    local tip=womatToolControlPointWorld()
    if tip ~= nil then
        points[#points+1]=tip[1]
        points[#points+1]=tip[2]
        points[#points+1]=tip[3]
    end
    return {ok=true, points=points}
end

function womatSweepCartesianPath(points)
    -- The same position-only IK environment and target dummy that
    -- womatSimpleMoveToPose will drive, walked in order from the current
    -- posture so each solve is seeded exactly as execution would seed it.
    if not womatEnsureSimplePositionIk() then
        return {ok=false, reason="simple_position_ik_setup_failed", message=womatMotionError}
    end
    local result=womatRunSweep(#points, function(step)
        local point=points[step]
        womatApplyCartesianTarget(point[1], point[2], point[3])
    end)
    if result.ok == false and result.reason == nil then
        result.reason="simple_cartesian_path_collision_detected"
    end
    return result
end

function womatPreflightJointPath(configs, includeGoalClearance, sampleOffset, exemptRoots)
    womatSetSweepExemption(exemptRoots)
    local ran, result=pcall(womatRunSweep, #configs, function(step)
        for i=1,#jointHandles,1 do
            sim.setJointPosition(jointHandles[i], configs[step][i])
        end
    end, includeGoalClearance, sampleOffset)
    womatSetSweepExemption(nil)
    if not ran then
        return {ok=false, reason="path_sweep_failed", message=tostring(result)}
    end
    if result.ok == false and result.reason == nil then
        result.reason="joint_path_collision_detected"
    end
    return result
end

function womatPointObstacleCollision(x, y, z)
    -- Callers pass robot-local coordinates; obstacles live in world coordinates.
    local world=womatLocalToWorld(x, y, z)
    x=world[1]
    y=world[2]
    z=world[3]
    local obstacleShapes=womatSceneObstacleShapes()
    for i=1,#obstacleShapes,1 do
        local obstacle=obstacleShapes[i]
        local minX=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_x)
        local maxX=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_x)
        local minY=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_y)
        local maxY=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_y)
        local minZ=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_z)
        local maxZ=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_z)
        local m=sim.getObjectMatrix(obstacle, sim.handle_world)
        local inv=sim.getMatrixInverse(m)
        local p=sim.multiplyVector(inv, {x, y, z})
        if p[1] >= minX and p[1] <= maxX and p[2] >= minY and p[2] <= maxY and p[3] >= minZ and p[3] <= maxZ then
            return true
        end
    end
    return false
end

heldGripObject=-1
gripperClosed=false
tcpGripper=-1
