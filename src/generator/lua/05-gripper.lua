womatCalibratedAttachLocalOffset=nil
womatGripperConfiguredOnce=false
womatHandoverIgnoredGripper=-1
womatHandoverIgnoredGripperAlias=nil
-- The substring that names the giver's contact pads. Its own manifest knows it
-- and this one cannot guess it -- the cell has RG2s whose pads are "touch" and
-- a Baxter gripper whose pads are "pad" -- so the receiver is told, and can
-- then measure which way the giver's jaws close. See womatClosingAxisOf.
womatHandoverIgnoredGripperPadAlias=nil
womatPickupApproachEnabled=false
-- Which block the top-down approach is allowed to touch. Picking from a support
-- and placing onto one are the same motion in opposite directions, and both end
-- with the cube resting against a solid body -- so both need that one body out
-- of the obstacle set, and neither needs any of the others out. Left unset it
-- is this robot's own pick support, which is what a pick has always meant.
womatPickupIgnoredSupportAlias=nil
-- Increase simulation step from 20 ms to 50 ms. Physics sub-steps stay at 10,
-- so each sub-step is 5 ms: sufficient for a robot arm. The main effect is
-- that the renderer fires at 20 Hz instead of 50 Hz, which delays the
-- Mesa/Gallium3D shader-state exhaustion crash from ~10 min to ~25 min.
pcall(sim.setSimulationTimeStep, 0.05)

function womatSetHandoverApproachGripper(alias, padAlias, tcpAlias)
    local gripper=womatObjectByAlias(alias)
    if gripper == -1 then
        return {success=false, reason="handover_gripper_missing", gripper=alias}
    end
    womatHandoverIgnoredGripper=gripper
    womatHandoverIgnoredGripperAlias=alias
    womatHandoverIgnoredGripperPadAlias=(padAlias ~= nil and padAlias ~= "") and padAlias or nil
    womatInvalidateSafetyCollections()
    return {
        success=true,
        state="handover_approach_enabled",
        ignoredGripper=alias,
        peerPadAlias=womatHandoverIgnoredGripperPadAlias,
        peerClosingAxis=womatPeerClosingAxisWorld(),
        peerToolAxis=womatPeerToolAxisWorld(gripper, tcpAlias)
    }
end

-- Read the giver at the rendezvous, after its wrist has moved. Its TCP's Y
-- is the tool direction; a global axis or the session's initial pose is stale.
function womatPeerToolAxisWorld(gripper, tcpAlias)
    if tcpAlias == nil or tcpAlias == "" then return nil end
    local flange=sim.getObjectParent(gripper)
    for _, handle in ipairs(sim.getObjectsInTree(flange, sim.handle_all, 0)) do
        if womatPlainAlias(handle) == tcpAlias and sim.getObjectParent(handle) == flange then
            local m=sim.getObjectMatrix(handle, sim.handle_world)
            return {m[2], m[6], m[10]}
        end
    end
    return nil
end

function womatClearHandoverApproachGripper()
    local previous=womatHandoverIgnoredGripperAlias
    womatHandoverIgnoredGripper=-1
    womatHandoverIgnoredGripperAlias=nil
    womatHandoverIgnoredGripperPadAlias=nil
    womatInvalidateSafetyCollections()
    return {success=true, state="handover_approach_disabled", ignoredGripper=previous}
end

function womatClosingAxisOf(root, padAlias)
    -- The line a gripper's jaws close along, in world coordinates: the
    -- direction between its two contact pads.
    --
    -- It is the second half of a gripper's orientation and the half nothing
    -- here used to look at. The tool axis says which way the gripper faces;
    -- this says which pair of the cube's faces it is holding, and at a handover
    -- that is what decides whether two grippers meeting at one cube take
    -- different faces or fight over the same ones.
    if root == nil or root == -1 or padAlias == nil or padAlias == "" then
        return nil
    end
    local shapes=sim.getObjectsInTree(root, sim.object_shape_type, 0)
    local pads={}
    for i=1,#shapes,1 do
        local alias=string.lower(womatPlainAlias(shapes[i]))
        -- The same decorative-mesh exclusion womatToolControlPointInFlange
        -- needs, and for the same reason: a duplicate render mesh hanging off
        -- a jaw link is not where the gripper touches anything.
        if string.find(alias, string.lower(padAlias), 1, true) ~= nil
            and not womatIsDecorativeShape(alias) then
            pads[#pads+1]=sim.getObjectPosition(shapes[i], sim.handle_world)
        end
    end
    if #pads < 2 then
        return nil
    end
    -- The two furthest apart, so a gripper with more than two matching shapes
    -- still measures across the jaws rather than between two parts of one.
    local best=nil
    local bestDistance=-1
    for i=1,#pads,1 do
        for j=i+1,#pads,1 do
            local d={pads[i][1]-pads[j][1], pads[i][2]-pads[j][2], pads[i][3]-pads[j][3]}
            local length=math.sqrt(d[1]*d[1]+d[2]*d[2]+d[3]*d[3])
            if length > bestDistance then
                bestDistance=length
                best=d
            end
        end
    end
    if best == nil or bestDistance < 1e-6 then
        return nil
    end
    return {best[1]/bestDistance, best[2]/bestDistance, best[3]/bestDistance}
end

function womatOwnClosingAxisWorld()
    return womatClosingAxisOf(tcpGripper, ${graspPadAliasLua})
end

function womatPeerClosingAxisWorld()
    return womatClosingAxisOf(womatHandoverIgnoredGripper, womatHandoverIgnoredGripperPadAlias)
end

function womatSetPickupApproach(enabled, siteAlias)
    womatPickupApproachEnabled=enabled == true
    if womatPickupApproachEnabled and siteAlias ~= nil and siteAlias ~= "" then
        womatPickupIgnoredSupportAlias=siteAlias
    else
        womatPickupIgnoredSupportAlias=nil
    end
    womatInvalidateSafetyCollections()
    return {
        success=true,
        state=womatPickupApproachEnabled and "pickup_approach_enabled" or "pickup_approach_disabled",
        enabled=womatPickupApproachEnabled,
        ignoredSupport=womatPickupApproachEnabled and womatApproachSupportAlias() or nil
    }
end

function womatApproachSupportAlias()
    if womatPickupIgnoredSupportAlias ~= nil and womatPickupIgnoredSupportAlias ~= "" then
        return womatPickupIgnoredSupportAlias
    end
    return ${supportNameLua}
end

function womatSetShapePhysics(root, isStatic, respondable)
    local shapes=sim.getObjectsInTree(root, sim.object_shape_type, 0)
    for i=1,#shapes,1 do
        sim.setObjectInt32Param(shapes[i], sim.shapeintparam_static, isStatic and 1 or 0)
        sim.setObjectInt32Param(shapes[i], sim.shapeintparam_respondable, respondable and 1 or 0)
        local okProperty, property=pcall(sim.getObjectSpecialProperty, shapes[i])
        if okProperty and property ~= nil then
            pcall(sim.setObjectSpecialProperty, shapes[i], property | sim.objectspecialproperty_collidable)
        end
        pcall(sim.resetDynamicObject, shapes[i])
    end
end

function womatCreateGripCuboid(alias, size, position, color, isStatic, respondable, yaw)
    -- Yaw only, about the site's own centre: these are boxes standing on the
    -- floor and the grasp point is the middle of the top face, which yaw does
    -- not move. Turning one changes what an arm has to reach around, not where
    -- it has to reach.
    local turn=yaw or 0
    local object=sim.getObject('/'..alias, {noError=true})
    if object ~= -1 then
        sim.setObjectPosition(object, sim.handle_world, position)
        sim.setObjectOrientation(object, sim.handle_world, {0, 0, turn})
        womatSetShapePhysics(object, isStatic, respondable)
        return object
    end

    local ok, handle=pcall(sim.createPrimitiveShape, sim.primitiveshape_cuboid, size, 0)
    if not ok or handle == nil or handle == -1 then
        handle=sim.createPureShape(0, 16, size, 0.05, nil)
    end

    sim.setObjectAlias(handle, alias)
    sim.setObjectPosition(handle, sim.handle_world, position)
    sim.setObjectOrientation(handle, sim.handle_world, {0, 0, turn})
    womatSetShapePhysics(handle, isStatic, respondable)
    pcall(sim.setShapeColor, handle, nil, sim.colorcomponent_ambient_diffuse, color)
    return handle
end

function womatRemoveGripperScripts()
    local all=sim.getObjectsInTree(tcpGripper, sim.handle_all, 0)
    all[#all+1]=tcpGripper
    for i=1,#all,1 do
        local h=sim.getScript(sim.scripttype_childscript, all[i])
        if h ~= nil and h ~= -1 then pcall(sim.removeScript, h) end
        h=sim.getScript(sim.scripttype_customizationscript, all[i])
        if h ~= nil and h ~= -1 then pcall(sim.removeScript, h) end
    end
end

function womatConfigureTcpGripperModel()
    if tcpGripper == -1 or womatGripperConfiguredOnce then
        return
    end

    -- Keep the stock RG2 scripts. This model exposes an attachment point but
    -- may drive its visible finger mechanism from scripts rather than joints.
    -- The runtime open/close signals are intended for those scripts.
    womatRemoveGeneratedVisualJaws()
    womatSetShapePhysics(tcpGripper, true, false)
    pcall(sim.resetDynamicObject, tcpGripper)
    womatGripperConfiguredOnce=true
end

function womatRemoveGeneratedVisualJaws()
    local aliases={
        ${gripperVisualJawLeftLua},
        ${gripperVisualJawRightLua}
    }
    for i=1,#aliases,1 do
        local object=sim.getObject('/'..aliases[i], {noError=true})
        if object ~= -1 then
            pcall(sim.removeObject, object)
        end
    end
end

function womatSafeCall(fn)
    local ok, result=pcall(fn)
    if not ok then
        return {success=false, reason="runtime_error", message=tostring(result)}
    end
    return result
end

function womatGripperJointInfo()
    if tcpGripper == -1 then
        return {count=0, positions={}}
    end

    local joints=sim.getObjectsInTree(tcpGripper, sim.object_joint_type, 0)
    local positions={}
    local aliases={}
    local intervals={}
    for i=1,#joints,1 do
        positions[i]=sim.getJointPosition(joints[i])
        aliases[i]=sim.getObjectAlias(joints[i], 4)
        local ok, cyclic, interval=pcall(sim.getJointInterval, joints[i])
        if ok and interval ~= nil then
            intervals[i]={cyclic=cyclic, low=interval[1], range=interval[2]}
        else
            intervals[i]={}
        end
    end
    return {count=#joints, positions=positions, aliases=aliases, intervals=intervals, drivenAliases=womatDrivenGripperJointAliases()}
end

function womatAliasLooksDriven(alias)
    local lower=string.lower(alias or "")
    return string.find(lower, "openclose") ~= nil
        or string.find(lower, "open_close") ~= nil
        or string.find(lower, "open%-close") ~= nil
        or string.find(lower, "finger") ~= nil
        or string.find(lower, "jaw") ~= nil
end

function womatDrivenGripperJoints()
    local joints=sim.getObjectsInTree(tcpGripper, sim.object_joint_type, 0)
    local driven={}
    for i=1,#joints,1 do
        local alias=sim.getObjectAlias(joints[i], 4)
        if womatAliasLooksDriven(alias) then
            driven[#driven+1]=joints[i]
        end
    end
    if #driven > 0 then
        return driven
    end

    for i=1,#joints,1 do
        local ok, cyclic, interval=pcall(sim.getJointInterval, joints[i])
        if ok and not cyclic and interval ~= nil and math.abs(interval[2]) > 0.001 and math.abs(interval[2]) < 0.2 then
            driven[#driven+1]=joints[i]
        end
    end
    if #driven > 0 then
        return driven
    end
    return joints
end

function womatDrivenGripperJointAliases()
    if tcpGripper == -1 then
        return {}
    end
    local joints=womatDrivenGripperJoints()
    local aliases={}
    for i=1,#joints,1 do
        aliases[i]=sim.getObjectAlias(joints[i], 4)
    end
    return aliases
end

function womatClamp(value, low, high)
    if value < low then
        return low
    end
    if value > high then
        return high
    end
    return value
end

function womatGripperJointTarget(joint, index, closed)
    local configured=${gripperOpenJointPositionsLua}
    if closed then
        configured=${gripperClosedJointPositionsLua}
    end
    local target=configured[((index-1) % #configured) + 1]
    local ok, cyclic, interval=pcall(sim.getJointInterval, joint)
    if ok and not cyclic and interval ~= nil and math.abs(interval[2]) > 0.0001 then
        local low=interval[1]
        local high=interval[1]+interval[2]
        if high < low then
            local tmp=low
            low=high
            high=tmp
        end
        if #configured == 1 then
            target=closed and low or high
        else
            target=womatClamp(target, low, high)
        end
    end
    return target
end

-- A gripper that actuates itself.
--
-- The Dobot's rod gripper is a four-bar linkage closed by dummy link pairs and
-- driven by two prismatic hand joints, and the model carries the script that
-- drives them: it reads a custom data block on the gripper root every step and
-- sets a target VELOCITY from it, so the jaws close until they meet something
-- and hold. Writing those joints from here as well is two controllers on one
-- mechanism -- ours would set a kinematic position, its would set a velocity,
-- and each step would undo the other. The block is the interface the mechanism
-- offers, so where one is named it is the only thing written.
function womatGripperActivityBlock()
    local name=${gripperActivityBlockLua}
    if name == nil or name == "" then
        return nil
    end
    return name
end

function womatSetTcpGripperActivity(closed)
    local block=womatGripperActivityBlock()
    if block == nil then
        return nil
    end
    local value=closed and ${gripperActivityClosedLua} or ${gripperActivityOpenLua}
    local ok=pcall(sim.writeCustomDataBlock, tcpGripper, block, value)
    return {block=block, value=value, written=ok}
end

function womatSetTcpGripperState(closed)
    if tcpGripper == -1 then
        return {success=false, reason="tcp_gripper_missing"}
    end

    local activity=womatSetTcpGripperActivity(closed)
    if activity ~= nil then
        -- The mechanism's own joints are left entirely alone: it is holding a
        -- velocity against whatever is between the jaws, and a position written
        -- over that is a command to let go.
        return {
            success=true,
            driver="activityDataBlock",
            activity=activity,
            signals=womatSetTcpGripperSignals(closed),
            closed=closed
        }
    end

    local signals=womatSetTcpGripperSignals(closed)
    local joints=womatDrivenGripperJoints()
    local targets={}
    local aliases={}
    for i=1,#joints,1 do
        local target=womatGripperJointTarget(joints[i], i, closed)
        targets[i]=target
        aliases[i]=sim.getObjectAlias(joints[i], 4)
        pcall(sim.setJointMode, joints[i], sim.jointmode_kinematic, 0)
        pcall(sim.setJointTargetForce, joints[i], 100)
        pcall(sim.setJointTargetVelocity, joints[i], closed and -0.05 or 0.05)
        pcall(sim.setJointTargetPosition, joints[i], target)
        pcall(sim.setJointPosition, joints[i], target)
    end

    return {success=true, driver="rg2Model", joints=#joints, aliases=aliases, targets=targets, signals=signals, closed=closed}
end

function womatSetTcpGripperSignals(closed)
    local openValue=closed and 0 or 1
    local closeValue=closed and 1 or 0
    local signals={
        {name="RG2_open", value=openValue},
        {name="RG2_close", value=closeValue},
        {name=${gripperModelNameLua}.."_open", value=openValue},
        {name=${gripperModelNameLua}.."_close", value=closeValue}
    }
    for i=1,#signals,1 do
        pcall(sim.setInt32Signal, signals[i].name, signals[i].value)
    end
    return signals
end

-- Where this robot's tool comes from. "model" is loaded from a file and bolted
-- to the flange; "builtin" is already a link of the arm's own chain; "none" is
-- no tool at all. See gripperSource in core/luaDriver.js.
womatGripperSource=${gripperSourceLua}

function womatEnsureTcpGripperModel()
    if womatGripperSource == "none" then
        -- Nothing to mount and nothing to measure: simTip stays the flange, and
        -- a grasp is the parenting it already was.
        tcpGripper=-1
        return {success=true, state="no_gripper_declared", model=nil}
    end

    local existing=womatObjectByAliasInRobot(${gripperModelNameLua})
    if womatGripperSource == "builtin" then
        if existing == -1 then
            existing=womatObjectByAliasInRobot(${gripperSourceAliasLua})
            if existing == -1 then
                return {success=false, reason="builtin_gripper_not_found", model=${gripperSourceAliasLua}}
            end
            sim.setObjectAlias(existing, ${gripperModelNameLua})
        end
        -- Every repair below is skipped on purpose. The tool is a LINK of the
        -- arm: removing it for having no joints deletes part of the robot, and
        -- reparenting it onto the flange dummy lifts it out of the kinematic
        -- chain -- after which the arm drives one link short and the tool is
        -- dragged along by a dummy that is no longer driven by anything.
        tcpGripper=existing
        local flange=womatResolveFlange()
        if flange == nil or flange == -1 then
            return {success=false, reason="flange_dummy_not_found", model=${gripperModelNameLua}}
        end
        womatFlangeTip=flange
        if not womatGripperConfiguredOnce then
            womatGripperConfiguredOnce=true
            womatInvalidateSafetyCollections()
        end
        womatEnsureToolControlPoint()
        return {
            success=true,
            state="builtin_gripper_adopted",
            model=${gripperModelNameLua},
            toolControlPointMode=womatToolControlPointMode
        }
    end

    if existing ~= -1 and not womatGripperConfiguredOnce then
        local existingJoints=sim.getObjectsInTree(existing, sim.object_joint_type, 0)
        if #existingJoints == 0 then
            -- Some older shared scenes contain a static placeholder carrying
            -- the correct alias but no RG2 mechanism. Replace it so open/close
            -- actions visibly drive real finger joints.
            local removed, result=pcall(sim.removeModel, existing)
            if not removed or result == false then
                pcall(sim.removeObject, existing)
            end
            existing=-1
        end
    end
    if existing == -1 then
        -- Robots spawned into a shared scene from their bare model have no
        -- gripper yet; load and alias it on first use.
        local okLoad, loaded=pcall(sim.loadModel, ${gripperModelPathLua})
        if okLoad and loaded ~= nil and loaded ~= -1 then
            sim.setObjectAlias(loaded, ${gripperModelNameLua})
            existing=loaded
        else
            return {success=false, reason="tcp_gripper_missing_in_scene", model=${gripperModelNameLua}}
        end
    end

    tcpGripper=existing
    local flange=womatResolveFlange()
    if flange == nil or flange == -1 then
        -- Refused rather than attempted: mounting against an unresolved flange
        -- silently leaves the gripper where it was.
        return {success=false, reason="flange_dummy_not_found", model=${gripperModelNameLua}}
    end
    -- A gripper hanging off anything but the flange is not a cosmetic problem:
    -- every pad measurement is taken through it, so a gripper mounted on its own
    -- tool control point reports jaws at twice the tool length. Checked on every
    -- call, because a stale scene can arrive already wired that way.
    local mountWrong=sim.getObjectParent(tcpGripper) ~= flange
    if not womatGripperConfiguredOnce or mountWrong then
        if mountWrong and womatPickTipDummy ~= nil and womatPickTipDummy ~= -1 then
            -- The marker was measured through the wrong mount, so it is wrong
            -- too. Drop it and let it be rebuilt from the repaired geometry.
            if womatPickTipDummy ~= simTip then
                pcall(sim.removeObject, womatPickTipDummy)
            end
            womatPickTipDummy=-1
womatIkOrientationTip=-1
            womatToolControlPoint=nil
            womatToolControlPointMode=nil
            simTip=flange
            womatInvalidateIkEnvironments()
        end
        womatFlangeTip=flange
        -- Not pcall-wrapped: a failure here is the difference between a gripper
        -- on the flange and a gripper a tool length adrift, and it must not pass
        -- for success.
        sim.setObjectParent(tcpGripper, womatFlangeTip, false)
        sim.setObjectPosition(tcpGripper, womatFlangeTip, ${gripperModelPositionLua})
        -- Orientation against the WRIST JOINT when the manifest asks for it: the
        -- flange dummy is a mount POINT and its orientation is whatever the last
        -- link's mesh origin left it, which differs between the UR and CR models
        -- in this cell. Position still comes off the flange -- that part of the
        -- dummy is correct, it sits on the wrist origin.
        local squareTo=${gripperSquareToWristJointLua} and womatWristJoint(womatFlangeTip) or -1
        if squareTo ~= -1 then
            sim.setObjectOrientation(tcpGripper, squareTo, ${gripperModelOrientationLua})
        else
            sim.setObjectOrientation(tcpGripper, womatFlangeTip, ${gripperModelOrientationLua})
        end
        local gripperLocalRotation=${gripperModelLocalRotationLua}
        if math.abs(gripperLocalRotation[1])+math.abs(gripperLocalRotation[2])+math.abs(gripperLocalRotation[3]) > 0.0000001 then
            local currentLocal=sim.getObjectMatrix(tcpGripper,womatFlangeTip)
            local extraLocal=sim.buildMatrix({0,0,0},gripperLocalRotation)
            sim.setObjectMatrix(tcpGripper,womatFlangeTip,sim.multiplyMatrices(currentLocal,extraLocal))
        end
        if ${applyGripperBaseOrientationCorrectionLua} and math.abs(womatBaseOrientation[1])+math.abs(womatBaseOrientation[2])+math.abs(womatBaseOrientation[3]) > 0.0000001 then
            local current=sim.getObjectMatrix(tcpGripper, sim.handle_world)
            local worldPosition={current[4],current[8],current[12]}
            local rotation=sim.buildMatrix({0,0,0},womatBaseOrientation)
            local rotated=sim.multiplyMatrices(rotation,current)
            rotated[4]=worldPosition[1]
            rotated[8]=worldPosition[2]
            rotated[12]=worldPosition[3]
            sim.setObjectMatrix(tcpGripper,sim.handle_world,rotated)
        end
        womatConfigureTcpGripperModel()
        womatInvalidateSafetyCollections()
        womatSetTcpGripperState(gripperClosed)
        -- Establish the fallback TCP-to-jaw transform while the robot is still
        -- in its startup posture. If the RG2 exposes an attach marker, that
        -- marker is used instead and no calibration is needed.
        local jawReference=womatAttachMarkerReference()
        if jawReference == -1 then
            womatCalibratedAttachPosition()
        end
    end
    -- Outside the configure-once guard on purpose: a scene that already carries
    -- a configured gripper still needs simTip rebound to the tool control
    -- point, and skipping that leaves the whole runtime steering the flange.
    womatEnsureToolControlPoint()
    return {
        success=true,
        state="found_in_scene",
        model=${gripperModelNameLua},
        handle=tcpGripper,
        toolControlPoint=womatToolControlPointWorld(),
        toolControlPointMarker=womatPickTipDummy
    }
end

-- The cell's other cube supports, built the same way the pick support is.
-- Idempotent by alias, so every robot's driver can ensure the whole list and
-- the first one to run is the one that creates them.
function womatEnsureTaskSites()
    local created={}
    for i=1,#womatTaskSiteDefinitions,1 do
        local site=womatTaskSiteDefinitions[i]
        local existing=sim.getObject('/'..site.alias, {noError=true})
        local handle=womatCreateGripCuboid(site.alias, site.size, site.position, {0.35,0.35,0.35}, true, true, site.yaw)
        if handle ~= -1 and existing == -1 then
            womatInvalidateSafetyCollections()
            created[#created+1]=site.alias
        end
    end
    return created
end

function womatEnsureGripObjects()
    local support=sim.getObject('/'..${supportNameLua}, {noError=true})
    local cube=sim.getObject('/'..${cubeNameLua}, {noError=true})
    if support == -1 then
        support=womatCreateGripCuboid(${supportNameLua}, ${supportSizeLua}, ${supportPositionLua}, {0.35,0.35,0.35}, true, true, ${gripYawLua})
        womatInvalidateSafetyCollections()
    end
    if cube == -1 then
        cube=womatCreateGripCuboid(${cubeNameLua}, ${cubeSizeLua}, ${cubePositionLua}, {0.9,0.25,0.1}, false, true, ${gripYawLua})
        womatInvalidateSafetyCollections()
    end
    if support == -1 or cube == -1 then
        return {
            success=false,
            reason="grip_scene_objects_missing",
            support=${supportNameLua},
            object=${cubeNameLua},
            supportMissing=support == -1,
            objectMissing=cube == -1
        }
    end

    sim.setObjectPosition(support, sim.handle_world, ${supportPositionLua})
    sim.setObjectOrientation(support, sim.handle_world, {0, 0, ${gripYawLua}})
    womatSetShapePhysics(support, true, true)
    if heldGripObject ~= cube then
        sim.setObjectPosition(cube, sim.handle_world, ${cubePositionLua})
        womatSetShapePhysics(cube, false, true)
    end
    local taskSites=womatEnsureTaskSites()
    local gripper=womatEnsureTcpGripperModel()
    return {
        success=true,
        support=${supportNameLua},
        object=${cubeNameLua},
        supportHandle=support,
        objectHandle=cube,
        taskSitesCreated=taskSites,
        gripperSuccess=gripper.success,
        gripperReason=gripper.reason,
        gripperModel=gripper.model
    }
end

function womatSafeEnsureGripObjects()
    return womatSafeCall(womatEnsureGripObjects)
end

function womatDistance(a, b)
    local dx=a[1]-b[1]
    local dy=a[2]-b[2]
    local dz=a[3]-b[3]
    return math.sqrt(dx*dx+dy*dy+dz*dz)
end

function womatAttachMarkerReference()
    if tcpGripper == nil or tcpGripper == -1 then
        return -1, "gripper_local_calibrated_offset"
    end
    local objects=sim.getObjectsInTree(tcpGripper, sim.handle_all, 0)
    local proximity=-1
    for i=1,#objects,1 do
        local alias=string.lower(womatPlainAlias(objects[i]))
        -- The stock RG2 model uses its attach point as the physical object
        -- attachment centre between the fingers.
        if string.find(alias, "attachpoint", 1, true) ~= nil then
            return objects[i], "gripper_attach_point"
        end
        if string.find(alias, "attachproxsensor", 1, true) ~= nil then
            proximity=objects[i]
        end
    end
    if proximity ~= -1 then
        return proximity, "gripper_proximity_sensor"
    end
    return -1, "gripper_local_calibrated_offset"
end

function womatCalibratedAttachPosition()
    local matrix=sim.getObjectMatrix(tcpGripper, sim.handle_world)
    if womatCalibratedAttachLocalOffset == nil then
        -- Last resort for a gripper that exposes neither contact pads nor an
        -- attach marker: a point one nominal tool length down the tool axis,
        -- converted into the gripper frame once so it follows wrist rotation.
        local axis=${toolAxisLua}
        local nominal=${toolLengthEstimate}
        local world=womatLocalVectorToWorld(axis[1]*nominal, axis[2]*nominal, axis[3]*nominal)
        womatCalibratedAttachLocalOffset={
            matrix[1]*world[1]+matrix[5]*world[2]+matrix[9]*world[3],
            matrix[2]*world[1]+matrix[6]*world[2]+matrix[10]*world[3],
            matrix[3]*world[1]+matrix[7]*world[2]+matrix[11]*world[3]
        }
    end
    local offset=womatCalibratedAttachLocalOffset
    return {
        matrix[4]+matrix[1]*offset[1]+matrix[2]*offset[2]+matrix[3]*offset[3],
        matrix[8]+matrix[5]*offset[1]+matrix[6]*offset[2]+matrix[7]*offset[3],
        matrix[12]+matrix[9]*offset[1]+matrix[10]*offset[2]+matrix[11]*offset[3]
    }
end

function womatAttachMarkerPosition()
    -- The RG2's own attach marker. This is NOT where an object is held: it sits
    -- at the base of the fingers, ~43 mm short of the contact pads. It survives
    -- only as the fallback origin for the tool control point when a gripper
    -- exposes no recognisable pads.
    local reference=womatAttachMarkerReference()
    if reference ~= -1 then
        return sim.getObjectPosition(reference, sim.handle_world)
    end
    return womatCalibratedAttachPosition()
end

function womatAttachMarkerMode()
    local _, mode=womatAttachMarkerReference()
    return mode
end

function getGripperState()
    local tcpState=womatSafeCall(womatEnsureTcpGripperModel)
    local jointInfo=womatSafeCall(womatGripperJointInfo)
    if jointInfo.success == false then
        jointInfo={count=0, positions={}, aliases={}, error=jointInfo.message}
    end
    return {
        closed=gripperClosed,
        holding=heldGripObject ~= -1,
        object=${cubeNameLua},
        tcpGripper=tcpGripper,
        tcpState=tcpState.state,
        tcpSuccess=tcpState.success,
        tcpError=tcpState.message,
        jointCount=jointInfo.count,
        jointPositions=jointInfo.positions,
        jointAliases=jointInfo.aliases,
        drivenJointAliases=jointInfo.drivenAliases,
        modelPosition=${gripperModelPositionLua},
        modelOrientation=${gripperModelOrientationLua},
        toolControlPoint=womatToolControlPointWorld(),
        toolControlPointMarker=womatPickTipDummy,
        toolControlPointMode=womatToolControlPointMode,
        attachMarkerMode=womatAttachMarkerMode(),
        attachMarkerPosition=womatAttachMarkerPosition(),
        carriedObjectCollisionMode=heldGripObject ~= -1 and "all_except_grip_support" or "not_holding",
        handoverApproachEnabled=womatHandoverIgnoredGripper ~= -1,
        handoverIgnoredGripper=womatHandoverIgnoredGripperAlias,
        objectParent=heldGripObject ~= -1 and sim.getObjectAlias(sim.getObjectParent(heldGripObject), 4) or nil
    }
end

function closeGripper()
    local gripSetup=womatEnsureGripObjects()
    if gripSetup.success == false then
        return gripSetup
    end
    if gripSetup.gripperSuccess == false then
        return {success=false, reason=gripSetup.gripperReason, model=gripSetup.gripperModel}
    end
    local object=sim.getObject('/'..${cubeNameLua}, {noError=true})
    if object == -1 then
        return {success=false, reason="grip_object_missing", object=${cubeNameLua}}
    end

    local tipPosition=sim.getObjectPosition(simTip, sim.handle_world)
    local jawPosition=womatAttachMarkerPosition()
    local objectPosition=sim.getObjectPosition(object, sim.handle_world)
    -- Measured from the contact pads, because that is what closes on the cube.
    -- The attach point is ~43 mm behind them, which on its own is enough to
    -- reject a cube sitting dead centre between the fingers.
    local graspPoint, graspMode=womatToolControlPointWorld()
    if graspPoint == nil then
        graspPoint=jawPosition
        graspMode="attach_point_fallback"
    end
    local distance=womatDistance(graspPoint, objectPosition)
    if distance > ${gripThreshold} then
        return {success=false, reason="object_not_in_grip_range", distance=distance, threshold=${gripThreshold}, tipPosition=tipPosition, graspPoint=graspPoint, graspReference=graspMode, jawPosition=jawPosition, objectPosition=objectPosition}
    end

    womatSetShapePhysics(object, true, false)
    sim.setObjectParent(object, tcpGripper, true)
    heldGripObject=object
    gripperClosed=true
    womatInvalidateSafetyCollections()
    local gripResult=womatSetTcpGripperState(true)
    if gripResult.success == false then
        return gripResult
    end
    return {success=true, state="gripped", object=${cubeNameLua}, distance=distance, tcpPosition=tipPosition, jawPosition=jawPosition, objectPosition=objectPosition, gripper=gripResult}
end

function openGripper()
    local gripperState=womatEnsureTcpGripperModel()
    if gripperState.success == false then
        return gripperState
    end
    if heldGripObject ~= -1 then
        sim.setObjectParent(heldGripObject, -1, true)
        womatSetShapePhysics(heldGripObject, false, true)
        heldGripObject=-1
        womatInvalidateSafetyCollections()
    end
    gripperClosed=false
    local gripResult=womatSetTcpGripperState(false)
    if gripResult.success == false then
        return gripResult
    end
    return {success=true, state="released", object=${cubeNameLua}, gripper=gripResult}
end

-- Handover checkpoint 1. The shared Node runtime serializes these two and owns
-- the authoritative cubeOwner state. The receiver must physically be at the
-- cube: it is refused otherwise, rather than teleporting the cube into the
-- jaws and reporting success.
function acceptTransferredGripObject()
    local gripperState=womatEnsureTcpGripperModel()
    if gripperState.success == false then
        return gripperState
    end
    local object=sim.getObject('/'..${cubeNameLua}, {noError=true})
    if object == -1 then
        return {success=false, reason="grip_object_missing", object=${cubeNameLua}}
    end

    -- The rendezvous is only real if the receiver's jaws are already on the
    -- cube. Snapping the cube here instead would hide every rendezvous error
    -- -- a wrist that drifted, a stale plan, an arm that never arrived -- and
    -- report a successful handover regardless.
    local cubePosition=sim.getObjectPosition(object, sim.handle_world)
    local receiverJawPosition=womatAttachMarkerPosition()
    -- Same reference as closeGripper: the contact pads, not the attach point.
    local graspPoint, graspMode=womatToolControlPointWorld()
    if graspPoint == nil then
        graspPoint=receiverJawPosition
        graspMode="attach_point_fallback"
    end
    local reach=womatDistance(graspPoint, cubePosition)
    if ${enableHandoverGraspGateLua} and reach > ${handoverGraspTolerance} then
        return {
            success=false,
            reason="receiver_not_at_cube",
            object=${cubeNameLua},
            distance=reach,
            tolerance=${handoverGraspTolerance},
            graspPoint={graspPoint[1], graspPoint[2], graspPoint[3]},
            graspReference=graspMode,
            jawPosition={receiverJawPosition[1], receiverJawPosition[2], receiverJawPosition[3]},
            objectPosition={cubePosition[1], cubePosition[2], cubePosition[3]},
            jawCenterMode=womatAttachMarkerMode()
        }
    end

    -- Re-parent in place. The cube is physically between the jaws already, so
    -- there is nothing to move.
    womatSetShapePhysics(object, true, false)
    sim.setObjectParent(object, tcpGripper, true)
    heldGripObject=object
    gripperClosed=true
    womatInvalidateSafetyCollections()
    local gripResult=womatSetTcpGripperState(true)
    return {
        success=gripResult.success,
        state="transfer_accepted",
        object=${cubeNameLua},
        parent=sim.getObjectAlias(tcpGripper, 4),
        placement=${handoverPlacementLabelLua},
        distance=reach,
        tolerance=${handoverGraspTolerance},
        objectPosition={cubePosition[1], cubePosition[2], cubePosition[3]},
        gripper=gripResult
    }
end

function relinquishTransferredGripObject()
    local object=sim.getObject('/'..${cubeNameLua}, {noError=true})
    if object == -1 then
        return {success=false, reason="grip_object_missing", object=${cubeNameLua}}
    end
    if heldGripObject ~= object then
        return {success=false, reason="source_not_holding_cube", object=${cubeNameLua}}
    end

    -- Do not unparent the cube: acceptTransferredGripObject has already moved
    -- it to the receiver. Only clear this script's stale ownership state.
    heldGripObject=-1
    gripperClosed=false
    womatInvalidateSafetyCollections()
    local gripResult=womatSetTcpGripperState(false)
    return {
        success=true,
        state="transfer_relinquished",
        object=${cubeNameLua},
        gripper=gripResult
    }
end

