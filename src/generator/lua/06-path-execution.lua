function womatPreflightCartesianBodyPath(points)
    -- Test the same IK poses used by moveToPosition without leaving the robot
    -- in any of those candidate configurations. This catches elbow/link
    -- collisions before visible motion starts (the point-only CSV/AABB check
    -- cannot do that).
    local obstacleCollection=womatObstacleCollection()
    if obstacleCollection == -1 then
        return {ok=true}
    end

    local saved={}
    local savedTargets={}
    for i=1,#jointHandles,1 do
        saved[i]=sim.getJointPosition(jointHandles[i])
        local ok, target=pcall(sim.getJointTargetPosition, jointHandles[i])
        savedTargets[i]=(ok and target ~= nil) and target or saved[i]
    end
    local savedTargetPose=sim.getObjectPose(simTarget, sim.handle_world)
    local robotCollection=womatRobotCollisionCollection()
    local blockedAt=nil

    for step=1,#points,1 do
        local point=points[step]
        sim.setObjectPosition(simTarget, sim.handle_world, womatLocalToWorld(point[1], point[2], point[3]))
        simIK.applyIkEnvironmentToScene(ikEnv, ikGroup)
        womatPoseAtCommandedConfig()
        if womatCollectionsUnsafe(robotCollection, obstacleCollection) then
            blockedAt=step
            break
        end
    end

    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], saved[i])
        pcall(sim.setJointTargetPosition, jointHandles[i], savedTargets[i])
    end
    sim.setObjectPose(simTarget, sim.handle_world, savedTargetPose)

    if blockedAt ~= nil then
        return {ok=false, reason="cartesian_body_path_collision_detected", blockedAtStep=blockedAt}
    end
    return {ok=true}
end

womatMotionError=nil

function womatRecordCoroutineFailure(thread, ok, result)
    if ok then
        return false
    end
    womatMotionError=debug.traceback(thread, tostring(result))
    rState=3
    pcall(sim.addLog, sim.verbosity_errors, "WoMAT motion stopped: "..womatMotionError)
    return true
end

function womatGetMotionError()
    return womatMotionError
end

function sysCall_actuation()
    if rState == 1 then
        if coroutine.status(corout_move) ~= 'dead' then
            local ok, result = coroutine.resume(corout_move, finalPos)
            womatRecordCoroutineFailure(corout_move, ok, result)
        else
            rState = 3
        end
    elseif rState == 2 then
        if coroutine.status(corout_moveJoint) ~= 'dead' then
            local ok, result = coroutine.resume(corout_moveJoint, var1, var2, var3, var4, var5)
            womatRecordCoroutineFailure(corout_moveJoint, ok, result)
        else
            rState = 3
        end
    elseif rState == 4 then
        if corout_womatPath ~= nil and coroutine.status(corout_womatPath) ~= 'dead' then
            local ok, result = coroutine.resume(corout_womatPath)
            womatRecordCoroutineFailure(corout_womatPath, ok, result)
        else
            rState = 3
        end
    end
end

-- Path-planned motion support. Waypoint paths are executed exactly as checked:
-- one waypoint per simulation step, so the executed joint-space path is
-- identical to the collision-checked one (unlike sim.moveToConfig, whose
-- per-joint S-curves can deviate from the uniformly interpolated preflight).
womatPathConfigs=nil
womatPathTolerance=${pathTrackingToleranceDeg}*math.pi/180
womatPathSettleMaxSteps=${pathSettleMaxSteps}
womatPathReached=false
womatPathIndex=0
womatPathWatchdog=false
-- The last path sample playback may send; nil plays the whole path.
womatPathLimit=nil

function womatStopPath(reason)
    for i=1,#jointHandles do
        pcall(sim.setJointTargetPosition, jointHandles[i], sim.getJointPosition(jointHandles[i]))
    end
    corout_womatPath=nil
    womatMotionError=reason
    rState=3
    return {ok=true}
end

-- Also where the core moves the hold point, so following another arm costs no
-- round trip beyond the status poll it already makes. A negative limit lifts
-- the hold; no argument leaves it where it is.
function womatHomePathStatus(limit)
    if limit ~= nil then
        womatPathLimit=(limit >= 0) and limit or nil
    end
    return {state=rState, index=womatPathIndex, reached=womatPathReached,
        limit=womatPathLimit,
        config=womatCurrentConfig(), frames=womatCurrentFramePositions().points,
        error=womatMotionError}
end

function womatPathCoroutine()
    -- One sample per simulation step, so this coroutine may yield only at its
    -- own switchThread. CoppeliaSim auto-yields a coroutine after 2 ms of
    -- computation, and the watchdog's contact test takes 3 to 40 ms: with
    -- auto-yield on, every sample cost two steps and the route ran at half
    -- its time law, as a staircase (E4). Stepping is kept per coroutine.
    pcall(sim.setStepping, true)
    local dof=#jointHandles
    local count=#womatPathConfigs/dof
    for s=1,count,1 do
        -- An ordered reservation's hold. The arm keeps the last sample it was
        -- sent and waits here, still watched, until the core raises the limit
        -- -- which it does once the arm this one is following has left the
        -- voxels the next samples pass through. Checked before the sample is
        -- sent, never after, so the arm is never commanded into space another
        -- arm still holds.
        while womatPathLimit ~= nil and s > womatPathLimit do
            if womatPathWatchdog and womatInMotionUnsafe() then
                womatStopPath("home_path_collision_detected")
                return
            end
            sim.switchThread()
        end
        -- The sample about to be sent, not the posture the arm is in: a
        -- dynamic joint reaches its target only during the following step.
        if womatPathWatchdog then
            local sample={}
            for i=1,dof,1 do
                sample[i]=womatPathConfigs[(s-1)*dof+i]
            end
            if womatCommandedPostureUnsafe(sample) then
                womatStopPath("home_path_collision_detected")
                return
            end
        end
        for i=1,dof,1 do
            local value=womatPathConfigs[(s-1)*dof+i]
            if sim.isDynamicallyEnabled(jointHandles[i]) then
                pcall(sim.setJointTargetPosition, jointHandles[i], value)
            else
                -- Generated URDF arms commonly use kinematic joints. A target
                -- position alone does not move those joints.
                pcall(sim.setJointPosition, jointHandles[i], value)
            end
        end
        womatPathIndex=s
        -- The path is resampled to a bounded joint delta, so it forms one
        -- continuous reference signal. Do not stop at every intermediate
        -- sample: tolerance-gated mini-goals make dynamic joints overshoot and
        -- then correct in the opposite direction.
        sim.switchThread()
    end

    -- Settle only at the actual goal and require several consecutive accurate
    -- samples, avoiding both premature completion and waypoint chatter.
    local stableSteps=0
    for settle=1,womatPathSettleMaxSteps,1 do
        sim.switchThread()
        if womatPathWatchdog and womatInMotionUnsafe() then
            womatStopPath("home_path_collision_detected")
            return
        end
        local reached=true
        for i=1,dof,1 do
            local target=womatPathConfigs[(count-1)*dof+i]
            if math.abs(sim.getJointPosition(jointHandles[i])-target) > womatPathTolerance then
                reached=false
                break
            end
        end
        if reached then
            stableSteps=stableSteps+1
            if stableSteps >= 5 then
                break
            end
        else
            stableSteps=0
        end
    end
    womatPathReached=stableSteps >= 5
end

function womatFollowConfigs(flatConfigs, watchdog, limit)
    womatPathConfigs=flatConfigs
    womatPathReached=false
    womatPathIndex=0
    womatPathLimit=(limit ~= nil and limit >= 0) and limit or nil
    womatPathWatchdog=watchdog == true
    if womatPathWatchdog then womatResetInMotionSafety() end
    womatMotionError=nil
    corout_womatPath=coroutine.create(womatPathCoroutine)
    rState=4
    return "started"
end

function womatPathDidReachGoal()
    return womatPathReached
end

function womatCurrentConfig()
    local conf={}
    for i=1,#jointHandles,1 do
        conf[i]=sim.getJointPosition(jointHandles[i])
    end
    return conf
end

function womatSmoothFlatPath(path, dof, passes)
    local count=math.floor(#path/dof)
    local current=path
    for pass=1,passes,1 do
        if count <= 2 then
            break
        end
        local nextPath={}
        for i=1,dof,1 do
            nextPath[i]=current[i]
            nextPath[(count-1)*dof+i]=current[(count-1)*dof+i]
        end
        for state=2,count-1,1 do
            for i=1,dof,1 do
                local index=(state-1)*dof+i
                nextPath[index]=0.2*current[index-dof]+0.6*current[index]+0.2*current[index+dof]
            end
        end
        current=nextPath
    end
    return current
end

function womatFlatSegmentCollisionFree(path, dof, fromState, toState, robotCollection, obstacleCollection)
    local from=(fromState-1)*dof
    local to=(toState-1)*dof
    local maxDelta=0
    for i=1,dof,1 do
        maxDelta=math.max(maxDelta, math.abs(path[to+i]-path[from+i]))
    end
    local maxCheckStep=${collisionCheckStepDeg}*math.pi/180
    local checks=math.max(1, math.ceil(maxDelta/maxCheckStep))
    for step=1,checks,1 do
        local alpha=step/checks
        for i=1,dof,1 do
            sim.setJointPosition(jointHandles[i], path[from+i]+(path[to+i]-path[from+i])*alpha)
        end
        if womatCollectionsUnsafe(robotCollection, obstacleCollection) then
            return false
        end
    end
    return true
end

function womatShortcutFlatPath(path, dof, robotCollection, obstacleCollection)
    -- RRTConnect often contains harmless but visually distracting detours.
    -- Greedily retain the furthest state reachable by a collision-free joint
    -- segment. Every shortcut is swept at the same resolution as execution.
    local count=math.floor(#path/dof)
    if count <= 2 then
        return path
    end
    local result={}
    local current=1
    for i=1,dof,1 do
        result[#result+1]=path[i]
    end
    while current < count do
        local nextState=current+1
        for candidate=count,current+1,-1 do
            if womatFlatSegmentCollisionFree(path, dof, current, candidate, robotCollection, obstacleCollection) then
                nextState=candidate
                break
            end
        end
        for i=1,dof,1 do
            result[#result+1]=path[(nextState-1)*dof+i]
        end
        current=nextState
    end
    return result
end

function womatDecimateFlatPath(path, dof, maxJointStep)
    -- Thin a validated path down to the states that will actually be played.
    --
    -- Validation and playback want different resolutions and used to be forced
    -- to share one. A swept check has to sample finely enough that no link
    -- crosses a thin obstacle between two samples; playback puts one state on
    -- each simulation step, so its resolution IS the arm's speed. Tying them
    -- together meant the finer of the two won both, and a path checked at half
    -- a degree was then walked at half a degree per step -- ten degrees a
    -- second, on a robot configured for forty-five.
    --
    -- Every state kept here is one the sweep already passed, and the ones
    -- dropped lie on the straight joint-space segment between two kept ones, so
    -- nothing is played that was not checked.
    local count=math.floor(#path/dof)
    if count <= 2 then
        return path
    end
    local thinned={}
    local anchor=0
    for i=1,dof,1 do
        thinned[#thinned+1]=path[i]
    end
    for state=2,count-1,1 do
        local current=(state-1)*dof
        local delta=0
        for i=1,dof,1 do
            delta=math.max(delta, math.abs(path[current+i]-path[anchor+i]))
        end
        if delta >= maxJointStep then
            for i=1,dof,1 do
                thinned[#thinned+1]=path[current+i]
            end
            anchor=current
        end
    end
    -- The goal state always survives, or the arm stops short of it.
    local last=(count-1)*dof
    for i=1,dof,1 do
        thinned[#thinned+1]=path[last+i]
    end
    return thinned
end

function womatDensifyFlatPath(path, dof, maxJointStep)
    local count=math.floor(#path/dof)
    if count <= 1 then
        return path
    end
    local dense={}
    for i=1,dof,1 do
        dense[#dense+1]=path[i]
    end
    for state=2,count,1 do
        local previous=(state-2)*dof
        local current=(state-1)*dof
        local maxDelta=0
        for i=1,dof,1 do
            maxDelta=math.max(maxDelta, math.abs(path[current+i]-path[previous+i]))
        end
        local steps=math.max(1, math.ceil(maxDelta/maxJointStep))
        for step=1,steps,1 do
            local alpha=step/steps
            for i=1,dof,1 do
                dense[#dense+1]=path[previous+i]+(path[current+i]-path[previous+i])*alpha
            end
        end
    end
    return dense
end

function womatFlatPathCollisionFree(path, dof, robotCollection, obstacleCollection, maxCheckStep)
    local count=math.floor(#path/dof)
    maxCheckStep=maxCheckStep or (${collisionCheckStepDeg}*math.pi/180)
    for state=1,count,1 do
        local checks=1
        if state > 1 then
            local maxDelta=0
            for i=1,dof,1 do
                maxDelta=math.max(maxDelta, math.abs(path[(state-1)*dof+i]-path[(state-2)*dof+i]))
            end
            checks=math.max(1, math.ceil(maxDelta/maxCheckStep))
        end
        for check=1,checks,1 do
            local alpha=check/checks
            for i=1,dof,1 do
                local value=path[(state-1)*dof+i]
                if state > 1 then
                    local previous=path[(state-2)*dof+i]
                    value=previous+(value-previous)*alpha
                end
                sim.setJointPosition(jointHandles[i], value)
            end
            if womatCollectionsUnsafe(robotCollection, obstacleCollection) then
                return false, state
            end
        end
    end
    return true, nil
end

function womatGoalConfigValid(config)
    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], config[i])
    end
    -- Self-collision as well: a goal that folds the gripper into a link is not
    -- a goal the arm may be sent to, however free of obstacles it is.
    return not womatRobotObstacleCollision() and not womatRobotSelfCollision()
end
