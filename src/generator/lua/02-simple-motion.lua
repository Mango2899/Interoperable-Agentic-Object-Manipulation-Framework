-- Presentation-oriented motion mode. CoppeliaSim/Ruckig calculates one
-- acceleration- and jerk-limited trajectory, while the existing IK group
-- continuously maps a Cartesian pose trajectory to the robot joints.
womatSimpleJointVelocity=${simpleJointVelocityDeg}*math.pi/180
womatSimpleJointAcceleration=${simpleJointAccelerationDeg}*math.pi/180
womatSimpleJointJerk=${simpleJointJerkDeg}*math.pi/180
womatSimpleCartesianVelocity=${simpleCartesianVelocity}
womatSimpleCartesianAcceleration=${simpleCartesianAcceleration}
womatSimpleCartesianJerk=${simpleCartesianJerk}
womatSimpleSafetyEnabled=${simpleSafetyEnabledLua}
womatSimpleIkEnv=nil
womatSimpleIkGroup=nil

function womatEnsureSimplePositionIk()
    if womatSimpleIkEnv ~= nil and womatSimpleIkGroup ~= nil then
        return true
    end
    local ok, err=pcall(function()
        womatSimpleIkEnv=simIK.createEnvironment()
        womatSimpleIkGroup=simIK.createIkGroup(womatSimpleIkEnv)
        simIK.setIkGroupCalculation(
            womatSimpleIkEnv,
            womatSimpleIkGroup,
            simIK.method_damped_least_squares,
            0.02,
            200
        )
        local element, simToIk=simIK.addIkElementFromScene(
            womatSimpleIkEnv,
            womatSimpleIkGroup,
            simBase,
            simTip,
            simTarget,
            ${simpleIkConstraintLua}
        )
        -- The general Cartesian move is where an unfrozen locked axis does the
        -- most damage: it runs on every request, and its collision rollback
        -- restores only jointHandles.
        womatFreezeLockedIkJoints(womatSimpleIkEnv, simToIk)
        simIK.setIkElementPrecision(womatSimpleIkEnv, womatSimpleIkGroup, element, {0.001, 0.01})
    end)
    if not ok then
        womatSimpleIkEnv=nil
        womatSimpleIkGroup=nil
        womatMotionError="simple_position_ik_setup_failed: "..tostring(err)
        return false
    end
    return true
end

-- Whether the general move constrains the tool's orientation as well as its
-- position depends on whether the arm HAS a free wrist. A six-axis arm does,
-- and taking it away costs reach for nothing. A Dobot does not: its tool is
-- held level by a parallelogram linkage that the kinematic tree describes as
-- three independent parallel axes, so a position-only solve is free to tilt
-- the tool out of level, which shortens the reach and swings the gripper into
-- the forearm. Constraining the pose puts the levelling back -- theta2 minus
-- theta3 plus theta4 equals zero falls straight out of it -- and it is how the
-- arm's own scene script has always done it.

-- Top-down grasp orientation.
--
-- The general Cartesian mode constrains position only, which leaves the wrist
-- free: the solver reaches the point at whatever orientation it drifts into,
-- so the tool arrives angled and its body sweeps into the pick support. It
-- also quietly breaks the jaw-offset model, which assumes a canonical wrist
-- pose. For the pickup sequence the orientation is therefore pinned so the
-- jaws hang vertically over the cube and descend straight onto it.
-- Preflight and execution must drive the IK through one entry point, or the
-- sweep validates postures the motion never takes.
--
-- On a dynamic joint the IK writes only the TARGET position; the joint stays
-- where it is until the physics moves it. A sweep or a prepared route reads
-- the posture back and tests it, so the solution is put into the joints here --
-- otherwise every sample is the starting posture and the IK never advances.
function womatPoseAtCommandedConfig()
    local commanded=womatCommandedConfig(jointHandles)
    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], commanded[i])
    end
end

function womatApplyCartesianTarget(x, y, z)
    sim.setObjectPosition(simTarget, sim.handle_world, womatLocalToWorld(x, y, z))
    simIK.applyIkEnvironmentToScene(womatSimpleIkEnv, womatSimpleIkGroup)
    womatPoseAtCommandedConfig()
end

function womatSimplePoseCallback(pose, velocity, accel, data)
    local saved={}
    for i=1,#jointHandles,1 do
        saved[i]=sim.getJointPosition(jointHandles[i])
    end
    sim.setObjectPose(data.target, sim.handle_world, pose)
    simIK.applyIkEnvironmentToScene(data.ikEnv, data.ikGroup)
    -- The IK gave dynamic joints targets only; test where they are being sent.
    if womatSimpleSafetyEnabled and womatCommandedPostureUnsafe(womatCommandedConfig(jointHandles)) then
        for i=1,#jointHandles,1 do
            sim.setJointPosition(jointHandles[i], saved[i])
            pcall(sim.setJointTargetPosition, jointHandles[i], saved[i])
        end
        sim.setObjectPose(data.target, sim.handle_world, sim.getObjectPose(data.tip, sim.handle_world))
        error("simple_cartesian_path_collision_detected")
    end
end

function womatSimpleMoveToPose(targetPose)
    if not womatEnsureSimplePositionIk() then
        return
    end
    local data={
        ikEnv=womatSimpleIkEnv,
        ikGroup=womatSimpleIkGroup,
        tip=simTip,
        target=simTarget,
        joints=simJoints
    }
    local currentPose=sim.getObjectPose(simTip, sim.handle_world)
    return sim.moveToPose(
        -1,
        currentPose,
        {womatSimpleCartesianVelocity,womatSimpleCartesianVelocity,womatSimpleCartesianVelocity,2.0},
        {womatSimpleCartesianAcceleration,womatSimpleCartesianAcceleration,womatSimpleCartesianAcceleration,4.0},
        {womatSimpleCartesianJerk,womatSimpleCartesianJerk,womatSimpleCartesianJerk,8.0},
        targetPose,
        womatSimplePoseCallback,
        data,
        nil
    )
end

function womatSimpleJointCallback(config, velocity, accel, handles)
    if not womatSimpleSafetyEnabled then
        movCallback(config, velocity, accel, handles)
        return
    end
    -- Test the configuration before it is sent. movCallback gives dynamic
    -- joints a target only, so testing after it would test the posture the
    -- arm is leaving, not the one it is being sent to.
    if womatCommandedPostureUnsafe(config, handles) then
        for i=1,#handles,1 do
            pcall(sim.setJointTargetPosition, handles[i], sim.getJointPosition(handles[i]))
        end
        error("simple_joint_path_collision_detected")
    end
    movCallback(config, velocity, accel, handles)
end

function womatSimpleMoveToConfig(handles, maxVel, maxAccel, maxJerk, targetConf)
    local current={}
    for i=1,#handles,1 do
        current[i]=sim.getJointPosition(handles[i])
    end
    return sim.moveToConfig(
        -1,current,nil,nil,maxVel,maxAccel,maxJerk,targetConf,nil,
        womatSimpleJointCallback,handles
    )
end

function womatStartSimpleCartesian(x, y, z)
    womatMotionError=nil
    womatResetInMotionSafety()
    if not womatEnsureSimplePositionIk() then
        rState=3
        return {success=false, reason="simple_position_ik_setup_failed", message=womatMotionError}
    end
    local world=womatLocalToWorld(x, y, z)
    -- Orientation is carried only to satisfy moveToPose's pose-shaped input;
    -- the position-only IK element ignores it.
    local targetPose=sim.getObjectPose(simTip, sim.handle_world)
    targetPose[1]=world[1]
    targetPose[2]=world[2]
    targetPose[3]=world[3]
    finalPos=targetPose
    corout_move=coroutine.create(womatSimpleMoveToPose)
    rState=1
    return "started"
end

function womatStartSimpleJoint(targetConf)
    womatMotionError=nil
    womatResetInMotionSafety()
    var1=jointHandles
    var2={}
    var3={}
    var4={}
    for i=1,#jointHandles,1 do
        var2[i]=womatSimpleJointVelocity
        var3[i]=womatSimpleJointAcceleration
        var4[i]=womatSimpleJointJerk
    end
    var5=targetConf
    corout_moveJoint=coroutine.create(womatSimpleMoveToConfig)
    rState=2
    return "started"
end

womatWorldGetCartesianposition=getCartesianposition
function getCartesianposition()
    local p=womatWorldGetCartesianposition()
    local localPosition=womatWorldToLocal(p.x, p.y, p.z)
    return {x=localPosition[1], y=localPosition[2], z=localPosition[3]}
end

function collisionCheck()
    local p=getCartesianposition()
    return womatPointObstacleCollision(p.x, p.y, p.z)
end


-- A handover-only version of the established smooth position-IK route. Solve
-- from the current posture, save the exact joint samples, then restore the
-- scene. The core reserves and rechecks those samples before replaying them;
-- an unrecorded IK/Ruckig motion never runs under a smaller spatial claim.
womatCartesianRouteJob=nil

function womatContinueSimpleCartesianRoute()
    local job=womatCartesianRouteJob
    if job == nil then return {ok=false, reason='cartesian_route_missing'} end
    local saved=womatCaptureSweepPosture()
    local began=os.clock()
    local ok, result=pcall(function()
        -- Restore the proposal's seed, not the live arm's pose between chunks.
        for i=1,#jointHandles do sim.setJointPosition(jointHandles[i], job.seed[i]) end
        local path={}
        if job.index == 0 then
            for i=1,#jointHandles do path[#path+1]=job.seed[i] end
        end
        local processed=0
        repeat
            job.index=job.index+1
            -- The core supplies the time law; this walks it. Keeping the
            -- profile in one place (core/timeLaw.js) is what stopped the
            -- reserved route from being timed differently to every other move.
            local a=job.schedule[job.index]
            womatApplyCartesianTarget(job.from.x+(job.x-job.from.x)*a,
                job.from.y+(job.y-job.from.y)*a, job.from.z+(job.z-job.from.z)*a)
            for i=1,#jointHandles do
                job.seed[i]=sim.getJointPosition(jointHandles[i])
                path[#path+1]=job.seed[i]
            end
            processed=processed+1
        until job.index >= job.steps or processed >= 8 or os.clock()-began >= 0.02
        if job.index >= job.steps then
            local actual=getCartesianposition()
            if math.sqrt((actual.x-job.x)^2+(actual.y-job.y)^2+(actual.z-job.z)^2) > job.tolerance then
                return {ok=false, reason='cartesian_route_target_not_reached'}
            end
        end
        return {ok=true, pending=job.index < job.steps, path=path, dof=#jointHandles}
    end)
    womatRestoreSweepPosture(saved)
    job.cpuSeconds=job.cpuSeconds+os.clock()-began
    if not ok then result={ok=false, reason='cartesian_route_ik_failed', message=tostring(result)} end
    if job.cpuSeconds > 4 then result={ok=false, reason='cartesian_route_budget_exceeded'} end
    if not result.ok or not result.pending then womatCartesianRouteJob=nil end
    return result
end

-- Where the route starts and how long a simulation step is, so the core can
-- build the schedule for it before asking for the route.
function womatSimpleCartesianRouteSetup()
    return {ok=true, from=getCartesianposition(), dt=sim.getSimulationTimeStep()}
end

-- `schedule` is the fraction of the way along the straight line at each
-- simulation step, one entry per step, ending at 1.
function womatPrepareSimpleCartesianRoute(x, y, z, tolerance, schedule)
    womatCartesianRouteJob=nil
    if not womatEnsureSimplePositionIk() then return {ok=false, reason='simple_position_ik_setup_failed'} end
    if schedule == nil or #schedule == 0 then return {ok=false, reason='cartesian_route_schedule_missing'} end
    if #schedule > 2000 then return {ok=false, reason='cartesian_route_too_long'} end
    local from=getCartesianposition()
    local seed={}
    for i=1,#jointHandles do seed[i]=sim.getJointPosition(jointHandles[i]) end
    womatCartesianRouteJob={from=from, x=x, y=y, z=z, tolerance=tolerance,
        steps=#schedule, schedule=schedule, index=0, seed=seed, cpuSeconds=0}
    return womatContinueSimpleCartesianRoute()
end
