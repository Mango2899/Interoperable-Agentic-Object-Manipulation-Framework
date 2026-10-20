function womatFollowDescent(flatConfigs)
    -- Velocity-limit the waypoints and play them one per simulation step, as
    -- the planner path does.
    --
    -- At the VELOCITY step, not the collision step. These waypoints were
    -- validated where they were planned; resampling them four times finer than
    -- the speed limit adds no check here -- this function performs none -- and
    -- only makes the arm crawl to the rendezvous at a quarter of the speed it
    -- is configured for.
    local dof=#jointHandles
    local step=${plannedJointMaxVelocityDeg}*math.pi/180*sim.getSimulationTimeStep()
    local dense=womatDensifyFlatPath(flatConfigs, dof, step)
    return womatFollowConfigs(dense)
end

function womatFindGoalConfig(x, y, z, maxTime)
    -- Find a COLLISION-FREE goal configuration for a robot-local cartesian
    -- target: simIK.findConfig retries random seeds and only accepts configs
    -- that pass the collision validation callback (so e.g. a target behind the
    -- wall yields an arm posture arcing over it, not through it). The joint
    -- state is restored afterwards; nothing moves visibly.
    local goalPos=womatLocalToWorld(x, y, z)
    local saved=womatCurrentConfig()
    local savedTargets={}
    for i=1,#jointHandles,1 do
        local ok, t=pcall(sim.getJointTargetPosition, jointHandles[i])
        savedTargets[i]=(ok and t ~= nil) and t or saved[i]
    end
    local savedTargetPose=sim.getObjectPose(simTarget, sim.handle_world)

    sim.setObjectPosition(simTarget, sim.handle_world, goalPos)
    local env2=simIK.createEnvironment()
    local group2=simIK.createIkGroup(env2)
    -- The TD action provides position only. Requiring the target dummy's
    -- inherited orientation here overconstrains mounted robots (notably
    -- Robot3 with its pitch correction) even when the TCP position is safely
    -- reachable. Let findConfig choose any collision-free wrist orientation.
    local okAdd, element, simToIk=pcall(simIK.addIkElementFromScene, env2, group2, simBase, simTip, simTarget, simIK.constraint_position)
    local config=nil
    local failReason=nil
    if okAdd and simToIk ~= nil then
        womatFreezeLockedIkJoints(env2, simToIk)
        local ikJoints={}
        for i=1,#jointHandles,1 do
            ikJoints[i]=simToIk[jointHandles[i]]
        end
        -- First solve from the current posture. This is deterministic, starts
        -- quickly, and avoids needless elbow/wrist flips between otherwise
        -- identical Cartesian requests.
        local okNearest, nearestResult=pcall(simIK.handleGroup, env2, group2)
        if okNearest and nearestResult == simIK.result_success then
            local nearest={}
            for i=1,#ikJoints,1 do
                nearest[i]=simIK.getJointPosition(env2, ikJoints[i])
            end
            if womatGoalConfigValid(nearest) then
                config=nearest
            end
        end
        -- Only pay for randomized configuration search when the continuous
        -- nearest solution is unavailable or in collision.
        if config == nil then
            -- Same search-gate parameter as the pick planner, and the same
            -- reason for its size: 0.05 rejected almost every random draw
            -- before IK ever ran, which made this fallback fail silently
            -- whenever the seeded solve above was in collision.
            local okFind, found=pcall(simIK.findConfig, env2, group2, ikJoints, ${pickSearchThresholdDist}, maxTime or 4, {1,1,1,0.1}, womatGoalConfigValid)
            if okFind then
                config=found
                if config == nil then
                    failReason='no_collision_free_goal_config'
                end
            else
                failReason='findconfig_error: '..tostring(found)
            end
        end
    else
        failReason='ik_scratch_env_failed: '..tostring(element)
    end
    pcall(simIK.eraseEnvironment, env2)

    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], saved[i])
        pcall(sim.setJointTargetPosition, jointHandles[i], savedTargets[i])
    end
    sim.setObjectPose(simTarget, sim.handle_world, savedTargetPose)

    if config == nil then
        return {config=nil, reason=failReason}
    end
    return {config=config, ikError=0, goalInCollision=false}
end

function womatPlanJointPath(targetConf, maxTime, stateCnt)
    local okOmpl, simOMPL=pcall(require, 'simOMPL')
    if not okOmpl or simOMPL == nil then
        return {ok=false, reason='simOMPL_unavailable'}
    end

    local obstacleCollection=womatObstacleCollection()
    if obstacleCollection == -1 then
        return {ok=false, reason='no_obstacles_to_avoid'}
    end
    local robotCollection=womatRobotCollisionCollection()

    local saved=womatCurrentConfig()
    local savedTargets={}
    for i=1,#jointHandles,1 do
        local ok, target=pcall(sim.getJointTargetPosition, jointHandles[i])
        savedTargets[i]=(ok and target ~= nil) and target or saved[i]
    end

    local task=simOMPL.createTask('womat_'..sim.getObjectAlias(simBase, 0))
    simOMPL.setAlgorithm(task, simOMPL.Algorithm.RRTConnect)
    local spaces={}
    for i=1,#jointHandles,1 do
        local cyclic, interval=sim.getJointInterval(jointHandles[i])
        local low, high
        if cyclic then
            low=-math.pi
            high=math.pi
        else
            low=interval[1]
            high=interval[1]+interval[2]
        end
        spaces[#spaces+1]=simOMPL.createStateSpace('womat_j'..i, simOMPL.StateSpaceType.joint_position, jointHandles[i], {low}, {high}, 1)
    end
    simOMPL.setStateSpace(task, spaces)
    -- Obstacles AND the arm against itself. Without the second pair, RRT-Connect
    -- samples states that fold the gripper into a link, and the validation that
    -- follows can only reject the whole path once it is planned.
    local collisionPairs={robotCollection, obstacleCollection}
    local toolCollection, linkCollection=womatPrepareSelfCollisionCollections()
    if toolCollection ~= -1 and linkCollection ~= -1 then
        collisionPairs[#collisionPairs+1]=toolCollection
        collisionPairs[#collisionPairs+1]=linkCollection
    end
    simOMPL.setCollisionPairs(task, collisionPairs)
    simOMPL.setStartState(task, saved)
    simOMPL.setGoalState(task, targetConf)
    simOMPL.setup(task)
    local okCompute, solved, path=pcall(simOMPL.compute, task, maxTime or 4, -1, stateCnt or 120)
    pcall(simOMPL.destroyTask, task)

    local rawStates=0
    local smoothed=false
    local blockedAt=nil
    if okCompute and solved and path ~= nil and #path > 0 then
        local dof=#jointHandles
        rawStates=#path/dof
        -- Execute the same fine-grained states used for swept collision
        -- validation. This prevents a kinematic link from jumping through a
        -- thin wall between two collision-free OMPL waypoints.
        local velocityStep=${plannedJointMaxVelocityDeg}*math.pi/180*sim.getSimulationTimeStep()
        local collisionStep=${collisionCheckStepDeg}*math.pi/180
        local maxJointStep=math.min(velocityStep, collisionStep)
        local shortcut=womatShortcutFlatPath(path, dof, robotCollection, obstacleCollection)
        local candidate=womatSmoothFlatPath(shortcut, dof, ${pathSmoothingPasses})
        candidate=womatDensifyFlatPath(candidate, dof, maxJointStep)
        local candidateSafe, candidateBlocked=womatFlatPathCollisionFree(candidate, dof, robotCollection, obstacleCollection)
        if candidateSafe then
            path=candidate
            smoothed=${pathSmoothingPasses} > 0
        else
            -- Corner smoothing can reduce obstacle clearance. Fall back to the
            -- original OMPL geometry, but still velocity-limit and validate
            -- the exact samples that will be executed.
            path=womatDensifyFlatPath(path, dof, maxJointStep)
            local rawSafe, rawBlocked=womatFlatPathCollisionFree(path, dof, robotCollection, obstacleCollection)
            if not rawSafe then
                solved=false
                blockedAt=rawBlocked or candidateBlocked
            end
        end
    end

    -- OMPL collision validation writes candidate states to the scene objects.
    -- Restore both positions and controller targets before rendering or path
    -- execution can observe the last sampled planner state.
    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], saved[i])
        pcall(sim.setJointTargetPosition, jointHandles[i], savedTargets[i])
    end

    if not okCompute then
        return {ok=false, reason='planner_error', message=tostring(solved)}
    end
    if not solved or path == nil or #path == 0 then
        return {ok=false, reason='no_safe_path_found', blockedAtStep=blockedAt}
    end
    -- Checked finely, walked at the speed limit. Every state below survived the
    -- sweep above; the ones dropped sit on the straight segments between them.
    local checkedStates=#path/#jointHandles
    path=womatDecimateFlatPath(
        path, #jointHandles,
        ${plannedJointMaxVelocityDeg}*math.pi/180*sim.getSimulationTimeStep()
    )
    return {
        ok=true,
        path=path,
        states=#path/#jointHandles,
        checkedStates=checkedStates,
        rawStates=rawStates,
        smoothed=smoothed
    }
end

function womatPlanAndFollowJointPath(targetConf, maxTime, stateCnt)
    local plan=womatPlanJointPath(targetConf, maxTime, stateCnt)
    if not plan.ok then
        return plan
    end
    -- Watched, like every other route this driver plays back. A planned route
    -- is validated densely before it starts, but validation places the joints
    -- where the samples say, while the dynamic arm reaches them only
    -- approximately; the watchdog is what stops a route whose execution
    -- diverges from what was checked.
    womatFollowConfigs(plan.path, true)
    return {ok=true, started=true, states=plan.states, rawStates=plan.rawStates, smoothed=plan.smoothed}
end


-- Plan without starting motion. Keep the same densely validated route for the
-- clearance leg and the remainder home so announcing clearance cannot be
-- invalidated by a second planner choosing a different route.
function womatPrepareJointRoute(targetConf, maxTime, stateCnt, allowPlanning)
    local start=womatCurrentConfig()
    if #targetConf ~= #start then
        return {ok=false, reason='joint_goal_size_mismatch'}
    end
    local flat={}
    for i=1,#start do flat[#flat+1]=start[i] end
    for i=1,#targetConf do flat[#flat+1]=targetConf[i] end
    local maxStep=math.min(${collisionCheckStepDeg}*math.pi/180,
        ${plannedJointMaxVelocityDeg}*math.pi/180*sim.getSimulationTimeStep())
    local path=womatDensifyFlatPath(flat, #start, maxStep)
    local configs={}
    for offset=1,#path,#start do
        local config={}
        for i=0,#start-1 do config[#config+1]=path[offset+i] end
        configs[#configs+1]=config
    end
    local safety=womatPreflightJointPath(configs)
    if safety.ok then return {ok=true, path=path} end
    if not allowPlanning then return safety end
    local plan=womatPlanJointPath(targetConf, maxTime, stateCnt)
    if not plan.ok then return plan end
    plan.path=womatDensifyFlatPath(plan.path, #start, maxStep)
    return plan
end
