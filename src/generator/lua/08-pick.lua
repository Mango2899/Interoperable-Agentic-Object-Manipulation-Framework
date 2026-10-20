-- simIK.findConfig reifies its validation callback by name, so the callback
-- has to be a named global; the pose it validates against travels in these.
womatPickValidationTip=-1
womatPickValidationTarget=nil
womatPickValidationAxis=nil

function womatUnwrapConfigToward(config, reference)
    -- Every joint here has a +/-360 degree range, so the same tool pose has up
    -- to three equivalent joint values per axis. findConfig's random restarts
    -- pick among them freely: a perfectly good hover pose comes back as
    -- joint6 = -360 degrees, which is the same orientation as 0 reached by
    -- unwinding a whole turn. Nothing downstream objects, because the POSE is
    -- correct -- it is only the travel to it that wraps the arm around itself.
    --
    -- So the pose is re-expressed in the angles nearest the posture the arm is
    -- actually in. Same tool frame, no full-turn excursions.
    local unwrapped={}
    for i=1,#config,1 do
        local low, high=nil, nil
        local ok, cyclic, interval=pcall(sim.getJointInterval, jointHandles[i])
        if ok and not cyclic and interval ~= nil then
            low=interval[1]
            high=interval[1]+interval[2]
        end
        local best=config[i]
        local bestDistance=math.abs(config[i]-(reference[i] or 0))
        for turn=-2,2,1 do
            if turn ~= 0 then
                local candidate=config[i]+turn*2*math.pi
                if low == nil or (candidate >= low and candidate <= high) then
                    local distance=math.abs(candidate-(reference[i] or 0))
                    if distance < bestDistance - 1e-9 then
                        best=candidate
                        bestDistance=distance
                    end
                end
            end
        end
        unwrapped[i]=best
    end
    return unwrapped
end

function womatConfigTravel(config, reference)
    local total=0
    for i=1,#config,1 do
        total=total+math.abs(config[i]-(reference[i] or 0))
    end
    return total
end

function womatClosestValidConfig(configs, reference)
    -- The branch a person would pick: the valid one reachable with the least
    -- joint travel from where the arm actually is.
    --
    -- findConfigs does sort its own result, but by distance from the IK
    -- ENVIRONMENT's current config -- and the search leaves that at the last
    -- configuration it happened to try, not the posture the arm started in. So
    -- the ordering it returns is not the ordering that matters here, and the
    -- selection is redone explicitly against the real starting posture.
    -- Ranked first, validated second, and only as far as the first one that
    -- passes. The full check -- the one carrying the clearance margin the
    -- search callback skipped -- is the expensive half, so running it over
    -- every candidate to then keep the cheapest by travel spends the whole
    -- cost to use one answer. Sorting by travel up front puts the answer at the
    -- front of the list, so the usual case pays for one full check.
    --
    -- Same result as before: the least-travel candidate that passes everything.
    local ranked={}
    for i=1,#configs,1 do
        local candidate=womatUnwrapConfigToward(configs[i], reference)
        ranked[#ranked+1]={config=candidate, travel=womatConfigTravel(candidate, reference)}
    end
    table.sort(ranked, function(a, b) return a.travel < b.travel end)
    for i=1,#ranked,1 do
        if womatPickConfigValid(ranked[i].config) then
            return ranked[i].config, ranked[i].travel
        end
    end
    return nil, nil
end

function womatPickConfigFailure(config, contactOnly)
    -- nil when the configuration is usable, otherwise a reason and its number.
    -- Multiple returns rather than a table: findConfig calls this thousands of
    -- times per search and most of those calls are rejections.
    --
    -- contactOnly is the difference between the SEARCH and the ANSWER. The
    -- search runs this on every candidate the solver produces -- thousands per
    -- plan -- so it gets the cheap half: the pose and axis tolerances, then
    -- contact. The clearance MARGIN is a distance query over full meshes and
    -- costs about as much again; paying it per candidate bought nothing,
    -- because a candidate is discarded either way and only the one that is
    -- finally chosen has to satisfy it. So it is paid on the finalists, in
    -- womatClosestValidConfig, which checks a handful rather than the lot.
    --
    -- Live cell: Robot1's pick from its home posture spent 35 seconds in this
    -- function before the arm moved. Same plan, same guarantees, 1.6 seconds.
    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], config[i])
    end
    if womatPickValidationTip == -1 or womatPickValidationTarget == nil then
        return "validation_target_unset"
    end
    local positionError, axisError=womatPickPoseError(
        womatPickValidationTip, womatPickValidationTarget, womatPickValidationAxis
    )
    if positionError > ${pickPositionTolerance} then
        return "pose_position_tolerance", positionError
    end
    if axisError > ${pickAxisToleranceDeg}*math.pi/180 then
        return "pose_axis_tolerance", axisError*180/math.pi
    end
    if contactOnly then
        local obstacles=womatObstacleCollection()
        if obstacles ~= -1
            and womatCollectionsTouch(womatRobotCollisionCollection(), obstacles) then
            return "collision", nil
        end
        return nil
    end
    if womatRobotObstacleCollision() then
        return "collision", nil
    end
    return nil
end

function womatPickConfigValid(config)
    return womatPickConfigFailure(config) == nil
end

function womatPickConfigReachable(config)
    -- The search callback: pose, axis and contact, no clearance margin.
    return womatPickConfigFailure(config, true) == nil
end

function womatPlanTopDownRelease(x, y, z, maxTime)
    -- Setting the cube down by letting go of it above the face.
    --
    -- The arm holds the cube, so the cube is already where the tool is: stop
    -- with the tool vertical at the release point, open the gripper, and the
    -- 45 mm cube falls the last few centimetres onto the support under gravity.
    -- No descent is planned and none is walked, which is the point -- descending
    -- with a cube in the jaws is the leg that has to thread the cube, the
    -- fingers and the support top through each other, and it buys nothing a
    -- short drop does not.
    --
    -- Same planner as the pick, asked for the hover and nothing after it: zero
    -- approach height, so the hover IS the commanded point, and zero descent
    -- steps, so the walk down is not solved at all. Everything that makes the
    -- pick's hover trustworthy -- the global collision-free search, the pose
    -- and axis verification against the scene -- applies here unchanged.
    return womatPlanTopDownPick(x, y, z, 0, 0, maxTime)
end

function womatTopDownDropCandidates(x, y, z, steps)
    -- Where on a support's top face this arm could set the cube down.
    --
    -- The centre of the face is the obvious drop point and the one the agent
    -- asks for, but obvious is not the same as reachable. A top-down pose costs
    -- reach: the tool has to stand vertically above the point, and for an arm
    -- near its limit the envelope that allows that is a good deal smaller than
    -- the one it can touch at any angle. So a support can sit with its CENTRE
    -- outside the top-down envelope while most of its face is inside it.
    --
    -- Live cell: Robot4's Dobot reaches 0.237 m from its base with the tool
    -- vertical at place height, and support_b's centre stands at 0.283 m --
    -- unplaceable at any hover height, since raising the hover only shrinks the
    -- envelope further. The near corner of that same face is at 0.216 m, which
    -- the arm reaches comfortably. Nothing was wrong with the support or the
    -- arm; the drop point was simply pinned to the one part of the face that
    -- could not be served.
    --
    -- So the face is offered as a set of candidates instead of a single point.
    -- They come back ordered by distance from the point that was ASKED for, so
    -- a centred drop is still tried first and still wins whenever it can be
    -- planned -- an arm with reach to spare behaves exactly as it did before.
    -- Only an arm that cannot serve the centre ever sees the rest of the list.
    local support=womatObjectByAlias(womatApproachSupportAlias())
    if support == -1 then
        return {}
    end
    -- Measured in the support's OWN frame, so a yawed support is handled by the
    -- frame rather than by rotating the grid: the face is an axis-aligned
    -- rectangle there whatever the box is turned to in the world.
    local ok, matrix=pcall(sim.getObjectMatrix, support, sim.handle_world)
    if not ok or matrix == nil then
        return {}
    end
    local bounds={}
    for axis, name in ipairs({"x", "y", "z"}) do
        local okLow, low=pcall(sim.getObjectFloatParam, support, sim["objfloatparam_objbbox_min_"..name])
        local okHigh, high=pcall(sim.getObjectFloatParam, support, sim["objfloatparam_objbbox_max_"..name])
        if not okLow or not okHigh or low == nil or high == nil then
            return {}
        end
        bounds[axis]={low, high}
    end

    -- The cube has to come to rest FULLY on the face, so the region its centre
    -- may occupy is the face shrunk by half the cube in each direction. The
    -- extra margin keeps it off the lip rather than balanced on it.
    local cube=${cubeSizeLua}
    local margin=${placeEdgeMarginLua}
    local centre={
        (bounds[1][1]+bounds[1][2])/2,
        (bounds[2][1]+bounds[2][2])/2,
        bounds[3][2]
    }
    local reach={
        (bounds[1][2]-bounds[1][1])/2-cube[1]/2-margin,
        (bounds[2][2]-bounds[2][1])/2-cube[2]/2-margin
    }
    for axis=1,2,1 do
        if reach[axis] < 0 then
            reach[axis]=0
        end
    end

    local requestedWorld=womatLocalToWorld(x, y, z)
    local divisions=math.max(1, math.floor(steps or 4))
    local candidates={}
    for i=-divisions,divisions,1 do
        for j=-divisions,divisions,1 do
            local world=sim.multiplyVector(matrix, {
                centre[1]+reach[1]*i/divisions,
                centre[2]+reach[2]*j/divisions,
                centre[3]
            })
            -- Keep the WORLD release height. Local Z may point sideways.
            world[3]=requestedWorld[3]
            local localPoint=womatWorldToLocal(world[1], world[2], world[3])
            candidates[#candidates+1]={
                x=localPoint[1],
                y=localPoint[2],
                z=localPoint[3],
                distance=math.sqrt((world[1]-requestedWorld[1])^2+(world[2]-requestedWorld[2])^2),
                -- Distance from the base axis, the tiebreak below.
                radius=math.sqrt(localPoint[1]^2+localPoint[2]^2)
            }
        end
    end
    -- Nearest the requested point first, so the placement stays as central as
    -- the arm allows. Within a ring of equally central candidates, nearest the
    -- BASE first: a grid step is symmetric about the face centre, so half of
    -- every ring lies further out than the point that just failed for want of
    -- reach, and planning those first spends a full search budget each to fail
    -- the same way. Ordering inside the ring costs nothing and changes no
    -- outcome -- the same candidate set, the same preference for the centre.
    table.sort(candidates, function(a, b)
        if math.abs(a.distance-b.distance) > 1e-9 then
            return a.distance < b.distance
        end
        return a.radius < b.radius
    end)
    return candidates
end

function womatPlanTopDownPick(x, y, z, approachHeight, descentSteps, maxTime)
    -- Solves the whole pick without moving anything: a global collision-free
    -- search for the hover configuration, then a continuous descent from it.
    -- Nothing here is singular, because the descent is solved from the hover
    -- posture rather than from the arm's fully extended home pose.
    local pickTip=womatEnsureToolControlPoint()
    if pickTip == -1 then
        return {ok=false, reason="pick_tcp_unavailable", message=womatMotionError}
    end

    local graspWorld=womatLocalToWorld(x, y, z)
    local hoverWorld={graspWorld[1], graspWorld[2], graspWorld[3]+approachHeight}

    local saved=womatCurrentConfig()
    local savedTargets={}
    for i=1,#jointHandles,1 do
        local ok, t=pcall(sim.getJointTargetPosition, jointHandles[i])
        savedTargets[i]=(ok and t ~= nil) and t or saved[i]
    end
    local savedTargetPose=sim.getObjectPose(simTarget, sim.handle_world)

    local env=simIK.createEnvironment()
    local group=simIK.createIkGroup(env)
    simIK.setIkGroupCalculation(env, group, simIK.method_damped_least_squares, ${pickIkDamping}, 200)
    local okAdd, element, simToIk=pcall(
        simIK.addIkElementFromScene,
        env, group, simBase, womatIkTip(pickTip), simTarget,
        simIK.constraint_position + simIK.constraint_alpha_beta
    )

    local result={ok=false, reason="pick_ik_setup_failed"}
    if okAdd and simToIk ~= nil then
        womatFreezeLockedIkJoints(env, simToIk)
        -- The solver's own convergence criterion has to be tighter than the
        -- tolerance the plan is accepted against, or every solution it returns
        -- gets rejected by the verification below. CoppeliaSim's element
        -- default is 5 mm / 0.1 rad, which is looser than a 45 mm cube can
        -- afford anyway.
        pcall(
            simIK.setElementPrecision, env, group, element,
            {${pickPositionTolerance}/2, ${pickAxisToleranceDeg}*math.pi/180/2}
        )
        local ikJoints={}
        for i=1,#jointHandles,1 do
            ikJoints[i]=simToIk[jointHandles[i]]
        end

        local function readConfig()
            local config={}
            for i=1,#ikJoints,1 do
                config[i]=simIK.getJointPosition(env, ikJoints[i])
            end
            return config
        end

        womatPickValidationTip=pickTip
        womatPickValidationTarget=hoverWorld
        womatPickValidationAxis={0, 0, -1}
        womatSetDownTarget(hoverWorld)
        -- addIkElementFromScene copied the target and joint state into a
        -- separate IK world. Updating simTarget in the scene does not update
        -- that copy, so synchronize before asking findConfig for the hover.
        simIK.syncFromSim(env, {group})

        -- Seeded solve first, exactly as a robot controller resolves a taught
        -- approach point: refine from the posture the arm is already in. It is
        -- deterministic, it is fast, and it keeps the elbow and wrist branch
        -- stable between consecutive picks. It is allowed to fail -- from the
        -- singular home posture damped least squares does thrash -- which is
        -- why the randomised search below still exists. What is NOT acceptable
        -- is skipping this and relying on a random search alone.
        local hover=nil
        local okSeed, seedResult=pcall(simIK.handleGroup, env, group)
        if okSeed and seedResult == simIK.result_success then
            local candidate=readConfig()
            if womatPickConfigValid(candidate) then
                hover=candidate
            end
        end

        local findError=nil
        if hover == nil then
            -- Randomised restarts: this is what steps over the singular home
            -- configuration instead of thrashing in it. Each stage below
            -- restores the posture the validation callbacks left behind and
            -- re-seeds the IK world from it, so every stage starts from the
            -- real arm state rather than from wherever the last one stopped.
            --
            -- findConfigs, not findConfig. The singular form returns ONE
            -- random valid branch, and "valid" says nothing about how far the
            -- arm must travel to reach it: that is how a cube straight ahead
            -- got answered with the base turned 173 degrees the other way and
            -- the arm folded back over itself. The plural form keeps searching
            -- for the whole budget and adds each solution's alternates, giving
            -- a set to choose from instead of a lottery ticket.
            -- Staged budget: a short look first, the full budget only if that
            -- found nothing.
            --
            -- maxTime bounds the SOLVER's time, not the callback's, and the
            -- callback is where the time actually goes -- so an 8 second budget
            -- is not an 8 second wait, it is however long several thousand
            -- collision checks take. findMultiple makes it spend the whole
            -- budget on purpose, gathering alternatives to choose between.
            --
            -- Almost every pick is solved in the first fraction of a second and
            -- gains nothing from the rest, so the reconnaissance stage runs
            -- first and the full budget is held in reserve for the pick that
            -- genuinely needs it. Nothing is given up: a stage that finds
            -- nothing falls through to the next, and the last stage is the
            -- budget this would have spent anyway.
            local budgets={math.min(0.4, maxTime or 4), maxTime or 4}
            local okFind, found=nil, nil
            for stage=1,#budgets,1 do
                if stage == 1 or hover == nil then
                    for i=1,#jointHandles,1 do
                        sim.setJointPosition(jointHandles[i], saved[i])
                    end
                    simIK.syncFromSim(env, {group})
                    okFind, found=pcall(
                        simIK.findConfigs,
                        env, group, ikJoints,
                        {
                            findMultiple=true,
                            findAlt=true,
                            -- Search gate, not solution accuracy: see pickSearchThresholdDist.
                            maxDist=${pickSearchThresholdDist},
                            maxTime=budgets[stage],
                            pMetric={1,1,1,0.1},
                            cb=womatPickConfigReachable
                        }
                    )
                    if okFind and found ~= nil and #found > 0 then
                        hover=womatClosestValidConfig(found, saved)
                    end
                end
                if hover ~= nil or budgets[stage] >= (maxTime or 4) then
                    break
                end
            end
            if hover == nil and not okFind then
                -- Older CoppeliaSim without findConfigs: one branch is still
                -- better than refusing to pick.
                local okOne, one=pcall(
                    simIK.findConfig,
                    env, group, ikJoints,
                    ${pickSearchThresholdDist}, maxTime or 4, {1,1,1,0.1},
                    womatPickConfigValid
                )
                if okOne then
                    hover=one
                else
                    findError=tostring(found)
                end
            end
        end

        if findError ~= nil then
            result={ok=false, reason="pick_findconfig_error", message=findError}
        elseif hover == nil then
            result={ok=false, reason="no_collision_free_hover_config"}
        else
            -- Before the descent is solved from it, and before anything moves:
            -- the descent is a continuous solve seeded from this posture, so an
            -- unwrapped hover keeps the whole plan in the same branch.
            -- Re-validated rather than assumed -- the pose is identical modulo
            -- a full turn, but joint limits and collision are not.
            local unwrapped=womatUnwrapConfigToward(hover, saved)
            if womatPickConfigValid(unwrapped) then
                hover=unwrapped
            end
            -- A release asks for no descent: the hover pose IS the whole
            -- plan. The arm stops above the face, opens, and the cube makes
            -- the last few centimetres on its own -- so there is nothing
            -- below the hover to solve, and nothing to check. The empty
            -- descent is what tells the caller that, as distinct from a
            -- one-waypoint one.
            if descentSteps ~= nil and descentSteps <= 0 then
                result={ok=true, hover=hover, descent={}, descentStates=0}
            else
                -- Walk the tool control point straight down from the hover pose,
                -- solving each step continuously so the wrist cannot flip between
                -- waypoints and the descent stays vertical.
                local steps=math.max(1, descentSteps)
                local descent={}
                local blockedAt=nil
                for i=1,#jointHandles,1 do
                    sim.setJointPosition(jointHandles[i], hover[i])
                end
                -- findConfig returns a configuration without changing the IK
                -- world. Seed that world from the selected hover posture before
                -- solving the continuous descent.
                simIK.syncFromSim(env, {group})
                for step=1,steps,1 do
                    local alpha=step/steps
                    womatSetDownTarget({
                        graspWorld[1],
                        graspWorld[2],
                        hoverWorld[3]+(graspWorld[3]-hoverWorld[3])*alpha
                    })
                    -- Each waypoint changes the scene target. Copy both that
                    -- target and the preceding joint solution into the IK world
                    -- so handleGroup solves the intended next descent segment.
                    simIK.syncFromSim(env, {group})
                    local okStep, stepResult=pcall(simIK.handleGroup, env, group)
                    if not okStep or stepResult ~= simIK.result_success then
                        blockedAt=step
                        break
                    end
                    local config=readConfig()
                    if not womatGoalConfigValid(config) then
                        blockedAt=step
                        break
                    end
                    for i=1,#config,1 do
                        descent[#descent+1]=config[i]
                    end
                    for i=1,#jointHandles,1 do
                        sim.setJointPosition(jointHandles[i], config[i])
                    end
                end
                if blockedAt ~= nil then
                    result={ok=false, reason="descent_unreachable_or_blocked", blockedAtStep=blockedAt}
                else
                    result={ok=true, hover=hover, descent=descent, descentStates=steps}
                end
            end
        end
    else
        result={ok=false, reason="pick_ik_setup_failed", message=tostring(element)}
    end

    pcall(simIK.eraseEnvironment, env)
    womatPickValidationTip=-1
    womatPickValidationTarget=nil
    womatPickValidationAxis=nil
    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], saved[i])
        pcall(sim.setJointTargetPosition, jointHandles[i], savedTargets[i])
    end
    sim.setObjectPose(simTarget, sim.handle_world, savedTargetPose)
    return result
end

