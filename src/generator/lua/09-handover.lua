-- Handover rendezvous pose.
--
-- The interaction-space plan places each robot's TIP one jaw offset short of
-- the shared cube point, so that both jaw centres land on the cube. That
-- arithmetic is only true if the wrist holds the pose the jaw offset was
-- measured in. The general Cartesian mode constrains position alone and lets
-- the wrist drift, which silently invalidates it -- and after a top-down pick
-- the giver arrives holding the tool vertically, roughly a quarter of a metre
-- away from where the plan assumes its jaws are. So the rendezvous leg pins
-- the tool axis to the direction the jaw offset was measured along, and the
-- plan's arithmetic becomes self-enforcing.
womatHandoverPoseEnabled=false
womatHandoverPoseAxis=nil
womatHandoverPoseAxisAlternates=nil
-- Roll angles about the pinned tool axis, in degrees, for the receiver to try.
-- Empty means the roll stays free, which is what every pair did before one arm
-- in this cell needed its jaws to miss another arm's. See womatApproachFrame.
womatHandoverPoseRolls=nil

function womatSetHandoverPoseLock(enabled, axis, alternates, rolls)
    womatHandoverPoseEnabled=enabled == true
    womatHandoverPoseAxis=nil
    womatHandoverPoseAxisAlternates=nil
    womatHandoverPoseRolls=nil
    if womatHandoverPoseEnabled and axis ~= nil and #axis == 3 then
        womatHandoverPoseAxis={axis[1], axis[2], axis[3]}
    end
    if womatHandoverPoseEnabled and alternates ~= nil then
        womatHandoverPoseAxisAlternates=alternates
    end
    if womatHandoverPoseEnabled and rolls ~= nil and #rolls > 0 then
        womatHandoverPoseRolls=rolls
    end
    return {
        success=true,
        state=womatHandoverPoseEnabled and "handover_pose_locked" or "handover_pose_free",
        axis=womatHandoverApproachAxisWorld(),
        alternateAxes=womatHandoverPoseAxisAlternates ~= nil and #womatHandoverPoseAxisAlternates or 0,
        rolls=womatHandoverPoseRolls ~= nil and #womatHandoverPoseRolls or 0
    }
end

function womatJawAngleInApproachFrame(axis, jawAxis)
    -- Where a closing axis sits in the frame womatApproachFrame builds for this
    -- approach direction, as an angle from that frame's X. Both grippers are
    -- measured in the same frame, which is the only way their jaw directions
    -- can be compared at all.
    local xAxis, yAxis=womatApproachFrame(axis)
    if xAxis == nil or jawAxis == nil then
        return nil
    end
    local u=jawAxis[1]*xAxis[1]+jawAxis[2]*xAxis[2]+jawAxis[3]*xAxis[3]
    local v=jawAxis[1]*yAxis[1]+jawAxis[2]*yAxis[2]+jawAxis[3]*yAxis[3]
    if math.abs(u) < 1e-9 and math.abs(v) < 1e-9 then
        return nil
    end
    return math.deg(math.atan(v, u)) % 180
end

function womatPredictedHandoverRoll(axis)
    -- The roll that puts THIS gripper's jaws across the giver's.
    --
    -- Two grippers meeting at one 45 mm cube have to take different pairs of
    -- its faces, and the roll about the pinned approach axis is what chooses
    -- which pair this one takes. Perpendicular to the giver's is the arrangement
    -- that leaves both sets of fingers somewhere to be -- measured live at the
    -- rendezvous, the pose the search settled on had the two closing axes 84.8
    -- degrees apart, which is that arrangement and not a coincidence.
    --
    -- A PREDICTION, not a constraint: it says where to start looking, and the
    -- search still walks the whole turn. Being roughly right is worth a great
    -- deal even so, because the alternative is finding the ten-degree window by
    -- exhaustion -- 129 refused plans on the live cell, which took the receiver
    -- past its own move deadline and collapsed the handover around it.
    local peer=womatPeerClosingAxisWorld()
    if peer == nil then
        return nil
    end
    local peerAngle=womatJawAngleInApproachFrame(axis, peer)
    if peerAngle == nil then
        return nil
    end
    -- This gripper's jaws sit at a fixed angle to its own tool frame, and with
    -- the full pose constrained that frame IS the rolled approach frame. So the
    -- offset measured wherever the arm is standing now holds at every roll.
    local own=womatOwnClosingAxisWorld()
    local tip=womatEnsureToolControlPoint()
    if own == nil or tip == -1 then
        return nil
    end
    local ok, matrix=pcall(sim.getObjectMatrix, womatIkTip(tip), sim.handle_world)
    if not ok or matrix == nil then
        return nil
    end
    local u=own[1]*matrix[1]+own[2]*matrix[5]+own[3]*matrix[9]
    local v=own[1]*matrix[2]+own[2]*matrix[6]+own[3]*matrix[10]
    if math.abs(u) < 1e-9 and math.abs(v) < 1e-9 then
        return nil
    end
    local ownOffset=math.deg(math.atan(v, u)) % 180
    return (peerAngle + 90 - ownOffset) % 180
end

function womatHandoverApproachRollCandidates(axis)
    -- The angles the driver will ask for, in the order it should ask. Nothing
    -- here is a relaxation either: every roll satisfies the pinned tool axis
    -- exactly, and they differ only in where the fingers end up around the cube.
    --
    -- Ordered nearest-first around the roll that crosses this gripper's jaws
    -- with the giver's, when that can be measured. The grid is the same grid --
    -- a full turn either way -- so a wrong prediction costs nothing but the
    -- order they are tried in, and a right one is the difference between seven
    -- attempts and seventy-two.
    if womatHandoverPoseRolls == nil then
        return {}
    end
    local rolls={}
    for i=1,#womatHandoverPoseRolls,1 do
        rolls[i]=womatHandoverPoseRolls[i]
    end
    local predicted=nil
    if axis ~= nil and #axis == 3 then
        predicted=womatPredictedHandoverRoll(axis)
    end
    if predicted == nil then
        return rolls
    end
    -- Jaws are a line, not an arrow, so a roll and its opposite place the
    -- fingers identically. Distance is therefore measured modulo 180.
    local function distance(roll)
        local d=math.abs((roll - predicted) % 180)
        if d > 90 then d=180-d end
        return d
    end
    table.sort(rolls, function(left, right)
        local dl=distance(left)
        local dr=distance(right)
        if math.abs(dl-dr) < 1e-9 then
            return left < right
        end
        return dl < dr
    end)
    -- ONE value. A second return leaks into the caller's list -- the driver
    -- hands these straight to the Node search, which reads them as a table of
    -- angles, and an appended prediction reads as one more angle to try.
    -- womatPredictedHandoverRoll is the way to ask for the prediction itself.
    return rolls
end

function womatHandoverApproachAxisCandidates()
    -- Every alternate is exactly as perpendicular to the giver's tool as the
    -- first one: they differ only in WHICH perpendicular, which the constraint
    -- leaves free. Trying them is not relaxing anything -- it is using a degree
    -- of freedom the rendezvous always had.
    local candidates={}
    local primary=womatHandoverApproachAxisWorld()
    if primary ~= nil then
        candidates[1]=primary
    end
    if womatHandoverPoseAxisAlternates ~= nil then
        for i=1,#womatHandoverPoseAxisAlternates,1 do
            local a=womatHandoverPoseAxisAlternates[i]
            if a ~= nil and #a == 3 then
                candidates[#candidates+1]={a[1], a[2], a[3]}
            end
        end
    end
    return candidates
end

function womatToolDirectionWorld()
    -- Where the tool is pointing RIGHT NOW, read off the frame the IK steers.
    --
    -- Not the same thing as womatToolAxisWorld, which rotates the CONFIGURED
    -- tool axis by the base and so answers with a constant of the mounting no
    -- matter how the arm is standing. That constant is the correct answer to
    -- "which way does this gripper face when the arm holds the pose the axis
    -- was measured in" and a fiction as an answer to "which way is this arm
    -- presenting the cube" -- and the second is what a receiver aiming opposed
    -- or perpendicular to a giver is asking. Live cell: Robot2 held the cube out
    -- with its tool straight up and reported a horizontal axis pointing away
    -- from the robot it was handing to, so Robot4's opposed approach was solved
    -- against a direction nobody was holding and its standoff landed on the far
    -- side of the cube.
    local tip=womatEnsureToolControlPoint()
    if tip == -1 then
        return nil
    end
    local ok, matrix=pcall(sim.getObjectMatrix, womatIkTip(tip), sim.handle_world)
    if not ok or matrix == nil then
        return nil
    end
    -- Third column: the IK frame's Z, which is the frame womatApproachFrame
    -- builds with Z along the tool.
    local direction={matrix[3], matrix[7], matrix[11]}
    local length=math.sqrt(direction[1]^2+direction[2]^2+direction[3]^2)
    if length < 1e-9 then
        return nil
    end
    return {direction[1]/length, direction[2]/length, direction[3]/length}
end

function womatToolAxisWorld()
    -- The direction the tool points when the arm holds the pose the tool axis
    -- was measured in, in world coordinates. A direction only: there is no
    -- length any more, because simTip IS the tool control point.
    local configured=${toolAxisLua}
    local world=womatLocalVectorToWorld(configured[1], configured[2], configured[3])
    local length=math.sqrt(world[1]*world[1]+world[2]*world[2]+world[3]*world[3])
    if length < 1e-9 then
        return nil
    end
    return {world[1]/length, world[2]/length, world[3]/length}
end

function womatHandoverApproachAxisWorld()
    -- An axis handed down with the pose lock wins. Which way the receiver comes
    -- in is a property of the PAIR -- opposed to the giver's tool, or
    -- perpendicular to it -- not of this robot's own mounting, and only the
    -- shared runtime knows both arms.
    if womatHandoverPoseAxis ~= nil then
        local a=womatHandoverPoseAxis
        local given=math.sqrt(a[1]*a[1]+a[2]*a[2]+a[3]*a[3])
        if given >= 1e-9 then
            return {a[1]/given, a[2]/given, a[3]/given}
        end
    end
    -- Otherwise the tool's own mounting direction, which for a pair of arms
    -- facing each other is already the opposed axis.
    return womatToolAxisWorld()
end

function womatWorstJointTravel(config, reference)
    -- The largest SINGLE-joint excursion between two configurations.
    --
    -- Deliberately not womatConfigTravel, which is the total across all joints
    -- and is the right measure for ranking pick branches against each other.
    -- What matters here is different: an arm that moves six joints a little is
    -- doing something quite unlike one that winds a single joint most of the
    -- way round, and the two can total the same.
    local largest=0
    for i=1,#config,1 do
        local delta=math.abs(config[i]-(reference[i] or 0))
        if delta > largest then largest=delta end
    end
    return largest
end

function womatSolveHandoverStandoff(env, group, ikJoints, readConfig, handoverTip,
                                    graspWorld, axis, standoffDistance, saved, budget, rollRad,
                                    maxTravelRad)
    -- Try progressively shorter standoffs before paying for a randomised
    -- search. A blocked standoff is usually blocked because it is too far back,
    -- not because the direction is wrong, and the seeded solve costs nothing.
    local fractions={1.0, 0.7, 0.45}
    local lastFailure=nil
    local lastMagnitude=nil
    for f=1,#fractions,1 do
        local distance=standoffDistance*fractions[f]
        local standoffWorld={
            graspWorld[1]-axis[1]*distance,
            graspWorld[2]-axis[2]*distance,
            graspWorld[3]-axis[3]*distance
        }
        womatPickValidationTip=handoverTip
        womatPickValidationTarget=standoffWorld
        womatPickValidationAxis=axis
        for i=1,#jointHandles,1 do
            sim.setJointPosition(jointHandles[i], saved[i])
        end
        womatSetApproachTarget(standoffWorld, axis, rollRad)
        simIK.syncFromSim(env, {group})
        local okSeed, seedResult=pcall(simIK.handleGroup, env, group)
        if okSeed and seedResult == simIK.result_success then
            -- Bring every joint to the turn closest to where the arm is
            -- standing before anything else looks at it. The solver is free to
            -- answer a wrist angle a full turn out from an identical pose, and
            -- the joint path to it then winds all the way round for nothing.
            -- The pick has always done this; the handover never did.
            local candidate=womatUnwrapConfigToward(readConfig(), saved)
            local failure, magnitude=womatPickConfigFailure(candidate)
            if failure == nil then
                -- Reachable, collision free, and pointing the right way is not
                -- yet enough. A pinned roll asks for the full pose, and damped
                -- least squares answers that from wherever the arm is standing
                -- -- sometimes by winding most of the way round. The pose is
                -- valid; the JOURNEY is a long sweep of the cell, slow to plan,
                -- slow to walk, and it strands the arm somewhere it cannot get
                -- home from in a straight line.
                --
                -- This is what simIK.findConfigs used to prevent for the free
                -- roll, by sorting candidates by closeness to the current
                -- posture. A pinned roll does its searching over the roll
                -- instead, so the same guarantee has to be asked for directly.
                -- Live cell: Robot4 took the cube with joint 1 at -169 degrees
                -- and then swept its own gripper through its link 2 on the way
                -- home.
                local travel=womatWorstJointTravel(candidate, saved)
                if maxTravelRad == nil or travel <= maxTravelRad then
                    return {
                        ok=true, config=candidate, world=standoffWorld,
                        distance=distance, route="seeded", travel=travel
                    }
                end
                lastFailure="travel"
                lastMagnitude=travel*180/math.pi
            else
                lastFailure=failure
                lastMagnitude=magnitude
            end
        else
            lastFailure="ik_diverged"
        end
    end

    -- A pinned roll does its searching in a different dimension. The driver is
    -- already walking a ring of rolls about this same axis, each of them a
    -- fresh problem for the seeded solve, so paying the randomised search's
    -- full time budget on every roll that does not work turns a search of
    -- twenty-four cheap attempts into one that takes a minute. The rolls ARE
    -- the fallback here.
    if rollRad ~= nil then
        return {
            ok=false,
            reason=lastFailure == "travel"
                and "handover_standoff_travel_exceeded"
                or "no_collision_free_handover_standoff",
            detail=lastMagnitude,
            seedFailure=lastFailure
        }
    end

    -- Nothing near the current posture works. Search, but take the config
    -- CLOSEST to where the arm already is: simIK.findConfigs sorts by exactly
    -- that, where findConfig returns an arbitrary valid branch and produces the
    -- long multi-joint contortions on the way to the standoff.
    local standoffWorld={
        graspWorld[1]-axis[1]*standoffDistance,
        graspWorld[2]-axis[2]*standoffDistance,
        graspWorld[3]-axis[3]*standoffDistance
    }
    womatPickValidationTarget=standoffWorld
    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], saved[i])
    end
    womatSetApproachTarget(standoffWorld, axis, rollRad)
    simIK.syncFromSim(env, {group})
    local okFind, found=pcall(simIK.findConfigs, env, group, ikJoints, {
        maxDist=${pickSearchThresholdDist},
        maxTime=budget,
        pMetric={1,1,1,0.1},
        findAlt=true,
        findMultiple=false,
        cb=womatPickConfigValid
    })
    if not okFind then
        return {ok=false, reason="handover_findconfig_error", message=tostring(found)}
    end
    if found ~= nil and #found > 0 then
        return {ok=true, config=found[1], world=standoffWorld, distance=standoffDistance, route="searched"}
    end
    return {
        ok=false,
        reason="no_collision_free_handover_standoff",
        detail=lastMagnitude,
        seedFailure=lastFailure
    }
end

function womatPlanHandoverApproach(x, y, z, standoffDistance, approachSteps, maxTime, axis, rollDeg, maxTravelDeg)
    -- ONE approach direction, and one roll about it, per call. Both searches
    -- live in the Node driver so that each script call stays short: a loop over
    -- candidates in here multiplies the worst case by the number of candidates,
    -- and a long-blocking script call is what raises CoppeliaSim's modal abort
    -- dialog and leaves the robot apparently counting forever.
    --
    -- Asking for a roll changes what is being solved. Without one the IK pins
    -- the tool AXIS and leaves the rotation about it to the solver
    -- (constraint_alpha_beta), which is right whenever nothing cares where the
    -- fingers end up. At a handover something does: both arms drive their tool
    -- control point to the SAME cube, so the two grippers close around one 45 mm
    -- block and the roll is the only thing deciding whether their fingers
    -- interleave or drive through each other. Nothing in the position-and-axis
    -- constraint can see that, which is why the giver's gripper had to be
    -- dropped from the obstacle set for the approach to plan at all -- and
    -- dropping it is what let the two grippers occupy the same space in silence.
    --
    -- So a pinned roll asks for the full pose, and it takes the giver's gripper
    -- back as a hard obstacle for the duration: the point of choosing the roll
    -- is to choose one that does not hit it, and that choice can only be made
    -- against a collision check that can still see it. Live cell: Robot4
    -- approaching Robot2 along world -X had a clean window at roughly 80-90
    -- degrees of roll and drove its fingers, and at some rolls its whole
    -- gripper body, through Robot2's RG2 at every other angle.
    --
    -- Two legs, for the same reason the pick has two. A joint-space line to the
    -- rendezvous sweeps a far larger volume than the straight line it looks
    -- like, and it rotates the tool through arbitrary orientations on the way.
    -- Leg 1 goes to a standoff backed off along the approach axis; leg 2 is a
    -- straight line in, with the tool axis held the whole way.
    local handoverTip=womatEnsureToolControlPoint()
    if handoverTip == -1 then
        return {ok=false, reason="handover_tip_unavailable", message=womatMotionError}
    end
    local rollRad=nil
    if rollDeg ~= nil then
        rollRad=rollDeg*math.pi/180
    end
    local maxTravelRad=nil
    if maxTravelDeg ~= nil and maxTravelDeg > 0 then
        maxTravelRad=maxTravelDeg*math.pi/180
    end
    -- The giver's gripper is only dropped from the obstacle set because the
    -- free-roll approach cannot avoid it. A pinned roll is chosen precisely so
    -- that it can, so it is checked against the cell as it really is.
    local suspendedGripper=nil
    if rollRad ~= nil and womatHandoverIgnoredGripper ~= -1 then
        suspendedGripper=womatHandoverIgnoredGripperAlias
        womatHandoverIgnoredGripper=-1
        womatHandoverIgnoredGripperAlias=nil
        womatInvalidateSafetyCollections()
    end
    local function restoreIgnoredGripper()
        if suspendedGripper ~= nil then
            local gripper=womatObjectByAlias(suspendedGripper)
            if gripper ~= -1 then
                womatHandoverIgnoredGripper=gripper
                womatHandoverIgnoredGripperAlias=suspendedGripper
            end
            suspendedGripper=nil
            womatInvalidateSafetyCollections()
        end
    end

    local approachAxis=nil
    if axis ~= nil and #axis == 3 then
        local length=math.sqrt(axis[1]*axis[1]+axis[2]*axis[2]+axis[3]*axis[3])
        if length >= 1e-9 then
            approachAxis={axis[1]/length, axis[2]/length, axis[3]/length}
        end
    else
        approachAxis=womatHandoverApproachAxisWorld()
    end
    if approachAxis == nil then
        restoreIgnoredGripper()
        return {ok=false, reason="handover_approach_axis_undefined"}
    end

    -- The commanded point IS the grasp point. simTip is the tool control point,
    -- so the interaction-space plan hands over the place the cube is exchanged
    -- and nothing has to be added back to recover it.
    local tipWorld=womatLocalToWorld(x, y, z)
    local graspWorld={tipWorld[1], tipWorld[2], tipWorld[3]}

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
    -- A pinned roll is a pose constraint; a free one is position plus the tool
    -- axis, which is the weaker problem and the one every pair that does not
    -- need its fingers placed should keep solving.
    local constraint=simIK.constraint_position + simIK.constraint_alpha_beta
    if rollRad ~= nil then
        constraint=simIK.constraint_pose
    end
    local okAdd, element, simToIk=pcall(
        simIK.addIkElementFromScene,
        env, group, simBase, womatIkTip(handoverTip), simTarget,
        constraint
    )

    local result={ok=false, reason="handover_ik_setup_failed"}
    if okAdd and simToIk ~= nil then
        womatFreezeLockedIkJoints(env, simToIk)
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

        local leg1=womatSolveHandoverStandoff(
            env, group, ikJoints, readConfig, handoverTip,
            graspWorld, approachAxis, standoffDistance or 0, saved, maxTime or 2.5, rollRad,
            maxTravelRad
        )
        if not leg1.ok then
            result=leg1
            result.axis=approachAxis
        else
            -- Straight in along the approach axis, solved continuously from the
            -- standoff posture so the wrist cannot flip between waypoints and
            -- the tool keeps pointing where the plan says it does.
            local steps=math.max(1, approachSteps or 1)
            local approach={}
            local blocked=nil
            for i=1,#jointHandles,1 do
                sim.setJointPosition(jointHandles[i], leg1.config[i])
            end
            simIK.syncFromSim(env, {group})
            for step=1,steps,1 do
                local alpha=step/steps
                local waypoint={
                    leg1.world[1]+(graspWorld[1]-leg1.world[1])*alpha,
                    leg1.world[2]+(graspWorld[2]-leg1.world[2])*alpha,
                    leg1.world[3]+(graspWorld[3]-leg1.world[3])*alpha
                }
                womatPickValidationTarget=waypoint
                womatSetApproachTarget(waypoint, approachAxis, rollRad)
                simIK.syncFromSim(env, {group})
                local okStep, stepResult=pcall(simIK.handleGroup, env, group)
                if not okStep or stepResult ~= simIK.result_success then
                    -- Almost always the tool passing through a singular region
                    -- of this arm rather than an obstacle: damped least squares
                    -- stops tracking before the Jacobian is rank deficient.
                    blocked={reason="handover_approach_ik_diverged", blockedAtStep=step}
                    break
                end
                local config=readConfig()
                local failure, magnitude=womatPickConfigFailure(config)
                if failure ~= nil then
                    blocked={
                        reason="handover_approach_"..failure,
                        blockedAtStep=step,
                        detail=magnitude
                    }
                    if failure == "collision" then
                        local obstacle, owner=womatBlockingObstacleAlias()
                        blocked.obstacle=obstacle
                        blocked.obstacleOwner=owner
                    end
                    break
                end
                for i=1,#config,1 do
                    approach[#approach+1]=config[i]
                end
                for i=1,#jointHandles,1 do
                    sim.setJointPosition(jointHandles[i], config[i])
                end
            end
            if blocked ~= nil then
                blocked.ok=false
                blocked.axis=approachAxis
                blocked.standoffDistance=leg1.distance
                result=blocked
            else
                local jaw=womatAttachMarkerPosition()
                local graspPoint, graspMode=womatToolControlPointWorld()
                result={
                    ok=true,
                    standoff=leg1.config,
                    standoffRoute=leg1.route,
                    standoffDistance=leg1.distance,
                    approach=approach,
                    approachStates=steps,
                    axis=approachAxis,
                    tipTarget=tipWorld,
                    graspTarget=graspWorld,
                    standoffTarget=leg1.world,
                    graspReference=graspMode,
                    graspPoint=graspPoint,
                    jawPosition={jaw[1], jaw[2], jaw[3]},
                    roll=rollDeg,
                    travel=leg1.travel ~= nil and leg1.travel*180/math.pi or nil
                }
            end
        end
    else
        result={ok=false, reason="handover_ik_setup_failed", message=tostring(element)}
    end

    pcall(simIK.eraseEnvironment, env)
    restoreIgnoredGripper()
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

