
-- All TD-facing cartesian coordinates are robot-local. The base pose maps them
-- into world coordinates for translated and rotated shared-cell robots.
womatRobotName=${robotNameLua}
womatBaseOffset=${baseOffsetLua}
womatBaseOrientation=${baseOrientationLua}
womatBaseYaw=${baseYaw}
womatBaseCos=math.cos(womatBaseYaw)
womatBaseSin=math.sin(womatBaseYaw)
womatBaseRotation=sim.buildMatrix({0,0,0},womatBaseOrientation)

-- A locked axis, as the SOLVER sees it.
--
-- Locking a joint removes it from jointHandles, which stops it being commanded,
-- counted, saved or restored. It does not stop simIK: addIkElementFromScene
-- builds its chain by walking the scene from base to tip, so every IK
-- environment the driver creates contains the locked axis and every Cartesian
-- move is free to rotate it. The rollback in womatSimplePoseCallback then
-- restores jointHandles only, so the drift is never undone and accumulates over
-- moves. Live cell: Robot4's joint_end, locked at zero, was found sitting at
-- 8.97 degrees with a target written to match, and the end link it carries had
-- tilted out of level.
--
-- Passive is exactly the statement being made: the joint is part of the chain
-- and its value matters, but the solver may not choose it. Guarded, because a
-- driver that cannot say so must still move the arm the way it did before.
function womatFreezeLockedIkJoints(env, simToIk)
    if simToIk == nil or womatLockedJointHandles == nil then
        return 0
    end
    local frozen = 0
    for i=1,#womatLockedJointHandles,1 do
        local ikJoint = simToIk[womatLockedJointHandles[i]]
        if ikJoint ~= nil then
            local ok = pcall(simIK.setJointMode, env, ikJoint, simIK.jointmode_passive)
            if ok then
                frozen = frozen + 1
            end
        end
    end
    return frozen
end

function womatLocalToWorld(x, y, z)
    return {
        womatBaseOffset[1]+womatBaseRotation[1]*x+womatBaseRotation[2]*y+womatBaseRotation[3]*z,
        womatBaseOffset[2]+womatBaseRotation[5]*x+womatBaseRotation[6]*y+womatBaseRotation[7]*z,
        womatBaseOffset[3]+womatBaseRotation[9]*x+womatBaseRotation[10]*y+womatBaseRotation[11]*z
    }
end

function womatLocalVectorToWorld(x, y, z)
    -- Direction/offset vectors rotate with the robot but are not translated.
    -- Prefer the actual scene matrix so later base roll/pitch/yaw changes are
    -- handled too; retain configured-yaw math as a compatibility fallback.
    local ok, matrix=pcall(sim.getObjectMatrix, simBase, sim.handle_world)
    if ok and matrix ~= nil then
        return {
            matrix[1]*x+matrix[2]*y+matrix[3]*z,
            matrix[5]*x+matrix[6]*y+matrix[7]*z,
            matrix[9]*x+matrix[10]*y+matrix[11]*z
        }
    end
    return {
        womatBaseCos*x-womatBaseSin*y,
        womatBaseSin*x+womatBaseCos*y,
        z
    }
end

function womatWorldToLocal(x, y, z)
    local dx=x-womatBaseOffset[1]
    local dy=y-womatBaseOffset[2]
    local dz=z-womatBaseOffset[3]
    return {
        womatBaseRotation[1]*dx+womatBaseRotation[5]*dy+womatBaseRotation[9]*dz,
        womatBaseRotation[2]*dx+womatBaseRotation[6]*dy+womatBaseRotation[10]*dz,
        womatBaseRotation[3]*dx+womatBaseRotation[7]*dy+womatBaseRotation[11]*dz
    }
end

function womatBasePose()
    -- Where this arm actually stands right now, straight out of the scene.
    -- The configured placement is reported alongside it so a caller can see
    -- when the two have drifted apart: the runtime's own local/world mapping
    -- is built from the configured one, and only the live one is ground truth
    -- once somebody has dragged a base or served with different offsets.
    local position=sim.getObjectPosition(simBase, sim.handle_world)
    local orientation=sim.getObjectOrientation(simBase, sim.handle_world)
    return {
        robot=womatRobotName,
        position=position,
        orientation=orientation,
        configuredOffset=womatBaseOffset,
        configuredOrientation=womatBaseOrientation
    }
end

function moveTojoint(targetConf)
    womatMotionError=nil
    return moveTojoint_async(targetConf)
end

function moveToPosition(targetPosition)
    womatMotionError=nil
    local ok, err=pcall(function()
        sim.setObjectPosition(simTarget, sim.handle_world, womatLocalToWorld(targetPosition[1], targetPosition[2], targetPosition[3]))
        simIK.applyIkEnvironmentToScene(ikEnv, ikGroup)
    end)
    rState=3
    if not ok then
        womatMotionError=tostring(err)
        return {success=false, reason="ik_execution_failed", message=womatMotionError}
    end
    return "success"
end

