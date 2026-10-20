-- Top-down pick.
--
-- Three things make this reliable where a Cartesian move with a free wrist is
-- not. The IK is driven through a dummy at the TOOL CONTROL POINT -- the
-- midpoint of the finger contact pads, not the RG2 attach point, which sits at
-- the base of the fingers ~43 mm short of where the object is actually held.
-- The approach pose is solved by simIK.findConfig, a randomised search with a
-- collision callback, rather than by gradient descent from the current posture:
-- the home pose has joint5 = 0 and joint3 = 0, the wrist and elbow
-- singularities together, and incremental damped least squares answers a
-- rank-deficient Jacobian there with a huge joint excursion. And the roll about
-- the tool axis is left unconstrained, because it is the degree of freedom the
-- wrist singularity destroys and a symmetric object does not care about it.
womatToolControlPoint=nil
womatToolControlPointMode=nil
womatPickTipDummy=-1
-- The '/tip' dummy the model ships with: the flange, where the gripper bolts
-- on. simTip is rebound away from it to the tool control point as soon as there
-- is a gripper, so everything downstream -- CoppeliaSim's IK included -- steers
-- the point that actually holds the object.
womatFlangeTip=nil

-- Why there is no "square the flange up" repair here.
--
-- robot_driver_workspace.lua used to create 'tip' with setObjectPosition alone,
-- which leaves a dummy square with the WORLD rather than with the wrist. That
-- is fixed at the source now, but every model built before it carries the old
-- dummy baked into its .ttm, and repairing those on resolve turned out to be a
-- worse cure than the disease:
--
--   * It can only recognise the fault by the flange being square with the
--     world, and that is only true at the ONE arm pose the dummy was created
--     at. So it fires at scene load, silently does nothing once the arm has
--     moved, and a simulation restart -- which restores the scene's own poses
--     -- undoes it. The tool jumps before the run and snaps back at the start
--     of it.
--   * The three CR models here already ship a flange square with their wrist
--     joint, so they never needed it. The two UR models ship a twisted one
--     (Robot1 [90,-83.9,0], Robot2 [-90,0,-90]) and their gripper mounts were
--     tuned around that twist, so squaring it up moved a correctly mounted tool.
--
-- A model whose flange really is wrong is fixed by regenerating it, which now
-- creates the dummy with the wrist's full pose -- not by rewriting the scene
-- underneath a mount that was calibrated against it.

-- The joint the tool is square with.
--
-- The 'tip' dummy is a MOUNT POINT, not a frame: robot_driver_workspace.lua
-- gives it whatever orientation the last link happened to have, and a URDF
-- visual shape carries its mesh's own origin. The three CR models here land
-- square with their wrist joint by luck; both UR models land twisted (Robot1
-- 83.9 degrees about it, Robot2 a quarter turn), and a mount measured against
-- that twist puts the jaws out sideways -- Robot1's tool control point sat at
-- [-0.005, 0.181, 0.048] in its own flange frame instead of straight out.
--
-- The axis a tool is actually square with is the one the wrist ROLLS about, so
-- that is what runtime.gripper.squareToWristJoint mounts against. It is the
-- flange's own parent chain, so nothing in the scene is moved to get it.
function womatWristJoint(flange)
    if flange == nil or flange == -1 then return -1 end
    local node=sim.getObjectParent(flange)
    local hops=0
    while node ~= -1 and hops < 12 do
        if sim.getObjectType(node) == sim.object_joint_type then return node end
        node=sim.getObjectParent(node)
        hops=hops+1
    end
    return -1
end

function womatResolveFlange()
    -- Never from simTip: that variable BECOMES the tool control point, so any
    -- path reading it afterwards mounts the gripper one tool length out and puts
    -- the jaws at two.
    --
    -- Found by searching the tree, not by path. './tip' is a direct-child
    -- lookup and this dummy hangs off the last LINK, not off the model root, so
    -- the path form can return -1 -- and every use of the result is inside a
    -- pcall, which turns that into a silent no-op that leaves the gripper
    -- mounted wherever it already was. That is indistinguishable from a working
    -- run until the jaws are measured.
    if womatFlangeTip ~= nil and womatFlangeTip ~= -1 and sim.isHandle(womatFlangeTip) then
        return womatFlangeTip
    end
    local candidates=sim.getObjectsInTree(simBase, sim.object_dummy_type, 0)
    for i=1,#candidates,1 do
        if womatPlainAlias(candidates[i]) == "tip" then
            womatFlangeTip=candidates[i]
            return womatFlangeTip
        end
    end
    local direct=sim.getObject('./tip', {noError=true})
    if direct ~= -1 then
        womatFlangeTip=direct
        return womatFlangeTip
    end
    womatFlangeTip=nil
    return nil
end

function womatToolControlPointWorld()
    -- simTip IS the tool control point once the gripper has been attached, so
    -- this is just its position. Before that it degrades to the flange, which
    -- is the best available answer while there is no gripper to measure.
    if simTip == nil or simTip == -1 then
        return nil, "unavailable"
    end
    return sim.getObjectPosition(simTip, sim.handle_world), womatToolControlPointMode or "flange"
end

function womatToolControlPointInFlange()
    -- Midpoint of the gripper's contact pads, in FLANGE-local coordinates.
    -- Measured against the flange rather than simTip because simTip is about to
    -- become this very point: the robot's tool control point is where it holds
    -- things, so that is what the whole runtime, and CoppeliaSim's IK, steers.
    if womatToolControlPoint ~= nil then
        return womatToolControlPoint
    end
    if tcpGripper == nil or tcpGripper == -1 or womatFlangeTip == nil or womatFlangeTip == -1 then
        return nil
    end
    local tipMatrix=sim.getObjectMatrix(womatFlangeTip, sim.handle_world)
    local inverse=sim.getMatrixInverse(tipMatrix)
    local low={math.huge, math.huge, math.huge}
    local high={-math.huge, -math.huge, -math.huge}
    local found=false
    for _, shape in ipairs(sim.getObjectsInTree(tcpGripper, sim.object_shape_type, 0)) do
        local alias=string.lower(womatPlainAlias(shape))
        -- womatIsDecorativeShape lives in 04-safety, where the same
        -- distinction keeps duplicate render meshes out of every collision
        -- collection. It matters here for a different reason: graspPadAlias is
        -- a SUBSTRING match, so a gripper whose pads are 'left_hand' and
        -- 'right_hand' matches 'left__hand_visual' and 'right_hand_visual' just
        -- as readily -- and those meshes hang off the moving jaw links, well off
        -- the tool centreline. Folding them into the pad bounding box drags its
        -- centre sideways, and since that centre IS the tool axis (see
        -- womatOrientToolFrame), the whole tool frame tilts with it. Live cell:
        -- Robot4's Dobot measured a midpoint 11.8 mm off the centreline and an
        -- axis 15.9 degrees off the gripper's own, so "point the tool straight
        -- down" was really asking for a gripper canted 15.9 degrees.
        if string.find(alias, ${graspPadAliasLua}, 1, true) ~= nil
            and not womatIsDecorativeShape(alias) then
            found=true
            local matrix=sim.getObjectMatrix(shape, sim.handle_world)
            local extents={
                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_x), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_x)},
                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_y), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_y)},
                {sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_z), sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_z)}
            }
            for a=1,2,1 do
                for b=1,2,1 do
                    for c=1,2,1 do
                        local world=sim.multiplyVector(matrix, {extents[1][a], extents[2][b], extents[3][c]})
                        local localCorner=sim.multiplyVector(inverse, world)
                        for k=1,3,1 do
                            low[k]=math.min(low[k], localCorner[k])
                            high[k]=math.max(high[k], localCorner[k])
                        end
                    end
                end
            end
        end
    end
    if not found then
        local declared=${gripperTcpOffsetLua}
        if declared ~= nil then
            -- A tool with nothing to measure -- a fixed tool, a suction cup, a
            -- gripper that is only a housing mesh. Its business end is declared
            -- in the TOOL's own frame, so it still follows the wrist. Measuring
            -- wins wherever there is anything to measure: pads move when the
            -- jaws open and a declared offset does not, so this is the fallback
            -- and never the first answer.
            local world=sim.multiplyVector(sim.getObjectMatrix(tcpGripper, sim.handle_world), declared)
            womatToolControlPoint=sim.multiplyVector(inverse, world)
            womatToolControlPointMode="declared_offset"
            return womatToolControlPoint
        end
        -- No recognisable pads: fall back to the RG2 attach marker, which is at
        -- least on the tool axis even if it is short of the contact surface.
        womatToolControlPoint=womatAttachPointInFlange()
        womatToolControlPointMode="attach_point_fallback"
        return womatToolControlPoint
    end
    womatToolControlPointMode="contact_pad_midpoint"
    womatToolControlPoint={
        (low[1]+high[1])/2,
        (low[2]+high[2])/2,
        (low[3]+high[3])/2
    }
    return womatToolControlPoint
end

function womatAttachPointInFlange()
    local tipMatrix=sim.getObjectMatrix(womatFlangeTip, sim.handle_world)
    local jaw=womatAttachMarkerPosition()
    local d={jaw[1]-tipMatrix[4], jaw[2]-tipMatrix[8], jaw[3]-tipMatrix[12]}
    return {
        tipMatrix[1]*d[1]+tipMatrix[5]*d[2]+tipMatrix[9]*d[3],
        tipMatrix[2]*d[1]+tipMatrix[6]*d[2]+tipMatrix[10]*d[3],
        tipMatrix[3]*d[1]+tipMatrix[7]*d[2]+tipMatrix[11]*d[3]
    }
end

function womatIkTip(tip)
    -- The frame the IK steers. The tool-frame child when there is one; the tip
    -- itself for a scene whose './tip' already sits on the pads, where there is
    -- no measured offset to build a frame from and the old Z-is-the-tool
    -- assumption is the only thing left to go on.
    if womatIkOrientationTip ~= -1 and sim.isHandle(womatIkOrientationTip) then
        return womatIkOrientationTip
    end
    -- A STALE handle is not the same thing as never having had one, and it must
    -- not take the fallback. The helper is a dummy created at run time, so
    -- CoppeliaSim reverts it whenever the simulation stops -- while these script
    -- globals, living in a customization script, survive. What is left is a
    -- handle to an object that no longer exists, and falling back on it pins the
    -- marker's Z while the tool runs down its Y. The two are orthogonal by
    -- construction, so the solver converges and womatPickPoseError then rejects
    -- every single solution at exactly 90 degrees. Live cell: Robot1 could not
    -- pick a cube 0.75 m in front of it, reachable and unobstructed, because all
    -- 2410 candidate configurations came back "pose_axis_tolerance ~ 89.5 deg"
    -- and the pick reported no_collision_free_hover_config.
    --
    -- So rebuild it. womatOrientToolFrame restates the marker's frame from the
    -- same measured offset it was built from, which is a no-op on the marker,
    -- and re-creates the child whose Z IS the marker's Y.
    if womatPickTipDummy ~= nil and womatPickTipDummy ~= -1 and sim.isHandle(womatPickTipDummy)
        and womatFlangeTip ~= nil and womatFlangeTip ~= -1 then
        local rebuilt=pcall(
            womatOrientToolFrame,
            womatPickTipDummy,
            sim.getObjectPosition(womatPickTipDummy, womatFlangeTip)
        )
        if rebuilt and womatIkOrientationTip ~= -1 and sim.isHandle(womatIkOrientationTip) then
            return womatIkOrientationTip
        end
    end
    return tip
end

function womatOrientToolFrame(marker, tcp)
    -- THE definition of the tool frame: Y runs down the tool, which is the
    -- gripper's own convention (runtime.gripper.toolAxis is +Y) and what the
    -- marker visibly shows in the scene. A top-down pick is therefore "the
    -- TCP's Y points along world -Z".
    --
    -- The axes come from womatApproachFrame rather than a second copy of the
    -- same Gram-Schmidt: it already returns an orthonormal frame whose Z is the
    -- direction handed to it, so this is a relabelling of that frame --
    -- X stays X, Y becomes its Z, and Z becomes -Y to stay right-handed.
    local matrix=womatToolFrameMatrix(tcp)
    if matrix == nil then return false end
    sim.setObjectMatrix(marker, womatFlangeTip, matrix)

    -- simIK's alpha_beta constraint pins a frame's Z and leaves the roll about
    -- it free; it cannot pin Y. So the IK is driven through a child at the same
    -- origin whose Z IS the marker's Y. Position IK is unchanged -- same point
    -- -- while the orientation constraint now pins the axis that runs down the
    -- tool, and the roll about it stays free exactly as before.
    local helper=sim.getObject('./'..${tcpIkAliasLua}, {noError=true})
    if helper == -1 then
        helper=sim.createDummy(0.001)
        sim.setObjectAlias(helper, ${tcpIkAliasLua})
    end
    sim.setObjectParent(helper, marker, false)
    sim.setObjectMatrix(helper, marker, {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0})
    womatIkOrientationTip=helper
    return true
end

function womatEnsureToolControlPoint()
    -- A dummy at the midpoint of the contact pads, whose Y axis runs down the
    -- tool. simTip is rebound to it, which is the whole point: the robot's tool
    -- control point becomes the place it actually grips, so a commanded
    -- Cartesian position IS where the object ends up and no offset arithmetic
    -- survives anywhere downstream.
    if womatPickTipDummy ~= nil and womatPickTipDummy ~= -1 and sim.isHandle(womatPickTipDummy) then
        return womatPickTipDummy
    end
    -- Script globals reset on re-init but scene objects do not. Adopt the
    -- marker already in the scene rather than stacking a second one -- and,
    -- more importantly, rather than measuring a fresh pad offset from a tip
    -- that is already sitting on the pads, which would place it a second tool
    -- length out.
    local flange=womatResolveFlange()
    local existingMarker=womatObjectByAliasInRobot(${tcpAliasLua})
    if existingMarker ~= -1 and sim.getObjectParent(existingMarker) == flange then
        womatPickTipDummy=existingMarker
        -- Re-measured, not adopted at its word. The marker's position is the
        -- tool offset AND the tool axis, so a marker placed by an older run --
        -- or by a run whose pad match was wrong -- keeps handing the IK a tool
        -- frame that no longer describes this gripper, and nothing downstream
        -- can tell. A scene saved with that marker in it carries the error
        -- forward across every restart, which is exactly how Robot4's 15.9
        -- degree tilt survived: see womatIsDecorativeShape.
        --
        -- The measurement is independent of this marker -- it is taken against
        -- the flange, which womatResolveFlange finds by the alias 'tip' and can
        -- never confuse with the marker -- so re-running it is idempotent and
        -- cannot compound an offset the way measuring from simTip would.
        local measured=womatToolControlPointInFlange()
        -- Recorded here too, not just on the measuring path: without it every
        -- report downstream -- toolControlPoint, closeGripper's graspReference
        -- -- calls this marker the flange, which reads exactly like the tool
        -- frame having silently collapsed back to the mount.
        if measured == nil then
            -- Nothing to measure against: the marker's own position is still
            -- the best record of where the tool is. Restated rather than
            -- trusted -- the pipeline sets its POSITION only, so its
            -- orientation is whatever it happened to inherit.
            measured=sim.getObjectPosition(existingMarker, womatFlangeTip)
            womatToolControlPointMode=womatToolControlPointMode or "adopted_contact_pad_marker"
        end
        womatOrientToolFrame(existingMarker, measured)
        simTip=existingMarker
        womatInvalidateIkEnvironments()
        return existingMarker
    end
    if existingMarker ~= -1 then
        -- Present but hanging off something else: it describes a tool frame this
        -- robot no longer has. Rebuild rather than trust it.
        pcall(sim.removeObject, existingMarker)
    end
    local tcp=womatToolControlPointInFlange()
    if tcp == nil then
        return -1
    end
    local length=math.sqrt(tcp[1]*tcp[1]+tcp[2]*tcp[2]+tcp[3]*tcp[3])
    if length < 0.002 then
        -- The pads are already AT this tip, so it is the tool control point
        -- rather than the flange -- a scene where './tip' was relocated onto
        -- the jaws. Adopt it. Adding the offset again would mount the gripper
        -- one tool length out and put the jaws at two, which is exactly the
        -- doubling this guard exists to stop.
        womatPickTipDummy=simTip
        womatToolControlPointMode=womatToolControlPointMode or "tip_already_at_pads"
        return simTip
    end
    local ok, dummy=pcall(function()
        local handle=sim.createDummy(${tcpMarkerSize})
        sim.setObjectAlias(handle, ${tcpAliasLua})
        -- Visible on purpose: this is where the robot actually holds things,
        -- and every offset in this driver is measured to it.
        pcall(sim.setObjectColor, handle, 0, sim.colorcomponent_ambient_diffuse, {1, 0.35, 0})
        pcall(sim.setObjectInt32Param, handle, sim.objintparam_visibility_layer, 1)
        sim.setObjectParent(handle, womatFlangeTip, true)
        if not womatOrientToolFrame(handle, tcp) then
            error("tool_frame_degenerate")
        end
        return handle
    end)
    if not ok or dummy == nil or dummy == -1 then
        womatMotionError="tcp_dummy_failed: "..tostring(dummy)
        return -1
    end
    womatPickTipDummy=dummy
    -- From here on the arm's tip IS the tool control point. Any IK environment
    -- built against the old tip describes a chain that no longer ends where the
    -- driver thinks it does, so drop the cached ones.
    simTip=dummy
    womatInvalidateIkEnvironments()
    return dummy
end

function womatInvalidateIkEnvironments()
    if womatSimpleIkEnv ~= nil then
        pcall(simIK.eraseEnvironment, womatSimpleIkEnv)
    end
    womatSimpleIkEnv=nil
    womatSimpleIkGroup=nil
    if womatRestrictedIkEnv ~= nil then
        pcall(simIK.eraseEnvironment, womatRestrictedIkEnv)
    end
    womatRestrictedIkEnv=nil
    womatRestrictedIkGroup=nil
    womatRestrictedIkJoints=nil
    return true
end

function womatSetApproachTarget(world, axis, rollRad)
    local xAxis, yAxis, zAxis=womatApproachFrame(axis, rollRad)
    if xAxis == nil then
        return false
    end
    sim.setObjectMatrix(simTarget, sim.handle_world, {
        xAxis[1], yAxis[1], zAxis[1], world[1],
        xAxis[2], yAxis[2], zAxis[2], world[2],
        xAxis[3], yAxis[3], zAxis[3], world[3]
    })
    return true
end

function womatSetDownTarget(world)
    return womatSetApproachTarget(world, {0, 0, -1})
end

function womatPickPoseError(pickTip, world, axis)
    -- Verify the solution against the scene instead of trusting the solver:
    -- how far the driven dummy is from the commanded point, and how far the
    -- tool axis is from the commanded approach direction.
    local matrix=sim.getObjectMatrix(pickTip, sim.handle_world)
    local dx=matrix[4]-world[1]
    local dy=matrix[8]-world[2]
    local dz=matrix[12]-world[3]
    -- Second column of the rotation part: the dummy's Y axis, which runs down
    -- the tool. See womatOrientToolFrame.
    local wanted=axis or {0, 0, -1}
    local length=math.sqrt(wanted[1]*wanted[1]+wanted[2]*wanted[2]+wanted[3]*wanted[3])
    if length < 1e-9 then
        return math.sqrt(dx*dx+dy*dy+dz*dz), math.pi
    end
    local axisDot=(matrix[2]*wanted[1]+matrix[6]*wanted[2]+matrix[10]*wanted[3])/length
    if axisDot > 1 then axisDot=1 end
    if axisDot < -1 then axisDot=-1 end
    return math.sqrt(dx*dx+dy*dy+dz*dz), math.acos(axisDot)
end

