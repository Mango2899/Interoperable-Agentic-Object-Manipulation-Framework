function womatApproachFrame(axis, rollRad)
    -- Orthonormal frame whose Z runs along the requested approach axis. The
    -- roll about that axis is left where the seed puts it unless one is asked
    -- for: with constraint_alpha_beta the IK ignores it anyway, so any seed
    -- will do, and the seed must then be DETERMINISTIC rather than merely
    -- arbitrary -- a rolled frame is only reproducible if the frame it is
    -- measured from is.
    --
    -- A roll is asked for when the receiver's jaws have to miss the giver's.
    -- Two tool control points meeting at one cube puts both grippers around the
    -- same 45 mm block, and whether their fingers interleave or crash is
    -- decided entirely by this angle. See womatPlanHandoverApproach.
    local length=math.sqrt(axis[1]*axis[1]+axis[2]*axis[2]+axis[3]*axis[3])
    if length < 1e-9 then
        return nil
    end
    local zAxis={axis[1]/length, axis[2]/length, axis[3]/length}
    local seed={1,0,0}
    if math.abs(zAxis[1]) > 0.9 then
        seed={0,1,0}
    end
    local dot=seed[1]*zAxis[1]+seed[2]*zAxis[2]+seed[3]*zAxis[3]
    local xLen=math.sqrt((seed[1]-dot*zAxis[1])^2+(seed[2]-dot*zAxis[2])^2+(seed[3]-dot*zAxis[3])^2)
    if xLen < 1e-9 then
        return nil
    end
    local xAxis={(seed[1]-dot*zAxis[1])/xLen, (seed[2]-dot*zAxis[2])/xLen, (seed[3]-dot*zAxis[3])/xLen}
    local yAxis={
        zAxis[2]*xAxis[3]-zAxis[3]*xAxis[2],
        zAxis[3]*xAxis[1]-zAxis[1]*xAxis[3],
        zAxis[1]*xAxis[2]-zAxis[2]*xAxis[1]
    }
    if rollRad ~= nil then
        -- Rotate the two free axes about Z. The angle means nothing on its own
        -- -- it is measured from the seeded frame above -- and everything
        -- relative to itself, which is all a search over it needs.
        local c=math.cos(rollRad)
        local s=math.sin(rollRad)
        local rolledX={
            xAxis[1]*c+yAxis[1]*s,
            xAxis[2]*c+yAxis[2]*s,
            xAxis[3]*c+yAxis[3]*s
        }
        local rolledY={
            zAxis[2]*rolledX[3]-zAxis[3]*rolledX[2],
            zAxis[3]*rolledX[1]-zAxis[1]*rolledX[3],
            zAxis[1]*rolledX[2]-zAxis[2]*rolledX[1]
        }
        xAxis=rolledX
        yAxis=rolledY
    end
    return xAxis, yAxis, zAxis
end

-- Visible TCP: Y points from flange to jaws; the IK child maps that Y to Z.
function womatToolFrameMatrix(tcp)
    local fx, fy, fz=womatApproachFrame(tcp)
    if fx == nil then return nil end
    return {
        fx[1], fz[1], -fy[1], tcp[1],
        fx[2], fz[2], -fy[2], tcp[2],
        fx[3], fz[3], -fy[3], tcp[3]
    }
end
