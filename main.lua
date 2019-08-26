-- TODO:
-- figure out what the FUCK is going on with the right and left wall collisions???? omg
-- -- why ise velocity GROWING from DAMPING when distance is TINY AF??????
-- inside tests!!
--  w mouse first
-- then with other polys!!
function vec(x,y)
    local v = {}
    v.x = x or 0
    v.y = y or 0
    v.add = addVec
    return v
end

function vecDiff(a,b)
    return vec(a.x-b.x,a.y-b.y)
end

function addVec(a,b)
    return vec(a.x+b.x,a.y+b.y)
end

function mulVec(a,c)
    return vec(a.x*c, a.y*c)
end

function dotProd(a,b)
    return a.x*b.x+a.y*b.y
end

function rotate(v,angle)
    -- rotate a vector around 0,0
    local a = math.atan2(v.y,v.x)+angle
    local d = math.sqrt(v.x*v.x+v.y*v.y)
    return vec(math.cos(a)*d,math.sin(a)*d)
end

function newBody(shape, pos, vel, angle, w)
    local a = {}
    a.shape = shape -- keep shape centered for center of gravity
    a.pos = pos or vec()
    a.vel = vel or vec()
    a.angle = angle or 0
    a.w = w or 0 -- angular vel in radians per second
    a.m = shape.m--mass
    a.I = shape.I--moment of inertia
    -- force and torque accumulator
    a.torque = 0
    a.force = vec()
    -- methods
    a.update = updateBody
    a.calculateForces = calculateForces
    a.applyForce = applyForce
    a.perim=bodyPerim
    a.normal=getNormal
    a.velAt=velAt
    a.isIn=isIn
    return a
end

function getNormal(a,n)
    -- get surface normal on perimeter
    local pn1 = (n-cStep)%1
    local pn2 = (n+cStep)%1
    local p1 = a:perim(pn1)
    local p2 = a:perim(pn2)
    local delta = vecDiff(p2,p1)
    local d = math.sqrt(delta.x*delta.x+delta.y*delta.y)
    local nml = mulVec(delta,1/d)
    return vec(nml.y,-nml.x)
end

function isIn(a,p)
    -- whether point p is inside the shape

    -- get closest point
    local closest = nil
    local cd = 0
    local cn = 0
    for n=0,1-cStep,cStep do
        local cp = a:perim(n)
        local delta = vecDiff(cp,p)
        local d = math.sqrt(delta.x*delta.x+delta.y*delta.y)
        if closest == nil or d < cd then
            cd = d
            cn = n
            closest = cp
        end
    end
    local sn = a:normal(cn) -- get surface normal at the closest point
    local delta = vecDiff(closest,p)
    --debugVec=delta
    --debugVecP=p
    --debugVec2=sn
    --debugVecP2=closest
    local dot = dotProd(sn,delta)
    -- is normal pointing away from the point in question?
    -- prepare the results
    local res = {}
    res.isIn = dot>0
    res.p = closest
    res.sn = sn
    res.delta = delta
    res.dot = dot
    return res
end

function bodyPerim(a, n)
    -- gets shape perim but rotated and translated
    local p = a.shape:perim(n)
    p=rotate(p,a.angle)
    p=addVec(p, a.pos)
    return p
end

function velAt(a,pos)
    -- get the velocity at a given absolute position
    -- get rotational velocity
    local delta = vecDiff(a.pos,pos)
    local ang = math.atan2(delta.y,delta.x)
    local d = math.sqrt(delta.x*delta.x+delta.y*delta.y)
    local v = vec(math.sin(ang)*d*a.w,-math.cos(ang)*d*a.w)
    -- add to positional velocity
    return addVec(a.vel, v)
end

function applyForce(a, force, pos)
    -- accumulate a force on body a

    -- translational force
    a.force = addVec(a.force, force)

    -- angular force
    if pos then
        local delta = vecDiff(a.pos,pos)
        local cross = delta.y*force.x-delta.x*force.y
        a.torque = a.torque + cross
        -- debug stuf
        debugVec=delta
        debugVecP=pos
        debugVec2=vec(delta.y,-delta.x)
        debugVecP2=pos
        debugVec3=mulVec(force,1/10)
        debugVecP3=pos
        debugVec4=mulVec(vec(delta.y,-delta.x), cross*10)
        debugVecP4=pos
    end
end

function calculateForces(a)
    -- se what forces are currently acting on the body

    -- collisions
    for n=0,1-cStep,cStep do
        local p = a:perim(n)

        -- walls
        if p.y < -1 then
            local pen = -p.y-1
            -- penetration spring
            local f = pen*collisionSpring*a.m
            -- damping
            local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), vec(0,1))*a.m
            local friction = dotProd(mulVec(a:velAt(p),pen*wallFriction), vec(1,0))*a.m
            a:applyForce(vec(-friction, -damping+f), p)
        elseif p.y > 1 then
            local pen = p.y-1
            -- penetration spring
            local f = -pen*collisionSpring*a.m
            -- damping
            local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), vec(0,1))*a.m
            local friction = dotProd(mulVec(a:velAt(p),pen*wallFriction), vec(1,0))*a.m
            a:applyForce(vec(-friction, -damping+f), p)
        end
        if p.x < -screenRatio then
            local pen = -p.x-screenRatio
            -- penetration spring
            local f = pen*collisionSpring*a.m
            -- damping
            local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), vec(1,0))*a.m
            local friction = dotProd(mulVec(a:velAt(p),pen*wallFriction), vec(0,1))*a.m
            a:applyForce(vec(-damping+f, -friction), p)
        elseif p.x > screenRatio then
            local pen = p.x-screenRatio
            -- penetration spring
            local f = -pen*collisionSpring*a.m
            -- damping
            local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), vec(1,0))*a.m
            local friction = dotProd(mulVec(a:velAt(p),pen*wallFriction), vec(0,1))*a.m
            a:applyForce(vec(-damping+f, -friction), p)
        end

        -- interbody collisions
        for k,b in pairs(bodies) do
            if not b~=a then -- only compare against OTHER bodies lol
                -- if this collision point is inside this other body, COLLISION DETECTED LOL
                local res = b:isIn(p)
                if res.isIn then
                    local pen = res.dot

                    local m = a.m+b.m
                    -- penetration spring
                    local f = pen*collisionSpring
                    ---- damping
                    local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), res.sn)
                    local friction = dotProd(mulVec(a:velAt(p),pen*friction), vec(res.sn.y,-res.sn.x))
                    f = f - damping
                    local fv = mulVec(res.sn,f)
                    a:applyForce(mulVec(fv, a.m*(b.m/m)), p)
                    b:applyForce(mulVec(fv,-b.m*(a.m/m)), p)
                end
            end
        end
    end


    -- gravity
    a:applyForce(vec(0,-gravity*a.m))
    --a:applyForce(vec(-gravity*a.m,0))

    -- mouse grab
    if a==grabbed and love.mouse.isDown(1) then
        local ang = moffa+a.angle
        local p = vec(math.cos(ang)*moffd+a.pos.x,math.sin(ang)*moffd+a.pos.y)
        local delta = vecDiff(mmouse, p)
        local d = math.sqrt(delta.x*delta.x+delta.y*delta.y)
        if d > 0 then
            local dir = mulVec(delta,-1/d)
            local mf = mulVec(delta,mouseSpring)
            local deltaVel = vecDiff(mouseVel, a:velAt(p))
            local dot = dotProd(deltaVel, dir)
            local proj = mulVec(dir, dot)
            local damping = mulVec(proj, mouseDamping)
            mf = addVec(mf,damping)
            a:applyForce(mulVec(mf,a.m),p)
        end
    end
end

function updateBody(a, dt)
    -- integrate position
    local accel = mulVec(a.force, 1/a.m)
    local d = math.sqrt(accel.x*accel.x+accel.y*accel.y)
    if d>maxAccel then
        accel = mulVec(accel, maxAccel/d)
    end
    a.vel = addVec(a.vel, mulVec(accel, dt))
    a.pos = addVec(a.pos, mulVec(a.vel, dt))
    
    -- integrate angle
    a.w = a.w + math.min(maxAngularAccel, math.max(-maxAngularAccel, a.torque/a.I)) * dt
    a.angle = (a.angle + a.w * dt)%(2*math.pi)

    -- reset accumulators
    a.torque = 0
    a.force = vec()
end

function drawBody(a)
    local v = {} -- vertices to draw
    for n=0,1-pStep,pStep do
        local p = a:perim(n)
        local pp = mapPixel(p)
        table.insert(v,pp.x)
        table.insert(v,pp.y)
    end
    love.graphics.setColor(1,1,1)
    love.graphics.polygon('fill',v)
    love.graphics.setColor(0,0,0)
    love.graphics.polygon('line',v)
    love.graphics.setColor(0,1,1)
    love.graphics.setPointSize(8)
    love.graphics.points(v)

    -- show collision shape
    local vc = {} -- vertices to draw
    for n=0,1-cStep,cStep do
        local p = a:perim(n)
        local pp = mapPixel(p)
        table.insert(vc,pp.x)
        table.insert(vc,pp.y)
    end
    love.graphics.setColor(1,0,0)
    love.graphics.setPointSize(4)
    love.graphics.points(vc)
end

function perimCircle(a, n)
    -- get points along perimiter of body x where 0 <= n <= 1
    local nn = n*math.pi*2
    local p = {}
    p.x = math.cos(nn)*a.r
    p.y = math.sin(nn)*a.r
    return p
end

function perimRect(a, n)
    -- get points along perimiter of body x where 0 <= n <= 1
    local p = {}
    local nn = n*4
    if nn<1 then
        p.x=nn
        p.y=0
    elseif nn<2 then
        p.x=1
        p.y=nn-1
    elseif nn<3 then
        p.x=3-nn
        p.y=1
    else
        p.x=0
        p.y=4-nn
    end
    p.x=(p.x-0.5)*a.w*2
    p.y=(p.y-0.5)*a.h*2
    return p
end

function newRect(w,h)
    local a = {}
    a.w=w or 1
    a.h=h or a.w
    a.perim=perimRect
    a.m=a.w*a.h
    a.I = a.m*(a.w+a.h)/12
    return a
end

function newCircle(r)
    local a = {}
    a.r = r or 1
    a.perim=perimCircle
    a.m=math.pi*a.r*a.r
    a.I = a.m*a.r*a.r/2
    return a
end

function mapPixel(p)
    -- remap normalized coordinates p to pixel coordinates
    n = {}
    n.x = (p.x+transX)*scaleX
    n.y = (p.y+transY)*scaleY
    return n
end

function love.draw()
    love.graphics.clear(.8,.8,.8)
    -- draw bodies in world
    for k,v in pairs(bodies) do
        drawBody(v)
    end

    -- debug vectors
    local p11 = mapPixel(debugVecP)
    local p21 = mapPixel(debugVecP2)
    local p31 = mapPixel(debugVecP3)
    local p41 = mapPixel(debugVecP4)
    local p1 = mapPixel(addVec(debugVecP,debugVec))
    local p2 = mapPixel(addVec(debugVecP2,debugVec2))
    local p3 = mapPixel(addVec(debugVecP3,debugVec3))
    local p4 = mapPixel(addVec(debugVecP4,debugVec4))
    love.graphics.setColor(1,0,0)
    love.graphics.line(p11.x,p11.y,p1.x,p1.y)
    love.graphics.setColor(0,0.5,0)
    love.graphics.line(p21.x,p21.y,p2.x,p2.y)
    love.graphics.setColor(0,0,1)
    love.graphics.line(p31.x,p31.y,p3.x,p3.y)
    love.graphics.setColor(0.5,0,1)
    love.graphics.line(p41.x,p41.y,p4.x,p4.y)
end

function love.resize(w,h)
    -- normal coordinates (-1 to 1) to pixel coordinates preparation
    screenRatio = love.graphics.getWidth()/love.graphics.getHeight()
    scaleX = love.graphics.getWidth()/2/screenRatio
    scaleY = -love.graphics.getHeight()/2
    transX = screenRatio
    transY = -1
end

function setRes(res)
    -- shape resolution
    pRes = res or 16
    pStep = 1/pRes
end

function setCollideRes(res)
    -- collision resolution
    cRes = res or 16
    cStep = 1/cRes
end

function love.mousepressed(x,y,button,istouch)
    grabbed = nil
    for k,a in pairs(bodies) do
        if a:isIn(mmouse).isIn then
            grabbed = a
        end
    end
    if grabbed ~=nil then
        local delta = vecDiff(mmouse,grabbed.pos)
        moffd = math.sqrt(delta.x*delta.x+delta.y*delta.y)
        moffa = math.atan2(delta.y,delta.x)-grabbed.angle
    end
end

function love.load()
    love.resize()

    setRes()
    setCollideRes()

    epsilon = 0.001
    gravity = 5
    collisionSpring = 3000
    collisionDamping = 75
    mouseSpring = 25
    mouseDamping = 5
    wallFriction = 3000
    friction = 3000
    sweeps = 10
    -- ad hoc limits to avoid instability
    maxAccel = 1000
    maxAngularAccel = 1000
    
    grabbed = nil -- grabbed body w cursor

    collisionDamping = collisionDamping * math.sqrt(collisionSpring)
    mouseDamping = mouseDamping * math.sqrt(mouseSpring)

    -- mouse
    -- normalized position
    mmouse = vec()
    mouseVel = vec() -- and velocity
    -- offset on object in polar coords
    moffd = 0--distance
    moffa = 0--angle

    -- bodies in world
    local body = newBody(newRect(0.4,0.05))
    local body2 = newBody(newCircle(0.2))
    local body3 = newBody(newRect(0.3,0.3))
    body.pos = vec(0.5,0.5)
    body3.pos = vec(-0.5,0.5)
    bodies = {body, body2, body3}
end

function love.update(dt)
    dt = 1/60--bleh
    local mx = (love.mouse.getX()/love.graphics.getWidth()*2-1)*screenRatio
    local my = -(love.mouse.getY()/love.graphics.getHeight()*2-1)
    local mv = vec((mx-mmouse.x)/dt, (my-mmouse.y)/dt)
    local mvsmooth = 3
    mouseVel = mulVec(vec(mv.x+mouseVel.x*mvsmooth,mv.y+mouseVel.y*mvsmooth),1/(mvsmooth+1))
    mmouse = vec(mx, my)
    --debug vecs
    debugVecP=mmouse
    debugVecP2=mmouse
    debugVecP3=mmouse
    debugVecP4=mmouse
    debugVec = vec()
    debugVec2 = vec()
    debugVec3 = vec()
    debugVec4 = vec()

    -- update bodies in world
    dt = dt/sweeps
    for i=1,sweeps do
        for k,v in pairs(bodies) do
            v:calculateForces(dt)
        end
        for k,v in pairs(bodies) do
            v:update(dt)
        end
    end
    ---TEMP for debugging:
    --bodies[2]:isIn(mmouse)
end
