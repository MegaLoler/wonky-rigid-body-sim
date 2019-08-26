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
    a.velAt=velAt
    return a
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
    local v = vec(-math.sin(ang)*d*a.w,-math.cos(ang)*d*a.w)
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
    end
end

function calculateForces(a)
    -- se what forces are currently acting on the body

    -- walls
    count = 0
    for n=0,1,cStep do
        local p = a:perim(n)
        if p.y < -1 then
            local pen = -p.y-1
            -- penetration spring
            local f = pen*collisionSpring
            -- damping
            local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), vec(0,1))
            a:applyForce(vec(0, -damping+f), p)
            count = count + 1
        elseif p.y > 1 then
            local pen = p.y-1
            -- penetration spring
            local f = -pen*collisionSpring
            -- damping
            local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), vec(0,1))
            a:applyForce(vec(0, -damping+f), p)
            count = count + 1
        elseif p.x < -screenRatio then
            local pen = -p.x-screenRatio
            -- penetration spring
            local f = pen*collisionSpring
            -- damping
            local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), vec(1,0))
            a:applyForce(vec(-damping+f, 0), p)
            count = count + 1
        elseif p.x > screenRatio then
            local pen = p.x-screenRatio
            -- penetration spring
            local f = -pen*collisionSpring
            -- damping
            local damping = dotProd(mulVec(a:velAt(p),pen*collisionDamping), vec(1,0))
            a:applyForce(vec(-damping+f, 0), p)
            count = count + 1
        end
    end
    if count > 0 then
        a.force = mulVec(a.force, 1/count)
    end

    -- gravity
    a:applyForce(vec(0,-gravity*a.m))

    -- mouse grab
    if love.mouse.isDown(1) then
        local ang = moffa+a.angle
        local p = vec(math.cos(ang)*moffd+a.pos.x,math.sin(ang)*moffd+a.pos.y)
        local delta = vecDiff(mmouse, p)
        local d = math.sqrt(delta.x*delta.x+delta.y*delta.y)
        if d > 0 then
            local dir = mulVec(delta,-1/d)
            local mf = mulVec(delta,mouseSpring)
            local deltaVel = vecDiff(mouseVel, a:velAt(p))
            local damping = mulVec(dir, dotProd(deltaVel,dir)*mouseDamping)
            mf = addVec(mf,damping)
            a:applyForce(mf,p)
        end
    end
end

function updateBody(a, dt)
    -- integrate position
    a.vel = addVec(a.vel, mulVec(a.force, dt/a.m))
    a.pos = addVec(a.pos, mulVec(a.vel, dt))
    
    -- integrate angle
    a.w = a.w + a.torque/a.I * dt
    a.angle = a.angle + a.w * dt

    -- reset accumulators
    a.torque = 0
    a.force = vec()
end

function drawBody(a)
    local v = {} -- vertices to draw
    for n=0,1,pStep do
        local p = a:perim(n)
        local pp = mapPixel(p)
        table.insert(v,pp.x)
        table.insert(v,pp.y)
    end
    love.graphics.setColor(1,1,1)
    love.graphics.polygon('fill',v)
    love.graphics.setColor(0,0,0)
    love.graphics.polygon('line',v)

    -- show collision shape
    --local vc = {} -- vertices to draw
    --for n=0,1,cStep do
    --    local p = a:perim(n)
    --    local pp = mapPixel(p)
    --    table.insert(vc,pp.x)
    --    table.insert(vc,pp.y)
    --end
    --love.graphics.setColor(0,1,0)
    --love.graphics.polygon('line',vc)
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
        p.x=n
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

    --local x = (love.mouse.getX()/love.graphics.getWidth()*2-1)*screenRatio
    --local y = -(love.mouse.getY()/love.graphics.getHeight()*2-1)
    --local v = bodies[1]:velAt(vec(x,y))
    --love.graphics.setColor(0,0,1)
    --love.graphics.line(200,200, 200+v.x*100,200+v.y*100)
    local p0 = mapPixel(mmouse)
    local p1 = mapPixel(addVec(mmouse,debugVec))
    love.graphics.line(p0.x,p0.y,p1.x,p1.y)
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
    pRes = res or 32
    pStep = 1/pRes
end

function setCollideRes(res)
    -- collision resolution
    cRes = res or 32
    cStep = 1/cRes
end

function love.mousepressed(x,y,button,istouch)
    local a = bodies[1]
    local delta = vecDiff(mmouse,a.pos)
    moffd = math.sqrt(delta.x*delta.x+delta.y*delta.y)
    moffa = math.atan2(delta.y,delta.x)-a.angle
end

function love.load()
    love.resize()

    setRes()
    setCollideRes()

    gravity = 1
    collisionSpring = 500
    collisionDamping = 10
    mouseSpring = 100
    mouseDamping = 100
    sweeps = 10


    debugVec = vec()

    -- mouse
    -- normalized position
    mmouse = vec()
    mouseVel = vec() -- and velocity
    -- offset on object in polar coords
    moffd = 0--distance
    moffa = 0--angle

    -- bodies in world
    local rect = newBody(newCircle(0.4))
    bodies = {rect}
end

function love.update(dt)
    local mx = (love.mouse.getX()/love.graphics.getWidth()*2-1)*screenRatio
    local my = -(love.mouse.getY()/love.graphics.getHeight()*2-1)
    mouseVel = vec(mx-mmouse.x, my-mmouse.y)
    mmouse = vec(mx, my)
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
end
