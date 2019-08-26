function vec(x,y)
    local v = {}
    v.x = x or 0
    v.y = y or 0
    v.add = addVec
    return v
end

function addVec(a,b)
    return vec(a.x+b.x,a.y+b.y)
end

function mulVec(a,c)
    return vec(a.x*c, a.y*c)
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
    a.w = w or 1 -- angular vel in radians per second
    a.update = updateBody
    a.m = 1--mass
    a.I = 1--moment of inertia
    a.perim=bodyPerim
    -- force and torque accumulator
    a.torque = 0
    a.force = vec()
    return a
end

function bodyPerim(a, n)
    -- gets shape perim but rotated and translated
    local p = a.shape:perim(n)
    p=rotate(p,a.angle)
    p=addVec(p, a.pos)
    return p
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
    p.x=p.x-0.5
    p.y=p.y-0.5
    return p
end

function newRect(w,h)
    local a = {}
    a.w=w or 1
    a.h=h or a.w
    a.perim=perimRect
    return a
end

function newCircle(r)
    local a = {}
    a.r = r or 1
    a.perim=perimCircle
    return a
end

function mapPixel(p)
    -- remap normalized coordinates p to pixel coordinates
    p.x = (p.x+transX)*scaleX
    p.y = (p.y+transY)*scaleY
    return p
end

function love.draw()
    love.graphics.clear(.8,.8,.8)
    -- draw bodies in world
    for k,v in pairs(bodies) do
        drawBody(v)
    end
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

function love.load()
    love.resize()

    setRes()

    -- bodies in world
    bodies = {newBody(newRect(.1))}
end

function love.update(dt)
    -- update bodies in world
    for k,v in pairs(bodies) do
        v:update(dt)
    end
end
