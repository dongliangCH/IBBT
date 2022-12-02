% check if a point is in collision
function collision_flag = collision_point(p, world)

collision_flag = 0;
if p(1) > world.endcorner(1) || p(1) < world.origincorner(1) || p(2) > world.endcorner(2) || p(2) < world.origincorner(1)
    collision_flag = 1;
    return;  
end

p = p * world.scale;
if world.obstacle_map(ceil(p(1)),ceil(p(2))) %|| world.obstacle_map(floor(p(1)),floor(p(2)))
    collision_flag = 1;
    return;
end

end
