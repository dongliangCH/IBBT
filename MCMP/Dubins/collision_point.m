% check if a point is in collision
function collision_flag = collision_point(p, world)

p = p * world.scale;
collision_flag = 0;
if world.obstacle_map(ceil(p(1)),ceil(p(2))) %|| world.obstacle_map(floor(p(1)),floor(p(2)))
    collision_flag = 1;
    return;
end

end
