% check if a point is in collision
function collision_flag = collision_point(p, world, dim)

collision_flag = 0;

for i=1:dim
   if (p(i)>world.endcorner(i))||(p(i)<world.origincorner(i))
       collision_flag = 1;
       return;
   end
end

p = p * world.scale;
if world.obstacle_map(ceil(p(1)),ceil(p(2))) %|| world.obstacle_map(floor(p(1)),floor(p(2)))
    collision_flag = 1;
    return;
end

end