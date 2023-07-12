function world = inflate_world(world, inflate)

world.radius = world.radius + inflate;

for j = 1:200
    for k = 1:200
        for i = 1:world.NumObstacles
            vec = [j*world.resolution; k*world.resolution]-[world.cx(i); world.cy(i)];
            if  sum(vec.*vec)<(world.radius(i)+0.05)^2 
                obstacle_map(j,k) = 1;
                break;
            end
        end
    end
end
world.obstacle_map = obstacle_map;

end