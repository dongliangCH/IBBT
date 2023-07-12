function world = inflate_world(world, inflate)
      
Obstacle = world.Obstacle;
new_Obstacle = [Obstacle(:,1:2) - inflate, Obstacle(:,3:4) + inflate];

resolution = 1/world.scale;
obstacle_map = zeros(200,200);

for i = 1 : size(Obstacle, 1)
    for j = floor(new_Obstacle(i, 1)/resolution) : ceil(new_Obstacle(i, 3)/resolution)
        for k = floor(new_Obstacle(i, 2)/resolution) : ceil(new_Obstacle(i, 4)/resolution)
            j = max(j, 1);
            j = min(j, 200);
            k = max(k, 1);
            k = min(k, 200);
            obstacle_map(j,k) = 1;
        end
    end        
end

world.Obstacle = new_Obstacle;
world.obstacle_map = obstacle_map;

end