function world = createKnownWorld(dim)

  if dim == 2
      
world_low_x = 0;
world_low_y = 0;
world_high_x = 5;
world_high_y = 8;
world.origincorner = [world_low_x world_low_y];
world.endcorner = [world_high_x world_high_y];

NumofObstacles = 2;
obstacle_map = zeros(50,80);
resolution = 0.1;
scale = 1/resolution;
Obstacle = zeros(NumofObstacles,4);

figure(1)
% axis([world_low_x, world_high_x, world_low_y, world_high_y]); hold on
  plot([world.origincorner(1) world.endcorner(1)  world.endcorner(1) world.origincorner(1) world.origincorner(1)], ...
      [world.origincorner(2) world.origincorner(2) world.endcorner(2) world.endcorner(2) world.origincorner(2)], 'color', 'k', 'linewidth', 1)
  axis equal
  axis off
  hold on
  xlim([0 5]); ylim([0 8]);

low_x = 0; high_x = 2; low_y = 5; high_y = 6;
X = [low_x  high_x  high_x  low_x   low_x];
Y = [low_y  low_y   high_y  high_y  low_y];
fill(X, Y, [0.3 0.3 0.3]); 
Obstacle(1,:) = [low_x low_y high_x high_y];

for j = floor(low_x/resolution) + 1 : ceil(high_x/resolution)
    for k = floor(low_y/resolution) : ceil(high_y/resolution)
        obstacle_map(j,k) = 1;
    end
end        

low_x = 3; high_x = 5; low_y = 5; high_y = 6;
X = [low_x  high_x  high_x  low_x   low_x];
Y = [low_y  low_y   high_y  high_y  low_y];
fill(X, Y, [0.3 0.3 0.3]); 
Obstacle(2,:) = [low_x low_y high_x high_y];

for j = floor(low_x/resolution) : ceil(high_x/resolution)
    for k = floor(low_y/resolution) : ceil(high_y/resolution)
        obstacle_map(j,k) = 1;
    end
end   


sensor_map = zeros(50,80);
sensor = [0 0 5 2];  % [low_x  low_y  high_x  high_y];
for i = 1:size(sensor,1)
    X = [sensor(i,1)  sensor(i,3)  sensor(i,3)  sensor(i,1)  sensor(i,1)];
    Y = [sensor(i,2)  sensor(i,2)  sensor(i,4)  sensor(i,4)  sensor(i,2)];
    fill(X, Y, [0.1 0.4470 0.6410]);
    
    for j = floor(sensor(i,1)/resolution) + 1 : ceil(sensor(i,3)/resolution)
        for k = floor(sensor(i,2)/resolution) +1 : ceil(sensor(i,4)/resolution)
            sensor_map(j,k) = 1;
        end
    end   
    
end

world.scale = scale;
world.obstacle_map = obstacle_map;
world.Obstacle = Obstacle;
world.sensor_map = sensor_map;
world.sensor = sensor;

  end

end