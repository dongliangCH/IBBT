function world = createKnownWorld(dim)

  if dim == 2
      
world_low_x = 0;
world_low_y = 0;
world_high_x = 20;
world_high_y = 20;
world.origincorner = [world_low_x world_low_y];
world.endcorner = [world_high_x world_high_y];

NumofObstacles = 14;
size_min = 1;
size_max = 2;
obstacle_map = zeros(200,200);
resolution = 0.1;
scale = 1/resolution;
Obstacle = zeros(NumofObstacles,4);

% figure(1)
% % axis([world_low_x, world_high_x, world_low_y, world_high_y]); hold on
%   plot([world.origincorner(1) world.endcorner(1)  world.endcorner(1) world.origincorner(1) world.origincorner(1)], ...
%       [world.origincorner(2) world.origincorner(2) world.endcorner(2) world.endcorner(2) world.origincorner(2)], 'color', 'k', 'linewidth', 1)
%   axis off
%   hold on
%   xlim([0 20]); ylim([0 20]);

i = 0;  
while i <= NumofObstacles
    
    low_x = world_low_x + rand * (world_high_x - world_low_x);
    low_y = world_low_y + rand * (world_high_y - world_low_y);
    length = size_min + rand * size_max;
    width = size_min + rand * size_max;
    high_x = low_x + length;
    high_y = low_y + width;
    if high_x < world_high_x && high_y < world_high_y
        i = i + 1;
%         X = [low_x  high_x  high_x  low_x   low_x];
%         Y = [low_y  low_y   high_y  high_y  low_y];
%         fill(X, Y, [0.3 0.3 0.3]); 
        Obstacle(i,:) = [low_x low_y high_x high_y];
        
        for j = floor(low_x/resolution) : ceil(high_x/resolution)
            for k = floor(low_y/resolution) : ceil(high_y/resolution)
                obstacle_map(j,k) = 1;
            end
        end        
    end
    
end

sensor_map = zeros(200,200);
sensor = [8  2  12 7;
          14 9.5  18 14;
          6  10.5 9 15.5;
          1  10 3  15];
for i = 1:size(sensor,1)
%     X = [sensor(i,1)  sensor(i,3)  sensor(i,3)  sensor(i,1)  sensor(i,1)];
%     Y = [sensor(i,2)  sensor(i,2)  sensor(i,4)  sensor(i,4)  sensor(i,2)];
%     fill(X, Y, [0.1 0.4470 0.6410]);
    
    for j = floor(sensor(i,1)/resolution) : ceil(sensor(i,3)/resolution)
        for k = floor(sensor(i,2)/resolution) : ceil(sensor(i,4)/resolution)
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