% World 20*20, discretize resolution 0.1*0.1, disretized map 200*200
% Generate random rectangles obstacles
rng(0)
world_low_x = 0;
world_low_y = 0;
world_high_x = 20;
world_high_y = 20;
NumofObstacles = 20;
size_min = 1;
size_max = 2;
discrete_map = zeros(200,200);
resolution = 0.1;
Obstacles = zeros(NumofObstacles,4);
figure(1)
axis([world_low_x, world_high_x, world_low_y, world_high_y]); hold on
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
        X = [low_x  high_x  high_x  low_x   low_x];
        Y = [low_y  low_y   high_y  high_y  low_y];
        fill(X, Y, [0.3 0.3 0.3]); 
        Obstacles(i,:) = [low_x low_y high_x high_y];
        
        for j = floor(low_x/resolution) : ceil(high_x/resolution)
            for k = floor(low_y/resolution) : ceil(high_y/resolution)
                discrete_map(j,k) = 1;
            end
        end        
    end
    
end

sensor_map = zeros(200,200);
sensor = [8 1 11 4;
          16 9 18 12;
          7 13 10 15];
for i = 1:size(sensor,1)
    X = [sensor(i,1)  sensor(i,3)  sensor(i,3)  sensor(i,1)  sensor(i,1)];
    Y = [sensor(i,2)  sensor(i,2)  sensor(i,4)  sensor(i,4)  sensor(i,2)];
    fill(X, Y, 'b');
    
    for j = floor(sensor(i,1)/resolution) : ceil(sensor(i,3)/resolution)
        for k = floor(sensor(i,2)/resolution) : ceil(sensor(i,4)/resolution)
            sensor_map(j,k) = 1;
        end
    end   
    
end


% figure(2)
% axis([world_low_x, world_high_x, world_low_y, world_high_y]); hold on
% for i = 1:size(discrete_map,1)
%     for j = 1:size(discrete_map,2)
%         if discrete_map(i,j)
%             plot(i*resolution,j*resolution,'r.'); hold on
%         end
%     end
% end
% for i = 1:size(sensor_map,1)
%     for j = 1:size(sensor_map,2)
%         if sensor_map(i,j)
%             plot(i*resolution,j*resolution,'b.'); hold on
%         end
%     end
% end
