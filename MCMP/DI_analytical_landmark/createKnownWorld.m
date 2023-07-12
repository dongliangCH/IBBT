function [world, NumObstacles] = createKnownWorld(endcorner, origincorner, dim)
rng(1)
  world_low_x = origincorner(1);
  world_low_y = origincorner(2);
  world_high_x = endcorner(1);
  world_high_y = endcorner(2);
  NumObstacles = 15;
  NumMarker = 5;
  if dim == 2
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2))
        disp('Not valid corner specifications!')
        world=[];
    % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        world.NumMarker = NumMarker;
    
        % create NumObstacles
        for i = 1:NumObstacles
            cx = world_low_x + rand * (world_high_x - world_low_x - 2) + 1;
            cy = world_low_y + rand * (world_high_y - world_low_y - 2) + 1;
            world.radius(i) = 1 + 2 * rand;
            world.cx(i) = cx;
            world.cy(i) = cy;
        end

        obstacle_map = zeros(200,200);
        resolution = 0.1;
        scale = 1/resolution;
        for j = 1:200
            for k = 1:200
                for i = 1:NumObstacles
                    vec = [j*resolution; k*resolution]-[world.cx(i); world.cy(i)];
                    if  sum(vec.*vec)<(world.radius(i)+0.1)^2 
                        obstacle_map(j,k) = 1;
                        break;
                    end
                end
            end
        end
        world.scale = scale;
        world.resolution= resolution;
        world.obstacle_map = obstacle_map;

        %% Landmarks
        k = 1;
        Marker = [];
        while k <= NumMarker
            marker = [world_low_x + rand * (world_high_x - world_low_x), world_low_y + rand * (world_high_y - world_low_y)];
            if ~collision_point(marker, world, dim)
                Marker = [Marker; marker];
                k = k + 1;
            end
        end
        world.Marker = Marker;
    end

  elseif dim == 3

    NumObstacles = 3;
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2)) || (endcorner(3) <= origincorner(3))
      disp('Not valid corner specifications!')
      world=[];

    % create world data structure
    else
    world.NumObstacles = NumObstacles;
    world.endcorner = endcorner;
    world.origincorner = origincorner;

        % create NumObstacles
        maxRadius = 3;

        world.radius(1) = maxRadius;
        cx = 10;
        cy = 10;
        cz = 10;
        world.cx(1) = cx;
        world.cy(1) = cy;
        world.cz(1) = cz;

        world.radius(2) = maxRadius;
        cx = 5;
        cy = 5;
        cz = 5;
        world.cx(2) = cx;
        world.cy(2) = cy;
        world.cz(2) = cz;

        world.radius(3) = maxRadius;
        cx = 15;
        cy = 15;
        cz = 15;
        world.cx(3) = cx;
        world.cy(3) = cy;
        world.cz(3) = cz;
     end
   end
end