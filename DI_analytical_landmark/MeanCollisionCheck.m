function collision_flag = MeanCollisionCheck(MeanTraj, world, dim)

collision_flag = 0;

if dim == 2
    if isempty(MeanTraj)
        return;
    end
    if max(MeanTraj(1,:)) > world.endcorner(1) || min(MeanTraj(1,:)) < world.origincorner(1) || max(MeanTraj(2,:)) > world.endcorner(2) || min(MeanTraj(2,:)) < world.origincorner(1)
        collision_flag = 1;
        return;  
    end
      
    MeanTraj = MeanTraj * world.scale;    
    for j = 2 : size(MeanTraj, 2)-1
        p = MeanTraj(:, j);
        if world.obstacle_map(ceil(p(1)),ceil(p(2))) %|| world.obstacle_map(floor(p(1)),floor(p(2)))
            collision_flag = 1;
            return;
        end
        
    end  
end

end