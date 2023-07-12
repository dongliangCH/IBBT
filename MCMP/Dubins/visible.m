function visible_flag = visible(p, world)

marker = world.Marker;
visible_flag = 0;
dim = 2;

for i = 1:size(marker,1)
    
    vec_norm = norm(marker(i,:) - [p(1), p(2)]);
    pathx = linspace(p(1), marker(i,1), vec_norm/0.2);
    pathy = linspace(p(2), marker(i,2), vec_norm/0.2);
    if ~MeanCollisionCheck([pathx; pathy], world, dim)
        visible_flag = 1;
        return;
    end
        
end

end