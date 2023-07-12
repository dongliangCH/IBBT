function connected_flag = Connected(Vertices, goal_cord, BeliefNodes, param, world, r)

    connected_flag = 0;
    dim = 2;
    tmp_dist = Vertices(:, 1 : dim) - goal_cord(1 : dim);
    dist_sqr = sqr_eucl_dist(tmp_dist, dim);
    near_idx = find(dist_sqr <= r^2);
    for i = 1:size(near_idx,1)
        [meanTraj, ~]  = dubins_curve(Vertices(near_idx(i), :), goal_cord, param.radi, param.dt, 1);
         if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
             for k = 1:size(BeliefNodes{near_idx(i)},2)
                 if ~isempty(BeliefNodes{near_idx(i)}{k})
                 [~, ~, ~, CollisionProb, ~] = propagate( BeliefNodes{near_idx(i)}{k}{1}, BeliefNodes{near_idx(i)}{k}{2}, param, meanTraj, world ); 
                 if CollisionProb <= param.chanceConstraint
                     connected_flag = 1;
                 end
                 end
             end
         end
    end
    
end