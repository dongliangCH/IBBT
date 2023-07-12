function Cost = TreesCost(TreesVertices, goal_cord, BeliefTrees, param, world, r)
Cost = [];
for j = 1:size(TreesVertices,2)
    Vertices = TreesVertices{j};
    BeliefNodes = BeliefTrees{j};
    min_cost = 1000;
    dim = 2;
    tmp_dist = Vertices(:, 1 : dim) - goal_cord(1 : dim);
    dist_sqr = sqr_eucl_dist(tmp_dist, dim);
    near_idx = find(dist_sqr <= r^2);
    for i = 1:size(near_idx,1)
        if size(BeliefNodes,1) >= near_idx(i)
         [meanTraj, MCost, N]  = meanControl(Vertices(near_idx(i), :)', goal_cord', param); 
         if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
             for k = 1:size(BeliefNodes{near_idx(i)},2)
             
                 if ~isempty(BeliefNodes{near_idx(i)}{k})
                 [~, ~, CovCost, CollisionProb, ~] = propagate( BeliefNodes{near_idx(i)}{k}{1}, BeliefNodes{near_idx(i)}{k}{2}, N, param, meanTraj, world ); 
                 if CollisionProb <= param.chanceConstraint
                    cost_to_come = BeliefNodes{near_idx(i)}{k}{3} + MCost + CovCost;
                    if cost_to_come < min_cost
                        min_cost = cost_to_come;
                    end
                 end
                 end
             end
         end
        end
    end
    Cost = [Cost min_cost];
end
end