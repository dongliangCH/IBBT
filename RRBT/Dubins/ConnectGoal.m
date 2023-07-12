function [min_cost_idx, goalP0, min_cost, Traj] = ConnectGoal(Vertices, goal_cord, BeliefNodes, param, world, r)

    min_cost = 1000;
    min_cost_idx = [];
    goalP0 = [];
    dim = 2;
    tmp_dist = Vertices(:, 1 : dim) - goal_cord(1 : dim);
    dist_sqr = sqr_eucl_dist(tmp_dist, dim);
    near_idx = find(dist_sqr <= r^2);
    for i = 1:size(near_idx,1)
         [meanTraj, V, MCost, N, Xbar]  = meanControl(Vertices(near_idx(i), :)', goal_cord', param); 
         if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
             for k = 1:size(BeliefNodes{near_idx(i)},2)
                 [endP0, endPtilde, CovCost, CollisionProb, K] = propagate( BeliefNodes{near_idx(i)}{k}{1}, BeliefNodes{near_idx(i)}{k}{2}, N, param, Xbar, V, world ); 
                 if CollisionProb < 0.3
                    cost_to_come = BeliefNodes{near_idx(i)}{k}{3} + MCost + CovCost;
                    if cost_to_come < min_cost
                        min_cost = cost_to_come;
                        goalP0 = endP0;
                        min_cost_idx = [near_idx(i); k];
                        Traj = meanTraj;
                    end
                 end
             end
         end
    end
    
end