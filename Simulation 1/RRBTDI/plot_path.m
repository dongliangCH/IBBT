function plot_path(Vertices, BeliefNodes, Path_idx, param, world)

plotCovariance(Vertices(Path_idx(1, 1), :)', BeliefNodes{Path_idx(1, 1)}{Path_idx(2, 1)}{1});

for i = 1:size(Path_idx, 2)-1
    
    [meanTraj, ~, N]  = meanControl(Vertices(Path_idx(1, i), :)', Vertices(Path_idx(1, i+1), :)', param);  
    plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
    [endP0, ~, ~, ~, ~] = propagate( BeliefNodes{Path_idx(1, i)}{Path_idx(2, i)}{1}, BeliefNodes{Path_idx(1, i)}{Path_idx(2, i)}{2}, N, param, meanTraj, world ); 
%     plotCovariance(meanTraj(:,end), endP0);
    
end

end