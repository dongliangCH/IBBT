function plot_path(Vertices, Path_idx, param)

for i = 1:size(Path_idx, 2)-1
    
    [meanTraj, ~]  = dubins_curve(Vertices(Path_idx(1, i), :), Vertices(Path_idx(1, i+1), :), param.radi, param.dt, 1);
    meanTraj = [meanTraj, Vertices(Path_idx(1, i+1), :)'];
    plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
    
end

end